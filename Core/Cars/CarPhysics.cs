using System;
using System.Numerics;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Cars;

public static class CarPhysics
{
    private const float Gravity = 9.80665f;
    private const float Epsilon = 1e-5f;
    private const float MinimumTireHeatLoadScale = 0.2f;
    private const float MaximumThermalOverLimit = 1f;
    private const float MinimumTemperatureGripFactor = 0.55f;
    private const float MaximumTemperatureGripFactor = 1.08f;
    private const float MinimumWearGripFactor = 0.45f;
    private const float RearSlipOnsetCombinedUse = 0.82f;
    private const float RearSlipDominanceRange = 0.2f;
    private const float DynamicYawMinimumSpeed = 5f;
    private const float DynamicYawBlendRange = 5f;
    private const float SideslipEnergyLossScale = 1f;
    internal static CarPerformanceLimits EstimatePerformanceLimits(
        CarState state,
        CarConfig config,
        TireConfig tires,
        CarStrategy strategy,
        float speed,
        float curvature,
        float gripUsage = 1f,
        float assumedLongitudinalAcceleration = 0f,
        float frontBrakeBiasOffset = 0f,
        float corneringEfficiency = 1f
    )
    {
        float lateralAcceleration = speed * speed * curvature;
        WheelLoads loads = CalculateWheelLoads(
            config,
            assumedLongitudinalAcceleration,
            lateralAcceleration,
            speed
        );
        float usage = Math.Clamp(gripUsage, 0.05f, 1f);
        float frontGrip = (
            loads.FrontLeft * CalculateTireMu(tires, state.FrontLeft) +
            loads.FrontRight * CalculateTireMu(tires, state.FrontRight)
        ) / Math.Max(config.MassKg, Epsilon) * usage;
        float rearGrip = (
            loads.RearLeft * CalculateTireMu(tires, state.RearLeft) +
            loads.RearRight * CalculateTireMu(tires, state.RearRight)
        ) / Math.Max(config.MassKg, Epsilon) * usage;

        float frontDemandShare = Math.Clamp(config.FrontStaticLoadShare, 0f, 1f);
        float rearDemandShare = 1f - frontDemandShare;
        float frontLateralLimit = frontDemandShare <= Epsilon
            ? float.PositiveInfinity
            : frontGrip / frontDemandShare;
        float rearLateralLimit = rearDemandShare <= Epsilon
            ? float.PositiveInfinity
            : rearGrip / rearDemandShare;
        float extraction = Math.Clamp(corneringEfficiency, 0.05f, 1f);
        float lateralLimit =
            Math.Min(frontLateralLimit, rearLateralLimit) * extraction;

        // What the corner costs the tyre, which is not what the corner is
        // worth to the car. A driver who only gets part of the cornering out
        // of a tyre still spends the tyre on all of it, so the grip left over
        // for braking has to be measured against the bill, not the benefit.
        // Charging the smaller number here is what let a plan brake as though
        // it were still on the straight while the car was already at the limit.
        float chargedLateralAcceleration = lateralAcceleration / extraction;
        float frontLateral = chargedLateralAcceleration * frontDemandShare;
        float rearLateral = chargedLateralAcceleration * rearDemandShare;
        float frontLongitudinal = RemainingLongitudinalGrip(
            frontGrip,
            frontLateral
        );
        float rearLongitudinal = RemainingLongitudinalGrip(
            rearGrip,
            rearLateral
        );

        float gripDriveLimit = DistributedLongitudinalLimit(
            frontLongitudinal,
            rearLongitudinal,
            config.FrontDriveShare
        );
        float batteryDriveLimit = CalculateDriveAccelLimit(
            state,
            config,
            strategy,
            speed
        );
        float maximumDrive = Math.Min(
            config.MaxDriveAcceleration,
            Math.Min(gripDriveLimit, batteryDriveLimit)
        );
        float frontBrakeShare = BiasedFrontBrakeShare(
            frontLongitudinal,
            rearLongitudinal,
            frontBrakeBiasOffset
        );
        float maximumBrake = Math.Min(
            config.MaxBrakeAccel,
            DistributedLongitudinalLimit(
                frontLongitudinal,
                rearLongitudinal,
                frontBrakeShare
            )
        );
        float lateralUse = Math.Abs(lateralAcceleration) / Math.Max(frontGrip + rearGrip, Epsilon);
        float loss = CalculateLossAccel(
            config,
            speed,
            lateralUse,
            state.AirVelocityDeficit
        );

        return new CarPerformanceLimits(
            Math.Max(0f, lateralLimit),
            Math.Max(0f, maximumDrive),
            Math.Max(0f, maximumBrake),
            Math.Max(0f, loss)
        );
    }

    public static void Step(
        CarState state,
        CarConfig config,
        TireConfig tires,
        CarPhysicsStepInput input,
        float dt
    )
    {
        if (dt <= 0f)
            return;

        state.Normalize();

        WheelLoads loads = CalculateWheelLoads(state, config);
        ApplyWheelLoads(state, loads);

        float frontGrip = CalculateAxleGripAccel(config, tires, state.FrontLeft, state.FrontRight);
        float rearGrip = CalculateAxleGripAccel(config, tires, state.RearLeft, state.RearRight);
        float totalGrip = Math.Max(Epsilon, frontGrip + rearGrip);

        float desiredCurvature = Math.Clamp(
            input.DriverInput.DesiredCurvature,
            -config.MaxCurvatureRequest,
            config.MaxCurvatureRequest
        );
        float desiredAccel = Math.Clamp(
            input.DriverInput.DesiredAccel,
            -config.MaxBrakeAccel,
            config.MaxDriveAcceleration
        );

        float requestedLateralAccel = state.Speed * state.Speed * desiredCurvature;
        float referenceYawRate = state.Speed * desiredCurvature;
        float dynamicYawBlend = CalculateDynamicYawBlend(state.Speed);
        LateralRequests lateralRequests = AllocateLateralRequests(
            state,
            config,
            requestedLateralAccel,
            referenceYawRate,
            dynamicYawBlend
        );
        float frontLatRequest = lateralRequests.Front;
        float rearLatRequest = lateralRequests.Rear;

        float frontLongRequest = 0f;
        float rearLongRequest = 0f;
        float requestedLongitudinalAccel;

        if (desiredAccel >= 0f)
        {
            float driveAccel = Math.Min(
                desiredAccel,
                CalculateDriveAccelLimit(state, config, input.Strategy)
            );
            frontLongRequest = driveAccel * config.FrontDriveShare;
            rearLongRequest = driveAccel - frontLongRequest;
            requestedLongitudinalAccel = driveAccel;
        }
        else
        {
            float brakeRequest = Math.Min(-desiredAccel, config.MaxBrakeAccel);
            AllocateBrakeRequest(
                brakeRequest,
                frontLatRequest,
                rearLatRequest,
                frontGrip,
                rearGrip,
                input.DriverInput.FrontBrakeBiasOffset,
                out float frontBrake,
                out float rearBrake
            );
            frontBrake = ApplyAntiLock(
                config, frontLatRequest, frontBrake, frontGrip);
            rearBrake = ApplyAntiLock(
                config, rearLatRequest, rearBrake, rearGrip);
            frontLongRequest = -frontBrake;
            rearLongRequest = -rearBrake;
            requestedLongitudinalAccel = -(frontBrake + rearBrake);
        }

        float tractionControlCutAccel = 0f;
        if (rearLongRequest > 0f)
        {
            float uncontrolledRearDrive = rearLongRequest;
            rearLongRequest = ApplyRearTractionControl(
                config,
                rearLatRequest,
                rearLongRequest,
                rearGrip
            );
            tractionControlCutAccel = Math.Max(
                0f,
                uncontrolledRearDrive - rearLongRequest
            );
        }

        float corneringEfficiency = Math.Clamp(input.CorneringEfficiency, 0.05f, 1f);
        AxleResult front = ResolveAxle(
            config, frontLatRequest, frontLongRequest, frontGrip, corneringEfficiency);
        AxleResult rear = ResolveAxle(
            config, rearLatRequest, rearLongRequest, rearGrip, corneringEfficiency);

        float actualLateralAccel = front.LateralAccel + rear.LateralAccel;
        float driveAccelActual = Math.Max(0f, front.LongitudinalAccel) + Math.Max(0f, rear.LongitudinalAccel);
        float brakeAccelActual = Math.Max(0f, -front.LongitudinalAccel) + Math.Max(0f, -rear.LongitudinalAccel);
        float axleLongitudinalAccel = front.LongitudinalAccel + rear.LongitudinalAccel;

        // What the tyre was worked, not what came out of it. A driver who
        // wastes part of the cornering still spent the tyre on all of it, and
        // charging the wear on the smaller number would hand the slower driver
        // longer-lasting tyres for being slow.
        float frontLateralUse = front.LateralUse;
        float rearLateralUse = rear.LateralUse;
        float frontLongitudinalUse = front.LongitudinalUse;
        float rearLongitudinalUse = rear.LongitudinalUse;
        float lateralUse = Math.Abs(actualLateralAccel) / totalGrip;
        float overLimit = Math.Max(front.OverLimit, rear.OverLimit);
        float actualYawAcceleration = CalculateYawAcceleration(
            config,
            front.LateralAccel,
            rear.LateralAccel
        );
        float rearSlideSeverity = CalculateRearSlideSeverity(
            front,
            rear,
            frontLatRequest,
            rearLatRequest
        );

        float sideslipLossAccel = CalculateSideslipLossAccel(
            actualLateralAccel,
            state.SideslipAngleRadians
        );
        float lossAccel = CalculateLossAccel(
            config,
            state.Speed,
            lateralUse,
            state.AirVelocityDeficit
        ) + sideslipLossAccel;
        float actualLongitudinalAccel = axleLongitudinalAccel - lossAccel;

        float oldSpeed = state.Speed;
        float newSpeed = Math.Max(0f, oldSpeed + actualLongitudinalAccel * dt);
        float averageSpeed = (oldSpeed + newSpeed) * 0.5f;

        float actualCurvature = averageSpeed > 0.5f
            ? actualLateralAccel / Math.Max(averageSpeed * averageSpeed, Epsilon)
            : 0f;
        referenceYawRate = averageSpeed * desiredCurvature;
        dynamicYawBlend = CalculateDynamicYawBlend(averageSpeed);
        float trajectoryYawRate = actualCurvature * averageSpeed;
        float headingDelta = trajectoryYawRate * dt;
        float velocityHeading = state.VelocityHeading;
        float travelHeading = velocityHeading + headingDelta * 0.5f;
        Vector2 travelDirection = new(MathF.Cos(travelHeading), MathF.Sin(travelHeading));
        float nextVelocityHeading = MathHelper.NormalizeAngle(velocityHeading + headingDelta);
        float dynamicYawRate = Math.Clamp(
            state.YawRateRadiansPerSecond + actualYawAcceleration * dt,
            -ReducedOrderDynamicsLimits.MaximumYawRateRadiansPerSecond,
            ReducedOrderDynamicsLimits.MaximumYawRateRadiansPerSecond
        );
        float nextYawRate = Lerp(trajectoryYawRate, dynamicYawRate, dynamicYawBlend);
        float dynamicBodyHeading = MathHelper.NormalizeAngle(
            state.Heading +
            (state.YawRateRadiansPerSecond + nextYawRate) * 0.5f * dt
        );
        float nextBodyHeading = LerpAngle(
            nextVelocityHeading,
            dynamicBodyHeading,
            dynamicYawBlend
        );
        float nextSideslipAngle = Math.Clamp(
            MathHelper.NormalizeAngle(nextVelocityHeading - nextBodyHeading),
            -ReducedOrderDynamicsLimits.MaximumBodySideslipRadians,
            ReducedOrderDynamicsLimits.MaximumBodySideslipRadians
        );
        nextBodyHeading = MathHelper.NormalizeAngle(
            nextVelocityHeading - nextSideslipAngle
        );

        state.Position += travelDirection * averageSpeed * dt;
        state.SideslipAngleRadians = nextSideslipAngle;
        state.YawRateRadiansPerSecond = nextYawRate;
        state.Heading = nextBodyHeading;
        state.Speed = newSpeed;
        float normalizedSideslip = Math.Clamp(
            Math.Abs(state.SideslipAngleRadians) /
            ReducedOrderDynamicsLimits.MaximumBodySideslipRadians,
            0f,
            1f
        );

        float drivePowerWatts = UpdateBattery(
            state,
            config,
            driveAccelActual,
            brakeAccelActual,
            averageSpeed,
            dt
        );
        float regenPowerWatts = CalculateRegenPower(config, brakeAccelActual, averageSpeed);

        float costedFrontOverLimit = CostedOverLimit(config, front.OverLimit);
        float costedRearOverLimit = CostedOverLimit(config, rear.OverLimit);
        UpdateTires(
            state.FrontLeft,
            config,
            tires,
            frontLateralUse,
            frontLongitudinalUse,
            costedFrontOverLimit,
            0f,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            dt,
            input.TireEnergyEfficiency
        );
        UpdateTires(
            state.FrontRight,
            config,
            tires,
            frontLateralUse,
            frontLongitudinalUse,
            costedFrontOverLimit,
            0f,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            dt,
            input.TireEnergyEfficiency
        );
        UpdateTires(
            state.RearLeft,
            config,
            tires,
            rearLateralUse,
            rearLongitudinalUse,
            costedRearOverLimit,
            normalizedSideslip,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            dt,
            input.TireEnergyEfficiency
        );
        UpdateTires(
            state.RearRight,
            config,
            tires,
            rearLateralUse,
            rearLongitudinalUse,
            costedRearOverLimit,
            normalizedSideslip,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            dt,
            input.TireEnergyEfficiency
        );

        float response = 1f - MathF.Exp(-config.LoadTransferResponse * dt);
        state.FilteredLongitudinalAccel = Lerp(state.FilteredLongitudinalAccel, actualLongitudinalAccel, response);
        state.FilteredLateralAccel = Lerp(state.FilteredLateralAccel, actualLateralAccel, response);

        state.Telemetry = new CarTelemetry(
            input.DriverInput,
            input.Strategy,
            requestedLateralAccel,
            actualLateralAccel,
            requestedLongitudinalAccel,
            actualLongitudinalAccel,
            lossAccel,
            actualCurvature,
            frontGrip,
            rearGrip,
            frontLateralUse,
            rearLateralUse,
            frontLongitudinalUse,
            rearLongitudinalUse,
            overLimit,
            drivePowerWatts,
            regenPowerWatts,
            tractionControlCutAccel,
            sideslipLossAccel,
            state.SideslipAngleRadians,
            rearSlideSeverity,
            referenceYawRate,
            state.YawRateRadiansPerSecond,
            actualYawAcceleration
        );

        state.Normalize();
    }

    private static float CalculateDriveAccelLimit(
        CarState state,
        CarConfig config,
        CarStrategy strategy
    )
    {
        return CalculateDriveAccelLimit(
            state,
            config,
            strategy,
            state.Speed
        );
    }

    private static float CalculateDriveAccelLimit(
        CarState state,
        CarConfig config,
        CarStrategy strategy,
        float speed
    )
    {
        if (state.BatterySoc <= 0f)
            return 0f;

        float socFactor = state.BatterySoc >= config.LowSocPowerLimitStart
            ? 1f
            : state.BatterySoc / Math.Max(config.LowSocPowerLimitStart, Epsilon);

        float forceLimitedAccel = config.MaxDriveAcceleration;
        float powerLimitedAccel =
            config.GetDrivePowerLimitWatts(strategy) /
            (config.MassKg * Math.Max(speed, config.MinPowerSpeed));

        return Math.Min(forceLimitedAccel, powerLimitedAccel) * socFactor;
    }

    private static float RemainingLongitudinalGrip(float grip, float lateralAcceleration)
    {
        float remainingSquared = grip * grip - lateralAcceleration * lateralAcceleration;
        return MathF.Sqrt(Math.Max(0f, remainingSquared));
    }

    /// <summary>
    /// Holds an axle's brakes back before the tyre gives up, the mirror of the
    /// traction control below and written the same way so the pair can be read
    /// together.
    /// </summary>
    private static float ApplyAntiLock(
        CarConfig config,
        float lateralRequest,
        float brakeRequest,
        float grip
    )
    {
        float strength = Math.Clamp(config.AntiLockStrength, 0f, 1f);
        if (brakeRequest <= 0f || grip <= Epsilon || strength <= 0f)
            return brakeRequest;

        float activationUse = Math.Clamp(
            config.AntiLockActivationUse,
            0.05f,
            1f
        );
        float available = RemainingLongitudinalGrip(
            grip * activationUse,
            lateralRequest
        );
        return Lerp(brakeRequest, Math.Min(brakeRequest, available), strength);
    }

    private static float ApplyRearTractionControl(
        CarConfig config,
        float lateralRequest,
        float driveRequest,
        float rearGrip
    )
    {
        float strength = Math.Clamp(config.TractionControlStrength, 0f, 1f);
        if (driveRequest <= 0f || rearGrip <= Epsilon || strength <= 0f)
            return driveRequest;

        float activationUse = Math.Clamp(
            config.TractionControlActivationUse,
            0.05f,
            1f
        );
        float activationGrip = rearGrip * activationUse;
        float availableAtActivation = RemainingLongitudinalGrip(
            activationGrip,
            lateralRequest
        );
        float targetDrive = Math.Min(driveRequest, availableAtActivation);
        return Lerp(driveRequest, targetDrive, strength);
    }

    private static float DistributedLongitudinalLimit(
        float frontCapacity,
        float rearCapacity,
        float frontShare
    )
    {
        float normalizedFrontShare = Math.Clamp(frontShare, 0f, 1f);
        float rearShare = 1f - normalizedFrontShare;
        float limit = float.PositiveInfinity;

        if (normalizedFrontShare > Epsilon)
            limit = Math.Min(limit, frontCapacity / normalizedFrontShare);
        if (rearShare > Epsilon)
            limit = Math.Min(limit, rearCapacity / rearShare);

        return float.IsFinite(limit) ? Math.Max(0f, limit) : 0f;
    }

    private static void AllocateBrakeRequest(
        float brakeRequest,
        float frontLateralRequest,
        float rearLateralRequest,
        float frontGrip,
        float rearGrip,
        float frontBrakeBiasOffset,
        out float frontBrake,
        out float rearBrake
    )
    {
        float frontCapacity = RemainingLongitudinalGrip(
            frontGrip,
            frontLateralRequest
        );
        float rearCapacity = RemainingLongitudinalGrip(
            rearGrip,
            rearLateralRequest
        );
        float frontShare = BiasedFrontBrakeShare(
            frontCapacity,
            rearCapacity,
            frontBrakeBiasOffset
        );
        if (frontCapacity + rearCapacity <= Epsilon)
        {
            float totalGrip = frontGrip + rearGrip;
            if (totalGrip <= Epsilon)
            {
                frontBrake = 0f;
                rearBrake = 0f;
                return;
            }

            // Both axles are already at their lateral limit. Preserve the
            // requested combined-demand behavior and let ResolveAxle clip it.
            frontShare = BiasedFrontBrakeShare(
                frontGrip,
                rearGrip,
                frontBrakeBiasOffset
            );
            frontBrake = brakeRequest * frontShare;
            rearBrake = brakeRequest - frontBrake;
            return;
        }

        frontBrake = brakeRequest * frontShare;
        rearBrake = brakeRequest - frontBrake;
    }

    private static float BiasedFrontBrakeShare(
        float frontCapacity,
        float rearCapacity,
        float frontBrakeBiasOffset
    )
    {
        float totalCapacity = frontCapacity + rearCapacity;
        float optimalFrontShare = totalCapacity <= Epsilon
            ? 0.5f
            : frontCapacity / totalCapacity;
        float finiteOffset = float.IsFinite(frontBrakeBiasOffset)
            ? Math.Clamp(frontBrakeBiasOffset, -0.25f, 0.25f)
            : 0f;
        return Math.Clamp(optimalFrontShare + finiteOffset, 0f, 1f);
    }

    /// <summary>
    /// What an axle actually delivers of what it was asked for, and how much
    /// of itself it spent doing so.
    ///
    /// The same corner costs one driver more tyre than another. Where the
    /// difference goes is not modelled in detail - it is line, hands, the
    /// hundred small things - only that it is spent: a driver who extracts
    /// nine tenths of what the tyre offers reaches the same corner speed
    /// having used a ninth more of it, and runs out of tyre that much sooner.
    ///
    /// The force itself is not discounted here. What the driver can reach at
    /// all is already lower, because the speed plan is drawn against the same
    /// figure, and taking it off the delivered force as well would charge the
    /// discount twice: the car would be planned to a corner speed it then
    /// could not hold, and would understeer wide of the line all lap.
    ///
    /// Only cornering. Applying the same discount to braking and driving would
    /// say that a slower driver cannot use the brakes, which is a different
    /// claim and a wrong one, and on a straight it would say nothing at all
    /// because the car is limited by its battery there and not by its tyres.
    /// </summary>
    private static AxleResult ResolveAxle(
        CarConfig config,
        float lateralRequest,
        float longitudinalRequest,
        float grip,
        float corneringEfficiency = 1f
    )
    {
        if (grip <= Epsilon)
            return default;

        float lateralUse = lateralRequest / (grip * corneringEfficiency);
        float longitudinalUse = longitudinalRequest / grip;
        float combinedUse = MathF.Sqrt(lateralUse * lateralUse + longitudinalUse * longitudinalUse);

        if (combinedUse <= 1f)
            return new AxleResult(
                lateralRequest,
                longitudinalRequest,
                Math.Max(0f, combinedUse - 1f),
                combinedUse,
                MathF.Abs(lateralUse),
                MathF.Abs(longitudinalUse)
            );

        float overLimit = combinedUse - 1f;
        float efficiency = CalculateOverLimitGripEfficiency(config, overLimit);
        float scale = efficiency / combinedUse;
        return new AxleResult(
            lateralRequest * scale,
            longitudinalRequest * scale,
            overLimit,
            combinedUse,
            MathF.Abs(lateralUse),
            MathF.Abs(longitudinalUse)
        );
    }

    private static LateralRequests AllocateLateralRequests(
        CarState state,
        CarConfig config,
        float totalLateralRequest,
        float referenceYawRate,
        float dynamicBlend
    )
    {
        float frontStaticShare = Math.Clamp(config.FrontStaticLoadShare, 0f, 1f);
        float staticFrontRequest = totalLateralRequest * frontStaticShare;
        if (dynamicBlend <= 0f)
            return new LateralRequests(
                staticFrontRequest,
                totalLateralRequest - staticFrontRequest
            );

        float wheelBase = Math.Max(config.WheelBaseMeters, Epsilon);
        float rearMomentArm = wheelBase * frontStaticShare;
        float yawResponseTime = Math.Max(config.YawResponseTimeSeconds, 0.05f);
        float sideslipRecoveryTime = Math.Max(
            config.SideslipRecoveryTimeSeconds,
            0.05f
        );
        float stabilizedYawRate = referenceYawRate +
                                  state.SideslipAngleRadians /
                                  sideslipRecoveryTime;
        float desiredYawAcceleration = Math.Clamp(
            (stabilizedYawRate - state.YawRateRadiansPerSecond) / yawResponseTime,
            -ReducedOrderDynamicsLimits.MaximumYawAccelerationRadiansPerSecondSquared,
            ReducedOrderDynamicsLimits.MaximumYawAccelerationRadiansPerSecondSquared
        );
        float yawInertiaPerMass = Math.Max(config.YawInertiaKgM2, Epsilon) /
                                  Math.Max(config.MassKg, Epsilon);
        float dynamicFrontRequest = (
            rearMomentArm * totalLateralRequest +
            yawInertiaPerMass * desiredYawAcceleration
        ) / wheelBase;
        float frontRequest = Lerp(
            staticFrontRequest,
            dynamicFrontRequest,
            dynamicBlend
        );
        return new LateralRequests(
            frontRequest,
            totalLateralRequest - frontRequest
        );
    }

    private static float CalculateYawAcceleration(
        CarConfig config,
        float frontLateralAcceleration,
        float rearLateralAcceleration
    )
    {
        float wheelBase = Math.Max(config.WheelBaseMeters, Epsilon);
        float rearMomentArm = wheelBase * Math.Clamp(
            config.FrontStaticLoadShare,
            0f,
            1f
        );
        float frontMomentArm = wheelBase - rearMomentArm;
        float yawMoment = config.MassKg * (
            frontMomentArm * frontLateralAcceleration -
            rearMomentArm * rearLateralAcceleration
        );
        return yawMoment / Math.Max(config.YawInertiaKgM2, Epsilon);
    }

    private static float CalculateDynamicYawBlend(float speed)
    {
        float t = Math.Clamp(
            (speed - DynamicYawMinimumSpeed) / DynamicYawBlendRange,
            0f,
            1f
        );
        return t * t * (3f - 2f * t);
    }

    private static float CalculateRearSlideSeverity(
        AxleResult front,
        AxleResult rear,
        float frontLateralRequest,
        float rearLateralRequest
    )
    {
        float overLimitImbalance = Math.Max(0f, rear.OverLimit - front.OverLimit);
        float frontDelivery = RelativeLateralDelivery(frontLateralRequest, front.LateralAccel);
        float rearDelivery = RelativeLateralDelivery(rearLateralRequest, rear.LateralAccel);
        float deliveryImbalance = Math.Max(0f, frontDelivery - rearDelivery);
        float rearNearLimit = Math.Max(
            0f,
            rear.CombinedRequest - RearSlipOnsetCombinedUse
        );
        float rearDominance = Math.Clamp(
            (rear.CombinedRequest - front.CombinedRequest) / RearSlipDominanceRange,
            0f,
            1f
        );
        float utilizationSeverity = rearNearLimit * rearDominance;
        return Math.Max(
            overLimitImbalance,
            Math.Max(deliveryImbalance, utilizationSeverity)
        );
    }

    private static float RelativeLateralDelivery(float request, float actual)
    {
        float absoluteRequest = Math.Abs(request);
        if (absoluteRequest <= Epsilon)
            return 1f;

        return Math.Clamp(Math.Abs(actual) / absoluteRequest, 0f, 1f);
    }

    /// <summary>
    /// What the car gives back to the air, the road and its own tyres.
    ///
    /// Air resistance is fought against the air the car is moving through and
    /// not against the ground, so a car whose air is already being dragged
    /// along by somebody in front meets less wind and pays the square of what
    /// is left. Writing it as a share of drag removed instead would be writing
    /// down the answer; written this way the squaring is the reason a tow is
    /// worth so much more at a car length than at ten.
    ///
    /// Only that part is reduced. Rolling resistance does not care what is in
    /// front, and the tyre scrub of cornering is not an air loss at all.
    /// </summary>
    private static float CalculateLossAccel(
        CarConfig config,
        float speed,
        float lateralUse,
        float airVelocityDeficit
    )
    {
        if (speed <= 0.01f)
            return 0f;

        float metAir = 1f - Math.Clamp(airVelocityDeficit, 0f, 1f);
        return
            config.RollingDragAccel +
            config.AeroDragAccelPerSpeedSquared * speed * speed *
            metAir * metAir +
            config.CorneringScrubAccel * lateralUse * lateralUse;
    }

    private static float CalculateSideslipLossAccel(
        float lateralAcceleration,
        float sideslipAngle
    )
    {
        return Math.Abs(
            lateralAcceleration * MathF.Sin(sideslipAngle)
        ) * SideslipEnergyLossScale;
    }

    private static float CalculateOverLimitGripEfficiency(CarConfig config, float overLimit)
    {
        float minEfficiency = Math.Clamp(config.OverLimitMinGripEfficiency, 0f, 1f);
        float t = Math.Clamp(overLimit / Math.Max(config.OverLimitCostCap, Epsilon), 0f, 1f);
        return 1f + (minEfficiency - 1f) * t;
    }

    private static float CostedOverLimit(CarConfig config, float overLimit)
    {
        return Math.Clamp(overLimit, 0f, Math.Max(0f, config.OverLimitCostCap));
    }

    private static float UpdateBattery(
        CarState state,
        CarConfig config,
        float driveAccel,
        float brakeAccel,
        float speed,
        float dt
    )
    {
        float drivePower = driveAccel <= 0f
            ? 0f
            : config.MassKg * driveAccel * speed / Math.Max(config.BatteryDriveEfficiency, Epsilon);
        float regenPower = CalculateRegenPower(config, brakeAccel, speed);

        float netEnergy = (drivePower - regenPower) * dt;

        state.BatterySoc = Math.Clamp(
            state.BatterySoc - netEnergy / Math.Max(config.BatteryCapacityJoules, Epsilon),
            0f,
            1f
        );

        return drivePower;
    }

    private static float CalculateRegenPower(CarConfig config, float brakeAccel, float speed)
    {
        if (brakeAccel <= 0f || speed <= 0f)
            return 0f;

        float brakePower = config.MassKg * brakeAccel * speed;
        return Math.Min(brakePower * config.RegenEfficiency, config.RegenPowerCapWatts);
    }

    private static void UpdateTires(
        TireState tire,
        CarConfig config,
        TireConfig tires,
        float lateralUse,
        float longitudinalUse,
        float overLimit,
        float sideslipRatio,
        float airTempC,
        float trackTempC,
        float speed,
        float dt,
        float tireEnergyEfficiency
    )
    {
        float loadScale = Math.Max(
            MinimumTireHeatLoadScale,
            tire.LoadN / Math.Max(config.MassKg * Gravity * 0.25f, Epsilon)
        );
        float thermalOverLimit = Math.Min(overLimit, MaximumThermalOverLimit);
        float normalizedLateralUse = Math.Clamp(Math.Abs(lateralUse), 0f, 1f);
        float normalizedLongitudinalUse = Math.Clamp(
            Math.Abs(longitudinalUse),
            0f,
            1f
        );
        float tireWorkSpeedMultiplier = Math.Clamp(
            Math.Max(0f, speed) /
            TireConfig.TireWorkReferenceSpeedMps,
            0f,
            TireConfig.MaximumTireWorkSpeedMultiplier
        );
        float longitudinalHeatUse = MathF.Pow(
            normalizedLongitudinalUse,
            TireConfig.LongitudinalHeatExponent
        );
        float combinedUse = Math.Clamp(
            MathF.Sqrt(
                normalizedLateralUse * normalizedLateralUse +
                normalizedLongitudinalUse * normalizedLongitudinalUse
            ),
            0f,
            1f
        );
        float nearLimitHeatStart = Math.Clamp(
            TireConfig.NearLimitHeatStartUse,
            0f,
            1f - Epsilon
        );
        float nearLimitProgress = Math.Clamp(
            (combinedUse - nearLimitHeatStart) /
            Math.Max(1f - nearLimitHeatStart, Epsilon),
            0f,
            1f
        );
        float nearLimitSmoothStep =
            nearLimitProgress * nearLimitProgress *
            (3f - 2f * nearLimitProgress);
        float slipHeatMultiplier =
            1f + Math.Max(0f, tires.NearLimitHeatGain) * nearLimitSmoothStep;
        float rollingHeatSpeedFactor = Math.Max(0f, speed) /
                                       (
                                           Math.Max(0f, speed) +
                                           TireConfig.RollingHeatReferenceSpeedMps
                                       );
        float rollingSurfaceHeat = TireConfig.RollingSurfaceHeatRate *
                                   loadScale * rollingHeatSpeedFactor;

        float driverSensitiveEnergyFactor = Math.Clamp(tireEnergyEfficiency, 0.9f, 1.1f);
        float directionalHeat =
            tires.LateralHeatRate * normalizedLateralUse * normalizedLateralUse +
            tires.LongitudinalHeatRate * longitudinalHeatUse;
        float surfaceHeat =
            tireWorkSpeedMultiplier * slipHeatMultiplier *
            directionalHeat * driverSensitiveEnergyFactor +
            TireConfig.OverLimitHeatRate * thermalOverLimit * thermalOverLimit +
            TireConfig.SideslipHeatRate * sideslipRatio * sideslipRatio;
        surfaceHeat *= loadScale;
        surfaceHeat += rollingSurfaceHeat;

        float airCoolingMultiplier = CalculateAirCoolingMultiplier(speed);
        float surfaceToAir = TireConfig.SurfaceCoolingRate * airCoolingMultiplier * (tire.SurfaceTempC - airTempC);
        float surfaceToTrack = TireConfig.TrackSurfaceTransferRate *
                               (tire.SurfaceTempC - trackTempC);
        float surfaceToCore = TireConfig.SurfaceCoreTransferRate * (tire.SurfaceTempC - tire.CoreTempC);
        float rollingCoreHeat = TireConfig.RollingCoreHeatRate *
                                loadScale * rollingHeatSpeedFactor;

        tire.SurfaceTempC += (
            surfaceHeat - surfaceToAir - surfaceToTrack - surfaceToCore
        ) * dt;
        // The carcass makes its own heat by flexing, so without somewhere to put
        // it the only way out is backwards through the tread, and it has to
        // stand hotter than the tread for that to happen - permanently, by an
        // amount nothing the driver does can change. It does have somewhere to
        // put it: the rim, which is metal and has air moving over it, and the
        // gas inside the tyre. Both are stood in for here by the outside air.
        //
        // Far slower than the tread, which is lying on the road inside its own
        // hurricane. On its own this path takes something like twenty minutes,
        // against the tread's forty seconds, which is what makes a soaked
        // carcass something a stint has to live with rather than something a
        // straight fixes.
        float coreToAir = TireConfig.CoreAirCoolingRate *
                          airCoolingMultiplier *
                          (tire.CoreTempC - airTempC);
        tire.CoreTempC += (
            rollingCoreHeat + surfaceToCore - coreToAir
        ) / TireConfig.CoreHeatCapacityRatio * dt;

        float tempWearFactor = 1f + Math.Max(0f, tire.SurfaceTempC - tires.HotWearStartTempC) * tires.HotWearSlope;
        float directionalWear =
            tires.LateralWearRate * lateralUse * lateralUse +
            tires.LongitudinalWearRate * longitudinalUse * longitudinalUse;
        float tireWorkWear =
            directionalWear * driverSensitiveEnergyFactor +
            tires.OverLimitWearRate * thermalOverLimit * thermalOverLimit +
            tires.SideslipWearRate * sideslipRatio * sideslipRatio;
        float wearDelta = tireWorkWear * tireWorkSpeedMultiplier *
                          tempWearFactor * loadScale * dt;

        tire.Wear = Math.Clamp(tire.Wear + wearDelta, 0f, 1f);
    }

    private static float CalculateAirCoolingMultiplier(float speed)
    {
        float speedFactor = Math.Max(0f, speed) /
                            (
                                Math.Max(0f, speed) +
                                TireConfig.SpeedCoolingReferenceMps
                            );
        return 1f +
               (TireConfig.MaximumSpeedCoolingMultiplier - 1f) * speedFactor;
    }

    private static WheelLoads CalculateWheelLoads(CarState state, CarConfig config)
    {
        return CalculateWheelLoads(
            config,
            state.FilteredLongitudinalAccel,
            state.FilteredLateralAccel,
            state.Speed
        );
    }

    /// <summary>
    /// What each tyre is being pressed into the road with.
    ///
    /// Weight, plus what the air is pushing down with, moved about by
    /// accelerating, braking and cornering. The air's share is the reason a
    /// quick corner is worth more than a slow one of the same radius, and
    /// leaving it out makes every corner the same corner: measured against
    /// published lap times, a circuit whose character is its fast corners came
    /// out slower than one whose character is a long straight, which is the
    /// wrong way round.
    ///
    /// Downforce is shared front to rear in the same proportion as weight.
    /// Real cars are trimmed away from that, and that trim is a setup choice
    /// this model does not offer yet.
    /// </summary>
    private static WheelLoads CalculateWheelLoads(
        CarConfig config,
        float longitudinalAcceleration,
        float lateralAcceleration,
        float speed
    )
    {
        float downforceAcceleration = config.DownforceAccelPerSpeedSquared *
                                      speed * speed;
        float totalLoad = config.MassKg * (Gravity + downforceAcceleration);
        float frontLoad = totalLoad * config.FrontStaticLoadShare;
        frontLoad -= config.MassKg * longitudinalAcceleration * config.CenterOfGravityHeightMeters /
                     Math.Max(config.WheelBaseMeters, Epsilon);
        frontLoad = Math.Clamp(frontLoad, 0f, totalLoad);

        float rearLoad = totalLoad - frontLoad;
        float lateralTransfer = config.MassKg * lateralAcceleration * config.CenterOfGravityHeightMeters /
                                Math.Max(config.TrackWidthMeters, Epsilon);
        float frontTransfer = lateralTransfer * frontLoad / Math.Max(totalLoad, Epsilon);
        float rearTransfer = lateralTransfer - frontTransfer;

        float fl = frontLoad * 0.5f - frontTransfer * 0.5f;
        float fr = frontLoad * 0.5f + frontTransfer * 0.5f;
        float rl = rearLoad * 0.5f - rearTransfer * 0.5f;
        float rr = rearLoad * 0.5f + rearTransfer * 0.5f;

        float minWheelLoad = totalLoad * 0.25f * config.MinimumWheelLoadShare;
        return new WheelLoads(
            Math.Max(fl, minWheelLoad),
            Math.Max(fr, minWheelLoad),
            Math.Max(rl, minWheelLoad),
            Math.Max(rr, minWheelLoad)
        );
    }

    private static void ApplyWheelLoads(CarState state, WheelLoads loads)
    {
        state.FrontLeft.LoadN = loads.FrontLeft;
        state.FrontRight.LoadN = loads.FrontRight;
        state.RearLeft.LoadN = loads.RearLeft;
        state.RearRight.LoadN = loads.RearRight;
    }

    private static float CalculateAxleGripAccel(
        CarConfig config,
        TireConfig tires,
        TireState left,
        TireState right
    )
    {
        float leftForce = left.LoadN * CalculateTireMu(tires, left);
        float rightForce = right.LoadN * CalculateTireMu(tires, right);
        return (leftForce + rightForce) / Math.Max(config.MassKg, Epsilon);
    }

    private static float CalculateTireMu(TireConfig tires, TireState tire)
    {
        float tempGrip = 1f;
        float idealLow = Math.Min(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        float idealHigh = Math.Max(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        if (tire.SurfaceTempC < idealLow)
            tempGrip -= (idealLow - tire.SurfaceTempC) * tires.ColdGripLossPerC;
        else if (tire.SurfaceTempC > idealHigh)
            tempGrip -= (tire.SurfaceTempC - idealHigh) * tires.HotGripLossPerC;

        tempGrip -= Math.Max(0f, tire.CoreTempC - tires.CoreOverheatTempC) * tires.CoreOverheatGripLossPerC;

        float wear = Math.Clamp(tire.Wear, 0f, 1f);
        float cliffStart = Math.Clamp(tires.WearCliffStart, 0f, 1f);
        float cliffProgress = Math.Clamp(
            (wear - cliffStart) / Math.Max(1f - cliffStart, Epsilon),
            0f,
            1f
        );
        float smoothCliff = cliffProgress * cliffProgress *
                            (3f - 2f * cliffProgress);
        float wearGrip = 1f -
                         Math.Max(0f, tires.WearLinearGripLoss) * wear -
                         Math.Max(0f, tires.WearCliffGripLoss) * smoothCliff;
        return tires.BaseMu *
               Math.Clamp(tempGrip, MinimumTemperatureGripFactor, MaximumTemperatureGripFactor) *
               Math.Clamp(wearGrip, MinimumWearGripFactor, 1f);
    }

    private static float Lerp(float from, float to, float weight)
    {
        return from + (to - from) * Math.Clamp(weight, 0f, 1f);
    }

    private static float LerpAngle(float from, float to, float weight)
    {
        float delta = MathHelper.NormalizeAngle(to - from);
        return MathHelper.NormalizeAngle(from + delta * Math.Clamp(weight, 0f, 1f));
    }

    private readonly record struct LateralRequests(float Front, float Rear);

    private readonly record struct AxleResult(
        float LateralAccel,
        float LongitudinalAccel,
        float OverLimit,
        float CombinedRequest,
        float LateralUse,
        float LongitudinalUse
    );

    private readonly record struct WheelLoads(
        float FrontLeft,
        float FrontRight,
        float RearLeft,
        float RearRight
    );
}
