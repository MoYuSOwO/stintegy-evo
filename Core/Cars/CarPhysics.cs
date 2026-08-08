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

    internal static float EffectiveDownforceAccelPerSpeedSquared(
        CarState state,
        CarConfig config
    )
    {
        return EffectiveDownforceAccelPerSpeedSquared(
            config,
            state.AirVelocityDeficit,
            state.WakeDownforceLoss
        );
    }

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
            speed,
            state.AirVelocityDeficit,
            state.WakeDownforceLoss
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

    /// <summary>
    /// Only how hard the car will corner, at a speed, round a curvature, with
    /// the weight where a given longitudinal acceleration puts it.
    ///
    /// The same figure the full estimate returns, and reached the same way, but
    /// without the drive limit, the brake distribution or the losses beside it.
    /// A speed plan that wants to know whether a corner survives the braking it
    /// is planning has to ask this once per point of the horizon, every frame,
    /// for every car, and everything the full answer carries is thrown away
    /// unread - including the battery model, which is the dearest part of it.
    /// </summary>
    internal static float EstimateLateralAccelerationLimit(
        CarState state,
        CarConfig config,
        TireConfig tires,
        float speed,
        float curvature,
        float gripUsage,
        float assumedLongitudinalAcceleration,
        float corneringEfficiency
    )
    {
        WheelLoads loads = CalculateWheelLoads(
            config,
            assumedLongitudinalAcceleration,
            speed * speed * curvature,
            speed,
            state.AirVelocityDeficit,
            state.WakeDownforceLoss
        );
        float usage = Math.Clamp(gripUsage, 0.05f, 1f);
        float mass = Math.Max(config.MassKg, Epsilon);
        float frontGrip = (
            loads.FrontLeft * CalculateTireMu(tires, state.FrontLeft) +
            loads.FrontRight * CalculateTireMu(tires, state.FrontRight)
        ) / mass * usage;
        float rearGrip = (
            loads.RearLeft * CalculateTireMu(tires, state.RearLeft) +
            loads.RearRight * CalculateTireMu(tires, state.RearRight)
        ) / mass * usage;

        float frontDemandShare = Math.Clamp(config.FrontStaticLoadShare, 0f, 1f);
        float rearDemandShare = 1f - frontDemandShare;
        float frontLimit = frontDemandShare <= Epsilon
            ? float.PositiveInfinity
            : frontGrip / frontDemandShare;
        float rearLimit = rearDemandShare <= Epsilon
            ? float.PositiveInfinity
            : rearGrip / rearDemandShare;
        return MathF.Max(
            0f,
            MathF.Min(frontLimit, rearLimit) *
            Math.Clamp(corneringEfficiency, 0.05f, 1f)
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
        float limitSettleUse = MathF.Max(input.LimitSettleUse, 0.5f);
        AxleResult front = ResolveAxle(
            config, frontLatRequest, frontLongRequest, frontGrip,
            corneringEfficiency, limitSettleUse);
        AxleResult rear = ResolveAxle(
            config, rearLatRequest, rearLongRequest, rearGrip,
            corneringEfficiency, limitSettleUse);

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
        float frontBrakeUse = front.LongitudinalAccel < 0f
            ? frontLongitudinalUse
            : 0f;
        float rearBrakeUse = rear.LongitudinalAccel < 0f
            ? rearLongitudinalUse
            : 0f;
        AxleLateralWorkScales lateralWorkScales =
            CalculateAxleLateralWorkScales(state, config, front, rear);
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
        float coolingAirSpeed = averageSpeed *
                                (
                                    1f - Math.Clamp(
                                        state.AirVelocityDeficit,
                                        0f,
                                        1f
                                    )
                                );
        UpdateTires(
            state.FrontLeft,
            config,
            tires,
            frontLateralUse,
            frontLongitudinalUse,
            frontBrakeUse,
            lateralWorkScales.FrontHeat,
            lateralWorkScales.FrontWear,
            costedFrontOverLimit,
            0f,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            state.WakeDownforceLoss,
            dt,
            input.TireEnergyEfficiency
        );
        UpdateTires(
            state.FrontRight,
            config,
            tires,
            frontLateralUse,
            frontLongitudinalUse,
            frontBrakeUse,
            lateralWorkScales.FrontHeat,
            lateralWorkScales.FrontWear,
            costedFrontOverLimit,
            0f,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            state.WakeDownforceLoss,
            dt,
            input.TireEnergyEfficiency
        );
        UpdateTires(
            state.RearLeft,
            config,
            tires,
            rearLateralUse,
            rearLongitudinalUse,
            rearBrakeUse,
            lateralWorkScales.RearHeat,
            lateralWorkScales.RearWear,
            costedRearOverLimit,
            normalizedSideslip,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            state.WakeDownforceLoss,
            dt,
            input.TireEnergyEfficiency
        );
        UpdateTires(
            state.RearRight,
            config,
            tires,
            rearLateralUse,
            rearLongitudinalUse,
            rearBrakeUse,
            lateralWorkScales.RearHeat,
            lateralWorkScales.RearWear,
            costedRearOverLimit,
            normalizedSideslip,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            coolingAirSpeed,
            state.WakeDownforceLoss,
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

        float socFactor = 1f;
        if (state.BatterySoc < config.LowSocPowerLimitStart)
        {
            socFactor = MathF.Pow(
                Math.Clamp(
                    state.BatterySoc /
                    Math.Max(config.LowSocPowerLimitStart, Epsilon),
                    0f,
                    1f
                ),
                MathF.Max(config.LowSocPowerFalloffExponent, 1f)
            );
        }

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
        float corneringEfficiency = 1f,
        float limitSettleUse = float.PositiveInfinity
    )
    {
        if (grip <= Epsilon)
            return default;

        float lateralUse = lateralRequest / (grip * corneringEfficiency);
        float longitudinalUse = longitudinalRequest / grip;
        float combinedUse = MathF.Sqrt(lateralUse * lateralUse + longitudinalUse * longitudinalUse);

        // Asked for more than the axle holds, the driver gets first go at
        // putting it right, and only what they leave behind reaches the tyre.
        // Here rather than anywhere in the driver because this is where the
        // number they are reacting to exists: one axle's share of one axle's
        // grip, which is the thing that actually lets go. Worked out from what
        // the whole car is doing, the loose end averages away against the end
        // that is fine, and the driver never feels the one that matters.
        if (combinedUse > 1f && limitSettleUse < combinedUse)
        {
            float given = limitSettleUse / combinedUse;
            lateralRequest *= given;
            longitudinalRequest *= given;
            lateralUse *= given;
            longitudinalUse *= given;
            combinedUse = limitSettleUse;
        }

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
        float brakeUse,
        float lateralHeatScale,
        float lateralWearScale,
        float overLimit,
        float sideslipRatio,
        float airTempC,
        float trackTempC,
        float speed,
        float coolingAirSpeed,
        float wakeDownforceLoss,
        float dt,
        float tireEnergyEfficiency
    )
    {
        float loadScale = CalculateTireWorkLoadScale(tire, config);
        float thermalOverLimit = Math.Min(overLimit, MaximumThermalOverLimit);
        float normalizedLateralUse = Math.Clamp(Math.Abs(lateralUse), 0f, 1f);
        float normalizedLongitudinalUse = Math.Clamp(
            Math.Abs(longitudinalUse),
            0f,
            1f
        );
        float normalizedBrakeUse = Math.Clamp(brakeUse, 0f, 1f);
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
        float partialSlipHeat = Math.Max(0f, tires.NearLimitHeatRate) *
                                nearLimitSmoothStep * nearLimitSmoothStep;
        float rollingHeatSpeedFactor = Math.Max(0f, speed) /
                                       (
                                           Math.Max(0f, speed) +
                                           TireConfig.RollingHeatReferenceSpeedMps
                                       );
        float rollingSurfaceHeat = TireConfig.RollingSurfaceHeatRate *
                                   loadScale * rollingHeatSpeedFactor;

        float driverSensitiveEnergyFactor = Math.Clamp(tireEnergyEfficiency, 0.9f, 1.1f);
        float directionalHeat =
            tires.LateralHeatRate * normalizedLateralUse * normalizedLateralUse *
            lateralHeatScale +
            tires.LongitudinalHeatRate * longitudinalHeatUse;
        float wakeCorneringHeat = TireConfig.WakeCorneringHeatRate *
                                   Math.Clamp(
                                       wakeDownforceLoss,
                                       0f,
                                       0.1f
                                   ) *
                                   normalizedLateralUse *
                                   normalizedLateralUse *
                                   lateralHeatScale *
                                   tireWorkSpeedMultiplier;
        float surfaceHeat =
            tireWorkSpeedMultiplier *
            (directionalHeat + partialSlipHeat) *
            driverSensitiveEnergyFactor +
            TireConfig.OverLimitHeatRate * thermalOverLimit * thermalOverLimit +
            TireConfig.SideslipHeatRate * sideslipRatio * sideslipRatio +
            wakeCorneringHeat;
        surfaceHeat *= loadScale;
        surfaceHeat += rollingSurfaceHeat;

        float airCoolingMultiplier = CalculateAirCoolingMultiplier(
            coolingAirSpeed
        );
        float surfaceToAir = TireConfig.SurfaceCoolingRate * airCoolingMultiplier * (tire.SurfaceTempC - airTempC);
        float surfaceToTrack = TireConfig.TrackSurfaceTransferRate *
                               (tire.SurfaceTempC - trackTempC);
        float surfaceToCore = TireConfig.SurfaceCoreTransferRate * (tire.SurfaceTempC - tire.CoreTempC);
        float rollingCoreHeat = TireConfig.RollingCoreHeatRate *
                                loadScale * rollingHeatSpeedFactor;
        float brakeCoreHeat = TireConfig.BrakeCoreHeatRate *
                              normalizedBrakeUse *
                              tireWorkSpeedMultiplier *
                              loadScale;

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
            rollingCoreHeat + brakeCoreHeat + surfaceToCore - coreToAir
        ) / TireConfig.CoreHeatCapacityRatio * dt;

        float tempWearFactor = CalculateTemperatureWearFactor(
            tires,
            tire.SurfaceTempC
        );
        float directionalWear =
            tires.LateralWearRate * lateralUse * lateralUse *
            lateralWearScale +
            tires.LongitudinalWearRate * longitudinalUse * longitudinalUse;
        float partialSlipWear = Math.Max(0f, tires.NearLimitWearRate) *
                                MathF.Pow(
                                    combinedUse,
                                    TireConfig.NearLimitWearExponent
                                );
        float tireWorkWear =
            (directionalWear + partialSlipWear) *
            driverSensitiveEnergyFactor +
            tires.OverLimitWearRate * thermalOverLimit * thermalOverLimit +
            tires.SideslipWearRate * sideslipRatio * sideslipRatio;
        float wearDelta = tireWorkWear * tireWorkSpeedMultiplier *
                          tempWearFactor * loadScale * dt;

        tire.Wear = Math.Clamp(tire.Wear + wearDelta, 0f, 1f);
    }

    private static AxleLateralWorkScales CalculateAxleLateralWorkScales(
        CarState state,
        CarConfig config,
        AxleResult front,
        AxleResult rear
    )
    {
        float frontLoadWeight =
            CalculateTireWorkLoadScale(state.FrontLeft, config) +
            CalculateTireWorkLoadScale(state.FrontRight, config);
        float rearLoadWeight =
            CalculateTireWorkLoadScale(state.RearLeft, config) +
            CalculateTireWorkLoadScale(state.RearRight, config);

        // Lateral rubber work is force times deformation. In this reduced
        // model deformation is not a state of its own, so use the small-angle
        // relation deformation ~= force * compliance. The common mass and
        // rear compliance cancel when only the front/rear share is needed.
        float frontForce = Math.Abs(front.LateralAccel);
        float rearForce = Math.Abs(rear.LateralAccel);
        float frontCompliance = Math.Max(
            0f,
            config.FrontLateralComplianceRatio
        );
        float physicalFrontWeight =
            frontForce * frontForce * frontCompliance;
        float physicalRearWeight = rearForce * rearForce;
        float physicalTotalWeight =
            physicalFrontWeight + physicalRearWeight;
        if (physicalTotalWeight <= Epsilon)
            return AxleLateralWorkScales.Identity;

        float targetFrontShare = physicalFrontWeight / physicalTotalWeight;
        float normalizedFrontUse = Math.Clamp(front.LateralUse, 0f, 1f);
        float normalizedRearUse = Math.Clamp(rear.LateralUse, 0f, 1f);
        AxleScales heat = RedistributeAxleWork(
            normalizedFrontUse * normalizedFrontUse * frontLoadWeight,
            normalizedRearUse * normalizedRearUse * rearLoadWeight,
            targetFrontShare
        );
        AxleScales wear = RedistributeAxleWork(
            front.LateralUse * front.LateralUse * frontLoadWeight,
            rear.LateralUse * rear.LateralUse * rearLoadWeight,
            targetFrontShare
        );
        return new AxleLateralWorkScales(
            heat.Front,
            heat.Rear,
            wear.Front,
            wear.Rear
        );
    }

    private static AxleScales RedistributeAxleWork(
        float currentFront,
        float currentRear,
        float targetFrontShare
    )
    {
        float total = currentFront + currentRear;
        if (total <= Epsilon)
            return AxleScales.Identity;

        float frontTarget = total * Math.Clamp(targetFrontShare, 0f, 1f);
        float rearTarget = total - frontTarget;
        float frontScale = currentFront <= Epsilon
            ? 1f
            : frontTarget / currentFront;
        float rearScale = currentRear <= Epsilon
            ? 1f
            : rearTarget / currentRear;
        return new AxleScales(frontScale, rearScale);
    }

    private static float CalculateTireWorkLoadScale(
        TireState tire,
        CarConfig config
    )
    {
        return Math.Max(
            MinimumTireHeatLoadScale,
            tire.LoadN /
            Math.Max(config.MassKg * Gravity * 0.25f, Epsilon)
        );
    }

    private static float CalculateTemperatureWearFactor(
        TireConfig tires,
        float surfaceTempC
    )
    {
        float idealLow = Math.Min(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        float idealHigh = Math.Max(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        float coldDistance = Math.Max(0f, idealLow - surfaceTempC);
        float hotDistance = Math.Max(0f, surfaceTempC - idealHigh);
        return 1f +
               Math.Max(0f, tires.ColdWearPerCSquared) *
               coldDistance * coldDistance +
               Math.Max(0f, tires.HotWearPerCSquared) *
               hotDistance * hotDistance;
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
            state.Speed,
            state.AirVelocityDeficit,
            state.WakeDownforceLoss
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
        float speed,
        float airVelocityDeficit,
        float wakeDownforceLoss
    )
    {
        float downforceAcceleration = EffectiveDownforceAccelPerSpeedSquared(
            config,
            airVelocityDeficit,
            wakeDownforceLoss
        ) * speed * speed;
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

    private static float EffectiveDownforceAccelPerSpeedSquared(
        CarConfig config,
        float airVelocityDeficit,
        float wakeDownforceLoss
    )
    {
        float metAir = 1f - Math.Clamp(airVelocityDeficit, 0f, 1f);
        float usableDownforce = 1f - Math.Clamp(wakeDownforceLoss, 0f, 1f);
        return MathF.Max(
            0f,
            config.DownforceAccelPerSpeedSquared *
            metAir * metAir *
            usableDownforce
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
        {
            float coldDistance = idealLow - tire.SurfaceTempC;
            tempGrip -= coldDistance * coldDistance *
                        tires.ColdGripLossPerCSquared;
        }
        else if (tire.SurfaceTempC > idealHigh)
        {
            float hotDistance = tire.SurfaceTempC - idealHigh;
            tempGrip -= hotDistance * hotDistance *
                        tires.HotGripLossPerCSquared;
        }

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

    private readonly record struct AxleScales(float Front, float Rear)
    {
        public static AxleScales Identity => new(1f, 1f);
    }

    private readonly record struct AxleLateralWorkScales(
        float FrontHeat,
        float RearHeat,
        float FrontWear,
        float RearWear
    )
    {
        public static AxleLateralWorkScales Identity => new(1f, 1f, 1f, 1f);
    }

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
