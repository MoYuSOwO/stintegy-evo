using System;
using System.Numerics;
using TheStint.Core.Util;

namespace TheStint.Core.Cars;

public static class CarPhysics
{
    private const float Gravity = 9.80665f;
    private const float Epsilon = 1e-5f;
    private const float MinimumTireHeatLoadScale = 0.2f;
    private const float MaximumThermalOverLimit = 1f;
    private const float MinimumTemperatureGripFactor = 0.55f;
    private const float MaximumTemperatureGripFactor = 1.08f;
    private const float MinimumWearGripFactor = 0.45f;
    private const float MaximumBodySideslipRadians = 0.174532925f;
    private const float RearSlipOnsetCombinedUse = 0.82f;
    private const float RearSlipDominanceRange = 0.2f;
    private const float DynamicYawMinimumSpeed = 5f;
    private const float DynamicYawBlendRange = 5f;
    private const float MaximumYawAccelerationRadiansPerSecondSquared = 2f;
    private const float MaximumYawRateRadiansPerSecond = 2.5f;
    private const float SideslipEnergyLossScale = 1f;

    internal static CarPerformanceLimits EstimatePerformanceLimits(
        CarState state,
        CarConfig config,
        TireConfig tires,
        CarStrategy strategy,
        float speed,
        float curvature,
        float gripUsage = 1f,
        float assumedLongitudinalAcceleration = 0f
    )
    {
        float lateralAcceleration = speed * speed * curvature;
        WheelLoads loads = CalculateWheelLoads(
            config,
            assumedLongitudinalAcceleration,
            lateralAcceleration
        );
        float usage = Math.Clamp(gripUsage, 0.05f, 1f);
        float frontGrip = (
            loads.FrontLeft * CalculateTireMu(tires, strategy.TireMode, state.FrontLeft) +
            loads.FrontRight * CalculateTireMu(tires, strategy.TireMode, state.FrontRight)
        ) / Math.Max(config.MassKg, Epsilon) * usage;
        float rearGrip = (
            loads.RearLeft * CalculateTireMu(tires, strategy.TireMode, state.RearLeft) +
            loads.RearRight * CalculateTireMu(tires, strategy.TireMode, state.RearRight)
        ) / Math.Max(config.MassKg, Epsilon) * usage;

        float frontDemandShare = Math.Clamp(config.FrontStaticLoadShare, 0f, 1f);
        float rearDemandShare = 1f - frontDemandShare;
        float frontLateralLimit = frontDemandShare <= Epsilon
            ? float.PositiveInfinity
            : frontGrip / frontDemandShare;
        float rearLateralLimit = rearDemandShare <= Epsilon
            ? float.PositiveInfinity
            : rearGrip / rearDemandShare;
        float lateralLimit = Math.Min(frontLateralLimit, rearLateralLimit);

        float frontLateral = lateralAcceleration * frontDemandShare;
        float rearLateral = lateralAcceleration * rearDemandShare;
        float frontLongitudinal = RemainingLongitudinalGrip(frontGrip, frontLateral);
        float rearLongitudinal = RemainingLongitudinalGrip(rearGrip, rearLateral);

        float gripDriveLimit = DistributedDriveLimit(
            frontLongitudinal,
            rearLongitudinal,
            config.FrontDriveShare
        );
        float batteryDriveLimit = CalculateDriveAccelLimit(
            state,
            config,
            strategy.BatteryMode,
            speed
        );
        float maximumDrive = Math.Min(
            config.MaxDriveAccelRequest,
            Math.Min(gripDriveLimit, batteryDriveLimit)
        );
        float maximumBrake = Math.Min(
            config.MaxBrakeAccel,
            frontLongitudinal + rearLongitudinal
        );
        float lateralUse = Math.Abs(lateralAcceleration) / Math.Max(frontGrip + rearGrip, Epsilon);
        float loss = CalculateLossAccel(config, speed, lateralUse);

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

        float frontGrip = CalculateAxleGripAccel(config, tires, input.Strategy.TireMode, state.FrontLeft, state.FrontRight);
        float rearGrip = CalculateAxleGripAccel(config, tires, input.Strategy.TireMode, state.RearLeft, state.RearRight);
        float totalGrip = Math.Max(Epsilon, frontGrip + rearGrip);

        float desiredCurvature = Math.Clamp(
            input.DriverInput.DesiredCurvature,
            -config.MaxCurvatureRequest,
            config.MaxCurvatureRequest
        );
        float desiredAccel = Math.Clamp(
            input.DriverInput.DesiredAccel,
            -config.MaxBrakeAccel,
            config.MaxDriveAccelRequest
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
            float driveAccel = Math.Min(desiredAccel, CalculateDriveAccelLimit(state, config, input.Strategy.BatteryMode));
            frontLongRequest = driveAccel * config.FrontDriveShare;
            rearLongRequest = driveAccel - frontLongRequest;
            requestedLongitudinalAccel = driveAccel;
        }
        else
        {
            float brakeRequest = Math.Min(-desiredAccel, config.MaxBrakeAccel);
            AllocateBrakeRequest(
                brakeRequest,
                frontGrip,
                rearGrip,
                out float frontBrake,
                out float rearBrake
            );
            frontLongRequest = -frontBrake;
            rearLongRequest = -rearBrake;
            requestedLongitudinalAccel = -brakeRequest;
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

        AxleResult front = ResolveAxle(config, frontLatRequest, frontLongRequest, frontGrip);
        AxleResult rear = ResolveAxle(config, rearLatRequest, rearLongRequest, rearGrip);

        float actualLateralAccel = front.LateralAccel + rear.LateralAccel;
        float driveAccelActual = Math.Max(0f, front.LongitudinalAccel) + Math.Max(0f, rear.LongitudinalAccel);
        float brakeAccelActual = Math.Max(0f, -front.LongitudinalAccel) + Math.Max(0f, -rear.LongitudinalAccel);
        float axleLongitudinalAccel = front.LongitudinalAccel + rear.LongitudinalAccel;

        float frontLateralUse = SafeUse(front.LateralAccel, frontGrip);
        float rearLateralUse = SafeUse(rear.LateralAccel, rearGrip);
        float frontLongitudinalUse = SafeUse(front.LongitudinalAccel, frontGrip);
        float rearLongitudinalUse = SafeUse(rear.LongitudinalAccel, rearGrip);
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
        float lossAccel = CalculateLossAccel(config, state.Speed, lateralUse) +
                          sideslipLossAccel;
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
            -MaximumYawRateRadiansPerSecond,
            MaximumYawRateRadiansPerSecond
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
            -MaximumBodySideslipRadians,
            MaximumBodySideslipRadians
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
            Math.Abs(state.SideslipAngleRadians) / MaximumBodySideslipRadians,
            0f,
            1f
        );

        float drivePowerWatts = UpdateBattery(
            state,
            config,
            input.Strategy.BatteryMode,
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
            input.Strategy.TireMode,
            frontLateralUse,
            frontLongitudinalUse,
            costedFrontOverLimit,
            0f,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            dt
        );
        UpdateTires(
            state.FrontRight,
            config,
            tires,
            input.Strategy.TireMode,
            frontLateralUse,
            frontLongitudinalUse,
            costedFrontOverLimit,
            0f,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            dt
        );
        UpdateTires(
            state.RearLeft,
            config,
            tires,
            input.Strategy.TireMode,
            rearLateralUse,
            rearLongitudinalUse,
            costedRearOverLimit,
            normalizedSideslip,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            dt
        );
        UpdateTires(
            state.RearRight,
            config,
            tires,
            input.Strategy.TireMode,
            rearLateralUse,
            rearLongitudinalUse,
            costedRearOverLimit,
            normalizedSideslip,
            input.AirTempC,
            input.TrackTempC,
            averageSpeed,
            dt
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

    private static float CalculateDriveAccelLimit(CarState state, CarConfig config, BatteryOutputMode mode)
    {
        return CalculateDriveAccelLimit(state, config, mode, state.Speed);
    }

    private static float CalculateDriveAccelLimit(
        CarState state,
        CarConfig config,
        BatteryOutputMode mode,
        float speed
    )
    {
        if (state.BatterySoc <= 0f)
            return 0f;

        float socFactor = state.BatterySoc >= config.LowSocPowerLimitStart
            ? 1f
            : state.BatterySoc / Math.Max(config.LowSocPowerLimitStart, Epsilon);

        float forceLimitedAccel = config.GetBatteryForceAccelLimit(mode);
        float powerLimitedAccel =
            config.GetBatteryPowerLimitWatts(mode) /
            (config.MassKg * Math.Max(speed, config.MinPowerSpeed));

        return Math.Min(forceLimitedAccel, powerLimitedAccel) * socFactor;
    }

    private static float RemainingLongitudinalGrip(float grip, float lateralAcceleration)
    {
        float remainingSquared = grip * grip - lateralAcceleration * lateralAcceleration;
        return MathF.Sqrt(Math.Max(0f, remainingSquared));
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

    private static float DistributedDriveLimit(
        float frontCapacity,
        float rearCapacity,
        float frontDriveShare
    )
    {
        float frontShare = Math.Clamp(frontDriveShare, 0f, 1f);
        float rearShare = 1f - frontShare;
        float limit = float.PositiveInfinity;

        if (frontShare > Epsilon)
            limit = Math.Min(limit, frontCapacity / frontShare);
        if (rearShare > Epsilon)
            limit = Math.Min(limit, rearCapacity / rearShare);

        return float.IsFinite(limit) ? Math.Max(0f, limit) : 0f;
    }

    private static void AllocateBrakeRequest(
        float brakeRequest,
        float frontGrip,
        float rearGrip,
        out float frontBrake,
        out float rearBrake
    )
    {
        float totalGrip = frontGrip + rearGrip;

        if (totalGrip <= Epsilon)
        {
            frontBrake = 0f;
            rearBrake = 0f;
            return;
        }

        frontBrake = brakeRequest * frontGrip / totalGrip;
        rearBrake = brakeRequest - frontBrake;
    }

    private static AxleResult ResolveAxle(
        CarConfig config,
        float lateralRequest,
        float longitudinalRequest,
        float grip
    )
    {
        if (grip <= Epsilon)
            return default;

        float lateralUse = lateralRequest / grip;
        float longitudinalUse = longitudinalRequest / grip;
        float combinedUse = MathF.Sqrt(lateralUse * lateralUse + longitudinalUse * longitudinalUse);

        if (combinedUse <= 1f)
            return new AxleResult(
                lateralRequest,
                longitudinalRequest,
                Math.Max(0f, combinedUse - 1f),
                combinedUse
            );

        float overLimit = combinedUse - 1f;
        float efficiency = CalculateOverLimitGripEfficiency(config, overLimit);
        float scale = efficiency / combinedUse;
        return new AxleResult(
            lateralRequest * scale,
            longitudinalRequest * scale,
            overLimit,
            combinedUse
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
            -MaximumYawAccelerationRadiansPerSecondSquared,
            MaximumYawAccelerationRadiansPerSecondSquared
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

    private static float CalculateLossAccel(CarConfig config, float speed, float lateralUse)
    {
        if (speed <= 0.01f)
            return 0f;

        return
            config.RollingDragAccel +
            config.AeroDragAccelPerSpeedSquared * speed * speed +
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
        BatteryOutputMode mode,
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

        float netEnergy =
            drivePower * config.GetBatteryDrainMultiplier(mode) * dt -
            regenPower * dt;

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
        TireUsageMode mode,
        float lateralUse,
        float longitudinalUse,
        float overLimit,
        float sideslipRatio,
        float airTempC,
        float trackTempC,
        float speed,
        float dt
    )
    {
        float loadScale = Math.Max(
            MinimumTireHeatLoadScale,
            tire.LoadN / Math.Max(config.MassKg * Gravity * 0.25f, Epsilon)
        );
        float modeHeat = tires.GetModeHeatFactor(mode);
        float modeWear = tires.GetModeWearFactor(mode);
        float thermalOverLimit = Math.Min(overLimit, MaximumThermalOverLimit);
        float normalizedLateralUse = Math.Clamp(Math.Abs(lateralUse), 0f, 1f);
        float normalizedLongitudinalUse = Math.Clamp(
            Math.Abs(longitudinalUse),
            0f,
            1f
        );
        float slipHeatSpeedMultiplier = Math.Clamp(
            Math.Max(0f, speed) /
            Math.Max(tires.SlipHeatReferenceSpeedMps, Epsilon),
            0f,
            Math.Max(1f, tires.MaxSlipHeatSpeedMultiplier)
        );
        float longitudinalHeatUse = MathF.Pow(
            normalizedLongitudinalUse,
            Math.Max(1f, tires.LongitudinalHeatExponent)
        );
        float rollingHeatSpeedFactor = Math.Max(0f, speed) /
                                       (
                                           Math.Max(0f, speed) +
                                           Math.Max(tires.RollingHeatReferenceSpeedMps, Epsilon)
                                       );
        float rollingSurfaceHeat = Math.Max(0f, tires.RollingSurfaceHeatRate) *
                                   loadScale * rollingHeatSpeedFactor;

        float surfaceHeat =
            slipHeatSpeedMultiplier * (
                tires.LateralHeatRate *
                normalizedLateralUse * normalizedLateralUse +
                tires.LongitudinalHeatRate * longitudinalHeatUse
            ) +
            tires.OverLimitHeatRate * thermalOverLimit * thermalOverLimit +
            tires.SideslipHeatRate * sideslipRatio * sideslipRatio;
        surfaceHeat *= modeHeat * loadScale;
        surfaceHeat += rollingSurfaceHeat;

        float airCoolingMultiplier = CalculateAirCoolingMultiplier(tires, speed);
        float surfaceToAir = tires.SurfaceCoolingRate * airCoolingMultiplier * (tire.SurfaceTempC - airTempC);
        float surfaceToTrack = Math.Max(0f, tires.TrackSurfaceTransferRate) *
                               (tire.SurfaceTempC - trackTempC);
        float surfaceToCore = tires.SurfaceCoreTransferRate * (tire.SurfaceTempC - tire.CoreTempC);
        float rollingCoreHeat = Math.Max(0f, tires.RollingCoreHeatRate) *
                                loadScale * rollingHeatSpeedFactor;
        float coreToAir = tires.CoreCoolingRate * airCoolingMultiplier *
                          (tire.CoreTempC - airTempC);
        float coreHeatCapacityRatio = Math.Max(1f, tires.CoreHeatCapacityRatio);

        tire.SurfaceTempC += (
            surfaceHeat - surfaceToAir - surfaceToTrack - surfaceToCore
        ) * dt;
        tire.CoreTempC += (
            rollingCoreHeat + surfaceToCore - coreToAir
        ) / coreHeatCapacityRatio * dt;

        float tempWearFactor = 1f + Math.Max(0f, tire.SurfaceTempC - tires.HotWearStartTempC) * tires.HotWearSlope;
        float wearDelta =
            tires.LateralWearRate * lateralUse * lateralUse +
            tires.LongitudinalWearRate * longitudinalUse * longitudinalUse +
            tires.OverLimitWearRate * thermalOverLimit * thermalOverLimit +
            tires.SideslipWearRate * sideslipRatio * sideslipRatio;
        wearDelta *= modeWear * tempWearFactor * loadScale * dt;

        tire.Wear = Math.Clamp(tire.Wear + wearDelta, 0f, 1f);
    }

    private static float CalculateAirCoolingMultiplier(TireConfig tires, float speed)
    {
        float maxMultiplier = Math.Max(1f, tires.MaxSpeedCoolingMultiplier);
        float referenceSpeed = Math.Max(Epsilon, tires.SpeedCoolingReferenceMps);
        float speedFactor = Math.Max(0f, speed) / (Math.Max(0f, speed) + referenceSpeed);
        return 1f + (maxMultiplier - 1f) * speedFactor;
    }

    private static WheelLoads CalculateWheelLoads(CarState state, CarConfig config)
    {
        return CalculateWheelLoads(
            config,
            state.FilteredLongitudinalAccel,
            state.FilteredLateralAccel
        );
    }

    private static WheelLoads CalculateWheelLoads(
        CarConfig config,
        float longitudinalAcceleration,
        float lateralAcceleration
    )
    {
        float totalLoad = config.MassKg * Gravity;
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
        TireUsageMode mode,
        TireState left,
        TireState right
    )
    {
        float leftForce = left.LoadN * CalculateTireMu(tires, mode, left);
        float rightForce = right.LoadN * CalculateTireMu(tires, mode, right);
        return (leftForce + rightForce) / Math.Max(config.MassKg, Epsilon);
    }

    private static float CalculateTireMu(TireConfig tires, TireUsageMode mode, TireState tire)
    {
        float tempGrip = 1f;
        if (tire.SurfaceTempC < tires.IdealSurfaceTempC)
            tempGrip -= (tires.IdealSurfaceTempC - tire.SurfaceTempC) * tires.ColdGripLossPerC;
        else
            tempGrip -= (tire.SurfaceTempC - tires.IdealSurfaceTempC) * tires.HotGripLossPerC;

        tempGrip -= Math.Max(0f, tire.CoreTempC - tires.CoreOverheatTempC) * tires.CoreOverheatGripLossPerC;

        float wearGrip = 1f - tire.Wear * tires.WearGripLoss;
        float modeGrip = tires.GetModeGripFactor(mode);
        return tires.BaseMu * modeGrip *
               Math.Clamp(tempGrip, MinimumTemperatureGripFactor, MaximumTemperatureGripFactor) *
               Math.Clamp(wearGrip, MinimumWearGripFactor, 1f);
    }

    private static float SafeUse(float accel, float grip)
    {
        return grip <= Epsilon ? 0f : Math.Abs(accel) / grip;
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
        float CombinedRequest
    );

    private readonly record struct WheelLoads(
        float FrontLeft,
        float FrontRight,
        float RearLeft,
        float RearRight
    );
}
