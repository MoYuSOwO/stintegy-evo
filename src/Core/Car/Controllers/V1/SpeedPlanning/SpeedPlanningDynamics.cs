using System;
using Godot;
using StintegyEVO.Core.Car.Components;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;

internal static class SpeedPlanningDynamics
{
    private const float ForceEpsilon = 1e-4f;

    public static float CalculateLateralSpeedLimit(CarConfig car, float curvature, SpeedPlanningConfig config)
    {
        return CalculateLateralSpeedLimit(car, SpeedPlanningState.FromConfig(car), curvature, config);
    }

    public static float CalculateLateralSpeedLimit(
        CarConfig car,
        SpeedPlanningState state,
        float curvature,
        SpeedPlanningConfig config
    )
    {
        float signedCurvature = curvature;
        float absCurvature = MathF.Abs(curvature);
        float maximumSpeed = MathF.Max(0.0f, config.MaximumSpeedMetersPerSecond);
        if (absCurvature <= MathF.Max(config.CurvatureEpsilon, 0.0f))
            return maximumSpeed;

        if (IsLateralFeasible(car, state, maximumSpeed, signedCurvature, config))
            return maximumSpeed;

        float low = 0.0f;
        float high = maximumSpeed;
        int iterations = Math.Max(1, config.LateralSpeedSearchIterations);
        for (int i = 0; i < iterations; i++)
        {
            float mid = (low + high) * 0.5f;
            if (IsLateralFeasible(car, state, mid, signedCurvature, config))
                low = mid;
            else
                high = mid;
        }

        return low;
    }

    public static float SolveMaxAcceleration(CarConfig car, float speed, float curvature, SpeedPlanningConfig config)
    {
        return SolveMaxAcceleration(car, SpeedPlanningState.FromConfig(car), speed, curvature, config);
    }

    public static float SolveMaxAcceleration(
        CarConfig car,
        SpeedPlanningState state,
        float speed,
        float curvature,
        SpeedPlanningConfig config
    )
    {
        float acceleration = 0.0f;
        int iterations = Math.Max(1, config.LoadTransferIterations);

        for (int i = 0; i < iterations; i++)
        {
            LongitudinalEnvelope envelope = CalculateEnvelope(car, state, speed, curvature, acceleration, config);
            float tireLimitedDriveForce = CalculateDistributionLimitedForce(envelope, car.Distributor.FrontBias);
            float systemLimitedDriveForce = config.IncludePowerLimit
                ? car.Power.CalcMaxDriveForceAtSpeed(MathF.Max(speed, 0.0f))
                : float.PositiveInfinity;
            float driveForce = MathF.Min(tireLimitedDriveForce, systemLimitedDriveForce);
            float nextAcceleration = MathF.Max(0.0f, (driveForce - envelope.DragForce) / envelope.Mass);

            if (MathF.Abs(nextAcceleration - acceleration) < 1e-3f)
                return nextAcceleration;

            acceleration = nextAcceleration;
        }

        return acceleration;
    }

    public static float SolveMaxDeceleration(CarConfig car, float speed, float curvature, SpeedPlanningConfig config)
    {
        return SolveMaxDeceleration(car, SpeedPlanningState.FromConfig(car), speed, curvature, config);
    }

    public static float SolveMaxDeceleration(
        CarConfig car,
        SpeedPlanningState state,
        float speed,
        float curvature,
        SpeedPlanningConfig config
    )
    {
        float deceleration = 0.0f;
        int iterations = Math.Max(1, config.LoadTransferIterations);

        for (int i = 0; i < iterations; i++)
        {
            LongitudinalEnvelope envelope = CalculateEnvelope(car, state, speed, curvature, -deceleration, config);
            float tireLimitedBrakeForce = CalculateDistributionLimitedForce(envelope, car.Distributor.FrontBias);
            float brakeForce = MathF.Min(tireLimitedBrakeForce, MathF.Max(0.0f, car.Power.MaxBrakeForce));
            float nextDeceleration = MathF.Max(0.0f, (brakeForce + envelope.DragForce) / envelope.Mass);

            if (MathF.Abs(nextDeceleration - deceleration) < 1e-3f)
                return nextDeceleration;

            deceleration = nextDeceleration;
        }

        return deceleration;
    }

    private static bool IsLateralFeasible(
        CarConfig car,
        SpeedPlanningState state,
        float speed,
        float curvature,
        SpeedPlanningConfig config
    )
    {
        LongitudinalEnvelope envelope = CalculateEnvelope(
            car,
            state,
            speed,
            curvature,
            longitudinalAcceleration: 0.0f,
            config
        );
        return envelope.LateralDemand <= envelope.LateralCapacity + 1e-3f;
    }

    private static LongitudinalEnvelope CalculateEnvelope(
        CarConfig car,
        SpeedPlanningState state,
        float speed,
        float curvature,
        float longitudinalAcceleration,
        SpeedPlanningConfig config
    )
    {
        float mass = state.Mass > 0.0f ? state.Mass : car.Chassis.DryMass;
        float safeSpeed = MathF.Max(0.0f, speed);
        float lateralAcceleration = safeSpeed * safeSpeed * curvature;
        AeroOutput aero = CalculateAero(car.Aero, safeSpeed, state.DirtyAirFactor, config);
        CarLoad load = CalculateSteadyStateLoad(
            mass,
            car.Chassis.WeightDistFront,
            car.Chassis.CgHeight,
            car.Chassis.WheelBase,
            car.Chassis.Width,
            car.Suspension.FrontRollBalance,
            longitudinalAcceleration,
            lateralAcceleration,
            aero.DownforceFront,
            aero.DownforceRear
        );

        float frictionScale = MathF.Max(
            0.0f,
            config.FrictionUsage * config.TrackFrictionMultiplier * state.TrackFrictionMultiplier
        );
        float latPeakFrontLeft = MathF.Max(0.0f, load.FrontLeft * state.TireFriction.GetLatPeak(TireType.FrontLeft) * frictionScale);
        float latPeakFrontRight = MathF.Max(0.0f, load.FrontRight * state.TireFriction.GetLatPeak(TireType.FrontRight) * frictionScale);
        float latPeakRearLeft = MathF.Max(0.0f, load.RearLeft * state.TireFriction.GetLatPeak(TireType.RearLeft) * frictionScale);
        float latPeakRearRight = MathF.Max(0.0f, load.RearRight * state.TireFriction.GetLatPeak(TireType.RearRight) * frictionScale);
        float longPeakFrontLeft = MathF.Max(0.0f, load.FrontLeft * state.TireFriction.GetLongPeak(TireType.FrontLeft) * frictionScale);
        float longPeakFrontRight = MathF.Max(0.0f, load.FrontRight * state.TireFriction.GetLongPeak(TireType.FrontRight) * frictionScale);
        float longPeakRearLeft = MathF.Max(0.0f, load.RearLeft * state.TireFriction.GetLongPeak(TireType.RearLeft) * frictionScale);
        float longPeakRearRight = MathF.Max(0.0f, load.RearRight * state.TireFriction.GetLongPeak(TireType.RearRight) * frictionScale);

        float lateralCapacity = latPeakFrontLeft + latPeakFrontRight + latPeakRearLeft + latPeakRearRight;
        float lateralDemand = mass * MathF.Abs(lateralAcceleration);
        float residualFrontLeft = 0.0f;
        float residualFrontRight = 0.0f;
        float residualRearLeft = 0.0f;
        float residualRearRight = 0.0f;

        if (lateralCapacity > ForceEpsilon && lateralDemand < lateralCapacity)
        {
            residualFrontLeft = CalculateResidualLongitudinalForce(latPeakFrontLeft, longPeakFrontLeft, lateralDemand, lateralCapacity);
            residualFrontRight = CalculateResidualLongitudinalForce(latPeakFrontRight, longPeakFrontRight, lateralDemand, lateralCapacity);
            residualRearLeft = CalculateResidualLongitudinalForce(latPeakRearLeft, longPeakRearLeft, lateralDemand, lateralCapacity);
            residualRearRight = CalculateResidualLongitudinalForce(latPeakRearRight, longPeakRearRight, lateralDemand, lateralCapacity);
        }

        return new LongitudinalEnvelope(
            residualFrontLeft,
            residualFrontRight,
            residualRearLeft,
            residualRearRight,
            aero.DragForce,
            mass,
            lateralDemand,
            lateralCapacity
        );
    }

    private static AeroOutput CalculateAero(
        AeroConfig config,
        float speed,
        float dirtyAirFactor,
        SpeedPlanningConfig planningConfig
    )
    {
        AeroOutput aero = AeroComponent.CalculateAero(config, speed, dirtyAirFactor);

        return new AeroOutput
        {
            DragForce = planningConfig.IncludeAeroDrag ? aero.DragForce : 0.0f,
            DownforceFront = planningConfig.IncludeAeroDownforce ? aero.DownforceFront : 0.0f,
            DownforceRear = planningConfig.IncludeAeroDownforce ? aero.DownforceRear : 0.0f
        };
    }

    private static CarLoad CalculateSteadyStateLoad(
        float mass,
        float staticWeightDistFront,
        float cgHeight,
        float wheelBase,
        float width,
        float frontRollBalance,
        float accelLong,
        float accelLat,
        float downforceFront,
        float downforceRear
    )
    {
        float totalWeight = mass * GeomUtil.g;
        float staticFront = totalWeight * staticWeightDistFront;
        float staticRear = totalWeight * (1.0f - staticWeightDistFront);

        float deltaWeightLong = mass * accelLong * cgHeight / wheelBase;
        float deltaWeightLat = mass * accelLat * cgHeight / width;

        float clampedFrontRollBalance = Math.Clamp(frontRollBalance, 0.0f, 1.0f);
        float deltaLatFront = deltaWeightLat * clampedFrontRollBalance;
        float deltaLatRear = deltaWeightLat * (1.0f - clampedFrontRollBalance);

        return new CarLoad
        {
            FrontLeft = MathF.Max(0.1f, staticFront / 2.0f - deltaWeightLong / 2.0f + deltaLatFront + downforceFront / 2.0f),
            FrontRight = MathF.Max(0.1f, staticFront / 2.0f - deltaWeightLong / 2.0f - deltaLatFront + downforceFront / 2.0f),
            RearLeft = MathF.Max(0.1f, staticRear / 2.0f + deltaWeightLong / 2.0f + deltaLatRear + downforceRear / 2.0f),
            RearRight = MathF.Max(0.1f, staticRear / 2.0f + deltaWeightLong / 2.0f - deltaLatRear + downforceRear / 2.0f)
        };
    }

    private static float CalculateResidualLongitudinalForce(
        float lateralPeak,
        float longitudinalPeak,
        float totalLateralDemand,
        float totalLateralCapacity
    )
    {
        if (lateralPeak <= ForceEpsilon || totalLateralCapacity <= ForceEpsilon)
            return 0.0f;

        float lateralForce = totalLateralDemand * lateralPeak / totalLateralCapacity;
        float lateralRatio = lateralForce / lateralPeak;
        return longitudinalPeak * MathF.Sqrt(MathF.Max(0.0f, 1.0f - lateralRatio * lateralRatio));
    }

    private static float CalculateDistributionLimitedForce(LongitudinalEnvelope envelope, float frontBias)
    {
        float clampedFrontBias = Math.Clamp(frontBias, 0.0f, 1.0f);
        float frontRatio = clampedFrontBias * 0.5f;
        float rearRatio = (1.0f - clampedFrontBias) * 0.5f;

        float limit = float.PositiveInfinity;
        bool hasDrivenWheel = false;
        ApplyTireDistributionLimit(envelope.ResidualFrontLeft, frontRatio, ref limit, ref hasDrivenWheel);
        ApplyTireDistributionLimit(envelope.ResidualFrontRight, frontRatio, ref limit, ref hasDrivenWheel);
        ApplyTireDistributionLimit(envelope.ResidualRearLeft, rearRatio, ref limit, ref hasDrivenWheel);
        ApplyTireDistributionLimit(envelope.ResidualRearRight, rearRatio, ref limit, ref hasDrivenWheel);

        return hasDrivenWheel ? MathF.Max(0.0f, limit) : 0.0f;
    }

    private static void ApplyTireDistributionLimit(
        float residualLongitudinalForce,
        float ratio,
        ref float limit,
        ref bool hasDrivenWheel
    )
    {
        if (ratio <= ForceEpsilon)
            return;

        hasDrivenWheel = true;
        limit = MathF.Min(limit, residualLongitudinalForce / ratio);
    }

    private readonly record struct LongitudinalEnvelope(
        float ResidualFrontLeft,
        float ResidualFrontRight,
        float ResidualRearLeft,
        float ResidualRearRight,
        float DragForce,
        float Mass,
        float LateralDemand,
        float LateralCapacity
    );
}
