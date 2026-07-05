using System;
using Godot;
using StintegyEVO.Core.Car.Configs;

namespace StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;

public readonly record struct EmpiricalHandlingDiagramModel(
    float KAy3V,
    float KAy3,
    float KAyV,
    float KAy,
    int FitSampleCount
)
{
    public static readonly EmpiricalHandlingDiagramModel Zero = new(0.0f, 0.0f, 0.0f, 0.0f, 0);

    public float PredictSteeringAngle(
        CarConfig carConfig,
        float lateralAccelerationMetersPerSecondSquared,
        float longitudinalSpeedMetersPerSecond
    )
    {
        float speed = MathF.Max(MathF.Abs(longitudinalSpeedMetersPerSecond), 1.0f);
        float ackermann = CalculateKinematicSteeringAngle(
            carConfig.Chassis.WheelBase,
            lateralAccelerationMetersPerSecondSquared,
            speed
        );
        float deviation = PredictSteeringDeviation(lateralAccelerationMetersPerSecondSquared, speed);
        return Mathf.Clamp(
            ackermann + deviation,
            -carConfig.Chassis.MaxSteerAngle,
            carConfig.Chassis.MaxSteerAngle
        );
    }

    public float PredictSteeringDeviation(float lateralAccelerationMetersPerSecondSquared, float longitudinalSpeedMetersPerSecond)
    {
        float ay = lateralAccelerationMetersPerSecondSquared;
        float ay2 = ay * ay;
        float secantSlope =
            KAy3V * ay2 * longitudinalSpeedMetersPerSecond +
            KAy3 * ay2 +
            KAyV * longitudinalSpeedMetersPerSecond +
            KAy;
        return secantSlope * ay;
    }

    public static float CalculateKinematicSteeringAngle(
        float wheelbase,
        float lateralAccelerationMetersPerSecondSquared,
        float longitudinalSpeedMetersPerSecond
    )
    {
        float speed = MathF.Max(MathF.Abs(longitudinalSpeedMetersPerSecond), 1.0f);
        return MathF.Atan(wheelbase * lateralAccelerationMetersPerSecondSquared / (speed * speed));
    }
}
