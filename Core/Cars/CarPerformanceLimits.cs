namespace StintegyEVO.Core.Cars;

/// <summary>
/// What a car can do at one instant, estimated from its published
/// parameters and its current physical state: how much lateral
/// acceleration the tyres will hold on the given corner, how hard it can
/// drive and brake on top of that, and what the air, road and tyres take
/// back. An envelope, not a command and not a plan.
/// </summary>
public readonly record struct CarPerformanceLimits(
    float LateralAccelerationLimit,
    float MaximumDriveAcceleration,
    float MaximumBrakeDeceleration,
    float LossAcceleration
);
