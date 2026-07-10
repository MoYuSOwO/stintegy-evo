namespace TheStint.Core.Cars;

internal readonly record struct CarPerformanceLimits(
    float LateralAccelerationLimit,
    float MaximumDriveAcceleration,
    float MaximumBrakeDeceleration,
    float LossAcceleration
);
