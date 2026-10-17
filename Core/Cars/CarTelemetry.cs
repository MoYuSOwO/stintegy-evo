namespace StintegyEVO.Core.Cars;

/// <summary>
/// What every car reports each step, whatever it is built like: what it was
/// asked for and what it did. A car's own instruments go beside this
/// (<see cref="ICar.TelemetryChannels"/>), and its own reading of itself
/// in its own state (<see cref="CarState.Own"/>).
/// </summary>
/// <param name="OverLimit">How far past what its tyres can give the car is
/// being driven, zero when it is within them.</param>
public readonly record struct CarTelemetry(
    DriverInput Input,
    CarStrategy Strategy,
    float RequestedLateralAccel,
    float ActualLateralAccel,
    float RequestedLongitudinalAccel,
    float ActualLongitudinalAccel,
    float LossAccel,
    float ActualCurvature,
    float OverLimit,
    float DrivePowerWatts,
    float RegenPowerWatts,
    float SideslipAngleRadians,
    float ReferenceYawRateRadiansPerSecond,
    float YawRateRadiansPerSecond,
    float YawAccelerationRadiansPerSecondSquared
);
