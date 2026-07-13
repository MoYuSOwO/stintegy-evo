namespace StintegyEVO.Core.Cars;

public readonly record struct CarTelemetry(
    DriverInput Input,
    CarStrategy Strategy,
    float RequestedLateralAccel,
    float ActualLateralAccel,
    float RequestedLongitudinalAccel,
    float ActualLongitudinalAccel,
    float LossAccel,
    float ActualCurvature,
    float FrontGripAccel,
    float RearGripAccel,
    float FrontLateralUse,
    float RearLateralUse,
    float FrontLongitudinalUse,
    float RearLongitudinalUse,
    float OverLimit,
    float DrivePowerWatts,
    float RegenPowerWatts,
    float TractionControlCutAccel,
    float SideslipLossAccel,
    float SideslipAngleRadians,
    float RearSlideSeverity,
    float ReferenceYawRateRadiansPerSecond,
    float YawRateRadiansPerSecond,
    float YawAccelerationRadiansPerSecondSquared
);
