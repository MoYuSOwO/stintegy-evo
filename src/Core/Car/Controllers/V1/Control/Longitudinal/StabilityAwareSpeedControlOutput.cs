namespace StintegyEVO.Core.Car.Controllers.V1.Control.Longitudinal;

public readonly record struct StabilityAwareSpeedControlOutput(
    float Input,
    float TargetSpeed,
    float TargetAcceleration,
    float RequestedAcceleration,
    float LimitedAcceleration,
    float MaximumAcceleration,
    float MaximumDeceleration,
    float LateralDemandAcceleration,
    float LateralSpeedLimit,
    float TrackingRisk,
    float StabilityRisk
);
