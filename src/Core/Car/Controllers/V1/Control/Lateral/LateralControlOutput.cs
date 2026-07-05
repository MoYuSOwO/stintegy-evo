using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;

public readonly record struct LateralControlOutput(
    float SteeringInput,
    float SteeringAngle,
    float FeedForwardSteeringAngle,
    float FeedbackSteeringAngle,
    float StabilitySteeringAngle,
    float YawDampingSteeringAngle,
    float BetaDampingSteeringAngle,
    float LookaheadDistance,
    Vector2 LookaheadPoint,
    float TargetSpeed,
    float TargetLateralAcceleration,
    int NearestProfileIndex,
    float ProfileDistance,
    float LateralError,
    float HeadingError,
    float Beta,
    float BetaReference,
    float YawRate,
    float YawRateReference,
    float YawRateError,
    float StabilityRisk,
    float TrackOffset = 0.0f,
    float TrackUsableHalfWidth = 0.0f,
    float TrackBoundaryExcess = 0.0f,
    float TrackBufferBoundaryExcess = 0.0f
);
