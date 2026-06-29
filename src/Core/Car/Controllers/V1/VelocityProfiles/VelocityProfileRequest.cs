using StintegyEVO.Core.Car.Controllers.V1.GraphBased;

namespace StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;

public readonly record struct VelocityProfileRequest(
    GraphPath Path,
    float StartSpeed,
    float MaxSpeed,
    float MaxLongitudinalAccel,
    float MaxLongitudinalDecel,
    float MaxLateralAccel,
    float EndSpeed = 0f,
    bool EnforceEndSpeed = false,
    float SafetyFactor = 0.98f,
    IAccelerationEnvelope? AccelerationEnvelope = null
);
