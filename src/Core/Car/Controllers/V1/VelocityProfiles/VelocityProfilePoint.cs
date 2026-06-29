namespace StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;

public readonly record struct VelocityProfilePoint(
    float Distance,
    float TargetSpeed,
    float TargetAcceleration
);
