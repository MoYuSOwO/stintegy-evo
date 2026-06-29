namespace StintegyEVO.Core.Car.Controllers.V1.GraphBased;

public readonly record struct GraphOnlineTrajectory(
    GraphPath Path,
    bool ReusedPreviousPath,
    bool ResetPreviousPath,
    float PreviousPathDistanceMeters,
    int PreviousPathNearestSample
);
