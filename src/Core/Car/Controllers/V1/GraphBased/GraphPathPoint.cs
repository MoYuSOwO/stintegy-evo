using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.GraphBased;

public readonly record struct GraphPathPoint(
    int TrackIndex,
    float Distance,
    float Offset,
    Vector2 Position,
    float Heading,
    float Curvature,
    float BoundaryClearance
);
