using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.RacingLines;

public readonly record struct TrackReferencePoint(
    int TrackIndex,
    float Offset,
    Vector2 Position,
    float Heading,
    float Curvature
);
