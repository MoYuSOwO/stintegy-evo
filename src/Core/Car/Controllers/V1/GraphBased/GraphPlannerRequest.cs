using Godot;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.GraphBased;

public readonly record struct GraphPlannerRequest(
    TrackData Track,
    Vector2 Position,
    float Heading,
    ITrackReferenceLine ReferenceLine,
    GraphPlannerConfig? Config = null,
    GraphPath? PreviousPath = null,
    GraphPlannerCache? Cache = null
);
