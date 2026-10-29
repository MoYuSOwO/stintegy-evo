using System.Numerics;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Racing;

public readonly record struct TrackBoundaryContact(
    TrackSide Side,
    float PenetrationMeters,
    float LimitD,
    Vector2 Normal,
    float ImpactFraction,
    Vector2 CorrectedPosition,
    TrackPose PoseBeforeCorrection
)
{
    /// <summary>Where the car met the wall: the corner that reached it, in world coordinates.</summary>
    public Vector2 Point { get; init; }

    /// <summary>What the wall gave the car to stop it going through, N·s, in world coordinates; zero for a car already moving clear.</summary>
    public Vector2 Impulse { get; init; }
}

public enum TrackRegion
{
    RacingSurface,
    Buffer,
    BeyondWall
}

public enum TrackSide
{
    Left,
    Right
}
