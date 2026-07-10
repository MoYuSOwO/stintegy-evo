using System.Numerics;
using TheStint.Core.Track;

namespace TheStint.Core.Racing;

public readonly record struct TrackBoundaryContact(
    TrackSide Side,
    float PenetrationMeters,
    float LimitD,
    Vector2 Normal,
    float ImpactFraction,
    Vector2 CorrectedPosition,
    TrackPose PoseBeforeCorrection
);

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
