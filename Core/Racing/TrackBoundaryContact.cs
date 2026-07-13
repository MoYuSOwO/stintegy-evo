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
