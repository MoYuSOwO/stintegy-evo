using System;
using System.Numerics;

namespace StintegyEVO.Core.Track;

/// <summary>
/// What the track builder knows about one metre of the physical road:
/// centreline pose, road width, run-off, and the surface model.  This is
/// deliberately road geometry, not a planner-generated target line.
/// </summary>
public readonly record struct TrackSurfaceContext(
    float DistanceMeters,
    float CentrelineCurvature,
    float HalfWidthMeters,
    float LapLengthMeters
);

/// <summary>
/// The shape of the road out of plane at one point: how steeply it climbs
/// along the way, and the section across it as
/// <c>z(d) = z0 + Slope*d + Curvature*d^2</c>.
/// </summary>
public readonly record struct TrackSurface(
    float Grade = 0f,
    float BankSlope = 0f,
    float BankCurvature = 0f,
    float VerticalRate = 0f
)
{
    public static readonly TrackSurface Flat = new();

    public bool IsFlat =>
        Grade == 0f && BankSlope == 0f &&
        BankCurvature == 0f && VerticalRate == 0f;

    public static TrackSurface Lerp(TrackSurface a, TrackSurface b, float t) =>
        new(
            a.Grade + (b.Grade - a.Grade) * t,
            a.BankSlope + (b.BankSlope - a.BankSlope) * t,
            a.BankCurvature + (b.BankCurvature - a.BankCurvature) * t,
            a.VerticalRate +
            (b.VerticalRate - a.VerticalRate) * t
        );
}

/// <summary>
/// A sampled piece of the authored/surveyed road centreline.  Curvature is
/// measured from this centreline itself; it is never supplied by a racing
/// line optimizer.
/// </summary>
internal readonly record struct TrackNode(
    Vector2 Center, Vector2 Tangent, float Curvature,
    float Width, float LeftBufferWidth, float RightBufferWidth,
    TrackSurface Surface = default
)
{
    public readonly float HalfWidth => Width / 2.0f;
    public readonly Vector2 Normal => new(Tangent.Y, -Tangent.X);
    public readonly Vector2 LeftEdge => Center + Normal * HalfWidth;
    public readonly Vector2 RightEdge => Center - Normal * HalfWidth;
    public readonly Vector2 LeftSpace => LeftEdge + Normal * LeftBufferWidth;
    public readonly Vector2 RightSpace => RightEdge - Normal * RightBufferWidth;

    // left +, right -
    public Vector2 GetOffsetPos(float offset) => Center + Normal * offset;
}
