using System.Numerics;
using StintegyEVO.Core.Track.RefLines;

namespace StintegyEVO.Core.Track;

/// <summary>
/// The shape of the road out of plane at one point: how steeply it climbs
/// along the way, and the section across it as
/// <c>z(d) = z0 + Slope*d + Curvature*d^2</c>. The quadratic term is what
/// separates this from a single bank angle — it is what a crown, a bowl,
/// and progressive banking all need, and all three exist on real circuits.
/// </summary>
public readonly record struct TrackSurface(
    float Grade = 0f,
    float BankSlope = 0f,
    float BankCurvature = 0f
)
{
    public static readonly TrackSurface Flat = new();

    public bool IsFlat =>
        Grade == 0f && BankSlope == 0f && BankCurvature == 0f;

    public static TrackSurface Lerp(TrackSurface a, TrackSurface b, float t) =>
        new(
            a.Grade + (b.Grade - a.Grade) * t,
            a.BankSlope + (b.BankSlope - a.BankSlope) * t,
            a.BankCurvature + (b.BankCurvature - a.BankCurvature) * t
        );
}

internal readonly record struct TrackNode(
    Vector2 Center, Vector2 Tangent,
    float Width, float LeftBufferWidth, float RightBufferWidth,
    RefLinePoint RefLinePoint,
    TrackSurface Surface = default
)
{
    public readonly float HalfWidth => Width / 2.0f;
    public readonly Vector2 Normal => new(Tangent.Y, -Tangent.X);
    public readonly float RefOffset => RefLinePoint.Offset;
    public readonly Vector2 Ref => Center + Normal * RefOffset;
    public readonly Vector2 LeftEdge => Center + Normal * HalfWidth;
    public readonly Vector2 RightEdge => Center - Normal * HalfWidth;
    public readonly Vector2 LeftSpace => LeftEdge + Normal * LeftBufferWidth;
    public readonly Vector2 RightSpace => RightEdge - Normal * RightBufferWidth;


    // left +, right -
    public Vector2 GetOffsetPos(float offset)
    {
        return Center + Normal * offset;
    }
}
