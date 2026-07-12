using System.Numerics;
using StintegyEVO.Core.Track.RefLines;

namespace StintegyEVO.Core.Track;

internal readonly record struct TrackNode(
    Vector2 Center, Vector2 Tangent,
    float Width, float LeftBufferWidth, float RightBufferWidth,
    RefLinePoint RefLinePoint
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
