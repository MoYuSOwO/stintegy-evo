using System.Numerics;

namespace StintegyEVO.Core.Track.RefLines;

public readonly record struct RefLineTrackPoint(
    Vector2 Center,
    Vector2 Tangent,
    float Width
)
{
    public float HalfWidth => Width * 0.5f;
    public Vector2 Normal => new(Tangent.Y, -Tangent.X);

    public Vector2 GetOffsetPos(float offset)
    {
        return Center + Normal * offset;
    }
}
