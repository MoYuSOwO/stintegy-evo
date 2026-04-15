using Godot;

namespace PloyRacing.Core.Track;

public readonly struct TrackNode(
    Vector2 center, Vector2 tangent, float width, float leftBuffer, float rightBuffer
)
{
    public readonly Vector2 Center = center;
    public readonly Vector2 Tangent = tangent;
    public readonly float Width = width;
    public readonly float LeftBuffer = leftBuffer;
    public readonly float RightBuffer = rightBuffer;

    public readonly float HalfWidth => Width / 2.0f;
    public readonly Vector2 Normal => new(Tangent.Y, -Tangent.X);
    public readonly Vector2 LeftEdge => Center + Normal * HalfWidth;
    public readonly Vector2 RightEdge => Center - Normal * HalfWidth;


    // 左加右减
    public Vector2 GetOffsetPos(float offset)
    {
        return Center + Normal * offset;
    }
}