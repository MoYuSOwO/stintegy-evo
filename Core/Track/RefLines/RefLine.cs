using System.Numerics;

namespace StintegyEVO.Core.Track.RefLines;

public readonly record struct RefLinePoint(
    float Offset,
    Vector2 Position,
    float Heading,
    float Curvature
);

public sealed class RefLine
{
    public readonly RefLinePoint[] Points;
    public int Count => Points.Length;

    public RefLine(RefLinePoint[] points)
    {
        Points = points;
    }

    public RefLinePoint GetPoint(int trackIndex) => this[trackIndex];

    public RefLinePoint this[int index]
    {
        get
        {
            int safeIndex = (index % Points.Length + Points.Length) % Points.Length;
            return Points[safeIndex];
        }
    }
}
