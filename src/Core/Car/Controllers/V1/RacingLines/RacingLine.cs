using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.RacingLines;

public readonly record struct RacingLinePoint(
    int TrackIndex,
    float Distance,
    float Offset,
    Vector2 Position,
    float Heading,
    float Curvature
);

public sealed class RacingLine
{
    public readonly RacingLinePoint[] Points;
    public int Count => Points.Length;

    public RacingLine(RacingLinePoint[] points)
    {
        Points = points;
    }

    public RacingLinePoint GetPoint(int trackIndex) => this[trackIndex];

    public RacingLinePoint this[int index]
    {
        get
        {
            int safeIndex = (index % Points.Length + Points.Length) % Points.Length;
            return Points[safeIndex];
        }
    }
}
