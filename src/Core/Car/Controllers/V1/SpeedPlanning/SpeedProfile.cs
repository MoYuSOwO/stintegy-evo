using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;

public readonly record struct SpeedProfilePoint(
    int SampleIndex,
    Vector2 Position,
    float Heading,
    float Curvature,
    float Distance,
    float Speed,
    float AccelerationToNext,
    float TimeFromStart,
    float MaxSpeed,
    float MaxAcceleration,
    float MaxDeceleration,
    float LateralAcceleration,
    float TrackProgress = 0.0f
);

public sealed class SpeedProfile
{
    public readonly SpeedProfilePoint[] Points;
    public readonly int AnchorTrackIndex;
    public readonly int TrackLength;

    public SpeedProfile(SpeedProfilePoint[] points, int anchorTrackIndex = -1, int trackLength = 0)
    {
        Points = points;
        AnchorTrackIndex = anchorTrackIndex;
        TrackLength = trackLength;
    }

    public int Count => Points.Length;
    public float TotalTime => Points.Length == 0 ? 0.0f : Points[^1].TimeFromStart;
    public float PhysicalLength => Points.Length == 0 ? 0.0f : Points[^1].Distance;
    public bool HasTrackProgress => AnchorTrackIndex >= 0 && TrackLength > 0 && Points.Length > 0;

    public SpeedProfilePoint this[int index] => Points[index];
}
