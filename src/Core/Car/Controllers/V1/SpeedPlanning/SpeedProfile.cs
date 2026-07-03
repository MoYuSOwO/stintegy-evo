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
    float LateralAcceleration
);

public sealed class SpeedProfile
{
    public readonly SpeedProfilePoint[] Points;

    public SpeedProfile(SpeedProfilePoint[] points)
    {
        Points = points;
    }

    public int Count => Points.Length;
    public float TotalTime => Points.Length == 0 ? 0.0f : Points[^1].TimeFromStart;
    public float PhysicalLength => Points.Length == 0 ? 0.0f : Points[^1].Distance;

    public SpeedProfilePoint this[int index] => Points[index];
}
