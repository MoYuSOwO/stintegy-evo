using System;

namespace TheStint.Core.Racing;

public readonly record struct VehicleSpeedProfilePoint(
    float TargetSpeed,
    float ReferenceAcceleration
);

public sealed class VehicleSpeedProfile
{
    private readonly VehicleSpeedProfilePoint[] _points;

    internal VehicleSpeedProfile(
        VehicleSpeedProfilePoint[] points,
        float stepLengthMeters,
        float lengthMeters
    )
    {
        _points = points;
        StepLengthMeters = stepLengthMeters;
        LengthMeters = lengthMeters;
    }

    public int Count => _points.Length;
    public float StepLengthMeters { get; }
    public float LengthMeters { get; }
    public VehicleSpeedProfilePoint this[int index] => _points[WrapIndex(index)];

    public VehicleSpeedProfilePoint Sample(float s)
    {
        if (_points.Length == 0 || LengthMeters <= 0f)
            return default;

        float wrapped = s % LengthMeters;
        if (wrapped < 0f)
            wrapped += LengthMeters;

        float scaled = wrapped / Math.Max(StepLengthMeters, 1e-5f);
        int index = (int)MathF.Floor(scaled);
        float t = scaled - index;
        VehicleSpeedProfilePoint a = this[index];
        VehicleSpeedProfilePoint b = this[index + 1];
        return new VehicleSpeedProfilePoint(
            Lerp(a.TargetSpeed, b.TargetSpeed, t),
            Lerp(a.ReferenceAcceleration, b.ReferenceAcceleration, t)
        );
    }

    private int WrapIndex(int index)
    {
        if (_points.Length == 0)
            return 0;
        return (index % _points.Length + _points.Length) % _points.Length;
    }

    private static float Lerp(float from, float to, float t)
    {
        return from + (to - from) * Math.Clamp(t, 0f, 1f);
    }
}
