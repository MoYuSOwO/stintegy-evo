using System;

namespace StintegyEVO.Core.Drivers;

public readonly record struct VehicleSpeedPlanPoint(
    float TargetSpeed,
    float ReferenceAcceleration
);

/// <summary>
/// Reusable open-chain speed plan indexed by distance ahead of the vehicle.
/// Unlike the former full-lap profile, sampling clamps at the local horizon
/// instead of wrapping around the track.
/// </summary>
public sealed class VehicleSpeedLookahead
{
    private VehicleSpeedPlanPoint[] _points = [];

    public int Count { get; private set; }
    public float StepLengthMeters { get; private set; }
    public float LengthMeters { get; private set; }

    public VehicleSpeedPlanPoint this[int index]
    {
        get
        {
            if ((uint)index >= (uint)Count)
                throw new ArgumentOutOfRangeException(nameof(index));
            return _points[index];
        }
    }

    public VehicleSpeedPlanPoint Sample(float distanceMeters)
    {
        if (Count == 0)
            return default;
        if (Count == 1 || StepLengthMeters <= 0f)
            return _points[0];

        float distance = Math.Clamp(distanceMeters, 0f, LengthMeters);
        float scaled = distance / StepLengthMeters;
        int index = Math.Min((int)MathF.Floor(scaled), Count - 1);
        if (index >= Count - 1)
            return _points[Count - 1];

        float t = scaled - index;
        VehicleSpeedPlanPoint a = _points[index];
        VehicleSpeedPlanPoint b = _points[index + 1];
        return new VehicleSpeedPlanPoint(
            Lerp(a.TargetSpeed, b.TargetSpeed, t),
            Lerp(a.ReferenceAcceleration, b.ReferenceAcceleration, t)
        );
    }

    internal void Reset(int requiredCapacity, float stepLengthMeters, float lengthMeters)
    {
        if (_points.Length < requiredCapacity)
            _points = new VehicleSpeedPlanPoint[requiredCapacity];

        Count = requiredCapacity;
        StepLengthMeters = stepLengthMeters;
        LengthMeters = lengthMeters;
    }

    internal void Set(int index, in VehicleSpeedPlanPoint point)
    {
        _points[index] = point;
    }

    private static float Lerp(float from, float to, float t) =>
        from + (to - from) * Math.Clamp(t, 0f, 1f);
}
