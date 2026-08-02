using System;
using System.Numerics;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Drivers;

internal readonly record struct TrafficMotionPlanPoint(
    float TimeSeconds,
    Vector2 Position,
    float HeadingRadians,
    float SpeedMetersPerSecond
);

/// <summary>
/// Reusable time parameterization of a driver's traffic-free spatial
/// prediction. A plan is frozen before any driver evaluates the next frame.
/// </summary>
internal sealed class TrafficMotionPlan
{
    private TrafficMotionPlanPoint[] _points = [];

    public int Count { get; private set; }
    public float EndTimeSeconds =>
        Count == 0 ? 0f : _points[Count - 1].TimeSeconds;

    public void Clear()
    {
        Count = 0;
    }

    public void BuildFrom(VehiclePathPrediction path)
    {
        ArgumentNullException.ThrowIfNull(path);
        if (path.Count == 0)
        {
            Clear();
            return;
        }

        EnsureCapacity(path.Count);
        Count = 0;
        float time = 0f;
        float previousSpeed = MathF.Max(0f, path[0].EstimatedSpeed);
        for (int i = 0; i < path.Count; i++)
        {
            VehiclePathPredictionPoint point = path[i];
            float speed = i == 0
                ? previousSpeed
                : MathF.Max(0f, point.EstimatedSpeed);
            if (i > 0)
            {
                float distance = Vector2.Distance(
                    path[i - 1].Position,
                    point.Position
                );
                float averageSpeed = MathF.Max(
                    (previousSpeed + speed) * 0.5f,
                    0.5f
                );
                time += distance / averageSpeed;
            }

            _points[Count++] = new TrafficMotionPlanPoint(
                time,
                point.Position,
                point.VelocityHeading,
                speed
            );
            previousSpeed = speed;
        }
    }

    public bool TrySample(float timeSeconds, out TrafficMotionPlanPoint point)
    {
        if (Count == 0 || timeSeconds < 0f || timeSeconds > EndTimeSeconds)
        {
            point = default;
            return false;
        }
        if (Count == 1 || timeSeconds <= 0f)
        {
            point = _points[0];
            return true;
        }
        if (timeSeconds >= EndTimeSeconds)
        {
            point = _points[Count - 1];
            return true;
        }

        int low = 0;
        int high = Count - 1;
        while (high - low > 1)
        {
            int middle = (low + high) >> 1;
            if (_points[middle].TimeSeconds <= timeSeconds)
                low = middle;
            else
                high = middle;
        }

        TrafficMotionPlanPoint before = _points[low];
        TrafficMotionPlanPoint after = _points[high];
        float duration = MathF.Max(
            after.TimeSeconds - before.TimeSeconds,
            1e-6f
        );
        float t = Math.Clamp(
            (timeSeconds - before.TimeSeconds) / duration,
            0f,
            1f
        );
        point = new TrafficMotionPlanPoint(
            timeSeconds,
            Vector2.Lerp(before.Position, after.Position, t),
            MathHelper.NormalizeAngle(
                before.HeadingRadians +
                MathHelper.NormalizeAngle(
                    after.HeadingRadians - before.HeadingRadians
                ) * t
            ),
            Lerp(
                before.SpeedMetersPerSecond,
                after.SpeedMetersPerSecond,
                t
            )
        );
        return true;
    }

    private void EnsureCapacity(int required)
    {
        if (_points.Length >= required)
            return;

        int capacity = Math.Max(required, Math.Max(16, _points.Length * 2));
        Array.Resize(ref _points, capacity);
    }

    private static float Lerp(float from, float to, float t)
    {
        return from + (to - from) * t;
    }
}

/// <summary>
/// Supplies a driver-owned motion buffer that remains immutable throughout
/// the following shared driver-evaluation phase.
/// </summary>
internal interface ITrafficMotionPlanSource
{
    TrafficMotionPlan? FreezeTrafficMotionPlan();
}
