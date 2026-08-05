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

    public void CopyFrom(TrafficMotionPlan source)
    {
        ArgumentNullException.ThrowIfNull(source);
        if (ReferenceEquals(this, source))
            return;

        EnsureCapacity(source.Count);
        Array.Copy(source._points, _points, source.Count);
        Count = source.Count;
    }

    public void BuildFrom(VehiclePathPrediction path)
    {
        BuildFromCore(path, speedPlan: null);
    }

    public void BuildFrom(
        VehiclePathPrediction path,
        VehicleSpeedLookahead speedPlan
    )
    {
        ArgumentNullException.ThrowIfNull(speedPlan);
        BuildFromCore(path, speedPlan);
    }

    private void BuildFromCore(
        VehiclePathPrediction path,
        VehicleSpeedLookahead? speedPlan
    )
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
        float previousSpeed = PointSpeed(path[0], speedPlan);
        for (int i = 0; i < path.Count; i++)
        {
            VehiclePathPredictionPoint point = path[i];
            float speed = i == 0
                ? previousSpeed
                : PointSpeed(point, speedPlan);
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

    private static float PointSpeed(
        in VehiclePathPredictionPoint point,
        VehicleSpeedLookahead? speedPlan
    )
    {
        float speed = speedPlan is null
            ? point.EstimatedSpeed
            : speedPlan.Sample(point.DistanceMeters).TargetSpeed;
        return MathF.Max(0f, speed);
    }

    /// <summary>
    /// When this plan expects to have travelled a given distance.
    ///
    /// This is what turns a gap in metres into a gap in seconds. Two cars a
    /// fixed distance apart are never at the same point of the road, so one is
    /// braking while the other accelerates, and the distance between them
    /// swings by ten metres a corner without either gaining anything. Asking
    /// instead how long it takes to reach where the other car is now compares
    /// them at the same point of the road, and the swing cancels: the answer
    /// moves only when one car is genuinely quicker than the other over the
    /// same ground. It is the figure a pit wall reads as +0.3.
    /// </summary>
    public bool TryTimeToTravel(float distanceMeters, out float seconds)
    {
        seconds = 0f;
        if (Count < 2 || distanceMeters < 0f)
            return false;

        float travelled = 0f;
        for (int i = 1; i < Count; i++)
        {
            float step = Vector2.Distance(
                _points[i - 1].Position,
                _points[i].Position
            );
            if (travelled + step >= distanceMeters)
            {
                float t = step > 1e-4f
                    ? (distanceMeters - travelled) / step
                    : 1f;
                seconds = _points[i - 1].TimeSeconds +
                          (_points[i].TimeSeconds - _points[i - 1].TimeSeconds) *
                          Math.Clamp(t, 0f, 1f);
                return true;
            }
            travelled += step;
        }
        return false;
    }

    /// <summary>
    /// How far this plan expects to have travelled after a given time.
    ///
    /// The mirror of asking when a distance is reached, and the one needed to
    /// place two cars against each other at a corner: the follower knows how
    /// long it will take to get there, and what settles the move is where the
    /// car in front will be by then. Comparing instead how long each takes to
    /// cover the same number of metres compares two different pieces of road,
    /// because the car in front starts further along.
    /// </summary>
    public bool TryDistanceTravelled(float seconds, out float meters)
    {
        meters = 0f;
        if (Count < 2 || seconds < 0f)
            return false;

        float travelled = 0f;
        for (int i = 1; i < Count; i++)
        {
            float step = Vector2.Distance(
                _points[i - 1].Position,
                _points[i].Position
            );
            float span = _points[i].TimeSeconds - _points[i - 1].TimeSeconds;
            if (_points[i].TimeSeconds >= seconds)
            {
                float t = span > 1e-5f
                    ? (seconds - _points[i - 1].TimeSeconds) / span
                    : 1f;
                meters = travelled + step * Math.Clamp(t, 0f, 1f);
                return true;
            }
            travelled += step;
        }
        return false;
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
    void PrepareTrafficMotionPlan(
        in RaceDriverFrameContext context,
        float dt
    );

    TrafficMotionPlan? FreezeTrafficMotionPlan();
}
