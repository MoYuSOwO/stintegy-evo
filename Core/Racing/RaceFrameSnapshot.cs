using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Racing;

/// <summary>
/// Immutable vehicle data captured before any driver is evaluated for a physics substep.
/// </summary>
public readonly record struct RaceCarSnapshot(
    string Id,
    Vector2 Position,
    float HeadingRadians,
    float SideslipAngleRadians,
    float YawRateRadiansPerSecond,
    float SpeedMetersPerSecond,
    float LongitudinalAccelMetersPerSecondSquared,
    float LateralAccelMetersPerSecondSquared,
    float TrackS,
    float TrackD,
    float TotalDistanceMeters,
    int Lap,
    TrackRegion Region,
    float LengthMeters,
    float WidthMeters,
    float MaximumBrakeDecelerationMetersPerSecondSquared,
    DriverInput LastInput
)
{
    public float VelocityHeadingRadians => MathHelper.NormalizeAngle(
        HeadingRadians + SideslipAngleRadians
    );

    public Vector2 Velocity => new(
        MathF.Cos(VelocityHeadingRadians) * SpeedMetersPerSecond,
        MathF.Sin(VelocityHeadingRadians) * SpeedMetersPerSecond
    );

    internal static RaceCarSnapshot Capture(RaceCar car, TrackPose pose)
    {
        CarState state = car.State;
        return new RaceCarSnapshot(
            car.Id,
            state.Position,
            state.Heading,
            state.SideslipAngleRadians,
            state.YawRateRadiansPerSecond,
            state.Speed,
            state.FilteredLongitudinalAccel,
            state.FilteredLateralAccel,
            pose.S,
            pose.D,
            car.Progress.TotalDistance,
            car.Progress.Lap,
            TrackBoundaryResolver.Classify(pose),
            car.Collision.LengthMeters,
            car.Collision.WidthMeters,
            car.CarConfig.MaxBrakeAccel,
            car.LastInput
        );
    }
}

/// <summary>
/// A stable view of every car at the beginning of one physics substep.
/// </summary>
public readonly struct RaceFrameSnapshot
{
    private readonly RaceCarSnapshot[]? _cars;
    private readonly TrafficMotionPlan?[]? _trafficMotionPlans;
    private readonly TrafficMotionPlan?[]? _previousTrafficMotionPlans;

    internal RaceFrameSnapshot(
        float raceTimeSeconds,
        RaceCarSnapshot[] cars,
        TrafficMotionPlan?[] trafficMotionPlans
    ) : this(raceTimeSeconds, cars, trafficMotionPlans, null)
    {
    }

    internal RaceFrameSnapshot(
        float raceTimeSeconds,
        RaceCarSnapshot[] cars,
        TrafficMotionPlan?[] trafficMotionPlans,
        TrafficMotionPlan?[]? previousTrafficMotionPlans
    )
    {
        RaceTimeSeconds = raceTimeSeconds;
        _cars = cars ?? throw new ArgumentNullException(nameof(cars));
        _trafficMotionPlans = trafficMotionPlans ??
                              throw new ArgumentNullException(
                                  nameof(trafficMotionPlans)
                              );
        _previousTrafficMotionPlans = previousTrafficMotionPlans;
    }

    public float RaceTimeSeconds { get; }
    public int Count => _cars?.Length ?? 0;
    public ReadOnlySpan<RaceCarSnapshot> Cars => _cars;

    public RaceCarSnapshot this[int index]
    {
        get
        {
            if (_cars is null)
                throw new IndexOutOfRangeException();
            return _cars[index];
        }
    }

    public bool TryGetCar(string id, out RaceCarSnapshot car)
    {
        ArgumentNullException.ThrowIfNull(id);

        if (_cars is not null)
        {
            foreach (RaceCarSnapshot candidate in _cars)
            {
                if (string.Equals(candidate.Id, id, StringComparison.Ordinal))
                {
                    car = candidate;
                    return true;
                }
            }
        }

        car = default;
        return false;
    }

    internal TrafficMotionPlan? GetTrafficMotionPlan(int carIndex)
    {
        if (_trafficMotionPlans is null ||
            (uint)carIndex >= (uint)Count ||
            carIndex >= _trafficMotionPlans.Length)
        {
            return null;
        }
        return _trafficMotionPlans[carIndex] is { Count: > 0 } plan
            ? plan
            : null;
    }

    internal TrafficMotionPlan? GetPreviousTrafficMotionPlan(int carIndex)
    {
        if (_previousTrafficMotionPlans is null ||
            (uint)carIndex >= (uint)Count ||
            carIndex >= _previousTrafficMotionPlans.Length)
        {
            return null;
        }
        return _previousTrafficMotionPlans[carIndex] is { Count: > 0 } plan
            ? plan
            : null;
    }

    internal TrafficMotionPlan? FindTrafficMotionPlan(string carId)
    {
        ArgumentNullException.ThrowIfNull(carId);
        if (_cars is null || _trafficMotionPlans is null)
            return null;

        int count = Math.Min(_cars.Length, _trafficMotionPlans.Length);
        for (int i = 0; i < count; i++)
        {
            if (string.Equals(_cars[i].Id, carId, StringComparison.Ordinal) &&
                _trafficMotionPlans[i] is { Count: > 0 } plan)
            {
                return plan;
            }
        }
        return null;
    }

    /// <summary>
    /// Finds a simulation-owned snapshot of the car's preceding frozen plan.
    /// This view is supplied only during the write-only planning phase; the
    /// current-frame plan array remains hidden until the freeze barrier.
    /// </summary>
    internal TrafficMotionPlan? FindPreviousTrafficMotionPlan(string carId)
    {
        ArgumentNullException.ThrowIfNull(carId);
        if (_cars is null || _previousTrafficMotionPlans is null)
            return null;

        int count = Math.Min(
            _cars.Length,
            _previousTrafficMotionPlans.Length
        );
        for (int i = 0; i < count; i++)
        {
            if (string.Equals(_cars[i].Id, carId, StringComparison.Ordinal))
            {
                return GetPreviousTrafficMotionPlan(i);
            }
        }
        return null;
    }
}
