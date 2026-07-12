using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
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

    internal RaceFrameSnapshot(float raceTimeSeconds, RaceCarSnapshot[] cars)
    {
        RaceTimeSeconds = raceTimeSeconds;
        _cars = cars ?? throw new ArgumentNullException(nameof(cars));
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
}
