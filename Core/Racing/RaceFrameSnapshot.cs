using System;
using System.Collections.Generic;
using System.Numerics;
using System.Collections.Immutable;
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
    public DriverProfile? DriverProfile { get; init; }
    public CarStrategy Strategy { get; init; }

    /// <summary>
    /// What the body is doing beyond its motion -- its wheels' angle, what
    /// the seat feels, a spin -- as the car tells it; none on a car without
    /// <see cref="ICarMotion"/>.
    /// </summary>
    public CarMotion? Motion { get; init; }

    /// <summary>The air the car sits in, worked out by the world from where every car is (<see cref="CarAir"/>).</summary>
    public float AirVelocityDeficit { get; init; }
    public float DownforceVelocityDeficit { get; init; }
    public float WakeDownforceLoss { get; init; }

    /// <summary>
    /// The four tyres, front left, front right, rear left, rear right; empty
    /// on a car without changeable tyres (<see cref="ICarTires"/>).
    /// </summary>
    public ImmutableArray<TireReading> Tires { get; init; }

    /// <summary>The compound on the car, by its full id; none on a car without changeable tyres.</summary>
    public string? TireCompound { get; init; }

    /// <summary>The car's buttons as they stand (<see cref="ICarButtons"/>), in its order; empty on a car with none.</summary>
    public ImmutableArray<CarButtonReading> Buttons { get; init; }

    /// <summary>How much of each axle's grip the car is using; none on a car that does not say (<see cref="ICarGripFeel"/>).</summary>
    public GripUse? GripUse { get; init; }

    /// <summary>The car's stores (<see cref="ICarStores"/>); empty on a car with none.</summary>
    public ImmutableArray<PowertrainResourceSnapshot> Resources { get; init; }

    /// <summary>The limits a controller is told about; zero on a car with no envelope (<see cref="ICarEnvelope"/>).</summary>
    public CarCapabilities Capabilities { get; init; }

    /// <summary>
    /// The road-wheel angle for any curvature, read off the car before the
    /// step: what a driver who knew the car exactly would turn the wheels
    /// to (<see cref="SteeringEstimate.AngleFor"/>). The car obeys the angle
    /// a driver gives it, not this. Empty on a car with no envelope
    /// (<see cref="ICarEnvelope"/>).
    /// </summary>
    public SteeringEstimate Steering { get; init; }
    public float BoundaryContactSeconds { get; init; }
    public bool HitCar { get; init; }
    public float RaceDistanceMeters { get; init; }
    public float TrackLengthMeters { get; init; }
    public float TrackWidthMeters { get; init; }
    public float WheelBaseMeters { get; init; }

    public float VelocityHeadingRadians => MathHelper.NormalizeAngle(
        HeadingRadians + SideslipAngleRadians
    );

    public Vector2 Velocity => new(
        MathF.Cos(VelocityHeadingRadians) * SpeedMetersPerSecond,
        MathF.Sin(VelocityHeadingRadians) * SpeedMetersPerSecond
    );

    internal static RaceCarSnapshot Capture(
        RaceCar car,
        TrackPose pose,
        float trackLengthMeters = 0f
    )
    {
        CarState state = car.State;
        ICar model = car.Car;
        var resources = ImmutableArray<PowertrainResourceSnapshot>.Empty;
        if (model is ICarStores stores)
        {
            PowertrainState left = stores.Stores(state);
            IReadOnlyList<PowertrainResourceInfo> declared = stores.Resources;
            var builder = ImmutableArray.CreateBuilder<PowertrainResourceSnapshot>(declared.Count);
            for (int i = 0; i < declared.Count && i < PowertrainState.Capacity; i++)
            {
                PowertrainResourceInfo info = declared[i];
                builder.Add(new(info.Id, info.Label, info.Class, left[i], info.FullMassKg * left[i]));
            }
            resources = builder.ToImmutable();
        }
        var tires = ImmutableArray<TireReading>.Empty;
        if (model is ICarTires tyres)
            tires = [
                tyres.Tire(state, WheelId.FrontLeft), tyres.Tire(state, WheelId.FrontRight),
                tyres.Tire(state, WheelId.RearLeft), tyres.Tire(state, WheelId.RearRight)];
        var buttons = ImmutableArray<CarButtonReading>.Empty;
        if (model is ICarButtons held)
        {
            var builder = ImmutableArray.CreateBuilder<CarButtonReading>(held.Buttons.Count);
            for (int i = 0; i < held.Buttons.Count; i++)
                builder.Add(held.Read(state, i));
            buttons = builder.ToImmutable();
        }
        ICarEnvelope? envelope = model as ICarEnvelope;
        return new RaceCarSnapshot(
            car.Id,
            state.Position,
            state.Heading,
            state.SideslipAngleRadians,
            state.YawRateRadiansPerSecond,
            state.Speed,
            pose.S,
            pose.D,
            car.Progress.TotalDistance,
            car.Progress.Lap,
            TrackBoundaryResolver.Classify(pose),
            car.Collision.LengthMeters,
            car.Collision.WidthMeters,
            model.Spec.MaximumBrakeDeceleration,
            car.LastInput
        )
        {
            DriverProfile = car.Driver?.Profile,
            Strategy = car.Strategy,
            Motion = (model as ICarMotion)?.Motion(state),
            Tires = tires,
            TireCompound = (model as ICarTires)?.Compound(state),
            Buttons = buttons,
            GripUse = (model as ICarGripFeel)?.GripUse(state),
            Resources = resources,
            Capabilities = envelope?.Capabilities(state, car.Strategy) ?? default,
            Steering = envelope?.EstimateSteering(state, RaceSimulation.RoadAttitudeAt(pose.Sample, pose.D)) ?? default,
            BoundaryContactSeconds = car.BoundaryContactSeconds,
            HitCar = car.HitCarThisStep,
            RaceDistanceMeters = car.Progress.RaceDistanceMeters,
            TrackLengthMeters = MathF.Max(0f, trackLengthMeters),
            TrackWidthMeters = pose.Sample.Width,
            WheelBaseMeters = model.Spec.WheelBaseMeters
        };
    }
}

/// <summary>
/// A value copy of one tyre, with where that tyre works as its car tells it.
/// It never exposes the live TireState.
/// </summary>
public readonly record struct PowertrainResourceSnapshot(
    string Id, string Label, PowertrainResourceClass Class, float Fraction, float RemainingMassKg);

public readonly record struct RaceEnvironmentSnapshot(float AirTempC, float TrackTempC, float SurfaceGripScalar);

/// <summary>
/// A retained, immutable physical world frame: positions, attitude, speeds, track
/// projection, tyres, energy, strategy state, contact and telemetry. No controller's
/// private plan, network handle, reward value or training-episode state is part of the
/// world, and a retained frame does not change as the live cars move on.
/// </summary>
public readonly struct RaceFrameSnapshot
{
    private readonly ImmutableArray<RaceCarSnapshot> _cars;

    internal RaceFrameSnapshot(float raceTimeSeconds, RaceEnvironmentSnapshot environment, RaceCarSnapshot[] cars)
    {
        RaceTimeSeconds = raceTimeSeconds;
        Environment = environment;
        _cars = ImmutableArray.CreateRange(cars);
    }

    public float RaceTimeSeconds { get; }
    public RaceEnvironmentSnapshot Environment { get; }
    public int Count => _cars.IsDefault ? 0 : _cars.Length;
    public ReadOnlySpan<RaceCarSnapshot> Cars => _cars.AsSpan();
    public RaceCarSnapshot this[int index] => _cars[index];

    public bool TryGetCar(string id, out RaceCarSnapshot car)
    {
        ArgumentNullException.ThrowIfNull(id);
        foreach (RaceCarSnapshot candidate in Cars)
        {
            if (string.Equals(candidate.Id, id, StringComparison.Ordinal))
            {
                car = candidate;
                return true;
            }
        }
        car = default;
        return false;
    }
}
