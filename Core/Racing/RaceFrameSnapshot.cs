using System;
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
    public DriverProfile? DriverProfile { get; init; }
    public CarStrategy Strategy { get; init; }
    public float SteerAngleRadians { get; init; }
    public float SideslipHoldSeconds { get; init; }
    public bool Spinning { get; init; }
    public float SpinSeconds { get; init; }
    public int SpinEvents { get; init; }
    public float AirVelocityDeficit { get; init; }
    public float DownforceVelocityDeficit { get; init; }
    public float WakeDownforceLoss { get; init; }
    public float DragReduction { get; init; }
    public TireSnapshot FrontLeft { get; init; }
    public TireSnapshot FrontRight { get; init; }
    public TireSnapshot RearLeft { get; init; }
    public TireSnapshot RearRight { get; init; }
    public ImmutableArray<PowertrainResourceSnapshot> Resources { get; init; }
    public CarCapabilities Capabilities { get; init; }
    public CarTelemetry Telemetry { get; init; }
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
        IPowertrain powertrain = car.CarConfig.Powertrain;
        var resources = ImmutableArray.CreateBuilder<PowertrainResourceSnapshot>(powertrain.Resources.Count);
        for (int i = 0; i < powertrain.Resources.Count && i < PowertrainState.Capacity; i++)
        {
            PowertrainResourceInfo info = powertrain.Resources[i];
            resources.Add(new(info.Id, info.Label, info.Class, state.Energy[i], info.FullMassKg * state.Energy[i]));
        }
        float mass = car.CarConfig.MassKg + powertrain.ConsumableMassKg(state.Energy);
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
        )
        {
            DriverProfile = car.Driver?.Profile,
            Strategy = car.Strategy,
            SteerAngleRadians = state.SteerAngleRadians,
            SideslipHoldSeconds = state.SideslipHoldSeconds,
            Spinning = state.Spinning,
            SpinSeconds = state.SpinSeconds,
            SpinEvents = state.SpinEvents,
            AirVelocityDeficit = state.AirVelocityDeficit,
            DownforceVelocityDeficit = state.DownforceVelocityDeficit,
            WakeDownforceLoss = state.WakeDownforceLoss,
            DragReduction = state.DragReduction,
            FrontLeft = TireSnapshot.Capture(state.FrontLeft),
            FrontRight = TireSnapshot.Capture(state.FrontRight),
            RearLeft = TireSnapshot.Capture(state.RearLeft),
            RearRight = TireSnapshot.Capture(state.RearRight),
            Resources = resources.ToImmutable(),
            Capabilities = new(
                mass, car.CarConfig.MaxCurvatureRequest, car.CarConfig.MaxSteerAngleRadians,
                car.CarConfig.SteerRateLimitRadiansPerSecond, car.CarConfig.MaxDriveAcceleration,
                powertrain.DriveAccelerationLimit(state.Energy, car.Strategy, state.Speed, mass, car.CarConfig.MaxDriveAcceleration),
                powertrain.OutputAvailability(state.Energy), car.TireConfig.CompoundId),
            Telemetry = state.Telemetry,
            BoundaryContactSeconds = car.BoundaryContactSeconds,
            HitCar = car.HitCarThisStep,
            RaceDistanceMeters = car.Progress.RaceDistanceMeters,
            TrackLengthMeters = MathF.Max(0f, trackLengthMeters),
            TrackWidthMeters = pose.Sample.Width,
            WheelBaseMeters = car.CarConfig.WheelBaseMeters
        };
    }
}

/// <summary>A value copy of one tyre. It never exposes the live TireState.</summary>
public readonly record struct TireSnapshot(float SurfaceTempC, float CoreTempC, float Wear, float LoadN, float SurfaceGrip)
{
    internal static TireSnapshot Capture(TireState tire) =>
        new(tire.SurfaceTempC, tire.CoreTempC, tire.Wear, tire.LoadN, tire.SurfaceGrip);
}

public readonly record struct PowertrainResourceSnapshot(
    string Id, string Label, PowertrainResourceClass Class, float Fraction, float RemainingMassKg);

/// <summary>Typed vehicle limits and current output, independent of a controller's encoding.</summary>
public readonly record struct CarCapabilities(
    float MassKg, float MaxCurvatureRequest, float MaxSteerAngleRadians,
    float SteerRateLimitRadiansPerSecond, float MaxDriveAcceleration,
    float AvailableDriveAcceleration, float OutputAvailability, string TireCompoundId);

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
