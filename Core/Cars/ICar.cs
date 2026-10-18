using System;
using System.Collections.Generic;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// A kind of car: how it moves, what it can do, and what it shows. One
/// object serves every car of that kind on the grid; each car's own running
/// state lives in its <see cref="CarState"/>.
///
/// This is the whole of what the race, the driver and the screen know about
/// a car. Nothing outside the car reads its configuration, its tyre model or
/// its physics directly -- the shipped car included, which is an ordinary
/// implementation of this interface and has no way in that another car does
/// not.
/// </summary>
public interface ICar
{
    /// <summary>The car as the world meets it: where its wheels are, how it turns under a blow, what air it leaves.</summary>
    CarSpec Spec { get; }

    /// <summary>
    /// What it runs on and what the strategist spends: the stores, the
    /// output ladder and what the powertrain can deliver.
    /// </summary>
    IPowertrain Powertrain { get; }

    /// <summary>
    /// The settings the strategist chooses between on this car, beside the
    /// tyre ladder every car has: an output ladder, an engine map, a
    /// hybrid's deployment. A strategy sets them in this order, and there are
    /// at most <see cref="CarStrategy.MaximumLadders"/>.
    /// </summary>
    IReadOnlyList<ModeLadder> Ladders { get; }

    /// <summary>What the car weighs at this moment, stores included.</summary>
    float TotalMassKg(CarState state);

    /// <summary>One step of the car being driven: the command in, the motion, the tyres and the stores out.</summary>
    void Step(CarState state, CarPhysicsStepInput input, float dt);

    /// <summary>
    /// The published performance envelope: what this car can do right now
    /// at a speed and round a curvature. Drivers and the offline bake plan
    /// against it; it changes nothing.
    /// </summary>
    /// <param name="gripUsage">The share of the friction circle to plan against, usually <see cref="GripUsage"/>.</param>
    CarPerformanceLimits EstimatePerformanceLimits(
        CarState state,
        CarStrategy strategy,
        float speed,
        float curvature,
        float gripUsage = 1f,
        float assumedLongitudinalAcceleration = 0f,
        float frontBrakeBiasOffset = 0f
    );

    /// <summary>The steady corner the car can hold at this speed, read with that corner's own load on the outside wheels.</summary>
    float EstimateCorneringLimit(CarState state, CarStrategy strategy, float speed, float gripUsage = 1f);

    /// <summary>
    /// The road-wheel angle that would turn this car onto a curvature, as it
    /// is now, on this road: the steering a driver who knew the car exactly
    /// would do. The car obeys the angle it is given, not this.
    /// </summary>
    SteeringEstimate EstimateSteering(CarState state, RoadAttitude road);

    /// <summary>How much of the tyres' grip the strategy's tyre rung lets the driver use.</summary>
    float GripUsage(CarStrategy strategy);

    /// <summary>
    /// The compounds this car can be given, from the tyre packs its pack
    /// declares; the first is what it is given when nobody says otherwise.
    /// No car runs a compound it does not list.
    /// </summary>
    IReadOnlyList<TireCompound> Compounds { get; }

    /// <summary>
    /// A new set of <paramref name="compound"/> (its full id) on every
    /// corner, off the blankets. A compound the car does not list is refused.
    /// </summary>
    void FitTires(CarState state, string compound);

    /// <summary>A new set of what is on, or of the first compound if nothing is.</summary>
    void InstallFreshTires(CarState state);

    /// <summary>
    /// Where the tyre on this corner works, for the driver deciding how hard
    /// to lean on it and the screen colouring its temperature.
    /// </summary>
    TireWindow TireWindow(CarState state, WheelId wheel);

    /// <summary>The limits a controller is told about, as they stand.</summary>
    CarCapabilities Capabilities(CarState state, CarStrategy strategy);

    /// <summary>Seconds of push-to-pass the car has left to spend.</summary>
    float PushToPassRemainingSeconds(CarState state);

    /// <summary>Everything the car wants on the strategist's screen, written into <paramref name="dashboard"/>.</summary>
    void Describe(CarDashboard dashboard, CarState state, CarStrategy strategy);

    /// <summary>
    /// What this car records beyond what every car records
    /// (<see cref="CarState.Telemetry"/>): its own instruments, named by it,
    /// in the order <see cref="ReadTelemetry"/> fills them.
    /// </summary>
    IReadOnlyList<TelemetryChannel> TelemetryChannels { get; }

    /// <summary>The current reading of each of <see cref="TelemetryChannels"/>, in order.</summary>
    void ReadTelemetry(CarState state, Span<float> values);
}

/// <summary>One of a car's own instruments, as a recorder or a graph shows it.</summary>
/// <param name="Id">Stable name, for a column header or a saved file.</param>
public readonly record struct TelemetryChannel(string Id, string Label, string Unit);

/// <summary>
/// The car as the rest of the world meets it, which is what a contact, a
/// patch of road or a car behind needs and nothing more.
/// </summary>
public sealed record CarSpec
{
    public float WheelBaseMeters { get; init; }

    /// <summary>Front axle ahead of the centre of mass.</summary>
    public float FrontAxleOffsetMeters { get; init; }

    /// <summary>Rear axle behind the centre of mass.</summary>
    public float RearAxleOffsetMeters { get; init; }

    public float TrackWidthMeters { get; init; }
    public float YawInertiaKgM2 { get; init; }

    /// <summary>The hardest the brakes will ever stop the car, for whoever plans around it.</summary>
    public float MaximumBrakeDeceleration { get; init; }

    /// <summary>
    /// Below this speed the car moves as its wheels point and carries no
    /// rotation of its own, so a blow can move it but not spin it.
    /// </summary>
    public float KinematicBelowSpeed { get; init; }

    /// <summary>The air the car leaves behind it.</summary>
    public CarWake Wake { get; init; } = new();

    /// <summary>How much of the disturbed air in front reaches the car's working surfaces.</summary>
    public float DirtyAirSensitivity { get; init; } = 1f;
}

/// <summary>The disturbed air a car leaves behind, as a car following reads it.</summary>
public sealed record CarWake
{
    /// <summary>How much slower the air is right behind the car, as a share of its speed.</summary>
    public float VelocityDeficit { get; init; }

    /// <summary>The gap at which the tow's drag relief has halved.</summary>
    public float TowHalfDistanceMeters { get; init; } = 1f;

    /// <summary>How fast the slowed air rises out of a follower's downforce, over distance.</summary>
    public float DownforceDecayLengthMeters { get; init; } = 1f;

    /// <summary>How much of a follower's downforce the turbulence takes, right behind.</summary>
    public float DownforceDisruption { get; init; }

    /// <summary>How fast that turbulence fades with distance.</summary>
    public float DirtyAirDecayLengthMeters { get; init; } = 1f;
}

/// <summary>Typed vehicle limits and current output, independent of a controller's encoding.</summary>
public readonly record struct CarCapabilities(
    float MassKg, float MaxCurvatureRequest, float MaxSteerAngleRadians,
    float SteerRateLimitRadiansPerSecond, float MaxDriveAcceleration,
    float AvailableDriveAcceleration, float OutputAvailability, string TireCompoundId);
