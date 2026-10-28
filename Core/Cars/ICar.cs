using System;
using System.Collections.Generic;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// A kind of car: how it moves, what it can do, and what it shows. One
/// object serves every car of that kind on the grid; each car's own running
/// state lives in its <see cref="CarState"/>.
///
/// This is what every car has: a body that moves on the road, a screen to
/// fill and a recorder's channels. Anything more -- tyres, stores, buttons,
/// an envelope to plan against -- a car has by implementing that feature's
/// interface (<see cref="ICarTires"/> and the rest, in CarFeatures). Nothing
/// outside the car reads its configuration, its tyre model or its physics
/// directly -- the shipped car included, which is an ordinary implementation
/// and has no way in that another car does not.
/// </summary>
public interface ICar
{
    /// <summary>The car as the world meets it: where its wheels are, how it turns under a blow, what air it leaves.</summary>
    CarSpec Spec { get; }

    /// <summary>
    /// The settings the strategist chooses between on this car, beside the
    /// tyre ladder every car has: an output ladder, an engine map, a
    /// hybrid's deployment. A strategy sets them in this order, and there are
    /// at most <see cref="CarStrategy.MaximumLadders"/>.
    /// </summary>
    IReadOnlyList<ModeLadder> Ladders { get; }

    /// <summary>What the car weighs at this moment, whatever it carries included.</summary>
    float TotalMassKg(CarState state);

    /// <summary>One step of the car being driven: the command in, the motion and whatever else the car keeps out.</summary>
    void Step(CarState state, CarStepInput input, float dt);

    /// <summary>Everything the car wants on the strategist's screen, written into <paramref name="dashboard"/>.</summary>
    void Describe(CarDashboard dashboard, CarState state, CarStrategy strategy);

    /// <summary>
    /// What this car records beyond what every car records
    /// (<see cref="CarState.Telemetry"/>): its own instruments, named by it,
    /// in the order <see cref="ReadTelemetry"/> fills them. For the screen
    /// and the recorder; a driver reads what it needs through the car's
    /// features (<see cref="ICarGripFeel"/>), not by an instrument's name.
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
    float AvailableDriveAcceleration, float OutputAvailability);
