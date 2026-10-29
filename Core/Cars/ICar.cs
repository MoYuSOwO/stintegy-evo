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
/// patch of road or a car behind needs and nothing more. Every number is the
/// car's to declare: none has a default standing in for one nobody chose,
/// and a car whose declaration is missing one, or has a nonsense one, is
/// refused before it races (<see cref="Problems"/>).
/// </summary>
public sealed record CarSpec
{
    /// <summary>From front axle to rear axle, m.</summary>
    public required float WheelBaseMeters { get; init; }

    /// <summary>Front axle ahead of the centre of mass, m: where the front wheels meet the road.</summary>
    public required float FrontAxleOffsetMeters { get; init; }

    /// <summary>Rear axle behind the centre of mass, m.</summary>
    public required float RearAxleOffsetMeters { get; init; }

    /// <summary>Between the left and right wheels' contact patches, m.</summary>
    public required float TrackWidthMeters { get; init; }

    /// <summary>Resistance to turning about the centre of mass, kg·m², for a blow off-centre.</summary>
    public required float YawInertiaKgM2 { get; init; }

    /// <summary>The hardest the brakes will ever stop the car, m/s², for whoever plans around it.</summary>
    public required float MaximumBrakeDeceleration { get; init; }

    /// <summary>
    /// Below this speed, m/s, the car moves as its wheels point and carries
    /// no rotation of its own, so a blow can move it but not spin it. Zero
    /// for a car that always carries its own rotation.
    /// </summary>
    public required float KinematicBelowSpeed { get; init; }

    /// <summary>The body other cars and the walls meet.</summary>
    public required CarBody Body { get; init; }

    /// <summary>The air the car leaves behind it.</summary>
    public required CarWake Wake { get; init; }

    /// <summary>How much of the disturbed air in front reaches the car's working surfaces: one all of it, zero none.</summary>
    public required float DirtyAirSensitivity { get; init; }

    /// <summary>What is wrong with this declaration, field by field; nothing for one the world can use.</summary>
    public IEnumerable<string> Problems()
    {
        foreach (string problem in Positive(
                     (nameof(WheelBaseMeters), WheelBaseMeters),
                     (nameof(FrontAxleOffsetMeters), FrontAxleOffsetMeters),
                     (nameof(RearAxleOffsetMeters), RearAxleOffsetMeters),
                     (nameof(TrackWidthMeters), TrackWidthMeters),
                     (nameof(YawInertiaKgM2), YawInertiaKgM2),
                     (nameof(MaximumBrakeDeceleration), MaximumBrakeDeceleration),
                     ($"{nameof(Body)}.{nameof(CarBody.LengthMeters)}", Body?.LengthMeters ?? 0f),
                     ($"{nameof(Body)}.{nameof(CarBody.WidthMeters)}", Body?.WidthMeters ?? 0f),
                     ($"{nameof(Wake)}.{nameof(CarWake.TowHalfDistanceMeters)}", Wake?.TowHalfDistanceMeters ?? 0f),
                     ($"{nameof(Wake)}.{nameof(CarWake.DownforceDecayLengthMeters)}", Wake?.DownforceDecayLengthMeters ?? 0f),
                     ($"{nameof(Wake)}.{nameof(CarWake.DirtyAirDecayLengthMeters)}", Wake?.DirtyAirDecayLengthMeters ?? 0f)))
            yield return problem;
        foreach (string problem in NotNegative(
                     (nameof(KinematicBelowSpeed), KinematicBelowSpeed),
                     (nameof(DirtyAirSensitivity), DirtyAirSensitivity),
                     ($"{nameof(Body)}.{nameof(CarBody.Friction)}", Body?.Friction ?? 0f),
                     ($"{nameof(Body)}.{nameof(CarBody.WallFriction)}", Body?.WallFriction ?? 0f),
                     ($"{nameof(Wake)}.{nameof(CarWake.VelocityDeficit)}", Wake?.VelocityDeficit ?? 0f),
                     ($"{nameof(Wake)}.{nameof(CarWake.DownforceDisruption)}", Wake?.DownforceDisruption ?? 0f)))
            yield return problem;
        foreach ((string name, float value) in new[]
                 {
                     ($"{nameof(Body)}.{nameof(CarBody.Restitution)}", Body?.Restitution ?? 0f),
                     ($"{nameof(Body)}.{nameof(CarBody.WallRestitution)}", Body?.WallRestitution ?? 0f),
                 })
            if (!(value >= 0f && value <= 1f))
                yield return $"{name} is {value}; a restitution is from zero to one";
        if (Body is null)
            yield return $"{nameof(Body)} is not declared";
        if (Wake is null)
            yield return $"{nameof(Wake)} is not declared";
    }

    private static IEnumerable<string> Positive(params (string Name, float Value)[] values)
    {
        foreach ((string name, float value) in values)
            if (!(value > 0f) || !float.IsFinite(value))
                yield return $"{name} is {value}; it must be above zero";
    }

    private static IEnumerable<string> NotNegative(params (string Name, float Value)[] values)
    {
        foreach ((string name, float value) in values)
            if (!(value >= 0f) || !float.IsFinite(value))
                yield return $"{name} is {value}; it must not be below zero";
    }
}

/// <summary>
/// The car's body as other cars and the walls meet it: a box about its centre
/// of mass, and how it gives and slides in a contact. The world pairs these
/// with its own walls and its contact solver.
/// </summary>
public sealed record CarBody
{
    /// <summary>Nose to tail, m.</summary>
    public required float LengthMeters { get; init; }

    /// <summary>Side to side at the widest, m.</summary>
    public required float WidthMeters { get; init; }

    /// <summary>How much of the closing speed comes back out of a contact with another car, zero to one. The pair takes the smaller.</summary>
    public required float Restitution { get; init; }

    /// <summary>Sliding friction against another car's body. The pair takes the smaller.</summary>
    public required float Friction { get; init; }

    /// <summary>
    /// How much of the speed into a barrier comes back out of it, zero to
    /// one. Full-scale crash tests into rigid barriers measure about a
    /// tenth: the car crumples and the barrier hardly gives.
    /// </summary>
    public required float WallRestitution { get; init; }

    /// <summary>Sliding friction against a barrier: carbon, aluminium and tyre sidewall on steel and concrete.</summary>
    public required float WallFriction { get; init; }
}

/// <summary>The disturbed air a car leaves behind, as a car following reads it.</summary>
public sealed record CarWake
{
    /// <summary>How much slower the air is right behind the car, as a share of its speed; zero for no tow.</summary>
    public required float VelocityDeficit { get; init; }

    /// <summary>The gap at which the tow's drag relief has halved, m.</summary>
    public required float TowHalfDistanceMeters { get; init; }

    /// <summary>How fast the slowed air rises out of a follower's downforce, over distance, m.</summary>
    public required float DownforceDecayLengthMeters { get; init; }

    /// <summary>How much of a follower's downforce the turbulence takes, right behind, as a share; zero for none.</summary>
    public required float DownforceDisruption { get; init; }

    /// <summary>How fast that turbulence fades with distance, m.</summary>
    public required float DirtyAirDecayLengthMeters { get; init; }
}

/// <summary>Typed vehicle limits and current output, independent of a controller's encoding.</summary>
public readonly record struct CarCapabilities(
    float MassKg, float MaxCurvatureRequest, float MaxSteerAngleRadians,
    float SteerRateLimitRadiansPerSecond, float MaxDriveAcceleration,
    float AvailableDriveAcceleration, float OutputAvailability);
