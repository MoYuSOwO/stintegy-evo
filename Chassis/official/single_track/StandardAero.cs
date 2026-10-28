using System;
using StintegyEVO.Core.Cars;

namespace Official.SingleTrack;

/// <summary>
/// The car against the air. The chassis asks it how hard the air presses
/// the car down and holds it back, given what the air around the car is
/// doing (<see cref="AeroAir"/>); how it gets there is the aero's own.
/// </summary>
public interface IAero
{
    /// <summary>Downforce, in newtons per squared metre per second, in this air.</summary>
    float EffectiveDownforcePerSpeedSquared(in AeroAir air);

    /// <summary>
    /// The share of the downforce on the front axle; none to trim it the
    /// same as the weight.
    /// </summary>
    float? FrontDownforceShare { get; }

    /// <summary>Drag, in newtons, at this speed in this air.</summary>
    float DragForce(float speed, in AeroAir air);

    /// <summary>
    /// How much of a leading car's dirty air the tyres still feel: what the
    /// wake takes off the downforce, less whatever the car hands back.
    /// </summary>
    float TireWakeDownforceLoss(in AeroAir air);
}

/// <summary>
/// The air the car is moving through.
/// </summary>
/// <param name="AirVelocityDeficit">How much slower the air reaching the body is, as a share, behind another car.</param>
/// <param name="DownforceVelocityDeficit">The same, as the wings and floor meet it.</param>
/// <param name="WakeDownforceLoss">The share of the downforce the turbulence behind another car takes.</param>
/// <param name="ActiveAero">How far the car's active aero is deployed, zero to one.</param>
public readonly record struct AeroAir(
    float AirVelocityDeficit,
    float DownforceVelocityDeficit,
    float WakeDownforceLoss,
    float ActiveAero
)
{
    public static AeroAir Of(CarState state) => new(
        state.AirVelocityDeficit,
        state.DownforceVelocityDeficit,
        state.WakeDownforceLoss,
        state.Chassis().ActiveAero
    );
}

/// <summary>
/// The game's own aero: downforce and drag that go with the square of the
/// speed, a wake that slows the air and spoils the downforce of a car
/// following, and active aero that hands back part of what the wake took.
/// </summary>
public sealed class StandardAero : IAero
{
    /// <summary>
    /// Downforce, in newtons per squared metre per second: half the air's
    /// density times the lift area. A force, the wings and floor against the
    /// air, the same whatever the car weighs.
    ///
    /// 5.6 is a lift area of about 9 m²: downforce equal to the car's own
    /// weight at about 145 km/h, the rule of thumb for a Formula One car.
    /// Set with the tyres' load sensitivity against what such a car
    /// corners at: about 2 g in Monaco's hairpin, 3.3 g through Tabac at
    /// 160 km/h (Pirelli), 5 to 6 g through Maggotts and Becketts at
    /// 300 km/h.
    ///
    /// The drag was left where it was, so the lift to drag ratio is about
    /// 6.9, twice a Formula One car's 2.5 to 3.5: a near-future car, whose
    /// floor makes downforce far more cheaply than any built yet. That is a
    /// setting, not an oversight; it is what keeps the straights quick on
    /// this powertrain.
    ///
    /// Zero is a car with no wings at all: grip that does not care how fast
    /// the car is going, so a fast corner is worth no more than a slow one of
    /// the same radius.
    /// </summary>
    public float DownforcePerSpeedSquared { get; init; } = 5.6f;

    /// <summary>
    /// Aerodynamic drag, in newtons per squared metre per second: half the
    /// air's density times the drag area. A force, because the air pushes on
    /// the car's shape and not on its mass; written as an acceleration it was,
    /// and a car carrying a full tank met a tenth more air than the same car
    /// empty. 0.81 is a drag area of about 1.3 m², where a high-downforce
    /// single-seater sits, and the 0.0009 per kilogram the car had at its
    /// 900 kg dry weight.
    /// </summary>
    public float DragPerSpeedSquared { get; init; } = 0.81f;

    /// <summary>
    /// Fraction of the downforce lost to a leading car's wake that the active
    /// aero hands back while push-to-pass runs: flaps that trim to the
    /// disturbed air rather than to clean air. It scales what the wake
    /// removed, so in clean air it does nothing, and at one it would erase
    /// the cornering penalty of following entirely; half keeps dirty air a
    /// real cost while letting a committed follower live close enough to
    /// strike. It does not touch drag: the push is power, not a stalled wing.
    /// </summary>
    public float ActiveWakeDownforceRecovery { get; init; } = 0.5f;

    /// <summary>
    /// The share of the downforce on the front axle. None trims the car so
    /// its downforce sits on the axles as its weight does.
    /// </summary>
    public float? FrontDownforceShare { get; init; }

    public float EffectiveDownforcePerSpeedSquared(in AeroAir air)
    {
        float metAir = 1f - Math.Clamp(air.DownforceVelocityDeficit, 0f, 1f);
        float usableDownforce = 1f - Math.Clamp(air.WakeDownforceLoss, 0f, 1f);
        float wakeFactor = metAir * metAir * usableDownforce;
        // Hands back part of what the wake took, and nothing more: in clean air
        // the factor is already one, so the active aero cannot make a car
        // quicker than itself with nobody in front.
        float restored = wakeFactor +
                         ActiveWakeDownforceRecovery *
                         Math.Clamp(air.ActiveAero, 0f, 1f) *
                         (1f - wakeFactor);
        return MathF.Max(
            0f,
            DownforcePerSpeedSquared * restored
        );
    }

    public float DragForce(float speed, in AeroAir air)
    {
        float metAir = 1f - Math.Clamp(air.AirVelocityDeficit, 0f, 1f);
        return DragPerSpeedSquared * speed * speed * metAir * metAir;
    }

    /// <summary>
    /// The share of the wake's downforce disruption the tires actually feel.
    /// Whatever the active aero hands back to the load model must also stop
    /// shaking the car, or an assisted car would corner on recovered grip
    /// while being charged full dirty-air tyre temperature for it.
    /// </summary>
    public float TireWakeDownforceLoss(in AeroAir air) =>
        air.WakeDownforceLoss *
        (1f - ActiveWakeDownforceRecovery * Math.Clamp(air.ActiveAero, 0f, 1f));
}
