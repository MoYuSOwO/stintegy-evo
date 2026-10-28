using System;
using StintegyEVO.Core.Cars;

namespace Official.SingleTrack;

/// <summary>
/// What the single-track chassis reads off its two axles each step: how
/// much each could give and how much of it was used. Another kind of car
/// has no axles to speak of in these terms, so this is the chassis's own
/// record and not part of what every car reports (<see cref="CarTelemetry"/>).
/// </summary>
public readonly record struct AxleTelemetry(
    float FrontGripAccel,
    float RearGripAccel,
    float FrontLateralUse,
    float RearLateralUse,
    float FrontLongitudinalUse,
    float RearLongitudinalUse,
    float CombinedGripLimiterCutAccel,
    float InducedDragAccel,
    float RearSlideSeverity
);

/// <summary>
/// The single-track chassis's own part of a car's state: where its wheels
/// point, the referee's count of its spins, the air it sits in, its four
/// tyres and the compound on them, its stores, its push-to-pass and active
/// aero, and what it last read off its axles. Nothing outside the chassis
/// reads this directly; the race and the driver go through the car's
/// features.
/// </summary>
public sealed class SingleTrackState : ICarStateExtension
{
    /// <summary>
    /// Where the front wheels are actually pointed, relative to the body.
    /// A state rather than a number worked out from the driver's request,
    /// because the wheels take time to get there and the time is the point:
    /// a slide is caught by lock that arrives soon enough, and not caught by
    /// the same lock arriving late.
    /// </summary>
    public float SteerAngleRadians { get; set; }

    /// <summary>
    /// How long the car has been sideways past the angle the referee
    /// watches. Reset the moment it comes back under, so a flick that is
    /// caught costs nothing.
    /// </summary>
    public float SideslipHoldSeconds { get; set; }

    /// <summary>
    /// True while the car is spinning: the driver's hands are off it and
    /// the physics is playing a scripted rotation until it hands back.
    /// </summary>
    public bool Spinning { get; set; }

    /// <summary>
    /// How long the current spin has been running. Zero when not spinning.
    /// </summary>
    public float SpinSeconds { get; set; }

    /// <summary>
    /// How many times this car has been declared lost. A scoreline: a
    /// graduating driver has none of these in a session.
    /// </summary>
    public int SpinEvents { get; set; }

    public float FilteredLongitudinalAccel { get; set; }
    public float FilteredLateralAccel { get; set; }

    /// <summary>
    /// How much of its own speed the air around this car is already carrying,
    /// because a car in front has dragged it along.
    ///
    /// This lives on the car rather than being worked out where it is used
    /// because two very different things read it. The physics needs it to know
    /// what the car is doing now; the speed planner needs it to know what the
    /// car could do next, and the planner is handed a state, not a grid. Put
    /// here, a car that has caught a tow plans on having it, which is the whole
    /// point: the run to the corner is quicker, so the road in front becomes
    /// somewhere a pass can be made, without anything in the racecraft being
    /// told that a tow exists.
    ///
    /// Set by the world once a frame from one frozen picture of where every
    /// car is, so it does not depend on the order the cars are stepped in:
    /// it arrives in the step input (<see cref="CarStepInput.Air"/>).
    /// </summary>
    public float AirVelocityDeficit { get; set; }

    /// <summary>
    /// Additional fraction of aerodynamic downforce made unavailable by the
    /// turbulent and rotating part of a leading car's wake. The loss caused
    /// directly by reduced air speed is not included here.
    /// </summary>
    public float WakeDownforceLoss { get; set; }

    /// <summary>
    /// Share of oncoming-air speed missing at this car's working aero
    /// surfaces. The body's own deficit is <see cref="AirVelocityDeficit"/>;
    /// this one decays much faster with distance, because the wake rises off
    /// the road while wings and floor stay close to it.
    /// </summary>
    public float DownforceVelocityDeficit { get; set; }

    /// <summary>What the chassis did last step, as its own instruments read it.</summary>
    public CarTelemetry Telemetry { get; set; }

    public AxleTelemetry Axles { get; set; }

    /// <summary>
    /// How much of each of its consumables the car has left. What the slots
    /// mean is declared by the powertrain fitted to it - charge on an
    /// electric car, fuel on a petrol one - so read them through that rather
    /// than assuming.
    /// </summary>
    public PowertrainState Energy { get; set; } = PowertrainState.Full;

    /// <summary>
    /// How strongly the active aero is trimmed to a leading car's wake, zero
    /// to one. Physics sets it from push-to-pass every step; it hands back
    /// the configured share of the downforce the wake took, capped at
    /// clean-air downforce, and does nothing in clean air.
    /// </summary>
    public float ActiveAero { get; set; }

    /// <summary>The car's push-to-pass: what is used, what is running, what is waiting.</summary>
    public PushToPassState PushToPass { get; set; }

    /// <summary>The compound on the car, by its full id; none before the car has been given any.</summary>
    public string? Compound { get; set; }

    public TireState FrontLeft { get; } = new();
    public TireState FrontRight { get; } = new();
    public TireState RearLeft { get; } = new();
    public TireState RearRight { get; } = new();

    public TireState GetTire(WheelId wheel) => wheel switch
    {
        WheelId.FrontLeft => FrontLeft,
        WheelId.FrontRight => FrontRight,
        WheelId.RearLeft => RearLeft,
        WheelId.RearRight => RearRight,
        _ => FrontLeft
    };

    /// <summary>A new tyre on every corner, off the blankets, whatever the compound.</summary>
    public void InstallFreshTires()
    {
        foreach (TireState tire in new[] { FrontLeft, FrontRight, RearLeft, RearRight })
            tire.Reset(Tire.BlanketTempC, Tire.BlanketTempC);
    }

    /// <summary>Every tyre brought to these temperatures, in place of the blankets' (<see cref="Tire.BlanketTempC"/>).</summary>
    public void WarmTires(float surfaceTempC, float coreTempC)
    {
        foreach (TireState tire in new[] { FrontLeft, FrontRight, RearLeft, RearRight })
        {
            tire.SurfaceTempC = surfaceTempC;
            tire.CoreTempC = coreTempC;
        }
    }

    /// <summary>The stores and the active aero kept within what they can be.</summary>
    public void Normalize()
    {
        PowertrainState energy = Energy;
        energy.Clamp();
        Energy = energy;
        ActiveAero = Math.Clamp(ActiveAero, 0f, 1f);
        AirVelocityDeficit = Math.Clamp(AirVelocityDeficit, 0f, 1f);
        WakeDownforceLoss = Math.Clamp(WakeDownforceLoss, 0f, 1f);
        DownforceVelocityDeficit = Math.Clamp(DownforceVelocityDeficit, 0f, 1f);
    }

    public ICarStateExtension Clone()
    {
        var clone = new SingleTrackState();
        clone.CopyFrom(this);
        return clone;
    }

    public void CopyFrom(ICarStateExtension other)
    {
        var from = (SingleTrackState)other;
        SteerAngleRadians = from.SteerAngleRadians;
        SideslipHoldSeconds = from.SideslipHoldSeconds;
        Spinning = from.Spinning;
        SpinSeconds = from.SpinSeconds;
        SpinEvents = from.SpinEvents;
        FilteredLongitudinalAccel = from.FilteredLongitudinalAccel;
        FilteredLateralAccel = from.FilteredLateralAccel;
        AirVelocityDeficit = from.AirVelocityDeficit;
        WakeDownforceLoss = from.WakeDownforceLoss;
        DownforceVelocityDeficit = from.DownforceVelocityDeficit;
        Telemetry = from.Telemetry;
        Axles = from.Axles;
        Energy = from.Energy;
        ActiveAero = from.ActiveAero;
        PushToPass = from.PushToPass;
        Compound = from.Compound;
        FrontLeft.CopyFrom(from.FrontLeft);
        FrontRight.CopyFrom(from.FrontRight);
        RearLeft.CopyFrom(from.RearLeft);
        RearRight.CopyFrom(from.RearRight);
    }

    /// <summary>The chassis's part of <paramref name="state"/>, made the first time it is asked for.</summary>
    public static SingleTrackState Of(CarState state)
    {
        if (state.Own is SingleTrackState own)
            return own;
        own = new SingleTrackState();
        state.Own = own;
        return own;
    }
}

public static class SingleTrackStateExtensions
{
    /// <summary>The axles as the single-track chassis last read them; empty on a car that is not one.</summary>
    public static AxleTelemetry Axles(this CarState state) =>
        state.Own is SingleTrackState own ? own.Axles : default;

    /// <summary>
    /// The single-track chassis's own part of <paramref name="state"/>, made
    /// the first time it is asked for: for the chassis itself and for tests
    /// of it. Anyone else reads a car through its features.
    /// </summary>
    public static SingleTrackState Chassis(this CarState state) => SingleTrackState.Of(state);
}
