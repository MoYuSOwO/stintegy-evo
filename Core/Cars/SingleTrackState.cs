using System;
namespace StintegyEVO.Core.Cars;

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
/// The single-track chassis's own part of a car's state: its four tyres and
/// the compound on them, its stores, its push-to-pass and active aero, and
/// what it last read off its axles. Nothing outside the chassis reads this
/// directly; the race and the driver go through the car's features.
/// </summary>
public sealed class SingleTrackState : ICarStateExtension
{
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
