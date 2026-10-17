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

/// <summary>The single-track chassis's own part of a car's state.</summary>
public sealed class SingleTrackState : ICarStateExtension
{
    public AxleTelemetry Axles { get; set; }

    public ICarStateExtension Clone() => new SingleTrackState { Axles = Axles };

    public void CopyFrom(ICarStateExtension other) => Axles = ((SingleTrackState)other).Axles;

    /// <summary>The chassis's part of <paramref name="state"/>, made the first time it is asked for.</summary>
    internal static SingleTrackState Of(CarState state)
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
}
