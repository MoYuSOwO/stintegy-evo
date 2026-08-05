namespace StintegyEVO.Core.Drivers;

/// <summary>
/// What this driver, as opposed to any other, changes about the plan.
///
/// The two scales are kept apart because they are not the same kind of thing
/// and they do not belong in the same places. Pace is real: the car genuinely
/// will not turn as hard for this driver as it would for a better one, and the
/// plan has to know that or it will lay out a lap the car cannot drive. The
/// estimation scale is a mistake: the driver has misread how much grip there
/// is, or how fast this corner goes, and the plan is wrong in exactly the way
/// the driver is wrong. Multiplying them together and carrying one number lost
/// the distinction, and with it the ability to put each where it belongs.
/// </summary>
public readonly record struct DriverPlanningModifiers(
    float PaceEfficiency,
    float EstimationScale,
    float FrontBrakeBiasOffset
)
{
    public static readonly DriverPlanningModifiers Neutral = new(1f, 1f, 0f);

    /// <summary>
    /// How hard the plan may ask the car to turn, as a share of what the tyres
    /// would give a perfect driver. Pace belongs here and only here: the tyre's
    /// limit is the tyre's, identical for everyone, but what comes out of it as
    /// cornering acceleration is not.
    /// </summary>
    internal float LateralConfidence => PaceEfficiency * EstimationScale;

    /// <summary>
    /// How hard the plan may accelerate and brake. No pace in it - everyone can
    /// press a pedal all the way down, and a slower driver who is behind on
    /// entry speed is still going to reach the same figure on the straight.
    /// </summary>
    internal float LongitudinalConfidence => EstimationScale;
}
