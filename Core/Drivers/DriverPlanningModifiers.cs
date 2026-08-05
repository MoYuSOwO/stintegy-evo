namespace StintegyEVO.Core.Drivers;

/// <summary>
/// What this driver, as opposed to any other, changes about the plan.
///
/// The two are kept apart because they are not the same kind of thing. Pace
/// is real, and it is a cornering figure: the car genuinely will not turn as
/// hard for this driver as it would for a better one, so it travels with the
/// car's limits rather than being applied on top of them. The grip scale is a
/// mistake - the driver has misread how much grip there is - and it belongs on
/// top, because a plan built on a wrong belief should be wrong in exactly the
/// way the belief is.
///
/// Neither of them touches the pedals any more. Everyone accelerates and
/// brakes as hard as the tyres allow; what separates drivers is how much of a
/// corner they can carry, and the speed a slower one loses there follows them
/// onto the straight by itself.
/// </summary>
public readonly record struct DriverPlanningModifiers(
    float PaceEfficiency,
    float EstimatedGripScale,
    float FrontBrakeBiasOffset
)
{
    public static readonly DriverPlanningModifiers Neutral = new(1f, 1f, 0f);
}
