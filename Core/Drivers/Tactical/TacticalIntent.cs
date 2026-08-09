namespace StintegyEVO.Core.Drivers;

internal enum TacticalManeuverPhase
{
    Observing,
    Anticipating,
    Committed,
    Executing,
    Returning
}

/// <summary>
/// The deliberately small output boundary between racecraft and the existing
/// reference-line controller. A maneuver chooses where the line should end
/// and the latest distance by which the transition should be complete; the
/// handover layer decides how gently it can get there. The foundation planner
/// currently emits Keep.
/// </summary>
internal readonly record struct TacticalIntent(
    float TargetOffsetMeters,
    float LatestCompletionDistanceMeters
)
{
    public static TacticalIntent Keep => new(
        TargetOffsetMeters: 0f,
        LatestCompletionDistanceMeters: float.PositiveInfinity
    );
}
