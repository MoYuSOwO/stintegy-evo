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
/// reference-line controller. The foundation planner currently emits Keep.
/// </summary>
internal readonly record struct TacticalIntent(float TargetOffsetMeters)
{
    public static TacticalIntent Keep => default;
}
