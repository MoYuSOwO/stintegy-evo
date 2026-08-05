namespace StintegyEVO.Core.Drivers;

internal enum TacticalManeuverPhase
{
    /// <summary>Nobody in front worth calling a rival.</summary>
    Clear,

    /// <summary>
    /// Sitting behind the car in front on the racing line, because there is
    /// nowhere ahead this car is quick enough to get by. Most of a race.
    /// </summary>
    Following,

    /// <summary>
    /// There is somewhere ahead this car can afford, and it is not there yet.
    /// The car closes up on the racing line and takes the tow; it does not
    /// pull out, because the racing line is the quicker road and every metre
    /// beside it before the move is a metre given away.
    /// </summary>
    Preparing,

    /// <summary>Moving alongside through the braking zone.</summary>
    Committed,

    /// <summary>
    /// The move did not come off by the corner, so the car drops back in
    /// behind rather than running side by side through it.
    /// </summary>
    Yielding
}

/// <summary>
/// The deliberately small output boundary between racecraft and the existing
/// reference-line controller: one offset from the racing line.
/// </summary>
internal readonly record struct TacticalIntent(float TargetOffsetMeters)
{
    public static TacticalIntent Keep => default;
}
