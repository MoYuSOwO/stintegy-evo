namespace StintegyEVO.Core.Drivers;

public enum TrafficSpeedConstraintKind
{
    None,
    Follow,
    Stop
}

/// <summary>
/// The most immediate traffic constraint applied to the current rolling speed
/// plan. OpponentId identifies snapshot state, never another driver's plan.
/// </summary>
public readonly record struct TrafficSpeedConstraint(
    TrafficSpeedConstraintKind Kind,
    string? OpponentId,
    float PathDistanceMeters,
    float TargetSpeedMetersPerSecond,
    float PredictedConflictTimeSeconds,
    float CurrentClearanceMeters
)
{
    public bool Active => Kind != TrafficSpeedConstraintKind.None;
}
