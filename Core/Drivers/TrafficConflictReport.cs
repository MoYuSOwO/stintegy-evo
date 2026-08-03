namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Read-only summary of how much the frozen traffic plans delayed the selected
/// ego path during one completed driver-evaluation phase.
/// </summary>
internal readonly record struct TrafficConflictReport(
    TrafficSpeedConstraint Constraint,
    float EvaluationDistanceMeters,
    float FreeArrivalTimeSeconds,
    float ConstrainedArrivalTimeSeconds
)
{
    public bool Active => Constraint.Active;
    public string? OpponentId => Constraint.OpponentId;

    public float TimeLossSeconds
    {
        get
        {
            if (!Active)
                return 0f;
            if (!float.IsFinite(FreeArrivalTimeSeconds))
                return 0f;
            if (float.IsPositiveInfinity(ConstrainedArrivalTimeSeconds))
                return float.PositiveInfinity;
            if (!float.IsFinite(ConstrainedArrivalTimeSeconds))
                return 0f;
            return MathF.Max(
                0f,
                ConstrainedArrivalTimeSeconds - FreeArrivalTimeSeconds
            );
        }
    }
}
