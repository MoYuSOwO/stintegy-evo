using System.Numerics;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Cross-frame traffic hysteresis committed only for the selected path. The
/// struct is copied into each candidate evaluation, so evaluating one path
/// cannot affect another and the copy itself does not allocate.
/// </summary>
internal struct TrafficConstraintMemory
{
    public string? OpponentId;
    public TrafficSpeedConstraintKind Kind;
    public float HeldUntilSeconds;
    public float RemainingDistanceMeters;
    public float TargetSpeedMetersPerSecond;
    public float ConflictTimeSeconds;
    public Vector2 EgoPosition;

    public void Clear()
    {
        OpponentId = null;
        Kind = TrafficSpeedConstraintKind.None;
        HeldUntilSeconds = 0f;
        RemainingDistanceMeters = 0f;
        TargetSpeedMetersPerSecond = 0f;
        ConflictTimeSeconds = 0f;
        EgoPosition = default;
    }
}

internal readonly record struct TrafficAwareSpeedPlan(
    DynamicPathSpeedPlan SpeedPlan,
    TrafficSpeedConstraint TrafficConstraint,
    TrafficConstraintMemory NextTrafficMemory
);
