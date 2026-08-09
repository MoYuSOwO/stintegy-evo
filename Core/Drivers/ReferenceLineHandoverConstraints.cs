namespace StintegyEVO.Core.Drivers;

/// <summary>
/// The car-specific speed envelope and tactical deadline available when a
/// reference-line handover is committed. It is a frozen input to that one
/// decision; the resulting spatial line is not replanned every frame.
/// </summary>
internal readonly record struct ReferenceLineHandoverConstraints(
    VehicleSpeedLookahead BaselineSpeedPlan,
    float StandingLateralAccelerationLimit,
    float DownforceAccelerationPerSpeedSquared,
    float LatestCompletionDistanceMeters
)
{
    private const float GravityMetersPerSecondSquared = 9.80665f;
    private const float SpeedToleranceMetersPerSecond = 0.1f;

    public bool IsUsable =>
        BaselineSpeedPlan is { Count: > 0 } &&
        StandingLateralAccelerationLimit > 0f;

    public float MaximumCurvatureAt(float distanceMeters)
    {
        if (!IsUsable)
            return 0f;

        float speed = MathF.Max(
            0f,
            BaselineSpeedPlan.Sample(distanceMeters).TargetSpeed -
            SpeedToleranceMetersPerSecond
        );
        if (speed <= SpeedToleranceMetersPerSecond)
            return float.PositiveInfinity;

        float standingLimit = MathF.Max(
            0f,
            StandingLateralAccelerationLimit
        );
        return standingLimit / (speed * speed) +
               standingLimit *
               MathF.Max(0f, DownforceAccelerationPerSpeedSquared) /
               GravityMetersPerSecondSquared;
    }
}
