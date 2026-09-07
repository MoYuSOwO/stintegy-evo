using StintegyEVO.Core.Cars;

namespace StintegyEVO.Core.Drivers;

public sealed class VehicleSpeedPlanningConfig
{
    public float MaximumSpeedEstimateMultiplier { get; init; } = 1.08f;
    // Driver confidence in the estimated longitudinal limit. The same value
    // shapes the planned profile and caps real-time acceleration requests.
    public float DriveAccelerationUsage { get; init; } = 1f;
    public float BrakeDecelerationUsage { get; init; } = 0.9f;
    public int IntegrationSubsteps { get; init; } = 3;
    public int AccelerationSolveIterations { get; init; } = 4;
    public float SpeedPlanningHorizonMeters { get; init; } = 600f;
    public float PathPredictionStepMeters { get; init; } = 2f;
    public float MinimumDynamicPredictionMeters { get; init; } = 30f;
    public float PredictionConvergenceHoldMeters { get; init; } = 10f;
    public float PredictionConvergenceLateralErrorMeters { get; init; } = 0.15f;
    public float PredictionConvergenceHeadingErrorRadians { get; init; } = 0.01f;
    public float PredictionConvergenceCurvatureError { get; init; } = 0.0015f;
    public bool EnableTrafficAvoidance { get; init; } = true;
    public float TrafficPredictionHorizonSeconds { get; init; } = 6f;
    public float TrafficMinimumGapMeters { get; init; } = 0.8f;
    public float TrafficFollowingTimeHeadwaySeconds { get; init; } = 0.075f;
    public float TrafficFollowingControlResponseSeconds { get; init; } = 0.4f;
    public float TrafficApproachDecelerationMetersPerSecondSquared { get; init; } = 3f;
    public float TrafficTimeHeadwaySeconds { get; init; } = 0.12f;
    public float TrafficLateralMergePredictionSeconds { get; init; } = 1f;
    public float TrafficLateralSafetyMarginMeters { get; init; } = 0.15f;
    public float TrafficLongitudinalSafetyMarginMeters { get; init; } = 0.25f;
    public float TrafficConstraintHoldSeconds { get; init; } = 0.35f;
    public int TrafficConstraintIterations { get; init; } = 2;
    // Cheap scalar iterations over the existing feasible speed profile. This
    // does not rerun vehicle-physics integration inside the solve.
    public int TrafficArrivalSolveIterations { get; init; } = 5;
    public float CurvatureEpsilon { get; init; } = 1e-4f;
    public float MinimumSegmentLengthMeters { get; init; } = 1e-4f;
    public float ConvergenceToleranceMetersPerSecond { get; init; } = 0.01f;
}
