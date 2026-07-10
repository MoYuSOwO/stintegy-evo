namespace TheStint.Core.Racing;

public sealed class VehicleSpeedPlanningConfig
{
    public float MaximumSpeedEstimateMultiplier { get; init; } = 1.08f;
    public float LateralGripUsage { get; init; } = 0.92f;
    public float DriveAccelerationUsage { get; init; } = 1f;
    public float BrakeDecelerationUsage { get; init; } = 0.9f;
    public float PlanningStepMeters { get; init; } = 2f;
    public int IntegrationSubsteps { get; init; } = 3;
    public int ClosedLoopPasses { get; init; } = 8;
    public int AccelerationSolveIterations { get; init; } = 4;
    public float ReplanIntervalSeconds { get; init; } = 5f;
    public float CurvatureCorrectionHorizonMeters { get; init; } = 150f;
    public float CurvatureCorrectionDecayTimeSeconds { get; init; } = 1f;
    public float MinimumCurvatureCorrectionDecayMeters { get; init; } = 15f;
    public float CurvatureCorrectionActivationThreshold { get; init; } = 0.002f;
    public float CurvatureEpsilon { get; init; } = 1e-4f;
    public float MinimumSegmentLengthMeters { get; init; } = 1e-4f;
    public float ConvergenceToleranceMetersPerSecond { get; init; } = 0.01f;
}
