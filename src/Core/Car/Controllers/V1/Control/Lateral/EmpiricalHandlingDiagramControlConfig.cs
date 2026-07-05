namespace StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;

public sealed class EmpiricalHandlingDiagramControlConfig
{
    public float LookaheadBaseMeters { get; init; } = 5.0f;
    public float LookaheadSpeedGainSeconds { get; init; } = 0.45f;
    public float MinimumLookaheadMeters { get; init; } = 8.0f;
    public float MaximumLookaheadMeters { get; init; } = 42.0f;
    public float MinimumControlSpeedMetersPerSecond { get; init; } = 2.0f;
    public float MaximumSteeringRateRadiansPerSecond { get; init; } = 7.0f;
    public float MaximumLateralErrorMeters { get; init; } = 8.0f;
    public float MaximumHeadingErrorRadians { get; init; } = 1.0f;
    public float MaximumFeedbackSteeringRadians { get; init; } = 0.55f;
    public float BetaDampingGain { get; init; } = 0.32f;
    public float YawRateDampingGain { get; init; } = 0.12f;
    public float MaximumBetaDampingSteeringRadians { get; init; } = 0.16f;
    public float MaximumYawDampingSteeringRadians { get; init; } = 0.24f;
    public float MaximumStabilitySteeringRadians { get; init; } = 0.24f;
    public float StabilityFeedbackCutFactor { get; init; } = 0.85f;
    public float ConflictingBetaDampingScale { get; init; } = 0.15f;
    public float PathFeedbackYawConflictStartRadiansPerSecond { get; init; } = 0.08f;
    public float PathFeedbackYawConflictFullRadiansPerSecond { get; init; } = 0.28f;
    public float ConflictingPathFeedbackScale { get; init; } = 0.15f;
    public float MaximumBetaReferenceRadians { get; init; } = 0.05f;
    public float BetaReferenceGain { get; init; } = 0.12f;
    public float VscBetaStartRadians { get; init; } = 0.10f;
    public float VscBetaFullRadians { get; init; } = 0.30f;
    public float VscYawRateErrorStartRadiansPerSecond { get; init; } = 0.45f;
    public float VscYawRateErrorFullRadiansPerSecond { get; init; } = 1.80f;
    public EmpiricalHandlingDiagramCalibrationConfig Calibration { get; init; } = new();
}
