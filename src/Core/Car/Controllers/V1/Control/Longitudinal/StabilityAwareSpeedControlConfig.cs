using StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;

namespace StintegyEVO.Core.Car.Controllers.V1.Control.Longitudinal;

public sealed class StabilityAwareSpeedControlConfig
{
    public float SpeedLookaheadBaseMeters { get; init; } = 12.0f;
    public float SpeedLookaheadTimeSeconds { get; init; } = 3.2f;
    public float SpeedErrorGain { get; init; } = 0.75f;
    public float AccelerationFeedForwardGain { get; init; } = 1.0f;
    public float MaximumSpeedErrorAccelerationMetersPerSecondSquared { get; init; } = 7.0f;
    public float MaximumCorrectiveBrakeMetersPerSecondSquared { get; init; } = 8.0f;
    public float FutureBrakingActivationMetersPerSecondSquared { get; init; } = 0.25f;
    public float StabilityRiskBrakeMetersPerSecondSquared { get; init; } = 5.0f;
    public float StabilityRiskThrottleCutStart { get; init; } = 0.15f;
    public float FullThrottleCutRisk { get; init; } = 0.65f;
    public float SlideThrottleCutFactor { get; init; } = 0.35f;
    public float MinimumSpeedForSlideRiskMetersPerSecond { get; init; } = 10.0f;
    public float StabilityBrakeStartSpeedMetersPerSecond { get; init; } = 6.0f;
    public float StabilityBrakeFullSpeedMetersPerSecond { get; init; } = 12.0f;
    public float TrackingLateralErrorStartMeters { get; init; } = 2.0f;
    public float TrackingLateralErrorFullMeters { get; init; } = 5.0f;
    public float TrackingHeadingErrorStartRadians { get; init; } = 0.25f;
    public float TrackingHeadingErrorFullRadians { get; init; } = 0.65f;
    public float TrackingRiskStartSpeedMetersPerSecond { get; init; } = 4.0f;
    public float TrackingRiskFullSpeedMetersPerSecond { get; init; } = 8.0f;
    public float TrackBoundaryExcessStartMeters { get; init; } = 0.25f;
    public float TrackBoundaryExcessFullMeters { get; init; } = 2.0f;
    public float TrackBoundaryRecoveryAccelerationCapMetersPerSecondSquared { get; init; } = 2.0f;
    public float TrackBoundaryRecoverySpeedMetersPerSecond { get; init; } = 5.0f;
    public float TrackBoundaryRecoveryDistanceMeters { get; init; } = 18.0f;
    public SpeedPlanningConfig EnvelopeConfig { get; init; } = new();
}
