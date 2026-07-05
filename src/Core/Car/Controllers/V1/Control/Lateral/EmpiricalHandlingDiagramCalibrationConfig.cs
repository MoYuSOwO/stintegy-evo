namespace StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;

public sealed class EmpiricalHandlingDiagramCalibrationConfig
{
    public float MinimumFitSpeedMetersPerSecond { get; init; } = 8.0f;
    public float MaximumFitSpeedMetersPerSecond { get; init; } = 44.0f;
    public float SpeedStepMetersPerSecond { get; init; } = 4.0f;
    public int SteeringSamplesPerSide { get; init; } = 8;
    public float MaximumCalibrationSteeringInput { get; init; } = 0.62f;
    public float MinimumFitLateralAccelerationMetersPerSecondSquared { get; init; } = 1.2f;
    public float MaximumFitLateralAccelerationMetersPerSecondSquared { get; init; } = 18.0f;
    public float SimulationDtSeconds { get; init; } = 1.0f / 60.0f;
    public float SettleSeconds { get; init; } = 0.90f;
    public float MeasureSeconds { get; init; } = 0.45f;
    public float SpeedHoldGain { get; init; } = 0.23f;
    public float MaximumThrottleInput { get; init; } = 0.70f;
    public float MaximumBrakeInput { get; init; } = 0.35f;
    public float MaximumMeanSpeedErrorMetersPerSecond { get; init; } = 2.0f;
    public float MaximumSlideFraction { get; init; } = 0.35f;
    public float Ridge { get; init; } = 1e-6f;
}
