namespace StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;

public sealed class SpeedPlanningConfig
{
    public float MaximumSpeedMetersPerSecond { get; init; } = 85.0f;
    public float TerminalSpeedMetersPerSecond { get; init; } = float.PositiveInfinity;
    public float FrictionUsage { get; init; } = 0.95f;
    public float TrackFrictionMultiplier { get; init; } = 1.0f;
    public int LoadTransferIterations { get; init; } = 4;
    public int IntegrationSubsteps { get; init; } = 4;
    public int LateralSpeedSearchIterations { get; init; } = 24;
    public float CurvatureEpsilon { get; init; } = 1e-4f;
    public float MinimumSegmentLengthMeters { get; init; } = 1e-4f;
    public bool IncludeAeroDownforce { get; init; } = true;
    public bool IncludeAeroDrag { get; init; } = true;
    public bool IncludePowerLimit { get; init; } = true;
}
