using TheStint.Core.Cars;

namespace TheStint.Core.Drivers;

public sealed class VehicleSpeedPlanningConfig
{
    public float MaximumSpeedEstimateMultiplier { get; init; } = 1.08f;
    public float ProtectAccelerationUsage { get; init; } = 0.95f;
    public float LightAccelerationUsage { get; init; } = 0.955f;
    public float NormalAccelerationUsage { get; init; } = 0.96f;
    public float PushAccelerationUsage { get; init; } = 0.98f;
    public float AttackAccelerationUsage { get; init; } = 1f;
    // Driver confidence in the estimated longitudinal limit. The same value
    // shapes the planned profile and caps real-time acceleration requests.
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

    public float GetAccelerationUsage(TireUsageMode mode)
    {
        return mode switch
        {
            TireUsageMode.Protect => ProtectAccelerationUsage,
            TireUsageMode.Light => LightAccelerationUsage,
            TireUsageMode.Normal => NormalAccelerationUsage,
            TireUsageMode.Push => PushAccelerationUsage,
            TireUsageMode.Attack => AttackAccelerationUsage,
            _ => NormalAccelerationUsage
        };
    }

    public float GetAccelerationUsage(CarStrategy strategy)
    {
        if (!strategy.TireGripUsageOverride.HasValue)
            return GetAccelerationUsage(strategy.TireMode);

        return Math.Clamp(
            strategy.TireGripUsageOverride.Value,
            ProtectAccelerationUsage,
            AttackAccelerationUsage
        );
    }

    public float GetAccelerationUsage(float sliderPosition)
    {
        float scaled = Math.Clamp(sliderPosition, 0f, 1f) * 4f;
        int segment = Math.Min((int)scaled, 3);
        float t = scaled - segment;
        return segment switch
        {
            0 => Lerp(ProtectAccelerationUsage, LightAccelerationUsage, t),
            1 => Lerp(LightAccelerationUsage, NormalAccelerationUsage, t),
            2 => Lerp(NormalAccelerationUsage, PushAccelerationUsage, t),
            _ => Lerp(PushAccelerationUsage, AttackAccelerationUsage, t)
        };
    }

    private static float Lerp(float from, float to, float t) =>
        from + (to - from) * t;
}
