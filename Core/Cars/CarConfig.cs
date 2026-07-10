namespace TheStint.Core.Cars;

public sealed class CarConfig
{
    public float MassKg { get; init; } = 820f;
    public float WheelBaseMeters { get; init; } = 3.1f;
    public float TrackWidthMeters { get; init; } = 1.65f;
    public float CenterOfGravityHeightMeters { get; init; } = 0.32f;
    public float FrontStaticLoadShare { get; init; } = 0.47f;
    public float FrontDriveShare { get; init; } = 0f;
    public float YawInertiaKgM2 { get; init; } = 1450f;
    public float YawResponseTimeSeconds { get; init; } = 0.15f;
    public float SideslipRecoveryTimeSeconds { get; init; } = 0.15f;

    public float MaxCurvatureRequest { get; init; } = 0.32f;
    public float MaxDriveAccelRequest { get; init; } = 8f;
    public float MaxBrakeAccel { get; init; } = 12f;
    public float TractionControlActivationUse { get; init; } = 0.99f;
    public float TractionControlStrength { get; init; } = 0.65f;
    public float MinPowerSpeed { get; init; } = 8f;
    public float BatteryCapacityJoules { get; init; } = 42000000f;
    public float BatteryDriveEfficiency { get; init; } = 0.92f;
    public float LowSocPowerLimitStart { get; init; } = 0.08f;
    public float RegenEfficiency { get; init; } = 0.56f;
    public float RegenPowerCapWatts { get; init; } = 260000f;

    public float RollingDragAccel { get; init; } = 0.18f;
    public float AeroDragAccelPerSpeedSquared { get; init; } = 0.00046f;
    public float CorneringScrubAccel { get; init; } = 1.15f;
    public float OverLimitMinGripEfficiency { get; init; } = 0.8f;
    public float OverLimitCostCap { get; init; } = 2.5f;

    public float LoadTransferResponse { get; init; } = 8f;
    public float MinimumWheelLoadShare { get; init; } = 0.08f;

    public float GetBatteryForceAccelLimit(BatteryOutputMode mode)
    {
        return BatteryForceAccelLimits[CarModeIndex.ToIndex(mode)];
    }

    public float GetBatteryPowerLimitWatts(BatteryOutputMode mode)
    {
        return BatteryPowerLimitsWatts[CarModeIndex.ToIndex(mode)];
    }

    public float GetBatteryDrainMultiplier(BatteryOutputMode mode)
    {
        return BatteryDrainMultipliers[CarModeIndex.ToIndex(mode)];
    }

    private static readonly float[] BatteryForceAccelLimits =
    [
        2.1f,
        2.9f,
        3.8f,
        4.8f,
        5.7f
    ];

    private static readonly float[] BatteryPowerLimitsWatts =
    [
        160000f,
        215000f,
        285000f,
        365000f,
        455000f
    ];

    private static readonly float[] BatteryDrainMultipliers =
    [
        0.82f,
        0.92f,
        1.00f,
        1.11f,
        1.25f
    ];
}
