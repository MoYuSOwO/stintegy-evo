namespace TheStint.Core.Cars;

public sealed class TireConfig
{
    public static readonly TireConfig Default = new();

    public string CompoundId { get; init; } = "default";
    public float StartingSurfaceTempC { get; init; } = 25f;
    public float StartingCoreTempC { get; init; } = 25f;

    public float BaseMu { get; init; } = 1.72f;
    public float IdealSurfaceTempC { get; init; } = 90f;
    public float ColdGripLossPerC { get; init; } = 0.006f;
    public float HotGripLossPerC { get; init; } = 0.004f;
    public float CoreOverheatTempC { get; init; } = 115f;
    public float CoreOverheatGripLossPerC { get; init; } = 0.003f;
    public float WearLinearGripLoss { get; init; } = 0.10f;
    public float WearCliffStart { get; init; } = 0.70f;
    public float WearCliffGripLoss { get; init; } = 0.15f;

    public float LateralHeatRate { get; init; } = 3.75f;
    public float LongitudinalHeatRate { get; init; } = 2.5f;
    public float LongitudinalHeatExponent { get; init; } = 4f;
    public float OverLimitHeatRate { get; init; } = 6f;
    public float SideslipHeatRate { get; init; } = 4f;
    public float SlipHeatReferenceSpeedMps { get; init; } = 30f;
    public float MaxSlipHeatSpeedMultiplier { get; init; } = 1.5f;
    public float RollingSurfaceHeatRate { get; init; } = 1f;
    public float RollingCoreHeatRate { get; init; } = 0.35f;
    public float RollingHeatReferenceSpeedMps { get; init; } = 30f;
    public float SurfaceCoolingRate { get; init; } = 0.012f;
    public float TrackSurfaceTransferRate { get; init; } = 0.004f;
    public float SurfaceCoreTransferRate { get; init; } = 0.035f;
    public float CoreCoolingRate { get; init; } = 0.002f;
    public float CoreHeatCapacityRatio { get; init; } = 4f;
    public float SpeedCoolingReferenceMps { get; init; } = 60f;
    public float MaxSpeedCoolingMultiplier { get; init; } = 2.4f;

    public float LateralWearRate { get; init; } = 0.00080f;
    public float LongitudinalWearRate { get; init; } = 0.00016f;
    public float OverLimitWearRate { get; init; } = 0.00110f;
    public float SideslipWearRate { get; init; } = 0.00070f;
    public float HotWearStartTempC { get; init; } = 105f;
    public float HotWearSlope { get; init; } = 0.035f;

    public float GetModeGripFactor(TireUsageMode mode)
    {
        return GripFactors[CarModeIndex.ToIndex(mode)];
    }

    public float GetModeHeatFactor(TireUsageMode mode)
    {
        return HeatFactors[CarModeIndex.ToIndex(mode)];
    }

    public float GetModeWearFactor(TireUsageMode mode)
    {
        return WearFactors[CarModeIndex.ToIndex(mode)];
    }

    private static readonly float[] GripFactors =
    [
        0.84f,
        0.93f,
        1.00f,
        1.06f,
        1.10f
    ];

    private static readonly float[] HeatFactors =
    [
        0.62f,
        0.82f,
        1.00f,
        1.28f,
        1.62f
    ];

    private static readonly float[] WearFactors =
    [
        0.55f,
        0.78f,
        1.00f,
        1.36f,
        1.78f
    ];
}
