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
    public float WearGripLoss { get; init; } = 0.36f;

    public float LateralHeatRate { get; init; } = 17f;
    public float LongitudinalHeatRate { get; init; } = 8f;
    public float OverLimitHeatRate { get; init; } = 28f;
    public float SurfaceCoolingRate { get; init; } = 0.42f;
    public float SurfaceCoreTransferRate { get; init; } = 0.18f;
    public float CoreCoolingRate { get; init; } = 0.026f;
    public float SpeedCoolingReferenceMps { get; init; } = 70f;
    public float MaxSpeedCoolingMultiplier { get; init; } = 2f;

    public float LateralWearRate { get; init; } = 0.00022f;
    public float LongitudinalWearRate { get; init; } = 0.00008f;
    public float OverLimitWearRate { get; init; } = 0.00055f;
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
