namespace StintegyEVO.Core.Cars;

public sealed class TireConfig
{
    public static readonly TireConfig Default = new();

    // Shared tire-model constants; compounds cannot override them per instance.
    public const float TireWorkReferenceSpeedMps = 30f;
    public const float MaximumTireWorkSpeedMultiplier = 1.5f;
    public const float LongitudinalHeatExponent = 4f;
    public const float NearLimitHeatStartUse = 0.99f;
    public const float OverLimitHeatRate = 6f;
    public const float SideslipHeatRate = 4f;
    // The reduced-order tyre has no local slip velocity. This small tread-only
    // term stands in for the micro-slip caused by working a tyre in aerodynamically
    // unsteady air, and vanishes unless the tyre is doing lateral work.
    public const float WakeCorneringHeatRate = 4f;
    public const float RollingSurfaceHeatRate = 1f;
    public const float RollingCoreHeatRate = 0.35f;
    public const float RollingHeatReferenceSpeedMps = 30f;
    public const float SurfaceCoolingRate = 0.012f;
    public const float TrackSurfaceTransferRate = 0.004f;
    public const float SurfaceCoreTransferRate = 0.035f;
    public const float CoreHeatCapacityRatio = 12f;
    public const float CoreAirCoolingRate = 0.0053f;
    public const float SpeedCoolingReferenceMps = 60f;
    public const float MaximumSpeedCoolingMultiplier = 2.4f;

    public string CompoundId { get; init; } = "default";
    public float StartingSurfaceTempC { get; init; } = 25f;
    public float StartingCoreTempC { get; init; } = 25f;

    public float BaseMu { get; init; } = 1.72f;
    public float IdealSurfaceTempLowC { get; init; } = 85f;
    public float IdealSurfaceTempHighC { get; init; } = 115f;
    public float ColdGripLossPerC { get; init; } = 0.006f;
    public float HotGripLossPerC { get; init; } = 0.004f;
    public float CoreOverheatTempC { get; init; } = 115f;
    public float CoreOverheatGripLossPerC { get; init; } = 0f;
    public float WearLinearGripLoss { get; init; } = 0.10f;
    public float WearCliffStart { get; init; } = 0.70f;
    public float WearCliffGripLoss { get; init; } = 0.15f;

    public float LateralHeatRate { get; init; } = 0.70f;
    public float LongitudinalHeatRate { get; init; } = 0.55f;
    /// <summary>
    /// Extra tread heat at the edge of the friction circle.
    ///
    /// The reduced-order tyre has no local slip ratio or slip velocity, so
    /// this is a bounded stand-in for the partial sliding that appears just
    /// before the axle saturates. It is additive: reaching the threshold must
    /// not multiply all of the ordinary cornering and traction heat.
    /// </summary>
    public float NearLimitHeatRate { get; init; } = 1.50f;

    public float LateralWearRate { get; init; } = 0.00055f;
    public float LongitudinalWearRate { get; init; } = 0.000146f;
    public float OverLimitWearRate { get; init; } = 0.00110f;
    public float SideslipWearRate { get; init; } = 0.00070f;
    public float HotWearStartTempC { get; init; } = 115f;
    public float HotWearSlope { get; init; } = 0.035f;
}
