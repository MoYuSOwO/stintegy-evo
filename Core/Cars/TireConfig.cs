namespace StintegyEVO.Core.Cars;

public sealed class TireConfig
{
    public static readonly TireConfig Default = new();

    // Shared tire-model constants; compounds cannot override them per instance.
    public const float TireWorkReferenceSpeedMps = 30f;
    public const float MaximumTireWorkSpeedMultiplier = 1.5f;
    public const float LongitudinalHeatExponent = 4f;
    // Ordinary directional work becomes progressively more dissipative as the
    // car approaches the friction-circle limit; the transition stays smooth.
    public const float DirectionalHeatRampStartUse = 0.90f;
    public const float MinimumDirectionalHeatScale = 0.20f;
    public const float NearLimitHeatStartUse = 0.99f;
    public const float NearLimitWearExponent = 8f;
    public const float OverLimitHeatRate = 6f;
    public const float SideslipHeatRate = 4f;
    // The reduced-order tyre has no local slip velocity. This small tread-only
    // term stands in for the micro-slip caused by working a tyre in aerodynamically
    // unsteady air, and vanishes unless the tyre is doing lateral work.
    public const float WakeCorneringHeatRate = 4f;
    public const float RollingSurfaceHeatRate = 1f;
    public const float RollingCoreHeatRate = 0.35f;
    // A small share of the work done by the brakes reaches the tyre through
    // the disc, wheel and enclosed air. It enters the slow carcass mass, not
    // the tread, and follows the axle that actually supplied the braking.
    public const float BrakeCoreHeatRate = 1.5f;
    public const float RollingHeatReferenceSpeedMps = 30f;
    public const float SurfaceCoolingRate = 0.012f;
    public const float TrackSurfaceTransferRate = 0.004f;
    // The carcass primarily sheds stored heat back through the tread. Keep the
    // direct path smaller so low-use running can recover without erasing the
    // heat soak that Push and Attack create near the tire limit.
    public const float SurfaceCoreTransferRate = 0.04375f;
    public const float CoreHeatCapacityRatio = 12f;
    public const float CoreAirCoolingRate = 0.0057f;
    public const float SpeedCoolingReferenceMps = 60f;
    public const float MaximumSpeedCoolingMultiplier = 2.4f;

    public string CompoundId { get; init; } = "default";
    public float StartingSurfaceTempC { get; init; } = 25f;
    public float StartingCoreTempC { get; init; } = 25f;

    public float BaseMu { get; init; } = 1.72f;
    public float IdealSurfaceTempLowC { get; init; } = 85f;
    public float IdealSurfaceTempHighC { get; init; } = 115f;
    // Grip is flat inside the working window, then falls smoothly with the
    // squared distance from its nearest edge. There is no second overheat
    // threshold: progressively hotter or colder tread simply moves farther
    // along the same curve.
    public float ColdGripLossPerCSquared { get; init; } = 0.00060f;
    public float HotGripLossPerCSquared { get; init; } = 0.00070f;
    public float CoreOverheatTempC { get; init; } = 115f;
    public float CoreOverheatGripLossPerC { get; init; } = 0f;
    public float WearLinearGripLoss { get; init; } = 0.10f;
    public float WearCliffStart { get; init; } = 0.70f;
    public float WearCliffGripLoss { get; init; } = 0.15f;

    public float LateralHeatRate { get; init; } = 1.10f;
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

    // The ordinary work rates are lower than the old purely-quadratic model:
    // part of the same total stint wear now lives in NearLimitWearRate. This
    // keeps Normal's established tyre life while letting harder use spend the
    // tyre disproportionately faster.
    public float LateralWearRate { get; init; } = 0.00048f;
    public float LongitudinalWearRate { get; init; } = 0.000127f;
    /// <summary>
    /// Abrasion added as friction-circle use approaches one.
    ///
    /// The reduced-order tyre has no local slip velocity. A smooth high power
    /// is used instead of an on/off threshold: ordinary running still follows
    /// the directional work terms, while the last few percent of available
    /// grip carry increasingly more partial-slip work.
    /// </summary>
    public float NearLimitWearRate { get; init; } = 0.00012f;
    public float OverLimitWearRate { get; init; } = 0.00110f;
    public float SideslipWearRate { get; init; } = 0.00070f;
    public float ColdWearPerCSquared { get; init; } = 0.0015f;
    public float HotWearPerCSquared { get; init; } = 0.0035f;
}
