using System;

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
    /// <summary>
    /// Where the longitudinal heat proxy reaches its full value.
    ///
    /// This constant used to be called <c>NearLimitHeatStartUse</c> and did
    /// two jobs: it ended this ramp, and it started a hand-placed branch
    /// that piled extra heat into a tyre near the limit. The second job is
    /// gone. Lateral tread heat is force times sliding now, and sliding
    /// keeps growing past the peak while force falls, so the near-limit
    /// acceleration that branch existed to supply comes out of the physics
    /// continuously and needs no threshold to switch it on.
    ///
    /// Being honest about what happened to the number: the branch was
    /// retired, the ramp's end point was not, and it is the same 0.99. This
    /// is a rename of a job that survived, not a constant that was quietly
    /// kept alive under a new name -- the dead half is dead, and it left
    /// this headstone.
    /// </summary>
    public const float DirectionalHeatRampEndUse = 0.99f;
    public const float NearLimitWearExponent = 8f;
    public const float OverLimitHeatRate = 6f;
    // The reduced-order tyre has no local slip velocity. This small tread-only
    // term stands in for the micro-slip caused by working a tyre in aerodynamically
    // unsteady air, and vanishes unless the tyre is doing lateral work.
    public const float WakeCorneringHeatRate = 4f;
    public const float RollingSurfaceHeatRate = 0.7f;
    public const float RollingCoreHeatRate = 0.3f;
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

    /// <summary>
    /// How long the rubber takes to get anywhere, as a multiple of what it
    /// used to take.
    ///
    /// Every thermal mass is divided by this, and only the masses are — so
    /// every equilibrium the car has is exactly where it was and only the
    /// journey to it is quicker. That is the whole reason it is one number
    /// applied uniformly rather than a rate tuned here and a rate tuned
    /// there: the steady-state skidpad temperature is a fixed point of
    /// heat against cooling, and neither is touched. The ratio between the
    /// tread and the carcass is untouched too, so a short spike still goes
    /// into the surface and not the core.
    ///
    /// It exists because the tyres could not be warmed up. From 25 C, two
    /// laps of hard cornering reached 59 C against a working floor of 85,
    /// and the range arrived at about 1500 seconds — fifteen laps. That
    /// was measured on the model before the heat rework as well, to within
    /// a degree, so it is a calibration the car always had and nothing had
    /// asked about, because nothing until now wanted to start a car cold.
    ///
    /// The anchor comes from outside: slicks with no blankets are in their
    /// window after one to three laps, which is what an out lap is for.
    /// </summary>
    public const float ThermalTimeScale = 0.20f;
    public const float CoreAirCoolingRate = 0.0057f;
    public const float SpeedCoolingReferenceMps = 60f;
    public const float MaximumSpeedCoolingMultiplier = 2.4f;

    public string CompoundId { get; init; } = "default";
    public float StartingSurfaceTempC { get; init; } = 25f;
    public float StartingCoreTempC { get; init; } = 25f;

    /// <summary>
    /// The share of the tyre's friction circle the combined-grip limiter lets
    /// the pedals take it to, the same whatever the tyre rung: the device's
    /// own margin, short of the peak where a tyre stops answering, at the
    /// figure the Normal rung always had, so a car on Normal drives exactly
    /// as before.
    ///
    /// It used to be a ladder, a smaller share at each rung down from Attack,
    /// so that Protect was a car held to 95 per cent of its tyres by the
    /// physics. The rungs are ways of driving now, and the driver carries
    /// them out (how much of each corner, of the brakes, of the throttle it
    /// spends); a second ladder underneath, in the car, doubled every gap
    /// and made the rung a property of the hardware rather than of the
    /// person driving it. A host that wants the limiter tighter still says
    /// so with <see cref="CarStrategy.WithTireGripUsage"/>.
    /// </summary>
    public float AccelerationUsage { get; init; } = 0.977f;

    /// <summary>The limiter's share at this rung: the same at every rung.</summary>
    public float GetAccelerationUsage(TireUsageMode mode) => AccelerationUsage;

    public float GetAccelerationUsage(CarStrategy strategy)
    {
        if (!strategy.TireGripUsageOverride.HasValue)
            return AccelerationUsage;
        return Math.Clamp(strategy.TireGripUsageOverride.Value, 0.05f, AccelerationUsage);
    }

    public float BaseMu { get; init; } = 1.72f;
    public float IdealSurfaceTempLowC { get; init; } = 85f;
    public float IdealSurfaceTempHighC { get; init; } = 115f;
    // Grip is flat inside the working window, then falls smoothly with the
    // squared distance from its nearest edge. There is no second overheat
    // threshold: progressively hotter or colder tread simply moves farther
    // along the same curve.
    public float ColdGripLossPerCSquared { get; init; } = 0.00060f;
    public float HotGripLossPerCSquared { get; init; } = 0.00025f;
    public float CoreOverheatTempC { get; init; } = 115f;
    public float CoreOverheatGripLossPerC { get; init; } = 0f;
    public float WearLinearGripLoss { get; init; } = 0.10f;
    public float WearCliffStart { get; init; } = 0.70f;
    public float WearCliffGripLoss { get; init; } = 0.15f;

    public float LateralHeatRate { get; init; } = 4.0f;
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
    public float LongitudinalWearRate { get; init; } = 0.0000546f;

    /// <summary>
    /// Lateral wear from slip work - force share times sliding speed - in
    /// place of the square of the force share. Off reproduces the old bill.
    /// </summary>
    public bool WearFromSlipWork { get; init; } = true;
    public float SlipWorkWearRate { get; init; } = 0.0002064f;
    /// <summary>
    /// Abrasion added as friction-circle use approaches one.
    ///
    /// The reduced-order tyre has no local slip velocity. A smooth high power
    /// is used instead of an on/off threshold: ordinary running still follows
    /// the directional work terms, while the last few percent of available
    /// grip carry increasingly more partial-slip work.
    /// </summary>
    public float NearLimitWearRate { get; init; } = 0.0000516f;
    public float OverLimitWearRate { get; init; } = 0.0004730f;
    public float ColdWearPerCSquared { get; init; } = 0.0015f;
    public float HotWearPerCSquared { get; init; } = 0.0035f;

    /// <summary>Tread temperature above which wear starts to climb.</summary>
    public float WearReferenceTempC { get; init; } = 100f;

    /// <summary>Degrees of tread above the reference for wear to grow by a factor of e.</summary>
    public float WearTempScaleC { get; init; } = 10f;

    /// <summary>
    /// The most a hot tread can multiply wear by. Past it the rubber is
    /// blistering whatever the temperature reads, and an uncapped curve
    /// would bill a spin for a set of tyres in a second.
    /// </summary>
    public float MaximumHotWearFactor { get; init; } = 6f;
}
