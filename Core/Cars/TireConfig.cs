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
    // Rolling heat in the tread. 0.8 since downforce became a force: before,
    // fuel aboard bought downforce, a full car pressed its tyres a tenth
    // harder, and the warm-up was tuned against that. Without it the tyres
    // ran three degrees cooler and a cold car at 80 per cent use never
    // reached its window; 0.8 puts both back where they were measured.
    //
    // Four fifths of that since the car took a Formula One car's downforce:
    // rolling heat goes with the load, and a fast lap now presses the tyres
    // about 1.6 times as hard (see LateralHeatRate for the split).
    public float RollingSurfaceHeatRate { get; init; } = 0.64f;
    public float RollingCoreHeatRate { get; init; } = 0.24f;
    // A small share of the work done by the brakes reaches the tyre through
    // the disc, wheel and enclosed air. It enters the slow carcass mass, not
    // the tread, and follows the axle that actually supplied the braking.
    public float BrakeCoreHeatRate { get; init; } = 1.5f;
    public const float RollingHeatReferenceSpeedMps = 30f;
    public const float SurfaceCoolingRate = 0.012f;
    public const float TrackSurfaceTransferRate = 0.004f;
    // The carcass primarily sheds stored heat back through the tread. Keep the
    // direct path smaller so low-use running can recover without erasing the
    // heat soak that Push and Attack create near the tire limit.
    public const float SurfaceCoreTransferRate = 0.04375f;
    // The carcass is what soaks. At 12 it was through in under a lap, so a
    // tyre driven too hard was overheated on the next straight and cooled
    // on the one after, and pushing cost nothing for a few laps the way it
    // does on a real car. At 18 a carcass held at the limit takes two or
    // three laps to come up to where it starts losing grip, and a lap of
    // easier driving to come back.
    public const float CoreHeatCapacityRatio = 18f;

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
    /// <summary>
    /// Where the tyre starts: off the blankets. The series heats its tyres
    /// before they go on the car, to about 70 C, so a car leaving the grid
    /// or the pit lane is a few degrees short of its window, not a whole
    /// ambient day short of it. A host running without blankets says 25.
    /// </summary>
    public float StartingSurfaceTempC { get; init; } = 70f;
    public float StartingCoreTempC { get; init; } = 70f;

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

    /// <summary>
    /// The tyre's friction at the load it was sized for, its own wheel's
    /// share of the car standing still: at its best temperature, new.
    /// </summary>
    public float BaseMu { get; init; } = 2.4f;

    /// <summary>
    /// How fast friction falls as the load rises: it goes as the load to
    /// the minus this. Zero is a tyre whose grip is exactly proportional
    /// to its load. 0.3 is a racing slick's: at the load of a fast corner,
    /// five times what it was sized for, it has two fifths less friction than
    /// standing still. It is why a car with this much downforce corners at
    /// under 6 g rather than 8.
    /// </summary>
    public float LoadSensitivity { get; init; } = 0.3f;
    public float IdealSurfaceTempLowC { get; init; } = 85f;
    public float IdealSurfaceTempHighC { get; init; } = 115f;
    // Grip is flat inside the working window, then falls smoothly with the
    // squared distance from its nearest edge. There is no second overheat
    // threshold: progressively hotter or colder tread simply moves farther
    // along the same curve.
    //
    // The cold side is measured against what cold slicks are known to cost:
    // with blankets an out lap gives away next to nothing, and the worst
    // case on record without them, a cold night at Le Mans, is about seven
    // per cent of a lap. At 0.0006 a 25 C start lost a quarter of its lap
    // at Silverstone; at 0.00008 it loses three per cent, and a 70 C start
    // off the blankets a tenth of a second. (The slower carcass below keeps
    // a cold tread cooler for longer, which is why this is a little flatter
    // than the 0.0001 that gave the same three per cent before it.)
    public float ColdGripLossPerCSquared { get; init; } = 0.00008f;

    // Too hot is two things. The heat is made in the tread, which spikes in
    // every corner and cools on the next straight; the carcass (the bulk of
    // the tyre, which is what grip is generally reckoned to follow best)
    // takes it in slowly and keeps it.
    //
    // The tread past its window costs a little and gives it back as soon as
    // it cools. Kept gentle because a qualifying lap at the limit is the
    // fastest lap a tyre does: at 0.00025 a car at the limit gave all of
    // its pace away to tread spikes on its first lap and was never quicker
    // than one two per cent short of it.
    //
    // The carcass past CoreOverheatTempC is the overheating a driver has to
    // manage, the kind described as being able to follow for a lap or two
    // before having to back off: it builds over laps and takes three per
    // cent of the grip for every degree. A car on Normal at Silverstone
    // holds its carcass at about 108 C; one held at the limit climbs past
    // 112 in two or three laps and is no faster than Normal by the fourth,
    // and a lap of Normal brings it back. How real tyres split the penalty
    // between tread and bulk is not published; these figures are set by
    // that behaviour, not by a measured curve.
    public float HotGripLossPerCSquared { get; init; } = 0.0001f;
    public float CoreOverheatTempC { get; init; } = 112f;
    public float CoreOverheatGripLossPerC { get; init; } = 0.03f;
    // A set run to the cliff is worth about two seconds at Silverstone, the
    // size of the gap between new and old tyres in the series this car
    // stands in for; at 0.10 it was four and a half. Past the cliff the
    // tyre still falls away, a little more gently than it did: about three
    // seconds over the next tenth of the tread rather than four.
    public float WearLinearGripLoss { get; init; } = 0.04f;
    public float WearCliffStart { get; init; } = 0.70f;
    public float WearCliffGripLoss { get; init; } = 0.12f;

    // Heat from work: what the tread carries times how fast it is dragged.
    // It is counted against the force the tyre gives, which with friction
    // falling with load grows more slowly than the load. Three fifths of
    // the rates the car had before its Formula One downforce, with the
    // rolling heat below at four fifths: the split between the two is what
    // puts both a stint at the limit and an out lap where they were. Held at
    // nine tenths of its grip at 55 m/s a tyre settles at about 113 C;
    // ordinary running at forty-odd metres a second holds the tread at
    // 74 C, as before; a cold set is in its window inside two and a half
    // laps.
    public float LateralHeatRate { get; init; } = 2.4f;
    public float LongitudinalHeatRate { get; init; } = 0.33f;
    /// <summary>
    /// Extra tread heat at the edge of what the car can hold.
    ///
    /// The reduced-order tyre has no contact patch, so it cannot see the
    /// patch starting to slide from the back as the corner fills it. Force
    /// times slip angle does not make up for it here: the car can only hold
    /// about nine tenths of the peak (<see cref="TireSlipCurve.SustainablePeakShare"/>),
    /// and that far below the peak the curve is still nearly straight. A
    /// car on Attack, flat against that limit, measured one degree hotter
    /// than the same car on Normal two per cent short of it, and no tyre
    /// could be driven too hard. This is the partial sliding, measured from
    /// the limit the car can actually reach rather than from a peak it
    /// never stands on. It is additive: reaching the edge must not multiply
    /// all of the ordinary cornering and traction heat.
    /// </summary>
    public float NearLimitHeatRate { get; init; } = 1.2f;

    /// <summary>
    /// Where the partial-sliding heat begins, as a share of the limit the
    /// car can hold. It grows with the square of the way from here to the
    /// limit, so a car a few per cent short pays little and one at the
    /// limit pays all of it.
    /// </summary>
    public float NearLimitHeatStartShare { get; init; } = 0.97f;

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
