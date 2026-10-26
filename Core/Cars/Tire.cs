using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// The tyre: friction that falls with load, a temperature window with a cold
/// and a hot side, a carcass that soaks, and wear that goes with sliding
/// work and falls off a cliff late in its life.
///
/// There is one tyre model and every car runs it, its own chassis or not;
/// a compound tunes it (<see cref="TireConfig"/>) and a tyre pack offers
/// compounds (<see cref="TireLibrary"/>). It is fixed because the drivers
/// read it: how hot is too hot, how much grip is left, what pushing costs
/// mean the same on every car only if every car's tyres are this one.
///
/// A chassis tells it how hard each tyre is pressed and how it is being
/// used -- loads, slip, how much of its grip each direction is taking -- and
/// it answers with the force it can give and keeps its own temperature and
/// wear.
/// </summary>
public sealed class Tire
{
    /// <summary>
    /// Where a fresh tyre starts: off the blankets. The series heats its
    /// tyres to 70 C before they go on the car, so a car leaving the grid or
    /// the pit lane is a few degrees short of its window, not a whole
    /// ambient day short of it. A rule of the series, not of a compound.
    /// </summary>
    public const float BlanketTempC = 70f;

    /// <summary>
    /// The share of its peak grip a tyre holds in a steady corner, as the
    /// physics has it (the slip curve's own figure): what a driver can lean
    /// on without the tyre letting go.
    /// </summary>
    public const float SustainablePeakShare = TireSlipCurve.SustainablePeakShare;

    /// <summary>The slip angle at which a tyre's grip peaks; past it the tyre is sliding.</summary>
    public const float PeakSlipAngleRadians = TireSlipCurve.PeakSlipAngleRadians;

    /// <summary>The slip angle at which a tyre gives <paramref name="share"/> of its peak grip, on the way up to it.</summary>
    public static float SlipAngleAtShare(float share) => TireSlipCurve.InverseEvaluate(share);

    private const float Epsilon = 1e-5f;
    private const float MinimumTireHeatLoadScale = 0.2f;
    private const float MaximumThermalOverLimit = 1f;
    private const float MinimumTemperatureGripFactor = 0.55f;
    private const float MaximumTemperatureGripFactor = 1.08f;
    private const float MinimumWearGripFactor = 0.45f;

    /// <param name="compound">The compound's name, as the car and the screen know it.</param>
    public Tire(TireConfig config, string compound)
    {
        Config = config ?? throw new ArgumentNullException(nameof(config));
        Window = new TireWindow(
            compound, config.IdealSurfaceTempLowC, config.IdealSurfaceTempHighC, config.CoreOverheatTempC);
    }

    public TireConfig Config { get; }
    public TireWindow Window { get; }

    /// <summary>A new tyre on this corner, off the blankets.</summary>
    public void Reset(TireState tire) => tire.Reset(BlanketTempC, BlanketTempC);

    /// <summary>
    /// The most force a tyre can give at this load: the load times its
    /// friction, and the friction falls as the load rises (load
    /// sensitivity). A tyre pressed twice as hard grips less than twice as
    /// hard, which is why a car's cornering grows more slowly than its
    /// downforce, and why moving load from the inside wheels to the
    /// outside ones in a corner costs the axle grip.
    ///
    /// Measured from the load the tyre was sized for: what its own wheel
    /// carries with the car standing. The axle carrying more of the car
    /// runs wider tyres, as the rear of a racing car does, so standing
    /// still front and rear grip in proportion to the weight each holds
    /// and the car is balanced; with one size all round the heavier axle
    /// would be the weaker one in every corner, and a car with its weight
    /// behind the middle would oversteer at the limit and spin.
    /// </summary>
    public float GripForce(TireState tire, float loadN, float sizedLoadN)
    {
        float ratio = MathF.Max(
            loadN / MathF.Max(sizedLoadN, Epsilon),
            MinimumLoadSensitivityRatio
        );
        return loadN * CalculateTireMu(Config, tire) *
               MathF.Pow(ratio, -Math.Max(0f, Config.LoadSensitivity));
    }

    /// <summary>
    /// The force a tyre can give at its load, against what it gives at the
    /// load it was sized for: the load scale with load sensitivity taken
    /// off (WheelGripForce). What a tyre's work is counted against.
    /// </summary>
    public float ForceScale(TireState tire, float sizedLoadN)
    {
        float scale = WorkLoadScale(tire, sizedLoadN);
        return MathF.Pow(
            MathF.Max(scale, MinimumLoadSensitivityRatio),
            1f - Math.Clamp(Config.LoadSensitivity, 0f, 1f)
        );
    }

    private static float WorkLoadScale(TireState tire, float sizedLoadN)
    {
        // Against the load the tyre was sized for (WheelGripForce), so the
        // wider tyres of the heavier axle are not charged for carrying the
        // weight they were chosen to carry. Chassis mass on purpose, not
        // what the car weighs at this moment: a reference that fell with
        // the fuel load would cancel the very effect it is there to show,
        // a car that has burned half its fuel pressing its tyres less hard.
        return Math.Max(
            MinimumTireHeatLoadScale,
            tire.LoadN / Math.Max(sizedLoadN, Epsilon)
        );
    }

    public void Update(TireState tire, in TireWork work, float dt)
    {
        float lateralUse = work.LateralUse;
        float lateralSlipWork = work.LateralSlipWork;
        float longitudinalUse = work.LongitudinalUse;
        float brakeUse = work.BrakeUse;
        float directionalHeatDemandUse = work.CombinedDemandUse;
        float lateralHeatScale = work.LateralHeatScale;
        float lateralWearScale = work.LateralWearScale;
        float overLimit = work.OverLimit;
        float airTempC = work.AirTempC;
        float trackTempC = work.TrackTempC;
        float speed = work.Speed;
        float coolingAirSpeed = work.CoolingAirSpeed;
        float wakeDownforceLoss = work.WakeDownforceLoss;
        float loadScale = WorkLoadScale(tire, work.SizedLoadN);
        // Work is force times sliding, and with friction falling with load
        // the force grows more slowly than the load does; heat and wear
        // from work go with the force. Rolling heat is the carcass flexing
        // under the load itself, and stays with the load.
        float forceScale = ForceScale(tire, work.SizedLoadN);
        float thermalOverLimit = Math.Min(overLimit, MaximumThermalOverLimit);
        float normalizedLateralUse = Math.Clamp(Math.Abs(lateralUse), 0f, 1f);
        float normalizedLongitudinalUse = Math.Clamp(
            Math.Abs(longitudinalUse),
            0f,
            1f
        );
        float normalizedBrakeUse = Math.Clamp(brakeUse, 0f, 1f);
        float tireWorkSpeedMultiplier = Math.Clamp(
            Math.Max(0f, speed) /
            TireConfig.TireWorkReferenceSpeedMps,
            0f,
            TireConfig.MaximumTireWorkSpeedMultiplier
        );
        float longitudinalHeatUse = MathF.Pow(
            normalizedLongitudinalUse,
            TireConfig.LongitudinalHeatExponent
        );
        float combinedUse = Math.Clamp(
            MathF.Sqrt(
                normalizedLateralUse * normalizedLateralUse +
                normalizedLongitudinalUse * normalizedLongitudinalUse
            ),
            0f,
            1f
        );
        float rollingHeatSpeedFactor = Math.Max(0f, speed) /
                                       (
                                           Math.Max(0f, speed) +
                                           TireConfig.RollingHeatReferenceSpeedMps
                                       );
        float rollingSurfaceHeat = TireConfig.RollingSurfaceHeatRate *
                                   loadScale * rollingHeatSpeedFactor;

        // Lateral tread heat is the rubber's own dissipation: the force it
        // is carrying times how fast it is being dragged sideways. The
        // constant is solved, not chosen. Below the peak the tyre curve is
        // linear, so force share is the cornering stiffness times the slip
        // angle, and force x slip and force-squared are the same function
        // of each other up to exactly this factor. Matching there is what
        // keeps every tyre temperature ever calibrated on this car -- the
        // same discipline the wear half was done under.
        //
        // Above the peak the two part company, which is the point. Force
        // falls away while the dragging keeps growing, so heat goes on
        // rising where a use-squared reading would have it fall. That is
        // the near-limit heating that used to need its own hand-placed
        // branch, and it arrives here for free and continuous.
        // Capped like the wear: a tyre dragged sideways in a spin is not
        // heated without end -- the tread smokes and the sliding surface
        // stops taking more -- and sliding heat that was allowed to run
        // with the slip angle all the way round put a spun car's fronts a
        // hundred and fifty degrees up in twelve seconds.
        float lateralSlipHeat =
            TireConfig.LateralHeatRate * SlipHeatCalibration *
            MathF.Min(Math.Max(0f, lateralSlipWork), MaximumSlipWorkHeat) * lateralHeatScale;
        float directionalHeat =
            TireConfig.LongitudinalHeatRate * longitudinalHeatUse;
        // Force demand is not the same thing as rubber dissipation. At modest
        // utilization most of the contact patch still adheres and relatively
        // little of the work becomes tread heat; local micro-slip grows quickly
        // as the friction circle fills. Keep this continuous and based on the
        // actual combined vehicle demand, rather than on the named strategy mode.
        float directionalHeatProgress = Math.Clamp(
            (directionalHeatDemandUse - TireConfig.DirectionalHeatRampStartUse) /
            Math.Max(
                TireConfig.DirectionalHeatRampEndUse -
                TireConfig.DirectionalHeatRampStartUse,
                Epsilon
            ),
            0f,
            1f
        );
        float directionalHeatSmoothStep =
            directionalHeatProgress * directionalHeatProgress *
            (3f - 2f * directionalHeatProgress);
        directionalHeat *=
            TireConfig.MinimumDirectionalHeatScale +
            (1f - TireConfig.MinimumDirectionalHeatScale) *
            directionalHeatSmoothStep;
        float wakeCorneringHeat = TireConfig.WakeCorneringHeatRate *
                                   Math.Clamp(
                                       wakeDownforceLoss,
                                       0f,
                                       0.1f
                                   ) *
                                   normalizedLateralUse *
                                   normalizedLateralUse *
                                   lateralHeatScale *
                                   tireWorkSpeedMultiplier;
        // There used to be one more term here: the rear axle's slip angle,
        // squared, as a rear-only heater. It dates from before lateral heat
        // was force times sliding, when nothing else charged a rear tyre for
        // being dragged sideways. Once lateralSlipHeat did that for all four
        // tyres the two were billing the same sliding twice, and the shorter
        // thermal time constant turned the duplicate into rear-tyre peaks
        // the car spun on. The rear's sliding is in lateralSlipWork; it does
        // not get a second heater of its own.
        // Partial sliding as the car nears what it can hold; see
        // TireConfig.NearLimitHeatRate. Past what it can hold the patch is
        // no longer partly sliding but sliding, which the slip-work term
        // already charges, so this one hands over: gone by the peak.
        float startShare = TireConfig.NearLimitHeatStartShare;
        float holdable = combinedUse / TireConfig.NearLimitHeatLimitShare;
        float rising = Math.Clamp((holdable - startShare) / (1f - startShare), 0f, 1f);
        float handedOver = Math.Clamp(
            (holdable - 1f) / (1f / TireConfig.NearLimitHeatLimitShare - 1f),
            0f,
            1f
        );
        float nearLimitHeat = TireConfig.NearLimitHeatRate *
                              rising * rising * (1f - handedOver);
        float surfaceHeat =
            tireWorkSpeedMultiplier *
            (directionalHeat + lateralSlipHeat + nearLimitHeat) +
            TireConfig.OverLimitHeatRate * thermalOverLimit * thermalOverLimit +
            wakeCorneringHeat;
        surfaceHeat *= forceScale;
        surfaceHeat += rollingSurfaceHeat;
        surfaceHeat *= Config.HeatScale;

        float airCoolingMultiplier = CalculateAirCoolingMultiplier(
            coolingAirSpeed
        );
        float surfaceToAir = TireConfig.SurfaceCoolingRate * airCoolingMultiplier * (tire.SurfaceTempC - airTempC);
        float surfaceToTrack = TireConfig.TrackSurfaceTransferRate *
                               (tire.SurfaceTempC - trackTempC);
        float surfaceToCore = TireConfig.SurfaceCoreTransferRate * (tire.SurfaceTempC - tire.CoreTempC);
        float rollingCoreHeat = TireConfig.RollingCoreHeatRate *
                                loadScale * rollingHeatSpeedFactor * Config.HeatScale;
        float brakeCoreHeat = TireConfig.BrakeCoreHeatRate *
                              normalizedBrakeUse *
                              tireWorkSpeedMultiplier *
                              forceScale * Config.HeatScale;

        tire.SurfaceTempC += (
            surfaceHeat - surfaceToAir - surfaceToTrack - surfaceToCore
        ) / TireConfig.ThermalTimeScale * dt;
        // The carcass makes its own heat by flexing, so without somewhere to put
        // it the only way out is backwards through the tread, and it has to
        // stand hotter than the tread for that to happen - permanently, by an
        // amount that the longitudinal or lateral command does not directly
        // control. It does have somewhere to
        // put it: the rim, which is metal and has air moving over it, and the
        // gas inside the tyre. Both are stood in for here by the outside air.
        //
        // Far slower than the tread, which is lying on the road inside its own
        // hurricane. On its own this path takes something like twenty minutes,
        // against the tread's forty seconds, which is what makes a soaked
        // carcass something a stint has to live with rather than something a
        // straight fixes.
        float coreToAir = TireConfig.CoreAirCoolingRate *
                          airCoolingMultiplier *
                          (tire.CoreTempC - airTempC);
        tire.CoreTempC += (
            rollingCoreHeat + brakeCoreHeat + surfaceToCore - coreToAir
        ) / (TireConfig.CoreHeatCapacityRatio * TireConfig.ThermalTimeScale) * dt;

        float tempWearFactor = CalculateTemperatureWearFactor(
            Config,
            tire.SurfaceTempC
        );
        // Sideways, a tyre spends rubber as it spends heat: force times how
        // fast the contact patch is dragged across the road, which is the
        // slip work the axle measured from its slip angle. Below the peak
        // that rises far faster than the force does, so the last few
        // percent of a corner are where a set of tyres goes.
        float lateralWear =
            TireConfig.SlipWorkWearRate * MathF.Min(Math.Max(0f, lateralSlipWork), MaximumSlipWorkWear) * lateralWearScale;
        float directionalWear =
            lateralWear +
            TireConfig.LongitudinalWearRate * longitudinalUse * longitudinalUse;
        float partialSlipWear = TireConfig.NearLimitWearRate *
                                MathF.Pow(
                                    combinedUse,
                                    TireConfig.NearLimitWearExponent
                                );
        // The rear used to carry one more term here, its slip angle squared
        // as rear-only wear -- the twin of the rear heater removed from the
        // heat sum above, and from the same era. The rear's scrub is already
        // in lateralUse through ScrubWeight, and driving harder is still
        // charged steeply by the near-limit power and the over-limit and
        // hot-tread terms; the proxy saturated at the peak and so never
        // priced anything past it.
        float tireWorkWear =
            directionalWear + partialSlipWear +
            TireConfig.OverLimitWearRate * thermalOverLimit * thermalOverLimit;
        // Sliding work per second goes with the speed the road passes
        // under the tyre, with no ceiling: a metre of corner costs the same
        // rubber however fast it is taken at the same force, so a slower lap
        // is not charged more for the time it takes.
        float wearSpeed = Math.Max(0f, speed) / TireConfig.TireWorkReferenceSpeedMps;
        float wearDelta = tireWorkWear * wearSpeed *
                          tempWearFactor * forceScale * dt * Config.WearScale *
                          MathF.Max(0f, tire.SurfaceAbrasion);

        tire.Wear = Math.Clamp(tire.Wear + wearDelta, 0f, 1f);
    }

    /// <summary>
    /// The force share the calibration is matched at: hard cornering, but
    /// not at the limit. This is where a racing car actually spends its
    /// cornering seconds, and matching there is what "the same below the
    /// peak" has to mean.
    /// </summary>
    private const float SlipHeatReferenceUse = 0.85f;

    /// <summary>
    /// The factor that makes force-times-sliding agree with the
    /// force-squared reading it replaces, at everyday sub-limit use.
    ///
    /// Solved, not fitted: set <c>use^2 = k x use x sin(alpha(use)) /
    /// sin(peak)</c> at the reference share and the constant falls out as
    /// <c>ref x sin(peak) / sin(alpha(ref))</c>, with the slip angle read
    /// off the tyre curve itself.
    ///
    /// Two corrections got it here, both found by measuring against the
    /// old model rather than by reading the algebra twice.
    ///
    /// The first version solved at the tangent at zero slip, on the
    /// reasoning that the curve is linear down there. It is, but the car
    /// is not: over a five-minute mixed-driving cycle that ran 1.4 C hot
    /// at 60% use and 4.8 C hot at 95%, because the curve has already left
    /// its tangent by 14% at 60%. Matching a model at an operating point
    /// nothing operates at is not matching it.
    ///
    /// The second is <see cref="TireConfig.MinimumDirectionalHeatScale"/>.
    /// The old lateral term did not stand alone — it went through the
    /// ramp that says most of the contact patch still adheres at modest
    /// use, and below <see cref="TireConfig.DirectionalHeatRampStartUse"/>
    /// that ramp is not a ramp at all but a flat multiplier of a fifth.
    /// The reference share sits below the ramp start, so the thing being
    /// matched is a fifth of what the term appeared to say, and leaving it
    /// out made this five times too hot. The lateral term does not go
    /// through that ramp any more — expressing "little of the work becomes
    /// heat at modest use" is exactly what slip power does natively — so
    /// the factor belongs here, in the constant, where it can be seen.
    ///
    /// Above the reference the two still part company, and that is the
    /// point of the change: force falls away past the peak while the
    /// dragging keeps growing, so heat goes on rising where a use-squared
    /// reading would have it fall. That is the near-limit heating that
    /// used to need its own hand-placed branch, and it arrives here for
    /// free and continuous.
    /// </summary>
    private static readonly float SlipHeatCalibration =
        SlipHeatReferenceUse *
        TireConfig.MinimumDirectionalHeatScale *
        MathF.Sin(TireSlipCurve.PeakSlipAngleRadians) /
        MathF.Max(
            MathF.Sin(TireSlipCurve.InverseEvaluate(SlipHeatReferenceUse)),
            Epsilon
        );

    /// <summary>A spin bills a set of tyres for a spin, not for the race: slip work past one and a half peaks is not charged.</summary>
    private const float MaximumSlipWorkWear = 1.5f;

    /// <summary>Slip work past two and a half peaks heats the tread no further.</summary>
    private const float MaximumSlipWorkHeat = 2.5f;

    private static float CalculateTemperatureWearFactor(
        TireConfig tires,
        float surfaceTempC
    )
    {
        float idealLow = Math.Min(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        float idealHigh = Math.Max(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        float coldDistance = Math.Max(0f, idealLow - surfaceTempC);
        // Warm rubber is soft rubber: it abrades faster the hotter the tread
        // runs, inside the window as well as past it, and steeply. A few
        // degrees of tread is what separates a tyre being looked after from
        // one being driven on, which is what makes saving a tyre cheap and
        // pushing one dear.
        float warm = Math.Max(0f, surfaceTempC - TireConfig.WearReferenceTempC);
        return 1f +
               TireConfig.ColdWearPerCSquared *
               coldDistance * coldDistance +
               MathF.Min(MathF.Exp(warm / TireConfig.WearTempScaleC) - 1f,
                   TireConfig.MaximumHotWearFactor - 1f);
    }

    private static float CalculateAirCoolingMultiplier(float speed)
    {
        // Still air cools the tyre at the base rate; past the speed where
        // the flow outdoes it, forced convection takes over.
        return MathF.Max(
            1f,
            TireConfig.SpeedCoolingAtReference * MathF.Pow(
                Math.Max(0f, speed) / TireConfig.SpeedCoolingReferenceMps,
                TireConfig.SpeedCoolingExponent
            )
        );
    }

    /// <summary>
    /// The lightest load, as a share of the one its tyre was sized for, whose friction still
    /// rises as it lightens. A wheel all but off the ground gives next to
    /// nothing anyway; without a floor its friction would run to infinity.
    /// </summary>
    private const float MinimumLoadSensitivityRatio = 0.2f;

    private static float CalculateTireMu(TireConfig tires, TireState tire)
    {
        float tempGrip = 1f;
        float idealLow = Math.Min(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        float idealHigh = Math.Max(
            tires.IdealSurfaceTempLowC,
            tires.IdealSurfaceTempHighC
        );
        if (tire.SurfaceTempC < idealLow)
        {
            float coldDistance = idealLow - tire.SurfaceTempC;
            tempGrip -= coldDistance * coldDistance *
                        tires.ColdGripLossPerCSquared;
        }
        else if (tire.SurfaceTempC > idealHigh)
        {
            float hotDistance = tire.SurfaceTempC - idealHigh;
            tempGrip -= hotDistance * hotDistance *
                        tires.HotGripLossPerCSquared;
        }

        tempGrip -= Math.Max(0f, tire.CoreTempC - tires.CoreOverheatTempC) * TireConfig.CoreOverheatGripLossPerC;

        float wear = Math.Clamp(tire.Wear, 0f, 1f);
        float cliffStart = TireConfig.WearCliffStart;
        float cliffProgress = Math.Clamp(
            (wear - cliffStart) / Math.Max(1f - cliffStart, Epsilon),
            0f,
            1f
        );
        float smoothCliff = cliffProgress * cliffProgress *
                            (3f - 2f * cliffProgress);
        float wearGrip = 1f -
                         TireConfig.WearLinearGripLoss * wear -
                         Math.Max(0f, tires.WearCliffGripLoss) * smoothCliff;
        return tires.BaseMu *
               Math.Clamp(tempGrip, MinimumTemperatureGripFactor, MaximumTemperatureGripFactor) *
               Math.Clamp(wearGrip, MinimumWearGripFactor, 1f);
    }

}
