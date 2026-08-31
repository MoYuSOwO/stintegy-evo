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

    /// <summary>
    /// How much of the grip it has, this compound is asked to give at each
    /// rung of the tyre ladder.
    ///
    /// On the tyre because it is the tyre's number. It used to live on the
    /// speed planner's configuration, which quietly made it a property of one
    /// particular driver: every car in the field shared one set of figures, a
    /// compound that punished over-driving had no way to say so, and the
    /// learned driver had to carry a copy of the analytic planner's settings
    /// just to find out what its own pit wall had asked it for.
    ///
    /// What the rungs are called stays the game's business - every car that
    /// races has tyres, and Protect means the same thing on all of them. What
    /// each rung costs is the rubber's.
    /// </summary>
    public float ProtectAccelerationUsage { get; init; } = 0.955f;
    public float LightAccelerationUsage { get; init; } = 0.966f;
    public float NormalAccelerationUsage { get; init; } = 0.977f;
    public float PushAccelerationUsage { get; init; } = 0.9885f;
    public float AttackAccelerationUsage { get; init; } = 1f;

    /// <summary>
    /// Checks the ladder is a ladder: five settings that ask for more grip as
    /// they climb, each of them a share of what the tyre has.
    ///
    /// Worth checking now that compounds carry their own figures. A ladder
    /// whose Push asked for less than its Normal would make the strategy
    /// table lie to whoever read it - the pit wall would call for more and
    /// the car would give less - and nothing downstream would notice, because
    /// every number involved is a plausible fraction.
    /// </summary>
    public void ValidateAccelerationLadder()
    {
        ValidateAccelerationUsage(ProtectAccelerationUsage);
        ValidateAccelerationUsage(LightAccelerationUsage);
        ValidateAccelerationUsage(NormalAccelerationUsage);
        ValidateAccelerationUsage(PushAccelerationUsage);
        ValidateAccelerationUsage(AttackAccelerationUsage);

        if (!(ProtectAccelerationUsage < LightAccelerationUsage &&
              LightAccelerationUsage < NormalAccelerationUsage &&
              NormalAccelerationUsage < PushAccelerationUsage &&
              PushAccelerationUsage < AttackAccelerationUsage))
        {
            throw new ArgumentOutOfRangeException(
                nameof(ProtectAccelerationUsage),
                "Tire-mode acceleration usages must increase from Protect to Attack."
            );
        }
    }

    private static void ValidateAccelerationUsage(float usage)
    {
        if (!float.IsFinite(usage) || usage <= 0f || usage > 1f)
        {
            throw new ArgumentOutOfRangeException(
                nameof(usage),
                "Acceleration usage must be finite and in (0, 1]."
            );
        }
    }

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
