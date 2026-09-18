using System;

namespace StintegyEVO.TrainingHost.Environment;

/// <summary>
/// The edges of the drawn range, and where each edge comes from.
/// </summary>
public static class EpisodeStartLimits
{
    /// <summary>
    /// How much of the time the draw stays in the band a race actually
    /// spends its life in. The rest is spread thin over everything the car
    /// can physically be in — the point is not to make the policy good at
    /// driving a ruined tyre, it is to stop a ruined tyre being outside
    /// the distribution when one turns up.
    /// </summary>
    public const float DefaultRaceNormalShare = 0.70f;

    /// <summary>
    /// The top of the wear range, and why it is one rather than something
    /// more cautious.
    ///
    /// The bound has to be the model's own, not the current exam's. The
    /// wear-grip curve is defined across the whole of [0, 1]: a linear
    /// tenth of grip lost across the range, plus a cliff that starts at
    /// seventy per cent and completes exactly at one. There is no region
    /// up there the model declines to have an opinion about, so there is
    /// none to pretend to cover.
    ///
    /// Being straight about what that means: the cliff is design rather
    /// than measurement. Nobody has held this car against a real worn
    /// tyre. What can be said is that the range is defined, monotone and
    /// finished at its end, which is the property that matters for drawing
    /// from it.
    /// </summary>
    public const float MaxCalibratedWear = 1.0f;

    /// <summary>
    /// The top of the temperature range, and this one is derived rather
    /// than chosen.
    ///
    /// Grip falls off above the ideal window as the square of how far past
    /// it the rubber is, and the result is clamped at a floor. Those two
    /// numbers decide where the curve stops saying anything new:
    /// 0.00070 per degree squared against a clamp 0.45 below one puts the
    /// last informative temperature at 115 + sqrt(0.45 / 0.00070), which
    /// is 140.3 C. Above that every temperature is the same car, so the
    /// range ends there.
    /// </summary>
    public const float MaxCalibratedTempC = 140f;

    /// <summary>
    /// The bottom, which is simply the air. A tyre cannot start colder
    /// than what is around it, and it must be allowed to start that cold:
    /// an out lap, a restart and the first corner after a safety car are
    /// all cold, and cold rubber slides for entirely different reasons
    /// than worn rubber does.
    ///
    /// This end of the range is only honest because the car can now do
    /// something about it. Before the thermal time constant was corrected,
    /// a cold start meant fifteen laps below the working range, which is
    /// not a hard case but a broken one.
    /// </summary>
    public const float AmbientTempC = 25f;

    /// <summary>
    /// What a circuit's surface is worth on a bad day and a good one.
    ///
    /// Green tarmac on a Friday morning, or one dusted over by a support
    /// race, gives away five to ten per cent; by the end of a weekend the
    /// racing line is rubbered in and gives a couple back. Nothing about
    /// the geometry moves — only what the road is worth.
    ///
    /// This is the distribution premise for driving by feel. A policy that
    /// has only ever driven one grip level cannot be said to be sensing
    /// grip; it has memorised a circuit, and the friction-circle and slip
    /// channels it is given are decoration. Varying it is what makes those
    /// channels load-bearing — and it is a free rehearsal for the dynamic
    /// grip era, since it enters through exactly the layer that era will
    /// use.
    ///
    /// Rain is not in here. Water does not exist in this world yet, and a
    /// dry circuit at 0.9 is not a wet one — teaching that it is would be
    /// worse than teaching nothing.
    /// </summary>
    public const float MinSurfaceGrip = 0.90f;
    public const float MaxSurfaceGrip = 1.05f;
}

/// <summary>
/// What state a car is in when an episode begins.
///
/// Every episode used to start the same way: tyres brand new at ninety
/// degrees, eighty per cent charge. A policy trained only on that has
/// never met a car that is running out of anything, and the two-arm
/// certification found exactly what that costs — fifty-seven spins across
/// two drivers who shared no weights, no entropy coefficient and no
/// ancestry, and not one of them below twenty-five per cent tyre wear.
/// Training episodes ran 240 seconds and never got there; certification
/// ran 600 and did.
///
/// The fix is coverage rather than longer episodes. A longer episode walks
/// the state space in one direction at whatever rate the consumables
/// happen to move, and pays for the walk in sample correlation. Drawing
/// the start instead hits a different point every episode and decouples
/// "what do I want it to have seen" from "how long must it drive to see
/// it".
///
/// What this teaches and what it does not is worth being explicit about:
/// it teaches driving a car in a given condition, not managing a car
/// towards one. Managing is the budget re-core's product, and when that
/// arrives the training distribution will need both kinds of episode —
/// drawn starts and full-arc long ones. Filed here so it is not
/// rediscovered.
/// </summary>
public sealed record EpisodeStartDistribution(
    float RaceNormalShare = EpisodeStartLimits.DefaultRaceNormalShare,
    float NormalWearMax = 0.40f,
    float NormalTempMinC = 80f,
    float NormalTempMaxC = 120f,
    float NormalChargeMin = 0.35f,
    float ExtremeWearMax = EpisodeStartLimits.MaxCalibratedWear,
    float ExtremeTempMinC = EpisodeStartLimits.AmbientTempC,
    float ExtremeTempMaxC = EpisodeStartLimits.MaxCalibratedTempC,
    float ExtremeChargeMin = 0.02f,
    float NormalGripMin = 0.97f,
    float NormalGripMax = 1.03f,
    float ExtremeGripMin = EpisodeStartLimits.MinSurfaceGrip,
    float ExtremeGripMax = EpisodeStartLimits.MaxSurfaceGrip,
    float AirTempMinC = 15f,
    float AirTempMaxC = 40f,
    float TrackTempMinC = 15f,
    float TrackTempMaxC = 50f
)
{
    public EpisodeStart Draw(
        float uniformBand,
        float wear,
        float temp,
        float charge,
        float grip = 0.5f,
        float airTemp = 0.4f,
        float trackTemp = 0.57f
    )
    {
        bool normal = uniformBand < RaceNormalShare;
        float wearMax = normal ? NormalWearMax : ExtremeWearMax;
        float tempMin = normal ? NormalTempMinC : ExtremeTempMinC;
        float tempMax = normal ? NormalTempMaxC : ExtremeTempMaxC;
        float chargeMin = normal ? NormalChargeMin : ExtremeChargeMin;

        float gripMin = normal ? NormalGripMin : ExtremeGripMin;
        float gripMax = normal ? NormalGripMax : ExtremeGripMax;

        return new EpisodeStart(
            Wear: Math.Clamp(wear, 0f, 1f) * wearMax,
            SurfaceTempC: Lerp(tempMin, tempMax, temp),
            CoreTempC: Lerp(tempMin, tempMax, temp),
            Charge: Lerp(chargeMin, 1f, charge),
            SurfaceGripScalar: Lerp(gripMin, gripMax, grip),
            AirTempC: Lerp(AirTempMinC, AirTempMaxC, airTemp),
            TrackTempC: Lerp(TrackTempMinC, TrackTempMaxC, trackTemp)
        );
    }

    private static float Lerp(float from, float to, float weight) =>
        from + (to - from) * Math.Clamp(weight, 0f, 1f);
}

/// <summary>One drawn starting condition.</summary>
public readonly record struct EpisodeStart(
    float Wear,
    float SurfaceTempC,
    float CoreTempC,
    float Charge,
    float SurfaceGripScalar = 1f,
    float AirTempC = 25f,
    float TrackTempC = 35f
);
