using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// How much of an axle's grip a tyre actually delivers sideways at a given
/// slip angle.
///
/// This is the part of the car that used to be missing. The model before
/// this one had no slip angle at all: the driver asked for a cornering
/// force, the tyre handed it over up to a circle, and the body's attitude
/// was a separate bookkeeping variable held inside a ten degree clamp. A
/// car like that cannot understeer, cannot oversteer, and cannot be caught
/// - which is why the clamp was load bearing, and why removing it made the
/// old model diverge.
///
/// Here the force is a function of the angle between where a wheel points
/// and where it is going, and everything else follows from that: turn in
/// too hard and the front runs past its peak and the car washes wide; ask
/// the rear for more than it has and the tail steps out and keeps stepping;
/// point the wheels back down the road and the front bites again.
///
/// The shape is the standard one, with the peak normalised to one:
///
///     f(a) = sin(C * atan(B * a))
///
/// Two numbers are quoted and the other two are solved from them, which is
/// the right way round: <see cref="PeakSlipAngleRadians"/> is where a tyre
/// gives its most, and <see cref="TailForceShare"/> is what is left of that
/// once it is well past caring. Cornering stiffness is then whatever those
/// two imply rather than a third free parameter, and the peak being exactly
/// one means the car's cornering envelope is still the one the chassis was
/// calibrated to - three and a half g, unchanged by any of this.
/// </summary>
internal static class TireSlipCurve
{
    /// <summary>
    /// Where a tyre gives its most. Eight degrees is a racing slick's
    /// figure; a road tyre peaks nearer five and a wet one lower still, so
    /// this is the first thing a future compound model would move.
    /// </summary>
    public const float PeakSlipAngleRadians = 0.13962634f;

    /// <summary>
    /// What is left, as a share of the peak, a long way past it. Seven
    /// tenths is what makes a slide cost something without making it
    /// unrecoverable: a car well past the peak is still generating most of
    /// its grip, so it can be caught, but never as much as one on the peak,
    /// so sliding is always slower than not sliding.
    /// </summary>
    public const float TailForceShare = 0.7f;

    /// <summary>
    /// Shape factor, solved from the tail: sin(C * pi/2) = tail.
    /// </summary>
    public static readonly float Shape =
        2f - 2f / MathF.PI * MathF.Asin(TailForceShare);

    /// <summary>
    /// Stiffness factor, solved from the peak: the curve turns over where
    /// C * atan(B * a) reaches pi/2.
    /// </summary>
    public static readonly float Stiffness =
        MathF.Tan(MathF.PI / (2f * Shape)) / PeakSlipAngleRadians;

    /// <summary>
    /// How much of the instantaneous peak a car can actually hold.
    ///
    /// A peak is a peak: sit exactly on it and the next disturbance takes
    /// force away rather than adding it, so nothing holds both ends of a
    /// car there at once. The model this replaced could - it had no peak to
    /// fall off - which is why the car's advertised cornering limit used to
    /// be the whole friction circle and could be planned against to the
    /// last percent.
    ///
    /// Measured on the constant-speed skidpad in the dynamics tests: the
    /// car settles at a little under nine tenths of the grip its axles
    /// have. Anything planning a corner speed has to plan against this
    /// number and not the circle, or it arrives at every apex asking for a
    /// tenth more than the tyres will give and running wide by exactly
    /// that.
    /// </summary>
    public const float SustainablePeakShare = 0.9f;

    /// <summary>
    /// Cornering stiffness at zero slip, as a share of the axle's grip per
    /// radian. Not used by the model - it falls out of the two numbers
    /// above - but it is the figure a chassis engineer would ask for, so it
    /// is worth being able to read.
    /// </summary>
    public static float NormalizedCorneringStiffness => Stiffness * Shape;

    /// <summary>
    /// Signed share of the axle's lateral capacity delivered at this slip
    /// angle: one at the peak, falling towards
    /// <see cref="TailForceShare"/> beyond it, odd about zero.
    ///
    /// <paramref name="peakScale"/> moves where this axle's peak sits
    /// relative to the quoted one, which is how the two ends of the car are
    /// given different characters without either of them being given more
    /// grip than it has.
    /// </summary>
    public static float Evaluate(float slipAngleRadians, float peakScale = 1f)
    {
        float scale = MathF.Max(peakScale, 0.05f);
        return MathF.Sin(
            Shape * MathF.Atan(Stiffness * slipAngleRadians / scale)
        );
    }

    /// <summary>
    /// The slip angle that would deliver this share of an axle's lateral
    /// capacity - the curve read backwards.
    ///
    /// A driver knows their car: asked for a corner, they do not turn the
    /// wheel to the angle geometry alone would need and then wait to find
    /// out how much the tyres gave away. They wind on the extra the car
    /// takes, and this is that extra. Undefined at and past the peak, where
    /// no angle delivers more, so the share is held just under it and the
    /// rest is left to the tyres to refuse - which is exactly what
    /// understeer at the limit is.
    /// </summary>
    public static float InverseEvaluate(float forceShare, float peakScale = 1f)
    {
        float scale = MathF.Max(peakScale, 0.05f);
        float share = Math.Clamp(forceShare, -0.995f, 0.995f);
        return scale * MathF.Tan(MathF.Asin(share) / Shape) / Stiffness;
    }

    /// <summary>
    /// How far past its peak this slip angle is, as a fraction of the peak,
    /// capped at one. This is what a driver feels as the axle letting go,
    /// and what the tyre is charged extra scrub for.
    /// </summary>
    public static float PastPeak(float slipAngleRadians, float peakScale = 1f)
    {
        float scale = MathF.Max(peakScale, 0.05f);
        return Math.Clamp(
            MathF.Abs(slipAngleRadians) / (PeakSlipAngleRadians * scale) - 1f,
            0f,
            1f
        );
    }
}
