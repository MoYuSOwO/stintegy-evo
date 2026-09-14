using System;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The reflex reads a tyre's share of its curve, and the curve turns over.
///
/// Past the peak slip angle the share falls as the slide deepens, so a
/// ceiling built on the share alone rises again exactly when the tyre is
/// letting go: at 1.8 times the peak it handed back 74% of the pedal, at
/// four times 92% -- more than it allowed a tyre gripping at half its peak
/// angle. parent2h's last three spins were slow-corner exits with the rear
/// at 1.8 to 1.9 times its peak and the throttle wide open. These pin the
/// repair: past the peak the throttle stays shut above walking pace, the
/// brakes and everything below the peak are left alone, and the ceiling has
/// no step at the peak on any rung that trims.
/// </summary>
public sealed class ReflexPastPeakTests
{
    private const float Grip = 10f;
    private static readonly float Normal =
        new TireConfig().GetAccelerationUsage(TireUsageMode.Normal);

    [Theory]
    [InlineData(1.5f)]
    [InlineData(1.8f)]
    [InlineData(2.5f)]
    [InlineData(4.0f)]
    public void PastThePeakTheThrottleStaysShut(float multipleOfPeak)
    {
        float slipAngle = TireSlipCurve.PeakSlipAngleRadians * multipleOfPeak;

        Assert.Equal(
            0f,
            CarPhysics.ReflexGovernedLongitudinal(
                Grip, Grip, slipAngle, 1f, Normal, speed: 35f
            )
        );
        Assert.Equal(
            0f,
            CarPhysics.ReflexGovernedLongitudinal(
                Grip, Grip, -slipAngle, 1f, Normal, speed: 35f
            )
        );
    }

    /// <summary>
    /// The brakes are left as they were on the far side of the peak.
    /// Slowing down is the recovery a sliding car always keeps -- anti-lock
    /// exists so a tyre can slide and still stop -- and taking it away would
    /// be a physical regression rather than a discipline.
    /// </summary>
    [Theory]
    [InlineData(1.8f)]
    [InlineData(4.0f)]
    public void PastThePeakTheBrakesAreLeftAsTheyWere(float multipleOfPeak)
    {
        float slipAngle = TireSlipCurve.PeakSlipAngleRadians * multipleOfPeak;
        float share = MathF.Abs(TireSlipCurve.Evaluate(slipAngle));
        float expected = -MathF.Sqrt(
            MathF.Max(0f, (Normal * Normal - share * share) / (1f - share * share))
        ) * Grip;

        Assert.Equal(
            expected,
            CarPhysics.ReflexGovernedLongitudinal(
                -Grip, Grip, slipAngle, 1f, Normal, speed: 35f
            ),
            4
        );
    }

    /// <summary>
    /// Below 10 m/s the slip angle is a kinematic estimate over a 3 m/s
    /// floor, and a car pulling off the grid or away from a wall reads past
    /// its peak while gripping. The throttle is not shut on that reading.
    /// </summary>
    [Fact]
    public void AtWalkingPaceAPastPeakReadingDoesNotShutTheThrottle()
    {
        float slipAngle = TireSlipCurve.PeakSlipAngleRadians * 1.8f;
        float share = MathF.Abs(TireSlipCurve.Evaluate(slipAngle));
        float expected = MathF.Sqrt(
            (Normal * Normal - share * share) / (1f - share * share)
        ) * Grip;

        Assert.Equal(
            expected,
            CarPhysics.ReflexGovernedLongitudinal(
                Grip, Grip, slipAngle, 1f, Normal, speed: 6f
            ),
            4
        );
    }

    [Fact]
    public void BelowThePeakTheCeilingIsWhatItWas()
    {
        // Half the peak angle: the curve is at 0.876 of itself, which
        // leaves room for 0.897 of the pedal inside a 0.977 allowance.
        float slipAngle = TireSlipCurve.PeakSlipAngleRadians * 0.5f;
        float share = MathF.Abs(TireSlipCurve.Evaluate(slipAngle));
        float expected = MathF.Sqrt(
            (Normal * Normal - share * share) / (1f - share * share)
        ) * Grip;

        float ceiling = CarPhysics.ReflexGovernedLongitudinal(
            Grip, Grip, slipAngle, 1f, Normal
        );

        Assert.Equal(expected, ceiling, 4);
        Assert.InRange(ceiling / Grip, 0.89f, 0.90f);
    }

    /// <summary>
    /// No new step. Sweeping the slip angle from nothing to four times the
    /// peak, the ceiling only ever falls and then stays at zero, on every
    /// rung the reflex trims and for both axles' curves -- the front's peak
    /// sits further out, and has to be honoured where it is.
    /// </summary>
    [Theory]
    [InlineData(TireUsageMode.Protect, 1f)]
    [InlineData(TireUsageMode.Normal, 1f)]
    [InlineData(TireUsageMode.Push, 1f)]
    [InlineData(TireUsageMode.Normal, 1.35f)]
    public void TheCeilingOnlyFallsAndHasNoStepAtThePeak(
        TireUsageMode mode, float peakScale
    )
    {
        float allowance = new TireConfig().GetAccelerationUsage(mode);
        float previous = float.PositiveInfinity;
        const int samples = 800;
        for (int i = 0; i <= samples; i++)
        {
            float slipAngle = TireSlipCurve.PeakSlipAngleRadians *
                              peakScale * 4f * i / samples;
            float ceiling = CarPhysics.ReflexGovernedLongitudinal(
                Grip, Grip, slipAngle, peakScale, allowance
            );
            Assert.True(
                ceiling <= previous + 1e-4f,
                $"{mode}: the ceiling rose from {previous:F4} to {ceiling:F4} " +
                $"at {slipAngle / (TireSlipCurve.PeakSlipAngleRadians * peakScale):F3} of the peak"
            );
            previous = ceiling;
        }
        Assert.Equal(0f, previous);
    }

    /// <summary>
    /// Attack allots the whole circle, so below the peak the ceiling is the
    /// whole pedal. Shutting it past the peak would be a drop from all of
    /// it to none at one slip angle -- exactly the step the other rungs
    /// avoid by reaching zero before the peak -- so Attack is left to the
    /// physics on both sides, as it always was.
    /// </summary>
    [Theory]
    [InlineData(0.5f)]
    [InlineData(1.8f)]
    [InlineData(4.0f)]
    public void AttackIsLeftToThePhysicsOnBothSidesOfThePeak(
        float multipleOfPeak
    )
    {
        float slipAngle = TireSlipCurve.PeakSlipAngleRadians * multipleOfPeak;
        float attack = new TireConfig().GetAccelerationUsage(TireUsageMode.Attack);

        Assert.Equal(
            Grip,
            CarPhysics.ReflexGovernedLongitudinal(
                Grip, Grip, slipAngle, 1f, attack
            )
        );
    }
}
