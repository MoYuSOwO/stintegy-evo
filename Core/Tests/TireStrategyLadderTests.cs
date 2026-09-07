using System;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The tyre half of the strategy table. Its figures live on the compound
/// now rather than on the speed planner's configuration, so a car can be
/// fitted with rubber that rewards looking after it more, or less, than the
/// rubber beside it - and so a driver that never asks the planner anything
/// can still find out what its pit wall wanted.
/// </summary>
public sealed class TireStrategyLadderTests
{
    [Fact]
    /// <summary>
    /// Leaning harder on the tyres is always at least as hard as leaning less,
    /// the slider lands between the named settings, and a figure asked for
    /// directly is honoured.
    ///
    /// The ordering is the part worth holding: a mode that says push has to
    /// use no less grip than one that says protect, whatever the numbers are
    /// retuned to. Asserting each setting equals the constant written beside
    /// it on the compound held nothing, since the only way to fail it was
    /// to change that constant on purpose.
    /// </summary>
    public void LeaningHarderOnTheTiresNeverUsesLessGrip()
    {
        TireConfig tires = new();
        float protect = tires.GetAccelerationUsage(TireUsageMode.Protect);
        float light = tires.GetAccelerationUsage(TireUsageMode.Light);
        float normal = tires.GetAccelerationUsage(TireUsageMode.Normal);
        float push = tires.GetAccelerationUsage(TireUsageMode.Push);
        float attack = tires.GetAccelerationUsage(TireUsageMode.Attack);

        Assert.True(protect <= light);
        Assert.True(light <= normal);
        Assert.True(normal <= push);
        Assert.True(push <= attack);

        float between = tires.GetAccelerationUsage(0.375f);
        Assert.InRange(between, light, normal);
        Assert.Equal((light + normal) * 0.5f, between, precision: 4);

        // Somewhere the named settings do not land, so it is the custom value
        // coming back and not one of them. Taken from the ends rather than
        // written down, because where those ends sit is a calibration.
        float betweenNamedSettings = (normal + push) * 0.5f;
        Assert.Equal(
            betweenNamedSettings,
            tires.GetAccelerationUsage(
                CarStrategy.Default.WithTireGripUsage(betweenNamedSettings)
            )
        );
    }

    [Fact]
    public void ACompoundWhoseLadderRunsBackwardsIsRefused()
    {
        // The figures are a modder's to write now, which is the point of
        // moving them and also why this check has to exist: a Push that
        // asked for less grip than Normal would have the pit wall call for
        // more and the car give less, and every number involved would still
        // look like a plausible fraction.
        TireConfig backwards = new() { PushAccelerationUsage = 0.9f };
        Assert.Throws<ArgumentOutOfRangeException>(
            backwards.ValidateAccelerationLadder
        );

        TireConfig impossible = new() { NormalAccelerationUsage = 1.4f };
        Assert.Throws<ArgumentOutOfRangeException>(
            impossible.ValidateAccelerationLadder
        );

        // And the compound that ships is a ladder.
        TireConfig.Default.ValidateAccelerationLadder();
    }

    [Fact]
    public void TwoCompoundsCanDisagreeAboutWhatProtectCosts()
    {
        // What the move buys: the same instruction from the pit wall means
        // something different on different rubber.
        TireConfig forgiving = new();
        TireConfig punishing = new() { ProtectAccelerationUsage = 0.88f };

        Assert.True(
            punishing.GetAccelerationUsage(TireUsageMode.Protect) <
            forgiving.GetAccelerationUsage(TireUsageMode.Protect),
            "a compound that asks to be looked after should say so"
        );
        Assert.Equal(
            forgiving.GetAccelerationUsage(TireUsageMode.Attack),
            punishing.GetAccelerationUsage(TireUsageMode.Attack)
        );
        punishing.ValidateAccelerationLadder();
    }
}
