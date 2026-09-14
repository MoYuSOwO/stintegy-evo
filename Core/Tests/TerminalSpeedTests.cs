using System;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The flat-road top speed certification compares straight-line speeds
/// against: where the most drive available meets the air and the rolling
/// resistance.
/// </summary>
public sealed class TerminalSpeedTests
{
    private static readonly TireConfig Tires =
        new() { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f };
    private static readonly PowertrainState Energy = PowertrainState.Filled(0.8f);

    [Fact]
    public void AtTheTerminalSpeedTheCarNeitherGainsNorLosesSpeed()
    {
        CarConfig config = new();
        CarStrategy strategy = new((TireUsageMode)3, 3);
        float speed = CarPhysics.TerminalSpeedOnTheFlat(config, Tires, strategy, Energy);

        CarState state = new() { Energy = Energy };
        state.InstallFreshTires(Tires);
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            state, config, Tires, strategy, speed, curvature: 0f
        );

        Assert.InRange(speed, 30f, 149f);
        Assert.Equal(
            limits.LossAcceleration,
            limits.MaximumDriveAcceleration,
            2
        );
    }

    [Fact]
    public void MorePowerIsMoreTopSpeed()
    {
        CarConfig config = new();
        float previous = 0f;
        for (int power = 1; power <= 5; power++)
        {
            float speed = CarPhysics.TerminalSpeedOnTheFlat(
                config, Tires, new CarStrategy((TireUsageMode)3, power), Energy
            );
            Assert.True(
                speed > previous,
                $"power rung {power} should be quicker than the one below, " +
                $"got {speed:F2} after {previous:F2} m/s"
            );
            previous = speed;
        }
    }
}
