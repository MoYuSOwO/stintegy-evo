using System;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// TireConfig.TireStressScale: one static coefficient on the heat and wear
/// that sliding puts into the tyre (era/world-v3). The same driving wears
/// and heats the tyres more above one and less below it, and rolling alone
/// is not touched.
/// </summary>
public sealed class TireStressScaleTests
{
    [Fact]
    public void TheScaleMovesSlidingWearAndHeatBothWays()
    {
        (float wearLow, float heatLow) = Corner(0.85f);
        (float wearMid, float heatMid) = Corner(1f);
        (float wearHigh, float heatHigh) = Corner(1.15f);

        Assert.True(wearLow < wearMid && wearMid < wearHigh,
            $"wear {wearLow} {wearMid} {wearHigh}");
        Assert.True(heatLow < heatMid && heatMid < heatHigh,
            $"heat {heatLow} {heatMid} {heatHigh}");
        // Wear scales with the coefficient step for step, since nothing
        // else in the run depends on it within a second of driving.
        Assert.InRange(wearHigh / wearMid, 1.12f, 1.18f);
    }

    [Fact]
    public void RollingStraightIsNotTheDrivers()
    {
        (float wearLow, float heatLow) = Straight(0.85f);
        (float wearHigh, float heatHigh) = Straight(1.15f);
        // A straight at a steady cruise slides almost nothing, so the scale
        // has almost nothing to act on.
        Assert.Equal(heatLow, heatHigh, 1);
        Assert.InRange(MathF.Abs(wearHigh - wearLow), 0f, 1e-6f);
    }

    private static (float Wear, float SurfaceTemp) Corner(float scale) =>
        Run(scale, new DriverInput(0.02f, 0f), 45f);

    private static (float Wear, float SurfaceTemp) Straight(float scale) =>
        Run(scale, new DriverInput(0f, 0f), 30f);

    private static (float Wear, float SurfaceTemp) Run(float scale, DriverInput input, float speed)
    {
        CarConfig car = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            TireStressScale = scale
        };
        CarState state = new()
        {
            Speed = speed,
            Energy = PowertrainState.Filled(0.8f)
        };
        state.InstallFreshTires(tires);
        CarStrategy strategy = new(TireUsageMode.Attack, PowerOutputMode.Normal);
        for (int i = 0; i < 60; i++)
            CarPhysics.Step(state, car, tires, new CarPhysicsStepInput(input, strategy, 25f, 35f), 1f / 60f);
        float wear = state.FrontLeft.Wear + state.FrontRight.Wear + state.RearLeft.Wear + state.RearRight.Wear;
        float heat = state.FrontLeft.SurfaceTempC + state.RearLeft.SurfaceTempC;
        return (wear, heat);
    }
}
