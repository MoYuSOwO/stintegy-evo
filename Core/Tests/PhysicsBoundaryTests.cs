using System;
using System.Linq;
using System.Reflection;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Regression tests for the boundary between a controller's command and the
/// vehicle model. DriverInput remains the public command shape; no driver
/// ability, reflex, or settle quota is smuggled into the physics step. The
/// one thing between the command and the tyres is a device on the car, the
/// combined-grip limiter, whose parameters are published on CarConfig.
/// </summary>
public sealed class PhysicsBoundaryTests
{
    [Fact]
    public void DriverInputKeepsOnlyCurvatureAccelerationAndBrakeBias()
    {
        string[] properties = typeof(DriverInput)
            .GetProperties(BindingFlags.Instance | BindingFlags.Public)
            .Select(property => property.Name)
            .OrderBy(name => name)
            .ToArray();

        Assert.Equal(
            new[] { "DesiredAccel", "DesiredCurvature", "FrontBrakeBiasOffset" },
            properties
        );
    }

    [Fact]
    public void PhysicsStepInputHasNoDriverEfficiencyOrSettleControls()
    {
        string[] properties = typeof(CarPhysicsStepInput)
            .GetProperties(BindingFlags.Instance | BindingFlags.Public)
            .Select(property => property.Name)
            .ToArray();

        Assert.DoesNotContain("TireEnergyEfficiency", properties);
        Assert.DoesNotContain("CorneringEfficiency", properties);
        Assert.DoesNotContain("LimitSettleUse", properties);
        Assert.Contains("DriverInput", properties);
        Assert.Contains("Strategy", properties);
        Assert.Contains("AirTempC", properties);
        Assert.Contains("TrackTempC", properties);
    }

    [Fact]
    public void WithoutTheLimiterTheTireModeDoesNotTouchTheSameCommand()
    {
        // The tyre rung reaches the physics only through the fitted
        // combined-grip limiter; a car without one drives the same command
        // identically on every rung.
        CarConfig car = new() { CombinedGripLimiterStrength = 0f };
        TireConfig tires = WarmTires();
        CarState protect = NewState(45f, tires);
        CarState attack = NewState(45f, tires);
        DriverInput command = new(0.006f, -12f);
        CarStrategy protectStrategy = new(
            TireUsageMode.Protect,
            PowerOutputMode.Normal
        );
        CarStrategy attackStrategy = new(
            TireUsageMode.Attack,
            PowerOutputMode.Normal
        );

        for (int i = 0; i < 60; i++)
        {
            CarPhysics.Step(
                protect,
                car,
                tires,
                StepInput(command, protectStrategy),
                1f / 60f
            );
            CarPhysics.Step(
                attack,
                car,
                tires,
                StepInput(command, attackStrategy),
                1f / 60f
            );
        }

        Assert.Equal(protect.Speed, attack.Speed, precision: 5);
        Assert.Equal(
            protect.Telemetry.ActualLongitudinalAccel,
            attack.Telemetry.ActualLongitudinalAccel,
            precision: 5
        );
        Assert.Equal(
            protect.Telemetry.ActualLateralAccel,
            attack.Telemetry.ActualLateralAccel,
            precision: 5
        );
        Assert.Equal(protect.FrontLeft.Wear, attack.FrontLeft.Wear, precision: 5);
        Assert.Equal(protect.RearLeft.SurfaceTempC, attack.RearLeft.SurfaceTempC, precision: 5);
    }

    [Fact]
    public void AFittedLimiterTrimsARealBrakeRequestAndAnUnfittedOneDoesNot()
    {
        CarConfig fitted = new();
        CarConfig unfitted = new() { CombinedGripLimiterStrength = 0f };
        TireConfig tires = WarmTires();
        CarState limited = NewState(45f, tires);
        CarState free = NewState(45f, tires);
        DriverInput command = new(0.006f, -30f);
        CarStrategy protect = new(TireUsageMode.Protect, PowerOutputMode.Normal);

        CarPhysics.Step(limited, fitted, tires, StepInput(command, protect), 1f / 60f);
        CarPhysics.Step(free, unfitted, tires, StepInput(command, protect), 1f / 60f);

        Assert.True(limited.Telemetry.CombinedGripLimiterCutAccel > 0f);
        Assert.Equal(0f, free.Telemetry.CombinedGripLimiterCutAccel);
        Assert.True(
            limited.Telemetry.FrontLongitudinalUse <
            free.Telemetry.FrontLongitudinalUse
        );
    }

    /// <summary>
    /// Strength zero is bit-for-bit the car without the device. The
    /// reference is the fitted car on the Attack rung, which authorises the
    /// whole circle and so leaves the device nothing to do; an unfitted car
    /// on any rung has to match it digit for digit through a sweep of
    /// cornering, full drive and full braking.
    /// </summary>
    [Theory]
    [InlineData(TireUsageMode.Protect)]
    [InlineData(TireUsageMode.Normal)]
    [InlineData(TireUsageMode.Push)]
    public void AnUnfittedLimiterIsBitForBitTheIdleDevice(TireUsageMode mode)
    {
        CarConfig fitted = new();
        CarConfig unfitted = new() { CombinedGripLimiterStrength = 0f };
        TireConfig tires = WarmTires();
        CarState idle = NewState(45f, tires);
        CarState absent = NewState(45f, tires);
        CarStrategy attack = new(TireUsageMode.Attack, PowerOutputMode.Normal);
        CarStrategy rung = new(mode, PowerOutputMode.Normal);

        for (int i = 0; i < 240; i++)
        {
            float pedal = i % 80 < 40 ? 12f : -30f;
            DriverInput command = new(0.008f * MathF.Sin(i * 0.05f), pedal);
            CarPhysics.Step(idle, fitted, tires, StepInput(command, attack), 1f / 60f);
            CarPhysics.Step(absent, unfitted, tires, StepInput(command, rung), 1f / 60f);

            Assert.Equal(idle.Speed, absent.Speed);
            Assert.Equal(idle.SideslipAngleRadians, absent.SideslipAngleRadians);
            Assert.Equal(idle.YawRateRadiansPerSecond, absent.YawRateRadiansPerSecond);
            Assert.Equal(idle.Heading, absent.Heading);
            Assert.Equal(idle.RearLeft.Wear, absent.RearLeft.Wear);
            Assert.Equal(idle.FrontRight.SurfaceTempC, absent.FrontRight.SurfaceTempC);
            Assert.Equal(0f, absent.Telemetry.CombinedGripLimiterCutAccel);
        }
    }

    private static CarPhysicsStepInput StepInput(
        DriverInput input,
        CarStrategy strategy
    ) => new(input, strategy, 25f, 35f);

    private static CarState NewState(float speed, TireConfig tires)
    {
        CarState state = new()
        {
            Speed = speed,
            Energy = PowertrainState.Filled(0.8f)
        };
        state.InstallFreshTires(tires);
        return state;
    }

    private static TireConfig WarmTires() => new()
    {
        StartingSurfaceTempC = 90f,
        StartingCoreTempC = 90f
    };
}
