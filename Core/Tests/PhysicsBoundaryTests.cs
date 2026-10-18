using System;
using System.Linq;
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
    public void WithoutTheLimiterTheTireModeDoesNotTouchTheSameCommand()
    {
        // The tyre rung reaches the physics only through the fitted
        // combined-grip limiter; a car without one drives the same command
        // identically on every rung.
        CarConfig car = new() { CombinedGripLimiterStrength = 0f };
        TestTires tires = WarmTires();
        CarState protect = NewState(45f, tires);
        CarState attack = NewState(45f, tires);
        Ask command = new(0.006f, -12f);
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
                StepInput(command.For(protect, car, tires), protectStrategy),
                1f / 60f
            );
            CarPhysics.Step(
                attack,
                car,
                tires,
                StepInput(command.For(attack, car, tires), attackStrategy),
                1f / 60f
            );
        }

        Near.Equal(protect.Speed, attack.Speed);
        Near.Equal(
            protect.Telemetry.ActualLongitudinalAccel,
            attack.Telemetry.ActualLongitudinalAccel);
        Near.Equal(
            protect.Telemetry.ActualLateralAccel,
            attack.Telemetry.ActualLateralAccel);
        Near.Equal(protect.FrontLeft.Wear, attack.FrontLeft.Wear, abs: 0f);
        Near.Equal(protect.RearLeft.SurfaceTempC, attack.RearLeft.SurfaceTempC);
    }

    [Fact]
    public void AFittedLimiterTrimsARealBrakeRequestAndAnUnfittedOneDoesNot()
    {
        CarConfig fitted = new();
        CarConfig unfitted = new() { CombinedGripLimiterStrength = 0f };
        TestTires tires = WarmTires();
        CarState limited = NewState(45f, tires);
        CarState free = NewState(45f, tires);
        CarStrategy protect = new(TireUsageMode.Protect, PowerOutputMode.Normal);
        // A tenth more braking than the tyres have in a straight line, in a
        // gentle corner: a real request, read off this car rather than a
        // number that was one for the car of the day.
        float brake = CarPhysics.EstimatePerformanceLimits(
            limited, fitted, tires, protect, 45f, curvature: 0f).MaximumBrakeDeceleration;
        Ask command = new(0.006f, -1.1f * brake);

        CarPhysics.Step(limited, fitted, tires, StepInput(command.For(limited, fitted, tires), protect), 1f / 60f);
        CarPhysics.Step(free, unfitted, tires, StepInput(command.For(free, unfitted, tires), protect), 1f / 60f);

        Assert.True(limited.Axles().CombinedGripLimiterCutAccel > 0f);
        Assert.Equal(0f, free.Axles().CombinedGripLimiterCutAccel);
        Assert.True(
            limited.Axles().FrontLongitudinalUse <
            free.Axles().FrontLongitudinalUse
        );
    }

    /// <summary>
    /// Strength zero is bit-for-bit the car without the device: whatever
    /// share of the circle the rung would allow a fitted limiter, an
    /// unfitted car never has a cut, and it drives digit for digit the same
    /// through a sweep of cornering, full drive and full braking.
    /// </summary>
    [Theory]
    [InlineData(TireUsageMode.Protect)]
    [InlineData(TireUsageMode.Normal)]
    [InlineData(TireUsageMode.Push)]
    public void AnUnfittedLimiterIsBitForBitTheIdleDevice(TireUsageMode mode)
    {
        // Unfitted, the limiter cuts nothing, so the rung -- which only sets
        // how much of the circle a fitted one allows -- changes nothing.
        CarConfig unfitted = new() { CombinedGripLimiterStrength = 0f };
        TestTires tires = WarmTires();
        CarState idle = NewState(45f, tires);
        CarState absent = NewState(45f, tires);
        CarStrategy attack = new(TireUsageMode.Attack, PowerOutputMode.Normal);
        CarStrategy rung = new CarStrategy(mode, PowerOutputMode.Normal).WithTireGripUsage(0.6f);

        for (int i = 0; i < 240; i++)
        {
            float pedal = i % 80 < 40 ? 12f : -30f;
            Ask command = new(0.008f * MathF.Sin(i * 0.05f), pedal);
            CarPhysics.Step(idle, unfitted, tires, StepInput(command.For(idle, unfitted, tires), attack), 1f / 60f);
            CarPhysics.Step(absent, unfitted, tires, StepInput(command.For(absent, unfitted, tires), rung), 1f / 60f);

            Assert.Equal(idle.Speed, absent.Speed);
            Assert.Equal(idle.SideslipAngleRadians, absent.SideslipAngleRadians);
            Assert.Equal(idle.YawRateRadiansPerSecond, absent.YawRateRadiansPerSecond);
            Assert.Equal(idle.Heading, absent.Heading);
            Assert.Equal(idle.RearLeft.Wear, absent.RearLeft.Wear);
            Assert.Equal(idle.FrontRight.SurfaceTempC, absent.FrontRight.SurfaceTempC);
            Assert.Equal(0f, absent.Axles().CombinedGripLimiterCutAccel);
        }
    }

    /// <summary>
    /// The performance envelope is published: a controller outside Core can
    /// ask what the car can do, and asking changes nothing about the car.
    /// </summary>
    [Fact]
    public void ThePublishedEnvelopeIsPublicAndReadsWithoutWriting()
    {
        // Published on the interface every car implements, so a driver can
        // read it off any car.
        var method = typeof(ICar).GetMethod(nameof(ICar.EstimatePerformanceLimits));
        Assert.NotNull(method);
        Assert.True(method!.IsPublic);
        Assert.True(typeof(CarPerformanceLimits).IsPublic);

        TestTires tires = WarmTires();
        ICar car = new SingleTrackCar(new CarConfig(), tires);
        CarState state = NewState(40f, tires);
        CarStrategy strategy = new(TireUsageMode.Normal, PowerOutputMode.Normal);
        CarState before = state.Clone();

        CarPerformanceLimits limits = car.EstimatePerformanceLimits(
            state, strategy, state.Speed, 0.01f, car.GripUsage(strategy)
        );

        Assert.True(limits.LateralAccelerationLimit > 0f);
        Assert.True(limits.MaximumDriveAcceleration > 0f);
        Assert.True(limits.MaximumBrakeDeceleration > 0f);
        Assert.Equal(before.Speed, state.Speed);
        Assert.Equal(before.Position, state.Position);
        Assert.Equal(before.FrontLeft.SurfaceTempC, state.FrontLeft.SurfaceTempC);
        Assert.Equal(before.RearRight.Wear, state.RearRight.Wear);
        Assert.Equal(before.Energy.Primary, state.Energy.Primary);
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

    private static TestTires WarmTires() => new(new TireConfig
    {
    }, 90f, 90f);
}
