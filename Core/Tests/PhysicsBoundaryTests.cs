using System;
using System.Linq;
using System.Reflection;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Regression tests for the boundary between a controller's command and the
/// vehicle model. DriverInput remains the public command shape; no driver
/// ability, reflex, or settle quota is smuggled into the physics step.
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
    public void SameDriverInputDoesNotAcquireATireModePedalReflex()
    {
        CarConfig car = new() { TractionControlStrength = 0f };
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
    public void ConfiguredTractionControlStillCutsARealRearDriveRequest()
    {
        CarConfig controlledCar = new()
        {
            FrontDriveShare = 0f,
            TractionControlStrength = 1f,
            TractionControlActivationUse = 0.5f,
            DownforceAccelPerSpeedSquared = 0f,
            AeroDragAccelPerSpeedSquared = 0f
        };
        CarConfig uncontrolledCar = new()
        {
            FrontDriveShare = 0f,
            TractionControlStrength = 0f,
            DownforceAccelPerSpeedSquared = 0f,
            AeroDragAccelPerSpeedSquared = 0f
        };
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            BaseMu = 0.4f
        };
        CarState controlled = NewState(20f, tires);
        CarState uncontrolled = NewState(20f, tires);
        DriverInput command = new(0f, 12f);
        CarStrategy strategy = new(
            TireUsageMode.Attack,
            PowerOutputMode.Attack
        );

        CarPhysics.Step(
            controlled,
            controlledCar,
            tires,
            StepInput(command, strategy),
            1f / 60f
        );
        CarPhysics.Step(
            uncontrolled,
            uncontrolledCar,
            tires,
            StepInput(command, strategy),
            1f / 60f
        );

        Assert.True(controlled.Telemetry.TractionControlCutAccel > 0f);
        Assert.Equal(0f, uncontrolled.Telemetry.TractionControlCutAccel, precision: 5);
        Assert.True(
            controlled.Telemetry.RearLongitudinalUse <
            uncontrolled.Telemetry.RearLongitudinalUse
        );
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
