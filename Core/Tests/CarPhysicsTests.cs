using System;
using System.Collections.Generic;
using System.Linq;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class CarPhysicsTests
{
    /// <summary>
    /// The tyre mode that authorises the whole friction circle, so the
    /// combined-grip limiter has nothing to trim.
    /// </summary>
    private static readonly CarStrategy Unclamped =
        new(TireUsageMode.Attack, PowerOutputMode.Normal);

    private const float TestAirTempC = 25f;
    private const float TestTrackTempC = 35f;

    /// <summary>
    /// With the lateral heat coefficient at zero, cornering must not heat
    /// the rear tyres at all.
    ///
    /// The rear once had a slip-angle heater of its own alongside the
    /// general force-times-sliding term, and switching the general term
    /// off left it running. Before it was removed this read about 0.009 C
    /// of rear-only heat in one step; the bound is two orders tighter.
    /// </summary>
    [Fact]
    public void ACompoundThatMakesNoHeatMakesNoneAnywhere()
    {
        CarConfig car = new()
        {
            FrontStaticLoadShare = 0.5f,
            CenterOfGravityHeightMeters = 0f,
            Aero = new StandardAero { DownforcePerSpeedSquared = 0f, DragPerSpeedSquared = 0f },
            RollingDragAccel = 0f,
            OverLimitCostCap = 0f
        };
        // Every source of heat in the tyre goes through the compound's own
        // scale: with it at nothing, the rear of a cornering car -- where a
        // second, rear-only slip heater once lived -- warms no more than the
        // rear of one going straight, and neither warms at all.
        TestTires tires = new(new TireConfig { HeatScale = 0f }, 90f, 90f);
        CarState straight = CreateState(30f, 0.8f, tires);
        CarState corner = CreateState(30f, 0.8f, tires);
        float curvature = CurvatureForGripShare(corner, car, tires, 0.6f);
        SetSteadyCorner(corner, car, tires, curvature);
        CarPhysics.Step(straight, car, tires,
            new CarPhysicsStepInput(new DriverInput(0f, 0f), CarStrategy.Default,
                AirTempC: 90f, TrackTempC: 90f), 1f / 120f);
        CarPhysics.Step(corner, car, tires,
            new CarPhysicsStepInput(new DriverInput(curvature, 0f), CarStrategy.Default,
                AirTempC: 90f, TrackTempC: 90f), 1f / 120f);

        Assert.True(corner.Axles().RearLateralUse > 0.1f);
        Assert.Equal(0, corner.SpinEvents);
        Assert.InRange(Math.Abs(AverageRearSurfaceTemp(corner) -
            AverageRearSurfaceTemp(straight)), 0f, 0.01f);
        Assert.True(AverageRearSurfaceTemp(corner) <= 90f, "nothing heats a tyre the compound makes no heat in");
    }

    [Fact]
    public void DriveRequestIncreasesSpeedAndConsumesBattery()
    {
        // The all-electric car: its pack is the only store, so drive comes
        // straight out of it. The range extender's own case is below.
        CarConfig car = new() { Powertrain = ElectricPowertrain.Default };
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 24f, batterySoc: 0.8f, tires);
        float startSpeed = state.Speed;
        float startSoc = state.Energy.Primary;

        StepMany(state, car, tires, new DriverInput(0f, 4f), CarStrategy.Default, steps: 30);

        Assert.True(state.Speed > startSpeed, "speed should increase under positive drive request");
        Assert.True(state.Energy.Primary < startSoc, "battery SOC should fall when drive power is used");
        Assert.True(state.Telemetry.DrivePowerWatts > 0f, "drive power should be reported");
        Assert.True(state.Telemetry.ActualLongitudinalAccel > 0f, "net longitudinal accel should remain positive");
    }

    [Fact]
    public void BrakeRequestSlowsCarAndRegeneratesEnergy()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 34f, batterySoc: 0.5f, tires);
        float startSpeed = state.Speed;
        float startSoc = state.Energy.Primary;

        StepMany(state, car, tires, new DriverInput(0f, -7f), CarStrategy.Default, steps: 20);

        Assert.True(state.Speed < startSpeed, "speed should fall under brake request");
        Assert.True(state.Energy.Primary > startSoc, "battery SOC should rise from regen during braking");
        Assert.True(state.Telemetry.RegenPowerWatts > 0f, "regen power should be reported");
        Assert.True(state.Telemetry.ActualLongitudinalAccel < 0f, "net longitudinal accel should be negative");
    }

    /// <summary>
    /// The pit wall's share of the tyre is no longer a price. It is a
    /// ceiling the pedals cannot be pushed through.
    ///
    /// Driving hard enough that the corner alone is spending most of the
    /// circle, and then asking for everything the motor and the brakes
    /// have: whatever the tyres end up spending has to land inside what
    /// the mode allots. The limiter solves the ellipse once — the slip
    /// angle's own share leaves room for a pedal share and no more — so
    /// this holds for the realised usage rather than for the request.
    ///
    /// Held to max(A, s) rather than A. Past a slip angle whose own share s
    /// already exceeds A the pedals are at zero and the excess is the
    /// steering's; with drive requested the car gets there here within a
    /// quarter of a second. The retired traction control used to trim drive
    /// early enough to keep this corner short of it, which is why the bound
    /// was once written as A alone.
    /// </summary>
    [Theory]
    [InlineData(TireUsageMode.Protect, 12f)]
    [InlineData(TireUsageMode.Protect, -12f)]
    [InlineData(TireUsageMode.Normal, 12f)]
    [InlineData(TireUsageMode.Normal, -12f)]
    [InlineData(TireUsageMode.Push, 12f)]
    [InlineData(TireUsageMode.Push, -12f)]
    public void ATyreModeCannotBeDrivenPastByThePedals(
        TireUsageMode mode, float accel
    )
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarStrategy strategy = new(mode, PowerOutputMode.Normal);
        CarState state = CreateState(speed: 45f, batterySoc: 0.8f, tires);
        float curvature = CurvatureForGripShare(
            state, car, tires, strategy, 0.8f
        );
        SetSteadyCorner(state, car, tires, curvature);
        float allowance = CarPhysics.LimiterGripShare;

        for (int i = 0; i < 30; i++)
        {
            CarPhysics.Step(
                state,
                car,
                tires,
                PhysicsInput(new DriverInput(curvature, accel), strategy),
                1f / 60f
            );
            AxleTelemetry telemetry = state.Axles();
            // use <= max(A, s): the pedals cannot take an axle past the
            // authorisation, and once the slip angle alone has, the limiter
            // has already cut them to nothing and what is left is steering.
            Assert.InRange(
                CombinedUse(
                    telemetry.FrontLateralUse, telemetry.FrontLongitudinalUse
                ),
                0f,
                MathF.Max(allowance, telemetry.FrontLateralUse) + 1e-3f
            );
            Assert.InRange(
                CombinedUse(
                    telemetry.RearLateralUse, telemetry.RearLongitudinalUse
                ),
                0f,
                MathF.Max(allowance, telemetry.RearLateralUse) + 1e-3f
            );
        }
    }

    /// <summary>
    /// A pedal request the mode can afford arrives untouched. The limiter is
    /// a ceiling, not a tax: a command inside its authorisation should not
    /// be able to tell that it is fitted.
    /// </summary>
    [Fact]
    public void APedalRequestInsideTheAllowanceArrivesUntouched()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState governed = CreateState(speed: 45f, batterySoc: 0.8f, tires);
        CarState free = CreateState(speed: 45f, batterySoc: 0.8f, tires);
        DriverInput gentle = new(0f, 2f);

        for (int i = 0; i < 30; i++)
        {
            CarPhysics.Step(
                governed, car, tires,
                PhysicsInput(gentle, CarStrategy.Default), 1f / 60f
            );
            CarPhysics.Step(
                free, car, tires, PhysicsInput(gentle, Unclamped), 1f / 60f
            );
        }

        Near.Equal(free.Speed, governed.Speed);
        Near.Equal(
            free.Telemetry.ActualLongitudinalAccel,
            governed.Telemetry.ActualLongitudinalAccel);
    }

    /// <summary>
    /// Whether a tyre can still be over its allowance once the pedals
    /// cannot put it there — which decides whether the steward penalty for
    /// it still has a job.
    ///
    /// It can, and by one path: the slip angle. What an axle spends is the
    /// slip angle's own share of the curve plus what is left of the circle
    /// after it, so a tyre worked near its peak angle is spending almost
    /// the whole circle on steering alone, and no pedal position subtracts
    /// from that. The limiter cuts the pedals to nothing there, which is all
    /// it can do — steering is deliberately not trimmed, because that is
    /// where a car is caught rather than lost.
    ///
    /// So mode_excess stays, and this is the only way to reach it.
    /// </summary>
    [Fact]
    public void TheAllowanceIsStillReachableThroughTheSlipAngleAlone()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarStrategy strategy = CarStrategy.Default;
        CarState state = CreateState(speed: 45f, batterySoc: 0.8f, tires);
        // A corner past what the car will hold, and not one pedal touched.
        float curvature = CurvatureForGripShare(
            state, car, tires, strategy, 1.3f
        );
        SetSteadyCorner(state, car, tires, curvature);
        float allowance = CarPhysics.LimiterGripShare;

        float worst = 0f;
        for (int i = 0; i < 30; i++)
        {
            CarPhysics.Step(
                state,
                car,
                tires,
                PhysicsInput(new DriverInput(curvature, 0f), strategy),
                1f / 60f
            );
            AxleTelemetry telemetry = state.Axles();
            worst = MathF.Max(
                worst,
                MathF.Max(
                    CombinedUse(
                        telemetry.FrontLateralUse,
                        telemetry.FrontLongitudinalUse
                    ),
                    CombinedUse(
                        telemetry.RearLateralUse,
                        telemetry.RearLongitudinalUse
                    )
                )
            );
        }

        Assert.True(
            worst > allowance,
            $"the steering path should still reach past the allowance, got " +
            $"{worst:F4} against {allowance:F4}"
        );
    }

    private static float CombinedUse(float lateral, float longitudinal) =>
        MathF.Min(
            1f,
            MathF.Sqrt(lateral * lateral + longitudinal * longitudinal)
        );


    [Fact]
    public void LowGripHeavyBrakingIsClippedByAxles()
    {
        // Without the combined-grip limiter: this is the physics of a tyre
        // asked for more than it has, which a fitted limiter would never let
        // the pedals reach below the Attack rung.
        CarConfig car = new() { CombinedGripLimiterStrength = 0f };
        TestTires tires = new(new TireConfig
        {
            BaseMu = 0.6f
        }, 90f, 90f);
        CarState state = CreateState(speed: 32f, batterySoc: 0.5f, tires);

        CarPhysics.Step(state, car, tires, PhysicsInput(new DriverInput(0f, -12f)), 1f / 60f);

        Assert.True(state.Telemetry.OverLimit > 0f, "brake demand above low-grip capacity should be reported as over limit");
        Assert.True(
            Math.Abs(state.Telemetry.ActualLongitudinalAccel) < Math.Abs(state.Telemetry.RequestedLongitudinalAccel),
            "actual braking should be clipped below the requested braking acceleration"
        );
        Assert.True(state.Telemetry.RegenPowerWatts > 0f, "clipped braking should still produce regen");
    }

    [Fact]
    public void BrakeOverlimitReducesDeliveredBrake()
    {
        // Without the combined-grip limiter: this is the physics of a tyre
        // asked for more than it has, which a fitted limiter would never let
        // the pedals reach below the Attack rung.
        CarConfig noEfficiencyLoss = new()
        {
            OverLimitMinGripEfficiency = 1f,
            CombinedGripLimiterStrength = 0f
        };
        CarConfig defaultEfficiencyLoss = new() { CombinedGripLimiterStrength = 0f };
        TestTires tires = new(new TireConfig
        {
            BaseMu = 0.6f
        }, 90f, 90f);
        CarState noEfficiencyLossState = CreateState(speed: 32f, batterySoc: 0.5f, tires);
        CarState defaultEfficiencyLossState = CreateState(speed: 32f, batterySoc: 0.5f, tires);
        CarPhysicsStepInput input = PhysicsInput(new DriverInput(0f, -12f));

        CarPhysics.Step(noEfficiencyLossState, noEfficiencyLoss, tires, input, 1f / 60f);
        CarPhysics.Step(defaultEfficiencyLossState, defaultEfficiencyLoss, tires, input, 1f / 60f);

        Assert.True(defaultEfficiencyLossState.Telemetry.OverLimit > 0f, "brake demand should still be reported as over limit");
        Assert.True(
            defaultEfficiencyLossState.Telemetry.RegenPowerWatts < noEfficiencyLossState.Telemetry.RegenPowerWatts,
            "over-limit braking should deliver less actual braking work"
        );
        Assert.True(
            defaultEfficiencyLossState.Telemetry.ActualLongitudinalAccel > noEfficiencyLossState.Telemetry.ActualLongitudinalAccel,
            "over-limit braking should be less negative, not faster"
        );
    }

    [Fact]
    public void FrontGripLimitReducesDeliveredCurvature()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 46f, batterySoc: 0.8f, tires);
        DriverInput input = new(0.03f, 0f);

        CarPhysics.Step(state, car, tires, PhysicsInput(input), 1f / 60f);

        Assert.True(
            Math.Abs(state.Telemetry.RequestedLateralAccel) > Math.Abs(state.Telemetry.ActualLateralAccel),
            "actual lateral accel should be clipped when requested curvature exceeds grip"
        );
        Assert.True(
            Math.Abs(state.Telemetry.ActualCurvature) < Math.Abs(input.DesiredCurvature),
            "delivered curvature should be below requested curvature under front grip limit"
        );
    }

    [Fact]
    public void ExtremeOverLimitRequestDoesNotEraseSpeedInOneStep()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 55f, batterySoc: 0.8f, tires);

        DriverInput absurd = new(car.MaxCurvatureRequest, 0f);
        // Held for a second, because the wheels now take time to reach the
        // lock stops and an absurd request is only absurd once they have.
        // The over-limit reading is taken along the way rather than at the
        // end: a car asked for six times the grip it has now scrubs enough
        // speed off to lose the back end before the second is out, and the
        // spin choreography that follows reports no utilisation at all.
        float worstOverLimit = 0f;
        for (int i = 0; i < 60; i++)
        {
            CarPhysics.Step(state, car, tires, PhysicsInput(absurd), 1f / 60f);
            worstOverLimit = Math.Max(worstOverLimit, state.Telemetry.OverLimit);
        }

        Assert.True(
            worstOverLimit > 0.5f,
            "telemetry should still expose rubber dragged well past its peak"
        );
        Assert.True(state.Speed > 30f, "the cost should saturate rather than delete the speed");
        Assert.True(
            Math.Abs(state.Telemetry.ActualCurvature) < car.MaxCurvatureRequest,
            "actual curvature should still be limited by available grip"
        );
    }

    [Fact]
    public void RearOverlimitCutsDeliveredDrive()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarStrategy strategy = new(TireUsageMode.Normal, PowerOutputMode.Attack);
        DriverInput input = new(0.018f, 7f);
        SetSteadyCorner(state, car, tires, 0.018f);

        CarPhysics.Step(state, car, tires, PhysicsInput(input, strategy), 1f / 60f);

        Assert.True(
            state.Axles().RearLongitudinalUse > 0.2f,
            "the rear should be spending a real share of its circle on drive"
        );
        Assert.True(
            state.Telemetry.ActualLongitudinalAccel < state.Telemetry.RequestedLongitudinalAccel,
            "delivered longitudinal acceleration should be lower after rear traction is cut"
        );
    }

    [Fact]
    public void RearCombinedSaturationScalesLateralAndDriveTogether()
    {
        // Drive the rear cannot take on top of the corner. At this speed
        // what the rear has left after the corner is more than the stock
        // car's 12 m/s^2 of drive, so its pedal never reaches the circle;
        // the limiter is under test, not the motor's ceiling.
        CarConfig car = new() { CombinedGripLimiterStrength = 0f, MaxDriveAcceleration = UncappedDriveAcceleration };
        TestTires tires = WarmTires();
        CarState cornering = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarState powered = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarStrategy attack = new(TireUsageMode.Normal, PowerOutputMode.Attack);
        // Most of the grip spent on the corner, so asking for drive on top of
        // it has to come out of the same circle.
        float curvature = CurvatureForGripShare(cornering, car, tires, attack, 0.9f);
        SetSteadyCorner(cornering, car, tires, curvature);
        SetSteadyCorner(powered, car, tires, curvature);
        float drive = DriveForShare(powered, car, tires, attack, curvature, 1.5f);

        DriverInput coasting = new(curvature, 0f);
        DriverInput driving = new(curvature, drive);
        SetSteadyCorner(cornering, car, tires, curvature);
        SetSteadyCorner(powered, car, tires, curvature);
        CarPhysics.Step(cornering, car, tires, PhysicsInput(coasting, attack), 1f / 60f);
        CarPhysics.Step(powered, car, tires, PhysicsInput(driving, attack), 1f / 60f);

        Assert.True(powered.Axles().RearLongitudinalUse > 0.5f);
        Assert.True(
            Math.Abs(powered.Telemetry.ActualLateralAccel) <
            Math.Abs(cornering.Telemetry.ActualLateralAccel),
            "rear combined saturation should reduce lateral force instead of preserving it at all costs"
        );
        Assert.True(powered.Axles().RearLongitudinalUse > 0f);
    }

    [Fact]
    public void BrakeDistributionUsesEachAxlesRemainingFrictionCircle()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 25f, batterySoc: 0.9f, tires);
        state.SideslipAngleRadians = 0.04f;

        CarPhysics.Step(
            state,
            car,
            tires,
            PhysicsInput(new DriverInput(0.006f, -2f)),
            1f / 60f
        );

        Assert.InRange(state.Telemetry.OverLimit, 0f, 1e-3f);
        float frontRemainingUse = MathF.Sqrt(MathF.Max(
            0f,
            1f - state.Axles().FrontLateralUse *
                 state.Axles().FrontLateralUse
        ));
        float rearRemainingUse = MathF.Sqrt(MathF.Max(
            0f,
            1f - state.Axles().RearLateralUse *
                 state.Axles().RearLateralUse
        ));
        float frontCapacityFraction =
            state.Axles().FrontLongitudinalUse / frontRemainingUse;
        float rearCapacityFraction =
            state.Axles().RearLongitudinalUse / rearRemainingUse;
        Assert.InRange(
            MathF.Abs(frontCapacityFraction - rearCapacityFraction),
            0f,
            0.01f
        );
    }

    [Fact]
    public void BrakeBiasErrorLeavesGripUnusedAtTheLimit()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState optimal = CreateState(speed: 25f, batterySoc: 0.9f, tires);
        CarState frontBiased = CreateState(speed: 25f, batterySoc: 0.9f, tires);
        // Braking hard enough into a corner that both axles are working, so a
        // split that favours one of them has to leave the other with grip
        // nobody is using.
        float curvature = CurvatureForGripShare(optimal, car, tires, 0.55f);
        // Just under, not over: braking pitches load forward, so the estimate
        // taken level is a little optimistic and asking for all of it puts the
        // balanced car over the limit too, which is the thing being contrasted.
        // All of it, not just under. The car's advertised cornering limit
        // came down when its tyres got slip angles, so the corner this asks
        // for is gentler than it used to be and leaves more of the circle
        // for the brakes - which meant a bad split no longer clipped
        // anything and the test stopped testing.
        float brake = BrakeForShare(
            optimal, car, tires, CarStrategy.Default, curvature, 1.0f);

        DriverInput balanced = new(curvature, brake);
        DriverInput biased = new(curvature, brake, 0.07f);
        SetSteadyCorner(optimal, car, tires, curvature);
        SetSteadyCorner(frontBiased, car, tires, curvature);
        CarPhysics.Step(optimal, car, tires, PhysicsInput(balanced), 1f / 60f);
        CarPhysics.Step(frontBiased, car, tires, PhysicsInput(biased), 1f / 60f);

        // The combined-grip limiter holds each axle's braking inside the
        // share of the circle the tyre rung authorises, so neither car drives
        // a tyre past what it has, and what the bias costs is braking the car
        // never gets.
        Near.Equal(0f, optimal.Telemetry.OverLimit);
        Near.Equal(0f, frontBiased.Telemetry.OverLimit);
        Assert.True(
            frontBiased.Telemetry.ActualLongitudinalAccel >
            optimal.Telemetry.ActualLongitudinalAccel + 0.1f,
            "a biased split should clip one axle before all remaining grip is used"
        );
        // Cornering is not pinned here. The anti-lock this test used to
        // lean on held the overloaded axle back from its circle and so left
        // steering a little more; the limiter that replaced it lets that
        // axle spend up to the rung's authorisation, trading some of its
        // cornering for braking, which is a different device doing its job.
    }

    [Fact]
    public void DefaultRearDriveMakesTheRearCloserToTheCombinedLimitUnderPower()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 30f, batterySoc: 0.9f, tires);
        SetSteadyCorner(state, car, tires, 0.008f);

        CarPhysics.Step(
            state,
            car,
            tires,
            PhysicsInput(new DriverInput(0.008f, 3f)),
            1f / 60f
        );

        float frontCombinedUse = MathF.Sqrt(
            state.Axles().FrontLateralUse * state.Axles().FrontLateralUse +
            state.Axles().FrontLongitudinalUse * state.Axles().FrontLongitudinalUse
        );
        float rearCombinedUse = MathF.Sqrt(
            state.Axles().RearLateralUse * state.Axles().RearLateralUse +
            state.Axles().RearLongitudinalUse * state.Axles().RearLongitudinalUse
        );
        Assert.True(
            rearCombinedUse > frontCombinedUse,
            "rear drive should move the rear axle closer to its combined limit under power"
        );
    }

    [Fact]
    public void RearAxleSaturationCreatesRecoverableBodySideslip()
    {
        // Without the combined-grip limiter, which exists to stop exactly
        // this and would cut the drive before the rear let go.
        CarConfig car = new() { CombinedGripLimiterStrength = 0f };
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        MakeRearTiresHotAndWorn(state);
        CarStrategy attack = new(TireUsageMode.Normal, PowerOutputMode.Attack);
        // Cornering near what the worn rear will bear and then asking for all
        // the drive there is. Drive alone cannot do it: the request is clipped
        // at the car's own maximum, so the way to make an axle let go is to
        // have the corner already using most of its circle.
        // Past what the ruined rear will hold, because the limit estimate
        // is rear-limited on this car: asking for nine tenths of it is
        // asking for a corner the worn rear can still take, and nothing
        // lets go.
        float curvature = CurvatureForGripShare(state, car, tires, attack, 1.25f);
        float drive = car.MaxDriveAcceleration;

        StepMany(state, car, tires, new DriverInput(curvature, drive), attack, steps: 60);

        float builtSideslip = Math.Abs(state.SideslipAngleRadians);
        // The direction, not the size. How far the tail comes round is a
        // property of one car's grip, inertia and recovery rate, and pinning a
        // figure to it means the test stops being about rear saturation and
        // starts being about that car: give it wings and the same slide
        // settles at two thirds of the angle while sliding just as plainly.
        // Which end has gone, not which way the body ended up pointing.
        // The sign of body sideslip was a reliable read of oversteer in a
        // model where it was the leftover between requested and delivered
        // yaw. In a real slide it is not: a rear that has collapsed takes
        // the car's cornering force with it, so the path straightens as
        // fast as the body swings and the difference can land either side
        // of zero. What is unambiguous is that the rear is further past its
        // own peak than the front, which is what the severity channel says
        // and what a driver would be feeling.
        Assert.True(
            state.Axles().RearSlideSeverity > 0.1f,
            $"rear saturation should expose slide severity, got " +
            $"{state.Axles().RearSlideSeverity:0.000}"
        );
        Assert.True(state.YawRateRadiansPerSecond > 0f, "left-turn rear saturation should build positive yaw rate");
        Assert.True(
            builtSideslip > 0.05f,
            $"the car should be well out of shape, sideslip was " +
            $"{state.SideslipAngleRadians * 180f / MathF.PI:0.0} deg"
        );

        // Longer than it used to need, and that is the point. The model
        // this replaced straightened the car with a formula on a fixed time
        // constant, so two seconds was always enough. Nothing straightens
        // it now except the tyres, so how long it takes is a property of
        // the car and the state it was left in.
        StepMany(state, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 480);

        Assert.True(
            Math.Abs(state.SideslipAngleRadians) < builtSideslip * 0.25f,
            $"the tyres should bring the car back once the corner stops " +
            $"being asked for: {builtSideslip * 180f / MathF.PI:0.0} deg " +
            $"became {state.SideslipAngleRadians * 180f / MathF.PI:0.0} deg"
        );
        Assert.True(
            Math.Abs(state.YawRateRadiansPerSecond) < 0.05f,
            "yaw rate should settle after the curvature request ends"
        );
    }

    [Fact]
    public void NeutralYawDynamicsTracksTheCurvatureYawRateReference()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 30f, batterySoc: 0.9f, tires);

        StepMany(
            state,
            car,
            tires,
            new DriverInput(0.006f, 1f),
            CarStrategy.Default,
            steps: 120
        );

        Assert.True(state.Telemetry.ReferenceYawRateRadiansPerSecond > 0f);
        Assert.True(state.YawRateRadiansPerSecond > 0f);
        Assert.InRange(
            Math.Abs(
                state.YawRateRadiansPerSecond -
                state.Telemetry.ReferenceYawRateRadiansPerSecond
            ),
            0f,
            0.15f
        );
        Assert.InRange(Math.Abs(state.SideslipAngleRadians), 0f, 0.08f);
    }

    [Fact]
    public void FrontAxleSaturationBuildsUndersteerSideslip()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        MakeFrontTiresHotAndWorn(state);
        // More corner than a worn front will hold, which is what saturating
        // it means now: the request alone no longer decides how the axles
        // are loaded.
        float overdriven = CurvatureForGripShare(state, car, tires, 1.15f);

        float maximumSideslip = 0f;
        float maximumYawDeficit = 0f;
        for (int step = 0; step < 60; step++)
        {
            CarPhysics.Step(
                state,
                car,
                tires,
                PhysicsInput(new DriverInput(overdriven, 0f)),
                1f / 60f
            );
            maximumSideslip = MathF.Max(
                maximumSideslip,
                state.SideslipAngleRadians
            );
            maximumYawDeficit = MathF.Max(
                maximumYawDeficit,
                state.Telemetry.ReferenceYawRateRadiansPerSecond -
                state.YawRateRadiansPerSecond
            );
        }

        // Understeer is a yaw rate deficit and nothing else. It used to
        // show up as body sideslip too, because sideslip was the leftover
        // between requested and delivered yaw; with real slip angles the
        // body sits nose-in through any corner at speed, understeering or
        // not, so the sign of the sideslip says nothing about the balance.
        Assert.True(
            maximumYawDeficit > 0.05f,
            $"front saturation should produce less yaw rate than requested, " +
            $"deficit was {maximumYawDeficit:0.000}"
        );
        Assert.True(
            maximumSideslip < 0.3f,
            "and should not be a spin"
        );
    }

    [Fact]
    public void TheGripLimiterCutsDriveNearTheCombinedGripLimit()
    {
        // On a rear-driven car, where all the drive meets the rear's corner
        // on one axle: the limiter is what is under test, not the drivetrain.
        // Drive the rear cannot take on top of the corner. At this speed
        // what the rear has left after the corner is more than the stock
        // car's 12 m/s^2 of drive, so its pedal never reaches the circle;
        // the limiter is under test, not the motor's ceiling.
        CarConfig controlledCar = new() { FrontDriveShare = 0f, MaxDriveAcceleration = UncappedDriveAcceleration };
        CarConfig uncontrolledCar = new() { FrontDriveShare = 0f, CombinedGripLimiterStrength = 0f, MaxDriveAcceleration = UncappedDriveAcceleration };
        TestTires tires = WarmTires();
        CarState controlled = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarState uncontrolled = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarStrategy attack = new(TireUsageMode.Protect, PowerOutputMode.Attack);
        // Cornering hard and asking for drive on top, which is where the
        // limiter is meant to step in.
        float curvature = CurvatureForGripShare(controlled, car: controlledCar,
            tires, attack, 1.0f);
        SetSteadyCorner(controlled, controlledCar, tires, curvature);
        SetSteadyCorner(uncontrolled, uncontrolledCar, tires, curvature);
        float drive = controlledCar.MaxDriveAcceleration;
        DriverInput asked = new(curvature, drive);
        CarPhysicsStepInput input = PhysicsInput(asked, attack);
        SetSteadyCorner(controlled, controlledCar, tires, curvature);
        SetSteadyCorner(uncontrolled, uncontrolledCar, tires, curvature);

        CarPhysics.Step(controlled, controlledCar, tires, input, 1f / 60f);
        CarPhysics.Step(uncontrolled, uncontrolledCar, tires, input, 1f / 60f);

        Assert.True(controlled.Axles().CombinedGripLimiterCutAccel > 0f);
        Near.Equal(0f, uncontrolled.Axles().CombinedGripLimiterCutAccel);
        Assert.True(
            controlled.Axles().RearLongitudinalUse < uncontrolled.Axles().RearLongitudinalUse,
            "the limiter should reduce rear drive use before the physics grip clip"
        );
        Assert.True(
            controlled.Telemetry.ActualLongitudinalAccel < uncontrolled.Telemetry.ActualLongitudinalAccel,
            "the limiter should trade acceleration for rear stability"
        );
    }

    [Fact]
    public void SideslipDissipatesSpeedAndHeatsRearTiresWhileTheLimiterIntervenes()
    {
        // Drive the rear cannot take on top of the corner. At this speed
        // what the rear has left after the corner is more than the stock
        // car's 12 m/s^2 of drive, so its pedal never reaches the circle;
        // the limiter is under test, not the motor's ceiling.
        CarConfig car = new() { MaxDriveAcceleration = UncappedDriveAcceleration };
        TestTires tires = WarmTires();
        CarState aligned = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarState sliding = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        // Most of the way to the rear's peak slip angle (0.14), so the
        // corner already takes nearly all of its circle.
        sliding.Heading = -0.12f;
        sliding.SideslipAngleRadians = 0.12f;
        CarStrategy attack = new(TireUsageMode.Normal, PowerOutputMode.Attack);
        float curvature = CurvatureForGripShare(aligned, car, tires, attack, 0.9f);
        float drive = DriveForShare(aligned, car, tires, attack, curvature, 1.5f);
        CarPhysicsStepInput input =
            PhysicsInput(new DriverInput(curvature, drive), attack);

        CarPhysics.Step(aligned, car, tires, input, 1f / 60f);
        CarPhysics.Step(sliding, car, tires, input, 1f / 60f);

        Assert.True(sliding.Axles().CombinedGripLimiterCutAccel > 0f);
        Assert.True(sliding.Axles().InducedDragAccel > 0.5f);
        Assert.True(
            sliding.Telemetry.ActualLongitudinalAccel < aligned.Telemetry.ActualLongitudinalAccel,
            "existing lateral slip should dissipate speed independently of the limiter"
        );
        Assert.True(
            AverageRearSurfaceTemp(sliding) > AverageRearSurfaceTemp(aligned),
            "sideslip should add heat at the rear tires"
        );
        Assert.True(
            AverageRearWear(sliding) > AverageRearWear(aligned),
            "sideslip should add rear tire wear"
        );
    }

    [Fact]
    public void TireUsageModeDoesNotChangePhysicsForSameDriverInput()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState protect = CreateState(speed: 37f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 37f, batterySoc: 0.8f, tires);
        DriverInput input = new(0.014f, 0f);

        StepMany(protect, car, tires, input, new CarStrategy(TireUsageMode.Protect, PowerOutputMode.Normal), steps: 240);
        StepMany(attack, car, tires, input, new CarStrategy(TireUsageMode.Attack, PowerOutputMode.Normal), steps: 240);

        Near.Equal(
            AverageSurfaceTemp(protect),
            AverageSurfaceTemp(attack));
        Near.Equal(AverageWear(protect), AverageWear(attack), abs: 0f);
        Near.Equal(
            protect.Axles().FrontGripAccel,
            attack.Axles().FrontGripAccel);
    }

    private static (float Front, float Rear) AxleDrive(CarState state) =>
        (state.Axles().FrontLongitudinalUse * state.Axles().FrontGripAccel,
         state.Axles().RearLongitudinalUse * state.Axles().RearGripAccel);

    [Fact]
    public void ARearDrivenCarPutsAllItsDriveOnTheRear()
    {
        CarConfig car = new() { FrontDriveShare = 0f };
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 20f, batterySoc: 0.8f, tires);
        StepMany(state, car, tires, new DriverInput(0f, 6f), CarStrategy.Default, steps: 5);
        (float front, float rear) = AxleDrive(state);
        Near.Equal(0f, front);
        Assert.True(rear > 0f);
    }

    [Fact]
    public void BothMotorsDriveInAStraightLine()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 20f, batterySoc: 0.8f, tires);
        StepMany(state, car, tires, new DriverInput(0f, 6f), CarStrategy.Default, steps: 5);
        (float front, float rear) = AxleDrive(state);
        Assert.True(front > 0.5f, $"front drive {front:F2}");
        Assert.True(rear > 0.5f, $"rear drive {rear:F2}");
    }

    private static float AxleUse(float lateral, float longitudinal) =>
        MathF.Sqrt(lateral * lateral + longitudinal * longitudinal);

    /// <summary>
    /// In a corner the drive goes to whichever axle has grip left over, so the
    /// busier axle ends up with more room than a fixed split would leave it.
    /// </summary>
    [Fact]
    public void InACornerTheDriveGoesWhereTheGripIsLeft()
    {
        TestTires tires = WarmTires();
        float Busiest(bool adaptive)
        {
            CarConfig car = new() { AdaptiveDriveSplit = adaptive };
            CarState state = CreateState(speed: 30f, batterySoc: 0.8f, tires);
            float curvature = CurvatureForGripShare(state, car, tires, 0.8f);
            SetSteadyCorner(state, car, tires, curvature);
            CarPhysics.Step(state, car, tires, PhysicsInput(new DriverInput(curvature, 3f)), 1f / 60f);
            AxleTelemetry tel = state.Axles();
            return MathF.Max(AxleUse(tel.FrontLateralUse, tel.FrontLongitudinalUse),
                AxleUse(tel.RearLateralUse, tel.RearLongitudinalUse));
        }
        float smart = Busiest(true), fixedSplit = Busiest(false);
        Assert.True(smart < fixedSplit, $"busiest axle {smart:F3} with the smart split against {fixedSplit:F3} fixed");
    }

    /// <summary>
    /// At full power each axle gets no more than its own motor: the front
    /// motor is 42 per cent of the car's peak, whatever the grip would allow.
    /// </summary>
    [Fact]
    public void NeitherAxleTakesMoreThanItsMotorCanGive()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 70f, batterySoc: 0.9f, tires);
        StepMany(state, car, tires, new DriverInput(0f, car.MaxDriveAcceleration), CarStrategy.Default, steps: 3);
        (float front, float rear) = AxleDrive(state);
        Assert.True(front / (front + rear) <= car.FrontDriveShare + 0.01f,
            $"front share {front / (front + rear):F3} against a {car.FrontDriveShare:F2} motor");
        Assert.True(rear > front);
    }

    [Fact]
    public void TheRangeExtenderDrivesOnItsPackAndRefillsItFromFuel()
    {
        CarConfig car = new();
        var powertrain = (RangeExtenderPowertrain)car.Powertrain;
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 30f, batterySoc: 0.5f, tires);
        float startFuel = state.Energy.Secondary;

        // Flat out for a few seconds: more than the generator makes, so the
        // pack runs down while the fuel burns.
        StepMany(state, car, tires, new DriverInput(0f, car.MaxDriveAcceleration), CarStrategy.Default, steps: 180);
        Assert.True(state.Energy.Secondary < startFuel, "the generator should burn fuel");
        float afterPush = state.Energy.Primary;
        Assert.True(afterPush < 0.5f, "full power should draw the pack down faster than the generator fills it");

        // Coasting: the generator alone refills the pack.
        StepMany(state, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 180);
        Assert.True(state.Energy.Primary > afterPush, "the generator should refill the pack while the motors rest");

        // Burnt fuel is mass the car no longer carries.
        Assert.True(CarPhysics.TotalMassKg(car, state.Energy) < car.MassKg + powertrain.FuelAllowanceKg);
    }

    [Fact]
    public void AttackBatteryModeAcceleratesMoreButSpendsMoreEnergyThanSave()
    {
        // The all-electric car's output ladder caps power by rung. The range
        // extender's race power is one figure; its rungs are driving.
        CarConfig car = new() { Powertrain = ElectricPowertrain.Default };
        TestTires tires = WarmTires();
        CarState save = CreateState(speed: 50f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 50f, batterySoc: 0.8f, tires);
        DriverInput input = new(0f, car.MaxDriveAcceleration);

        StepMany(save, car, tires, input, new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Save), steps: 60);
        StepMany(attack, car, tires, input, new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Attack), steps: 60);

        Assert.True(attack.Speed > save.Speed, "attack battery mode should produce more straight-line speed");
        Assert.True(attack.Energy.Primary < save.Energy.Primary, "attack battery mode should spend more energy");
        Assert.True(attack.Telemetry.DrivePowerWatts > save.Telemetry.DrivePowerWatts, "attack mode should report higher drive power");
    }

    [Fact]
    /// <summary>
    /// The slider lands between the named settings rather than on the nearest
    /// one, and a figure asked for directly is honoured.
    ///
    /// Only the parts that could be wrong. Checking that each named setting
    /// returns the number written beside it in the configuration is checking
    /// that a table is its own contents: it can fail for one reason, that
    /// somebody deliberately changed the number, and it reports that as a
    /// broken test.
    /// </summary>
    public void DrivePowerFallsBetweenTheNamedSettings()
    {
        ElectricPowertrain powertrain = ElectricPowertrain.Default;
        float eco = powertrain.GetDrivePowerLimitWatts(PowerOutputMode.Eco);
        float normal = powertrain.GetDrivePowerLimitWatts(PowerOutputMode.Normal);
        float between = powertrain.GetDrivePowerLimitWatts(0.375f);

        Assert.InRange(between, MathF.Min(eco, normal), MathF.Max(eco, normal));
        Assert.Equal((eco + normal) * 0.5f, between, precision: 1);
    }

    [Fact]
    public void BatteryModesShareLowSpeedAccelerationAndEfficiency()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState save = CreateState(speed: 10f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 10f, batterySoc: 0.8f, tires);
        DriverInput input = new(0f, 7f);

        CarPhysics.Step(
            save,
            car,
            tires,
            PhysicsInput(input, new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Save)),
            1f / 60f
        );
        CarPhysics.Step(
            attack,
            car,
            tires,
            PhysicsInput(input, new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Attack)),
            1f / 60f
        );

        Near.Equal(
            save.Telemetry.RequestedLongitudinalAccel,
            attack.Telemetry.RequestedLongitudinalAccel);
        Near.Equal(save.Energy.Primary, attack.Energy.Primary);
    }

    [Fact]
    public void LowBatterySocLimitsDriveOutput()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState full = CreateState(speed: 26f, batterySoc: 0.8f, tires);
        // Inside the last sliver of the usable window, where output runs from
        // the sag down to the limp: the sag alone (85 per cent at worst) does
        // not bite at this speed and request, so the test sits below it.
        CarState low = CreateState(speed: 26f, batterySoc: 0.002f, tires);
        CarStrategy strategy = new(TireUsageMode.Normal, PowerOutputMode.Attack);
        DriverInput input = new(0f, 7f);

        CarPhysics.Step(full, car, tires, PhysicsInput(input, strategy), 1f / 60f);
        CarPhysics.Step(low, car, tires, PhysicsInput(input, strategy), 1f / 60f);

        Assert.True(
            low.Telemetry.RequestedLongitudinalAccel < full.Telemetry.RequestedLongitudinalAccel,
            "low SOC should reduce the drive acceleration accepted from the battery envelope"
        );
        Assert.True(
            low.Telemetry.DrivePowerWatts < full.Telemetry.DrivePowerWatts,
            "low SOC should reduce actual drive power"
        );
    }

    [Fact]
    public void TireUsageModeDoesNotChangeAvailablePhysicalGrip()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState protect = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        DriverInput input = new(0.018f, 0f);

        CarPhysics.Step(protect, car, tires, PhysicsInput(input, new CarStrategy(TireUsageMode.Protect, PowerOutputMode.Normal)), 1f / 60f);
        CarPhysics.Step(attack, car, tires, PhysicsInput(input, new CarStrategy(TireUsageMode.Attack, PowerOutputMode.Normal)), 1f / 60f);

        Near.Equal(
            protect.Telemetry.ActualCurvature,
            attack.Telemetry.ActualCurvature);
        Near.Equal(
            protect.Axles().FrontGripAccel,
            attack.Axles().FrontGripAccel);
    }

    [Fact]
    public void HotWornTiresDeliverLessGripThanFreshWarmTires()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState fresh = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        CarState worn = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        MakeTiresHotAndWorn(worn);
        DriverInput input = new(0.018f, 0f);

        CarPhysics.Step(fresh, car, tires, PhysicsInput(input), 1f / 60f);
        CarPhysics.Step(worn, car, tires, PhysicsInput(input), 1f / 60f);

        Assert.True(worn.Axles().FrontGripAccel < fresh.Axles().FrontGripAccel, "hot worn front tires should have less grip");
        Assert.True(worn.Axles().RearGripAccel < fresh.Axles().RearGripAccel, "hot worn rear tires should have less grip");
        Assert.True(
            Math.Abs(worn.Telemetry.ActualCurvature) < Math.Abs(fresh.Telemetry.ActualCurvature),
            "hot worn tires should deliver less curvature under the same request"
        );
    }

    [Fact]
    public void SurfaceTemperatureGripUsesAFlatWindowAndSmoothOuterCurve()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState cold = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState lowIdeal = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState middleIdeal = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState highIdeal = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState hotNear = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState hotMiddle = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState hotFar = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        // Read off the compound rather than written down, so moving the window
        // moves the test with it instead of leaving it asserting about a band
        // no tyre has any more.
        float bandLow = tires.Config.IdealSurfaceTempLowC;
        float bandHigh = tires.Config.IdealSurfaceTempHighC;
        SetTireTemps(cold, surfaceTempC: bandLow - 10f, coreTempC: 90f);
        SetTireTemps(lowIdeal, surfaceTempC: bandLow, coreTempC: 90f);
        SetTireTemps(
            middleIdeal,
            surfaceTempC: (bandLow + bandHigh) * 0.5f,
            coreTempC: 90f
        );
        SetTireTemps(highIdeal, surfaceTempC: bandHigh, coreTempC: 90f);
        SetTireTemps(hotNear, surfaceTempC: bandHigh + 5f, coreTempC: 90f);
        SetTireTemps(hotMiddle, surfaceTempC: bandHigh + 10f, coreTempC: 90f);
        SetTireTemps(hotFar, surfaceTempC: bandHigh + 15f, coreTempC: 90f);

        foreach (CarState state in new[]
                 {
                     cold,
                     lowIdeal,
                     middleIdeal,
                     highIdeal,
                     hotNear,
                     hotMiddle,
                     hotFar
                 })
        {
            CarPhysics.Step(
                state,
                car,
                tires,
                PhysicsInput(new DriverInput(0f, 0f)),
                1f / 120f
            );
        }

        Near.Equal(
            lowIdeal.Axles().FrontGripAccel,
            middleIdeal.Axles().FrontGripAccel);
        Near.Equal(
            middleIdeal.Axles().FrontGripAccel,
            highIdeal.Axles().FrontGripAccel);
        Assert.True(
            cold.Axles().FrontGripAccel < lowIdeal.Axles().FrontGripAccel
        );
        float hotNearLoss = highIdeal.Axles().FrontGripAccel -
                            hotNear.Axles().FrontGripAccel;
        float hotMiddleLoss = highIdeal.Axles().FrontGripAccel -
                              hotMiddle.Axles().FrontGripAccel;
        float hotFarLoss = highIdeal.Axles().FrontGripAccel -
                           hotFar.Axles().FrontGripAccel;
        Assert.True(hotNearLoss > 0f);
        Assert.True(hotMiddleLoss > hotNearLoss);
        Assert.True(hotFarLoss > hotMiddleLoss);
    }

    /// <summary>
    /// Warm rubber wears faster: from the reference temperature up, wear
    /// grows by a factor of e for every <see cref="TireConfig.WearTempScaleC"/>
    /// degrees of tread, inside the window as well as past it, and stops at
    /// <see cref="TireConfig.MaximumHotWearFactor"/>, where the tread is
    /// blistering whatever the gauge says.
    /// </summary>
    [Fact]
    public void HotTreadWearClimbsWithTemperatureAndIsCapped()
    {
        CarConfig car = new() { Aero = new StandardAero { DownforcePerSpeedSquared = 0f } };
        // The temperature multiplies every kind of wear alike, so the same
        // corner at three treads compares the factor alone.
        TestTires tires = new(new TireConfig(), 90f, 90f);
        float reference = TireConfig.WearReferenceTempC;
        CarState atReference = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState warmer = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState scorching = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        SetTireTemps(atReference, reference, reference);
        SetTireTemps(warmer, reference + TireConfig.WearTempScaleC, reference + TireConfig.WearTempScaleC);
        SetTireTemps(scorching, reference + 8f * TireConfig.WearTempScaleC, reference + 8f * TireConfig.WearTempScaleC);

        const float gripShare = 0.6f;
        const float dt = 1f / 600f;
        foreach (CarState state in new[] { atReference, warmer, scorching })
        {
            float curvature = CurvatureForGripShare(state, car, tires, gripShare);
            SetSteadyCorner(state, car, tires, curvature);
            StepAtMatchingAmbient(state, car, tires, curvature, dt);
        }

        float baseline = AverageWear(atReference);
        Assert.True(baseline > 0f);
        Assert.InRange(AverageWear(warmer) / baseline, MathF.E * 0.85f, MathF.E * 1.15f);
        Assert.InRange(AverageWear(scorching) / baseline, TireConfig.MaximumHotWearFactor * 0.8f, TireConfig.MaximumHotWearFactor * 1.2f);
    }

    [Fact]
    public void WearRisesFasterThanSquaredDemandNearTheLimit()
    {
        CarConfig car = new() { Aero = new StandardAero { DownforcePerSpeedSquared = 0f } };
        TestTires tires = new(new TireConfig(), 90f, 90f);
        CarState moderate = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState nearLimit = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        float moderateCurvature = CurvatureForGripShare(
            moderate, car, tires, 0.35f);
        float nearLimitCurvature = CurvatureForGripShare(
            nearLimit, car, tires, 1.0f);
        SetSteadyCorner(moderate, car, tires, moderateCurvature);
        SetSteadyCorner(nearLimit, car, tires, nearLimitCurvature);

        CarPhysics.Step(
            moderate,
            car,
            tires,
            PhysicsInput(new DriverInput(moderateCurvature, 0f)),
            1f / 60f
        );
        CarPhysics.Step(
            nearLimit,
            car,
            tires,
            PhysicsInput(new DriverInput(nearLimitCurvature, 0f)),
            1f / 60f
        );

        float moderateUseSquared = AverageAxleLateralUseSquared(moderate);
        float nearLimitUseSquared = AverageAxleLateralUseSquared(nearLimit);
        float moderateWearPerSquaredUse =
            AverageWear(moderate) / moderateUseSquared;
        float nearLimitWearPerSquaredUse =
            AverageWear(nearLimit) / nearLimitUseSquared;
        // Still faster than squared, and measurably less so than it used
        // to be. The near-limit abrasion term goes as the eighth power of
        // combined use, and the car can no longer reach the last sixth of
        // its friction circle - it tops out near 0.84 where it used to sit
        // at 0.98 - so that term now contributes about a third of what it
        // did. Real, and worth knowing before the next stint-wear number
        // is compared with an old one.
        Assert.True(
            nearLimitWearPerSquaredUse > moderateWearPerSquaredUse * 1.05f,
            $"high utilisation should still represent growing partial-slip " +
            $"abrasion: {moderateWearPerSquaredUse:E3} then " +
            $"{nearLimitWearPerSquaredUse:E3}"
        );
    }

    [Fact]
    public void EqualTireUseAtHigherSpeedAccumulatesMoreWorkBasedWear()
    {
        // No wings, because the premise is that the same lateral acceleration
        // is the same demand on the tyre at either speed, and downforce breaks
        // exactly that: at speed the tyre is pressed down harder, so the same
        // acceleration is a smaller share of what it can give. This is about
        // the tyre model, so the car is the one where speed changes nothing
        // else.
        CarConfig car = new() { Aero = new StandardAero { DownforcePerSpeedSquared = 0f } };
        TestTires tires = new(new TireConfig(), 90f, 90f);
        CarState slow = CreateState(speed: 15f, batterySoc: 0.8f, tires);
        CarState fast = CreateState(speed: 45f, batterySoc: 0.8f, tires);
        // Two fifths of the car's corner, the share six m/s^2 was when this
        // was written: much less and the steering's slip compensation fades
        // and the faster car falls short of what it was asked for.
        float targetLateralAccel = 0.4f * CarPhysics.EstimateCorneringLimit(
            slow, car, tires, CarStrategy.Default, slow.Speed);
        float slowCurvature = targetLateralAccel / (slow.Speed * slow.Speed);
        float fastCurvature = targetLateralAccel / (fast.Speed * fast.Speed);
        SetSteadyCorner(slow, car, tires, slowCurvature);
        SetSteadyCorner(fast, car, tires, fastCurvature);

        CarPhysics.Step(
            slow,
            car,
            tires,
            new CarPhysicsStepInput(
                new DriverInput(slowCurvature, 0f),
                CarStrategy.Default,
                AirTempC: 90f,
                TrackTempC: 90f
            ),
            1f / 60f
        );
        CarPhysics.Step(
            fast,
            car,
            tires,
            new CarPhysicsStepInput(
                new DriverInput(fastCurvature, 0f),
                CarStrategy.Default,
                AirTempC: 90f,
                TrackTempC: 90f
            ),
            1f / 60f
        );

        Assert.InRange(
            MathF.Abs(
                slow.Telemetry.ActualLateralAccel -
                fast.Telemetry.ActualLateralAccel
            ),
            0f,
            0.05f
        );
        Assert.True(
            AverageWear(fast) > AverageWear(slow) * 2.5f,
            "the speed proxy should approximate greater slip work at equal tire use"
        );
    }

    [Fact]
    public void WakeDownforceLossReducesHighSpeedLateralLimit()
    {
        CarConfig car = new();
        TestTires tires = new(new TireConfig
        {
        }, 90f, 90f);
        CarState cleanAir = CreateState(speed: 55f, batterySoc: 0.8f, tires);
        CarState dirtyAir = cleanAir.Clone();
        dirtyAir.WakeDownforceLoss = 0.1f;

        CarPerformanceLimits cleanLimits = CarPhysics.EstimatePerformanceLimits(
            cleanAir,
            car,
            tires,
            CarStrategy.Default,
            speed: cleanAir.Speed,
            curvature: 0f
        );
        CarPerformanceLimits dirtyLimits = CarPhysics.EstimatePerformanceLimits(
            dirtyAir,
            car,
            tires,
            CarStrategy.Default,
            speed: dirtyAir.Speed,
            curvature: 0f
        );

        Assert.True(
            dirtyLimits.LateralAccelerationLimit <
            cleanLimits.LateralAccelerationLimit,
            "turbulent wake must remove some of the high-speed aero grip"
        );
    }

    [Fact]
    public void AirVelocityDeficitAlsoReducesHighSpeedLateralLimit()
    {
        CarConfig car = new();
        TestTires tires = new(new TireConfig
        {
        }, 90f, 90f);
        CarState cleanAir = CreateState(speed: 55f, batterySoc: 0.8f, tires);
        // The wake's two faces are carried by separate fields: the body's
        // deficit trims drag and cooling, the surface deficit is what the
        // downforce model reads. A body-only tow must leave cornering alone;
        // the surface deficit must take it away.
        CarState bodyOnlyTow = cleanAir.Clone();
        bodyOnlyTow.AirVelocityDeficit = 0.08f;
        CarState surfaceWake = cleanAir.Clone();
        surfaceWake.DownforceVelocityDeficit = 0.08f;

        CarPerformanceLimits cleanLimits = CarPhysics.EstimatePerformanceLimits(
            cleanAir,
            car,
            tires,
            CarStrategy.Default,
            speed: cleanAir.Speed,
            curvature: 0f
        );
        CarPerformanceLimits bodyTowLimits = CarPhysics.EstimatePerformanceLimits(
            bodyOnlyTow,
            car,
            tires,
            CarStrategy.Default,
            speed: bodyOnlyTow.Speed,
            curvature: 0f
        );
        CarPerformanceLimits surfaceWakeLimits = CarPhysics.EstimatePerformanceLimits(
            surfaceWake,
            car,
            tires,
            CarStrategy.Default,
            speed: surfaceWake.Speed,
            curvature: 0f
        );

        Near.Equal(
            cleanLimits.LateralAccelerationLimit,
            bodyTowLimits.LateralAccelerationLimit);
        Assert.True(
            surfaceWakeLimits.LateralAccelerationLimit <
            cleanLimits.LateralAccelerationLimit,
            "the surface deficit is what must take cornering grip away"
        );
    }

    [Fact]
    public void AirVelocityDeficitReducesHighSpeedAeroLoss()
    {
        CarConfig car = new();
        TestTires tires = new(new TireConfig
        {
        }, 90f, 90f);
        CarState cleanAir = CreateState(speed: 55f, batterySoc: 0.8f, tires);
        CarState tow = cleanAir.Clone();
        tow.AirVelocityDeficit = 0.08f;

        CarPerformanceLimits cleanLimits = CarPhysics.EstimatePerformanceLimits(
            cleanAir,
            car,
            tires,
            CarStrategy.Default,
            speed: cleanAir.Speed,
            curvature: 0f
        );
        CarPerformanceLimits towLimits = CarPhysics.EstimatePerformanceLimits(
            tow,
            car,
            tires,
            CarStrategy.Default,
            speed: tow.Speed,
            curvature: 0f
        );

        Assert.True(
            towLimits.LossAcceleration < cleanLimits.LossAcceleration,
            "the follower should still gain straight-line tow while losing aero grip"
        );
    }

    [Fact]
    public void CorneringScrubSlowsCarEvenWithoutBrakeRequest()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState straight = CreateState(speed: 38f, batterySoc: 0.8f, tires);
        CarState cornering = CreateState(speed: 38f, batterySoc: 0.8f, tires);

        StepMany(straight, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 120);
        StepMany(cornering, car, tires, new DriverInput(0.01f, 0f), CarStrategy.Default, steps: 120);

        Assert.True(cornering.Speed < straight.Speed, "cornering scrub should cost more speed than straight coasting");
        Assert.True(cornering.Telemetry.LossAccel > straight.Telemetry.LossAccel, "cornering should report higher loss accel");
    }

    [Fact]
    public void HighSpeedStraightCoastProducesVisibleDeceleration()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 60f, batterySoc: 0.8f, tires);

        StepMany(state, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 300);

        Assert.True(state.Speed < 55.5f, "five seconds of zero-throttle coasting should lose visible speed");
        Assert.True(state.Telemetry.ActualLongitudinalAccel < -1.3f, "high-speed drag should keep slowing the car");
        Near.Equal(-state.Telemetry.LossAccel, state.Telemetry.ActualLongitudinalAccel);
        Near.Equal(0f, state.Telemetry.DrivePowerWatts, abs: 1f);
    }

    [Fact]
    public void RepresentativeRunningKeepsWarmTiresInABoundedWindow()
    {
        CarConfig car = new();
        TestTires tires = new(new TireConfig
        {
        }, 86f, 84f);
        CarState state = CreateState(speed: 36f, batterySoc: 0.8f, tires);
        const float dt = 1f / 60f;

        // A lap's worth of ordinary running, as shares of what this car has:
        // a corner at under half its grip, a straight with a quarter of the
        // drive, a braking zone at a fifth of the brakes into a corner at
        // two fifths. The shares are what the old fixture's numbers came to
        // on the car they were written for.
        CarStrategy strategy = CarStrategy.Default;
        for (int step = 0; step < 120 * 60; step++)
        {
            int phase = step % (10 * 60);
            DriverInput input;
            if (phase < 4 * 60)
            {
                state.Speed = 36f;
                input = new DriverInput(CurvatureForGripShare(state, car, tires, 0.46f), 0f);
            }
            else if (phase < 7 * 60)
            {
                state.Speed = 48f;
                float curvature = CurvatureForGripShare(state, car, tires, 0.09f);
                input = new DriverInput(curvature, DriveForShare(state, car, tires, strategy, curvature, 0.25f));
            }
            else
            {
                state.Speed = 42f;
                float curvature = CurvatureForGripShare(state, car, tires, 0.38f);
                input = new DriverInput(curvature, BrakeForShare(state, car, tires, strategy, curvature, 0.19f));
            }

            state.Energy = PowertrainState.Filled(0.8f);
            CarPhysics.Step(state, car, tires, PhysicsInput(input), dt);
        }

        float surface = AverageSurfaceTemp(state);
        float core = AverageCoreTemp(state);
        Assert.InRange(surface, 70f, 110f);
        Assert.InRange(core, 70f, 105f);
    }

    [Fact]
    public void AxleLateralComplianceRedistributesHeatAndWearWithoutChangingTotalWork()
    {
        CarConfig equalCompliance = new()
        {
            FrontStaticLoadShare = 0.5f,
            FrontLateralComplianceRatio = 1f,
            CenterOfGravityHeightMeters = 0f,
            Aero = new StandardAero { DownforcePerSpeedSquared = 0f, DragPerSpeedSquared = 0f },
            RollingDragAccel = 0f,
        };
        CarConfig softerFront = new()
        {
            FrontStaticLoadShare = 0.5f,
            FrontLateralComplianceRatio = 2f,
            CenterOfGravityHeightMeters = 0f,
            Aero = new StandardAero { DownforcePerSpeedSquared = 0f, DragPerSpeedSquared = 0f },
            RollingDragAccel = 0f,
        };
        TestTires tires = new(new TireConfig(), 100f, 100f);
        CarState equal = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState frontWorkingHarder = CreateState(
            speed: 30f,
            batterySoc: 0.8f,
            tires
        );
        float curvature = CurvatureForGripShare(
            equal,
            equalCompliance,
            tires,
            share: 0.6f
        );
        SetSteadyCorner(equal, equalCompliance, tires, curvature);
        SetSteadyCorner(frontWorkingHarder, softerFront, tires, curvature);
        CarPhysicsStepInput input = new(
            new DriverInput(curvature, 0f),
            CarStrategy.Default,
            AirTempC: 100f,
            TrackTempC: 100f
        );

        CarPhysics.Step(equal, equalCompliance, tires, input, 1f / 60f);
        CarPhysics.Step(frontWorkingHarder, softerFront, tires, input, 1f / 60f);

        float equalFrontTemp = AverageFrontSurfaceTemp(equal);
        float equalRearTemp = AverageRearSurfaceTemp(equal);
        float shiftedFrontTemp = AverageFrontSurfaceTemp(frontWorkingHarder);
        float shiftedRearTemp = AverageRearSurfaceTemp(frontWorkingHarder);
        float equalFrontWear = AverageFrontWear(equal);
        float equalRearWear = AverageRearWear(equal);
        float shiftedFrontWear = AverageFrontWear(frontWorkingHarder);
        float shiftedRearWear = AverageRearWear(frontWorkingHarder);

        Assert.True(shiftedFrontTemp > equalFrontTemp);
        Assert.True(shiftedRearTemp < equalRearTemp);
        Assert.True(shiftedFrontWear > equalFrontWear);
        Assert.True(shiftedRearWear < equalRearWear);
        // An absolute band, not a count of decimal places. The two totals
        // sit near 200 C, where adjacent floats are 1.5e-5 apart, and a
        // decimal-place comparison fails whenever a one-bit difference
        // happens to straddle a rounding boundary -- which is what it did
        // the day an unrelated rear heat term was removed and the totals
        // moved. Three floats' worth of slack is the arithmetic; a real
        // leak would be the size of the shift itself, and the second
        // assertion keeps the band small against that.
        const float sumTolerance = 5e-5f;
        Assert.InRange(
            (shiftedFrontTemp + shiftedRearTemp) -
            (equalFrontTemp + equalRearTemp),
            -sumTolerance,
            sumTolerance
        );
        Assert.True(
            shiftedFrontTemp - equalFrontTemp > 10f * sumTolerance,
            "the redistribution must be large against the tolerance or the " +
            "conservation check is not checking anything"
        );
        Near.Equal(
            equalFrontWear + equalRearWear,
            shiftedFrontWear + shiftedRearWear, abs: 0f);
        Near.Equal(
            equal.Telemetry.ActualLateralAccel,
            frontWorkingHarder.Telemetry.ActualLateralAccel);
    }

    [Fact]
    public void NearZeroAxleContributionDoesNotDiscardTotalLateralWork()
    {
        CarConfig ordinaryCompliance = new()
        {
            FrontStaticLoadShare = 0.0001f,
            FrontLateralComplianceRatio = 1f,
            CenterOfGravityHeightMeters = 0f,
            Aero = new StandardAero { DownforcePerSpeedSquared = 0f, DragPerSpeedSquared = 0f },
            RollingDragAccel = 0f,
        };
        CarConfig extremeCompliance = new()
        {
            FrontStaticLoadShare = 0.0001f,
            FrontLateralComplianceRatio = 100_000_000f,
            CenterOfGravityHeightMeters = 0f,
            Aero = new StandardAero { DownforcePerSpeedSquared = 0f, DragPerSpeedSquared = 0f },
            RollingDragAccel = 0f,
        };
        TestTires tires = new(new TireConfig(), 100f, 100f);
        CarState ordinary = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState extreme = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        float curvature = CurvatureForGripShare(
            ordinary,
            ordinaryCompliance,
            tires,
            share: 0.6f
        );
        SetSteadyCorner(ordinary, ordinaryCompliance, tires, curvature);
        SetSteadyCorner(extreme, extremeCompliance, tires, curvature);
        CarPhysicsStepInput input = new(
            new DriverInput(curvature, 0f),
            CarStrategy.Default,
            AirTempC: 100f,
            TrackTempC: 100f
        );

        CarPhysics.Step(
            ordinary,
            ordinaryCompliance,
            tires,
            input,
            1f / 60f
        );
        CarPhysics.Step(
            extreme,
            extremeCompliance,
            tires,
            input,
            1f / 60f
        );

        Near.Equal(
            AverageFrontSurfaceTemp(ordinary) +
            AverageRearSurfaceTemp(ordinary),
            AverageFrontSurfaceTemp(extreme) +
            AverageRearSurfaceTemp(extreme));
        Near.Equal(
            AverageFrontWear(ordinary) + AverageRearWear(ordinary),
            AverageFrontWear(extreme) + AverageRearWear(extreme), abs: 0f);
    }

    [Fact]
    public void StraightRollingBuildsCoreHeatWithoutTireSlip()
    {
        CarConfig car = new();
        TestTires tires = new(new TireConfig
        {
        }, 25f, 25f);
        CarState state = CreateState(speed: 40f, batterySoc: 0.8f, tires);
        CarPhysicsStepInput input = new(
            new DriverInput(0f, 0f),
            CarStrategy.Default,
            AirTempC: 25f,
            TrackTempC: 25f
        );

        for (int step = 0; step < 30 * 60; step++)
            CarPhysics.Step(state, car, tires, input, 1f / 60f);

        Assert.True(
            AverageCoreTemp(state) > 25.25f,
            "cyclic tire deformation should warm the core even without commanded slip"
        );
    }

    [Fact]
    public void BrakeWorkHeatsTheCoreAtTheAxleThatActuallyDoesIt()
    {
        CarConfig car = new()
        {
            FrontStaticLoadShare = 0.5f,
            CenterOfGravityHeightMeters = 0f,
            Aero = new StandardAero { DownforcePerSpeedSquared = 0f }
        };
        TestTires tires = new(new TireConfig
        {
        }, 90f, 90f);
        CarState frontBiased = CreateState(speed: 50f, batterySoc: 0.8f, tires);
        CarState rearBiased = CreateState(speed: 50f, batterySoc: 0.8f, tires);

        CarPhysics.Step(
            frontBiased,
            car,
            tires,
            new CarPhysicsStepInput(
                new DriverInput(0f, -5f, FrontBrakeBiasOffset: 0.20f),
                CarStrategy.Default,
                AirTempC: 90f,
                TrackTempC: 90f
            ),
            1f / 60f
        );
        CarPhysics.Step(
            rearBiased,
            car,
            tires,
            new CarPhysicsStepInput(
                new DriverInput(0f, -5f, FrontBrakeBiasOffset: -0.20f),
                CarStrategy.Default,
                AirTempC: 90f,
                TrackTempC: 90f
            ),
            1f / 60f
        );

        Assert.True(
            AverageFrontCoreTemp(frontBiased) >
            AverageFrontCoreTemp(rearBiased)
        );
        Assert.True(
            AverageRearCoreTemp(frontBiased) <
            AverageRearCoreTemp(rearBiased)
        );
    }

    [Fact]
    public void CoreShedsHeatToTheAirFarMoreSlowlyThanTheTread()
    {
        CarConfig car = new();
        TestTires tires = new(new TireConfig
        {
        }, 100f, 100f);
        CarState state = CreateState(speed: 0f, batterySoc: 0.8f, tires);

        StepMany(
            state,
            car,
            tires,
            new DriverInput(0f, 0f),
            CarStrategy.Default,
            steps: 60
        );

        float treadDrop = 100f - AverageSurfaceTemp(state);
        float coreDrop = 100f - AverageCoreTemp(state);
        // The carcass is not sealed - it reaches the outside through the rim
        // and the gas inside - but it is buried, and the tread it is buried
        // under is lying on the road inside its own hurricane. Both cool; only
        // one of them cools at a rate a straight is long enough to notice.
        Assert.True(treadDrop > 0f);
        Assert.True(coreDrop > 0f);
        Assert.True(treadDrop > coreDrop * 10f);
    }

    [Fact]
    public void CoreTemperatureRespondsMoreSlowlyThanTreadSurface()
    {
        CarConfig car = new();
        TestTires tires = new(new TireConfig
        {
        }, 70f, 70f);
        CarState state = CreateState(speed: 36f, batterySoc: 0.8f, tires);
        // Working the tyre hard enough that the tread actually heats: the point
        // is which of the two follows the other, and a corner the tyre strolls
        // through leaves nothing for either of them to follow.
        float curvature = CurvatureForGripShare(
            state,
            car,
            tires,
            CarStrategy.Default,
            1f
        );

        // Air and road at the tyres' own temperature. The question is which
        // layer follows the other, and with the usual 25 C air a 70 C tyre
        // is shedding heat the whole time, so the net rise is the working
        // minus the cooling -- a number that went below this test's floor
        // when an unrelated rear heat term was removed, without the order
        // of the two layers changing at all. Holding the surroundings level
        // leaves only the heat going in.
        CarPhysicsStepInput input = new(
            new DriverInput(curvature, 0f),
            CarStrategy.Default,
            AirTempC: 70f,
            TrackTempC: 70f
        );
        for (int i = 0; i < 120; i++)
            CarPhysics.Step(state, car, tires, input, 1f / 60f);

        float surfaceRise = AverageSurfaceTemp(state) - 70f;
        float coreRise = AverageCoreTemp(state) - 70f;
        Assert.True(surfaceRise > 1f, $"the tread rose only {surfaceRise:F2} C");
        Assert.True(
            surfaceRise > coreRise * 4f,
            "the high-capacity core should not follow a short tread heat spike"
        );
    }

    /// <summary>
    /// A soaked core comes back down when the driving eases, and comes
    /// down further the more it eases.
    ///
    /// This replaces a test that asked something the model no longer
    /// answers. It used to contrast 95.5% use, which cooled a 103 C core
    /// below 100, against 100% use, which was supposed to hold it above
    /// 105 — and that cliff between them was not the tyre. It was a branch
    /// that switched on when *demand* crossed 99% of the friction circle,
    /// whether or not the rubber was actually sliding any faster. Heat is
    /// charged for sliding now rather than for asking, and between 95.5%
    /// and 100% of a limit the car is holding, the sliding barely changes.
    ///
    /// So the cliff is gone and what is left is a slope, which is what a
    /// tyre has. The separation asserted here is the one the model
    /// actually delivers.
    /// </summary>
    [Fact]
    public void AHeatSoakedCoreComesDownFurtherTheMoreTheDrivingEases()
    {
        CarState easy = RunRepresentativeCoreDutyCycle(0.60f);
        CarState hard = RunRepresentativeCoreDutyCycle(0.955f);

        float easyCore = AverageCoreTemp(easy);
        float hardCore = AverageCoreTemp(hard);

        Assert.True(
            easyCore < hardCore,
            $"easing off must cool the core further: {easyCore:F2} vs {hardCore:F2} C"
        );
        Assert.True(
            hardCore < 103f,
            $"five minutes of any sub-limit driving should draw a soaked 103 C core down, got {hardCore:F2} C"
        );
        Assert.True(
            hardCore - easyCore > 2f,
            $"the two duty cycles need a readable separation, got {easyCore:F2} versus {hardCore:F2} C"
        );
    }

    /// <summary>
    /// Gate two of the heat rework: a car held at a steady cornering load
    /// settles at a temperature instead of running away.
    ///
    /// The old heat source was bounded by construction — force share is at
    /// most one, so its square is too. Force times sliding has no such
    /// ceiling: past the peak the sliding keeps growing, and a source with
    /// no bound sitting inside a loop where temperature costs grip, and
    /// less grip means more slip, is exactly the shape a thermal runaway
    /// has. So it is checked rather than assumed. Cooling is proportional
    /// to the excess over ambient and the heat is not, which is what makes
    /// the fixed point exist, but that argument is worth precisely as much
    /// as the measurement behind it.
    /// </summary>
    [Fact]
    public void SteadyCorneringSettlesAtATemperatureInsteadOfRunningAway()
    {
        TestTires tires = new(new TireConfig
        {
        }, 90f, 90f);
        CarConfig car = new();
        CarState state = CreateState(speed: 55f, batterySoc: 1f, tires);
        float curvature = CurvatureForGripShare(
            state,
            car,
            tires,
            share: CalibratedCornerUse(0.90f)
        );
        SetSteadyCorner(state, car, tires, curvature);

        CarPhysicsStepInput input = new(
            new DriverInput(curvature, 0f),
            CarStrategy.Default,
            AirTempC: 35f,
            TrackTempC: 45f
        );

        float previous = 0f;
        float settled = 0f;
        for (int minute = 0; minute < 5; minute++)
        {
            for (int step = 0; step < 60 * 60; step++)
            {
                state.Speed = 55f;
                // Nine tenths of what the car can hold now, read again
                // each step as a driver reads it: a carcass past its
                // threshold takes grip away, and nine tenths of the grip
                // it had at the start is soon more than it has.
                curvature = CurvatureForGripShare(state, car, tires, share: CalibratedCornerUse(0.90f));
                input = input with { DriverInput = new DriverInput(curvature, 0f) };
                CarPhysics.Step(state, car, tires, input, 1f / 60f);
                // The heat balance is what this gate is about. Sixteen
                // kilometres of cornering at nine tenths wears a set of
                // tyres as it should, and a worn tyre runs at another
                // temperature; hold the wear so the drift that is left is
                // the thermal model's own.
                foreach (TireState tire in new[] { state.FrontLeft, state.FrontRight, state.RearLeft, state.RearRight })
                    tire.Wear = 0f;
            }
            previous = settled;
            settled = AverageFrontSurfaceTemp(state);
        }

        System.Console.WriteLine($"GATE2 settled {settled:0.00} prev {previous:0.00}");
        Assert.InRange(
            settled,
            tires.Config.IdealSurfaceTempLowC - 20f,
            tires.Config.IdealSurfaceTempHighC + 20f
        );
        // Converged, not merely finite: the last minute must move it less
        // than the resolution anyone would care about.
        Assert.True(
            MathF.Abs(settled - previous) < 0.5f,
            $"the temperature is still moving after five minutes: " +
            $"{previous:F2} then {settled:F2} C"
        );
    }

    /// <summary>
    /// Gate four: from cold, hard driving puts the tyres in their working
    /// range inside one to three laps.
    ///
    /// The anchor is outside the project — slicks with no blankets are in
    /// their window after an out lap or two, which is what an out lap is
    /// for. What the car did before this batch was reach 59 C after two
    /// laps against a working floor of 85, and arrive at about 1500
    /// seconds, which is fifteen laps. That was measured on the model
    /// before the heat rework too, to within a degree: a calibration the
    /// car always had, that nothing had asked about, because nothing until
    /// now wanted to start a car cold.
    ///
    /// The fix was one number — every thermal mass divided by the same
    /// factor — so that no equilibrium moved and only the journey to it
    /// got shorter. See <see cref="TireConfig.ThermalTimeScale"/>.
    ///
    /// Pinned on both sides, and the lower bound is the important one. A
    /// one-sided version of this test passed while the car, unable to grip
    /// on cold rubber, spun on entry and cooked itself to 225 C. Tyres
    /// that came up instantly would be just as wrong as tyres that never
    /// came up, and only a band says so.
    ///
    /// The one-to-three is for an out lap's mixture, driven at eight
    /// tenths. Held flat against what the car can hold, the tread starts
    /// to slide in part and heats much faster (TireConfig.NearLimitHeatRate),
    /// the way a track with more and harder corners warms its tyres sooner
    /// than one with fewer. So that case is held only to coming up faster
    /// than the out lap and not instantly.
    /// </summary>
    [Fact]
    public void ColdTyresReachTheirWorkingRangeInOneToThreeLaps()
    {
        // A lap of Silverstone at this pace, near enough for a bound
        // expressed in laps.
        const float lapSeconds = 101f;
        CarState outLap = RunColdOutLap(0.80f, seconds: 400, out int reached);
        CarState atLimit = RunColdOutLap(0.90f, seconds: 400, out int reachedAtLimit);

        Assert.True(
            reached >= 0 && reachedAtLimit >= 0,
            "the tyres never reached their working range at all; the last " +
            $"readings were {AverageFrontSurfaceTemp(outLap):F1} and " +
            $"{AverageFrontSurfaceTemp(atLimit):F1} C"
        );

        Assert.InRange(reached / lapSeconds, 1f, 3f);
        Assert.InRange(reachedAtLimit / lapSeconds, 0.1f, 3f);
        Assert.True(
            reachedAtLimit < reached,
            $"driving harder has to warm the tyres sooner: {reachedAtLimit} s at the limit against {reached} s"
        );
    }

    /// <summary>
    /// A cold out lap: hard cornering, braking and traction in the mixture
    /// a driver actually uses to put heat in, from twenty-five degrees.
    /// </summary>
    private static CarState RunColdOutLap(
        float use,
        int seconds,
        out int reachedSeconds
    )
    {
        TestTires tires = new(new TireConfig
        {
        }, 25f, 25f);
        CarConfig car = new();
        CarState state = CreateState(speed: 55f, batterySoc: 1f, tires);
        CarStrategy strategy = CarStrategy.Default;
        const int cycleSteps = 20 * 60;
        reachedSeconds = -1;

        for (int step = 0; step < seconds * 60; step++)
        {
            state.Speed = 55f;
            float phase = (step % cycleSteps) / 60f;
            CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
                state, car, tires, strategy, state.Speed, curvature: 0f
            );
            float curvature = phase < 2f
                ? CarPhysics.EstimateCorneringLimit(state, car, tires, strategy, state.Speed) * use /
                  (state.Speed * state.Speed)
                : 0f;
            float accel = phase switch
            {
                >= 2f and < 3f => -limits.MaximumBrakeDeceleration * use,
                >= 3f and < 5f => limits.MaximumDriveAcceleration * use,
                _ => 0f
            };
            CarPhysics.Step(
                state,
                car,
                tires,
                new CarPhysicsStepInput(
                    new DriverInput(curvature, accel),
                    strategy,
                    AirTempC: 25f,
                    TrackTempC: 30f
                ),
                1f / 60f
            );
            if (reachedSeconds < 0 &&
                AverageFrontSurfaceTemp(state) >= tires.Config.IdealSurfaceTempLowC)
            {
                reachedSeconds = step / 60;
            }
        }
        return state;
    }

    /// <summary>
    /// Sliding past the peak heats the tyre, where merely asking for more
    /// grip does not.
    ///
    /// This is the whole point of moving lateral heat onto force times
    /// sliding, and it is the same correction the wear side had: past the
    /// peak the force falls away, so anything charged as a share of force
    /// bills a car sliding at forty degrees *less* than one sitting neatly
    /// on the limit. Rubber does not work that way. The two readings meet
    /// at the peak and go opposite ways after it.
    /// </summary>
    [Theory]
    [InlineData(1.5f)]
    [InlineData(2.0f)]
    [InlineData(3.0f)]
    [InlineData(5.0f)]
    public void SlidingPastThePeakHeatsWhereAskingForGripDoesNot(float slipRatio)
    {
        static (float SlipWork, float ForceSquared) Readings(float ratio)
        {
            float slip = TireSlipCurve.PeakSlipAngleRadians * ratio;
            float force = TireSlipCurve.Evaluate(slip);
            float slipShare = MathF.Sin(slip) / MathF.Sin(TireSlipCurve.PeakSlipAngleRadians);
            return (force * slipShare, force * force);
        }

        (float atPeakWork, float atPeakSquared) = Readings(1f);
        (float slipWork, float forceSquared) = Readings(slipRatio);

        Assert.True(slipWork > atPeakWork, "sliding further past the peak has to cost more heat");
        Assert.True(forceSquared < atPeakSquared, "the force share falls past the peak");
        Assert.True(slipWork > forceSquared);
    }

    [Fact]
    public void StraightLineSpeedIncreasesTreadSurfaceCooling()
    {
        // No wings, so the only thing speed changes is the air moving over the
        // tyre, which is what this is about. With them the load at fifty five
        // metres a second is more than doubled and the heat that comes with it
        // swamps the cooling being measured.
        CarConfig car = new() { Aero = new StandardAero { DownforcePerSpeedSquared = 0f } };
        TestTires tires = WarmTires();
        CarState parked = CreateState(speed: 0f, batterySoc: 0.8f, tires);
        CarState fast = CreateState(speed: 55f, batterySoc: 0.8f, tires);
        SetTireTemps(parked, surfaceTempC: 112f, coreTempC: 104f);
        SetTireTemps(fast, surfaceTempC: 112f, coreTempC: 104f);

        // Twenty-four steps, not the hundred and twenty this used to run.
        //
        // The window has to sit inside the transient, and the transient got
        // five times shorter when the thermal time constant did. At two
        // seconds both cars are now most of the way to their own
        // equilibria, and at equilibrium the fast one is very slightly the
        // hotter of the two -- its rolling heat outweighs the extra air
        // moving over it -- so the reading flips sign for a reason that has
        // nothing to do with the claim being made. The claim is about the
        // cooling term, which is a rate, and a rate is visible while the
        // temperature is still moving.
        StepMany(parked, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 24);
        StepMany(fast, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 24);

        Assert.True(
            AverageSurfaceTemp(fast) < AverageSurfaceTemp(parked),
            "fast straight-line running should increase air cooling at the tire surface"
        );
    }

    [Fact]
    public void AirVelocityDeficitReducesForcedTireCooling()
    {
        // Hold every source of heat and every ground-speed effect equal. The
        // only physical difference is how much air the following tyre meets.
        CarConfig car = new()
        {
            Aero = new StandardAero { DownforcePerSpeedSquared = 0f, DragPerSpeedSquared = 0f },
            RollingDragAccel = 0f
        };
        TestTires tires = WarmTires();
        CarState cleanAir = CreateState(speed: 55f, batterySoc: 0.8f, tires);
        CarState wake = cleanAir.Clone();
        wake.AirVelocityDeficit = 0.08f;
        SetTireTemps(cleanAir, surfaceTempC: 112f, coreTempC: 104f);
        SetTireTemps(wake, surfaceTempC: 112f, coreTempC: 104f);
        CarPhysicsStepInput input = new(
            new DriverInput(0f, 0f),
            CarStrategy.Default,
            AirTempC: 25f,
            TrackTempC: 112f
        );

        for (int step = 0; step < 10 * 60; step++)
        {
            cleanAir.Speed = 55f;
            wake.Speed = 55f;
            CarPhysics.Step(cleanAir, car, tires, input, 1f / 60f);
            CarPhysics.Step(wake, car, tires, input, 1f / 60f);
        }

        Assert.True(
            AverageSurfaceTemp(wake) > AverageSurfaceTemp(cleanAir),
            "a tyre meeting less air at the same ground speed must retain more tread heat"
        );
        Assert.True(
            AverageCoreTemp(wake) > AverageCoreTemp(cleanAir),
            "reduced forced convection must also slow the much smaller core-to-air path"
        );
    }

    [Fact]
    public void WakeDownforceLossAddsOnlyCorneringSurfaceHeat()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState cleanCorner = CreateState(speed: 55f, batterySoc: 0.8f, tires);
        CarState dirtyCorner = cleanCorner.Clone();
        dirtyCorner.WakeDownforceLoss = 0.08f;
        float cleanCurvature = CurvatureForGripShare(
            cleanCorner,
            car,
            tires,
            share: 0.7f
        );
        float dirtyCurvature = CurvatureForGripShare(
            dirtyCorner,
            car,
            tires,
            share: 0.7f
        );
        SetSteadyCorner(cleanCorner, car, tires, cleanCurvature);
        SetSteadyCorner(dirtyCorner, car, tires, dirtyCurvature);

        CarState cleanStraight = CreateState(speed: 55f, batterySoc: 0.8f, tires);
        CarState dirtyStraight = cleanStraight.Clone();
        dirtyStraight.WakeDownforceLoss = 0.08f;
        CarPhysicsStepInput cleanCornerInput = PhysicsInput(
            new DriverInput(cleanCurvature, 0f)
        );
        CarPhysicsStepInput dirtyCornerInput = PhysicsInput(
            new DriverInput(dirtyCurvature, 0f)
        );
        CarPhysicsStepInput straightInput = PhysicsInput(
            new DriverInput(0f, 0f)
        );

        for (int step = 0; step < 10 * 60; step++)
        {
            cleanCorner.Speed = 55f;
            dirtyCorner.Speed = 55f;
            cleanStraight.Speed = 55f;
            dirtyStraight.Speed = 55f;
            CarPhysics.Step(
                cleanCorner,
                car,
                tires,
                cleanCornerInput,
                1f / 60f
            );
            CarPhysics.Step(
                dirtyCorner,
                car,
                tires,
                dirtyCornerInput,
                1f / 60f
            );
            CarPhysics.Step(cleanStraight, car, tires, straightInput, 1f / 60f);
            CarPhysics.Step(dirtyStraight, car, tires, straightInput, 1f / 60f);
        }

        Assert.True(
            AverageSurfaceTemp(dirtyCorner) > AverageSurfaceTemp(cleanCorner),
            "unresolved micro-slip in a loaded dirty-air corner must leave tread heat"
        );
        Assert.True(
            AverageSurfaceTemp(dirtyStraight) <=
            AverageSurfaceTemp(cleanStraight),
            "turbulent downforce loss alone must not invent heat on a straight"
        );
    }

    [Fact]
    public void BrakingWhileCorneringHasLessDeliveredBrakeThanStraightBraking()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState straight = CreateState(speed: 44f, batterySoc: 0.6f, tires);
        CarState cornering = CreateState(speed: 44f, batterySoc: 0.6f, tires);

        CarPhysics.Step(straight, car, tires, PhysicsInput(new DriverInput(0f, -10f)), 1f / 60f);
        CarPhysics.Step(cornering, car, tires, PhysicsInput(new DriverInput(0.018f, -10f)), 1f / 60f);

        Assert.True(
            cornering.Telemetry.RegenPowerWatts < straight.Telemetry.RegenPowerWatts,
            "cornering should leave less tire capacity for braking and therefore less regen power"
        );
        Assert.True(
            cornering.Telemetry.RegenPowerWatts > 0f,
            "cornering brake should remain continuous instead of being pre-clipped to zero"
        );
        Assert.True(
            Math.Abs(cornering.Telemetry.ActualLateralAccel) > Math.Abs(straight.Telemetry.ActualLateralAccel),
            "cornering case should still be doing lateral work"
        );
    }

    [Fact]
    public void HeavyCorneringBrakeStillAppliesSomeDeceleration()
    {
        CarConfig car = new();
        TestTires tires = WarmTires();
        CarState state = CreateState(speed: 44f, batterySoc: 0.6f, tires);
        // Enough brake that the axles cannot hold it, which is now the
        // only way to be past the circle: cornering force comes from slip
        // angles and can never exceed it.
        DriverInput heavy = new(0.03f, -35f);
        // Into the corner first, then the brake demand that is past what
        // the axles hold. Held rather than applied, the car would simply
        // spin - which is the model working, and not what this measures.
        SetSteadyCorner(state, car, tires, 0.03f);

        CarPhysics.Step(state, car, tires, PhysicsInput(heavy), 1f / 60f);

        Assert.True(
            state.Telemetry.OverLimit > 0f,
            "heavy cornering and braking together should have something past " +
            "what it has - a wheel asked to stop harder than it can, or rubber " +
            "dragged past the angle where it stops paying"
        );
        Assert.True(state.Telemetry.RegenPowerWatts > 0f, "brake request should still produce some braking work");
        Assert.True(state.Telemetry.ActualLongitudinalAccel < 0f, "net acceleration should be braking, not coasting");
        Assert.True(
            Math.Abs(state.Telemetry.ActualCurvature) < Math.Abs(state.Telemetry.Input.DesiredCurvature),
            "front axle clipping should turn excess trail braking into understeer"
        );
    }

    [Fact]
    public void FreshTireInstallResetsTireLedger()
    {
        TestTires tires = new(new TireConfig
        {
        }, 32f, 30f);
        CarState state = new();
        state.FrontLeft.SurfaceTempC = 118f;
        state.FrontLeft.CoreTempC = 104f;
        state.FrontLeft.Wear = 0.72f;
        state.FrontLeft.LoadN = 1234f;

        state.InstallFreshTires(tires);

        Near.Equal(32f, state.FrontLeft.SurfaceTempC);
        Near.Equal(30f, state.FrontLeft.CoreTempC);
        Near.Equal(0f, state.FrontLeft.Wear);
        Near.Equal(0f, state.FrontLeft.LoadN);
    }


    /// <summary>
    /// Put the car into the corner before measuring it.
    ///
    /// Every test here that is about the limit used to be able to start with
    /// the wheels straight, no yaw rate and no sideslip, and be at the limit
    /// one sixtieth of a second later - because cornering force was granted
    /// on request. It is generated from slip angles now, and slip angles take
    /// a moment to build behind a steering rack that can only move so fast,
    /// so one step from nothing measures a car that is barely turning.
    ///
    /// The speed is held at whatever the test named, because the condition a
    /// test names is a speed and a corner, and letting the car scrub its way
    /// out of the first while settling into the second measures neither.
    /// </summary>
    private static void EnterCorner(
        CarState state,
        CarConfig car,
        TireConfig tires,
        DriverInput input,
        CarStrategy strategy,
        int steps = 120
    )
    {
        float speed = state.Speed;
        for (int i = 0; i < steps; i++)
        {
            CarPhysics.Step(
                state, car, tires, PhysicsInput(input, strategy), 1f / 60f
            );
            state.Speed = speed;
        }
    }

    private static void EnterCorner(
        CarState state,
        CarConfig car,
        TireConfig tires,
        DriverInput input,
        int steps = 120
    ) => EnterCorner(state, car, tires, input, CarStrategy.Default, steps);

    /// <summary>A drive ceiling high enough that the tyres, not the motor, are what limits it.</summary>
    private const float UncappedDriveAcceleration = 30f;

    /// <summary>
    /// Braking into a fast corner, the rear is light. Asked for a little more
    /// corner than it can carry, the car turns at about the most it can: it
    /// does not wind the wheel off and run straight on. The steering's
    /// inversion once asked the rear for a slip angle at its peak and the
    /// front for little, and "front minus rear" turned the wheel the wrong
    /// way: here, 38 m/s^2 asked and 17 given, where 34 asked had given 27.
    /// </summary>
    [Fact]
    public void AskedPastTheRearsGripBrakingTheCarStillTurnsNearItsLimit()
    {
        var car = new CarConfig();
        TestTires tires = WarmTires();
        (float Asked, float Got) Corner(float share)
        {
            CarState state = CreateState(65f, 0.8f, tires);
            float settle = CurvatureForGripShare(state, car, tires, 0.5f);
            SetSteadyCorner(state, car, tires, settle);
            float curvature = CurvatureForGripShare(state, car, tires, share);
            float asked = 0f, got = 0f;
            for (int i = 0; i < 30; i++)
            {
                CarPhysics.Step(state, car, tires, PhysicsInput(new DriverInput(curvature, -25f)), 1f / 60f);
                if (i >= 10)
                {
                    asked += curvature * state.Speed * state.Speed;
                    got += MathF.Abs(state.Telemetry.ActualLateralAccel);
                }
            }
            return (asked / 20f, got / 20f);
        }
        var rows = new[] { 0.6f, 0.75f, 0.9f, 1.0f, 1.1f, 1.25f }.Select(Corner).ToArray();
        string table = string.Join("  ", rows.Select(r => $"{r.Asked:F1}->{r.Got:F1}"));
        // Asked for more, it never gives much less than it gave for less.
        for (int i = 1; i < rows.Length; i++)
            Assert.True(rows[i].Got >= 0.9f * rows[i - 1].Got, table);
    }

    private static CarState CreateState(float speed, float batterySoc, TestTires tires)
    {
        CarState state = new()
        {
            Speed = speed,
            Energy = PowertrainState.Filled(batterySoc)
        };
        state.InstallFreshTires(tires);
        return state;
    }

    /// <summary>
    /// A curvature that asks the tyres for a given share of the grip they have
    /// at the speed the car is doing.
    ///
    /// Every test below that is about what happens at the limit used to reach
    /// the limit by naming a speed and a curvature that happened to sit there
    /// for the car of the day. That is a fixture with an expiry date: give the
    /// car wings and the same numbers no longer trouble it, and a dozen tests
    /// fail saying only that something they expected to be true was not,
    /// having quietly stopped testing anything at all. Asked for as a share of
    /// the car's own limit, the same tests keep meaning what they meant on any
    /// car, with wings or without.
    /// </summary>
    private static float CurvatureForGripShare(
        CarState state,
        CarConfig car,
        TireConfig tires,
        CarStrategy strategy,
        float share
    )
    {
        float lateral = CarPhysics.EstimateCorneringLimit(
            state, car, tires, strategy, state.Speed) * share;
        return lateral / MathF.Max(state.Speed * state.Speed, 1e-3f);
    }

    private static float CurvatureForGripShare(
        CarState state,
        CarConfig car,
        TireConfig tires,
        float share
    )
    {
        return CurvatureForGripShare(state, car, tires, CarStrategy.Default, share);
    }

    /// <summary>Drive as a share of what the car can actually put down here.</summary>
    private static float DriveForShare(
        CarState state,
        CarConfig car,
        TireConfig tires,
        CarStrategy strategy,
        float curvature,
        float share
    )
    {
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            state,
            car,
            tires,
            strategy,
            state.Speed,
            curvature
        );
        return limits.MaximumDriveAcceleration * share;
    }

    /// <summary>Braking as a share of what the car can actually stop with here.</summary>
    private static float BrakeForShare(
        CarState state,
        CarConfig car,
        TireConfig tires,
        CarStrategy strategy,
        float curvature,
        float share
    )
    {
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            state,
            car,
            tires,
            strategy,
            state.Speed,
            curvature
        );
        return -limits.MaximumBrakeDeceleration * share;
    }

    /// <summary>
    /// Put the car into a steady corner before the measurement starts.
    ///
    /// This used to set the yaw rate and nothing else, which was the whole
    /// of a steady corner in the model it was written for: cornering force
    /// was granted on request, so a car with the right yaw rate was already
    /// doing everything a cornering car does. It is generated from slip
    /// angles now, and slip angles need the wheels turned, the body at an
    /// angle to its own path, and a moment for both to arrive.
    ///
    /// So the corner is rehearsed on a copy and only the attitude is
    /// brought back - the wheel angle, the sideslip, the yaw rate. The
    /// tyres the caller is about to measure stay exactly as fresh, as hot
    /// and as worn as the caller made them.
    /// </summary>
    private static void SetSteadyCorner(
        CarState state,
        CarConfig car,
        TireConfig tires,
        float curvature
    )
    {
        CarState rehearsal = state.Clone();
        float speed = rehearsal.Speed;
        for (int i = 0; i < 300; i++)
        {
            CarPhysics.Step(
                rehearsal,
                car,
                tires,
                PhysicsInput(new DriverInput(curvature, 0f)),
                1f / 60f
            );
            // The corner is rehearsed on the tyres the caller built, not on
            // whatever they would have become two and a half seconds later
            // at whatever ambient this helper happens to use. Their heat and
            // wear decide the grip, the grip decides the slip angles, and
            // the slip angles are the whole of what is being carried back.
            rehearsal.Speed = speed;
            rehearsal.FrontLeft.CopyFrom(state.FrontLeft);
            rehearsal.FrontRight.CopyFrom(state.FrontRight);
            rehearsal.RearLeft.CopyFrom(state.RearLeft);
            rehearsal.RearRight.CopyFrom(state.RearRight);
        }
        state.SteerAngleRadians = rehearsal.SteerAngleRadians;
        state.SideslipAngleRadians = rehearsal.SideslipAngleRadians;
        state.YawRateRadiansPerSecond = rehearsal.YawRateRadiansPerSecond;
        // The weight has to have moved too. Load transfer runs off these,
        // the wheel loads run off the transfer, and the grip runs off the
        // loads - so a car handed a corner's attitude with its weight still
        // sitting flat is a different car from the one that drove into it.
        state.FilteredLongitudinalAccel = rehearsal.FilteredLongitudinalAccel;
        state.FilteredLateralAccel = rehearsal.FilteredLateralAccel;
        state.Telemetry = rehearsal.Telemetry;
    }

    private static void StepAtMatchingAmbient(
        CarState state,
        CarConfig car,
        TireConfig tires,
        float curvature,
        float dt
    )
    {
        float temperature = state.FrontLeft.SurfaceTempC;
        CarPhysics.Step(
            state,
            car,
            tires,
            new CarPhysicsStepInput(
                new DriverInput(curvature, 0f),
                CarStrategy.Default,
                AirTempC: temperature,
                TrackTempC: temperature
            ),
            dt
        );
    }

    private static float AverageAxleLateralUseSquared(CarState state)
    {
        float front = state.Axles().FrontLateralUse;
        float rear = state.Axles().RearLateralUse;
        return (front * front + rear * rear) * 0.5f;
    }

    private static void MakeTiresHotAndWorn(CarState state)
    {
        foreach (TireState tire in Tires(state))
        {
            tire.SurfaceTempC = 130f;
            tire.CoreTempC = 122f;
            tire.Wear = 0.65f;
        }
    }

    private static void MakeRearTiresHotAndWorn(CarState state)
    {
        foreach (TireState tire in new[] { state.RearLeft, state.RearRight })
        {
            tire.SurfaceTempC = 140f;
            tire.CoreTempC = 130f;
            tire.Wear = 0.95f;
        }
    }

    private static void MakeFrontTiresHotAndWorn(CarState state)
    {
        foreach (TireState tire in new[] { state.FrontLeft, state.FrontRight })
        {
            tire.SurfaceTempC = 130f;
            tire.CoreTempC = 122f;
            tire.Wear = 0.65f;
        }
    }

    private static void SetTireTemps(CarState state, float surfaceTempC, float coreTempC)
    {
        foreach (TireState tire in Tires(state))
        {
            tire.SurfaceTempC = surfaceTempC;
            tire.CoreTempC = coreTempC;
        }
    }

    /// <summary>
    /// A share of the car's cornering limit as the tyres' heat was calibrated
    /// against it. The limit rose when the steering stopped giving a tenth of
    /// it away; the medium's heat has not been calibrated on the car as it
    /// now corners, so the heat tests ask its tyres for what they were asked
    /// when it was (TireConfig.NearLimitHeatLimitShare).
    /// </summary>
    private static float CalibratedCornerUse(float share) =>
        share * TireConfig.NearLimitHeatLimitShare / TireSlipCurve.SustainablePeakShare;

    private static CarState RunRepresentativeCoreDutyCycle(float use)
    {
        TestTires tires = new(new TireConfig
        {
        }, 107f, 103f);
        CarConfig car = new()
        {
            Aero = new StandardAero { DragPerSpeedSquared = 0f },
            RollingDragAccel = 0f,
        };
        CarState state = CreateState(speed: 55f, batterySoc: 1f, tires);
        CarStrategy strategy = CarStrategy.Default;
        const int cycleSteps = 20 * 60;
        const int totalSteps = 5 * 60 * 60;

        for (int step = 0; step < totalSteps; step++)
        {
            state.Speed = 55f;
            float phaseSeconds = (step % cycleSteps) / 60f;
            CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
                state,
                car,
                tires,
                strategy,
                state.Speed,
                curvature: 0f
            );
            float curvature = phaseSeconds < 2f
                ? CarPhysics.EstimateCorneringLimit(state, car, tires, strategy, state.Speed) * CalibratedCornerUse(use) /
                  (state.Speed * state.Speed)
                : 0f;
            float acceleration = phaseSeconds switch
            {
                >= 2f and < 3f =>
                    -limits.MaximumBrakeDeceleration * use,
                >= 3f and < 5f =>
                    limits.MaximumDriveAcceleration * use,
                _ => 0f
            };
            SetSteadyCorner(state, car, tires, curvature);
            CarPhysics.Step(
                state,
                car,
                tires,
                new CarPhysicsStepInput(
                    new DriverInput(curvature, acceleration),
                    strategy,
                    AirTempC: TestAirTempC,
                    TrackTempC: TestTrackTempC
                ),
                1f / 60f
            );
        }

        return state;
    }

    private static IEnumerable<TireState> Tires(CarState state)
    {
        yield return state.FrontLeft;
        yield return state.FrontRight;
        yield return state.RearLeft;
        yield return state.RearRight;
    }

    private static TestTires WarmTires()
    {
        return new(new TireConfig
        {
        }, 90f, 90f);
    }

    private static void StepMany(
        CarState state,
        CarConfig car,
        TireConfig tires,
        DriverInput input,
        CarStrategy strategy,
        int steps
    )
    {
        const float dt = 1f / 60f;
        for (int i = 0; i < steps; i++)
            CarPhysics.Step(state, car, tires, PhysicsInput(input, strategy), dt);
    }

    private static CarPhysicsStepInput PhysicsInput(DriverInput input)
    {
        return PhysicsInput(input, CarStrategy.Default);
    }

    private static CarPhysicsStepInput PhysicsInput(DriverInput input, CarStrategy strategy)
    {
        return new CarPhysicsStepInput(
            input,
            strategy,
            TestAirTempC,
            TestTrackTempC
        );
    }

    private static float AverageSurfaceTemp(CarState state)
    {
        return (
            state.FrontLeft.SurfaceTempC +
            state.FrontRight.SurfaceTempC +
            state.RearLeft.SurfaceTempC +
            state.RearRight.SurfaceTempC
        ) * 0.25f;
    }

    private static float AverageCoreTemp(CarState state)
    {
        return (
            state.FrontLeft.CoreTempC +
            state.FrontRight.CoreTempC +
            state.RearLeft.CoreTempC +
            state.RearRight.CoreTempC
        ) * 0.25f;
    }

    private static float AverageFrontCoreTemp(CarState state)
    {
        return (state.FrontLeft.CoreTempC + state.FrontRight.CoreTempC) * 0.5f;
    }

    private static float AverageRearCoreTemp(CarState state)
    {
        return (state.RearLeft.CoreTempC + state.RearRight.CoreTempC) * 0.5f;
    }

    private static float AverageRearSurfaceTemp(CarState state)
    {
        return (state.RearLeft.SurfaceTempC + state.RearRight.SurfaceTempC) * 0.5f;
    }

    private static float AverageFrontSurfaceTemp(CarState state)
    {
        return (state.FrontLeft.SurfaceTempC + state.FrontRight.SurfaceTempC) * 0.5f;
    }

    private static float AverageFrontWear(CarState state)
    {
        return (state.FrontLeft.Wear + state.FrontRight.Wear) * 0.5f;
    }

    private static float AverageRearWear(CarState state)
    {
        return (state.RearLeft.Wear + state.RearRight.Wear) * 0.5f;
    }

    private static float AverageWear(CarState state)
    {
        return (
            state.FrontLeft.Wear +
            state.FrontRight.Wear +
            state.RearLeft.Wear +
            state.RearRight.Wear
        ) * 0.25f;
    }
}
