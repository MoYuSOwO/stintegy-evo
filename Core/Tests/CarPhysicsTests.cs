using System;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class CarPhysicsTests
{
    private const float TestAirTempC = 25f;
    private const float TestTrackTempC = 35f;

    [Fact]
    public void DriveRequestIncreasesSpeedAndConsumesBattery()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 24f, batterySoc: 0.8f, tires);
        float startSpeed = state.Speed;
        float startSoc = state.BatterySoc;

        StepMany(state, car, tires, new DriverInput(0f, 4f), CarStrategy.Default, steps: 30);

        Assert.True(state.Speed > startSpeed, "speed should increase under positive drive request");
        Assert.True(state.BatterySoc < startSoc, "battery SOC should fall when drive power is used");
        Assert.True(state.Telemetry.DrivePowerWatts > 0f, "drive power should be reported");
        Assert.True(state.Telemetry.ActualLongitudinalAccel > 0f, "net longitudinal accel should remain positive");
    }

    [Fact]
    public void BrakeRequestSlowsCarAndRegeneratesEnergy()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 34f, batterySoc: 0.5f, tires);
        float startSpeed = state.Speed;
        float startSoc = state.BatterySoc;

        StepMany(state, car, tires, new DriverInput(0f, -7f), CarStrategy.Default, steps: 20);

        Assert.True(state.Speed < startSpeed, "speed should fall under brake request");
        Assert.True(state.BatterySoc > startSoc, "battery SOC should rise from regen during braking");
        Assert.True(state.Telemetry.RegenPowerWatts > 0f, "regen power should be reported");
        Assert.True(state.Telemetry.ActualLongitudinalAccel < 0f, "net longitudinal accel should be negative");
    }

    [Fact]
    public void LowGripHeavyBrakingIsClippedByAxles()
    {
        CarConfig car = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            BaseMu = 0.6f
        };
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
        CarConfig noEfficiencyLoss = new() { OverLimitMinGripEfficiency = 1f };
        CarConfig defaultEfficiencyLoss = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            BaseMu = 0.6f
        };
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
        TireConfig tires = WarmTires();
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
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 55f, batterySoc: 0.8f, tires);

        CarPhysics.Step(state, car, tires, PhysicsInput(new DriverInput(car.MaxCurvatureRequest, 0f)), 1f / 60f);

        Assert.True(state.Telemetry.OverLimit > 10f, "telemetry should still expose the raw excessive demand");
        Assert.True(state.Speed > 50f, "over-limit cost should saturate instead of deleting speed in one tick");
        Assert.True(
            Math.Abs(state.Telemetry.ActualCurvature) < car.MaxCurvatureRequest,
            "actual curvature should still be limited by available grip"
        );
    }

    [Fact]
    public void RearOverlimitCutsDeliveredDrive()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarStrategy strategy = new(TireUsageMode.Normal, BatteryOutputMode.Attack);
        DriverInput input = new(0.018f, 7f);

        CarPhysics.Step(state, car, tires, PhysicsInput(input, strategy), 1f / 60f);

        Assert.True(state.Telemetry.OverLimit > 0f, "combined demand should exceed available grip");
        Assert.True(
            state.Telemetry.ActualLongitudinalAccel < state.Telemetry.RequestedLongitudinalAccel,
            "delivered longitudinal acceleration should be lower after rear traction is cut"
        );
    }

    [Fact]
    public void RearCombinedSaturationScalesLateralAndDriveTogether()
    {
        CarConfig car = new() { TractionControlStrength = 0f };
        TireConfig tires = WarmTires();
        CarState cornering = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarState powered = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarStrategy attack = new(TireUsageMode.Normal, BatteryOutputMode.Attack);
        // Most of the grip spent on the corner, so asking for drive on top of
        // it has to come out of the same circle.
        float curvature = CurvatureForGripShare(cornering, car, tires, attack, 0.9f);
        SetSteadyYawRate(cornering, curvature);
        SetSteadyYawRate(powered, curvature);
        float drive = DriveForShare(powered, car, tires, attack, curvature, 1.5f);

        CarPhysics.Step(
            cornering,
            car,
            tires,
            PhysicsInput(new DriverInput(curvature, 0f), attack),
            1f / 60f
        );
        CarPhysics.Step(
            powered,
            car,
            tires,
            PhysicsInput(new DriverInput(curvature, drive), attack),
            1f / 60f
        );

        Assert.True(powered.Telemetry.OverLimit > 0f);
        Assert.True(
            Math.Abs(powered.Telemetry.ActualLateralAccel) <
            Math.Abs(cornering.Telemetry.ActualLateralAccel),
            "rear combined saturation should reduce lateral force instead of preserving it at all costs"
        );
        Assert.True(powered.Telemetry.RearLongitudinalUse > 0f);
    }

    [Fact]
    public void BrakeDistributionUsesEachAxlesRemainingFrictionCircle()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 25f, batterySoc: 0.9f, tires);
        state.SideslipAngleRadians = 0.04f;

        CarPhysics.Step(
            state,
            car,
            tires,
            PhysicsInput(new DriverInput(0.006f, -2f)),
            1f / 60f
        );

        Assert.InRange(state.Telemetry.OverLimit, 0f, 1e-5f);
        float frontRemainingUse = MathF.Sqrt(MathF.Max(
            0f,
            1f - state.Telemetry.FrontLateralUse *
                 state.Telemetry.FrontLateralUse
        ));
        float rearRemainingUse = MathF.Sqrt(MathF.Max(
            0f,
            1f - state.Telemetry.RearLateralUse *
                 state.Telemetry.RearLateralUse
        ));
        float frontCapacityFraction =
            state.Telemetry.FrontLongitudinalUse / frontRemainingUse;
        float rearCapacityFraction =
            state.Telemetry.RearLongitudinalUse / rearRemainingUse;
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
        TireConfig tires = WarmTires();
        CarState optimal = CreateState(speed: 25f, batterySoc: 0.9f, tires);
        CarState frontBiased = CreateState(speed: 25f, batterySoc: 0.9f, tires);
        // Braking hard enough into a corner that both axles are working, so a
        // split that favours one of them has to leave the other with grip
        // nobody is using.
        float curvature = CurvatureForGripShare(optimal, car, tires, 0.55f);
        // Just under, not over: braking pitches load forward, so the estimate
        // taken level is a little optimistic and asking for all of it puts the
        // balanced car over the limit too, which is the thing being contrasted.
        float brake = BrakeForShare(
            optimal, car, tires, CarStrategy.Default, curvature, 0.9f);

        CarPhysics.Step(
            optimal,
            car,
            tires,
            PhysicsInput(new DriverInput(curvature, brake)),
            1f / 60f
        );
        CarPhysics.Step(
            frontBiased,
            car,
            tires,
            PhysicsInput(new DriverInput(curvature, brake, 0.07f)),
            1f / 60f
        );

        Assert.InRange(optimal.Telemetry.OverLimit, 0f, 1e-5f);
        Assert.True(frontBiased.Telemetry.OverLimit > 0.02f);
        Assert.True(
            frontBiased.Telemetry.ActualLongitudinalAccel >
            optimal.Telemetry.ActualLongitudinalAccel + 0.1f,
            "a biased split should clip one axle before all remaining grip is used"
        );
        Assert.True(
            frontBiased.Telemetry.ActualLateralAccel <
            optimal.Telemetry.ActualLateralAccel,
            "excess front braking should trade away some front lateral force"
        );
    }

    [Fact]
    public void DefaultRearDriveMakesTheRearCloserToTheCombinedLimitUnderPower()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 30f, batterySoc: 0.9f, tires);
        SetSteadyYawRate(state, 0.008f);

        CarPhysics.Step(
            state,
            car,
            tires,
            PhysicsInput(new DriverInput(0.008f, 3f)),
            1f / 60f
        );

        float frontCombinedUse = MathF.Sqrt(
            state.Telemetry.FrontLateralUse * state.Telemetry.FrontLateralUse +
            state.Telemetry.FrontLongitudinalUse * state.Telemetry.FrontLongitudinalUse
        );
        float rearCombinedUse = MathF.Sqrt(
            state.Telemetry.RearLateralUse * state.Telemetry.RearLateralUse +
            state.Telemetry.RearLongitudinalUse * state.Telemetry.RearLongitudinalUse
        );
        Assert.True(
            rearCombinedUse > frontCombinedUse,
            "rear drive should move the rear axle closer to its combined limit under power"
        );
    }

    [Fact]
    public void RearAxleSaturationCreatesRecoverableBodySideslip()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        MakeRearTiresHotAndWorn(state);
        CarStrategy attack = new(TireUsageMode.Normal, BatteryOutputMode.Attack);
        // Cornering near what the worn rear will bear and then asking for all
        // the drive there is. Drive alone cannot do it: the request is clipped
        // at the car's own maximum, so the way to make an axle let go is to
        // have the corner already using most of its circle.
        float curvature = CurvatureForGripShare(state, car, tires, attack, 0.9f);
        float drive = car.MaxDriveAcceleration;

        StepMany(state, car, tires, new DriverInput(curvature, drive), attack, steps: 60);

        float builtSideslip = Math.Abs(state.SideslipAngleRadians);
        Assert.True(state.Telemetry.RearSlideSeverity > 0.1f, "rear saturation should expose slide severity");
        // The direction, not the size. How far the tail comes round is a
        // property of one car's grip, inertia and recovery rate, and pinning a
        // figure to it means the test stops being about rear saturation and
        // starts being about that car: give it wings and the same slide
        // settles at two thirds of the angle while sliding just as plainly.
        Assert.True(
            state.SideslipAngleRadians < -0.01f,
            "a left turn should step the tail outward"
        );
        Assert.True(state.YawRateRadiansPerSecond > 0f, "left-turn rear saturation should build positive yaw rate");
        Assert.True(
            state.Heading > state.VelocityHeading,
            "the body should point farther into the left turn than the velocity direction"
        );

        StepMany(state, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 120);

        Assert.True(
            Math.Abs(state.SideslipAngleRadians) < builtSideslip * 0.1f,
            "the vehicle layer should stabilize sideslip after rear saturation ends"
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
        TireConfig tires = WarmTires();
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
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        MakeFrontTiresHotAndWorn(state);

        float maximumSideslip = 0f;
        float maximumYawDeficit = 0f;
        for (int step = 0; step < 60; step++)
        {
            CarPhysics.Step(
                state,
                car,
                tires,
                PhysicsInput(new DriverInput(0.01f, 0f)),
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

        Assert.True(
            maximumSideslip > 0.02f,
            "front saturation in a left turn should leave the velocity direction ahead of the body"
        );
        Assert.True(
            maximumYawDeficit > 0.05f,
            "front saturation should produce less yaw rate than requested"
        );
    }

    [Fact]
    public void TractionControlCutsDriveNearTheRearCombinedGripLimit()
    {
        CarConfig controlledCar = new();
        CarConfig uncontrolledCar = new() { TractionControlStrength = 0f };
        TireConfig tires = WarmTires();
        CarState controlled = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarState uncontrolled = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarStrategy attack = new(TireUsageMode.Normal, BatteryOutputMode.Attack);
        // Cornering hard and asking for drive on top, which is where traction
        // control is meant to step in.
        float curvature = CurvatureForGripShare(controlled, car: controlledCar,
            tires, attack, 0.9f);
        SetSteadyYawRate(controlled, curvature);
        SetSteadyYawRate(uncontrolled, curvature);
        float drive = DriveForShare(
            controlled, controlledCar, tires, attack, curvature, 1.2f);
        CarPhysicsStepInput input =
            PhysicsInput(new DriverInput(curvature, drive), attack);

        CarPhysics.Step(controlled, controlledCar, tires, input, 1f / 60f);
        CarPhysics.Step(uncontrolled, uncontrolledCar, tires, input, 1f / 60f);

        Assert.True(controlled.Telemetry.TractionControlCutAccel > 0f);
        Assert.Equal(0f, uncontrolled.Telemetry.TractionControlCutAccel, precision: 4);
        Assert.True(
            controlled.Telemetry.RearLongitudinalUse < uncontrolled.Telemetry.RearLongitudinalUse,
            "TC should reduce rear drive use before the physics grip clip"
        );
        Assert.True(
            controlled.Telemetry.ActualLongitudinalAccel < uncontrolled.Telemetry.ActualLongitudinalAccel,
            "TC intervention should trade acceleration for rear stability"
        );
    }

    [Fact]
    public void SideslipDissipatesSpeedAndHeatsRearTiresWhileTcIntervenes()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState aligned = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarState sliding = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        sliding.Heading = -0.1f;
        sliding.SideslipAngleRadians = 0.1f;
        CarStrategy attack = new(TireUsageMode.Normal, BatteryOutputMode.Attack);
        float curvature = CurvatureForGripShare(aligned, car, tires, attack, 0.9f);
        float drive = DriveForShare(aligned, car, tires, attack, curvature, 1.5f);
        CarPhysicsStepInput input =
            PhysicsInput(new DriverInput(curvature, drive), attack);

        CarPhysics.Step(aligned, car, tires, input, 1f / 60f);
        CarPhysics.Step(sliding, car, tires, input, 1f / 60f);

        Assert.True(sliding.Telemetry.TractionControlCutAccel > 0f);
        Assert.True(sliding.Telemetry.SideslipLossAccel > 0.5f);
        Assert.True(
            sliding.Telemetry.ActualLongitudinalAccel < aligned.Telemetry.ActualLongitudinalAccel,
            "existing lateral slip should dissipate speed independently of TC"
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
        TireConfig tires = WarmTires();
        CarState protect = CreateState(speed: 37f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 37f, batterySoc: 0.8f, tires);
        DriverInput input = new(0.014f, 0f);

        StepMany(protect, car, tires, input, new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Normal), steps: 240);
        StepMany(attack, car, tires, input, new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Normal), steps: 240);

        Assert.Equal(
            AverageSurfaceTemp(protect),
            AverageSurfaceTemp(attack),
            precision: 5
        );
        Assert.Equal(AverageWear(protect), AverageWear(attack), precision: 5);
        Assert.Equal(
            protect.Telemetry.FrontGripAccel,
            attack.Telemetry.FrontGripAccel,
            precision: 5
        );
    }

    [Fact]
    public void AttackBatteryModeAcceleratesMoreButSpendsMoreEnergyThanSave()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState save = CreateState(speed: 50f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 50f, batterySoc: 0.8f, tires);
        DriverInput input = new(0f, car.MaxDriveAcceleration);

        StepMany(save, car, tires, input, new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Save), steps: 60);
        StepMany(attack, car, tires, input, new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Attack), steps: 60);

        Assert.True(attack.Speed > save.Speed, "attack battery mode should produce more straight-line speed");
        Assert.True(attack.BatterySoc < save.BatterySoc, "attack battery mode should spend more energy");
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
        CarConfig car = new();
        float eco = car.GetDrivePowerLimitWatts(BatteryOutputMode.Eco);
        float normal = car.GetDrivePowerLimitWatts(BatteryOutputMode.Normal);
        float between = car.GetDrivePowerLimitWatts(0.375f);

        Assert.InRange(between, MathF.Min(eco, normal), MathF.Max(eco, normal));
        Assert.Equal((eco + normal) * 0.5f, between, precision: 1);
        Assert.Equal(
            407000f,
            car.GetDrivePowerLimitWatts(
                CarStrategy.Default.WithDrivePowerLimitWatts(407000f)
            )
        );
    }

    [Fact]
    public void BatteryModesShareLowSpeedAccelerationAndEfficiency()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState save = CreateState(speed: 10f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 10f, batterySoc: 0.8f, tires);
        DriverInput input = new(0f, 7f);

        CarPhysics.Step(
            save,
            car,
            tires,
            PhysicsInput(input, new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Save)),
            1f / 60f
        );
        CarPhysics.Step(
            attack,
            car,
            tires,
            PhysicsInput(input, new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Attack)),
            1f / 60f
        );

        Assert.Equal(
            save.Telemetry.RequestedLongitudinalAccel,
            attack.Telemetry.RequestedLongitudinalAccel,
            precision: 5
        );
        Assert.Equal(save.BatterySoc, attack.BatterySoc, precision: 7);
    }

    [Fact]
    public void LowBatterySocLimitsDriveOutput()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState full = CreateState(speed: 26f, batterySoc: 0.8f, tires);
        CarState low = CreateState(speed: 26f, batterySoc: 0.02f, tires);
        CarStrategy strategy = new(TireUsageMode.Normal, BatteryOutputMode.Attack);
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
        TireConfig tires = WarmTires();
        CarState protect = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        DriverInput input = new(0.018f, 0f);

        CarPhysics.Step(protect, car, tires, PhysicsInput(input, new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Normal)), 1f / 60f);
        CarPhysics.Step(attack, car, tires, PhysicsInput(input, new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Normal)), 1f / 60f);

        Assert.Equal(
            protect.Telemetry.ActualCurvature,
            attack.Telemetry.ActualCurvature,
            precision: 5
        );
        Assert.Equal(
            protect.Telemetry.FrontGripAccel,
            attack.Telemetry.FrontGripAccel,
            precision: 5
        );
    }

    [Fact]
    public void HotWornTiresDeliverLessGripThanFreshWarmTires()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState fresh = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        CarState worn = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        MakeTiresHotAndWorn(worn);
        DriverInput input = new(0.018f, 0f);

        CarPhysics.Step(fresh, car, tires, PhysicsInput(input), 1f / 60f);
        CarPhysics.Step(worn, car, tires, PhysicsInput(input), 1f / 60f);

        Assert.True(worn.Telemetry.FrontGripAccel < fresh.Telemetry.FrontGripAccel, "hot worn front tires should have less grip");
        Assert.True(worn.Telemetry.RearGripAccel < fresh.Telemetry.RearGripAccel, "hot worn rear tires should have less grip");
        Assert.True(
            Math.Abs(worn.Telemetry.ActualCurvature) < Math.Abs(fresh.Telemetry.ActualCurvature),
            "hot worn tires should deliver less curvature under the same request"
        );
    }

    [Fact]
    public void SurfaceTemperaturesInsideIdealBandKeepFullGrip()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState cold = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState lowIdeal = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState middleIdeal = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState highIdeal = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState hot = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        SetTireTemps(cold, surfaceTempC: 75f, coreTempC: 90f);
        SetTireTemps(lowIdeal, surfaceTempC: 85f, coreTempC: 90f);
        SetTireTemps(middleIdeal, surfaceTempC: 100f, coreTempC: 90f);
        SetTireTemps(highIdeal, surfaceTempC: 115f, coreTempC: 90f);
        SetTireTemps(hot, surfaceTempC: 125f, coreTempC: 90f);

        foreach (CarState state in new[]
                 {
                     cold,
                     lowIdeal,
                     middleIdeal,
                     highIdeal,
                     hot
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

        Assert.Equal(
            lowIdeal.Telemetry.FrontGripAccel,
            middleIdeal.Telemetry.FrontGripAccel,
            precision: 4
        );
        Assert.Equal(
            middleIdeal.Telemetry.FrontGripAccel,
            highIdeal.Telemetry.FrontGripAccel,
            precision: 4
        );
        Assert.True(
            cold.Telemetry.FrontGripAccel < lowIdeal.Telemetry.FrontGripAccel
        );
        Assert.True(
            hot.Telemetry.FrontGripAccel < highIdeal.Telemetry.FrontGripAccel
        );
    }

    [Theory]
    [InlineData(0f, 1f)]
    [InlineData(0.70f, 0.93f)]
    [InlineData(0.85f, 0.84f)]
    [InlineData(1f, 0.75f)]
    public void TireWearUsesLinearLossBeforeSmoothGripCliff(
        float wear,
        float expectedGripFactor
    )
    {
        CarConfig car = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        CarState fresh = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState worn = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        foreach (TireState tire in Tires(worn))
            tire.Wear = wear;

        CarPhysicsStepInput input = new(
            new DriverInput(0f, 0f),
            CarStrategy.Default,
            AirTempC: 90f,
            TrackTempC: 90f
        );
        CarPhysics.Step(fresh, car, tires, input, 1f / 120f);
        CarPhysics.Step(worn, car, tires, input, 1f / 120f);

        float frontGripFactor =
            worn.Telemetry.FrontGripAccel /
            fresh.Telemetry.FrontGripAccel;
        float rearGripFactor =
            worn.Telemetry.RearGripAccel /
            fresh.Telemetry.RearGripAccel;
        Assert.InRange(
            MathF.Abs(frontGripFactor - expectedGripFactor),
            0f,
            1e-4f
        );
        Assert.InRange(
            MathF.Abs(rearGripFactor - expectedGripFactor),
            0f,
            1e-4f
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
        CarConfig car = new() { DownforceAccelPerSpeedSquared = 0f };
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            LateralWearRate = 0.01f,
            LongitudinalWearRate = 0f,
            OverLimitWearRate = 0f,
            SideslipWearRate = 0f,
            HotWearStartTempC = 1000f
        };
        CarState slow = CreateState(speed: 15f, batterySoc: 0.8f, tires);
        CarState fast = CreateState(speed: 45f, batterySoc: 0.8f, tires);
        const float targetLateralAccel = 6f;
        float slowCurvature = targetLateralAccel / (slow.Speed * slow.Speed);
        float fastCurvature = targetLateralAccel / (fast.Speed * fast.Speed);
        SetSteadyYawRate(slow, slowCurvature);
        SetSteadyYawRate(fast, fastCurvature);

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
    public void CorneringScrubSlowsCarEvenWithoutBrakeRequest()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
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
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 60f, batterySoc: 0.8f, tires);

        StepMany(state, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 300);

        Assert.True(state.Speed < 55.5f, "five seconds of zero-throttle coasting should lose visible speed");
        Assert.True(state.Telemetry.ActualLongitudinalAccel < -1.3f, "high-speed drag should keep slowing the car");
        Assert.Equal(-state.Telemetry.LossAccel, state.Telemetry.ActualLongitudinalAccel, precision: 4);
        Assert.Equal(0f, state.Telemetry.DrivePowerWatts, precision: 4);
    }

    [Fact]
    public void RepresentativeRunningKeepsWarmTiresInABoundedWindow()
    {
        CarConfig car = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 86f,
            StartingCoreTempC = 84f
        };
        CarState state = CreateState(speed: 36f, batterySoc: 0.8f, tires);
        const float dt = 1f / 60f;

        for (int step = 0; step < 120 * 60; step++)
        {
            int phase = step % (10 * 60);
            DriverInput input;
            if (phase < 4 * 60)
            {
                state.Speed = 36f;
                input = new DriverInput(0.0075f, 0f);
            }
            else if (phase < 7 * 60)
            {
                state.Speed = 48f;
                input = new DriverInput(0.001f, 3f);
            }
            else
            {
                state.Speed = 42f;
                input = new DriverInput(0.005f, -5f);
            }

            state.BatterySoc = 0.8f;
            CarPhysics.Step(state, car, tires, PhysicsInput(input), dt);
        }

        float surface = AverageSurfaceTemp(state);
        float core = AverageCoreTemp(state);
        Assert.InRange(surface, 70f, 110f);
        Assert.InRange(core, 70f, 105f);
    }

    [Fact]
    public void CombinedNearLimitUseAmplifiesExistingSlipHeatOnce()
    {
        CarConfig car = new();
        TireConfig baselineTires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            NearLimitHeatGain = 0f
        };
        TireConfig amplifiedTires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            NearLimitHeatGain = 2f
        };
        CarState baseline = CreateState(speed: 38f, batterySoc: 0.8f, baselineTires);
        CarState amplified = CreateState(speed: 38f, batterySoc: 0.8f, amplifiedTires);
        // Near the limit, which is the condition the extra heat is about.
        float curvature = CurvatureForGripShare(
            baseline, car, baselineTires, 0.85f);
        float drive = DriveForShare(
            baseline, car, baselineTires, CarStrategy.Default, curvature, 0.6f);
        DriverInput input = new(curvature, drive);

        StepMany(baseline, car, baselineTires, input, CarStrategy.Default, steps: 120);
        StepMany(amplified, car, amplifiedTires, input, CarStrategy.Default, steps: 120);

        Assert.True(
            AverageSurfaceTemp(amplified) > AverageSurfaceTemp(baseline),
            "combined friction-circle use should multiply the existing directional slip heat"
        );
    }

    [Fact]
    public void StraightRollingBuildsCoreHeatWithoutTireSlip()
    {
        CarConfig car = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 25f,
            StartingCoreTempC = 25f
        };
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
    public void IsolatedCoreDoesNotExchangeHeatDirectlyWithAmbientAir()
    {
        CarConfig car = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 100f,
            StartingCoreTempC = 100f
        };
        CarState state = CreateState(speed: 0f, batterySoc: 0.8f, tires);

        CarPhysics.Step(
            state,
            car,
            tires,
            new CarPhysicsStepInput(
                new DriverInput(0f, 0f),
                CarStrategy.Default,
                AirTempC: 25f,
                TrackTempC: 25f
            ),
            1f / 60f
        );

        Assert.True(AverageSurfaceTemp(state) < 100f);
        Assert.Equal(100f, AverageCoreTemp(state), precision: 4);
    }

    [Fact]
    public void CoreTemperatureRespondsMoreSlowlyThanTreadSurface()
    {
        CarConfig car = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 70f,
            StartingCoreTempC = 70f
        };
        CarState state = CreateState(speed: 36f, batterySoc: 0.8f, tires);

        StepMany(
            state,
            car,
            tires,
            new DriverInput(0.012f, 0f),
            CarStrategy.Default,
            steps: 120
        );

        float surfaceRise = AverageSurfaceTemp(state) - 70f;
        float coreRise = AverageCoreTemp(state) - 70f;
        Assert.True(surfaceRise > 1f);
        Assert.True(
            surfaceRise > coreRise * 4f,
            "the high-capacity core should not follow a short tread heat spike"
        );
    }

    [Fact]
    public void StraightLineSpeedIncreasesTreadSurfaceCooling()
    {
        // No wings, so the only thing speed changes is the air moving over the
        // tyre, which is what this is about. With them the load at fifty five
        // metres a second is more than doubled and the heat that comes with it
        // swamps the cooling being measured.
        CarConfig car = new() { DownforceAccelPerSpeedSquared = 0f };
        TireConfig tires = WarmTires();
        CarState parked = CreateState(speed: 0f, batterySoc: 0.8f, tires);
        CarState fast = CreateState(speed: 55f, batterySoc: 0.8f, tires);
        SetTireTemps(parked, surfaceTempC: 112f, coreTempC: 104f);
        SetTireTemps(fast, surfaceTempC: 112f, coreTempC: 104f);

        StepMany(parked, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 120);
        StepMany(fast, car, tires, new DriverInput(0f, 0f), CarStrategy.Default, steps: 120);

        Assert.True(
            AverageSurfaceTemp(fast) < AverageSurfaceTemp(parked),
            "fast straight-line running should increase air cooling at the tire surface"
        );
    }

    [Fact]
    public void BrakingWhileCorneringHasLessDeliveredBrakeThanStraightBraking()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
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
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 44f, batterySoc: 0.6f, tires);

        CarPhysics.Step(state, car, tires, PhysicsInput(new DriverInput(0.03f, -10f)), 1f / 60f);

        Assert.True(state.Telemetry.OverLimit > 0f, "heavy cornering should exceed the combined tire budget");
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
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 32f,
            StartingCoreTempC = 30f
        };
        CarState state = new();
        state.FrontLeft.SurfaceTempC = 118f;
        state.FrontLeft.CoreTempC = 104f;
        state.FrontLeft.Wear = 0.72f;
        state.FrontLeft.LoadN = 1234f;

        state.InstallFreshTires(tires);

        Assert.Equal(32f, state.FrontLeft.SurfaceTempC, precision: 4);
        Assert.Equal(30f, state.FrontLeft.CoreTempC, precision: 4);
        Assert.Equal(0f, state.FrontLeft.Wear, precision: 4);
        Assert.Equal(0f, state.FrontLeft.LoadN, precision: 4);
    }

    private static CarState CreateState(float speed, float batterySoc, TireConfig tires)
    {
        CarState state = new()
        {
            Speed = speed,
            BatterySoc = batterySoc
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
        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            state,
            car,
            tires,
            strategy,
            state.Speed,
            curvature: 0f
        );
        float lateral = limits.LateralAccelerationLimit * share;
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

    private static void SetSteadyYawRate(CarState state, float curvature)
    {
        state.YawRateRadiansPerSecond = state.Speed * curvature;
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

    private static IEnumerable<TireState> Tires(CarState state)
    {
        yield return state.FrontLeft;
        yield return state.FrontRight;
        yield return state.RearLeft;
        yield return state.RearRight;
    }

    private static TireConfig WarmTires()
    {
        return new TireConfig
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
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

    private static float AverageRearSurfaceTemp(CarState state)
    {
        return (state.RearLeft.SurfaceTempC + state.RearRight.SurfaceTempC) * 0.5f;
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
