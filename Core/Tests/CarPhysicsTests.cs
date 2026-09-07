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
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 34f, batterySoc: 0.5f, tires);
        float startSpeed = state.Speed;
        float startSoc = state.Energy.Primary;

        StepMany(state, car, tires, new DriverInput(0f, -7f), CarStrategy.Default, steps: 20);

        Assert.True(state.Speed < startSpeed, "speed should fall under brake request");
        Assert.True(state.Energy.Primary > startSoc, "battery SOC should rise from regen during braking");
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

        DriverInput absurd = new(car.MaxCurvatureRequest, 0f);
        // Held for a second, because the wheels now take time to reach the
        // lock stops and an absurd request is only absurd once they have.
        for (int i = 0; i < 60; i++)
            CarPhysics.Step(state, car, tires, PhysicsInput(absurd), 1f / 60f);

        Assert.True(
            state.Telemetry.OverLimit > 0.5f,
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
        TireConfig tires = WarmTires();
        CarState state = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarStrategy strategy = new(TireUsageMode.Normal, PowerOutputMode.Attack);
        DriverInput input = new(0.018f, 7f);
        SetSteadyCorner(state, car, tires, 0.018f);

        CarPhysics.Step(state, car, tires, PhysicsInput(input, strategy), 1f / 60f);

        Assert.True(
            state.Telemetry.RearLongitudinalUse > 0.2f,
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
        CarConfig car = new() { TractionControlStrength = 0f };
        TireConfig tires = WarmTires();
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

        Assert.True(powered.Telemetry.RearLongitudinalUse > 0.5f);
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

        // The anti-lock takes most of the excess a bad split creates, so what
        // the bias costs is mainly braking the car never gets rather than a
        // tyre driven past what it has. Only most, though: some is still there,
        // and the balanced car has none of it.
        // The over-limit reading is gone from both, and that is the
        // anti-lock doing its job rather than anything being lost. A bad
        // split used to leave a little of the excess on the tyre because
        // the lateral demand it was measured against was a request; it is
        // now what the tyre is actually delivering, the anti-lock sees the
        // circle that is really there, and it takes the whole excess. What
        // the bias costs is therefore all of it braking the car never gets
        // - which is what the two readings below say, and what the name of
        // this test has always been about.
        Assert.Equal(0f, optimal.Telemetry.OverLimit, precision: 4);
        Assert.Equal(0f, frontBiased.Telemetry.OverLimit, precision: 4);
        Assert.True(
            frontBiased.Telemetry.ActualLongitudinalAccel >
            optimal.Telemetry.ActualLongitudinalAccel + 0.1f,
            "a biased split should clip one axle before all remaining grip is used"
        );
        // And it costs no cornering at all - it buys a little, which is
        // the anti-lock's whole purpose stated backwards. A split that
        // overloads one axle gets that axle held further back from its
        // circle, so more of the circle is left over for steering. What a
        // bad split costs is entirely stopping, which is the reading above.
        Assert.True(
            frontBiased.Telemetry.ActualLateralAccel >=
            optimal.Telemetry.ActualLateralAccel,
            $"a split the anti-lock has to correct should not cost cornering: " +
            $"{optimal.Telemetry.ActualLateralAccel:0.000} became " +
            $"{frontBiased.Telemetry.ActualLateralAccel:0.000}"
        );
    }

    [Fact]
    public void DefaultRearDriveMakesTheRearCloserToTheCombinedLimitUnderPower()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
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
            state.Telemetry.RearSlideSeverity > 0.1f,
            $"rear saturation should expose slide severity, got " +
            $"{state.Telemetry.RearSlideSeverity:0.000}"
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
    public void TractionControlCutsDriveNearTheRearCombinedGripLimit()
    {
        CarConfig controlledCar = new();
        CarConfig uncontrolledCar = new() { TractionControlStrength = 0f };
        TireConfig tires = WarmTires();
        CarState controlled = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarState uncontrolled = CreateState(speed: 36f, batterySoc: 0.9f, tires);
        CarStrategy attack = new(TireUsageMode.Normal, PowerOutputMode.Attack);
        // Cornering hard and asking for drive on top, which is where traction
        // control is meant to step in.
        float curvature = CurvatureForGripShare(controlled, car: controlledCar,
            tires, attack, 0.9f);
        SetSteadyCorner(controlled, controlledCar, tires, curvature);
        SetSteadyCorner(uncontrolled, uncontrolledCar, tires, curvature);
        float drive = DriveForShare(
            controlled, controlledCar, tires, attack, curvature, 1.2f);
        DriverInput asked = new(curvature, drive);
        CarPhysicsStepInput input = PhysicsInput(asked, attack);
        SetSteadyCorner(controlled, controlledCar, tires, curvature);
        SetSteadyCorner(uncontrolled, uncontrolledCar, tires, curvature);

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
        CarStrategy attack = new(TireUsageMode.Normal, PowerOutputMode.Attack);
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

        StepMany(protect, car, tires, input, new CarStrategy(TireUsageMode.Protect, PowerOutputMode.Normal), steps: 240);
        StepMany(attack, car, tires, input, new CarStrategy(TireUsageMode.Attack, PowerOutputMode.Normal), steps: 240);

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
        Assert.Equal(
            407000f,
            powertrain.GetDrivePowerLimitWatts(
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

        Assert.Equal(
            save.Telemetry.RequestedLongitudinalAccel,
            attack.Telemetry.RequestedLongitudinalAccel,
            precision: 5
        );
        Assert.Equal(save.Energy.Primary, attack.Energy.Primary, precision: 7);
    }

    [Fact]
    public void LowBatterySocLimitsDriveOutput()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState full = CreateState(speed: 26f, batterySoc: 0.8f, tires);
        CarState low = CreateState(speed: 26f, batterySoc: 0.02f, tires);
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
        TireConfig tires = WarmTires();
        CarState protect = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        DriverInput input = new(0.018f, 0f);

        CarPhysics.Step(protect, car, tires, PhysicsInput(input, new CarStrategy(TireUsageMode.Protect, PowerOutputMode.Normal)), 1f / 60f);
        CarPhysics.Step(attack, car, tires, PhysicsInput(input, new CarStrategy(TireUsageMode.Attack, PowerOutputMode.Normal)), 1f / 60f);

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
    public void SurfaceTemperatureGripUsesAFlatWindowAndSmoothOuterCurve()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
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
        float bandLow = tires.IdealSurfaceTempLowC;
        float bandHigh = tires.IdealSurfaceTempHighC;
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
        float hotNearLoss = highIdeal.Telemetry.FrontGripAccel -
                            hotNear.Telemetry.FrontGripAccel;
        float hotMiddleLoss = highIdeal.Telemetry.FrontGripAccel -
                              hotMiddle.Telemetry.FrontGripAccel;
        float hotFarLoss = highIdeal.Telemetry.FrontGripAccel -
                           hotFar.Telemetry.FrontGripAccel;
        Assert.True(hotNearLoss > 0f);
        Assert.InRange(hotMiddleLoss / hotNearLoss, 3.9f, 4.1f);
        Assert.InRange(hotFarLoss / hotNearLoss, 8.9f, 9.1f);
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
    public void HotTreadWearRisesSmoothlyWithSquaredDistanceFromTheWindow()
    {
        CarConfig car = new() { DownforceAccelPerSpeedSquared = 0f };
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            LateralWearRate = 1f,
            LongitudinalWearRate = 0f,
            OverLimitWearRate = 0f,
            SideslipWearRate = 0f
        };
        CarState edge = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState hotFive = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        CarState hotTen = CreateState(speed: 30f, batterySoc: 0.8f, tires);
        SetTireTemps(edge, tires.IdealSurfaceTempHighC, tires.IdealSurfaceTempHighC);
        SetTireTemps(hotFive, tires.IdealSurfaceTempHighC + 5f, tires.IdealSurfaceTempHighC + 5f);
        SetTireTemps(hotTen, tires.IdealSurfaceTempHighC + 10f, tires.IdealSurfaceTempHighC + 10f);

        const float gripShare = 0.6f;
        const float dt = 1f / 600f;
        float edgeCurvature = CurvatureForGripShare(edge, car, tires, gripShare);
        float hotFiveCurvature = CurvatureForGripShare(hotFive, car, tires, gripShare);
        float hotTenCurvature = CurvatureForGripShare(hotTen, car, tires, gripShare);
        SetSteadyCorner(edge, car, tires, edgeCurvature);
        SetSteadyCorner(hotFive, car, tires, hotFiveCurvature);
        SetSteadyCorner(hotTen, car, tires, hotTenCurvature);

        StepAtMatchingAmbient(edge, car, tires, edgeCurvature, dt);
        StepAtMatchingAmbient(hotFive, car, tires, hotFiveCurvature, dt);
        StepAtMatchingAmbient(hotTen, car, tires, hotTenCurvature, dt);

        float edgeWear = AverageWear(edge);
        float fiveDegreeExtra = AverageWear(hotFive) / edgeWear - 1f;
        float tenDegreeExtra = AverageWear(hotTen) / edgeWear - 1f;
        Assert.True(fiveDegreeExtra > 0f);
        Assert.InRange(tenDegreeExtra / fiveDegreeExtra, 3.9f, 4.1f);
    }

    [Fact]
    public void WearRisesFasterThanSquaredDemandNearTheLimit()
    {
        CarConfig car = new() { DownforceAccelPerSpeedSquared = 0f };
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            LateralWearRate = 0.00055f,
            LongitudinalWearRate = 0f,
            OverLimitWearRate = 0f,
            SideslipWearRate = 0f
        };
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
        CarConfig car = new() { DownforceAccelPerSpeedSquared = 0f };
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            LateralWearRate = 0.01f,
            LongitudinalWearRate = 0f,
            OverLimitWearRate = 0f,
            SideslipWearRate = 0f
        };
        CarState slow = CreateState(speed: 15f, batterySoc: 0.8f, tires);
        CarState fast = CreateState(speed: 45f, batterySoc: 0.8f, tires);
        const float targetLateralAccel = 6f;
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
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
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
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
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

        Assert.Equal(
            cleanLimits.LateralAccelerationLimit,
            bodyTowLimits.LateralAccelerationLimit,
            precision: 4
        );
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
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
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

            state.Energy = PowertrainState.Filled(0.8f);
            CarPhysics.Step(state, car, tires, PhysicsInput(input), dt);
        }

        float surface = AverageSurfaceTemp(state);
        float core = AverageCoreTemp(state);
        Assert.InRange(surface, 70f, 110f);
        Assert.InRange(core, 70f, 105f);
    }

    [Fact]
    public void CombinedNearLimitUseAddsPartialSlipHeat()
    {
        CarConfig car = new();
        TireConfig baselineTires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            NearLimitHeatRate = 0f
        };
        TireConfig partialSlipTires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            NearLimitHeatRate = 0.5f
        };
        CarState baseline = CreateState(speed: 38f, batterySoc: 0.8f, baselineTires);
        CarState withPartialSlip = CreateState(
            speed: 38f, batterySoc: 0.8f, partialSlipTires);
        // Near the limit, which is the condition the extra heat is about.
        float curvature = CurvatureForGripShare(
            baseline, car, baselineTires, 0.9f);
        float drive = DriveForShare(
            baseline, car, baselineTires, CarStrategy.Default, curvature, 0.8f);
        DriverInput input = new(curvature, drive);

        StepMany(baseline, car, baselineTires, input, CarStrategy.Default, steps: 120);
        StepMany(
            withPartialSlip,
            car,
            partialSlipTires,
            input,
            CarStrategy.Default,
            steps: 120
        );

        Assert.True(
            AverageSurfaceTemp(withPartialSlip) > AverageSurfaceTemp(baseline),
            "combined friction-circle use should add partial-slip heat"
        );
    }

    [Fact]
    public void NearLimitPartialSlipHeatDoesNotMultiplyDirectionalHeat()
    {
        CarConfig car = new();
        TireConfig lowBase = NearLimitTires(
            lateralHeatRate: 0.25f, partialSlipHeatRate: 0f);
        TireConfig lowAdded = NearLimitTires(
            lateralHeatRate: 0.25f, partialSlipHeatRate: 0.5f);
        TireConfig highBase = NearLimitTires(
            lateralHeatRate: 4f, partialSlipHeatRate: 0f);
        TireConfig highAdded = NearLimitTires(
            lateralHeatRate: 4f, partialSlipHeatRate: 0.5f);
        CarState lowBaseState = CreateState(speed: 38f, batterySoc: 0.8f, lowBase);
        CarState lowAddedState = CreateState(speed: 38f, batterySoc: 0.8f, lowAdded);
        CarState highBaseState = CreateState(speed: 38f, batterySoc: 0.8f, highBase);
        CarState highAddedState = CreateState(speed: 38f, batterySoc: 0.8f, highAdded);
        float curvature = CurvatureForGripShare(
            lowBaseState, car, lowBase, 0.95f);
        DriverInput input = new(curvature, 0f);
        SetSteadyCorner(lowBaseState, car, lowBase, curvature);
        SetSteadyCorner(lowAddedState, car, lowAdded, curvature);
        SetSteadyCorner(highBaseState, car, highBase, curvature);
        SetSteadyCorner(highAddedState, car, highAdded, curvature);

        StepMany(lowBaseState, car, lowBase, input, CarStrategy.Default, steps: 1);
        StepMany(lowAddedState, car, lowAdded, input, CarStrategy.Default, steps: 1);
        StepMany(highBaseState, car, highBase, input, CarStrategy.Default, steps: 1);
        StepMany(highAddedState, car, highAdded, input, CarStrategy.Default, steps: 1);

        float lowExtra = AverageSurfaceTemp(lowAddedState) - AverageSurfaceTemp(lowBaseState);
        float highExtra = AverageSurfaceTemp(highAddedState) - AverageSurfaceTemp(highBaseState);
        Assert.True(lowExtra > 0f);
        Assert.InRange(highExtra / lowExtra, 0.95f, 1.05f);
    }

    [Fact]
    public void DirectionalHeatPerUnitWorkRisesTowardTheLimit()
    {
        // Read against the limit the car can hold rather than against the
        // whole friction circle. The top of the old range is no longer a
        // place a car can be: both ends would have to sit exactly on their
        // peaks, and nothing does, so probing there measured a corner the
        // car was failing to take rather than a tyre working hard.
        // A wide span, because the top of the old one no longer exists.
        // These shares used to be 95.5, 96.6 and 100 percent of the whole
        // friction circle - three points inside the last twentieth of it,
        // which the car could hold when cornering force was granted on
        // request. It cannot hold either end of itself on a peak now, so
        // the same three points are a single operating condition measured
        // three times, and the effect they are looking for needs room.
        float protect = MeasureDirectionalHeatPerSquaredUse(0.25f);
        float light = MeasureDirectionalHeatPerSquaredUse(0.60f);
        float attack = MeasureDirectionalHeatPerSquaredUse(1.0f);

        Assert.True(
            protect < light,
            $"heat per unit work should grow from a quarter to two thirds of " +
            $"the limit, got {protect:F6} versus {light:F6}"
        );
        Assert.True(
            light < attack,
            $"heat per unit work should keep growing to the limit, got {light:F6} versus {attack:F6}"
        );
    }

    [Fact]
    public void AxleLateralComplianceRedistributesHeatAndWearWithoutChangingTotalWork()
    {
        CarConfig equalCompliance = new()
        {
            FrontStaticLoadShare = 0.5f,
            FrontLateralComplianceRatio = 1f,
            CenterOfGravityHeightMeters = 0f,
            DownforceAccelPerSpeedSquared = 0f,
            AeroDragAccelPerSpeedSquared = 0f,
            RollingDragAccel = 0f,
            CorneringScrubAccel = 0f
        };
        CarConfig softerFront = new()
        {
            FrontStaticLoadShare = 0.5f,
            FrontLateralComplianceRatio = 2f,
            CenterOfGravityHeightMeters = 0f,
            DownforceAccelPerSpeedSquared = 0f,
            AeroDragAccelPerSpeedSquared = 0f,
            RollingDragAccel = 0f,
            CorneringScrubAccel = 0f
        };
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 100f,
            StartingCoreTempC = 100f,
            LateralHeatRate = 1f,
            LongitudinalHeatRate = 0f,
            NearLimitHeatRate = 0f,
            LateralWearRate = 0.001f,
            LongitudinalWearRate = 0f,
            NearLimitWearRate = 0f,
            OverLimitWearRate = 0f,
            SideslipWearRate = 0f
        };
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
        Assert.Equal(
            equalFrontTemp + equalRearTemp,
            shiftedFrontTemp + shiftedRearTemp,
            precision: 5
        );
        Assert.Equal(
            equalFrontWear + equalRearWear,
            shiftedFrontWear + shiftedRearWear,
            precision: 7
        );
        Assert.Equal(
            equal.Telemetry.ActualLateralAccel,
            frontWorkingHarder.Telemetry.ActualLateralAccel,
            precision: 5
        );
    }

    [Fact]
    public void NearZeroAxleContributionDoesNotDiscardTotalLateralWork()
    {
        CarConfig ordinaryCompliance = new()
        {
            FrontStaticLoadShare = 0.0001f,
            FrontLateralComplianceRatio = 1f,
            CenterOfGravityHeightMeters = 0f,
            DownforceAccelPerSpeedSquared = 0f,
            AeroDragAccelPerSpeedSquared = 0f,
            RollingDragAccel = 0f,
            CorneringScrubAccel = 0f
        };
        CarConfig extremeCompliance = new()
        {
            FrontStaticLoadShare = 0.0001f,
            FrontLateralComplianceRatio = 100_000_000f,
            CenterOfGravityHeightMeters = 0f,
            DownforceAccelPerSpeedSquared = 0f,
            AeroDragAccelPerSpeedSquared = 0f,
            RollingDragAccel = 0f,
            CorneringScrubAccel = 0f
        };
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 100f,
            StartingCoreTempC = 100f,
            LateralHeatRate = 1f,
            LongitudinalHeatRate = 0f,
            NearLimitHeatRate = 0f,
            LateralWearRate = 0.001f,
            LongitudinalWearRate = 0f,
            NearLimitWearRate = 0f,
            OverLimitWearRate = 0f,
            SideslipWearRate = 0f
        };
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

        Assert.Equal(
            AverageFrontSurfaceTemp(ordinary) +
            AverageRearSurfaceTemp(ordinary),
            AverageFrontSurfaceTemp(extreme) +
            AverageRearSurfaceTemp(extreme),
            precision: 5
        );
        Assert.Equal(
            AverageFrontWear(ordinary) + AverageRearWear(ordinary),
            AverageFrontWear(extreme) + AverageRearWear(extreme),
            precision: 7
        );
    }

    private static TireConfig NearLimitTires(
        float lateralHeatRate,
        float partialSlipHeatRate
    )
    {
        return new TireConfig
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            LateralHeatRate = lateralHeatRate,
            LongitudinalHeatRate = 0f,
            NearLimitHeatRate = partialSlipHeatRate
        };
    }

    private static float MeasureDirectionalHeatPerSquaredUse(float gripShare)
    {
        CarConfig car = new();
        TireConfig baselineTires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            LateralHeatRate = 0f,
            LongitudinalHeatRate = 0f,
            NearLimitHeatRate = 0f
        };
        // The near-limit term is left at the car's own value rather than
        // zeroed. It is the whole mechanism being asked about: with it
        // switched off the heat model is exactly quadratic in use, so heat
        // per unit work is a constant by construction and the question has
        // no answer. It used to have one anyway, because lateral use could
        // exceed the circle when cornering force was granted on request.
        TireConfig workingTires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f,
            LateralHeatRate = 10f,
            LongitudinalHeatRate = 0f
        };
        CarState baseline = CreateState(
            speed: 38f,
            batterySoc: 0.8f,
            baselineTires
        );
        CarState working = CreateState(
            speed: 38f,
            batterySoc: 0.8f,
            workingTires
        );
        float curvature = CurvatureForGripShare(
            baseline,
            car,
            baselineTires,
            gripShare
        );
        SetSteadyCorner(baseline, car, baselineTires, curvature);
        SetSteadyCorner(working, car, workingTires, curvature);

        StepAtMatchingAmbient(
            baseline,
            car,
            baselineTires,
            curvature,
            1f / 60f
        );
        StepAtMatchingAmbient(
            working,
            car,
            workingTires,
            curvature,
            1f / 60f
        );

        float squaredUse = AverageAxleLateralUseSquared(working);
        return (
            AverageSurfaceTemp(working) - AverageSurfaceTemp(baseline)
        ) / Math.Max(squaredUse, 1e-6f);
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
    public void BrakeWorkHeatsTheCoreAtTheAxleThatActuallyDoesIt()
    {
        CarConfig car = new()
        {
            FrontStaticLoadShare = 0.5f,
            CenterOfGravityHeightMeters = 0f,
            DownforceAccelPerSpeedSquared = 0f
        };
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
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
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 100f,
            StartingCoreTempC = 100f
        };
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
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 70f,
            StartingCoreTempC = 70f
        };
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

        StepMany(
            state,
            car,
            tires,
            new DriverInput(curvature, 0f),
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
    public void LowerTireUseRecoversHeatSoakedCoreWhileLimitUseKeepsItHot()
    {
        CarState lowerUse = RunRepresentativeCoreDutyCycle(0.955f);
        CarState limitUse = RunRepresentativeCoreDutyCycle(1f);

        float lowerCore = AverageCoreTemp(lowerUse);
        float limitCore = AverageCoreTemp(limitUse);
        Assert.True(
            lowerCore < 100f,
            $"a five-minute low-use run should cool a 103 C soaked core below 100 C, got {lowerCore:F2} C"
        );
        Assert.True(
            limitCore > 105f,
            $"limit use should retain the established high-use heat behavior, got {limitCore:F2} C"
        );
        Assert.True(
            limitCore - lowerCore > 5f,
            $"the low and high uses need a useful core-temperature separation, got {lowerCore:F2} versus {limitCore:F2} C"
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
    public void AirVelocityDeficitReducesForcedTireCooling()
    {
        // Hold every source of heat and every ground-speed effect equal. The
        // only physical difference is how much air the following tyre meets.
        CarConfig car = new()
        {
            DownforceAccelPerSpeedSquared = 0f,
            AeroDragAccelPerSpeedSquared = 0f,
            RollingDragAccel = 0f
        };
        TireConfig tires = WarmTires();
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
        TireConfig tires = WarmTires();
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

    private static CarState CreateState(float speed, float batterySoc, TireConfig tires)
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
        float front = state.Telemetry.FrontLateralUse;
        float rear = state.Telemetry.RearLateralUse;
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

    private static CarState RunRepresentativeCoreDutyCycle(float use)
    {
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 107f,
            StartingCoreTempC = 103f
        };
        CarConfig car = new()
        {
            AeroDragAccelPerSpeedSquared = 0f,
            RollingDragAccel = 0f,
            CorneringScrubAccel = 0f
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
                ? limits.LateralAccelerationLimit * use /
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
