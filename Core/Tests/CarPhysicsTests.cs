using TheStint.Core.Cars;
using Xunit;

namespace TheStint.Core.Tests;

public sealed class CarPhysicsTests
{
    private const float TestAirTempC = 25f;

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
    public void AttackTireModeHeatsAndWearsMoreThanProtect()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState protect = CreateState(speed: 37f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 37f, batterySoc: 0.8f, tires);
        DriverInput input = new(0.014f, 0f);

        StepMany(protect, car, tires, input, new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Normal), steps: 240);
        StepMany(attack, car, tires, input, new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Normal), steps: 240);

        Assert.True(
            AverageSurfaceTemp(attack) > AverageSurfaceTemp(protect),
            "attack mode should heat tire surfaces more than protect mode"
        );
        Assert.True(
            AverageWear(attack) > AverageWear(protect),
            "attack mode should wear tires more than protect mode"
        );
    }

    [Fact]
    public void AttackBatteryModeAcceleratesMoreButSpendsMoreEnergyThanSave()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState save = CreateState(speed: 28f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 28f, batterySoc: 0.8f, tires);
        DriverInput input = new(0f, 7f);

        StepMany(save, car, tires, input, new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Save), steps: 60);
        StepMany(attack, car, tires, input, new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Attack), steps: 60);

        Assert.True(attack.Speed > save.Speed, "attack battery mode should produce more straight-line speed");
        Assert.True(attack.BatterySoc < save.BatterySoc, "attack battery mode should spend more energy");
        Assert.True(attack.Telemetry.DrivePowerWatts > save.Telemetry.DrivePowerWatts, "attack mode should report higher drive power");
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
    public void AttackTireModeCanDeliverMoreCurvatureThanProtect()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState protect = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        CarState attack = CreateState(speed: 42f, batterySoc: 0.8f, tires);
        DriverInput input = new(0.018f, 0f);

        CarPhysics.Step(protect, car, tires, PhysicsInput(input, new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Normal)), 1f / 60f);
        CarPhysics.Step(attack, car, tires, PhysicsInput(input, new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Normal)), 1f / 60f);

        Assert.True(
            Math.Abs(attack.Telemetry.ActualCurvature) > Math.Abs(protect.Telemetry.ActualCurvature),
            "attack tire mode should deliver more curvature when cornering is grip-limited"
        );
        Assert.True(
            attack.Telemetry.FrontGripAccel > protect.Telemetry.FrontGripAccel,
            "attack tire mode should expose more front grip"
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
    public void StraightLineSpeedIncreasesTireCooling()
    {
        CarConfig car = new();
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
        Assert.True(
            AverageCoreTemp(fast) < AverageCoreTemp(parked),
            "fast straight-line running should also improve slow core cooling"
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

    private static void MakeTiresHotAndWorn(CarState state)
    {
        foreach (TireState tire in Tires(state))
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
        return new CarPhysicsStepInput(input, strategy, TestAirTempC);
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
