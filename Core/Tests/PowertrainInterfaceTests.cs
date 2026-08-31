using System;
using System.Collections.Generic;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// An interface with one implementation is not an interface, it is a class
/// with extra steps. These tests fit the car with a powertrain that is not a
/// battery - two stores instead of one, fuel that burns off and takes weight
/// with it, a ladder whose rungs are called something else - and check that
/// the rest of the car copes without knowing.
///
/// The powertrain below is deliberately crude and deliberately not shipped.
/// A petrol car worth racing needs its power curve and its consumption
/// calibrated against real lap times, the way the electric one was, and
/// inventing those numbers here would put an uncalibrated car in the game
/// under cover of a refactor. What is being tested is the seam, not the
/// engine.
/// </summary>
public sealed class PowertrainInterfaceTests
{
    private const float Dt = 1f / 60f;
    private const float FullTankMassKg = 100f;

    [Fact]
    public void FuelWeighsSomethingAndTheWholeCarFeelsIt()
    {
        // The one thing an electric car could never test: mass that moves.
        // If the load model were still reading a fixed number off the
        // configuration, these two cars would press on the road equally hard
        // and the difference below would be zero.
        float fullTank = TotalWheelLoadAfterAStep(1f);
        float emptyTank = TotalWheelLoadAfterAStep(0f);

        CarConfig config = FuelCar();
        float speed = 30f;
        float downforceAccel =
            config.DownforceAccelPerSpeedSquared * speed * speed;
        float expected = FullTankMassKg * (9.80665f + downforceAccel);

        Assert.Equal(expected, fullTank - emptyTank, 1);
    }

    [Fact]
    public void ACarCarryingFuelIsHeavierThanOneThatHasBurnedIt()
    {
        CarConfig config = FuelCar();

        Assert.Equal(
            config.MassKg + FullTankMassKg,
            CarPhysics.TotalMassKg(config, PowertrainState.Filled(1f)),
            3
        );
        Assert.Equal(
            config.MassKg + FullTankMassKg * 0.4f,
            CarPhysics.TotalMassKg(config, new PowertrainState(0.4f, 1f)),
            3
        );
        Assert.Equal(
            config.MassKg,
            CarPhysics.TotalMassKg(config, PowertrainState.Filled(0f)),
            3
        );
    }

    [Fact]
    public void DrivingBurnsTheFirstStoreAndLeavesTheSecondAlone()
    {
        CarConfig config = FuelCar();
        TireConfig tires = WarmTires();
        CarState state = new()
        {
            Speed = 55f,
            Energy = PowertrainState.Filled(1f)
        };
        state.InstallFreshTires(tires);

        for (int i = 0; i < 600; i++)
        {
            CarPhysics.Step(
                state,
                config,
                tires,
                new CarPhysicsStepInput(
                    new DriverInput(0f, 8f),
                    CarStrategy.Default,
                    28f
                ),
                Dt
            );
        }

        Assert.InRange(state.Energy.Primary, 0.01f, 0.999f);
        Assert.Equal(1f, state.Energy.Secondary);
        Assert.True(
            CarPhysics.TotalMassKg(config, state.Energy) <
            config.MassKg + FullTankMassKg,
            "a car that has been driving should have burned something"
        );
    }

    [Fact]
    public void TheDashboardReadsWhateverIsFitted()
    {
        CarDashboard dashboard = new();
        CarState state = new() { Energy = new PowertrainState(0.5f, 0.25f) };
        state.InstallFreshTires(WarmTires());

        dashboard.Refresh(
            FuelCar(),
            state,
            new CarStrategy(TireUsageMode.Push, PowerOutputMode.Save)
        );

        // Two stores, named by the machinery rather than by the panel.
        Assert.Equal(2, dashboard.Resources.Count);
        Assert.Equal("Fuel", dashboard.Resources[0].Label);
        Assert.Equal(0.5f, dashboard.Resources[0].Fraction);
        Assert.Equal(FullTankMassKg * 0.5f, dashboard.Resources[0].RemainingMassKg, 3);
        Assert.Equal("Charge", dashboard.Resources[1].Label);
        Assert.Equal(0.25f, dashboard.Resources[1].Fraction);

        // Two ladders: the universal one for tyres, and the engine's own
        // words for the other.
        Assert.Equal(2, dashboard.Modes.Count);
        Assert.Equal("Tyres", dashboard.Modes[0].Label);
        Assert.Equal("Push", dashboard.Modes[0].Rung);
        Assert.Equal("Engine", dashboard.Modes[1].Label);
        Assert.Equal("Lean", dashboard.Modes[1].Rung);
        Assert.Equal(1, dashboard.Modes[1].Ordinal);
        Assert.Equal(5, dashboard.Modes[1].RungCount);

        Assert.Equal(4, dashboard.Tires.Count);
        Assert.Equal(1f, dashboard.Tires[0].LifeFraction);
    }

    [Fact]
    public void TheDashboardOfTheCarWeActuallyShipStillReadsRight()
    {
        CarDashboard dashboard = new();
        CarConfig config = new();
        CarState state = new() { Energy = PowertrainState.Filled(0.62f) };
        state.InstallFreshTires(WarmTires());

        dashboard.Refresh(config, state, CarStrategy.Default);

        Assert.Single(dashboard.Resources);
        Assert.Equal("Charge", dashboard.Resources[0].Label);
        Assert.Equal(0.62f, dashboard.Resources[0].Fraction);
        // A pack weighs the same flat as full, so none of this car's mass is
        // in the store and the reading should say so.
        Assert.Equal(0f, dashboard.Resources[0].RemainingMassKg);
        Assert.Equal(config.MassKg, dashboard.MassKg, 3);
        Assert.Equal(1f, dashboard.OutputAvailability);
        Assert.Equal("Power", dashboard.Modes[1].Label);
        Assert.Equal("Normal", dashboard.Modes[1].Rung);
    }

    [Fact]
    public void TheSagIsVisibleOnTheGaugeBeforeItIsFeltInTheSeat()
    {
        CarDashboard dashboard = new();
        CarConfig config = new();
        CarState state = new() { Energy = PowertrainState.Filled(0.10f) };
        state.InstallFreshTires(WarmTires());

        dashboard.Refresh(config, state, CarStrategy.Default);

        // A tenth of a pack is half of where the limiter starts, and the
        // fall-off is squared, so a quarter of the output is left.
        Assert.Equal(0.25f, dashboard.OutputAvailability, 4);
    }

    [Fact]
    public void EveryPowertrainFitsWhatACarCanCarryAndSelect()
    {
        IPowertrain[] fitted =
        {
            ElectricPowertrain.Default,
            new TankAndBatteryPowertrain()
        };

        foreach (IPowertrain powertrain in fitted)
        {
            // A store past the last slot would silently never run down.
            Assert.InRange(
                powertrain.Resources.Count,
                1,
                PowertrainState.Capacity
            );
            // How long the ladder is, is the machinery's business; that it is
            // a choice at all is not.
            Assert.True(
                powertrain.OutputLadder.RungCount >= ModeLadder.MinimumRungs
            );
        }
    }

    [Fact]
    public void AnEngineWithSevenMapsIsUsableAllTheWayToTheSeventh()
    {
        // The shipped car has five settings, so five is everywhere in this
        // codebase by habit. Nothing may depend on it. An engine that says it
        // has seven gets seven: the panel counts to seven, names the seventh,
        // and the seventh buys pace the third does not.
        CarConfig config = new()
        {
            Powertrain = new TankAndBatteryPowertrain(SevenMaps)
        };

        CarDashboard dashboard = new();
        CarState state = new() { Energy = PowertrainState.Filled(1f) };
        state.InstallFreshTires(WarmTires());
        dashboard.Refresh(
            config,
            state,
            new CarStrategy(TireUsageMode.Normal, 7)
        );

        Assert.Equal(7, dashboard.Modes[1].RungCount);
        Assert.Equal(7, dashboard.Modes[1].Ordinal);
        Assert.Equal("Qualifying", dashboard.Modes[1].Rung);
        // The tyre ladder alongside it is untouched at five, because that one
        // belongs to the game rather than to the machinery.
        Assert.Equal(5, dashboard.Modes[0].RungCount);

        Assert.True(
            SpeedAfterAStint(config, 7) > SpeedAfterAStint(config, 3),
            "the seventh map should be worth something over the third"
        );
    }

    [Fact]
    public void AnEngineWithThreeMapsIsAlsoFine()
    {
        CarConfig config = new()
        {
            Powertrain = new TankAndBatteryPowertrain(ThreeMaps)
        };

        Assert.Equal(
            2,
            CarStrategy.DefaultFor(config.Powertrain).PowerRung
        );
        Assert.True(
            SpeedAfterAStint(config, 3) > SpeedAfterAStint(config, 1),
            "the top of a three-rung ladder should still be the top"
        );
    }

    [Fact]
    public void ASettingFromALongerLadderLandsOnTheTopOfAShorterOne()
    {
        // A saved setup outlives the car it was written for. Asked for a
        // seventh setting, a five-rung pack gives its fifth rather than
        // refusing to start or quietly giving its middle.
        ElectricPowertrain pack = ElectricPowertrain.Default;

        Assert.Equal(
            pack.GetDrivePowerLimitWatts(PowerOutputMode.Attack),
            pack.GetDrivePowerLimitWatts(7)
        );
        Assert.Equal(
            pack.GetDrivePowerLimitWatts(PowerOutputMode.Save),
            pack.GetDrivePowerLimitWatts(0)
        );
    }

    [Fact]
    public void ALadderNeedsMoreThanOneRungToBeAChoice()
    {
        Assert.Throws<ArgumentException>(
            () => new ModeLadder("Engine", "The only setting")
        );
    }

    private static readonly ModeLadder SevenMaps = new(
        "Engine",
        "Lean",
        "Economy",
        "Standard",
        "Fast",
        "Rich",
        "Overtake",
        "Qualifying"
    );

    private static readonly ModeLadder ThreeMaps = new(
        "Engine",
        "Lean",
        "Standard",
        "Rich"
    );

    /// <summary>
    /// Drives flat out on one setting for a while and reports the speed
    /// reached, which is how a rung proves it is worth more than the one
    /// below it.
    /// </summary>
    private static float SpeedAfterAStint(CarConfig config, int rung)
    {
        TireConfig tires = WarmTires();
        CarState state = new()
        {
            Speed = 40f,
            Energy = PowertrainState.Filled(1f)
        };
        state.InstallFreshTires(tires);
        CarStrategy strategy = new(TireUsageMode.Normal, rung);

        for (int i = 0; i < 600; i++)
        {
            CarPhysics.Step(
                state,
                config,
                tires,
                new CarPhysicsStepInput(
                    new DriverInput(0f, 12f),
                    strategy,
                    28f
                ),
                Dt
            );
        }

        return state.Speed;
    }

    private static float TotalWheelLoadAfterAStep(float tankFraction)
    {
        CarConfig config = FuelCar();
        TireConfig tires = WarmTires();
        CarState state = new()
        {
            Speed = 30f,
            Energy = new PowertrainState(tankFraction, 1f)
        };
        state.InstallFreshTires(tires);
        CarPhysics.Step(
            state,
            config,
            tires,
            new CarPhysicsStepInput(
                new DriverInput(0f, 0f),
                CarStrategy.Default,
                28f
            ),
            Dt
        );
        return state.FrontLeft.LoadN + state.FrontRight.LoadN +
               state.RearLeft.LoadN + state.RearRight.LoadN;
    }

    private static CarConfig FuelCar()
    {
        return new CarConfig { Powertrain = new TankAndBatteryPowertrain() };
    }

    private static TireConfig WarmTires()
    {
        return new TireConfig
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
    }

    /// <summary>
    /// Two stores, one of which weighs something, and a ladder whose rungs
    /// are engine maps. Crude on purpose - see the note on the class above.
    /// </summary>
    private sealed class TankAndBatteryPowertrain : IPowertrain
    {
        private const float TankJoules = 900000000f;

        private readonly ModeLadder _maps;

        public TankAndBatteryPowertrain(ModeLadder? maps = null)
        {
            _maps = maps ?? Maps;
        }

        private static readonly PowertrainResourceInfo[] Stores =
        {
            new("fuel", "Fuel", FullTankMassKg),
            new("battery", "Charge", 0f)
        };

        private static readonly ModeLadder Maps = new(
            "Engine",
            "Lean",
            "Economy",
            "Standard",
            "Rich",
            "Qualifying"
        );

        public IReadOnlyList<PowertrainResourceInfo> Resources => Stores;

        public ModeLadder OutputLadder => _maps;

        public float DriveAccelerationLimit(
            in PowertrainState energy,
            in CarStrategy strategy,
            float speed,
            float massKg,
            float chassisForceCeiling
        )
        {
            if (energy.Primary <= 0f)
                return 0f;

            // Each map up the ladder is worth a little more power, whatever
            // length the ladder happens to be.
            float watts = 300000f + 15000f * _maps.Clamp(strategy.PowerRung);
            float powerLimited = watts / (massKg * MathF.Max(speed, 8f));
            return MathF.Min(chassisForceCeiling, powerLimited);
        }

        public PowertrainSettlement Settle(
            in PowertrainState energy,
            float driveAccel,
            float brakeAccel,
            float speed,
            float massKg,
            float dt
        )
        {
            float drivePower = driveAccel <= 0f
                ? 0f
                : massKg * driveAccel * speed / 0.3f;
            PowertrainState spent = energy;
            spent.Primary = Math.Clamp(
                spent.Primary - drivePower * dt / TankJoules,
                0f,
                1f
            );
            return new PowertrainSettlement(spent, drivePower);
        }

        // Nothing to recover: a friction brake turns speed into heat and
        // that is the end of it.
        public float RecoveredPowerWatts(float brakeAccel, float speed, float massKg) => 0f;

        public float ConsumableMassKg(in PowertrainState energy)
        {
            return FullTankMassKg * Math.Clamp(energy.Primary, 0f, 1f);
        }

        public float OutputAvailability(in PowertrainState energy) => 1f;
    }
}
