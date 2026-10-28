using System;
using System.Collections.Generic;
using Official.SingleTrack;
using StintegyEVO.Core.Cars;
using Xunit;

namespace Official.SingleTrack.Tests;

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

        // Fuel weighs what it weighs and buys no downforce: the air presses
        // the same on a full car as on an empty one.
        float expected = FullTankMassKg * 9.80665f;

        Assert.Equal(expected, fullTank - emptyTank, 1);
    }

    [Fact]
    public void ACarCarryingFuelIsHeavierThanOneThatHasBurnedIt()
    {
        CarConfig config = FuelCar();

        Near.Equal(
            config.MassKg + FullTankMassKg,
            CarPhysics.TotalMassKg(config, PowertrainState.Filled(1f)));
        Near.Equal(
            config.MassKg + FullTankMassKg * 0.4f,
            CarPhysics.TotalMassKg(config, new PowertrainState(0.4f, 1f)));
        Near.Equal(
            config.MassKg,
            CarPhysics.TotalMassKg(config, PowertrainState.Filled(0f)));
    }

    [Fact]
    public void DrivingBurnsTheFirstStoreAndLeavesTheSecondAlone()
    {
        CarConfig config = FuelCar();
        TestTires tires = WarmTires();
        CarState state = new CarState()
        {
            Speed = 55f
        }.WithEnergy(PowertrainState.Filled(1f));
        state.InstallFreshTires(tires);

        for (int i = 0; i < 600; i++)
        {
            CarPhysics.Step(
                state,
                config,
                tires,
                new CarStepInput(
                    new DriverInput(0f, 8f),
                    CarStrategy.Default,
                    28f
                ),
                Dt
            );
        }

        Assert.InRange(state.Chassis().Energy.Primary, 0.01f, 0.999f);
        Assert.Equal(1f, state.Chassis().Energy.Secondary);
        Assert.True(
            CarPhysics.TotalMassKg(config, state.Chassis().Energy) <
            config.MassKg + FullTankMassKg,
            "a car that has been driving should have burned something"
        );
    }

    [Fact]
    public void TheDashboardReadsWhateverIsFitted()
    {
        CarDashboard dashboard = new();
        CarState state = new CarState() { }.WithEnergy(new PowertrainState(0.5f, 0.25f));
        state.InstallFreshTires(WarmTires());

        dashboard.Refresh(
            new SingleTrackCar(FuelCar(), WarmTires()),
            state,
            new CarStrategy(TireUsageMode.Push, PowerOutputMode.Save)
        );

        // Two stores, named by the machinery rather than by the panel.
        Assert.Equal(2, dashboard.Resources.Count);
        Assert.Equal("Fuel", dashboard.Resources[0].Label);
        Assert.Equal(0.5f, dashboard.Resources[0].Fraction);
        Near.Equal(FullTankMassKg * 0.5f, dashboard.Resources[0].RemainingMassKg);
        Assert.Equal("Charge", dashboard.Resources[1].Label);
        Assert.Equal(0.25f, dashboard.Resources[1].Fraction);

        // Two ladders: the universal one for tyres, and the engine's own
        // words for the other.
        Assert.Equal(2, dashboard.Modes.Count);
        Assert.Equal("Tyres", dashboard.Modes[0].Label);
        Assert.Equal("Push", dashboard.Modes[0].Nearest);
        Assert.Equal("Engine", dashboard.Modes[1].Label);
        Assert.Equal("Lean", dashboard.Modes[1].Nearest);
        Assert.Equal(0f, dashboard.Modes[1].Setting);

        Assert.Equal(4, dashboard.Tires.Count);
        Assert.Equal(1f, dashboard.Tires[0].LifeFraction);
    }

    [Fact]
    public void TheDashboardOfTheCarWeActuallyShipStillReadsRight()
    {
        CarDashboard dashboard = new();
        CarConfig config = new();
        CarState state = new CarState() { }.WithEnergy(PowertrainState.Filled(0.62f));
        state.InstallFreshTires(WarmTires());

        dashboard.Refresh(new SingleTrackCar(config, WarmTires()), state, CarStrategy.Default);

        // The range extender: a pack to drive from and fuel to refill it.
        var powertrain = (RangeExtenderPowertrain)config.Powertrain;
        Assert.Equal(2, dashboard.Resources.Count);
        Assert.Equal("Charge", dashboard.Resources[0].Label);
        Assert.Equal(0.62f, dashboard.Resources[0].Fraction);
        // A pack weighs the same flat as full, so none of its mass is in the
        // store; the fuel is, and the car is that much heavier for it.
        Assert.Equal(0f, dashboard.Resources[0].RemainingMassKg);
        Assert.Equal("Fuel", dashboard.Resources[1].Label);
        Assert.Equal(0.62f, dashboard.Resources[1].Fraction);
        Near.Equal(0.62f * powertrain.FuelAllowanceKg, dashboard.Resources[1].RemainingMassKg);
        Near.Equal(config.MassKg + 0.62f * powertrain.FuelAllowanceKg, dashboard.MassKg);
        Assert.Equal(1f, dashboard.OutputAvailability);
        Assert.Equal("Power", dashboard.Modes[1].Label);
        Assert.Equal("Normal", dashboard.Modes[1].Nearest);
    }

    [Fact]
    public void TheSagIsVisibleOnTheGaugeBeforeItIsFeltInTheSeat()
    {
        CarDashboard dashboard = new();
        CarConfig config = new();
        CarState state = new CarState() { }.WithEnergy(PowertrainState.Filled(0.02f));
        state.InstallFreshTires(WarmTires());

        dashboard.Refresh(new SingleTrackCar(config, WarmTires()), state, CarStrategy.Default);

        // Two per cent is two fifths of the way up the sag at the bottom of
        // the usable window, so the gauge shows the sag's floor plus two
        // fifths of what is left above it. Written from the constants, so
        // that moving the sag moves this with it instead of breaking it.
        ElectricPowertrain pack = ElectricPowertrain.Default;
        float expected = pack.LowVoltageOutputFraction +
                         (1f - pack.LowVoltageOutputFraction) * (0.02f / pack.LowVoltageStart);
        Near.Equal(expected, dashboard.OutputAvailability);
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
        // has seven gets seven: the top of its travel is the seventh, the
        // panel names it, and it buys pace the third does not.
        CarConfig config = new()
        {
            Powertrain = new TankAndBatteryPowertrain(SevenMaps)
        };

        CarDashboard dashboard = new();
        CarState state = new CarState() { }.WithEnergy(PowertrainState.Filled(1f));
        state.InstallFreshTires(WarmTires());
        dashboard.Refresh(
            new SingleTrackCar(config, WarmTires()),
            state,
            new CarStrategy(TireLadder.Usage.DefaultSetting, 1f)
        );

        Assert.Equal("Qualifying", dashboard.Modes[1].Nearest);
        // The tyre ladder alongside it keeps its own five names, because that
        // one belongs to the game rather than to the machinery.
        Assert.Equal("Normal", dashboard.Modes[0].Nearest);

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

        // Nobody has set it, so it sits in the middle of the ladder it has.
        Assert.Equal(
            2,
            config.Powertrain.OutputLadder.Nearest(CarStrategy.Default.Setting(0, config.Powertrain.OutputLadder))
        );
        Assert.True(
            SpeedAfterAStint(config, 3) > SpeedAfterAStint(config, 1),
            "the top of a three-rung ladder should still be the top"
        );
    }

    [Fact]
    public void ASettingPastTheEndsOfTheTravelLandsOnThem()
    {
        // A saved setup outlives the car it was written for. Asked for more
        // than the top, a pack gives its top rather than refusing to start
        // or quietly giving its middle; asked for less than the bottom, its
        // bottom.
        ElectricPowertrain pack = ElectricPowertrain.Default;

        Assert.Equal(
            pack.GetDrivePowerLimitWatts(PowerOutputMode.Attack),
            pack.GetDrivePowerLimitWatts(1.4f)
        );
        Assert.Equal(
            pack.GetDrivePowerLimitWatts(PowerOutputMode.Save),
            pack.GetDrivePowerLimitWatts(-0.3f)
        );
    }

    [Fact]
    public void TheNamedMarksAreTheNamedSettingsAndBetweenTwoIsBetweenThem()
    {
        // The travel is continuous, but a setting on a mark is exactly that
        // named setting, and one partway from a mark to the next asks for
        // power partway between the two.
        ElectricPowertrain pack = ElectricPowertrain.Default;
        ModeLadder ladder = pack.OutputLadder;
        for (int rung = 1; rung <= ladder.RungCount; rung++)
            Assert.Equal(pack.GetDrivePowerLimitWatts(rung), pack.GetDrivePowerLimitWatts(ladder.Mark(rung)));
        for (int rung = 1; rung < ladder.RungCount; rung++)
        {
            float between = pack.GetDrivePowerLimitWatts((ladder.Mark(rung) + ladder.Mark(rung + 1)) / 2f);
            float low = MathF.Min(pack.GetDrivePowerLimitWatts(rung), pack.GetDrivePowerLimitWatts(rung + 1));
            float high = MathF.Max(pack.GetDrivePowerLimitWatts(rung), pack.GetDrivePowerLimitWatts(rung + 1));
            Assert.InRange(between, low, high);
            if (high > low)
                Assert.True(between > low && between < high, $"between rungs {rung} and {rung + 1}");
        }
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
        TestTires tires = WarmTires();
        CarState state = new CarState()
        {
            Speed = 40f
        }.WithEnergy(PowertrainState.Filled(1f));
        state.InstallFreshTires(tires);
        CarStrategy strategy = new(TireLadder.Usage.DefaultSetting, config.Powertrain.OutputLadder.Mark(rung));

        for (int i = 0; i < 600; i++)
        {
            CarPhysics.Step(
                state,
                config,
                tires,
                new CarStepInput(
                    new DriverInput(0f, 12f),
                    strategy,
                    28f
                ),
                Dt
            );
        }

        return state.Speed;
    }

    /// <summary>
    /// The air pushes on the car's shape, not on its mass: the drag force at
    /// a speed is the same on a full tank as on an empty one, so the full car
    /// slows less for it, by exactly the ratio of the masses.
    /// </summary>
    [Fact]
    public void FuelAboardDoesNotChangeTheAirsForce()
    {
        CarConfig config = FuelCar();
        float Drag(float tankFraction)
        {
            TestTires tires = WarmTires();
            CarState state = new CarState() { Speed = 70f }.WithEnergy(new PowertrainState(tankFraction, 1f));
            state.InstallFreshTires(tires);
            CarPhysics.Step(state, config, tires, new CarStepInput(new DriverInput(0f, 0f), CarStrategy.Default, 28f), Dt);
            float mass = config.MassKg + FullTankMassKg * tankFraction;
            return (-state.Chassis().Telemetry.ActualLongitudinalAccel - config.RollingDragAccel) * mass;
        }
        Assert.Equal(Drag(0f), Drag(1f), 0);
        Assert.True(Drag(0f) > 1000f, "at 70 m/s the air is a few kilonewtons");
    }

    private static float TotalWheelLoadAfterAStep(float tankFraction)
    {
        CarConfig config = FuelCar();
        TestTires tires = WarmTires();
        CarState state = new CarState()
        {
            Speed = 30f
        }.WithEnergy(new PowertrainState(tankFraction, 1f));
        state.InstallFreshTires(tires);
        CarPhysics.Step(
            state,
            config,
            tires,
            new CarStepInput(
                new DriverInput(0f, 0f),
                CarStrategy.Default,
                28f
            ),
            Dt
        );
        return state.Chassis().FrontLeft.LoadN + state.Chassis().FrontRight.LoadN +
               state.Chassis().RearLeft.LoadN + state.Chassis().RearRight.LoadN;
    }

    private static CarConfig FuelCar()
    {
        return new CarConfig { Powertrain = new TankAndBatteryPowertrain() };
    }

    private static TestTires WarmTires()
    {
        return new(new TireConfig
        {
        }, 90f, 90f);
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
            float output,
            float speed,
            float massKg,
            float chassisForceCeiling,
            bool pushToPass = false
        )
        {
            if (energy.Primary <= 0f)
                return 0f;

            // Each map up the ladder is worth a little more power, whatever
            // length the ladder happens to be.
            float watts = 300000f + 15000f * (1f + _maps.RungsUp(output));
            float powerLimited = watts / (massKg * MathF.Max(speed, 8f));
            return MathF.Min(chassisForceCeiling, powerLimited);
        }

        public PowertrainSettlement Settle(
            in PowertrainState energy,
            float driveAccel,
            float brakeAccel,
            float speed,
            float massKg,
            float dt,
            bool pushToPass = false
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
