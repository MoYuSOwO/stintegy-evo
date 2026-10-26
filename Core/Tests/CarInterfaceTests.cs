using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The race knows a car only through <see cref="ICar"/> and the features the
/// car says it has. A car that shares nothing with the game's own chassis --
/// its own motion, its own numbers, its own instruments -- races, is seen by
/// a driver and fills a screen exactly as the shipped car does; a car with
/// nothing but a body races too, and a driver is not put in a car without
/// what that driver needs.
/// </summary>
public sealed class CarInterfaceTests
{
    [Fact]
    public void ACarOfItsOwnRacesThroughTheInterfaceAlone()
    {
        TrackData track = new TrackBuilder(Vector2.Zero, 14f)
            .AddStraight(400f).AddTurn(180f, 50f).AddStraight(400f).AddTurn(180f, 50f)
            .CloseLoop().Build(new TrackGridConfig());
        TrackSample start = track.Sample(10f);
        var model = new RollingBox();
        var car = new RaceCar("box", model, state: new CarState
        {
            Position = start.Center,
            Heading = start.Heading,
            Speed = 20f
        });
        var simulation = new RaceSimulation(track);
        simulation.AddCar(car);

        for (int i = 0; i < 60; i++)
            simulation.Step(1f / 60f);

        // Its own motion, not the chassis's: twenty metres in a second.
        Assert.InRange(car.Progress.TotalDistance, 19f, 21f);
        Assert.True(model.Steps > 0, "the race never stepped it");

        // The driver's frame carries what the car says about itself.
        RaceCarSnapshot seen = simulation.CaptureFrame()[0];
        Assert.Equal(RollingBox.Window, seen.Tires[0].Window);
        Assert.Equal("hard", seen.TireCompound);
        Assert.Equal(RollingBox.Brake, seen.MaximumBrakeDecelerationMetersPerSecondSquared);
        Assert.Equal(RollingBox.SteerLock, seen.Capabilities.MaxSteerAngleRadians);

        // And so does the screen.
        var dashboard = new CarDashboard();
        dashboard.Refresh(model, car.State, car.Strategy);
        Assert.Equal(4, dashboard.Tires.Count);
        Assert.Equal("hard", dashboard.Tires[0].Window.Compound);
        Assert.Contains(dashboard.Modes, m => m.Label == "Box ladder");

        // Its instruments are its own.
        Span<float> readings = stackalloc float[model.TelemetryChannels.Count];
        model.ReadTelemetry(car.State, readings);
        Assert.Equal("box_odometer", model.TelemetryChannels[0].Id);
        Assert.InRange(readings[0], 19f, 21f);
    }

    /// <summary>
    /// A car that is a body and nothing more -- no tyres to change, no
    /// stores, no buttons, no envelope -- races, and the frame says it has
    /// none of them rather than inventing any.
    /// </summary>
    [Fact]
    public void ACarThatIsOnlyABodyRaces()
    {
        TrackData track = new TrackBuilder(Vector2.Zero, 14f)
            .AddStraight(400f).AddTurn(180f, 50f).AddStraight(400f).AddTurn(180f, 50f)
            .CloseLoop().Build(new TrackGridConfig());
        TrackSample start = track.Sample(10f);
        var car = new RaceCar("body", new BareBody(), state: new CarState
        {
            Position = start.Center,
            Heading = start.Heading,
            Speed = 20f
        });
        var simulation = new RaceSimulation(track);
        simulation.AddCar(car);
        car.HoldButtonsUntilLine();
        car.Strategy = car.Strategy.WithButton(0, true);

        for (int i = 0; i < 60; i++)
            simulation.Step(1f / 60f);

        Assert.InRange(car.Progress.TotalDistance, 19f, 21f);
        RaceCarSnapshot seen = simulation.CaptureFrame()[0];
        Assert.Empty(seen.Tires);
        Assert.Null(seen.TireCompound);
        Assert.Empty(seen.Buttons);
        Assert.Empty(seen.Resources);
        Assert.Null(seen.GripUse);
    }

    /// <summary>
    /// A driver who cannot drive without feeling the grip is not put in a
    /// car that does not say; one who only uses it when it is there is.
    /// </summary>
    [Fact]
    public void ADriverIsNotPutInACarWithoutWhatItNeeds()
    {
        var needsFeel = new Driver(new DriverProfile("feel", new DriverAbilities()),
            new Needing(new CarNeeds([typeof(ICarGripFeel)], [])));
        var unfit = Assert.Throws<CarUnfitForDriverException>(() => new RaceCar("box", new RollingBox(), needsFeel));
        Assert.Equal([typeof(ICarGripFeel)], unfit.Missing);
        Assert.Contains(nameof(ICarGripFeel), unfit.Message);

        var usesFeel = new Driver(new DriverProfile("feel", new DriverAbilities()),
            new Needing(new CarNeeds([typeof(ICarEnvelope)], [typeof(ICarGripFeel)])));
        _ = new RaceCar("box", new RollingBox(), usesFeel);
        _ = new RaceCar("chassis", new SingleTrackCar(new CarConfig(), new TireConfig()), needsFeel);
    }

    private sealed class Needing(CarNeeds needs) : IDriverController
    {
        public CarNeeds Needs => needs;
        public DriverInput GetControl(in DriverContext context, float dt) => default;
    }

    /// <summary>
    /// A car with two ladders of its own -- an engine map and a hybrid's
    /// deployment -- has each set on its own, shown on the screen under its
    /// own names, and a strategy that never touched one leaves it where the
    /// ladder sits.
    /// </summary>
    [Fact]
    public void ACarDeclaresAsManyLaddersAsItHas()
    {
        var maps = new ModeLadder("engine_map", "Engine map", ["Lean", "Rich", "Qualifying"]);
        var deploy = new ModeLadder("deployment", "Deployment", ["Harvest", "Balanced", "Deploy", "Overtake"]);

        CarStrategy strategy = CarStrategy.Default.WithSetting(1, 1f);
        Assert.Equal(maps.DefaultSetting, strategy.Setting(0, maps));
        Assert.Equal(1f, strategy.Setting(1, deploy));
        Assert.Equal(1f, strategy.WithSetting(0, 9f).Setting(0, maps));
        Assert.Equal(CarStrategy.Default.WithSetting(1, 1f), strategy);

        var dashboard = new CarDashboard();
        foreach ((ModeLadder ladder, int i) in new[] { (maps, 0), (deploy, 1) })
            dashboard.AddMode(ladder, strategy.Setting(i, ladder));
        Assert.Equal(["Rich", "Overtake"], dashboard.Modes.Select(m => m.Nearest));
    }

    /// <summary>
    /// A car's own electronics on the game's chassis: built on it, with one
    /// thing done its own way, and nothing in the race able to tell.
    /// </summary>
    [Fact]
    public void ACarOnTheChassisCanBringItsOwnElectronics()
    {
        float SpeedAfter(ICar model)
        {
            var state = new CarState { Speed = 5f }.WithEnergy(PowertrainState.Filled(0.9f));
            model.As<ICarTires>().InstallFreshTires(state);
            var input = new CarPhysicsStepInput(new DriverInput(0f, 12f), CarStrategy.Default, 25f, 35f);
            for (int i = 0; i < 60; i++)
                model.Step(state, input, 1f / 60f);
            return state.Speed;
        }

        float plain = SpeedAfter(new SingleTrackCar(new CarConfig(), new TireConfig()));
        float governed = SpeedAfter(new LaunchControlledCar());
        Assert.True(plain > 10f, $"the plain car reached only {plain:F1} m/s");
        // Three metres a second squared for the second from five: eight.
        Assert.InRange(governed, 7f, 8.5f);
    }

    /// <summary>Holds a launch to three metres a second squared below twenty metres a second.</summary>
    private sealed class LaunchControlledCar() : SingleTrackCar(new CarConfig(), new TireConfig())
    {
        public override void Step(CarState state, CarPhysicsStepInput input, float dt)
        {
            if (state.Speed < 20f && input.DriverInput.DesiredAccel > 3f)
                input = input with { DriverInput = input.DriverInput with { DesiredAccel = 3f } };
            base.Step(state, input, dt);
        }
    }

    /// <summary>An engine with two settings and a tank that never empties.</summary>
    private sealed class BoxEngine : IPowertrain
    {
        public IReadOnlyList<PowertrainResourceInfo> Resources { get; } =
            [new PowertrainResourceInfo("box_fuel", "Fuel", 0f, PowertrainResourceClass.Fuel)];

        public ModeLadder OutputLadder { get; } = new("Box ladder", "Slow", "Fast");

        public float DriveAccelerationLimit(in PowertrainState energy, float output, float speed,
            float massKg, float chassisForceCeiling, bool pushToPass = false) => 5f;

        public PowertrainSettlement Settle(in PowertrainState energy, float driveAccel, float brakeAccel,
            float speed, float massKg, float dt, bool pushToPass = false) => new(energy, 0f);

        public float RecoveredPowerWatts(float brakeAccel, float speed, float massKg) => 0f;
        public float ConsumableMassKg(in PowertrainState energy) => 0f;
        public float OutputAvailability(in PowertrainState energy) => 1f;
    }

    /// <summary>A body that rolls straight on at its speed, and has nothing else at all.</summary>
    private sealed class BareBody : ICar
    {
        public CarSpec Spec { get; } = new()
        {
            WheelBaseMeters = 2.5f,
            FrontAxleOffsetMeters = 1.2f,
            RearAxleOffsetMeters = 1.3f,
            TrackWidthMeters = 1.5f,
            YawInertiaKgM2 = 1200f,
            MaximumBrakeDeceleration = 9f,
            KinematicBelowSpeed = 1f,
        };

        public IReadOnlyList<ModeLadder> Ladders => [];
        public float TotalMassKg(CarState state) => 800f;

        public void Step(CarState state, CarPhysicsStepInput input, float dt) =>
            state.Position += new Vector2(MathF.Cos(state.Heading), MathF.Sin(state.Heading)) * state.Speed * dt;

        public void Describe(CarDashboard dashboard, CarState state, CarStrategy strategy) => dashboard.SetMass(800f);
        public IReadOnlyList<TelemetryChannel> TelemetryChannels => [];
        public void ReadTelemetry(CarState state, Span<float> values) { }
    }

    /// <summary>
    /// A box that rolls straight on at its speed and turns where its nose
    /// points, on tyres it keeps in its own part of the state, with an
    /// envelope a driver can plan against.
    /// </summary>
    private sealed class RollingBox : ICar, ICarTires, ICarEnvelope
    {
        public const float Brake = 9f;
        public const float SteerLock = 0.5f;
        public static readonly TireWindow Window = new("hard", 60f, 90f, 110f);

        public int Steps { get; private set; }
        private float _odometer;

        private sealed class BoxState : ICarStateExtension
        {
            public string? Compound { get; set; }
            public ICarStateExtension Clone() => new BoxState { Compound = Compound };
            public void CopyFrom(ICarStateExtension other) => Compound = ((BoxState)other).Compound;
        }

        private static BoxState Own(CarState state) => (BoxState)(state.Own ??= new BoxState());

        public CarSpec Spec { get; } = new()
        {
            WheelBaseMeters = 2.5f,
            FrontAxleOffsetMeters = 1.2f,
            RearAxleOffsetMeters = 1.3f,
            TrackWidthMeters = 1.5f,
            YawInertiaKgM2 = 1200f,
            MaximumBrakeDeceleration = Brake,
            KinematicBelowSpeed = 1f,
        };

        private readonly BoxEngine _engine = new();

        public IReadOnlyList<ModeLadder> Ladders => [_engine.OutputLadder];

        public float TotalMassKg(CarState state) => 800f;

        public void Step(CarState state, CarPhysicsStepInput input, float dt)
        {
            Steps++;
            state.Position += new Vector2(MathF.Cos(state.Heading), MathF.Sin(state.Heading)) * state.Speed * dt;
            _odometer += state.Speed * dt;
        }

        public CarPerformanceLimits EstimatePerformanceLimits(
            CarState state, CarStrategy strategy, float speed, float curvature, float gripUsage = 1f,
            float assumedLongitudinalAcceleration = 0f, float frontBrakeBiasOffset = 0f) =>
            new(10f, 5f, Brake, 0f);

        public float EstimateCorneringLimit(CarState state, CarStrategy strategy, float speed, float gripUsage = 1f) => 10f;
        public SteeringEstimate EstimateSteering(CarState state, RoadAttitude road) =>
            new(state.Speed, Spec.WheelBaseMeters, Spec.FrontAxleOffsetMeters, Spec.RearAxleOffsetMeters,
                10f, 10f, 1f, 0f, SteerLock, 0f, 0f, road.CurvatureDemandScale, 0f);
        public float GripUsage(CarStrategy strategy) => 1f;

        public IReadOnlyList<TireCompound> Compounds { get; } = [TireCompound.Of(new TireConfig(), "hard")];
        public void FitTires(CarState state, string compound) => Own(state).Compound = compound;
        public void InstallFreshTires(CarState state) => Own(state).Compound ??= Compounds[0].Id;
        public void WarmTires(CarState state, float surfaceTempC, float coreTempC) { }
        public string? Compound(CarState state) => Own(state).Compound;
        public TireReading Tire(CarState state, WheelId wheel) => new(70f, 70f, 0f, 2000f, 1f) { Window = Window };
        public void Assume(CarState plan, string? compound, ReadOnlySpan<TireReading> tires) => Own(plan).Compound = compound;

        public CarCapabilities Capabilities(CarState state, CarStrategy strategy) =>
            new(800f, 0.2f, SteerLock, 1f, 5f, 5f, 1f);

        public AxleLayout Layout { get; } = new(0.5f, 0f, false, null);

        public void Describe(CarDashboard dashboard, CarState state, CarStrategy strategy)
        {
            foreach (string label in new[] { "FL", "FR", "RL", "RR" })
                dashboard.AddTire(new DashboardTire(label, 1f, 70f, 70f, 2000f, Window));
            dashboard.AddMode(_engine.OutputLadder, strategy.Setting(0, _engine.OutputLadder));
            dashboard.SetMass(800f);
        }

        public IReadOnlyList<TelemetryChannel> TelemetryChannels { get; } =
            [new TelemetryChannel("box_odometer", "Odometer", "m")];

        public void ReadTelemetry(CarState state, Span<float> values) => values[0] = _odometer;
    }
}
