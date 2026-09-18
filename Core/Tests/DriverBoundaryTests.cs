using System.Numerics;
using System.Reflection;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class DriverBoundaryTests
{
    private static readonly Lazy<TrackData> Road = new(() =>
        new TrackBuilder(Vector2.Zero, 14f)
            .AddStraight(400f).AddTurn(180f, 50f)
            .AddStraight(400f).AddTurn(180f, 50f).CloseLoop().Build(new TrackGridConfig()));

    [Fact]
    public void CoreDefinesAControllerContractButShipsNoControllerImplementation()
    {
        Assembly core = typeof(IDriverController).Assembly;
        Assert.DoesNotContain(core.GetTypes(), t => t.IsClass && typeof(IDriverController).IsAssignableFrom(t) && t != typeof(IDriverController));
        Assert.DoesNotContain(core.GetReferencedAssemblies(), a =>
            a.Name!.Contains("Godot", StringComparison.OrdinalIgnoreCase) ||
            a.Name.Contains("Highs", StringComparison.OrdinalIgnoreCase) ||
            a.Name.Contains("Blas", StringComparison.OrdinalIgnoreCase));
        Assert.Equal(new[] { "GetControl", "Initialize" },
            typeof(IDriverController).GetMethods().Select(m => m.Name).Order().ToArray());
    }

    [Fact]
    public void ContextExposesTypedValuesNotLiveWorldEntitiesOrPhysicalOverrides()
    {
        var types = typeof(DriverContext).GetProperties().Select(p => p.PropertyType).ToArray();
        Assert.DoesNotContain(typeof(RaceCar), types);
        Assert.DoesNotContain(typeof(CarState), types);
        Assert.DoesNotContain(typeof(TrackData), types);
        Assert.DoesNotContain(typeof(RaceEnvironment), types);
        Assert.Equal(typeof(RaceCarSnapshot), typeof(DriverContext).GetProperty("Car")!.PropertyType);
        Assert.Equal(typeof(DriverTrackView), typeof(DriverContext).GetProperty("Track")!.PropertyType);
        Assert.All(typeof(DriverTrackView).GetProperties(), property => Assert.Null(property.SetMethod));
        Assert.DoesNotContain(typeof(DriverTrackView).GetMethods(), m => m.ReturnType == typeof(TrackData));
        Assert.Null(typeof(RaceCar).GetProperty("Driver")!.SetMethod);
    }

    [Fact]
    public void NoControllerIsAValidEntryAndDoesNotInventCommands()
    {
        RaceSimulation race = new(Road.Value);
        RaceCar car = Car("external", 50f);
        race.AddCar(car);
        Vector2 position = car.State.Position;
        race.Step(0.5f);
        Assert.Null(car.Driver);
        Assert.Equal(default, car.LastInput);
        Assert.Equal(position, car.State.Position);
        Assert.Equal(0f, car.State.Speed);
        Assert.InRange(race.RaceTimeSeconds, 0.49999f, 0.50001f);
        Assert.Throws<InvalidOperationException>(() => race.CaptureFrameContext(car));
    }

    [Fact]
    public void ExternalHeldCommandsDriveTheSamePhysicsWithoutAController()
    {
        RaceSimulation race = new(Road.Value);
        RaceCar car = Car("external", 50f);
        car.ExternalInput = new(0f, 5f);
        race.AddCar(car);
        Vector2 position = car.State.Position;
        race.Step(0.5f);
        Assert.True(car.State.Speed > 0f);
        Assert.NotEqual(position, car.State.Position);
        Assert.Equal(car.ExternalInput, car.LastInput);
        car.ExternalInput = new(0f, -5f);
        float speed = car.State.Speed;
        race.Step(0.2f);
        Assert.True(car.State.Speed < speed);
    }

    [Fact]
    public void ExternalControllerReceivesIdentityOnceThenFrozenDecisionFrames()
    {
        ProbeController controller = new();
        RaceCar car = Car("controlled", 50f, controller);
        car.ExternalInput = new(0f, 99f);
        RaceSimulation race = new(Road.Value);
        race.AddCar(car);
        Assert.Equal(1, controller.Initializations);
        Assert.Same(car.Driver!.Profile, controller.Initial.Profile);
        Assert.Equal(car.Id, controller.Initial.Car.Id);
        race.Step(1f / 30f);
        Assert.Equal(2, controller.Frames.Count);
        Assert.Equal(new DriverInput(0f, 2f), car.LastInput);
        Assert.Equal(0f, controller.Frames[0].RaceTimeSeconds);
        Assert.InRange(controller.Frames[1].RaceTimeSeconds, 0.01666f, 0.01667f);
        Assert.All(controller.Frames, c => Assert.Same(car.Driver.Profile, c.Profile));
    }

    [Fact]
    public void RetainingAFrameCannotAliasTyresResourcesEnvironmentOrLaterFrames()
    {
        ProbeController controller = new();
        RaceCar car = Car("controlled", 50f, controller);
        car.State.FrontLeft.SurfaceTempC = 87f;
        car.State.Energy = PowertrainState.Filled(0.7f);
        RaceSimulation race = new(Road.Value);
        race.AddCar(car);
        DriverContext frame = race.CaptureFrameContext(car);
        CarStrategy strategy = car.Strategy;
        Vector2 position = car.State.Position;
        car.State.FrontLeft.SurfaceTempC = 120f;
        car.State.Energy = PowertrainState.Filled(0.2f);
        car.Strategy = new(TireUsageMode.Attack, PowerOutputMode.Attack);
        race.Environment.AirTempC = 42f;
        race.Environment.TrackTempC = 66f;
        race.Environment.SurfaceGripScalar = 0.9f;
        race.Step(0.1f);
        Assert.Equal(position, frame.Car.Position);
        Assert.Equal(87f, frame.Car.FrontLeft.SurfaceTempC);
        Assert.Equal(0.7f, frame.Car.Resources[0].Fraction);
        Assert.Equal(strategy, frame.Car.Strategy);
        Assert.Equal(25f, frame.Environment.AirTempC);
        Assert.Equal(35f, frame.Environment.TrackTempC);
        Assert.Equal(1f, frame.Environment.SurfaceGripScalar);
        Assert.Equal(0f, frame.RaceTimeSeconds);
        Assert.NotEqual(position, race.CaptureFrame()[0].Position);
    }

    [Fact]
    public void CapturingFreshWakeDoesNotMutateTheRaceOrCallControllers()
    {
        ProbeController controller = new();
        RaceCar follower = Car("follower", 50f, controller);
        RaceCar leader = Car("leader", 65f);
        follower.State.Speed = leader.State.Speed = 20f;
        follower.State.AirVelocityDeficit = 0.123f;
        RaceSimulation race = new(Road.Value);
        race.AddCar(follower);
        race.AddCar(leader);
        RaceFrameSnapshot frame = race.CaptureFrame();
        Assert.True(frame[0].AirVelocityDeficit > 0f);
        Assert.Equal(0.123f, follower.State.AirVelocityDeficit);
        Assert.Empty(controller.Frames);
        Assert.Equal(0f, race.RaceTimeSeconds);
        Assert.Equal(default, follower.LastInput);
    }

    [Fact]
    public void AllHostCommandsAndStrategiesAreFrozenBeforeAnyControllerRuns()
    {
        RaceCar external = Car("external", 120f);
        external.ExternalInput = new(0f, 3f);
        CarStrategy original = external.Strategy;
        ProbeController controller = new()
        {
            OnDecision = _ =>
            {
                external.ExternalInput = new(0f, 11f);
                external.Strategy = new(TireUsageMode.Attack, PowerOutputMode.Attack);
            }
        };
        RaceSimulation race = new(Road.Value);
        race.AddCar(Car("controlled", 50f, controller));
        race.AddCar(external);
        race.Step(1f / 120f);
        Assert.Equal(new DriverInput(0f, 3f), external.LastInput);
        Assert.Equal(original, external.State.Telemetry.Strategy);
        race.Step(1f / 120f);
        Assert.Equal(new DriverInput(0f, 11f), external.LastInput);
    }

    [Theory]
    [InlineData(float.NaN)]
    [InlineData(float.PositiveInfinity)]
    [InlineData(float.NegativeInfinity)]
    public void InvalidCommandsAreRejectedBeforeAnyPhysicsRuns(float invalid)
    {
        ProbeController controller = new() { Input = new(invalid, 0f) };
        RaceCar car = Car("bad", 50f, controller);
        RaceSimulation race = new(Road.Value);
        race.AddCar(car);
        Vector2 position = car.State.Position;
        Assert.Throws<InvalidOperationException>(() => race.Step(1f / 60f));
        Assert.Equal(0f, race.RaceTimeSeconds);
        Assert.Equal(position, car.State.Position);
        controller.Input = new(0f, 1f);
        race.Step(1f / 60f); // Failure must release the re-entry guard.
        Assert.True(race.RaceTimeSeconds > 0f);
    }

    [Theory]
    [InlineData(float.NaN)]
    [InlineData(float.PositiveInfinity)]
    [InlineData(-1f)]
    public void InvalidTimeStepsCannotHangOrCorruptTheRace(float dt) =>
        Assert.Throws<ArgumentOutOfRangeException>(() => new RaceSimulation(Road.Value).Step(dt));

    [Fact]
    public void DuplicateEntriesOrSharedStatefulControllersAreRejected()
    {
        RaceSimulation race = new(Road.Value);
        ProbeController controller = new();
        race.AddCar(Car("first", 50f, controller));
        Assert.Throws<ArgumentException>(() => race.AddCar(Car("first", 100f)));
        Assert.Throws<ArgumentException>(() => race.AddCar(Car("second", 100f, controller)));
        Assert.Single(race.Cars);
    }

    [Fact]
    public void CallbacksCannotChangeTheEntryListOrNestSimulationSteps()
    {
        RaceSimulation race = new(Road.Value);
        ProbeController controller = new()
        {
            OnDecision = _ =>
            {
                Assert.Throws<InvalidOperationException>(() => race.AddCar(Car("late", 150f)));
                Assert.Throws<InvalidOperationException>(() => race.Step(0.1f));
            }
        };
        race.AddCar(Car("first", 50f, controller));
        race.Step(1f / 60f);
        Assert.Single(race.Cars);
    }

    [Fact]
    public void FailedInitializationDoesNotRegisterAHalfInitializedDriver()
    {
        RaceSimulation race = new(Road.Value);
        ProbeController controller = new() { FailInitialization = true };
        Assert.Throws<InvalidOperationException>(() => race.AddCar(Car("bad", 50f, controller)));
        Assert.Empty(race.Cars);
        race.AddCar(Car("good", 50f));
        race.Step(0.1f);
        Assert.Single(race.Cars);
    }

    [Fact]
    public void SnapshotIncludesTheAccumulatedSideslipState()
    {
        RaceCar car = Car("sliding", 50f);
        RaceSimulation race = new(Road.Value);
        race.AddCar(car);
        car.State.SideslipHoldSeconds = 0.12f;
        var first = race.CaptureFrame();
        car.State.SideslipHoldSeconds = 0.24f;
        var second = race.CaptureFrame();
        Assert.Equal(0.12f, first[0].SideslipHoldSeconds);
        Assert.Equal(0.24f, second[0].SideslipHoldSeconds);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FailedControlDoesNotPublishWakeCorrectionOrCommands(bool throws)
    {
        ProbeController controller = new()
        {
            Input = new(float.NaN, 0),
            OnDecision = _ => { if (throws) throw new ApplicationException("failed control"); }
        };
        RaceCar bad = Car("bad", 65f, controller);
        RaceCar follower = Car("follower", 50f);
        RaceSimulation race = new(Road.Value);
        race.AddCar(follower);
        race.AddCar(bad);
        follower.State.AirVelocityDeficit = 0.123f;
        follower.State.DownforceVelocityDeficit = 0.234f;
        follower.State.WakeDownforceLoss = 0.345f;
        follower.ExternalInput = new(0, 3);
        // Exercise provisional wall correction as well as the new wake values.
        TrackSample at = Road.Value.Sample(50);
        follower.State.Position = at.Center + at.Normal * 100f;
        Vector2 original = follower.State.Position;
        follower.BoundaryContactSeconds = 0.05f;
        follower.HitCarThisStep = true;
        Assert.ThrowsAny<Exception>(() => race.Step(1f / 60f));
        Assert.Equal(0f, race.RaceTimeSeconds);
        Assert.Equal(original, follower.State.Position);
        Assert.Equal(0.123f, follower.State.AirVelocityDeficit);
        Assert.Equal(0.234f, follower.State.DownforceVelocityDeficit);
        Assert.Equal(0.345f, follower.State.WakeDownforceLoss);
        Assert.Equal(default, follower.LastInput);
        Assert.Equal(0.05f, follower.BoundaryContactSeconds);
        Assert.True(follower.HitCarThisStep);
    }

    private static RaceCar Car(string id, float s, IDriverController? controller = null)
    {
        TrackSample at = Road.Value.Sample(s);
        Driver? driver = controller is null ? null : new Driver(new DriverProfile(id, new(), 42), controller);
        return new RaceCar(id, new(), new(), driver,
            new CarState { Position = at.Center, Heading = at.Heading });
    }

    private sealed class ProbeController : IDriverController
    {
        public int Initializations { get; private set; }
        public DriverContext Initial { get; private set; }
        public List<DriverContext> Frames { get; } = [];
        public DriverInput Input { get; set; } = new(0f, 2f);
        public Action<DriverContext>? OnDecision { get; init; }
        public bool FailInitialization { get; init; }
        public void Initialize(in DriverContext context)
        {
            if (FailInitialization) throw new InvalidOperationException("initialization failed");
            Initializations++;
            Initial = context;
        }
        public DriverInput GetControl(in DriverContext context, float dt)
        {
            Frames.Add(context);
            OnDecision?.Invoke(context);
            return Input;
        }
    }
}
