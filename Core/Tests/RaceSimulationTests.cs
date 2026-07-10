using System.Numerics;
using TheStint.Core.Cars;
using TheStint.Core.Racing;
using TheStint.Core.Track;
using Xunit;

namespace TheStint.Core.Tests;

public sealed class RaceSimulationTests
{
    [Fact]
    public void StepFeedsTrackContextToDriverAndUpdatesProgress()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        FixedDriver driver = new(new DriverInput(0f, 1.5f));
        RaceCar car = CreateRaceCar("player", track, start, speed: 20f, driver);
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        simulation.Step(0.1f);

        Assert.Equal(1, driver.InitCount);
        Assert.Same(car, driver.LastInitContext.Car);
        Assert.Equal(track, driver.LastInitContext.Track);
        Assert.True(driver.CallCount > 0);
        Assert.Same(car, driver.LastContext.Car);
        Assert.Equal(track, driver.LastContext.Track);
        Assert.True(car.Progress.TotalDistance > 0f, "car progress should advance along the projected track");
        Assert.True(simulation.RaceTimeSeconds > 0f, "race clock should advance after stepping");
        Assert.Equal(driver.Input, car.LastInput);
    }

    [Fact]
    public void RaceEnvironmentAirTemperatureFeedsTireCooling()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);

        RaceCar coldAir = CreateRaceCar("cold", track, start, speed: 0f, new FixedDriver(new DriverInput(0f, 0f)));
        RaceCar hotAir = CreateRaceCar("hot", track, start, speed: 0f, new FixedDriver(new DriverInput(0f, 0f)));
        SetTireTemps(coldAir.State, surfaceTempC: 112f, coreTempC: 104f);
        SetTireTemps(hotAir.State, surfaceTempC: 112f, coreTempC: 104f);

        RaceSimulation coldSimulation = new(track, new RaceEnvironment { AirTempC = 10f });
        RaceSimulation hotSimulation = new(track, new RaceEnvironment { AirTempC = 45f });
        coldSimulation.AddCar(coldAir);
        hotSimulation.AddCar(hotAir);

        StepMany(coldSimulation, steps: 120);
        StepMany(hotSimulation, steps: 120);

        Assert.True(
            AverageSurfaceTemp(coldAir.State) < AverageSurfaceTemp(hotAir.State),
            "colder race air should cool tire surfaces more than hotter race air"
        );
        Assert.True(
            AverageCoreTemp(coldAir.State) < AverageCoreTemp(hotAir.State),
            "race air temperature should also affect slow core cooling"
        );
    }

    [Fact]
    public void AddCarAndStepKeepCarBodyInsideTrackWalls()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        CarCollisionConfig collision = new() { WidthMeters = 1.8f };
        float outsideLeftWallD = start.HalfWidth + start.LeftBufferWidth + 0.8f;
        CarState state = new()
        {
            Position = start.Center + start.Normal * outsideLeftWallD,
            Heading = start.RefHeading,
            Speed = 0f,
            BatterySoc = 0.8f
        };
        RaceCar car = new(
            "wall-test",
            new CarConfig(),
            WarmTires(),
            new FixedDriver(new DriverInput(0f, 0f)),
            state,
            collision
        );
        RaceSimulation simulation = new(track);

        simulation.AddCar(car);
        AssertCarInsideTrackWalls(track, car, collision);
        Assert.True(car.LastBoundaryContact.HasValue, "initial wall correction should report a boundary contact");

        car.State.Position = start.Center + start.Normal * outsideLeftWallD;
        simulation.Step(1f / 60f);

        AssertCarInsideTrackWalls(track, car, collision);
        Assert.True(car.LastBoundaryContact.HasValue, "step correction should report a boundary contact");
        Assert.Equal(TrackSide.Left, car.LastBoundaryContact.Value.Side);
        Assert.True(car.Progress.HitWallThisFrame, "race progress should expose wall contact for later systems");
    }

    [Fact]
    public void BoundaryResolverUsesCarCorners()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        CarCollisionConfig collision = new() { LengthMeters = 4.8f, WidthMeters = 1.8f };
        var limits = TrackBoundaryResolver.GetWallLimits(start);
        float centerD = limits.LeftWallD - collision.HalfLengthMeters * 0.5f;
        CarState state = new()
        {
            Position = start.Center + start.Normal * centerD,
            Heading = MathF.Atan2(start.Normal.Y, start.Normal.X),
            Speed = 0f,
            BatterySoc = 0.8f
        };

        Assert.False(
            TrackBoundaryResolver.IsInsideTrackWalls(track, state, collision),
            "a protruding nose corner should count as crossing the wall even when the center is inside"
        );

        TrackBoundaryContact? contact = TrackBoundaryResolver.ResolveCurrent(track, state, collision);

        Assert.True(contact.HasValue);
        Assert.True(TrackBoundaryResolver.IsInsideTrackWalls(track, state, collision));
    }

    [Fact]
    public void SweepStopsCarBeforeWallCrossing()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        CarCollisionConfig collision = new() { LengthMeters = 4.8f, WidthMeters = 1.8f };
        var limits = TrackBoundaryResolver.GetWallLimits(start);
        float centerD = limits.LeftWallD - collision.HalfLengthMeters - 0.4f;
        CarState state = new()
        {
            Position = start.Center + start.Normal * centerD,
            Heading = MathF.Atan2(start.Normal.Y, start.Normal.X),
            Speed = 42f,
            BatterySoc = 0.8f
        };
        RaceCar car = new(
            "wall-sweep",
            new CarConfig(),
            WarmTires(),
            new FixedDriver(new DriverInput(0f, 0f)),
            state,
            collision
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        simulation.Step(1f / 30f);

        AssertCarInsideTrackWalls(track, car, collision);
        Assert.True(car.LastBoundaryContact.HasValue);
        Assert.InRange(car.LastBoundaryContact.Value.ImpactFraction, 0f, 1f);
        Assert.True(car.State.Speed < 42f, "wall contact should absorb some impact energy");
    }

    [Fact]
    public void RightWallSweepReportsRightSideAndKeepsBodyInside()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        CarCollisionConfig collision = new() { LengthMeters = 4.8f, WidthMeters = 1.8f };
        var limits = TrackBoundaryResolver.GetWallLimits(start);
        float centerD = limits.RightWallD + collision.HalfLengthMeters + 0.4f;
        CarState state = new()
        {
            Position = start.Center + start.Normal * centerD,
            Heading = MathF.Atan2(-start.Normal.Y, -start.Normal.X),
            Speed = 42f,
            BatterySoc = 0.8f
        };
        RaceCar car = new(
            "right-wall-sweep",
            new CarConfig(),
            WarmTires(),
            new FixedDriver(new DriverInput(0f, 0f)),
            state,
            collision
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        simulation.Step(1f / 30f);

        AssertCarInsideTrackWalls(track, car, collision);
        Assert.True(car.LastBoundaryContact.HasValue);
        Assert.Equal(TrackSide.Right, car.LastBoundaryContact.Value.Side);
        Assert.True(car.State.Speed < 42f, "right wall contact should absorb some impact energy");
    }

    [Fact]
    public void CarContactResolverSeparatesOverlappingRectangles()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        CarCollisionConfig collision = new() { LengthMeters = 4.8f, WidthMeters = 1.8f };
        RaceCar a = CreateRaceCar("a", track, start, speed: 0f, new FixedDriver(new DriverInput(0f, 0f)), collision);
        RaceCar b = CreateRaceCar("b", track, start, speed: 0f, new FixedDriver(new DriverInput(0f, 0f)), collision);
        b.State.Position += new Vector2(MathF.Cos(start.RefHeading), MathF.Sin(start.RefHeading)) * 2f;
        RaceSimulation simulation = new(track);
        simulation.AddCar(a);
        simulation.AddCar(b);

        Assert.True(CarContactResolver.AreOverlapping(a, b));

        simulation.Step(1f / 60f);

        Assert.False(CarContactResolver.AreOverlapping(a, b));
        AssertCarInsideTrackWalls(track, a, collision);
        AssertCarInsideTrackWalls(track, b, collision);
    }

    [Fact]
    public void CarContactTransfersSpeedFromRearCarToFrontCar()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        CarCollisionConfig collision = new() { LengthMeters = 4.8f, WidthMeters = 1.8f };
        Vector2 forward = new(MathF.Cos(start.RefHeading), MathF.Sin(start.RefHeading));
        RaceCar rear = CreateRaceCar("rear", track, start, speed: 30f, new FixedDriver(new DriverInput(0f, 0f)), collision);
        RaceCar front = CreateRaceCar("front", track, start, speed: 0f, new FixedDriver(new DriverInput(0f, 0f)), collision);
        front.State.Position += forward * 4f;
        RaceSimulation simulation = new(track);
        simulation.AddCar(rear);
        simulation.AddCar(front);

        simulation.Step(1f / 60f);

        Assert.True(rear.State.Speed < 30f, "rear car should lose speed in a front impact");
        Assert.True(front.State.Speed > 0f, "front car should receive speed from the impact");
        Assert.False(CarContactResolver.AreOverlapping(rear, front));
    }

    [Fact]
    public void RaceProgressWrapsForwardAcrossStartLine()
    {
        TrackData track = BuildTrack();
        RaceProgress progress = new();
        TrackPose beforeLine = track.Project(track.Sample(track.LengthMeters - 2f).Center);
        TrackPose afterLine = track.Project(track.Sample(3f).Center);

        progress.Reset(beforeLine, TrackBoundaryResolver.Classify(beforeLine));
        progress.Update(track, afterLine, TrackBoundaryResolver.Classify(afterLine), hitWallThisFrame: false);

        Assert.InRange(progress.LastDeltaS, 4.5f, 5.5f);
        Assert.InRange(progress.TotalDistance, 4.5f, 5.5f);
    }

    private static void AssertCarInsideTrackWalls(TrackData track, RaceCar car, CarCollisionConfig collision)
    {
        Assert.True(
            TrackBoundaryResolver.IsInsideTrackWalls(track, car.State, collision),
            "the whole car body should be inside the wall envelope"
        );
    }

    private static RaceCar CreateRaceCar(
        string id,
        TrackData track,
        TrackSample start,
        float speed,
        IRaceDriver driver,
        CarCollisionConfig? collision = null
    )
    {
        return new RaceCar(
            id,
            new CarConfig(),
            WarmTires(),
            driver,
            new CarState
            {
                Position = start.Center,
                Heading = start.RefHeading,
                Speed = speed,
                BatterySoc = 0.8f
            },
            collision
        );
    }

    private static TrackData BuildTrack()
    {
        return new TrackBuilder(new Vector2(0f, 0f), startWidth: 10f, startLeftBuffer: 3f, startRightBuffer: 3f)
            .AddStraight(80f)
            .AddTurn(180f, 24f)
            .AddStraight(80f)
            .AddTurn(180f, 24f)
            .CloseLoop()
            .Build(new TrackGridConfig
            {
                StartingLineIdx = 0,
                GridCount = 8,
                GridOffset = 2f,
                FirstGridIdx = 12,
                IsFirstGridLeft = true,
                GridStepDist = 8
            });
    }

    private static TireConfig WarmTires()
    {
        return new TireConfig
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
    }

    private static void StepMany(RaceSimulation simulation, int steps)
    {
        for (int i = 0; i < steps; i++)
            simulation.Step(1f / 60f);
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

    private sealed class FixedDriver(DriverInput input) : IRaceDriver
    {
        public DriverInput Input { get; } = input;
        public int InitCount { get; private set; }
        public int CallCount { get; private set; }
        public RaceDriverInitContext LastInitContext { get; private set; }
        public RaceDriverFrameContext LastContext { get; private set; }

        public void Initialize(in RaceDriverInitContext context)
        {
            InitCount++;
            LastInitContext = context;
        }

        public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
        {
            CallCount++;
            LastContext = context;
            return Input;
        }
    }
}
