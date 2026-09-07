using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class RaceSimulationTests
{
    [Fact]
    public void TheTowOutlivesTheDirtyAirFartherBack()
    {
        // The wake rises off the road as it ages: the follower's body stays
        // sunk in slowed air, so the tow keeps a long tail, while the
        // disturbance that costs downforce lives in the risen wake and is
        // gone within a few car lengths.
        (float closeTow, float closeDirtyAir) = MeasureWake(
            centerSeparationMeters: 12f,
            lateralOffsetMeters: 0f
        );
        (float farTow, float farDirtyAir) = MeasureWake(
            centerSeparationMeters: 45f,
            lateralOffsetMeters: 0f
        );

        Assert.True(closeTow > farTow && farTow > 0.01f);
        Assert.True(closeDirtyAir > farDirtyAir);
        Assert.True(
            farTow / closeTow > farDirtyAir / MathF.Max(closeDirtyAir, 1e-6f),
            "the tow must retain more of its strength far back than the dirty air does"
        );
    }

    [Fact]
    public void MovingSidewaysRecoversDownforceFasterThanItLosesTheTow()
    {
        (float centeredTow, float centeredDirtyAir) = MeasureWake(
            centerSeparationMeters: 12f,
            lateralOffsetMeters: 0f
        );
        (float offsetTow, float offsetDirtyAir) = MeasureWake(
            centerSeparationMeters: 12f,
            lateralOffsetMeters: 3f
        );

        Assert.True(offsetTow < centeredTow);
        Assert.True(offsetDirtyAir < centeredDirtyAir);
        Assert.True(
            offsetDirtyAir / centeredDirtyAir < offsetTow / centeredTow,
            "leaving the wake sideways should restore downforce especially quickly"
        );
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void PlanningReadsThePreviousFrozenPlanWithoutSeeingCurrentPlans(
        bool reverseOrder
    )
    {
        TrackData track = BuildTrack();
        PreviousFramePlanProbeDriver firstDriver = new("second");
        PreviousFramePlanProbeDriver secondDriver = new("first");
        RaceCar first = CreateRaceCar(
            "first",
            track,
            track.Sample(12f),
            speed: 12f,
            firstDriver
        );
        RaceCar second = CreateRaceCar(
            "second",
            track,
            track.Sample(42f),
            speed: 24f,
            secondDriver
        );
        RaceSimulation simulation = new(track);
        if (reverseOrder)
        {
            simulation.AddCar(second);
            simulation.AddCar(first);
        }
        else
        {
            simulation.AddCar(first);
            simulation.AddCar(second);
        }

        simulation.Step(1f / 120f);
        simulation.Step(1f / 120f);

        Assert.Equal(2, firstDriver.PrepareCount);
        Assert.Equal(2, secondDriver.PrepareCount);
        Assert.Equal([false, false], firstDriver.SawCurrentPlanDuringPrepare);
        Assert.Equal([false, false], secondDriver.SawCurrentPlanDuringPrepare);
        Assert.Equal([null, 101f], firstDriver.PreviousPlanSpeedsDuringPrepare);
        Assert.Equal([null, 101f], secondDriver.PreviousPlanSpeedsDuringPrepare);
        Assert.Equal([101f, 102f], firstDriver.CurrentPlanSpeedsDuringControl);
        Assert.Equal([101f, 102f], secondDriver.CurrentPlanSpeedsDuringControl);
        Assert.Equal([false, false], firstDriver.SawPreviousPlanDuringControl);
        Assert.Equal([false, false], secondDriver.SawPreviousPlanDuringControl);
    }

    [Fact]
    public void FirstDriverStepCollectsCurrentFramePlansBeforeControl()
    {
        TrackData track = BuildTrack();
        CurrentFramePlanProbeDriver firstDriver = new("second");
        CurrentFramePlanProbeDriver secondDriver = new("first");
        RaceCar first = CreateRaceCar(
            "first",
            track,
            track.Sample(12f),
            speed: 12f,
            firstDriver
        );
        RaceCar second = CreateRaceCar(
            "second",
            track,
            track.Sample(42f),
            speed: 24f,
            secondDriver
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(first);
        simulation.AddCar(second);

        simulation.Step(1f / 120f);

        Assert.Equal(1, firstDriver.PrepareCount);
        Assert.Equal(1, secondDriver.PrepareCount);
        Assert.True(firstDriver.SawOtherCurrentFramePlan);
        Assert.True(secondDriver.SawOtherCurrentFramePlan);
    }

    [Fact]
    public void ReferenceDriverPublishesAPlanOnTheFirstDriverStep()
    {
        TrackData track = BuildTrack();
        RaceCar planned = CreateRaceCar(
            "planned",
            track,
            track.Sample(12f),
            speed: 12f,
            new ReferenceLineDriver()
        );
        PlanObserverDriver observerDriver = new("planned");
        RaceCar observer = CreateRaceCar(
            "observer",
            track,
            track.Sample(80f),
            speed: 10f,
            observerDriver
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(planned);
        simulation.AddCar(observer);

        simulation.Step(1f / 120f);

        Assert.True(observerDriver.SawObservedPlan);
    }

    [Fact]
    public void ReferenceDriverPublishesItsPlannedPathSpeed()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(70f);
        ReferenceLineDriver plannedDriver = new();
        RaceCar planned = new(
            "planned",
            new CarConfig(),
            WarmTires(),
            plannedDriver,
            new CarState
            {
                Position = start.Center + start.Normal * 3f,
                Heading = start.RefHeading + 0.3f,
                Speed = 40f,
                Energy = PowertrainState.Filled(0.8f)
            }
        );
        PlanObserverDriver observerDriver = new("planned");
        RaceCar observer = CreateRaceCar(
            "observer",
            track,
            track.Sample(300f),
            speed: 10f,
            observerDriver
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(planned);
        simulation.AddCar(observer);

        simulation.Step(1f / 120f);

        float plannedStartSpeed =
            plannedDriver.CurrentSpeedLookahead.Sample(0f).TargetSpeed;
        Assert.True(
            MathF.Abs(
                plannedStartSpeed -
                plannedDriver.CurrentPathPrediction[0].EstimatedSpeed
            ) > 0.1f,
            "the fixture must distinguish the path speed plan from the predictor seed"
        );
        Assert.Equal(
            plannedStartSpeed,
            observerDriver.ObservedPlanStartSpeedMetersPerSecond,
            precision: 3
        );
    }

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
    public void DriverPlanningRunsAtSixtyHertzWhilePhysicsUsesSubsteps()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        FixedDriver driver = new(new DriverInput(0f, 1.5f));
        RaceCar car = CreateRaceCar(
            "player",
            track,
            start,
            speed: 20f,
            driver
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        simulation.Step(1f / 60f);
        Assert.Equal(1, driver.CallCount);

        simulation.Step(1f / 30f);
        Assert.Equal(3, driver.CallCount);
        Assert.InRange(
            simulation.RaceTimeSeconds,
            0.05f - 1e-5f,
            0.05f + 1e-5f
        );
    }

    [Fact]
    public void StepSharesOneFrozenVehicleFrameWithEveryDriver()
    {
        TrackData track = BuildTrack();
        TrackSample firstStart = track.Sample(12f);
        TrackSample secondStart = track.Sample(42f);
        SnapshotTrafficDriver firstDriver = new("second");
        SnapshotTrafficDriver secondDriver = new("first");
        RaceCar first = CreateRaceCar("first", track, firstStart, speed: 12f, firstDriver);
        RaceCar second = CreateRaceCar("second", track, secondStart, speed: 24f, secondDriver);
        RaceSimulation simulation = new(track);
        simulation.AddCar(first);
        simulation.AddCar(second);

        simulation.Step(1f / 120f);

        Assert.True(firstDriver.LastContext.HasFrameSnapshot);
        Assert.True(secondDriver.LastContext.HasFrameSnapshot);
        Assert.Equal("first", firstDriver.LastContext.CarSnapshot.Id);
        Assert.Equal("second", secondDriver.LastContext.CarSnapshot.Id);
        Assert.Equal(2, firstDriver.LastContext.Frame.Count);
        Assert.Equal(2, secondDriver.LastContext.Frame.Count);
        Assert.Equal(24f, firstDriver.ObservedSpeedMetersPerSecond);
        Assert.Equal(12f, secondDriver.ObservedSpeedMetersPerSecond);
        Assert.Equal(
            firstDriver.LastContext.Frame[0],
            secondDriver.LastContext.Frame[0]
        );
        Assert.Equal(
            firstDriver.LastContext.Frame[1],
            secondDriver.LastContext.Frame[1]
        );

        RaceFrameSnapshot capturedFrame = firstDriver.LastContext.Frame;
        first.State.Speed = 99f;
        Assert.True(capturedFrame.TryGetCar("first", out RaceCarSnapshot capturedFirst));
        Assert.Equal(12f, capturedFirst.SpeedMetersPerSecond);
    }

    [Fact]
    public void PhysicsResultsAreCommittedAfterEveryDriverEvaluates()
    {
        TrackData track = BuildTrack();
        TrackSample firstStart = track.Sample(12f);
        TrackSample secondStart = track.Sample(42f);
        FixedDriver acceleratingDriver = new(new DriverInput(0f, 8f));
        RaceCar first = CreateRaceCar("first", track, firstStart, speed: 12f, acceleratingDriver);
        LiveSpeedProbeDriver probeDriver = new(first);
        RaceCar second = CreateRaceCar("second", track, secondStart, speed: 12f, probeDriver);
        RaceSimulation simulation = new(track);
        simulation.AddCar(first);
        simulation.AddCar(second);

        simulation.Step(1f / 120f);

        Assert.Equal(12f, probeDriver.ObservedSpeedMetersPerSecond);
        Assert.NotEqual(12f, first.State.Speed);
    }

    [Fact]
    public void SnapshotBasedInputsDoNotDependOnCarInsertionOrder()
    {
        TrackData track = BuildTrack();
        var forwardOrder = BuildTrafficAwareSimulation(track, reverseOrder: false);
        var reverseOrder = BuildTrafficAwareSimulation(track, reverseOrder: true);

        forwardOrder.Simulation.Step(1f / 120f);
        reverseOrder.Simulation.Step(1f / 120f);

        Assert.Equal(forwardOrder.First.LastInput, reverseOrder.First.LastInput);
        Assert.Equal(forwardOrder.Second.LastInput, reverseOrder.Second.LastInput);
        Assert.Equal(2.4f, forwardOrder.First.LastInput.DesiredAccel);
        Assert.Equal(1.2f, forwardOrder.Second.LastInput.DesiredAccel);
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
    public void RaceEnvironmentTrackTemperatureFeedsTireSurfaceExchange()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        RaceCar coldTrack = CreateRaceCar(
            "cold-track",
            track,
            start,
            speed: 0f,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        RaceCar hotTrack = CreateRaceCar(
            "hot-track",
            track,
            start,
            speed: 0f,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        SetTireTemps(coldTrack.State, surfaceTempC: 80f, coreTempC: 80f);
        SetTireTemps(hotTrack.State, surfaceTempC: 80f, coreTempC: 80f);

        RaceSimulation coldSimulation = new(
            track,
            new RaceEnvironment { AirTempC = 25f, TrackTempC = 10f }
        );
        RaceSimulation hotSimulation = new(
            track,
            new RaceEnvironment { AirTempC = 25f, TrackTempC = 60f }
        );
        coldSimulation.AddCar(coldTrack);
        hotSimulation.AddCar(hotTrack);

        StepMany(coldSimulation, steps: 120);
        StepMany(hotSimulation, steps: 120);

        Assert.True(
            AverageSurfaceTemp(coldTrack.State) < AverageSurfaceTemp(hotTrack.State),
            "a warmer track should reduce tread heat loss through the contact patch"
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
            Energy = PowertrainState.Filled(0.8f)
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
            Energy = PowertrainState.Filled(0.8f)
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
            Energy = PowertrainState.Filled(0.8f)
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
            Energy = PowertrainState.Filled(0.8f)
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
    public void ConfiguredContactPassesSeparateAThreeCarPileup()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(12f);
        CarCollisionConfig collision = new()
        {
            LengthMeters = 4.8f,
            WidthMeters = 1.8f,
            SolverIterations = 6
        };
        Vector2 forward = start.Tangent;
        RaceCar first = CreateRaceCar(
            "pileup-first",
            track,
            start,
            speed: 0f,
            new FixedDriver(default),
            collision
        );
        RaceCar second = CreateRaceCar(
            "pileup-second",
            track,
            start,
            speed: 0f,
            new FixedDriver(default),
            collision
        );
        RaceCar third = CreateRaceCar(
            "pileup-third",
            track,
            start,
            speed: 0f,
            new FixedDriver(default),
            collision
        );
        second.State.Position += forward * 2f;
        third.State.Position += forward * 4f;
        RaceSimulation simulation = new(track);
        simulation.AddCar(first);
        simulation.AddCar(second);
        simulation.AddCar(third);

        simulation.Step(1f / 120f);

        Assert.False(
            CarContactResolver.AreOverlapping(first, second),
            $"first={first.State.Position}, second={second.State.Position}"
        );
        Assert.False(
            CarContactResolver.AreOverlapping(second, third),
            $"second={second.State.Position}, third={third.State.Position}"
        );
        Assert.False(
            CarContactResolver.AreOverlapping(first, third),
            $"first={first.State.Position}, third={third.State.Position}"
        );
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
    public void ContactAndBoundaryQueriesReuseValueGeometry()
    {
        TrackData track = BuildTrack();
        TrackSample start = track.Sample(20f);
        CarCollisionConfig collision = new()
        {
            LengthMeters = 4.8f,
            WidthMeters = 1.8f
        };
        RaceCar first = CreateRaceCar(
            "first-allocation-probe",
            track,
            start,
            speed: 15f,
            new FixedDriver(default),
            collision
        );
        RaceCar second = CreateRaceCar(
            "second-allocation-probe",
            track,
            start,
            speed: 15f,
            new FixedDriver(default),
            collision
        );
        second.State.Position += start.Tangent * 3f;

        CarContactResolver.TryGetContact(first, second, out _);
        TrackBoundaryResolver.IsInsideTrackWalls(
            track,
            first.State,
            collision
        );

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1_000; i++)
        {
            CarContactResolver.TryGetContact(first, second, out _);
            TrackBoundaryResolver.IsInsideTrackWalls(
                track,
                first.State,
                collision
            );
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.InRange(allocated, 0L, 256L);
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

    [Fact]
    public void RaceProgressCountsLapsAtConfiguredStartingLine()
    {
        TrackData track = BuildTrack();
        RaceProgress progress = new();
        float spawnS = track.LengthMeters - 10f;
        TrackPose spawn = track.Project(track.Sample(spawnS).Center);
        progress.Reset(
            track,
            spawn,
            TrackBoundaryResolver.Classify(spawn)
        );

        float travelled = 0f;
        while (travelled < track.LengthMeters - 1f)
        {
            travelled = MathF.Min(
                travelled + 20f,
                track.LengthMeters - 1f
            );
            TrackPose pose = track.Project(
                track.Sample(spawnS + travelled).Center
            );
            progress.Update(
                track,
                pose,
                TrackBoundaryResolver.Classify(pose),
                hitWallThisFrame: false
            );
        }

        Assert.Equal(0, progress.Lap);

        TrackPose afterFinish = track.Project(
            track.Sample(spawnS + track.LengthMeters + 11f).Center
        );
        progress.Update(
            track,
            afterFinish,
            TrackBoundaryResolver.Classify(afterFinish),
            hitWallThisFrame: false
        );

        Assert.Equal(1, progress.Lap);
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
                Energy = PowertrainState.Filled(0.8f)
            },
            collision
        );
    }

    private static (float Tow, float DirtyAir) MeasureWake(
        float centerSeparationMeters,
        float lateralOffsetMeters
    )
    {
        TrackData track = BuildTrack();
        const float followerS = 10f;
        TrackSample followerStart = track.Sample(followerS);
        RaceCar follower = CreateRaceCar(
            "wake-follower",
            track,
            followerStart,
            speed: 30f,
            new FixedDriver(default)
        );
        follower.State.Position += followerStart.Normal * lateralOffsetMeters;
        RaceCar leader = CreateRaceCar(
            "wake-leader",
            track,
            track.Sample(followerS + centerSeparationMeters),
            speed: 30f,
            new FixedDriver(default)
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(follower);
        simulation.AddCar(leader);

        simulation.Step(1f / 120f);

        return (
            follower.State.AirVelocityDeficit,
            follower.State.WakeDownforceLoss
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

    private static (
        RaceSimulation Simulation,
        RaceCar First,
        RaceCar Second
    ) BuildTrafficAwareSimulation(TrackData track, bool reverseOrder)
    {
        SnapshotTrafficDriver firstDriver = new("second");
        SnapshotTrafficDriver secondDriver = new("first");
        RaceCar first = CreateRaceCar(
            "first",
            track,
            track.Sample(12f),
            speed: 12f,
            firstDriver
        );
        RaceCar second = CreateRaceCar(
            "second",
            track,
            track.Sample(42f),
            speed: 24f,
            secondDriver
        );
        RaceSimulation simulation = new(track);
        if (reverseOrder)
        {
            simulation.AddCar(second);
            simulation.AddCar(first);
        }
        else
        {
            simulation.AddCar(first);
            simulation.AddCar(second);
        }

        return (simulation, first, second);
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

    private sealed class SnapshotTrafficDriver(string observedCarId) : IRaceDriver
    {
        public RaceDriverFrameContext LastContext { get; private set; }
        public float ObservedSpeedMetersPerSecond { get; private set; }

        public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
        {
            LastContext = context;
            if (!context.Frame.TryGetCar(observedCarId, out RaceCarSnapshot observed))
                throw new InvalidOperationException($"Missing car snapshot '{observedCarId}'.");

            ObservedSpeedMetersPerSecond = observed.SpeedMetersPerSecond;
            return new DriverInput(0f, observed.SpeedMetersPerSecond * 0.1f);
        }
    }

    private sealed class CurrentFramePlanProbeDriver(string observedCarId) :
        IRaceDriver,
        ITrafficMotionPlanSource
    {
        private readonly VehiclePathPrediction _path = new();
        private readonly TrafficMotionPlan _plan = new();

        public int PrepareCount { get; private set; }
        public bool SawOtherCurrentFramePlan { get; private set; }

        public void PrepareTrafficMotionPlan(
            in RaceDriverFrameContext context,
            float dt
        )
        {
            PrepareCount++;
            _path.Reset(2);
            Vector2 forward = context.Pose.Sample.Tangent;
            float speed = context.Car.State.Speed;
            _path.Add(new VehiclePathPredictionPoint(
                0f,
                context.Car.State.Position,
                context.Car.State.VelocityHeading,
                context.Pose.S,
                0f,
                0f,
                0f,
                0f,
                0f,
                speed
            ));
            _path.Add(new VehiclePathPredictionPoint(
                10f,
                context.Car.State.Position + forward * 10f,
                context.Car.State.VelocityHeading,
                context.Track.WrapS(context.Pose.S + 10f),
                0f,
                0f,
                0f,
                0f,
                0f,
                speed
            ));
            _plan.BuildFrom(_path);
        }

        public TrafficMotionPlan? FreezeTrafficMotionPlan()
        {
            return _plan.Count > 0 ? _plan : null;
        }

        public DriverInput GetControl(
            in RaceDriverFrameContext context,
            float dt
        )
        {
            SawOtherCurrentFramePlan =
                context.Frame.FindTrafficMotionPlan(observedCarId) is not null;
            return default;
        }
    }

    private sealed class PreviousFramePlanProbeDriver(string observedCarId) :
        IRaceDriver,
        ITrafficMotionPlanSource
    {
        private readonly VehiclePathPrediction _path = new();
        private readonly TrafficMotionPlan _plan = new();

        public int PrepareCount { get; private set; }
        public List<bool> SawCurrentPlanDuringPrepare { get; } = [];
        public List<float?> PreviousPlanSpeedsDuringPrepare { get; } = [];
        public List<float?> CurrentPlanSpeedsDuringControl { get; } = [];
        public List<bool> SawPreviousPlanDuringControl { get; } = [];

        public void PrepareTrafficMotionPlan(
            in RaceDriverFrameContext context,
            float dt
        )
        {
            PrepareCount++;
            SawCurrentPlanDuringPrepare.Add(
                context.Frame.FindTrafficMotionPlan(observedCarId) is not null
            );
            PreviousPlanSpeedsDuringPrepare.Add(StartSpeed(
                context.Frame.FindPreviousTrafficMotionPlan(observedCarId)
            ));

            float markerSpeed = 100f + PrepareCount;
            _path.Reset(2);
            Vector2 forward = context.Pose.Sample.Tangent;
            _path.Add(new VehiclePathPredictionPoint(
                0f,
                context.Car.State.Position,
                context.Car.State.VelocityHeading,
                context.Pose.S,
                0f,
                0f,
                0f,
                0f,
                0f,
                markerSpeed
            ));
            _path.Add(new VehiclePathPredictionPoint(
                10f,
                context.Car.State.Position + forward * 10f,
                context.Car.State.VelocityHeading,
                context.Track.WrapS(context.Pose.S + 10f),
                0f,
                0f,
                0f,
                0f,
                0f,
                markerSpeed
            ));
            _plan.BuildFrom(_path);
        }

        public TrafficMotionPlan? FreezeTrafficMotionPlan()
        {
            return _plan.Count > 0 ? _plan : null;
        }

        public DriverInput GetControl(
            in RaceDriverFrameContext context,
            float dt
        )
        {
            CurrentPlanSpeedsDuringControl.Add(StartSpeed(
                context.Frame.FindTrafficMotionPlan(observedCarId)
            ));
            SawPreviousPlanDuringControl.Add(
                context.Frame.FindPreviousTrafficMotionPlan(observedCarId) is not null
            );
            return default;
        }

        private static float? StartSpeed(TrafficMotionPlan? plan)
        {
            return plan is not null &&
                   plan.TrySample(0f, out TrafficMotionPlanPoint start)
                ? start.SpeedMetersPerSecond
                : null;
        }
    }

    private sealed class PlanObserverDriver(string observedCarId) : IRaceDriver
    {
        public bool SawObservedPlan { get; private set; }
        public float ObservedPlanStartSpeedMetersPerSecond { get; private set; }

        public DriverInput GetControl(
            in RaceDriverFrameContext context,
            float dt
        )
        {
            TrafficMotionPlan? plan =
                context.Frame.FindTrafficMotionPlan(observedCarId);
            SawObservedPlan = plan is not null;
            if (plan is not null &&
                plan.TrySample(0f, out TrafficMotionPlanPoint start))
            {
                ObservedPlanStartSpeedMetersPerSecond =
                    start.SpeedMetersPerSecond;
            }
            return default;
        }
    }

    private sealed class LiveSpeedProbeDriver(RaceCar observedCar) : IRaceDriver
    {
        public float ObservedSpeedMetersPerSecond { get; private set; }

        public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
        {
            ObservedSpeedMetersPerSecond = observedCar.State.Speed;
            return new DriverInput(0f, 0f);
        }
    }
}
