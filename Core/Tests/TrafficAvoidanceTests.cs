using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class TrafficAvoidanceTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void ReferenceDriverStopsBeforeStationaryCar()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ReferenceLineDriver egoDriver = new();
        RaceCar ego = CreateCar(
            "ego",
            track,
            s: 100f,
            d: 0f,
            speed: 25f,
            egoDriver
        );
        RaceCar obstacle = CreateCar(
            "obstacle",
            track,
            s: 140f,
            d: 0f,
            speed: 0f,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(ego);
        simulation.AddCar(obstacle);

        float minimumCenterDistance = float.PositiveInfinity;
        float maximumObstacleSpeed = 0f;
        bool avoidanceActivated = false;
        for (int i = 0; i < 6 * 60; i++)
        {
            simulation.Step(Dt);
            minimumCenterDistance = MathF.Min(
                minimumCenterDistance,
                Vector2.Distance(ego.State.Position, obstacle.State.Position)
            );
            maximumObstacleSpeed = MathF.Max(
                maximumObstacleSpeed,
                obstacle.State.Speed
            );
            avoidanceActivated |= egoDriver.LastTelemetry.TrafficConstraintKind ==
                                  TrafficSpeedConstraintKind.Stop;
        }

        Assert.True(avoidanceActivated, "the stationary obstacle should create a stop constraint");
        Assert.True(
            minimumCenterDistance >
            (ego.Collision.LengthMeters + obstacle.Collision.LengthMeters) * 0.5f,
            $"cars should not touch; minimum center distance was {minimumCenterDistance:0.00} m"
        );
        Assert.InRange(maximumObstacleSpeed, 0f, 0.05f);
        Assert.InRange(ego.State.Speed, 0f, 0.5f);
    }

    [Fact]
    public void ReferenceDriverMatchesSlowerCarWithoutContact()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ReferenceLineDriver egoDriver = new();
        RaceCar ego = CreateCar(
            "ego",
            track,
            s: 100f,
            d: 0f,
            speed: 25f,
            egoDriver
        );
        const float leadSpeed = 12f;
        RaceCar lead = CreateCar(
            "lead",
            track,
            s: 135f,
            d: 0f,
            speed: leadSpeed,
            new FixedDriver(
                new DriverInput(
                    0f,
                    0.18f + 0.00046f * leadSpeed * leadSpeed
                )
            )
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(ego);
        simulation.AddCar(lead);

        float minimumCenterDistance = float.PositiveInfinity;
        bool avoidanceActivated = false;
        for (int i = 0; i < 8 * 60; i++)
        {
            simulation.Step(Dt);
            minimumCenterDistance = MathF.Min(
                minimumCenterDistance,
                Vector2.Distance(ego.State.Position, lead.State.Position)
            );
            avoidanceActivated |= egoDriver.LastTelemetry.TrafficConstraintKind ==
                                  TrafficSpeedConstraintKind.Follow;
        }

        Assert.True(avoidanceActivated, "the slower car should create a follow constraint");
        Assert.True(
            minimumCenterDistance >
            (ego.Collision.LengthMeters + lead.Collision.LengthMeters) * 0.5f,
            $"cars should not touch; minimum center distance was {minimumCenterDistance:0.00} m"
        );
        Assert.InRange(lead.State.Speed, leadSpeed - 0.6f, leadSpeed + 0.6f);
        Assert.InRange(ego.State.Speed, lead.State.Speed - 1f, lead.State.Speed + 1.5f);
    }

    [Fact]
    public void ParallelCarOutsideEgoPathDoesNotTriggerBraking()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ReferenceLineDriver egoDriver = new();
        RaceCar ego = CreateCar(
            "ego",
            track,
            s: 100f,
            d: 0f,
            speed: 20f,
            egoDriver
        );
        RaceCar adjacent = CreateCar(
            "adjacent",
            track,
            s: 120f,
            d: 5f,
            speed: 20f,
            new FixedDriver(new DriverInput(0f, 0.4f))
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(ego);
        simulation.AddCar(adjacent);

        simulation.Step(Dt);

        Assert.Equal(
            TrafficSpeedConstraintKind.None,
            egoDriver.LastTelemetry.TrafficConstraintKind
        );
        Assert.True(ego.LastInput.DesiredAccel > 0f);
    }

    [Fact]
    public void CrossingCarCreatesStopConstraintWithoutReadingItsDriverPlan()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ReferenceLineDriver egoDriver = new();
        RaceCar ego = CreateCar(
            "yielding-ego",
            track,
            s: 100f,
            d: 0f,
            speed: 20f,
            egoDriver
        );
        RaceCar crossing = CreateCar(
            "crossing-car",
            track,
            s: 120f,
            d: 4f,
            speed: 6f,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        TrackSample crossingSample = track.Sample(120f);
        Vector2 towardReferenceLine = -crossingSample.Normal;
        crossing.State.Heading = MathF.Atan2(
            towardReferenceLine.Y,
            towardReferenceLine.X
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(ego);
        simulation.AddCar(crossing);

        simulation.Step(Dt);

        Assert.Equal(
            TrafficSpeedConstraintKind.Stop,
            egoDriver.LastTelemetry.TrafficConstraintKind
        );
        Assert.Equal(
            crossing.Id,
            egoDriver.LastTelemetry.TrafficOpponentId
        );
    }

    [Fact]
    public void TrafficPlanningIsIndependentOfCarInsertionOrder()
    {
        (RaceSimulation firstSimulation, RaceCar firstEgo) =
            BuildStationaryObstacleSimulation(reverseOrder: false);
        (RaceSimulation secondSimulation, RaceCar secondEgo) =
            BuildStationaryObstacleSimulation(reverseOrder: true);

        for (int i = 0; i < 30; i++)
        {
            firstSimulation.Step(Dt);
            secondSimulation.Step(Dt);
        }

        ReferenceLineDriver firstDriver = (ReferenceLineDriver)firstEgo.Driver;
        ReferenceLineDriver secondDriver = (ReferenceLineDriver)secondEgo.Driver;
        Assert.Equal(firstEgo.LastInput.DesiredAccel, secondEgo.LastInput.DesiredAccel, 5);
        Assert.Equal(firstEgo.State.Speed, secondEgo.State.Speed, 5);
        Assert.Equal(
            firstDriver.LastTelemetry.TrafficConstraintKind,
            secondDriver.LastTelemetry.TrafficConstraintKind
        );
        Assert.Equal(
            firstDriver.LastTelemetry.TrafficConstraintDistanceMeters,
            secondDriver.LastTelemetry.TrafficConstraintDistanceMeters,
            4
        );
    }

    [Fact]
    public void ReferenceDriverRespondsToLeadCarEmergencyBraking()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ReferenceLineDriver egoDriver = new();
        RaceCar ego = CreateCar(
            "ego",
            track,
            s: 100f,
            d: 0f,
            speed: 28f,
            egoDriver
        );
        RaceCar lead = CreateCar(
            "lead",
            track,
            s: 135f,
            d: 0f,
            speed: 22f,
            new FixedDriver(new DriverInput(0f, -12f))
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(ego);
        simulation.AddCar(lead);

        float minimumCenterDistance = float.PositiveInfinity;
        bool stopConstraintSeen = false;
        for (int i = 0; i < 5 * 60; i++)
        {
            simulation.Step(Dt);
            minimumCenterDistance = MathF.Min(
                minimumCenterDistance,
                Vector2.Distance(ego.State.Position, lead.State.Position)
            );
            stopConstraintSeen |= egoDriver.LastTelemetry.TrafficConstraintKind ==
                                  TrafficSpeedConstraintKind.Stop;
        }

        Assert.True(stopConstraintSeen);
        Assert.True(
            minimumCenterDistance >
            (ego.Collision.LengthMeters + lead.Collision.LengthMeters) * 0.5f,
            $"cars should not touch while the lead car brakes; minimum center distance was {minimumCenterDistance:0.00} m"
        );
        Assert.InRange(ego.State.Speed, 0f, 0.5f);
    }

    [Fact]
    public void IdenticalReferenceDriversLaunchWithoutTrafficAccordion()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceSimulation simulation = new(track);
        RaceCar[] cars = new RaceCar[6];
        for (int i = 0; i < cars.Length; i++)
        {
            cars[i] = CreateCar(
                $"launch-{i + 1}",
                track,
                s: 160f - i * 8f,
                d: 0f,
                speed: 0f,
                new ReferenceLineDriver()
            );
            simulation.AddCar(cars[i]);
        }

        for (int i = 0; i < 30; i++)
            simulation.Step(Dt);

        float leaderSpeed = cars[0].State.Speed;
        float tailSpeed = cars[^1].State.Speed;
        Assert.True(
            tailSpeed >= leaderSpeed - 1f,
            $"identical cars should launch together; leader={leaderSpeed * 3.6f:0.0} km/h, " +
            $"tail={tailSpeed * 3.6f:0.0} km/h"
        );
    }

    [Fact]
    public void PredictedRecoveryPathBrakesForObstacleOnReferenceLine()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ReferenceLineDriver egoDriver = new();
        RaceCar ego = CreateCar(
            "ego",
            track,
            s: 100f,
            d: 4f,
            speed: 18f,
            egoDriver
        );
        RaceCar obstacle = CreateCar(
            "obstacle",
            track,
            s: 132f,
            d: 0f,
            speed: 0f,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(ego);
        simulation.AddCar(obstacle);

        float minimumCenterDistance = float.PositiveInfinity;
        bool stopConstraintSeen = false;
        for (int i = 0; i < 5 * 60; i++)
        {
            simulation.Step(Dt);
            minimumCenterDistance = MathF.Min(
                minimumCenterDistance,
                Vector2.Distance(ego.State.Position, obstacle.State.Position)
            );
            stopConstraintSeen |= egoDriver.LastTelemetry.TrafficConstraintKind ==
                                  TrafficSpeedConstraintKind.Stop;
        }

        Assert.True(stopConstraintSeen, "the predicted return path should intersect the obstacle");
        Assert.True(
            minimumCenterDistance >
            (ego.Collision.LengthMeters + obstacle.Collision.LengthMeters) * 0.5f,
            $"cars should not touch during recovery; minimum center distance was {minimumCenterDistance:0.00} m"
        );
    }

    private static (RaceSimulation Simulation, RaceCar Ego)
        BuildStationaryObstacleSimulation(bool reverseOrder)
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar ego = CreateCar(
            "ego",
            track,
            s: 100f,
            d: 0f,
            speed: 25f,
            new ReferenceLineDriver()
        );
        RaceCar obstacle = CreateCar(
            "obstacle",
            track,
            s: 140f,
            d: 0f,
            speed: 0f,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        RaceCar fartherObstacle = CreateCar(
            "farther-obstacle",
            track,
            s: 180f,
            d: 0f,
            speed: 0f,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        RaceSimulation simulation = new(track);
        if (reverseOrder)
        {
            simulation.AddCar(fartherObstacle);
            simulation.AddCar(obstacle);
            simulation.AddCar(ego);
        }
        else
        {
            simulation.AddCar(ego);
            simulation.AddCar(obstacle);
            simulation.AddCar(fartherObstacle);
        }
        return (simulation, ego);
    }

    private static RaceCar CreateCar(
        string id,
        TrackData track,
        float s,
        float d,
        float speed,
        IRaceDriver driver
    )
    {
        TrackSample sample = track.Sample(s);
        return new RaceCar(
            id,
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 90f,
                StartingCoreTempC = 90f
            },
            driver,
            new CarState
            {
                Position = sample.RefPosition + sample.Normal * d,
                Heading = sample.RefHeading,
                Speed = speed,
                BatterySoc = 0.8f
            }
        );
    }

    private sealed class FixedDriver(DriverInput input) : IRaceDriver
    {
        public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
        {
            return input;
        }
    }
}
