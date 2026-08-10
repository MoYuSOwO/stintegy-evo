using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Track.RefLines;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class ContinuousFollowingTests
{
    private const float Dt = 1f / 60f;
    private static readonly Lazy<TrackData> DraftingTrack = new(
        BuildDraftingTrack
    );
    private static readonly Lazy<FollowingDuelResult> SixSecondDuel = new(
        () => RunDuel(6f)
    );
    private static readonly Lazy<FollowingDuelResult> ThreeSecondDuel = new(
        () => RunDuel(3f)
    );

    [Fact]
    public void PredictionHorizonDoesNotSetSteadyFollowingGap()
    {
        FollowingDuelResult sixSeconds = SixSecondDuel.Value;
        FollowingDuelResult threeSeconds = ThreeSecondDuel.Value;

        Assert.InRange(sixSeconds.FinalClearanceMeters, 6f, 8f);
        Assert.InRange(threeSeconds.FinalClearanceMeters, 6f, 8f);
        Assert.InRange(
            MathF.Abs(
                sixSeconds.FinalClearanceMeters -
                threeSeconds.FinalClearanceMeters
            ),
            0f,
            0.5f
        );
    }

    [Fact]
    public void MovingLeaderDoesNotCreateAStopConstraintAtTheHorizon()
    {
        FollowingDuelResult result = SixSecondDuel.Value;

        Assert.Equal(0, result.StopFrames);
    }

    [Fact]
    public void FollowerInsideTargetGapCreatesSpaceBeforeLeaderBrakes()
    {
        TrackData track = DraftingTrack.Value;
        ReferenceLineDriver leaderDriver = CreateDriver(
            "close-leader-driver",
            pace: 100f,
            predictionHorizonSeconds: 6f
        );
        ReferenceLineDriver followerDriver = CreateDriver(
            "close-follower-driver",
            pace: 100f,
            predictionHorizonSeconds: 6f
        );
        const float leaderS = 500f;
        const float initialClearance = 2.4f;
        RaceCar leader = CreateCar(
            "close-leader",
            track,
            leaderS,
            leaderDriver
        );
        RaceCar follower = CreateCar(
            "close-follower",
            track,
            leaderS - leader.Collision.LengthMeters - initialClearance,
            followerDriver
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(leader);
        simulation.AddCar(follower);

        for (int frame = 0; frame < 60; frame++)
            simulation.Step(Dt);

        float clearance = CurrentClearance(track, leader, follower);
        Assert.True(
            clearance >= initialClearance + 0.5f,
            $"Follower preserved an unsafe {clearance:0.00} m clearance."
        );
    }

    [Fact]
    public void FollowerKeepsBodyClearanceWhenLeaderBrakesWithoutAPlan()
    {
        TrackData track = DraftingTrack.Value;
        const float leaderS = 500f;
        const float initialClearance = 3.64f;
        RaceCar leader = CreateCar(
            "braking-leader",
            track,
            leaderS,
            new ConstantBrakeDriver(14f),
            speed: 31.56f
        );
        RaceCar follower = CreateCar(
            "braking-follower",
            track,
            leaderS - leader.Collision.LengthMeters - initialClearance,
            CreateDriver(
                "braking-follower-driver",
                pace: 100f,
                predictionHorizonSeconds: 6f
            ),
            speed: 37.74f
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(leader);
        simulation.AddCar(follower);

        float minimumClearance = initialClearance;
        for (int frame = 0; frame < 60; frame++)
        {
            simulation.Step(Dt);
            minimumClearance = MathF.Min(
                minimumClearance,
                CurrentClearance(track, leader, follower)
            );
        }

        Assert.True(
            minimumClearance >= 0.25f,
            $"Follower reached only {minimumClearance:0.00} m clearance."
        );
    }

    private static FollowingDuelResult RunDuel(float predictionHorizonSeconds)
    {
        TrackData track = DraftingTrack.Value;
        ReferenceLineDriver leaderDriver = CreateDriver(
            "leader-driver",
            pace: 55f,
            predictionHorizonSeconds
        );
        ReferenceLineDriver chaserDriver = CreateDriver(
            "chaser-driver",
            pace: 100f,
            predictionHorizonSeconds
        );
        const float leaderS = 500f;
        RaceCar leader = CreateCar(
            "leader",
            track,
            leaderS,
            leaderDriver
        );
        leader.Strategy = new CarStrategy(
            TireUsageMode.Protect,
            BatteryOutputMode.Save
        );
        RaceCar chaser = CreateCar(
            "chaser",
            track,
            leaderS - 40f,
            chaserDriver
        );
        chaser.Strategy = new CarStrategy(
            TireUsageMode.Attack,
            BatteryOutputMode.Attack
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(leader);
        simulation.AddCar(chaser);

        int stopFrames = 0;
        for (int frame = 0; frame < 35 * 60; frame++)
        {
            simulation.Step(Dt);
            if (chaserDriver.LastTelemetry.TrafficConstraintKind ==
                TrafficSpeedConstraintKind.Stop)
            {
                stopFrames++;
            }
        }

        float clearance = CurrentClearance(track, leader, chaser);
        return new FollowingDuelResult(clearance, stopFrames);
    }

    private static float CurrentClearance(
        TrackData track,
        RaceCar leader,
        RaceCar follower
    )
    {
        TrackPose leaderPose = track.Project(leader.State.Position);
        TrackPose followerPose = track.Project(follower.State.Position);
        float centerDistance = track.WrapS(leaderPose.S - followerPose.S);
        if (centerDistance > track.LengthMeters * 0.5f)
            centerDistance -= track.LengthMeters;
        return centerDistance -
               (leader.Collision.LengthMeters +
                follower.Collision.LengthMeters) * 0.5f;
    }

    private static ReferenceLineDriver CreateDriver(
        string id,
        float pace,
        float predictionHorizonSeconds
    )
    {
        return new ReferenceLineDriver(
            new VehicleSpeedPlanningConfig
            {
                TrafficPredictionHorizonSeconds = predictionHorizonSeconds
            },
            new DriverProfile(
                id,
                new DriverAbilities { Pace = pace },
                randomSeed: (ulong)id.GetHashCode()
            )
        );
    }

    private static RaceCar CreateCar(
        string id,
        TrackData track,
        float s,
        IRaceDriver driver,
        float speed = 60f
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
                Position = sample.RefPosition,
                Heading = sample.RefHeading,
                Speed = speed,
                BatterySoc = 0.9f
            }
        );
    }

    private static TrackData BuildDraftingTrack()
    {
        return new TrackBuilder(
                Vector2.Zero,
                startWidth: 20f,
                refLineSolver: CenterLineRefLineSolver.Instance
            )
            .AddStraight(3000f)
            .AddTurn(180f, 400f)
            .AddStraight(3000f)
            .AddTurn(180f, 400f)
            .CloseLoop()
            .Build(new TrackGridConfig());
    }

    private readonly record struct FollowingDuelResult(
        float FinalClearanceMeters,
        int StopFrames
    );

    private sealed class ConstantBrakeDriver(float deceleration) : IRaceDriver
    {
        public DriverInput GetControl(
            in RaceDriverFrameContext context,
            float dt
        ) => new(0f, -deceleration);
    }
}
