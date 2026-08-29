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
    private const int DefaultRosterSeed = 0x5345564F;

    [Fact]
    public void DefaultGridPairDoesNotTouchThroughFirstCorner()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        Random random = new(DefaultRosterSeed);
        DriverProfile firstProfile = CreateDefaultGridProfile("grid-01", random);
        DriverProfile secondProfile = CreateDefaultGridProfile("grid-02", random);
        RaceCar first = CreateGridCar(
            "grid-01",
            track,
            gridPosition: 1,
            firstProfile
        );
        RaceCar second = CreateGridCar(
            "grid-02",
            track,
            gridPosition: 2,
            secondProfile
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(first);
        simulation.AddCar(second);

        float minimumClearance = float.PositiveInfinity;
        float minimumClearanceTime = 0f;
        float firstMaximumSpeed = 0f;
        float secondMaximumSpeed = 0f;
        float firstPlannedTrafficBrakingTime = float.PositiveInfinity;
        float secondMinimumSpeedAfterLaunch = float.PositiveInfinity;
        bool secondHasLaunched = false;
        for (int i = 0; i < 15 * 60; i++)
        {
            simulation.Step(Dt);
            float clearance = OrientedBodyClearance(first, second);
            if (clearance < minimumClearance)
            {
                minimumClearance = clearance;
                minimumClearanceTime = simulation.RaceTimeSeconds;
            }
            firstMaximumSpeed = MathF.Max(firstMaximumSpeed, first.State.Speed);
            secondMaximumSpeed = MathF.Max(secondMaximumSpeed, second.State.Speed);
            secondHasLaunched |= second.State.Speed > 10f;
            if (secondHasLaunched)
            {
                secondMinimumSpeedAfterLaunch = MathF.Min(
                    secondMinimumSpeedAfterLaunch,
                    second.State.Speed
                );
            }
            ReferenceLineDriver secondDriver =
                (ReferenceLineDriver)second.Driver;
            if (secondDriver.LastTelemetry.TrafficConstraintKind !=
                    TrafficSpeedConstraintKind.None &&
                secondDriver.LastTelemetry.ReferenceAcceleration < -0.5f)
            {
                firstPlannedTrafficBrakingTime = MathF.Min(
                    firstPlannedTrafficBrakingTime,
                    simulation.RaceTimeSeconds
                );
            }
        }

        Assert.True(firstMaximumSpeed > 10f, "the first grid car should launch normally");
        Assert.True(secondMaximumSpeed > 10f, "the second grid car should launch normally");
        Assert.True(
            firstPlannedTrafficBrakingTime < 7.5f,
            $"traffic braking should be planned before the old late-response window; " +
            $"first planned braking was {firstPlannedTrafficBrakingTime:0.000} s"
        );
        Assert.True(
            secondMinimumSpeedAfterLaunch > 5f,
            $"the following car should yield without stopping; minimum speed after launch " +
            $"was {secondMinimumSpeedAfterLaunch:0.00} m/s"
        );
        Assert.True(
            minimumClearance >= 0.03f,
            $"grid cars should retain clearance; minimum was {minimumClearance:0.000} m " +
            $"at {minimumClearanceTime:0.000} s"
        );
    }

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
            // Holds its speed by feedback rather than by a throttle
            // setting calibrated to cancel the losses of a level road,
            // which stopped being true once this layout was given a
            // gradient. What the test is about is the car behind.
            new HoldSpeedDriver(leadSpeed)
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
    public void TrafficTimeLossBecomesAvailableToTacticsOnTheNextFrame()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ReferenceLineDriver egoDriver = new();
        RaceCar ego = CreateCar(
            "reporting-ego",
            track,
            s: 100f,
            d: 0f,
            speed: 25f,
            egoDriver
        );
        const float leadSpeed = 12f;
        RaceCar lead = CreateCar(
            "reporting-lead",
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

        simulation.Step(Dt);

        TrafficConflictReport firstReport =
            egoDriver.LastTrafficConflictReport;
        Assert.True(firstReport.Active);
        Assert.Equal(lead.Id, firstReport.OpponentId);
        Assert.True(firstReport.EvaluationDistanceMeters > 0f);
        Assert.True(float.IsFinite(firstReport.FreeArrivalTimeSeconds));
        Assert.True(float.IsFinite(firstReport.ConstrainedArrivalTimeSeconds));
        Assert.True(firstReport.TimeLossSeconds > 0f);
        Assert.Equal(
            firstReport.TimeLossSeconds,
            egoDriver.LastTelemetry.TrafficTimeLossSeconds
        );
        Assert.False(egoDriver.LastTacticalConflictReport.Active);
        Assert.Equal(0f, egoDriver.LastTacticalOffsetMeters);
        Assert.Equal(
            TacticalManeuverPhase.Observing,
            egoDriver.LastTacticalPhase
        );

        simulation.Step(Dt);

        Assert.Equal(
            firstReport,
            egoDriver.LastTacticalConflictReport
        );
        Assert.Equal(0f, egoDriver.LastTacticalOffsetMeters);
        Assert.Equal(
            TacticalManeuverPhase.Observing,
            egoDriver.LastTacticalPhase
        );
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
        Assert.False(egoDriver.LastTrafficConflictReport.Active);
        Assert.Equal(0f, egoDriver.LastTrafficConflictReport.TimeLossSeconds);
        Assert.True(ego.LastInput.DesiredAccel > 0f);
    }

    /// <summary>
    /// The same car alongside, on a corner instead of a straight.
    ///
    /// Nothing about the pair has changed: three or four metres apart across
    /// the road, the same speed, and each one holding the line it is already
    /// on. Only the road is different.
    ///
    /// Two cars a few lengths apart on a bend point in different directions
    /// merely by being at different places on it, so one car's velocity read
    /// against the other's heading carries the road's own turn in it. A rival
    /// a few degrees around a corner then reads as diving several metres a
    /// second at a car it is simply keeping station with, and the room beside
    /// it is judged to be closing when nothing is moving across the road at
    /// all. Straights hide this, because there the two headings agree.
    ///
    /// Passing happens in corners, so a car that brakes for the room beside it
    /// there can never finish a move it has begun.
    /// </summary>
    [Theory]
    [InlineData(525f, -4f)]
    [InlineData(1675f, 3f)]
    public void ParallelCarThroughACornerDoesNotTriggerBraking(
        float cornerS,
        float lateralOffsetMeters
    )
    {
        const float Speed = 30f;
        const float AlongsideGapMeters = 8f;
        TrackData track = TrackFactory.SimpleTestTrack();
        ReferenceLineDriver egoDriver = new();
        RaceCar ego = CreateCar(
            "ego",
            track,
            s: cornerS,
            d: 0f,
            speed: Speed,
            egoDriver
        );
        // The car alongside holds the corner at its own radius. Driving it
        // straight would cut across the ego and make the conflict real.
        TrackSample alongside = track.Sample(cornerS + AlongsideGapMeters);
        RaceCar adjacent = CreateCar(
            "adjacent",
            track,
            s: cornerS + AlongsideGapMeters,
            d: lateralOffsetMeters,
            speed: Speed,
            new FixedDriver(new DriverInput(alongside.RefCurvature, 0.4f))
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(ego);
        simulation.AddCar(adjacent);

        simulation.Step(Dt);

        Assert.Equal(
            TrafficSpeedConstraintKind.None,
            egoDriver.LastTelemetry.TrafficConstraintKind
        );
        Assert.False(egoDriver.LastTrafficConflictReport.Active);
        Assert.Equal(0f, egoDriver.LastTrafficConflictReport.TimeLossSeconds);
    }

    [Theory]
    [InlineData(525f, -4f, -0.12f)]
    [InlineData(1675f, 3f, 0.12f)]
    public void CarActuallyMergingThroughACornerTriggersCloseFollowingConstraint(
        float cornerS,
        float opponentOffsetMeters,
        float headingDeltaRadians
    )
    {
        const float EgoSpeed = 32f;
        const float OpponentSpeed = 30f;
        const float OpponentLeadMeters = 8f;
        const float PathLengthMeters = 20f;
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar ego = CreateCar(
            "merging-ego",
            track,
            s: cornerS,
            d: 0f,
            speed: EgoSpeed,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        RaceCar opponent = CreateCar(
            "merging-opponent",
            track,
            s: cornerS + OpponentLeadMeters,
            d: opponentOffsetMeters,
            speed: OpponentSpeed,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        // The opponent starts outside the lateral safety envelope but points
        // across the reference line toward the ego strongly enough to enter it
        // within the one-second close-following lookahead.
        TrackSample opponentSample = track.Sample(
            cornerS + OpponentLeadMeters
        );
        opponent.State.Heading = opponentSample.RefHeading +
                                 headingDeltaRadians;

        float[] pathDistances = [0f, PathLengthMeters];
        VehiclePathPrediction path = new();
        path.Reset(pathDistances.Length);
        foreach (float distance in pathDistances)
        {
            TrackSample sample = track.Sample(cornerS + distance);
            path.Add(new VehiclePathPredictionPoint(
                distance,
                sample.RefPosition,
                sample.RefHeading,
                sample.S,
                0f,
                sample.RefCurvature,
                sample.RefCurvature,
                0f,
                sample.RefCurvature,
                EgoSpeed
            ));
        }

        RaceCarSnapshot[] cars =
        [
            RaceCarSnapshot.Capture(
                ego,
                track.Project(ego.State.Position)
            ),
            RaceCarSnapshot.Capture(
                opponent,
                track.Project(opponent.State.Position)
            )
        ];
        RaceFrameSnapshot frame = new(
            raceTimeSeconds: 0f,
            cars,
            new TrafficMotionPlan?[cars.Length]
        );
        float[] segmentLengths = [PathLengthMeters, 0f];
        float[] speeds = [EgoSpeed, EgoSpeed];
        float[] speedLimits = [80f, 80f];
        float[] arrivalTimes = new float[path.Count];
        // Keep the full trajectory search shorter than the first path segment
        // so this test specifically exercises the close-following fallback.
        VehicleSpeedPlanningConfig config = new()
        {
            TrafficPredictionHorizonSeconds = 0.01f,
            TrafficLateralMergePredictionSeconds = 1f
        };
        TrafficConstraintMemory memory = default;
        TrafficSpeedConstraint constraint = default;

        bool changed = TrafficConflictEvaluator.ApplyConstraints(
            config,
            track,
            path,
            in frame,
            egoSnapshotIndex: 0,
            segmentLengths,
            speeds,
            speedLimits,
            arrivalTimes,
            ref memory,
            ref constraint,
            out bool requiresReevaluation
        );

        Assert.True(changed);
        Assert.False(requiresReevaluation);
        Assert.Equal(TrafficSpeedConstraintKind.Follow, constraint.Kind);
        Assert.Equal(opponent.Id, constraint.OpponentId);
        Assert.Equal(0f, constraint.PredictedConflictTimeSeconds);
        Assert.True(constraint.CurrentClearanceMeters > 0f);
    }

    [Fact]
    public void FollowingConstraintReleasesAfterPredictedPathClearsLaterally()
    {
        const float EgoS = 100f;
        const float EgoSpeed = 30f;
        const float OpponentSpeed = 20f;
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar ego = CreateCar(
            "clearing-ego",
            track,
            EgoS,
            d: 0f,
            speed: EgoSpeed,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        RaceCar opponent = CreateCar(
            "clearing-opponent",
            track,
            EgoS + 15f,
            d: 0f,
            speed: OpponentSpeed,
            new FixedDriver(new DriverInput(0f, 0f))
        );

        float[] pathDistances = [0f, 10f, 20f, 40f, 60f, 80f];
        float[] pathOffsets = [0f, 0f, 0f, 4f, 4f, 4f];
        VehiclePathPrediction path = new();
        path.Reset(pathDistances.Length);
        for (int i = 0; i < pathDistances.Length; i++)
        {
            TrackSample sample = track.Sample(EgoS + pathDistances[i]);
            path.Add(new VehiclePathPredictionPoint(
                pathDistances[i],
                sample.RefPosition + sample.Normal * pathOffsets[i],
                sample.RefHeading,
                sample.S,
                0f,
                sample.RefCurvature,
                sample.RefCurvature,
                0f,
                sample.RefCurvature,
                EgoSpeed
            ));
        }

        RaceCarSnapshot[] cars =
        [
            RaceCarSnapshot.Capture(ego, track.Project(ego.State.Position)),
            RaceCarSnapshot.Capture(
                opponent,
                track.Project(opponent.State.Position)
            )
        ];
        RaceFrameSnapshot frame = new(
            raceTimeSeconds: 0f,
            cars,
            new TrafficMotionPlan?[cars.Length]
        );
        float[] segmentLengths = [10f, 10f, 20f, 20f, 20f, 0f];
        float[] speeds =
            [EgoSpeed, EgoSpeed, EgoSpeed, EgoSpeed, EgoSpeed, EgoSpeed];
        float[] speedLimits = [80f, 80f, 80f, 80f, 80f, 80f];
        float[] arrivalTimes = new float[path.Count];
        VehicleSpeedPlanningConfig config = new();
        TrafficConstraintMemory memory = default;
        TrafficSpeedConstraint constraint = default;

        bool changed = TrafficConflictEvaluator.ApplyConstraints(
            config,
            track,
            path,
            in frame,
            egoSnapshotIndex: 0,
            segmentLengths,
            speeds,
            speedLimits,
            arrivalTimes,
            ref memory,
            ref constraint,
            out _
        );

        Assert.True(changed);
        Assert.Equal(TrafficSpeedConstraintKind.Follow, constraint.Kind);
        Assert.True(speedLimits[1] < EgoSpeed);
        Assert.Equal(80f, speedLimits[3]);
        Assert.Equal(80f, speedLimits[4]);
        Assert.Equal(80f, speedLimits[5]);
    }

    [Fact]
    public void CloseFollowingClearanceIncludesRotatedBodyCorners()
    {
        const float EgoS = 100f;
        const float CenterLeadMeters = 5.1f;
        const float LateralOffsetMeters = 1.4f;
        const float BodyAngleRadians = 0.08f;
        const float Speed = 30f;
        const float PathLengthMeters = 20f;
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar ego = CreateCar(
            "angled-ego",
            track,
            EgoS,
            d: 0f,
            speed: Speed,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        RaceCar opponent = CreateCar(
            "angled-opponent",
            track,
            EgoS + CenterLeadMeters,
            d: LateralOffsetMeters,
            speed: Speed,
            new FixedDriver(new DriverInput(0f, 0f))
        );
        TrackSample egoSample = track.Sample(EgoS);
        TrackSample opponentSample = track.Sample(EgoS + CenterLeadMeters);
        ego.State.Heading = egoSample.RefHeading + BodyAngleRadians;
        ego.State.SideslipAngleRadians = -BodyAngleRadians;
        opponent.State.Heading = opponentSample.RefHeading - BodyAngleRadians;
        opponent.State.SideslipAngleRadians = BodyAngleRadians;

        VehiclePathPrediction path = new();
        path.Reset(2);
        foreach (float distance in new[] { 0f, PathLengthMeters })
        {
            TrackSample sample = track.Sample(EgoS + distance);
            path.Add(new VehiclePathPredictionPoint(
                distance,
                sample.RefPosition,
                sample.RefHeading,
                sample.S,
                0f,
                sample.RefCurvature,
                sample.RefCurvature,
                0f,
                sample.RefCurvature,
                Speed
            ));
        }

        RaceCarSnapshot[] cars =
        [
            RaceCarSnapshot.Capture(ego, track.Project(ego.State.Position)),
            RaceCarSnapshot.Capture(
                opponent,
                track.Project(opponent.State.Position)
            )
        ];
        RaceFrameSnapshot frame = new(
            raceTimeSeconds: 0f,
            cars,
            new TrafficMotionPlan?[cars.Length]
        );
        float[] segmentLengths = [PathLengthMeters, 0f];
        float[] speeds = [Speed, Speed];
        float[] speedLimits = [80f, 80f];
        float[] arrivalTimes = new float[path.Count];
        VehicleSpeedPlanningConfig config = new()
        {
            TrafficPredictionHorizonSeconds = 0.01f
        };
        TrafficConstraintMemory memory = default;
        TrafficSpeedConstraint constraint = default;

        bool changed = TrafficConflictEvaluator.ApplyConstraints(
            config,
            track,
            path,
            in frame,
            egoSnapshotIndex: 0,
            segmentLengths,
            speeds,
            speedLimits,
            arrivalTimes,
            ref memory,
            ref constraint,
            out _
        );

        Assert.True(changed);
        Assert.Equal(TrafficSpeedConstraintKind.Follow, constraint.Kind);
        Assert.InRange(constraint.CurrentClearanceMeters, 0.04f, 0.12f);
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

    private static RaceCar CreateGridCar(
        string id,
        TrackData track,
        int gridPosition,
        DriverProfile profile
    )
    {
        Grid grid = track.Grids[gridPosition];
        TrackSample sample = track.Sample(grid.S);
        return new RaceCar(
            id,
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 86f,
                StartingCoreTempC = 84f
            },
            new ReferenceLineDriver(profile),
            new CarState
            {
                Position = grid.Position,
                Heading = sample.RefHeading,
                Speed = 0f,
                BatterySoc = 0.82f
            }
        );
    }

    private static DriverProfile CreateDefaultGridProfile(
        string id,
        Random random
    )
    {
        return new DriverProfile(
            id,
            new DriverAbilities
            {
                Pace = NextRating(random, 84f, 96f),
                Consistency = NextRating(random, 82f, 96f),
                CarControl = NextRating(random, 84f, 97f),
                TireManagement = NextRating(random, 78f, 94f),
                Adaptability = NextRating(random, 82f, 96f),
                Reactions = NextRating(random, 82f, 97f),
                Awareness = NextRating(random, 82f, 97f),
                Overtaking = NextRating(random, 80f, 96f),
                Defending = NextRating(random, 80f, 96f)
            },
            (ulong)random.NextInt64(1, long.MaxValue)
        );
    }

    private static float NextRating(
        Random random,
        float minimum,
        float maximum
    )
    {
        return minimum + (float)random.NextDouble() * (maximum - minimum);
    }

    private static float OrientedBodyClearance(RaceCar first, RaceCar second)
    {
        CarBodyGeometry firstBody = CarBodyGeometry.FromState(
            first.State,
            first.Collision
        );
        CarBodyGeometry secondBody = CarBodyGeometry.FromState(
            second.State,
            second.Collision
        );
        float clearance = AxisClearance(
            in firstBody,
            in secondBody,
            firstBody.Forward
        );
        clearance = MathF.Max(
            clearance,
            AxisClearance(in firstBody, in secondBody, firstBody.Left)
        );
        clearance = MathF.Max(
            clearance,
            AxisClearance(in firstBody, in secondBody, secondBody.Forward)
        );
        return MathF.Max(
            clearance,
            AxisClearance(in firstBody, in secondBody, secondBody.Left)
        );
    }

    private static float AxisClearance(
        in CarBodyGeometry first,
        in CarBodyGeometry second,
        Vector2 axis
    )
    {
        first.ProjectOntoAxis(axis, out float firstMinimum, out float firstMaximum);
        second.ProjectOntoAxis(axis, out float secondMinimum, out float secondMaximum);
        return MathF.Max(
            secondMinimum - firstMaximum,
            firstMinimum - secondMaximum
        );
    }

    private sealed class HoldSpeedDriver(float targetSpeed) : IRaceDriver
    {
        public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
        {
            float error = targetSpeed - context.Car.State.Speed;
            return new DriverInput(0f, Math.Clamp(5f * error, -8f, 8f));
        }
    }

    private sealed class FixedDriver(DriverInput input) : IRaceDriver
    {
        public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
        {
            return input;
        }
    }
}
