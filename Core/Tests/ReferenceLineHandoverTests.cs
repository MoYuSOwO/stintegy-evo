using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class ReferenceLineHandoverTests
{
    private static readonly Lazy<TrackData> SimpleTrack =
        new(() => TrackFactory.SimpleTestTrack());
    private static readonly Lazy<TrackData> SilverstoneTrack =
        new(TrackFactory.SilverstoneStyleTestTrack);
    private static readonly Lazy<TrackData> ShanghaiTrack =
        new(TrackFactory.ShanghaiStyleTestTrack);

    [Fact]
    public void TacticalIntentCarriesItsLatestCompletionDistance()
    {
        TacticalIntent intent = new(
            TargetOffsetMeters: 4f,
            LatestCompletionDistanceMeters: 180f
        );

        Assert.Equal(4f, intent.TargetOffsetMeters);
        Assert.Equal(180f, intent.LatestCompletionDistanceMeters);
    }

    [Fact]
    public void HandoverUsesTheAvailableTacticalDeadlineWhenItCannotStayFlat()
    {
        TrackData track = SilverstoneTrack.Value;
        const float startS = 4656f;
        const float speed = 50f;
        const float targetOffset = 4f;
        const float deadline = 95f;
        TrackSample start = track.Sample(startS);
        RaceCar car = CreateCar(track, start, speed);
        VehicleSpeedPlanner planner = new();
        VehicleSpeedLookahead baseline = planner.PlanReferenceLookahead(
            new VehicleSpeedLookahead(),
            car,
            track,
            startS,
            horizonMeters: 300f,
            stepMeters: 2f,
            DriverPlanningModifiers.Neutral
        );
        ReferenceLineHandoverConstraints constraints =
            planner.CreateHandoverConstraints(
                car,
                baseline,
                latestCompletionDistanceMeters: deadline
            ) with
            {
                // Force every curvature candidate to fail the speed envelope;
                // the selector must then spend all, but no more than, the
                // tactical window.
                StandingLateralAccelerationLimit = 0.001f
            };
        TrackConstrainedLateralOffset profile = new();
        profile.UpdateCommittedTarget(
            track,
            startS,
            speed,
            targetOffset,
            car.Collision.HalfWidthMeters,
            in constraints
        );

        Assert.InRange(
            profile.CommittedHandoverLengthMeters,
            deadline - 0.001f,
            deadline + 0.001f
        );
    }

    [Fact]
    public void BoundaryClampDoesNotMakeTheHandoverNeedlesslyLong()
    {
        TrackData track = BuildNarrowTrack();
        const float startS = 20f;
        const float requestedOffset = 6f;
        const float halfWidth = 0.95f;
        TrackConstrainedLateralOffset committed = new();
        committed.Prepare(track);
        committed.UpdateCommittedTarget(
            track,
            startS,
            currentSpeedMetersPerSecond: 20f,
            requestedOffset,
            halfWidth
        );
        TrackSample afterSixtyMeters = track.Sample(startS + 60f);
        float actual = committed.Resolve(
            track,
            in afterSixtyMeters,
            requestedOffset,
            executionOffsetMeters: 0f,
            halfWidth
        );
        TrackConstrainedLateralOffset endpoint = new();
        endpoint.Prepare(track);
        float available = endpoint.Resolve(
            track,
            in afterSixtyMeters,
            requestedOffset,
            executionOffsetMeters: 0f,
            halfWidth
        );

        Assert.True(
            available > 0.25f,
            $"expected useful narrow-track room, got {available:F3} m; " +
            $"handover reached {actual:F3} m"
        );
        Assert.InRange(actual, available * 0.9f, available + 1e-4f);
    }

    [Fact]
    public void RepeatedHandoverSamplingDoesNotAllocate()
    {
        TrackData track = SilverstoneTrack.Value;
        const float startS = 4656f;
        const float targetOffset = 4f;
        const float halfWidth = 0.95f;
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);
        profile.UpdateCommittedTarget(
            track,
            startS,
            currentSpeedMetersPerSecond: 50f,
            targetOffset,
            halfWidth
        );
        for (int i = 0; i < 32; i++)
        {
            profile.SampleGeometry(
                track,
                startS + i,
                targetOffset,
                executionOffsetMeters: 0f,
                halfWidth
            );
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1000; i++)
        {
            profile.SampleGeometry(
                track,
                startS + i % 120,
                targetOffset,
                executionOffsetMeters: 0f,
                halfWidth
            );
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.InRange(allocated, 0L, 1024L);
    }

    [Fact]
    public void RepeatedFrameUpdatesDoNotMoveTheCommittedHandover()
    {
        TrackData track = SilverstoneTrack.Value;
        const float startS = 4656f;
        const float targetOffset = 4f;
        const float halfWidth = 0.95f;
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);
        profile.UpdateCommittedTarget(
            track,
            startS,
            currentSpeedMetersPerSecond: 50f,
            targetOffset,
            halfWidth
        );
        TrackSample fixedSample = track.Sample(startS + 50f);
        float before = profile.Resolve(
            track,
            in fixedSample,
            targetOffset,
            executionOffsetMeters: 0f,
            halfWidth
        );

        profile.UpdateCommittedTarget(
            track,
            startS + 10f,
            currentSpeedMetersPerSecond: 35f,
            targetOffset,
            halfWidth
        );
        float after = profile.Resolve(
            track,
            in fixedSample,
            targetOffset,
            executionOffsetMeters: 0f,
            halfWidth
        );

        Assert.Equal(before, after);
    }

    [Fact]
    public void ReturningToTheRacingLineUsesACommittedHandover()
    {
        TrackData track = SilverstoneTrack.Value;
        const float startS = 4656f;
        const float targetOffset = 4f;
        const float speed = 50f;
        const float halfWidth = 0.95f;
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);
        profile.UpdateCommittedTarget(
            track,
            startS,
            speed,
            targetOffset,
            halfWidth
        );
        float firstEndS = startS + 300f;
        profile.UpdateCommittedTarget(
            track,
            firstEndS,
            speed,
            targetOffset,
            halfWidth
        );
        TrackSample returnStart = track.Sample(firstEndS);
        float heldOffset = profile.Resolve(
            track,
            in returnStart,
            targetOffset,
            executionOffsetMeters: 0f,
            halfWidth
        );

        profile.UpdateCommittedTarget(
            track,
            firstEndS,
            speed,
            requestedOffsetMeters: 0f,
            halfWidth
        );
        float returnStartOffset = profile.Resolve(
            track,
            in returnStart,
            tacticalOffsetMeters: 0f,
            executionOffsetMeters: 0f,
            halfWidth
        );
        float returnEndS = firstEndS + 300f;
        TrackSample returnEnd = track.Sample(returnEndS);
        float returnEndOffset = profile.Resolve(
            track,
            in returnEnd,
            tacticalOffsetMeters: 0f,
            executionOffsetMeters: 0f,
            halfWidth
        );

        Assert.True(profile.HasCommittedProfile);
        Assert.Equal(heldOffset, returnStartOffset, 3);
        Assert.InRange(MathF.Abs(returnEndOffset), 0f, 0.01f);

        profile.UpdateCommittedTarget(
            track,
            returnEndS + 1f,
            speed,
            requestedOffsetMeters: 0f,
            halfWidth
        );
        Assert.False(profile.HasCommittedProfile);
    }

    [Fact]
    public void HandoverAcrossTheLapSeamStaysInsideTheTrack()
    {
        TrackData track = SilverstoneTrack.Value;
        float startS = track.LengthMeters - 20f;
        const float targetOffset = 6f;
        const float halfWidth = 0.95f;
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);
        profile.UpdateCommittedTarget(
            track,
            startS,
            currentSpeedMetersPerSecond: 50f,
            targetOffset,
            halfWidth
        );

        for (float distance = 0f;
             distance <= 300f;
             distance += 1f)
        {
            TrackSample sample = track.Sample(startS + distance);
            float offset = profile.Resolve(
                track,
                in sample,
                targetOffset,
                executionOffsetMeters: 0f,
                halfWidth
            );
            float minimum = -sample.HalfWidth +
                            halfWidth +
                            TrackConstrainedLateralOffset.EdgeMarginMeters -
                            sample.RefOffset;
            float maximum = sample.HalfWidth -
                            halfWidth -
                            TrackConstrainedLateralOffset.EdgeMarginMeters -
                            sample.RefOffset;
            Assert.InRange(offset, minimum - 1e-4f, maximum + 1e-4f);
        }
    }

    [Fact]
    public void CommittedHandoverRespectsTheOffsetSlopeLimit()
    {
        TrackData track = SilverstoneTrack.Value;
        const float startS = 4656f;
        const float targetOffset = 4f;
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);
        profile.UpdateCommittedTarget(
            track,
            startS,
            currentSpeedMetersPerSecond: 20f,
            targetOffset,
            vehicleHalfWidthMeters: 0.95f
        );

        TrackSample firstSample = track.Sample(startS);
        float previous = profile.Resolve(
            track,
            in firstSample,
            targetOffset,
            executionOffsetMeters: 0f,
            vehicleHalfWidthMeters: 0.95f
        );
        for (float distance = 1f;
             distance <= 300f;
             distance += 1f)
        {
            TrackSample sample = track.Sample(startS + distance);
            float current = profile.Resolve(
                track,
                in sample,
                targetOffset,
                executionOffsetMeters: 0f,
                vehicleHalfWidthMeters: 0.95f
            );
            Assert.InRange(MathF.Abs(current - previous), 0f, 0.0801f);
            previous = current;
        }
    }

    [Theory]
    [InlineData("Simple", 285f, 30f)]
    [InlineData("Simple", 285f, 50f)]
    [InlineData("Silverstone", 4656f, 50f)]
    [InlineData("Silverstone", 4656f, 70f)]
    [InlineData("Shanghai", 4206f, 50f)]
    public void CommittingOffsetDoesNotTurnTheSwitchPointIntoAHairpin(
        string trackName,
        float startS,
        float speed
    )
    {
        TrackData track = TrackNamed(trackName);
        const float targetOffset = 4f;
        TrackSample start = track.Sample(startS);
        RaceCar car = CreateCar(track, start, speed);
        float vehicleHalfWidth = car.Collision.HalfWidthMeters;
        VehicleSpeedPlanningConfig config = new()
        {
            SpeedPlanningHorizonMeters = 600f
        };
        VehicleSpeedPlanner planner = new(config);
        VehicleSpeedLookahead referenceSpeeds = planner.PlanReferenceLookahead(
            new VehicleSpeedLookahead(),
            car,
            track,
            startS,
            config.SpeedPlanningHorizonMeters,
            config.PathPredictionStepMeters,
            DriverPlanningModifiers.Neutral
        );
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);
        ReferenceLineHandoverConstraints handoverConstraints =
            planner.CreateHandoverConstraints(
                car,
                referenceSpeeds,
                latestCompletionDistanceMeters: 300f
            );
        profile.UpdateCommittedTarget(
            track,
            startS,
            speed,
            targetOffset,
            vehicleHalfWidth,
            in handoverConstraints
        );

        StanleyControlSample control = StanleyControlLaw.Sample(
            track,
            car.State.Position,
            car.State.VelocityHeading,
            speed,
            car.CarConfig.WheelBaseMeters,
            vehicleHalfWidth,
            profile,
            targetOffset,
            executionOffsetMeters: 0f,
            stanleyGain: 2f,
            stanleySofteningSpeed: 4f,
            headingGain: 1f,
            curvaturePreviewTimeSeconds: 0.15f,
            maximumCurvaturePreviewMeters: 6f,
            car.CarConfig.MaxCurvatureRequest
        );
        VehiclePathPrediction path = new StanleyPathPredictor().Predict(
            new VehiclePathPrediction(),
            car,
            track,
            referenceSpeeds,
            targetOffset,
            profile,
            executionOffsetMeters: 0f,
            stanleyGain: 2f,
            stanleySofteningSpeed: 4f,
            headingGain: 1f,
            curvaturePreviewTimeSeconds: 0.15f,
            maximumCurvaturePreviewMeters: 6f,
            horizonMeters: config.SpeedPlanningHorizonMeters,
            stepMeters: config.PathPredictionStepMeters,
            minimumDynamicMeters: config.MinimumDynamicPredictionMeters,
            convergenceHoldMeters: config.PredictionConvergenceHoldMeters,
            convergenceLateralErrorMeters:
                config.PredictionConvergenceLateralErrorMeters,
            convergenceHeadingErrorRadians:
                config.PredictionConvergenceHeadingErrorRadians,
            convergenceCurvatureError:
                config.PredictionConvergenceCurvatureError,
            gripUsage: config.GetAccelerationUsage(car.Strategy),
            initialCommandedCurvature: control.DesiredCurvature
        );
        DynamicPathSpeedPlan speedPlan = planner.PlanPredictedPath(
            new VehicleSpeedLookahead(),
            car,
            path
        );
        Assert.InRange(MathF.Abs(control.LateralErrorMeters), 0f, 0.05f);
        Assert.InRange(MathF.Abs(control.DesiredCurvature), 0f, 0.005f);
        Assert.InRange(
            speedPlan.Current.TargetSpeed,
            speed * 0.9f,
            speed + 0.01f
        );
    }

    [Theory]
    [InlineData("Simple")]
    [InlineData("Silverstone")]
    [InlineData("Shanghai")]
    public void HandoverKeepsTransitionFromLoweringTheCurrentSpeedPlan(
        string trackName
    )
    {
        TrackData track = TrackNamed(trackName);
        const float speed = 50f;
        float worstRetainedSpeed = 1f;
        float worstS = 0f;
        float worstOffset = 0f;
        float worstLength = 0f;
        for (float startS = 0f;
             startS < track.LengthMeters;
             startS += 500f)
        {
            foreach (float targetOffset in new[] { -4f, 4f })
            {
                (float retained, float length) = HandoverOutcome(
                    track,
                    startS,
                    speed,
                    targetOffset
                );
                if (retained >= worstRetainedSpeed)
                    continue;

                worstRetainedSpeed = retained;
                worstS = startS;
                worstOffset = targetOffset;
                worstLength = length;
            }
        }

        Assert.True(
            worstRetainedSpeed >= 0.98f,
            $"worst transition retained {worstRetainedSpeed:P1} at " +
            $"s={worstS:F0} m, offset={worstOffset:F1} m, " +
            $"length={worstLength:F1} m"
        );
    }

    private static (float RetainedSpeed, float LengthMeters) HandoverOutcome(
        TrackData track,
        float startS,
        float speed,
        float targetOffset
    )
    {
        TrackSample start = track.Sample(startS);
        RaceCar car = CreateCar(track, start, speed);
        float vehicleHalfWidth = car.Collision.HalfWidthMeters;
        VehicleSpeedPlanningConfig config = new()
        {
            SpeedPlanningHorizonMeters = 600f
        };
        VehicleSpeedPlanner planner = new(config);
        VehicleSpeedLookahead referenceSpeeds = planner.PlanReferenceLookahead(
            new VehicleSpeedLookahead(),
            car,
            track,
            startS,
            config.SpeedPlanningHorizonMeters,
            config.PathPredictionStepMeters,
            DriverPlanningModifiers.Neutral
        );
        StanleyControlSample baselineControl = StanleyControlLaw.Sample(
            track,
            car.State.Position,
            car.State.VelocityHeading,
            speed,
            car.CarConfig.WheelBaseMeters,
            lateralTargetOffsetMeters: 0f,
            stanleyGain: 2f,
            stanleySofteningSpeed: 4f,
            headingGain: 1f,
            curvaturePreviewTimeSeconds: 0.15f,
            maximumCurvaturePreviewMeters: 6f,
            car.CarConfig.MaxCurvatureRequest
        );
        VehiclePathPrediction baselinePath = new StanleyPathPredictor().Predict(
            new VehiclePathPrediction(),
            car,
            track,
            referenceSpeeds,
            lateralTargetOffsetMeters: 0f,
            stanleyGain: 2f,
            stanleySofteningSpeed: 4f,
            headingGain: 1f,
            curvaturePreviewTimeSeconds: 0.15f,
            maximumCurvaturePreviewMeters: 6f,
            horizonMeters: config.SpeedPlanningHorizonMeters,
            stepMeters: config.PathPredictionStepMeters,
            minimumDynamicMeters: config.MinimumDynamicPredictionMeters,
            convergenceHoldMeters: config.PredictionConvergenceHoldMeters,
            convergenceLateralErrorMeters:
                config.PredictionConvergenceLateralErrorMeters,
            convergenceHeadingErrorRadians:
                config.PredictionConvergenceHeadingErrorRadians,
            convergenceCurvatureError:
                config.PredictionConvergenceCurvatureError,
            gripUsage: config.GetAccelerationUsage(car.Strategy),
            initialCommandedCurvature: baselineControl.DesiredCurvature
        );
        VehicleSpeedLookahead baselineSpeedPlan = new();
        DynamicPathSpeedPlan baselineDynamicPlan = planner.PlanPredictedPath(
            baselineSpeedPlan,
            car,
            baselinePath
        );
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);
        ReferenceLineHandoverConstraints handoverConstraints =
            planner.CreateHandoverConstraints(
                car,
                baselineSpeedPlan,
                latestCompletionDistanceMeters: 300f
            );
        profile.UpdateCommittedTarget(
            track,
            startS,
            speed,
            targetOffset,
            vehicleHalfWidth,
            in handoverConstraints
        );
        ReferenceLineHandoverSpeedSelector.Refine(
            profile,
            baselineDynamicPlan.Current.TargetSpeed,
            EvaluateSelectedHandoverSpeed
        );
        float selectedTargetSpeed = EvaluateSelectedHandoverSpeed();
        return (
            selectedTargetSpeed /
            MathF.Max(baselineDynamicPlan.Current.TargetSpeed, 1e-3f),
            profile.CommittedHandoverLengthMeters
        );

        float EvaluateSelectedHandoverSpeed()
        {
            StanleyControlSample control = StanleyControlLaw.Sample(
                track,
                car.State.Position,
                car.State.VelocityHeading,
                speed,
                car.CarConfig.WheelBaseMeters,
                vehicleHalfWidth,
                profile,
                targetOffset,
                executionOffsetMeters: 0f,
                stanleyGain: 2f,
                stanleySofteningSpeed: 4f,
                headingGain: 1f,
                curvaturePreviewTimeSeconds: 0.15f,
                maximumCurvaturePreviewMeters: 6f,
                car.CarConfig.MaxCurvatureRequest
            );
            VehiclePathPrediction path = new StanleyPathPredictor().Predict(
                new VehiclePathPrediction(),
                car,
                track,
                referenceSpeeds,
                targetOffset,
                profile,
                executionOffsetMeters: 0f,
                stanleyGain: 2f,
                stanleySofteningSpeed: 4f,
                headingGain: 1f,
                curvaturePreviewTimeSeconds: 0.15f,
                maximumCurvaturePreviewMeters: 6f,
                horizonMeters: config.SpeedPlanningHorizonMeters,
                stepMeters: config.PathPredictionStepMeters,
                minimumDynamicMeters: config.MinimumDynamicPredictionMeters,
                convergenceHoldMeters: config.PredictionConvergenceHoldMeters,
                convergenceLateralErrorMeters:
                    config.PredictionConvergenceLateralErrorMeters,
                convergenceHeadingErrorRadians:
                    config.PredictionConvergenceHeadingErrorRadians,
                convergenceCurvatureError:
                    config.PredictionConvergenceCurvatureError,
                gripUsage: config.GetAccelerationUsage(car.Strategy),
                initialCommandedCurvature: control.DesiredCurvature
            );
            DynamicPathSpeedPlan speedPlan = planner.PlanPredictedPath(
                new VehicleSpeedLookahead(),
                car,
                path
            );
            return speedPlan.Current.TargetSpeed;
        }
    }

    private static RaceCar CreateCar(
        TrackData track,
        in TrackSample start,
        float speed
    )
    {
        return new RaceCar(
            "handover-test",
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 90f,
                StartingCoreTempC = 90f
            },
            new ReferenceLineDriver(),
            new CarState
            {
                Position = start.RefPosition,
                Heading = start.RefHeading,
                Speed = speed,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
    }

    private static TrackData TrackNamed(string name) => name switch
    {
        "Simple" => SimpleTrack.Value,
        "Shanghai" => ShanghaiTrack.Value,
        _ => SilverstoneTrack.Value
    };

    private static TrackData BuildNarrowTrack()
    {
        return new TrackBuilder(
                Vector2.Zero,
                startWidth: 6f,
                startLeftBuffer: 0f,
                startRightBuffer: 0f
            )
            .AddStraight(100f, targetEndWidth: 6f)
            .AddTurn(180f, 30f, targetEndWidth: 6f)
            .AddStraight(100f, targetEndWidth: 6f)
            .AddTurn(180f, 30f, targetEndWidth: 6f)
            .CloseLoop()
            .Build(new TrackGridConfig());
    }
}
