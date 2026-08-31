using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class StanleyPathPredictorTests
{
    [Fact]
    public void TrackConstrainedOffsetStaysInsideEveryTrackSample()
    {
        TrackData track = BuildVariableWidthTrack();
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);

        const float vehicleHalfWidth = 1f;
        for (int i = 0; i < track.Length; i++)
        {
            TrackSample sample = track.Sample(i * TrackData.StepLength);
            float offset = profile.Resolve(
                track,
                in sample,
                tacticalOffsetMeters: 6f,
                executionOffsetMeters: 0f,
                vehicleHalfWidth
            );
            float maximumOffset = MathF.Max(
                0f,
                sample.HalfWidth -
                sample.RefOffset -
                vehicleHalfWidth -
                0.3f
            );

            Assert.InRange(offset, 0f, maximumOffset + 1e-4f);
        }
    }

    [Fact]
    public void TrackConstrainedOffsetChangesSmoothlyAroundClosedLoop()
    {
        TrackData track = BuildVariableWidthTrack();
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);
        float[] offsets = new float[track.Length];

        for (int i = 0; i < track.Length; i++)
        {
            TrackSample sample = track.Sample(i * TrackData.StepLength);
            offsets[i] = profile.Resolve(
                track,
                in sample,
                tacticalOffsetMeters: 6f,
                executionOffsetMeters: 0f,
                vehicleHalfWidthMeters: 1f
            );
        }

        for (int i = 0; i < offsets.Length; i++)
        {
            float next = offsets[(i + 1) % offsets.Length];
            Assert.InRange(MathF.Abs(next - offsets[i]), 0f, 0.0801f);
        }
    }

    [Fact]
    public void OffsetGeometryUsesTheResolvedTrackConstrainedPosition()
    {
        TrackData track = BuildVariableWidthTrack();
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);
        TrackSample sample = track.Sample(25f);
        float resolvedOffset = profile.Resolve(
            track,
            in sample,
            tacticalOffsetMeters: 6f,
            executionOffsetMeters: 0f,
            vehicleHalfWidthMeters: 1f
        );

        TrackLateralTargetSample target = profile.SampleGeometry(
            track,
            sample.S,
            tacticalOffsetMeters: 6f,
            executionOffsetMeters: 0f,
            vehicleHalfWidthMeters: 1f
        );

        Vector2 expectedPosition = sample.RefPosition +
                                   sample.Normal * resolvedOffset;
        Assert.InRange(
            Vector2.Distance(target.Position, expectedPosition),
            0f,
            1e-4f
        );
        Assert.True(float.IsFinite(target.Heading));
        Assert.True(float.IsFinite(target.Curvature));
        Assert.Equal(resolvedOffset, target.OffsetMeters, 4);
    }

    [Fact]
    public void PredictionConvergesToCommittedOffsetReference()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, speed: 25f, lateralOffset: 0f);
        VehicleSpeedPlanningConfig config = new()
        {
            SpeedPlanningHorizonMeters = 600f
        };
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            car,
            track,
            config
        );
        TrackConstrainedLateralOffset profile = new();
        profile.Prepare(track);

        VehiclePathPrediction prediction = PredictWithOffset(
            car,
            track,
            speedEstimate,
            config,
            tacticalOffsetMeters: 2.5f,
            profile
        );

        VehiclePathPredictionPoint terminal = prediction[prediction.Count - 1];
        TrackPose terminalPose = track.Project(terminal.Position);
        float expectedOffset = profile.Resolve(
            track,
            in terminalPose.Sample,
            tacticalOffsetMeters: 2.5f,
            executionOffsetMeters: 0f,
            car.Collision.HalfWidthMeters
        );
        float actualOffset = terminalPose.D - terminalPose.Sample.RefOffset;
        Assert.InRange(MathF.Abs(actualOffset - expectedOffset), 0f, 0.15f);
    }

    [Fact]
    public void ActiveStabilityCorrectionPersistsIntoPredictedPath()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample sample = track.Sample(20f);
        const float sideslip = -0.12f;
        ReferenceLineDriver driver = new(
            new VehicleSpeedPlanningConfig
            {
                SpeedPlanningHorizonMeters = 20f
            }
        );
        RaceCar car = new(
            "stability-prediction",
            new CarConfig(),
            WarmTires(),
            driver,
            new CarState
            {
                Position = sample.RefPosition,
                Heading = sample.RefHeading - sideslip,
                SideslipAngleRadians = sideslip,
                YawRateRadiansPerSecond = 0f,
                Speed = 55f,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
        TrackPose pose = track.Project(car.State.Position);
        RaceEnvironment environment = new();
        RaceDriverInitContext init = new(car, track, pose, environment, 0f);
        driver.Initialize(in init);
        RaceDriverFrameContext frame = new(car, track, pose, environment, 0f);

        driver.GetControl(in frame, 1f / 60f);

        VehiclePathPrediction prediction = driver.CurrentPathPrediction;
        Assert.True(driver.LastTelemetry.IsRecovering);
        Assert.True(
            MathF.Abs(driver.LastTelemetry.ControlCurvatureCorrection) > 0.005f
        );
        Assert.True(prediction.Count >= 2);
        Assert.True(
            MathF.Abs(prediction[1].StabilityCurvatureCorrection) > 0.005f
        );
        Assert.Equal(
            MathF.Sign(driver.LastTelemetry.ControlCurvatureCorrection),
            MathF.Sign(prediction[1].StabilityCurvatureCorrection)
        );
        Assert.InRange(
            MathF.Abs(
                prediction[1].CommandedCurvature -
                prediction[0].CommandedCurvature
            ),
            0f,
            0.01f
        );
    }

    [Fact]
    public void OffsetPredictionConvergesTowardReferenceLine()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, speed: 16f, lateralOffset: 3f);
        VehicleSpeedPlanningConfig config = new();
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            car,
            track,
            config
        );

        VehiclePathPrediction prediction = Predict(
            car,
            track,
            speedEstimate,
            config,
            0.08f
        );

        Assert.True(prediction.Count > 2);
        Assert.True(
            MathF.Abs(prediction.TerminalLateralErrorMeters) <
            MathF.Abs(prediction[0].LateralErrorMeters)
        );
        Assert.InRange(
            Vector2.Distance(prediction[0].Position, car.State.Position),
            0f,
            1e-5f
        );
    }

    [Fact]
    public void MotionCurvatureRespectsGripWhenCommandIsNotCurrentlyReachable()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, speed: 80f, lateralOffset: 3f);
        VehicleSpeedPlanningConfig config = new();
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            car,
            track,
            config
        );

        VehiclePathPrediction prediction = Predict(
            car,
            track,
            speedEstimate,
            config,
            0.2f
        );

        Assert.InRange(
            MathF.Abs(prediction[0].CommandedCurvature - 0.2f),
            0f,
            1e-5f
        );
        Assert.True(
            MathF.Abs(prediction[0].MotionCurvature) <
            MathF.Abs(prediction[0].CommandedCurvature)
        );
    }

    [Fact]
    public void RepeatedPredictionReusesItsPointStorage()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, speed: 20f, lateralOffset: 2f);
        VehicleSpeedPlanningConfig config = new();
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            car,
            track,
            config
        );
        StanleyPathPredictor predictor = new();
        VehiclePathPrediction destination = new();
        // Warmed properly before anything is counted. One pass leaves branches
        // the prediction only reaches occasionally still uncompiled, and the
        // compiling of them lands in the measurement as bytes this is meant to
        // be watching for - which shows up as a failure that goes away when the
        // test is run by itself.
        for (int i = 0; i < 16; i++)
        {
            Predict(
                car,
                track,
                speedEstimate,
                config,
                0.08f,
                predictor,
                destination
            );
        }

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 100; i++)
        {
            Predict(
                car,
                track,
                speedEstimate,
                config,
                0.08f,
                predictor,
                destination
            );
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.InRange(allocated, 0L, 1024L);
    }

    [Fact]
    public void CallerOwnedPredictionsDoNotAliasEachOther()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, speed: 20f, lateralOffset: 2f);
        VehicleSpeedPlanningConfig config = new();
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            car,
            track,
            config
        );
        StanleyPathPredictor predictor = new();
        VehiclePathPrediction first = new();
        VehiclePathPrediction second = new();

        VehiclePathPrediction returnedFirst = Predict(
            car,
            track,
            speedEstimate,
            config,
            initialCurvature: 0.08f,
            predictor,
            first
        );
        int firstCount = first.Count;
        VehiclePathPredictionPoint firstStart = first[0];
        VehiclePathPredictionPoint firstEnd = first[first.Count - 1];

        VehiclePathPrediction returnedSecond = Predict(
            car,
            track,
            speedEstimate,
            config,
            initialCurvature: -0.08f,
            predictor,
            second
        );

        Assert.Same(first, returnedFirst);
        Assert.Same(second, returnedSecond);
        Assert.Equal(firstCount, first.Count);
        Assert.Equal(firstStart, first[0]);
        Assert.Equal(firstEnd, first[first.Count - 1]);
        Assert.NotEqual(first[0].CommandedCurvature, second[0].CommandedCurvature);
    }

    [Fact]
    public void ConvergedPredictionJoinsReferenceLineBeforeLocalHorizon()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, speed: 30f, lateralOffset: 0f);
        VehicleSpeedPlanningConfig config = new()
        {
            SpeedPlanningHorizonMeters = 600f
        };
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            car,
            track,
            config
        );

        VehiclePathPrediction prediction = Predict(
            car,
            track,
            speedEstimate,
            config,
            initialCurvature: track.Sample(track.Grids[1].S).RefCurvature
        );

        Assert.True(prediction.JoinsReferenceLine);
        Assert.InRange(
            prediction.DynamicPredictionLengthMeters,
            config.MinimumDynamicPredictionMeters,
            200f
        );
        Assert.InRange(
            prediction.ReferenceLineJoinCurvatureDelta,
            0f,
            0.002f
        );
        Assert.Equal(600f, prediction.LengthMeters);
    }

    [Fact]
    public void PredictionStaysCloseToSubsequentClosedLoopMotion()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample start = track.Sample(track.Grids[1].S);
        ReferenceLineDriver driver = new();
        RaceCar car = new(
            "prediction-accuracy",
            new CarConfig(),
            WarmTires(),
            driver,
            new CarState
            {
                Position = track.Grids[1].Position,
                Heading = start.RefHeading,
                Speed = 10f,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);
        simulation.Step(1f / 120f);

        VehiclePathPrediction livePrediction = driver.CurrentPathPrediction;
        VehiclePathPredictionPoint[] prediction =
            new VehiclePathPredictionPoint[livePrediction.Count];
        for (int i = 0; i < prediction.Length; i++)
            prediction[i] = livePrediction[i];

        Vector2 previousPosition = car.State.Position;
        float travelled = 0f;
        float squaredError = 0f;
        float maximumError = 0f;
        int samples = 0;
        int nextPoint = 1;
        while (nextPoint < prediction.Length &&
               prediction[nextPoint].DistanceMeters <= 100f)
        {
            simulation.Step(1f / 120f);
            travelled += Vector2.Distance(previousPosition, car.State.Position);
            previousPosition = car.State.Position;
            if (travelled < prediction[nextPoint].DistanceMeters)
                continue;

            float error = Vector2.Distance(
                car.State.Position,
                prediction[nextPoint].Position
            );
            squaredError += error * error;
            maximumError = MathF.Max(maximumError, error);
            samples++;
            nextPoint++;
        }

        float rmsError = MathF.Sqrt(squaredError / Math.Max(samples, 1));
        Assert.True(samples >= 45);
        Assert.InRange(rmsError, 0f, 0.75f);
        Assert.InRange(maximumError, 0f, 1f);
    }

    private static VehiclePathPrediction Predict(
        RaceCar car,
        TrackData track,
        VehicleSpeedLookahead speedEstimate,
        VehicleSpeedPlanningConfig config,
        float initialCurvature,
        StanleyPathPredictor? predictor = null,
        VehiclePathPrediction? destination = null
    )
    {
        return (predictor ?? new StanleyPathPredictor()).Predict(
            destination ?? new VehiclePathPrediction(),
            car,
            track,
            speedEstimate,
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
            gripUsage: car.TireConfig.GetAccelerationUsage(car.Strategy),
            initialCommandedCurvature: initialCurvature
        );
    }

    private static VehiclePathPrediction PredictWithOffset(
        RaceCar car,
        TrackData track,
        VehicleSpeedLookahead speedEstimate,
        VehicleSpeedPlanningConfig config,
        float tacticalOffsetMeters,
        TrackConstrainedLateralOffset profile
    )
    {
        return new StanleyPathPredictor().Predict(
            new VehiclePathPrediction(),
            car,
            track,
            speedEstimate,
            tacticalOffsetMeters,
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
            gripUsage: car.TireConfig.GetAccelerationUsage(car.Strategy),
            initialCommandedCurvature: track.Project(car.State.Position)
                .Sample.RefCurvature
        );
    }

    private static VehicleSpeedLookahead ReferenceLookahead(
        RaceCar car,
        TrackData track,
        VehicleSpeedPlanningConfig config
    )
    {
        float startS = track.Project(car.State.Position).S;
        return new VehicleSpeedPlanner(config).PlanReferenceLookahead(
            new VehicleSpeedLookahead(),
            car,
            track,
            startS,
            config.SpeedPlanningHorizonMeters,
            config.PathPredictionStepMeters,
            DriverPlanningModifiers.Neutral
        );
    }

    private static RaceCar CreateCar(
        TrackData track,
        float speed,
        float lateralOffset
    )
    {
        TrackSample start = track.Sample(track.Grids[1].S);
        return new RaceCar(
            "predictor-test",
            new CarConfig(),
            WarmTires(),
            new ReferenceLineDriver(),
            new CarState
            {
                Position = start.RefPosition + start.Normal * lateralOffset,
                Heading = start.RefHeading,
                Speed = speed,
                Energy = PowertrainState.Filled(0.9f)
            }
        );
    }

    private static TireConfig WarmTires() => new()
    {
        StartingSurfaceTempC = 90f,
        StartingCoreTempC = 90f
    };

    private static TrackData BuildVariableWidthTrack()
    {
        return new TrackBuilder(
                Vector2.Zero,
                startWidth: 12f,
                startLeftBuffer: 2f,
                startRightBuffer: 2f
            )
            .AddStraight(40f, targetEndWidth: 8f)
            .AddTurn(180f, 24f, targetEndWidth: 8f)
            .AddStraight(40f, targetEndWidth: 12f)
            .AddTurn(180f, 24f, targetEndWidth: 12f)
            .CloseLoop()
            .Build(new TrackGridConfig());
    }
}
