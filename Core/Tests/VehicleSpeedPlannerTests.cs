using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class VehicleSpeedPlannerTests
{
    [Fact]
    public void TireModePresetsIncreaseControllerAccelerationUsage()
    {
        VehicleSpeedPlanningConfig config = new();

        Assert.Equal(0.95f, config.GetAccelerationUsage(TireUsageMode.Protect));
        Assert.Equal(0.955f, config.GetAccelerationUsage(TireUsageMode.Light));
        Assert.Equal(0.96f, config.GetAccelerationUsage(TireUsageMode.Normal));
        Assert.Equal(0.98f, config.GetAccelerationUsage(TireUsageMode.Push));
        Assert.Equal(1f, config.GetAccelerationUsage(TireUsageMode.Attack));
    }

    [Fact]
    public void TireSliderInterpolatesBetweenNonUniformPresets()
    {
        VehicleSpeedPlanningConfig config = new();

        Assert.Equal(0.95f, config.GetAccelerationUsage(0f));
        Assert.Equal(0.955f, config.GetAccelerationUsage(0.25f));
        Assert.Equal(0.96f, config.GetAccelerationUsage(0.5f));
        Assert.Equal(0.98f, config.GetAccelerationUsage(0.75f));
        Assert.Equal(1f, config.GetAccelerationUsage(1f));
        Assert.Equal(0.9575f, config.GetAccelerationUsage(0.375f));
    }

    [Fact]
    public void CustomTireUsageOverridesPresetWithinCalibratedRange()
    {
        VehicleSpeedPlanningConfig config = new();
        CarStrategy strategy = CarStrategy.Default.WithTireGripUsage(0.972f);

        Assert.Equal(0.972f, config.GetAccelerationUsage(strategy));
    }

    [Fact]
    public void ReferenceLookaheadUsesConfiguredLocalHorizon()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedPlanningConfig config = new()
        {
            SpeedPlanningHorizonMeters = 600f,
            PathPredictionStepMeters = 2f
        };
        VehicleSpeedPlanner planner = new(config);

        VehicleSpeedLookahead lookahead = ReferenceLookahead(
            planner,
            car,
            track
        );

        Assert.Equal(301, lookahead.Count);
        Assert.Equal(600f, lookahead.LengthMeters);
        Assert.Equal(2f, lookahead.StepLengthMeters);
    }

    [Fact]
    public void ReferenceLookaheadBrakesBeforeTightCorners()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedLookahead profile = ReferenceLookahead(
            new VehicleSpeedPlanner(),
            car,
            track
        );

        float minimumSpeed = float.PositiveInfinity;
        float maximumSpeed = 0f;
        bool hasBraking = false;
        for (int i = 0; i < profile.Count; i++)
        {
            minimumSpeed = MathF.Min(minimumSpeed, profile[i].TargetSpeed);
            maximumSpeed = MathF.Max(maximumSpeed, profile[i].TargetSpeed);
            hasBraking |= profile[i].ReferenceAcceleration < -0.5f;
        }

        Assert.True(maximumSpeed - minimumSpeed > 5f);
        Assert.True(hasBraking);
    }

    [Fact]
    public void ReferenceLookaheadUsesEachCarsStrategyAndCondition()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar protect = CreateCar(
            track,
            new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Save)
        );
        RaceCar attack = CreateCar(
            track,
            new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Attack)
        );
        float protectAverage = AverageSpeed(ReferenceLookahead(
            new VehicleSpeedPlanner(),
            protect,
            track
        ));
        float attackAverage = AverageSpeed(ReferenceLookahead(
            new VehicleSpeedPlanner(),
            attack,
            track
        ));

        Assert.True(attackAverage > protectAverage);
    }

    [Fact]
    public void BrakeBiasErrorIsIncludedInSpeedPlanning()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedPlanner idealPlanner = new();
        VehicleSpeedPlanner biasedPlanner = new();

        VehicleSpeedLookahead ideal = ReferenceLookahead(
            idealPlanner,
            car,
            track,
            DriverPlanningModifiers.Neutral
        );
        VehicleSpeedLookahead biased = ReferenceLookahead(
            biasedPlanner,
            car,
            track,
            new DriverPlanningModifiers(1f, 1f, 0.07f)
        );

        Assert.True(
            AverageSpeed(biased) < AverageSpeed(ideal),
            "the planner should brake earlier when axle allocation is imperfect"
        );
    }

    [Fact]
    public void MaximumSpeedEstimateAdaptsToVehicleCapabilityBeyondOldFixedCap()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar defaultCar = CreateCar(track, CarStrategy.Default);
        RaceCar lowDragCar = CreateCar(
            track,
            new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Attack),
            new CarConfig
            {
                MassKg = 760f,
                AeroDragAccelPerSpeedSquared = 0.0002f
            }
        );
        VehicleSpeedPlanner planner = new();

        float defaultMaximum = planner.EstimateMaximumSpeedMetersPerSecond(defaultCar);
        float lowDragMaximum = planner.EstimateMaximumSpeedMetersPerSecond(lowDragCar);

        Assert.InRange(defaultMaximum, 105f, 115f);
        Assert.True(lowDragMaximum > 110f);
        Assert.True(lowDragMaximum > defaultMaximum + 20f);
        Assert.Equal(
            lowDragMaximum,
            planner.EstimateLateralSpeedLimit(lowDragCar, curvature: 0f),
            precision: 3
        );
    }

    [Fact]
    public void ReferenceAccelerationMatchesAdjacentLookaheadSpeed()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedLookahead profile = ReferenceLookahead(
            new VehicleSpeedPlanner(),
            car,
            track
        );

        int index = FindModerateAccelerationPoint(profile);
        Assert.True(index >= 0, "profile should contain a non-saturated acceleration segment");

        int next = index + 1;
        float distance = profile.StepLengthMeters;
        float startSpeed = profile[index].TargetSpeed;
        float targetSpeed = profile[next].TargetSpeed;
        float expected = (targetSpeed * targetSpeed - startSpeed * startSpeed) /
                         (2f * distance);

        Assert.InRange(
            MathF.Abs(profile[index].ReferenceAcceleration - expected),
            0f,
            1e-4f
        );
    }

    [Fact]
    public void PredictedRecoveryPathLowersSpeedForExtraCommandedCurvature()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedPlanner planner = new();
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            planner,
            car,
            track
        );
        VehiclePathPrediction baselinePath = PredictPath(
            car,
            track,
            speedEstimate,
            lateralError: 0f,
            initialCurvature: 0f
        );
        DynamicPathSpeedPlan baseline = PlanPredictedPath(
            planner,
            car,
            baselinePath
        );
        speedEstimate = ReferenceLookahead(planner, car, track);
        VehiclePathPrediction correctedPath = PredictPath(
            car,
            track,
            speedEstimate,
            lateralError: 4f,
            initialCurvature: 0.08f
        );
        DynamicPathSpeedPlan corrected = PlanPredictedPath(
            planner,
            car,
            correctedPath
        );

        Assert.True(corrected.Current.TargetSpeed < baseline.Current.TargetSpeed);
        Assert.True(corrected.MaximumAbsoluteCurvature >= 0.08f - 1e-5f);
    }

    [Fact]
    public void DynamicPathPlanUsesInstantaneousAccelerationAtLaunch()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        car.State.Speed = 0f;
        VehicleSpeedPlanner planner = new();
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            planner,
            car,
            track
        );

        VehiclePathPrediction path = PredictPath(
            car,
            track,
            speedEstimate,
            lateralError: 4f,
            initialCurvature: 0.08f
        );
        DynamicPathSpeedPlan plan = PlanPredictedPath(
            planner,
            car,
            path
        );
        float segmentAverage = (
            plan.NextTargetSpeed * plan.NextTargetSpeed -
            plan.Current.TargetSpeed * plan.Current.TargetSpeed
        ) / (2f * plan.FirstSegmentLengthMeters);
        Assert.InRange(
            car.CarConfig.MaxDriveAcceleration - plan.Current.ReferenceAcceleration,
            0f,
            car.CarConfig.MaxDriveAcceleration
        );
        Assert.True(plan.Current.ReferenceAcceleration > segmentAverage);
    }

    [Fact]
    public void AttackBatteryLaunchIsNotCappedAtFourMetersPerSecondSquared()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(
            track,
            new CarStrategy(TireUsageMode.Normal, BatteryOutputMode.Attack)
        );
        car.State.Speed = 0f;
        VehicleSpeedPlanner planner = new();
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            planner,
            car,
            track
        );

        VehiclePathPrediction path = PredictPath(
            car,
            track,
            speedEstimate,
            lateralError: 4f,
            initialCurvature: 0.08f
        );
        DynamicPathSpeedPlan plan = PlanPredictedPath(
            planner,
            car,
            path
        );
        Assert.True(plan.Current.ReferenceAcceleration > 4f);
        Assert.True(
            plan.Current.ReferenceAcceleration <= car.CarConfig.MaxDriveAcceleration
        );
    }

    [Fact]
    public void DriveAccelerationUsageScalesCalculatedLaunchCapability()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        car.State.Speed = 0f;
        VehicleSpeedPlanner fullPlanner = new();
        VehicleSpeedLookahead fullSpeedEstimate = ReferenceLookahead(
            fullPlanner,
            car,
            track
        );
        VehiclePathPrediction fullPath = PredictPath(
            car,
            track,
            fullSpeedEstimate,
            lateralError: 4f,
            initialCurvature: 0.08f
        );
        DynamicPathSpeedPlan fullPlan = PlanPredictedPath(
            fullPlanner,
            car,
            fullPath
        );
        VehicleSpeedPlanner planner = new(
            new VehicleSpeedPlanningConfig { DriveAccelerationUsage = 0.8f }
        );
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            planner,
            car,
            track
        );

        VehiclePathPrediction path = PredictPath(
            car,
            track,
            speedEstimate,
            lateralError: 4f,
            initialCurvature: 0.08f
        );
        DynamicPathSpeedPlan plan = PlanPredictedPath(
            planner,
            car,
            path
        );
        Assert.True(
            plan.Current.ReferenceAcceleration <=
            car.CarConfig.MaxDriveAcceleration * 0.8f + 0.02f
        );
        Assert.True(
            plan.Current.ReferenceAcceleration < fullPlan.Current.ReferenceAcceleration
        );
    }

    [Fact]
    public void CallerOwnedSpeedLookaheadsDoNotAliasEachOther()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedPlanner planner = new();
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            planner,
            car,
            track
        );
        VehiclePathPrediction path = PredictPath(
            car,
            track,
            speedEstimate,
            lateralError: 2f,
            initialCurvature: 0.08f
        );
        VehicleSpeedLookahead first = new();
        VehicleSpeedLookahead second = new();

        planner.PlanPredictedPath(first, car, path);
        int firstCount = first.Count;
        VehicleSpeedPlanPoint firstStart = first[0];
        VehicleSpeedPlanPoint firstEnd = first[first.Count - 1];

        planner.PlanPredictedPath(second, car, path);

        Assert.Equal(firstCount, first.Count);
        Assert.Equal(firstStart, first[0]);
        Assert.Equal(firstEnd, first[first.Count - 1]);
        Assert.NotSame(first, second);
    }

    [Fact]
    public void RepeatedPredictedPathPlanningReusesOutputStorage()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        RaceCar car = CreateCar(track, CarStrategy.Default);
        VehicleSpeedPlanner planner = new();
        VehicleSpeedLookahead speedEstimate = ReferenceLookahead(
            planner,
            car,
            track
        );
        VehiclePathPrediction path = PredictPath(
            car,
            track,
            speedEstimate,
            lateralError: 2f,
            initialCurvature: 0.08f
        );
        VehicleSpeedLookahead destination = new();
        planner.PlanPredictedPath(destination, car, path);

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 50; i++)
            planner.PlanPredictedPath(destination, car, path);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.InRange(allocated, 0L, 256L);
    }

    private static RaceCar CreateCar(
        TrackData track,
        CarStrategy strategy,
        CarConfig? config = null
    )
    {
        TrackSample start = track.Sample(track.Grids[1].S);
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        RaceCar car = new(
            "planner-test",
            config ?? new CarConfig(),
            tires,
            new ReferenceLineDriver(),
            new CarState
            {
                Position = start.RefPosition,
                Heading = start.RefHeading,
                Speed = 20f,
                BatterySoc = 0.8f
            }
        )
        {
            Strategy = strategy
        };
        return car;
    }

    private static VehiclePathPrediction PredictPath(
        RaceCar car,
        TrackData track,
        VehicleSpeedLookahead speedEstimate,
        float lateralError,
        float initialCurvature
    )
    {
        float s = track.Grids[1].S;
        TrackSample sample = track.Sample(s);
        car.State.Position = sample.RefPosition + sample.Normal * lateralError;
        car.State.Heading = sample.RefHeading;
        VehicleSpeedPlanningConfig config = new();
        return new StanleyPathPredictor().Predict(
            new VehiclePathPrediction(),
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
            gripUsage: config.GetAccelerationUsage(car.Strategy),
            initialCommandedCurvature: initialCurvature
        );
    }

    private static VehicleSpeedLookahead ReferenceLookahead(
        VehicleSpeedPlanner planner,
        RaceCar car,
        TrackData track
    )
    {
        return ReferenceLookahead(
            planner,
            car,
            track,
            DriverPlanningModifiers.Neutral
        );
    }

    private static VehicleSpeedLookahead ReferenceLookahead(
        VehicleSpeedPlanner planner,
        RaceCar car,
        TrackData track,
        DriverPlanningModifiers modifiers
    )
    {
        VehicleSpeedPlanningConfig config = planner.Config;
        return planner.PlanReferenceLookahead(
            new VehicleSpeedLookahead(),
            car,
            track,
            track.Project(car.State.Position).S,
            config.SpeedPlanningHorizonMeters,
            config.PathPredictionStepMeters,
            modifiers
        );
    }

    private static DynamicPathSpeedPlan PlanPredictedPath(
        VehicleSpeedPlanner planner,
        RaceCar car,
        VehiclePathPrediction path
    )
    {
        return planner.PlanPredictedPath(
            new VehicleSpeedLookahead(),
            car,
            path
        );
    }

    private static float AverageSpeed(VehicleSpeedLookahead profile)
    {
        float sum = 0f;
        for (int i = 0; i < profile.Count; i++)
            sum += profile[i].TargetSpeed;
        return sum / Math.Max(profile.Count, 1);
    }

    private static int FindModerateAccelerationPoint(
        VehicleSpeedLookahead profile
    )
    {
        for (int i = 0; i < profile.Count - 1; i++)
        {
            float command = profile[i].ReferenceAcceleration;
            if (command > 0.4f && command < 3.5f)
                return i;
        }
        return -1;
    }
}
