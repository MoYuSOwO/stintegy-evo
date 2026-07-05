using Godot;
using GdUnit4;
using StintegyEVO.Core.Car;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Online;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;
using StintegyEVO.Core.Track;
using StintegyEVO.Nodes.Race;
using static GdUnit4.Assertions;

namespace StintegyEVO.Tests.Controllers.V1.SpeedPlanning;

[TestSuite]
public sealed class SpeedProfilePlannerTests
{
    [TestCase]
    [RequireGodotRuntime]
    public void PlannerBuildsFiniteProfileForDynamicPath()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 70.0f, height: 24.0f, trackWidth: 18.0f);
        RacingLine racingLine = CenterLineRacingLineSolver.Instance.Generate(track);
        DynamicPathOfflineGraph graph = new DynamicPathOfflineGraphBuilder().Build(track, racingLine, car);
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlinePath path = new DynamicPathOnlinePathPlanner(new DynamicPathOnlineConfig
        {
            MinimumPlanHorizonMeters = 70.0f,
            StartLayerLookahead = 2
        }).PlanFromPose(graph, track, poseNode.Position, poseNode.Heading);

        SpeedProfile profile = new SpeedProfilePlanner(new SpeedPlanningConfig
        {
            MaximumSpeedMetersPerSecond = 70.0f
        }).Plan(path, car, initialSpeedMetersPerSecond: 5.0f);

        AssertThat(profile.Count).IsEqual(path.Samples.Length);
        AssertThat(profile.TotalTime).IsGreater(0.0f);
        AssertThat(profile.PhysicalLength).IsEqualApprox(path.PhysicalLength, 0.5f);
        AssertThat(profile[0].Speed).IsEqualApprox(5.0f, 0.001f);

        foreach (SpeedProfilePoint point in profile.Points)
        {
            AssertThat(float.IsFinite(point.Speed)).IsTrue();
            AssertThat(float.IsFinite(point.TimeFromStart)).IsTrue();
            AssertThat(float.IsFinite(point.AccelerationToNext)).IsTrue();
            AssertThat(point.Speed).IsGreaterEqual(0.0f);
            AssertThat(point.Speed).IsLessEqual(point.MaxSpeed + 0.001f);
        }
    }

    [TestCase]
    [RequireGodotRuntime]
    public void PlannerLimitsSpeedByPathCurvature()
    {
        DynamicPathEdgeSample[] samples = CreateSyntheticSamples(
            sampleCount: 90,
            segmentLength: 2.0f,
            curvedStart: 30,
            curvedEndExclusive: 60,
            curveCurvature: 0.08f
        );
        SpeedPlanningConfig config = new()
        {
            MaximumSpeedMetersPerSecond = 70.0f,
            IncludeAeroDownforce = false,
            IncludeAeroDrag = false
        };

        SpeedProfile profile = new SpeedProfilePlanner(config).Plan(samples, new CarConfig());

        AssertThat(profile[5].MaxSpeed).IsEqualApprox(config.MaximumSpeedMetersPerSecond, 0.001f);
        AssertThat(profile[40].MaxSpeed).IsLess(config.MaximumSpeedMetersPerSecond * 0.5f);
        for (int i = 30; i < 60; i++)
            AssertThat(profile[i].Speed).IsLessEqual(profile[i].MaxSpeed + 0.001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void LoadTransferIterationFeedsBackIntoAccelerationLimit()
    {
        CarConfig car = new();
        car.Distributor.FrontBias = 1.0f;
        car.Power.MaxDriveForce = 100000.0f;
        car.Power.MaxPower = 10000000.0f;
        car.Aero.BaseDragCoef = 0.0f;
        car.Aero.DownforceCoef = 0.0f;
        DynamicPathEdgeSample[] samples =
        [
            new(Vector2.Zero, 0.0f, 0.0f, 20.0f),
            new(new Vector2(20.0f, 0.0f), 0.0f, 0.0f, 0.0f)
        ];
        SpeedPlanningConfig onePassConfig = new()
        {
            MaximumSpeedMetersPerSecond = 90.0f,
            FrictionUsage = 1.0f,
            IncludeAeroDownforce = false,
            IncludeAeroDrag = false,
            LoadTransferIterations = 1
        };
        SpeedPlanningConfig iteratedConfig = new()
        {
            MaximumSpeedMetersPerSecond = 90.0f,
            FrictionUsage = 1.0f,
            IncludeAeroDownforce = false,
            IncludeAeroDrag = false,
            LoadTransferIterations = 5
        };

        SpeedProfile onePassProfile = new SpeedProfilePlanner(onePassConfig).Plan(
            samples,
            car,
            initialSpeedMetersPerSecond: 5.0f
        );
        SpeedProfile iteratedProfile = new SpeedProfilePlanner(iteratedConfig).Plan(
            samples,
            car,
            initialSpeedMetersPerSecond: 5.0f
        );

        AssertThat(onePassProfile[0].AccelerationToNext).IsGreater(7.5f);
        AssertThat(iteratedProfile[0].AccelerationToNext).IsLess(onePassProfile[0].AccelerationToNext - 0.5f);
        AssertThat(iteratedProfile[0].AccelerationToNext).IsGreater(5.5f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void CurrentFramePlanningUsesRuntimeTireGripAndTrackFriction()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 70.0f, height: 24.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        DynamicPathEdgeSample[] samples = CreateSyntheticSamples(
            sampleCount: 60,
            segmentLength: 2.0f,
            curvedStart: 0,
            curvedEndExclusive: 60,
            curveCurvature: 0.06f
        );
        SpeedProfilePlanner planner = new(new SpeedPlanningConfig
        {
            MaximumSpeedMetersPerSecond = 70.0f,
            IncludeAeroDownforce = false,
            IncludeAeroDrag = false
        });
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(5.0f, 0.0f)
        };

        SpeedProfile nominalProfile = planner.Plan(samples, car, initialSpeedMetersPerSecond: sensor.LinearVelocity.Length());

        logic.TireFrontLeft.Pressure = TireConfig.AbsoluteMaxPressure;
        logic.TireFrontRight.Pressure = TireConfig.AbsoluteMaxPressure;
        logic.TireRearLeft.Pressure = TireConfig.AbsoluteMaxPressure;
        logic.TireRearRight.Pressure = TireConfig.AbsoluteMaxPressure;
        track.FrictionMultiplier = 0.55f;

        SpeedProfile currentFrameProfile = planner.PlanCurrentFrame(samples, sensor, logic, track);

        AssertThat(currentFrameProfile[0].MaxSpeed).IsLess(nominalProfile[0].MaxSpeed * 0.8f);
        AssertThat(currentFrameProfile[0].Speed).IsLessEqual(currentFrameProfile[0].MaxSpeed + 0.001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void SignedCurvatureUsesTheLoadedSideTireGrip()
    {
        CarConfig car = new();
        SpeedPlanningState weakLeftTires = new(
            Mass: car.Chassis.DryMass,
            BatterySoc: 1.0f,
            TrackFrictionMultiplier: 1.0f,
            DirtyAirFactor: 0.0f,
            TireFriction: new SpeedPlanningTireFriction(
                LatFrontLeft: 0.5f,
                LatFrontRight: 1.3f,
                LatRearLeft: 0.5f,
                LatRearRight: 1.3f,
                LongFrontLeft: 1.5f,
                LongFrontRight: 1.5f,
                LongRearLeft: 1.5f,
                LongRearRight: 1.5f
            )
        );
        SpeedProfilePlanner planner = new(new SpeedPlanningConfig
        {
            MaximumSpeedMetersPerSecond = 70.0f,
            IncludeAeroDownforce = false,
            IncludeAeroDrag = false
        });
        DynamicPathEdgeSample[] positiveCurvature =
        [
            new(Vector2.Zero, 0.0f, 0.06f, 2.0f),
            new(new Vector2(2.0f, 0.0f), 0.0f, 0.06f, 0.0f)
        ];
        DynamicPathEdgeSample[] negativeCurvature =
        [
            new(Vector2.Zero, 0.0f, -0.06f, 2.0f),
            new(new Vector2(2.0f, 0.0f), 0.0f, -0.06f, 0.0f)
        ];

        SpeedProfile positiveProfile = planner.Plan(positiveCurvature, car, weakLeftTires);
        SpeedProfile negativeProfile = planner.Plan(negativeCurvature, car, weakLeftTires);

        AssertThat(positiveProfile[0].MaxSpeed).IsLess(negativeProfile[0].MaxSpeed - 1.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void IntegrationSubstepsRecomputeAccelerationInsideEachSegment()
    {
        CarConfig car = new();
        car.Power.MaxDriveForce = 100000.0f;
        car.Power.MaxPower = 100000.0f;
        car.Aero.BaseDragCoef = 0.0f;
        car.Aero.DownforceCoef = 0.0f;
        DynamicPathEdgeSample[] samples =
        [
            new(Vector2.Zero, 0.0f, 0.0f, 100.0f),
            new(new Vector2(100.0f, 0.0f), 0.0f, 0.0f, 0.0f)
        ];

        SpeedProfile oneStepProfile = new SpeedProfilePlanner(new SpeedPlanningConfig
        {
            MaximumSpeedMetersPerSecond = 90.0f,
            IncludeAeroDownforce = false,
            IncludeAeroDrag = false,
            IntegrationSubsteps = 1
        }).Plan(samples, car, initialSpeedMetersPerSecond: 10.0f);
        SpeedProfile fourStepProfile = new SpeedProfilePlanner(new SpeedPlanningConfig
        {
            MaximumSpeedMetersPerSecond = 90.0f,
            IncludeAeroDownforce = false,
            IncludeAeroDrag = false,
            IntegrationSubsteps = 4
        }).Plan(samples, car, initialSpeedMetersPerSecond: 10.0f);

        AssertThat(fourStepProfile[1].Speed).IsGreater(oneStepProfile[1].Speed + 1.0f);
        AssertThat(fourStepProfile[1].Speed).IsGreater(10.0f);
    }

    private static DynamicPathEdgeSample[] CreateSyntheticSamples(
        int sampleCount,
        float segmentLength,
        int curvedStart,
        int curvedEndExclusive,
        float curveCurvature
    )
    {
        DynamicPathEdgeSample[] samples = new DynamicPathEdgeSample[sampleCount];
        for (int i = 0; i < samples.Length; i++)
        {
            float curvature = i >= curvedStart && i < curvedEndExclusive ? curveCurvature : 0.0f;
            float lengthToNext = i == samples.Length - 1 ? 0.0f : segmentLength;
            samples[i] = new DynamicPathEdgeSample(
                new Vector2(i * segmentLength, 0.0f),
                0.0f,
                curvature,
                lengthToNext
            );
        }

        return samples;
    }
}
