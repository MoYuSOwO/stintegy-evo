using System;
using Godot;
using GdUnit4;
using StintegyEVO.Core.Car.Controllers.V1.GraphBased;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;
using StintegyEVO.Core.Track;
using static GdUnit4.Assertions;

namespace StintegyEVO.Tests.Controllers.V1;

[TestSuite]
public sealed class GraphBasedLocalPlannerTests
{
    private static readonly GraphPlannerConfig TestConfig = new()
    {
        HorizonMeters = 80f,
        LayerStepMeters = 4f,
        LateralResolutionMeters = 0.5f,
        VehicleHalfWidthMeters = TrackPlanningBounds.VehicleHalfWidthMeters,
        EdgeSafetyMarginMeters = TrackPlanningBounds.EdgeSafetyMarginMeters,
        LateralOffsetPerMeter = 0.25f,
        VehicleTurnRadiusMeters = 0f
    };

    [TestCase]
    [RequireGodotRuntime]
    public void ReferenceLineInterfaceWrapsExistingRacingLine()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        RacingLine racingLine = CenterLineRacingLineSolver.Instance.Generate(track);
        ITrackReferenceLine reference = new RacingLineReferenceAdapter(racingLine);

        TrackReferencePoint point = reference.GetPoint(12);

        AssertThat(point.TrackIndex).IsEqual(12);
        AssertThat(point.Offset).IsBetween(-0.001f, 0.001f);
        AssertThat(point.Position.DistanceTo(track[12].Center)).IsLess(0.001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void PlannerDoesNotDependOnV1Types()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        var planner = new GraphBasedLocalPlanner();
        var reference = new FixedOffsetReferenceLine(track, 2f);

        GraphPath path = planner.Plan(new GraphPlannerRequest(
            track,
            track[0].Center,
            track[0].Tangent.Angle(),
            reference,
            CreateReferenceSeekingTestConfig()
        ));

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(path.Count).IsEqual(81);
        AssertThat(path[path.Count - 1].Offset).IsGreater(1.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void CenterReferenceKeepsPathNearCenter()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        GraphPath path = PlanFrom(track, offset: 0f, new CenterTrackReferenceLine(track));

        AssertThat(path.IsFallback).IsFalse();
        foreach (GraphPathPoint point in path.Points)
            AssertThat(Mathf.Abs(point.Offset)).IsLess(0.75f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OffsetReferenceIsFollowedSoftly()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        GraphPath path = PlanFrom(track, offset: 0f, new FixedOffsetReferenceLine(track, 3f), CreateReferenceSeekingTestConfig());

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(path[1].Offset).IsLess(1.0f);
        AssertThat(path[20].Offset).IsGreater(0.5f);
        AssertThat(path[path.Count - 1].Offset).IsGreater(1.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OffsetStartReturnsGradually()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        GraphPath path = PlanFrom(track, offset: 5f, new CenterTrackReferenceLine(track));

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(path[0].Offset).IsGreater(4.5f);
        AssertThat(Mathf.Abs(path[2].Offset - path[0].Offset)).IsLess(1.1f);
        AssertThat(Mathf.Abs(path[path.Count - 1].Offset)).IsLess(1.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void PathStaysInsideUsableTrack()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        GraphPath path = PlanFrom(track, offset: 4f, new FixedOffsetReferenceLine(track, 3f));

        AssertThat(path.IsFallback).IsFalse();
        foreach (GraphPathPoint point in path.Points)
            AssertThat(point.BoundaryClearance).IsGreaterEqual(0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void CurvatureAndHeadingAreFinite()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        GraphPath path = PlanFrom(track, offset: 2f, new FixedOffsetReferenceLine(track, -2f));

        AssertThat(path.IsFallback).IsFalse();
        foreach (GraphPathPoint point in path.Points)
        {
            AssertThat(float.IsFinite(point.Heading)).IsTrue();
            AssertThat(float.IsFinite(point.Curvature)).IsTrue();
            AssertThat(Mathf.Abs(point.Curvature)).IsLess(1.0f);
        }
    }

    [TestCase]
    [RequireGodotRuntime]
    public void PlannerIsDeterministic()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ITrackReferenceLine reference = new FixedOffsetReferenceLine(track, 2.5f);
        GraphPath first = PlanFrom(track, offset: -3f, reference);
        GraphPath second = PlanFrom(track, offset: -3f, reference);

        AssertThat(first.Count).IsEqual(second.Count);
        for (int i = 0; i < first.Count; i++)
        {
            AssertThat(first[i].TrackIndex).IsEqual(second[i].TrackIndex);
            AssertThat(Mathf.Abs(first[i].Offset - second[i].Offset)).IsLess(0.0001f);
        }
    }

    [TestCase]
    [RequireGodotRuntime]
    public void PreviousPathBiasKeepsComparablePathStable()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        var planner = new GraphBasedLocalPlanner();
        GraphPath previousPath = PlanFrom(track, offset: 0f, new CenterTrackReferenceLine(track));
        var reference = new FixedOffsetReferenceLine(track, 3f);
        var config = new GraphPlannerConfig
        {
            HorizonMeters = 80f,
            LayerStepMeters = TestConfig.LayerStepMeters,
            LateralResolutionMeters = 0.5f,
            VehicleHalfWidthMeters = TrackPlanningBounds.VehicleHalfWidthMeters,
            EdgeSafetyMarginMeters = TrackPlanningBounds.EdgeSafetyMarginMeters,
            LateralOffsetPerMeter = TestConfig.LateralOffsetPerMeter,
            VehicleTurnRadiusMeters = 0f,
            RacingLineWeight = 50f,
            RacingLineSaturationWeight = 50f,
            PreviousPathWeight = 8f
        };

        GraphPath path = planner.Plan(new GraphPlannerRequest(
            track,
            track[0].Center,
            track[0].Tangent.Angle(),
            reference,
            config,
            previousPath
        ));

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(Mathf.Abs(path[20].Offset)).IsLess(Mathf.Abs(reference.GetPoint(path[20].TrackIndex).Offset));
    }

    [TestCase]
    [RequireGodotRuntime]
    public void PlannedPathCarriesSelectedGraphNodes()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        GraphPath path = PlanFrom(track, offset: 0f, new CenterTrackReferenceLine(track));

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(path.Nodes.Length).IsGreater(0);
        AssertThat(path.Nodes[0].TrackIndex).IsEqual(track[Mathf.RoundToInt(TestConfig.LayerStepMeters)].Index);
        AssertThat(path.Nodes[0].NodeIndex).IsGreaterEqual(0);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void InitialPlanStartsReturningTowardReferenceLineImmediately()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        GraphPath path = PlanFrom(track, offset: 4f, new CenterTrackReferenceLine(track));

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(path[0].Offset).IsGreater(3.5f);
        AssertThat(path[4].Offset).IsLess(path[0].Offset);
        AssertThat(path[12].Offset).IsLess(path[4].Offset);
        AssertThat(Mathf.Abs(path[28].Offset)).IsLess(1.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void RollingPlanKeepsPreviousPrefixAndStartsSearchAhead()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        var planner = new GraphBasedLocalPlanner();
        ITrackReferenceLine reference = new CenterTrackReferenceLine(track);
        GraphPath previousPath = PlanFrom(track, offset: 4f, reference);
        GraphPathPoint currentPoint = previousPath[3];

        GraphPath path = planner.Plan(new GraphPlannerRequest(
            track,
            currentPoint.Position,
            currentPoint.Heading,
            reference,
            TestConfig,
            previousPath
        ));

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(path[0].TrackIndex).IsEqual(previousPath[3].TrackIndex);
        AssertThat(path[1].TrackIndex).IsEqual(previousPath[4].TrackIndex);
        AssertThat(path.Nodes.Length).IsGreater(0);
        AssertThat(path.Nodes[0].SampleIndex).IsGreaterEqual(1);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void SplineEdgeOutputUsesContinuousOffsetsBetweenLatticeNodes()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(80f, 30f, 18f);
        ITrackReferenceLine reference = new CenterTrackReferenceLine(track);
        GraphPlannerConfig config = CreateSplineEdgeTestConfig();

        GraphPath path = PlanFrom(track, offset: 4.3f, reference, config);

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(Mathf.Abs(path[0].Offset - 4.3f)).IsLess(0.05f);

        int continuousInteriorCount = 0;
        for (int i = 1; i < Mathf.Min(path.Count, 30); i++)
        {
            float nearestLattice = Mathf.Round(path[i].Offset / config.LateralResolutionMeters) *
                                   config.LateralResolutionMeters;
            if (Mathf.Abs(path[i].Offset - nearestLattice) > 0.01f)
                continuousInteriorCount++;
        }

        AssertThat(continuousInteriorCount).IsGreater(5);
        AssertThat(MaxAbsSecondDiff(path)).IsLess(0.75f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void SimpleTestGridStartDoesNotHoldStraightLineBeforeReturning()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        var solver = new MinimumCurvatureRacingLineSolver();
        ITrackReferenceLine reference = new RacingLineReferenceAdapter(solver.Generate(track));
        Grid grid = track.Grids[1];

        var planner = new GraphBasedLocalPlanner();
        GraphPath path = planner.Plan(new GraphPlannerRequest(
            track,
            grid.Position,
            track[grid.Index].Tangent.Angle(),
            reference,
            TestConfig
        ));

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(path[0].Offset).IsGreater(4.0f);
        TrackReferencePoint startReference = reference.GetPoint(path[0].TrackIndex);
        TrackReferencePoint laterReference = reference.GetPoint(path[28].TrackIndex);
        float startReferenceError = Mathf.Abs(path[0].Offset - startReference.Offset);
        float laterReferenceError = Mathf.Abs(path[28].Offset - laterReference.Offset);
        AssertThat(laterReferenceError).IsLess(startReferenceError);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void RollingPlanDoesNotReusePreviousPathWhenCarIsFarAway()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ITrackReferenceLine reference = new CenterTrackReferenceLine(track);
        Grid grid = track.Grids[1];
        var planner = new GraphBasedLocalPlanner();
        GraphPath previousPath = planner.Plan(new GraphPlannerRequest(
            track,
            grid.Position,
            track[grid.Index].Tangent.Angle(),
            reference,
            TestConfig
        ));

        Vector2 oppositeSide = track[grid.Index].GetOffsetPos(-5f);
        GraphPath path = planner.Plan(new GraphPlannerRequest(
            track,
            oppositeSide,
            track[grid.Index].Tangent.Angle(),
            reference,
            TestConfig,
            previousPath
        ));

        AssertThat(path.IsFallback).IsFalse();
        AssertThat(path[0].Offset).IsLess(-4.5f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OnlineHandlerInitializesThenReusesPreviousPath()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ITrackReferenceLine reference = new CenterTrackReferenceLine(track);
        Grid grid = track.Grids[1];
        var handler = new GraphOnlineTrajectoryHandler(new GraphBasedLocalPlanner());

        GraphOnlineTrajectory first = handler.Plan(new GraphPlannerRequest(
            track,
            grid.Position,
            track[grid.Index].Tangent.Angle(),
            reference,
            TestConfig
        ));
        GraphPathPoint currentPoint = first.Path[3];
        GraphOnlineTrajectory second = handler.Plan(new GraphPlannerRequest(
            track,
            currentPoint.Position,
            currentPoint.Heading,
            reference,
            TestConfig
        ));

        AssertThat(first.Path.IsFallback).IsFalse();
        AssertThat(first.ReusedPreviousPath).IsFalse();
        AssertThat(first.ResetPreviousPath).IsFalse();
        AssertThat(second.Path.IsFallback).IsFalse();
        AssertThat(second.ReusedPreviousPath).IsTrue();
        AssertThat(second.ResetPreviousPath).IsFalse();
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OnlineHandlerResetsPreviousPathWhenCarIsFarAway()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        ITrackReferenceLine reference = new CenterTrackReferenceLine(track);
        Grid grid = track.Grids[1];
        var handler = new GraphOnlineTrajectoryHandler(new GraphBasedLocalPlanner());

        GraphOnlineTrajectory first = handler.Plan(new GraphPlannerRequest(
            track,
            grid.Position,
            track[grid.Index].Tangent.Angle(),
            reference,
            TestConfig
        ));
        Vector2 oppositeSide = track[grid.Index].GetOffsetPos(-5f);
        GraphOnlineTrajectory second = handler.Plan(new GraphPlannerRequest(
            track,
            oppositeSide,
            track[grid.Index].Tangent.Angle(),
            reference,
            TestConfig
        ));

        AssertThat(first.Path.IsFallback).IsFalse();
        AssertThat(second.Path.IsFallback).IsFalse();
        AssertThat(second.ReusedPreviousPath).IsFalse();
        AssertThat(second.ResetPreviousPath).IsTrue();
        AssertThat(second.Path[0].Offset).IsLess(-4.5f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void VelocityProfileOnStraightPathAcceleratesTowardMaxSpeed()
    {
        GraphPath path = BuildSyntheticPath(80, curvature: 0f);
        var solver = new ForwardBackwardVelocityProfileSolver();

        VelocityProfile profile = solver.Solve(new VelocityProfileRequest(
            path,
            StartSpeed: 5f,
            MaxSpeed: 20f,
            MaxLongitudinalAccel: 3f,
            MaxLongitudinalDecel: 7f,
            MaxLateralAccel: 6f
        ));

        AssertThat(profile.Count).IsEqual(path.Count);
        AssertThat(profile[0].TargetSpeed).IsEqualApprox(5f, 0.001f);
        AssertThat(profile[10].TargetSpeed).IsGreater(profile[0].TargetSpeed);
        AssertThat(profile[profile.Count - 1].TargetSpeed).IsLessEqual(20.001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void VelocityProfileLimitsSpeedFromPathCurvature()
    {
        GraphPath path = BuildSyntheticPath(40, curvature: 0.1f);
        var solver = new ForwardBackwardVelocityProfileSolver();

        VelocityProfile profile = solver.Solve(new VelocityProfileRequest(
            path,
            StartSpeed: 20f,
            MaxSpeed: 30f,
            MaxLongitudinalAccel: 5f,
            MaxLongitudinalDecel: 8f,
            MaxLateralAccel: 6f,
            SafetyFactor: 1f
        ));

        float expectedCurveSpeed = Mathf.Sqrt(6f / 0.1f);
        AssertThat(profile[0].TargetSpeed).IsEqualApprox(expectedCurveSpeed, 0.001f);
        foreach (VelocityProfilePoint point in profile.Points)
            AssertThat(point.TargetSpeed).IsLessEqual(expectedCurveSpeed + 0.001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void VelocityProfileBackwardPassBrakesBeforeEndLimit()
    {
        GraphPath path = BuildSyntheticPath(80, curvature: 0f);
        var solver = new ForwardBackwardVelocityProfileSolver();

        VelocityProfile profile = solver.Solve(new VelocityProfileRequest(
            path,
            StartSpeed: 20f,
            MaxSpeed: 30f,
            MaxLongitudinalAccel: 5f,
            MaxLongitudinalDecel: 5f,
            MaxLateralAccel: 6f,
            EndSpeed: 5f,
            EnforceEndSpeed: true
        ));

        AssertThat(profile[profile.Count - 1].TargetSpeed).IsEqualApprox(5f, 0.001f);
        AssertThat(profile[profile.Count - 10].TargetSpeed).IsGreater(profile[profile.Count - 1].TargetSpeed);
        AssertThat(profile[profile.Count - 10].TargetAcceleration).IsLess(0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FbgaVelocityProfileBrakesBeforeCurvatureLimit()
    {
        GraphPath path = BuildSyntheticPath(100, index => index < 50 ? 0f : 0.08f);
        var envelope = new EllipticAccelerationEnvelope(4f, 7f, 6f);
        var solver = new FbgaVelocityProfileSolver();

        VelocityProfile profile = solver.Solve(new VelocityProfileRequest(
            path,
            StartSpeed: 25f,
            MaxSpeed: 30f,
            MaxLongitudinalAccel: 4f,
            MaxLongitudinalDecel: 7f,
            MaxLateralAccel: 6f,
            AccelerationEnvelope: envelope
        ));

        float curveSpeedLimit = Mathf.Sqrt(6f / 0.08f);
        AssertThat(profile[55].TargetSpeed).IsLessEqual(curveSpeedLimit + 0.01f);
        AssertThat(profile[40].TargetSpeed).IsGreater(profile[55].TargetSpeed);
        bool hasBrakingBeforeCorner = false;
        for (int i = 35; i < 55; i++)
            hasBrakingBeforeCorner |= profile[i].TargetAcceleration < 0f;
        AssertThat(hasBrakingBeforeCorner).IsTrue();
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FbgaVelocityProfileRespectsAccelerationEnvelope()
    {
        GraphPath path = BuildSyntheticPath(90, index => index < 30 ? 0.0f : index < 60 ? 0.04f : 0.08f);
        var envelope = new EllipticAccelerationEnvelope(4f, 7f, 6f);
        var solver = new FbgaVelocityProfileSolver();

        VelocityProfile profile = solver.Solve(new VelocityProfileRequest(
            path,
            StartSpeed: 12f,
            MaxSpeed: 30f,
            MaxLongitudinalAccel: 4f,
            MaxLongitudinalDecel: 7f,
            MaxLateralAccel: 6f,
            AccelerationEnvelope: envelope
        ));

        for (int i = 0; i < profile.Count - 1; i++)
        {
            float speed = profile[i].TargetSpeed;
            float lateralAccel = path[i].Curvature * speed * speed;
            envelope.GetLateralBounds(speed, out float minAy, out float maxAy);
            envelope.GetLongitudinalBounds(lateralAccel, speed, out float minAx, out float maxAx);

            AssertThat(lateralAccel).IsGreaterEqual(minAy - 0.001f);
            AssertThat(lateralAccel).IsLessEqual(maxAy + 0.001f);
            AssertThat(profile[i].TargetAcceleration).IsGreaterEqual(minAx - 0.01f);
            AssertThat(profile[i].TargetAcceleration).IsLessEqual(maxAx + 0.01f);
        }
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FbgaVelocityProfileUsesRemainingFrictionForAcceleration()
    {
        GraphPath straight = BuildSyntheticPath(30, curvature: 0f);
        GraphPath curved = BuildSyntheticPath(30, curvature: 0.05f);
        var envelope = new EllipticAccelerationEnvelope(4f, 7f, 6f);
        var solver = new FbgaVelocityProfileSolver();

        VelocityProfile straightProfile = solver.Solve(new VelocityProfileRequest(
            straight,
            StartSpeed: 8f,
            MaxSpeed: 30f,
            MaxLongitudinalAccel: 4f,
            MaxLongitudinalDecel: 7f,
            MaxLateralAccel: 6f,
            AccelerationEnvelope: envelope
        ));
        VelocityProfile curvedProfile = solver.Solve(new VelocityProfileRequest(
            curved,
            StartSpeed: 8f,
            MaxSpeed: 30f,
            MaxLongitudinalAccel: 4f,
            MaxLongitudinalDecel: 7f,
            MaxLateralAccel: 6f,
            AccelerationEnvelope: envelope
        ));

        AssertThat(straightProfile[0].TargetAcceleration).IsGreater(curvedProfile[0].TargetAcceleration);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FbgaVelocityProfileNeverExceedsMaxSpeedOrSaturation()
    {
        GraphPath path = BuildSyntheticPath(80, index => index < 40 ? 0f : 0.06f);
        var envelope = new EllipticAccelerationEnvelope(4f, 7f, 6f);
        var solver = new FbgaVelocityProfileSolver();
        const float maxSpeed = 22f;

        VelocityProfile profile = solver.Solve(new VelocityProfileRequest(
            path,
            StartSpeed: 25f,
            MaxSpeed: maxSpeed,
            MaxLongitudinalAccel: 4f,
            MaxLongitudinalDecel: 7f,
            MaxLateralAccel: 6f,
            AccelerationEnvelope: envelope
        ));

        float curveSaturation = Mathf.Sqrt(6f / 0.06f);
        for (int i = 0; i < profile.Count; i++)
        {
            AssertThat(profile[i].TargetSpeed).IsLessEqual(maxSpeed + 0.001f);
            if (i >= 40)
                AssertThat(profile[i].TargetSpeed).IsLessEqual(curveSaturation + 0.001f);
        }
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FbgaVelocityProfileStartsFromActualSpeedAndStaysForwardReachable()
    {
        GraphPath path = BuildSyntheticPath(80, curvature: 0f);
        var envelope = new EllipticAccelerationEnvelope(4f, 7f, 6f);
        var solver = new FbgaVelocityProfileSolver();

        VelocityProfile profile = solver.Solve(new VelocityProfileRequest(
            path,
            StartSpeed: 0f,
            MaxSpeed: 30f,
            MaxLongitudinalAccel: 4f,
            MaxLongitudinalDecel: 7f,
            MaxLateralAccel: 6f,
            AccelerationEnvelope: envelope
        ));

        AssertThat(profile[0].TargetSpeed).IsEqualApprox(0f, 0.001f);
        AssertThat(profile[6].TargetSpeed).IsLessEqual(Mathf.Sqrt(2f * 4f * 6f) + 0.001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FbgaVelocityProfileAccelerationReferenceStaysBoundedWhenSpeedCapIsUnreachable()
    {
        GraphPath path = BuildSyntheticPath(40, index => index < 5 ? 0f : 0.2f);
        var envelope = new EllipticAccelerationEnvelope(4f, 7f, 6f);
        var solver = new FbgaVelocityProfileSolver();

        VelocityProfile profile = solver.Solve(new VelocityProfileRequest(
            path,
            StartSpeed: 25f,
            MaxSpeed: 30f,
            MaxLongitudinalAccel: 4f,
            MaxLongitudinalDecel: 7f,
            MaxLateralAccel: 6f,
            AccelerationEnvelope: envelope
        ));

        foreach (VelocityProfilePoint point in profile.Points)
        {
            AssertThat(point.TargetAcceleration).IsGreaterEqual(-7.001f);
            AssertThat(point.TargetAcceleration).IsLessEqual(4.001f);
        }
    }

    private static GraphPath PlanFrom(TrackData track, float offset, ITrackReferenceLine reference)
    {
        return PlanFrom(track, offset, reference, TestConfig);
    }

    private static GraphPath PlanFrom(TrackData track, float offset, ITrackReferenceLine reference, GraphPlannerConfig config)
    {
        TrackPoint start = track[0];
        var planner = new GraphBasedLocalPlanner();
        return planner.Plan(new GraphPlannerRequest(
            track,
            start.GetOffsetPos(offset),
            start.Tangent.Angle(),
            reference,
            config
        ));
    }

    private static GraphPlannerConfig CreateSplineEdgeTestConfig()
    {
        return new GraphPlannerConfig
        {
            HorizonMeters = TestConfig.HorizonMeters,
            LayerStepMeters = TestConfig.LayerStepMeters,
            LateralResolutionMeters = TestConfig.LateralResolutionMeters,
            VehicleHalfWidthMeters = TestConfig.VehicleHalfWidthMeters,
            EdgeSafetyMarginMeters = TestConfig.EdgeSafetyMarginMeters,
            LateralOffsetPerMeter = 0.3125f,
            VehicleTurnRadiusMeters = 0f
        };
    }

    private static GraphPlannerConfig CreateReferenceSeekingTestConfig()
    {
        return new GraphPlannerConfig
        {
            HorizonMeters = TestConfig.HorizonMeters,
            LayerStepMeters = TestConfig.LayerStepMeters,
            LateralResolutionMeters = TestConfig.LateralResolutionMeters,
            VehicleHalfWidthMeters = TestConfig.VehicleHalfWidthMeters,
            EdgeSafetyMarginMeters = TestConfig.EdgeSafetyMarginMeters,
            LateralOffsetPerMeter = TestConfig.LateralOffsetPerMeter,
            VehicleTurnRadiusMeters = 0f,
            RacingLineWeight = 50f,
            RacingLineSaturationWeight = 50f
        };
    }

    private static float MaxAbsSecondDiff(GraphPath path)
    {
        float max = 0f;
        for (int i = 2; i < path.Count; i++)
        {
            float secondDiff = path[i].Offset - 2f * path[i - 1].Offset + path[i - 2].Offset;
            max = Mathf.Max(max, Mathf.Abs(secondDiff));
        }

        return max;
    }

    private static GraphPath BuildSyntheticPath(int count, float curvature)
    {
        return BuildSyntheticPath(count, _ => curvature);
    }

    private static GraphPath BuildSyntheticPath(int count, Func<int, float> curvature)
    {
        GraphPathPoint[] points = new GraphPathPoint[count];
        for (int i = 0; i < count; i++)
        {
            points[i] = new GraphPathPoint(
                i,
                i,
                0f,
                new Vector2(i, 0f),
                0f,
                curvature(i),
                5f
            );
        }

        return new GraphPath(points, isFallback: false);
    }

    private sealed class FixedOffsetReferenceLine(TrackData track, float offset) : ITrackReferenceLine
    {
        public TrackReferencePoint GetPoint(int trackIndex)
        {
            TrackPoint point = track[trackIndex];
            Vector2 prev = track[trackIndex - 1].Center;
            Vector2 next = track[trackIndex + 1].Center;
            return new TrackReferencePoint(
                point.Index,
                offset,
                point.GetOffsetPos(offset),
                (next - prev).Angle(),
                0f
            );
        }
    }
}
