using System;
using Godot;
using GdUnit4;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Online;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Track;
using StintegyEVO.Util;
using static GdUnit4.Assertions;

namespace StintegyEVO.Tests.Controllers.V1.DynamicPath.Online;

[TestSuite]
public sealed class GraphBasedLocalPlannerTests
{
    [TestCase]
    [RequireGodotRuntime]
    public void OnlinePlannerConnectsPoseToLatticeAndReturnsSingleVehiclePath()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlineConfig config = new()
        {
            MinimumPlanHorizonMeters = 60.0f,
            StartLayerLookahead = 2
        };

        DynamicPathOnlinePath path = new DynamicPathOnlinePathPlanner(config).PlanFromPose(
            graph,
            track,
            poseNode.Position,
            poseNode.Heading
        );

        AssertThat(path.StartNode.Layer).IsEqual(2);
        AssertThat(path.NodePath.Length).IsGreater(2);
        AssertThat(path.EdgePath.Length).IsEqual(path.NodePath.Length - 1);
        AssertThat(path.SmoothedSplines.Length).IsEqual(path.EdgePath.Length);
        AssertThat(path.InitialConnectorSpline).IsNotNull();
        AssertThat(path.HeadingCompatible).IsTrue();
        AssertThat(path.Samples.Length).IsGreater(path.NodePath.Length);
        AssertThat(path.PhysicalLength).IsGreater(0.0f);
        AssertThat(path.StartPosition.X).IsEqualApprox(poseNode.Position.X, 0.001f);
        AssertThat(path.StartPosition.Y).IsEqualApprox(poseNode.Position.Y, 0.001f);

        AssertPathSamplesAreFinite(path);
        AssertNodeSamplesAlignWithGraph(graph, path);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FreshPlannerAcceptsPoseInsideTrackBuffer()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        TrackPoint trackPoint = track[poseLayer.TrackIndex];
        Vector2 bufferPosition = trackPoint.Center
            + trackPoint.Normal * (trackPoint.HalfWidth + trackPoint.LeftBufferWidth * 0.5f);
        int anchorTrackIndex = track.FindNearestIndex(bufferPosition);
        DynamicPathOnlineConfig config = new()
        {
            MinimumPlanHorizonMeters = 80.0f,
            StartLayerLookahead = 0,
            MaxPoseOffsetMeters = 0.25f
        };

        DynamicPathOnlinePath path = new DynamicPathOnlinePathPlanner(config).PlanFromPose(
            graph,
            track,
            bufferPosition,
            trackPoint.Tangent.Angle()
        );

        AssertThat(path.UsedPreviousPath).IsFalse();
        AssertThat(path.InitialConnectorSpline).IsNotNull();
        AssertThat(path.AnchorTrackIndex).IsEqual(anchorTrackIndex);
        AssertThat(path.Samples[0].Position.X).IsEqualApprox(bufferPosition.X, 0.001f);
        AssertThat(path.Samples[0].Position.Y).IsEqualApprox(bufferPosition.Y, 0.001f);
        AssertPathSamplesAreFinite(path);
        AssertNodeSamplesAlignWithGraph(graph, path);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OnlinePlannerSearchesToVirtualGoalAndRecomputesContinuousSpline()
    {
        (_, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer startLayer = graph.Layers[0];
        DynamicPathNodeId startNode = startLayer.Nodes[startLayer.RaceLineNodeIndex].Id;
        DynamicPathOnlineConfig config = new()
        {
            MinimumPlanHorizonMeters = 50.0f
        };

        DynamicPathOnlinePath path = new DynamicPathOnlinePathPlanner(config).PlanFromNode(graph, startNode);

        AssertThat(path.NodePath[0]).IsEqual(startNode);
        AssertThat(path.GoalLayer).IsEqual(path.GoalNode.Layer);
        AssertThat(graph.GetNode(path.GoalNode).IsRaceLine).IsTrue();
        AssertThat(path.VirtualGoalCost).IsEqualApprox(0.0f, 0.0001f);
        AssertThat(path.InitialConnectorSpline).IsNull();
        AssertThat(path.Samples.Length).IsGreater(path.EdgePath.Length);
        AssertThat(path.StartPosition.X).IsEqualApprox(graph.GetNode(startNode).Position.X, 0.001f);
        AssertThat(path.StartPosition.Y).IsEqualApprox(graph.GetNode(startNode).Position.Y, 0.001f);

        AssertPathSamplesAreFinite(path);
        AssertNodeSamplesAlignWithGraph(graph, path);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OnlinePlannerCanUseLayerCountHorizon()
    {
        (_, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer startLayer = graph.Layers[0];
        DynamicPathNodeId startNode = startLayer.Nodes[startLayer.RaceLineNodeIndex].Id;
        DynamicPathOnlineConfig config = new()
        {
            PlanHorizonMode = DynamicPathPlanHorizonMode.Layers,
            MinimumPlanHorizonLayers = 3
        };

        DynamicPathOnlinePath path = new DynamicPathOnlinePathPlanner(config).PlanFromNode(graph, startNode);

        AssertThat(path.RequestedGoalLayer).IsEqual(3);
        AssertThat(path.GoalLayer).IsEqual(3);
        AssertNodeSamplesAlignWithGraph(graph, path);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FreshPlannerSkipsPrunedLateralStartNode()
    {
        DynamicPathOfflineGraph graph = BuildGraphWithIsolatedLateralNodes();
        DynamicPathNode isolatedNode = graph.Layers[0].Nodes[2];
        DynamicPathNode reachableNode = graph.Layers[0].Nodes[1];
        DynamicPathOnlineConfig config = new()
        {
            PlanHorizonMode = DynamicPathPlanHorizonMode.Layers,
            MinimumPlanHorizonLayers = 1,
            StartLayerLookahead = 0
        };

        DynamicPathOnlinePath path = new DynamicPathOnlinePathPlanner(config).PlanFromPose(
            graph,
            isolatedNode.Position,
            isolatedNode.Heading
        );

        AssertThat(graph.GetOutgoingEdges(isolatedNode.Id).Count).IsEqual(0);
        AssertThat(graph.GetOutgoingEdges(reachableNode.Id).Count).IsGreater(0);
        AssertThat(path.StartNode).IsEqual(reachableNode.Id);
        AssertNodeSamplesAlignWithGraph(graph, path);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void ContinuingPlannerKeepsPreviousPrefixAndReplansFromFutureNode()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlineConfig config = new()
        {
            MinimumPlanHorizonMeters = 80.0f,
            StartLayerLookahead = 0,
            CalculationTimeSafetyFactor = 0.0f
        };
        DynamicPathOnlinePathPlanner planner = new(config);

        DynamicPathOnlinePath first = planner.PlanFromPoseContinuing(
            graph,
            track,
            poseNode.Position,
            poseNode.Heading,
            speedMetersPerSecond: 0.0f
        );
        int sampleBeforeNextNode = Math.Max(
            first.NodeSampleIndexes[0] + 1,
            (first.NodeSampleIndexes[0] + first.NodeSampleIndexes[1]) / 2
        );
        sampleBeforeNextNode = Math.Min(sampleBeforeNextNode, first.NodeSampleIndexes[1] - 1);
        DynamicPathEdgeSample currentSample = first.Samples[sampleBeforeNextNode];

        DynamicPathOnlinePath second = planner.PlanFromPoseContinuing(
            graph,
            track,
            currentSample.Position,
            currentSample.Heading,
            speedMetersPerSecond: 0.0f
        );

        AssertThat(second.UsedPreviousPath).IsTrue();
        AssertThat(second.ConstantPrefixSampleCount).IsGreater(1);
        AssertThat(second.StartNode).IsEqual(first.NodePath[1]);
        AssertThat(second.Samples[0].Position.X).IsEqualApprox(currentSample.Position.X, 0.001f);
        AssertThat(second.Samples[0].Position.Y).IsEqualApprox(currentSample.Position.Y, 0.001f);

        float expectedSuffixStartHeading = first.Samples[first.NodeSampleIndexes[1]].Heading;
        float actualSuffixStartHeading = second.SmoothedSplines[0].Derivative(0.0f).Angle();
        AssertThat(Mathf.Abs(GeomUtil.NormalizeAngle(actualSuffixStartHeading - expectedSuffixStartHeading))).IsLess(0.001f);

        AssertPathSamplesAreFinite(second);
        AssertNodeSamplesAlignWithGraph(graph, second);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void ContinuingPlannerCutsFromProjectionOnPreviousTrajectory()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlineConfig config = new()
        {
            MinimumPlanHorizonMeters = 80.0f,
            StartLayerLookahead = 0,
            CalculationTimeSafetyFactor = 0.0f
        };
        DynamicPathOnlinePathPlanner planner = new(config);

        DynamicPathOnlinePath first = planner.PlanFromPoseContinuing(
            graph,
            track,
            poseNode.Position,
            poseNode.Heading,
            speedMetersPerSecond: 0.0f
        );
        int sampleBeforeNextNode = Math.Max(
            first.NodeSampleIndexes[0] + 1,
            (first.NodeSampleIndexes[0] + first.NodeSampleIndexes[1]) / 2
        );
        sampleBeforeNextNode = Math.Min(sampleBeforeNextNode, first.NodeSampleIndexes[1] - 1);
        DynamicPathEdgeSample projectedSample = first.Samples[sampleBeforeNextNode];
        TrackPoint projectedTrackPoint = track[track.FindNearestIndex(projectedSample.Position)];
        float offsetDistance = MathF.Min(
            graph.VehicleHalfWidthMeters,
            graph.Config.LateralResolutionMeters * 1.5f
        );
        Vector2 offsetPosition = projectedSample.Position + projectedTrackPoint.Normal * offsetDistance;

        DynamicPathOnlinePath second = planner.PlanFromPoseContinuing(
            graph,
            track,
            offsetPosition,
            projectedSample.Heading,
            speedMetersPerSecond: 0.0f
        );

        AssertThat(second.UsedPreviousPath).IsTrue();
        AssertThat(second.Samples[0].Position.DistanceTo(projectedSample.Position)).IsLess(2.0f);
        AssertThat(second.Samples[0].Position.DistanceTo(offsetPosition)).IsGreater(offsetDistance * 0.5f);
        AssertThat(second.SampleTrackProgress[0]).IsEqualApprox(0.0f, 0.001f);
        AssertPathSamplesAreFinite(second);
        AssertNodeSamplesAlignWithGraph(graph, second);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void ContinuingPlannerKeepsPreviousPathAfterLargeLateralOffsetInsideTrack()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlineConfig config = new()
        {
            MinimumPlanHorizonMeters = 80.0f,
            StartLayerLookahead = 0,
            CalculationTimeSafetyFactor = 0.0f
        };
        DynamicPathOnlinePathPlanner planner = new(config);

        DynamicPathOnlinePath first = planner.PlanFromPoseContinuing(
            graph,
            track,
            poseNode.Position,
            poseNode.Heading,
            speedMetersPerSecond: 0.0f
        );
        int sampleBeforeNextNode = Math.Max(
            first.NodeSampleIndexes[0] + 1,
            (first.NodeSampleIndexes[0] + first.NodeSampleIndexes[1]) / 2
        );
        sampleBeforeNextNode = Math.Min(sampleBeforeNextNode, first.NodeSampleIndexes[1] - 1);
        DynamicPathEdgeSample projectedSample = first.Samples[sampleBeforeNextNode];
        TrackPoint projectedTrackPoint = track[track.FindNearestIndex(projectedSample.Position)];
        float inheritedOffsetLimit = MathF.Max(
            graph.VehicleHalfWidthMeters * 2.0f,
            graph.Config.LateralResolutionMeters * 2.0f
        );
        Vector2 offsetPosition = projectedSample.Position
            + projectedTrackPoint.Normal * (inheritedOffsetLimit + graph.Config.LateralResolutionMeters);

        DynamicPathOnlinePath second = planner.PlanFromPoseContinuing(
            graph,
            track,
            offsetPosition,
            projectedSample.Heading,
            speedMetersPerSecond: 0.0f
        );

        AssertThat(second.UsedPreviousPath).IsTrue();
        AssertThat(second.InitialConnectorSpline).IsNull();
        AssertThat(second.ConstantPrefixSampleCount).IsGreater(1);
        AssertThat(second.StartNode).IsEqual(first.NodePath[1]);
        AssertThat(second.Samples[0].Position.DistanceTo(projectedSample.Position)).IsLess(2.0f);
        AssertThat(second.Samples[0].Position.DistanceTo(offsetPosition)).IsGreater(offsetPosition.DistanceTo(projectedSample.Position) * 0.5f);
        AssertPathSamplesAreFinite(second);
        AssertNodeSamplesAlignWithGraph(graph, second);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void ContinuingPlannerTreatsReachedNodeIndexAsConsumed()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlineConfig config = new()
        {
            MinimumPlanHorizonMeters = 80.0f,
            StartLayerLookahead = 0,
            CalculationTimeSafetyFactor = 0.0f
        };
        DynamicPathOnlinePathPlanner planner = new(config);

        DynamicPathOnlinePath first = planner.PlanFromPoseContinuing(
            graph,
            track,
            poseNode.Position,
            poseNode.Heading,
            speedMetersPerSecond: 0.0f
        );
        DynamicPathNode reachedNode = graph.GetNode(first.NodePath[1]);

        DynamicPathOnlinePath second = planner.PlanFromPoseContinuing(
            graph,
            track,
            reachedNode.Position,
            reachedNode.Heading,
            speedMetersPerSecond: 0.0f
        );

        AssertThat(second.UsedPreviousPath).IsTrue();
        AssertThat(second.StartNode).IsEqual(first.NodePath[2]);
        AssertNodeSamplesAlignWithGraph(graph, second);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void ContinuingPlannerCutsAwaySamplesBehindCurrentTrackIndex()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlineConfig config = new()
        {
            MinimumPlanHorizonMeters = 80.0f,
            StartLayerLookahead = 0,
            CalculationTimeSafetyFactor = 0.0f
        };
        DynamicPathOnlinePathPlanner planner = new(config);

        DynamicPathOnlinePath first = planner.PlanFromPoseContinuing(
            graph,
            track,
            poseNode.Position,
            poseNode.Heading,
            speedMetersPerSecond: 0.0f
        );
        DynamicPathLayer passedLayer = graph.Layers[first.NodePath[1].Layer];
        int currentTrackIndex = (passedLayer.TrackIndex + 1) % track.Length;
        TrackPoint currentTrackPoint = track[currentTrackIndex];

        DynamicPathOnlinePath second = planner.PlanFromPoseContinuing(
            graph,
            track,
            currentTrackPoint.Center,
            currentTrackPoint.Tangent.Angle(),
            speedMetersPerSecond: 0.0f
        );

        AssertThat(second.UsedPreviousPath).IsTrue();
        AssertThat(second.AnchorTrackIndex).IsEqual(currentTrackIndex);
        AssertThat(second.StartNode).IsEqual(first.NodePath[2]);
        AssertThat(second.SampleTrackProgress[0]).IsGreaterEqual(-0.001f);
        AssertSampleProgressIsMonotonic(second);
        AssertNodeSamplesAlignWithGraph(graph, second);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FreshPlannerTreatsCurrentLayerIndexAsConsumed()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlineConfig config = new()
        {
            MinimumPlanHorizonMeters = 80.0f,
            StartLayerLookahead = 0
        };

        DynamicPathOnlinePath path = new DynamicPathOnlinePathPlanner(config).PlanFromPose(
            graph,
            track,
            poseNode.Position,
            poseNode.Heading
        );

        AssertThat(path.StartNode.Layer).IsEqual(1);
        AssertThat(path.AnchorTrackIndex).IsEqual(poseLayer.TrackIndex);
        AssertSampleProgressIsMonotonic(path);
        AssertNodeSamplesAlignWithGraph(graph, path);
    }


    [TestCase]
    [RequireGodotRuntime]
    public void ContinuingPlannerDoesNotReturnStaleFullBackup()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlinePathPlanner planner = new(new DynamicPathOnlineConfig
        {
            MinimumPlanHorizonMeters = 80.0f,
            StartLayerLookahead = 0
        });

        DynamicPathOnlinePath first = planner.PlanFromPoseContinuing(
            graph,
            track,
            poseNode.Position,
            poseNode.Heading,
            speedMetersPerSecond: 0.0f
        );
        bool threw = false;
        try
        {
            _ = planner.PlanFromPoseContinuing(
                graph,
                track,
                new Vector2(10000.0f, 10000.0f),
                poseNode.Heading,
                speedMetersPerSecond: 0.0f
            );
        }
        catch (InvalidOperationException)
        {
            threw = true;
        }

        AssertThat(first.Samples.Length).IsGreater(0);
        AssertThat(threw).IsTrue();
    }

    [TestCase]
    [RequireGodotRuntime]
    public void ContinuingPlannerReinitializesAfterPassingPreviousPath()
    {
        (TrackData track, DynamicPathOfflineGraph graph) = BuildGraph();
        DynamicPathLayer poseLayer = graph.Layers[0];
        DynamicPathNode poseNode = poseLayer.Nodes[poseLayer.RaceLineNodeIndex];
        DynamicPathOnlineConfig config = new()
        {
            PlanHorizonMode = DynamicPathPlanHorizonMode.Layers,
            MinimumPlanHorizonLayers = 4,
            StartLayerLookahead = 0,
            CalculationTimeSafetyFactor = 0.0f
        };
        DynamicPathOnlinePathPlanner planner = new(config);

        DynamicPathOnlinePath first = planner.PlanFromPoseContinuing(
            graph,
            track,
            poseNode.Position,
            poseNode.Heading,
            speedMetersPerSecond: 0.0f
        );
        int passedLayerIndex = (first.GoalLayer + 2) % graph.LayerCount;
        DynamicPathLayer passedLayer = graph.Layers[passedLayerIndex];
        DynamicPathNode passedNode = passedLayer.Nodes[passedLayer.RaceLineNodeIndex];

        DynamicPathOnlinePath second = planner.PlanFromPoseContinuing(
            graph,
            track,
            passedNode.Position,
            passedNode.Heading,
            speedMetersPerSecond: 0.0f
        );

        AssertThat(first.Samples.Length).IsGreater(0);
        AssertThat(second.UsedPreviousPath).IsFalse();
        AssertThat(second.InitialConnectorSpline).IsNotNull();
        AssertThat(second.ConstantPrefixSampleCount).IsEqual(0);
        AssertThat(second.AnchorTrackIndex).IsEqual(passedLayer.TrackIndex);
        AssertThat(second.StartNode.Layer).IsEqual((passedLayerIndex + 1) % graph.LayerCount);
        AssertThat(second.Samples[0].Position.X).IsEqualApprox(passedNode.Position.X, 0.001f);
        AssertThat(second.Samples[0].Position.Y).IsEqualApprox(passedNode.Position.Y, 0.001f);
        AssertPathSamplesAreFinite(second);
        AssertSampleProgressIsMonotonic(second);
        AssertNodeSamplesAlignWithGraph(graph, second);
    }

    private static (TrackData Track, DynamicPathOfflineGraph Graph) BuildGraph()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(width: 70.0f, height: 24.0f, trackWidth: 18.0f);
        RacingLine racingLine = CenterLineRacingLineSolver.Instance.Generate(track);
        CarConfig car = new();
        return (track, new DynamicPathOfflineGraphBuilder().Build(track, racingLine, car));
    }

    private static DynamicPathOfflineGraph BuildGraphWithIsolatedLateralNodes()
    {
        DynamicPathOfflineConfig config = new();
        DynamicPathLayer[] layers =
        [
            BuildTestLayer(index: 0, x: 0.0f),
            BuildTestLayer(index: 1, x: 10.0f),
            BuildTestLayer(index: 2, x: 20.0f)
        ];
        DynamicPathEdge[] edges =
        [
            BuildTestEdge(layers[0].Nodes[1], layers[1].Nodes[1]),
            BuildTestEdge(layers[1].Nodes[1], layers[2].Nodes[1]),
            BuildTestEdge(layers[2].Nodes[1], layers[0].Nodes[1])
        ];

        return new DynamicPathOfflineGraph(
            config,
            layers,
            edges,
            vehicleHalfWidthMeters: 1.4f,
            minTurnRadiusMeters: 7.0f
        );
    }

    private static DynamicPathLayer BuildTestLayer(int index, float x)
    {
        DynamicPathNode[] nodes =
        [
            new(new DynamicPathNodeId(index, 0), new Vector2(x, -1.0f), 0.0f, -1.0f, isRaceLine: false),
            new(new DynamicPathNodeId(index, 1), new Vector2(x, 0.0f), 0.0f, 0.0f, isRaceLine: true),
            new(new DynamicPathNodeId(index, 2), new Vector2(x, 1.0f), 0.0f, 1.0f, isRaceLine: false)
        ];
        return new DynamicPathLayer(index, trackIndex: index * 10, raceLineDistance: index * 10.0f, raceLineNodeIndex: 1, nodes);
    }

    private static DynamicPathEdge BuildTestEdge(DynamicPathNode from, DynamicPathNode to)
    {
        float length = from.Position.DistanceTo(to.Position);
        DynamicPathSplineSegment spline = new(
            from.Position,
            to.Position - from.Position,
            Vector2.Zero,
            Vector2.Zero
        );
        DynamicPathEdgeSample[] samples =
        [
            new(from.Position, from.Heading, Curvature: 0.0f, LengthToNext: length),
            new(to.Position, to.Heading, Curvature: 0.0f, LengthToNext: 0.0f)
        ];
        return new DynamicPathEdge(from.Id, to.Id, spline, samples, length, offlineCost: length, isRaceLineEdge: true);
    }

    private static void AssertPathSamplesAreFinite(DynamicPathOnlinePath path)
    {
        for (int i = 0; i < path.Samples.Length; i++)
        {
            DynamicPathEdgeSample sample = path.Samples[i];
            AssertThat(float.IsFinite(sample.Position.X)).IsTrue();
            AssertThat(float.IsFinite(sample.Position.Y)).IsTrue();
            AssertThat(float.IsFinite(sample.Heading)).IsTrue();
            AssertThat(float.IsFinite(sample.Curvature)).IsTrue();
            AssertThat(sample.LengthToNext).IsGreaterEqual(0.0f);
        }
    }

    private static void AssertNodeSamplesAlignWithGraph(DynamicPathOfflineGraph graph, DynamicPathOnlinePath path)
    {
        AssertThat(path.NodeSampleIndexes.Length).IsEqual(path.NodePath.Length);

        int previousSampleIndex = -1;
        for (int i = 0; i < path.NodePath.Length; i++)
        {
            int sampleIndex = path.NodeSampleIndexes[i];
            AssertThat(sampleIndex).IsBetween(0, path.Samples.Length - 1);
            AssertThat(sampleIndex).IsGreater(previousSampleIndex);
            previousSampleIndex = sampleIndex;

            DynamicPathNode node = graph.GetNode(path.NodePath[i]);
            DynamicPathEdgeSample sample = path.Samples[sampleIndex];
            AssertThat(sample.Position.X).IsEqualApprox(node.Position.X, 0.001f);
            AssertThat(sample.Position.Y).IsEqualApprox(node.Position.Y, 0.001f);
        }
    }

    private static void AssertSampleProgressIsMonotonic(DynamicPathOnlinePath path)
    {
        AssertThat(path.SampleTrackProgress.Length).IsEqual(path.Samples.Length);

        float previous = float.NegativeInfinity;
        for (int i = 0; i < path.SampleTrackProgress.Length; i++)
        {
            AssertThat(float.IsFinite(path.SampleTrackProgress[i])).IsTrue();
            AssertThat(path.SampleTrackProgress[i]).IsGreaterEqual(previous - 0.001f);
            previous = path.SampleTrackProgress[i];
        }
    }
}
