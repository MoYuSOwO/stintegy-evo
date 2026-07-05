using System.Collections.Generic;
using Godot;
using GdUnit4;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Track;
using static GdUnit4.Assertions;

namespace StintegyEVO.Tests.Controllers.V1.DynamicPath.Offline;

[TestSuite]
public sealed class GraphBasedLocalPlannerTests
{
    [TestCase]
    [RequireGodotRuntime]
    public void OfflineGraphBuildsSpatialLatticeWithoutVelocityProfile()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(width: 70.0f, height: 24.0f, trackWidth: 18.0f);
        RacingLine racingLine = CenterLineRacingLineSolver.Instance.Generate(track);
        CarConfig car = new();

        DynamicPathOfflineGraph graph = new DynamicPathOfflineGraphBuilder().Build(track, racingLine, car);

        for (int i = 0; i < racingLine.Count; i++)
            AssertThat(racingLine[i].TrackIndex).IsEqual(i);

        AssertThat(graph.LayerCount).IsGreater(4);
        AssertThat(graph.NodeCount).IsGreater(graph.LayerCount);
        AssertThat(graph.EdgeCount).IsGreater(graph.LayerCount);
        AssertThat(graph.VehicleHalfWidthMeters).IsEqualApprox(car.Chassis.Width * 0.5f, 0.0001f);
        AssertThat(graph.MinTurnRadiusMeters).IsEqualApprox(car.Chassis.WheelBase / Mathf.Tan(car.Chassis.MaxSteerAngle), 0.0001f);

        foreach (DynamicPathLayer layer in graph.Layers)
        {
            AssertThat(racingLine[layer.TrackIndex].TrackIndex).IsEqual(layer.TrackIndex);
            AssertThat(layer.RaceLineNodeIndex).IsBetween(0, layer.Nodes.Length - 1);
            AssertThat(layer.Nodes[layer.RaceLineNodeIndex].IsRaceLine).IsTrue();
            AssertThat(layer.Nodes[layer.RaceLineNodeIndex].Offset).IsEqualApprox(racingLine[layer.TrackIndex].Offset, 0.0001f);
        }

        foreach (DynamicPathEdge edge in graph.Edges)
        {
            AssertThat(edge.Length).IsGreater(0.0f);
            AssertThat(float.IsFinite(edge.OfflineCost)).IsTrue();
            AssertThat(edge.Samples.Length).IsGreater(1);
        }
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OfflineGraphKeepsNodesInsideVehicleAdjustedTrackBounds()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(width: 60.0f, height: 20.0f, trackWidth: 16.0f);
        RacingLine racingLine = CenterLineRacingLineSolver.Instance.Generate(track);
        CarConfig car = new();
        DynamicPathOfflineConfig config = new()
        {
            SafetyMarginMeters = 0.75f
        };

        DynamicPathOfflineGraph graph = new DynamicPathOfflineGraphBuilder().Build(track, racingLine, car, config);
        float usableHalfWidth = track[0].HalfWidth - car.Chassis.Width * 0.5f - config.SafetyMarginMeters;

        foreach (DynamicPathLayer layer in graph.Layers)
        {
            foreach (DynamicPathNode node in layer.Nodes)
            {
                AssertThat(node.Offset).IsGreaterEqual(-usableHalfWidth - 0.001f);
                AssertThat(node.Offset).IsLessEqual(usableHalfWidth + 0.001f);
            }
        }
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OfflineGraphClampsGenericRacingLineToVehicleAdjustedBounds()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(width: 60.0f, height: 20.0f, trackWidth: 16.0f);
        RacingLine centerLine = CenterLineRacingLineSolver.Instance.Generate(track);
        CarConfig car = new();
        DynamicPathOfflineConfig config = new();
        float usableHalfWidth = track[0].HalfWidth - car.Chassis.Width * 0.5f - config.SafetyMarginMeters;
        RacingLinePoint[] points = new RacingLinePoint[centerLine.Count];

        for (int i = 0; i < centerLine.Count; i++)
        {
            TrackPoint trackPoint = track[i];
            float offset = usableHalfWidth + 0.3f;
            points[i] = new RacingLinePoint(
                i,
                centerLine[i].Distance,
                offset,
                trackPoint.GetOffsetPos(offset),
                centerLine[i].Heading,
                centerLine[i].Curvature
            );
        }

        DynamicPathOfflineGraph graph = new DynamicPathOfflineGraphBuilder().Build(track, new RacingLine(points), car, config);

        foreach (DynamicPathLayer layer in graph.Layers)
        {
            DynamicPathNode raceReferenceNode = layer.Nodes[layer.RaceLineNodeIndex];
            AssertThat(raceReferenceNode.Offset).IsLessEqual(usableHalfWidth + 0.001f);
            AssertThat(raceReferenceNode.Offset).IsGreaterEqual(usableHalfWidth - config.LateralResolutionMeters - 0.001f);
        }
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OfflineGraphAddsVirtualGoalEdgesForEveryLayer()
    {
        TrackData track = TrackFactory.SimpleOvalTrack(width: 70.0f, height: 24.0f, trackWidth: 18.0f);
        RacingLine racingLine = CenterLineRacingLineSolver.Instance.Generate(track);
        CarConfig car = new();
        DynamicPathOfflineConfig config = new();

        DynamicPathOfflineGraph graph = new DynamicPathOfflineGraphBuilder().Build(track, racingLine, car, config);

        foreach (DynamicPathLayer layer in graph.Layers)
        {
            IReadOnlyList<DynamicPathVirtualGoalEdge> virtualEdges = graph.GetVirtualGoalEdges(layer.Index);

            AssertThat(virtualEdges.Count).IsEqual(layer.Nodes.Length);
            AssertThat(virtualEdges[layer.RaceLineNodeIndex].OfflineCost).IsEqualApprox(0.0f, 0.0001f);

            for (int nodeIndex = 0; nodeIndex < layer.Nodes.Length; nodeIndex++)
            {
                DynamicPathVirtualGoalEdge virtualEdge = virtualEdges[nodeIndex];
                float expectedCost = Mathf.Abs(nodeIndex - layer.RaceLineNodeIndex)
                    * config.LateralResolutionMeters
                    * config.CostWeights.VirtualGoal;

                AssertThat(virtualEdge.From).IsEqual(layer.Nodes[nodeIndex].Id);
                AssertThat(virtualEdge.GoalLayer).IsEqual(layer.Index);
                AssertThat(virtualEdge.OfflineCost).IsEqualApprox(expectedCost, 0.0001f);
            }
        }
    }
}
