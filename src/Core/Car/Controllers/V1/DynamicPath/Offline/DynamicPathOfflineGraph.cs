using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath;

namespace StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;

public readonly record struct DynamicPathNodeId(int Layer, int Node);

public sealed class DynamicPathNode
{
    public readonly DynamicPathNodeId Id;
    public readonly Vector2 Position;
    public readonly float Heading;
    public readonly float Offset;
    public readonly bool IsRaceLine;

    public DynamicPathNode(DynamicPathNodeId id, Vector2 position, float heading, float offset, bool isRaceLine)
    {
        Id = id;
        Position = position;
        Heading = heading;
        Offset = offset;
        IsRaceLine = isRaceLine;
    }
}

public sealed class DynamicPathLayer
{
    public readonly int Index;
    public readonly int TrackIndex;
    public readonly float RaceLineDistance;
    public readonly int RaceLineNodeIndex;
    public readonly DynamicPathNode[] Nodes;

    public DynamicPathLayer(int index, int trackIndex, float raceLineDistance, int raceLineNodeIndex, DynamicPathNode[] nodes)
    {
        Index = index;
        TrackIndex = trackIndex;
        RaceLineDistance = raceLineDistance;
        RaceLineNodeIndex = raceLineNodeIndex;
        Nodes = nodes;
    }
}

public sealed class DynamicPathEdge
{
    public readonly DynamicPathNodeId From;
    public readonly DynamicPathNodeId To;
    public readonly DynamicPathSplineSegment Spline;
    public readonly DynamicPathEdgeSample[] Samples;
    public readonly float Length;
    public readonly float OfflineCost;
    public readonly bool IsRaceLineEdge;

    public DynamicPathEdge(
        DynamicPathNodeId from,
        DynamicPathNodeId to,
        DynamicPathSplineSegment spline,
        DynamicPathEdgeSample[] samples,
        float length,
        float offlineCost,
        bool isRaceLineEdge
    )
    {
        From = from;
        To = to;
        Spline = spline;
        Samples = samples;
        Length = length;
        OfflineCost = offlineCost;
        IsRaceLineEdge = isRaceLineEdge;
    }
}

public readonly record struct DynamicPathVirtualGoalEdge(
    DynamicPathNodeId From,
    int GoalLayer,
    float OfflineCost
);

public sealed class DynamicPathOfflineGraph
{
    private readonly Dictionary<long, DynamicPathEdge[]> _outgoingEdges;
    private readonly DynamicPathVirtualGoalEdge[][] _virtualGoalEdges;
    private readonly int[] _layerStarts;

    public readonly DynamicPathOfflineConfig Config;
    public readonly DynamicPathLayer[] Layers;
    public readonly DynamicPathEdge[] Edges;
    public readonly float VehicleHalfWidthMeters;
    public readonly float MinTurnRadiusMeters;
    public readonly bool LayerTrackIndexesAreMonotonic;

    public int NodeCount { get; }
    public int LayerCount => Layers.Length;
    public int EdgeCount => Edges.Length;

    public DynamicPathOfflineGraph(
        DynamicPathOfflineConfig config,
        DynamicPathLayer[] layers,
        DynamicPathEdge[] edges,
        float vehicleHalfWidthMeters,
        float minTurnRadiusMeters
    )
    {
        Config = config;
        Layers = layers;
        Edges = edges;
        VehicleHalfWidthMeters = vehicleHalfWidthMeters;
        MinTurnRadiusMeters = minTurnRadiusMeters;

        _layerStarts = new int[layers.Length];
        int nodeCount = 0;
        for (int i = 0; i < layers.Length; i++)
        {
            _layerStarts[i] = nodeCount;
            nodeCount += layers[i].Nodes.Length;
        }
        NodeCount = nodeCount;

        bool monotonicTrackIndexes = true;
        for (int i = 1; i < layers.Length; i++)
        {
            if (layers[i].TrackIndex < layers[i - 1].TrackIndex)
            {
                monotonicTrackIndexes = false;
                break;
            }
        }
        LayerTrackIndexesAreMonotonic = monotonicTrackIndexes;

        Dictionary<long, List<DynamicPathEdge>> outgoing = [];
        for (int i = 0; i < edges.Length; i++)
        {
            long key = EdgeKey(edges[i].From);
            if (!outgoing.TryGetValue(key, out List<DynamicPathEdge>? edgeList))
            {
                edgeList = [];
                outgoing[key] = edgeList;
            }
            edgeList.Add(edges[i]);
        }

        _outgoingEdges = [];
        foreach (KeyValuePair<long, List<DynamicPathEdge>> pair in outgoing)
            _outgoingEdges[pair.Key] = [.. pair.Value];

        _virtualGoalEdges = new DynamicPathVirtualGoalEdge[layers.Length][];
        for (int layerIndex = 0; layerIndex < layers.Length; layerIndex++)
        {
            DynamicPathLayer layer = layers[layerIndex];
            DynamicPathVirtualGoalEdge[] virtualEdges = new DynamicPathVirtualGoalEdge[layer.Nodes.Length];
            for (int nodeIndex = 0; nodeIndex < layer.Nodes.Length; nodeIndex++)
            {
                float lateralDistance = Math.Abs(nodeIndex - layer.RaceLineNodeIndex) * config.LateralResolutionMeters;
                virtualEdges[nodeIndex] = new DynamicPathVirtualGoalEdge(
                    From: layer.Nodes[nodeIndex].Id,
                    GoalLayer: layerIndex,
                    OfflineCost: lateralDistance * config.CostWeights.VirtualGoal
                );
            }

            _virtualGoalEdges[layerIndex] = virtualEdges;
        }
    }

    public DynamicPathNode GetNode(DynamicPathNodeId id)
    {
        if ((uint)id.Layer >= (uint)Layers.Length)
            throw new ArgumentOutOfRangeException(nameof(id));
        DynamicPathNode[] nodes = Layers[id.Layer].Nodes;
        if ((uint)id.Node >= (uint)nodes.Length)
            throw new ArgumentOutOfRangeException(nameof(id));
        return nodes[id.Node];
    }

    public IReadOnlyList<DynamicPathEdge> GetOutgoingEdges(DynamicPathNodeId id)
    {
        return _outgoingEdges.TryGetValue(EdgeKey(id), out DynamicPathEdge[]? edges)
            ? edges
            : Array.Empty<DynamicPathEdge>();
    }

    public IReadOnlyList<DynamicPathVirtualGoalEdge> GetVirtualGoalEdges(int goalLayer)
    {
        if ((uint)goalLayer >= (uint)_virtualGoalEdges.Length)
            throw new ArgumentOutOfRangeException(nameof(goalLayer));
        return _virtualGoalEdges[goalLayer];
    }

    public int GetGlobalNodeIndex(DynamicPathNodeId id)
    {
        if ((uint)id.Layer >= (uint)_layerStarts.Length)
            throw new ArgumentOutOfRangeException(nameof(id));
        return GetGlobalNodeIndexUnchecked(id);
    }

    internal int GetGlobalNodeIndexUnchecked(DynamicPathNodeId id)
    {
        return _layerStarts[id.Layer] + id.Node;
    }

    private static long EdgeKey(DynamicPathNodeId id)
    {
        return ((long)id.Layer << 32) | (uint)id.Node;
    }
}
