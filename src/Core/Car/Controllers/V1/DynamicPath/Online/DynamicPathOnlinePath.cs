using System;
using Godot;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;

namespace StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Online;

public sealed class DynamicPathOnlinePath
{
    public readonly DynamicPathNodeId StartNode;
    public readonly DynamicPathNodeId GoalNode;
    public readonly int RequestedGoalLayer;
    public readonly int GoalLayer;
    public readonly int AnchorTrackIndex;
    public readonly bool HorizonReduced;
    public readonly bool HeadingCompatible;
    public readonly DynamicPathNodeId[] NodePath;
    public readonly DynamicPathEdge[] EdgePath;
    public readonly int[] NodeSampleIndexes;
    public readonly DynamicPathSplineSegment? InitialConnectorSpline;
    public readonly DynamicPathSplineSegment[] SmoothedSplines;
    public readonly DynamicPathEdgeSample[] Samples;
    public readonly float[] SampleTrackProgress;
    public readonly float[] NodeTrackProgress;
    public readonly float SearchCost;
    public readonly float VirtualGoalCost;
    public readonly float PhysicalLength;
    public readonly bool UsedPreviousPath;
    public readonly int ConstantPrefixSampleCount;

    public DynamicPathOnlinePath(
        DynamicPathNodeId startNode,
        DynamicPathNodeId goalNode,
        int requestedGoalLayer,
        int goalLayer,
        int anchorTrackIndex,
        bool horizonReduced,
        bool headingCompatible,
        DynamicPathNodeId[] nodePath,
        DynamicPathEdge[] edgePath,
        int[] nodeSampleIndexes,
        DynamicPathSplineSegment? initialConnectorSpline,
        DynamicPathSplineSegment[] smoothedSplines,
        DynamicPathEdgeSample[] samples,
        float[]? sampleTrackProgress,
        float searchCost,
        float virtualGoalCost,
        float physicalLength,
        bool usedPreviousPath,
        int constantPrefixSampleCount
    )
    {
        StartNode = startNode;
        GoalNode = goalNode;
        RequestedGoalLayer = requestedGoalLayer;
        GoalLayer = goalLayer;
        AnchorTrackIndex = anchorTrackIndex;
        HorizonReduced = horizonReduced;
        HeadingCompatible = headingCompatible;
        NodePath = nodePath;
        EdgePath = edgePath;
        NodeSampleIndexes = nodeSampleIndexes;
        InitialConnectorSpline = initialConnectorSpline;
        SmoothedSplines = smoothedSplines;
        Samples = samples;
        SampleTrackProgress = sampleTrackProgress ?? Array.Empty<float>();
        NodeTrackProgress = BuildNodeTrackProgress(NodeSampleIndexes, SampleTrackProgress);
        SearchCost = searchCost;
        VirtualGoalCost = virtualGoalCost;
        PhysicalLength = physicalLength;
        UsedPreviousPath = usedPreviousPath;
        ConstantPrefixSampleCount = constantPrefixSampleCount;
    }

    public Vector2 StartPosition => Samples.Length > 0 ? Samples[0].Position : Vector2.Zero;
    public Vector2 EndPosition => Samples.Length > 0 ? Samples[^1].Position : Vector2.Zero;

    private static float[] BuildNodeTrackProgress(int[] nodeSampleIndexes, float[] sampleTrackProgress)
    {
        float[] nodeProgress = new float[nodeSampleIndexes.Length];
        if (sampleTrackProgress.Length == 0)
            return nodeProgress;

        for (int i = 0; i < nodeSampleIndexes.Length; i++)
        {
            int sampleIndex = Math.Clamp(nodeSampleIndexes[i], 0, sampleTrackProgress.Length - 1);
            nodeProgress[i] = sampleTrackProgress[sampleIndex];
        }

        return nodeProgress;
    }
}
