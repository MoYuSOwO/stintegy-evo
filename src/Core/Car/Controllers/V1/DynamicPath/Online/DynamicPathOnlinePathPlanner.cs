using System;
using System.Collections.Generic;
using System.Diagnostics;
using Godot;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;
using StintegyEVO.Core.Track;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Online;

public sealed class DynamicPathOnlinePathPlanner
{
    private readonly DynamicPathOnlineConfig _config;
    private readonly Queue<float> _planningIntervals = [];
    private DynamicPathOnlinePath? _lastPath;
    private DynamicPathOnlinePath? _backupPath;
    private SearchScratch? _searchScratch;
    private long _lastPlanningTimestamp;
    private bool _hasLastPlanningTimestamp;

    public DynamicPathOnlinePathPlanner(DynamicPathOnlineConfig? config = null)
    {
        _config = config ?? new DynamicPathOnlineConfig();
        _config.Validate();
    }

    public void ResetMemory()
    {
        _lastPath = null;
        _backupPath = null;
        _planningIntervals.Clear();
        _lastPlanningTimestamp = 0;
        _hasLastPlanningTimestamp = false;
    }

    public DynamicPathOnlinePath PlanFromPose(
        DynamicPathOfflineGraph graph,
        Vector2 position,
        float heading
    )
    {
        ArgumentNullException.ThrowIfNull(graph);
        return PlanFreshFromPose(graph, track: null, position, heading);
    }

    public DynamicPathOnlinePath PlanFromPose(
        DynamicPathOfflineGraph graph,
        TrackData track,
        Vector2 position,
        float heading
    )
    {
        ArgumentNullException.ThrowIfNull(graph);
        ArgumentNullException.ThrowIfNull(track);
        return PlanFreshFromPose(graph, track, position, heading);
    }

    public DynamicPathOnlinePath PlanFromPoseContinuing(
        DynamicPathOfflineGraph graph,
        Vector2 position,
        float heading,
        float speedMetersPerSecond
    )
    {
        ArgumentNullException.ThrowIfNull(graph);
        return PlanFromPoseContinuingCore(graph, track: null, position, heading, speedMetersPerSecond);
    }

    public DynamicPathOnlinePath PlanFromPoseContinuing(
        DynamicPathOfflineGraph graph,
        TrackData track,
        Vector2 position,
        float heading,
        float speedMetersPerSecond
    )
    {
        ArgumentNullException.ThrowIfNull(track);
        return PlanFromPoseContinuingCore(graph, track, position, heading, speedMetersPerSecond);
    }

    private DynamicPathOnlinePath PlanFromPoseContinuingCore(
        DynamicPathOfflineGraph graph,
        TrackData? track,
        Vector2 position,
        float heading,
        float speedMetersPerSecond
    )
    {
        ArgumentNullException.ThrowIfNull(graph);

        float compensationSeconds = BeginPlanningIteration();
        try
        {
            if (_lastPath != null)
            {
                if (TryPlanFromPreviousPath(
                        graph,
                        track,
                        position,
                        heading,
                        speedMetersPerSecond,
                        compensationSeconds,
                        out DynamicPathOnlinePath inheritedPath
                    ))
                {
                    StoreSuccessfulPath(inheritedPath);
                    return inheritedPath;
                }

                DynamicPathOnlinePath fallbackPath = PlanFreshFromPose(graph, track, position, heading);
                StoreSuccessfulPath(fallbackPath);
                return fallbackPath;
            }

            DynamicPathOnlinePath freshPath = PlanFreshFromPose(graph, track, position, heading);
            StoreSuccessfulPath(freshPath);
            return freshPath;
        }
        catch (Exception)
        {
            if (TryCreateBackupPath(graph, track, position, heading, out DynamicPathOnlinePath backupPath))
                return backupPath;

            throw;
        }
    }

    public DynamicPathOnlinePath PlanFromNode(
        DynamicPathOfflineGraph graph,
        DynamicPathNodeId startNode
    )
    {
        ArgumentNullException.ThrowIfNull(graph);
        graph.GetNode(startNode);
        return PlanFromNode(
            graph,
            startNode,
            headingCompatible: true,
            initialConnectorSpline: null,
            prefixSamples: null,
            prefixSampleTrackProgress: null,
            anchorTrackIndex: graph.Layers[startNode.Layer].TrackIndex,
            trackLength: 0,
            preferredEdgeCostFactors: null,
            usedPreviousPath: false,
            constantPrefixSampleCount: 0
        );
    }

    private DynamicPathOnlinePath PlanFreshFromPose(
        DynamicPathOfflineGraph graph,
        TrackData? track,
        Vector2 position,
        float heading
    )
    {
        DynamicPathNodeId startNode;
        int anchorTrackIndex;
        float poseOffset;
        if (track != null)
        {
            startNode = SelectTrackProgressStartNode(graph, track, position, out anchorTrackIndex, out poseOffset);
        }
        else
        {
            DynamicPathNodeId closestNode = FindClosestNode(graph, position, out poseOffset);
            int startLayer = WrapLayer(closestNode.Layer + _config.StartLayerLookahead, graph.LayerCount);
            startNode = SelectLookaheadStartNode(graph, closestNode, startLayer);
            anchorTrackIndex = graph.Layers[closestNode.Layer].TrackIndex;
        }

        if (poseOffset > _config.MaxPoseOffsetMeters)
            throw new InvalidOperationException("Pose is too far outside the track boundary to select a reliable start node.");

        DynamicPathNode start = graph.GetNode(startNode);
        bool headingCompatible = IsHeadingCompatible(heading, start.Heading);

        DynamicPathSplineSegment connectorSpline = DynamicPathSplineMath.CreateUnclosedSegment(
            position,
            start.Position,
            heading,
            start.Heading
        );
        DynamicPathEdgeSample[] connectorSamples = SampleSplines(
            [connectorSpline],
            graph.Config.EdgeSampleStepMeters
        );
        float startNodeProgress = track != null
            ? ForwardTrackDistance(anchorTrackIndex, graph.Layers[startNode.Layer].TrackIndex, track.Length)
            : CalculatePhysicalLength(connectorSamples);
        float[] connectorSampleProgress = BuildSampleProgressByLength(connectorSamples, 0.0f, startNodeProgress);

        return PlanFromNode(
            graph,
            startNode,
            headingCompatible,
            connectorSpline,
            connectorSamples,
            connectorSampleProgress,
            anchorTrackIndex,
            track?.Length ?? 0,
            preferredEdgeCostFactors: null,
            usedPreviousPath: false,
            constantPrefixSampleCount: 0
        );
    }

    private bool TryPlanFromPreviousPath(
        DynamicPathOfflineGraph graph,
        TrackData? track,
        Vector2 position,
        float heading,
        float speedMetersPerSecond,
        float compensationSeconds,
        out DynamicPathOnlinePath path
    )
    {
        path = null!;

        DynamicPathOnlinePath? previousPath = _lastPath;
        if (previousPath == null || previousPath.Samples.Length < 2 || previousPath.NodePath.Length < 2)
            return false;
        if (previousPath.NodeSampleIndexes.Length != previousPath.NodePath.Length)
            return false;
        if (previousPath.SampleTrackProgress.Length != previousPath.Samples.Length)
            return false;

        int startNodePathIndex;
        int anchorTrackIndex;
        PrefixSegment inheritedPrefix;
        PathProjection projection;
        int currentTrackIndex = -1;
        float currentTrackProgress = 0.0f;
        if (track != null && previousPath.AnchorTrackIndex >= 0)
        {
            currentTrackIndex = track.FindNearestIndex(position);
            currentTrackProgress = ForwardTrackDistance(previousPath.AnchorTrackIndex, currentTrackIndex, track.Length);
            if (!TryProjectOntoPathProgressNear(
                    previousPath.Samples,
                    previousPath.SampleTrackProgress,
                    position,
                    currentTrackProgress,
                    CalculateProgressProjectionWindow(graph),
                    out projection
                ))
            {
                return false;
            }
        }
        else if (!TryProjectOntoPathProgress(
                     previousPath.Samples,
                     previousPath.SampleTrackProgress,
                     position,
                     out projection
                 ))
        {
            return false;
        }

        if (track == null && projection.Distance > CalculateMaximumInheritedPathOffset(graph))
            return false;

        float delayDistance = MathF.Max(0.0f, speedMetersPerSecond) * compensationSeconds;
        if (!TryAdvanceProjectionByDistance(
                previousPath.Samples,
                previousPath.SampleTrackProgress,
                projection,
                delayDistance,
                out int delayedSampleIndex,
                out float delayedProgress
            ))
        {
            return false;
        }

        if (track != null && previousPath.AnchorTrackIndex >= 0)
        {
            float[] nodeProgress = previousPath.NodeTrackProgress;
            float poseOffset = CalculateTrackBoundaryExcess(track[WrapIndex(currentTrackIndex, track.Length)], position);
            if (poseOffset > _config.MaxPoseOffsetMeters)
                return false;

            if (currentTrackProgress > nodeProgress[^1] + _config.MaxPoseOffsetMeters)
                return false;

            startNodePathIndex = FindFirstUneatenNodeAfterProgress(nodeProgress, delayedProgress);
            if (startNodePathIndex < 0)
                return false;

            int startNodeSampleIndex = previousPath.NodeSampleIndexes[startNodePathIndex];
            inheritedPrefix = BuildInheritedPrefix(
                previousPath.Samples,
                previousPath.SampleTrackProgress,
                projection,
                startNodeSampleIndex
            );
            anchorTrackIndex = currentTrackIndex;
        }
        else
        {
            startNodePathIndex = FindFirstNodeAfterSample(previousPath.NodeSampleIndexes, delayedSampleIndex);
            if (startNodePathIndex < 0)
                return false;

            int startNodeSampleIndex = previousPath.NodeSampleIndexes[startNodePathIndex];
            inheritedPrefix = BuildInheritedPrefix(
                previousPath.Samples,
                previousPath.SampleTrackProgress,
                projection,
                startNodeSampleIndex
            );
            anchorTrackIndex = previousPath.AnchorTrackIndex;
        }

        if (startNodePathIndex < 0 || startNodePathIndex >= previousPath.NodePath.Length)
            return false;

        DynamicPathNodeId startNode = previousPath.NodePath[startNodePathIndex];
        DynamicPathNode graphStart = graph.GetNode(startNode);

        PreferredEdgeCostFactor[]? previousEdgeCostFactors =
            BuildPreviousEdgeCostFactors(previousPath, startNodePathIndex);

        path = PlanFromNode(
            graph,
            startNode,
            IsHeadingCompatible(heading, graphStart.Heading),
            initialConnectorSpline: null,
            prefixSamples: inheritedPrefix.Samples,
            prefixSampleTrackProgress: inheritedPrefix.SampleTrackProgress,
            anchorTrackIndex,
            track?.Length ?? 0,
            preferredEdgeCostFactors: previousEdgeCostFactors,
            usedPreviousPath: true,
            constantPrefixSampleCount: inheritedPrefix.Samples.Length
        );
        return true;
    }

    private DynamicPathOnlinePath PlanFromNode(
        DynamicPathOfflineGraph graph,
        DynamicPathNodeId startNode,
        bool headingCompatible,
        DynamicPathSplineSegment? initialConnectorSpline,
        DynamicPathEdgeSample[]? prefixSamples,
        float[]? prefixSampleTrackProgress,
        int anchorTrackIndex,
        int trackLength,
        PreferredEdgeCostFactor[]? preferredEdgeCostFactors,
        bool usedPreviousPath,
        int constantPrefixSampleCount
    )
    {
        int requestedGoalLayer = SelectGoalLayer(graph, startNode.Layer);
        int goalLayer = requestedGoalLayer;

        SearchResult? search = null;
        for (int attempts = 0; attempts < graph.LayerCount - 1; attempts++)
        {
            if (goalLayer == startNode.Layer)
                break;

            search = SearchToGoalLayer(graph, startNode, goalLayer, preferredEdgeCostFactors);
            if (search is not null)
                break;

            goalLayer = PreviousLayer(goalLayer, graph.LayerCount);
        }

        if (search is null)
            throw new InvalidOperationException("No feasible single-vehicle path was found in the offline graph.");

        float? startHeadingOverride = prefixSamples is { Length: > 0 }
            ? prefixSamples[^1].Heading
            : null;
        DynamicPathSplineSegment[] smoothedSplines = CreateSmoothedSplines(
            graph,
            search.NodePath,
            search.EdgePath,
            startHeadingOverride
        );
        SampledPath graphSampledPath = SamplePathSplines(smoothedSplines, graph.Config.EdgeSampleStepMeters);
        float[] graphNodeTrackProgress = trackLength > 0
            ? CalculateNodeTrackProgress(graph, search.NodePath, anchorTrackIndex, trackLength)
            : CalculateNodePhysicalProgress(graphSampledPath.Samples, graphSampledPath.NodeSampleIndexes);
        float[] graphSampleTrackProgress = BuildSampleProgressFromNodes(
            graphSampledPath.Samples,
            graphSampledPath.NodeSampleIndexes,
            graphNodeTrackProgress
        );
        DynamicPathEdgeSample[] samples = prefixSamples is null
            ? graphSampledPath.Samples
            : CombineConnectorAndGraphSamples(prefixSamples, graphSampledPath.Samples);
        float[] sampleTrackProgress = prefixSamples is null
            ? graphSampleTrackProgress
            : CombineConnectorAndGraphProgress(prefixSampleTrackProgress, graphSampleTrackProgress);
        int sampleIndexOffset = prefixSamples is null ? 0 : Math.Max(0, prefixSamples.Length - 1);
        int[] nodeSampleIndexes = OffsetNodeSampleIndexes(graphSampledPath.NodeSampleIndexes, sampleIndexOffset);
        float physicalLength = CalculatePhysicalLength(samples);

        return new DynamicPathOnlinePath(
            startNode: startNode,
            goalNode: search.GoalNode,
            requestedGoalLayer: requestedGoalLayer,
            goalLayer: goalLayer,
            anchorTrackIndex: anchorTrackIndex,
            horizonReduced: goalLayer != requestedGoalLayer,
            headingCompatible: headingCompatible,
            nodePath: search.NodePath,
            edgePath: search.EdgePath,
            nodeSampleIndexes: nodeSampleIndexes,
            initialConnectorSpline: initialConnectorSpline,
            smoothedSplines: smoothedSplines,
            samples: samples,
            sampleTrackProgress: sampleTrackProgress,
            searchCost: search.SearchCost,
            virtualGoalCost: search.VirtualGoalCost,
            physicalLength: physicalLength,
            usedPreviousPath: usedPreviousPath,
            constantPrefixSampleCount: constantPrefixSampleCount
        );
    }

    private void StoreSuccessfulPath(DynamicPathOnlinePath path)
    {
        _lastPath = path;
        _backupPath = path;
    }

    private static float CalculateMaximumInheritedPathOffset(DynamicPathOfflineGraph graph)
    {
        float vehicleWidth = graph.VehicleHalfWidthMeters * 2.0f;
        float graphResolutionBand = graph.Config.LateralResolutionMeters * 2.0f;
        return MathF.Max(vehicleWidth, graphResolutionBand);
    }

    private static float CalculateProgressProjectionWindow(DynamicPathOfflineGraph graph)
    {
        float layerStep = MathF.Max(
            graph.Config.LongitudinalStraightStepMeters,
            graph.Config.LongitudinalCurveStepMeters
        );
        return MathF.Max(layerStep + graph.Config.EdgeSampleStepMeters * 2.0f, 1e-4f);
    }

    private bool TryCreateBackupPath(
        DynamicPathOfflineGraph graph,
        TrackData? track,
        Vector2 position,
        float heading,
        out DynamicPathOnlinePath path
    )
    {
        path = null!;
        DynamicPathOnlinePath? backupPath = _backupPath;
        if (track == null || backupPath == null || backupPath.AnchorTrackIndex < 0)
            return false;
        if (backupPath.Samples.Length < 2 || backupPath.NodePath.Length < 2)
            return false;
        if (backupPath.NodeSampleIndexes.Length != backupPath.NodePath.Length)
            return false;
        if (backupPath.SampleTrackProgress.Length != backupPath.Samples.Length)
            return false;

        int currentTrackIndex = track.FindNearestIndex(position);
        float currentTrackProgress = ForwardTrackDistance(backupPath.AnchorTrackIndex, currentTrackIndex, track.Length);
        if (!TryProjectOntoPathProgressNear(
                backupPath.Samples,
                backupPath.SampleTrackProgress,
                position,
                currentTrackProgress,
                CalculateProgressProjectionWindow(graph),
                out PathProjection projection
            ))
        {
            return false;
        }

        float[] nodeProgress = backupPath.NodeTrackProgress;
        float poseOffset = CalculateTrackBoundaryExcess(track[WrapIndex(currentTrackIndex, track.Length)], position);
        if (poseOffset > _config.MaxPoseOffsetMeters)
            return false;
        if (currentTrackProgress > nodeProgress[^1] + _config.MaxPoseOffsetMeters)
            return false;

        int startNodePathIndex = FindFirstUneatenNodeAfterProgress(nodeProgress, projection.Progress);
        if (startNodePathIndex < 0 || startNodePathIndex >= backupPath.NodePath.Length)
            return false;

        int startNodeSampleIndex = backupPath.NodeSampleIndexes[startNodePathIndex];
        DynamicPathNodeId startNodeId = backupPath.NodePath[startNodePathIndex];
        DynamicPathNode startNode = graph.GetNode(startNodeId);
        PrefixSegment prefix = BuildInheritedPrefix(
            backupPath.Samples,
            backupPath.SampleTrackProgress,
            projection,
            startNodeSampleIndex
        );

        int nodeCount = backupPath.NodePath.Length - startNodePathIndex;
        DynamicPathNodeId[] nodePath = new DynamicPathNodeId[nodeCount];
        Array.Copy(backupPath.NodePath, startNodePathIndex, nodePath, 0, nodeCount);

        int edgeCount = Math.Max(0, backupPath.EdgePath.Length - startNodePathIndex);
        DynamicPathEdge[] edgePath = new DynamicPathEdge[edgeCount];
        if (edgeCount > 0)
            Array.Copy(backupPath.EdgePath, startNodePathIndex, edgePath, 0, edgeCount);

        DynamicPathSplineSegment[] splines = new DynamicPathSplineSegment[edgeCount];
        if (edgeCount > 0)
            Array.Copy(backupPath.SmoothedSplines, startNodePathIndex, splines, 0, edgeCount);

        DynamicPathEdgeSample[] samples = CopyBackupSamplesFromNode(
            backupPath.Samples,
            prefix.Samples,
            startNodeSampleIndex
        );
        float[] sampleTrackProgress = CopyBackupSampleProgressFromNode(
            backupPath.SampleTrackProgress,
            prefix.SampleTrackProgress,
            startNodeSampleIndex,
            projection.Progress
        );
        int[] nodeSampleIndexes = CopyBackupNodeSampleIndexes(
            backupPath.NodeSampleIndexes,
            startNodePathIndex,
            startNodeSampleIndex,
            prefix.Samples.Length
        );

        path = new DynamicPathOnlinePath(
            startNode: startNodeId,
            goalNode: nodePath[^1],
            requestedGoalLayer: backupPath.RequestedGoalLayer,
            goalLayer: backupPath.GoalLayer,
            anchorTrackIndex: currentTrackIndex,
            horizonReduced: backupPath.HorizonReduced,
            headingCompatible: IsHeadingCompatible(heading, startNode.Heading),
            nodePath: nodePath,
            edgePath: edgePath,
            nodeSampleIndexes: nodeSampleIndexes,
            initialConnectorSpline: null,
            smoothedSplines: splines,
            samples: samples,
            sampleTrackProgress: sampleTrackProgress,
            searchCost: backupPath.SearchCost,
            virtualGoalCost: backupPath.VirtualGoalCost,
            physicalLength: CalculatePhysicalLength(samples),
            usedPreviousPath: true,
            constantPrefixSampleCount: prefix.Samples.Length
        );
        StoreSuccessfulPath(path);
        return true;
    }

    private float BeginPlanningIteration()
    {
        long timestamp = Stopwatch.GetTimestamp();
        if (!_hasLastPlanningTimestamp)
        {
            _lastPlanningTimestamp = timestamp;
            _hasLastPlanningTimestamp = true;
            return 0.0f;
        }

        float intervalSeconds = (float)((timestamp - _lastPlanningTimestamp) / (double)Stopwatch.Frequency);
        _lastPlanningTimestamp = timestamp;
        RememberPlanningInterval(intervalSeconds);
        return CalculateCompensationSeconds();
    }

    private void RememberPlanningInterval(float intervalSeconds)
    {
        if (!float.IsFinite(intervalSeconds) || intervalSeconds < 0.0f)
            return;

        _planningIntervals.Enqueue(intervalSeconds);
        while (_planningIntervals.Count > _config.CalculationTimeBufferLength)
            _planningIntervals.Dequeue();
    }

    private float CalculateCompensationSeconds()
    {
        if (_planningIntervals.Count == 0)
            return 0.0f;

        float sum = 0.0f;
        foreach (float interval in _planningIntervals)
            sum += interval;

        float average = sum / _planningIntervals.Count;
        float compensated = average * _config.CalculationTimeSafetyFactor;
        return MathF.Min(compensated, _config.MaxConstantPrefixSeconds);
    }

    private static DynamicPathNodeId SelectLookaheadStartNode(
        DynamicPathOfflineGraph graph,
        DynamicPathNodeId closestNode,
        int startLayer
    )
    {
        DynamicPathLayer closestLayer = graph.Layers[closestNode.Layer];
        DynamicPathLayer targetLayer = graph.Layers[startLayer];
        float targetOffset = closestLayer.Nodes[closestNode.Node].Offset;
        return SelectClosestReachableOffsetNode(graph, targetLayer, targetOffset);
    }

    private DynamicPathNodeId SelectTrackProgressStartNode(
        DynamicPathOfflineGraph graph,
        TrackData track,
        Vector2 position,
        out int anchorTrackIndex,
        out float poseOffset
    )
    {
        anchorTrackIndex = track.FindNearestIndex(position);
        TrackPoint anchorPoint = track[anchorTrackIndex];
        float lateralOffset = (position - anchorPoint.Center).Dot(anchorPoint.Normal);
        poseOffset = CalculateTrackBoundaryExcess(anchorPoint, lateralOffset);

        int baseLayer = FindFirstLayerAtOrAfterTrackIndex(
            graph,
            anchorTrackIndex,
            track.Length,
            out float distanceToBaseLayer
        );
        int layerLookahead = distanceToBaseLayer <= 1e-4f
            ? Math.Max(1, _config.StartLayerLookahead)
            : _config.StartLayerLookahead;
        int startLayer = WrapLayer(baseLayer + layerLookahead, graph.LayerCount);
        return SelectClosestReachableOffsetNode(graph, graph.Layers[startLayer], lateralOffset);
    }

    private static int FindFirstLayerAtOrAfterTrackIndex(
        DynamicPathOfflineGraph graph,
        int trackIndex,
        int trackLength,
        out float distanceToLayer
    )
    {
        if (graph.LayerTrackIndexesAreMonotonic)
        {
            int normalizedTrackIndex = WrapIndex(trackIndex, trackLength);
            int low = 0;
            int high = graph.LayerCount;
            while (low < high)
            {
                int mid = low + (high - low) / 2;
                if (graph.Layers[mid].TrackIndex < normalizedTrackIndex)
                    low = mid + 1;
                else
                    high = mid;
            }

            int layer = low < graph.LayerCount ? low : 0;
            distanceToLayer = ForwardTrackDistance(normalizedTrackIndex, graph.Layers[layer].TrackIndex, trackLength);
            return layer;
        }

        int bestLayer = 0;
        float bestDistance = float.PositiveInfinity;

        for (int layerIndex = 0; layerIndex < graph.LayerCount; layerIndex++)
        {
            float distance = ForwardTrackDistance(trackIndex, graph.Layers[layerIndex].TrackIndex, trackLength);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestLayer = layerIndex;
            }
        }

        distanceToLayer = bestDistance;
        return bestLayer;
    }

    private static DynamicPathNodeId SelectClosestReachableOffsetNode(
        DynamicPathOfflineGraph graph,
        DynamicPathLayer layer,
        float offset
    )
    {
        int bestNodeIndex = -1;
        float bestOffsetError = float.PositiveInfinity;
        for (int nodeIndex = 0; nodeIndex < layer.Nodes.Length; nodeIndex++)
        {
            DynamicPathNode node = layer.Nodes[nodeIndex];
            if (graph.GetOutgoingEdges(node.Id).Count == 0)
                continue;

            float offsetError = MathF.Abs(node.Offset - offset);
            if (offsetError < bestOffsetError)
            {
                bestOffsetError = offsetError;
                bestNodeIndex = nodeIndex;
            }
        }

        if (bestNodeIndex >= 0)
            return layer.Nodes[bestNodeIndex].Id;

        return SelectClosestOffsetNode(layer, offset);
    }

    private static DynamicPathNodeId SelectClosestOffsetNode(DynamicPathLayer layer, float offset)
    {
        int bestNodeIndex = 0;
        float bestOffsetError = float.PositiveInfinity;
        for (int nodeIndex = 0; nodeIndex < layer.Nodes.Length; nodeIndex++)
        {
            float offsetError = MathF.Abs(layer.Nodes[nodeIndex].Offset - offset);
            if (offsetError < bestOffsetError)
            {
                bestOffsetError = offsetError;
                bestNodeIndex = nodeIndex;
            }
        }

        return layer.Nodes[bestNodeIndex].Id;
    }

    private static DynamicPathNodeId FindClosestNode(
        DynamicPathOfflineGraph graph,
        Vector2 position,
        out float distance
    )
    {
        DynamicPathNodeId closest = graph.Layers[0].Nodes[0].Id;
        float bestDistanceSquared = float.PositiveInfinity;

        for (int layerIndex = 0; layerIndex < graph.Layers.Length; layerIndex++)
        {
            DynamicPathNode[] nodes = graph.Layers[layerIndex].Nodes;
            for (int nodeIndex = 0; nodeIndex < nodes.Length; nodeIndex++)
            {
                float distanceSquared = position.DistanceSquaredTo(nodes[nodeIndex].Position);
                if (distanceSquared < bestDistanceSquared)
                {
                    bestDistanceSquared = distanceSquared;
                    closest = nodes[nodeIndex].Id;
                }
            }
        }

        distance = MathF.Sqrt(bestDistanceSquared);
        return closest;
    }

    private bool IsHeadingCompatible(float poseHeading, float nodeHeading)
    {
        float headingError = MathF.Abs(GeomUtil.NormalizeAngle(poseHeading - nodeHeading));
        return headingError <= _config.MaxInitialHeadingOffsetRadians + 1e-4f;
    }

    private int SelectGoalLayer(DynamicPathOfflineGraph graph, int startLayer)
    {
        return _config.PlanHorizonMode switch
        {
            DynamicPathPlanHorizonMode.Layers => SelectGoalLayerByLayers(graph, startLayer),
            _ => SelectGoalLayerByDistance(graph, startLayer)
        };
    }

    private int SelectGoalLayerByLayers(DynamicPathOfflineGraph graph, int startLayer)
    {
        int layer = WrapLayer(startLayer + _config.MinimumPlanHorizonLayers, graph.LayerCount);
        return layer == startLayer ? WrapLayer(layer + 1, graph.LayerCount) : layer;
    }

    private int SelectGoalLayerByDistance(DynamicPathOfflineGraph graph, int startLayer)
    {
        int layer = startLayer;
        float distance = 0.0f;
        float loopLength = CalculateLayerLoopLength(graph);
        int allowedSteps = Math.Max(
            graph.LayerCount,
            graph.LayerCount * (Mathf.CeilToInt(_config.MinimumPlanHorizonMeters / Mathf.Max(loopLength, 1e-4f)) + 1)
        );

        for (int step = 0; step < allowedSteps; step++)
        {
            int nextLayer = WrapLayer(layer + 1, graph.LayerCount);
            distance += RaceLineDistanceBetweenAdjacentLayers(graph, layer, nextLayer, loopLength);
            layer = nextLayer;

            if (distance >= _config.MinimumPlanHorizonMeters)
                return layer == startLayer ? WrapLayer(layer + 1, graph.LayerCount) : layer;
        }

        return layer == startLayer ? WrapLayer(layer + 1, graph.LayerCount) : layer;
    }

    private static float CalculateLayerLoopLength(DynamicPathOfflineGraph graph)
    {
        float firstDistance = graph.Layers[0].RaceLineDistance;
        float lastDistance = graph.Layers[^1].RaceLineDistance;
        float closingDistance = RaceNodeDistance(graph, graph.LayerCount - 1, 0);
        return Mathf.Max(lastDistance - firstDistance + closingDistance, 1e-4f);
    }

    private static float RaceLineDistanceBetweenAdjacentLayers(
        DynamicPathOfflineGraph graph,
        int fromLayer,
        int toLayer,
        float loopLength
    )
    {
        float fromDistance = graph.Layers[fromLayer].RaceLineDistance;
        float toDistance = graph.Layers[toLayer].RaceLineDistance;
        float distance = toDistance >= fromDistance
            ? toDistance - fromDistance
            : loopLength - fromDistance + toDistance;
        return Mathf.Max(distance, 1e-4f);
    }

    private static float RaceNodeDistance(DynamicPathOfflineGraph graph, int fromLayer, int toLayer)
    {
        DynamicPathLayer from = graph.Layers[fromLayer];
        DynamicPathLayer to = graph.Layers[toLayer];
        Vector2 fromPosition = from.Nodes[from.RaceLineNodeIndex].Position;
        Vector2 toPosition = to.Nodes[to.RaceLineNodeIndex].Position;
        return Mathf.Max(fromPosition.DistanceTo(toPosition), 1e-4f);
    }

    private SearchResult? SearchToGoalLayer(
        DynamicPathOfflineGraph graph,
        DynamicPathNodeId startNode,
        int goalLayer,
        PreferredEdgeCostFactor[]? preferredEdgeCostFactors
    )
    {
        SearchScratch scratch = GetSearchScratch(graph);
        scratch.BeginSearch();

        int startIndex = graph.GetGlobalNodeIndexUnchecked(startNode);
        scratch.Distances[startIndex] = 0.0f;
        scratch.Queue.Enqueue(startNode, 0.0f);

        DynamicPathNodeId? bestGoalNode = null;
        float bestSearchCost = float.PositiveInfinity;
        float bestVirtualCost = 0.0f;

        while (scratch.Queue.TryDequeue(out DynamicPathNodeId current, out float queuedCost))
        {
            int currentIndex = graph.GetGlobalNodeIndexUnchecked(current);
            if (scratch.Settled[currentIndex])
                continue;
            if (queuedCost > scratch.Distances[currentIndex] + 1e-5f)
                continue;
            if (queuedCost >= bestSearchCost)
                break;

            scratch.Settled[currentIndex] = true;

            if (current.Layer == goalLayer)
            {
                DynamicPathVirtualGoalEdge virtualGoal = graph.GetVirtualGoalEdges(goalLayer)[current.Node];
                float searchCost = queuedCost + virtualGoal.OfflineCost;
                if (searchCost < bestSearchCost)
                {
                    bestSearchCost = searchCost;
                    bestVirtualCost = virtualGoal.OfflineCost;
                    bestGoalNode = current;
                }
            }

            IReadOnlyList<DynamicPathEdge> outgoing = graph.GetOutgoingEdges(current);
            for (int i = 0; i < outgoing.Count; i++)
            {
                DynamicPathEdge edge = outgoing[i];
                int nextIndex = graph.GetGlobalNodeIndexUnchecked(edge.To);
                float edgeCost = edge.OfflineCost;
                if (preferredEdgeCostFactors != null
                    && TryGetPreferredEdgeCostFactor(preferredEdgeCostFactors, edge.From, edge.To, out float costFactor))
                {
                    edgeCost *= costFactor;
                }

                float nextCost = queuedCost + edgeCost;
                if (nextCost + 1e-5f >= scratch.Distances[nextIndex])
                    continue;

                scratch.Distances[nextIndex] = nextCost;
                scratch.PreviousNodes[nextIndex] = current;
                scratch.PreviousEdges[nextIndex] = edge;
                scratch.Queue.Enqueue(edge.To, nextCost);
            }
        }

        if (bestGoalNode is null)
            return null;

        return ReconstructSearchResult(
            startNode,
            bestGoalNode.Value,
            bestSearchCost,
            bestVirtualCost,
            scratch,
            graph
        );
    }

    private static SearchResult ReconstructSearchResult(
        DynamicPathNodeId startNode,
        DynamicPathNodeId goalNode,
        float searchCost,
        float virtualGoalCost,
        SearchScratch scratch,
        DynamicPathOfflineGraph graph
    )
    {
        List<DynamicPathNodeId> reversedNodes = [];
        List<DynamicPathEdge> reversedEdges = [];
        DynamicPathNodeId current = goalNode;
        reversedNodes.Add(current);

        while (current != startNode)
        {
            int currentIndex = graph.GetGlobalNodeIndexUnchecked(current);
            DynamicPathEdge? previousEdge = scratch.PreviousEdges[currentIndex];
            DynamicPathNodeId? previousNode = scratch.PreviousNodes[currentIndex];
            if (previousEdge is null || previousNode is null)
                throw new InvalidOperationException("Failed to reconstruct dynamic path search result.");

            reversedEdges.Add(previousEdge);
            current = previousNode.Value;
            reversedNodes.Add(current);
        }

        reversedNodes.Reverse();
        reversedEdges.Reverse();

        return new SearchResult(
            goalNode,
            [.. reversedNodes],
            [.. reversedEdges],
            searchCost,
            virtualGoalCost
        );
    }

    private static DynamicPathSplineSegment[] CreateSmoothedSplines(
        DynamicPathOfflineGraph graph,
        DynamicPathNodeId[] nodePath,
        DynamicPathEdge[] edgePath,
        float? startHeadingOverride
    )
    {
        if (nodePath.Length < 2)
            throw new ArgumentException("At least two nodes are required.", nameof(nodePath));
        if (edgePath.Length != nodePath.Length - 1)
            throw new ArgumentException("Edge count must be one less than node count.", nameof(edgePath));

        Vector2[] points = new Vector2[nodePath.Length];
        for (int i = 0; i < nodePath.Length; i++)
            points[i] = graph.GetNode(nodePath[i]).Position;

        float[] lengths = new float[edgePath.Length];
        for (int i = 0; i < edgePath.Length; i++)
            lengths[i] = Mathf.Max(edgePath[i].Length, 1e-4f);

        float startHeading = startHeadingOverride ?? graph.GetNode(nodePath[0]).Heading;
        DynamicPathEdgeSample[] lastEdgeSamples = edgePath[^1].Samples;
        float endHeading = lastEdgeSamples.Length > 0
            ? lastEdgeSamples[^1].Heading
            : graph.GetNode(nodePath[^1]).Heading;

        return DynamicPathSplineMath.CreateUnclosedPath(points, startHeading, endHeading, lengths);
    }

    private static DynamicPathEdgeSample[] SampleSplines(
        DynamicPathSplineSegment[] splines,
        float sampleStep
    )
    {
        return SamplePathSplines(splines, sampleStep).Samples;
    }

    private static SampledPath SamplePathSplines(
        DynamicPathSplineSegment[] splines,
        float sampleStep
    )
    {
        List<Vector2> positions = [];
        List<float> headings = [];
        List<float> curvatures = [];
        List<int> nodeSampleIndexes = [0];

        for (int segmentIndex = 0; segmentIndex < splines.Length; segmentIndex++)
        {
            DynamicPathSplineSegment spline = splines[segmentIndex];
            float estimatedLength = DynamicPathSplineMath.EstimateLength(spline);
            int sampleCount = Math.Max(2, Mathf.CeilToInt(Mathf.Max(estimatedLength, sampleStep) / sampleStep) + 1);

            for (int i = 0; i < sampleCount; i++)
            {
                if (segmentIndex > 0 && i == 0)
                    continue;

                float t = i / (float)(sampleCount - 1);
                Vector2 derivative = spline.Derivative(t);
                Vector2 secondDerivative = spline.SecondDerivative(t);
                float speedSquared = derivative.LengthSquared();
                float denominator = Mathf.Pow(speedSquared, 1.5f);

                positions.Add(spline.Evaluate(t));
                headings.Add(speedSquared > 1e-8f ? derivative.Angle() : 0.0f);
                curvatures.Add(denominator > 1e-8f ? derivative.Cross(secondDerivative) / denominator : 0.0f);
            }

            nodeSampleIndexes.Add(positions.Count - 1);
        }

        return new SampledPath(BuildSamples(positions, headings, curvatures), [.. nodeSampleIndexes]);
    }

    private static PrefixSegment BuildInheritedPrefix(
        DynamicPathEdgeSample[] previousSamples,
        float[] previousSampleTrackProgress,
        PathProjection projection,
        int startNodeSampleIndex
    )
    {
        if (previousSamples.Length == 0)
            return new PrefixSegment([], []);

        int firstCopiedSample = Math.Clamp(projection.CutSampleIndex, 0, previousSamples.Length - 1);
        int lastCopiedSample = Math.Clamp(startNodeSampleIndex, 0, previousSamples.Length - 1);
        if (lastCopiedSample < firstCopiedSample)
            lastCopiedSample = firstCopiedSample;

        bool projectionIsExistingSample =
            previousSamples[firstCopiedSample].Position.DistanceSquaredTo(projection.Position) <= 1e-6f;
        int count = lastCopiedSample - firstCopiedSample + 1 + (projectionIsExistingSample ? 0 : 1);
        DynamicPathEdgeSample[] samples = new DynamicPathEdgeSample[count];
        float[] sampleTrackProgress = new float[count];

        int outputIndex = 0;
        if (!projectionIsExistingSample)
        {
            samples[outputIndex] = new DynamicPathEdgeSample(
                projection.Position,
                projection.Heading,
                projection.Curvature,
                LengthToNext: 0.0f
            );
            sampleTrackProgress[outputIndex] = 0.0f;
            outputIndex++;
        }

        for (int i = firstCopiedSample; i <= lastCopiedSample; i++)
        {
            samples[outputIndex] = previousSamples[i];
            sampleTrackProgress[outputIndex] = MathF.Max(0.0f, previousSampleTrackProgress[i] - projection.Progress);
            outputIndex++;
        }

        sampleTrackProgress[0] = 0.0f;
        samples = RecalculateSampleLengths(samples);
        return new PrefixSegment(samples, sampleTrackProgress);
    }

    private static DynamicPathEdgeSample[] BuildSamples(
        List<Vector2> positions,
        List<float> headings,
        List<float> curvatures
    )
    {
        DynamicPathEdgeSample[] samples = new DynamicPathEdgeSample[positions.Count];
        for (int i = 0; i < positions.Count; i++)
        {
            float lengthToNext = i < positions.Count - 1
                ? positions[i].DistanceTo(positions[i + 1])
                : 0.0f;
            samples[i] = new DynamicPathEdgeSample(positions[i], headings[i], curvatures[i], lengthToNext);
        }

        return samples;
    }

    private static DynamicPathEdgeSample[] CombineConnectorAndGraphSamples(
        DynamicPathEdgeSample[] connectorSamples,
        DynamicPathEdgeSample[] graphSamples
    )
    {
        if (connectorSamples.Length == 0)
            return graphSamples;
        if (graphSamples.Length == 0)
            return connectorSamples;

        DynamicPathEdgeSample[] combined = new DynamicPathEdgeSample[connectorSamples.Length + graphSamples.Length - 1];
        for (int i = 0; i < connectorSamples.Length - 1; i++)
            combined[i] = connectorSamples[i];
        for (int i = 0; i < graphSamples.Length; i++)
            combined[connectorSamples.Length - 1 + i] = graphSamples[i];
        return combined;
    }

    private static float[] CombineConnectorAndGraphProgress(
        float[]? connectorProgress,
        float[] graphProgress
    )
    {
        if (connectorProgress == null || connectorProgress.Length == 0)
            return graphProgress;
        if (graphProgress.Length == 0)
            return connectorProgress;

        float[] combined = new float[connectorProgress.Length + graphProgress.Length - 1];
        for (int i = 0; i < connectorProgress.Length - 1; i++)
            combined[i] = connectorProgress[i];
        for (int i = 0; i < graphProgress.Length; i++)
            combined[connectorProgress.Length - 1 + i] = graphProgress[i];
        return combined;
    }

    private static int[] OffsetNodeSampleIndexes(int[] nodeSampleIndexes, int offset)
    {
        int[] shifted = new int[nodeSampleIndexes.Length];
        for (int i = 0; i < nodeSampleIndexes.Length; i++)
            shifted[i] = nodeSampleIndexes[i] + offset;
        return shifted;
    }

    private static DynamicPathEdgeSample[] CopyBackupSamplesFromNode(
        DynamicPathEdgeSample[] backupSamples,
        DynamicPathEdgeSample[] prefixSamples,
        int startNodeSampleIndex
    )
    {
        int remainingCount = Math.Max(0, backupSamples.Length - startNodeSampleIndex - 1);
        DynamicPathEdgeSample[] samples = new DynamicPathEdgeSample[prefixSamples.Length + remainingCount];
        for (int i = 0; i < prefixSamples.Length; i++)
            samples[i] = prefixSamples[i];

        for (int i = 0; i < remainingCount; i++)
            samples[prefixSamples.Length + i] = backupSamples[startNodeSampleIndex + 1 + i];

        return RecalculateSampleLengths(samples);
    }

    private static float[] CopyBackupSampleProgressFromNode(
        float[] backupSampleProgress,
        float[] prefixSampleProgress,
        int startNodeSampleIndex,
        float consumedProgress
    )
    {
        int remainingCount = Math.Max(0, backupSampleProgress.Length - startNodeSampleIndex - 1);
        float[] sampleProgress = new float[prefixSampleProgress.Length + remainingCount];
        for (int i = 0; i < prefixSampleProgress.Length; i++)
            sampleProgress[i] = prefixSampleProgress[i];

        for (int i = 0; i < remainingCount; i++)
        {
            sampleProgress[prefixSampleProgress.Length + i] = MathF.Max(
                0.0f,
                backupSampleProgress[startNodeSampleIndex + 1 + i] - consumedProgress
            );
        }

        return sampleProgress;
    }

    private static int[] CopyBackupNodeSampleIndexes(
        int[] backupNodeSampleIndexes,
        int startNodePathIndex,
        int startNodeSampleIndex,
        int prefixSampleCount
    )
    {
        int[] nodeSampleIndexes = new int[backupNodeSampleIndexes.Length - startNodePathIndex];
        int offset = prefixSampleCount - 1 - startNodeSampleIndex;
        for (int i = 0; i < nodeSampleIndexes.Length; i++)
            nodeSampleIndexes[i] = backupNodeSampleIndexes[startNodePathIndex + i] + offset;
        return nodeSampleIndexes;
    }

    private static DynamicPathEdgeSample[] RecalculateSampleLengths(DynamicPathEdgeSample[] samples)
    {
        DynamicPathEdgeSample[] recalculated = new DynamicPathEdgeSample[samples.Length];
        for (int i = 0; i < samples.Length; i++)
        {
            float lengthToNext = i < samples.Length - 1
                ? samples[i].Position.DistanceTo(samples[i + 1].Position)
                : 0.0f;
            recalculated[i] = samples[i] with { LengthToNext = lengthToNext };
        }

        return recalculated;
    }

    private PreferredEdgeCostFactor[]? BuildPreviousEdgeCostFactors(
        DynamicPathOnlinePath previousPath,
        int startNodePathIndex
    )
    {
        if (_config.PreviousPathCostFactors.Length == 0)
            return null;

        int count = Math.Min(
            _config.PreviousPathCostFactors.Length,
            Math.Max(0, previousPath.EdgePath.Length - startNodePathIndex)
        );
        if (count == 0)
            return null;

        PreferredEdgeCostFactor[] costFactors = new PreferredEdgeCostFactor[count];
        for (int factorIndex = 0; factorIndex < count; factorIndex++)
        {
            int edgeIndex = startNodePathIndex + factorIndex;
            DynamicPathEdge edge = previousPath.EdgePath[edgeIndex];
            costFactors[factorIndex] = new PreferredEdgeCostFactor(
                edge.From,
                edge.To,
                _config.PreviousPathCostFactors[factorIndex]
            );
        }

        return costFactors;
    }

    private static bool TryProjectOntoPathProgress(
        DynamicPathEdgeSample[] samples,
        float[] sampleProgress,
        Vector2 position,
        out PathProjection projection
    )
    {
        projection = default;
        if (samples.Length == 0 || sampleProgress.Length != samples.Length)
            return false;

        if (samples.Length == 1)
        {
            projection = new PathProjection(
                CutSampleIndex: 0,
                Progress: sampleProgress[0],
                Position: samples[0].Position,
                Heading: samples[0].Heading,
                Curvature: samples[0].Curvature,
                Distance: position.DistanceTo(samples[0].Position)
            );
            return true;
        }

        float bestDistanceSquared = float.PositiveInfinity;
        Vector2 bestPosition = samples[0].Position;
        float bestProgress = sampleProgress[0];
        float bestHeading = samples[0].Heading;
        float bestCurvature = samples[0].Curvature;

        for (int i = 0; i < samples.Length - 1; i++)
        {
            Vector2 start = samples[i].Position;
            Vector2 end = samples[i + 1].Position;
            Vector2 segment = end - start;
            float segmentLengthSquared = segment.LengthSquared();
            float t = segmentLengthSquared > 1e-8f
                ? Mathf.Clamp((position - start).Dot(segment) / segmentLengthSquared, 0.0f, 1.0f)
                : 0.0f;
            Vector2 projected = start + segment * t;
            float distanceSquared = position.DistanceSquaredTo(projected);
            if (distanceSquared >= bestDistanceSquared)
                continue;

            bestDistanceSquared = distanceSquared;
            bestPosition = projected;
            bestProgress = sampleProgress[i] + (sampleProgress[i + 1] - sampleProgress[i]) * t;
            bestHeading = segmentLengthSquared > 1e-8f ? segment.Angle() : samples[i].Heading;
            bestCurvature = samples[i].Curvature + (samples[i + 1].Curvature - samples[i].Curvature) * t;
        }

        int cutSampleIndex = FindFirstSampleAtOrAfterProgress(sampleProgress, bestProgress);
        if (cutSampleIndex < 0)
            cutSampleIndex = samples.Length - 1;

        projection = new PathProjection(
            CutSampleIndex: cutSampleIndex,
            Progress: bestProgress,
            Position: bestPosition,
            Heading: bestHeading,
            Curvature: bestCurvature,
            Distance: MathF.Sqrt(bestDistanceSquared)
        );
        return true;
    }

    private static bool TryProjectOntoPathProgressNear(
        DynamicPathEdgeSample[] samples,
        float[] sampleProgress,
        Vector2 position,
        float targetProgress,
        float progressWindow,
        out PathProjection projection
    )
    {
        projection = default;
        if (samples.Length == 0 || sampleProgress.Length != samples.Length)
            return false;
        if (samples.Length == 1)
        {
            projection = new PathProjection(
                CutSampleIndex: 0,
                Progress: sampleProgress[0],
                Position: samples[0].Position,
                Heading: samples[0].Heading,
                Curvature: samples[0].Curvature,
                Distance: position.DistanceTo(samples[0].Position)
            );
            return true;
        }

        float minProgress = targetProgress - MathF.Max(0.0f, progressWindow);
        float maxProgress = targetProgress + MathF.Max(0.0f, progressWindow);
        float bestDistanceSquared = float.PositiveInfinity;
        Vector2 bestPosition = samples[0].Position;
        float bestProgress = sampleProgress[0];
        float bestHeading = samples[0].Heading;
        float bestCurvature = samples[0].Curvature;
        bool found = false;

        for (int i = 0; i < samples.Length - 1; i++)
        {
            float startProgress = sampleProgress[i];
            float endProgress = sampleProgress[i + 1];
            float segmentMinProgress = MathF.Min(startProgress, endProgress);
            float segmentMaxProgress = MathF.Max(startProgress, endProgress);
            if (segmentMaxProgress < minProgress || segmentMinProgress > maxProgress)
                continue;

            Vector2 start = samples[i].Position;
            Vector2 end = samples[i + 1].Position;
            Vector2 segment = end - start;
            float segmentLengthSquared = segment.LengthSquared();
            float t = segmentLengthSquared > 1e-8f
                ? Mathf.Clamp((position - start).Dot(segment) / segmentLengthSquared, 0.0f, 1.0f)
                : 0.0f;
            float projectedProgress = startProgress + (endProgress - startProgress) * t;
            if (projectedProgress < minProgress || projectedProgress > maxProgress)
                continue;

            Vector2 projected = start + segment * t;
            float distanceSquared = position.DistanceSquaredTo(projected);
            if (distanceSquared >= bestDistanceSquared)
                continue;

            found = true;
            bestDistanceSquared = distanceSquared;
            bestPosition = projected;
            bestProgress = projectedProgress;
            bestHeading = segmentLengthSquared > 1e-8f ? segment.Angle() : samples[i].Heading;
            bestCurvature = samples[i].Curvature + (samples[i + 1].Curvature - samples[i].Curvature) * t;
        }

        if (!found)
            return false;

        int cutSampleIndex = FindFirstSampleAtOrAfterProgress(sampleProgress, bestProgress);
        if (cutSampleIndex < 0)
            cutSampleIndex = samples.Length - 1;

        projection = new PathProjection(
            CutSampleIndex: cutSampleIndex,
            Progress: bestProgress,
            Position: bestPosition,
            Heading: bestHeading,
            Curvature: bestCurvature,
            Distance: MathF.Sqrt(bestDistanceSquared)
        );
        return true;
    }

    private static bool TryAdvanceProjectionByDistance(
        DynamicPathEdgeSample[] samples,
        float[] sampleProgress,
        PathProjection projection,
        float distance,
        out int sampleIndex,
        out float progress
    )
    {
        sampleIndex = -1;
        progress = 0.0f;
        if (samples.Length == 0 || sampleProgress.Length != samples.Length)
            return false;

        if (samples.Length == 1)
        {
            sampleIndex = 0;
            progress = sampleProgress[0];
            return true;
        }

        float remainingDistance = MathF.Max(0.0f, distance);
        int nextSampleIndex = Math.Clamp(projection.CutSampleIndex, 0, samples.Length - 1);
        if (remainingDistance <= 1e-5f)
        {
            sampleIndex = nextSampleIndex;
            progress = projection.Progress;
            return true;
        }

        float distanceToNextSample = projection.Position.DistanceTo(samples[nextSampleIndex].Position);
        if (remainingDistance <= distanceToNextSample + 1e-5f)
        {
            float t = distanceToNextSample > 1e-5f
                ? Mathf.Clamp(remainingDistance / distanceToNextSample, 0.0f, 1.0f)
                : 1.0f;
            sampleIndex = nextSampleIndex;
            progress = projection.Progress + (sampleProgress[nextSampleIndex] - projection.Progress) * t;
            return true;
        }

        remainingDistance -= distanceToNextSample;
        int currentSampleIndex = nextSampleIndex;
        while (currentSampleIndex < samples.Length - 1)
        {
            float segmentLength = samples[currentSampleIndex].LengthToNext;
            if (remainingDistance <= segmentLength + 1e-5f)
            {
                float t = segmentLength > 1e-5f
                    ? Mathf.Clamp(remainingDistance / segmentLength, 0.0f, 1.0f)
                    : 1.0f;
                sampleIndex = currentSampleIndex + 1;
                progress = sampleProgress[currentSampleIndex]
                    + (sampleProgress[currentSampleIndex + 1] - sampleProgress[currentSampleIndex]) * t;
                return true;
            }

            remainingDistance -= Mathf.Max(segmentLength, 1e-5f);
            currentSampleIndex++;
        }

        sampleIndex = samples.Length - 1;
        progress = sampleProgress[^1];
        return true;
    }

    private static int FindClosestSampleIndex(
        DynamicPathEdgeSample[] samples,
        Vector2 position,
        out float distance
    )
    {
        int closest = -1;
        float bestDistanceSquared = float.PositiveInfinity;
        for (int i = 0; i < samples.Length; i++)
        {
            float distanceSquared = position.DistanceSquaredTo(samples[i].Position);
            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                closest = i;
            }
        }

        distance = closest < 0 ? float.PositiveInfinity : MathF.Sqrt(bestDistanceSquared);
        return closest;
    }

    private static int AdvanceSampleIndex(
        DynamicPathEdgeSample[] samples,
        int startSampleIndex,
        float distance
    )
    {
        int sampleIndex = Math.Clamp(startSampleIndex, 0, samples.Length - 1);
        float remainingDistance = distance;
        while (sampleIndex < samples.Length - 1 && remainingDistance > samples[sampleIndex].LengthToNext)
        {
            remainingDistance -= Mathf.Max(samples[sampleIndex].LengthToNext, 1e-4f);
            sampleIndex++;
        }

        return sampleIndex;
    }

    private static int FindFirstNodeAfterSample(int[] nodeSampleIndexes, int sampleIndex)
    {
        for (int i = 0; i < nodeSampleIndexes.Length; i++)
        {
            if (nodeSampleIndexes[i] > sampleIndex)
                return i;
        }

        return -1;
    }

    private static float[] BuildSampleProgressByLength(
        DynamicPathEdgeSample[] samples,
        float startProgress,
        float endProgress
    )
    {
        float[] progress = new float[samples.Length];
        if (samples.Length == 0)
            return progress;

        float physicalLength = CalculatePhysicalLength(samples);
        float progressLength = MathF.Max(0.0f, endProgress - startProgress);
        float physicalProgress = 0.0f;
        for (int i = 0; i < samples.Length; i++)
        {
            float ratio = physicalLength > 1e-4f
                ? Mathf.Clamp(physicalProgress / physicalLength, 0.0f, 1.0f)
                : 0.0f;
            progress[i] = startProgress + ratio * progressLength;
            physicalProgress += samples[i].LengthToNext;
        }

        progress[^1] = endProgress;
        return progress;
    }

    private static float[] BuildSampleProgressFromNodes(
        DynamicPathEdgeSample[] samples,
        int[] nodeSampleIndexes,
        float[] nodeProgress
    )
    {
        float[] sampleProgress = new float[samples.Length];
        if (samples.Length == 0)
            return sampleProgress;
        if (nodeSampleIndexes.Length == 0 || nodeProgress.Length == 0)
        {
            float cumulative = 0.0f;
            for (int i = 0; i < samples.Length; i++)
            {
                sampleProgress[i] = cumulative;
                cumulative += samples[i].LengthToNext;
            }
            return sampleProgress;
        }

        for (int nodeIndex = 0; nodeIndex < nodeSampleIndexes.Length - 1; nodeIndex++)
        {
            int firstSampleIndex = Math.Clamp(nodeSampleIndexes[nodeIndex], 0, samples.Length - 1);
            int lastSampleIndex = Math.Clamp(nodeSampleIndexes[nodeIndex + 1], firstSampleIndex, samples.Length - 1);
            float startProgress = nodeProgress[nodeIndex];
            float endProgress = nodeProgress[Math.Min(nodeIndex + 1, nodeProgress.Length - 1)];
            float segmentLength = 0.0f;
            for (int sampleIndex = firstSampleIndex; sampleIndex < lastSampleIndex; sampleIndex++)
                segmentLength += samples[sampleIndex].LengthToNext;

            float physicalProgress = 0.0f;
            for (int sampleIndex = firstSampleIndex; sampleIndex <= lastSampleIndex; sampleIndex++)
            {
                float ratio = segmentLength > 1e-4f
                    ? Mathf.Clamp(physicalProgress / segmentLength, 0.0f, 1.0f)
                    : 0.0f;
                sampleProgress[sampleIndex] = startProgress + ratio * MathF.Max(0.0f, endProgress - startProgress);
                if (sampleIndex < lastSampleIndex)
                    physicalProgress += samples[sampleIndex].LengthToNext;
            }
        }

        sampleProgress[nodeSampleIndexes[^1]] = nodeProgress[^1];
        return sampleProgress;
    }

    private static float[] CalculateNodePhysicalProgress(
        DynamicPathEdgeSample[] samples,
        int[] nodeSampleIndexes
    )
    {
        float[] nodeProgress = new float[nodeSampleIndexes.Length];
        float cumulative = 0.0f;
        int nextNode = 0;
        for (int sampleIndex = 0; sampleIndex < samples.Length; sampleIndex++)
        {
            while (nextNode < nodeSampleIndexes.Length && nodeSampleIndexes[nextNode] == sampleIndex)
            {
                nodeProgress[nextNode] = cumulative;
                nextNode++;
            }

            cumulative += samples[sampleIndex].LengthToNext;
        }

        return nodeProgress;
    }

    private static float[] CalculateNodeTrackProgress(
        DynamicPathOfflineGraph graph,
        DynamicPathNodeId[] nodePath,
        int anchorTrackIndex,
        int trackLength
    )
    {
        float[] progress = new float[nodePath.Length];
        int previousTrackIndex = anchorTrackIndex;
        float cumulativeDistance = 0.0f;

        for (int i = 0; i < nodePath.Length; i++)
        {
            int nodeTrackIndex = graph.Layers[nodePath[i].Layer].TrackIndex;
            cumulativeDistance += ForwardTrackDistance(previousTrackIndex, nodeTrackIndex, trackLength);
            progress[i] = cumulativeDistance;
            previousTrackIndex = nodeTrackIndex;
        }

        return progress;
    }

    private static int FindFirstUneatenNodeAfterProgress(float[] nodeProgress, float targetProgress)
    {
        for (int i = 0; i < nodeProgress.Length; i++)
        {
            if (nodeProgress[i] > targetProgress + 1e-4f)
                return i;
        }

        return -1;
    }

    private static bool TryGetPreferredEdgeCostFactor(
        PreferredEdgeCostFactor[] costFactors,
        DynamicPathNodeId from,
        DynamicPathNodeId to,
        out float factor
    )
    {
        for (int i = 0; i < costFactors.Length; i++)
        {
            PreferredEdgeCostFactor costFactor = costFactors[i];
            if (costFactor.From == from && costFactor.To == to)
            {
                factor = costFactor.Factor;
                return true;
            }
        }

        factor = 0.0f;
        return false;
    }

    private static int FindFirstSampleAtOrAfterProgress(float[] sampleProgress, float targetProgress)
    {
        for (int i = 0; i < sampleProgress.Length; i++)
        {
            if (sampleProgress[i] >= targetProgress - 1e-4f)
                return i;
        }

        return -1;
    }

    private static float CalculateTrackBoundaryExcess(TrackPoint point, Vector2 position)
    {
        float lateralOffset = (position - point.Center).Dot(point.Normal);
        return CalculateTrackBoundaryExcess(point, lateralOffset);
    }

    private static float CalculateTrackBoundaryExcess(TrackPoint point, float lateralOffset)
    {
        float boundaryHalfWidth = lateralOffset >= 0.0f
            ? point.HalfWidth + point.LeftBufferWidth
            : point.HalfWidth + point.RightBufferWidth;
        return MathF.Max(0.0f, MathF.Abs(lateralOffset) - MathF.Max(0.0f, boundaryHalfWidth));
    }

    private static float ForwardTrackDistance(int fromTrackIndex, int toTrackIndex, int trackLength)
    {
        int from = WrapIndex(fromTrackIndex, trackLength);
        int to = WrapIndex(toTrackIndex, trackLength);
        int delta = to - from;
        if (delta < 0)
            delta += trackLength;
        return delta * TrackData.StepLength;
    }

    private static float CalculatePhysicalLength(DynamicPathEdgeSample[] samples)
    {
        float length = 0.0f;
        for (int i = 0; i < samples.Length; i++)
            length += samples[i].LengthToNext;
        return length;
    }

    private static int PreviousLayer(int layer, int layerCount)
    {
        return WrapLayer(layer - 1, layerCount);
    }

    private static int WrapLayer(int layer, int layerCount)
    {
        int wrapped = layer % layerCount;
        return wrapped < 0 ? wrapped + layerCount : wrapped;
    }

    private static int WrapIndex(int index, int length)
    {
        int wrapped = index % length;
        return wrapped < 0 ? wrapped + length : wrapped;
    }

    private SearchScratch GetSearchScratch(DynamicPathOfflineGraph graph)
    {
        if (_searchScratch is null || !ReferenceEquals(_searchScratch.Graph, graph))
            _searchScratch = new SearchScratch(graph);
        return _searchScratch;
    }

    private sealed class SearchScratch
    {
        public readonly DynamicPathOfflineGraph Graph;
        public readonly float[] Distances;
        public readonly bool[] Settled;
        public readonly DynamicPathNodeId?[] PreviousNodes;
        public readonly DynamicPathEdge?[] PreviousEdges;
        public readonly PriorityQueue<DynamicPathNodeId, float> Queue = new();

        public SearchScratch(DynamicPathOfflineGraph graph)
        {
            Graph = graph;
            Distances = new float[graph.NodeCount];
            Settled = new bool[graph.NodeCount];
            PreviousNodes = new DynamicPathNodeId?[graph.NodeCount];
            PreviousEdges = new DynamicPathEdge?[graph.NodeCount];
        }

        public void BeginSearch()
        {
            Array.Fill(Distances, float.PositiveInfinity);
            Array.Clear(Settled);
            Array.Clear(PreviousNodes);
            Array.Clear(PreviousEdges);
            Queue.Clear();
        }
    }

    private sealed record SearchResult(
        DynamicPathNodeId GoalNode,
        DynamicPathNodeId[] NodePath,
        DynamicPathEdge[] EdgePath,
        float SearchCost,
        float VirtualGoalCost
    );

    private sealed record SampledPath(
        DynamicPathEdgeSample[] Samples,
        int[] NodeSampleIndexes
    );

    private sealed record PrefixSegment(
        DynamicPathEdgeSample[] Samples,
        float[] SampleTrackProgress
    );

    private readonly record struct PreferredEdgeCostFactor(
        DynamicPathNodeId From,
        DynamicPathNodeId To,
        float Factor
    );

    private readonly record struct PathProjection(
        int CutSampleIndex,
        float Progress,
        Vector2 Position,
        float Heading,
        float Curvature,
        float Distance
    );
}
