using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Track;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Controllers.V1.GraphBased;

public sealed class GraphBasedLocalPlanner
{
    private const float InvalidCost = float.PositiveInfinity;
    private readonly SearchWorkspace _workspace = new();

    public GraphPlannerCache BuildCache(
        TrackData track,
        ITrackReferenceLine referenceLine,
        GraphPlannerConfig? config = null
    )
    {
        ArgumentNullException.ThrowIfNull(track);
        ArgumentNullException.ThrowIfNull(referenceLine);

        GraphPlannerConfig resolvedConfig = config ?? new GraphPlannerConfig();
        int layerStepSamples = LayerStepSamples(resolvedConfig);
        GraphPlannerLayer[] layers = new GraphPlannerLayer[track.Length];
        GraphPlannerEdgeCosts[] edgeCosts = new GraphPlannerEdgeCosts[track.Length];

        for (int trackIndex = 0; trackIndex < track.Length; trackIndex++)
            layers[trackIndex] = BuildLayer(track, referenceLine, trackIndex, resolvedConfig);

        for (int trackIndex = 0; trackIndex < track.Length; trackIndex++)
        {
            edgeCosts[trackIndex] = BuildEdgeCosts(layers, trackIndex, layerStepSamples, resolvedConfig);
        }

        GraphPlannerCache cache = new(track, resolvedConfig, layerStepSamples, layers, edgeCosts);
        _workspace.PrepareForSearch(SearchLayerCount(resolvedConfig), cache.MaxNodeCount);
        return cache;
    }

    public GraphPath Plan(GraphPlannerRequest request)
    {
        TrackData track = request.Track ?? throw new ArgumentNullException(nameof(request.Track));
        GraphPlannerCache cache = request.Cache ?? BuildCache(
            track,
            request.ReferenceLine ?? throw new ArgumentNullException(nameof(request.ReferenceLine)),
            request.Config
        );
        if (!ReferenceEquals(track, cache.Track))
            throw new ArgumentException("Graph planner cache was baked for a different track.", nameof(request));

        GraphPlannerConfig config = cache.Config;

        int startIndex = track.FindNearestIndex(request.Position);
        GraphPlannerLayer[] layers = BuildLayers(cache, startIndex, config);
        if (layers.Length < 2)
            return BuildFallbackPath(track, cache, startIndex, request.Position, config);

        float startOffset = ClampOffset(
            (request.Position - track[startIndex].Center).Dot(track[startIndex].Normal),
            layers[0].MinOffset,
            layers[0].MaxOffset
        );

        if (TryPlanFromPreviousPath(track, cache, request, config, out GraphPath rollingPath))
            return rollingPath;

        float[] previousLayerOffsets = BuildPreviousLayerOffsets(layers, request.PreviousPath, track.Length, config);
        int[] previousNodeIndices = BuildPreviousNodeIndices(layers, request.PreviousPath);
        SearchResult result = Search(
            layers,
            cache,
            request.Position,
            request.Heading,
            startOffset,
            previousLayerOffsets,
            previousNodeIndices,
            entryLayerIndex: 1,
            config
        );
        return result.Found
            ? BuildPath(
                track,
                cache,
                layers,
                startIndex,
                request.Position,
                request.Heading,
                result,
                request.PreviousPath,
                config,
                isFallback: false
            )
            : BuildFallbackPath(track, cache, startIndex, request.Position, config);
    }

    private static GraphPlannerLayer[] BuildLayers(
        GraphPlannerCache cache,
        int startIndex,
        GraphPlannerConfig config
    )
    {
        int layerCount = SearchLayerCount(config);
        GraphPlannerLayer[] layers = new GraphPlannerLayer[layerCount];

        for (int layerIndex = 0; layerIndex < layerCount; layerIndex++)
        {
            int trackIndex = startIndex + layerIndex * cache.LayerStepSamples;
            layers[layerIndex] = cache.GetLayer(trackIndex);
        }

        return layers;
    }

    private static GraphPlannerLayer BuildLayer(
        TrackData track,
        ITrackReferenceLine referenceLine,
        int trackIndex,
        GraphPlannerConfig config
    )
    {
        TrackPoint point = track[trackIndex];
        TrackReferencePoint reference = referenceLine.GetPoint(point.Index);
        float usableShrink = Math.Max(0f, config.VehicleHalfWidthMeters + config.EdgeSafetyMarginMeters);
        float minOffset = -point.HalfWidth + usableShrink;
        float maxOffset = point.HalfWidth - usableShrink;
        if (minOffset > maxOffset)
        {
            float center = (minOffset + maxOffset) * 0.5f;
            minOffset = center;
            maxOffset = center;
        }

        float referenceOffset = ClampOffset(reference.Offset, minOffset, maxOffset);
        float referenceHeading = float.IsFinite(reference.Heading) ? reference.Heading : point.Tangent.Angle();
        float leftHeading = OffsetLineHeading(track, trackIndex, minOffset);
        float rightHeading = OffsetLineHeading(track, trackIndex, maxOffset);
        float[] offsets = BuildOffsets(minOffset, maxOffset, referenceOffset, config.LateralResolutionMeters);
        float[] headings = BuildNodeHeadings(offsets, minOffset, maxOffset, referenceOffset, referenceHeading, leftHeading, rightHeading);
        int referenceNodeIndex = ClosestOffsetIndex(offsets, referenceOffset);

        return new GraphPlannerLayer(
            point.Index,
            point.Center,
            point.Normal,
            minOffset,
            maxOffset,
            referenceOffset,
            offsets,
            headings,
            referenceNodeIndex
        );
    }

    private SearchResult Search(
        GraphPlannerLayer[] layers,
        GraphPlannerCache cache,
        Vector2 startPosition,
        float startHeading,
        float startOffset,
        float[] previousLayerOffsets,
        int[] previousNodeIndices,
        int entryLayerIndex,
        GraphPlannerConfig config
    )
    {
        _workspace.EnsureLayerCapacity(layers.Length);
        int firstSearchLayerIndex = Math.Clamp(entryLayerIndex, 1, layers.Length - 1);
        GraphPlannerLayer firstLayer = layers[firstSearchLayerIndex];
        State[] previousStates = _workspace.EnsureStateLayerCapacity(firstSearchLayerIndex, firstLayer.Offsets.Length);
        int previousCount = 0;
        int firstNodeStart = 0;
        int firstNodeEnd = firstLayer.Offsets.Length;
        if (firstSearchLayerIndex > 1)
        {
            firstNodeStart = firstLayer.ReferenceNodeIndex;
            firstNodeEnd = firstNodeStart + 1;
        }

        for (int currentIndex = firstNodeStart; currentIndex < firstNodeEnd; currentIndex++)
        {
            float currentOffset = firstLayer.Offsets[currentIndex];
            if (firstSearchLayerIndex == 1 && Math.Abs(currentOffset - startOffset) > MaxLateralDelta(cache.LayerStepSamples, config))
                continue;

            GraphPlannerEdge? startEdge = BuildSplineEdge(
                cache,
                layers[0].TrackIndex,
                firstSearchLayerIndex * cache.LayerStepSamples,
                startPosition,
                HeadingVector(startHeading),
                firstLayer.GetPosition(currentOffset),
                firstLayer.GetTangent(currentIndex),
                config,
                RacingLineDistance(firstLayer, currentIndex, config)
            );
            if (startEdge == null)
                continue;

            float startEdgeCost = ApplyPreviousStartNodeFactor(startEdge.Cost, previousNodeIndices, currentIndex, config);

            float cost =
                NodeCost(firstLayer, currentOffset, config) +
                PreviousPathCost(previousLayerOffsets, 1, currentOffset, config) +
                TransitionShapeCost(
                    startOffset,
                    startOffset,
                    currentOffset,
                    firstSearchLayerIndex * cache.LayerStepSamples,
                    config
                ) +
                startEdgeCost +
                TerminalCost(layers, 1, currentOffset, config) +
                StartHeadingCost(startPosition, startHeading, layers[0], startOffset, firstLayer, currentOffset, 1, config);

            previousStates[previousCount++] = new State(
                PreviousLayerNodeIndex: -1,
                CurrentNodeIndex: currentIndex,
                cost,
                ParentStateIndex: -1
            );
        }

        if (previousCount == 0)
            return SearchResult.NotFound;

        _workspace.StateCounts[firstSearchLayerIndex] = previousCount;

        for (int layerIndex = firstSearchLayerIndex + 1; layerIndex < layers.Length; layerIndex++)
        {
            GraphPlannerLayer previousLayer = layers[layerIndex - 1];
            GraphPlannerLayer currentLayer = layers[layerIndex];
            State[] priorStates = _workspace.StatesByLayer[layerIndex - 1];
            int priorStateCount = _workspace.StateCounts[layerIndex - 1];
            int previousNodeCount = previousLayer.Offsets.Length;
            int currentNodeCount = currentLayer.Offsets.Length;
            GraphPlannerEdgeCosts edgeCosts = cache.GetEdgeCosts(previousLayer.TrackIndex);
            int bestCellCapacity = previousNodeCount * currentNodeCount;
            _workspace.EnsureBestCellCapacity(bestCellCapacity);
            int generation = _workspace.NextBestGeneration();
            int touchedCellCount = 0;

            for (int stateIndex = 0; stateIndex < priorStateCount; stateIndex++)
            {
                State state = priorStates[stateIndex];
                int previousNodeIndex = state.CurrentNodeIndex;
                float previousOffset = previousLayer.Offsets[previousNodeIndex];
                float beforePreviousOffset = state.PreviousLayerNodeIndex < 0
                    ? startOffset
                    : layers[layerIndex - 2].Offsets[state.PreviousLayerNodeIndex];

                int currentStartIndex = edgeCosts.StartIndices[previousNodeIndex];
                int currentEndIndex = edgeCosts.EndIndices[previousNodeIndex];
                for (int currentIndex = currentStartIndex; currentIndex < currentEndIndex; currentIndex++)
                {
                    GraphPlannerEdge? edge = edgeCosts.GetEdge(previousNodeIndex, currentIndex);
                    if (edge == null)
                        continue;

                    float currentOffset = currentLayer.Offsets[currentIndex];
                    float edgeCost = ApplyPreviousEdgeFactor(
                        edge.Cost,
                        previousNodeIndices,
                        layerIndex,
                        previousNodeIndex,
                        currentIndex,
                        config
                    );

                    float cost =
                        state.Cost +
                        NodeCost(currentLayer, currentOffset, config) +
                        PreviousPathCost(previousLayerOffsets, layerIndex, currentOffset, config) +
                        TransitionShapeCost(
                            beforePreviousOffset,
                            previousOffset,
                            currentOffset,
                            cache.LayerStepSamples,
                            config
                        ) +
                        edgeCost +
                        TerminalCost(layers, layerIndex, currentOffset, config) +
                        StartHeadingCost(
                            previousLayer.GetPosition(previousOffset),
                            startHeading,
                            previousLayer,
                            previousOffset,
                            currentLayer,
                            currentOffset,
                            layerIndex,
                            config
                        );

                    int bestCellIndex = previousNodeIndex * currentNodeCount + currentIndex;
                    if (_workspace.BestCellStamps[bestCellIndex] != generation)
                    {
                        _workspace.BestCellStamps[bestCellIndex] = generation;
                        _workspace.BestCosts[bestCellIndex] = InvalidCost;
                        _workspace.BestParents[bestCellIndex] = -1;
                        _workspace.TouchedBestCells[touchedCellCount++] = bestCellIndex;
                    }

                    if (cost >= _workspace.BestCosts[bestCellIndex])
                        continue;

                    _workspace.BestCosts[bestCellIndex] = cost;
                    _workspace.BestParents[bestCellIndex] = stateIndex;
                }
            }

            State[] nextStates = _workspace.EnsureStateLayerCapacity(layerIndex, touchedCellCount);
            int nextStateCount = 0;
            for (int touchedIndex = 0; touchedIndex < touchedCellCount; touchedIndex++)
            {
                int bestCellIndex = _workspace.TouchedBestCells[touchedIndex];
                float cost = _workspace.BestCosts[bestCellIndex];
                if (!float.IsFinite(cost))
                    continue;

                int previousNodeIndex = bestCellIndex / currentNodeCount;
                int currentNodeIndex = bestCellIndex - previousNodeIndex * currentNodeCount;
                nextStates[nextStateCount++] = new State(
                    PreviousLayerNodeIndex: previousNodeIndex,
                    CurrentNodeIndex: currentNodeIndex,
                    cost,
                    ParentStateIndex: _workspace.BestParents[bestCellIndex]
                );
            }

            if (nextStateCount == 0)
                return SearchResult.NotFound;

            _workspace.StateCounts[layerIndex] = nextStateCount;
        }

        int bestStateIndex = 0;
        float bestCost = InvalidCost;
        int lastLayerIndex = layers.Length - 1;
        State[] lastStates = _workspace.StatesByLayer[lastLayerIndex];
        int lastStateCount = _workspace.StateCounts[lastLayerIndex];
        for (int i = 0; i < lastStateCount; i++)
        {
            if (lastStates[i].Cost < bestCost)
            {
                bestCost = lastStates[i].Cost;
                bestStateIndex = i;
            }
        }

        int[] nodeIndices = new int[layers.Length];
        Array.Fill(nodeIndices, -1);
        int stateCursor = bestStateIndex;
        for (int layerIndex = layers.Length - 1; layerIndex >= 1; layerIndex--)
        {
            if (layerIndex < firstSearchLayerIndex)
                break;

            State state = _workspace.StatesByLayer[layerIndex][stateCursor];
            nodeIndices[layerIndex] = state.CurrentNodeIndex;
            stateCursor = state.ParentStateIndex;
        }

        return new SearchResult(true, nodeIndices);
    }

    private static float NodeCost(GraphPlannerLayer layer, float offset, GraphPlannerConfig config)
    {
        if (config.BoundaryWeight <= 0f || config.BoundarySoftMarginMeters <= 0f)
            return 0f;

        float clearance = layer.GetBoundaryClearance(offset);
        if (clearance >= config.BoundarySoftMarginMeters)
            return 0f;

        float normalized = (config.BoundarySoftMarginMeters - Math.Max(0f, clearance)) /
                           config.BoundarySoftMarginMeters;
        return config.BoundaryWeight * Square(normalized);
    }

    private static float TransitionShapeCost(
        float beforePreviousOffset,
        float previousOffset,
        float currentOffset,
        int layerStepSamples,
        GraphPlannerConfig config
    )
    {
        float cost = 0f;
        float longitudinalMeters = Math.Max(layerStepSamples * TrackData.StepLength, TrackData.StepLength);
        if (config.OffsetChangeWeight > 0f)
        {
            float offsetSlope = (currentOffset - previousOffset) / longitudinalMeters;
            cost += config.OffsetChangeWeight * Square(offsetSlope) * longitudinalMeters;
        }

        if (config.OffsetSecondDiffWeight > 0f)
        {
            float secondDiff = currentOffset - 2f * previousOffset + beforePreviousOffset;
            cost += config.OffsetSecondDiffWeight * Square(secondDiff);
        }

        return cost;
    }

    private static float PreviousPathCost(float[] previousLayerOffsets, int layerIndex, float offset, GraphPlannerConfig config)
    {
        if (config.PreviousPathWeight <= 0f || layerIndex < 0 || layerIndex >= previousLayerOffsets.Length)
            return 0f;

        float previousOffset = previousLayerOffsets[layerIndex];
        if (float.IsNaN(previousOffset))
            return 0f;

        return config.PreviousPathWeight * Square(offset - previousOffset);
    }

    private static float ApplyPreviousEdgeFactor(
        float edgeCost,
        int[] previousNodeIndices,
        int layerIndex,
        int previousNodeIndex,
        int currentNodeIndex,
        GraphPlannerConfig config
    )
    {
        if (config.LastEdgeCostFactors.Length == 0 ||
            layerIndex <= 1 ||
            layerIndex >= previousNodeIndices.Length)
        {
            return edgeCost;
        }

        if (previousNodeIndices[layerIndex - 1] != previousNodeIndex ||
            previousNodeIndices[layerIndex] != currentNodeIndex)
        {
            return edgeCost;
        }

        int factorIndex = Math.Clamp(layerIndex - 1, 0, config.LastEdgeCostFactors.Length - 1);
        return edgeCost * Mathf.Clamp(config.LastEdgeCostFactors[factorIndex], 0f, 1f);
    }

    private static float ApplyPreviousStartNodeFactor(
        float edgeCost,
        int[] previousNodeIndices,
        int currentNodeIndex,
        GraphPlannerConfig config
    )
    {
        if (config.LastEdgeCostFactors.Length == 0 ||
            previousNodeIndices.Length <= 1 ||
            previousNodeIndices[1] != currentNodeIndex)
        {
            return edgeCost;
        }

        return edgeCost * Mathf.Clamp(config.LastEdgeCostFactors[0], 0f, 1f);
    }

    private static float TerminalCost(GraphPlannerLayer[] layers, int layerIndex, float offset, GraphPlannerConfig config)
    {
        if (layerIndex != layers.Length - 1 || config.VirtualGoalWeight <= 0f)
            return 0f;

        return config.VirtualGoalWeight * Math.Abs(offset - layers[layerIndex].ReferenceOffset);
    }

    private static float RacingLineDistance(GraphPlannerLayer layer, int nodeIndex, GraphPlannerConfig config)
    {
        return Math.Abs(layer.ReferenceNodeIndex - nodeIndex) * Math.Max(config.LateralResolutionMeters, 0f);
    }

    private static GraphPlannerEdgeCosts BuildEdgeCosts(
        GraphPlannerLayer[] layers,
        int startTrackIndex,
        int layerStepSamples,
        GraphPlannerConfig config
    )
    {
        GraphPlannerLayer previousLayer = layers[startTrackIndex];
        GraphPlannerLayer currentLayer = layers[(startTrackIndex + layerStepSamples) % layers.Length];
        int previousNodeCount = previousLayer.Offsets.Length;
        int currentNodeCount = currentLayer.Offsets.Length;
        float[,] edgeCosts = new float[previousNodeCount, currentNodeCount];
        GraphPlannerEdge?[,] edges = new GraphPlannerEdge[previousNodeCount, currentNodeCount];
        int[] startIndices = new int[previousNodeCount];
        int[] endIndices = new int[previousNodeCount];

        for (int previousNodeIndex = 0; previousNodeIndex < previousNodeCount; previousNodeIndex++)
        {
            float previousOffset = previousLayer.Offsets[previousNodeIndex];
            Vector2 previousPosition = previousLayer.GetPosition(previousOffset);
            int endNodeReference = currentLayer.ReferenceNodeIndex + previousNodeIndex - previousLayer.ReferenceNodeIndex;
            int clampedEndNodeReference = Math.Clamp(endNodeReference, 0, currentNodeCount - 1);
            float referenceDistance = previousPosition.DistanceTo(currentLayer.GetPosition(currentLayer.Offsets[clampedEndNodeReference]));
            int lateralSteps = Math.Max(
                0,
                Mathf.RoundToInt(referenceDistance * Math.Max(config.LateralOffsetPerMeter, 0f) / Math.Max(config.LateralResolutionMeters, 1e-4f))
            );
            int startIndex = currentNodeCount;
            int endIndex = currentNodeCount;
            for (
                int currentNodeIndex = Math.Max(0, endNodeReference - lateralSteps);
                currentNodeIndex < Math.Min(currentNodeCount, endNodeReference + lateralSteps + 1);
                currentNodeIndex++
            )
            {
                float currentOffset = currentLayer.Offsets[currentNodeIndex];
                GraphPlannerEdge? edge = BuildSplineEdge(
                    layers,
                    startTrackIndex,
                    layerStepSamples,
                    previousPosition,
                    previousLayer.GetTangent(previousNodeIndex),
                    currentLayer.GetPosition(currentOffset),
                    currentLayer.GetTangent(currentNodeIndex),
                    config,
                    RacingLineDistance(currentLayer, currentNodeIndex, config),
                    previousNodeIndex == previousLayer.ReferenceNodeIndex &&
                    currentNodeIndex == currentLayer.ReferenceNodeIndex
                );
                if (edge == null)
                    continue;

                if (startIndex == currentNodeCount)
                    startIndex = currentNodeIndex;
                endIndex = currentNodeIndex + 1;

                edges[previousNodeIndex, currentNodeIndex] = edge;
                edgeCosts[previousNodeIndex, currentNodeIndex] = edge.Cost;
            }

            startIndices[previousNodeIndex] = startIndex;
            endIndices[previousNodeIndex] = endIndex;
        }

        return new GraphPlannerEdgeCosts(edgeCosts, startIndices, endIndices, edges);
    }

    private static GraphPlannerEdge? BuildSplineEdge(
        GraphPlannerCache cache,
        int startTrackIndex,
        Vector2 start,
        Vector2 startTangent,
        Vector2 end,
        Vector2 endTangent,
        GraphPlannerConfig config,
        float racingLineDistance = 0f,
        bool isReferenceEdge = false
    )
    {
        return BuildSplineEdge(
            cache,
            startTrackIndex,
            cache.LayerStepSamples,
            start,
            startTangent,
            end,
            endTangent,
            config,
            racingLineDistance,
            isReferenceEdge
        );
    }

    private static GraphPlannerEdge? BuildSplineEdge(
        GraphPlannerCache cache,
        int startTrackIndex,
        int layerStepSamples,
        Vector2 start,
        Vector2 startTangent,
        Vector2 end,
        Vector2 endTangent,
        GraphPlannerConfig config,
        float racingLineDistance = 0f,
        bool isReferenceEdge = false
    )
    {
        GraphPlannerLayer[] layers = new GraphPlannerLayer[layerStepSamples + 1];
        for (int i = 0; i <= layerStepSamples; i++)
            layers[i] = cache.GetLayer(startTrackIndex + i);

        return BuildSplineEdge(
            layers,
            0,
            layerStepSamples,
            start,
            startTangent,
            end,
            endTangent,
            config,
            racingLineDistance,
            isReferenceEdge
        );
    }

    private static GraphPlannerEdge? BuildSplineEdge(
        GraphPlannerLayer[] layers,
        int startTrackIndex,
        int layerStepSamples,
        Vector2 start,
        Vector2 startTangent,
        Vector2 end,
        Vector2 endTangent,
        GraphPlannerConfig config,
        float racingLineDistance = 0f,
        bool isReferenceEdge = false
    )
    {
        if (isReferenceEdge)
            return BuildReferenceLineEdge(layers, startTrackIndex, layerStepSamples, config);

        int outputSamples = Math.Max(1, layerStepSamples);
        float chordLength = start.DistanceTo(end);
        if (chordLength < 1e-4f)
            return null;

        CubicSpline2D spline = CubicSpline2D.FromBoundaryHeadings(start, startTangent, end, endTangent);
        int costSamples = Math.Max(
            2,
            Math.Max(
                config.EdgeCostSamples,
                Mathf.CeilToInt(chordLength / Math.Max(config.SplineSampleStepMeters, 0.1f)) + 1
            )
        );
        Vector2 previous = spline.Position(0f);
        float length = 0f;
        float curvatureSum = 0f;
        float minCurvature = float.PositiveInfinity;
        float maxCurvature = float.NegativeInfinity;
        float maxAbsCurvature = 0f;
        int curvatureCount = 0;

        for (int i = 0; i < costSamples; i++)
        {
            float t = costSamples == 1 ? 0f : i / (float)(costSamples - 1);
            Vector2 current = spline.Position(t);
            if (i > 0)
                length += previous.DistanceTo(current);

            int sampleLayerIndex = Mathf.RoundToInt(t * layerStepSamples);
            GraphPlannerLayer sampleLayer = layers[(startTrackIndex + sampleLayerIndex) % layers.Length];
            float sampleOffset = sampleLayer.ProjectOffset(current);
            if (sampleOffset < sampleLayer.MinOffset - 1e-3f || sampleOffset > sampleLayer.MaxOffset + 1e-3f)
                return null;

            float curvature = spline.Curvature(t);
            if (float.IsFinite(curvature))
            {
                float absCurvature = Math.Abs(curvature);
                curvatureSum += absCurvature;
                minCurvature = Math.Min(minCurvature, curvature);
                maxCurvature = Math.Max(maxCurvature, curvature);
                maxAbsCurvature = Math.Max(maxAbsCurvature, absCurvature);
                curvatureCount++;
            }

            previous = current;
        }

        float[] outputOffsets = new float[outputSamples + 1];
        float[] outputHeadings = new float[outputSamples + 1];
        float[] outputCurvatures = new float[outputSamples + 1];
        for (int i = 0; i <= outputSamples; i++)
        {
            float t = i / (float)outputSamples;
            GraphPlannerLayer layer = layers[(startTrackIndex + i) % layers.Length];
            Vector2 position = spline.Position(t);
            float offset = layer.ProjectOffset(position);
            if (offset < layer.MinOffset - 1e-3f || offset > layer.MaxOffset + 1e-3f)
                return null;

            offset = ClampOffset(offset, layer.MinOffset, layer.MaxOffset);
            outputOffsets[i] = offset;
            outputHeadings[i] = spline.Heading(t);
            outputCurvatures[i] = spline.Curvature(t);
        }

        if (!isReferenceEdge && config.VehicleTurnRadiusMeters > 0f && maxAbsCurvature > 1f / config.VehicleTurnRadiusMeters)
            return null;

        float averageCurvature = curvatureCount == 0 ? 0f : curvatureSum / curvatureCount;
        float curvatureRange = curvatureCount == 0 ? 0f : maxCurvature - minCurvature;
        float cost = EdgeCost(length, averageCurvature, curvatureRange, racingLineDistance, config);

        return new GraphPlannerEdge(spline, outputOffsets, outputHeadings, outputCurvatures, length, cost);
    }

    private static GraphPlannerEdge? BuildReferenceLineEdge(
        GraphPlannerLayer[] layers,
        int startTrackIndex,
        int layerStepSamples,
        GraphPlannerConfig config
    )
    {
        int outputSamples = Math.Max(1, layerStepSamples);
        float[] outputOffsets = new float[outputSamples + 1];
        float[] outputHeadings = new float[outputSamples + 1];
        float[] outputCurvatures = new float[outputSamples + 1];
        Vector2[] positions = new Vector2[outputSamples + 1];

        for (int i = 0; i <= outputSamples; i++)
        {
            GraphPlannerLayer layer = layers[(startTrackIndex + i) % layers.Length];
            outputOffsets[i] = ClampOffset(layer.ReferenceOffset, layer.MinOffset, layer.MaxOffset);
            outputHeadings[i] = layer.GetHeading(layer.ReferenceNodeIndex);
            positions[i] = layer.GetPosition(outputOffsets[i]);
        }

        float length = 0f;
        for (int i = 1; i < positions.Length; i++)
            length += positions[i - 1].DistanceTo(positions[i]);

        if (length <= 1e-4f || !float.IsFinite(length))
            return null;

        float curvatureSum = 0f;
        float minCurvature = float.PositiveInfinity;
        float maxCurvature = float.NegativeInfinity;
        int curvatureCount = 0;
        for (int i = 0; i < outputCurvatures.Length; i++)
        {
            float curvature;
            if (i == 0)
                curvature = outputCurvatures.Length > 2 ? GeomUtil.Curvature(positions[0], positions[1], positions[2]) : 0f;
            else if (i == outputCurvatures.Length - 1)
                curvature = outputCurvatures.Length > 2
                    ? GeomUtil.Curvature(positions[^3], positions[^2], positions[^1])
                    : 0f;
            else
                curvature = GeomUtil.Curvature(positions[i - 1], positions[i], positions[i + 1]);

            if (!float.IsFinite(curvature))
                curvature = 0f;

            outputCurvatures[i] = curvature;
            curvatureSum += Math.Abs(curvature);
            minCurvature = Math.Min(minCurvature, curvature);
            maxCurvature = Math.Max(maxCurvature, curvature);
            curvatureCount++;
        }

        float averageCurvature = curvatureCount == 0 ? 0f : curvatureSum / curvatureCount;
        float curvatureRange = curvatureCount == 0 ? 0f : maxCurvature - minCurvature;
        float cost = EdgeCost(length, averageCurvature, curvatureRange, racingLineDistance: 0f, config);
        CubicSpline2D spline = CubicSpline2D.FromBoundaryHeadings(
            positions[0],
            HeadingVector(outputHeadings[0]),
            positions[^1],
            HeadingVector(outputHeadings[^1])
        );

        return new GraphPlannerEdge(spline, outputOffsets, outputHeadings, outputCurvatures, length, cost);
    }

    private static float EdgeCost(
        float length,
        float averageCurvature,
        float curvatureRange,
        float racingLineDistance,
        GraphPlannerConfig config
    )
    {
        return
            config.EdgeLengthWeight * length +
            config.EdgeAverageCurvatureWeight * Square(averageCurvature) * length +
            config.EdgePeakCurvatureWeight * Square(curvatureRange) * length +
            Math.Min(
                config.RacingLineWeight * length * racingLineDistance,
                config.RacingLineSaturationWeight * length
            );
    }

    private static float StartHeadingCost(
        Vector2 previousPosition,
        float startHeading,
        GraphPlannerLayer previousLayer,
        float previousOffset,
        GraphPlannerLayer currentLayer,
        float currentOffset,
        int layerIndex,
        GraphPlannerConfig config
    )
    {
        if (config.StartHeadingWeight <= 0f || layerIndex > config.StartHeadingLayers)
            return 0f;

        Vector2 from = previousPosition;
        if (layerIndex > 1)
            from = previousLayer.GetPosition(previousOffset);

        Vector2 to = currentLayer.GetPosition(currentOffset);
        Vector2 segment = to - from;
        if (segment.LengthSquared() < 1e-6f)
            return 0f;

        float decay = 1f - (layerIndex - 1f) / Math.Max(config.StartHeadingLayers, 1);
        float angleError = GeomUtil.NormalizeAngle(segment.Angle() - startHeading);
        return config.StartHeadingWeight * decay * Square(angleError);
    }

    private static float[] BuildPreviousLayerOffsets(
        GraphPlannerLayer[] layers,
        GraphPath? previousPath,
        int trackLength,
        GraphPlannerConfig config
    )
    {
        float[] offsets = new float[layers.Length];
        Array.Fill(offsets, float.NaN);

        if (previousPath == null || previousPath.Count == 0 || config.PreviousPathWeight <= 0f || trackLength <= 0)
            return offsets;

        int maxGap = Math.Max(0, config.PreviousPathMaxIndexGapSamples);
        for (int layerIndex = 0; layerIndex < layers.Length; layerIndex++)
        {
            int bestGap = int.MaxValue;
            float bestOffset = float.NaN;
            int targetIndex = layers[layerIndex].TrackIndex;

            for (int pathIndex = 0; pathIndex < previousPath.Count; pathIndex++)
            {
                GraphPathPoint point = previousPath[pathIndex];
                int gap = CircularIndexDistance(targetIndex, point.TrackIndex, trackLength);
                if (gap >= bestGap)
                    continue;

                bestGap = gap;
                bestOffset = point.Offset;
                if (gap == 0)
                    break;
            }

            if (bestGap <= maxGap)
                offsets[layerIndex] = bestOffset;
        }

        return offsets;
    }

    private static int[] BuildPreviousNodeIndices(GraphPlannerLayer[] layers, GraphPath? previousPath)
    {
        int[] nodeIndices = new int[layers.Length];
        Array.Fill(nodeIndices, -1);
        if (previousPath == null || previousPath.Nodes.Length == 0)
            return nodeIndices;

        for (int layerIndex = 0; layerIndex < layers.Length; layerIndex++)
        {
            int targetTrackIndex = layers[layerIndex].TrackIndex;
            for (int previousIndex = 0; previousIndex < previousPath.Nodes.Length; previousIndex++)
            {
                GraphPathNode previousNode = previousPath.Nodes[previousIndex];
                if (previousNode.TrackIndex != targetTrackIndex)
                    continue;

                nodeIndices[layerIndex] = previousNode.NodeIndex;
                break;
            }
        }

        return nodeIndices;
    }

    private bool TryPlanFromPreviousPath(
        TrackData track,
        GraphPlannerCache cache,
        GraphPlannerRequest request,
        GraphPlannerConfig config,
        out GraphPath path
    )
    {
        path = null!;
        GraphPath? previousPath = request.PreviousPath;
        if (previousPath == null || previousPath.IsFallback || previousPath.Count < 2 || previousPath.Nodes.Length == 0)
            return false;

        int nearestSample = FindNearestPathSample(previousPath, request.Position);
        if (config.PreviousPathReuseMaxDistanceMeters > 0f)
        {
            float maxReuseDistanceSquared = Square(config.PreviousPathReuseMaxDistanceMeters);
            if (previousPath[nearestSample].Position.DistanceSquaredTo(request.Position) > maxReuseDistanceSquared)
                return false;
        }

        int minStartSample = Math.Min(
            previousPath.Count - 1,
            nearestSample + Math.Max(1, config.RollingStartLeadSamples)
        );
        if (!TryFindRollingStartNode(previousPath, minStartSample, out GraphPathNode rollingStartNode))
            return false;

        int prefixStartSample = Math.Clamp(nearestSample, 0, previousPath.Count - 1);
        int prefixEndSample = Math.Clamp(rollingStartNode.SampleIndex, prefixStartSample + 1, previousPath.Count - 1);
        GraphPathPoint startPoint = previousPath[prefixEndSample];
        int startIndex = startPoint.TrackIndex;
        GraphPlannerLayer[] layers = BuildLayers(cache, startIndex, config);
        if (layers.Length < 2)
            return false;

        GraphPlannerLayer startLayer = layers[0];
        int startNodeIndex = Math.Clamp(rollingStartNode.NodeIndex, 0, startLayer.Offsets.Length - 1);
        float startOffset = startLayer.Offsets[startNodeIndex];
        float[] previousLayerOffsets = BuildPreviousLayerOffsets(layers, previousPath, track.Length, config);
        int[] previousNodeIndices = BuildPreviousNodeIndices(layers, previousPath);
        SearchResult result = Search(
            layers,
            cache,
            startPoint.Position,
            startPoint.Heading,
            startOffset,
            previousLayerOffsets,
            previousNodeIndices,
            entryLayerIndex: 1,
            config
        );
        if (!result.Found)
            return false;

        GraphPath suffix = BuildPath(
            track,
            cache,
            layers,
            startIndex,
            startPoint.Position,
            startPoint.Heading,
            result,
            previousPath: null,
            config,
            isFallback: false
        );
        if (suffix.IsFallback || suffix.Count < 2)
            return false;

        path = CombineConstantPrefixWithSuffix(previousPath, prefixStartSample, prefixEndSample, suffix, config);
        return true;
    }

    private static int FindNearestPathSample(GraphPath path, Vector2 position)
    {
        int bestIndex = 0;
        float bestDistanceSquared = float.PositiveInfinity;
        for (int i = 0; i < path.Count; i++)
        {
            float distanceSquared = path[i].Position.DistanceSquaredTo(position);
            if (distanceSquared >= bestDistanceSquared)
                continue;

            bestDistanceSquared = distanceSquared;
            bestIndex = i;
        }

        return bestIndex;
    }

    private static bool TryFindRollingStartNode(GraphPath previousPath, int minStartSample, out GraphPathNode node)
    {
        for (int i = 0; i < previousPath.Nodes.Length; i++)
        {
            GraphPathNode candidate = previousPath.Nodes[i];
            if (candidate.SampleIndex < minStartSample || candidate.SampleIndex >= previousPath.Count)
                continue;

            node = candidate;
            return true;
        }

        node = default;
        return false;
    }

    private static GraphPath CombineConstantPrefixWithSuffix(
        GraphPath previousPath,
        int prefixStartSample,
        int prefixEndSample,
        GraphPath suffix,
        GraphPlannerConfig config
    )
    {
        int maxCount = SampleCount(config);
        List<GraphPathPoint> points = [];
        for (int sample = prefixStartSample; sample <= prefixEndSample && points.Count < maxCount; sample++)
            points.Add(ReindexPoint(previousPath[sample], points.Count));

        for (int sample = 1; sample < suffix.Count && points.Count < maxCount; sample++)
            points.Add(ReindexPoint(suffix[sample], points.Count));

        int prefixCount = Math.Max(0, prefixEndSample - prefixStartSample + 1);
        List<GraphPathNode> nodes = [];
        for (int i = 0; i < previousPath.Nodes.Length; i++)
        {
            GraphPathNode previousNode = previousPath.Nodes[i];
            if (previousNode.SampleIndex < prefixStartSample || previousNode.SampleIndex > prefixEndSample)
                continue;

            int adjustedSample = previousNode.SampleIndex - prefixStartSample;
            if (adjustedSample < points.Count)
                nodes.Add(previousNode with { SampleIndex = adjustedSample });
        }

        for (int i = 0; i < suffix.Nodes.Length; i++)
        {
            GraphPathNode suffixNode = suffix.Nodes[i];
            int adjustedSample = prefixCount - 1 + suffixNode.SampleIndex;
            if (adjustedSample <= 0 || adjustedSample >= points.Count)
                continue;

            nodes.Add(suffixNode with { SampleIndex = adjustedSample });
        }

        return new GraphPath([.. points], isFallback: false, [.. nodes]);
    }

    private static GraphPathPoint ReindexPoint(GraphPathPoint point, int sampleIndex)
    {
        return new GraphPathPoint(
            point.TrackIndex,
            sampleIndex * TrackData.StepLength,
            point.Offset,
            point.Position,
            point.Heading,
            point.Curvature,
            point.BoundaryClearance
        );
    }

    private static GraphPath BuildFallbackPath(
        TrackData track,
        GraphPlannerCache cache,
        int startIndex,
        Vector2 startPosition,
        GraphPlannerConfig config
    )
    {
        int sampleCount = SampleCount(config);
        float[] offsets = new float[sampleCount];
        for (int sample = 0; sample < sampleCount; sample++)
        {
            TrackPoint point = track[startIndex + sample];
            float usableShrink = Math.Max(0f, config.VehicleHalfWidthMeters + config.EdgeSafetyMarginMeters);
            float minOffset = -point.HalfWidth + usableShrink;
            float maxOffset = point.HalfWidth - usableShrink;
            offsets[sample] = ClampOffset(cache.GetLayer(point.Index).ReferenceOffset, minOffset, maxOffset);
        }

        float startOffset = (startPosition - track[startIndex].Center).Dot(track[startIndex].Normal);
        offsets[0] = ClampOffset(offsets[0], startOffset, startOffset);
        return BuildPathFromSampleOffsets(track, cache, startIndex, offsets, isFallback: true, nodes: []);
    }

    private static GraphPath BuildPath(
        TrackData track,
        GraphPlannerCache cache,
        GraphPlannerLayer[] layers,
        int startIndex,
        Vector2 startPosition,
        float startHeading,
        SearchResult result,
        GraphPath? previousPath,
        GraphPlannerConfig config,
        bool isFallback
    )
    {
        int sampleCount = SampleCount(config);
        int layerStepSamples = cache.LayerStepSamples;
        float[] sampleOffsets = new float[sampleCount];
        bool[] filledSamples = new bool[sampleCount];
        GraphPathNode[] pathNodes = BuildPathNodes(layers, result, layerStepSamples);
        int firstValidLayerIndex = FirstValidNodeLayer(result);

        if (layers.Length < 2 || firstValidLayerIndex <= 0)
            return BuildFallbackPath(track, cache, startIndex, startPosition, config);

        if (TryBuildRefitPathOffsets(track, cache, layers, startIndex, startPosition, startHeading, result, sampleOffsets, config))
        {
            ApplyConstantPathSegment(track, startIndex, sampleOffsets, previousPath, config);
            return BuildPathFromSampleOffsets(track, cache, startIndex, sampleOffsets, isFallback, pathNodes);
        }

        GraphPlannerLayer firstLayer = layers[firstValidLayerIndex];
        float firstOffset = firstLayer.Offsets[result.NodeIndices[firstValidLayerIndex]];
        GraphPlannerEdge? startEdge = BuildSplineEdge(
            cache,
            layers[0].TrackIndex,
            firstValidLayerIndex * layerStepSamples,
            startPosition,
            HeadingVector(startHeading),
            firstLayer.GetPosition(firstOffset),
            firstLayer.GetTangent(result.NodeIndices[firstValidLayerIndex]),
            config,
            RacingLineDistance(firstLayer, result.NodeIndices[firstValidLayerIndex], config)
        );
        if (startEdge == null)
            return BuildFallbackPath(track, cache, startIndex, startPosition, config);

        CopyEdgeOffsets(startEdge, sampleOffsets, filledSamples, startSample: 0, includeStart: true);

        for (int layerIndex = firstValidLayerIndex; layerIndex < layers.Length - 1; layerIndex++)
        {
            int startNodeIndex = result.NodeIndices[layerIndex];
            int endNodeIndex = result.NodeIndices[layerIndex + 1];
            if (startNodeIndex < 0 || endNodeIndex < 0)
                return BuildFallbackPath(track, cache, startIndex, startPosition, config);

            GraphPlannerEdge? edge = cache.GetEdgeCosts(layers[layerIndex].TrackIndex).GetEdge(startNodeIndex, endNodeIndex);
            if (edge == null)
                return BuildFallbackPath(track, cache, startIndex, startPosition, config);

            CopyEdgeOffsets(
                edge,
                sampleOffsets,
                filledSamples,
                startSample: layerIndex * layerStepSamples,
                includeStart: false
            );
        }

        FillMissingSamples(sampleOffsets, filledSamples);
        ApplyConstantPathSegment(track, startIndex, sampleOffsets, previousPath, config);
        return BuildPathFromSampleOffsets(track, cache, startIndex, sampleOffsets, isFallback, pathNodes);
    }

    private static GraphPathNode[] BuildPathNodes(GraphPlannerLayer[] layers, SearchResult result, int layerStepSamples)
    {
        int count = Math.Min(layers.Length, result.NodeIndices.Length);
        if (count <= 1)
            return [];

        List<GraphPathNode> nodes = [];
        for (int i = 1; i < count; i++)
        {
            if (result.NodeIndices[i] < 0)
                continue;

            nodes.Add(new GraphPathNode(layers[i].TrackIndex, result.NodeIndices[i], i * layerStepSamples));
        }

        return [.. nodes];
    }

    private static int FirstValidNodeLayer(SearchResult result)
    {
        for (int i = 1; i < result.NodeIndices.Length; i++)
        {
            if (result.NodeIndices[i] >= 0)
                return i;
        }

        return -1;
    }

    private static bool TryBuildRefitPathOffsets(
        TrackData track,
        GraphPlannerCache cache,
        GraphPlannerLayer[] layers,
        int startIndex,
        Vector2 startPosition,
        float startHeading,
        SearchResult result,
        float[] sampleOffsets,
        GraphPlannerConfig config
    )
    {
        int controlCount = Math.Min(layers.Length, result.NodeIndices.Length);
        if (controlCount < 2)
            return false;

        float[] controlDistances = new float[controlCount];
        float[] x = new float[controlCount];
        float[] y = new float[controlCount];

        controlDistances[0] = 0f;
        x[0] = startPosition.X;
        y[0] = startPosition.Y;

        float endHeading = startHeading;
        float cumulativeDistance = 0f;
        for (int i = 1; i < controlCount; i++)
        {
            int nodeIndex = result.NodeIndices[i];
            if (nodeIndex < 0 || nodeIndex >= layers[i].Offsets.Length)
                return false;

            float offset = layers[i].Offsets[nodeIndex];
            Vector2 position = layers[i].GetPosition(offset);
            GraphPlannerEdge? edge;
            if (i == 1)
            {
                edge = BuildSplineEdge(
                    cache,
                    layers[0].TrackIndex,
                    startPosition,
                    HeadingVector(startHeading),
                    position,
                    layers[i].GetTangent(nodeIndex),
                    config,
                    RacingLineDistance(layers[i], nodeIndex, config)
                );
            }
            else
            {
                int previousNodeIndex = result.NodeIndices[i - 1];
                if (previousNodeIndex < 0)
                    return false;

                edge = cache.GetEdgeCosts(layers[i - 1].TrackIndex).GetEdge(previousNodeIndex, nodeIndex);
            }

            if (edge == null || edge.Length <= 1e-4f || !float.IsFinite(edge.Length))
                return false;

            cumulativeDistance += edge.Length;
            controlDistances[i] = cumulativeDistance;
            x[i] = position.X;
            y[i] = position.Y;
            endHeading = layers[i].GetHeading(nodeIndex);
        }

        if (!TrySolveClampedSecondDerivatives(
                controlDistances,
                x,
                Mathf.Cos(startHeading),
                Mathf.Cos(endHeading),
                out float[] xSecondDerivatives
            ) ||
            !TrySolveClampedSecondDerivatives(
                controlDistances,
                y,
                Mathf.Sin(startHeading),
                Mathf.Sin(endHeading),
                out float[] ySecondDerivatives
            ))
        {
            return false;
        }

        int segmentIndex = 0;
        for (int sample = 0; sample < sampleOffsets.Length; sample++)
        {
            float distance = Math.Min(sample * TrackData.StepLength, controlDistances[^1]);
            while (segmentIndex < controlDistances.Length - 2 && distance > controlDistances[segmentIndex + 1])
                segmentIndex++;

            Vector2 position = new(
                EvaluateCubicSpline(controlDistances, x, xSecondDerivatives, segmentIndex, distance),
                EvaluateCubicSpline(controlDistances, y, ySecondDerivatives, segmentIndex, distance)
            );
            GraphPlannerLayer layer = cache.GetLayer(track[startIndex + sample].Index);
            float offset = layer.ProjectOffset(position);
            if (offset < layer.MinOffset - 1e-3f || offset > layer.MaxOffset + 1e-3f)
                return false;

            sampleOffsets[sample] = ClampOffset(offset, layer.MinOffset, layer.MaxOffset);
        }

        return true;
    }

    private static void ApplyConstantPathSegment(
        TrackData track,
        int startIndex,
        float[] sampleOffsets,
        GraphPath? previousPath,
        GraphPlannerConfig config
    )
    {
        if (previousPath == null || previousPath.Count == 0 || previousPath.IsFallback || config.ConstantPathMeters <= 0f)
            return;

        int constantSamples = Math.Min(
            sampleOffsets.Length - 1,
            Mathf.RoundToInt(config.ConstantPathMeters / TrackData.StepLength)
        );
        if (constantSamples <= 0)
            return;

        for (int sample = 1; sample <= constantSamples; sample++)
        {
            int targetTrackIndex = track[startIndex + sample].Index;
            int bestGap = int.MaxValue;
            float bestOffset = float.NaN;

            for (int previousIndex = 0; previousIndex < previousPath.Count; previousIndex++)
            {
                GraphPathPoint point = previousPath[previousIndex];
                int gap = CircularIndexDistance(targetTrackIndex, point.TrackIndex, track.Length);
                if (gap >= bestGap)
                    continue;

                bestGap = gap;
                bestOffset = point.Offset;
                if (gap == 0)
                    break;
            }

            if (bestGap <= config.PreviousPathMaxIndexGapSamples && float.IsFinite(bestOffset))
                sampleOffsets[sample] = bestOffset;
        }
    }

    private static GraphPath BuildPathFromSampleOffsets(
        TrackData track,
        GraphPlannerCache cache,
        int startIndex,
        float[] offsets,
        bool isFallback,
        GraphPathNode[] nodes
    )
    {
        GraphPathPoint[] points = new GraphPathPoint[offsets.Length];
        Vector2[] positions = new Vector2[offsets.Length];
        float[] clearances = new float[offsets.Length];

        for (int i = 0; i < offsets.Length; i++)
        {
            TrackPoint trackPoint = track[startIndex + i];
            GraphPlannerLayer layer = cache.GetLayer(trackPoint.Index);
            float offset = ClampOffset(offsets[i], layer.MinOffset, layer.MaxOffset);
            offsets[i] = offset;
            positions[i] = trackPoint.GetOffsetPos(offset);
            clearances[i] = Math.Min(layer.MaxOffset - offset, offset - layer.MinOffset);
        }

        for (int i = 0; i < offsets.Length; i++)
        {
            Vector2 prev = positions[Math.Max(i - 1, 0)];
            Vector2 next = positions[Math.Min(i + 1, offsets.Length - 1)];
            Vector2 tangent = next - prev;
            if (tangent.LengthSquared() < 1e-6f)
                tangent = track[startIndex + i].Tangent;

            float curvature = 0f;
            if (i > 0 && i < offsets.Length - 1)
                curvature = GeomUtil.Curvature(positions[i - 1], positions[i], positions[i + 1]);

            points[i] = new GraphPathPoint(
                track[startIndex + i].Index,
                i * TrackData.StepLength,
                offsets[i],
                positions[i],
                GeomUtil.NormalizeAngle(tangent.Angle()),
                curvature,
                clearances[i]
            );
        }

        return new GraphPath(points, isFallback, nodes);
    }

    private static void CopyEdgeOffsets(
        GraphPlannerEdge edge,
        float[] sampleOffsets,
        bool[] filledSamples,
        int startSample,
        bool includeStart
    )
    {
        int firstEdgeSample = includeStart ? 0 : 1;
        for (int edgeSample = firstEdgeSample; edgeSample < edge.Offsets.Length; edgeSample++)
        {
            int sampleIndex = startSample + edgeSample;
            if (sampleIndex >= sampleOffsets.Length)
                break;

            sampleOffsets[sampleIndex] = edge.Offsets[edgeSample];
            filledSamples[sampleIndex] = true;
        }
    }

    private static bool TrySolveClampedSecondDerivatives(
        float[] distances,
        float[] values,
        float startDerivative,
        float endDerivative,
        out float[] secondDerivatives
    )
    {
        int count = values.Length;
        secondDerivatives = new float[count];
        if (count < 2 || distances.Length != count)
            return false;

        float[] lower = new float[count];
        float[] diagonal = new float[count];
        float[] upper = new float[count];
        float[] rhs = new float[count];

        float firstH = distances[1] - distances[0];
        if (firstH <= 1e-4f)
            return false;

        diagonal[0] = 2f * firstH;
        upper[0] = firstH;
        rhs[0] = 6f * ((values[1] - values[0]) / firstH - startDerivative);

        for (int i = 1; i < count - 1; i++)
        {
            float hPrev = distances[i] - distances[i - 1];
            float hNext = distances[i + 1] - distances[i];
            if (hPrev <= 1e-4f || hNext <= 1e-4f)
                return false;

            lower[i] = hPrev;
            diagonal[i] = 2f * (hPrev + hNext);
            upper[i] = hNext;
            rhs[i] = 6f * ((values[i + 1] - values[i]) / hNext - (values[i] - values[i - 1]) / hPrev);
        }

        float lastH = distances[count - 1] - distances[count - 2];
        if (lastH <= 1e-4f)
            return false;

        lower[count - 1] = lastH;
        diagonal[count - 1] = 2f * lastH;
        rhs[count - 1] = 6f * (endDerivative - (values[count - 1] - values[count - 2]) / lastH);

        for (int i = 1; i < count; i++)
        {
            if (Math.Abs(diagonal[i - 1]) <= 1e-6f)
                return false;

            float factor = lower[i] / diagonal[i - 1];
            diagonal[i] -= factor * upper[i - 1];
            rhs[i] -= factor * rhs[i - 1];
        }

        if (Math.Abs(diagonal[count - 1]) <= 1e-6f)
            return false;

        secondDerivatives[count - 1] = rhs[count - 1] / diagonal[count - 1];
        for (int i = count - 2; i >= 0; i--)
        {
            if (Math.Abs(diagonal[i]) <= 1e-6f)
                return false;

            secondDerivatives[i] = (rhs[i] - upper[i] * secondDerivatives[i + 1]) / diagonal[i];
        }

        return true;
    }

    private static float EvaluateCubicSpline(
        float[] distances,
        float[] values,
        float[] secondDerivatives,
        int segmentIndex,
        float distance
    )
    {
        int safeSegment = Math.Clamp(segmentIndex, 0, values.Length - 2);
        float start = distances[safeSegment];
        float end = distances[safeSegment + 1];
        float h = Math.Max(end - start, 1e-4f);
        float clampedDistance = Mathf.Clamp(distance, start, end);
        float a = (end - clampedDistance) / h;
        float b = (clampedDistance - start) / h;
        return
            a * values[safeSegment] +
            b * values[safeSegment + 1] +
            ((a * a * a - a) * secondDerivatives[safeSegment] +
             (b * b * b - b) * secondDerivatives[safeSegment + 1]) *
            h * h / 6f;
    }

    private static void FillMissingSamples(float[] sampleOffsets, bool[] filledSamples)
    {
        float lastOffset = 0f;
        for (int i = 0; i < sampleOffsets.Length; i++)
        {
            if (!filledSamples[i])
            {
                sampleOffsets[i] = lastOffset;
                continue;
            }

            lastOffset = sampleOffsets[i];
        }
    }

    private static float OffsetLineHeading(TrackData track, int trackIndex, float offset)
    {
        Vector2 prev = track[trackIndex - 1].GetOffsetPos(offset);
        Vector2 next = track[trackIndex + 1].GetOffsetPos(offset);
        Vector2 delta = next - prev;
        return delta.LengthSquared() > 1e-6f
            ? GeomUtil.NormalizeAngle(delta.Angle())
            : track[trackIndex].Tangent.Angle();
    }

    private static float[] BuildNodeHeadings(
        float[] offsets,
        float minOffset,
        float maxOffset,
        float referenceOffset,
        float referenceHeading,
        float leftHeading,
        float rightHeading
    )
    {
        float[] headings = new float[offsets.Length];
        for (int i = 0; i < offsets.Length; i++)
        {
            float offset = offsets[i];
            if (offset <= referenceOffset)
            {
                float denom = Math.Max(referenceOffset - minOffset, 1e-4f);
                float t = Mathf.Clamp((offset - minOffset) / denom, 0f, 1f);
                headings[i] = LerpAngle(leftHeading, referenceHeading, t);
            }
            else
            {
                float denom = Math.Max(maxOffset - referenceOffset, 1e-4f);
                float t = Mathf.Clamp((offset - referenceOffset) / denom, 0f, 1f);
                headings[i] = LerpAngle(referenceHeading, rightHeading, t);
            }
        }

        return headings;
    }

    private static float[] BuildOffsets(float minOffset, float maxOffset, float referenceOffset, float resolution)
    {
        float safeResolution = Math.Max(resolution, 0.05f);
        List<float> offsets = [];
        float first = Mathf.Ceil(minOffset / safeResolution) * safeResolution;
        float last = Mathf.Floor(maxOffset / safeResolution) * safeResolution;

        for (float offset = first; offset <= last + 1e-4f; offset += safeResolution)
            offsets.Add(ClampOffset(offset, minOffset, maxOffset));

        AddUniqueOffset(offsets, ClampOffset(referenceOffset, minOffset, maxOffset), safeResolution * 0.25f);
        if (offsets.Count == 0)
            offsets.Add((minOffset + maxOffset) * 0.5f);

        offsets.Sort();
        return [.. offsets];
    }

    private static void AddUniqueOffset(List<float> offsets, float offset, float tolerance)
    {
        for (int i = 0; i < offsets.Count; i++)
        {
            if (Math.Abs(offsets[i] - offset) <= tolerance)
                return;
        }
        offsets.Add(offset);
    }

    private static int ClosestOffsetIndex(float[] offsets, float target)
    {
        if (offsets.Length == 0)
            return 0;

        int bestIndex = 0;
        float bestError = float.MaxValue;
        for (int i = 0; i < offsets.Length; i++)
        {
            float error = Math.Abs(offsets[i] - target);
            if (error < bestError)
            {
                bestError = error;
                bestIndex = i;
            }
        }

        return bestIndex;
    }

    private static int LayerStepSamples(GraphPlannerConfig config)
    {
        return Math.Max(1, Mathf.RoundToInt(config.LayerStepMeters / TrackData.StepLength));
    }

    private static int SampleCount(GraphPlannerConfig config)
    {
        return Math.Max(2, Mathf.RoundToInt(config.HorizonMeters / TrackData.StepLength) + 1);
    }

    private static int SearchLayerCount(GraphPlannerConfig config)
    {
        return Math.Max(1, Mathf.RoundToInt(config.HorizonMeters / config.LayerStepMeters)) + 1;
    }

    private static float MaxLateralDelta(int layerStepSamples, GraphPlannerConfig config)
    {
        return Math.Max(
            config.LateralResolutionMeters,
            layerStepSamples * TrackData.StepLength * Math.Max(config.LateralOffsetPerMeter, 0f)
        );
    }

    private static float ClampOffset(float offset, float minOffset, float maxOffset)
    {
        return Mathf.Clamp(offset, Math.Min(minOffset, maxOffset), Math.Max(minOffset, maxOffset));
    }

    private static float LerpAngle(float from, float to, float t)
    {
        return GeomUtil.NormalizeAngle(from + GeomUtil.NormalizeAngle(to - from) * Mathf.Clamp(t, 0f, 1f));
    }

    private static Vector2 HeadingVector(float heading)
    {
        return new Vector2(Mathf.Cos(heading), Mathf.Sin(heading));
    }

    private static int CircularIndexDistance(int a, int b, int length)
    {
        int diff = Math.Abs(a - b) % length;
        return Math.Min(diff, length - diff);
    }

    private static float Square(float value) => value * value;

    private readonly record struct State(
        int PreviousLayerNodeIndex,
        int CurrentNodeIndex,
        float Cost,
        int ParentStateIndex
    );

    private readonly record struct SearchResult(bool Found, int[] NodeIndices)
    {
        public static readonly SearchResult NotFound = new(false, []);
    }

    private sealed class SearchWorkspace
    {
        public State[][] StatesByLayer = [];
        public int[] StateCounts = [];
        public float[] BestCosts = [];
        public int[] BestParents = [];
        public int[] BestCellStamps = [];
        public int[] TouchedBestCells = [];
        private int _bestGeneration;

        public void EnsureLayerCapacity(int layerCount)
        {
            if (StatesByLayer.Length < layerCount)
                Array.Resize(ref StatesByLayer, layerCount);
            if (StateCounts.Length < layerCount)
                Array.Resize(ref StateCounts, layerCount);
        }

        public State[] EnsureStateLayerCapacity(int layerIndex, int stateCapacity)
        {
            State[]? states = StatesByLayer[layerIndex];
            if (states == null || states.Length < stateCapacity)
            {
                states = new State[stateCapacity];
                StatesByLayer[layerIndex] = states;
            }

            return states;
        }

        public void EnsureBestCellCapacity(int cellCapacity)
        {
            if (BestCosts.Length < cellCapacity)
            {
                BestCosts = new float[cellCapacity];
                BestParents = new int[cellCapacity];
                BestCellStamps = new int[cellCapacity];
                TouchedBestCells = new int[cellCapacity];
                _bestGeneration = 0;
                return;
            }

            if (TouchedBestCells.Length < cellCapacity)
                Array.Resize(ref TouchedBestCells, cellCapacity);
        }

        public int NextBestGeneration()
        {
            _bestGeneration++;
            if (_bestGeneration < int.MaxValue)
                return _bestGeneration;

            Array.Clear(BestCellStamps);
            _bestGeneration = 1;
            return _bestGeneration;
        }

        public void PrepareForSearch(int layerCount, int maxNodeCount)
        {
            EnsureLayerCapacity(layerCount);
            int maxStateCapacity = Math.Max(1, maxNodeCount * maxNodeCount);
            EnsureBestCellCapacity(maxStateCapacity);
            for (int layerIndex = 1; layerIndex < layerCount; layerIndex++)
                EnsureStateLayerCapacity(layerIndex, maxStateCapacity);
        }
    }

}
