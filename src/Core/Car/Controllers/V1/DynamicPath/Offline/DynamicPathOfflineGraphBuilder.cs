using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers.V1.DynamicPath;
using StintegyEVO.Core.Car.Controllers.V1.RacingLines;
using StintegyEVO.Core.Track;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Controllers.V1.DynamicPath.Offline;

public sealed class DynamicPathOfflineGraphBuilder
{
    public DynamicPathOfflineGraph Build(
        TrackData track,
        RacingLine racingLine,
        CarConfig carConfig,
        DynamicPathOfflineConfig? config = null
    )
    {
        ArgumentNullException.ThrowIfNull(track);
        ArgumentNullException.ThrowIfNull(racingLine);
        ArgumentNullException.ThrowIfNull(carConfig);

        config ??= new DynamicPathOfflineConfig();
        config.Validate();

        if (track.Length != racingLine.Count)
            throw new ArgumentException("Racing line must be anchored to every track index.", nameof(racingLine));
        if (track.Length < 4)
            throw new ArgumentException("Track must contain at least four points.", nameof(track));

        float vehicleHalfWidth = carConfig.Chassis.Width * 0.5f;
        float minTurnRadius = CalculateMinTurnRadius(carConfig);
        float[] raceSegmentLengths = CalculateRaceSegmentLengths(racingLine);
        float[] raceDistances = CalculateCumulativeDistances(raceSegmentLengths);
        int[] layerTrackIndexes = SelectLayerTrackIndexes(racingLine, raceSegmentLengths, config);

        DynamicPathLayer[] layers = GenerateLayers(track, racingLine, raceDistances, layerTrackIndexes, vehicleHalfWidth, config);
        List<DynamicPathEdge> edges = GenerateEdges(layers, config, minTurnRadius);
        DynamicPathEdge[] prunedEdges = PruneDeadEndEdges(layers, edges);

        return new DynamicPathOfflineGraph(config, layers, prunedEdges, vehicleHalfWidth, minTurnRadius);
    }

    private static float CalculateMinTurnRadius(CarConfig carConfig)
    {
        float maxSteer = Mathf.Abs(carConfig.Chassis.MaxSteerAngle);
        float tanSteer = Mathf.Tan(maxSteer);
        if (tanSteer <= 1e-5f)
            return float.PositiveInfinity;

        return carConfig.Chassis.WheelBase / tanSteer;
    }

    private static float[] CalculateRaceSegmentLengths(RacingLine racingLine)
    {
        float[] lengths = new float[racingLine.Count];
        for (int i = 0; i < racingLine.Count; i++)
            lengths[i] = Mathf.Max(racingLine[i].Position.DistanceTo(racingLine[i + 1].Position), 1e-4f);
        return lengths;
    }

    private static float[] CalculateCumulativeDistances(float[] segmentLengths)
    {
        float[] distances = new float[segmentLengths.Length];
        for (int i = 1; i < distances.Length; i++)
            distances[i] = distances[i - 1] + segmentLengths[i - 1];
        return distances;
    }

    private static int[] SelectLayerTrackIndexes(RacingLine racingLine, float[] segmentLengths, DynamicPathOfflineConfig config)
    {
        List<int> indexes = [];
        float nextDistance = 0.0f;
        float nextMinimumDistance = 0.0f;
        float currentDistance = 0.0f;

        for (int i = 0; i < racingLine.Count; i++)
        {
            float segmentLength = segmentLengths[i];
            float absCurvature = Mathf.Abs(racingLine[i].Curvature);

            if (currentDistance + segmentLength > nextMinimumDistance && absCurvature > config.CurveThresholdRadPerMeter)
                nextDistance = currentDistance;

            if (currentDistance + segmentLength > nextDistance)
            {
                indexes.Add(i);
                nextDistance += absCurvature < config.CurveThresholdRadPerMeter
                    ? config.LongitudinalStraightStepMeters
                    : config.LongitudinalCurveStepMeters;
                nextMinimumDistance = currentDistance + config.LongitudinalCurveStepMeters;
            }

            currentDistance += segmentLength;
        }

        if (indexes.Count < 2)
            throw new InvalidOperationException("The selected lattice requires at least two layers.");

        return [.. indexes];
    }

    private static DynamicPathLayer[] GenerateLayers(
        TrackData track,
        RacingLine racingLine,
        float[] raceDistances,
        int[] layerTrackIndexes,
        float vehicleHalfWidth,
        DynamicPathOfflineConfig config
    )
    {
        Vector2[] leftBounds = new Vector2[layerTrackIndexes.Length];
        Vector2[] rightBounds = new Vector2[layerTrackIndexes.Length];

        for (int i = 0; i < layerTrackIndexes.Length; i++)
        {
            TrackPoint point = track[layerTrackIndexes[i]];
            leftBounds[i] = point.LeftEdge;
            rightBounds[i] = point.RightEdge;
        }

        DynamicPathLayer[] layers = new DynamicPathLayer[layerTrackIndexes.Length];
        for (int layerIndex = 0; layerIndex < layerTrackIndexes.Length; layerIndex++)
        {
            int trackIndex = layerTrackIndexes[layerIndex];
            TrackPoint point = track[trackIndex];
            RacingLinePoint racePoint = racingLine[trackIndex];
            float usableLeft = point.HalfWidth - vehicleHalfWidth - config.SafetyMarginMeters;
            float usableRight = point.HalfWidth - vehicleHalfWidth - config.SafetyMarginMeters;

            if (usableLeft < 0.0f || usableRight < 0.0f)
                throw new InvalidOperationException("Track is too narrow for the configured vehicle and margin.");

            float referenceOffset = Mathf.Clamp(racePoint.Offset, -usableRight, usableLeft);

            int raceLineNodeIndex = Mathf.FloorToInt((referenceOffset + usableRight) / config.LateralResolutionMeters);
            raceLineNodeIndex = Math.Max(0, raceLineNodeIndex);
            float startOffset = referenceOffset - raceLineNodeIndex * config.LateralResolutionMeters;

            List<float> offsets = [];
            for (float offset = startOffset; offset <= usableLeft + 1e-4f; offset += config.LateralResolutionMeters)
            {
                if (offset >= -usableRight - 1e-4f)
                    offsets.Add(offset);
            }

            if ((uint)raceLineNodeIndex >= (uint)offsets.Count)
                throw new InvalidOperationException("Failed to place a lattice node on the racing line.");

            DynamicPathNode[] nodes = new DynamicPathNode[offsets.Count];
            float rightHeading = HeadingFromClosedPolyline(rightBounds, layerIndex, point.Tangent.Angle());
            float leftHeading = HeadingFromClosedPolyline(leftBounds, layerIndex, point.Tangent.Angle());
            float raceHeading = racePoint.Heading;

            for (int nodeIndex = 0; nodeIndex < offsets.Count; nodeIndex++)
            {
                float offset = offsets[nodeIndex];
                float heading = config.VariableHeading
                    ? InterpolateNodeHeading(nodeIndex, raceLineNodeIndex, offsets.Count, rightHeading, raceHeading, leftHeading)
                    : raceHeading;

                nodes[nodeIndex] = new DynamicPathNode(
                    id: new DynamicPathNodeId(layerIndex, nodeIndex),
                    position: point.GetOffsetPos(offset),
                    heading: heading,
                    offset: offset,
                    isRaceLine: nodeIndex == raceLineNodeIndex
                );
            }

            layers[layerIndex] = new DynamicPathLayer(layerIndex, trackIndex, raceDistances[trackIndex], raceLineNodeIndex, nodes);
        }

        return layers;
    }

    private static List<DynamicPathEdge> GenerateEdges(
        DynamicPathLayer[] layers,
        DynamicPathOfflineConfig config,
        float minTurnRadius
    )
    {
        List<DynamicPathEdge> edges = [];
        float maxCurvature = float.IsFinite(minTurnRadius) && minTurnRadius > 1e-4f
            ? 1.0f / minTurnRadius
            : 0.0f;
        DynamicPathSplineSegment[] raceLineSplines = CreateRaceLineSplines(layers);

        for (int layerIndex = 0; layerIndex < layers.Length; layerIndex++)
        {
            DynamicPathLayer startLayer = layers[layerIndex];
            DynamicPathLayer endLayer = layers[(layerIndex + 1) % layers.Length];

            for (int startNodeIndex = 0; startNodeIndex < startLayer.Nodes.Length; startNodeIndex++)
            {
                DynamicPathNode startNode = startLayer.Nodes[startNodeIndex];
                int endNodeReference = endLayer.RaceLineNodeIndex + startNodeIndex - startLayer.RaceLineNodeIndex;
                int clippedReference = Math.Clamp(endNodeReference, 0, endLayer.Nodes.Length - 1);
                float centerDistance = startNode.Position.DistanceTo(endLayer.Nodes[clippedReference].Position);
                int lateralSteps = Mathf.RoundToInt(centerDistance * config.LateralOffsetPerMeter / config.LateralResolutionMeters);

                int firstEndNode = Math.Max(0, endNodeReference - lateralSteps);
                int lastEndNode = Math.Min(endLayer.Nodes.Length - 1, endNodeReference + lateralSteps);

                for (int endNodeIndex = firstEndNode; endNodeIndex <= lastEndNode; endNodeIndex++)
                {
                    DynamicPathNode endNode = endLayer.Nodes[endNodeIndex];
                    bool isRaceLineEdge = startNode.IsRaceLine && endNode.IsRaceLine;
                    DynamicPathSplineSegment spline = isRaceLineEdge
                        ? raceLineSplines[layerIndex]
                        : DynamicPathSplineMath.CreateLengthRefinedUnclosedSegment(startNode.Position, endNode.Position, startNode.Heading, endNode.Heading);
                    DynamicPathEdgeSample[] samples = SampleSpline(spline, config.EdgeSampleStepMeters, out float length);

                    if (!isRaceLineEdge && !SatisfiesTurnRadius(samples, maxCurvature))
                        continue;

                    float offlineCost = CalculateOfflineCost(
                        samples,
                        length,
                        Math.Abs(endNodeIndex - endLayer.RaceLineNodeIndex) * config.LateralResolutionMeters,
                        config.CostWeights
                    );

                    edges.Add(new DynamicPathEdge(startNode.Id, endNode.Id, spline, samples, length, offlineCost, isRaceLineEdge));
                }
            }
        }

        return edges;
    }

    private static DynamicPathSplineSegment[] CreateRaceLineSplines(DynamicPathLayer[] layers)
    {
        Vector2[] raceLinePoints = new Vector2[layers.Length];
        for (int i = 0; i < layers.Length; i++)
            raceLinePoints[i] = layers[i].Nodes[layers[i].RaceLineNodeIndex].Position;
        return DynamicPathSplineMath.CreateClosed(raceLinePoints);
    }

    private static DynamicPathEdgeSample[] SampleSpline(
        DynamicPathSplineSegment spline,
        float sampleStep,
        out float length
    )
    {
        float estimatedLength = DynamicPathSplineMath.EstimateLength(spline);
        int sampleCount = Math.Max(2, Mathf.CeilToInt(Mathf.Max(estimatedLength, sampleStep) / sampleStep) + 1);
        Vector2[] positions = new Vector2[sampleCount];
        float[] headings = new float[sampleCount];
        float[] curvatures = new float[sampleCount];

        for (int i = 0; i < sampleCount; i++)
        {
            float t = i / (float)(sampleCount - 1);
            positions[i] = spline.Evaluate(t);
            Vector2 derivative = spline.Derivative(t);
            Vector2 secondDerivative = spline.SecondDerivative(t);
            headings[i] = derivative.LengthSquared() > 1e-8f ? derivative.Angle() : 0.0f;

            float speedSq = derivative.LengthSquared();
            float denominator = Mathf.Pow(speedSq, 1.5f);
            curvatures[i] = denominator > 1e-8f
                ? derivative.Cross(secondDerivative) / denominator
                : 0.0f;
        }

        DynamicPathEdgeSample[] samples = new DynamicPathEdgeSample[sampleCount];
        length = 0.0f;
        for (int i = 0; i < sampleCount; i++)
        {
            float lengthToNext = 0.0f;
            if (i < sampleCount - 1)
            {
                lengthToNext = positions[i].DistanceTo(positions[i + 1]);
                length += lengthToNext;
            }

            samples[i] = new DynamicPathEdgeSample(positions[i], headings[i], curvatures[i], lengthToNext);
        }

        return samples;
    }

    private static bool SatisfiesTurnRadius(DynamicPathEdgeSample[] samples, float maxCurvature)
    {
        if (maxCurvature <= 0.0f)
            return false;

        for (int i = 0; i < samples.Length; i++)
        {
            if (Mathf.Abs(samples[i].Curvature) > maxCurvature + 1e-4f)
                return false;
        }

        return true;
    }

    private static float CalculateOfflineCost(
        DynamicPathEdgeSample[] samples,
        float length,
        float racelineDistance,
        DynamicPathCostWeights weights
    )
    {
        float curvatureAbsSum = 0.0f;
        float curvatureMin = float.PositiveInfinity;
        float curvatureMax = float.NegativeInfinity;

        for (int i = 0; i < samples.Length; i++)
        {
            float curvature = samples[i].Curvature;
            curvatureAbsSum += Mathf.Abs(curvature);
            curvatureMin = Mathf.Min(curvatureMin, curvature);
            curvatureMax = Mathf.Max(curvatureMax, curvature);
        }

        float averageAbsCurvature = curvatureAbsSum / samples.Length;
        float curvatureRange = curvatureMax - curvatureMin;

        float cost = 0.0f;
        cost += weights.CurvatureAverage * averageAbsCurvature * averageAbsCurvature * length;
        cost += weights.CurvaturePeak * curvatureRange * curvatureRange * length;
        cost += weights.Length * length;
        cost += Mathf.Min(weights.Raceline * length * racelineDistance, weights.RacelineSaturation * length);
        return cost;
    }

    private static DynamicPathEdge[] PruneDeadEndEdges(DynamicPathLayer[] layers, List<DynamicPathEdge> edges)
    {
        int[] layerStarts = new int[layers.Length];
        int nodeCount = 0;
        for (int i = 0; i < layers.Length; i++)
        {
            layerStarts[i] = nodeCount;
            nodeCount += layers[i].Nodes.Length;
        }

        bool[] active = new bool[edges.Count];
        Array.Fill(active, true);

        bool changed;
        do
        {
            changed = false;
            bool[] hasIncoming = new bool[nodeCount];
            bool[] hasOutgoing = new bool[nodeCount];

            for (int i = 0; i < edges.Count; i++)
            {
                if (!active[i])
                    continue;

                DynamicPathEdge edge = edges[i];
                hasOutgoing[GlobalNodeIndex(edge.From, layerStarts)] = true;
                hasIncoming[GlobalNodeIndex(edge.To, layerStarts)] = true;
            }

            bool[] deadNode = new bool[nodeCount];
            for (int i = 0; i < nodeCount; i++)
                deadNode[i] = !hasIncoming[i] || !hasOutgoing[i];

            for (int i = 0; i < edges.Count; i++)
            {
                if (!active[i])
                    continue;

                DynamicPathEdge edge = edges[i];
                if (deadNode[GlobalNodeIndex(edge.From, layerStarts)] || deadNode[GlobalNodeIndex(edge.To, layerStarts)])
                {
                    active[i] = false;
                    changed = true;
                }
            }
        } while (changed);

        List<DynamicPathEdge> pruned = [];
        for (int i = 0; i < edges.Count; i++)
        {
            if (active[i])
                pruned.Add(edges[i]);
        }

        return [.. pruned];
    }

    private static int GlobalNodeIndex(DynamicPathNodeId id, int[] layerStarts)
    {
        return layerStarts[id.Layer] + id.Node;
    }

    private static float InterpolateNodeHeading(
        int nodeIndex,
        int raceLineNodeIndex,
        int nodeCount,
        float rightHeading,
        float raceHeading,
        float leftHeading
    )
    {
        if (nodeIndex <= raceLineNodeIndex)
        {
            float t = raceLineNodeIndex == 0 ? 1.0f : nodeIndex / (float)raceLineNodeIndex;
            return LerpAngle(rightHeading, raceHeading, t);
        }

        int leftSpan = nodeCount - raceLineNodeIndex - 1;
        float leftT = leftSpan == 0 ? 1.0f : (nodeIndex - raceLineNodeIndex) / (float)leftSpan;
        return LerpAngle(raceHeading, leftHeading, leftT);
    }

    private static float HeadingFromClosedPolyline(Vector2[] points, int index, float fallback)
    {
        Vector2 prev = points[(index - 1 + points.Length) % points.Length];
        Vector2 next = points[(index + 1) % points.Length];
        Vector2 tangent = next - prev;
        return tangent.LengthSquared() > 1e-8f ? tangent.Angle() : fallback;
    }

    private static float LerpAngle(float from, float to, float t)
    {
        return GeomUtil.NormalizeAngle(from + GeomUtil.NormalizeAngle(to - from) * Mathf.Clamp(t, 0.0f, 1.0f));
    }
}
