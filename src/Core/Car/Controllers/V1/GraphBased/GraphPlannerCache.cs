using System;
using Godot;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.GraphBased;

public sealed class GraphPlannerCache
{
    internal readonly TrackData Track;
    internal readonly GraphPlannerConfig Config;
    internal readonly int LayerStepSamples;
    internal readonly int MaxNodeCount;
    private readonly GraphPlannerLayer[] _layers;
    private readonly GraphPlannerEdgeCosts[] _edgeCosts;

    internal GraphPlannerCache(
        TrackData track,
        GraphPlannerConfig config,
        int layerStepSamples,
        GraphPlannerLayer[] layers,
        GraphPlannerEdgeCosts[] edgeCosts
    )
    {
        Track = track;
        Config = config;
        LayerStepSamples = layerStepSamples;
        _layers = layers;
        _edgeCosts = edgeCosts;
        for (int i = 0; i < layers.Length; i++)
            MaxNodeCount = Math.Max(MaxNodeCount, layers[i].Offsets.Length);
    }

    internal GraphPlannerLayer GetLayer(int trackIndex)
    {
        int safeIndex = (trackIndex % _layers.Length + _layers.Length) % _layers.Length;
        return _layers[safeIndex];
    }

    internal GraphPlannerEdgeCosts GetEdgeCosts(int trackIndex)
    {
        int safeIndex = (trackIndex % _edgeCosts.Length + _edgeCosts.Length) % _edgeCosts.Length;
        return _edgeCosts[safeIndex];
    }
}

internal sealed class GraphPlannerEdgeCosts(
    float[,] costs,
    int[] startIndices,
    int[] endIndices,
    GraphPlannerEdge?[,] edges
)
{
    public readonly float[,] Costs = costs;
    public readonly int[] StartIndices = startIndices;
    public readonly int[] EndIndices = endIndices;
    private readonly GraphPlannerEdge?[,] _edges = edges;

    public GraphPlannerEdge? GetEdge(int startNodeIndex, int endNodeIndex)
    {
        return _edges[startNodeIndex, endNodeIndex];
    }
}

internal sealed class GraphPlannerEdge(
    CubicSpline2D spline,
    float[] offsets,
    float[] headings,
    float[] curvatures,
    float length,
    float cost
)
{
    public readonly CubicSpline2D Spline = spline;
    public readonly float[] Offsets = offsets;
    public readonly float[] Headings = headings;
    public readonly float[] Curvatures = curvatures;
    public readonly float Length = length;
    public readonly float Cost = cost;
}

internal readonly record struct CubicSpline2D(Vector2 A, Vector2 B, Vector2 C, Vector2 D)
{
    public static CubicSpline2D FromBoundaryHeadings(Vector2 start, Vector2 startTangent, Vector2 end, Vector2 endTangent)
    {
        float length = Math.Max(start.DistanceTo(end), 1e-4f);
        Vector2 dStart = UnitOrDefault(startTangent) * length;
        Vector2 dEnd = UnitOrDefault(endTangent) * length;
        Vector2 delta = end - start;
        return new CubicSpline2D(
            start,
            dStart,
            3f * delta - 2f * dStart - dEnd,
            -2f * delta + dStart + dEnd
        );
    }

    public Vector2 Position(float t)
    {
        float clamped = Mathf.Clamp(t, 0f, 1f);
        float t2 = clamped * clamped;
        float t3 = t2 * clamped;
        return A + B * clamped + C * t2 + D * t3;
    }

    public Vector2 FirstDerivative(float t)
    {
        float clamped = Mathf.Clamp(t, 0f, 1f);
        return B + 2f * C * clamped + 3f * D * clamped * clamped;
    }

    public Vector2 SecondDerivative(float t)
    {
        float clamped = Mathf.Clamp(t, 0f, 1f);
        return 2f * C + 6f * D * clamped;
    }

    public float Heading(float t)
    {
        Vector2 first = FirstDerivative(t);
        return first.LengthSquared() > 1e-8f ? first.Angle() : 0f;
    }

    public float Curvature(float t)
    {
        Vector2 first = FirstDerivative(t);
        float speedSquared = first.LengthSquared();
        if (speedSquared <= 1e-8f)
            return 0f;

        Vector2 second = SecondDerivative(t);
        float cross = first.X * second.Y - first.Y * second.X;
        return cross / Mathf.Pow(speedSquared, 1.5f);
    }

    private static Vector2 UnitOrDefault(Vector2 value)
    {
        return value.LengthSquared() > 1e-8f ? value.Normalized() : Vector2.Right;
    }
}

internal sealed class GraphPlannerLayer(
    int trackIndex,
    Vector2 center,
    Vector2 normal,
    float minOffset,
    float maxOffset,
    float referenceOffset,
    float[] offsets,
    float[] headings,
    int referenceNodeIndex
)
{
    public readonly int TrackIndex = trackIndex;
    private readonly Vector2 _center = center;
    private readonly Vector2 _normal = normal;
    public readonly float MinOffset = minOffset;
    public readonly float MaxOffset = maxOffset;
    public readonly float ReferenceOffset = referenceOffset;
    public readonly float[] Offsets = offsets;
    private readonly float[] _headings = headings;
    public readonly int ReferenceNodeIndex = referenceNodeIndex;

    public Vector2 GetPosition(float offset)
    {
        return _center + _normal * offset;
    }

    public float GetBoundaryClearance(float offset)
    {
        return Math.Max(0f, Math.Min(MaxOffset - offset, offset - MinOffset));
    }

    public float ProjectOffset(Vector2 position)
    {
        return (position - _center).Dot(_normal);
    }

    public float GetHeading(int nodeIndex)
    {
        return _headings[Math.Clamp(nodeIndex, 0, _headings.Length - 1)];
    }

    public float GetHeadingForOffset(float offset)
    {
        if (Offsets.Length == 0)
            return 0f;

        int bestIndex = 0;
        float bestError = float.MaxValue;
        for (int i = 0; i < Offsets.Length; i++)
        {
            float error = Math.Abs(Offsets[i] - offset);
            if (error < bestError)
            {
                bestError = error;
                bestIndex = i;
            }
        }

        return _headings[bestIndex];
    }

    public Vector2 GetTangent(int nodeIndex)
    {
        float heading = GetHeading(nodeIndex);
        return new Vector2(Mathf.Cos(heading), Mathf.Sin(heading));
    }

    public Vector2 GetTangentForOffset(float offset)
    {
        float heading = GetHeadingForOffset(offset);
        return new Vector2(Mathf.Cos(heading), Mathf.Sin(heading));
    }

    public int GetClosestNodeIndex(float offset)
    {
        if (Offsets.Length == 0)
            return 0;

        int bestIndex = 0;
        float bestError = float.MaxValue;
        for (int i = 0; i < Offsets.Length; i++)
        {
            float error = Math.Abs(Offsets[i] - offset);
            if (error < bestError)
            {
                bestError = error;
                bestIndex = i;
            }
        }

        return bestIndex;
    }
}
