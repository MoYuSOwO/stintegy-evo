using System;

namespace StintegyEVO.Core.Car.Controllers.V1.GraphBased;

public sealed class GraphPath(GraphPathPoint[] points, bool isFallback, GraphPathNode[]? nodes = null)
{
    public readonly GraphPathPoint[] Points = points;
    public readonly bool IsFallback = isFallback;
    public readonly GraphPathNode[] Nodes = nodes ?? [];
    public int Count => Points.Length;

    public GraphPathPoint this[int index]
    {
        get
        {
            int safeIndex = Math.Clamp(index, 0, Points.Length - 1);
            return Points[safeIndex];
        }
    }
}

public readonly record struct GraphPathNode(int TrackIndex, int NodeIndex, int SampleIndex);
