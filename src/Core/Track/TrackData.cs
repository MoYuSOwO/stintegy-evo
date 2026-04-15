using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using Godot;
using PloyRacing.Util;

namespace PloyRacing.Core.Track;

public struct TrackGridConfig
{
    public int StartingLineIdx;
    public int GridCount;
    public float GridOffset;
    public int FirstGridIdx;
    public bool IsFirstGridLeft;
    public int GridStepDist;

    public const float GridLength = 4.5f;
    public const float GridWidth = 2.4f;
}

public readonly struct TrackPoint
{
    private readonly TrackNode _node;
    public readonly int Index;
    public readonly Vector2 Center => _node.Center;
    public readonly Vector2 Tangent => _node.Tangent;
    public readonly float Width => _node.Width;
    public readonly float LeftBufferWidth => _node.LeftBufferWidth;
    public readonly float RightBufferWidth => _node.RightBufferWidth;
    public readonly float HalfWidth => _node.HalfWidth;
    public readonly Vector2 Normal => _node.Normal;
    public readonly Vector2 LeftEdge => _node.LeftEdge;
    public readonly Vector2 RightEdge => _node.RightEdge;
    public readonly Vector2 LeftBufferEdge => Center + Normal * (HalfWidth + LeftBufferWidth);
    public readonly Vector2 RightBufferEdge => Center + Normal * (HalfWidth + RightBufferWidth);
    public readonly float OptimalOffset;
    public readonly Vector2 Optimal => _node.GetOffsetPos(OptimalOffset);
    public readonly float Friction;

    public Vector2 GetOffsetPos(float offset)
    {
        return Center + Normal * offset;
    }

    internal TrackPoint(int index, TrackNode node, float optimalOffset, float friction)
    {
        _node = node;
        Index = index;
        OptimalOffset = optimalOffset;
        Friction = friction;
    }
}

public readonly struct Grid
{
    public readonly int GridPos;
    public readonly Vector2 Position;
    public readonly int Index;

    internal Grid(int gridPos, Vector2 position, int index)
    {
        GridPos = gridPos;
        Position = position;
        Index = index;
    }
}

public readonly struct StartingGridAccessor(TrackData data)
{
    public Grid this[int gridPos] => new(gridPos, GetPosition(gridPos), GetNodeIndex(gridPos));

    public Vector2 GetPosition(int gridPos)
    {
        var config = data.GridConfig;
        bool left = config.IsFirstGridLeft ? gridPos % 2 == 1 : gridPos % 2 == 0;
        float offset = left ? config.GridOffset : -config.GridOffset;
        
        int nodeIdx = config.FirstGridIdx - config.GridStepDist * (gridPos - 1);
        return data[nodeIdx].GetOffsetPos(offset);
    }

    public int GetNodeIndex(int gridPos)
    {
        var config = data.GridConfig;
        return (config.FirstGridIdx - (gridPos - 1) * config.GridStepDist) % data.Length;
    }
}

public class TrackData
{
    public const float StepLength = 1.0f;
    public const float BaseFriction = 1.0f;
    public const float SafeMargin = 1.2f;

    private const float CellSize = 5.0f;
    private readonly Dictionary<long, List<int>> spatialBuckets = [];

    private readonly ImmutableArray<TrackNode> Nodes;
    private readonly ImmutableArray<float> OptimalLines;
    public float FrictionMultiplier { get; set; } = 1.0f;
    public float Friction => BaseFriction * FrictionMultiplier;

    public int Length => Nodes.Length;
    public readonly TrackGridConfig GridConfig;
    public TrackPoint this[int index] 
    {
        get
        {
            int safeIdx = (index % Length + Length) % Length;
            return new TrackPoint(safeIdx, Nodes[safeIdx], OptimalLines[safeIdx], Friction);
        }
    }
    public readonly StartingGridAccessor Grids;

    internal TrackData(IList<TrackNode> nodes, TrackGridConfig gridConfig)
    {
        Nodes = [.. nodes];
        Grids = new(this);
        OptimalLines = TrackLineSolver.GenerateOptimalLines(nodes, SafeMargin);
        GridConfig = gridConfig;
        BuildSpatialHash();
    }

    private void BuildSpatialHash()
    {
        for (int i = 0; i < Nodes.Length; i++)
        {
            long key = GetKey(Nodes[i].Center);
            if (!spatialBuckets.ContainsKey(key))
                spatialBuckets[key] = [];
            
            spatialBuckets[key].Add(i);
        }
    }

    private static long GetKey(Vector2 pos)
    {
        long x = (long)Math.Floor(pos.X / CellSize);
        long y = (long)Math.Floor(pos.Y / CellSize);

        return (x << 32) | (y & 0xFFFFFFFFL);
    }

    public int FindNearestIndex(Vector2 pos)
    {
        long key = GetKey(pos);
        float minDistSq = float.MaxValue;
        int bestIdx = 0;

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                long neighborKey = key + ((long)x << 32) + (y & 0xFFFFFFFFL);

                if (spatialBuckets.TryGetValue(neighborKey, out var nodeIndices))
                {
                    foreach (int idx in nodeIndices)
                    {
                        float distSq = (pos - Nodes[idx].Center).LengthSquared();
                        if (distSq < minDistSq)
                        {
                            minDistSq = distSq;
                            bestIdx = idx;
                        }
                    }
                }
            }
        }
        return bestIdx;
    }
    
}