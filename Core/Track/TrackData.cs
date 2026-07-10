using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Numerics;
using TheStint.Core.Util;

namespace TheStint.Core.Track;

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

public readonly struct Grid
{
    public readonly int GridPos;
    public readonly float S;
    public readonly Vector2 Position;

    internal Grid(int gridPos, float s, Vector2 position)
    {
        GridPos = gridPos;
        S = s;
        Position = position;
    }
}

public readonly struct TrackSample
{
    public readonly float S;
    public readonly Vector2 Center;
    public readonly Vector2 Tangent;
    public readonly Vector2 Normal;
    public readonly float Width;
    public readonly float LeftBufferWidth;
    public readonly float RightBufferWidth;
    public readonly float RefOffset;
    public readonly Vector2 RefPosition;
    public readonly float RefHeading;
    public readonly float RefCurvature;

    public float HalfWidth => Width * 0.5f;
    public Vector2 LeftEdge => Center + Normal * HalfWidth;
    public Vector2 RightEdge => Center - Normal * HalfWidth;
    public Vector2 LeftSpace => LeftEdge + Normal * LeftBufferWidth;
    public Vector2 RightSpace => RightEdge - Normal * RightBufferWidth;

    internal TrackSample(
        float s,
        Vector2 center,
        Vector2 tangent,
        float width,
        float leftBufferWidth,
        float rightBufferWidth,
        float refOffset,
        Vector2 refPosition,
        float refHeading,
        float refCurvature
    )
    {
        S = s;
        Center = center;
        Tangent = tangent;
        Normal = new(Tangent.Y, -Tangent.X);
        Width = width;
        LeftBufferWidth = leftBufferWidth;
        RightBufferWidth = rightBufferWidth;
        RefOffset = refOffset;
        RefPosition = refPosition;
        RefHeading = refHeading;
        RefCurvature = refCurvature;
    }
}

public readonly struct TrackPose
{
    public readonly float S;
    public readonly float D;
    public readonly TrackSample Sample;

    internal TrackPose(float s, float d, TrackSample sample)
    {
        S = s;
        D = d;
        Sample = sample;
    }
}

public sealed class StartingGridAccessor
{
    private readonly TrackData _data;

    internal StartingGridAccessor(TrackData data)
    {
        _data = data;
    }

    public Grid this[int gridPos]
    {
        get
        {
            float s = GetS(gridPos);
            return new Grid(gridPos, s, GetPosition(gridPos));
        }
    }

    public Vector2 GetPosition(int gridPos)
    {
        var config = _data.GridConfig;
        bool left = config.IsFirstGridLeft ? gridPos % 2 == 1 : gridPos % 2 == 0;
        float offset = left ? config.GridOffset : -config.GridOffset;

        TrackSample sample = _data.Sample(GetS(gridPos));
        return sample.Center + sample.Normal * offset;
    }

    public float GetS(int gridPos)
    {
        var config = _data.GridConfig;
        float s = (config.FirstGridIdx - (gridPos - 1) * config.GridStepDist) * TrackData.StepLength;
        return _data.WrapS(s);
    }
}

public class TrackData
{
    public const float StepLength = 1.0f;
    public const float BaseFriction = 1.0f;

    private readonly float cellSize;
    private readonly Dictionary<long, List<int>> spatialBuckets = [];

    private readonly ImmutableArray<TrackNode> Nodes;
    public float FrictionMultiplier { get; set; } = 1.0f;
    public float Friction => BaseFriction * FrictionMultiplier;

    internal int Length => Nodes.Length;
    public float LengthMeters => Length * StepLength;
    public float StartingLineS => WrapS(GridConfig.StartingLineIdx * StepLength);
    public int StartingGridCount => Math.Max(0, GridConfig.GridCount);
    internal readonly TrackGridConfig GridConfig;
    private TrackNode this[int index]
    {
        get
        {
            int safeIdx = (index % Length + Length) % Length;
            return Nodes[safeIdx];
        }
    }
    public readonly StartingGridAccessor Grids;

    internal TrackData(IReadOnlyList<TrackNode> nodes, TrackGridConfig gridConfig)
    {
        Nodes = [.. nodes];
        Grids = new(this);
        GridConfig = gridConfig;

        float maxWidth = 0.0f;
        for (int i = 0; i < Length; i++)
        {
            maxWidth = MathF.Max(this[i].Width, maxWidth);
        }
        cellSize = maxWidth * 0.7f;
        BuildSpatialHash();
    }

    public TrackSample Sample(float s)
    {
        float wrappedS = WrapS(s);
        float scaled = wrappedS / StepLength;
        int index = (int)MathF.Floor(scaled);
        float t = scaled - index;

        TrackNode a = this[index];
        TrackNode b = this[index + 1];

        Vector2 tangent = Vector2.Lerp(a.Tangent, b.Tangent, t);
        if (tangent.LengthSquared() < 1e-8f)
            tangent = a.Tangent;
        tangent = Vector2.Normalize(tangent);

        return new TrackSample(
            wrappedS,
            Vector2.Lerp(a.Center, b.Center, t),
            tangent,
            Lerp(a.Width, b.Width, t),
            Lerp(a.LeftBufferWidth, b.LeftBufferWidth, t),
            Lerp(a.RightBufferWidth, b.RightBufferWidth, t),
            Lerp(a.RefOffset, b.RefOffset, t),
            Vector2.Lerp(a.Ref, b.Ref, t),
            LerpAngle(a.RefLinePoint.Heading, b.RefLinePoint.Heading, t),
            Lerp(a.RefLinePoint.Curvature, b.RefLinePoint.Curvature, t)
        );
    }

    public TrackPose Project(Vector2 pos)
    {
        int nearestIndex = FindNearestNodeIndex(pos);
        float bestS = nearestIndex * StepLength;
        float minDistSq = float.MaxValue;

        for (int offset = -2; offset <= 2; offset++)
        {
            int segmentIndex = WrapIndex(nearestIndex + offset, Length);
            Vector2 a = Nodes[segmentIndex].Center;
            Vector2 b = Nodes[WrapIndex(segmentIndex + 1, Length)].Center;
            Vector2 ab = b - a;
            float lenSq = ab.LengthSquared();
            if (lenSq < 1e-8f)
                continue;

            float t = Math.Clamp(Vector2.Dot(pos - a, ab) / lenSq, 0f, 1f);
            Vector2 projected = a + ab * t;
            float distSq = (pos - projected).LengthSquared();
            if (distSq >= minDistSq)
                continue;

            minDistSq = distSq;
            bestS = (segmentIndex + t) * StepLength;
        }

        TrackSample sample = Sample(bestS);
        float d = Vector2.Dot(pos - sample.Center, sample.Normal);
        return new TrackPose(sample.S, d, sample);
    }

    public float WrapS(float s)
    {
        float lengthMeters = LengthMeters;
        float wrapped = s % lengthMeters;
        return wrapped < 0f ? wrapped + lengthMeters : wrapped;
    }

    internal static int WrapIndex(int index, int length)
    {
        return (index % length + length) % length;
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

    private static long GetCellKey(long cellX, long cellY)
    {
        return (cellX << 32) | (cellY & 0xFFFFFFFFL);
    }

    private long GetKey(Vector2 pos)
    {
        long x = (long)Math.Floor(pos.X / cellSize);
        long y = (long)Math.Floor(pos.Y / cellSize);
        return GetCellKey(x, y);
    }

    private int FindNearestNodeIndex(Vector2 pos)
    {
        long baseX = (long)Math.Floor(pos.X / cellSize);
        long baseY = (long)Math.Floor(pos.Y / cellSize);

        float minDistSq = float.MaxValue;
        int bestIdx = 0;

        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                long neighborX = baseX + x;
                long neighborY = baseY + y;

                long neighborKey = GetCellKey(neighborX, neighborY);

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

    private static float Lerp(float from, float to, float weight)
    {
        return from + (to - from) * weight;
    }

    private static float LerpAngle(float from, float to, float weight)
    {
        float delta = MathHelper.NormalizeAngle(to - from);
        return MathHelper.NormalizeAngle(from + delta * weight);
    }

}
