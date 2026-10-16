using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Numerics;

namespace StintegyEVO.Core.Track;

/// <summary>
/// The four things about a starting grid that belong to the circuit, and
/// nothing else.
/// </summary>
/// <remarks>
/// Where the line is, how many boxes are painted, which side pole sits on and
/// how far out the columns stand are facts a circuit's author knows and no
/// geometry can recover: a road does not say where somebody chose to paint a
/// line, counting how many boxes would fit is not the same question as how
/// many are there, and how far apart two columns of cars stand depends on how
/// wide the road is where they stand — ten and a half metres at Monaco
/// against seventeen at Sepang.
///
/// Everything below those — the spacing along the road, the setback, the size
/// of a box — is the same on every circuit in the world, so it is written
/// once here rather than copied into eleven places where the copies can drift.
/// Nothing in this file checks that an authored value is sensible: a circuit
/// that wants its grid in the middle of a corner is entitled to one.
/// </remarks>
public struct TrackGridConfig
{
    /// <summary>Which centreline node the start/finish line crosses.</summary>
    public int StartingLineIdx;

    /// <summary>How many boxes this circuit paints. A fact, not a capacity.</summary>
    public int GridCount;

    /// <summary>Whether pole is on the left of the centreline.</summary>
    public bool IsFirstGridLeft;

    /// <summary>
    /// Half the lateral stagger: pole stands this far one side of the
    /// centreline and second this far the other. How wide the road is at the
    /// grid sets the ceiling, which is why it is authored and not a constant.
    /// </summary>
    public float GridOffset;

    public const float GridLength = 4.5f;
    public const float GridWidth = 2.4f;

    /// <summary>
    /// Box to box along the road. Consecutive boxes alternate sides, so the
    /// gap between two cars in the same column is sixteen metres, which is
    /// the Grand Prix figure.
    /// </summary>
    public const int GridStepMeters = 8;

    /// <summary>How far behind the line pole sits.</summary>
    public const int PoleSetbackMeters = 10;
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
    /// <summary>Signed curvature of the physical road centreline.</summary>
    public readonly float Curvature;

    /// <summary>Heading of the physical road centreline in radians.</summary>
    public readonly float Heading;

    /// <summary>Rise over run along the direction of travel; positive uphill.</summary>
    public readonly float Grade;

    /// <summary>
    /// Cross slope at the centreline, and how that slope itself changes
    /// across the road. Together they describe the section as
    /// <c>z(d) = z0 + BankSlope*d + BankCurvature*d^2</c>, which is the
    /// cheapest form that still covers what real circuits do: a constant
    /// bank, a crown shedding water off both edges, and progressive banking
    /// where the outside is steeper than the inside. A single bank angle per
    /// point would force every section to be a straight line and could
    /// represent none of the last two.
    /// </summary>
    public readonly float BankSlope;
    public readonly float BankCurvature;

    /// <summary>
    /// How sharply the road bends in the vertical plane, positive into a
    /// compression and negative over a crest. Multiplied by the square of
    /// the speed it is the extra load the tarmac carries, or fails to.
    /// </summary>
    public readonly float VerticalRate;

    /// <summary>
    /// The cross slope actually under a car at lateral offset <paramref name="d"/>.
    /// On a progressively banked corner this is what makes the high line a
    /// real choice: more bank, more grip, longer way round.
    /// </summary>
    public float BankSlopeAt(float d) => BankSlope + 2f * BankCurvature * d;

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
        float curvature,
        float grade = 0f,
        float bankSlope = 0f,
        float bankCurvature = 0f,
        float verticalRate = 0f
    )
    {
        S = s;
        Center = center;
        Tangent = tangent;
        Normal = new(Tangent.Y, -Tangent.X);
        Width = width;
        LeftBufferWidth = leftBufferWidth;
        RightBufferWidth = rightBufferWidth;
        Curvature = curvature;
        Heading = MathF.Atan2(tangent.Y, tangent.X);
        Grade = grade;
        BankSlope = bankSlope;
        BankCurvature = bankCurvature;
        VerticalRate = verticalRate;
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
        float s = (
            config.StartingLineIdx
            - TrackGridConfig.PoleSetbackMeters
            - (gridPos - 1) * TrackGridConfig.GridStepMeters
        ) * TrackData.StepLength;
        return _data.WrapS(s);
    }
}

/// <summary>
/// The physical road a car can run on: a sampled centreline with its tangent,
/// normal and curvature; widths, edges and run-off; elevation, slope and
/// banking; surfaces and their grip; projection of world positions to
/// <c>(s, d)</c>, wrapping, the starting line and the grid.
/// </summary>
/// <remarks>
/// The centreline is a geometric reference, not the right answer: the same road
/// admits any line a controller chooses, and no preferred, offset or reference
/// line, curvature objective or path planner belongs here. What happens at the
/// edge is priced by surfaces, grip and walls, not by a hidden off-track flag;
/// judging fault or conduct is a rules layer's business outside the world.
/// </remarks>
public class TrackData
{
    public const float StepLength = 1.0f;

    /// <summary>
    /// How far past a wall a queried position can legitimately be: the
    /// corner of a car pressed against the barrier, which the wall resolver
    /// projects in order to push it back, and a predicted pose one step
    /// ahead of it. A car's half-diagonal is about 2.6 m and a step at
    /// 100 m/s covers 1.7 m; five metres covers both.
    /// </summary>
    public const float BodyReachMeters = 5.0f;
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

        // The world is closed by its walls: no position a car can occupy is
        // further from the centreline than half the road plus the wider
        // run-off, and no point of a car is further than that plus a body
        // (BodyReachMeters). The index is sized to that reach, plus one node
        // spacing for the gap between the nearest point and the nearest
        // node, so the three-by-three block around any such position always
        // holds its nearest node. Sized from the road alone, as it was, a
        // car in a wide run-off found no node and was projected onto the
        // start line.
        float wallReach = 0.0f;
        for (int i = 0; i < Length; i++)
        {
            TrackNode node = this[i];
            wallReach = MathF.Max(
                wallReach,
                node.Width * 0.5f + MathF.Max(node.LeftBufferWidth, node.RightBufferWidth)
            );
        }
        cellSize = wallReach + BodyReachMeters + StepLength;
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
            Lerp(a.Curvature, b.Curvature, t),
            Lerp(a.Surface.Grade, b.Surface.Grade, t),
            Lerp(a.Surface.BankSlope, b.Surface.BankSlope, t),
            Lerp(a.Surface.BankCurvature, b.Surface.BankCurvature, t),
            Lerp(a.Surface.VerticalRate, b.Surface.VerticalRate, t)
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
        int bestIdx = -1;

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

        // Every node outside the three-by-three block is at least a cell
        // away, and the cell covers the whole of the walled world and a car
        // body past its walls, so any position a car can put a point at
        // finds its nearest node here. Finding none means the position is
        // outside the world.
        if (bestIdx < 0)
        {
            throw new InvalidOperationException(
                $"Position {pos} is beyond the track walls; no centreline node lies within {cellSize:F1} m."
            );
        }

        return bestIdx;
    }

    private static float Lerp(float from, float to, float weight)
    {
        return from + (to - from) * weight;
    }

}
