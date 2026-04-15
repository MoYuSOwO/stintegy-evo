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

public class TrackData
{
    public const float BaseFriction = 1.0f;
    public const float SafeMargin = 1.2f;

    private const float CellSize = 5.0f;
    private readonly Dictionary<long, List<int>> spatialBuckets = [];

    public readonly ImmutableArray<TrackNode> Nodes;
    public readonly ImmutableArray<float> OptimalLines;
    public readonly TrackGridConfig GridConfig;
    public float FrictionMultiplier { get; set; } = BaseFriction;

    public int NodeCounts => Nodes.Length;

    public TrackData(IList<TrackNode> nodes, TrackGridConfig gridConfig)
    {
        Nodes = [.. nodes];
        OptimalLines = GenerateOptimalLines(nodes);
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

    public int Vector2ToIndex(Vector2 pos)
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

    public Vector2 GridPosToVector2(int gridPos)
    {
        bool left = GridConfig.IsFirstGridLeft ? gridPos % 2 == 1 : gridPos % 2 == 0;
        float offset = GridConfig.GridOffset;
        if (!left) offset = -offset;
        Vector2 pos = Nodes.GetCircular(GridConfig.FirstGridIdx - GridConfig.GridStepDist * (gridPos - 1)).GetOffsetPos(offset);
        return pos;
    }

    public int GridPosToIdx(int gridPos)
    {
        return (GridConfig.FirstGridIdx - (gridPos - 1) * GridConfig.GridStepDist) % NodeCounts;
    }

    public static ImmutableArray<float> GenerateOptimalLines(IList<TrackNode> nodes)
    {
        float[] optimalLines = new float[nodes.Count];
        int N = nodes.Count;
        
        if (N < 3)
        {
            for (int i = 0; i < N; i++) optimalLines[i] = 0.0f;
            return [.. optimalLines];
        }

        double[] lower = new double[N];
        double[] upper = new double[N];
        for (int i = 0; i < N; i++)
        {
            lower[i] = -(nodes[i].HalfWidth - SafeMargin);
            upper[i] = nodes[i].HalfWidth - SafeMargin;
        }

        // 计算中心线投影曲率 K
        double[] K = new double[N];

        for (int i = 0; i < N; i++)
        {
            int im1 = (i - 1 + N) % N;
            int ip1 = (i + 1) % N;

            Vector2 D = nodes[im1].Center - 2.0f * nodes[i].Center + nodes[ip1].Center;
            K[i] = D.Dot(nodes[i].Normal);
        }

        List<int> hRows = [];
        List<int> hCols = [];
        List<double> hVals = [];
        double[] q = new double[N];

        void AddH(int r, int c, double v)
        {
            if (Math.Abs(v) < 1e-12) return;
            if (r > c) {
                (c, r) = (r, c);
            }
            hRows.Add(r); hCols.Add(c); hVals.Add(v);
        }

        // 严格组装 q 和 H (只遍历合法的二阶差分区间)
        for (int i = 0; i < N; i++)
        {
            int im1 = (i - 1 + N) % N;
            int ip1 = (i + 1) % N;

            double Ki = K[i];

            // 线性项 q 分发
            q[im1] += 2.0 * Ki;
            q[i]   -= 4.0 * Ki;
            q[ip1] += 2.0 * Ki;

            // 二次项 H 五对角展开
            AddH(im1, im1, 2.0);
            AddH(i, i, 8.0);
            AddH(ip1, ip1, 2.0);

            AddH(im1, i, -4.0);
            AddH(i, ip1, -4.0);
            AddH(im1, ip1, 2.0);
        }

        // 正则化保证正定
        for (int i = 0; i < N; i++)
        {
            AddH(i, i, 1e-6);
        }

        // ALGLIB 求解
        alglib.sparsecreate(N, N, out alglib.sparsematrix hMatrix);
        for (int idx = 0; idx < hRows.Count; idx++)
        {
            alglib.sparseadd(hMatrix, hRows[idx], hCols[idx], hVals[idx]);
        }
        alglib.sparseconverttocrs(hMatrix);
        alglib.minqpcreate(N, out alglib.minqpstate state);
        alglib.minqpsetquadratictermsparse(state, hMatrix, true);
        alglib.minqpsetlinearterm(state, q);
        alglib.minqpsetbc(state, lower, upper);
        alglib.minqpsetscaleautodiag(state);
        alglib.minqpsetalgoquickqp(state, 0.0, 0.0, 0.0, 0, true);
        alglib.minqpoptimize(state);
        alglib.minqpresults(state, out double[] resultAlpha, out alglib.minqpreport rep);

        for (int i = 0; i < N; i++)
        {
            optimalLines[i] = (float)resultAlpha[i];
        }

        SmoothRacingLine(ref optimalLines);

        if (optimalLines.Length != N)
        {
            GD.PrintErr($"Invalid number of optimal lines. Expected {N}, got {optimalLines.Length}.");
        }

        return [.. optimalLines];
    }

    private static void SmoothRacingLine(ref float[] lines, int passes = 3, int window = 2)
    {
        if (lines == null || lines.Length < 3) return;

        int N = lines.Length;

        float[] currentAlphas = new float[N];
        for (int i = 0; i < N; i++) currentAlphas[i] = lines[i];

        float[] tempAlphas = new float[N];

        for (int p = 0; p < passes; p++)
        {
            for (int i = 0; i < N; i++)
            {
                float sum = 0;
                float weightSum = 0;

                // 收集窗口内的点
                for (int j = -window; j <= window; j++)
                {
                    int idx;
                    idx = (i + j + N) % N; // 环形取模，接缝完美融合

                    // 简单的距离衰减权重 (中间高，两边低)
                    float weight = 1.0f / (1.0f + Math.Abs(j)); 
                    sum += currentAlphas[idx] * weight;
                    weightSum += weight;
                }
                tempAlphas[i] = sum / weightSum;
            }
            
            // 将本趟平滑结果覆写回去，准备下一趟
            Array.Copy(tempAlphas, currentAlphas, N);
        }

        // 把平滑后的结果写回真实赛车线数据
        for (int i = 0; i < N; i++)
        {
            lines[i] = currentAlphas[i];
        }
    }
}