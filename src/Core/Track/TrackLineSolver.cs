using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using Godot;

namespace StintegyEVO.Core.Track;

public static class TrackLineSolver
{
    internal static ImmutableArray<float> GenerateOptimalLines(IList<TrackNode> nodes, float safeMargin)
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
            lower[i] = -(nodes[i].HalfWidth - safeMargin);
            upper[i] = nodes[i].HalfWidth - safeMargin;
        }

        // Calculate the curvature K of the centerline projection
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

        // Strictly assemble q and H (only traverse valid second-order difference intervals)
        for (int i = 0; i < N; i++)
        {
            int im1 = (i - 1 + N) % N;
            int ip1 = (i + 1) % N;

            double Ki = K[i];

            // Linear term q
            q[im1] += 2.0 * Ki;
            q[i]   -= 4.0 * Ki;
            q[ip1] += 2.0 * Ki;

            // Five-diagonal expansion of the quadratic term H
            AddH(im1, im1, 2.0);
            AddH(i, i, 8.0);
            AddH(ip1, ip1, 2.0);

            AddH(im1, i, -4.0);
            AddH(i, ip1, -4.0);
            AddH(im1, ip1, 2.0);
        }

        // Regularization guarantees positive definiteness
        for (int i = 0; i < N; i++)
        {
            AddH(i, i, 1e-6);
        }

        // Solve by ALGLIB
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
        alglib.minqpresults(state, out double[] resultAlpha, out _);

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

                for (int j = -window; j <= window; j++)
                {
                    int idx;
                    idx = (i + j + N) % N;

                    float weight = 1.0f / (1.0f + Math.Abs(j)); 
                    sum += currentAlphas[idx] * weight;
                    weightSum += weight;
                }
                tempAlphas[i] = sum / weightSum;
            }
            
            Array.Copy(tempAlphas, currentAlphas, N);
        }

        for (int i = 0; i < N; i++)
        {
            lines[i] = currentAlphas[i];
        }
    }
}