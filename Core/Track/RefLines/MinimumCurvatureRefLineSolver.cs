using System;
using System.Collections.Generic;
using System.Diagnostics;
using TheStint.Core.Track.Numerics;
using System.Numerics;
using TheStint.Core.Util;

namespace TheStint.Core.Track.RefLines;

public sealed class MinimumCurvatureRefLineSolver : IRefLineSolver
{
    private const float OptimizationStepMeters = 6f;
    private const float KappaBound = 0.12f;
    private readonly float _optimizationStepMeters;
    private readonly ILinearAlgebraBackend _linearAlgebra;
    private readonly IQpSolver _qpSolver;

    public MinimumCurvatureRefLineSolver()
        : this(OptimizationStepMeters)
    {
    }

    public MinimumCurvatureRefLineSolver(float optimizationStepMeters)
        : this(optimizationStepMeters, LinearAlgebraBackends.CreateDefault(), QpSolvers.CreateDefault())
    {
    }

    public MinimumCurvatureRefLineSolver(ILinearAlgebraBackend linearAlgebra, IQpSolver qpSolver)
        : this(OptimizationStepMeters, linearAlgebra, qpSolver)
    {
    }

    public MinimumCurvatureRefLineSolver(float optimizationStepMeters, ILinearAlgebraBackend linearAlgebra, IQpSolver qpSolver)
    {
        _optimizationStepMeters = Math.Max(TrackData.StepLength, optimizationStepMeters);
        _linearAlgebra = linearAlgebra;
        _qpSolver = qpSolver;
    }

    public RefLine Generate(IReadOnlyList<RefLineTrackPoint> track)
    {
        Stopwatch totalTimer = Stopwatch.StartNew();
        if (track.Count < 8)
            return CenterLineRefLineSolver.Instance.Generate(track);

        int stride = (int)Math.Max(1, MathF.Round(_optimizationStepMeters / TrackData.StepLength));
        int count = Math.Max(
            4,
            (int)MathF.Round(track.Count / (float)stride)
        );

        Vector2[] refLine = new Vector2[count];
        int[] trackIndices = new int[count];
        double[] widthRight = new double[count];
        double[] widthLeft = new double[count];

        for (int i = 0; i < count; i++)
        {
            int trackIndex = (int)MathF.Round(
                i * track.Count / (float)count
            ) % track.Count;
            RefLineTrackPoint point = track[trackIndex];
            refLine[i] = point.Center;
            trackIndices[i] = trackIndex;
            widthRight[i] = point.HalfWidth;
            widthLeft[i] = point.HalfWidth;
        }

        Stopwatch stageTimer = Stopwatch.StartNew();
        SplineData refSpline = SplineData.CreateClosed(refLine, useDistanceScaling: true, _linearAlgebra);
        LogStage("reference spline", stageTimer);

        double[] alpha = SolveMinimumCurvature(refSpline, widthRight, widthLeft, _qpSolver, _linearAlgebra);
        LogStage("minimum-curvature QP", stageTimer);
        alpha = SmoothLateralOffsets(alpha, widthRight, widthLeft, passes: 1);

        Vector2[] optimizedRaceline = new Vector2[count];
        for (int i = 0; i < count; i++)
        {
            optimizedRaceline[i] = refLine[i] + refSpline.Normals[i] * (float)alpha[i];
        }

        SplineData racelineSpline = SplineData.CreateClosed(optimizedRaceline, useDistanceScaling: false, _linearAlgebra);
        LogStage("raceline spline", stageTimer);

        RefLine line = BuildProjectedSplineLine(track, trackIndices, racelineSpline);
        LogStage("projected raceline sampling", stageTimer);
        Console.WriteLine($"V1 minimum-curvature total: {totalTimer.Elapsed.TotalMilliseconds:0.0} ms, opt nodes={count}, output={line.Count}");
        return line;
    }

    private static double[] SmoothLateralOffsets(
        double[] offsets,
        double[] widthRight,
        double[] widthLeft,
        int passes
    )
    {
        double[] current = (double[])offsets.Clone();
        double[] next = new double[offsets.Length];
        for (int pass = 0; pass < passes; pass++)
        {
            for (int i = 0; i < current.Length; i++)
            {
                double smoothed = (
                    current[(i - 1 + current.Length) % current.Length] +
                    2.0 * current[i] +
                    current[(i + 1) % current.Length]
                ) * 0.25;
                double lower = -(
                    widthLeft[i] - TrackPlanningBounds.VehicleHalfWidthMeters
                );
                double upper =
                    widthRight[i] - TrackPlanningBounds.VehicleHalfWidthMeters;
                next[i] = Math.Clamp(smoothed, lower, upper);
            }
            (current, next) = (next, current);
        }
        return current;
    }

    private static void LogStage(string name, Stopwatch timer)
    {
        string message = $"V1 minimum-curvature {name}: {timer.Elapsed.TotalMilliseconds:0.0} ms";
        Console.WriteLine(message);
        timer.Restart();
    }

    private static double[] SolveMinimumCurvature(
        SplineData spline,
        double[] widthRight,
        double[] widthLeft,
        IQpSolver qpSolver,
        ILinearAlgebraBackend linearAlgebra
    )
    {
        int n = spline.Count;
        double[,] curvatureExtraction = CreateCurvatureExtractionMatrix(spline.Matrix, linearAlgebra);

        double[,] tnx = new double[n, n];
        double[,] tny = new double[n, n];

        for (int col = 0; col < n; col++)
        {
            int prevSpline = (col - 1 + n) % n;
            int currentRow = col * 4;
            int previousEndRow = prevSpline * 4 + 1;
            double normalX = spline.Normals[col].X;
            double normalY = spline.Normals[col].Y;

            for (int row = 0; row < n; row++)
            {
                double extractionSum = curvatureExtraction[row, currentRow] + curvatureExtraction[row, previousEndRow];
                tnx[row, col] = normalX * extractionSum;
                tny[row, col] = normalY * extractionSum;
            }
        }

        double[] xPrime = new double[n];
        double[] yPrime = new double[n];
        double[] xSecond = new double[n];
        double[] ySecond = new double[n];

        for (int i = 0; i < n; i++)
        {
            xPrime[i] = spline.CoeffsX[i, 1];
            yPrime[i] = spline.CoeffsY[i, 1];
            xSecond[i] = 2.0 * spline.CoeffsX[i, 2];
            ySecond[i] = 2.0 * spline.CoeffsY[i, 2];
        }

        double[] pxx = new double[n];
        double[] pxy = new double[n];
        double[] pyy = new double[n];
        double[] qx = new double[n];
        double[] qy = new double[n];
        double[] kappaRef = new double[n];

        for (int i = 0; i < n; i++)
        {
            double speedSq = xPrime[i] * xPrime[i] + yPrime[i] * yPrime[i];
            double denom = Math.Pow(Math.Max(speedSq, 1e-12), 1.5);
            double curvPart = 1.0 / denom;
            double curvPartSq = curvPart * curvPart;

            pxx[i] = curvPartSq * yPrime[i] * yPrime[i];
            pxy[i] = curvPartSq * -2.0 * xPrime[i] * yPrime[i];
            pyy[i] = curvPartSq * xPrime[i] * xPrime[i];
            qx[i] = curvPart * yPrime[i];
            qy[i] = curvPart * xPrime[i];
            kappaRef[i] = qy[i] * ySecond[i] - qx[i] * xSecond[i];
        }

        double[,] h = new double[n, n];
        double[] f = new double[n];
        double[,] eKappa = new double[n, n];

        for (int i = 0; i < n; i++)
        {
            for (int a = 0; a < n; a++)
            {
                double tnxIa = tnx[i, a];
                double tnyIa = tny[i, a];

                f[a] +=
                    2.0 * xSecond[i] * pxx[i] * tnxIa +
                    xSecond[i] * pxy[i] * tnyIa +
                    ySecond[i] * pxy[i] * tnxIa +
                    2.0 * ySecond[i] * pyy[i] * tnyIa;

                eKappa[i, a] = qy[i] * tnyIa - qx[i] * tnxIa;

                for (int b = a; b < n; b++)
                {
                    double value =
                        tnxIa * pxx[i] * tnx[i, b] +
                        tnyIa * pyy[i] * tny[i, b] +
                        0.5 * pxy[i] * (tnyIa * tnx[i, b] + tny[i, b] * tnxIa);
                    h[a, b] += value;
                }
            }
        }

        for (int r = 0; r < n; r++)
        {
            for (int c = r + 1; c < n; c++)
                h[c, r] = h[r, c];
        }

        return SolveQuadraticProgram(qpSolver, h, f, eKappa, kappaRef, widthRight, widthLeft);
    }

    private static double[,] CreateCurvatureExtractionMatrix(double[,] matrix, ILinearAlgebraBackend linearAlgebra)
    {
        int size = matrix.GetLength(0);
        int n = size / 4;
        double[,] transpose = new double[size, size];
        double[,] rhs = new double[size, n];

        for (int r = 0; r < size; r++)
        {
            for (int c = 0; c < size; c++)
                transpose[c, r] = matrix[r, c];
        }

        for (int row = 0; row < n; row++)
            rhs[row * 4 + 2, row] = 2.0;

        double[,] solution = linearAlgebra.Solve(transpose, rhs);
        double[,] extraction = new double[n, size];
        for (int row = 0; row < n; row++)
        {
            for (int col = 0; col < size; col++)
                extraction[row, col] = solution[col, row];
        }

        return extraction;
    }

    private static double[] SolveQuadraticProgram(
        IQpSolver qpSolver,
        double[,] h,
        double[] f,
        double[,] eKappa,
        double[] kappaRef,
        double[] widthRight,
        double[] widthLeft
    )
    {
        int n = f.Length;
        double[] lower = new double[n];
        double[] upper = new double[n];

        for (int i = 0; i < n; i++)
        {
            lower[i] = -(widthLeft[i] - TrackPlanningBounds.VehicleHalfWidthMeters);
            upper[i] = widthRight[i] - TrackPlanningBounds.VehicleHalfWidthMeters;
            if (lower[i] > upper[i])
                throw new InvalidOperationException("Track is too narrow for minimum-curvature optimization.");
        }

        double[,] linearConstraints = new double[n * 2, n];
        double[] constraintLowerBounds = new double[n * 2];
        double[] constraintUpperBounds = new double[n * 2];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                linearConstraints[i, j] = eKappa[i, j];
                linearConstraints[i + n, j] = -eKappa[i, j];
            }

            constraintLowerBounds[i] = double.NegativeInfinity;
            constraintLowerBounds[i + n] = double.NegativeInfinity;
            constraintUpperBounds[i] = KappaBound - kappaRef[i];
            constraintUpperBounds[i + n] = KappaBound + kappaRef[i];
        }

        DenseQpProblem problem = new(
            h,
            f,
            lower,
            upper,
            linearConstraints,
            constraintLowerBounds,
            constraintUpperBounds
        );

        return qpSolver.Solve(problem);
    }

    // Evaluate the optimized Cartesian spline directly, then adapt it to
    // TrackData's centerline-distance parameterization by projecting each
    // sample onto the corresponding track normal. Position, heading and
    // curvature are all derived from that one projected line, so the
    // controller never mixes two different longitudinal parameterizations.
    private static RefLine BuildProjectedSplineLine(
        IReadOnlyList<RefLineTrackPoint> track,
        int[] trackIndices,
        SplineData racelineSpline
    )
    {
        Vector2[] positions = new Vector2[track.Count];
        float[] offsets = new float[track.Count];
        RefLinePoint[] points = new RefLinePoint[track.Count];

        int segment = 0;
        int lastSegment = trackIndices.Length - 1;
        for (int i = 0; i < track.Count; i++)
        {
            while (segment < lastSegment && i >= trackIndices[segment + 1])
                segment++;

            RefLineTrackPoint trackPoint = track[i];
            int segmentStart = trackIndices[segment];
            int segmentEnd = segment == lastSegment ? trackIndices[0] + track.Count : trackIndices[segment + 1];
            int wrappedIndex = i < segmentStart ? i + track.Count : i;
            float segmentProgress = segmentEnd <= segmentStart
                ? 0f
                : Math.Clamp((wrappedIndex - segmentStart) / (float)(segmentEnd - segmentStart), 0f, 1f);

            Vector2 splinePosition = racelineSpline.Evaluate(segment, segmentProgress);
            float offset = Vector2.Dot(
                splinePosition - trackPoint.Center,
                trackPoint.Normal
            );
            float maxOffset = Math.Max(
                0f,
                trackPoint.HalfWidth - TrackPlanningBounds.VehicleHalfWidthMeters
            );
            offset = Math.Clamp(offset, -maxOffset, maxOffset);
            offsets[i] = offset;
            positions[i] = trackPoint.GetOffsetPos(offset);
        }

        float[] headings = new float[track.Count];
        for (int i = 0; i < track.Count; i++)
        {
            Vector2 tangent = MathHelper.FivePointStencil(
                positions[WrapIndex(i - 2, track.Count)],
                positions[WrapIndex(i - 1, track.Count)],
                positions[WrapIndex(i + 1, track.Count)],
                positions[WrapIndex(i + 2, track.Count)]
            );
            headings[i] = MathHelper.NormalizeAngle(tangent.Angle());
        }

        const int curvatureRadius = 3;
        for (int i = 0; i < track.Count; i++)
        {
            int previous = WrapIndex(i - curvatureRadius, track.Count);
            int next = WrapIndex(i + curvatureRadius, track.Count);
            float arcLength = 0f;
            for (int delta = -curvatureRadius; delta < curvatureRadius; delta++)
            {
                arcLength += Vector2.Distance(
                    positions[WrapIndex(i + delta, track.Count)],
                    positions[WrapIndex(i + delta + 1, track.Count)]
                );
            }
            float headingDelta = MathHelper.NormalizeAngle(
                headings[next] - headings[previous]
            );
            float curvature = arcLength <= 1e-4f
                ? 0f
                : headingDelta / arcLength;
            points[i] = new RefLinePoint(
                offsets[i],
                positions[i],
                headings[i],
                curvature
            );
        }

        return new RefLine(points);
    }

    private static int WrapIndex(int index, int length) => (index % length + length) % length;

    private sealed class SplineData
    {
        public readonly double[,] Matrix;
        public readonly double[,] CoeffsX;
        public readonly double[,] CoeffsY;
        public readonly Vector2[] Normals;
        public int Count => Normals.Length;

        private SplineData(double[,] matrix, double[,] coeffsX, double[,] coeffsY, Vector2[] normals)
        {
            Matrix = matrix;
            CoeffsX = coeffsX;
            CoeffsY = coeffsY;
            Normals = normals;
        }

        public static SplineData CreateClosed(Vector2[] points, bool useDistanceScaling, ILinearAlgebraBackend linearAlgebra)
        {
            int n = points.Length;
            int size = n * 4;
            double[,] m = new double[size, size];
            double[,] rhs = new double[size, 2];
            double[] lengths = new double[n + 1];

            for (int i = 0; i < n; i++)
            {
                Vector2 a = points[i];
                Vector2 b = points[(i + 1) % n];
                lengths[i] = Math.Max(Vector2.Distance(a, b), 1e-6f);
            }
            lengths[n] = lengths[0];

            double[] scaling = new double[n];
            for (int i = 0; i < n; i++)
            {
                scaling[i] = useDistanceScaling ? lengths[i] / lengths[i + 1] : 1.0;
            }

            for (int i = 0; i < n; i++)
            {
                int row = i * 4;
                int col = i * 4;

                if (i < n - 1)
                {
                    FillTemplate(m, row, col, scaling[i]);
                }
                else
                {
                    m[row, col] = 1.0;
                    m[row + 1, col] = 1.0;
                    m[row + 1, col + 1] = 1.0;
                    m[row + 1, col + 2] = 1.0;
                    m[row + 1, col + 3] = 1.0;
                }

                Vector2 curr = points[i];
                Vector2 next = points[(i + 1) % n];
                rhs[row, 0] = curr.X;
                rhs[row, 1] = curr.Y;
                rhs[row + 1, 0] = next.X;
                rhs[row + 1, 1] = next.Y;
            }

            m[size - 2, 1] = scaling[n - 1];
            m[size - 2, size - 3] = -1.0;
            m[size - 2, size - 2] = -2.0;
            m[size - 2, size - 1] = -3.0;

            m[size - 1, 2] = 2.0 * scaling[n - 1] * scaling[n - 1];
            m[size - 1, size - 2] = -2.0;
            m[size - 1, size - 1] = -6.0;

            double[,] solved = linearAlgebra.Solve(m, rhs);

            double[,] coeffsX = new double[n, 4];
            double[,] coeffsY = new double[n, 4];
            Vector2[] normals = new Vector2[n];

            for (int i = 0; i < n; i++)
            {
                for (int c = 0; c < 4; c++)
                {
                    coeffsX[i, c] = solved[i * 4 + c, 0];
                    coeffsY[i, c] = solved[i * 4 + c, 1];
                }

                Vector2 normal = new((float)coeffsY[i, 1], (float)-coeffsX[i, 1]);
                normals[i] = normal.LengthSquared() > 1e-12f ? Vector2.Normalize(normal) : Vector2.Zero;
            }

            return new SplineData(m, coeffsX, coeffsY, normals);
        }

        private static void FillTemplate(double[,] m, int row, int col, double scaling)
        {
            m[row, col] = 1.0;

            m[row + 1, col] = 1.0;
            m[row + 1, col + 1] = 1.0;
            m[row + 1, col + 2] = 1.0;
            m[row + 1, col + 3] = 1.0;

            m[row + 2, col + 1] = 1.0;
            m[row + 2, col + 2] = 2.0;
            m[row + 2, col + 3] = 3.0;
            m[row + 2, col + 5] = -scaling;

            m[row + 3, col + 2] = 2.0;
            m[row + 3, col + 3] = 6.0;
            m[row + 3, col + 6] = -2.0 * scaling * scaling;
        }

        public Vector2 Evaluate(int index, double t)
        {
            double x = CoeffsX[index, 0] + CoeffsX[index, 1] * t + CoeffsX[index, 2] * t * t + CoeffsX[index, 3] * t * t * t;
            double y = CoeffsY[index, 0] + CoeffsY[index, 1] * t + CoeffsY[index, 2] * t * t + CoeffsY[index, 3] * t * t * t;
            return new Vector2((float)x, (float)y);
        }

        public float Heading(int index, double t)
        {
            double xd = CoeffsX[index, 1] + 2.0 * CoeffsX[index, 2] * t + 3.0 * CoeffsX[index, 3] * t * t;
            double yd = CoeffsY[index, 1] + 2.0 * CoeffsY[index, 2] * t + 3.0 * CoeffsY[index, 3] * t * t;
            return MathHelper.NormalizeAngle((float)Math.Atan2(yd, xd));
        }

        public float Curvature(int index, double t)
        {
            double xd = CoeffsX[index, 1] + 2.0 * CoeffsX[index, 2] * t + 3.0 * CoeffsX[index, 3] * t * t;
            double yd = CoeffsY[index, 1] + 2.0 * CoeffsY[index, 2] * t + 3.0 * CoeffsY[index, 3] * t * t;
            double xdd = 2.0 * CoeffsX[index, 2] + 6.0 * CoeffsX[index, 3] * t;
            double ydd = 2.0 * CoeffsY[index, 2] + 6.0 * CoeffsY[index, 3] * t;
            double denom = Math.Pow(xd * xd + yd * yd, 1.5);
            if (denom < 1e-12) return 0f;
            return (float)((xd * ydd - yd * xdd) / denom);
        }

        public double[] CalculateSplineLengths()
        {
            const int Samples = 15;
            double[] lengths = new double[Count];

            for (int i = 0; i < Count; i++)
            {
                Vector2 previous = Evaluate(i, 0.0);
                double sum = 0.0;
                for (int s = 1; s <= Samples; s++)
                {
                    Vector2 current = Evaluate(i, s / (double)Samples);
                    sum += Vector2.Distance(previous, current);
                    previous = current;
                }
                lengths[i] = Math.Max(sum, 1e-6);
            }

            return lengths;
        }
    }

}
