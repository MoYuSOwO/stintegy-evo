using System;
using Godot;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Controllers.V1.DynamicPath;

internal static class DynamicPathSplineMath
{
    private const int LengthRefinementIterations = 3;

    public static DynamicPathSplineSegment CreateUnclosedSegment(
        Vector2 start,
        Vector2 end,
        float startHeading,
        float endHeading
    )
    {
        float length = Mathf.Max(start.DistanceTo(end), 1e-4f);
        return CalculateUnclosedSegment(start, end, startHeading, endHeading, length);
    }

    public static DynamicPathSplineSegment CreateLengthRefinedUnclosedSegment(
        Vector2 start,
        Vector2 end,
        float startHeading,
        float endHeading
    )
    {
        float length = Mathf.Max(start.DistanceTo(end), 1e-4f);
        DynamicPathSplineSegment segment = CalculateUnclosedSegment(start, end, startHeading, endHeading, length);

        for (int i = 0; i < LengthRefinementIterations; i++)
        {
            float refinedLength = EstimateLength(segment);
            if (Mathf.Abs(refinedLength - length) < 1e-3f)
                break;

            length = Mathf.Max(refinedLength, 1e-4f);
            segment = CalculateUnclosedSegment(start, end, startHeading, endHeading, length);
        }

        return segment;
    }

    public static DynamicPathSplineSegment[] CreateUnclosedPath(
        Vector2[] points,
        float startHeading,
        float endHeading,
        float[] elementLengths
    )
    {
        if (points.Length < 2)
            throw new ArgumentException("At least two points are required.", nameof(points));
        if (elementLengths.Length != points.Length - 1)
            throw new ArgumentException("Element length count must be one less than point count.", nameof(elementLengths));

        int segmentCount = points.Length - 1;
        float[] lengths = new float[elementLengths.Length];
        for (int i = 0; i < lengths.Length; i++)
            lengths[i] = Mathf.Max(elementLengths[i], 1e-4f);

        double[,] matrix = new double[segmentCount * 4, segmentCount * 4];
        double[] rhsX = new double[segmentCount * 4];
        double[] rhsY = new double[segmentCount * 4];

        FillSplineSystem(points, lengths, closed: false, startHeading, endHeading, matrix, rhsX, rhsY);
        double[] solvedX = SolveLinearSystem(matrix, rhsX);
        double[] solvedY = SolveLinearSystem(matrix, rhsY);

        DynamicPathSplineSegment[] segments = new DynamicPathSplineSegment[segmentCount];
        for (int i = 0; i < segmentCount; i++)
            segments[i] = SegmentFromSolution(solvedX, solvedY, i);
        return segments;
    }

    public static DynamicPathSplineSegment[] CreateClosed(Vector2[] points)
    {
        if (points.Length < 2)
            throw new ArgumentException("At least two points are required.", nameof(points));

        int segmentCount = points.Length;
        Vector2[] closedPath = new Vector2[segmentCount + 1];
        Array.Copy(points, closedPath, points.Length);
        closedPath[^1] = points[0];

        float[] lengths = new float[segmentCount + 1];
        for (int i = 0; i < segmentCount; i++)
            lengths[i] = Mathf.Max(closedPath[i].DistanceTo(closedPath[i + 1]), 1e-4f);
        lengths[segmentCount] = lengths[0];

        double[,] matrix = new double[segmentCount * 4, segmentCount * 4];
        double[] rhsX = new double[segmentCount * 4];
        double[] rhsY = new double[segmentCount * 4];

        FillSplineSystem(closedPath, lengths, closed: true, 0.0f, 0.0f, matrix, rhsX, rhsY);
        double[] solvedX = SolveLinearSystem(matrix, rhsX);
        double[] solvedY = SolveLinearSystem(matrix, rhsY);

        DynamicPathSplineSegment[] segments = new DynamicPathSplineSegment[segmentCount];
        for (int i = 0; i < segmentCount; i++)
            segments[i] = SegmentFromSolution(solvedX, solvedY, i);
        return segments;
    }

    public static float EstimateLength(DynamicPathSplineSegment segment, int samples = 15)
    {
        float length = 0.0f;
        Vector2 previous = segment.Evaluate(0.0f);
        for (int i = 1; i < samples; i++)
        {
            float t = i / (float)(samples - 1);
            Vector2 current = segment.Evaluate(t);
            length += previous.DistanceTo(current);
            previous = current;
        }

        return length;
    }

    private static DynamicPathSplineSegment CalculateUnclosedSegment(
        Vector2 start,
        Vector2 end,
        float startHeading,
        float endHeading,
        float length
    )
    {
        Vector2[] path = [start, end];
        float[] lengths = [Mathf.Max(length, 1e-4f)];
        double[,] matrix = new double[4, 4];
        double[] rhsX = new double[4];
        double[] rhsY = new double[4];

        FillSplineSystem(path, lengths, closed: false, startHeading, endHeading, matrix, rhsX, rhsY);
        double[] solvedX = SolveLinearSystem(matrix, rhsX);
        double[] solvedY = SolveLinearSystem(matrix, rhsY);
        return SegmentFromSolution(solvedX, solvedY, 0);
    }

    private static void FillSplineSystem(
        Vector2[] path,
        float[] lengths,
        bool closed,
        float startHeading,
        float endHeading,
        double[,] matrix,
        double[] rhsX,
        double[] rhsY
    )
    {
        int segmentCount = path.Length - 1;
        double[] scaling = new double[Math.Max(segmentCount, 1)];
        for (int i = 0; i < segmentCount - 1; i++)
            scaling[i] = lengths[i] / lengths[i + 1];
        if (closed)
            scaling[segmentCount - 1] = lengths[segmentCount - 1] / lengths[segmentCount];

        for (int i = 0; i < segmentCount; i++)
        {
            int row = i * 4;
            int col = i * 4;

            if (i < segmentCount - 1)
            {
                FillTemplate(matrix, row, col, scaling[i]);
            }
            else
            {
                matrix[row, col] = 1.0;
                matrix[row + 1, col] = 1.0;
                matrix[row + 1, col + 1] = 1.0;
                matrix[row + 1, col + 2] = 1.0;
                matrix[row + 1, col + 3] = 1.0;
            }

            rhsX[row] = path[i].X;
            rhsY[row] = path[i].Y;
            rhsX[row + 1] = path[i + 1].X;
            rhsY[row + 1] = path[i + 1].Y;
        }

        if (closed)
        {
            int size = segmentCount * 4;
            matrix[size - 2, 1] = scaling[segmentCount - 1];
            matrix[size - 2, size - 3] = -1.0;
            matrix[size - 2, size - 2] = -2.0;
            matrix[size - 2, size - 1] = -3.0;

            matrix[size - 1, 2] = 2.0 * scaling[segmentCount - 1] * scaling[segmentCount - 1];
            matrix[size - 1, size - 2] = -2.0;
            matrix[size - 1, size - 1] = -6.0;
        }
        else
        {
            Vector2 startTangent = HeadingVector(startHeading) * lengths[0];
            Vector2 endTangent = HeadingVector(endHeading) * lengths[^1];
            int size = segmentCount * 4;

            matrix[size - 2, 1] = 1.0;
            rhsX[size - 2] = startTangent.X;
            rhsY[size - 2] = startTangent.Y;

            matrix[size - 1, size - 4] = 0.0;
            matrix[size - 1, size - 3] = 1.0;
            matrix[size - 1, size - 2] = 2.0;
            matrix[size - 1, size - 1] = 3.0;
            rhsX[size - 1] = endTangent.X;
            rhsY[size - 1] = endTangent.Y;
        }
    }

    private static void FillTemplate(double[,] matrix, int row, int col, double scaling)
    {
        matrix[row, col] = 1.0;

        matrix[row + 1, col] = 1.0;
        matrix[row + 1, col + 1] = 1.0;
        matrix[row + 1, col + 2] = 1.0;
        matrix[row + 1, col + 3] = 1.0;

        matrix[row + 2, col + 1] = 1.0;
        matrix[row + 2, col + 2] = 2.0;
        matrix[row + 2, col + 3] = 3.0;
        matrix[row + 2, col + 5] = -scaling;

        matrix[row + 3, col + 2] = 2.0;
        matrix[row + 3, col + 3] = 6.0;
        matrix[row + 3, col + 6] = -2.0 * scaling * scaling;
    }

    private static DynamicPathSplineSegment SegmentFromSolution(double[] solvedX, double[] solvedY, int segmentIndex)
    {
        int offset = segmentIndex * 4;
        return new DynamicPathSplineSegment(
            new Vector2((float)solvedX[offset], (float)solvedY[offset]),
            new Vector2((float)solvedX[offset + 1], (float)solvedY[offset + 1]),
            new Vector2((float)solvedX[offset + 2], (float)solvedY[offset + 2]),
            new Vector2((float)solvedX[offset + 3], (float)solvedY[offset + 3])
        );
    }

    private static double[] SolveLinearSystem(double[,] matrixIn, double[] rhsIn)
    {
        int n = rhsIn.Length;
        double[,] matrix = (double[,])matrixIn.Clone();
        double[] rhs = (double[])rhsIn.Clone();

        for (int col = 0; col < n; col++)
        {
            int pivot = col;
            double pivotAbs = Math.Abs(matrix[col, col]);
            for (int row = col + 1; row < n; row++)
            {
                double candidate = Math.Abs(matrix[row, col]);
                if (candidate > pivotAbs)
                {
                    pivot = row;
                    pivotAbs = candidate;
                }
            }

            if (pivotAbs < 1e-12)
                throw new InvalidOperationException("Spline equation system is singular.");

            if (pivot != col)
            {
                for (int c = col; c < n; c++)
                    (matrix[col, c], matrix[pivot, c]) = (matrix[pivot, c], matrix[col, c]);
                (rhs[col], rhs[pivot]) = (rhs[pivot], rhs[col]);
            }

            double pivotValue = matrix[col, col];
            for (int row = col + 1; row < n; row++)
            {
                double factor = matrix[row, col] / pivotValue;
                if (Math.Abs(factor) < 1e-18)
                    continue;

                matrix[row, col] = 0.0;
                for (int c = col + 1; c < n; c++)
                    matrix[row, c] -= factor * matrix[col, c];
                rhs[row] -= factor * rhs[col];
            }
        }

        double[] solution = new double[n];
        for (int row = n - 1; row >= 0; row--)
        {
            double value = rhs[row];
            for (int col = row + 1; col < n; col++)
                value -= matrix[row, col] * solution[col];
            solution[row] = value / matrix[row, row];
        }

        return solution;
    }

    private static Vector2 HeadingVector(float heading)
    {
        heading = GeomUtil.NormalizeAngle(heading);
        return new Vector2(Mathf.Cos(heading), Mathf.Sin(heading));
    }
}
