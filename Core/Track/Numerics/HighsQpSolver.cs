using System;
using System.Collections.Generic;
using Highs;

namespace TheStint.Core.Track.Numerics;

public sealed class HighsQpSolver : IQpSolver
{
    private const double Infinity = 1.0e30;
    private const double SparseTolerance = 1.0e-12;

    public double[] Solve(DenseQpProblem problem)
    {
        int n = problem.VariableCount;
        int rows = problem.ConstraintCount;
        BuildSparseRows(problem.ConstraintMatrix, rows, n, out int[] astart, out int[] aindex, out double[] avalue);
        BuildSparseLowerTriangular(problem.Hessian, out int[] qstart, out int[] qindex, out double[] qvalue);

        double[] rowLower = rows == 0 ? Array.Empty<double>() : ConvertBounds(problem.ConstraintLowerBounds!);
        double[] rowUpper = rows == 0 ? Array.Empty<double>() : ConvertBounds(problem.ConstraintUpperBounds!);

        HighsModel model = new(
            (double[])problem.Linear.Clone(),
            ConvertBounds(problem.LowerBounds),
            ConvertBounds(problem.UpperBounds),
            rowLower,
            rowUpper,
            astart,
            aindex,
            avalue,
            null,
            0.0,
            HighsMatrixFormat.kRowwise,
            HighsObjectiveSense.kMinimize
        );

        using HighsLpSolver solver = new();
        int outputFlag = Environment.GetEnvironmentVariable("V1_HIGHS_DEBUG") == "1" ? 1 : 0;
        solver.setBoolOptionValue("output_flag", outputFlag);

        CheckStatus(solver.passLp(model), "passLp");
        HighsHessian hessian = new(n, qstart, qindex, qvalue, HessianFormat.kTriangular);
        CheckStatus(solver.passHessian(hessian), "passHessian");
        CheckStatus(solver.run(), "run");

        HighsModelStatus modelStatus = solver.GetModelStatus();
        if (modelStatus != HighsModelStatus.kOptimal)
            throw new InvalidOperationException($"HiGHS QP did not reach optimal status: {modelStatus}.");

        HighsSolution solution = solver.getSolution();
        if (solution.colvalue == null || solution.colvalue.Length != n)
            throw new InvalidOperationException("HiGHS returned an invalid QP solution vector.");

        return solution.colvalue;
    }

    private static void CheckStatus(HighsStatus status, string operation)
    {
        if (status == HighsStatus.kError)
            throw new InvalidOperationException($"HiGHS {operation} failed with status {status}.");
    }

    private static double[] ConvertBounds(double[] bounds)
    {
        double[] converted = new double[bounds.Length];
        for (int i = 0; i < bounds.Length; i++)
        {
            double value = bounds[i];
            if (double.IsNegativeInfinity(value))
                converted[i] = -Infinity;
            else if (double.IsPositiveInfinity(value))
                converted[i] = Infinity;
            else
                converted[i] = value;
        }

        return converted;
    }

    private static void BuildSparseRows(
        double[,]? matrix,
        int rows,
        int columns,
        out int[] starts,
        out int[] indices,
        out double[] values
    )
    {
        starts = new int[rows + 1];
        if (rows == 0 || matrix == null)
        {
            indices = Array.Empty<int>();
            values = Array.Empty<double>();
            return;
        }

        List<int> sparseIndices = new();
        List<double> sparseValues = new();
        for (int r = 0; r < rows; r++)
        {
            starts[r] = sparseIndices.Count;
            for (int c = 0; c < columns; c++)
            {
                double value = matrix[r, c];
                if (Math.Abs(value) <= SparseTolerance)
                    continue;

                sparseIndices.Add(c);
                sparseValues.Add(value);
            }
        }

        starts[rows] = sparseIndices.Count;
        indices = sparseIndices.ToArray();
        values = sparseValues.ToArray();
    }

    private static void BuildSparseLowerTriangular(
        double[,] matrix,
        out int[] starts,
        out int[] indices,
        out double[] values
    )
    {
        int n = matrix.GetLength(0);
        starts = new int[n + 1];
        List<int> sparseIndices = new();
        List<double> sparseValues = new();

        for (int c = 0; c < n; c++)
        {
            starts[c] = sparseIndices.Count;
            for (int r = c; r < n; r++)
            {
                double value = matrix[r, c];
                if (Math.Abs(value) <= SparseTolerance)
                    continue;

                sparseIndices.Add(r);
                sparseValues.Add(value);
            }
        }

        starts[n] = sparseIndices.Count;
        indices = sparseIndices.ToArray();
        values = sparseValues.ToArray();
    }
}
