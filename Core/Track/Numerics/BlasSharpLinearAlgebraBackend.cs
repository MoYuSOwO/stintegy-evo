using System;
using BlasSharp;

namespace StintegyEVO.Core.Track.Numerics;

public sealed class BlasSharpLinearAlgebraBackend(ILapackOperations lapack, string name) : ILinearAlgebraBackend
{
    private readonly ILapackOperations _lapack = lapack;

    public string Name { get; } = name;

    public unsafe double[] Solve(double[,] matrix, double[] rhs)
    {
        int n = EnsureSquare(matrix);
        if (rhs.Length != n)
            throw new ArgumentException("Right-hand side length must match matrix size.", nameof(rhs));

        double[,] rhsMatrix = new double[n, 1];
        for (int i = 0; i < n; i++)
            rhsMatrix[i, 0] = rhs[i];

        double[,] solution = Solve(matrix, rhsMatrix);
        double[] result = new double[n];
        for (int i = 0; i < n; i++)
            result[i] = solution[i, 0];

        return result;
    }

    private static int _dumpIndex;

    public unsafe double[,] Solve(double[,] matrix, double[,] rhs)
    {
        int n = EnsureSquare(matrix);
        if (rhs.GetLength(0) != n)
            throw new ArgumentException("Right-hand side row count must match matrix size.", nameof(rhs));

        double[] a = ToColumnMajor(matrix);
        double[] b = ToColumnMajor(rhs);

        // Ground truth for a cross-platform numerics dispute: every system
        // handed to LAPACK, written out before the call so an independent
        // implementation on either machine can be shown the same bytes.
        // Diagnostic only, dormant without the environment variable.
        if (Environment.GetEnvironmentVariable("STINTEGY_DUMP_LA") is
                { Length: > 0 } dumpPrefix &&
            _dumpIndex < 4)
        {
            using var writer = new System.IO.BinaryWriter(
                System.IO.File.Create($"{dumpPrefix}.{_dumpIndex++}"));
            writer.Write(n);
            writer.Write(rhs.GetLength(1));
            foreach (double value in a)
                writer.Write(value);
            foreach (double value in b)
                writer.Write(value);
        }
        int[] pivot = new int[n];
        int nrhs = rhs.GetLength(1);
        int lda = n;
        int ldb = n;
        int info = 0;

        fixed (double* aPtr = a)
        fixed (double* bPtr = b)
        fixed (int* pivotPtr = pivot)
        {
            _lapack.Dgesv(&n, &nrhs, aPtr, &lda, pivotPtr, bPtr, &ldb, &info);
        }

        if (info != 0 &&
            Environment.GetEnvironmentVariable("STINTEGY_DUMP_LA_FAIL") is
                { Length: > 0 } dumpPath)
        {
            // Ground truth for a cross-platform numerics dispute: the exact
            // system this process asked LAPACK to solve, so an independent
            // implementation can be shown the same bytes.
            using var writer = new System.IO.BinaryWriter(
                System.IO.File.Create(dumpPath));
            writer.Write(n);
            writer.Write(nrhs);
            double[] aOriginal = ToColumnMajor(matrix);
            foreach (double value in aOriginal)
                writer.Write(value);
            double[] bOriginal = ToColumnMajor(rhs);
            foreach (double value in bOriginal)
                writer.Write(value);
        }
        ThrowIfLapackFailed(info, "Dgesv");
        return FromColumnMajor(b, n, nrhs);
    }

    public unsafe double[,] Inverse(double[,] matrix)
    {
        int n = EnsureSquare(matrix);
        double[] a = ToColumnMajor(matrix);
        int[] pivot = new int[n];
        int lda = n;
        int info = 0;

        fixed (double* aPtr = a)
        fixed (int* pivotPtr = pivot)
        {
            int m = n;
            _lapack.Dgetrf(&m, &n, aPtr, &lda, pivotPtr, &info);
            ThrowIfLapackFailed(info, "Dgetrf");

            int queryWork = -1;
            double workSize = 0.0;
            _lapack.Dgetri(&n, aPtr, &lda, pivotPtr, &workSize, &queryWork, &info);
            ThrowIfLapackFailed(info, "Dgetri workspace query");

            int workLength = Math.Max(1, (int)Math.Ceiling(workSize));
            double[] work = new double[workLength];
            fixed (double* workPtr = work)
            {
                _lapack.Dgetri(&n, aPtr, &lda, pivotPtr, workPtr, &workLength, &info);
            }
        }

        ThrowIfLapackFailed(info, "Dgetri");
        return FromColumnMajor(a, n, n);
    }

    private static int EnsureSquare(double[,] matrix)
    {
        int rows = matrix.GetLength(0);
        int columns = matrix.GetLength(1);
        if (rows != columns)
            throw new ArgumentException("Matrix must be square.", nameof(matrix));
        return rows;
    }

    private static double[] ToColumnMajor(double[,] matrix)
    {
        int rows = matrix.GetLength(0);
        int columns = matrix.GetLength(1);
        double[] data = new double[rows * columns];

        for (int c = 0; c < columns; c++)
        {
            int columnOffset = c * rows;
            for (int r = 0; r < rows; r++)
                data[columnOffset + r] = matrix[r, c];
        }

        return data;
    }

    private static double[,] FromColumnMajor(double[] data, int rows, int columns)
    {
        double[,] matrix = new double[rows, columns];
        for (int c = 0; c < columns; c++)
        {
            int columnOffset = c * rows;
            for (int r = 0; r < rows; r++)
                matrix[r, c] = data[columnOffset + r];
        }

        return matrix;
    }

    private static void ThrowIfLapackFailed(int info, string operation)
    {
        if (info < 0)
            throw new InvalidOperationException($"{operation} received an invalid argument at position {-info}.");
        if (info > 0)
            throw new InvalidOperationException($"{operation} failed because the matrix is singular at diagonal {info}.");
    }
}
