using System;
using BlasSharp;

namespace StintegyEVO.Core.Car.Controllers.V1.Numerics;

public sealed class BlasSharpLinearAlgebraBackend : ILinearAlgebraBackend
{
    private readonly ILapackOperations _lapack;

    public BlasSharpLinearAlgebraBackend(ILapackOperations lapack, string name)
    {
        _lapack = lapack;
        Name = name;
    }

    public string Name { get; }

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

    public unsafe double[,] Solve(double[,] matrix, double[,] rhs)
    {
        int n = EnsureSquare(matrix);
        if (rhs.GetLength(0) != n)
            throw new ArgumentException("Right-hand side row count must match matrix size.", nameof(rhs));

        double[] a = ToColumnMajor(matrix);
        double[] b = ToColumnMajor(rhs);
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
