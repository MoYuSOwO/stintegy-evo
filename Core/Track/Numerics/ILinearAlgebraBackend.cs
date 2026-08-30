using System;
using System.Runtime.InteropServices;
using BlasSharp.AppleAccelerate;
using BlasSharp.OpenBlas;

namespace StintegyEVO.Core.Track.Numerics;

public interface ILinearAlgebraBackend
{
    double[] Solve(double[,] matrix, double[] rhs);

    double[,] Solve(double[,] matrix, double[,] rhs);

    double[,] Inverse(double[,] matrix);
}

public static class LinearAlgebraBackends
{
    public static ILinearAlgebraBackend CreateDefault()
    {
        // Diagnostic override: force one BlasSharp binding regardless of
        // platform, so a suspect binding can be tried where the other one
        // is known to work.
        string? forced = Environment.GetEnvironmentVariable("STINTEGY_LAPACK");
        if (string.Equals(forced, "openblas", StringComparison.OrdinalIgnoreCase))
            return new BlasSharpLinearAlgebraBackend(OpenBlasLapackOperations.Default, "OpenBLAS");
        if (string.Equals(forced, "accelerate", StringComparison.OrdinalIgnoreCase))
            return new BlasSharpLinearAlgebraBackend(AppleAccelerateLapackOperations.Default, "AppleAccelerate");

        if (OperatingSystem.IsMacOS() && RuntimeInformation.ProcessArchitecture == Architecture.Arm64)
            return new BlasSharpLinearAlgebraBackend(AppleAccelerateLapackOperations.Default, "AppleAccelerate");

        return new BlasSharpLinearAlgebraBackend(OpenBlasLapackOperations.Default, "OpenBLAS");
    }
}
