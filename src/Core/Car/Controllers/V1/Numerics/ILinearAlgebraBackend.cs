using System;
using System.Runtime.InteropServices;
using BlasSharp.AppleAccelerate;
using BlasSharp.OpenBlas;

namespace StintegyEVO.Core.Car.Controllers.V1.Numerics;

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
        if (OperatingSystem.IsMacOS() && RuntimeInformation.ProcessArchitecture == Architecture.Arm64)
            return new BlasSharpLinearAlgebraBackend(AppleAccelerateLapackOperations.Default, "AppleAccelerate");

        return new BlasSharpLinearAlgebraBackend(OpenBlasLapackOperations.Default, "OpenBLAS");
    }
}
