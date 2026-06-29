namespace StintegyEVO.Core.Car.Controllers.V1.Numerics;

public interface IQpSolver
{
    double[] Solve(DenseQpProblem problem);
}
public static class QpSolvers
{
    public static IQpSolver CreateDefault() => new HighsQpSolver();
}
