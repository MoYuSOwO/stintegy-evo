namespace StintegyEVO.Core.Track.Numerics;

public interface IQpSolver
{
    double[] Solve(DenseQpProblem problem);
}
public static class QpSolvers
{
    public static IQpSolver CreateDefault() => new HighsQpSolver();
}
