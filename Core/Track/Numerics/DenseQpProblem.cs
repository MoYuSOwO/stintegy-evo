using System;

namespace TheStint.Core.Track.Numerics;

public sealed class DenseQpProblem
{
    public DenseQpProblem(
        double[,] hessian,
        double[] linear,
        double[] lowerBounds,
        double[] upperBounds,
        double[,]? constraintMatrix,
        double[]? constraintLowerBounds,
        double[]? constraintUpperBounds
    )
    {
        Hessian = hessian;
        Linear = linear;
        LowerBounds = lowerBounds;
        UpperBounds = upperBounds;
        ConstraintMatrix = constraintMatrix;
        ConstraintLowerBounds = constraintLowerBounds;
        ConstraintUpperBounds = constraintUpperBounds;
        Validate();
    }

    public double[,] Hessian { get; }
    public double[] Linear { get; }
    public double[] LowerBounds { get; }
    public double[] UpperBounds { get; }
    public double[,]? ConstraintMatrix { get; }
    public double[]? ConstraintLowerBounds { get; }
    public double[]? ConstraintUpperBounds { get; }

    public int VariableCount => Linear.Length;
    public int ConstraintCount => ConstraintMatrix?.GetLength(0) ?? 0;

    private void Validate()
    {
        int n = Linear.Length;
        if (Hessian.GetLength(0) != n || Hessian.GetLength(1) != n)
            throw new ArgumentException("QP Hessian must be square and match the linear term length.");
        if (LowerBounds.Length != n || UpperBounds.Length != n)
            throw new ArgumentException("QP variable bounds must match the linear term length.");

        if (ConstraintMatrix == null)
        {
            if (ConstraintLowerBounds != null || ConstraintUpperBounds != null)
                throw new ArgumentException("QP constraint bounds require a constraint matrix.");
            return;
        }

        int rows = ConstraintMatrix.GetLength(0);
        if (ConstraintMatrix.GetLength(1) != n)
            throw new ArgumentException("QP constraint matrix width must match the linear term length.");
        if (ConstraintLowerBounds == null || ConstraintUpperBounds == null)
            throw new ArgumentException("QP constraint matrix requires lower and upper row bounds.");
        if (ConstraintLowerBounds.Length != rows || ConstraintUpperBounds.Length != rows)
            throw new ArgumentException("QP row bounds must match the constraint matrix row count.");
    }
}
