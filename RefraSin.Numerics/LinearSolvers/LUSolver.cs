using MathNet.Numerics.LinearAlgebra;

namespace RefraSin.Numerics.LinearSolvers;

public class LUSolver(bool doIterativeImprovement = true) : ILinearSolver
{
    /// <inheritdoc />
    public Vector<double> Solve(
        Matrix<double> matrix,
        Vector<double> rightSide,
        Vector<double>? initialGuess = null
    )
    {
        var lu = matrix.LU();
        var directSolution = lu.Solve(rightSide);

        if (!DoIterativeImprovement)
            return directSolution;

        var error = matrix * directSolution - rightSide;
        var correction = lu.Solve(error);

        return directSolution + correction;
    }

    public bool DoIterativeImprovement { get; } = doIterativeImprovement;
}
