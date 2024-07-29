using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Complex;
using MathNet.Numerics.LinearAlgebra.Solvers;

namespace RefraSin.Numerics.LinearSolvers;

public class IterativeSolver(
    IIterativeSolver<double> solver,
    Iterator<double> iterator,
    IPreconditioner<double> preconditioner
) : ILinearSolver
{
    public IIterativeSolver<double> Solver { get; } = solver;

    public Iterator<double> Iterator { get; } = iterator;

    public IPreconditioner<double> Preconditioner { get; } = preconditioner;

    /// <inheritdoc />
    public Vector<double> Solve(
        Matrix<double> matrix,
        Vector<double> rightSide,
        Vector<double>? initialGuess = null
    )
    {
        Iterator.Reset();
        var sol = initialGuess is null
            ? Vector<double>.Build.Dense(matrix.ColumnCount)
            : Vector<double>.Build.DenseOfVector(initialGuess);
        Solver.Solve(matrix, rightSide, sol, Iterator, Preconditioner);

        return sol;
    }
}
