using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.Optimization.LineSearch;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Logging.Abstractions;
using RefraSin.Numerics.Exceptions;
using RefraSin.Numerics.LinearSolvers;

namespace RefraSin.Numerics.RootFinding;

public class NewtonRaphsonRootFinder(
    ILinearSolver jacobianStepSolver,
    bool useLineSearch = false,
    int maxIterationCount = 100,
    double absoluteTolerance = 1e-8,
    WolfeLineSearch? wolfeLineSearch = null
) : IRootFinder
{
    /// <inheritdoc />
    public Vector<double> FindRoot(
        Func<Vector<double>, Vector<double>> function,
        Func<Vector<double>, Matrix<double>> jacobian,
        Vector<double> initialGuess,
        ILogger? logger = null
    )
    {
        logger ??= NullLogger<NewtonRaphsonRootFinder>.Instance;

        int i;

        var x = initialGuess.Clone();
        var y = function(x);
        var error = y.L2Norm();
        var dxOld = Vector<double>.Build.Dense(x.Count, double.NaN);

        var objectiveFunction = ObjectiveFunction.Gradient(
            v => function(v).L2Norm(),
            v => jacobian(v).ColumnNorms(2)
        );

        for (i = 0; i < MaxIterationCount; i++)
        {
            logger.LogDebug("Current error {Error}.", error);

            if (error <= AbsoluteTolerance)
            {
                logger.LogDebug("Solution found.");
                return x;
            }

            var jac = jacobian(x);
            jac.CoerceZero(1e-8);
            var dx = JacobianStepSolver.Solve(jac, -y);
            logger.LogDebug("Next step {Step}.", dx);

            if (!dx.ForAll(double.IsFinite))
                throw new UncriticalIterationInterceptedException(
                    nameof(NewtonRaphsonRootFinder),
                    InterceptReason.InvalidStateOccured,
                    i,
                    furtherInformation: "Infinite step occured."
                );

            if (
                (dx - dxOld).L2Norm() < AbsoluteTolerance
                || (dx + dxOld).L2Norm() < AbsoluteTolerance
            )
                return x;

            if (UseLineSearch)
            {
                objectiveFunction.EvaluateAt(x);
                var lineSearchResult = WolfeLineSearch.FindConformingStep(
                    objectiveFunction,
                    dx,
                    0.5,
                    1
                );
                x = lineSearchResult.MinimizingPoint;
            }
            else
            {
                x += dx;
            }

            y = function(x);
            error = y.L2Norm();
            dxOld = dx;
        }

        logger.LogWarning("Maximum iteration count exceeded. Continuing anyway.");
        return x;
    }

    public int MaxIterationCount { get; } = maxIterationCount;

    public double AbsoluteTolerance { get; } = absoluteTolerance;

    public ILinearSolver JacobianStepSolver { get; } = jacobianStepSolver;

    public bool UseLineSearch { get; } = useLineSearch;

    public WolfeLineSearch WolfeLineSearch { get; } =
        wolfeLineSearch ?? new WeakWolfeLineSearch(1e-4, 0.9, 0.1, 100);
}
