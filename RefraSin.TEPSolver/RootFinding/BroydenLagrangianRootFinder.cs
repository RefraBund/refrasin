using MathNet.Numerics.RootFinding;
using Microsoft.Extensions.Logging;
using RefraSin.TEPSolver.EquationSystem;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.RootFinding;

public class BroydenLagrangianRootFinder(int maxIterationCount = 100, double accuracy = 1e-8) : ILagrangianRootFinder
{
    /// <inheritdoc />
    public StepVector FindRoot(
        SolutionState currentState,
        StepVector initialGuess,
        ILogger logger
    )
    {
        double[] Fun(double[] vector)
        {
            var result = Lagrangian.EvaluateAt(currentState, new StepVector(vector, initialGuess.StepVectorMap)).AsArray();
            return result;
        }

        var result = Broyden.FindRoot(
            Fun,
            initialGuess: initialGuess.AsArray(),
            maxIterations: MaxIterationCount,
            accuracy: Accuracy
        );

        return new StepVector(result, initialGuess.StepVectorMap);
    }

    public int MaxIterationCount { get; } = maxIterationCount;

    public double Accuracy { get; } = accuracy;

    /// <inheritdoc />
    public void RegisterWithSolver(SinteringSolver solver) { }
}