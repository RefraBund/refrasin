using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.TimeSteppers;

public class AdamsMoultonTimeStepper : ITimeStepper
{
    /// <inheritdoc />
    public StepVector Step(
        ISolverSession solverSession,
        SolutionState baseState,
        StepVector initialGuess
    )
    {
        var step = solverSession.Routines.LagrangianRootFinder.FindRoot(
            baseState,
            initialGuess,
            solverSession.Logger
        );

        if (_lastSteps.TryGetValue(solverSession.Id, out var lastStep))
            return (step + lastStep) / 2;

        return step;
    }

    private readonly Dictionary<Guid, StepVector> _lastSteps = new();

    /// <inheritdoc />
    public void RegisterWithSolver(SinteringSolver solver)
    {
        solver.StepSuccessfullyCalculated += HandleStepSuccessfullyCalculated;
    }

    private void HandleStepSuccessfullyCalculated(
        object? sender,
        SinteringSolver.StepSuccessfullyCalculatedEventArgs e
    )
    {
        _lastSteps[e.SolverSession.Id] = e.StepVector;
    }
}
