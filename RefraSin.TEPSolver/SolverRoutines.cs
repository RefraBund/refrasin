using RefraSin.Numerics.LinearSolvers;
using RefraSin.Numerics.RootFinding;
using RefraSin.TEPSolver.Normalization;
using RefraSin.TEPSolver.Recovery;
using RefraSin.TEPSolver.RootFinding;
using RefraSin.TEPSolver.StepEstimators;
using RefraSin.TEPSolver.StepValidators;
using RefraSin.TEPSolver.StepWidthControllers;
using RefraSin.TEPSolver.TimeSteppers;

namespace RefraSin.TEPSolver;

public record SolverRoutines(
    IStepEstimator StepEstimator,
    ITimeStepper TimeStepper,
    IEnumerable<IStepValidator> StepValidators,
    ILagrangianRootFinder LagrangianRootFinder,
    INormalizer Normalizer,
    IStepWidthController StepWidthController,
    IEnumerable<IStateRecoverer> StateRecoverers) : ISolverRoutines
{
    public static readonly SolverRoutines Default = new(
        new StepEstimator(),
        new AdamsMoultonTimeStepper(),
        new IStepValidator[]
        {
            // new InstabilityDetector()
        },
        new TearingLagrangianRootFinder(
            new NewtonRaphsonRootFinder(
                new LUSolver()
            ),
            new NewtonRaphsonRootFinder(
                new LUSolver()
            )
        ),
        new DefaultNormalizer(),
        new TrialAndErrorStepWidthController(),
        new IStateRecoverer[]
        {
            new StepBackStateRecoverer()
        }
    );

    /// <inheritdoc />
    public void RegisterWithSolver(SinteringSolver solver)
    {
        StepEstimator.RegisterWithSolver(solver);
        TimeStepper.RegisterWithSolver(solver);
        LagrangianRootFinder.RegisterWithSolver(solver);
        Normalizer.RegisterWithSolver(solver);
        StepWidthController.RegisterWithSolver(solver);

        foreach (var validator in StepValidators)
            validator.RegisterWithSolver(solver);
        foreach (var recoverer in StateRecoverers)
            recoverer.RegisterWithSolver(solver);
    }
}