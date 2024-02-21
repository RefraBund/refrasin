using RefraSin.TEPSolver.EquationSystem;
using RefraSin.TEPSolver.RootFinding;
using RefraSin.TEPSolver.StepValidators;
using RefraSin.TEPSolver.TimeSteppers;

namespace RefraSin.TEPSolver;

public record SolverRoutines(
    IStepEstimator StepEstimator,
    ITimeStepper TimeStepper,
    IEnumerable<IStepValidator> StepValidators,
    IRootFinder RootFinder
) : ISolverRoutines
{
    public static SolverRoutines Default = new SolverRoutines(
        null,
        new AdamsMoultonTimeStepper(),
        new[]
        {
            new InstabilityDetector()
        },
        new BroydenRootFinder()
    );
}