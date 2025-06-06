using RefraSin.ProcessModel.Sintering;
using RefraSin.TEPSolver.Normalization;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver;

/// <summary>
/// Interface for objects holding session data of a solution procedure.
/// </summary>
public interface ISolverSession : ISinteringConditions
{
    /// <summary>
    /// Unique ID of this session.
    /// </summary>
    Guid Id { get; }

    public SolutionState CurrentState { get; }

    /// <summary>
    /// COllection of solver routines to use.
    /// </summary>
    public ISolverRoutines Routines { get; }

    INorm Norm { get; }
}
