using RefraSin.Numerics.Exceptions;
using RefraSin.ParticleModel.Remeshing;
using RefraSin.ParticleModel.System;
using RefraSin.ProcessModel;
using RefraSin.ProcessModel.Sintering;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.Recovery;
using RefraSin.TEPSolver.StepValidators;
using RefraSin.TEPSolver.StepVectors;
using RefraSin.TEPSolver.TimeSteppers;

namespace RefraSin.TEPSolver;

/// <summary>
/// Solver for performing time integration of sintering processes based on the thermodynamic extremal principle (TEP).
/// </summary>
public class SinteringSolver : IProcessStepSolver<ISinteringStep>
{
    public SinteringSolver(ISolverRoutines routines, int remeshingEverySteps = 10)
    {
        Routines = routines;
        RemeshingEverySteps = remeshingEverySteps;
        routines.RegisterWithSolver(this);
    }

    /// <summary>
    /// Collection of subroutines to use.
    /// </summary>
    public ISolverRoutines Routines { get; }

    /// <summary>
    /// Count of time steps to compute before a remeshing is performed.
    /// </summary>
    public int RemeshingEverySteps { get; }

    /// <summary>
    /// Run the solution procedure starting with the given state till the specified time.
    /// </summary>
    public ISystemState Solve(ISinteringStep processStep, ISystemState inputState)
    {
        var session = new SolverSession(this, inputState, processStep);
        InvokeSessionInitialized(session);
        session.ReportCurrentState();
        TryTimeIntegration(ref session);

        return new SystemState(
            session.CurrentState.Id,
            session.CurrentState.Time,
            session.CurrentState.Particles
        );
    }

    private void TryTimeIntegration(ref SolverSession session)
    {
        try
        {
            DoTimeIntegration(ref session);
        }
        catch (Exception e)
        {
            session.Logger.Error(e, "Solution procedure failed due to exception.");
            InvokeSolutionFailed(session.CurrentState);
            throw;
        }
    }

    private void DoTimeIntegration(ref SolverSession session)
    {
        session.Logger.Information("Starting time integration.");

        int i = 0;
        var recoverersArray = session.Routines.StateRecoverers.ToArray();

        while (session.CurrentState.Time < session.EndTime)
        {
            var stepVector = SolveStepUntilValid(session, session.CurrentState, recoverersArray);
            var timeStepWidth = session.Routines.StepWidthController.GetStepWidth(
                session,
                session.CurrentState,
                stepVector
            );
            var newState = session.CurrentState.ApplyTimeStep(stepVector, timeStepWidth);

            InvokeStepSuccessfullyCalculated(session, session.CurrentState, newState, stepVector);

            session.CurrentState = newState;
            session.ReportCurrentState(stepVector);

            session.Logger.Information(
                "Time step {Index} successfully calculated. ({Time:e2}/{EndTime:e2} = {Percent:f2}%)",
                i,
                session.CurrentState.Time,
                session.EndTime,
                session.CurrentState.Time / session.EndTime * 100
            );
            i++;

            if (i % RemeshingEverySteps == 0)
            {
                session.CurrentState.Sanitize();
                var remeshedState = new SystemState(
                    Guid.NewGuid(),
                    session.CurrentState.Time,
                    session.Routines.Remeshers.Aggregate<
                        IParticleSystemRemesher,
                        IParticleSystem<IParticle<IParticleNode>, IParticleNode>
                    >(session.CurrentState, (state, remesher) => remesher.RemeshSystem(state))
                );

                session = new SolverSession(session, remeshedState);
                InvokeSessionInitialized(session);
                session.Logger.Information(
                    "Remeshed session created. Now {NodeCount} nodes present.",
                    remeshedState.Nodes.Count
                );
                session.ReportCurrentState();
            }
        }

        session.Logger.Information("End time successfully reached after {StepCount} steps.", i);
    }

    private StepVector SolveStepUntilValid(
        SolverSession session,
        SolutionState baseState,
        IStateRecoverer[] recoverers
    )
    {
        StepVector? stepVector;

        try
        {
            stepVector = session.Routines.TimeStepper.Step(session, baseState);
        }
        catch (StepFailedException failedException)
        {
            session.Logger.Error(failedException, "Step calculation failed. Trying to recover.");
            InvokeStepFailed(session, baseState);

            try
            {
                stepVector = TryRecover(session, baseState, recoverers);
            }
            catch (RecoveryFailedException recoveryFailedException)
            {
                session.Logger.Error(
                    recoveryFailedException,
                    "Recovery failed. Trying next recoverer."
                );
                stepVector = TryRecover(session, baseState, recoverers[1..]);
            }
        }

        try
        {
            foreach (var validator in session.Routines.StepValidators)
            {
                validator.Validate(baseState, stepVector);
            }
        }
        catch (InvalidStepException stepRejectedException)
        {
            session.Logger.Error(
                stepRejectedException,
                "Calculated step was rejected. Trying to recover."
            );
            InvokeStepRejected(session, baseState, stepVector);

            try
            {
                stepVector = TryRecover(session, baseState, recoverers);
            }
            catch (RecoveryFailedException recoveryFailedException)
            {
                session.Logger.Error(
                    recoveryFailedException,
                    "Recovery failed. Trying next recoverer."
                );
                stepVector = TryRecover(session, baseState, recoverers[1..]);
            }
        }

        return stepVector;
    }

    private StepVector TryRecover(
        SolverSession session,
        SolutionState invalidState,
        IStateRecoverer[] recoverers
    )
    {
        try
        {
            var recoveredState = recoverers[0].RecoverState(session, invalidState);
            return SolveStepUntilValid(session, recoveredState, recoverers);
        }
        catch (IndexOutOfRangeException)
        {
            session.Logger.Error("No more recoverers available.");
            throw new CriticalIterationInterceptedException(
                nameof(SolveStepUntilValid),
                InterceptReason.ExceptionOccured,
                furtherInformation: "Recovery of solution finally failed."
            );
        }
    }

    public event EventHandler<StepSuccessfullyCalculatedEventArgs>? StepSuccessfullyCalculated;

    private void InvokeStepSuccessfullyCalculated(
        SolverSession solverSession,
        SolutionState oldState,
        SolutionState newState,
        StepVector stepVector
    )
    {
        StepSuccessfullyCalculated?.Invoke(
            this,
            new StepSuccessfullyCalculatedEventArgs(solverSession, oldState, newState, stepVector)
        );
    }

    public event EventHandler<StepRejectedEventArgs>? StepRejected;

    private void InvokeStepRejected(
        SolverSession solverSession,
        SolutionState baseState,
        StepVector stepVector
    )
    {
        StepRejected?.Invoke(this, new StepRejectedEventArgs(solverSession, baseState, stepVector));
    }

    public event EventHandler<SessionInitializedEventArgs>? SessionInitialized;

    private void InvokeSessionInitialized(SolverSession solverSession)
    {
        SessionInitialized?.Invoke(this, new SessionInitializedEventArgs(solverSession));
    }

    public class StepSuccessfullyCalculatedEventArgs(
        ISolverSession solverSession,
        SolutionState oldState,
        SolutionState newState,
        StepVector stepVector
    ) : EventArgs
    {
        public ISolverSession SolverSession { get; } = solverSession;
        public SolutionState OldState { get; } = oldState;
        public SolutionState NewState { get; } = newState;
        public StepVector StepVector { get; } = stepVector;
    }

    public class StepRejectedEventArgs(
        ISolverSession solverSession,
        SolutionState baseState,
        StepVector stepVector
    ) : EventArgs
    {
        public ISolverSession SolverSession { get; } = solverSession;
        public SolutionState BaseState { get; } = baseState;
        public StepVector StepVector { get; } = stepVector;
    }

    public class SessionInitializedEventArgs(ISolverSession solverSession) : EventArgs
    {
        public ISolverSession SolverSession { get; } = solverSession;
    }

    public event EventHandler<StepFailedEventArgs>? StepFailed;

    private void InvokeStepFailed(SolverSession solverSession, SolutionState baseState)
    {
        StepFailed?.Invoke(this, new StepFailedEventArgs(solverSession, baseState));
    }

    public class StepFailedEventArgs(ISolverSession solverSession, SolutionState baseState)
        : EventArgs
    {
        public ISolverSession SolverSession { get; } = solverSession;
        public SolutionState BaseState { get; } = baseState;
    }

    public event EventHandler<SolutionFailedEventArgs>? SolutionFailed;

    private void InvokeSolutionFailed(SolutionState lastState)
    {
        SolutionFailed?.Invoke(this, new SolutionFailedEventArgs(lastState));
    }

    public class SolutionFailedEventArgs(SolutionState lastState) : EventArgs
    {
        public SolutionState LastState { get; } = lastState;
    }
}
