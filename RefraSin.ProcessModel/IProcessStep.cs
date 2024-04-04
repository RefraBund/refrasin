namespace RefraSin.ProcessModel;

/// <summary>
/// Interface for classes representing a process step transforming one <see cref="ISystemState"/> to another.
/// </summary>
public interface IProcessStep
{
    /// <summary>
    /// Run the solution procedure of this step.
    /// </summary>
    /// <param name="inputState">the incoming state</param>
    /// <returns></returns>
    ISystemState Solve(ISystemState inputState);

    event EventHandler<SystemStateReportedEventArgs>? SystemStateReported;
    event EventHandler<SystemStateTransitionReportedEventArgs>? SystemStateTransitionReported;

    class SystemStateTransitionReportedEventArgs(IProcessStep processStep, ISystemStateTransition stateTransition) : EventArgs
    {
        public IProcessStep ProcessStep { get; } = processStep;
        public ISystemStateTransition StateTransition { get; } = stateTransition;
    }

    class SystemStateReportedEventArgs(IProcessStep processStep, ISystemState state) : EventArgs
    {
        public IProcessStep ProcessStep { get; } = processStep;
        
        public ISystemState State { get; } = state;
    }
}