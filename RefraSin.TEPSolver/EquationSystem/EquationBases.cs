using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public abstract class GlobalEquationBase : IEquation
{
    protected readonly SolutionState State;
    protected readonly StepVector Step;

    public GlobalEquationBase(SolutionState state, StepVector step)
    {
        State = state;
        Step = step;
    }

    /// <inheritdoc />
    public abstract double Value();

    /// <inheritdoc />
    public abstract IEnumerable<(int, double)> Derivative();
}

public abstract class ParticleEquationBase : IEquation
{
    protected readonly Particle Particle;
    protected readonly StepVector Step;

    public ParticleEquationBase(Particle particle, StepVector step)
    {
        Particle = particle;
        Step = step;
    }

    /// <inheritdoc />
    public abstract double Value();

    /// <inheritdoc />
    public abstract IEnumerable<(int, double)> Derivative();
}

public abstract class ContactEquationBase : IEquation
{
    protected readonly ParticleContact Contact;
    protected readonly StepVector Step;

    public ContactEquationBase(ParticleContact contact, StepVector step)
    {
        Contact = contact;
        Step = step;
    }

    /// <inheritdoc />
    public abstract double Value();

    /// <inheritdoc />
    public abstract IEnumerable<(int, double)> Derivative();
}

public abstract class NodeEquationBase<TNode> : IEquation
{
    protected readonly TNode Node;
    protected readonly StepVector Step;

    public NodeEquationBase(TNode node, StepVector step)
    {
        Node = node;
        Step = step;
    }

    /// <inheritdoc />
    public abstract double Value();

    /// <inheritdoc />
    public abstract IEnumerable<(int, double)> Derivative();
}
