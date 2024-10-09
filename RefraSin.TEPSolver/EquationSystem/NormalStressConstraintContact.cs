using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class NormalStressConstraintContact : NodeEquationBase<ContactNodeBase>
{
    /// <inheritdoc />
    public NormalStressConstraintContact(ContactNodeBase node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value() =>
        Step.NormalStress(Node) - Step.NormalStress(Node.ContactedNode);

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative() => throw new NotImplementedException();
}
