using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class TangentialStressConstraintContact : NodeEquationBase<ContactNodeBase>
{
    /// <inheritdoc />
    public TangentialStressConstraintContact(ContactNodeBase node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value() =>
        Step.TangentialStress(Node) - Step.TangentialStress(Node.ContactedNode);

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative() => throw new NotImplementedException();
}
