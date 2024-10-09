using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class ContactDistanceConstraint : NodeEquationBase<ContactNodeBase>
{
    /// <inheritdoc />
    public ContactDistanceConstraint(ContactNodeBase node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value() =>
        Step.RadialDisplacement(Node.Contact)
        - Node.ContactDistanceGradient.Normal * Step.NormalDisplacement(Node)
        - Node.ContactedNode.ContactDistanceGradient.Normal
            * Step.NormalDisplacement(Node.ContactedNode)
        - Node.ContactDistanceGradient.Tangential * Step.TangentialDisplacement(Node)
        - Node.ContactedNode.ContactDistanceGradient.Tangential
            * Step.TangentialDisplacement(Node.ContactedNode)
        + Node.ContactedNode.Coordinates.R
            * Sin(Node.ContactedNode.AngleDistanceToContactDirection)
            * Step.RotationDisplacement(Node.Contact);

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative() => throw new NotImplementedException();
}
