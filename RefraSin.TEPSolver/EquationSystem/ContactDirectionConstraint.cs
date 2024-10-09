using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class ContactDirectionConstraint : NodeEquationBase<ContactNodeBase>
{
    /// <inheritdoc />
    public ContactDirectionConstraint(ContactNodeBase node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value() =>
        Step.AngleDisplacement(Node.Contact)
        - Node.ContactDirectionGradient.Normal * Step.NormalDisplacement(Node)
        - Node.ContactedNode.ContactDirectionGradient.Normal
            * Step.NormalDisplacement(Node.ContactedNode)
        - Node.ContactDirectionGradient.Tangential * Step.TangentialDisplacement(Node)
        - Node.ContactedNode.ContactDirectionGradient.Tangential
            * Step.TangentialDisplacement(Node.ContactedNode)
        - Node.ContactedNode.Coordinates.R
            / Node.ContactDistance
            * Cos(Node.ContactedNode.AngleDistanceToContactDirection)
            * Step.RotationDisplacement(Node.Contact);

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative() => throw new NotImplementedException();
}
