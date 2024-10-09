using RefraSin.Coordinates;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class NormalStressDerivative : NodeEquationBase<NodeBase>
{
    /// <inheritdoc />
    public NormalStressDerivative(NodeBase node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value()
    {
        var gibbsTerm =
            0.5
            * Node.SurfaceDistance.Sum
            * Step.NormalDisplacement(Node)
            * (1 + Step.LambdaDissipation());
        var constraintsTerm = Node.Type == NodeType.Surface ? Step.LambdaNormalStress(Node) : 0;
        var horizontalTerm =
            Cos(Node.Coordinates.Phi + (Angle.Half - Node.RadiusNormalAngle.ToUpper))
            * Step.LambdaHorizontalForceBalance(Node.Particle);
        var verticalTerm =
            Sin(Node.Coordinates.Phi + (Angle.Half - Node.RadiusNormalAngle.ToUpper))
            * Step.LambdaVerticalForceBalance(Node.Particle);
        var torqueTerm =
            Step.LambdaTorqueBalance(Node.Particle)
            * Sin(Node.RadiusNormalAngle.ToUpper)
            * Node.Coordinates.R;
        var contactTerm = Node is ContactNodeBase contactNode
            ? Step.LambdaNormalStress(contactNode) * (contactNode.IsParentsNode ? 1 : -1)
            : 0;

        return -gibbsTerm
            + constraintsTerm
            + horizontalTerm
            + verticalTerm
            + torqueTerm
            + contactTerm;
    }

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative() => throw new NotImplementedException();
}
