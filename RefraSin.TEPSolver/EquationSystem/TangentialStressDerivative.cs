using RefraSin.Coordinates;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class TangentialStressDerivative : NodeEquationBase<NodeBase>
{
    /// <inheritdoc />
    public TangentialStressDerivative(NodeBase node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value()
    {
        var gibbsTerm = Step.TangentialDisplacement(Node) * (1 + Step.LambdaDissipation());
        var constraintsTerm = Node.Type == NodeType.Surface ? Step.LambdaTangentialStress(Node) : 0;
        var horizontalTerm =
            Cos(Node.Coordinates.Phi + (Angle.Half - Node.RadiusTangentAngle.ToUpper))
            * Step.LambdaHorizontalForceBalance(Node.Particle);
        var verticalTerm =
            Sin(Node.Coordinates.Phi + (Angle.Half - Node.RadiusTangentAngle.ToUpper))
            * Step.LambdaVerticalForceBalance(Node.Particle);
        var torqueTerm =
            Step.LambdaTorqueBalance(Node.Particle)
            * Sin(Node.RadiusTangentAngle.ToUpper)
            * Node.Coordinates.R;
        var contactTerm = Node is ContactNodeBase contactNode
            ? Step.LambdaTangentialStress(contactNode) * (contactNode.IsParentsNode ? 1 : -1)
            : 0;

        return gibbsTerm
            + constraintsTerm
            + horizontalTerm
            + verticalTerm
            + torqueTerm
            + contactTerm;
    }

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative()
    {
        yield return (Map.TangentialDisplacement(Node), 1 + Step.LambdaDissipation());
        yield return (Map.LambdaDissipation(), Step.TangentialDisplacement(Node));
        yield return (
            Map.LambdaHorizontalForceBalance(Particle),
            Cos(Node.Coordinates.Phi + (Angle.Half - Node.RadiusTangentAngle.ToUpper))
        );
        yield return (
            Map.LambdaVerticalForceBalance(Particle),
            Sin(Node.Coordinates.Phi + (Angle.Half - Node.RadiusTangentAngle.ToUpper))
        );
        yield return (
            Map.LambdaTorqueBalance(Particle),
            Sin(Node.RadiusTangentAngle.ToUpper) * Node.Coordinates.R
        );
        if (Node is ContactNodeBase contactNode)
        {
            yield return (Map.LambdaTangentialStress(Node), contactNode.IsParentsNode ? 1 : -1);
        }
        else
        {
            yield return (Map.LambdaTangentialStress(Node), 1);
        }
    }
}
