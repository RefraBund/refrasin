using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class VolumeBalanceConstraint : NodeEquationBase<NodeBase>
{
    /// <inheritdoc />
    public VolumeBalanceConstraint(NodeBase node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value()
    {
        var normalVolumeTerm = Node.VolumeGradient.Normal * Step.NormalDisplacement(Node);
        var tangentialVolumeTerm = 0.0;

        if (Node is ContactNodeBase contactNode)
        {
            tangentialVolumeTerm =
                Node.VolumeGradient.Tangential * Step.TangentialDisplacement(contactNode);
        }

        var fluxTerm = Step.FluxToUpper(Node) - Step.FluxToUpper(Node.Lower);

        return normalVolumeTerm + tangentialVolumeTerm - fluxTerm;
    }

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative() => throw new NotImplementedException();
}
