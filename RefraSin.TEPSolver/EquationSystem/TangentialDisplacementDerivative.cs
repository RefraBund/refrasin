using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class TangentialDisplacementDerivative : NodeEquationBase<ContactNodeBase>
{
    /// <inheritdoc />
    public TangentialDisplacementDerivative(ContactNodeBase node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value()
    {
        var gibbsTerm =
            (
                Node.GibbsEnergyGradient.Tangential
                + 0.5 * Node.SurfaceDistance.Sum * Step.TangentialStress(Node)
            ) * (1 + Step.LambdaDissipation());
        var requiredConstraintsTerm = Node.VolumeGradient.Tangential * Step.LambdaVolume(Node);
        var contactTerm =
            -Node.ContactDistanceGradient.Tangential * Step.LambdaContactDistance(Node)
            - Node.ContactDirectionGradient.Tangential * Step.LambdaContactDirection(Node);

        return -gibbsTerm + requiredConstraintsTerm + contactTerm;
    }

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative() => throw new NotImplementedException();
}
