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
    public override IEnumerable<(int, double)> Derivative()
    {
        yield return (
            Map.LambdaDissipation(),
            -(
                Node.GibbsEnergyGradient.Tangential
                + 0.5 * Node.SurfaceDistance.Sum * Step.TangentialStress(Node)
            )
        );
        yield return (
            Map.TangentialStress(Node),
            -0.5 * Node.SurfaceDistance.Sum * Step.LambdaDissipation()
        );
        yield return (Map.LambdaVolume(Node), Node.VolumeGradient.Tangential);
        yield return (Map.LambdaContactDistance(Node), -Node.ContactDistanceGradient.Tangential);
        yield return (Map.LambdaContactDirection(Node), -Node.ContactDirectionGradient.Tangential);
    }
}
