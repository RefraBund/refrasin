using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class DissipationEqualityConstraint : GlobalEquationBase
{
    /// <inheritdoc />
    public DissipationEqualityConstraint(SolutionState state, StepVector step)
        : base(state, step) { }

    /// <inheritdoc />
    public override double Value()
    {
        var dissipationNormal = State
            .Nodes.Select(n =>
                -(n.GibbsEnergyGradient.Normal + 0.5 * n.SurfaceDistance.Sum * Step.NormalStress(n))
                * Step.NormalDisplacement(n)
            )
            .Sum();

        var dissipationTangential = State
            .Nodes.OfType<ContactNodeBase>()
            .Select(n =>
                -(
                    n.GibbsEnergyGradient.Tangential
                    + 0.5 * n.SurfaceDistance.Sum * Step.TangentialStress(n)
                ) * Step.TangentialDisplacement(n)
            )
            .Sum();

        var dissipationFunction = State
            .Nodes.Select(n =>
                n.Particle.VacancyVolumeEnergy
                * n.SurfaceDistance.ToUpper
                * Pow(Step.FluxToUpper(n), 2)
                / n.InterfaceDiffusionCoefficient.ToUpper
            )
            .Sum();

        return dissipationNormal + dissipationTangential - dissipationFunction;
    }

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative()
    {
        foreach (var node in State.Nodes)
        {
            yield return (Map.NormalDisplacement(node), -node.GibbsEnergyGradient.Normal);
            yield return (
                Map.FluxToUpper(node),
                -2
                    * node.Particle.VacancyVolumeEnergy
                    * node.SurfaceDistance.ToUpper
                    / node.InterfaceDiffusionCoefficient.ToUpper
                    * Step.FluxToUpper(node)
            );

            if (node is ContactNodeBase contactNode)
                yield return (
                    Map.TangentialDisplacement(contactNode),
                    -node.GibbsEnergyGradient.Tangential
                );
        }
    }
}
