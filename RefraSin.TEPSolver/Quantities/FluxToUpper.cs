using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.Quantities;

public class FluxToUpper : IFlux, INodeItem
{
    private FluxToUpper(NodeBase node)
    {
        Node = node;
    }

    public static INodeItem Create(NodeBase node) => new FluxToUpper(node);

    public double DissipationFactor(StepVector stepVector) =>
        Node.Particle.VacancyVolumeEnergy
        * Node.SurfaceDistance.ToUpper
        / Node.InterfaceDiffusionCoefficient.ToUpper;

    public NodeBase Node { get; }

    public override string ToString() => $"flux to upper from {Node}";
}
