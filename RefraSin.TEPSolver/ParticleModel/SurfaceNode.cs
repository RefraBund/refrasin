using RefraSin.Coordinates;
using RefraSin.Coordinates.Helpers;
using RefraSin.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.ParticleModel;

/// <summary>
/// Oberflächenknoten, der Teil einer freien Oberfläche ist.
/// </summary>
public class SurfaceNode : NodeBase, ISurfaceNode
{
    /// <inheritdoc />
    public SurfaceNode(INode node, Particle particle, ISolverSession solverSession) : base(node, particle, solverSession) { }

    private SurfaceNode(Guid id, double r, Angle phi, Particle particle, ISolverSession solverSession) : base(id, r, phi, particle, solverSession) { }

    /// <inheritdoc />
    public override NodeType Type => NodeType.SurfaceNode;

    /// <inheritdoc />
    public override ToUpperToLower<double> SurfaceEnergy => _surfaceEnergy ??= new ToUpperToLower<double>(
        Particle.Material.SurfaceEnergy,
        Particle.Material.SurfaceEnergy
    );

    private ToUpperToLower<double>? _surfaceEnergy;

    /// <inheritdoc />
    public override ToUpperToLower<double> SurfaceDiffusionCoefficient => _surfaceDiffusionCoefficient ??= new ToUpperToLower<double>(
        Particle.Material.SurfaceDiffusionCoefficient,
        Particle.Material.SurfaceDiffusionCoefficient
    );

    private ToUpperToLower<double>? _surfaceDiffusionCoefficient;

    /// <inheritdoc />
    public override double TransferCoefficient => 0;

    /// <inheritdoc />
    public override NodeBase ApplyTimeStep(StepVector stepVector, double timeStepWidth, Particle particle)
    {
        var normalDisplacement = stepVector.NormalDisplacement(this) * timeStepWidth;
        var angle = SurfaceRadiusAngle.ToUpper + SurfaceVectorAngle.Normal;
        var newR = CosLaw.C(Coordinates.R, normalDisplacement, angle);
        var dPhi = SinLaw.Alpha(normalDisplacement, newR, angle);

        return new SurfaceNode(Id, newR, Coordinates.Phi + dPhi, particle, SolverSession);
    }
}