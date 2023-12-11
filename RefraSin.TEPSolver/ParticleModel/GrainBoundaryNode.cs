using RefraSin.Coordinates;
using RefraSin.Coordinates.Helpers;
using RefraSin.MaterialData;
using RefraSin.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.ParticleModel;

/// <summary>
/// Oberflächenknoten, der Teil einer Korngrenze ist.
/// </summary>
public class GrainBoundaryNode : ContactNodeBase<GrainBoundaryNode>, IGrainBoundaryNode
{
    /// <inheritdoc />
    public GrainBoundaryNode(INode node, Particle particle, ISolverSession solverSession) : base(node, particle, solverSession) { }

    private GrainBoundaryNode(Guid id, double r, Angle phi, Particle particle, ISolverSession solverSession, Guid contactedNodeId,
        Guid contactedParticleId) : base(id, r, phi, particle,
        solverSession, contactedNodeId, contactedParticleId) { }

    /// <inheritdoc />
    public override NodeType Type => NodeType.GrainBoundaryNode;

    /// <inheritdoc />
    public override ToUpperToLower SurfaceEnergy => _surfaceEnergy ??= new ToUpperToLower(
        MaterialInterface.InterfaceEnergy,
        MaterialInterface.InterfaceEnergy
    );

    private ToUpperToLower? _surfaceEnergy;

    /// <inheritdoc />
    public override ToUpperToLower SurfaceDiffusionCoefficient => _surfaceDiffusionCoefficient ??= new ToUpperToLower(
        MaterialInterface.DiffusionCoefficient,
        MaterialInterface.DiffusionCoefficient
    );

    private ToUpperToLower? _surfaceDiffusionCoefficient;

    /// <inheritdoc />
    public override double TransferCoefficient => 0;

    /// <inheritdoc />
    public override NodeBase ApplyTimeStep(StepVector stepVector, double timeStepWidth, Particle particle)
    {
        var normalDisplacement = stepVector[this].NormalDisplacement * timeStepWidth;
        var angle = SurfaceRadiusAngle.ToUpper + SurfaceVectorAngle.Normal;
        var newR = CosLaw.C(Coordinates.R, normalDisplacement, angle);
        var dPhi = SinLaw.Alpha(normalDisplacement, newR, angle);

        return new GrainBoundaryNode(Id, newR, Coordinates.Phi + dPhi, particle, SolverSession, ContactedNodeId, ContactedParticleId);
    }
}