using RefraSin.Coordinates;
using RefraSin.Coordinates.Helpers;
using RefraSin.MaterialData;
using RefraSin.ParticleModel;
using RefraSin.ParticleModel.Nodes;
using RefraSin.TEPSolver.Quantities;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.ParticleModel;

/// <summary>
/// Oberflächenknoten, der Teil einer Korngrenze ist.
/// </summary>
public class GrainBoundaryNode : ContactNodeBase<GrainBoundaryNode>
{
    /// <inheritdoc />
    public GrainBoundaryNode(
        INode node,
        Particle particle,
        Guid? contactedNodeId = null,
        Guid? contactedParticleId = null
    )
        : base(node, particle, contactedNodeId, contactedParticleId) { }

    private GrainBoundaryNode(
        Guid id,
        double r,
        Angle phi,
        Particle particle,
        Guid? contactedNodeId = null,
        Guid? contactedParticleId = null
    )
        : base(id, r, phi, particle, contactedNodeId, contactedParticleId) { }

    /// <inheritdoc />
    public override NodeType Type => NodeType.GrainBoundary;

    /// <inheritdoc />
    public override ToUpperToLower<double> InterfaceEnergy =>
        _interfaceEnergy ??= new ToUpperToLower<double>(
            InterfaceProperties.Energy / 2,
            InterfaceProperties.Energy / 2
        );

    private ToUpperToLower<double>? _interfaceEnergy;

    /// <inheritdoc />
    public override ToUpperToLower<double> InterfaceDiffusionCoefficient =>
        _interfaceDiffusionCoefficient ??= new ToUpperToLower<double>(
            InterfaceProperties.DiffusionCoefficient,
            InterfaceProperties.DiffusionCoefficient
        );

    private ToUpperToLower<double>? _interfaceDiffusionCoefficient;

    /// <inheritdoc />
    public override NodeBase ApplyTimeStep(
        StepVector stepVector,
        double timeStepWidth,
        Particle particle
    )
    {
        var normalDisplacement = stepVector.ItemValue<NormalDisplacement>(this) * timeStepWidth;
        var angle = SurfaceRadiusAngle.ToUpper + SurfaceNormalAngle.ToUpper;
        var newR = CosLaw.C(Coordinates.R, normalDisplacement, angle);
        var dPhi = SinLaw.Alpha(normalDisplacement, newR, angle);

        return new GrainBoundaryNode(
            Id,
            newR,
            Coordinates.Phi + dPhi,
            particle,
            ContactedNodeId,
            ContactedParticleId
        );
    }
}
