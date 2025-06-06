using RefraSin.Coordinates;
using RefraSin.Coordinates.Polar;
using RefraSin.ParticleModel;
using RefraSin.ParticleModel.Nodes;
using RefraSin.ParticleModel.Particles;
using RefraSin.TEPSolver.Normalization;
using RefraSin.TEPSolver.Quantities;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.ParticleModel;

internal record NodeReturn : IParticleNode, INodeGradients, INodeShifts, INodeFluxes
{
    public NodeReturn(
        NodeBase template,
        StepVector? stepVector,
        ParticleReturn particle,
        INorm norm
    )
    {
        Id = template.Id;
        ParticleId = template.ParticleId;
        Coordinates = new PolarPoint(
            template.Coordinates.Phi,
            template.Coordinates.R * norm.Length,
            particle
        );
        Type = template.Type;
        SurfaceDistance = template.SurfaceDistance * norm.Length;
        SurfaceRadiusAngle = template.SurfaceRadiusAngle;
        AngleDistance = template.AngleDistance;
        Volume = template.Volume * norm.Area;
        SurfaceNormalAngle = template.SurfaceNormalAngle;
        SurfaceTangentAngle = template.SurfaceTangentAngle;
        RadiusNormalAngle = template.RadiusNormalAngle;
        RadiusTangentAngle = template.RadiusTangentAngle;
        InterfaceFlux = stepVector is not null
            ? new ToUpperToLower<double>(
                stepVector.ItemValue<FluxToUpper>(template),
                -stepVector.ItemValue<FluxToUpper>(template.Lower)
            )
                * norm.Area
                / norm.Time
            : 0;
        VolumeFlux = 0;
        TransferFlux = 0;
        GibbsEnergyGradient = template.GibbsEnergyGradient * norm.Energy / norm.Length / norm.Time;
        VolumeGradient = template.VolumeGradient * norm.Length / norm.Time;
        Shift = stepVector is not null
            ? new NormalTangential<double>(
                stepVector.ItemValue<NormalDisplacement>(template),
                stepVector.StepVectorMap.HasItem<TangentialDisplacement>(template)
                    ? stepVector.ItemValue<TangentialDisplacement>(template)
                    : 0
            )
                * norm.Length
                / norm.Time
            : 0;
        Particle = particle;
    }

    public Guid Id { get; }
    public Guid ParticleId { get; }
    public IPolarPoint Coordinates { get; }
    public NodeType Type { get; }
    public ToUpperToLower<double> SurfaceDistance { get; }
    public ToUpperToLower<Angle> SurfaceRadiusAngle { get; }
    public ToUpperToLower<Angle> AngleDistance { get; }
    public ToUpperToLower<double> Volume { get; }
    public ToUpperToLower<Angle> SurfaceNormalAngle { get; }
    public ToUpperToLower<Angle> SurfaceTangentAngle { get; }
    public ToUpperToLower<Angle> RadiusNormalAngle { get; }
    public ToUpperToLower<Angle> RadiusTangentAngle { get; }
    public ToUpperToLower<double> InterfaceFlux { get; }
    public ToUpperToLower<double> VolumeFlux { get; }
    public double TransferFlux { get; }
    public NormalTangential<double> GibbsEnergyGradient { get; }
    public NormalTangential<double> VolumeGradient { get; }
    public NormalTangential<double> Shift { get; }
    public IParticle<IParticleNode> Particle { get; }
}

internal record ContactNodeReturn : NodeReturn
{
    /// <inheritdoc />
    public ContactNodeReturn(
        ContactNodeBase template,
        StepVector? stepVector,
        ParticleReturn particle,
        INorm norm
    )
        : base(template, stepVector, particle, norm)
    {
        ContactedNodeId = template.ContactedNodeId;
        ContactedParticleId = template.ContactedParticleId;
    }

    public Guid ContactedNodeId { get; }
    public Guid ContactedParticleId { get; }
}
