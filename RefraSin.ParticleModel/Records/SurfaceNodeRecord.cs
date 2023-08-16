using RefraSin.Coordinates.Absolute;
using RefraSin.Coordinates.Polar;

namespace RefraSin.ParticleModel.Records;

/// <summary>
/// Represents an immutable record of a surface node.
/// </summary>
public record SurfaceNodeRecord : NodeRecord, ISurfaceNode
{
    /// <inheritdoc />
    public SurfaceNodeRecord(INode template) : base(template) { }

    /// <summary>
    /// Represents an immutable record of a surface node.
    /// </summary>
    public SurfaceNodeRecord(Guid Id,
        Guid ParticleId,
        PolarPoint Coordinates,
        AbsolutePoint AbsoluteCoordinates,
        ToUpperToLower SurfaceDistance,
        ToUpperToLowerAngle SurfaceRadiusAngle,
        ToUpperToLowerAngle AngleDistance,
        ToUpperToLower Volume,
        NormalTangentialAngle SurfaceAngle,
        ToUpperToLower SurfaceEnergy,
        ToUpperToLower SurfaceDiffusionCoefficient,
        NormalTangential GibbsEnergyGradient,
        NormalTangential VolumeGradient
    ) : base(
        Id,
        ParticleId,
        Coordinates,
        AbsoluteCoordinates,
        SurfaceDistance,
        SurfaceRadiusAngle,
        AngleDistance,
        Volume,
        SurfaceAngle,
        SurfaceEnergy,
        SurfaceDiffusionCoefficient,
        GibbsEnergyGradient,
        VolumeGradient
    ) { }
}