using RefraSin.Coordinates;
using RefraSin.Coordinates.Absolute;
using RefraSin.Coordinates.Polar;

namespace RefraSin.ParticleModel;

/// <summary>
/// Represents an immutable record of a particle.
/// </summary>
public record Particle : IParticle
{
    /// <summary>
    /// Represents an immutable record of a particle.
    /// </summary>
    public Particle(
        Guid id,
        PolarPoint centerCoordinates,
        AbsolutePoint absoluteCenterCoordinates,
        Angle rotationAngle,
        Guid materialId,
        IReadOnlyList<INode> nodes,
        IReadOnlyList<INeck>? necks = null
    )
    {
        Id = id;
        CenterCoordinates = centerCoordinates;
        AbsoluteCenterCoordinates = absoluteCenterCoordinates;
        RotationAngle = rotationAngle;
        MaterialId = materialId;
        Nodes = nodes;
        Necks = necks ?? Array.Empty<INeck>();
    }

    /// <summary>
    /// Kopierkonstruktor.
    /// </summary>
    /// <param name="template">Vorlage</param>
    public Particle(IParticle template) : this(
        template.Id,
        template.CenterCoordinates,
        template.AbsoluteCenterCoordinates,
        template.RotationAngle,
        template.MaterialId,
        template.Nodes.Select<INode, INode>(
            k => k switch
            {
                INeckNode nk          => new NeckNode(nk),
                IGrainBoundaryNode ck => new GrainBoundaryNode(ck),
                _                     => new SurfaceNode(k)
            }
        ).ToArray(),
        template.Necks.Select(n => new Neck(n)).ToArray()
    ) { }

    /// <inheritdoc />
    public Guid Id { get; }

    /// <inheritdoc />
    public PolarPoint CenterCoordinates { get; }

    /// <inheritdoc />
    public AbsolutePoint AbsoluteCenterCoordinates { get; }

    /// <inheritdoc />
    public Angle RotationAngle { get; }

    /// <inheritdoc />
    public IReadOnlyList<INode> Nodes { get; }

    IReadOnlyList<INodeSpec> IParticleSpec.NodeSpecs => Nodes;

    /// <inheritdoc />
    public IReadOnlyList<INeck> Necks { get; }

    /// <inheritdoc />
    public Guid MaterialId { get; }
}