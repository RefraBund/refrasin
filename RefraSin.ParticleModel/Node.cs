using RefraSin.Coordinates.Polar;

namespace RefraSin.ParticleModel;

public record Node(Guid Id, Guid ParticleId, PolarPoint Coordinates) : INode
{
    public Node(INode template) : this(
        template.Id,
        template.ParticleId,
        template.Coordinates
    ) { }
}