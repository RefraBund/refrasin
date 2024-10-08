using RefraSin.ParticleModel.Particles;

namespace RefraSin.ParticleModel.Nodes;

public interface INodeContactNeighbors : INodeContact
{
    IParticle ContactedParticle { get; }

    INodeContactNeighbors ContactedNode { get; }
}
