namespace RefraSin.ParticleModel.Remeshing;

public interface IParticleRemesher
{
    IParticle Remesh(IParticle particle);
}