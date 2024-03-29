namespace RefraSin.ParticleModel;

public interface INodeTimeStep
{
    /// <summary>
    /// Unique ID of the node this time step belongs to.
    /// </summary>
    public Guid NodeId { get; }

    /// <summary>
    /// Distance od displacement in normal direction to the surface.
    /// </summary>
    public double NormalDisplacement { get; }

    /// <summary>
    /// Distance od displacement in tangential direction to the surface.
    /// </summary>
    public double TangentialDisplacement { get; }

    /// <summary>
    /// Diffusional flow to/from the neighbor nodes.
    /// </summary>
    public ToUpperToLower<double> DiffusionalFlow { get; }

    /// <summary>
    /// Diffusional flow to/from the environment (e.g. matrix, neighbor particle).
    /// </summary>
    public double OuterDiffusionalFlow { get; }
}