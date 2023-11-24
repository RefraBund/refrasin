namespace RefraSin.Graphs;

public record UndirectedEdge(IVertex Start, IVertex End) : IEdge
{
    public UndirectedEdge(IEdge edge) : this(edge.Start, edge.End) { }

    /// <inheritdoc />
    public virtual bool Equals(IEdge? other)
    {
        if (ReferenceEquals(null, other)) return false;
        if (ReferenceEquals(this, other)) return true;
        return (Start.Equals(other.Start) && End.Equals(other.End)) || (Start.Equals(other.End) && End.Equals(other.Start));
    }

    /// <inheritdoc />
    public override int GetHashCode() => Start.GetHashCode() * End.GetHashCode();

    /// <inheritdoc />
    public bool IsDirected => false;
}