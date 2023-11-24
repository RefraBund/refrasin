namespace RefraSin.Graphs;

public record DirectedEdge(IVertex Start, IVertex End) : IEdge
{
    /// <inheritdoc />
    public virtual bool Equals(IEdge? other)
    {
        if (ReferenceEquals(null, other)) return false;
        if (ReferenceEquals(this, other)) return true;
        return Start.Equals(other.Start) && End.Equals(other.End);
    }

    /// <inheritdoc />
    public override int GetHashCode() => HashCode.Combine(Start, End);

    /// <inheritdoc />
    public bool IsDirected => true;
}