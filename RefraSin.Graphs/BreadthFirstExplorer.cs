namespace RefraSin.Graphs;

public class BreadthFirstExplorer<TVertex> : IGraphTraversal<TVertex> where TVertex : IVertex
{
    private readonly Edge<TVertex>[] _exploredEdges;

    private BreadthFirstExplorer(TVertex start, Edge<TVertex>[] exploredEdges)
    {
        Start = start;
        _exploredEdges = exploredEdges;
    }

    public static BreadthFirstExplorer<TVertex> Explore<TEdge>(IGraph<TVertex, TEdge> graph, TVertex start) where TEdge : IEdge<TVertex> =>
        new(
            start,
            DoExplore(graph, start).ToArray()
        );

    public static BreadthFirstExplorer<TVertex> Explore<TEdge>(IRootedGraph<TVertex, TEdge> graph) where TEdge : IEdge<TVertex> =>
        new(
            graph.Root,
            DoExplore(graph, graph.Root).ToArray()
        );

    private static IEnumerable<Edge<TVertex>> DoExplore<TEdge>(IGraph<TVertex, TEdge> graph, TVertex start) where TEdge : IEdge<TVertex>
    {
        var verticesVisited = new HashSet<IVertex>(graph.VertexCount) { start };
        var edgesVisited = new HashSet<TEdge>(graph.EdgeCount);

        var queue = new Queue<TVertex>();
        queue.Enqueue(start);

        while (queue.TryDequeue(out var current))
        {
            foreach (var edge in graph.EdgesFrom(current))
            {
                if (edgesVisited.Contains(edge, EqualityComparer<TEdge>.Default))
                    continue;

                edgesVisited.Add(edge);

                var child = edge.From.Equals(current) ? edge.To : edge.From;

                if (verticesVisited.Contains(child))
                {
                    yield return new Edge<TVertex>(current, child, true);
                    continue;
                }

                yield return new Edge<TVertex>(current, child, true);

                verticesVisited.Add(child);
                queue.Enqueue(child);
            }
        }
    }

    /// <inheritdoc />
    public TVertex Start { get; }

    /// <inheritdoc />
    public IEnumerable<IEdge<TVertex>> TraversedEdges => _exploredEdges;
}