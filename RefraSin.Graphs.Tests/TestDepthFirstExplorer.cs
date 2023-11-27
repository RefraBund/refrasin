namespace RefraSin.Graphs.Tests;

[TestFixture]
public class TestDepthFirstExplorer
{
    private Vertex[] _vertices;
    private IEdge<Vertex>[] _edges;
    private DirectedGraph<Vertex> _graph;

    [SetUp]
    public void Setup()
    {
        _vertices = new Vertex[]
        {
            new Vertex(Guid.NewGuid(), "0"),
            new Vertex(Guid.NewGuid(), "1"),
            new Vertex(Guid.NewGuid(), "2"),
            new Vertex(Guid.NewGuid(), "3"),
            new Vertex(Guid.NewGuid(), "4"),
            new Vertex(Guid.NewGuid(), "5"),
        };

        _edges = new IEdge<Vertex>[]
        {
            new DirectedEdge<Vertex>(_vertices[0], _vertices[1]),
            new DirectedEdge<Vertex>(_vertices[1], _vertices[2]),
            new DirectedEdge<Vertex>(_vertices[1], _vertices[3]),
            new DirectedEdge<Vertex>(_vertices[0], _vertices[4]),
            new DirectedEdge<Vertex>(_vertices[2], _vertices[5]),
        };
    }

    [Test]
    public void TestOrderDirected()
    {
        var graph = new DirectedGraph<Vertex>(_vertices, _edges);

        var explorer = DepthFirstExplorer<Vertex>.Explore(graph, _vertices[0]);

        using var enm = explorer.ExploredEdges.GetEnumerator();

        for (int i = 0; i < 2; i++)
        {
            enm.MoveNext();

            if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[0], _vertices[1])))
            {
                for (int j = 0; j < 2; j++)
                {
                    enm.MoveNext();
                    if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[1], _vertices[2])))
                    {
                        enm.MoveNext();
                        Assert.That(enm.Current, Is.EqualTo(new DirectedEdge<Vertex>(_vertices[2], _vertices[5])));
                        continue;
                    }

                    if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[1], _vertices[3])))
                        continue;
                    Assert.Fail("Wrong edges from Vertex 1.");
                }

                continue;
            }

            if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[0], _vertices[4])))
                continue;

            Assert.Fail("Wrong edges from Vertex 0.");
        }
    }

    [Test]
    public void TestOrderUndirected()
    {
        var graph = new UndirectedGraph<Vertex>(_vertices, _edges);

        var explorer = DepthFirstExplorer<Vertex>.Explore(graph, _vertices[0]);

        using var enm = explorer.ExploredEdges.GetEnumerator();

        for (int i = 0; i < 2; i++)
        {
            enm.MoveNext();

            if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[0], _vertices[1])))
            {
                for (int j = 0; j < 3; j++)
                {
                    enm.MoveNext();
                    if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[1], _vertices[2])))
                    {
                        for (int k = 0; k < 2; k++)
                        {
                            enm.MoveNext();
                            if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[2], _vertices[5])))
                            {
                                enm.MoveNext();
                                Assert.That(enm.Current, Is.EqualTo(new DirectedEdge<Vertex>(_vertices[5], _vertices[2])));
                                continue;
                            }

                            if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[2], _vertices[1])))
                                continue;
                            Assert.Fail("Wrong edges from Vertex 2.");
                        }

                        continue;
                    }

                    if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[1], _vertices[3])))
                    {
                        enm.MoveNext();
                        Assert.That(enm.Current, Is.EqualTo(new DirectedEdge<Vertex>(_vertices[3], _vertices[1])));
                        continue;
                    }

                    if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[1], _vertices[0])))
                        continue;
                    Assert.Fail("Wrong edges from Vertex 1.");
                }

                continue;
            }

            if (enm.Current.Equals(new DirectedEdge<Vertex>(_vertices[0], _vertices[4])))
                continue;

            Assert.Fail("Wrong edges from Vertex 0.");
        }
    }
}