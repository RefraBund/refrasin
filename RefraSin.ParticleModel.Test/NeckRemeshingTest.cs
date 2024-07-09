using System.Globalization;
using RefraSin.Coordinates.Absolute;
using RefraSin.Coordinates.Polar;
using RefraSin.ParticleModel.ParticleFactories;
using RefraSin.ParticleModel.Remeshing;
using ScottPlot;
using static RefraSin.Coordinates.Constants;

namespace RefraSin.ParticleModel.Test;

[TestFixture]
public class NeckRemeshingTest
{
    private string _tempDir;

    [SetUp]
    public void Setup()
    {
        _tempDir = Path.GetTempFileName().Replace(".tmp", "");
        Directory.CreateDirectory(_tempDir);
        TestContext.WriteLine(_tempDir);
    }

    [Test]
    [TestCase(13e-6, 2)]
    [TestCase(2e-6, 0)]
    public void TestNodeDeletion(double initialNeck, int expectedRemovedNodeCount)
    {
        var baseParticle = new ShapeFunctionParticleFactory(100e-6, 0.1, 5, 0.1, Guid.NewGuid())
        {
            NodeCount = 50
        }.GetParticle();
        var nodes = baseParticle
            .Nodes.Skip(1)
            .Concat(
                new INode[]
                {
                    new Node(
                        Guid.NewGuid(),
                        baseParticle.Id,
                        new PolarPoint(new AbsolutePoint(120e-6, -initialNeck)),
                        NodeType.Neck
                    ),
                    new Node(
                        Guid.NewGuid(),
                        baseParticle.Id,
                        new PolarPoint(new AbsolutePoint(120e-6, 0)),
                        NodeType.GrainBoundary
                    ),
                    new Node(
                        Guid.NewGuid(),
                        baseParticle.Id,
                        new PolarPoint(new AbsolutePoint(120e-6, initialNeck)),
                        NodeType.Neck
                    ),
                }
            )
            .ToArray();
        var particle = new Particle(baseParticle.Id, new(0, 0), 0, baseParticle.MaterialId, nodes);

        var remesher = new NeckNeighborhoodRemesher(deletionLimit: 0.4, additionLimit: double.PositiveInfinity);
        var remeshedParticle = remesher.Remesh(particle);

        var plt = new Plot();
        plt.Axes.SquareUnits();

        PlotParticle(plt, particle);
        PlotParticle(plt, remeshedParticle);

        plt.SavePng(Path.Combine(_tempDir, $"{nameof(TestNodeDeletion)}.png"), 1600, 900);

        Assert.That(remeshedParticle.Nodes.Count, Is.EqualTo(particle.Nodes.Count - expectedRemovedNodeCount));
    }

    [Test]
    [TestCase(13e-6, 2)]
    [TestCase(2e-6, 0)]
    public void TestNodeAddition(double initialNeck, int expectedAddedNodeCount)
    {
        var baseParticle = new ShapeFunctionParticleFactory(100e-6, 0.1, 5, 0.1, Guid.NewGuid())
        {
            NodeCount = 50
        }.GetParticle();
        var nodes = baseParticle
            .Nodes.Skip(1)
            .Concat(
                new INode[]
                {
                    new Node(
                        Guid.NewGuid(),
                        baseParticle.Id,
                        new PolarPoint(new AbsolutePoint(120e-6, -initialNeck)),
                        NodeType.Neck
                    ),
                    new Node(
                        Guid.NewGuid(),
                        baseParticle.Id,
                        new PolarPoint(new AbsolutePoint(120e-6, 0)),
                        NodeType.GrainBoundary
                    ),
                    new Node(
                        Guid.NewGuid(),
                        baseParticle.Id,
                        new PolarPoint(new AbsolutePoint(120e-6, initialNeck)),
                        NodeType.Neck
                    ),
                }
            )
            .ToArray();
        var particle = new Particle(baseParticle.Id, new(0, 0), 0, baseParticle.MaterialId, nodes);

        var remesher = new NeckNeighborhoodRemesher(deletionLimit: 0, additionLimit: 0.8);
        var remeshedParticle = remesher.Remesh(particle);

        var plt = new Plot();
        plt.Axes.SquareUnits();

        PlotParticle(plt, particle);
        PlotParticle(plt, remeshedParticle);

        plt.SavePng(Path.Combine(_tempDir, $"{nameof(TestNodeDeletion)}.png"), 1600, 900);

        Assert.That(remeshedParticle.Nodes.Count, Is.EqualTo(particle.Nodes.Count + expectedAddedNodeCount));
    }

    void PlotParticle(Plot plot, IParticle particle)
    {
        plot.Add.Scatter(particle
            .Nodes.Append(particle.Nodes[0])
            .Select(n => new ScottPlot.Coordinates(
                n.Coordinates.Absolute.X,
                n.Coordinates.Absolute.Y
            ))
            .ToArray());
    }
}