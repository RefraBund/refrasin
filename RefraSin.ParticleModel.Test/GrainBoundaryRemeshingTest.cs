using Plotly.NET;
using RefraSin.Coordinates.Absolute;
using RefraSin.Coordinates.Polar;
using RefraSin.ParticleModel.Nodes;
using RefraSin.ParticleModel.ParticleFactories;
using RefraSin.ParticleModel.Particles;
using RefraSin.ParticleModel.Remeshing;
using RefraSin.ParticleModel.System;
using RefraSin.Plotting;
using static RefraSin.Coordinates.Constants;

namespace RefraSin.ParticleModel.Test;

[TestFixture]
public class GrainBoundaryRemeshingTest
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
    [TestCase(30, 2)]
    [TestCase(2, 0)]
    public void TestNodeAddition(double initialNeck, int expectedAddedNodeCount)
    {
        var baseParticleFactory = new ShapeFunctionParticleFactoryCosOvalityCosPeaks(
            Guid.Empty,
            (0, 0),
            0,
            50,
            100,
            0.2,
            5,
            0.2
        );

        IEnumerable<IParticleNode> NodeFactory(IParticle<IParticleNode> particle) =>
            baseParticleFactory
                .GetParticle()
                .Nodes.Skip(2)
                .SkipLast(1)
                .Select(n => new ParticleNode(n, particle))
                .Concat(
                    [
                        new ParticleNode(
                            Guid.NewGuid(),
                            particle,
                            new PolarPoint(new AbsolutePoint(120, -initialNeck)),
                            NodeType.Neck
                        ),
                        new ParticleNode(
                            Guid.NewGuid(),
                            particle,
                            new PolarPoint(new AbsolutePoint(120, 0)),
                            NodeType.GrainBoundary
                        ),
                        new ParticleNode(
                            Guid.NewGuid(),
                            particle,
                            new PolarPoint(new AbsolutePoint(120, initialNeck)),
                            NodeType.Neck
                        ),
                    ]
                );

        var particle1 = new Particle<IParticleNode>(
            Guid.NewGuid(),
            new AbsolutePoint(0, 0),
            0,
            Guid.Empty,
            NodeFactory
        );
        var particle2 = new Particle<IParticleNode>(
            Guid.NewGuid(),
            new AbsolutePoint(240, 0),
            Pi,
            Guid.Empty,
            NodeFactory
        );
        var particleSystem = new ParticleSystem<IParticle<IParticleNode>, IParticleNode>(
            [particle1, particle2]
        );

        var remesher = new GrainBoundaryRemesher(additionLimit: 1.2);
        var remeshedParticleSystem = remesher.RemeshSystem(particleSystem);

        var plot = ParticlePlot.PlotParticles<IParticle<IParticleNode>, IParticleNode>(
            remeshedParticleSystem.Particles
        );
        plot.SaveHtml(Path.Combine(_tempDir, $"{nameof(TestNodeAddition)}-{initialNeck}.html"));

        Assert.That(
            remeshedParticleSystem.Particles[0].Nodes.Count,
            Is.EqualTo(particle1.Nodes.Count + expectedAddedNodeCount)
        );
        Assert.That(
            remeshedParticleSystem.Particles[1].Nodes.Count,
            Is.EqualTo(particle2.Nodes.Count + expectedAddedNodeCount)
        );
    }
}
