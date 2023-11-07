using Microsoft.Extensions.Logging;
using RefraSin.Coordinates.Absolute;
using RefraSin.MaterialData;
using RefraSin.ParticleModel;
using RefraSin.ParticleModel.ParticleSpecFactories;
using RefraSin.ProcessModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.Test;

[TestFixture]
public class StepVectorMapTest
{
    private IParticleSpec _particleSpec1;
    private IParticleSpec _particleSpec2;
    private StepVectorMap _map;

    [SetUp]
    public void Setup()
    {
        _particleSpec1 = new ShapeFunctionParticleSpecFactory(100, 0.1, 5, 0.1, Guid.Empty).GetParticleSpec();
        _particleSpec2 = new ShapeFunctionParticleSpecFactory(100, 0.1, 5, 0.1, Guid.Empty)
                { CenterCoordinates = new AbsolutePoint(200, 0) }
            .GetParticleSpec();
        var particles = new[] { _particleSpec1, _particleSpec2 };

        _map = new StepVectorMap(particles, particles.SelectMany(p => p.NodeSpecs));
    }

    [Test]
    public void TestGlobalIndices()
    {
        Assert.That(_map.GetIndex(GlobalUnknown.Lambda1), Is.EqualTo(0));
    }

    [Test]
    public void TestNodeIndices()
    {
        // first particle

        // first node
        Assert.That(_map.GetIndex(_particleSpec1.NodeSpecs[0].Id, NodeUnknown.NormalDisplacement), Is.EqualTo(1));
        Assert.That(_map.GetIndex(_particleSpec1.NodeSpecs[0].Id, NodeUnknown.FluxToUpper), Is.EqualTo(2));
        Assert.That(_map.GetIndex(_particleSpec1.NodeSpecs[0].Id, NodeUnknown.Lambda2), Is.EqualTo(3));

        // second node
        Assert.That(_map.GetIndex(_particleSpec1.NodeSpecs[1].Id, NodeUnknown.NormalDisplacement), Is.EqualTo(4));
        Assert.That(_map.GetIndex(_particleSpec1.NodeSpecs[1].Id, NodeUnknown.FluxToUpper), Is.EqualTo(5));
        Assert.That(_map.GetIndex(_particleSpec1.NodeSpecs[1].Id, NodeUnknown.Lambda2), Is.EqualTo(6));

        // last node
        Assert.That(_map.GetIndex(_particleSpec1.NodeSpecs[99].Id, NodeUnknown.NormalDisplacement), Is.EqualTo(298));
        Assert.That(_map.GetIndex(_particleSpec1.NodeSpecs[99].Id, NodeUnknown.FluxToUpper), Is.EqualTo(299));
        Assert.That(_map.GetIndex(_particleSpec1.NodeSpecs[99].Id, NodeUnknown.Lambda2), Is.EqualTo(300));

        // second particle

        // first node
        Assert.That(_map.GetIndex(_particleSpec2.NodeSpecs[0].Id, NodeUnknown.NormalDisplacement), Is.EqualTo(301));
        Assert.That(_map.GetIndex(_particleSpec2.NodeSpecs[0].Id, NodeUnknown.FluxToUpper), Is.EqualTo(302));
        Assert.That(_map.GetIndex(_particleSpec2.NodeSpecs[0].Id, NodeUnknown.Lambda2), Is.EqualTo(303));

        // second node
        Assert.That(_map.GetIndex(_particleSpec2.NodeSpecs[1].Id, NodeUnknown.NormalDisplacement), Is.EqualTo(304));
        Assert.That(_map.GetIndex(_particleSpec2.NodeSpecs[1].Id, NodeUnknown.FluxToUpper), Is.EqualTo(305));
        Assert.That(_map.GetIndex(_particleSpec2.NodeSpecs[1].Id, NodeUnknown.Lambda2), Is.EqualTo(306));

        // last node
        Assert.That(_map.GetIndex(_particleSpec2.NodeSpecs[99].Id, NodeUnknown.NormalDisplacement), Is.EqualTo(598));
        Assert.That(_map.GetIndex(_particleSpec2.NodeSpecs[99].Id, NodeUnknown.FluxToUpper), Is.EqualTo(599));
        Assert.That(_map.GetIndex(_particleSpec2.NodeSpecs[99].Id, NodeUnknown.Lambda2), Is.EqualTo(600));
    }
}