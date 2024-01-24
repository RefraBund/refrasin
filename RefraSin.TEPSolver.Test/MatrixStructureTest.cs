using MathNet.Numerics.LinearAlgebra;
using MoreLinq;
using RefraSin.ParticleModel.ParticleFactories;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;
using ScottPlot;

namespace RefraSin.TEPSolver.Test;

public class MatrixStructureTest
{
    private StepVectorMap _map;
    private Particle[] _particles;
    private NodeBase[] _nodes;

    [SetUp]
    public void Setup()
    {
        var fac = new ShapeFunctionParticleFactory(100, 0.1, 5, 0.1, Guid.Empty) { NodeCount = 20 };

        _particles = Enumerable.Range(0, 3).Select(_ => new Particle(fac.GetParticle(), null!)).ToArray();
        _nodes = _particles.SelectMany(p => p.Nodes).ToArray();

        _map = new StepVectorMap(new[]
            {
                new ParticleContact(_particles[0], _particles[1]), 
                new ParticleContact(_particles[0], _particles[2]),
                new ParticleContact(_particles[1], _particles[2])
            },
            _nodes);
    }

    [Test]
    public void PlotMatrixStructure()
    {
        var matrix = Matrix<double>.Build.SparseOfRows(YieldEquations());

        var plt = new Plot();
        plt.XAxes.ForEach(x => x.IsVisible = false);
        plt.YAxes.ForEach(x => x.IsVisible = false);
        plt.Margins(0, 0);

        plt.Add.Heatmap(matrix.ToArray());

        plt.SavePng(Path.GetTempFileName().Replace(".tmp", ".png"), _map.TotalUnknownsCount, _map.TotalUnknownsCount);
    }

    internal IEnumerable<Vector<double>> YieldEquations()
    {
        yield return Vector<double>.Build.SparseOfIndexed(
            _map.TotalUnknownsCount,
            _particles.SelectMany(
                p => p.Nodes.SelectMany(
                    (n, i) =>
                        new (int, double)[]
                        {
                            (_map.GetIndex(n.Id, NodeUnknown.NormalDisplacement), 1),
                            (_map.GetIndex(n.Id, NodeUnknown.FluxToUpper), 2),
                            (_map.GetIndex(p.Nodes[i - 1].Id, NodeUnknown.FluxToUpper), 2)
                        }
                )
            )
        );
        // fix root particle to origin
        var first = _particles[0];
        yield return Vector<double>.Build.SparseOfIndexed(_map.TotalUnknownsCount, new (int, double)[]
        {
            (_map.GetIndex(first.Id, ContactUnknown.RadialDisplacement), 1),
        });
        yield return Vector<double>.Build.SparseOfIndexed(_map.TotalUnknownsCount, new (int, double)[]
        {
            (_map.GetIndex(first.Id, ContactUnknown.AngleDisplacement), 1),
        });
        yield return Vector<double>.Build.SparseOfIndexed(_map.TotalUnknownsCount, new (int, double)[]
        {
            (_map.GetIndex(first.Id, ContactUnknown.RotationDisplacement), 1),
        });

        // yield particle displacement equations
        foreach (var particle in _particles.Skip(1))
        {
            yield return Vector<double>.Build.SparseOfIndexed(_map.TotalUnknownsCount, new (int, double)[]
            {
                (_map.GetIndex(particle.Id, ContactUnknown.RadialDisplacement), 1),
            });
            yield return Vector<double>.Build.SparseOfIndexed(_map.TotalUnknownsCount, new (int, double)[]
            {
                (_map.GetIndex(particle.Id, ContactUnknown.AngleDisplacement), 1),
            });
            yield return Vector<double>.Build.SparseOfIndexed(_map.TotalUnknownsCount, new (int, double)[]
            {
                (_map.GetIndex(particle.Id, ContactUnknown.RotationDisplacement), 1),
            });
        }

        foreach (var particleSpec in _particles)
        foreach (var (i, node) in particleSpec.Nodes.Index())
        {
            yield return Vector<double>.Build.SparseOfIndexed(_map.TotalUnknownsCount, new (int, double)[]
            {
                (_map.GetIndex(GlobalUnknown.Lambda1), 1),
                (_map.GetIndex(node.Id, NodeUnknown.Lambda2), 1),
            });
            yield return Vector<double>.Build.SparseOfIndexed(_map.TotalUnknownsCount, new (int, double)[]
            {
                (_map.GetIndex(node.Id, NodeUnknown.FluxToUpper), 1),
                (_map.GetIndex(GlobalUnknown.Lambda1), 1),
                (_map.GetIndex(node.Id, NodeUnknown.Lambda2), 1),
                (_map.GetIndex(particleSpec.Nodes[i + 1].Id, NodeUnknown.Lambda2), 1),
            });
            yield return Vector<double>.Build.SparseOfIndexed(_map.TotalUnknownsCount, new (int, double)[]
            {
                (_map.GetIndex(node.Id, NodeUnknown.NormalDisplacement), 1),
                (_map.GetIndex(node.Id, NodeUnknown.FluxToUpper), 1),
                (_map.GetIndex(particleSpec.Nodes[i - 1].Id, NodeUnknown.FluxToUpper), 1),
            });
        }
    }
}