using System.Runtime.InteropServices.ComTypes;
using RefraSin.TEPSolver.Constraints;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.Quantities;

namespace RefraSin.TEPSolver;

public class EquationSystemBuilder
{
    private readonly List<(
        Func<SolutionState, bool> condition,
        Func<SolutionState, IQuantity> ctor
    )> _globalQuantities = new();

    private readonly List<(
        Func<SolutionState, Particle, bool> condition,
        Func<SolutionState, Particle, IQuantity> ctor
    )> _particleQuantities = new();

    private readonly List<(
        Func<SolutionState, NodeBase, bool> condition,
        Func<SolutionState, NodeBase, IQuantity> ctor
    )> _nodeQuantities = new();

    public EquationSystemBuilder AddGlobalQuantity<TQuantity>(
        Func<SolutionState, bool>? condition = null
    )
        where TQuantity : IGlobalQuantity
    {
        _globalQuantities.Add((condition ?? (_ => true), TQuantity.Create));
        return this;
    }

    public EquationSystemBuilder AddParticleQuantity<TQuantity>(
        Func<SolutionState, Particle, bool>? condition = null
    )
        where TQuantity : IParticleQuantity
    {
        _particleQuantities.Add((condition ?? ((_, _) => true), TQuantity.Create));
        return this;
    }

    public EquationSystemBuilder AddNodeQuantity<TQuantity>(
        Func<SolutionState, NodeBase, bool>? condition = null
    )
        where TQuantity : INodeQuantity
    {
        _nodeQuantities.Add((condition ?? ((_, _) => true), TQuantity.Create));
        return this;
    }

    public EquationSystemBuilder AddNodeQuantity<TQuantity, TNode>()
        where TQuantity : INodeQuantity
        where TNode : NodeBase
    {
        _nodeQuantities.Add(((_, n) => n is TNode, TQuantity.Create));
        return this;
    }

    private readonly List<(
        Func<SolutionState, bool> condition,
        Func<SolutionState, IConstraint> ctor
    )> _globalConstraints = new();

    private readonly List<(
        Func<SolutionState, Particle, bool> condition,
        Func<SolutionState, Particle, IConstraint> ctor
    )> _particleConstraints = new();

    private readonly List<(
        Func<SolutionState, NodeBase, bool> condition,
        Func<SolutionState, NodeBase, IConstraint> ctor
    )> _nodeConstraints = new();

    public EquationSystemBuilder AddGlobalConstraint<TConstraint>(
        Func<SolutionState, bool>? condition = null
    )
        where TConstraint : IGlobalConstraint
    {
        _globalConstraints.Add((condition ?? (_ => true), TConstraint.Create));
        return this;
    }

    public EquationSystemBuilder AddParticleConstraint<TConstraint>(
        Func<SolutionState, Particle, bool>? condition = null
    )
        where TConstraint : IParticleConstraint
    {
        _particleConstraints.Add((condition ?? ((_, _) => true), TConstraint.Create));
        return this;
    }

    public EquationSystemBuilder AddNodeConstraint<TConstraint>(
        Func<SolutionState, NodeBase, bool>? condition = null
    )
        where TConstraint : INodeConstraint
    {
        _nodeConstraints.Add((condition ?? ((_, _) => true), TConstraint.Create));
        return this;
    }

    public EquationSystemBuilder AddNodeConstraint<TConstraint, TNode>()
        where TConstraint : INodeConstraint
        where TNode : NodeBase
    {
        _nodeConstraints.Add(((_, n) => n is TNode, TConstraint.Create));
        return this;
    }

    public EquationSystem Build(SolutionState solutionState) =>
        new(solutionState, CreateQuantities(solutionState), CreateConstraints(solutionState));

    private IEnumerable<IQuantity> CreateQuantities(SolutionState solutionState)
    {
        foreach (var (condition, globalQuantity) in _globalQuantities)
        {
            if (condition(solutionState))
                yield return globalQuantity(solutionState);
        }

        foreach (var particle in solutionState.Particles)
        {
            foreach (var (condition, particleQuantity) in _particleQuantities)
            {
                if (condition(solutionState, particle))
                    yield return particleQuantity(solutionState, particle);
            }

            foreach (var node in particle.Nodes)
            foreach (var (condition, nodeQuantity) in _nodeQuantities)
            {
                if (condition(solutionState, node))
                    yield return nodeQuantity(solutionState, node);
            }
        }
    }

    private IEnumerable<IConstraint> CreateConstraints(SolutionState solutionState)
    {
        foreach (var (condition, globalConstraint) in _globalConstraints)
        {
            if (condition(solutionState))
                yield return globalConstraint(solutionState);
        }

        foreach (var particle in solutionState.Particles)
        {
            foreach (var (condition, particleConstraint) in _particleConstraints)
            {
                if (condition(solutionState, particle))
                    yield return particleConstraint(solutionState, particle);
            }

            foreach (var node in particle.Nodes)
            foreach (var (condition, nodeConstraint) in _nodeConstraints)
            {
                if (condition(solutionState, node))
                    yield return nodeConstraint(solutionState, node);
            }
        }
    }
}
