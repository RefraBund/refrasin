using MathNet.Numerics.LinearAlgebra;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class EquationSystem
{
    private readonly SolutionState _solutionState;
    private readonly StepVector _stepVector;

    private readonly IReadOnlyDictionary<Guid, IEquation[]> _particleBlocks;
    private readonly IReadOnlyDictionary<(Guid, Guid), IEquation[]> _contactBlocks;
    private readonly IReadOnlyList<IEquation> _globalBlock;

    public EquationSystem(SolutionState solutionState, StepVector stepVector)
    {
        _solutionState = solutionState;
        _stepVector = stepVector;

        _particleBlocks = solutionState.Particles.ToDictionary(
            p => p.Id,
            p => YieldParticleEquations(p).ToArray()
        );
        _contactBlocks = solutionState.ParticleContacts.ToDictionary(
            c => (c.From.Id, c.To.Id),
            c => YieldContactEquations(c).ToArray()
        );
        _globalBlock = YieldGlobalEquations().ToArray();
    }

    private IEnumerable<IEquation> YieldParticleEquations(Particle p)
    {
        foreach (var n in p.Nodes)
        {
            yield return new NormalDisplacementDerivative(n, _stepVector);
            if (n is ContactNodeBase cn)
                yield return new TangentialDisplacementDerivative(cn, _stepVector);
            yield return new FluxDerivative(n, _stepVector);
            yield return new VolumeBalanceConstraint(n, _stepVector);
            yield return new NormalStressDerivative(n, _stepVector);
            yield return new TangentialStressDerivative(n, _stepVector);

            if (n is SurfaceNode sn)
            {
                yield return new NormalStressConstraintSurface(sn, _stepVector);
                yield return new TangentialStressConstraintSurface(sn, _stepVector);
            }
        }

        yield return new ParticleHorizontalForceBalanceConstraint(p, _stepVector);
        yield return new ParticleVerticalForceBalanceConstraint(p, _stepVector);
        yield return new ParticleTorqueBalanceConstraint(p, _stepVector);
    }

    private IEnumerable<IEquation> YieldContactEquations(ParticleContact contact)
    {
        foreach (var n in contact.FromNodes)
        {
            yield return new ContactDistanceConstraint(n, _stepVector);
            yield return new ContactDirectionConstraint(n, _stepVector);

            yield return new NormalStressConstraintContact(n, _stepVector);
            yield return new TangentialStressConstraintContact(n, _stepVector);
        }

        yield return new ContactDistanceDerivative(contact, _stepVector);
        yield return new ContactDirectionDerivative(contact, _stepVector);
    }

    private IEnumerable<IEquation> YieldGlobalEquations()
    {
        yield return new DissipationEqualityConstraint(_solutionState, _stepVector);
    }

    public IReadOnlyList<IEquation> Equations =>
        _particleBlocks
            .SelectMany(kvp => kvp.Value)
            .Concat(_contactBlocks.SelectMany(kvp => kvp.Value))
            .Concat(_globalBlock)
            .ToArray();

    public IReadOnlyList<IEquation> GlobalBlockEquations => _globalBlock;

    public IReadOnlyList<IEquation> ParticleBlockEquations(IParticle particle) =>
        _particleBlocks[particle.Id];

    public IReadOnlyList<IEquation> ContactBlockEquations(IParticleContactEdge contact) =>
        _contactBlocks[(contact.From, contact.To)];

    public Vector<double> Lagrangian() =>
        Vector<double>.Build.DenseOfEnumerable(Equations.Select(e => e.Value()));

    public Matrix<double> Jacobian() =>
        Matrix<double>.Build.SparseOfIndexed(
            _stepVector.Count,
            _stepVector.Count,
            Equations.SelectMany((e, i) => e.Derivative().Select(c => (i, c.Item1, c.Item2)))
        );

    public Vector<double> ParticleBlockLagrangian(IParticle particle) =>
        Vector<double>.Build.DenseOfEnumerable(
            ParticleBlockEquations(particle).Select(e => e.Value())
        );

    public Matrix<double> ParticleBlockJacobian(IParticle particle)
    {
        var equations = ParticleBlockEquations(particle);
        return Matrix<double>.Build.SparseOfIndexed(
            equations.Count,
            equations.Count,
            equations.SelectMany((e, i) => e.Derivative().Select(c => (i, c.Item1, c.Item2)))
        );
    }

    public Vector<double> ContactBlockLagrangian(IParticleContactEdge contact) =>
        Vector<double>.Build.DenseOfEnumerable(
            ContactBlockEquations(contact).Select(e => e.Value())
        );

    public Matrix<double> ContactBlockJacobian(IParticleContactEdge contact)
    {
        var equations = ContactBlockEquations(contact);
        return Matrix<double>.Build.SparseOfIndexed(
            equations.Count,
            equations.Count,
            equations.SelectMany((e, i) => e.Derivative().Select(c => (i, c.Item1, c.Item2)))
        );
    }

    public Vector<double> GlobalBlockLagrangian() =>
        Vector<double>.Build.DenseOfEnumerable(GlobalBlockEquations.Select(e => e.Value()));

    public Matrix<double> GlobalBlockJacobian() =>
        Matrix<double>.Build.SparseOfIndexed(
            GlobalBlockEquations.Count,
            GlobalBlockEquations.Count,
            GlobalBlockEquations.SelectMany(
                (e, i) => e.Derivative().Select(c => (i, c.Item1, c.Item2))
            )
        );
}
