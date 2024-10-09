using MathNet.Numerics.LinearAlgebra;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;
using static RefraSin.TEPSolver.EquationSystem.Helper;

namespace RefraSin.TEPSolver.EquationSystem;

public class EquationSystem
{
    private readonly SolutionState _solutionState;
    private readonly StepVector _stepVector;

    private readonly IReadOnlyDictionary<Guid, IReadOnlyList<IEquation>> _particleBlocks;
    private readonly IReadOnlyDictionary<(Guid, Guid), IReadOnlyList<IEquation>> _contactBlocks;
    private readonly IReadOnlyList<IEquation> _globalBlock;

    public EquationSystem(SolutionState solutionState, StepVector stepVector)
    {
        _solutionState = solutionState;
        _stepVector = stepVector;
    }

    public IReadOnlyList<IEquation> Equations =>
        Join(
                _particleBlocks.SelectMany(kvp => kvp.Value),
                _contactBlocks.SelectMany(kvp => kvp.Value),
                _globalBlock
            )
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

    public Matrix<double> ParticleBlockJacobian(IParticle particle) =>
        Matrix<double>.Build.SparseOfIndexed(
            _stepVector.Count,
            _stepVector.Count,
            ParticleBlockEquations(particle)
                .SelectMany((e, i) => e.Derivative().Select(c => (i, c.Item1, c.Item2)))
        );

    public Vector<double> ContactBlockLagrangian(IParticleContactEdge contact) =>
        Vector<double>.Build.DenseOfEnumerable(
            ContactBlockEquations(contact).Select(e => e.Value())
        );

    public Matrix<double> ContactBlockJacobian(IParticleContactEdge contact) =>
        Matrix<double>.Build.SparseOfIndexed(
            _stepVector.Count,
            _stepVector.Count,
            ContactBlockEquations(contact)
                .SelectMany((e, i) => e.Derivative().Select(c => (i, c.Item1, c.Item2)))
        );

    public Vector<double> GlobalBlockLagrangian() =>
        Vector<double>.Build.DenseOfEnumerable(GlobalBlockEquations.Select(e => e.Value()));

    public Matrix<double> GlobalBlockJacobian() =>
        Matrix<double>.Build.SparseOfIndexed(
            _stepVector.Count,
            _stepVector.Count,
            GlobalBlockEquations.SelectMany(
                (e, i) => e.Derivative().Select(c => (i, c.Item1, c.Item2))
            )
        );
}
