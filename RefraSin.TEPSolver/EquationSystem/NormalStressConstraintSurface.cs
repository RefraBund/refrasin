using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class NormalStressConstraintSurface : NodeEquationBase<SurfaceNode>
{
    /// <inheritdoc />
    public NormalStressConstraintSurface(SurfaceNode node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value() => Step.NormalStress(Node);

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative() => throw new NotImplementedException();
}
