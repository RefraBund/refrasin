using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.EquationSystem;

public class TangentialStressConstraintSurface : NodeEquationBase<SurfaceNode>
{
    /// <inheritdoc />
    public TangentialStressConstraintSurface(SurfaceNode node, StepVector step)
        : base(node, step) { }

    /// <inheritdoc />
    public override double Value() => Step.TangentialStress(Node);

    /// <inheritdoc />
    public override IEnumerable<(int, double)> Derivative()
    {
        yield return (Map.TangentialStress(Node), 1);
    }
}
