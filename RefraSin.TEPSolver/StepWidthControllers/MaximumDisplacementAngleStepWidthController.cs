using Microsoft.Extensions.Logging;
using RefraSin.Coordinates.Helpers;
using RefraSin.TEPSolver.StepVectors;

namespace RefraSin.TEPSolver.StepWidthControllers;

public class MaximumDisplacementAngleStepWidthController(
    double initialTimeStepWidth = 1,
    double maximumDisplacementAngle = 0.05,
    double increaseFactor = 2,
    double decreaseFactor = 0.8,
    double minimalTimeStepWidth = double.NegativeInfinity,
    double maximalTimeStepWidth = double.PositiveInfinity
) :
    IStepWidthController
{
    /// <inheritdoc />
    public void RegisterWithSolver(SinteringSolver solver)
    {
        solver.SessionInitialized += SolverOnSessionInitialized;
    }

    private void SolverOnSessionInitialized(object? sender, SinteringSolver.SessionInitializedEventArgs e)
    {
        _stepWidths[e.SolverSession.Id] = InitialTimeStepWidth;
    }

    /// <inheritdoc />
    public double GetStepWidth(ISolverSession solverSession, SolutionState currentState, StepVector stepVector)
    {
        var stepWidth = _stepWidths[solverSession.Id] * IncreaseFactor;

        while (true)
        {
            var stepWidth1 = stepWidth;
            var displacementAngles = currentState.Nodes.Select(n =>
            {
                var displacement = stepVector.NormalDisplacement(n) * stepWidth1;
                var upperAngle = SinLaw.Alpha(displacement, CosLaw.C(displacement, n.SurfaceDistance.ToUpper, n.SurfaceNormalAngle.ToUpper),
                    n.SurfaceNormalAngle.ToUpper);
                var lowerAngle = SinLaw.Alpha(displacement, CosLaw.C(displacement, n.SurfaceDistance.ToLower, n.SurfaceNormalAngle.ToLower),
                    n.SurfaceNormalAngle.ToLower);
                return double.Max(upperAngle, lowerAngle);
            });

            var maxAngle = displacementAngles.Max();

            if (maxAngle < MaximumDisplacementAngle)
            {
                _stepWidths[solverSession.Id] = stepWidth;

                if (stepWidth < MinimalTimeStepWidth)
                    return MinimalTimeStepWidth;
                if (stepWidth > MaximalTimeStepWidth)
                    return MaximalTimeStepWidth;

                return stepWidth;
            }

            stepWidth *= DecreaseFactor;
        }
    }

    private readonly Dictionary<Guid, double> _stepWidths = new();

    public double InitialTimeStepWidth { get; } = initialTimeStepWidth;
    public double MaximumDisplacementAngle { get; } = maximumDisplacementAngle;
    public double IncreaseFactor { get; } = increaseFactor;
    public double DecreaseFactor { get; } = decreaseFactor;
    public double MinimalTimeStepWidth { get; } = minimalTimeStepWidth;
    public double MaximalTimeStepWidth { get; } = maximalTimeStepWidth;
}