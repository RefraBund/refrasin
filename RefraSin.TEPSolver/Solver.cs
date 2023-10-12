using MathNet.Numerics;
using MathNet.Numerics.RootFinding;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Logging.Abstractions;
using RefraSin.Iteration;
using RefraSin.ParticleModel;
using RefraSin.ProcessModel;
using RefraSin.Storage;
using RefraSin.TEPSolver.Step;
using static System.Math;

namespace RefraSin.TEPSolver;

/// <summary>
/// Solver for performing time integration of sintering processes based on the thermodynamic extremal principle (TEP).
/// </summary>
public class Solver
{
    private ISolverSession? _session;

    /// <summary>
    /// Numeric options to control solver behavior.
    /// </summary>
    public ISolverOptions Options { get; set; } = new SolverOptions();

    /// <summary>
    /// Storage for solution data.
    /// </summary>
    public ISolutionStorage SolutionStorage { get; set; } = new InMemorySolutionStorage();

    /// <summary>
    /// Factory for loggers used in the Session.
    /// </summary>
    public ILoggerFactory LoggerFactory { get; set; } = new NullLoggerFactory();

    internal ISolverSession Session
    {
        get => _session ?? throw new InvalidOperationException("Solution procedure is not initialized.");
        private set => _session = value;
    }

    /// <summary>
    /// Creates a new solver Session for the given process.
    /// </summary>
    internal SolverSession CreateSession(ISinteringProcess process)
    {
        var session = new SolverSession(this, process);
        Session = session;
        return session;
    }

    /// <summary>
    /// Run the solution procedure starting with the given state till the specified time.
    /// </summary>
    public void Solve(ISinteringProcess process)
    {
        CreateSession(process);
        DoTimeIntegration();
    }

    private void DoTimeIntegration()
    {
        Session.StoreCurrentState();

        while (Session.CurrentTime < Session.EndTime)
        {
            var stepVector = TrySolveStepUntilValid();
            var particleTimeSteps = GenerateTimeStepsFromGradientSolution(stepVector).ToArray();
            Session.StoreStep(particleTimeSteps);

            Session.IncreaseCurrentTime();

            foreach (var timeStep in particleTimeSteps)
                Session.Particles[timeStep.ParticleId].ApplyTimeStep(timeStep);

            Session.StoreCurrentState();
            Session.MayIncreaseTimeStepWidth();
        }

        Session.Logger.LogInformation("End time successfully reached after {StepCount} steps.", Session.TimeStepIndex + 1);
    }

    private StepVector TrySolveStepUntilValid()
    {
        int i;

        for (i = 0; i < Session.Options.MaxIterationCount; i++)
        {
            try
            {
                var particleTimeSteps = TrySolveStepWithLastStepOrGuess();

                return particleTimeSteps;
            }
            catch (Exception e)
            {
                Session.Logger.LogError(e, "Exception occured during time step solution. Lowering time step width and try again.");
                Session.DecreaseTimeStepWidth();
            }
        }

        throw new CriticalIterationInterceptedException(nameof(TrySolveStepUntilValid), InterceptReason.MaxIterationCountExceeded, i);
    }

    private StepVector TrySolveStepWithLastStepOrGuess()
    {
        StepVector solution = Session.LastStep ?? GuessSolution();

        try
        {
            solution = new StepVector(Broyden.FindRoot(
                EvaluateLagrangianGradientAt,
                initialGuess: solution.AsArray(),
                maxIterations: Session.Options.RootFindingMaxIterationCount,
                accuracy: Session.Options.RootFindingAccuracy
            ), Session.StepVectorMap);
        }
        catch (NonConvergenceException e)
        {
            solution = new StepVector(Broyden.FindRoot(
                EvaluateLagrangianGradientAt,
                initialGuess: GuessSolution().AsArray(),
                maxIterations: Session.Options.RootFindingMaxIterationCount,
                accuracy: Session.Options.RootFindingAccuracy
            ), Session.StepVectorMap);
        }

        return solution;
    }

    private IEnumerable<IParticleTimeStep> GenerateTimeStepsFromGradientSolution(StepVector stepVector)
    {
        foreach (var p in Session.Particles.Values)
            yield return new ParticleTimeStep(
                p.Id,
                0,
                0,
                0,
                p.Surface.Select(n =>
                    new NodeTimeStep(n.Id,
                        stepVector[n].NormalDisplacement,
                        0,
                        new ToUpperToLower(
                            stepVector[n].FluxToUpper,
                            -stepVector[n].FluxToUpper
                        ),
                        0
                    )));
    }

    private StepVector EvaluateLagrangianGradientAt(StepVector stepVector)
    {
        var evaluation = YieldEquations(stepVector).ToArray();

        if (evaluation.Any(x => !double.IsFinite(x)))
        {
            throw new InvalidOperationException("One ore more components of the gradient evaluated to an infinite value.");
        }

        return new StepVector(evaluation, Session.StepVectorMap);
    }

    private double[] EvaluateLagrangianGradientAt(double[] vector) =>
        EvaluateLagrangianGradientAt(new StepVector(vector, Session.StepVectorMap)).AsArray();

    private IEnumerable<double> YieldEquations(StepVector stepVector) =>
        YieldStateVelocityDerivatives(stepVector)
            .Concat(
                YieldFluxDerivatives(stepVector)
            )
            .Concat(
                YieldDissipationEquality(stepVector)
            )
            .Concat(
                YieldRequiredConstraints(stepVector)
            );

    private IEnumerable<double> YieldStateVelocityDerivatives(StepVector stepVector)
    {
        foreach (var node in Session.Nodes.Values)
        {
            // Normal Displacement
            var gibbsTerm = -node.GibbsEnergyGradient.Normal * (1 + stepVector.Lambda1);
            var requiredConstraintsTerm = node.VolumeGradient.Normal * stepVector[node].Lambda2;

            yield return gibbsTerm + requiredConstraintsTerm;
        }
    }

    private IEnumerable<double> YieldFluxDerivatives(StepVector stepVector)
    {
        foreach (var node in Session.Nodes.Values) // for each flux
        {
            // Flux To Upper
            var dissipationTerm =
                2 * Session.GasConstant * Session.Temperature * Session.TimeStepWidth
              / (node.Particle.Material.MolarVolume * node.Particle.Material.EquilibriumVacancyConcentration)
              * node.SurfaceDistance.ToUpper * stepVector[node].FluxToUpper / node.SurfaceDiffusionCoefficient.ToUpper
              * stepVector.Lambda1;
            var thisRequiredConstraintsTerm = Session.TimeStepWidth * stepVector[node].Lambda2;
            var upperRequiredConstraintsTerm = Session.TimeStepWidth * stepVector[node.Upper].Lambda2;

            yield return -dissipationTerm - thisRequiredConstraintsTerm + upperRequiredConstraintsTerm;
        }
    }

    private IEnumerable<double> YieldDissipationEquality(StepVector stepVector)
    {
        var dissipation = Session.Nodes.Values.Select(n =>
            -n.GibbsEnergyGradient.Normal * stepVector[n].NormalDisplacement
        ).Sum();

        var dissipationFunction =
            Session.GasConstant * Session.Temperature * Session.TimeStepWidth / 2
          * Session.Nodes.Values.Select(n =>
                (
                    n.SurfaceDistance.ToUpper * Pow(stepVector[n].FluxToUpper, 2) / n.SurfaceDiffusionCoefficient.ToUpper
                  + n.SurfaceDistance.ToLower * Pow(stepVector[n.Lower].FluxToUpper, 2) / n.SurfaceDiffusionCoefficient.ToLower
                ) / (n.Particle.Material.MolarVolume * n.Particle.Material.EquilibriumVacancyConcentration)
            ).Sum();

        yield return dissipation - dissipationFunction;
    }

    private IEnumerable<double> YieldRequiredConstraints(StepVector stepVector)
    {
        foreach (var node in Session.Nodes.Values)
        {
            var volumeTerm = node.VolumeGradient.Normal * stepVector[node].NormalDisplacement;
            var fluxTerm =
                Session.TimeStepWidth *
                (
                    stepVector[node].FluxToUpper
                  - stepVector[node.Lower].FluxToUpper
                );

            yield return volumeTerm - fluxTerm;
        }
    }

    private StepVector GuessSolution() => new(YieldInitialGuess().ToArray(), Session.StepVectorMap);

    private IEnumerable<double> YieldInitialGuess() =>
        YieldGlobalUnknownsInitialGuess()
            .Concat(
                YieldParticleUnknownsInitialGuess()
            )
            .Concat(
                YieldNodeUnknownsInitialGuess()
            );

    private IEnumerable<double> YieldGlobalUnknownsInitialGuess()
    {
        yield return 1;
    }

    private IEnumerable<double> YieldParticleUnknownsInitialGuess()
    {
        yield break;

        foreach (var particle in Session.Particles.Values) { }
    }

    private IEnumerable<double> YieldNodeUnknownsInitialGuess()
    {
        foreach (var node in Session.Nodes.Values)
        {
            yield return node.GuessNormalDisplacement();
            yield return node.GuessFluxToUpper();
            ;
            yield return 1;
        }
    }
}