using MathNet.Numerics;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Logging.Abstractions;
using RefraSin.Coordinates.Polar;
using RefraSin.Graphs;
using RefraSin.ParticleModel;
using RefraSin.ProcessModel;
using RefraSin.Storage;
using RefraSin.TEPSolver.Exceptions;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.RootFinding;
using RefraSin.TEPSolver.StepValidators;
using RefraSin.TEPSolver.StepVectors;
using RefraSin.TEPSolver.TimeSteppers;
using NeckNode = RefraSin.TEPSolver.ParticleModel.NeckNode;
using Particle = RefraSin.TEPSolver.ParticleModel.Particle;
using ParticleContact = RefraSin.TEPSolver.ParticleModel.ParticleContact;

namespace RefraSin.TEPSolver;

/// <summary>
/// Solver for performing time integration of sintering processes based on the thermodynamic extremal principle (TEP).
/// </summary>
public class Solver
{
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

    public ITimeStepper TimeStepper { get; set; } = new AdamsMoultonTimeStepper();

    public IEnumerable<IStepValidator> StepValidators { get; set; } = new[] { new InstabilityDetector() };

    public IRootFinder RootFinder { get; } = new BroydenRootFinder();

    /// <summary>
    /// Run the solution procedure starting with the given state till the specified time.
    /// </summary>
    public void Solve(ISinteringProcess process)
    {
        var session = new SolverSession(this, process);

        InitCurrentState(process, session);
        DoTimeIntegration(session);
    }

    private static void InitCurrentState(ISinteringProcess process, SolverSession session)
    {
        var particles = process.Particles.Select(ps => new Particle(ps, session)).ToArray();
        session.CurrentState = new SolutionState(
            session.StartTime,
            particles,
            Array.Empty<(Guid, Guid)>()
        );
        session.CurrentState = new SolutionState(
            session.StartTime,
            particles,
            GetParticleContacts(particles)
        );
        session.StoreCurrentState();
    }

    private static (Guid from, Guid to)[] GetParticleContacts(Particle[] particles)
    {
        var edges = particles.SelectMany(p => p.Nodes.OfType<NeckNode>())
            .Select(n => new UndirectedEdge<Particle>(n.Particle, n.ContactedNode.Particle));
        var graph = new UndirectedGraph<Particle>(particles, edges);
        var explorer = BreadthFirstExplorer<Particle>.Explore(graph, particles[0]);

        return explorer.TraversedEdges.Select(e => (e.From.Id, e.To.Id)).ToArray();
    }

    private static void DoTimeIntegration(SolverSession session)
    {
        while (session.CurrentState.Time < session.EndTime)
        {
            var stepVector = TrySolveStepUntilValid(session);
            session.LastStep = stepVector;

            CreateNewState(session, stepVector);
            session.TimeStepIndex += 1;
            session.MayIncreaseTimeStepWidth();
        }

        session.Logger.LogInformation("End time successfully reached after {StepCount} steps.", session.TimeStepIndex + 1);
    }

    private static void CreateNewState(SolverSession session, StepVector stepVector)
    {
        var newParticles = new Dictionary<Guid, Particle>()
        {
            [session.CurrentState.Particles.Root.Id] = session.CurrentState.Particles.Root.ApplyTimeStep(null, stepVector, session.TimeStepWidth)
        };

        foreach (var contact in session.CurrentState.Contacts.Values)
        {
            newParticles[contact.To.Id] = contact.To.ApplyTimeStep(newParticles[contact.From.Id], stepVector, session.TimeStepWidth);
        }

        var newState = new SolutionState(session.CurrentState.Time + session.TimeStepWidth,
            newParticles.Values,
            session.CurrentState.Contacts.Keys
        );

        StoreSolutionStep(session, stepVector, newState);

        session.CurrentState = newState;
        session.StoreCurrentState();
    }

    private static void StoreSolutionStep(SolverSession session, StepVector stepVector, SolutionState newState)
    {
        var solutionStep = new SolutionStep(
            session.CurrentState.Time,
            newState.Time,
            session.CurrentState.Particles.Zip(newState.Particles).Select(t =>
            {
                var (current, next) = t;

                var centerShift = next.CenterCoordinates - current.CenterCoordinates;

                return new ParticleTimeStep(
                    current.Id,
                    centerShift.X,
                    centerShift.Y,
                    next.RotationAngle - current.RotationAngle,
                    current.Nodes.Select(n =>
                    {
                        if (n is ContactNodeBase contactNode)
                        {
                            return new NodeTimeStep(
                                n.Id,
                                stepVector[n].NormalDisplacement,
                                stepVector[contactNode].TangentialDisplacement,
                                new ToUpperToLower<double>(stepVector[n].FluxToUpper, stepVector[n.Lower].FluxToUpper),
                                0
                            );
                        }

                        return new NodeTimeStep(
                            n.Id,
                            stepVector[n].NormalDisplacement,
                            0,
                            new ToUpperToLower<double>(stepVector[n].FluxToUpper, stepVector[n.Lower].FluxToUpper),
                            0
                        );
                    })
                );
            })
        );

        session.StoreStep(solutionStep);
    }

    internal static StepVector TrySolveStepUntilValid(SolverSession session)
    {
        int i;

        for (i = 0; i < session.Options.MaxIterationCount; i++)
        {
            try
            {
                var step = TrySolveStepWithLastStepOrGuess(session);

                foreach (var validator in session.StepValidators)
                {
                    validator.Validate(session.CurrentState, step, session.Options);
                }

                return step;
            }
            catch (Exception e)
            {
                session.Logger.LogError(e, "Exception occured during time step solution. Lowering time step width and try again.");
                session.DecreaseTimeStepWidth();

                if (e is InstabilityException)
                {
                    session.CurrentState = session.StateMemory.Pop();
                }
            }
        }

        throw new CriticalIterationInterceptedException(nameof(TrySolveStepUntilValid), InterceptReason.MaxIterationCountExceeded, i);
    }

    private static StepVector TrySolveStepWithLastStepOrGuess(SolverSession session)
    {
        try
        {
            return session.TimeStepper.Step(session, session.LastStep ?? LagrangianGradient.GuessSolution(session));
        }
        catch (NonConvergenceException)
        {
            return session.TimeStepper.Step(session, LagrangianGradient.GuessSolution(session));
        }
    }
}