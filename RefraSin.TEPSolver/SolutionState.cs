using RefraSin.Graphs;
using RefraSin.MaterialData;
using RefraSin.ParticleModel;
using RefraSin.ProcessModel;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;
using NeckNode = RefraSin.TEPSolver.ParticleModel.NeckNode;
using Particle = RefraSin.TEPSolver.ParticleModel.Particle;
using ParticleContact = RefraSin.TEPSolver.ParticleModel.ParticleContact;

namespace RefraSin.TEPSolver;

public class SolutionState : ISystemState
{
    public SolutionState(Guid id, double time, IEnumerable<Particle> particles, IEnumerable<(Guid id, Guid from, Guid to)>? contacts = null)
    {
        Time = time;
        Id = id;
        Particles = particles as IReadOnlyParticleCollection<Particle> ?? new ReadOnlyParticleCollection<Particle>(particles);
        Nodes = Particles.SelectMany(p => p.Nodes).ToNodeCollection();

        contacts ??= GetParticleContacts();
        Contacts = contacts.Select(t => new ParticleContact(t.id, Particles[t.from], Particles[t.to])).ToParticleContactCollection();
    }

    private IEnumerable<(Guid id, Guid from, Guid to)> GetParticleContacts()
    {
        var edges = Particles.SelectMany(p => p.Nodes.OfType<NeckNode>())
            .Select(n => new UndirectedEdge<Particle>(n.Particle, n.ContactedNode.Particle));
        var graph = new UndirectedGraph<Particle>(Particles, edges);
        var explorer = BreadthFirstExplorer<Particle>.Explore(graph, Particles[0]);

        return explorer.TraversedEdges.Select(e => (e.Id, e.From.Id, e.To.Id)).ToArray();
    }

    /// <inheritdoc />
    public Guid Id { get; }

    /// <inheritdoc />
    public double Time { get; }

    /// <inheritdoc cref="ISystemState.Nodes"/>>
    public IReadOnlyNodeCollection<NodeBase> Nodes { get; }

    /// <inheritdoc cref="ISystemState.Particles"/>>
    public IReadOnlyParticleCollection<Particle> Particles { get; }

    public IReadOnlyParticleContactCollection<ParticleContact> Contacts { get; }

    IReadOnlyNodeCollection<INode> ISystemState.Nodes => Nodes;
    IReadOnlyParticleCollection<IParticle> ISystemState.Particles => Particles;

    public SolutionState ApplyTimeStep(StepVector stepVector, double timeStepWidth)
    {
        var newParticles = new Dictionary<Guid, Particle>()
        {
            [Particles.Root.Id] =
                Particles.Root.ApplyTimeStep(
                    null,
                    stepVector,
                    timeStepWidth
                )
        };

        foreach (var contact in Contacts)
        {
            newParticles[contact.To.Id] = contact.To.ApplyTimeStep(
                newParticles[contact.From.Id],
                stepVector,
                timeStepWidth
            );
        }

        var newState = new SolutionState(
            Guid.NewGuid(),
            Time + timeStepWidth,
            newParticles.Values,
            Contacts.Select(c => (c.Id, c.From.Id, c.To.Id))
        );

        return newState;
    }
}