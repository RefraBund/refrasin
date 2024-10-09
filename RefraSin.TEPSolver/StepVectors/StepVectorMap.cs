using RefraSin.ParticleModel;
using RefraSin.ParticleModel.Nodes;
using RefraSin.ParticleModel.Particles;
using RefraSin.TEPSolver.ParticleModel;

namespace RefraSin.TEPSolver.StepVectors;

public class StepVectorMap
{
    public StepVectorMap(SolutionState currentState)
    {
        _index = 0;

        foreach (var particle in currentState.Particles)
        {
            var startIndex = _index;

            foreach (var node in particle.Nodes)
            {
                if (node is not INodeContact)
                {
                    AddUnknown(node.Id, Unknown.NormalDisplacement);
                    AddUnknown(node.Id, Unknown.FluxToUpper);
                    AddUnknown(node.Id, Unknown.LambdaVolume);
                    AddUnknown(node.Id, Unknown.NormalStress);
                    AddUnknown(node.Id, Unknown.TangentialStress);
                    AddUnknown(node.Id, Unknown.LambdaNormalStress);
                    AddUnknown(node.Id, Unknown.LambdaTangentialStress);
                }
            }

            AddUnknown(particle.Id, Unknown.LambdaHorizontalForceBalance);
            AddUnknown(particle.Id, Unknown.LambdaVerticalForceBalance);
            AddUnknown(particle.Id, Unknown.LambdaTorqueBalance);

            _particleBlocks[particle.Id] = (startIndex, _index - startIndex);
        }

        foreach (var contact in currentState.ParticleContacts)
        {
            var startIndex = _index;

            AddUnknown(contact.MergedId, Unknown.RadialDisplacement);
            AddUnknown(contact.MergedId, Unknown.AngleDisplacement);
            AddUnknown(contact.MergedId, Unknown.RotationDisplacement);

            foreach (var contactNode in contact.FromNodes)
            {
                AddUnknown(contactNode.Id, Unknown.LambdaContactDistance);
                AddUnknown(contactNode.Id, Unknown.LambdaContactDirection);
                LinkUnknown(
                    contactNode.Id,
                    contactNode.ContactedNodeId,
                    Unknown.LambdaContactDistance
                );
                LinkUnknown(
                    contactNode.Id,
                    contactNode.ContactedNodeId,
                    Unknown.LambdaContactDirection
                );

                AddUnknown(contactNode.Id, Unknown.NormalDisplacement);
                AddUnknown(contactNode.Id, Unknown.TangentialDisplacement);
                AddUnknown(contactNode.Id, Unknown.FluxToUpper);
                AddUnknown(contactNode.Id, Unknown.LambdaVolume);
                AddUnknown(contactNode.Id, Unknown.NormalStress);
                AddUnknown(contactNode.Id, Unknown.TangentialStress);
                AddUnknown(contactNode.Id, Unknown.LambdaNormalStress);
                AddUnknown(contactNode.Id, Unknown.LambdaTangentialStress);

                AddUnknown(contactNode.ContactedNodeId, Unknown.NormalDisplacement);
                AddUnknown(contactNode.ContactedNodeId, Unknown.TangentialDisplacement);
                AddUnknown(contactNode.ContactedNodeId, Unknown.FluxToUpper);
                AddUnknown(contactNode.ContactedNodeId, Unknown.LambdaVolume);
                AddUnknown(contactNode.ContactedNodeId, Unknown.NormalStress);
                AddUnknown(contactNode.ContactedNodeId, Unknown.TangentialStress);
                LinkUnknown(
                    contactNode.Id,
                    contactNode.ContactedNodeId,
                    Unknown.LambdaNormalStress
                );
                LinkUnknown(
                    contactNode.Id,
                    contactNode.ContactedNodeId,
                    Unknown.LambdaTangentialStress
                );
            }

            _contactBlocks[(contact.From.Id, contact.To.Id)] = (startIndex, _index - startIndex);
        }

        GlobalStart = _index;

        AddUnknown(Guid.Empty, Unknown.LambdaDissipation);

        GlobalLength = _index - GlobalStart;
        TotalLength = _index;
    }

    public int TotalLength { get; }

    private void AddUnknown(Guid id, Unknown unknown)
    {
        _indices[(id, unknown)] = _index;
        _index++;
    }

    private void LinkUnknown(Guid existingId, Guid newId, Unknown unknown)
    {
        _indices[(newId, unknown)] = _indices[(existingId, unknown)];
    }

    private int _index;
    private readonly Dictionary<(Guid, Unknown), int> _indices = new();
    private readonly Dictionary<Guid, (int start, int length)> _particleBlocks = new();
    private readonly Dictionary<(Guid, Guid), (int start, int length)> _contactBlocks = new();

    public (int start, int length) this[IParticle particle] => _particleBlocks[particle.Id];

    public (int start, int length) this[IParticleContactEdge contact] =>
        _contactBlocks[(contact.From, contact.To)];

    public int GlobalStart { get; }

    public int GlobalLength { get; }

    public int LambdaDissipation() => _indices[(Guid.Empty, Unknown.LambdaDissipation)];

    public int NormalDisplacement(INode node) => _indices[(node.Id, Unknown.NormalDisplacement)];

    public int FluxToUpper(INode node) => _indices[(node.Id, Unknown.FluxToUpper)];

    public int LambdaVolume(INode node) => _indices[(node.Id, Unknown.LambdaVolume)];

    public int LambdaNormalStress(INode node) => _indices[(node.Id, Unknown.LambdaNormalStress)];

    public int LambdaTangentialStress(INode node) =>
        _indices[(node.Id, Unknown.LambdaTangentialStress)];

    public int TangentialDisplacement(INode node) =>
        _indices[(node.Id, Unknown.TangentialDisplacement)];

    public int NormalStress(INode node) => _indices[(node.Id, Unknown.NormalStress)];

    public int TangentialStress(INode node) => _indices[(node.Id, Unknown.TangentialStress)];

    public int LambdaContactDistance(INode node) =>
        _indices[(node.Id, Unknown.LambdaContactDistance)];

    public int LambdaContactDirection(INode node) =>
        _indices[(node.Id, Unknown.LambdaContactDirection)];

    public int LambdaHorizontalForceBalance(Particle particle) =>
        _indices[(particle.Id, Unknown.LambdaHorizontalForceBalance)];

    public int LambdaVerticalForceBalance(Particle particle) =>
        _indices[(particle.Id, Unknown.LambdaVerticalForceBalance)];

    public int LambdaTorqueBalance(Particle particle) =>
        _indices[(particle.Id, Unknown.LambdaTorqueBalance)];

    public int RadialDisplacement(ParticleContact contact) =>
        _indices[(contact.MergedId, Unknown.RadialDisplacement)];

    public int AngleDisplacement(ParticleContact contact) =>
        _indices[(contact.MergedId, Unknown.AngleDisplacement)];

    public int RotationDisplacement(ParticleContact contact) =>
        _indices[(contact.MergedId, Unknown.RotationDisplacement)];

    private enum Unknown
    {
        NormalDisplacement,
        TangentialDisplacement,
        FluxToUpper,
        LambdaVolume,
        LambdaContactDistance,
        LambdaContactDirection,
        NormalStress,
        TangentialStress,
        LambdaNormalStress,
        LambdaTangentialStress,
        LambdaHorizontalForceBalance,
        LambdaVerticalForceBalance,
        LambdaTorqueBalance,
        RadialDisplacement,
        AngleDisplacement,
        RotationDisplacement,
        LambdaDissipation,
    }
}
