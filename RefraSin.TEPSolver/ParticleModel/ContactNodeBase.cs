using RefraSin.Coordinates;
using RefraSin.Coordinates.Polar;
using RefraSin.MaterialData;
using RefraSin.ParticleModel;
using static System.Math;
using static MathNet.Numerics.Constants;

namespace RefraSin.TEPSolver.ParticleModel;

/// <summary>
/// Abstrakte Basisklasse für Oberflächenknoten eines Partikels, welche Kontakt zur Oberfläche eines anderen partiekls haben.
/// </summary>
public abstract class ContactNodeBase<TContacted> : ContactNodeBase where TContacted : ContactNodeBase<TContacted>
{
    /// <inheritdoc />
    protected ContactNodeBase(INode node, Particle particle, ISolverSession solverSession) : base(node, particle, solverSession) { }

    protected ContactNodeBase(Guid id, double r, Angle phi, Particle particle, ISolverSession solverSession, Guid contactedNodeId,
        Guid contactedParticleId) :
        base(id, r, phi, particle, solverSession, contactedNodeId, contactedParticleId) { }

    /// <summary>
    /// Verbundener Knoten des anderen Partikels.
    /// </summary>
    public TContacted ContactedNode => _contactedNode ??=
        SolverSession.CurrentState.AllNodes[ContactedNodeId] as TContacted ??
        throw new InvalidCastException(
            $"Given contacted node {ContactedNodeId} does not refer to an instance of type {typeof(TContacted)}."
        );

    private TContacted? _contactedNode;

    /// <summary>
    /// Properties of the interface between two materials.
    /// </summary>
    public IMaterialInterface MaterialInterface => _materialInterface ??= Particle.MaterialInterfaces[ContactedNode.Particle.Material.Id];

    private IMaterialInterface? _materialInterface;

    /// <inheritdoc />
    public override double TransferCoefficient => MaterialInterface.TransferCoefficient;

    /// <inheritdoc />
    public override double ContactDistance => _contactDistance ??= Particle.CenterCoordinates.DistanceTo(ContactedNode.Particle.CenterCoordinates);

    private double? _contactDistance;

    /// <inheritdoc />
    public override Angle ContactDirection => _contactDirection ??=
        new PolarVector(Particle.CenterCoordinates - ContactedNode.Particle.CenterCoordinates, Particle.LocalCoordinateSystem).Phi;

    private Angle? _contactDirection;

    /// <inheritdoc />
    public override NormalTangential<Angle> CenterShiftVectorDirection => _centerShiftVectorDirection ??= new NormalTangential<Angle>(
        Pi - (Coordinates.Phi - ContactDirection) + (Pi - SurfaceVectorAngle.Normal - SurfaceRadiusAngle.ToLower),
        -(Coordinates.Phi - ContactDirection) + (Pi - SurfaceVectorAngle.Tangential - SurfaceRadiusAngle.ToLower)
    );

    private NormalTangential<Angle>? _centerShiftVectorDirection;

    /// <inheritdoc />
    public override NormalTangential<double> ContactDistanceGradient => _contactDistanceGradient ??= new NormalTangential<double>(
        -Cos(CenterShiftVectorDirection.Normal),
        -Cos(CenterShiftVectorDirection.Tangential)
    );

    private NormalTangential<double>? _contactDistanceGradient;

    /// <inheritdoc />
    public override NormalTangential<double> ContactDirectionGradient => _contactDirectionGradient ??= new NormalTangential<double>(
        Sin(CenterShiftVectorDirection.Normal) / ContactDistance,
        Sin(CenterShiftVectorDirection.Tangential) / ContactDistance
    );

    private NormalTangential<double>? _contactDirectionGradient;
}

/// <summary>
/// Abstrakte Basisklasse für Oberflächenknoten eines Partikels, welche Kontakt zur Oberfläche eines anderen partiekls haben.
/// </summary>
public abstract class ContactNodeBase : NodeBase, IContactNode
{
    private Guid? _contactedParticleId;
    private Guid? _contactedNodeId;

    /// <inheritdoc />
    protected ContactNodeBase(INode node, Particle particle, ISolverSession solverSession) : base(node, particle, solverSession)
    {
        if (node is INodeContact nodeContact)
        {
            _contactedNodeId = nodeContact.ContactedNodeId;
            _contactedParticleId = nodeContact.ContactedParticleId;
        }
        else
        {
            _contactedNodeId = null;
            _contactedParticleId = null;
        }
    }

    protected ContactNodeBase(Guid id, double r, Angle phi, Particle particle, ISolverSession solverSession, Guid contactedNodeId,
        Guid contactedParticleId) :
        base(id, r, phi, particle, solverSession)
    {
        _contactedNodeId = contactedNodeId;
        _contactedParticleId = contactedParticleId;
    }

    /// <inheritdoc />
    public Guid ContactedParticleId => _contactedParticleId ??= SolverSession.CurrentState.AllNodes[ContactedNodeId].ParticleId;

    /// <inheritdoc />
    public Guid ContactedNodeId
    {
        get
        {
            if (_contactedNodeId.HasValue)
                return _contactedNodeId.Value;

            var error = SolverSession.Options.RelativeNodeCoordinateEquivalencePrecision * Particle.MeanRadius;

            var contactedNode =
                SolverSession.CurrentState.AllNodes.Values.FirstOrDefault(n =>
                    n.Id != Id && n.Coordinates.Absolute.Equals(Coordinates.Absolute, error)
                );

            _contactedNodeId = contactedNode?.Id ?? throw new InvalidOperationException("No corresponding node with same location could be found.");
            return _contactedNodeId.Value;
        }
    }

    public ContactNodeBase ContactedNodeBase => _contactedNodeBase ??=
        SolverSession.CurrentState.AllNodes[ContactedNodeId] as ContactNodeBase ??
        throw new InvalidCastException(
            $"Given contacted node {ContactedNodeId} does not refer to an instance of type {typeof(ContactNodeBase)}."
        );

    private ContactNodeBase? _contactedNodeBase;

    /// <inheritdoc />
    public abstract double ContactDistance { get; }

    /// <inheritdoc />
    public abstract Angle ContactDirection { get; }

    /// <inheritdoc />
    public abstract NormalTangential<Angle> CenterShiftVectorDirection { get; }

    /// <inheritdoc />
    public abstract NormalTangential<double> ContactDistanceGradient { get; }

    /// <inheritdoc />
    public abstract NormalTangential<double> ContactDirectionGradient { get; }
}