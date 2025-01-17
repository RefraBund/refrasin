using RefraSin.Coordinates;
using RefraSin.MaterialData;
using RefraSin.ParticleModel;

namespace RefraSin.TEPSolver.ParticleModel;

/// <summary>
/// Abstrakte Basisklasse für Oberflächenknoten eines Partikels, welche Kontakt zur Oberfläche eines anderen partiekls haben.
/// </summary>
public abstract class ContactNodeBase<TContacted> : ContactNodeBase
    where TContacted : ContactNodeBase<TContacted>
{
    /// <inheritdoc />
    protected ContactNodeBase(INode node, Particle particle)
        : base(node, particle) { }

    protected ContactNodeBase(
        Guid id,
        double r,
        Angle phi,
        Particle particle,
        Guid contactedNodeId,
        Guid contactedParticleId
    )
        : base(id, r, phi, particle, contactedNodeId, contactedParticleId) { }

    /// <summary>
    /// Verbundener Knoten des anderen Partikels.
    /// </summary>
    public new TContacted ContactedNode =>
        _contactedNode ??=
            Particle.SolutionState.Nodes[ContactedNodeId] as TContacted
            ?? throw new InvalidCastException(
                $"Given contacted node {ContactedNodeId} does not refer to an instance of type {typeof(TContacted)}."
            );

    private TContacted? _contactedNode;

    /// <summary>
    /// Properties of the interface between two materials.
    /// </summary>
    public IInterfaceProperties InterfaceProperties =>
        _materialInterface ??= Particle.InterfaceProperties[ContactedNode.Particle.MaterialId];

    private IInterfaceProperties? _materialInterface;

    /// <inheritdoc />
    public override NormalTangential<Angle> CenterShiftVectorDirection =>
        _centerShiftVectorDirection ??= IsParentsNode
            ? new NormalTangential<Angle>(
                AngleDistanceToContactDirection - (Pi - RadiusNormalAngle.ToLower),
                AngleDistanceToContactDirection + (Pi - RadiusTangentAngle.ToUpper)
            )
            : new NormalTangential<Angle>(
                (Pi - RadiusNormalAngle.ToUpper) + AngleDistanceToContactDirection,
                (Pi - RadiusTangentAngle.ToUpper) + AngleDistanceToContactDirection
            );

    private NormalTangential<Angle>? _centerShiftVectorDirection;

    /// <inheritdoc />
    public override NormalTangential<Angle> TorqueProjectionAngle =>
        _torqueProjectionAngle ??= IsParentsNode
            ? new NormalTangential<Angle>(
                RadiusNormalAngle.ToLower
                    - (
                        Pi
                        - AngleDistanceToContactDirection
                        + ContactedNode.AngleDistanceToContactDirection
                    ),
                Pi
                    - RadiusTangentAngle.ToLower
                    - AngleDistanceToContactDirection
                    + ContactedNode.AngleDistanceToContactDirection
            ) * -1
            : new NormalTangential<Angle>(
                Pi - RadiusNormalAngle.ToUpper,
                RadiusTangentAngle.ToUpper
            );

    private NormalTangential<Angle>? _torqueProjectionAngle;

    public override NormalTangential<double> TorqueLeverArm =>
        _torqueLeverArm ??=
            new NormalTangential<double>(
                Sin(TorqueProjectionAngle.Normal),
                Sin(TorqueProjectionAngle.Tangential)
            ) * (IsParentsNode ? ContactedNode.Coordinates.R : Coordinates.R);

    private NormalTangential<double>? _torqueLeverArm;

    /// <inheritdoc />
    public override NormalTangential<double> ContactDistanceGradient =>
        _contactDistanceGradient ??= new NormalTangential<double>(
            Cos(CenterShiftVectorDirection.Normal),
            Cos(CenterShiftVectorDirection.Tangential)
        );

    private NormalTangential<double>? _contactDistanceGradient;

    /// <inheritdoc />
    public override NormalTangential<double> ContactDirectionGradient =>
        _contactDirectionGradient ??= new NormalTangential<double>(
            Sin(CenterShiftVectorDirection.Normal) / ContactDistance,
            Sin(CenterShiftVectorDirection.Tangential) / ContactDistance
        );

    private NormalTangential<double>? _contactDirectionGradient;
}

/// <summary>
/// Abstrakte Basisklasse für Oberflächenknoten eines Partikels, welche Kontakt zur Oberfläche eines anderen partiekls haben.
/// </summary>
public abstract class ContactNodeBase
    : NodeBase,
        INodeContactGradients,
        INodeContactNeighbors,
        INodeContactGeometry
{
    private Guid? _contactedParticleId;
    private Guid? _contactedNodeId;

    /// <inheritdoc />
    protected ContactNodeBase(INode node, Particle particle)
        : base(node, particle)
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

    protected ContactNodeBase(
        Guid id,
        double r,
        Angle phi,
        Particle particle,
        Guid contactedNodeId,
        Guid contactedParticleId
    )
        : base(id, r, phi, particle)
    {
        _contactedNodeId = contactedNodeId;
        _contactedParticleId = contactedParticleId;
    }

    /// <inheritdoc />
    public Guid ContactedParticleId =>
        _contactedParticleId ??= Particle.SolutionState.Nodes[ContactedNodeId].ParticleId;

    /// <inheritdoc />
    public Guid ContactedNodeId =>
        _contactedNodeId ??= Particle.SolutionState.NodeContacts.From(Id).Single().To.Id;

    public Particle ContactedParticle => ContactedNode.Particle;

    IParticle INodeContactNeighbors.ContactedParticle => ContactedParticle;

    public ContactNodeBase ContactedNode =>
        _contactedNode ??=
            Particle.SolutionState.Nodes[ContactedNodeId] as ContactNodeBase
            ?? throw new InvalidCastException(
                $"Given contacted node {ContactedNodeId} does not refer to an instance of type {typeof(ContactNodeBase)}."
            );

    private ContactNodeBase? _contactedNode;

    INodeContactNeighbors INodeContactNeighbors.ContactedNode => ContactedNode;

    public IPolarVector ContactVector =>
        IsParentsNode ? Contact.ContactVector : Contact.Reversed().ContactVector;

    public double ContactDistance => ContactVector.R;

    public Angle ContactDirection => IsParentsNode ? Contact.DirectionFrom : Contact.DirectionTo;

    /// <inheritdoc />
    public Angle AngleDistanceToContactDirection =>
        _angleDistanceFromContactDirection ??= ContactVector.AngleTo(
            Coordinates,
            allowNegative: true
        );

    private Angle? _angleDistanceFromContactDirection;

    /// <inheritdoc />
    public abstract NormalTangential<Angle> CenterShiftVectorDirection { get; }

    public abstract NormalTangential<Angle> TorqueProjectionAngle { get; }

    public abstract NormalTangential<double> TorqueLeverArm { get; }

    /// <inheritdoc />
    public abstract NormalTangential<double> ContactDistanceGradient { get; }

    /// <inheritdoc />
    public abstract NormalTangential<double> ContactDirectionGradient { get; }

    public bool IsParentsNode => _isParentsNode ??= Contact.From.Id == ParticleId;
    private bool? _isParentsNode;

    public ParticleContact Contact
    {
        get
        {
            if (_contact is not null)
                return _contact;

            _contact = Particle.SolutionState.ParticleContacts.FirstOrDefault(c =>
                c.From.Id == ParticleId && c.To.Id == ContactedParticleId
            );

            if (_contact is not null)
                return _contact;

            _contact = Particle.SolutionState.ParticleContacts.FirstOrDefault(c =>
                c.To.Id == ParticleId && c.From.Id == ContactedParticleId
            );

            if (_contact is not null)
                return _contact;

            throw new InvalidOperationException("Related contact was not found.");
        }
    }

    private ParticleContact? _contact;
}
