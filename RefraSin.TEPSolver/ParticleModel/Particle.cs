using System.Globalization;
using RefraSin.Coordinates;
using RefraSin.Coordinates.Absolute;
using RefraSin.Coordinates.Cartesian;
using RefraSin.Coordinates.Polar;
using RefraSin.Graphs;
using RefraSin.MaterialData;
using RefraSin.ParticleModel.Collections;
using RefraSin.ParticleModel.Nodes;
using RefraSin.ParticleModel.Particles;
using RefraSin.TEPSolver.StepVectors;
using static System.Math;

namespace RefraSin.TEPSolver.ParticleModel;

/// <summary>
/// Stellt ein Pulverpartikel dar.
/// </summary>
public class Particle : IParticle
{
    private ReadOnlyParticleSurface<NodeBase> _nodes;
    private double? _meanRadius;

    public Particle(IParticle particle, ISolverSession solverSession)
    {
        Id = particle.Id;
        Coordinates = particle.Coordinates.Absolute;
        RotationAngle = particle.RotationAngle;

        LocalCoordinateSystem = new PolarCoordinateSystem
        {
            OriginSource = () => Coordinates,
            RotationAngleSource = () => RotationAngle
        };

        MaterialId = particle.MaterialId;
        var material = solverSession.Materials[particle.MaterialId];
        VacancyVolumeEnergy =
            solverSession.Temperature
          * solverSession.GasConstant
          / (material.Substance.MolarVolume * material.Bulk.EquilibriumVacancyConcentration);

        SurfaceProperties = material.Surface;

        InterfaceProperties = solverSession.MaterialInterfaces[material.Id].ToDictionary(mi => mi.To, mi => mi.Properties);

        SolverSession = solverSession;
        _nodes = particle
            .Nodes.Select(node =>
                node switch
                {
                    INeckNode neckNode => new NeckNode(neckNode, this, solverSession),
                    IGrainBoundaryNode grainBoundaryNode
                        => new GrainBoundaryNode(grainBoundaryNode, this, solverSession),
                    { Type: NodeType.GrainBoundary }
                        => new GrainBoundaryNode(node, this, solverSession),
                    { Type: NodeType.Neck } => new NeckNode(node, this, solverSession),
                    _                           => (NodeBase)new SurfaceNode(node, this, solverSession),
                }
            )
            .ToParticleSurface();
    }

    private Particle(
        Particle? parent,
        Particle previousState,
        StepVector stepVector,
        double timeStepWidth
    )
    {
        Id = previousState.Id;

        LocalCoordinateSystem = new PolarCoordinateSystem
        {
            OriginSource = () => Coordinates,
            RotationAngleSource = () => RotationAngle
        };

        MaterialId = previousState.MaterialId;
        VacancyVolumeEnergy = previousState.VacancyVolumeEnergy;
        SurfaceProperties = previousState.SurfaceProperties;
        InterfaceProperties = previousState.InterfaceProperties;

        SolverSession = previousState.SolverSession;

        // Apply time step changes
        if (parent is null) // is root particle
        {
            Coordinates = previousState.Coordinates;
            RotationAngle = previousState.RotationAngle;
        }
        else
        {
            var contact = SolverSession.CurrentState.ParticleContacts[parent.Id, previousState.Id];
            var displacementVector = new PolarVector(
                stepVector.AngleDisplacement(contact) * timeStepWidth,
                stepVector.RadialDisplacement(contact) * timeStepWidth,
                parent.LocalCoordinateSystem
            );
            Coordinates = previousState.Coordinates + displacementVector.Absolute;

            RotationAngle = previousState.RotationAngle;
        }

        _nodes = previousState
            .Nodes.Select(n => n.ApplyTimeStep(stepVector, timeStepWidth, this))
            .ToParticleSurface();
    }

    /// <inheritdoc/>
    public Guid Id { get; }

    /// <inheritdoc />
    public Guid MaterialId { get; }

    public IInterfaceProperties SurfaceProperties { get; }

    /// <summary>
    /// Dictionary of material IDs to material interface data, assuming that the current instances material is always on the from side.
    /// </summary>
    public IReadOnlyDictionary<Guid, IInterfaceProperties> InterfaceProperties { get; }

    /// <summary>
    /// Lokales Koordinatensystem des Partikels. Bearbeitung über <see cref="Coordinates"/> und <see cref="RotationAngle"/>. Sollte nicht direkt verändert werden!!!
    /// </summary>
    internal PolarCoordinateSystem LocalCoordinateSystem { get; }

    /// <summary>
    /// Koordinaten des Ursprungs des lokalen Koordinatensystem ausgedrückt im Koordinatensystem des <see cref="Parent"/>
    /// </summary>
    public AbsolutePoint Coordinates { get; }

    ICartesianPoint IParticle.Coordinates => Coordinates;

    /// <summary>
    /// Drehwinkel des Partikels.
    /// </summary>
    public Angle RotationAngle { get; }

    public IReadOnlyParticleSurface<NodeBase> Nodes => _nodes;

    IReadOnlyParticleSurface<INodeGeometry> IParticle.Nodes => Nodes;

    /// <summary>
    /// Reference to the current solver session.
    /// </summary>
    private ISolverSession SolverSession { get; }

    public double MeanRadius => _meanRadius ??= Sqrt(Nodes.Sum(n => n.Volume.ToUpper)) / PI;

    public double VacancyVolumeEnergy { get; }

    public Particle ApplyTimeStep(Particle? parent, StepVector stepVector, double timeStepWidth) =>
        new(parent, this, stepVector, timeStepWidth);

    /// <inheritdoc/>
    public override string ToString() => $"{GetType().Name} {Id} @ {Coordinates.ToString("(,)", CultureInfo.InvariantCulture)}";

    /// <inheritdoc />
    public virtual bool Equals(IVertex? other) => other is IParticle && Id == other.Id;
}