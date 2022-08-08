namespace RefraSin.Core.ParticleModel.States
{
    /// <summary>
    /// Stellt den Zustand eines Korngrenzenknotens dar.
    /// </summary>
    public class GrainBoundaryNodeState : ContactNodeState, IGrainBoundaryNode
    {
        /// <summary>
        /// Kopierkonstruktor.
        /// </summary>
        /// <param name="template">Vorlage</param>
        public GrainBoundaryNodeState(IGrainBoundaryNode template) : base(template)
        {
        }
    }
}