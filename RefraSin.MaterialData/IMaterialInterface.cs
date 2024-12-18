namespace RefraSin.MaterialData;

/// <summary>
/// Interface for types providing material data of interfaces (boundaries, ...).
/// </summary>
public interface IMaterialInterface
{
    /// <summary>
    /// Unique ID of the material the view is originating from.
    /// </summary>
    Guid From { get; }

    /// <summary>
    /// Unique ID of the material the view is directing to.
    /// </summary>
    Guid To { get; }

    IInterfaceProperties Properties { get; }
}
