using RefraSin.Coordinates;

namespace RefraSin.ParticleModel;

/// <summary>
/// Structure encapsulating quantities with two components, one in normal direction, one in tangential.
/// </summary>
public readonly record struct NormalTangential<TValue>(TValue Normal, TValue Tangential)
{
    /// <summary>
    /// Returns the components of the instance as 2-element array.
    /// </summary>
    public TValue[] ToArray() => new[] { Normal, Tangential };

    public static implicit operator NormalTangential<TValue>(TValue both) => new(both, both);
    public static implicit operator NormalTangential<TValue>((TValue Normal, TValue Tangential) t) => new(t.Normal, t.Tangential);
}

public static class NormalTangentialExtensions
{
    public static double[] ToDoubleArray(this NormalTangential<Angle> self) => new double[] { self.Normal, self.Tangential };
}