using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Microsoft.VisualBasic.CompilerServices;
using RefraSin.ParticleModel;

namespace RefraSin.TEPSolver.StepVectors;

public class StepVector : DenseVector
{
    /// <inheritdoc />
    public StepVector(double[] storage, StepVectorMap stepVectorMap) : base(storage)
    {
        StepVectorMap = stepVectorMap;
    }

    /// <inheritdoc />
    public StepVector(Vector<double> vector, StepVectorMap stepVectorMap) : base(vector.AsArray() ?? vector.ToArray())
    {
        StepVectorMap = stepVectorMap;
    }

    public StepVectorMap StepVectorMap { get; }

    public double LambdaDissipation(IParticle particle) => this[StepVectorMap.LambdaDissipation(particle)];

    public double NormalDisplacement(INode node) => this[StepVectorMap.NormalDisplacement(node)];

    public double FluxToUpper(INode node) => this[StepVectorMap.FluxToUpper(node)];

    public double LambdaVolume(INode node) => this[StepVectorMap.LambdaVolume(node)];

    public double TangentialDisplacement(IContactNode node) => this[StepVectorMap.TangentialDisplacement(node)];

    public double LambdaContactDistance(IContactNode node) => this[StepVectorMap.LambdaContactDistance(node)];

    public double LambdaContactDirection(IContactNode node) => this[StepVectorMap.LambdaContactDirection(node)];

    public double RadialDisplacement(IParticleContact contact) => this[StepVectorMap.RadialDisplacement(contact)];

    public double AngleDisplacement(IParticleContact contact) => this[StepVectorMap.AngleDisplacement(contact)];

    public double RotationDisplacement(IParticleContact contact) => this[StepVectorMap.RotationDisplacement(contact)];

    public static StepVector operator +(StepVector leftSide, StepVector rightSide) => new((DenseVector)leftSide + rightSide, leftSide.StepVectorMap);
    public static StepVector operator -(StepVector leftSide, StepVector rightSide) => new((DenseVector)leftSide - rightSide, leftSide.StepVectorMap);
    public static StepVector operator *(StepVector leftSide, double rightSide) => new((DenseVector)leftSide * rightSide, leftSide.StepVectorMap);
    public static StepVector operator *(double leftSide, StepVector rightSide) => rightSide * leftSide;
    public static StepVector operator /(StepVector leftSide, double rightSide) => new((DenseVector)leftSide / rightSide, leftSide.StepVectorMap);

    public StepVector Copy() => new(Build.DenseOfVector(this), StepVectorMap);

    public double[] ParticleBlock(IParticle particle)
    {
        var block = StepVectorMap[particle];

        return Values[block.start .. (block.start + block.length)];
    }

    public double[] BorderBlock() => Values[StepVectorMap.BorderStart ..];

    public void UpdateParticleBlock(IParticle particle, double[] data)
    {
        var block = StepVectorMap[particle];

        if (data.Length != block.length)
            throw new InvalidOperationException("'data' must have exactly the length of the particle block.");

        data.CopyTo(Values, block.start);
    }

    public void UpdateBorderBlock(double[] data)
    {
        if (data.Length != StepVectorMap.BorderLength)
            throw new InvalidOperationException("'data' must have exactly the length of the particle block.");

        data.CopyTo(Values, StepVectorMap.BorderStart);
    }
}