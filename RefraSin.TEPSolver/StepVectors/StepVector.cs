using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using RefraSin.Graphs;
using RefraSin.TEPSolver.ParticleModel;

namespace RefraSin.TEPSolver.StepVectors;

public class StepVector : DenseVector
{
    /// <inheritdoc />
    public StepVector(double[] storage, StepVectorMap stepVectorMap)
        : base(storage)
    {
        StepVectorMap = stepVectorMap;
    }

    /// <inheritdoc />
    public StepVector(Vector<double> vector, StepVectorMap stepVectorMap)
        : base(vector.AsArray() ?? vector.ToArray())
    {
        StepVectorMap = stepVectorMap;
    }

    public StepVectorMap StepVectorMap { get; }

    public double LambdaDissipation() => this[StepVectorMap.LambdaDissipation()];

    public double NormalDisplacement(INode node) => this[StepVectorMap.NormalDisplacement(node)];

    public double FluxToUpper(INode node) => this[StepVectorMap.FluxToUpper(node)];

    public double LambdaVolume(INode node) => this[StepVectorMap.LambdaVolume(node)];

    public double TangentialDisplacement(INode node) =>
        this[StepVectorMap.TangentialDisplacement(node)];

    public double LambdaContactDistance(INode node) =>
        this[StepVectorMap.LambdaContactDistance(node)];

    public double LambdaContactDirection(INode node) =>
        this[StepVectorMap.LambdaContactDirection(node)];

    public double LambdaCycleX(IGraphCycle<Particle, ParticleContact> cycle) =>
        this[StepVectorMap.LambdaCycleX(cycle)];

    public double LambdaCycleY(IGraphCycle<Particle, ParticleContact> cycle) =>
        this[StepVectorMap.LambdaCycleY(cycle)];

    public double RadialDisplacement(ParticleContact contact) =>
        this[StepVectorMap.RadialDisplacement(contact)];

    public double AngleDisplacement(ParticleContact contact) =>
        this[StepVectorMap.AngleDisplacement(contact)];

    public void LambdaDissipation(double value) => this[StepVectorMap.LambdaDissipation()] = value;

    public void NormalDisplacement(INode node, double value) =>
        this[StepVectorMap.NormalDisplacement(node)] = value;

    public void FluxToUpper(INode node, double value) =>
        this[StepVectorMap.FluxToUpper(node)] = value;

    public void LambdaVolume(INode node, double value) =>
        this[StepVectorMap.LambdaVolume(node)] = value;

    public void TangentialDisplacement(INode node, double value) =>
        this[StepVectorMap.TangentialDisplacement(node)] = value;

    public void LambdaContactDistance(INode node, double value) =>
        this[StepVectorMap.LambdaContactDistance(node)] = value;

    public void LambdaContactDirection(INode node, double value) =>
        this[StepVectorMap.LambdaContactDirection(node)] = value;

    public void RadialDisplacement(ParticleContact contact, double value) =>
        this[StepVectorMap.RadialDisplacement(contact)] = value;

    public void AngleDisplacement(ParticleContact contact, double value) =>
        this[StepVectorMap.AngleDisplacement(contact)] = value;

    public static StepVector operator +(StepVector leftSide, StepVector rightSide) =>
        new((DenseVector)leftSide + rightSide, leftSide.StepVectorMap);

    public static StepVector operator -(StepVector leftSide, StepVector rightSide) =>
        new((DenseVector)leftSide - rightSide, leftSide.StepVectorMap);

    public static StepVector operator *(StepVector leftSide, double rightSide) =>
        new((DenseVector)leftSide * rightSide, leftSide.StepVectorMap);

    public static StepVector operator *(double leftSide, StepVector rightSide) =>
        rightSide * leftSide;

    public static StepVector operator /(StepVector leftSide, double rightSide) =>
        new((DenseVector)leftSide / rightSide, leftSide.StepVectorMap);

    public StepVector Copy() => new(Build.DenseOfVector(this), StepVectorMap);

    public Vector<double> ParticleBlock(IParticle particle)
    {
        var block = StepVectorMap[particle];
        return new DenseVector(Values[block.start..(block.start + block.length)]);
    }

    public Vector<double> ContactBlock(IParticleContactEdge contact)
    {
        var block = StepVectorMap[contact];
        return new DenseVector(Values[block.start..(block.start + block.length)]);
    }

    public Vector<double> GlobalBlock() => new DenseVector(Values[StepVectorMap.GlobalStart..]);

    public void Update(double[] data)
    {
        data.CopyTo(Values, 0);
    }

    public void UpdateParticleBlock(IParticle particle, double[] data)
    {
        var block = StepVectorMap[particle];

        if (data.Length != block.length)
            throw new InvalidOperationException(
                "'data' must have exactly the length of the particle block."
            );

        data.CopyTo(Values, block.start);
    }

    public void UpdateContactBlock(IParticleContactEdge contact, double[] data)
    {
        var block = StepVectorMap[contact];

        if (data.Length != block.length)
            throw new InvalidOperationException(
                "'data' must have exactly the length of the contact block."
            );

        data.CopyTo(Values, block.start);
    }

    public void UpdateGlobalBlock(double[] data)
    {
        if (data.Length != StepVectorMap.GlobalLength)
            throw new InvalidOperationException(
                "'data' must have exactly the length of the global block."
            );

        data.CopyTo(Values, StepVectorMap.GlobalStart);
    }
}
