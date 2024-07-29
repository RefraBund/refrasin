using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Random;
using RefraSin.Numerics.Exceptions;

namespace RefraSin.Numerics.LinearSolvers;

/// <summary>
/// Kaczmarz solver inspired by L. Föcke - Inverse Problems for Random Measurements
/// </summary>
public class KaczmarzSolver(
    int randomSeed = 980323704,
    int maximumEpochCount = 1000,
    double precision = 1e-8,
    double stepBreakFactor = 1,
    KaczmarzSolver.IWeightingMatrixFactory? weightingMatrixFactory = null
) : ILinearSolver
{
    /// <inheritdoc />
    public Vector<double> Solve(
        Matrix<double> matrix,
        Vector<double> rightSide,
        Vector<double>? initialGuess = null
    )
    {
        int epoch;
        var equationCount = rightSide.Count;

        var weightingMatrix = WeightingMatrixFactory.Compute(matrix, rightSide);
        var randomSource = new MersenneTwister(RandomSeed);
        var probabilityDistributions = weightingMatrix
            .EnumerateRows()
            .Select(r => new Categorical(r.ToArray(), randomSource))
            .ToArray();

        var solutionVector = initialGuess is null
            ? Vector<double>.Build.Dense(matrix.ColumnCount)
            : Vector<double>.Build.DenseOfVector(initialGuess);
        var stepSizes = matrix
            .EnumerateRows()
            .Select(r => StepBreakFactor * double.Pow(r.L2Norm(), -2))
            .ToArray();

        for (epoch = 0; epoch < MaximumEpochCount; epoch++)
        {
            int randomEquation = randomSource.Next(matrix.RowCount);

            for (var use = 0; use < equationCount; use++)
            {
                var row = matrix.Row(randomEquation);
                var step = (row.DotProduct(solutionVector) - rightSide[randomEquation]) * row;

                solutionVector -= stepSizes[randomEquation] * step;

                randomEquation = probabilityDistributions[randomEquation].Sample();
            }

            if ((matrix * solutionVector - rightSide).L2Norm() < Precision)
                return solutionVector;
        }

        throw new UncriticalIterationInterceptedException(
            nameof(KaczmarzSolver),
            InterceptReason.MaxIterationCountExceeded,
            epoch
        );
    }

    public IWeightingMatrixFactory WeightingMatrixFactory { get; } =
        weightingMatrixFactory ?? new SequentialWeightingMatrixFactory();

    public int MaximumEpochCount { get; } = maximumEpochCount;

    public int RandomSeed { get; } = randomSeed;

    public double Precision { get; } = precision;

    public double StepBreakFactor { get; } = stepBreakFactor;

    public interface IWeightingMatrixFactory
    {
        Matrix<double> Compute(Matrix<double> systemMatrix, Vector<double> rightSide);
    }

    public class SequentialWeightingMatrixFactory : IWeightingMatrixFactory
    {
        /// <inheritdoc />
        public Matrix<double> Compute(Matrix<double> systemMatrix, Vector<double> rightSide) =>
            Matrix<double>.Build.SparseOfIndexed(
                systemMatrix.RowCount,
                systemMatrix.RowCount,
                Enumerable
                    .Range(0, systemMatrix.RowCount - 1)
                    .Select(i => (i, i + 1, 1.0))
                    .Append((systemMatrix.RowCount - 1, 0, 1.0))
            );
    }

    public class UniformWeightingMatrixFactory : IWeightingMatrixFactory
    {
        /// <inheritdoc />
        public Matrix<double> Compute(Matrix<double> systemMatrix, Vector<double> rightSide) =>
            Matrix<double>.Build.Dense(
                systemMatrix.RowCount,
                systemMatrix.RowCount,
                1.0 / systemMatrix.RowCount
            );
    }

    public class RowNormWeightingMatrixFactory : IWeightingMatrixFactory
    {
        /// <inheritdoc />
        public Matrix<double> Compute(Matrix<double> systemMatrix, Vector<double> rightSide)
        {
            var systemNorm = systemMatrix.L2Norm();
            var weights = Vector<double>.Build.DenseOfEnumerable(
                systemMatrix.EnumerateRows().Select(r => r.L2Norm() / systemNorm)
            );

            return Matrix<double>.Build.DenseOfRowVectors(Enumerable.Repeat(weights, systemMatrix.RowCount));
        }
    }
}