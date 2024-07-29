using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double.Solvers;
using MathNet.Numerics.LinearAlgebra.Solvers;
using RefraSin.Numerics.LinearSolvers;

namespace RefraSin.Numerics.Tests;

[TestFixtureSource(nameof(GenerateFixtureData))]
public class LinearSolverTest
{
    public LinearSolverTest(double[,] systemMatrix, double[] solution)
    {
        _systemMatrix = Matrix<double>.Build.DenseOfArray(systemMatrix);
        _solution = Vector<double>.Build.DenseOfArray(solution);
        _rightSide = _systemMatrix * _solution;
    }

    public static IEnumerable<TestFixtureData> GenerateFixtureData()
    {
        yield return new TestFixtureData(
            new double[,]
            {
                { 1, 2, 3 },
                { 4, 8, 1 },
                { 7, 2, 2 }
            },
            new double[] { 1, 1, 2 }
        );
        yield return new TestFixtureData(
            Matrix<double>.Build.Random(10, 10, 123456).ToArray(),
            Vector<double>.Build.Random(10, 123456).ToArray()
        );
        yield return new TestFixtureData(
            Matrix<double>.Build.Random(100, 100, 123456).ToArray(),
            Vector<double>.Build.Random(100, 123456).ToArray()
        );
        yield return new TestFixtureData(
            Matrix<double>.Build.Random(1000, 1000, 123456).ToArray(),
            Vector<double>.Build.Random(1000, 123456).ToArray()
        );
        yield return new TestFixtureData(
            Matrix<double>.Build.Random(100, 50, 123456).ToArray(),
            Vector<double>.Build.Random(50, 123456).ToArray()
        );
        yield return new TestFixtureData(
            Matrix<double>.Build.Random(1000, 500, 123456).ToArray(),
            Vector<double>.Build.Random(500, 123456).ToArray()
        );
    }

    private Matrix<double> _systemMatrix;
    private Vector<double> _solution;
    private Vector<double> _rightSide;

    [Test]
    public void TestKaczmarzWithSequence()
    {
        var solver = new KaczmarzSolver(
            weightingMatrixFactory: new KaczmarzSolver.SequentialWeightingMatrixFactory(),
            maximumEpochCount: 10000
        );
        var solution = solver.Solve(_systemMatrix, _rightSide, null);

        Assert.That(solution.ToArray(), Is.EqualTo(_solution.ToArray()).Within(1e-7));
    }

    [Test]
    public void TestKaczmarzWithUniform()
    {
        var solver = new KaczmarzSolver(
            weightingMatrixFactory: new KaczmarzSolver.UniformWeightingMatrixFactory(),
            maximumEpochCount: 10000
        );
        var solution = solver.Solve(_systemMatrix, _rightSide, null);

        Assert.That(solution.ToArray(), Is.EqualTo(_solution.ToArray()).Within(1e-7));
    }

    [Test]
    public void TestKaczmarzWithRowNorm()
    {
        var solver = new KaczmarzSolver(
            weightingMatrixFactory: new KaczmarzSolver.RowNormWeightingMatrixFactory(),
            maximumEpochCount: 10000
        );
        var solution = solver.Solve(_systemMatrix, _rightSide, null);

        Assert.That(solution.ToArray(), Is.EqualTo(_solution.ToArray()).Within(1e-7));
    }

    [Test]
    public void TestLU()
    {
        if (_systemMatrix.RowCount != _systemMatrix.ColumnCount)
            Assert.Inconclusive("matrix must be square for this algorithm");

        var solver = new LUSolver();
        var solution = solver.Solve(_systemMatrix, _rightSide, null);

        Assert.That(solution.ToArray(), Is.EqualTo(_solution.ToArray()).Within(1e-7));
    }

    [Test]
    public void TestIterative()
    {
        if (_systemMatrix.RowCount != _systemMatrix.ColumnCount)
            Assert.Inconclusive("matrix must be square for this algorithm");

        var solver = new IterativeSolver(
            new MlkBiCgStab(),
            new Iterator<double>(),
            new DiagonalPreconditioner()
        );
        var solution = solver.Solve(_systemMatrix, _rightSide, null);

        Assert.That(solution.ToArray(), Is.EqualTo(_solution.ToArray()).Within(1e-7));
    }
}