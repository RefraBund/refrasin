using System.Globalization;
using MathNet.Numerics.Statistics;
using Microsoft.Extensions.Logging;
using static MoreLinq.Extensions.IndexExtension;
using RefraSin.MaterialData;
using RefraSin.ParticleModel;
using RefraSin.ParticleModel.ParticleSpecFactories;
using RefraSin.ProcessModel;
using RefraSin.Storage;
using ScottPlot;
using Serilog;
using static NUnit.Framework.Assert;
using Particle = RefraSin.TEPSolver.ParticleModel.Particle;

namespace RefraSin.TEPSolver.Test;

[TestFixture]
public class OneParticleTest
{
    [SetUp]
    public void Setup()
    {
        var endTime = 1e4;

        _particleSpec = new ShapeFunctionParticleSpecFactory(100e-6, 0.1, 5, 0.1, Guid.NewGuid()).GetParticleSpec();
        _solutionStorage = new InMemorySolutionStorage();

        _tempDir = Path.GetTempFileName().Replace(".tmp", "");
        Directory.CreateDirectory(_tempDir);
        TestContext.WriteLine(_tempDir);

        var loggerFactory = LoggerFactory.Create(builder => { builder.AddFile(Path.Combine(_tempDir, "test.log")); });

        _solver = new Solver
        {
            LoggerFactory = loggerFactory,
            Options = new SolverOptions
            {
                InitialTimeStepWidth = 1,
                MinTimeStepWidth = 0.1,
                // MaxTimeStepWidth = 5,
                TimeStepAdaptationFactor = 1.5,
            },
            SolutionStorage = _solutionStorage
        };

        _material = new Material(
            _particleSpec.MaterialId,
            "Al2O3",
            1.65e-10,
            0,
            1e-4,
            0.9,
            1.8e3,
            101.96e-3
        );

        _materialInterface = new MaterialInterface(
            _material.Id,
            _material.Id,
            0.5,
            1.65e-10,
            0
        );

        _process = new SinteringProcess(
            0,
            endTime,
            new[] { _particleSpec },
            new[] { _material },
            new[] { _materialInterface },
            2073
        );
    }

    private IParticleSpec _particleSpec;
    private Solver _solver;
    private IMaterial _material;
    private IMaterialInterface _materialInterface;
    private ISinteringProcess _process;
    private InMemorySolutionStorage _solutionStorage;
    private string _tempDir;

    [Test]
    public void TestCreateSession()
    {
        _solver.CreateSession(_process);
        var particle = _solver.Session.Particles.Values.First();

        That(particle.Id, Is.EqualTo(_particleSpec.Id));
        That(particle.Material, Is.EqualTo(_material));
        That(particle.Nodes, Has.Count.EqualTo(100));
        That(particle, Is.TypeOf<Particle>());
    }

    [Test]
    public void TestSolution()
    {
        try
        {
            _solver.Solve(_process);
        }
        finally
        {
            PlotParticles();
            PlotDisplacements();
            PlotTimeSteps();
        }
    }

    private void PlotParticles()
    {
        var dir = Path.Combine(_tempDir, "p");
        Directory.CreateDirectory(dir);

        foreach (var (i, state) in _solutionStorage.States.Index())
        {
            var plt = new Plot();

            var coordinates = state.ParticleStates[0].Nodes
                .Append(state.ParticleStates[0].Nodes[0])
                .Select(n => new ScottPlot.Coordinates(n.AbsoluteCoordinates.X, n.AbsoluteCoordinates.Y))
                .ToArray();
            plt.Add.Scatter(coordinates);

            plt.Title($"t = {state.Time.ToString(CultureInfo.InvariantCulture)}");

            plt.SavePng(Path.Combine(dir, $"{i}.png"), 3000, 3000);
        }
    }

    private void PlotDisplacements()
    {
        var dir = Path.Combine(_tempDir, "nd");
        Directory.CreateDirectory(dir);

        foreach (var (i, step) in _solutionStorage.Steps.Index())
        {
            var plt = new Plot();

            var coordinates = step.ParticleTimeSteps[0].NodeTimeSteps.Values
                .Select((n, j) => new ScottPlot.Coordinates(j, n.NormalDisplacement))
                .ToArray();
            plt.Add.Scatter(coordinates);

            plt.Add.Line(0, 0, coordinates.Length, 0);

            plt.Title($"t = {step.StartTime.ToString(CultureInfo.InvariantCulture)} - {step.EndTime.ToString(CultureInfo.InvariantCulture)}");

            plt.SavePng(Path.Combine(dir, $"{i}.png"), 600, 400);
        }
    }

    private void PlotTimeSteps()
    {
        var plt = new Plot();

        var steps = _solutionStorage.Steps.Select(s => new ScottPlot.Coordinates(s.StartTime, s.TimeStepWidth)).ToArray();
        plt.Add.Scatter(steps);

        var meanStepWidth = steps.Select(s => s.Y).Mean();
        plt.Add.Line(0, meanStepWidth, _process.EndTime, meanStepWidth);

        plt.SavePng(Path.Combine(_tempDir, "timeSteps.png"), 600, 400);
    }
}