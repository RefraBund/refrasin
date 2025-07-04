using Microsoft.Extensions.Logging;
using Microsoft.FSharp.Core;
using Plotly.NET;
using RefraSin.MaterialData;
using RefraSin.ParticleModel.Nodes;
using RefraSin.ParticleModel.Particles;
using RefraSin.Plotting;
using RefraSin.ProcessModel;
using RefraSin.ProcessModel.Sintering;
using RefraSin.Storage;
using Serilog;
using Serilog.Events;
using Serilog.Sinks.TestCorrelator;

namespace RefraSin.TEPSolver.Test;

[TestFixtureSource(nameof(GetTestFixtureData))]
public class SimulationTest
{
    [SetUp]
    public void SetUpLogging()
    {
        Log.Logger = new LoggerConfiguration()
            .MinimumLevel.Information()
            // .WriteTo.TestCorrelator()
            .WriteTo.Console()
            .WriteTo.File(Path.Combine(_tempDir, "test.log"))
            .CreateLogger();
    }

    [TearDown]
    public void TearDownLogging()
    {
        Log.CloseAndFlush();
        Log.Logger = Serilog.Core.Logger.None;
    }

    public SimulationTest(ISystemState<IParticle<IParticleNode>, IParticleNode> initialState)
    {
        _initialState = initialState;

        var solver = new SinteringSolver(SolverRoutines.Default, 20);
        var plotHandler = new PlotEventHandler(_tempDir);
        solver.SessionInitialized += plotHandler.Handle;

        _sinteringProcess = new SinteringStep(Conditions, solver, [Material]);
        _sinteringProcess.UseStorage(_solutionStorage);
        _sinteringProcess.UseStorage(
            new ParquetStorage.ParquetStorage(Path.Combine(_tempDir, "results.parquet"))
        );
    }

    public static IEnumerable<TestFixtureData> GetTestFixtureData() =>
        InitialStates.Generate().Select(s => new TestFixtureData(s.state) { TestName = s.label });

    private static readonly ISinteringConditions Conditions = new SinteringConditions(2073, 3.6e2);
    private readonly ISystemState<IParticle<IParticleNode>, IParticleNode> _initialState;
    private readonly SinteringStep _sinteringProcess;
    private readonly InMemorySolutionStorage _solutionStorage = new();

    private static readonly IParticleMaterial Material = new ParticleMaterial(
        InitialStates.MaterialId,
        "Al2O3",
        SubstanceProperties.FromDensityAndMolarMass(1.8e3, 101.96e-3),
        new InterfaceProperties(1.65e-14, 0.9),
        new Dictionary<Guid, IInterfaceProperties>
        {
            { InitialStates.MaterialId, new InterfaceProperties(1.65e-14, 0.5) },
        }
    );

    private readonly string _tempDir = TempPath.CreateTempDir();

    [Test]
    public void TestSolution()
    {
        Exception? exception = null;

        try
        {
            using (TestCorrelator.CreateContext())
            {
                _sinteringProcess.Solve(_initialState);

                Assert.That(
                    TestCorrelator.GetLogEventsFromCurrentContext(),
                    Has.None.Matches<LogEvent>(e => e.Exception is not null)
                );
            }
        }
        catch (Exception e)
        {
            exception = e;
        }
        finally
        {
            PlotShrinkage();
            PlotNeckWidths();
            PlotTimeSteps();
            PlotParticleCenter();
        }

        if (exception is not null)
        {
            System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(exception).Throw();
        }
    }

    private void PlotTimeSteps()
    {
        if (_solutionStorage.States.Count == 0)
            return;

        var plot = ProcessPlot.PlotTimeSteps(_solutionStorage.States);
        plot.SaveHtml(Path.Combine(_tempDir, "timeSteps.html"));
    }

    private void PlotParticleCenter()
    {
        if (_solutionStorage.States.Count == 0)
            return;

        var centers = ProcessPlot.PlotParticleCenters(_solutionStorage.States);
        var start = ParticlePlot.PlotParticles<IParticle<IParticleNode>, IParticleNode>(
            _solutionStorage.States[0].Particles
        );
        var end = ParticlePlot.PlotParticles<IParticle<IParticleNode>, IParticleNode>(
            _solutionStorage.States[^1].Particles
        );
        var plot = Chart.Combine([centers, start, end]);
        plot.SaveHtml(Path.Combine(_tempDir, "particleCenters.html"));
    }

    private void PlotShrinkage()
    {
        if (_solutionStorage.States.Count == 0)
            return;

        var plot = ProcessPlot.PlotShrinkagesByDistance(_solutionStorage.States);
        plot.SaveHtml(Path.Combine(_tempDir, "shrinkages.html"));
    }

    private void PlotNeckWidths()
    {
        if (_solutionStorage.States.Count == 0)
            return;

        var plot = ProcessPlot.PlotNeckWidths(_solutionStorage.States);
        plot.SaveHtml(Path.Combine(_tempDir, "necks.html"));
    }

    class PlotEventHandler(string dir)
    {
        private int _counter;

        public void Handle(object? sender, SinteringSolver.SessionInitializedEventArgs e)
        {
            var plot = ParticlePlot.PlotParticles<IParticle<IParticleNode>, IParticleNode>(
                e.SolverSession.CurrentState.Particles
            );
            plot.SaveHtml(Path.Combine(dir, $"session_{_counter}.html"));
            _counter++;
        }
    }
}
