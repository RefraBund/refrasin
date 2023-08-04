using System.Threading;
using MathNet.Numerics;
using Microsoft.Extensions.Logging;
using NUnit.Framework;
using RefraSin.Coordinates.Polar;
using RefraSin.Core;
using RefraSin.Core.Materials;
using RefraSin.Core.ParticleSources;
using RefraSin.Core.ParticleTreeSources;
using RefraSin.Core.SinteringProcesses;

namespace Tests;

public class TwoParticleTest
{
    [SetUp]
    public void Init()
    {
        Configuration.UseLoggerFactory(LoggerFactory.Create(builder => { builder.AddNUnit(); }));
    }

    [Test]
    public void RunTwoParticleTest()
    {
        var material = new Material(
            "",
            8e-6,
            1,
            1e-9,
            1e-4
        );

        var materialInterfaces = new MaterialInterfaceCollection()
        {
            new(material, material, 0.7, 0.8e-9)
        };

        var particleSource = new TrigonometricParticleSource(
            material,
            0.5,
            0.2,
            0.2,
            5
        );

        var treeSource = new ExplicitTreeSource(
            new ExplicitTreeItem(new PolarPoint(), particleSource, 0, new[]
            {
                new ExplicitTreeItem(new PolarPoint(0, 1.5), particleSource, Constants.Pi)
            })
        );

        var process = new SinteringProcess(
            treeSource,
            0,
            1e4,
            materialInterfaces,
            1273
        );

        var solution = process.Solve(CancellationToken.None);

        Assert.IsTrue(solution.Succeeded, "solution.Succeeded");
    }
}