using HDF5CSharp;
using HDF5CSharp.DataTypes;
using RefraSin.Coordinates.Polar;
using RefraSin.ProcessModel;
using RefraSin.ProcessModel.Sintering;
using RefraSin.Storage;

namespace Refrasin.HDF5Storage;

public class Hdf5SolutionStorage : ISolutionStorage, IDisposable
{
    public Hdf5SolutionStorage(
        string filePath,
        string statesGroupName = "States",
        string stepsGroupName = "Steps"
    )
    {
        FilePath = filePath;
        StatesGroupName = statesGroupName;
        StepsGroupName = stepsGroupName;
        FileId = Hdf5.CreateFile(FilePath);

        StatesGroupId = Hdf5.CreateOrOpenGroup(FileId, statesGroupName);
        StepsGroupId = Hdf5.CreateOrOpenGroup(FileId, stepsGroupName);
    }

    public string FilePath { get; }
    public string StatesGroupName { get; }
    public string StepsGroupName { get; }

    public long FileId { get; }

    public long StatesGroupId { get; }

    public long StepsGroupId { get; }

    private int _stateIndex = 0;
    private int _stepIndex = 0;

    /// <inheritdoc />
    public void StoreState(IProcessStep processStep, ISystemState state)
    {
        var stateId = Hdf5.CreateOrOpenGroup(StatesGroupId, _stateIndex.ToString());
        Hdf5.WriteAttribute(stateId, nameof(state.Time), state.Time);

        Hdf5.WriteCompounds(
            stateId,
            "Particles",
            state.Particles.Select(p => new ParticleCompound(p)).ToArray(),
            new Dictionary<string, List<string>>()
        );

        var nodesGroupId = Hdf5.CreateOrOpenGroup(stateId, "Nodes");

        foreach (var particleState in state.Particles)
        {
            Hdf5.WriteCompounds(
                nodesGroupId,
                particleState.Id.ToString(),
                particleState.Nodes.Select(n => new NodeCompound(n)).ToArray(),
                new Dictionary<string, List<string>>()
            );
        }

        _stateIndex += 1;
    }

    /// <inheritdoc />
    public void StoreStateTransition(IProcessStep processStep, ISystemStateTransition transition)
    {
        var stepId = Hdf5.CreateOrOpenGroup(StepsGroupId, _stepIndex.ToString());
        Hdf5.WriteAttribute(stepId, "InputState", transition.InputStateId);
        Hdf5.WriteAttribute(stepId, "OutputState", transition.OutputStateId);
        if (transition is ISinteringStateStateTransition sinteringStateChange)
        {
            Hdf5.WriteAttribute(
                stepId,
                nameof(sinteringStateChange.TimeStepWidth),
                sinteringStateChange.TimeStepWidth
            );

            Hdf5.WriteCompounds(
                stepId,
                "Particles",
                sinteringStateChange
                    .ParticleTimeSteps.Select(p => new ParticleTimeStepCompound(p))
                    .ToArray(),
                new Dictionary<string, List<string>>()
            );

            var nodesGroupId = Hdf5.CreateOrOpenGroup(stepId, "Nodes");

            foreach (var particleTimeStep in sinteringStateChange.ParticleTimeSteps)
            {
                Hdf5.WriteCompounds(
                    nodesGroupId,
                    particleTimeStep.ParticleId.ToString(),
                    particleTimeStep
                        .NodeTimeSteps.Values.Select(n => new NodeTimeStepCompound(n))
                        .ToArray(),
                    new Dictionary<string, List<string>>()
                );
            }
        }

        _stepIndex += 1;
    }

    /// <inheritdoc />
    public void Dispose()
    {
        Hdf5.CloseFile(FileId);
    }
}
