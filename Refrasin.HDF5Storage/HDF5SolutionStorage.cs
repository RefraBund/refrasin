using HDF5CSharp;
using HDF5CSharp.DataTypes;
using RefraSin.Coordinates.Polar;
using RefraSin.Storage;

namespace Refrasin.HDF5Storage;

public class HDF5SolutionStorage : ISolutionStorage, IDisposable
{
    public HDF5SolutionStorage(string filePath, string statesGroupName = "States", string stepsGroupName = "Steps")
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
    public void StoreState(ISolutionState state)
    {
        var stateId = Hdf5.CreateOrOpenGroup(StatesGroupId, _stateIndex.ToString());
        Hdf5.WriteAttribute(stateId, nameof(state.Time), state.Time);

        foreach (var particleState in state.ParticleStates)
        {
            Hdf5.WriteObject(stateId, new ParticleGroup(particleState), particleState.Id.ToString());
        }

        _stateIndex += 1;
    }

    /// <inheritdoc />
    public void StoreStep(ISolutionStep step)
    {
        var stepId = Hdf5.CreateOrOpenGroup(StepsGroupId, _stepIndex.ToString());
        Hdf5.WriteAttribute(stepId, nameof(step.StartTime), step.StartTime);
        Hdf5.WriteAttribute(stepId, nameof(step.EndTime), step.EndTime);
        Hdf5.WriteAttribute(stepId, nameof(step.TimeStepWidth), step.TimeStepWidth);

        foreach (var particleTimeStep in step.ParticleTimeSteps)
        {
            Hdf5.WriteObject(stepId, new ParticleTimeStepGroup(particleTimeStep), particleTimeStep.ParticleId.ToString());
        }

        _stepIndex += 1;
    }

    /// <inheritdoc />
    public void Dispose()
    {
        Hdf5.CloseFile(FileId);
    }
}