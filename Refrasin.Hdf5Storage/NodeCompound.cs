using System.Runtime.InteropServices;
using RefraSin.ParticleModel;

namespace Refrasin.HDF5Storage;

internal struct NodeCompound
{
    [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 37)]
    public readonly string Id;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] Coordinates;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] AbsoluteCoordinates = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] SurfaceDistance = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] SurfaceRadiusAngle = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] AngleDistance = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] Volume = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] SurfaceAngle = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] SurfaceEnergy = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] SurfaceDiffusionCoefficient = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] GibbsEnergyGradient = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
    public readonly double[] VolumeGradient = { 0.0, 0.0 };

    [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 37)]
    public readonly string ContactedParticleId = string.Empty;

    [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 37)]
    public readonly string ContactedNodeId = string.Empty;

    public NodeCompound(INode node)
    {
        Id = node.Id.ToString();
        Coordinates = node.Coordinates.ToArray();

        if (node is INodeGeometry nodeGeometry)
        {
            AbsoluteCoordinates = nodeGeometry.AbsoluteCoordinates.ToArray();
            SurfaceDistance = nodeGeometry.SurfaceDistance.ToArray();
            SurfaceRadiusAngle = nodeGeometry.SurfaceRadiusAngle.ToDoubleArray();
            AngleDistance = nodeGeometry.SurfaceRadiusAngle.ToDoubleArray();
            Volume = nodeGeometry.Volume.ToArray();
            SurfaceAngle = nodeGeometry.SurfaceRadiusAngle.ToDoubleArray();
        }

        if (node is INodeMaterialProperties nodeMaterialProperties)
        {
            SurfaceEnergy = nodeMaterialProperties.SurfaceEnergy.ToArray();
            SurfaceDiffusionCoefficient = nodeMaterialProperties.SurfaceDiffusionCoefficient.ToArray();
        }

        if (node is INodeGradients nodeGradients)
        {
            GibbsEnergyGradient = nodeGradients.GibbsEnergyGradient.ToArray();
            VolumeGradient = nodeGradients.VolumeGradient.ToArray();
        }

        if (node is INodeContact contactNode)
        {
            ContactedParticleId = contactNode.ContactedParticleId.ToString();
            ContactedNodeId = contactNode.ContactedNodeId.ToString();
        }
    }
}