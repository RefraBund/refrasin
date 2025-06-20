using RefraSin.Coordinates;
using RefraSin.Coordinates.Helpers;
using static System.Math;
using static RefraSin.Coordinates.Constants;

namespace RefraSin.ParticleModel.Nodes.Extensions;

public static class NodeGeometryExtensions
{
    public static ToUpperToLower<double> SurfaceDistance<TNode>(this TNode self)
        where TNode : INode, INodeGeometry, INodeNeighbors =>
        new(
            CosLaw.C(self.Upper.Coordinates.R, self.Coordinates.R, self.AngleDistance.ToUpper),
            CosLaw.C(self.Lower.Coordinates.R, self.Coordinates.R, self.AngleDistance.ToLower)
        );

    public static ToUpperToLower<Angle> AngleDistance<TNode>(this TNode self)
        where TNode : INode, INodeGeometry, INodeNeighbors =>
        new(
            self.Coordinates.AngleTo(self.Upper.Coordinates, true),
            self.Lower.Coordinates.AngleTo(self.Coordinates, true)
        );

    public static ToUpperToLower<Angle> SurfaceRadiusAngle<TNode>(this TNode self)
        where TNode : INode, INodeGeometry, INodeNeighbors
    {
        var alphau = CosLaw.Gamma(self.SurfaceDistance.ToUpper, self.Coordinates.R, self.Upper.Coordinates.R);
        if (self.AngleDistance.ToUpper < 0)
            alphau = Angle.Full - alphau;

        var alphal = CosLaw.Gamma(self.SurfaceDistance.ToLower, self.Coordinates.R, self.Lower.Coordinates.R);
        if (self.AngleDistance.ToLower < 0)
            alphal = Angle.Full - alphal;

        return new ToUpperToLower<Angle>(alphau, alphal);
    }

    public static ToUpperToLower<double> Volume<TNode>(this TNode self)
        where TNode : INode, INodeGeometry, INodeNeighbors =>
        new(
            0.5 * self.Coordinates.R * self.Upper.Coordinates.R * Sin(self.AngleDistance.ToUpper),
            0.5 * self.Coordinates.R * self.Lower.Coordinates.R * Sin(self.AngleDistance.ToLower)
        );

    public static ToUpperToLower<Angle> SurfaceNormalAngle<TNode>(this TNode self)
        where TNode : INode, INodeGeometry, INodeNeighbors
    {
        if (self.Type == NodeType.Neck) // neck normal is meant normal to existing grain boundary
        {
            return self.Upper.Type == NodeType.GrainBoundary
                ? new ToUpperToLower<Angle>(
                    HalfOfPi,
                    ThreeHalfsOfPi
                  - self.SurfaceRadiusAngle.ToUpper
                  - self.SurfaceRadiusAngle.ToLower
                )
                : new ToUpperToLower<Angle>(
                    ThreeHalfsOfPi
                  - self.SurfaceRadiusAngle.ToUpper
                  - self.SurfaceRadiusAngle.ToLower,
                    HalfOfPi
                );
        }

        var angle = PI - 0.5 * (self.SurfaceRadiusAngle.ToUpper + self.SurfaceRadiusAngle.ToLower);
        return new ToUpperToLower<Angle>(angle, angle);
    }

    public static ToUpperToLower<Angle> SurfaceTangentAngle<TNode>(this TNode self)
        where TNode : INode, INodeGeometry, INodeNeighbors
    {
        if (self.Type is NodeType.Neck) // neck tangent is meant tangential to existing grain boundary
        {
            return self.Upper.Type == NodeType.GrainBoundary
                ? new ToUpperToLower<Angle>(
                    0,
                    Pi - self.SurfaceRadiusAngle.ToUpper - self.SurfaceRadiusAngle.ToLower
                )
                : new ToUpperToLower<Angle>(
                    Pi - self.SurfaceRadiusAngle.ToUpper - self.SurfaceRadiusAngle.ToLower,
                    0
                );
        }

        var angle =
            PI / 2 - 0.5 * (self.SurfaceRadiusAngle.ToUpper + self.SurfaceRadiusAngle.ToLower);
        return new ToUpperToLower<Angle>(angle, angle);
    }

    public static ToUpperToLower<Angle> RadiusNormalAngle<TNode>(this TNode self)
        where TNode : INode, INodeGeometry, INodeNeighbors =>
        new(
            self.SurfaceRadiusAngle.ToUpper + self.SurfaceNormalAngle.ToUpper,
            self.SurfaceRadiusAngle.ToLower + self.SurfaceNormalAngle.ToLower
        );

    public static ToUpperToLower<Angle> RadiusTangentAngle<TNode>(this TNode self)
        where TNode : INode, INodeGeometry, INodeNeighbors =>
        new(
            self.SurfaceRadiusAngle.ToUpper + self.SurfaceTangentAngle.ToUpper,
            self.SurfaceRadiusAngle.ToLower + self.SurfaceTangentAngle.ToLower
        );
}