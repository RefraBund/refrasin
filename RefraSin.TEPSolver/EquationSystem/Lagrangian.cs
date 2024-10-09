using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using RefraSin.Coordinates;
using RefraSin.TEPSolver.ParticleModel;
using RefraSin.TEPSolver.StepVectors;
using static RefraSin.TEPSolver.EquationSystem.Helper;
using Particle = RefraSin.TEPSolver.ParticleModel.Particle;
using ParticleContact = RefraSin.TEPSolver.ParticleModel.ParticleContact;

namespace RefraSin.TEPSolver.EquationSystem;

public static class Lagrangian
{
    public static Vector<double> EvaluateAt(SolutionState currentState, StepVector stepVector)
    {
        var evaluation = YieldEquations(currentState, stepVector).ToArray();

        if (evaluation.Any(x => !double.IsFinite(x)))
        {
            throw new InvalidOperationException(
                "One ore more components of the gradient evaluated to an infinite value."
            );
        }

        return new DenseVector(evaluation);
    }

    public static IEnumerable<double> YieldEquations(
        SolutionState currentState,
        StepVector stepVector
    ) =>
        Join(
            currentState.Particles.SelectMany(p => YieldParticleBlockEquations(p, stepVector)),
            YieldBorderBlockEquations(currentState, stepVector)
        );

    public static IEnumerable<double> YieldParticleBlockEquations(
        Particle particle,
        StepVector stepVector
    ) =>
        Join(
            YieldNodeEquations(particle.Nodes, stepVector),
            [
                ParticleHorizontalForceBalance(stepVector, particle),
                ParticleVerticalForceBalance(stepVector, particle),
                ParticleTorqueBalance(stepVector, particle)
            ]
        );

    public static IEnumerable<double> YieldBorderBlockEquations(
        SolutionState currentState,
        StepVector stepVector
    ) =>
        YieldLinearBorderBlockEquations(currentState, stepVector)
            .Append(DissipationEquality(currentState, stepVector));

    public static IEnumerable<double> YieldLinearBorderBlockEquations(
        SolutionState currentState,
        StepVector stepVector
    ) => YieldContactsEquations(currentState.ParticleContacts, stepVector);

    public static IEnumerable<double> YieldNodeEquations(
        IEnumerable<NodeBase> nodes,
        StepVector stepVector
    )
    {
        foreach (var node in nodes)
        {
            if (node is not ContactNodeBase)
            {
                yield return StateVelocityDerivativeNormal(stepVector, node);
                yield return FluxDerivative(stepVector, node);
                yield return RequiredConstraint(stepVector, node);
                yield return NormalStressDerivative(stepVector, node);
                yield return TangentialStressDerivative(stepVector, node);
                yield return SurfaceNodeNormalStressConstraint(stepVector, node);
                yield return SurfaceNodeTangentialStressConstraint(stepVector, node);
            }
        }
    }

    public static IEnumerable<double> YieldContactsEquations(
        IEnumerable<ParticleContact> contacts,
        StepVector stepVector
    ) =>
        contacts.SelectMany(contact =>
            Join(
                YieldContactAuxiliaryDerivatives(stepVector, contact),
                YieldContactNodesEquations(stepVector, contact)
            )
        );

    public static IEnumerable<double> YieldContactNodesEquations(
        StepVector stepVector,
        ParticleContact contact
    )
    {
        foreach (var contactNode in contact.FromNodes)
        {
            yield return ContactConstraintDistance(stepVector, contact, contactNode);
            yield return ContactConstraintDirection(stepVector, contact, contactNode);

            yield return StateVelocityDerivativeNormal(stepVector, contactNode);
            yield return StateVelocityDerivativeTangential(stepVector, contactNode);
            yield return FluxDerivative(stepVector, contactNode);
            yield return RequiredConstraint(stepVector, contactNode);
            yield return NormalStressDerivative(stepVector, contactNode);
            yield return TangentialStressDerivative(stepVector, contactNode);
            yield return ContactNodeNormalStressConstraint(stepVector, contactNode);
            yield return ContactNodeTangentialStressConstraint(stepVector, contactNode);

            yield return StateVelocityDerivativeNormal(stepVector, contactNode.ContactedNode);
            yield return StateVelocityDerivativeTangential(stepVector, contactNode.ContactedNode);
            yield return FluxDerivative(stepVector, contactNode.ContactedNode);
            yield return RequiredConstraint(stepVector, contactNode.ContactedNode);
            yield return NormalStressDerivative(stepVector, contactNode.ContactedNode);
            yield return TangentialStressDerivative(stepVector, contactNode.ContactedNode);
        }
    }

    public static IEnumerable<double> YieldContactAuxiliaryDerivatives(
        StepVector stepVector,
        ParticleContact contact
    )
    {
        yield return ParticleRadialDisplacementDerivative(stepVector, contact);
        yield return ParticleAngleDisplacementDerivative(stepVector, contact);
        yield return ParticleRotationDerivative(stepVector, contact);
    }

    public static double ParticleRadialDisplacementDerivative(
        StepVector stepVector,
        ParticleContact contact
    ) => contact.FromNodes.Sum(stepVector.LambdaContactDistance);

    public static double ParticleAngleDisplacementDerivative(
        StepVector stepVector,
        ParticleContact contact
    ) => contact.FromNodes.Sum(stepVector.LambdaContactDirection);

    private static double ParticleRotationDerivative(
        StepVector stepVector,
        ParticleContact contact
    ) =>
        contact.FromNodes.Sum(n =>
            n.ContactedNode.Coordinates.R
                * Sin(n.ContactedNode.AngleDistanceToContactDirection)
                * stepVector.LambdaContactDistance(n)
            - n.ContactedNode.Coordinates.R
                / contact.Distance
                * Cos(n.ContactedNode.AngleDistanceToContactDirection)
                * stepVector.LambdaContactDirection(n)
        );

    public static double StateVelocityDerivativeNormal(StepVector stepVector, NodeBase node)
    {
        var gibbsTerm =
            (
                node.GibbsEnergyGradient.Normal
                + 0.5 * node.SurfaceDistance.Sum * stepVector.NormalStress(node)
            ) * (1 + stepVector.LambdaDissipation());
        var requiredConstraintsTerm = node.VolumeGradient.Normal * stepVector.LambdaVolume(node);

        double contactTerm = 0;

        if (node is ContactNodeBase contactNode)
        {
            contactTerm =
                -contactNode.ContactDistanceGradient.Normal
                    * stepVector.LambdaContactDistance(contactNode)
                - contactNode.ContactDirectionGradient.Normal
                    * stepVector.LambdaContactDirection(contactNode);
        }

        return -gibbsTerm + requiredConstraintsTerm + contactTerm;
    }

    public static double StateVelocityDerivativeTangential(
        StepVector stepVector,
        ContactNodeBase node
    )
    {
        var gibbsTerm =
            (
                node.GibbsEnergyGradient.Tangential
                + 0.5 * node.SurfaceDistance.Sum * stepVector.TangentialStress(node)
            ) * (1 + stepVector.LambdaDissipation());
        var requiredConstraintsTerm =
            node.VolumeGradient.Tangential * stepVector.LambdaVolume(node);
        var contactTerm =
            -node.ContactDistanceGradient.Tangential * stepVector.LambdaContactDistance(node)
            - node.ContactDirectionGradient.Tangential * stepVector.LambdaContactDirection(node);

        return -gibbsTerm + requiredConstraintsTerm + contactTerm;
    }

    public static double FluxDerivative(StepVector stepVector, NodeBase node)
    {
        var dissipationTerm =
            2
            * node.Particle.VacancyVolumeEnergy
            * node.SurfaceDistance.ToUpper
            / node.InterfaceDiffusionCoefficient.ToUpper
            * stepVector.FluxToUpper(node)
            * stepVector.LambdaDissipation();
        var thisRequiredConstraintsTerm = stepVector.LambdaVolume(node);
        var upperRequiredConstraintsTerm = stepVector.LambdaVolume(node.Upper);

        return -dissipationTerm - thisRequiredConstraintsTerm + upperRequiredConstraintsTerm;
    }

    public static double RequiredConstraint(StepVector stepVector, NodeBase node)
    {
        var normalVolumeTerm = node.VolumeGradient.Normal * stepVector.NormalDisplacement(node);
        var tangentialVolumeTerm = 0.0;

        if (node is ContactNodeBase contactNode)
        {
            tangentialVolumeTerm =
                node.VolumeGradient.Tangential * stepVector.TangentialDisplacement(contactNode);
        }

        var fluxTerm = stepVector.FluxToUpper(node) - stepVector.FluxToUpper(node.Lower);

        return normalVolumeTerm + tangentialVolumeTerm - fluxTerm;
    }

    public static double NormalStressDerivative(StepVector stepVector, NodeBase node)
    {
        var gibbsTerm =
            0.5
            * node.SurfaceDistance.Sum
            * stepVector.NormalDisplacement(node)
            * (1 + stepVector.LambdaDissipation());
        var constraintsTerm =
            node.Type == NodeType.Surface ? stepVector.LambdaNormalStress(node) : 0;
        var horizontalTerm =
            Cos(node.Coordinates.Phi + (Angle.Half - node.RadiusNormalAngle.ToUpper))
            * stepVector.LambdaHorizontalForceBalance(node.Particle);
        var verticalTerm =
            Sin(node.Coordinates.Phi + (Angle.Half - node.RadiusNormalAngle.ToUpper))
            * stepVector.LambdaVerticalForceBalance(node.Particle);
        var torqueTerm =
            stepVector.LambdaTorqueBalance(node.Particle)
            * Sin(node.RadiusNormalAngle.ToUpper)
            * node.Coordinates.R;
        var contactTerm = node is ContactNodeBase contactNode
            ? stepVector.LambdaNormalStress(contactNode) * (contactNode.IsParentsNode ? 1 : -1)
            : 0;

        return -gibbsTerm
            + constraintsTerm
            + horizontalTerm
            + verticalTerm
            + torqueTerm
            + contactTerm;
    }

    public static double TangentialStressDerivative(StepVector stepVector, NodeBase node)
    {
        var gibbsTerm =
            0.5
            * node.SurfaceDistance.Sum
            * stepVector.TangentialDisplacement(node)
            * (1 + stepVector.LambdaDissipation());
        var constraintsTerm =
            node.Type == NodeType.Surface ? stepVector.LambdaTangentialStress(node) : 0;
        var horizontalTerm =
            Cos(node.Coordinates.Phi + (Angle.Half - node.RadiusTangentAngle.ToUpper))
            * stepVector.LambdaHorizontalForceBalance(node.Particle);
        var verticalTerm =
            Sin(node.Coordinates.Phi + (Angle.Half - node.RadiusTangentAngle.ToUpper))
            * stepVector.LambdaVerticalForceBalance(node.Particle);
        var torqueTerm =
            stepVector.LambdaTorqueBalance(node.Particle)
            * Sin(node.RadiusTangentAngle.ToUpper)
            * node.Coordinates.R;
        var contactTerm = node is ContactNodeBase contactNode
            ? stepVector.LambdaTangentialStress(contactNode) * (contactNode.IsParentsNode ? 1 : -1)
            : 0;

        return -gibbsTerm
            + constraintsTerm
            + horizontalTerm
            + verticalTerm
            + torqueTerm
            + contactTerm;
    }

    public static double SurfaceNodeNormalStressConstraint(StepVector stepVector, NodeBase node) =>
        stepVector.NormalStress(node);

    public static double SurfaceNodeTangentialStressConstraint(
        StepVector stepVector,
        NodeBase node
    ) => stepVector.TangentialStress(node);

    public static double DissipationEquality(SolutionState currentState, StepVector stepVector)
    {
        var dissipationNormal = currentState
            .Nodes.Select(n =>
                -(
                    n.GibbsEnergyGradient.Normal
                    + 0.5 * n.SurfaceDistance.Sum * stepVector.NormalStress(n)
                ) * stepVector.NormalDisplacement(n)
            )
            .Sum();

        var dissipationTangential = currentState
            .Nodes.OfType<ContactNodeBase>()
            .Select(n =>
                -(
                    n.GibbsEnergyGradient.Tangential
                    + 0.5 * n.SurfaceDistance.Sum * stepVector.TangentialStress(n)
                ) * stepVector.TangentialDisplacement(n)
            )
            .Sum();

        var dissipationFunction = currentState
            .Nodes.Select(n =>
                n.Particle.VacancyVolumeEnergy
                * n.SurfaceDistance.ToUpper
                * Pow(stepVector.FluxToUpper(n), 2)
                / n.InterfaceDiffusionCoefficient.ToUpper
            )
            .Sum();

        return dissipationNormal + dissipationTangential - dissipationFunction;
    }

    public static double ContactConstraintDistance(
        StepVector stepVector,
        ParticleContact contact,
        ContactNodeBase node
    ) =>
        stepVector.RadialDisplacement(contact)
        - node.ContactDistanceGradient.Normal * stepVector.NormalDisplacement(node)
        - node.ContactedNode.ContactDistanceGradient.Normal
            * stepVector.NormalDisplacement(node.ContactedNode)
        - node.ContactDistanceGradient.Tangential * stepVector.TangentialDisplacement(node)
        - node.ContactedNode.ContactDistanceGradient.Tangential
            * stepVector.TangentialDisplacement(node.ContactedNode)
        + node.ContactedNode.Coordinates.R
            * Sin(node.ContactedNode.AngleDistanceToContactDirection)
            * stepVector.RotationDisplacement(contact);

    public static double ContactConstraintDirection(
        StepVector stepVector,
        ParticleContact contact,
        ContactNodeBase node
    ) =>
        stepVector.AngleDisplacement(contact)
        - node.ContactDirectionGradient.Normal * stepVector.NormalDisplacement(node)
        - node.ContactedNode.ContactDirectionGradient.Normal
            * stepVector.NormalDisplacement(node.ContactedNode)
        - node.ContactDirectionGradient.Tangential * stepVector.TangentialDisplacement(node)
        - node.ContactedNode.ContactDirectionGradient.Tangential
            * stepVector.TangentialDisplacement(node.ContactedNode)
        - node.ContactedNode.Coordinates.R
            / contact.Distance
            * Cos(node.ContactedNode.AngleDistanceToContactDirection)
            * stepVector.RotationDisplacement(contact);

    public static double ContactNodeNormalStressConstraint(
        StepVector stepVector,
        ContactNodeBase node
    ) => stepVector.NormalStress(node) - stepVector.NormalStress(node.ContactedNode);

    public static double ContactNodeTangentialStressConstraint(
        StepVector stepVector,
        ContactNodeBase node
    ) => stepVector.TangentialStress(node) - stepVector.TangentialStress(node.ContactedNode);

    public static double ParticleHorizontalForceBalance(StepVector stepVector, Particle particle) =>
        particle
            .Nodes.Select(n =>
                Cos(n.Coordinates.Phi + (Angle.Half - n.RadiusNormalAngle.ToUpper))
                    * stepVector.NormalStress(n)
                + Cos(n.Coordinates.Phi + (Angle.Half - n.RadiusTangentAngle.ToUpper))
                    * stepVector.TangentialStress(n)
            )
            .Sum();

    public static double ParticleVerticalForceBalance(StepVector stepVector, Particle particle) =>
        particle
            .Nodes.Select(n =>
                Sin(n.Coordinates.Phi + (Angle.Half - n.RadiusNormalAngle.ToUpper))
                    * stepVector.NormalStress(n)
                + Sin(n.Coordinates.Phi + (Angle.Half - n.RadiusTangentAngle.ToUpper))
                    * stepVector.TangentialStress(n)
            )
            .Sum();

    public static double ParticleTorqueBalance(StepVector stepVector, Particle particle) =>
        particle
            .Nodes.Select(n =>
                stepVector.NormalStress(n) * Sin(n.RadiusNormalAngle.ToUpper) * n.Coordinates.R
                + stepVector.TangentialStress(n)
                    * Sin(n.RadiusTangentAngle.ToUpper)
                    * n.Coordinates.R
            )
            .Sum();
}
