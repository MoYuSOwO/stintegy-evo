using System;
using System.Collections.Generic;
using System.Numerics;
using TheStint.Core.Cars;

namespace TheStint.Core.Racing;

public static class CarContactResolver
{
    private const float Epsilon = 1e-5f;

    public static void Resolve(IReadOnlyList<RaceCar> cars)
    {
        int iterations = 1;
        foreach (RaceCar car in cars)
            iterations = Math.Max(iterations, car.Collision.SolverIterations);

        for (int iteration = 0; iteration < iterations; iteration++)
        {
            for (int i = 0; i < cars.Count; i++)
            {
                for (int j = i + 1; j < cars.Count; j++)
                {
                    ResolvePair(cars[i], cars[j]);
                }
            }
        }
    }

    public static bool AreOverlapping(RaceCar a, RaceCar b)
    {
        return TryGetContact(a, b, out _);
    }

    public static bool TryGetContact(RaceCar a, RaceCar b, out CarContact contact)
    {
        CarBodyGeometry bodyA = CarBodyGeometry.FromState(a.State, a.Collision);
        CarBodyGeometry bodyB = CarBodyGeometry.FromState(b.State, b.Collision);

        Vector2[] axes =
        [
            bodyA.Forward,
            bodyA.Left,
            bodyB.Forward,
            bodyB.Left
        ];

        float minOverlap = float.MaxValue;
        Vector2 bestAxis = Vector2.UnitX;

        foreach (Vector2 axis in axes)
        {
            Project(bodyA, axis, out float minA, out float maxA);
            Project(bodyB, axis, out float minB, out float maxB);

            float overlap = MathF.Min(maxA, maxB) - MathF.Max(minA, minB);
            if (overlap <= 0f)
            {
                contact = default;
                return false;
            }

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = axis;
            }
        }

        Vector2 centerDelta = bodyB.Center - bodyA.Center;
        if (Vector2.Dot(centerDelta, bestAxis) < 0f)
            bestAxis = -bestAxis;

        contact = new CarContact(a, b, bestAxis, minOverlap);
        return true;
    }

    private static void ResolvePair(RaceCar a, RaceCar b)
    {
        if (!TryGetContact(a, b, out CarContact contact))
            return;

        float invMassA = InverseMass(a);
        float invMassB = InverseMass(b);
        float invMassSum = invMassA + invMassB;
        if (invMassSum <= Epsilon)
            return;

        Vector2 correction = contact.Normal * (contact.PenetrationMeters / invMassSum);
        a.State.Position -= correction * invMassA;
        b.State.Position += correction * invMassB;

        ApplyImpulse(a, b, contact.Normal, invMassA, invMassB, invMassSum);
    }

    private static void ApplyImpulse(
        RaceCar a,
        RaceCar b,
        Vector2 normal,
        float invMassA,
        float invMassB,
        float invMassSum
    )
    {
        Vector2 velocityA = a.State.Velocity;
        Vector2 velocityB = b.State.Velocity;
        Vector2 relativeVelocity = velocityB - velocityA;
        float normalSpeed = Vector2.Dot(relativeVelocity, normal);

        if (normalSpeed >= 0f)
            return;

        float restitution = MathF.Min(a.Collision.Restitution, b.Collision.Restitution);
        float impulseMagnitude = -(1f + restitution) * normalSpeed / invMassSum;
        Vector2 impulse = normal * impulseMagnitude;

        velocityA -= impulse * invMassA;
        velocityB += impulse * invMassB;

        Vector2 tangentVelocity = relativeVelocity - normal * normalSpeed;
        if (tangentVelocity.LengthSquared() > Epsilon)
        {
            Vector2 tangent = Vector2.Normalize(tangentVelocity);
            float tangentSpeed = Vector2.Dot(relativeVelocity, tangent);
            float tangentImpulseMagnitude = -tangentSpeed / invMassSum;
            float friction = MathF.Min(a.Collision.Friction, b.Collision.Friction);
            float maxFrictionImpulse = impulseMagnitude * friction;
            tangentImpulseMagnitude = Math.Clamp(
                tangentImpulseMagnitude,
                -maxFrictionImpulse,
                maxFrictionImpulse
            );
            Vector2 tangentImpulse = tangent * tangentImpulseMagnitude;
            velocityA -= tangentImpulse * invMassA;
            velocityB += tangentImpulse * invMassB;
        }

        ApplyVelocity(a.State, velocityA);
        ApplyVelocity(b.State, velocityB);
    }

    private static void Project(CarBodyGeometry body, Vector2 axis, out float min, out float max)
    {
        Vector2[] corners = body.GetCorners();
        min = Vector2.Dot(corners[0], axis);
        max = min;

        for (int i = 1; i < corners.Length; i++)
        {
            float projected = Vector2.Dot(corners[i], axis);
            min = MathF.Min(min, projected);
            max = MathF.Max(max, projected);
        }
    }

    private static float InverseMass(RaceCar car)
    {
        return 1f / Math.Max(car.CarConfig.MassKg, Epsilon);
    }

    private static void ApplyVelocity(CarState state, Vector2 velocity)
    {
        float speed = velocity.Length();
        state.Speed = speed;
        if (speed > 0.05f)
        {
            state.Heading = MathF.Atan2(velocity.Y, velocity.X);
            state.SideslipAngleRadians = 0f;
            state.YawRateRadiansPerSecond = 0f;
        }
    }
}

public readonly record struct CarContact(
    RaceCar A,
    RaceCar B,
    Vector2 Normal,
    float PenetrationMeters
);
