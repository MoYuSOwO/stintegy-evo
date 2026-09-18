using System;
using System.Collections.Generic;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Racing;

public static class CarContactResolver
{
    private const float Epsilon = 1e-5f;

    /// <summary>
    /// How far past touching the cars are pushed apart. Correcting by the
    /// penetration exactly leaves them touching, and on a flank that is not
    /// axis-aligned float rounding then reads that as a fresh overlap of a
    /// few micrometres; a millimetre of daylight settles it.
    /// </summary>
    private const float SeparationSlopMeters = 1e-3f;

    public static void Resolve(IReadOnlyList<RaceCar> cars)
    {
        ResolveUntilSeparated(cars);
    }

    internal static bool ResolveUntilSeparated(IReadOnlyList<RaceCar> cars)
    {
        int iterations = 1;
        for (int i = 0; i < cars.Count; i++)
        {
            iterations = Math.Max(
                iterations,
                cars[i].Collision.SolverIterations
            );
        }

        bool resolvedAny = false;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            bool resolvedThisPass = ResolveSinglePass(cars);
            resolvedAny |= resolvedThisPass;
            if (!resolvedThisPass)
                break;
        }
        return resolvedAny;
    }

    private static bool ResolveSinglePass(IReadOnlyList<RaceCar> cars)
    {
        bool resolvedAny = false;
        for (int i = 0; i < cars.Count; i++)
        {
            for (int j = i + 1; j < cars.Count; j++)
            {
                resolvedAny |= ResolvePair(cars[i], cars[j]);
            }
        }
        return resolvedAny;
    }

    public static bool AreOverlapping(RaceCar a, RaceCar b)
    {
        return TryGetContact(a, b, out _);
    }

    public static bool TryGetContact(RaceCar a, RaceCar b, out CarContact contact)
    {
        CarBodyGeometry bodyA = CarBodyGeometry.FromState(a.State, a.Collision);
        CarBodyGeometry bodyB = CarBodyGeometry.FromState(b.State, b.Collision);

        float minOverlap = float.MaxValue;
        Vector2 bestAxis = Vector2.UnitX;
        bool referenceIsA = true;
        if (!TryUpdateMinimumOverlap(
                in bodyA,
                in bodyB,
                bodyA.Forward,
                true,
                ref minOverlap,
                ref bestAxis,
                ref referenceIsA
            ) ||
            !TryUpdateMinimumOverlap(
                in bodyA,
                in bodyB,
                bodyA.Left,
                true,
                ref minOverlap,
                ref bestAxis,
                ref referenceIsA
            ) ||
            !TryUpdateMinimumOverlap(
                in bodyA,
                in bodyB,
                bodyB.Forward,
                false,
                ref minOverlap,
                ref bestAxis,
                ref referenceIsA
            ) ||
            !TryUpdateMinimumOverlap(
                in bodyA,
                in bodyB,
                bodyB.Left,
                false,
                ref minOverlap,
                ref bestAxis,
                ref referenceIsA
            ))
        {
            contact = default;
            return false;
        }

        Vector2 centerDelta = bodyB.Center - bodyA.Center;
        if (Vector2.Dot(centerDelta, bestAxis) < 0f)
            bestAxis = -bestAxis;

        Vector2 point = referenceIsA
            ? ClipContactPoint(in bodyA, in bodyB, bestAxis)
            : ClipContactPoint(in bodyB, in bodyA, -bestAxis);
        contact = new CarContact(a, b, bestAxis, minOverlap, point);
        return true;
    }

    /// <summary>
    /// Where two overlapping boxes touch, by clipping: the face of the
    /// reference box that the separating axis came from, and the edge of
    /// the other box that faces it most squarely. The incident edge is cut
    /// to the reference face's width, and what is left of it inside the
    /// reference box is the contact; its middle is where the impulse acts.
    /// A corner driven into a flank lands on that corner, two flanks side by
    /// side land on the middle of the length they share.
    /// </summary>
    private static Vector2 ClipContactPoint(
        in CarBodyGeometry reference,
        in CarBodyGeometry incident,
        Vector2 referenceNormal
    )
    {
        bool alongForward =
            MathF.Abs(Vector2.Dot(referenceNormal, reference.Forward)) > 0.5f;
        float faceOffset = alongForward ? reference.HalfLength : reference.HalfWidth;
        Vector2 sideAxis = alongForward ? reference.Left : reference.Forward;
        float sideHalf = alongForward ? reference.HalfWidth : reference.HalfLength;
        Vector2 faceCenter = reference.Center + referenceNormal * faceOffset;

        // The incident edge is the one whose outward normal points most
        // nearly back at the reference face.
        Vector2 edgeNormal = incident.Forward;
        float edgeOffset = incident.HalfLength;
        Vector2 edgeAxis = incident.Left;
        float edgeHalf = incident.HalfWidth;
        float best = Vector2.Dot(incident.Forward, referenceNormal);
        ConsiderEdge(-incident.Forward, incident.HalfLength, incident.Left, incident.HalfWidth,
            referenceNormal, ref best, ref edgeNormal, ref edgeOffset, ref edgeAxis, ref edgeHalf);
        ConsiderEdge(incident.Left, incident.HalfWidth, incident.Forward, incident.HalfLength,
            referenceNormal, ref best, ref edgeNormal, ref edgeOffset, ref edgeAxis, ref edgeHalf);
        ConsiderEdge(-incident.Left, incident.HalfWidth, incident.Forward, incident.HalfLength,
            referenceNormal, ref best, ref edgeNormal, ref edgeOffset, ref edgeAxis, ref edgeHalf);

        Vector2 edgeCenter = incident.Center + edgeNormal * edgeOffset;
        Vector2 p1 = edgeCenter + edgeAxis * edgeHalf;
        Vector2 p2 = edgeCenter - edgeAxis * edgeHalf;

        // Clip to the reference face's two side planes.
        float t1 = Vector2.Dot(p1 - faceCenter, sideAxis);
        float t2 = Vector2.Dot(p2 - faceCenter, sideAxis);
        ClipToSlab(ref p1, ref t1, ref p2, ref t2, sideHalf);

        // Keep what is inside the reference box.
        float d1 = Vector2.Dot(p1 - faceCenter, referenceNormal);
        float d2 = Vector2.Dot(p2 - faceCenter, referenceNormal);
        bool in1 = d1 <= Epsilon;
        bool in2 = d2 <= Epsilon;
        if (in1 && !in2)
            return p1;
        if (in2 && !in1)
            return p2;
        return (p1 + p2) * 0.5f;
    }

    private static void ConsiderEdge(
        Vector2 normal,
        float offset,
        Vector2 axis,
        float half,
        Vector2 referenceNormal,
        ref float best,
        ref Vector2 edgeNormal,
        ref float edgeOffset,
        ref Vector2 edgeAxis,
        ref float edgeHalf
    )
    {
        float alignment = Vector2.Dot(normal, referenceNormal);
        if (alignment >= best)
            return;
        best = alignment;
        edgeNormal = normal;
        edgeOffset = offset;
        edgeAxis = axis;
        edgeHalf = half;
    }

    private static void ClipToSlab(
        ref Vector2 p1,
        ref float t1,
        ref Vector2 p2,
        ref float t2,
        float half
    )
    {
        ClipBelow(ref p1, ref t1, ref p2, ref t2, half);
        // The other side is the same cut seen from the opposite direction.
        t1 = -t1;
        t2 = -t2;
        ClipBelow(ref p1, ref t1, ref p2, ref t2, half);
        t1 = -t1;
        t2 = -t2;
    }

    /// <summary>Moves whichever end is beyond <c>t = limit</c> back onto it.</summary>
    private static void ClipBelow(
        ref Vector2 p1,
        ref float t1,
        ref Vector2 p2,
        ref float t2,
        float limit
    )
    {
        if (t1 > limit && t2 > limit)
        {
            // Wholly outside: nothing better than the nearer end.
            if (t1 < t2) { p2 = p1; t2 = t1; } else { p1 = p2; t1 = t2; }
            return;
        }
        if (t1 > limit)
        {
            float f = (limit - t2) / (t1 - t2);
            p1 = p2 + (p1 - p2) * f;
            t1 = limit;
        }
        else if (t2 > limit)
        {
            float f = (limit - t1) / (t2 - t1);
            p2 = p1 + (p2 - p1) * f;
            t2 = limit;
        }
    }

    private static bool ResolvePair(RaceCar a, RaceCar b)
    {
        if (!TryGetContact(a, b, out CarContact contact))
            return false;

        a.HitCarThisStep = true;
        b.HitCarThisStep = true;
        float invMassA = InverseMass(a);
        float invMassB = InverseMass(b);
        float invMassSum = invMassA + invMassB;
        if (invMassSum <= Epsilon)
            return false;

        Vector2 correction = contact.Normal *
                             ((contact.PenetrationMeters + SeparationSlopMeters) / invMassSum);
        a.State.Position -= correction * invMassA;
        b.State.Position += correction * invMassB;

        ApplyImpulse(a, b, contact.Normal, contact.Point, invMassA, invMassB);
        return true;
    }

    /// <summary>
    /// A rigid-body impulse at the contact point: normal with restitution,
    /// then Coulomb friction along the sliding direction, each with its
    /// lever arm about both cars' centres, so an off-centre hit turns the
    /// cars as well as pushing them.
    ///
    /// What comes out is a velocity and a yaw rate, and that is all that is
    /// written back. The body keeps pointing where it was pointing; the
    /// difference between that and where it is now going is sideslip, and
    /// the slip-angle dynamics take it from there. A contact that snapped
    /// the heading onto the velocity and zeroed the slide belonged to a car
    /// model that could not carry sideslip, and on this one it handed any
    /// car that touched another a free recovery.
    /// </summary>
    private static void ApplyImpulse(
        RaceCar a,
        RaceCar b,
        Vector2 normal,
        Vector2 point,
        float invMassA,
        float invMassB
    )
    {
        float invInertiaA = InverseInertia(a);
        float invInertiaB = InverseInertia(b);
        Vector2 armA = point - a.State.Position;
        Vector2 armB = point - b.State.Position;
        Vector2 velocityA = a.State.Velocity;
        Vector2 velocityB = b.State.Velocity;
        float yawA = a.State.YawRateRadiansPerSecond;
        float yawB = b.State.YawRateRadiansPerSecond;

        Vector2 relativeVelocity =
            PointVelocity(velocityB, yawB, armB) -
            PointVelocity(velocityA, yawA, armA);
        float normalSpeed = Vector2.Dot(relativeVelocity, normal);
        if (normalSpeed >= 0f)
            return;

        float restitution = MathF.Min(a.Collision.Restitution, b.Collision.Restitution);
        float normalMass = EffectiveInverseMass(
            normal, armA, armB, invMassA, invMassB, invInertiaA, invInertiaB);
        float impulseMagnitude = -(1f + restitution) * normalSpeed / normalMass;
        Apply(normal * impulseMagnitude);

        relativeVelocity =
            PointVelocity(velocityB, yawB, armB) -
            PointVelocity(velocityA, yawA, armA);
        Vector2 tangentVelocity =
            relativeVelocity - normal * Vector2.Dot(relativeVelocity, normal);
        if (tangentVelocity.LengthSquared() > Epsilon)
        {
            Vector2 tangent = Vector2.Normalize(tangentVelocity);
            float tangentMass = EffectiveInverseMass(
                tangent, armA, armB, invMassA, invMassB, invInertiaA, invInertiaB);
            float tangentImpulseMagnitude =
                -Vector2.Dot(relativeVelocity, tangent) / tangentMass;
            float friction = MathF.Min(a.Collision.Friction, b.Collision.Friction);
            float maxFrictionImpulse = impulseMagnitude * friction;
            tangentImpulseMagnitude = Math.Clamp(
                tangentImpulseMagnitude,
                -maxFrictionImpulse,
                maxFrictionImpulse
            );
            Apply(tangent * tangentImpulseMagnitude);
        }

        ApplyMotion(a.State, velocityA, yawA);
        ApplyMotion(b.State, velocityB, yawB);

        void Apply(Vector2 impulse)
        {
            velocityA -= impulse * invMassA;
            yawA -= Cross(armA, impulse) * invInertiaA;
            velocityB += impulse * invMassB;
            yawB += Cross(armB, impulse) * invInertiaB;
        }
    }

    private static float EffectiveInverseMass(
        Vector2 direction,
        Vector2 armA,
        Vector2 armB,
        float invMassA,
        float invMassB,
        float invInertiaA,
        float invInertiaB
    )
    {
        float leverA = Cross(armA, direction);
        float leverB = Cross(armB, direction);
        return invMassA + invMassB +
               leverA * leverA * invInertiaA +
               leverB * leverB * invInertiaB;
    }

    private static Vector2 PointVelocity(Vector2 velocity, float yawRate, Vector2 arm) =>
        velocity + new Vector2(-arm.Y, arm.X) * yawRate;

    private static float Cross(Vector2 a, Vector2 b) => a.X * b.Y - a.Y * b.X;

    private static bool TryUpdateMinimumOverlap(
        in CarBodyGeometry bodyA,
        in CarBodyGeometry bodyB,
        Vector2 axis,
        bool axisBelongsToA,
        ref float minimumOverlap,
        ref Vector2 bestAxis,
        ref bool referenceIsA
    )
    {
        bodyA.ProjectOntoAxis(axis, out float minA, out float maxA);
        bodyB.ProjectOntoAxis(axis, out float minB, out float maxB);
        float overlap = MathF.Min(maxA, maxB) - MathF.Max(minA, minB);
        if (overlap <= 0f)
            return false;

        if (overlap < minimumOverlap)
        {
            minimumOverlap = overlap;
            bestAxis = axis;
            referenceIsA = axisBelongsToA;
        }
        return true;
    }

    private static float InverseMass(RaceCar car)
    {
        return 1f / Math.Max(car.CarConfig.MassKg, Epsilon);
    }

    private static float InverseInertia(RaceCar car)
    {
        return 1f / Math.Max(car.CarConfig.YawInertiaKgM2, Epsilon);
    }

    /// <summary>
    /// Writes a post-contact velocity and yaw rate without touching the
    /// heading: the new direction of travel becomes sideslip. At a crawl the
    /// direction of a near-zero velocity means nothing, so the sideslip is
    /// left as it was.
    /// </summary>
    private static void ApplyMotion(CarState state, Vector2 velocity, float yawRate)
    {
        float speed = velocity.Length();
        state.Speed = speed;
        state.YawRateRadiansPerSecond = yawRate;
        if (speed > 0.05f)
        {
            state.SideslipAngleRadians = MathHelper.NormalizeAngle(
                MathF.Atan2(velocity.Y, velocity.X) - state.Heading
            );
        }
    }
}

public readonly record struct CarContact(
    RaceCar A,
    RaceCar B,
    Vector2 Normal,
    float PenetrationMeters,
    Vector2 Point
);
