using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Racing;

public static class TrackBoundaryResolver
{
    private const int SweepIterations = 16;
    private const float Epsilon = 1e-4f;

    public static TrackRegion Classify(TrackPose pose)
    {
        float d = pose.D;
        TrackSample sample = pose.Sample;

        if (MathF.Abs(d) <= sample.HalfWidth)
            return TrackRegion.RacingSurface;

        if (d > 0f)
            return d <= sample.HalfWidth + sample.LeftBufferWidth ? TrackRegion.Buffer : TrackRegion.BeyondWall;

        return d >= -sample.HalfWidth - sample.RightBufferWidth ? TrackRegion.Buffer : TrackRegion.BeyondWall;
    }

    public static (float LeftWallD, float RightWallD) GetWallLimits(TrackSample sample)
    {
        return (
            sample.HalfWidth + sample.LeftBufferWidth,
            -sample.HalfWidth - sample.RightBufferWidth
        );
    }

    public static bool IsInsideTrackWalls(TrackData track, CarState state, CarCollisionConfig collision)
    {
        return !TryFindDeepestViolation(
            track,
            CarBodyGeometry.FromState(state, collision),
            out _
        );
    }

    public static TrackBoundaryContact? ResolveCurrent(
        TrackData track,
        CarState state,
        CarCollisionConfig collision
    ) => ResolveCurrent(track, state, collision, new CarConfig());

    public static TrackBoundaryContact? ResolveCurrent(
        TrackData track,
        CarState state,
        CarCollisionConfig collision,
        CarConfig car
    )
    {
        TrackBoundaryContact? lastContact = null;

        for (int i = 0; i < Math.Max(1, collision.SolverIterations); i++)
        {
            CarBodyGeometry body = CarBodyGeometry.FromState(state, collision);
            if (!TryFindDeepestViolation(track, body, out WallViolation violation))
                return lastContact;

            state.Position += violation.Normal * (violation.Penetration + Epsilon);
            ApplyWallVelocityResponse(state, collision, car, violation.Normal, violation.Arm);
            lastContact = CreateContact(violation, impactFraction: 0f, state.Position);
        }

        return lastContact;
    }

    public static TrackBoundaryContact? ResolveSweep(
        TrackData track,
        CarState startState,
        CarState targetState,
        CarCollisionConfig collision
    ) => ResolveSweep(track, startState, targetState, collision, new CarConfig());

    public static TrackBoundaryContact? ResolveSweep(
        TrackData track,
        CarState startState,
        CarState targetState,
        CarCollisionConfig collision,
        CarConfig car
    )
    {
        CarBodyGeometry startBody = CarBodyGeometry.FromState(startState, collision);
        if (TryFindDeepestViolation(track, startBody, out _))
            return ResolveCurrent(track, targetState, collision, car);

        CarBodyGeometry targetBody = CarBodyGeometry.FromState(targetState, collision);
        if (!TryFindDeepestViolation(track, targetBody, out WallViolation targetViolation))
            return null;

        Vector2 targetPosition = targetState.Position;
        float targetHeading = targetState.Heading;
        float low = 0f;
        float high = 1f;
        for (int i = 0; i < SweepIterations; i++)
        {
            float mid = (low + high) * 0.5f;
            CarBodyGeometry midBody = InterpolateBody(
                startState.Position,
                startState.Heading,
                targetPosition,
                targetHeading,
                collision,
                mid
            );

            if (TryFindDeepestViolation(track, midBody, out _))
                high = mid;
            else
                low = mid;
        }

        Vector2 safePosition = Vector2.Lerp(startState.Position, targetPosition, low);
        float safeHeading = LerpAngle(startState.Heading, targetHeading, low);
        // The search above moves position and heading as one fraction, so
        // when it is the rotation that reaches the wall -- a corner swinging
        // into the barrier while the car itself is moving clear -- the
        // translation is stopped at the same instant. With a corner already
        // against the wall that instant is zero, the car is put back where it
        // started, and the next step asks for the same rotation: a car stuck
        // at one spot with its wheels turning. Whatever the wall refused, the
        // rest of the translation is searched again at the heading it did
        // allow. A car driving into the wall gets nothing more from this
        // than it had; a car sliding along or away from it keeps moving.
        //
        // The same is true the other way round. A car whose front corner is
        // on the wall and which is still drifting towards it has its
        // translation refused at once, and the combined search then refuses
        // the rotation with it -- including a rotation taking the nose away
        // from the wall. The heading is put back every step while the tyres
        // go on building yaw that is never allowed to act, and the car
        // scrapes along the barrier pinned at the angle it arrived at. The
        // wall used to hide this by snapping the heading along itself on
        // every contact. So the rotation the wall allows is searched on its
        // own first, then the translation at that heading.
        safeHeading = SweepRotation(
            track,
            safePosition,
            safeHeading,
            targetHeading,
            collision
        );
        safePosition = SweepTranslation(
            track,
            safePosition,
            targetPosition,
            safeHeading,
            collision
        );
        targetState.Position = safePosition;
        targetState.Heading = safeHeading;

        CarBodyGeometry contactBody = InterpolateBody(
            startState.Position,
            startState.Heading,
            targetPosition,
            targetHeading,
            collision,
            Math.Min(1f, high)
        );
        if (!TryFindDeepestViolation(track, contactBody, out WallViolation contactViolation))
            contactViolation = targetViolation;

        ApplyWallVelocityResponse(
            targetState, collision, car, contactViolation.Normal, contactViolation.Arm);
        return CreateContact(contactViolation, low, targetState.Position);
    }

    /// <summary>
    /// How far along <paramref name="from"/> to <paramref name="to"/> a body
    /// held at <paramref name="heading"/> can go without crossing a wall,
    /// then sliding along the wall it met for whatever of the move was
    /// parallel to it. <paramref name="from"/> must itself be clear; every
    /// position returned is one that was checked.
    ///
    /// Stopping the whole move at the first touch is right for a car
    /// driving into the wall and wrong for one running along it: a car
    /// flush with the barrier whose heading is a thousandth of a radian into
    /// it touches on every substep, and used to lose most of every step's
    /// travel to that. The wall used to hide it by laying every car it
    /// touched exactly parallel to itself.
    /// </summary>
    private static Vector2 SweepTranslation(
        TrackData track,
        Vector2 from,
        Vector2 to,
        float heading,
        CarCollisionConfig collision
    )
    {
        Vector2 reached = SweepStraight(track, from, to, heading, collision, out WallViolation? blocked);
        if (blocked is not WallViolation wall)
            return reached;

        Vector2 remaining = to - reached;
        float into = Vector2.Dot(remaining, wall.Normal);
        if (into >= 0f)
            return reached;

        Vector2 along = reached + (remaining - wall.Normal * into);
        return SweepStraight(track, reached, along, heading, collision, out _);
    }

    /// <summary>
    /// The furthest checked-clear position on the segment, and the wall that
    /// stopped it there if one did.
    /// </summary>
    private static Vector2 SweepStraight(
        TrackData track,
        Vector2 from,
        Vector2 to,
        float heading,
        CarCollisionConfig collision,
        out WallViolation? blocked
    )
    {
        if (!TryFindDeepestViolation(
                track,
                InterpolateBody(from, heading, to, heading, collision, 1f),
                out WallViolation atEnd))
        {
            blocked = null;
            return to;
        }

        float low = 0f;
        float high = 1f;
        WallViolation stop = atEnd;
        for (int i = 0; i < SweepIterations; i++)
        {
            float mid = (low + high) * 0.5f;
            if (TryFindDeepestViolation(
                    track,
                    InterpolateBody(from, heading, to, heading, collision, mid),
                    out WallViolation found))
            {
                high = mid;
                stop = found;
            }
            else
            {
                low = mid;
            }
        }

        blocked = stop;
        return Vector2.Lerp(from, to, low);
    }

    /// <summary>
    /// How far from <paramref name="from"/> towards <paramref name="to"/> a
    /// body held at <paramref name="position"/> can turn without crossing a
    /// wall. The body at <paramref name="from"/> must itself be clear.
    /// </summary>
    private static float SweepRotation(
        TrackData track,
        Vector2 position,
        float from,
        float to,
        CarCollisionConfig collision
    )
    {
        if (!TryFindDeepestViolation(
                track,
                InterpolateBody(position, from, position, to, collision, 1f),
                out _))
        {
            return to;
        }

        float low = 0f;
        float high = 1f;
        for (int i = 0; i < SweepIterations; i++)
        {
            float mid = (low + high) * 0.5f;
            if (TryFindDeepestViolation(
                    track,
                    InterpolateBody(position, from, position, to, collision, mid),
                    out _))
            {
                high = mid;
            }
            else
            {
                low = mid;
            }
        }

        return LerpAngle(from, to, low);
    }

    private static CarBodyGeometry InterpolateBody(
        Vector2 startPosition,
        float startHeading,
        Vector2 targetPosition,
        float targetHeading,
        CarCollisionConfig collision,
        float t
    )
    {
        float heading = LerpAngle(startHeading, targetHeading, t);
        Vector2 forward = new(MathF.Cos(heading), MathF.Sin(heading));
        Vector2 left = new(-forward.Y, forward.X);
        return new CarBodyGeometry(
            Vector2.Lerp(startPosition, targetPosition, t),
            forward,
            left,
            Math.Max(0f, collision.HalfLengthMeters),
            Math.Max(0f, collision.HalfWidthMeters)
        );
    }

    private static bool TryFindDeepestViolation(
        TrackData track,
        CarBodyGeometry body,
        out WallViolation violation
    )
    {
        violation = default;
        bool found = false;

        for (int cornerIndex = 0; cornerIndex < 4; cornerIndex++)
        {
            Vector2 corner = body.GetCorner(cornerIndex);
            TrackPose pose = track.Project(corner);
            var limits = GetWallLimits(pose.Sample);

            if (pose.D > limits.LeftWallD)
            {
                float penetration = pose.D - limits.LeftWallD;
                if (!found || penetration > violation.Penetration)
                {
                    violation = new WallViolation(
                        TrackSide.Left,
                        penetration,
                        limits.LeftWallD,
                        -pose.Sample.Normal,
                        corner,
                        corner - body.Center,
                        pose
                    );
                    found = true;
                }
            }
            else if (pose.D < limits.RightWallD)
            {
                float penetration = limits.RightWallD - pose.D;
                if (!found || penetration > violation.Penetration)
                {
                    violation = new WallViolation(
                        TrackSide.Right,
                        penetration,
                        limits.RightWallD,
                        pose.Sample.Normal,
                        corner,
                        corner - body.Center,
                        pose
                    );
                    found = true;
                }
            }
        }

        return found;
    }

    /// <summary>
    /// The wall's impulse on the car, at the corner that reached it.
    ///
    /// The wall does not move, so the whole impulse is the car's. Normal:
    /// the corner's approach speed comes back at the wall restitution.
    /// Along the wall: the corner loses the share of its sliding speed the
    /// impact severity calls for, which is the calibration the wall has
    /// always had, now taken at the corner. Both are solved with the
    /// corner's lever arm, so a car that clips the wall with its nose is
    /// turned by it, and the linear change is what a body that can also
    /// rotate takes from the same impulse.
    ///
    /// What is written back is velocity and yaw rate. The heading is not
    /// snapped onto the new velocity and the slide is not zeroed: that
    /// belonged to the car model before slip angles, and on this one it
    /// made the wall a free recovery for a car that was losing it.
    /// </summary>
    private static void ApplyWallVelocityResponse(
        CarState state,
        CarCollisionConfig collision,
        CarConfig car,
        Vector2 normal,
        Vector2 arm
    )
    {
        float invMass = 1f / MathF.Max(car.MassKg, Epsilon);
        float invInertia = 1f / MathF.Max(car.YawInertiaKgM2, Epsilon);
        Vector2 velocity = state.Velocity;
        float yawRate = state.YawRateRadiansPerSecond;

        Vector2 pointVelocity = PointVelocity(velocity, yawRate, arm);
        float normalSpeed = Vector2.Dot(pointVelocity, normal);
        if (normalSpeed >= 0f)
            return;

        float severity = Math.Clamp(
            -normalSpeed / Math.Max(collision.ReferenceImpactSpeed, Epsilon),
            0f,
            1f
        );

        float normalImpulse = -(1f + collision.WallRestitution) * normalSpeed /
                              EffectiveInverseMass(normal, arm, invMass, invInertia);
        Apply(normal * normalImpulse);

        pointVelocity = PointVelocity(velocity, yawRate, arm);
        Vector2 tangentVelocity =
            pointVelocity - normal * Vector2.Dot(pointVelocity, normal);
        float slidingSpeed = tangentVelocity.Length();
        if (slidingSpeed > Epsilon)
        {
            Vector2 tangent = tangentVelocity / slidingSpeed;
            float removed = slidingSpeed *
                            Math.Clamp(collision.WallFriction * severity, 0f, 1f);
            float tangentImpulse = -removed /
                                   EffectiveInverseMass(tangent, arm, invMass, invInertia);
            Apply(tangent * tangentImpulse);
        }

        state.Speed = velocity.Length();
        if (state.Speed <= CarPhysics.DynamicYawMinimumSpeed)
        {
            // At a crawl the car model does not carry sideslip or a yaw rate
            // of its own: the body follows its path. A turn handed over here
            // would be discarded on the next step, and a car nosed into the
            // barrier, with no reverse gear, would stay there -- forward is
            // into the wall and every turn swings a front corner into it.
            // So in this regime the wall still lays the car along the
            // direction the impulse left it moving, as it always has. Above
            // it the slide is real and is kept.
            if (state.Speed > 0.05f)
            {
                state.Heading = MathF.Atan2(velocity.Y, velocity.X);
                state.SideslipAngleRadians = 0f;
                state.YawRateRadiansPerSecond = 0f;
            }
            return;
        }

        state.YawRateRadiansPerSecond = yawRate;
        state.SideslipAngleRadians = MathHelper.NormalizeAngle(
            MathF.Atan2(velocity.Y, velocity.X) - state.Heading
        );

        void Apply(Vector2 impulse)
        {
            velocity += impulse * invMass;
            yawRate += Cross(arm, impulse) * invInertia;
        }
    }

    private static float EffectiveInverseMass(
        Vector2 direction,
        Vector2 arm,
        float invMass,
        float invInertia
    )
    {
        float lever = Cross(arm, direction);
        return invMass + lever * lever * invInertia;
    }

    private static Vector2 PointVelocity(Vector2 velocity, float yawRate, Vector2 arm) =>
        velocity + new Vector2(-arm.Y, arm.X) * yawRate;

    private static float Cross(Vector2 a, Vector2 b) => a.X * b.Y - a.Y * b.X;

    private static TrackBoundaryContact CreateContact(
        WallViolation violation,
        float impactFraction,
        Vector2 correctedPosition
    )
    {
        return new TrackBoundaryContact(
            violation.Side,
            violation.Penetration,
            violation.LimitD,
            violation.Normal,
            impactFraction,
            correctedPosition,
            violation.Pose
        );
    }

    private static float LerpAngle(float from, float to, float weight)
    {
        float delta = MathHelper.NormalizeAngle(to - from);
        return MathHelper.NormalizeAngle(from + delta * weight);
    }

    private readonly record struct WallViolation(
        TrackSide Side,
        float Penetration,
        float LimitD,
        Vector2 Normal,
        Vector2 Point,
        Vector2 Arm,
        TrackPose Pose
    );
}
