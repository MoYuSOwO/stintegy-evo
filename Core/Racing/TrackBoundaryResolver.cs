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

    /// <summary>
    /// The most a slide along the wall may be put back out of it at its end:
    /// what a wall curving towards the car bulges into a straight slide over
    /// one step. A five-metre radius bulges five centimetres into the
    /// longest step there is; anything deeper is a wall the car ran into,
    /// not one it is running along.
    /// </summary>
    private const float WallFollowMeters = 0.05f;

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
        CarCollisionConfig collision,
        ICar car
    )
    {
        TrackBoundaryContact? lastContact = null;
        Vector2 impulse = Vector2.Zero;

        for (int i = 0; i < Math.Max(1, collision.SolverIterations); i++)
        {
            CarBodyGeometry body = CarBodyGeometry.FromState(state, collision);
            if (!TryFindDeepestViolation(track, body, out WallViolation violation))
                break;

            state.Position += violation.Normal * (violation.Penetration + Epsilon);
            impulse += ApplyWallVelocityResponse(state, collision, car, violation.Normal, violation.Arm);
            lastContact = CreateContact(violation, impactFraction: 0f, state.Position) with { Impulse = impulse };
        }

        return lastContact;
    }

    public static TrackBoundaryContact? ResolveSweep(
        TrackData track,
        CarState startState,
        CarState targetState,
        CarCollisionConfig collision,
        ICar car
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

        Vector2 impulse = ApplyWallVelocityResponse(
            targetState, collision, car, contactViolation.Normal, contactViolation.Arm);
        return CreateContact(contactViolation, low, targetState.Position) with { Impulse = impulse };
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
    ///
    /// The slide is straight, along the wall where it was met, and a wall
    /// is not: on the outside of a bend it curves into the slide. A car put
    /// back a tenth of a millimetre clear touches it again a few centimetres
    /// down the slide, on every step, and with a slide that cannot get
    /// anywhere the car stood still against the wall at full speed for as
    /// long as it pressed on it (Monaco after Sainte Devote, once its
    /// barriers stood on the kerb). So the slide's end is put back out of
    /// the wall it bent into, the way a car is put back out of one at the
    /// start of a step, and the car follows the wall round.
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

        // What is left of the move, less whatever of it went into the wall
        // where the wall was met. A move already heading out of it there is
        // kept whole: it was stopped by the wall curving in further on.
        Vector2 remaining = to - reached;
        float into = Vector2.Dot(remaining, wall.Normal);
        Vector2 along = into < 0f ? reached + (remaining - wall.Normal * into) : to;
        Vector2 slid = SweepStraight(track, reached, along, heading, collision, out WallViolation? bent);
        if (bent is null)
            return slid;

        Vector2 followed = along;
        for (int i = 0; i < 4; i++)
        {
            if (!TryFindDeepestViolation(
                    track,
                    InterpolateBody(followed, heading, followed, heading, collision, 0f),
                    out WallViolation bulge))
            {
                return followed;
            }
            if (bulge.Penetration > WallFollowMeters)
                break;
            followed += bulge.Normal * (bulge.Penetration + Epsilon);
        }
        return slid;
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
    /// Along the wall: Coulomb friction, at most the friction coefficient
    /// times that normal impulse, and never more than stops the corner's
    /// slide. Both are solved with the
    /// corner's lever arm, so a car that clips the wall with its nose is
    /// turned by it, and the linear change is what a body that can also
    /// rotate takes from the same impulse.
    ///
    /// What is written back is velocity and yaw rate. The heading is not
    /// snapped onto the new velocity and the slide is not zeroed: that
    /// belonged to the car model before slip angles, and on this one it
    /// made the wall a free recovery for a car that was losing it.
    ///
    /// Returns the impulse the wall gave the car, N·s.
    /// </summary>
    private static Vector2 ApplyWallVelocityResponse(
        CarState state,
        CarCollisionConfig collision,
        ICar car,
        Vector2 normal,
        Vector2 arm
    )
    {
        float invMass = 1f / MathF.Max(car.TotalMassKg(state), Epsilon);
        float invInertia = 1f / MathF.Max(car.Spec.YawInertiaKgM2, Epsilon);
        Vector2 velocity = state.Velocity;
        float yawRate = state.YawRateRadiansPerSecond;

        // At a crawl the car model is kinematic, and the yaw rate it reports
        // is not a rotation the body carries: it is the heading being laid
        // onto the direction of travel, a whole angle in a step, which reads
        // as ten radians a second and more. Taken as real, that rate puts a
        // corner through the wall at tens of metres a second, and the
        // impulse that stops the corner turns the phantom rotation into
        // translation: a car stopped against the barrier at half a metre a
        // second came off it at four, with ten times its static tyre load
        // for the step. Below the dynamic speed the wall meets the car as a
        // body that translates only.
        if (state.Speed <= car.Spec.KinematicBelowSpeed)
        {
            yawRate = 0f;
            invInertia = 0f;
        }

        Vector2 pointVelocity = PointVelocity(velocity, yawRate, arm);
        float normalSpeed = Vector2.Dot(pointVelocity, normal);
        if (normalSpeed >= 0f)
            return Vector2.Zero;

        Vector2 total = Vector2.Zero;
        float normalImpulse = -(1f + collision.WallRestitution) * normalSpeed /
                              EffectiveInverseMass(normal, arm, invMass, invInertia);
        Apply(normal * normalImpulse);

        // Coulomb friction at the corner: the wall resists the corner's
        // slide along it with at most the friction coefficient times the
        // impulse it pushed back with, and never more than stops the slide.
        // A hard hit scrubs hard; a car leaning on the barrier through a
        // bend, pressed into it a little every step by its own tyres, is
        // pushed back a little every step and scrubs a little every step:
        // the steady drag of a car rubbing the wall, which a share of the
        // slide taken by impact speed alone never gave.
        pointVelocity = PointVelocity(velocity, yawRate, arm);
        Vector2 tangentVelocity =
            pointVelocity - normal * Vector2.Dot(pointVelocity, normal);
        float slidingSpeed = tangentVelocity.Length();
        if (slidingSpeed > Epsilon)
        {
            Vector2 tangent = tangentVelocity / slidingSpeed;
            float stopping = slidingSpeed /
                             EffectiveInverseMass(tangent, arm, invMass, invInertia);
            float tangentImpulse = -MathF.Min(collision.WallFriction * normalImpulse, stopping);
            Apply(tangent * tangentImpulse);
        }

        state.Speed = velocity.Length();
        if (state.Speed <= car.Spec.KinematicBelowSpeed)
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
            return total;
        }

        state.YawRateRadiansPerSecond = yawRate;
        state.SideslipAngleRadians = MathHelper.NormalizeAngle(
            MathF.Atan2(velocity.Y, velocity.X) - state.Heading
        );
        return total;

        void Apply(Vector2 impulse)
        {
            total += impulse;
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
        )
        {
            Point = violation.Point
        };
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
