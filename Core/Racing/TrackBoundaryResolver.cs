using System;
using System.Numerics;
using TheStint.Core.Cars;
using TheStint.Core.Track;
using TheStint.Core.Util;

namespace TheStint.Core.Racing;

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
    )
    {
        TrackBoundaryContact? lastContact = null;

        for (int i = 0; i < Math.Max(1, collision.SolverIterations); i++)
        {
            CarBodyGeometry body = CarBodyGeometry.FromState(state, collision);
            if (!TryFindDeepestViolation(track, body, out WallViolation violation))
                return lastContact;

            state.Position += violation.Normal * (violation.Penetration + Epsilon);
            ApplyWallVelocityResponse(state, collision, violation.Normal);
            lastContact = CreateContact(violation, impactFraction: 0f, state.Position);
        }

        return lastContact;
    }

    public static TrackBoundaryContact? ResolveSweep(
        TrackData track,
        CarState startState,
        CarState targetState,
        CarCollisionConfig collision
    )
    {
        CarBodyGeometry startBody = CarBodyGeometry.FromState(startState, collision);
        if (TryFindDeepestViolation(track, startBody, out _))
            return ResolveCurrent(track, targetState, collision);

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

        ApplyWallVelocityResponse(targetState, collision, contactViolation.Normal);
        return CreateContact(contactViolation, low, targetState.Position);
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

        foreach (Vector2 corner in body.GetCorners())
        {
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
                        pose
                    );
                    found = true;
                }
            }
        }

        return found;
    }

    private static void ApplyWallVelocityResponse(
        CarState state,
        CarCollisionConfig collision,
        Vector2 normal
    )
    {
        Vector2 velocity = state.Velocity;
        float normalSpeed = Vector2.Dot(velocity, normal);
        if (normalSpeed >= 0f)
            return;

        Vector2 normalVelocity = normal * normalSpeed;
        Vector2 tangentVelocity = velocity - normalVelocity;
        float severity = Math.Clamp(
            -normalSpeed / Math.Max(collision.ReferenceImpactSpeed, Epsilon),
            0f,
            1f
        );

        Vector2 after =
            tangentVelocity * MathF.Max(0f, 1f - collision.WallFriction * severity) -
            normalVelocity * collision.WallRestitution;

        ApplyVelocity(state, after);
    }

    private static void ApplyVelocity(CarState state, Vector2 velocity)
    {
        float speed = velocity.Length();
        state.Speed = speed;
        if (speed > 0.05f)
            state.Heading = MathF.Atan2(velocity.Y, velocity.X);
    }

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
        TrackPose Pose
    );
}
