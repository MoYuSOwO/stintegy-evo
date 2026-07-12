using System;
using System.Numerics;
using StintegyEVO.Core.Cars;

namespace StintegyEVO.Core.Racing;

internal readonly record struct CarBodyGeometry(
    Vector2 Center,
    Vector2 Forward,
    Vector2 Left,
    float HalfLength,
    float HalfWidth
)
{
    public Vector2 FrontLeft => Center + Forward * HalfLength + Left * HalfWidth;
    public Vector2 FrontRight => Center + Forward * HalfLength - Left * HalfWidth;
    public Vector2 RearLeft => Center - Forward * HalfLength + Left * HalfWidth;
    public Vector2 RearRight => Center - Forward * HalfLength - Left * HalfWidth;

    public Vector2[] GetCorners()
    {
        return [FrontLeft, FrontRight, RearRight, RearLeft];
    }

    public static CarBodyGeometry FromState(CarState state, CarCollisionConfig collision)
    {
        return FromPose(
            state.Position,
            state.Heading,
            collision.LengthMeters,
            collision.WidthMeters
        );
    }

    public static CarBodyGeometry FromPose(
        Vector2 center,
        float headingRadians,
        float lengthMeters,
        float widthMeters
    )
    {
        Vector2 forward = new(
            MathF.Cos(headingRadians),
            MathF.Sin(headingRadians)
        );
        Vector2 left = new(-forward.Y, forward.X);
        return new CarBodyGeometry(
            center,
            forward,
            left,
            Math.Max(0f, lengthMeters * 0.5f),
            Math.Max(0f, widthMeters * 0.5f)
        );
    }

    public bool Overlaps(in CarBodyGeometry other)
    {
        return HasOverlapOnAxis(in other, Forward) &&
               HasOverlapOnAxis(in other, Left) &&
               HasOverlapOnAxis(in other, other.Forward) &&
               HasOverlapOnAxis(in other, other.Left);
    }

    private bool HasOverlapOnAxis(
        in CarBodyGeometry other,
        Vector2 axis
    )
    {
        float centerDistance = MathF.Abs(
            Vector2.Dot(other.Center - Center, axis)
        );
        float thisRadius = ProjectionRadius(axis);
        float otherRadius = other.ProjectionRadius(axis);
        return centerDistance <= thisRadius + otherRadius;
    }

    private float ProjectionRadius(Vector2 axis)
    {
        return HalfLength * MathF.Abs(Vector2.Dot(Forward, axis)) +
               HalfWidth * MathF.Abs(Vector2.Dot(Left, axis));
    }
}
