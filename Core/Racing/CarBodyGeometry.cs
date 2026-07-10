using System;
using System.Numerics;
using TheStint.Core.Cars;

namespace TheStint.Core.Racing;

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
        Vector2 forward = new(MathF.Cos(state.Heading), MathF.Sin(state.Heading));
        Vector2 left = new(-forward.Y, forward.X);
        return new CarBodyGeometry(
            state.Position,
            forward,
            left,
            Math.Max(0f, collision.HalfLengthMeters),
            Math.Max(0f, collision.HalfWidthMeters)
        );
    }
}
