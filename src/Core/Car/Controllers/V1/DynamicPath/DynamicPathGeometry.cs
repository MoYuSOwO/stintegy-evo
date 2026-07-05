using Godot;

namespace StintegyEVO.Core.Car.Controllers.V1.DynamicPath;

public readonly record struct DynamicPathEdgeSample(
    Vector2 Position,
    float Heading,
    float Curvature,
    float LengthToNext
);

public sealed class DynamicPathSplineSegment
{
    public readonly Vector2 A;
    public readonly Vector2 B;
    public readonly Vector2 C;
    public readonly Vector2 D;

    public DynamicPathSplineSegment(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        A = a;
        B = b;
        C = c;
        D = d;
    }

    public Vector2 Evaluate(float t)
    {
        return A + B * t + C * t * t + D * t * t * t;
    }

    public Vector2 Derivative(float t)
    {
        return B + 2.0f * C * t + 3.0f * D * t * t;
    }

    public Vector2 SecondDerivative(float t)
    {
        return 2.0f * C + 6.0f * D * t;
    }
}
