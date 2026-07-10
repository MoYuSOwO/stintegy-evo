using System;
using System.Numerics;

namespace TheStint.Core.Util;

public static class MathHelper
{
    private const double Deg2Rad = Math.PI / 180.0;

    public static float DegToRad(float degrees)
    {
        return (float)(degrees * Deg2Rad);
    }

    public static float RadToDeg(float radians)
    {
        return (float)(radians / Deg2Rad);
    }

    public static Vector2 FivePointStencil(Vector2 p_2, Vector2 p_1, Vector2 p1, Vector2 p2)
    {
        float tx = -p2.X + 8 * p1.X - 8 * p_1.X + p_2.X;
        float ty = -p2.Y + 8 * p1.Y - 8 * p_1.Y + p_2.Y;

        Vector2 tangent = new(tx, ty);

        if (tangent.LengthSquared() < 1e-6f)
        {
            tangent = p1 - p_1;
        }

        return Vector2.Normalize(tangent);
    }

    public static float NormalizeAngle(float angle) {
        while (angle > Math.PI) angle -= (float) (2 * Math.PI);
        while (angle < -Math.PI) angle += (float) (2 * Math.PI);
        return angle;
    }

    public static float Curvature(Vector2 a, Vector2 b, Vector2 c)
    {
        Vector2 ab = b - a, ac = c - a, bc = c - b;
        float cross = Cross(ab, ac);
        if (MathF.Abs(cross) < 1e-6f) return 0f;
        float lenAB = ab.Length();
        float lenAC = ac.Length();
        float lenBC = bc.Length();
        float product = lenAB * lenAC * lenBC;
        if (product < 1e-6f) return 0f;

        return 2 * cross / product;
    }

    public static float Cross(Vector2 a, Vector2 b)
    {
        return a.X * b.Y - a.Y * b.X;
    }

    public static float Angle(this Vector2 vector)
    {
        return MathF.Atan2(vector.Y, vector.X);
    }
}
