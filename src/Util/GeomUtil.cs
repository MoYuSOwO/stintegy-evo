using Godot;

namespace PloyRacing.Util;

public static class GeomUtil
{
    public const float g = 9.8f;

    public static Vector2 FivePointStencil(Vector2 p_2, Vector2 p_1, Vector2 p1, Vector2 p2)
    {
        float tx = -p2.X + 8 * p1.X - 8 * p_1.X + p_2.X;
        float ty = -p2.Y + 8 * p1.Y - 8 * p_1.Y + p_2.Y;
        
        Vector2 tangent = new(tx, ty);

        if (tangent.LengthSquared() < 1e-6f) 
        {
            tangent = p1 - p_1;
        }

        return tangent.Normalized();
    }

    public static float NormalizeAngle(float angle) {
        while (angle > Mathf.Pi) angle -= (float) (2 * Mathf.Pi);
        while (angle < -Mathf.Pi) angle += (float) (2 * Mathf.Pi);
        return angle;
    }

    public static float Curvature(Vector2 a, Vector2 b, Vector2 c)
    {
        Vector2 ab = b - a, ac = c - a, bc = c - b;
        float cross = ab.Cross(ac);
        if (Mathf.Abs(cross) < 1e-6f) return 0f;
        float lenAB = ab.Length();
        float lenAC = ac.Length();
        float lenBC = bc.Length();
        float product = lenAB * lenAC * lenBC;
        if (product < 1e-6f) return 0f;

        return (2 * cross) / product;
    }
}