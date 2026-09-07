using System;
using System.Collections.Generic;
using Godot;

namespace StintegyEVO.GodotApp.LowPoly;

/// <summary>One vertex-colored mesh per static chunk, with deliberately flat face normals.</summary>
public sealed class LowPolyMesh
{
    private readonly List<Vector3> _vertices = [];
    private readonly List<Vector3> _normals = [];
    private readonly List<Color> _colors = [];
    public static readonly StandardMaterial3D Material = new()
    {
        VertexColorUseAsAlbedo = true,
        VertexColorIsSrgb = true,
        Roughness = 0.93f,
        CullMode = BaseMaterial3D.CullModeEnum.Back
    };
    public static Color Ink => Color.FromHtml("#222c30");
    public static Color Ivory => Color.FromHtml("#eee8d6");
    public static Color Vermilion => Color.FromHtml("#c84e38");

    // Input order follows conventional outward-facing cross products. Godot uses clockwise faces.
    public void Triangle(Vector3 a, Vector3 b, Vector3 c, Color color)
    {
        Vector3 normal = (b - a).Cross(c - a).Normalized();
        if (normal.LengthSquared() < 0.1f) return;
        foreach (var p in new[] { a, c, b })
        {
            _vertices.Add(p); _normals.Add(normal); _colors.Add(color);
        }
    }
    public void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color color)
    {
        Triangle(a, b, c, color); Triangle(a, c, d, color);
    }
    public void GroundQuad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color color)
    {
        if ((b - a).Cross(c - a).Y < 0f) Quad(d, c, b, a, color);
        else Quad(a, b, c, d, color);
    }
    public void Box(Vector3 center, Vector3 size, Color color)
    {
        var h = size * 0.5f;
        Vector3 a = center + new Vector3(-h.X, -h.Y, -h.Z), b = center + new Vector3(h.X, -h.Y, -h.Z);
        Vector3 c = center + new Vector3(h.X, -h.Y, h.Z), d = center + new Vector3(-h.X, -h.Y, h.Z);
        Vector3 e = a + Vector3.Up * size.Y, f = b + Vector3.Up * size.Y, g = c + Vector3.Up * size.Y, j = d + Vector3.Up * size.Y;
        Quad(e, j, g, f, color); Quad(a, b, c, d, color);
        Quad(a, e, f, b, color); Quad(b, f, g, c, color); Quad(c, g, j, d, color); Quad(d, j, e, a, color);
    }
    public void Hull(float rear, float front, float rearWidth, float frontWidth, float bottom, float rearTop, float frontTop, Color color)
    {
        Vector3 a = new(rear, bottom, -rearWidth), b = new(front, bottom, -frontWidth), c = new(front, bottom, frontWidth), d = new(rear, bottom, rearWidth);
        Vector3 e = new(rear, rearTop, -rearWidth), f = new(front, frontTop, -frontWidth), g = new(front, frontTop, frontWidth), h = new(rear, rearTop, rearWidth);
        Quad(e, h, g, f, color); Quad(a, b, c, d, color.Darkened(0.1f));
        Quad(a, e, f, b, color); Quad(b, f, g, c, color); Quad(c, g, h, d, color); Quad(d, h, e, a, color);
    }
    public void Cone(Vector3 center, float bottomRadius, float topRadius, float height, Color color, int sides = 7)
    {
        for (int i = 0; i < sides; i++)
        {
            float a = i * Mathf.Tau / sides, b = (i + 1) * Mathf.Tau / sides;
            Vector3 p = center + new Vector3(MathF.Cos(a) * bottomRadius, 0, MathF.Sin(a) * bottomRadius);
            Vector3 q = center + new Vector3(MathF.Cos(b) * bottomRadius, 0, MathF.Sin(b) * bottomRadius);
            Vector3 r = center + new Vector3(MathF.Cos(b) * topRadius, height, MathF.Sin(b) * topRadius);
            Vector3 s = center + new Vector3(MathF.Cos(a) * topRadius, height, MathF.Sin(a) * topRadius);
            Quad(q, p, s, r, color);
            Triangle(center + Vector3.Up * height, r, s, color);
            Triangle(center, p, q, color);
        }
    }
    public ArrayMesh Build()
    {
        var arrays = new Godot.Collections.Array(); arrays.Resize((int)Mesh.ArrayType.Max);
        arrays[(int)Mesh.ArrayType.Vertex] = _vertices.ToArray();
        arrays[(int)Mesh.ArrayType.Normal] = _normals.ToArray();
        arrays[(int)Mesh.ArrayType.Color] = _colors.ToArray();
        var mesh = new ArrayMesh(); mesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, arrays);
        mesh.SurfaceSetMaterial(0, Material); return mesh;
    }
    public MeshInstance3D Instance(string name) => new() { Name = name, Mesh = Build() };
    public static Vector3 V(System.Numerics.Vector3 v) => new(v.X, v.Y, v.Z);
}
