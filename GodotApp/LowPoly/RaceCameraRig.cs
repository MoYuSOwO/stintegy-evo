using System;
using Godot;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class RaceCameraRig : Node3D
{
    public Camera3D Lens { get; } = new() { Name = "Lens", Projection = Camera3D.ProjectionType.Orthogonal, Near = 0.3f, Far = 16000f, Current = true, KeepAspect = Camera3D.KeepAspectEnum.Height };
    public int Mode { get; private set; } = 1;
    private TrackSurfaceGeometry _surface = null!;
    private Vector3 _focus;
    private float _zoom = 1f, _yaw = -0.65f;
    private bool _snap = true;
    public void Initialize(TrackSurfaceGeometry surface) { _surface = surface; AddChild(Lens); }
    public void SetMode(int mode) { Mode = Math.Clamp(mode, 1, 3); _zoom = 1; _yaw = Mode == 3 ? 0.12f : -0.65f; _snap = true; }
    public void Zoom(float multiplier) => _zoom = Math.Clamp(_zoom * multiplier, Mode == 1 ? 0.45f : 0.35f, Mode == 1 ? 2.8f : 1.8f);
    public void Orbit(float radians) => _yaw += radians;
    public void Update(double delta, FormulaCarView3D car)
    {
        if (_surface == null) return;
        Vector3 center = LowPolyMesh.V((_surface.Minimum + _surface.Maximum) * 0.5f);
        Vector3 desired = Mode == 1 ? car.GlobalPosition + car.GlobalBasis.X * 12f : center;
        float blend = _snap ? 1f : 1f - MathF.Exp(-8f * (float)delta);
        _focus = _focus.Lerp(desired, blend);
        float pitch = Mathf.DegToRad(Mode == 1 ? 53f : Mode == 2 ? 70f : 32f);
        Vector3 offset = new(MathF.Sin(_yaw) * MathF.Cos(pitch), MathF.Sin(pitch), MathF.Cos(_yaw) * MathF.Cos(pitch));
        float distance = Mode == 1 ? 180f : (_surface.Maximum - _surface.Minimum).Length() * 1.4f;
        Lens.Position = _focus + offset * distance;
        Lens.LookAt(_focus, Vector3.Up);
        float size = Mode == 1 ? 58f : GlobalSize();
        Lens.Size = Mathf.Lerp(Lens.Size, size * _zoom, blend);
        _snap = false;
    }
    private float GlobalSize()
    {
        var min = _surface.Minimum; var max = _surface.Maximum;
        float left = float.PositiveInfinity, right = float.NegativeInfinity, bottom = float.PositiveInfinity, top = float.NegativeInfinity;
        var basis = Lens.GlobalBasis;
        foreach (float x in new[] { min.X - 35f, max.X + 35f }) foreach (float z in new[] { min.Z - 35f, max.Z + 35f }) foreach (float y in new[] { min.Y, max.Y + 12f })
        {
            Vector3 p = new Vector3(x, y, z) - _focus;
            float px = p.Dot(basis.X), py = p.Dot(basis.Y);
            left = MathF.Min(left, px); right = MathF.Max(right, px); bottom = MathF.Min(bottom, py); top = MathF.Max(top, py);
        }
        var viewport = GetViewport().GetVisibleRect().Size;
        float aspect = viewport.X / MathF.Max(viewport.Y, 1f);
        return MathF.Max(top - bottom, (right - left) / aspect) * 1.32f;
    }
}
