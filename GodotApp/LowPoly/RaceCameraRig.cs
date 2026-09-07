using System;
using Godot;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class RaceCameraRig : Node3D
{
    public Camera3D Lens { get; } = new() { Name = "Lens", Near = 0.3f, Far = 16000f, Current = true, KeepAspect = Camera3D.KeepAspectEnum.Height };
    public int Mode { get; private set; } = 1;
    private TrackSurfaceGeometry _surface = null!;
    private Vector3 _focus;
    private float _zoom = 1f, _orbit, _heading;
    private bool _snap = true;
    public void Initialize(TrackSurfaceGeometry surface) { _surface = surface; AddChild(Lens); }
    public void SetMode(int mode) { Mode = Math.Clamp(mode, 1, 3); _zoom = 1; _orbit = 0; _snap = true; }
    public void Zoom(float multiplier) => _zoom = Math.Clamp(_zoom * multiplier, Mode == 1 ? 0.65f : 0.35f, Mode == 3 ? 1.8f : 2.8f);
    public void Orbit(float radians) => _orbit += radians;
    public void Update(double delta, FormulaCarView3D car)
    {
        if (_surface == null) return;
        float blend = _snap ? 1f : 1f - MathF.Exp(-8f * (float)delta);
        float carHeading = MathF.Atan2(car.GlobalBasis.X.Z, car.GlobalBasis.X.X);
        _heading = _snap ? carHeading : Mathf.LerpAngle(_heading, carHeading, 1f - MathF.Exp(-5f * (float)delta));
        Vector3 forward = new(MathF.Cos(_heading), 0, MathF.Sin(_heading));
        Vector3 side = forward.Cross(Vector3.Up);
        Vector3 center = LowPolyMesh.V((_surface.Minimum + _surface.Maximum) * 0.5f);
        _focus = _focus.Lerp(Mode == 3 ? center : car.GlobalPosition, blend);
        if (Mode == 1)
        {
            // Perspective chase: stay behind the car as its heading changes, with a level horizon.
            Lens.Projection = Camera3D.ProjectionType.Perspective;
            Lens.Fov = 58f;
            Vector3 offset = (-forward * 10f + Vector3.Up * 6f).Rotated(Vector3.Up, _orbit);
            Lens.Position = _focus + offset * _zoom;
            Lens.LookAt(_focus + forward * 5f + Vector3.Up * 0.7f, Vector3.Up);
        }
        else if (Mode == 2)
        {
            // A closer side-on composition keeps the silhouette and surrounding track readable.
            Lens.Projection = Camera3D.ProjectionType.Orthogonal;
            Vector3 offset = (side * 35f - forward * 10f + Vector3.Up * 30f).Rotated(Vector3.Up, _orbit);
            Vector3 target = _focus + forward * 5f;
            Lens.Position = target + offset;
            Lens.LookAt(target, Vector3.Up);
            Lens.Size = Mathf.Lerp(Lens.Size, 30f * _zoom, blend);
        }
        else
        {
            // Keep one clearly distinct global view for the complete circuit.
            Lens.Projection = Camera3D.ProjectionType.Orthogonal;
            float pitch = Mathf.DegToRad(38f), yaw = 0.12f + _orbit;
            Vector3 offset = new(MathF.Sin(yaw) * MathF.Cos(pitch), MathF.Sin(pitch), MathF.Cos(yaw) * MathF.Cos(pitch));
            Lens.Position = _focus + offset * (_surface.Maximum - _surface.Minimum).Length() * 1.4f;
            Lens.LookAt(_focus, Vector3.Up);
            Lens.Size = Mathf.Lerp(Lens.Size, GlobalSize() * _zoom, blend);
        }
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
