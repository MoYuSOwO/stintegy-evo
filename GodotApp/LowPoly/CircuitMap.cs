using System.Collections.Generic;
using Godot;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class CircuitMap : Control
{
    private Vector2[] _line = [];
    private Vector2 _min, _range;
    private IReadOnlyList<RaceCar>? _cars;
    private int _selected;
    private Vector2[] _positions = [];
    public void Initialize(TrackSurfaceGeometry surface, IReadOnlyList<RaceCar> cars)
    {
        _cars = cars; _min = new(surface.Minimum.X, surface.Minimum.Z); _range = new(surface.Maximum.X - _min.X, surface.Maximum.Z - _min.Y);
        _line = new Vector2[401];
        for (int i = 0; i < _line.Length; i++) { var p = surface.Track.Sample(surface.Track.LengthMeters * i / 400f).Center; _line[i] = new(p.X, p.Y); }
        MouseFilter = MouseFilterEnum.Ignore;
    }
    public void Refresh(int selected)
    {
        _selected = selected;
        if (_cars == null) return;
        if (_positions.Length != _cars.Count) _positions = new Vector2[_cars.Count];
        for (int i = 0; i < _cars.Count; i++) { var p = _cars[i].State.Position; _positions[i] = new Vector2(p.X, p.Y); }
        QueueRedraw();
    }
    private Vector2 Map(Vector2 point)
    {
        float scale = Mathf.Min((Size.X - 24f) / _range.X, (Size.Y - 24f) / _range.Y);
        return (point - _min - _range * 0.5f) * scale + Size * 0.5f;
    }
    public override void _Draw()
    {
        if (_line.Length == 0 || _cars == null) return;
        var path = new Vector2[_line.Length]; for (int i = 0; i < path.Length; i++) path[i] = Map(_line[i]);
        DrawPolyline(path, Color.FromHtml("#a6aaa0"), 2f, true);
        for (int i = 0; i < _positions.Length; i++)
        {
            var at = Map(_positions[i]);
            DrawCircle(at, i == _selected ? 4f : 2f, i == _selected ? LowPolyMesh.Vermilion : LowPolyMesh.Ink);
        }
    }
}
