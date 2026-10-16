using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.Core.Track;

namespace StintegyEVO.GodotApp.LowPoly;

/// <summary>
/// The furniture of a circuit that every circuit has: catch fencing behind
/// the barrier, marshal posts round the lap, distance boards before the
/// braking zones and a start gantry over the line.
///
/// Fence and marshal posts stand behind the barrier, the gantry above the
/// road. The distance boards are the exception, as they are at a real
/// circuit: on the grass beside the track, inside the barrier, where a
/// driver at speed can read them. None of it touches the racing surface,
/// and none of it is a boundary: the barrier is still the only one there
/// is. It is placed from the circuit's own geometry rather than authored,
/// because it belongs wherever the shape of the track puts it — a braking
/// zone is where the corner is.
/// </summary>
public partial class TrackDressing3D : Node3D
{
    private static readonly Color Post = Color.FromHtml("#9aa3a3");
    private static readonly Color Rail = Color.FromHtml("#7d8687");
    private static readonly Color Hut = Color.FromHtml("#eee8d6");
    private static readonly Color MarshalOrange = Color.FromHtml("#e07a2e");
    private static readonly Color Board = Color.FromHtml("#f2efe4");
    private static readonly Color Gantry = Color.FromHtml("#3a4346");
    private static readonly Color LightRed = Color.FromHtml("#d23a2a");

    private CircuitView3D _circuit = null!;
    private TrackData _track = null!;
    private Scenery.SceneryFurniture _plan = new();

    public void Initialize(CircuitView3D circuit, Scenery.SceneryFurniture plan)
    {
        _circuit = circuit;
        _track = circuit.Surface.Track;
        _plan = plan;
        if (plan.Fence) BuildFence();
        if (plan.Marshals) BuildMarshalPosts();
        if (plan.Boards) BuildDistanceBoards();
        if (plan.Gantry) BuildStartGantry();
        if (plan.Flags) BuildFlags();
    }

    private readonly List<(Node3D Flag, float Phase)> _flags = [];
    private double _time;

    public override void _Process(double delta)
    {
        _time += delta;
        foreach ((Node3D flag, float phase) in _flags)
            flag.Rotation = new Vector3(0f, flag.Rotation.Y, 0.18f * MathF.Sin((float)_time * 2.3f + phase));
    }

    /// <summary>A row of flags either side of the start straight, behind the fence.</summary>
    private void BuildFlags()
    {
        Color[] colours =
        [
            LowPolyMesh.Vermilion, LowPolyMesh.Ivory, Color.FromHtml("#2f7f86"),
            Color.FromHtml("#d9a441"), Color.FromHtml("#3b5aa0"), LowPolyMesh.Ink,
        ];
        var poles = new LowPolyMesh();
        int k = 0;
        float spacing = MathF.Max(_plan.FlagSpacing, 2f);
        for (float offset = -_plan.FlagRange; offset <= _plan.FlagRange; offset += spacing)
        {
            float s = _track.WrapS(_track.StartingLineS + offset);
            foreach (int side in new[] { -1, 1 })
            {
                Vector3 foot = Behind(s, side, 2.6f);
                poles.Box(foot + Vector3.Up * 3.5f, new Vector3(0.1f, 7f, 0.1f), Post);
                var flagMesh = new LowPolyMesh();
                Color colour = colours[k++ % colours.Length];
                flagMesh.Quad(new Vector3(0, -0.6f, 0), new Vector3(1.8f, -0.6f, 0), new Vector3(1.8f, 0.6f, 0), new Vector3(0, 0.6f, 0), colour);
                flagMesh.Quad(new Vector3(0, 0.6f, 0), new Vector3(1.8f, 0.6f, 0), new Vector3(1.8f, -0.6f, 0), new Vector3(0, -0.6f, 0), colour);
                var flag = flagMesh.Instance($"Flag_{k}");
                flag.Position = foot + Vector3.Up * 6.3f;
                Vector3 along = (_circuit.Point(s + 1f, 0) - _circuit.Point(s, 0)).Normalized();
                flag.Rotation = new Vector3(0f, -MathF.Atan2(along.Z, along.X), 0f);
                AddChild(flag);
                _flags.Add((flag, k * 0.7f));
            }
        }
        AddChild(poles.Instance("FlagPoles"));
    }

    private float WallOffset(float s, int side)
    {
        TrackSample sample = _track.Sample(s);
        return sample.HalfWidth + (side == 1 ? sample.LeftBufferWidth : sample.RightBufferWidth);
    }

    private Vector3 Behind(float s, int side, float metres, float lift = 0f) =>
        _circuit.Point(s, side * (WallOffset(s, side) + metres), lift);

    /// <summary>
    /// Where a distance board stands: on the grass beside the track, past
    /// the kerb and short of the barrier. Three metres out from the track's
    /// edge where the run-off is wide; where it is narrower, halfway between
    /// the kerb and the barrier. Where there is no room for a board between
    /// them at all, as on a street circuit, it goes on the barrier: just
    /// behind it, standing higher, as street circuits mount them. The second
    /// value is how much higher.
    /// </summary>
    private (Vector3 At, float Raise) BoardSpot(float s, int side)
    {
        const float boardWidth = 1.8f;
        TrackSample sample = _track.Sample(s);
        float kerb = SurfaceGrip.KerbWidthMeters;
        float buffer = side == 1 ? sample.LeftBufferWidth : sample.RightBufferWidth;
        float clear = MathF.Max(0f, buffer - kerb);
        if (clear < boardWidth + 0.4f)
            return (Behind(s, side, 0.3f), 0.9f);
        float out_ = clear >= 4.5f ? 3f : kerb + clear * 0.5f;
        return (_circuit.Point(s, side * (sample.HalfWidth + out_)), 0f);
    }

    /// <summary>
    /// Posts every six metres a little behind the barrier, with two rails:
    /// the fence that keeps debris out of the crowd, all the way round.
    /// </summary>
    private void BuildFence()
    {
        float spacing = MathF.Max(_plan.FenceSpacing, 1f);
        const float height = 3.2f;
        int posts = (int)MathF.Ceiling(_track.LengthMeters / spacing);
        float step = _track.LengthMeters / posts;
        for (int begin = 0; begin < posts; begin += 120)
        {
            var mesh = new LowPolyMesh();
            for (int i = begin; i < Math.Min(begin + 120, posts); i++)
            {
                float a = i * step, b = (i + 1) * step;
                foreach (int side in new[] { -1, 1 })
                {
                    Vector3 p = Behind(a, side, 0.9f), q = Behind(b, side, 0.9f);
                    mesh.Box(p + Vector3.Up * height * 0.5f, new Vector3(0.12f, height, 0.12f), Post);
                    foreach (float h in new[] { 1.4f, height - 0.1f })
                        Beam(mesh, p + Vector3.Up * h, q + Vector3.Up * h, 0.07f, Rail);
                }
            }
            var instance = mesh.Instance($"Fence_{begin}");
            instance.CastShadow = GeometryInstance3D.ShadowCastingSetting.Off;
            AddChild(instance);
        }
    }

    /// <summary>A marshal post every three hundred metres or so, sides alternating.</summary>
    private void BuildMarshalPosts()
    {
        var mesh = new LowPolyMesh();
        // Named posts replace the even spacing; otherwise one every so often, sides alternating.
        var posts = new List<(float S, int Side)>(_plan.MarshalsAt);
        if (posts.Count == 0)
        {
            int count = Math.Max(1, (int)(_track.LengthMeters / MathF.Max(_plan.MarshalSpacing, 20f)));
            for (int k = 0; k < count; k++)
                posts.Add(((k + 0.5f) * _track.LengthMeters / count, k % 2 == 0 ? -1 : 1));
        }
        foreach ((float s, int side) in posts)
        {
            Vector3 at = Behind(s, side, 4.5f);
            mesh.Box(at + Vector3.Up * 1.2f, new Vector3(2.2f, 2.4f, 2.2f), Hut);
            mesh.Box(at + Vector3.Up * 2.5f, new Vector3(2.6f, 0.25f, 2.6f), MarshalOrange);
            Vector3 pole = at + (Behind(s, side, 2.5f) - at);
            mesh.Box(pole + Vector3.Up * 2.2f, new Vector3(0.08f, 4.4f, 0.08f), Post);
            Vector3 along = (_circuit.Point(s + 1f, 0) - _circuit.Point(s, 0)).Normalized();
            Panel(mesh, pole + Vector3.Up * 4.0f + along * 0.45f, along, 0.9f, 0.6f, along.Cross(Vector3.Up), MarshalOrange);
        }
        AddChild(mesh.Instance("MarshalPosts"));
    }

    /// <summary>
    /// 150, 100 and 50 metre boards before every corner a car brakes hard
    /// for: found where a long gentle run meets a tight bend, and put on the
    /// outside, on the grass beside the track where a driver looks for them.
    /// </summary>
    private void BuildDistanceBoards()
    {
        var mesh = new LowPolyMesh();
        var zones = new List<(float Corner, int Side)>(_plan.BoardsAt);
        if (_plan.BoardsAuto)
            zones.AddRange(BrakingZones());
        foreach ((float corner, int side) in zones)
        {
            for (int bars = 3; bars >= 1; bars--)
            {
                float s = corner - bars * 50f;
                (Vector3 foot, float raise) = BoardSpot(s, side);
                Vector3 at = foot + Vector3.Up * raise;
                Vector3 along = (_circuit.Point(s + 1f, 0) - _circuit.Point(s, 0)).Normalized();
                Vector3 across = along.Cross(Vector3.Up).Normalized();
                // A board facing the oncoming car: thin along the road, wide across it.
                Panel(mesh, at + Vector3.Up * 1.6f, across, 1.8f, 1.4f, along, Board);
                for (int b = 0; b < bars; b++)
                {
                    float offset = (b - (bars - 1) * 0.5f) * 0.45f;
                    Panel(mesh, at + Vector3.Up * 1.6f + across * offset - along * 0.03f, across, 0.22f, 1.0f, along, LowPolyMesh.Ink);
                }
                float pole = 0.9f + raise;
                mesh.Box(foot + Vector3.Up * pole * 0.5f, new Vector3(0.1f, pole, 0.1f), Post);
            }
        }
        AddChild(mesh.Instance("DistanceBoards"));
    }

    private IEnumerable<(float Corner, int Side)> BrakingZones()
    {
        const float step = 5f, tight = 1f / 90f, gentle = 1f / 400f, runUp = 250f;
        float gentleSince = 0f;
        bool inCorner = false;
        for (float s = 0f; s < _track.LengthMeters; s += step)
        {
            float k = MathF.Abs(_track.Sample(s).Curvature);
            if (k < gentle)
            {
                gentleSince += step;
                inCorner = false;
            }
            else if (k > tight && !inCorner)
            {
                inCorner = true;
                if (gentleSince >= runUp)
                {
                    float wall = WallOffset(s, 1);
                    int outside = _circuit.IsOutside(s + 10f, 1, wall) ? 1 : -1;
                    yield return (s, outside);
                }
                gentleSince = 0f;
            }
        }
    }

    /// <summary>A gantry across the road a little before the line, with the five start lights.</summary>
    private void BuildStartGantry()
    {
        var mesh = new LowPolyMesh();
        float s = _plan.GantryS ?? _track.StartingLineS - 6f;
        Vector3 left = Behind(s, 1, 1.4f), right = Behind(s, -1, 1.4f);
        const float height = 7.5f;
        foreach (Vector3 foot in new[] { left, right })
            mesh.Box(foot + Vector3.Up * height * 0.5f, new Vector3(0.6f, height, 0.6f), Gantry);
        Vector3 topL = left + Vector3.Up * height, topR = right + Vector3.Up * height;
        Beam(mesh, topL, topR, 0.55f, Gantry);
        Vector3 across = (topR - topL).Normalized();
        Vector3 middle = (topL + topR) * 0.5f;
        Vector3 along = across.Cross(Vector3.Up).Normalized();
        for (int i = -2; i <= 2; i++)
        {
            Vector3 lamp = middle + across * i * 0.9f - Vector3.Up * 0.75f;
            mesh.Box(lamp, new Vector3(0.7f, 0.9f, 0.7f), Gantry);
            mesh.Box(lamp + along * 0.36f, new Vector3(0.45f, 0.45f, 0.05f), LightRed);
            mesh.Box(lamp - along * 0.36f, new Vector3(0.45f, 0.45f, 0.05f), LightRed);
        }
        AddChild(mesh.Instance("StartGantry"));
    }

    /// <summary>A square beam from one point to another.</summary>
    private static void Beam(LowPolyMesh mesh, Vector3 from, Vector3 to, float thickness, Color color)
    {
        Vector3 axis = to - from;
        if (axis.LengthSquared() < 1e-6f)
            return;
        Vector3 dir = axis.Normalized();
        Vector3 side = dir.Cross(Vector3.Up);
        if (side.LengthSquared() < 1e-6f) side = Vector3.Right;
        side = side.Normalized() * thickness * 0.5f;
        Vector3 up = side.Cross(dir).Normalized() * thickness * 0.5f;
        Vector3 a = from - side - up, b = from + side - up, c = from + side + up, d = from - side + up;
        Vector3 e = to - side - up, f = to + side - up, g = to + side + up, h = to - side + up;
        mesh.Quad(d, c, g, h, color); mesh.Quad(a, e, f, b, color);
        mesh.Quad(b, f, g, c, color); mesh.Quad(a, d, h, e, color);
    }

    /// <summary>A flat panel facing along <paramref name="normal"/>, both faces drawn.</summary>
    private static void Panel(LowPolyMesh mesh, Vector3 center, Vector3 across, float width, float height, Vector3 normal, Color color)
    {
        Vector3 w = across.Normalized() * width * 0.5f, h = Vector3.Up * height * 0.5f;
        Vector3 a = center - w - h, b = center + w - h, c = center + w + h, d = center - w + h;
        mesh.Quad(a, b, c, d, color);
        mesh.Quad(d, c, b, a, color);
    }
}
