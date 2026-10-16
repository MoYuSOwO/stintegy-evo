using System;
using Godot;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.GodotApp.LowPoly;

public partial class FormulaCarView3D : Node3D
{
    public RaceCar Car { get; private set; } = null!;
    public Color TeamColor { get; private set; }
    private TrackSurfaceGeometry _surface = null!;
    private readonly record struct RenderPose(Transform3D Transform, float Steering);
    private SnapshotTimeline<RenderPose> _poses = null!;
    private MeshInstance3D _selection = null!;
    private Label3D _number = null!;
    private readonly Node3D[] _wheels = new Node3D[4];
    private CpuParticles3D _smoke = null!, _dust = null!;

    // A car sliding this far is scrubbing rubber off; one this fast with
    // wheels off the road is throwing up the verge.
    private const float SmokeSlide = 0.085f, SmokeSpeed = 9f, DustSpeed = 7f;
    private static readonly StandardMaterial3D Rubber = new() { AlbedoColor = Color.FromHtml("#252a2b"), Roughness = 1 };
    private static readonly Mesh Tire = LowPolyMesh.Faceted(new CylinderMesh { TopRadius = 0.34f, BottomRadius = 0.34f, Height = 0.36f, RadialSegments = 10, Rings = 1, Material = Rubber });
    public void Bind(RaceCar car, TrackSurfaceGeometry surface, Color color, int number)
    {
        Car = car; _surface = surface; TeamColor = color; Name = $"Car_{number:D2}";
        var mesh = new LowPolyMesh(); var dark = LowPolyMesh.Ink;
        mesh.Box(new(0, 0.17f, 0), new(3.8f, 0.14f, 1.45f), dark);
        mesh.Hull(-1.8f, 0.65f, 0.5f, 0.29f, 0.22f, 0.65f, 0.58f, color);
        mesh.Hull(0.4f, 2.13f, 0.29f, 0.12f, 0.3f, 0.55f, 0.38f, color);
        mesh.Box(new(0.75f, 0.559f, 0), new(1.2f, 0.025f, 0.075f), LowPolyMesh.Ivory);
        foreach (int side in new[] { -1, 1 })
        {
            mesh.Box(new(-0.55f, 0.35f, side * 0.52f), new(1.65f, 0.35f, 0.35f), color.Darkened(0.08f));
            mesh.Box(new(-0.55f, 0.54f, side * 0.55f), new(1.35f, 0.025f, 0.1f), LowPolyMesh.Ivory);
            foreach (float x in new[] { -1.42f, 1.35f }) mesh.Box(new(x, 0.3f, side * 0.5f), new(0.1f, 0.08f, 0.75f), dark);
            mesh.Box(new(1.95f, 0.3f, side * 0.83f), new(0.5f, 0.27f, 0.055f), color);
            mesh.Box(new(-2f, 0.67f, side * 0.7f), new(0.45f, 0.5f, 0.07f), color);
        }
        mesh.Box(new(1.95f, 0.22f, 0), new(0.5f, 0.09f, 1.75f), color);
        mesh.Box(new(-2f, 0.84f, 0), new(0.5f, 0.1f, 1.5f), dark);
        mesh.Box(new(-2.02f, 0.9f, 0), new(0.15f, 0.025f, 1.35f), LowPolyMesh.Ivory);
        mesh.Box(new(-0.05f, 0.62f, 0), new(0.67f, 0.06f, 0.46f), dark);
        mesh.Cone(new(-0.15f, 0.63f, 0), 0.2f, 0.12f, 0.3f, LowPolyMesh.Ivory, 8);
        mesh.Box(new(0.015f, 0.8f, 0), new(0.08f, 0.1f, 0.29f), dark);
        mesh.Hull(-0.9f, -0.4f, 0.16f, 0.12f, 0.55f, 0.91f, 0.95f, color);
        AddChild(mesh.Instance("Body"));
        int index = 0;
        foreach (float x in new[] { 1.35f, -1.42f }) foreach (int side in new[] { -1, 1 })
        {
            var pivot = new Node3D { Position = new Vector3(x, 0.34f, side * 0.77f) };
            var wheel = new MeshInstance3D { Mesh = Tire, Rotation = new Vector3(Mathf.Pi / 2, 0, 0) };
            pivot.AddChild(wheel); AddChild(pivot); _wheels[index++] = pivot;
        }
        var ringMaterial = new StandardMaterial3D { AlbedoColor = LowPolyMesh.Ivory, ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded };
        _selection = new MeshInstance3D { Name = "Selection", Mesh = new TorusMesh { InnerRadius = 2.65f, OuterRadius = 2.71f, Rings = 32, RingSegments = 4, Material = ringMaterial }, Position = new Vector3(0, 0.1f, 0), Scale = new Vector3(1, 0.07f, 0.55f), CastShadow = GeometryInstance3D.ShadowCastingSetting.Off, Visible = false };
        AddChild(_selection);
        _number = new Label3D
        {
            Text = $"{number:00}",
            FontSize = 48,
            PixelSize = 0.012f,
            OutlineSize = 5,
            Modulate = LowPolyMesh.Ivory,
            Position = new Vector3(0, 2.6f, 0),
            Billboard = BaseMaterial3D.BillboardModeEnum.Enabled,
            NoDepthTest = false,
            Visible = false
        };
        AddChild(_number);
        _smoke = Plume("Smoke", new Color(0.9f, 0.9f, 0.88f, 0.38f), 0.55f, 1.6f);
        _dust = Plume("Dust", new Color(0.78f, 0.72f, 0.55f, 0.42f), 0.7f, 1.2f);
        var pose = MakePose(Car.State.Position, Car.State.Heading, Car.State.SteerAngleRadians);
        _poses = new SnapshotTimeline<RenderPose>(pose); Transform = pose.Transform;
    }
    /// <summary>
    /// Rubber smoke from the rear when the car is sliding, dust when it is
    /// off the road: what the state of the car looks like from outside. Set
    /// once per simulated step from that step's state, never invented here.
    /// </summary>
    public void Effects(float slide, float speed, bool offTrack)
    {
        _smoke.Emitting = slide > SmokeSlide && speed > SmokeSpeed && !offTrack;
        _dust.Emitting = offTrack && speed > DustSpeed;
    }

    private CpuParticles3D Plume(string name, Color color, float size, float lifetime)
    {
        var material = new StandardMaterial3D
        {
            AlbedoColor = color,
            Transparency = BaseMaterial3D.TransparencyEnum.Alpha,
            ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded,
            VertexColorUseAsAlbedo = true,
        };
        var puff = new SphereMesh { Radius = size, Height = size * 1.6f, RadialSegments = 6, Rings = 3, Material = material };
        var fade = new Gradient();
        fade.SetColor(0, color);
        fade.SetColor(1, new Color(color, 0f));
        var particles = new CpuParticles3D
        {
            Name = name,
            Emitting = false,
            Amount = 40,
            Lifetime = lifetime,
            Mesh = LowPolyMesh.Faceted(puff),
            LocalCoords = false,
            Position = new Vector3(-1.6f, 0.25f, 0f),
            EmissionShape = CpuParticles3D.EmissionShapeEnum.Box,
            EmissionBoxExtents = new Vector3(0.2f, 0.05f, 0.8f),
            Direction = new Vector3(-1f, 0.35f, 0f),
            Spread = 28f,
            InitialVelocityMin = 1.5f,
            InitialVelocityMax = 3.5f,
            Gravity = new Vector3(0f, 0.7f, 0f),
            DampingMin = 1.2f,
            DampingMax = 2.0f,
            ScaleAmountMin = 0.7f,
            ScaleAmountMax = 1.6f,
            ColorRamp = fade,
            CastShadow = GeometryInstance3D.ShadowCastingSetting.Off,
        };
        AddChild(particles);
        return particles;
    }

    public void Select(bool selected, bool closeView) { _selection.Visible = selected && closeView; _number.Visible = selected && closeView; }
    /// <summary>
    /// The wheels are drawn where they actually are.
    ///
    /// They used to be drawn from the curvature the driver asked for, times
    /// a constant, which meant a car stepping its tail out was rendered
    /// with the steering still wound into the corner - the one thing that
    /// made the old physics look fake even when it was behaving. The car
    /// now carries a real front wheel angle, so opposite lock is not an
    /// effect added here: it is the state, and it appears because the
    /// driver put it there.
    /// </summary>
    public void Capture(double time, System.Numerics.Vector2 position, float heading, float steerAngle)
        => _poses.Add(time, MakePose(position, heading, steerAngle));
    private RenderPose MakePose(System.Numerics.Vector2 position, float heading, float steerAngle)
    {
        var pose = _surface.Track.Project(position);
        float bank = pose.Sample.BankSlopeAt(pose.D);
        var t = pose.Sample.Tangent; var n = pose.Sample.Normal;
        float gx = pose.Sample.Grade * t.X + bank * n.X, gz = pose.Sample.Grade * t.Y + bank * n.Y;
        Vector3 x = new(MathF.Cos(heading), gx * MathF.Cos(heading) + gz * MathF.Sin(heading), MathF.Sin(heading));
        x = x.Normalized(); Vector3 up = new Vector3(-gx, 1, -gz).Normalized(); Vector3 z = x.Cross(up).Normalized();
        var transform = new Transform3D(new Basis(x, z.Cross(x).Normalized(), z), new Vector3(position.X, _surface.Height(pose.S, pose.D) + 0.08f, position.Y));
        return new RenderPose(transform, steerAngle);
    }
    public void Render(double time)
    {
        var (previous, next, fraction) = _poses.Sample(time);
        Transform = previous.Transform.InterpolateWith(next.Transform, fraction);
        float steering = Mathf.Lerp(previous.Steering, next.Steering, fraction);
        _wheels[0].Rotation = new Vector3(0, -steering, 0);
        _wheels[1].Rotation = new Vector3(0, -steering, 0);
    }
}
