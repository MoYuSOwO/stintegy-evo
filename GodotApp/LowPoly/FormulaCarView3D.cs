using System;
using Godot;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.GodotApp.Interop;

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
    // Front left, front right, rear left, rear right; a pack's model may
    // leave any out, and its wheels then do not steer.
    private readonly Node3D?[] _wheels = new Node3D?[4];
    private readonly Vector3[] _wheelRest = new Vector3[4];
    private static readonly string[] WheelNodes = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"];
    private CpuParticles3D _smoke = null!, _dust = null!;

    // Tyres this far past their peak (a tenth: sliding, spinning up or
    // locked) are scrubbing rubber off; a car this fast with wheels off
    // the road is throwing up the verge.
    private const float SmokeSlide = 0.1f, SmokeSpeed = 9f, DustSpeed = 7f;
    private static readonly StandardMaterial3D Rubber = new() { AlbedoColor = Color.FromHtml("#252a2b"), Roughness = 1 };
    private static readonly Mesh Tire = LowPolyMesh.Faceted(new CylinderMesh { TopRadius = 0.34f, BottomRadius = 0.34f, Height = 0.36f, RadialSegments = 10, Rings = 1, Material = Rubber });
    public void Bind(RaceCar car, TrackSurfaceGeometry surface, Color color, int number)
    {
        Car = car; _surface = surface; TeamColor = color; Name = $"Car_{number:D2}";
        // Drawn where the physics has its axles: the centre of mass is this
        // node's origin, the axles sit the car's own distances either side.
        CarSpec spec = car.Car.Spec;
        bool packModel = GodotMods.Registry.Origin(car.Car) is { Model: not null, DiskFolder: { } folder } registered &&
                         BindPackModel(System.IO.Path.Combine(folder, registered.Manifest.Model!));
        if (!packModel)
            BuildGameModel(color, spec.FrontAxleOffsetMeters, -spec.RearAxleOffsetMeters, spec.TrackWidthMeters / 2f);
        for (int i = 0; i < _wheels.Length; i++)
            _wheelRest[i] = _wheels[i]?.Rotation ?? Vector3.Zero;
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
        var pose = MakePose(Car.State.Position, Car.State.Heading, (Car.Car as ICarMotion)?.Motion(Car.State).WheelAngleRadians ?? 0f);
        _poses = new SnapshotTimeline<RenderPose>(pose); Transform = pose.Transform;
    }
    /// <summary>
    /// The game's own look, for a car whose pack brings none: a single-seater
    /// in the team's colour with its wheels where the car has them.
    /// </summary>
    private void BuildGameModel(Color color, float front, float rear, float halfTrack)
    {
        var mesh = new LowPolyMesh(); var dark = LowPolyMesh.Ink;
        mesh.Box(new(0, 0.17f, 0), new(3.8f, 0.14f, 1.45f), dark);
        mesh.Hull(-1.8f, 0.65f, 0.5f, 0.29f, 0.22f, 0.65f, 0.58f, color);
        mesh.Hull(0.4f, 2.13f, 0.29f, 0.12f, 0.3f, 0.55f, 0.38f, color);
        mesh.Box(new(0.75f, 0.559f, 0), new(1.2f, 0.025f, 0.075f), LowPolyMesh.Ivory);
        foreach (int side in new[] { -1, 1 })
        {
            mesh.Box(new(-0.55f, 0.35f, side * 0.52f), new(1.65f, 0.35f, 0.35f), color.Darkened(0.08f));
            mesh.Box(new(-0.55f, 0.54f, side * 0.55f), new(1.35f, 0.025f, 0.1f), LowPolyMesh.Ivory);
            foreach (float x in new[] { rear, front }) mesh.Box(new(x, 0.3f, side * 0.5f), new(0.1f, 0.08f, 0.75f), dark);
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
        foreach (float x in new[] { front, rear }) foreach (int side in new[] { -1, 1 })
        {
            var pivot = new Node3D { Position = new Vector3(x, 0.34f, side * halfTrack) };
            var wheel = new MeshInstance3D { Mesh = Tire, Rotation = new Vector3(Mathf.Pi / 2, 0, 0) };
            pivot.AddChild(wheel); AddChild(pivot); _wheels[index++] = pivot;
        }
    }

    /// <summary>
    /// A pack's own model (car.json's <c>model</c>), a glTF file in metres
    /// with Y up and the car facing glTF's forward (+Z), its origin on the
    /// ground under the centre of mass. Nodes called wheel_fl, wheel_fr,
    /// wheel_rl and wheel_rr are its wheels; the front two steer. A model
    /// that will not load leaves the car drawn the game's way.
    /// </summary>
    private bool BindPackModel(string path)
    {
        if (RuntimeModels.Load(path, "car")?.Instantiate() is not Node3D model)
            return false;
        // glTF's forward onto this node's, which is +X.
        model.Rotation = new Vector3(0f, Mathf.Pi / 2f, 0f);
        AddChild(model);
        for (int i = 0; i < WheelNodes.Length; i++)
            _wheels[i] = model.FindChild(WheelNodes[i], recursive: true, owned: false) as Node3D;
        return true;
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
        for (int i = 0; i < 2; i++)
            if (_wheels[i] is { } wheel)
                wheel.Rotation = _wheelRest[i] + new Vector3(0, -steering, 0);
    }
}
