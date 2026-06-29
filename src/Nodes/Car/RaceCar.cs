using Godot;
using StintegyEVO.Core.Car;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace StintegyEVO.Nodes.Car;

public partial class RaceCar : RigidBody2D
{
    public static bool EnableDebugPrints { get; set; }

	private CarLogic? _logic;

    private IController? _controller;

	private static T GetNotNull<T>(T? obj) where T : class 
	{
        return obj ?? throw new ArgumentNullException(nameof(obj), "Car have not fully initialized!");
	}

	public CarLogic Logic => GetNotNull(_logic);
    public CarConfig Config => Logic.Config;
    public IController Controller => GetNotNull(_controller);
    public bool IsInit { get; private set; } = false;

    private Node2D bodyAnchor = new();
    private Vector2 lastGlobalVel = Vector2.Zero;
    private float telemetryTime;
    private StreamWriter? telemetryFile;
    private int telemetryFrame;
    private const int TelemetryWriteBufferChars = 1 << 20;
    private readonly StringBuilder telemetryLineBuilder = new(2048);
    private readonly StringBuilder telemetryWriteBuffer = new(65536);
    private long racePhysicsPreTelemetryUsec;
    private long raceControllerUsec;
    private long raceDebugPathUsec;
    private long raceTelemetryUsec;
    private long raceBufferWriteUsec;
    private int raceBufferWriteChars;
    private int raceGc0Delta;
    private int raceGc1Delta;
    private int raceGc2Delta;
    private readonly List<Line2D> debugLines = [];
    private readonly List<Vector2[]> debugPathPointBuffers = [];
    private readonly List<long> debugPathVersions = [];
    private Node2D? debugLineParent;
    public string? TelemetryName { get; set; }
    public int TelemetryFrameStride { get; set; } = 1;
    public Line2D _debug_line = CreateDebugLine(ControllerDebugPathStyle.Default);

    public float VisualOffsetX => (0.5f - Config.Chassis.WeightDistFront) * Config.Chassis.WheelBase;

	// Called when the node enters the scene tree for the first time.
    public override void _Ready()
	{
	}

    public override void _ExitTree()
    {
        FlushTelemetryBuffer(flushWriter: true);
        telemetryFile?.Dispose();
        telemetryFile = null;
    }

	public void Init(CarLogic car, IController controller, Vector2 startPos, float startRotation, Node2D node)
    {
        debugLineParent = node;
        if (_debug_line.GetParent() == null)
            node.AddChild(_debug_line);
        if (debugLines.Count == 0)
        {
            debugLines.Add(_debug_line);
            debugPathPointBuffers.Add([]);
            debugPathVersions.Add(long.MinValue);
        }

        _logic = car;
        _controller = controller;
        _controller.Init(car, car.Track);

        Vector2 globalVisualOffset = new Vector2(VisualOffsetX, 0).Rotated(startRotation);

		// Initialize world state
        GravityScale = 0f;
        Position = startPos - globalVisualOffset;
        Rotation = startRotation;
        LinearVelocity = Vector2.Zero;
        AngularVelocity = 0f;

        // Initialize rigid body physical properties
        Mass = car.Config.Chassis.DryMass;
        Inertia = car.Config.Chassis.DryI; 
        // All chassis drag and yaw damping come from CarLogic. Leaving Godot's
        // default body damping enabled silently adds a second braking force.
        LinearDampMode = DampMode.Replace;
        AngularDampMode = DampMode.Replace;
        LinearDamp = 0f;
        AngularDamp = 0f;

        // Assemble visual effects and collision boxes
        BuildCarVisuals();
        BuildCollisionShape();

        lastGlobalVel = Vector2.Zero;
        telemetryTime = 0f;
        telemetryFrame = 0;
        if (TelemetryName != null) InitTelemetry(TelemetryName);

        IsInit = true;
    }

	public void BuildCarVisuals()
    {
        // Clean up old nodes
        foreach (Node child in GetChildren()) 
        {
            child.QueueFree();
        }

        bodyAnchor = new Node2D { Position = new Vector2(VisualOffsetX, 0) };
        AddChild(bodyAnchor);

        float L = Config.Chassis.Length;
        float W = Config.Chassis.Width;

        // Front coordinate system (derived backwards from the frontmost tip of the vehicle's nose)
        float frontWingTip = L / 2f;
        float frontWingBase = frontWingTip - Config.Visual.FrontWingDepth;
        
        float frontStrutTip = frontWingBase;
        float frontStrutBase = frontStrutTip - Config.Visual.StrutLength; // Where car body truly begins

        // Rear coordinate system (derived backwards from the very tip of the tail fin)
        float rearWingTip = -L / 2f;
        float rearWingBase = rearWingTip + Config.Visual.RearWingDepth;
        
        float rearStrutTip = rearWingBase;
        float rearStrutBase = rearStrutTip + Config.Visual.StrutLength;   // Where car body truly ends

        // Draw Struts (ZIndex = 10)
        // Front
        Vector2[] frontStrutL = GetRectPoints(frontStrutBase - 0.2f, -Config.Visual.StrutWidth * 1.5f, frontStrutTip, -Config.Visual.StrutWidth * 0.5f);
        Vector2[] frontStrutR = GetRectPoints(frontStrutBase - 0.2f, Config.Visual.StrutWidth * 0.5f, frontStrutTip, Config.Visual.StrutWidth * 1.5f);
        bodyAnchor.AddChild(CreatePolygon(frontStrutL, Config.Visual.StrutColor, 10));
        bodyAnchor.AddChild(CreatePolygon(frontStrutR, Config.Visual.StrutColor, 10));

        // Rear
        Vector2[] rearStrutL = GetRectPoints(rearWingTip, -Config.Visual.StrutWidth * 1.5f, rearStrutBase, -Config.Visual.StrutWidth * 0.5f);
        Vector2[] rearStrutR = GetRectPoints(rearWingTip, Config.Visual.StrutWidth * 0.5f, rearStrutBase, Config.Visual.StrutWidth * 1.5f);
        bodyAnchor.AddChild(CreatePolygon(rearStrutL, Config.Visual.StrutColor, 10));
        bodyAnchor.AddChild(CreatePolygon(rearStrutR, Config.Visual.StrutColor, 10));

        // Draw the chassis/body (ZIndex = 11)
        float narrowW = W / 2f * Config.Visual.BodyNarrowFactor;
        Vector2[] bodyPoints = 
        [
            new(frontStrutBase, 0),                         // The tip of the front of the car
            new(frontStrutBase - 0.5f, narrowW),            // Right anterior contraction point
            new(rearStrutBase + 0.5f, narrowW),             // Right posterior contraction point
            new(rearStrutBase, W / 2f * 0.4f),              // Right side of the tail
            new(rearStrutBase, -W / 2f * 0.4f),             // Left side of the tail
            new(rearStrutBase + 0.5f, -narrowW),            // Left posterior contraction point
            new(frontStrutBase - 0.5f, -narrowW)            // Left anterior contraction point
        ];
        bodyAnchor.AddChild(CreatePolygon(bodyPoints, Config.Visual.BodyColor, 11));

        // Draw the cockpit (ZIndex = 12)
        Vector2[] cockpitPoints = 
        [
            new(Config.Visual.CockpitOffset + 0.8f, 0),         
            new(Config.Visual.CockpitOffset + 0.2f, 0.4f),      
            new(Config.Visual.CockpitOffset - 0.8f, 0.4f),     
            new(Config.Visual.CockpitOffset - 0.8f, -0.4f),    
            new(Config.Visual.CockpitOffset + 0.2f, -0.4f)      
        ];
        bodyAnchor.AddChild(CreatePolygon(cockpitPoints, Config.Visual.CockpitColor, 12));

        // Draw the front wing (ZIndex = 13)
        Vector2[] fwPoints = GetRectPoints(frontWingBase, -Config.Visual.FrontWingWidth / 2f, frontWingTip, Config.Visual.FrontWingWidth / 2f);
        bodyAnchor.AddChild(CreatePolygon(fwPoints, Config.Visual.WingColor, 13));

        // Draw the rear wing (ZIndex = 14)
        Vector2[] rwPoints = GetRectPoints(rearWingTip, -Config.Visual.RearWingWidth / 2f, rearWingBase, Config.Visual.RearWingWidth / 2f);
        bodyAnchor.AddChild(CreatePolygon(rwPoints, Config.Visual.WingColor, 14));

        // Draw four tires (ZIndex = 15, 16)
        float trackWidthHalf = W / 2f - Config.Visual.TireWidth / 2f; 

        DrawTire(Logic.FrontAxleX, -trackWidthHalf); // FL
        DrawTire(Logic.FrontAxleX, trackWidthHalf);  // FR
        DrawTire(Logic.RearAxleX, -trackWidthHalf);  // RL
        DrawTire(Logic.RearAxleX, trackWidthHalf);   // RR
    }

	private void DrawTire(float cx, float cy)
    {
        Vector2[] tirePts = GetRectPoints(cx - Config.Visual.TireRadius, cy - Config.Visual.TireWidth / 2f, cx + Config.Visual.TireRadius, cy + Config.Visual.TireWidth / 2f);
        AddChild(CreatePolygon(tirePts, Config.Visual.TireColor, 15));

        Vector2[] rimPts = GetRectPoints(cx - Config.Visual.TireRadius * 0.6f, cy - Config.Visual.TireWidth * 0.2f, cx + Config.Visual.TireRadius * 0.6f, cy + Config.Visual.TireWidth * 0.2f);
        AddChild(CreatePolygon(rimPts, Config.Visual.RimColor, 16));
    }

    // Generate a collision box
    private void BuildCollisionShape()
    {
        foreach (Node child in GetChildren())
        {
            if (child is CollisionShape2D) child.QueueFree();
        }

        RectangleShape2D rectShape = new()
        {
            Size = new Vector2(Config.Chassis.Length, Config.Chassis.Width)
        };

        CollisionShape2D collisionNode = new()
        {
            Shape = rectShape,
            Position = bodyAnchor.Position
        };

        AddChild(collisionNode);
    }

    private static Vector2[] GetRectPoints(float minX, float minY, float maxX, float maxY)
    {
        return
        [
            new(minX, minY),
            new(maxX, minY),
            new(maxX, maxY),
            new(minX, maxY)
        ];
    }

    private static Polygon2D CreatePolygon(Vector2[] points, Color color, int zIndex)
    {
        return new Polygon2D
        {
            Polygon = points,
            Color = color,
            ZIndex = zIndex,
            Antialiased = true 
        };
    }

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
    {
    }

    public override void _PhysicsProcess(double delta)
    {
        if (!IsInit) return;

        ulong physicsStart = Time.GetTicksUsec();
        int gc0Before = GC.CollectionCount(0);
        int gc1Before = GC.CollectionCount(1);
        int gc2Before = GC.CollectionCount(2);
        float dt = (float)delta;
        telemetryTime += dt;

        Vector2 globalAccel = (LinearVelocity - lastGlobalVel) / dt;
        lastGlobalVel = LinearVelocity;

        Vector2 localAccel = Transform.BasisXformInv(globalAccel);
        Vector2 localVel = Transform.BasisXformInv(LinearVelocity);

        float input = Mathf.Clamp(Controller.Input, -1f, 1f);
        float steer = Mathf.Clamp(Controller.Steer, -1f, 1f);

        var output = Logic.Tick(dt, input, steer, localVel, localAccel, AngularVelocity, Mass);

        CarSensor carData = new()
        {
            Mass = Mass,
            LinearVelocity = LinearVelocity,
            AngularVelocity = AngularVelocity,
            Position = GlobalPosition,
            Rotation = GlobalRotation,
            LocalAccel = localAccel,
            Params = output.Params
        };

        ApplyCentralForce(Transform.BasisXform(output.DragForce));

        ApplyTireForceToBody(output.FrontLeft.Force, output.FrontLeft.Pos);
        ApplyTireForceToBody(output.FrontRight.Force, output.FrontRight.Pos);
        ApplyTireForceToBody(output.RearLeft.Force, output.RearLeft.Pos);
        ApplyTireForceToBody(output.RearRight.Force, output.RearRight.Pos);

        if (EnableDebugPrints)
        {
            GD.Print("drag force: ", output.DragForce);
            GD.Print("force: ", output.FrontLeft.Force, output.FrontRight.Force, output.RearLeft.Force, output.RearRight.Force);
        }

        ulong controllerStart = Time.GetTicksUsec();
        Controller.ThinkTick(dt, carData, Logic, Logic.Track);
        raceControllerUsec = (long)(Time.GetTicksUsec() - controllerStart);
        ulong debugPathStart = Time.GetTicksUsec();
        UpdateDebugPath();
        raceDebugPathUsec = (long)(Time.GetTicksUsec() - debugPathStart);
        racePhysicsPreTelemetryUsec = (long)(Time.GetTicksUsec() - physicsStart);
        raceGc0Delta = GC.CollectionCount(0) - gc0Before;
        raceGc1Delta = GC.CollectionCount(1) - gc1Before;
        raceGc2Delta = GC.CollectionCount(2) - gc2Before;
        ulong telemetryStart = Time.GetTicksUsec();
        WriteTelemetry(input, steer, carData, output);
        raceTelemetryUsec = (long)(Time.GetTicksUsec() - telemetryStart);

        if (EnableDebugPrints)
        {
            GD.Print("speed: ", LinearVelocity.Length());

            float latMiu = float.MaxValue, longMiu = float.MaxValue;
            foreach (var tire in Logic.Tires)
            {
                latMiu = Mathf.Min(latMiu, tire.CurrLatPeakFriction);
                longMiu = Mathf.Min(longMiu, tire.CurrLongPeakFriction);
            }
            GD.Print("latMinGrid: ", latMiu, ", longMinGrid: ", longMiu);
            string stemp = "surface temp: ", ctemp = "core temp: ", wear = "wear: ", wheelVel = "wheelVel: ";
            foreach (var t in Logic.Tires)
            {
                stemp += $"{t.SurfaceTemp}, ";
                ctemp += $"{t.CoreTemp}, ";
                wear += $"{t.Wear}, ";
                wheelVel += $"{t.WheelAngularVel}, ";
            }
            GD.Print(stemp);
            GD.Print(ctemp);
            GD.Print(wear);
            GD.Print(wheelVel);
            GD.Print();
        }
    }

    private void ApplyTireForceToBody(Vector2 localTireForce, Vector2 localPos)
    {
        Vector2 globalForce = Transform.BasisXform(localTireForce);
        Vector2 globalOffset = Transform.BasisXform(localPos);
        
        ApplyForce(globalForce, globalOffset);
    }

    private void UpdateDebugPath()
    {
        if (Controller is IControllerDebugPaths debugPaths)
        {
            UpdateDebugPaths(debugPaths);
            return;
        }

        ClearDebugLines();
    }

    private void UpdateDebugPaths(IControllerDebugPaths debugPaths)
    {
        int lineCount = Math.Max(debugPaths.DebugPathLineCount, 0);
        EnsureDebugLineCapacity(lineCount);

        for (int lineIndex = 0; lineIndex < lineCount; lineIndex++)
        {
            int pointCount = Math.Max(debugPaths.GetDebugPathPointCount(lineIndex), 0);
            Line2D line = debugLines[lineIndex];
            ApplyDebugLineStyle(line, debugPaths.GetDebugPathStyle(lineIndex));
            long version = debugPaths.GetDebugPathVersion(lineIndex);

            if (pointCount <= 1)
            {
                if (debugPathPointBuffers[lineIndex].Length != 0 || debugPathVersions[lineIndex] != version)
                    line.Points = [];
                debugPathPointBuffers[lineIndex] = [];
                debugPathVersions[lineIndex] = version;
                continue;
            }

            if (version >= 0 && debugPathVersions[lineIndex] == version && debugPathPointBuffers[lineIndex].Length == pointCount)
                continue;

            Vector2[] points = debugPathPointBuffers[lineIndex];
            if (points.Length != pointCount)
            {
                points = new Vector2[pointCount];
                debugPathPointBuffers[lineIndex] = points;
            }

            for (int pointIndex = 0; pointIndex < pointCount; pointIndex++)
                points[pointIndex] = debugPaths.GetDebugPathPoint(lineIndex, pointIndex);

            line.Points = points;
            debugPathVersions[lineIndex] = version;
        }

        for (int lineIndex = lineCount; lineIndex < debugLines.Count; lineIndex++)
        {
            debugLines[lineIndex].Points = [];
            debugPathPointBuffers[lineIndex] = [];
            debugPathVersions[lineIndex] = long.MinValue;
        }
    }

    private void EnsureDebugLineCapacity(int lineCount)
    {
        if (lineCount <= 0)
            return;

        if (debugLines.Count == 0)
        {
            debugLines.Add(_debug_line);
            debugPathPointBuffers.Add([]);
            debugPathVersions.Add(long.MinValue);
            if (_debug_line.GetParent() == null)
                debugLineParent?.AddChild(_debug_line);
        }

        while (debugLines.Count < lineCount)
        {
            Line2D line = CreateDebugLine(ControllerDebugPathStyle.Default);
            debugLineParent?.AddChild(line);
            debugLines.Add(line);
            debugPathPointBuffers.Add([]);
            debugPathVersions.Add(long.MinValue);
        }
    }

    private void ClearDebugLines()
    {
        for (int i = 0; i < debugLines.Count; i++)
        {
            debugLines[i].Points = [];
            debugPathPointBuffers[i] = [];
            debugPathVersions[i] = long.MinValue;
        }
    }

    private static Line2D CreateDebugLine(ControllerDebugPathStyle style)
    {
        Line2D line = new()
        {
            Antialiased = true,
            JointMode = Line2D.LineJointMode.Round,
            Closed = false,
            TopLevel = true
        };
        ApplyDebugLineStyle(line, style);
        return line;
    }

    private static void ApplyDebugLineStyle(Line2D line, ControllerDebugPathStyle style)
    {
        line.Width = style.Width;
        line.DefaultColor = style.Color;
        line.ZIndex = style.ZIndex;
    }

    public void EnableTelemetry(string telemetryName)
    {
        TelemetryName = telemetryName;
        if (IsInit) InitTelemetry(telemetryName);
    }

    private void InitTelemetry(string telemetryName)
    {
        FlushTelemetryBuffer(flushWriter: true);
        telemetryFile?.Dispose();
        telemetryWriteBuffer.Clear();
        string telemetryDir = ProjectSettings.GlobalizePath("res://.tmp");
        Directory.CreateDirectory(telemetryDir);
        string telemetryPath = Path.Combine(telemetryDir, $"{telemetryName}.csv");
        FileStream stream = new(telemetryPath, FileMode.Create, System.IO.FileAccess.Write, FileShare.Read, TelemetryWriteBufferChars);
        telemetryFile = new StreamWriter(stream, new UTF8Encoding(encoderShouldEmitUTF8Identifier: false), TelemetryWriteBufferChars)
        {
            NewLine = "\n"
        };
        List<string> columns =
        [
            "time", "input", "steer", "speed", "angular_vel", "pos_x", "pos_y", "rotation",
            "fl_slip", "fr_slip", "rl_slip", "rr_slip", "fl_angle", "fr_angle", "rl_angle", "rr_angle",
            "fl_slide", "fr_slide", "rl_slide", "rr_slide", "fl_wear", "fr_wear", "rl_wear", "rr_wear",
            "fl_surface", "fr_surface", "rl_surface", "rr_surface", "fl_core", "fr_core", "rl_core", "rr_core",
            "fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel",
            "drive_request", "tire_long_force", "drag_force",
            "race_physics_pre_telemetry_usec", "race_controller_usec", "race_telemetry_usec",
            "race_debug_path_usec", "race_buffer_write_usec", "race_buffer_write_chars",
            "race_gc0_delta", "race_gc1_delta", "race_gc2_delta"
        ];
        if (Controller is IControllerTelemetry controllerTelemetry)
            columns.AddRange(controllerTelemetry.TelemetryColumns);

        telemetryFile.WriteLine(string.Join(",", columns));
    }

    private void WriteTelemetry(float input, float steer, CarSensor sensor, PhysicsOutput output)
    {
        if (telemetryFile == null) return;
        telemetryFrame++;
        if (telemetryFrame % Mathf.Max(TelemetryFrameStride, 1) != 0) return;

        var p = output.Params;
        telemetryLineBuilder.Clear();
        TelemetryCsv.Append(telemetryLineBuilder, telemetryTime);
        TelemetryCsv.Append(telemetryLineBuilder, input);
        TelemetryCsv.Append(telemetryLineBuilder, steer);
        TelemetryCsv.Append(telemetryLineBuilder, sensor.LinearVelocity.Length());
        TelemetryCsv.Append(telemetryLineBuilder, sensor.AngularVelocity);
        TelemetryCsv.Append(telemetryLineBuilder, sensor.Position.X);
        TelemetryCsv.Append(telemetryLineBuilder, sensor.Position.Y);
        TelemetryCsv.Append(telemetryLineBuilder, sensor.Rotation);
        TelemetryCsv.Append(telemetryLineBuilder, p.FrontLeft.SlipRatio);
        TelemetryCsv.Append(telemetryLineBuilder, p.FrontRight.SlipRatio);
        TelemetryCsv.Append(telemetryLineBuilder, p.RearLeft.SlipRatio);
        TelemetryCsv.Append(telemetryLineBuilder, p.RearRight.SlipRatio);
        TelemetryCsv.Append(telemetryLineBuilder, p.FrontLeft.SlipAngle);
        TelemetryCsv.Append(telemetryLineBuilder, p.FrontRight.SlipAngle);
        TelemetryCsv.Append(telemetryLineBuilder, p.RearLeft.SlipAngle);
        TelemetryCsv.Append(telemetryLineBuilder, p.RearRight.SlipAngle);
        TelemetryCsv.Append(telemetryLineBuilder, p.FrontLeft.IsSliding);
        TelemetryCsv.Append(telemetryLineBuilder, p.FrontRight.IsSliding);
        TelemetryCsv.Append(telemetryLineBuilder, p.RearLeft.IsSliding);
        TelemetryCsv.Append(telemetryLineBuilder, p.RearRight.IsSliding);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireFrontLeft.Wear);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireFrontRight.Wear);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireRearLeft.Wear);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireRearRight.Wear);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireFrontLeft.SurfaceTemp);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireFrontRight.SurfaceTemp);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireRearLeft.SurfaceTemp);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireRearRight.SurfaceTemp);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireFrontLeft.CoreTemp);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireFrontRight.CoreTemp);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireRearLeft.CoreTemp);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireRearRight.CoreTemp);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireFrontLeft.WheelAngularVel);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireFrontRight.WheelAngularVel);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireRearLeft.WheelAngularVel);
        TelemetryCsv.Append(telemetryLineBuilder, Logic.TireRearRight.WheelAngularVel);
        TelemetryCsv.Append(telemetryLineBuilder, p.Power.Drive);
        TelemetryCsv.Append(telemetryLineBuilder, p.FrontLeft.Force.X + p.FrontRight.Force.X + p.RearLeft.Force.X + p.RearRight.Force.X);
        TelemetryCsv.Append(telemetryLineBuilder, output.DragForce.Length());
        TelemetryCsv.Append(telemetryLineBuilder, (int)racePhysicsPreTelemetryUsec);
        TelemetryCsv.Append(telemetryLineBuilder, (int)raceControllerUsec);
        TelemetryCsv.Append(telemetryLineBuilder, (int)raceTelemetryUsec);
        TelemetryCsv.Append(telemetryLineBuilder, (int)raceDebugPathUsec);
        TelemetryCsv.Append(telemetryLineBuilder, (int)raceBufferWriteUsec);
        TelemetryCsv.Append(telemetryLineBuilder, raceBufferWriteChars);
        TelemetryCsv.Append(telemetryLineBuilder, raceGc0Delta);
        TelemetryCsv.Append(telemetryLineBuilder, raceGc1Delta);
        TelemetryCsv.Append(telemetryLineBuilder, raceGc2Delta);
        if (Controller is IControllerTelemetry controllerTelemetry)
            controllerTelemetry.AppendTelemetryValues(telemetryLineBuilder);

        telemetryWriteBuffer.Append(telemetryLineBuilder);
        telemetryWriteBuffer.Append('\n');
        if (telemetryWriteBuffer.Length >= TelemetryWriteBufferChars)
            FlushTelemetryBuffer();
    }

    private void FlushTelemetryBuffer(bool flushWriter = false)
    {
        if (telemetryFile == null)
            return;

        if (telemetryWriteBuffer.Length > 0)
        {
            raceBufferWriteChars = telemetryWriteBuffer.Length;
            ulong start = Time.GetTicksUsec();
            telemetryFile.Write(telemetryWriteBuffer);
            raceBufferWriteUsec = (long)(Time.GetTicksUsec() - start);
            telemetryWriteBuffer.Clear();
        }

        if (flushWriter)
            telemetryFile.Flush();
    }

}
