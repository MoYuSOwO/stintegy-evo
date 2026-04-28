using Godot;
using StintegyEVO.Core.Car;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers;
using System;
using System.Linq;
using System.Text;

namespace StintegyEVO.Nodes.Car;

public partial class RaceCar : RigidBody2D
{
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
    public Line2D _debug_line = new()
    { 
        Width = 1.0f,
        DefaultColor = Color.FromHtml("#65c4ff"),
        ZIndex = 50,
        Antialiased = true,
        JointMode = Line2D.LineJointMode.Round,
        Closed = false,
        TopLevel = true
    };

    public float VisualOffsetX => (0.5f - Config.Chassis.WeightDistFront) * Config.Chassis.WheelBase;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	public void Init(CarLogic car, IController controller, Vector2 startPos, float startRotation, Node2D node)
    {
        node.AddChild(_debug_line);
        _logic = car;
        _controller = controller;

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

        // Assemble visual effects and collision boxes
        BuildCarVisuals();
		BuildCollisionShape();

        lastGlobalVel = Vector2.Zero;

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
            Position = Vector2.Zero
        };
		
        bodyAnchor.AddChild(collisionNode);
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
        _debug_line.Points = [];
    }

    public override void _PhysicsProcess(double delta)
    {
        if (!IsInit) return;

        float dt = (float)delta;

        Vector2 globalAccel = (LinearVelocity - lastGlobalVel) / dt;
        lastGlobalVel = LinearVelocity;

        Vector2 localAccel = Transform.BasisXformInv(globalAccel);
        Vector2 localVel = Transform.BasisXformInv(LinearVelocity);

        float input = Controller.Input;
        float steer = Controller.SteeringAngle;

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

        GD.Print("drag force: ", output.DragForce);
        GD.Print("force: ", output.FrontLeft.Force, output.FrontRight.Force, output.RearLeft.Force, output.RearRight.Force);

        Controller.ThinkTick(dt, carData, Logic, Logic.Track);

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

    private void ApplyTireForceToBody(Vector2 localTireForce, Vector2 localPos)
    {
        Vector2 globalForce = Transform.BasisXform(localTireForce);
        Vector2 globalOffset = Transform.BasisXform(localPos);
        
        ApplyForce(globalForce, globalOffset);
    }

}
