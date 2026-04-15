using Godot;
using PloyRacing.Core.Car;
using PloyRacing.Core.Car.Configs;
using PloyRacing.Core.Car.Controllers;
using System;

namespace PloyRacing.Nodes.Car;

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

    public float VisualOffsetX => (0.5f - Config.Chassis.WeightDistFront) * Config.Chassis.WheelBase;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	public void Init(CarLogic car, IController controller, Vector2 startPos, float startRotation)
    {
        _logic = car;
        _controller = controller;

        Vector2 globalVisualOffset = new Vector2(VisualOffsetX, 0).Rotated(startRotation);

		// 初始化世界状态
        GravityScale = 0f;
        Position = startPos - globalVisualOffset;
        Rotation = startRotation;
        LinearVelocity = Vector2.Zero;
        AngularVelocity = 0f;

        // 初始化刚体物理属性
        Mass = car.Config.Chassis.DryMass;
        Inertia = car.Config.Chassis.DryI; 

        // 组装视觉效果与碰撞盒
        BuildCarVisuals();
		BuildCollisionShape();

        IsInit = true;
    }

	public void BuildCarVisuals()
    {
        // 清理旧节点 (支持在编辑器里实时刷新)
        foreach (Node child in GetChildren()) 
        {
            child.QueueFree();
        }

        bodyAnchor = new Node2D { Position = new Vector2(VisualOffsetX, 0) };
        AddChild(bodyAnchor);

        float L = Config.Chassis.Length;
        float W = Config.Chassis.Width;

        // 前部坐标系 (从最前端的车鼻尖往回倒推)
        float frontWingTip = L / 2f;
        float frontWingBase = frontWingTip - Config.Visual.FrontWingDepth;
        
        float frontStrutTip = frontWingBase;
        float frontStrutBase = frontStrutTip - Config.Visual.StrutLength; // 车身肉体真正开始的地方

        // 后部坐标系 (从最后端的尾翼末端往前倒推)
        float rearWingTip = -L / 2f;
        float rearWingBase = rearWingTip + Config.Visual.RearWingDepth;
        
        float rearStrutTip = rearWingBase;
        float rearStrutBase = rearStrutTip + Config.Visual.StrutLength;   // 车身肉体真正结束的地方

        // --- 1. 绘制连接杆 (Struts) (ZIndex = 10, 在底盘下面) ---
        // 前连接杆 (连接前翼和车鼻)
        Vector2[] frontStrutL = GetRectPoints(frontStrutBase - 0.2f, -Config.Visual.StrutWidth * 1.5f, frontStrutTip, -Config.Visual.StrutWidth * 0.5f);
        Vector2[] frontStrutR = GetRectPoints(frontStrutBase - 0.2f, Config.Visual.StrutWidth * 0.5f, frontStrutTip, Config.Visual.StrutWidth * 1.5f);
        bodyAnchor.AddChild(CreatePolygon(frontStrutL, Config.Visual.StrutColor, 10));
        bodyAnchor.AddChild(CreatePolygon(frontStrutR, Config.Visual.StrutColor, 10));

        // 后连接杆 (连接尾翼和变速箱)
        Vector2[] rearStrutL = GetRectPoints(rearWingTip, -Config.Visual.StrutWidth * 1.5f, rearStrutBase, -Config.Visual.StrutWidth * 0.5f);
        Vector2[] rearStrutR = GetRectPoints(rearWingTip, Config.Visual.StrutWidth * 0.5f, rearStrutBase, Config.Visual.StrutWidth * 1.5f);
        bodyAnchor.AddChild(CreatePolygon(rearStrutL, Config.Visual.StrutColor, 10));
        bodyAnchor.AddChild(CreatePolygon(rearStrutR, Config.Visual.StrutColor, 10));

        // --- 2. 绘制底盘/车身 (ZIndex = 11) ---
        float narrowW = W / 2f * Config.Visual.BodyNarrowFactor;
        Vector2[] bodyPoints = 
        [
            new(frontStrutBase, 0),                          // 车头正中尖端
            new(frontStrutBase - 0.5f, narrowW),             // 右前收缩点
            new(rearStrutBase + 0.5f, narrowW),            // 右后收缩点
            new(rearStrutBase, W / 2f * 0.4f),             // 尾部右侧
            new(rearStrutBase, -W / 2f * 0.4f),            // 尾部左侧
            new(rearStrutBase + 0.5f, -narrowW),           // 左后收缩点
            new(frontStrutBase - 0.5f, -narrowW)             // 左前收缩点
        ];
        bodyAnchor.AddChild(CreatePolygon(bodyPoints, Config.Visual.BodyColor, 11));

        // --- 3. 绘制驾驶舱 (ZIndex = 12) ---
        // 引入 CockpitOffset 允许调整驾驶室靠前还是靠后
        Vector2[] cockpitPoints = 
        [
            new(Config.Visual.CockpitOffset + 0.8f, 0),         
            new(Config.Visual.CockpitOffset + 0.2f, 0.4f),      
            new(Config.Visual.CockpitOffset - 0.8f, 0.4f),     
            new(Config.Visual.CockpitOffset - 0.8f, -0.4f),    
            new(Config.Visual.CockpitOffset + 0.2f, -0.4f)      
        ];
        bodyAnchor.AddChild(CreatePolygon(cockpitPoints, Config.Visual.CockpitColor, 12));

        // --- 4. 绘制前翼 (ZIndex = 13) ---
        Vector2[] fwPoints = GetRectPoints(frontWingBase, -Config.Visual.FrontWingWidth / 2f, frontWingTip, Config.Visual.FrontWingWidth / 2f);
        bodyAnchor.AddChild(CreatePolygon(fwPoints, Config.Visual.WingColor, 13));

        // --- 5. 绘制尾翼 (ZIndex = 14) ---
        Vector2[] rwPoints = GetRectPoints(rearWingTip, -Config.Visual.RearWingWidth / 2f, rearWingBase, Config.Visual.RearWingWidth / 2f);
        bodyAnchor.AddChild(CreatePolygon(rwPoints, Config.Visual.WingColor, 14));

        // --- 6. 绘制四个轮胎 (ZIndex = 15, 16) ---
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

    // 生成碰撞盒 (完美贴合 ChassisConfig 的长宽)
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
	}

    public override void _PhysicsProcess(double delta)
    {
        if (!IsInit) return;

        float dt = (float)delta;

        CarSensor carData = new()
        {
            LinearVelocity = LinearVelocity,
            AngularVelocity = AngularVelocity,
            Position = GlobalPosition,
            Rotation = GlobalRotation
        };

        Controller.ThinkTick(dt, carData, Logic, Logic.Track);
        float throttle = Controller.Throttle;
        float brake = Controller.Brake;
        float steer = Controller.SteeringAngle;

        Vector2 localVel = Transform.BasisXformInv(LinearVelocity);

        var output = Logic.Tick(dt, throttle, brake, steer, localVel, AngularVelocity, Mass);

        // 空气阻力
        ApplyCentralForce(Transform.BasisXform(output.DragForce));

        // 四个轮胎的抓地力
        ApplyTireForceToBody(output.FrontLeft.Force, output.FrontLeft.Pos);
        ApplyTireForceToBody(output.FrontRight.Force, output.FrontRight.Pos);
        ApplyTireForceToBody(output.RearLeft.Force, output.RearLeft.Pos);
        ApplyTireForceToBody(output.RearRight.Force, output.RearRight.Pos);

        // 施加差速器惩罚力矩
        ApplyTorque(output.YawPenaltyTorque);
    }

    private void ApplyTireForceToBody(Vector2 localTireForce, Vector2 localPos)
    {
        Vector2 globalForce = Transform.BasisXform(localTireForce);
        Vector2 globalOffset = Transform.BasisXform(localPos);
        
        ApplyForce(globalForce, globalOffset);
    }

}
