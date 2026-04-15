using Godot;
using PloyRacing.Core.Car.Components;
using PloyRacing.Core.Car.Configs;
using PloyRacing.Core.Track;
using PloyRacing.Nodes.Car.Controllers;
using PloyRacing.Nodes.Race;
using System;
using PowerComponent = PloyRacing.Core.Car.Components.PowerComponent;

namespace PloyRacing.Nodes.Car;

public partial class RaceCar : RigidBody2D
{
	private CarConfig? _config;
	private PowerComponent? _power;
    private FuelComponent? _fuel;
    private AeroComponent? _aero;
    private BrakeComponent? _brake;
    private DifferentialComponent? _differential;
    private SuspensionComponent? _suspension;
	private TireComponent[]? _tires;
    private BaseController? _driver;
    private TrackData? _track;
    private IEnvironment? _environment;

	private static T GetNotNull<T>(T? obj) where T : class 
	{
        return obj ?? throw new ArgumentNullException(nameof(obj), "Car have not fully initialized!");
	}

	public CarConfig Config => GetNotNull(_config);
	public PowerComponent Power => GetNotNull(_power);
	public FuelComponent Fuel => GetNotNull(_fuel);
	public AeroComponent Aero => GetNotNull(_aero);
	public BrakeComponent Brake => GetNotNull(_brake);
	public DifferentialComponent Differential => GetNotNull(_differential);
	public SuspensionComponent Suspension => GetNotNull(_suspension);
	public TireComponent TireFrontLeft => GetNotNull(_tires)[0];
	public TireComponent TireFrontRight => GetNotNull(_tires)[1];
	public TireComponent TireRearLeft => GetNotNull(_tires)[2];
	public TireComponent TireRearRight => GetNotNull(_tires)[3];
    public TrackData Track => GetNotNull(_track);
    public BaseController Driver => GetNotNull(_driver);
    public IEnvironment Environment => GetNotNull(_environment);
    public bool IsInit { get; private set; } = false;

    public float VisualOffsetX => (0.5f - Config.Chassis.WeightDistFront) * Config.Chassis.WheelBase;
    public float FrontAxleX => (1.0f - Config.Chassis.WeightDistFront) * Config.Chassis.WheelBase;
    public float RearAxleX => -Config.Chassis.WeightDistFront * Config.Chassis.WheelBase;

    private Node2D bodyAnchor = new();
    private Vector2 lastLocalVel = Vector2.Zero;

    public void ChangeTire(TireType tireType, TireConfig tireConfig)
    {
        TireComponent[] tires = GetNotNull(_tires);
        tires[(int)tireType] = new TireComponent(tireConfig, Environment.EnvTemp);
    }
    

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	public void Init(CarConfig config, TrackData track, IEnvironment environment, BaseController controller, Vector2 startPos, float startRotation)
    {
        _config = config;
        _track = track;
        _environment = environment;

        Vector2 globalVisualOffset = new Vector2(VisualOffsetX, 0).Rotated(startRotation);

		// 初始化世界状态
        GravityScale = 0f;
        Position = startPos - globalVisualOffset;
        Rotation = startRotation;
        LinearVelocity = Vector2.Zero;
        AngularVelocity = 0f;

        // 初始化刚体物理属性
        Mass = config.Chassis.DryMass;
        Inertia = config.Chassis.DryI; 

        // 实例化所有物理逻辑组件
        _power = new PowerComponent(config.Power);
        _fuel = new FuelComponent(config.Fuel, config.InitFuelL);
        _aero = new AeroComponent(config.Aero);
        _brake = new BrakeComponent(config.Brake, environment.EnvTemp, config.InitBiasFront);
        _differential = new DifferentialComponent(config.Differential);
        _suspension = new SuspensionComponent(config.Suspension, config.InitLoad);

        // 实例化轮胎
		_tires = new TireComponent[4];
        foreach (var tire in config.Tires)
        {
            _tires[(int)tire.Type] = new TireComponent(tire, environment.EnvTemp);
        }

        // 组装视觉效果与碰撞盒
        BuildCarVisuals();
		BuildCollisionShape();

        _driver = controller;
        lastLocalVel = Transform.BasisXformInv(LinearVelocity);
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

        DrawTire(FrontAxleX, -trackWidthHalf); // FL
        DrawTire(FrontAxleX, trackWidthHalf);  // FR
        DrawTire(RearAxleX, -trackWidthHalf);  // RL
        DrawTire(RearAxleX, trackWidthHalf);   // RR
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

        Driver.Think(dt, this, Track); 
        float throttle = Driver.Throttle;
        float brake = Driver.Brake;
        float steer = Driver.SteeringAngle;

        Vector2 globalVel = LinearVelocity;
        Vector2 localVel = Transform.BasisXformInv(globalVel);
        float speed = localVel.Length();

        Vector2 localAccel = (localVel - lastLocalVel) / dt;
        lastLocalVel = localVel;

        // 油量更新
        Fuel.UpdateFuel(Power.RPMRatio, throttle, dt);
        float currentDynamicMass = Config.Chassis.DryMass + Fuel.CurrentFuelMassKg;
        Mass = currentDynamicMass;

        // 空气动力学 (先假设没有脏空气)
        AeroOutput aeroData = Aero.UpdateAero(speed, 0f);

        // 悬挂系统
        CarLoad currentLoad = Suspension.UpdateLoads(
            currentDynamicMass,
            Config.Chassis.WeightDistFront,
            Config.Chassis.CgHeight,
            Config.Chassis.WheelBase,
            Config.Chassis.Width,
            localAccel.X,
            localAccel.Y,
            aeroData.DownforceFront,
            aeroData.DownforceRear,
            dt
        );

        // 动力层提需求
        float totalDriveForce = Power.CalculateDriveForce(throttle, speed, dt);
        CarBrakeForce brakeDemand = Brake.GetBrakeDemand(brake);

        DifferentialOutput differentialOutput;
        float envTemp = Environment.EnvTemp;
        TireOutput outFL, outFR, outRL, outRR;
        if (Config.DriveType == CarDriveType.FrontDrive)
        {
            float gripLimitFL = currentLoad.FrontLeft * TireFrontLeft.Config.LongPeakFriction;
            float gripLimitFR = currentLoad.FrontRight * TireFrontRight.Config.LongPeakFriction;
            differentialOutput = Differential.DistributeForce(
                totalDriveForce, gripLimitFL, gripLimitFR, AngularVelocity, Config.Chassis.Width, speed
            );

            outFL = TireFrontLeft.UpdatePhysics(
                differentialOutput.ForceLeft - brakeDemand.FrontLeft, currentLoad.FrontLeft, localVel.X, localVel.Y, dt, envTemp, steer
            );
            outFR = TireFrontRight.UpdatePhysics(
                differentialOutput.ForceRight - brakeDemand.FrontRight, currentLoad.FrontRight, localVel.X, localVel.Y, dt, envTemp, steer
            );

            outRL = TireRearLeft.UpdatePhysics(
                -brakeDemand.RearLeft, currentLoad.RearLeft, localVel.X, localVel.Y, dt, envTemp, 0f
            );
            outRR = TireRearRight.UpdatePhysics(
                -brakeDemand.RearRight, currentLoad.RearRight, localVel.X, localVel.Y, dt, envTemp, 0f
            );
        }
        else if (Config.DriveType == CarDriveType.RearDrive)
        {
            float gripLimitRL = currentLoad.RearLeft * TireRearLeft.Config.LongPeakFriction;
            float gripLimitRR = currentLoad.RearRight * TireRearRight.Config.LongPeakFriction;
            differentialOutput = Differential.DistributeForce(
                totalDriveForce, gripLimitRL, gripLimitRR, AngularVelocity, Config.Chassis.Width, speed
            );

            outFL = TireFrontLeft.UpdatePhysics(
                -brakeDemand.FrontLeft, currentLoad.FrontLeft, localVel.X, localVel.Y, dt, envTemp, steer
            );
            outFR = TireFrontRight.UpdatePhysics(
                -brakeDemand.FrontRight, currentLoad.FrontRight, localVel.X, localVel.Y, dt, envTemp, steer
            );

            outRL = TireRearLeft.UpdatePhysics(
                differentialOutput.ForceLeft - brakeDemand.RearLeft, currentLoad.RearLeft, localVel.X, localVel.Y, dt, envTemp, 0f
            );
            outRR = TireRearRight.UpdatePhysics(
                differentialOutput.ForceRight - brakeDemand.RearRight, currentLoad.RearRight, localVel.X, localVel.Y, dt, envTemp, 0f
            );
        }
        else
        {
            throw new NotImplementedException("You should really check what drive type you add......");
        }

        CarBrakeForce actualBrakeForces = new()
        {
            FrontLeft = Mathf.Min(Mathf.Abs(outFL.Force.X), brakeDemand.FrontLeft),
            FrontRight = Mathf.Min(Mathf.Abs(outFR.Force.X), brakeDemand.FrontRight),
            RearLeft = Mathf.Min(Mathf.Abs(outRL.Force.X), brakeDemand.RearLeft),
            RearRight = Mathf.Min(Mathf.Abs(outRR.Force.X), brakeDemand.RearRight)
        };
        Brake.UpdateThermodynamics(actualBrakeForces, envTemp, outFL.IsLockedUp, outFR.IsLockedUp, outRL.IsLockedUp, outRR.IsLockedUp, speed, dt);

        float halfTrack = Config.Chassis.Width / 2f;

        // 空气阻力
        if (localVel.LengthSquared() > 0.1f) 
        {
            Vector2 dragDirection = -localVel.Normalized();
            Vector2 localDrag = dragDirection * aeroData.DragForce; 
            ApplyCentralForce(Transform.BasisXform(localDrag));
        }

        // 四个轮胎的抓地力
        ApplyTireForceToBody(outFL.Force, new Vector2(FrontAxleX, -halfTrack));
        ApplyTireForceToBody(outFR.Force, new Vector2(FrontAxleX, halfTrack));
        ApplyTireForceToBody(outRL.Force, new Vector2(RearAxleX, -halfTrack));
        ApplyTireForceToBody(outRR.Force, new Vector2(RearAxleX, halfTrack));

        // 施加差速器惩罚力矩
        ApplyTorque(differentialOutput.YawPenalty);
    }

    private void ApplyTireForceToBody(Vector2 localTireForce, Vector2 localPos)
    {
        Vector2 globalForce = Transform.BasisXform(localTireForce);
        Vector2 globalOffset = Transform.BasisXform(localPos);
        
        ApplyForce(globalForce, globalOffset);
    }

}
