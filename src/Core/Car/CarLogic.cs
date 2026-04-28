using System;
using System.Collections.Immutable;
using Godot;
using PloyRacing.Core.Car.Components;
using PloyRacing.Core.Car.Configs;
using PloyRacing.Core.Track;
using PloyRacing.Nodes.Race;

namespace PloyRacing.Core.Car;

public struct ForceGiver
{
    public Vector2 Force;
    public Vector2 Pos;
}

public struct IntermediateParams
{
    public CarLoad Load;
    public AeroOutput Areo;
    public PowerOutput Power;
    public DistributorOutput Regen, Drive;
    public TireOutput FrontLeft, FrontRight, RearLeft, RearRight;
    public readonly ImmutableArray<TireOutput> Tires => [FrontLeft, FrontRight, RearLeft, RearRight];
}

public struct PhysicsOutput
{
    public Vector2 DragForce;
    public ForceGiver FrontLeft;
    public ForceGiver FrontRight;
    public ForceGiver RearLeft;
    public ForceGiver RearRight;
    public IntermediateParams Params;
}

public class CarLogic
{
    public readonly CarConfig Config;
	public readonly PowerComponent Power;
	public readonly BatteryComponent Battery;
	public readonly AeroComponent Aero;
	public readonly DistributorComponent Distributor;
	public readonly SuspensionComponent Suspension;
    public readonly TrackData Track;
    public readonly IEnvironment Environment;

    private readonly TireComponent[] _tires;
    public TireComponent TireFrontLeft => _tires[0];
	public TireComponent TireFrontRight => _tires[1];
	public TireComponent TireRearLeft => _tires[2];
	public TireComponent TireRearRight => _tires[3];
    public ImmutableArray<TireComponent> Tires => [.. _tires];

    public void ChangeTire(TireType tireType, TireConfig tireConfig)
    {
        _tires[(int)tireType] = new TireComponent(tireConfig, Environment.EnvTemp);
    }

    public float FrontAxleX => (1.0f - Config.Chassis.WeightDistFront) * Config.Chassis.WheelBase;
    public float RearAxleX => -Config.Chassis.WeightDistFront * Config.Chassis.WheelBase;
    public float LeftAxleY => -Config.Chassis.HalfTrackWidth;
    public float RightAxleY => Config.Chassis.HalfTrackWidth;

    private float time = 0;

    public CarLogic(CarConfig config, TrackData track, IEnvironment environment)
    {
        Config = config;
        Track = track;
        Environment = environment;

        Power = new PowerComponent(config.Power);
        Battery = new BatteryComponent(config.Battery, config.InitKWh);
        Aero = new AeroComponent(config.Aero);
        Distributor = new DistributorComponent(config.Distributor);
        Suspension = new SuspensionComponent(config.Suspension, config.InitLoad);

        _tires = new TireComponent[4];
        foreach (var tire in config.Tires)
        {
            _tires[(int)tire.Type] = new TireComponent(tire, environment.EnvTemp);
        }
    }

    public PhysicsOutput Tick(
        float dt,
        float input, float steer,
        Vector2 localVel, Vector2 localAccel, float angularVel, float currentMass
    )
    {
        // 阿克曼转向几何换算
        float maxSteerAngle = Config.Chassis.MaxSteerAngle; 
        float deltaRef = steer * maxSteerAngle;

        float steerFL = deltaRef;
        float steerFR = deltaRef;

        float L = Config.Chassis.WheelBase;
        float W = Config.Chassis.TrackWidth;
        float tanDelta = Mathf.Tan(Mathf.Abs(deltaRef));

        if (tanDelta > 0.001f)
        {
            float R = L / tanDelta; // 计算转弯半径
            float deltaInner = Mathf.Atan(L / (R - W / 2f)); // 内侧轮转角更大
            float deltaOuter = Mathf.Atan(L / (R + W / 2f)); // 外侧轮转角更小

            if (deltaRef > 0) // 右转
            {
                steerFR = deltaInner;
                steerFL = deltaOuter;
            }
            else // 左转
            {
                steerFL = -deltaInner;
                steerFR = -deltaOuter;
            }
        }        

        float speed = localVel.Length();

        // 空气动力学 (先假设没有脏空气)
        AeroOutput aeroOutput = Aero.CalculateAero(speed, 0f);

        // 悬挂系统
        CarLoad currentLoad = Suspension.UpdateAndGetLoad(
            currentMass,
            Config.Chassis.WeightDistFront,
            Config.Chassis.CgHeight,
            Config.Chassis.WheelBase,
            Config.Chassis.Width,
            localAccel.X,
            localAccel.Y,
            aeroOutput.DownforceFront,
            aeroOutput.DownforceRear,
            dt
        );

        // 动力层提需求
        PowerOutput powerOutput = Power.UpdateAndGetDriveForce(input, speed, Battery.SocPct);

        var driveDist = Distributor.CalculateDistributeForce(powerOutput.Drive, steer, speed);
        var regenDist = Distributor.CalculateDistributeForce(powerOutput.Regen, steer, speed);
        float envTemp = Environment.EnvTemp;

        float halfTrack = Config.Chassis.HalfTrackWidth;
        float axFL = FrontAxleX, ayFL = -halfTrack;
        float axFR = FrontAxleX, ayFR = halfTrack;
        float axRL = RearAxleX, ayRL = -halfTrack;
        float axRR = RearAxleX, ayRR = halfTrack;

        // 计算轮胎位置的车身坐标系速度
        Vector2 velFL_Chassis = localVel + new Vector2(-angularVel * ayFL, angularVel * axFL);
        Vector2 velFR_Chassis = localVel + new Vector2(-angularVel * ayFR, angularVel * axFR);
        Vector2 velRL_Chassis = localVel + new Vector2(-angularVel * ayRL, angularVel * axRL);
        Vector2 velRR_Chassis = localVel + new Vector2(-angularVel * ayRR, angularVel * axRR);

        // 将前轮速度旋转到轮胎局部坐标系
        Vector2 velFL_Tire = velFL_Chassis.Rotated(-steerFL);
        Vector2 velFR_Tire = velFR_Chassis.Rotated(-steerFR);
        Vector2 velRL_Tire = velRL_Chassis;
        Vector2 velRR_Tire = velRR_Chassis;

        Vector2[] velTires = [velFL_Tire, velFR_Tire, velRL_Tire, velRR_Tire];

        TireOutput[] tireOutputs = new TireOutput[4];
        for (int i = 0; i < 4; i++)
        {
            var tire = Tires[i];
            float radius = tire.Config.Radius;
            tireOutputs[i] = tire.UpdateAndGetTire(
                driveDist.Tires[i] * radius,
                currentLoad.Tires[i],
                velTires[i],
                dt,
                envTemp
            );
        }

        GD.Print($"input: {input}, steer: {steer}, Drive: {powerOutput.Drive}, Time: {time} s");
        time += dt;
        string torque = "powerTorque: ", force = "tireForce: ";
        foreach (var t in Tires)
        {
            torque += $"{driveDist.Tires[(int)t.Config.Type] * t.Config.Radius}, ";
            force += $"{tireOutputs[(int)t.Config.Type].Force.X}, ";
        }
        GD.Print(torque);
        GD.Print(force);

        PhysicsOutput output = new();

        // 空气阻力
        if (localVel.LengthSquared() > 0.1f) 
        {
            Vector2 dragDirection = -localVel.Normalized();
            Vector2 localDrag = dragDirection * aeroOutput.DragForce; 
            output.DragForce = localDrag;
        }

        // 精确计算驱动消耗和回收充电（基于轮速）
        // 纯驱动状态，电机输出正扭矩
        if (powerOutput.Drive > 0)  
        {
            float totalDrivePower = 0f;
            for (int i = 0; i < 4; i++)
            {
                float driveTorque = driveDist.Tires[i] * Tires[i].Config.Radius;
                float wheelOmega = Tires[i].WheelAngularVel;
                totalDrivePower += driveTorque * wheelOmega;
            }
            Battery.Consume(totalDrivePower, dt);
        }

        // 存在动能回收
        if (powerOutput.Regen > 0)  
        {
            float totalRegenPower = 0f;
            for (int i = 0; i < 4; i++)
            {
                float regenTorque = regenDist.Tires[i] * Tires[i].Config.Radius;
                float wheelOmega = Tires[i].WheelAngularVel;
                totalRegenPower += regenTorque * wheelOmega;
            }
            Battery.Regen(totalRegenPower, dt);
        }

        // 将前轮算出的力，旋转回车身坐标系
        Vector2 forceFL_Chassis = tireOutputs[0].Force.Rotated(steerFL);
        Vector2 forceFR_Chassis = tireOutputs[1].Force.Rotated(steerFR);
        Vector2 forceRL_Chassis = tireOutputs[2].Force;
        Vector2 forceRR_Chassis = tireOutputs[3].Force;

        // 四个轮胎的抓地力
        output.FrontLeft = new()
        {
            Force = forceFL_Chassis,
            Pos = new(FrontAxleX, -halfTrack)
        };
        output.FrontRight = new()
        {
            Force = forceFR_Chassis,
            Pos = new(FrontAxleX, halfTrack)
        };
        output.RearLeft = new()
        {
            Force = forceRL_Chassis,
            Pos = new(RearAxleX, -halfTrack)
        };
        output.RearRight = new()
        {
            Force = forceRR_Chassis,
            Pos = new(RearAxleX, halfTrack)
        };

        output.Params = new()
        {
            Load = currentLoad,
            Power = powerOutput,
            Regen = regenDist,
            Drive = driveDist,
            FrontLeft = tireOutputs[0],
            FrontRight = tireOutputs[1],
            RearLeft = tireOutputs[2],
            RearRight = tireOutputs[3]
        };

        return output;
    }

}