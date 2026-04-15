using System;
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

public struct PhysicsOutput
{
    public float DynamicMass;
    public Vector2 DragForce;
    public ForceGiver FrontLeft;
    public ForceGiver FrontRight;
    public ForceGiver RearLeft;
    public ForceGiver RearRight;
    public float YawPenaltyTorque;
}

public class CarLogic
{
    public readonly CarConfig Config;
	public readonly PowerComponent Power;
	public readonly FuelComponent Fuel;
	public readonly AeroComponent Aero;
	public readonly BrakeComponent Brake;
	public readonly DifferentialComponent Differential;
	public readonly SuspensionComponent Suspension;
    public readonly TrackData Track;
    public readonly IEnvironment Environment;

    private readonly TireComponent[] _tires;
    public TireComponent TireFrontLeft => _tires[0];
	public TireComponent TireFrontRight => _tires[1];
	public TireComponent TireRearLeft => _tires[2];
	public TireComponent TireRearRight => _tires[3];

    public void ChangeTire(TireType tireType, TireConfig tireConfig)
    {
        _tires[(int)tireType] = new TireComponent(tireConfig, Environment.EnvTemp);
    }

    public float FrontAxleX => (1.0f - Config.Chassis.WeightDistFront) * Config.Chassis.WheelBase;
    public float RearAxleX => -Config.Chassis.WeightDistFront * Config.Chassis.WheelBase;

    private Vector2 lastLocalVel = Vector2.Zero;

    public CarLogic(CarConfig config, TrackData track, IEnvironment environment)
    {
        Config = config;
        Track = track;
        Environment = environment;

        Power = new PowerComponent(config.Power);
        Fuel = new FuelComponent(config.Fuel, config.InitFuelL);
        Aero = new AeroComponent(config.Aero);
        Brake = new BrakeComponent(config.Brake, environment.EnvTemp, config.InitBiasFront);
        Differential = new DifferentialComponent(config.Differential);
        Suspension = new SuspensionComponent(config.Suspension, config.InitLoad);

        _tires = new TireComponent[4];
        foreach (var tire in config.Tires)
        {
            _tires[(int)tire.Type] = new TireComponent(tire, environment.EnvTemp);
        }
    }

    public PhysicsOutput Tick(
        float dt,
        float throttle, float brake, float steer,
        Vector2 localVel, float angularVel, float currentMass
    )
    {
        float speed = localVel.Length();

        Vector2 localAccel = (localVel - lastLocalVel) / dt;
        lastLocalVel = localVel;

        // 空气动力学 (先假设没有脏空气)
        AeroOutput aeroData = Aero.UpdateAero(speed, 0f);

        // 悬挂系统
        CarLoad currentLoad = Suspension.UpdateLoads(
            currentMass,
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
                totalDriveForce, gripLimitFL, gripLimitFR, angularVel, Config.Chassis.Width, speed
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
                totalDriveForce, gripLimitRL, gripLimitRR, angularVel, Config.Chassis.Width, speed
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

        PhysicsOutput output = new();

        // 空气阻力
        if (localVel.LengthSquared() > 0.1f) 
        {
            Vector2 dragDirection = -localVel.Normalized();
            Vector2 localDrag = dragDirection * aeroData.DragForce; 
            output.DragForce = localDrag;
        }

        // 油量更新
        Fuel.UpdateFuel(Power.RPMRatio, throttle, dt);
        output.DynamicMass = Config.Chassis.DryMass + Fuel.CurrentFuelMassKg;

        // 四个轮胎的抓地力
        output.FrontLeft = new()
        {
            Force = outFL.Force,
            Pos = new(FrontAxleX, -halfTrack)
        };
        output.FrontRight = new()
        {
            Force = outFR.Force,
            Pos = new(FrontAxleX, halfTrack)
        };
        output.RearLeft = new()
        {
            Force = outRL.Force,
            Pos = new(RearAxleX, -halfTrack)
        };
        output.RearRight = new()
        {
            Force = outRR.Force,
            Pos = new(RearAxleX, halfTrack)
        };

        // 施加差速器惩罚力矩
        output.YawPenaltyTorque = differentialOutput.YawPenalty;

        return output;
    }

}