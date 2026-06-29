using System.Collections.Immutable;
using Godot;
using StintegyEVO.Core.Car.Components;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Track;
using StintegyEVO.Nodes.Race;

namespace StintegyEVO.Core.Car;

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
    public static bool EnableDebugPrints { get; set; }

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
        // Ackermann steering geometry
        float maxSteerAngle = Config.Chassis.MaxSteerAngle; 
        float deltaRef = steer * maxSteerAngle;

        float steerFL = deltaRef;
        float steerFR = deltaRef;

        float L = Config.Chassis.WheelBase;
        float W = Config.Chassis.TrackWidth;
        float tanDelta = Mathf.Tan(Mathf.Abs(deltaRef));

        if (tanDelta > 0.001f)
        {
            float R = L / tanDelta; // Calculate the turning radius
            float deltaInner = Mathf.Atan(L / (R - W / 2f)); // Inner wheel has a larger turning angle
            float deltaOuter = Mathf.Atan(L / (R + W / 2f)); // Outter wheel has a smaller turning angle

            // Turn right
            if (deltaRef > 0)
            {
                steerFR = deltaInner;
                steerFL = deltaOuter;
            }
            // Turn left
            else
            {
                steerFL = -deltaInner;
                steerFR = -deltaOuter;
            }
        }        

        float speed = localVel.Length();

        // Aerodynamics (assuming no dirty air)
        AeroOutput aeroOutput = Aero.CalculateAero(speed, 0f);

        // Suspension system
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

        // Power
        PowerOutput powerOutput = Power.UpdateAndGetDriveForce(input, speed, Battery.SocPct);

        var driveDist = Distributor.CalculateDistributeForce(powerOutput.Drive, steer, speed);
        var regenDist = Distributor.CalculateDistributeForce(powerOutput.Regen, steer, speed);
        float envTemp = Environment.EnvTemp;

        float halfTrack = Config.Chassis.HalfTrackWidth;
        float axFL = FrontAxleX, ayFL = -halfTrack;
        float axFR = FrontAxleX, ayFR = halfTrack;
        float axRL = RearAxleX, ayRL = -halfTrack;
        float axRR = RearAxleX, ayRR = halfTrack;

        // Calculate the vehicle coordinate system velocity of the tire position
        Vector2 velFL_Chassis = localVel + new Vector2(-angularVel * ayFL, angularVel * axFL);
        Vector2 velFR_Chassis = localVel + new Vector2(-angularVel * ayFR, angularVel * axFR);
        Vector2 velRL_Chassis = localVel + new Vector2(-angularVel * ayRL, angularVel * axRL);
        Vector2 velRR_Chassis = localVel + new Vector2(-angularVel * ayRR, angularVel * axRR);

        // Rotate the front wheel speed to the tire's local coordinate system
        Vector2 velFL_Tire = velFL_Chassis.Rotated(-steerFL);
        Vector2 velFR_Tire = velFR_Chassis.Rotated(-steerFR);
        Vector2 velRL_Tire = velRL_Chassis;
        Vector2 velRR_Tire = velRR_Chassis;

        TireComponent tireFL = TireFrontLeft;
        TireComponent tireFR = TireFrontRight;
        TireComponent tireRL = TireRearLeft;
        TireComponent tireRR = TireRearRight;

        TireOutput tireOutputFL = tireFL.UpdateAndGetTire(
            driveDist.FrontLeft * tireFL.Config.Radius,
            currentLoad.FrontLeft,
            velFL_Tire,
            dt,
            envTemp
        );
        TireOutput tireOutputFR = tireFR.UpdateAndGetTire(
            driveDist.FrontRight * tireFR.Config.Radius,
            currentLoad.FrontRight,
            velFR_Tire,
            dt,
            envTemp
        );
        TireOutput tireOutputRL = tireRL.UpdateAndGetTire(
            driveDist.RearLeft * tireRL.Config.Radius,
            currentLoad.RearLeft,
            velRL_Tire,
            dt,
            envTemp
        );
        TireOutput tireOutputRR = tireRR.UpdateAndGetTire(
            driveDist.RearRight * tireRR.Config.Radius,
            currentLoad.RearRight,
            velRR_Tire,
            dt,
            envTemp
        );

        if (EnableDebugPrints) GD.Print($"input: {input}, steer: {steer}, Drive: {powerOutput.Drive}, Time: {time} s");
        time += dt;
        if (EnableDebugPrints)
        {
            string torque = "powerTorque: ", force = "tireForce: ";
            torque += $"{driveDist.FrontLeft * tireFL.Config.Radius}, ";
            torque += $"{driveDist.FrontRight * tireFR.Config.Radius}, ";
            torque += $"{driveDist.RearLeft * tireRL.Config.Radius}, ";
            torque += $"{driveDist.RearRight * tireRR.Config.Radius}, ";
            force += $"{tireOutputFL.Force.X}, ";
            force += $"{tireOutputFR.Force.X}, ";
            force += $"{tireOutputRL.Force.X}, ";
            force += $"{tireOutputRR.Force.X}, ";
            GD.Print(torque);
            GD.Print(force);
        }

        PhysicsOutput output = new();

        // Air resistance
        if (localVel.LengthSquared() > 0.1f) 
        {
            Vector2 dragDirection = -localVel.Normalized();
            Vector2 localDrag = dragDirection * aeroOutput.DragForce; 
            output.DragForce = localDrag;
        }

        // Calculate drive consumption and recharge recovery (based on wheel speed)
        // Pure drive mode, the motor outputs positive torque
        if (powerOutput.Drive > 0)  
        {
            float totalDrivePower =
                driveDist.FrontLeft * tireFL.Config.Radius * tireFL.WheelAngularVel +
                driveDist.FrontRight * tireFR.Config.Radius * tireFR.WheelAngularVel +
                driveDist.RearLeft * tireRL.Config.Radius * tireRL.WheelAngularVel +
                driveDist.RearRight * tireRR.Config.Radius * tireRR.WheelAngularVel;
            Battery.Consume(totalDrivePower, dt);
        }

        // Regen exists
        if (powerOutput.Regen > 0)  
        {
            float totalRegenPower =
                regenDist.FrontLeft * tireFL.Config.Radius * tireFL.WheelAngularVel +
                regenDist.FrontRight * tireFR.Config.Radius * tireFR.WheelAngularVel +
                regenDist.RearLeft * tireRL.Config.Radius * tireRL.WheelAngularVel +
                regenDist.RearRight * tireRR.Config.Radius * tireRR.WheelAngularVel;
            Battery.Regen(totalRegenPower, dt);
        }

        // Rotate the force calculated for the front wheels back into the vehicle's coordinate system
        Vector2 forceFL_Chassis = tireOutputFL.Force.Rotated(steerFL);
        Vector2 forceFR_Chassis = tireOutputFR.Force.Rotated(steerFR);
        Vector2 forceRL_Chassis = tireOutputRL.Force;
        Vector2 forceRR_Chassis = tireOutputRR.Force;

        // Grip of four tires
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
            FrontLeft = tireOutputFL,
            FrontRight = tireOutputFR,
            RearLeft = tireOutputRL,
            RearRight = tireOutputRR
        };

        return output;
    }

}
