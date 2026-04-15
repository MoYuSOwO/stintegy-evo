using Godot;
using PloyRacing.Util;

namespace PloyRacing.Core.Car.Configs;

public enum CarDriveType
{
    RearDrive,
    FrontDrive
}

[GlobalClass]
public partial class CarConfig : Resource
{
    [Export] public string CarName { get; set; } = "Formula 01";
    [Export] public CarDriveType DriveType { get; set; } = CarDriveType.RearDrive;

    [Export] public ChassisConfig Chassis { get; set; } = new();
    [Export] public PowerConfig Power { get; set; } = new();
    [Export] public AeroConfig Aero { get; set; } = new();
    [Export] public TireConfig[] Tires { get; set; } = [
        new(TireType.FrontLeft),
        new(TireType.FrontRight),
        new(TireType.RearLeft),
        new(TireType.RearRight)
    ];
    [Export] public BrakeConfig Brake { get; set; } = new();
    [Export] public DifferentialConfig Differential { get; set; } = new();
    [Export] public FuelConfig Fuel { get; set; } = new();
    [Export] public SuspensionConfig Suspension { get; set; } = new();
    [Export] public VisualConfig Visual { get; set; } = new();

    [Export(PropertyHint.Range, "0.3, 0.8")] public float InitBiasFront { get; set; } = 0.6f;
    [Export] public float InitFuelL { get; set; } = 10f;
    [Export] public bool IsGrid { get; set; } = true;
    [Export] public int CarGridOrPitIdx { get; set; } = 1;

    public CarLoad InitLoad
    {
        get
        {
            float totalWeight = Chassis.DryMass * GeomUtil.g;
            float weightFront = totalWeight * Chassis.WeightDistFront;
            float weightRear = totalWeight * (1.0f - Chassis.WeightDistFront);
            return new()
            {
                FrontLeft = weightFront / 2f,
                FrontRight = weightFront / 2f,
                RearLeft = weightRear / 2f,
                RearRight = weightRear / 2f
            };
        }
    }
}