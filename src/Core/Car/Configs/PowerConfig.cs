using Godot;

namespace PloyRacing.Core.Car.Configs;

[GlobalClass]
public partial class GearConfig : Resource
{
    [Export] public float Multiplier { get; set; }
    [Export] public float MaxSpeed { get; set; }
}

[GlobalClass]
public partial class PowerConfig : Resource
{
    [ExportGroup("Engine Power (引擎动力)")]
    [Export] public float BaseForce { get; set; } = 1500f;
    [Export] public Curve? RPMForceCurve { get; set; }

    [ExportGroup("RPM Settings (转速设定)")]
    [Export] public float IdleRPM { get; set; } = 1000f;
    [Export] public float RedlineRPM { get; set; } = 8000f;

    [ExportGroup("Gearbox Settings (变速箱配置)")]
    [Export] public GearConfig[] GearboxConfig { get; set; } = [
        new GearConfig()
        {
            Multiplier = 3.5f,
            MaxSpeed = 20f
        },
        new GearConfig()
        {
            Multiplier = 2.0f,
            MaxSpeed = 40f
        },
        new GearConfig()
        {
            Multiplier = 1.4f,
            MaxSpeed = 65f
        },
        new GearConfig()
        {
            Multiplier = 1.0f,
            MaxSpeed = 95f
        },
        new GearConfig()
        {
            Multiplier = 0.8f,
            MaxSpeed = 130f
        },
    ];
    [Export(PropertyHint.Range, "2.5, 5.5")] public float FinalDrive { get; set; } = 4.0f;

    public float GetForceRatioAtRPMRatio(float rpmRatio)
    {
        return RPMForceCurve == null ? 1.0f - rpmRatio : RPMForceCurve.Sample(rpmRatio);
    }

    public float InternalFinalDrive => 0.8f + (FinalDrive - 2.5f) / 3.0f * 0.45f;

    public int MaxGear => GearboxConfig.Length;

    public float GetMultiplierAtGear(int gear)
    {
        return GearboxConfig[gear - 1].Multiplier * InternalFinalDrive;
    }

    public float GetMaxSpeedAtGear(int gear)
    {
        return GearboxConfig[gear - 1].MaxSpeed / InternalFinalDrive;
    }
}