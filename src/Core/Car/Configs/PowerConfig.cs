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
    [Export] public Curve RPMForceCurve { get; set; } = GetDefualtCurve();

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
        return RPMForceCurve.Sample(rpmRatio);
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

    private static Curve? defaultCurve;

    private static Curve GetDefualtCurve()
    {
        if (defaultCurve != null) return defaultCurve;
        defaultCurve = new();
        defaultCurve.AddPoint(
            position: new Vector2(0.0f, 0.3f),
            leftTangent: 0, 
            rightTangent: 1.0f,
            leftMode: Curve.TangentMode.Linear,
            rightMode: Curve.TangentMode.Free
        );
        defaultCurve.AddPoint(
            position: new Vector2(0.4f, 0.65f),
            leftTangent: 1.0f,
            rightTangent: 1.5f,
            leftMode: Curve.TangentMode.Free,
            rightMode: Curve.TangentMode.Free
        );
        defaultCurve.AddPoint(
            position: new Vector2(0.8f, 1.0f),
            leftTangent: 0.5f,
            rightTangent: -2.0f,
            leftMode: Curve.TangentMode.Free,
            rightMode: Curve.TangentMode.Free
        );
        defaultCurve.AddPoint(
            position: new Vector2(1.0f, 0.5f),
            leftTangent: -2.0f,
            rightTangent: 0,
            leftMode: Curve.TangentMode.Free,
            rightMode: Curve.TangentMode.Linear
        );
        defaultCurve.BakeResolution = 100;
        defaultCurve.Bake();
        return defaultCurve;
    }
}