using Godot;

namespace PloyRacing.Core.Car.Configs;

public struct TireOutput
{
    public Vector2 Force;       // 最终施加给地面的真实力
    public bool IsSliding;      // 是否正在打滑（横向或纵向）
    public bool IsLockedUp;     // 是否处于刹车抱死状态（用于触发 UI 报警或胎烟特效）
}

public enum TireType
{
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight
}

public static class TireTypeExtentions
{
    public static bool IsFront(this TireType type)
    {
        return type == TireType.FrontLeft || type == TireType.FrontRight;
    }
}

[GlobalClass]
public partial class TireConfig(TireType tireType) : Resource
{
    [ExportGroup("Base Properties")]
    [Export] public TireType Type { get; set; } = tireType;
    [Export] public float Radius { get; set; } = 0.33f;
    [Export] public float Mass { get; set; } = 10.0f;
    [Export] public float DefaultRollingResCoef { get; set; } = 0.015f;

    [ExportGroup("Lateral Pacejka")]
    [Export] public float LatStiffness { get; set; } = 15.0f;
    public const float LatShape = 1.30f;
    [Export] public float LatDrop { get; set; } = 0.5f;
    [Export] public float LatPeakFriction { get; set; } = 1.0f;

    [ExportGroup("Longitudinal Fit Curve")]
    public const float LongOptimalSlipRatio = 1.15f; 
    [Export] public float LongPeakFriction { get; set; } = 1.1f;
    public const float LongSlideMultiplier = 0.8f;

    [ExportGroup("Thermodynamics")]
    public const float SpecificHeat = 400.0f;
    public const float SurfaceMassRatio = 0.2f;
    [Export] public float OptimalMinTemp { get; set; } = 70.0f;
    [Export] public float OptimalMaxTemp { get; set; } = 100.0f;
    public const float ColdSensitivity = 0.005f;
    public const float HotSensitivity = 0.0002f;
    public const float InternalHeatTransCoef = 0.5f;
    public const float BaseCoolingCoef = 10.0f;
    public const float AirCoolingCoef = 0.5f;
    public const float MaxTempRiseRate = 50.0f;
    public const float WastedForceToSlipVCoef = 0.005f;
    public const float StartingBurnoutBaseSlipV = 5.0f;

    [ExportGroup("Pressure & Wear")]
    [Export(PropertyHint.Range, "1.0, 3.0")] public float OptimalMinPressure { get; set; } = 1.7f;
    [Export(PropertyHint.Range, "1.0, 3.0")] public float OptimalMaxPressure { get; set; } = 2.2f;
    [Export(PropertyHint.Range, "1.0, 3.0")] public float InitPressure { get; set; } = 2.0f;
    public const float AbsoluteMinPressure = 1.0f;
    public const float AbsoluteMaxPressure = 3.0f;
    public const float LowPressureSensitivity = 0.2f;
    public const float HighPressureSensitivity = 0.5f;
    public const float StiffnessSensitivity = 0.1f;
    [Export] public float WearCoef { get; set; } = 0.000001f;
    public const float AblationCoef = 5.0f;
    public const float ThermalCoef = 0.01f;
}