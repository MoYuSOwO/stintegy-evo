using Godot;

namespace StintegyEVO.Core.Car.Configs;

public struct TireOutput
{
    public Vector2 Force;
    public float SlipRatio;
    public float SlipAngle;
    public bool IsSliding;
    public bool IsLockedUp;
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
    [Export] public bool IsSteer { get; set; } = tireType.IsFront();
    [Export] public float Radius { get; set; } = 0.33f;
    [Export] public float Mass { get; set; } = 12.0f;
    [Export] public float Inertia { get; set; } = 1.0f;
    [Export] public float DefaultRollingResCoef { get; set; } = 0.015f;

    [ExportGroup("Lateral Pacejka")]
    [Export] public float LatStiffness { get; set; } = 15.0f;
    public const float LatShape = 1.30f;
    [Export] public float LatDrop { get; set; } = 0.8f;
    [Export] public float LatPeakFriction { get; set; } = 1.3f;

    [ExportGroup("Longitudinal Pacejka")]
    [Export] public float LongStiffness { get; set; } = 10.0f;
    public const float LongShape = 1.65f;
    [Export] public float LongDrop { get; set; } = 0.4f;
    [Export] public float LongPeakFriction { get; set; } = 1.5f;

    // Low speed tire model
    public const float MinStableSlipSpeed = 1.0f;
    public const float FullPacejkaSpeed = 3.0f;
    public const float LowSpeedLongSlipStiffness = 0.6f;
    public const float LowSpeedLatDamping = 3.0f;

    // Force relaxation
    public const float LongRelaxationLength = 0.12f;
    public const float LatRelaxationLength = 0.5f;

    // Wheel stop guard
    public const float BrakeLockAngularVel = 0.5f;
    public const float WheelStopDeadbandAngularVel = 0.1f;

    [ExportGroup("Thermodynamics")]
    public const float SpecificHeat = 400.0f;
    public const float SurfaceMassRatio = 0.2f;
    [Export] public float OptimalMinTemp { get; set; } = 60.0f;
    [Export] public float OptimalMaxTemp { get; set; } = 110.0f;
    [Export] public float ThermalWearCoef { get; set; } = 0.005f;
    public const float ColdSensitivity = 0.005f;
    public const float HotSensitivity = 0.0002f;
    public const float InternalHeatTransCoef = 0.5f;
    public const float BaseCoolingCoef = 10.0f;
    public const float AirCoolingCoef = 0.5f;

    [ExportGroup("Pressure & Wear")]
    [Export(PropertyHint.Range, "1.0, 3.0")] public float OptimalMinPressure { get; set; } = 1.7f;
    [Export(PropertyHint.Range, "1.0, 3.0")] public float OptimalMaxPressure { get; set; } = 2.2f;
    [Export(PropertyHint.Range, "1.0, 3.0")] public float InitPressure { get; set; } = 2.0f;
    public const float AbsoluteMinPressure = 1.0f;
    public const float AbsoluteMaxPressure = 3.0f;
    public const float LowPressureSensitivity = 0.2f;
    public const float HighPressureSensitivity = 0.5f;
    public const float StiffnessSensitivity = 0.1f;
    [Export] public float BaseLongWearRate { get; set; } = 0.0000005f;
    [Export] public float BaseLatWearRate { get; set; } = 0.0000012f;
}
