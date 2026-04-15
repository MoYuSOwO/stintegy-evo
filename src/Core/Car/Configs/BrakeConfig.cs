using Godot;

namespace PloyRacing.Core.Car.Configs;

public struct CarBrakeForce
{
    public float FrontLeft;
    public float FrontRight;
    public float RearLeft;
    public float RearRight;
}

public struct CarBrakeTemp
{
    public float FrontLeft;
    public float FrontRight;
    public float RearLeft;
    public float RearRight;

    public readonly bool IsFailure => (
        FrontLeft >= BrakeConfig.AbsoluteMaxTemp ||
        FrontRight >= BrakeConfig.AbsoluteMaxTemp ||
        RearLeft >= BrakeConfig.AbsoluteMaxTemp ||
        RearRight >= BrakeConfig.AbsoluteMaxTemp
    );
}

[GlobalClass]
public partial class BrakeConfig : Resource
{
    [ExportGroup("Mechanical (刹车基础)")]
    [Export] public float MaxBrakeForce { get; set; } = 15000f;
    public const float MaxBiasFront = 0.8f;
    public const float MinBiasFront = 0.3f;
    public const float AbsoluteMaxTemp = 1200f; 
    public const float FatalFailureEfficiency = 0.05f;

    [ExportGroup("Thermodynamics (热力学)")]
    [Export] public float SpecificHeat { get; set; } = 800f;
    [Export] public float CoolingRateBase { get; set; } = 15f;
    [Export] public float CoolingRateAir { get; set; } = 0.5f;

    [ExportGroup("Fade Window (工作温度窗口)")]
    [Export] public float OptimalMinTemp { get; set; } = 300f;
    [Export] public float OptimalMaxTemp { get; set; } = 800f;
    [Export] public float ColdEfficiency { get; set; } = 0.6f;
    [Export] public float OverheatEfficiency { get; set; } = 0.2f;
}