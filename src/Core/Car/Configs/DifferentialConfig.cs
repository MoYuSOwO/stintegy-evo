using Godot;

namespace PloyRacing.Core.Car.Configs;

public struct DifferentialOutput
{
    public float ForceLeft;
    public float ForceRight;
    public float YawPenalty; 
}

[GlobalClass]
public partial class DifferentialConfig : Resource
{
    [Export(PropertyHint.Range, "0.0, 1.0")] 
    public float PowerRamp { get; set; } = 0.6f; // 踩油门时的锁止率
    
    [Export(PropertyHint.Range, "0.0, 1.0")] 
    public float CoastRamp { get; set; } = 0.3f; // 松油门/刹车时的锁止率
    
    [Export] public float LockedStiffness { get; set; } = 60000f; // 完全锁死时的推头惩罚系数
}