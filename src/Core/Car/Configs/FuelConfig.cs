using Godot;

namespace PloyRacing.Core.Car.Configs;

[GlobalClass]
public partial class FuelConfig : Resource
{
    [Export] public float CapacityL { get; set; } = 60f; // 油箱容量 (升)
    [Export] public float BasicBurnRate { get; set; } = 0.05f; // 怠速/基础油耗 (升/秒)
    [Export] public float MaxBurnRate { get; set; } = 1.2f; // 满负荷最大额外油耗 (升/秒)
    
    public const float FuelDensity = 0.75f;
}