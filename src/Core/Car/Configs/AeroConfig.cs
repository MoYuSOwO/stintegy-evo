using Godot;

namespace PloyRacing.Core.Car.Configs;

public struct AeroOutput
{
    public float DragForce;       // 向后的阻力
    public float DownforceFront;  // 压在前轴的下压力
    public float DownforceRear;   // 压在后轴的下压力
}

[GlobalClass]
public partial class AeroConfig : Resource
{
    [ExportGroup("Drag (阻力)")]
    [Export] public float BaseDragCoef { get; set; } = 0.8f; // 基础风阻
    [Export] public float FrontalArea { get; set; } = 1.5f;  // 迎风面积

    [ExportGroup("Downforce (下压力)")]
    [Export] public float DownforceCoef { get; set; } = 2.5f; // 下压力系数 (蒙扎调小，摩纳哥调大)
    [Export] public float AeroBalanceFront { get; set; } = 0.45f; // 45% 下压力给前轮，影响高速弯推头还是甩尾
    
    // 空气密度常数
    public const float AirDensity = 1.225f; 
}