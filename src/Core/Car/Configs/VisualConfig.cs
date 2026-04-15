using Godot;
using PloyRacing.Util;

namespace PloyRacing.Core.Car.Configs;

[GlobalClass]
public partial class VisualConfig : Resource
{
    [ExportGroup("Palette (赛车配色)")]
    [Export] public Color BodyColor = Color.FromHtml("#ff5252");     
    [Export] public Color WingColor = Color.FromHtml("#2c3e50");     
    [Export] public Color CockpitColor = Color.FromHtml("#1e272e");  
    [Export] public Color TireColor = Color.FromHtml("#2f3640");     
    [Export] public Color RimColor = Color.FromHtml("#718093");
    [Export] public Color StrutColor = new(0.15f, 0.15f, 0.15f);

    [ExportGroup("Visual Config (视觉微调)")]
    // 轮胎宽度
    [Export] public float TireWidth { get; set; } = 0.35f;
    // 轮胎半径
    [Export] public float TireRadius { get; set; } = 0.33f; 
    // 前翼
    [Export] public float FrontWingDepth { get; set; } = 0.2f;
    [Export] public float FrontWingWidth { get; set; } = 1.2f; 
    // 尾翼
    [Export] public float RearWingDepth { get; set; } = 0.25f;
    [Export] public float RearWingWidth { get; set; } = 1.6f;
    // 连接杆长度
    [Export] public float StrutLength { get; set; } = 0.12f;
    [Export] public float StrutWidth { get; set; } = 0.1f;
    
    
    // 车身收腰系数！
    // 1.0 = 完全不收腰（方盒子）；0.5 = 中段宽度只有车宽的一半（极其修长）
    [Export(PropertyHint.Range, "0.4, 1.0")] 
    public float BodyNarrowFactor { get; set; } = 0.7f; 
    
    // 驾驶舱前后偏移量 (0为重心正中间)
    [Export(PropertyHint.Range, "-1.0, 1.0")] 
    public float CockpitOffset { get; set; } = 0.0f;
}