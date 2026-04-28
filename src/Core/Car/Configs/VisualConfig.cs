using Godot;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Configs;

[GlobalClass]
public partial class VisualConfig : Resource
{
    [ExportGroup("Palette")]
    [Export] public Color BodyColor = Color.FromHtml("#ff5252");     
    [Export] public Color WingColor = Color.FromHtml("#2c3e50");     
    [Export] public Color CockpitColor = Color.FromHtml("#1e272e");  
    [Export] public Color TireColor = Color.FromHtml("#2f3640");     
    [Export] public Color RimColor = Color.FromHtml("#718093");
    [Export] public Color StrutColor = new(0.15f, 0.15f, 0.15f);

    [ExportGroup("Visual Config")]
    [Export] public float TireWidth { get; set; } = 0.35f;
    [Export] public float TireRadius { get; set; } = 0.33f; 
    [Export] public float FrontWingDepth { get; set; } = 0.2f;
    [Export] public float FrontWingWidth { get; set; } = 1.2f; 
    [Export] public float RearWingDepth { get; set; } = 0.25f;
    [Export] public float RearWingWidth { get; set; } = 1.6f;
    [Export] public float StrutLength { get; set; } = 0.12f;
    [Export] public float StrutWidth { get; set; } = 0.1f;
    
    
    // Body width ratio
    // 1.0 = No width reduction at all (boxy)
    // 0.5 = Mid-section width is only half the width of the car (extremely long and sleek)
    [Export(PropertyHint.Range, "0.4, 1.0")] 
    public float BodyNarrowFactor { get; set; } = 0.7f; 
    
    // Forward and backward offset of the cockpit (0 represents the center of gravity being exactly in the middle).
    [Export(PropertyHint.Range, "-1.0, 1.0")] 
    public float CockpitOffset { get; set; } = 0.0f;
}