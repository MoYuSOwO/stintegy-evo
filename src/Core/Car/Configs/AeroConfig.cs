using Godot;

namespace StintegyEVO.Core.Car.Configs;

public struct AeroOutput
{
    public float DragForce;
    public float DownforceFront;
    public float DownforceRear;
}

[GlobalClass]
public partial class AeroConfig : Resource
{
    [ExportGroup("Drag")]
    [Export] public float BaseDragCoef { get; set; } = 0.3f; // Basic wind resistance
    [Export] public float FrontalArea { get; set; } = 2.0f;  // Windward area

    [ExportGroup("Downforce")]
    [Export] public float DownforceCoef { get; set; } = 0.5f;
    [Export] public float AeroBalanceFront { get; set; } = 0.45f; // Represents how much downforce is applied to the front wheels.
    
    public const float AirDensity = 1.225f; 
}