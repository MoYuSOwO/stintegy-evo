using Godot;

namespace PloyRacing.Core.Car.Configs;

[GlobalClass]
public partial class ChassisConfig : Resource
{
    [Export] public float DryMass { get; set; } = 1400.0f;
    [Export] public float DryI { get; set; } = 2800.0f;
    [Export] public float MaxSteerAngle { get; set; } = Mathf.DegToRad(35);
    [Export] public float CgHeight { get; set; } = 0.35f;
    [Export] public float WheelBase { get; set; } = 2.6f;
    [Export] public float Width { get; set; } = 1.85f;
    [Export] public float Length { get; set; } = 4.4f;
    [Export(PropertyHint.Range, "0.3, 0.7")] public float WeightDistFront { get; set; } = 0.55f;

    public float TrackWidth => Width;
    public float HalfTrackWidth => TrackWidth / 2.0f;
    public float HalfWheelBase => WheelBase / 2.0f;
}