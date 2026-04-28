using Godot;

namespace PloyRacing.Core.Car.Configs;

public struct PowerOutput
{
    public float Drive;
    public float Regen;
}

[GlobalClass]
public partial class PowerConfig : Resource
{
    [Export] public float MaxDriveForce { get; set; } = 7500f;

    [Export] public float MaxPower { get; set; } = 250000f;

    [Export] public float MaxRegenForce { get; set; } = 4000f;

    [Export] public float MaxBrakeForce { get; set; } = 15000f;

    [Export] public float BaseRegenForce { get; set; } = 200f;

    public float BaseSpeed => MaxPower / MaxDriveForce;

    public float CalcMaxDriveForceAtSpeed(float speed)
    {
        if (speed < BaseSpeed) return MaxDriveForce;
        return MaxPower / speed;
    }

    public const float InputDeadZone = 0.02f;
}