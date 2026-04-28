using System.Collections.Immutable;
using Godot;

namespace StintegyEVO.Core.Car.Configs;

public struct CarLoad
{
    public float FrontLeft;
    public float FrontRight;
    public float RearLeft;
    public float RearRight;
    public readonly ImmutableArray<float> Tires => [FrontLeft, FrontRight, RearLeft, RearRight];
}

[GlobalClass]
public partial class SuspensionConfig : Resource
{
    [Export(PropertyHint.Range, "0.1, 0.9")] public float FrontRollBalance { get; set; } = 0.60f;

    [Export] public float DampingSpeed { get; set; } = 10.0f;
}