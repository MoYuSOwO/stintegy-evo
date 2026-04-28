using System.Collections.Immutable;
using Godot;

namespace StintegyEVO.Core.Car.Configs;

public struct DistributorOutput
{
    public float FrontLeft, FrontRight, RearLeft, RearRight;
    public float PowerLoss;
    public readonly ImmutableArray<float> Tires => [FrontLeft, FrontRight, RearLeft, RearRight];
}

[GlobalClass]
public partial class DistributorConfig : Resource
{
    [Export(PropertyHint.Range, "0.0, 1.0")]
    public float FrontBias { get; set; } = 0.5f; // Front axle drive force ratio (0 = pure rear-wheel drive, 1 = pure front-wheel drive)

    [Export(PropertyHint.Range, "0.0, 1.0")]
    public float VectoringStrength { get; set; } = 0.35f; // Torque vector strength (0 = no vector, 1 = maximum)

    [Export(PropertyHint.Range, "0.0, 1.0")]
    public float FrontVectoringScale { get; set; } = 0.3f; // Front axle vector scaling

    public const float VectoringLossFactor = 0.03f; // Vector loss factor (the proportion of power lost per 1N force difference)
}