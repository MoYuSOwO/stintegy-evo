using System.Collections.Immutable;
using Godot;

namespace PloyRacing.Core.Car.Configs;

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
    public float FrontBias { get; set; } = 0.5f; // 前轴驱动力占比 (0=纯后驱，1=纯前驱)

    [Export(PropertyHint.Range, "0.0, 1.0")]
    public float VectoringStrength { get; set; } = 0.35f; // 扭矩矢量强度 (0=无矢量，1=最大)

    [Export(PropertyHint.Range, "0.0, 1.0")]
    public float FrontVectoringScale { get; set; } = 0.3f; // 前轴矢量缩放 (通常比后轴弱)

    public const float VectoringLossFactor = 0.03f; // 矢量损耗系数 (每1N力差损耗的功率比例)
}