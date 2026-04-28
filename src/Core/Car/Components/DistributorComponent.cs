using Godot;
using PloyRacing.Core.Car.Configs;

namespace PloyRacing.Core.Car.Components;

public class DistributorComponent(DistributorConfig config)
{
    public readonly DistributorConfig Config = config;

    public DistributorOutput CalculateDistributeForce(float totalForce, float steerInput, float carSpeed)
    {
        DistributorOutput output = new();
        // 1. 前后轴分配
        float frontForce = totalForce * Config.FrontBias;
        float rearForce = totalForce * (1f - Config.FrontBias);

        // 2. 扭矩矢量偏置量 (由方向盘角度和强度决定)
        float vectoringOffset = Mathf.Clamp(steerInput, -1f, 1f) * Config.VectoringStrength;

        // 后轴左右分配 (矢量主战场)
        float rlBias = Mathf.Clamp(0.5f + vectoringOffset * 0.5f, 0f, 1f);
        float rrBias = 1f - rlBias;

        // 前轴左右分配 (带缩放)
        float frontVectoringOffset = vectoringOffset * Config.FrontVectoringScale;
        float flBias = Mathf.Clamp(0.5f + frontVectoringOffset * 0.5f, 0f, 1f);
        float frBias = 1f - flBias;

        // 3. 最终输出
        output.FrontLeft = frontForce * flBias;
        output.FrontRight = frontForce * frBias;
        output.RearLeft = rearForce * rlBias;
        output.RearRight = rearForce * rrBias;

        float forceDiff = Mathf.Abs(output.RearLeft - output.RearRight) + Mathf.Abs(output.FrontLeft - output.FrontRight);
        output.PowerLoss = forceDiff * DistributorConfig.VectoringLossFactor * carSpeed;

        return output;
    }
}