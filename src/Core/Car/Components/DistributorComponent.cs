using Godot;
using StintegyEVO.Core.Car.Configs;

namespace StintegyEVO.Core.Car.Components;

public class DistributorComponent(DistributorConfig config)
{
    public readonly DistributorConfig Config = config;

    public DistributorOutput CalculateDistributeForce(float totalForce, float steerInput, float carSpeed)
    {
        DistributorOutput output = new();
        // 1. Front and rear axle distribution
        float frontForce = totalForce * Config.FrontBias;
        float rearForce = totalForce * (1f - Config.FrontBias);

        // 2. Torque vector offset (determined by steering wheel angle and strength in config)
        float vectoringOffset = Mathf.Clamp(steerInput, -1f, 1f) * Config.VectoringStrength;

        // Rear axle left and right distribution
        float rlBias = Mathf.Clamp(0.5f + vectoringOffset * 0.5f, 0f, 1f);
        float rrBias = 1f - rlBias;

        // Front axle left and right distribution (with scaling)
        float frontVectoringOffset = vectoringOffset * Config.FrontVectoringScale;
        float flBias = Mathf.Clamp(0.5f + frontVectoringOffset * 0.5f, 0f, 1f);
        float frBias = 1f - flBias;

        // 3. Final output
        output.FrontLeft = frontForce * flBias;
        output.FrontRight = frontForce * frBias;
        output.RearLeft = rearForce * rlBias;
        output.RearRight = rearForce * rrBias;

        float forceDiff = Mathf.Abs(output.RearLeft - output.RearRight) + Mathf.Abs(output.FrontLeft - output.FrontRight);
        output.PowerLoss = forceDiff * DistributorConfig.VectoringLossFactor * carSpeed;

        return output;
    }
}