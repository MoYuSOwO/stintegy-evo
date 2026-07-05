using GdUnit4;
using StintegyEVO.Core.Car.Components;
using StintegyEVO.Core.Car.Configs;
using static GdUnit4.Assertions;

namespace StintegyEVO.Tests.Components;

[TestSuite]
public sealed class DistributorComponentTests
{
    [TestCase]
    [RequireGodotRuntime]
    public void PositiveSteerCreatesPositiveDriveYawAndOpposingBrakeYaw()
    {
        DistributorComponent distributor = new(new DistributorConfig
        {
            FrontBias = 0.5f,
            VectoringStrength = 0.4f,
            FrontVectoringScale = 1.0f
        });

        DistributorOutput drive = distributor.CalculateDistributeForce(1000.0f, steerInput: 0.5f, carSpeed: 20.0f);
        DistributorOutput brake = distributor.CalculateDistributeForce(-1000.0f, steerInput: 0.5f, carSpeed: 20.0f);

        AssertThat(CalculateYawMomentProxy(drive)).IsGreater(0.0f);
        AssertThat(CalculateYawMomentProxy(brake)).IsLess(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void NegativeSteerCreatesNegativeDriveYawAndOpposingBrakeYaw()
    {
        DistributorComponent distributor = new(new DistributorConfig
        {
            FrontBias = 0.5f,
            VectoringStrength = 0.4f,
            FrontVectoringScale = 1.0f
        });

        DistributorOutput drive = distributor.CalculateDistributeForce(1000.0f, steerInput: -0.5f, carSpeed: 20.0f);
        DistributorOutput brake = distributor.CalculateDistributeForce(-1000.0f, steerInput: -0.5f, carSpeed: 20.0f);

        AssertThat(CalculateYawMomentProxy(drive)).IsLess(0.0f);
        AssertThat(CalculateYawMomentProxy(brake)).IsGreater(0.0f);
    }

    private static float CalculateYawMomentProxy(DistributorOutput output)
    {
        float leftForce = output.FrontLeft + output.RearLeft;
        float rightForce = output.FrontRight + output.RearRight;
        return leftForce - rightForce;
    }
}
