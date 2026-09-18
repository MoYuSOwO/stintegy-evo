using StintegyEVO.Core.Cars;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// The world-v3 observation contract, pinned (freeze design, section 4):
/// the blocks add up and sit where the Python side reads them, absence is
/// written on capacity, the heading pair is a unit vector, and every
/// channel stays finite and O(1) on real driving, since nothing is
/// statistically normalised.
/// </summary>
public sealed class ObservationLayoutTests
{
    [Fact]
    public void TheBlocksAddUpToTheContract()
    {
        Assert.Equal(198, DirectDriveObservation.GeometryPointCount *
                          DirectDriveObservation.GeometryFloatsPerPoint);
        Assert.Equal(198, DirectDriveObservation.TireAndBatteryOffset);
        Assert.Equal(215, DirectDriveObservation.ModeOffset);
        Assert.Equal(216, DirectDriveObservation.AeroOffset);
        Assert.Equal(219, DirectDriveObservation.RoadAndLimitsOffset);
        Assert.Equal(232, DirectDriveObservation.ResourceAndBudgetOffset);
        Assert.Equal(5, DirectDriveObservation.ResourceAndBudgetSize);
        Assert.Equal(237, DirectDriveObservation.EgoOffset);
        Assert.Equal(251, DirectDriveObservation.OpponentOffset);
        Assert.Equal(347, DirectDriveObservation.PreviousDynamicOffset);
        Assert.Equal(110, DirectDriveObservation.DynamicBlockSize);
        Assert.Equal(457, DirectDriveObservation.ObservationSize);
    }

    [Fact]
    public void ABatteryCarWritesItsMissingSecondStoreAsZeroCapacity()
    {
        float[] observation = Reset("silverstone");
        int slots = DirectDriveObservation.ResourceAndBudgetOffset;

        Assert.InRange(observation[slots], 0.01f, 1f);                  // charge
        Assert.Equal(1f, observation[slots + 1], 4);                     // 1100 MJ
        Assert.Equal(0f, observation[slots + 2]);                        // absent
        Assert.Equal(0f, observation[slots + 3]);                        // absent
    }

    [Theory]
    [InlineData("silverstone")]
    [InlineData("monaco")]
    [InlineData("banked-sweeper")]
    public void ZeroInputCoastingStaysFiniteAndOrderOne(string track)
    {
        AssertDriving(track, new[] { 0f, 0f });
    }

    [Theory]
    [InlineData("silverstone")]
    [InlineData("daytona")]
    public void AConstantCommandStaysFiniteAndOrderOne(string track)
    {
        AssertDriving(track, new[] { 0.05f, 0.6f });
    }

    [Fact]
    public void TheLimiterChannelIsAShareOfFullDrive()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true,
            egoStrategy: new CarStrategy(TireUsageMode.Protect, PowerOutputMode.Attack)
        );
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 11, observation);
        float largest = 0f;
        for (int i = 0; i < 300; i++)
        {
            environment.Step(new[] { 0.6f, 1f }, observation);
            float channel = observation[DirectDriveObservation.EgoOffset + 13];
            float expected = environment.Ego.State.Telemetry.CombinedGripLimiterCutAccel /
                             environment.Ego.CarConfig.MaxDriveAcceleration;
            Assert.Equal(expected, channel, 4);
            largest = MathF.Max(largest, channel);
            if (environment.IsTerminal)
                break;
        }
        Assert.True(largest > 0f, "the limiter never cut, so nothing was checked");
    }

    /// <summary>
    /// The same per-channel bounds as train.OBSERVATION_BOUNDS: 3, except the
    /// tyre loads (6), and the ego's yaw rate (10) and sideslip (2 pi / 0.5)
    /// in the current and previous frame.
    /// </summary>
    private static float Bound(int channel)
    {
        int tyre = DirectDriveObservation.TireAndBatteryOffset;
        if (channel == tyre + 3 || channel == tyre + 7 || channel == tyre + 11 || channel == tyre + 15)
            return 6f;
        foreach (int ego in new[] { DirectDriveObservation.EgoOffset, DirectDriveObservation.PreviousDynamicOffset })
        {
            if (channel == ego + 3) return 10f;
            if (channel == ego + 4) return 2f * MathF.PI / 0.5f + 1e-3f;
        }
        return 3f;
    }

    private static float[] Reset(string track)
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack(track, 3, observation);
        return observation;
    }

    private static void AssertDriving(string track, float[] action)
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack(track, 5, observation);
        int sin = DirectDriveObservation.EgoOffset + 5;
        int cos = DirectDriveObservation.EgoOffset + 6;
        for (int i = 0; i < 450; i++)
        {
            for (int c = 0; c < observation.Length; c++)
            {
                Assert.True(float.IsFinite(observation[c]), $"channel {c} at step {i}");
                Assert.InRange(MathF.Abs(observation[c]), 0f, Bound(c));
            }
            float unit = observation[sin] * observation[sin] +
                         observation[cos] * observation[cos];
            Assert.Equal(1f, unit, 3);
            environment.Step(action, observation);
            if (environment.IsTerminal)
                break;
        }
    }
}
