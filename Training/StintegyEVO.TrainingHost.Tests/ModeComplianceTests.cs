using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers.Learned;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// Pins the pit wall's authority over the learned driver. Output modes are
/// capped by the powertrain, but nothing in the physics stops a car from
/// spending more grip than its tire mode allots, so the steward penalty is
/// the only thing that makes the strategy game's instruction real.
/// </summary>
public sealed class ModeComplianceTests
{
    [Fact]
    public void EveryTireModeIsDrawnAcrossEpisodes()
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation =
            new float[DirectDriveObservation.ObservationSize];
        HashSet<TireUsageMode> seen = [];

        for (int seed = 0; seed < 60; seed++)
        {
            environment.ResetTrack("speedway", seed, observation);
            seen.Add(environment.EgoStrategy.TireMode);
        }

        Assert.Equal(5, seen.Count);
    }

    [Fact]
    public void TireModeReachesTheObservation()
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation =
            new float[DirectDriveObservation.ObservationSize];
        for (int seed = 0; seed < 60; seed++)
        {
            environment.ResetTrack("speedway", seed, observation);
            // The tyre setting reaches the driver as the share of grip it is
            // allowed to spend, in the road and limits block. It used to have
            // a channel of its own in the mode block; that channel was
            // removed as a quantity the car could already work out, and this
            // test went on reading the slot rather than the number.
            float expected = new TireConfig()
                .GetAccelerationUsage(environment.EgoStrategy.TireMode);
            Assert.Equal(
                expected,
                observation[DirectDriveObservation.RoadAndLimitsOffset + 8],
                3
            );
        }
    }

    [Fact]
    public void AttackIsNeverPenalizedWhileProtectIsWhenDrivenHard()
    {
        float attackExcess = DriveHardAndSumExcess(TireUsageMode.Attack);
        float protectExcess = DriveHardAndSumExcess(TireUsageMode.Protect);

        Assert.Equal(0f, attackExcess);
        Assert.True(
            protectExcess < 0f,
            "driving at the limit under Protect should be stewarded"
        );
    }

    /// <summary>
    /// Runs the same hard-cornering input under a chosen tire mode by
    /// re-seeding until the environment draws that mode, so the comparison
    /// isolates the instruction rather than the scenario.
    /// </summary>
    private static float DriveHardAndSumExcess(TireUsageMode mode)
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation =
            new float[DirectDriveObservation.ObservationSize];
        for (int seed = 0; seed < 400; seed++)
        {
            environment.ResetScenario(
                "simple-right",
                seed,
                egoStartS: 600f,
                forwardGapMeters: 20f,
                startSpeedMetersPerSecond: 45f,
                observation
            );
            if (environment.EgoStrategy.TireMode != mode)
                continue;

            float total = 0f;
            float[] action = [0.85f, 0.2f];
            for (int step = 0; step < 60 && !environment.IsTerminal; step++)
                total += environment.Step(action, observation).ModeExcessPenalty;
            return total;
        }

        throw new InvalidOperationException(
            $"No seed produced tire mode {mode}."
        );
    }
}
