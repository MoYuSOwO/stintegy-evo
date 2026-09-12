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

    /// <summary>
    /// The price curve, pinned at the three points it was calibrated on.
    ///
    /// The audit that set it measured what a unit of excess buys in the
    /// step it is spent — same station, over-the-allowance steps against
    /// compliant ones — and got at most +0.032 of progress against the
    /// 0.074 a compliant step earns. The hinge charges three times that
    /// for the first unit, so the first epsilon past the line is a loss on
    /// the ledger that can actually be measured, and a square term takes
    /// over towards the edge of the circle, where every spin in the audit
    /// was sitting in the second before it began.
    ///
    /// Quoted per step at fifteen decisions a second, which is the rate
    /// the calibration was done at. The constants themselves are per
    /// second, so the same curve holds at any rate.
    /// </summary>
    [Fact]
    public void TheExcessPriceIsAHingeCalibratedAtThreePoints()
    {
        const float step = DirectDriveDuelEnvironment.DefaultAgentStepSeconds;
        // Normal allots 97.7% of the circle and usage saturates at one, so
        // this is the whole excess the car can reach in this mode.
        const float wholeCircle = 1f - 0.977f;

        // Obedience is free, and not nearly free.
        Assert.Equal(0f, DirectDriveDuelEnvironment.ModeExcessPrice(-0.05f));
        Assert.Equal(0f, DirectDriveDuelEnvironment.ModeExcessPrice(0f));

        // The first unit past the line, read as a slope rather than
        // asserted as a constant, so the test still means this if the
        // curve is ever written differently.
        float epsilon = 1e-5f;
        float slope =
            DirectDriveDuelEnvironment.ModeExcessPrice(epsilon) / epsilon * step;
        Assert.InRange(slope, 0.096f, 0.12f);

        // And the edge of the circle: felt every step, not drowning the
        // progress the car is paid for.
        float atTheEdge =
            DirectDriveDuelEnvironment.ModeExcessPrice(wholeCircle) * step;
        Assert.InRange(atTheEdge, 0.004f, 0.006f);

        // Convex, so that the edge costs more per unit than the first
        // step past the line does.
        float half = DirectDriveDuelEnvironment.ModeExcessPrice(
            wholeCircle * 0.5f
        );
        Assert.True(
            DirectDriveDuelEnvironment.ModeExcessPrice(wholeCircle) >
            2f * half,
            "the price has to bend upwards, or it is the linear one again"
        );
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
