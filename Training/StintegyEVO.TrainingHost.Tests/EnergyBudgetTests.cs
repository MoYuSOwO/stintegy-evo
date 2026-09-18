using StintegyEVO.Core.Cars;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// The budget re-core (freeze design section 3): target lines by rung,
/// the potential and its shaping, the flag as a true terminal, and the
/// joint (progress, charge) start.
/// </summary>
public sealed class EnergyBudgetTests
{
    [Fact]
    public void TheLinesRunFromFullToEachRungsFinishShare()
    {
        Assert.Equal(1f, EnergyBudget.Target(0f, 1)!.Value, 5);
        Assert.Equal(0.15f, EnergyBudget.Target(1f, 1)!.Value, 5);
        Assert.Equal(0.10f, EnergyBudget.Target(1f, 2)!.Value, 5);
        Assert.Equal(0.09f, EnergyBudget.Target(1f, 3)!.Value, 5);
        Assert.Null(EnergyBudget.Target(0.5f, 4));
        Assert.Null(EnergyBudget.Target(0.5f, 5));
        Assert.Equal(0f, EnergyBudget.Deviation(0.5f, 0.1f, 5));
        Assert.Equal(0f, EnergyBudget.Potential(0.5f, 0.1f, 5, 1000f));
        // Ahead of the line costs nothing; behind it costs lambda per unit.
        Assert.Equal(0f, EnergyBudget.Potential(0.5f, 0.9f, 3, 1000f));
        float target = EnergyBudget.Target(0.5f, 3)!.Value;
        Assert.Equal(-1000f * 0.05f, EnergyBudget.Potential(0.5f, target - 0.05f, 3, 1000f), 2);
    }

    [Fact]
    public void ANominalStartSitsOnTheNormalLine()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true, egoStrategy: new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Normal));
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 1, observation);
        Assert.Equal(0f, environment.BudgetDeviation, 3);
        Assert.Equal(
            environment.BudgetDeviation,
            observation[DirectDriveObservation.ResourceAndBudgetOffset + 4],
            5);
    }

    [Fact]
    public void TheShapingIsGammaPhiNextMinusPhi()
    {
        const float lambda = 1000f;
        const float gamma = 0.99f;
        DirectDriveDuelEnvironment environment = new(
            solo: true,
            egoStrategy: new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Save),
            budgetLambda: lambda,
            budgetGamma: gamma);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 2, observation);
        float phi = Phi(environment, lambda);
        Assert.True(phi < 0f, "a nominal start is under the Save line, so it owes");
        for (int i = 0; i < 60; i++)
        {
            var result = environment.Step(new[] { 0f, 1f }, observation);
            float next = Phi(environment, lambda);
            Assert.Equal(gamma * next - phi, result.BudgetShaping, 3);
            Assert.Equal(result.BudgetShaping, result.GetComponent(11));
            phi = next;
        }
    }

    [Fact]
    public void TheFlagIsATrueTerminal()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true, episodeDurationSeconds: 120f, raceKilometres: 0.1f);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 3, observation);
        TrainingTerminalReason reason = TrainingTerminalReason.None;
        for (int i = 0; i < 1800 && reason == TrainingTerminalReason.None; i++)
            reason = environment.Step(new[] { 0f, 0.5f }, observation).TerminalReason;
        Assert.True(reason == TrainingTerminalReason.Finished, $"ended {reason} at progress {environment.RaceProgress:F3}");
        Assert.True(environment.RaceProgress >= 1f);
    }

    [Fact]
    public void TheRandomisedStartCoversTheProgressAndChargePlane()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true, randomiseEpisodeStart: true,
            egoStrategy: new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Normal));
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        float minProgress = 1f, maxProgress = 0f, minDev = 1f, maxDev = -1f;
        for (long seed = 0; seed < 200; seed++)
        {
            environment.ResetTrack("silverstone", seed, observation);
            minProgress = MathF.Min(minProgress, environment.RaceProgress);
            maxProgress = MathF.Max(maxProgress, environment.RaceProgress);
            minDev = MathF.Min(minDev, environment.BudgetDeviation);
            maxDev = MathF.Max(maxDev, environment.BudgetDeviation);
        }
        Assert.True(minProgress < 0.05f && maxProgress > 0.95f,
            $"progress spans {minProgress:F2}..{maxProgress:F2}");
        Assert.True(minDev < -0.12f && maxDev > 0.07f,
            $"deviation spans {minDev:F3}..{maxDev:F3}");
    }

    private static float Phi(DirectDriveDuelEnvironment environment, float lambda) =>
        EnergyBudget.Potential(
            environment.RaceProgress,
            environment.Ego.State.Energy.Primary,
            environment.EgoStrategy.PowerRung,
            lambda);
}
