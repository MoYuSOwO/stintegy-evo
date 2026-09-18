using StintegyEVO.Core.Cars;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// The budget re-core with anchored lines (freeze design section 3, user
/// ruling 2026-09-19): each rung is a spending slope, anchored where the
/// instruction was given; the same rung asks for the same driving whatever
/// came before; changing rung earns nothing by itself and collects no old
/// debt; the flag is a true terminal.
/// </summary>
public sealed class EnergyBudgetTests
{
    [Fact]
    public void EachRungIsASlopeFromItsAnchor()
    {
        var anchor = new EnergyBudget.Anchor(0.3f, 0.6f);
        Assert.Equal(0.6f, EnergyBudget.Target(anchor, 0.3f, 3)!.Value, 5);
        // Normal: 91% of the pack over a whole race.
        Assert.Equal(0.6f - 0.91f * 0.2f, EnergyBudget.Target(anchor, 0.5f, 3)!.Value, 5);
        Assert.Equal(0.6f - 0.85f * 0.2f, EnergyBudget.Target(anchor, 0.5f, 1)!.Value, 5);
        Assert.Equal(0.6f - 0.90f * 0.2f, EnergyBudget.Target(anchor, 0.5f, 2)!.Value, 5);
        Assert.Null(EnergyBudget.Target(anchor, 0.5f, 4));
        Assert.Null(EnergyBudget.Target(anchor, 0.5f, 5));
        Assert.Equal(0f, EnergyBudget.Potential(anchor, 0.5f, 0.1f, 5, 1000f));
        float target = EnergyBudget.Target(anchor, 0.5f, 3)!.Value;
        Assert.Equal(0f, EnergyBudget.Potential(anchor, 0.5f, target + 0.01f, 3, 1000f));
        Assert.Equal(-1000f * 0.05f, EnergyBudget.Potential(anchor, 0.5f, target - 0.05f, 3, 1000f), 2);
    }

    /// <summary>
    /// The same rung asks for the same driving wherever the episode begins:
    /// every start, however low its charge, begins on its own line, owing
    /// nothing. (The first implementation drew one line from full at the
    /// start, and six in ten starts began under it with a debt to repay.)
    /// </summary>
    [Fact]
    public void EveryStartBeginsOnItsOwnLineOwingNothing()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true, randomiseEpisodeStart: true,
            egoStrategy: new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Save));
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        float minCharge = 1f, maxCharge = 0f;
        for (long seed = 0; seed < 100; seed++)
        {
            environment.ResetTrack("silverstone", seed, observation);
            Assert.Equal(0f, environment.BudgetDeviation, 5);
            Assert.Equal(0f, observation[DirectDriveObservation.ResourceAndBudgetOffset + 4], 5);
            Assert.Equal(environment.RaceProgress, environment.BudgetAnchor.Progress, 5);
            minCharge = MathF.Min(minCharge, environment.Ego.State.Energy.Primary);
            maxCharge = MathF.Max(maxCharge, environment.Ego.State.Energy.Primary);
        }
        Assert.True(maxCharge - minCharge > 0.5f, $"starts span {minCharge:F2}..{maxCharge:F2}");
    }

    [Fact]
    public void TheShapingIsPhiNextMinusPhi()
    {
        const float lambda = 1000f;
        DirectDriveDuelEnvironment environment = new(
            solo: true,
            egoStrategy: new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Save),
            budgetLambda: lambda);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 2, observation);
        float phi = Phi(environment, lambda);
        Assert.Equal(0f, phi);
        float total = 0f;
        for (int i = 0; i < 120; i++)
        {
            var result = environment.Step(new[] { 0f, 1f }, observation);
            float next = Phi(environment, lambda);
            Assert.Equal(next - phi, result.BudgetShaping, 3);
            Assert.Equal(result.BudgetShaping, result.GetComponent(11));
            total += result.BudgetShaping;
            phi = next;
        }
        // Telescoping: what the episode has been paid is exactly the debt
        // it now carries against its anchored line, and nothing else.
        Assert.Equal(phi, total, 2);
    }

    /// <summary>
    /// Changing rung is free in itself: the potential restarts at the new
    /// anchor with no shaping reward for the jump, and the observation's
    /// deviation reads zero there.
    /// </summary>
    [Fact]
    public void AReanchorEarnsAndCostsNothing()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true, egoStrategy: new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Save));
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 4, observation);
        for (int i = 0; i < 120; i++)
            environment.Step(new[] { 0f, 1f }, observation);
        Assert.True(environment.BudgetDeviation < 0f, "flat out on Save should be behind its line");

        environment.Reanchor(new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Normal));
        Assert.Equal(0f, environment.BudgetDeviation, 5);
        var result = environment.Step(new[] { 0f, 0f }, observation);
        // One step of coasting after the switch: only that step's own
        // driving is priced, not the debt carried from Save.
        Assert.InRange(result.BudgetShaping, -0.5f, 0.5f);
    }

    /// <summary>
    /// Attack burns what it burns; switching back to Normal does not send
    /// the car to pay it back. The Normal line starts from the charge left.
    /// </summary>
    [Fact]
    public void AttackThenNormalDoesNotRepay()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true, egoStrategy: new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Attack));
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 5, observation);
        float charge0 = environment.Ego.State.Energy.Primary;
        for (int i = 0; i < 300; i++)
            environment.Step(new[] { 0f, 1f }, observation);
        Assert.True(environment.Ego.State.Energy.Primary < charge0);

        environment.Reanchor(new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Normal));
        Assert.Equal(environment.Ego.State.Energy.Primary, environment.BudgetAnchor.Charge, 5);
        Assert.Equal(0f, environment.BudgetDeviation, 5);
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
        Assert.True(reason == TrainingTerminalReason.Finished,
            $"ended {reason} at progress {environment.RaceProgress:F3}");
    }

    private static float Phi(DirectDriveDuelEnvironment environment, float lambda) =>
        EnergyBudget.Potential(
            environment.BudgetAnchor,
            environment.RaceProgress,
            environment.Ego.State.Energy.Primary,
            environment.EgoStrategy.PowerRung,
            lambda);
}
