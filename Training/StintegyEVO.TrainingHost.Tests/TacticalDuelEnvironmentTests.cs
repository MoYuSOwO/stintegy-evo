using StintegyEVO.Core.Drivers;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

public sealed class TacticalDuelEnvironmentTests
{
    [Fact]
    public void ResetIsDeterministicForSeed()
    {
        TacticalDuelEnvironment first = new();
        TacticalDuelEnvironment second = new();
        float[] firstObservation = new float[TacticalPolicyShape.ObservationSize];
        float[] secondObservation = new float[TacticalPolicyShape.ObservationSize];

        first.Reset(42, firstObservation);
        second.Reset(42, secondObservation);

        Assert.Equal(first.TrackFamily, second.TrackFamily);
        Assert.Equal(first.EgoStartS, second.EgoStartS);
        Assert.Equal(first.InitialForwardGapMeters, second.InitialForwardGapMeters);
        Assert.Equal(firstObservation, secondObservation);
    }

    [Fact]
    public void StepUsesOneBoundedActionAndReturnsFiniteValues()
    {
        TacticalDuelEnvironment environment = new();
        float[] observation = new float[TacticalPolicyShape.ObservationSize];
        environment.Reset(7, observation);

        TrainingStepResult result = environment.Step(
            stackalloc float[] { 0.5f },
            observation
        );

        Assert.True(float.IsFinite(result.Reward));
        for (int i = 0; i < TrainingStepResult.ComponentCount; i++)
            Assert.True(float.IsFinite(result.GetComponent(i)));
        Assert.All(observation, value => Assert.InRange(value, -1f, 1f));
    }

    [Fact]
    public void ContactTerminatesWithLargeNegativeReward()
    {
        TacticalDuelEnvironment environment = new();
        float[] observation = new float[TacticalPolicyShape.ObservationSize];
        environment.Reset(11, observation);
        environment.Opponent.State.Position = environment.Ego.State.Position;

        TrainingStepResult result = environment.Step(
            stackalloc float[] { 0f },
            observation
        );

        Assert.Equal(TrainingTerminalReason.Contact, result.TerminalReason);
        Assert.True(result.Done);
        Assert.True(result.ContactPenalty <= -50f);
    }
}
