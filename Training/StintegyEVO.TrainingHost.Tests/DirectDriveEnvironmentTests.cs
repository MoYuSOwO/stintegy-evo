using StintegyEVO.Core.Drivers.Learned;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

public sealed class DirectDriveEnvironmentTests
{
    [Fact]
    public void ResetIsDeterministicForSeed()
    {
        DirectDriveDuelEnvironment first = new();
        DirectDriveDuelEnvironment second = new();
        float[] firstObservation =
            new float[DirectDriveObservation.ObservationSize];
        float[] secondObservation =
            new float[DirectDriveObservation.ObservationSize];

        first.Reset(42, firstObservation);
        second.Reset(42, secondObservation);

        Assert.Equal(first.TrackFamily, second.TrackFamily);
        Assert.Equal(first.EgoStartS, second.EgoStartS);
        Assert.Equal(
            first.InitialForwardGapMeters,
            second.InitialForwardGapMeters
        );
        Assert.Equal(firstObservation, secondObservation);
    }

    [Fact]
    public void SteppingTheSameActionsReproducesTheSameTrajectory()
    {
        DirectDriveDuelEnvironment first = new();
        DirectDriveDuelEnvironment second = new();
        float[] firstObservation =
            new float[DirectDriveObservation.ObservationSize];
        float[] secondObservation =
            new float[DirectDriveObservation.ObservationSize];
        first.ResetTrack("simple-right", 7, firstObservation);
        second.ResetTrack("simple-right", 7, secondObservation);

        float[] action = new float[DirectDriveObservation.ActionSize];
        for (int step = 0; step < 60; step++)
        {
            action[0] = MathF.Sin(step * 0.11f) * 0.2f;
            action[1] = 0.4f;
            TrainingStepResult firstResult = first.Step(
                action,
                firstObservation
            );
            TrainingStepResult secondResult = second.Step(
                action,
                secondObservation
            );
            Assert.Equal(firstResult, secondResult);
            Assert.Equal(firstObservation, secondObservation);
            Assert.True(float.IsFinite(firstResult.Reward));
            if (firstResult.Done)
                break;
        }
    }

    [Fact]
    public void ObservationsStayFiniteUnderSteadyDriving()
    {
        DirectDriveDuelEnvironment environment = new();
        float[] observation =
            new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("shanghai", 3, observation);

        float[] action = new float[DirectDriveObservation.ActionSize];
        for (int step = 0; step < 120 && !environment.IsTerminal; step++)
        {
            // Steer at a point down the road and hold a modest throttle.
            // What is being pinned is that nothing in the observation goes
            // non-finite while the car is actually driving, so any policy
            // that keeps it moving will do.
            Steer(observation, action);
            environment.Step(action, observation);
            foreach (float value in observation)
                Assert.True(float.IsFinite(value));
        }

        Assert.True(environment.Ego.State.Speed > 5f);
    }

    [Fact]
    public void SoloEnvironmentRunsWithoutOpponent()
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation =
            new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("speedway", 11, observation);

        float[] action = new float[DirectDriveObservation.ActionSize];
        for (int step = 0; step < 60 && !environment.IsTerminal; step++)
        {
            Steer(observation, action);
            TrainingStepResult result = environment.Step(action, observation);
            Assert.Equal(0f, result.RelativeProgressReward);
        }

        Assert.Throws<InvalidOperationException>(() => environment.Opponent);
    }

    /// <summary>Pure pursuit off the geometry block, thirty metres ahead.</summary>
    private static void Steer(float[] observation, float[] action)
    {
        int cursor = DirectDriveObservation.GeometryOffset +
                     5 * DirectDriveObservation.GeometryFloatsPerPoint;
        float ahead = observation[cursor] * 600f;
        float across = observation[cursor + 1] * 30f;
        float rangeSquared = ahead * ahead + across * across;
        float curvature = rangeSquared > 1f ? 2f * across / rangeSquared : 0f;
        action[0] = Math.Clamp(curvature / 0.05f, -1f, 1f);
        action[1] = 0.4f;
    }
}
