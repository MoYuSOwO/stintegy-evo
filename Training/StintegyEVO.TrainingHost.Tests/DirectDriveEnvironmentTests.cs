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
    public void ObservationsStayFiniteUnderCoachFollowing()
    {
        DirectDriveDuelEnvironment environment = new();
        float[] observation =
            new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("shanghai", 3, observation);

        float[] action = new float[DirectDriveObservation.ActionSize];
        for (int step = 0; step < 120 && !environment.IsTerminal; step++)
        {
            // Follow the coach block: the observation carries the analytic
            // suggestion in action units, so echoing it back is the
            // baseline-driving policy.
            action[0] = observation[
                DirectDriveObservation.CoachCurvatureIndex
            ];
            action[1] = observation[
                DirectDriveObservation.CoachAccelerationIndex
            ];
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
            action[0] = observation[
                DirectDriveObservation.CoachCurvatureIndex
            ];
            action[1] = observation[
                DirectDriveObservation.CoachAccelerationIndex
            ];
            TrainingStepResult result = environment.Step(action, observation);
            Assert.Equal(0f, result.RelativeProgressReward);
        }

        Assert.Throws<InvalidOperationException>(() => environment.Opponent);
    }
}
