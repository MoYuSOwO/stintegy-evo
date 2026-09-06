using StintegyEVO.Core.Drivers.Learned;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// The contract between an agent step and a driver decision, pinned.
///
/// It was broken for the whole of the first training era and nothing
/// noticed, because every symptom it produced looked like a learning
/// problem: the driver kept its own ten-hertz clock, which free-ran
/// against the hundred-millisecond agent step, so an action arrived
/// five-sixths of the way through the step it was credited with and the
/// observation handed back was sampled before that action had touched
/// anything at all. Full throttle and full brake from the same state
/// returned two identical observations while the cars ended up at
/// different speeds. Every transition SAC learned from attached a reward
/// and a successor state to the wrong action.
///
/// One sentence holds the whole contract, and these tests are that
/// sentence in three parts: the observation is sampled at t, the action
/// answering it drives every substep of [t, t + step), and the next
/// observation is sampled at t + step.
/// </summary>
public sealed class DecisionClockTests
{
    private const int Speed = DirectDriveObservation.EgoOffset;
    private const float SpeedScale = 100f;

    /// <summary>
    /// The decisive one. Two identical worlds, told to do opposite things,
    /// must hand back different worlds - and the difference has to be
    /// visible in the observation, not merely in the physics behind it,
    /// because the observation is all the learner ever sees.
    /// </summary>
    [Fact]
    public void OppositeActionsProduceDifferentSuccessorObservations()
    {
        float[] throttle = Drive(1f);
        float[] brake = Drive(-1f);

        float difference = 0f;
        for (int i = 0; i < throttle.Length; i++)
            difference = MathF.Max(difference, MathF.Abs(throttle[i] - brake[i]));

        Assert.True(
            difference > 1e-4f,
            "full throttle and full brake returned the same observation; " +
            "the action is not reaching the interval it is credited with"
        );
        Assert.True(
            throttle[Speed] > brake[Speed],
            "the throttled car should be the faster one"
        );
    }

    /// <summary>
    /// The observation must describe the car at the end of the step, not
    /// somewhere in the middle of it. Speed is the cheapest place to catch
    /// that: the returned reading has to be the car's actual speed once the
    /// step is over.
    /// </summary>
    [Fact]
    public void TheReturnedSpeedIsTheSpeedTheCarEndedTheStepAt()
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 4242, observation);

        for (int step = 0; step < 12; step++)
        {
            environment.Step(new[] { 0.1f, 1f }, observation);
            Assert.Equal(
                environment.Ego.State.Speed,
                observation[Speed] * SpeedScale,
                3
            );
        }
    }

    /// <summary>
    /// Reset must leave the driver on the same beat as the environment.
    ///
    /// The old reset ran a hair of simulated time to settle the telemetry,
    /// and that hair consumed the driver's first decision - which put every
    /// step afterwards out of phase by five driver frames. The check is
    /// that the very first action already bites: one step of full throttle
    /// and one of full brake, straight out of reset, must part company.
    /// </summary>
    [Fact]
    public void ResetLeavesNoOffsetForTheFirstStepToInherit()
    {
        float[] fast = DriveOnce(1f);
        float[] slow = DriveOnce(-1f);

        Assert.True(
            fast[Speed] > slow[Speed] + 1e-5f,
            "the first action after a reset did not reach the first step"
        );
    }

    /// <summary>
    /// And the ego driver must actually be the externally clocked kind, so
    /// that a future edit cannot quietly hand the clock back.
    /// </summary>
    [Fact]
    public void TheEnvironmentOwnsTheClock()
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 7, observation);

        Assert.Equal(DecisionClock.External, environment.EgoDriver.Clock);
        Assert.Equal(
            environment.AgentStepSeconds,
            environment.EgoDriver.DecisionPeriodSeconds,
            6
        );
    }

    /// <summary>
    /// Same seed, same first step, one action each. Returns the observation
    /// the environment hands back.
    /// </summary>
    private static float[] DriveOnce(float acceleration)
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 991, observation);
        environment.Step(new[] { 0f, acceleration }, observation);
        return observation;
    }

    /// <summary>
    /// Same seed, the same settling step, then the step under test - so the
    /// two branches part company only where the action differs.
    /// </summary>
    private static float[] Drive(float acceleration)
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 991, observation);
        environment.Step(new[] { 0f, 0.5f }, observation);
        environment.Step(new[] { 0f, acceleration }, observation);
        return observation;
    }
}
