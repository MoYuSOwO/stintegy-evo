using StintegyEVO.Core.Cars;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// The delta-action pilot: what the first action means, and nothing else.
///
/// Absolute, as every generation so far has been driven: the number is the
/// curvature to hold, and a policy that changes its mind at fifteen hertz
/// puts the wheel somewhere new fifteen times a second. Incremental: the
/// number is how far to move the command, the seat carries the command
/// between decisions, and holding a line is what happens when the policy
/// asks for nothing.
///
/// Everything these tests are about is the contract. The physics, the
/// reward, the curriculum and the 457 channels are untouched, and with the
/// pilot off the world is the old one to the bit —
/// <see cref="SoloFingerprintTests"/> is what says so.
/// </summary>
public sealed class DeltaActionTests
{
    private static readonly float[] Straight = [0f, 0f];

    /// <summary>
    /// The cap is the car's own number: the command crosses its range no
    /// faster than the wheels can, so the pilot cannot reach a line the
    /// absolute contract could not.
    /// </summary>
    [Fact]
    public void TheIncrementIsCalibratedAgainstTheRack()
    {
        CarConfig car = new();
        float cap = DirectDriveController.CurvatureDeltaCap(
            car, DirectDriveController.DefaultDecisionHz
        );

        // 0.8 rad of lock at 1.047 rad/s is 0.764 s, which is 11.46
        // decisions at fifteen hertz; two units of command over that many
        // decisions is 0.1745 each.
        Assert.Equal(0.17450f, cap, 4);

        // And the same statement, read back: full lock to full lock takes
        // the command as long as it takes the wheels to reach lock.
        float decisions = 2f / cap;
        float seconds = decisions / DirectDriveController.DefaultDecisionHz;
        Assert.Equal(
            car.MaxSteerAngleRadians / car.SteerRateLimitRadiansPerSecond,
            seconds,
            3
        );
    }

    /// <summary>
    /// A held increment winds the command on and then stops at the end of
    /// its range, and a zero increment holds whatever was reached. That
    /// second half is the point of the pilot: a straight line costs the
    /// policy no decisions at all.
    /// </summary>
    [Fact]
    public void AHeldIncrementWindsTheCommandOnAndThenHoldsIt()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true, deltaActions: true
        );
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 5, observation);
        int command = DirectDriveObservation.EgoOffset + 10;

        // Six decisions of full increment is a little over the whole range.
        for (int i = 0; i < 6; i++)
            environment.Step([1f, 0f], observation);
        Assert.Equal(1f, observation[command], 3);

        // Asking for nothing holds it, which under the absolute contract
        // would have meant driving straight instead.
        for (int i = 0; i < 10; i++)
            environment.Step(Straight, observation);
        Assert.Equal(1f, observation[command], 3);

        // And it winds back off at the same rate rather than jumping.
        environment.Step([-1f, 0f], observation);
        Assert.Equal(
            1f - DirectDriveController.CurvatureDeltaCap(
                new CarConfig(), DirectDriveController.DefaultDecisionHz
            ),
            observation[command],
            3
        );
    }

    /// <summary>
    /// The command the car is holding is in the observation, in the
    /// channel that has always carried what was last commanded. A driver
    /// that cannot see the wheel in its hands is being asked to integrate
    /// blind, which is the mode-excess mistake in another costume.
    /// </summary>
    [Fact]
    public void ThePolicyCanSeeTheCommandItIsHolding()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true, deltaActions: true
        );
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 9, observation);
        int command = DirectDriveObservation.EgoOffset + 10;
        int previousCommand = DirectDriveObservation.PreviousDynamicOffset + 10;

        Assert.Equal(0f, observation[command]);
        environment.Step([1f, 0f], observation);
        float first = observation[command];
        Assert.True(first > 0f, "the integrator never moved");
        environment.Step([1f, 0f], observation);

        // Two frames of it, so the increment itself is recoverable: the
        // policy is never told a number it cannot check.
        Assert.Equal(first, observation[previousCommand], 4);
        Assert.Equal(
            DirectDriveController.CurvatureDeltaCap(
                new CarConfig(), DirectDriveController.DefaultDecisionHz
            ),
            observation[command] - observation[previousCommand],
            4
        );
    }

    /// <summary>
    /// A fresh episode starts with the wheel straight. An integrator that
    /// survived a reset would hand the next episode a corner nobody asked
    /// for, and the lane would look like a policy failure.
    /// </summary>
    [Fact]
    public void AFreshEpisodeStartsWithTheWheelStraight()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: true, deltaActions: true
        );
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 11, observation);
        int command = DirectDriveObservation.EgoOffset + 10;

        for (int i = 0; i < 6; i++)
            environment.Step([1f, 0f], observation);
        Assert.True(observation[command] > 0.9f);

        environment.ResetTrack("silverstone", 13, observation);
        Assert.Equal(0f, observation[command]);
    }

    /// <summary>
    /// With the pilot off, the first action is the command it always was.
    /// </summary>
    [Fact]
    public void WithoutThePilotTheActionIsStillTheCommand()
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        environment.ResetTrack("silverstone", 5, observation);
        int command = DirectDriveObservation.EgoOffset + 10;

        environment.Step([0.4f, 0f], observation);
        Assert.Equal(0.4f, observation[command], 4);
        environment.Step([-0.9f, 0f], observation);
        Assert.Equal(-0.9f, observation[command], 4);
    }
}
