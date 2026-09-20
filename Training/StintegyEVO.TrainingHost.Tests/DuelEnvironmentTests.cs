using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// Wheel-to-wheel at the environment's level: two seats on one simulation.
///
/// This is a smoke test and says so. Whether an impulse conserves momentum
/// is settled in the Core suite, where the contact resolver is; what is
/// asked here is whether the training environment ever gets there at all —
/// whether a second car exists in the frame both drivers read, whether the
/// cars can touch and the simulation resolves it rather than passing them
/// through each other, and whether an episode that ends puts two cars back
/// on the road.
/// </summary>
public sealed class DuelEnvironmentTests
{
    private static (float[] Ego, float[] Opponent) Frames() => (
        new float[DirectDriveObservation.ObservationSize],
        new float[DirectDriveObservation.ObservationSize]
    );

    private static readonly float[] Coast = [0f, 0f];

    [Fact]
    public void ASoloEnvironmentStillRefusesToBeAskedForAnOpponentSeat()
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        (float[] ego, float[] opponent) = Frames();
        Assert.Throws<InvalidOperationException>(
            () => environment.ResetTrack("silverstone", 7, ego, opponent)
        );
    }

    [Fact]
    public void BothSeatsSeeTheOtherCarInTheirOwnOpponentBlock()
    {
        DirectDriveDuelEnvironment environment = new(solo: false);
        (float[] ego, float[] opponent) = Frames();
        environment.ResetTrack("silverstone", 11, ego, opponent);

        int present = DirectDriveObservation.OpponentOffset;
        int longitudinal = present + 1;
        Assert.Equal(1f, ego[present]);
        Assert.Equal(1f, opponent[present]);
        // The partner starts ahead of the ego, so each reads the other on
        // its own side: positive forward for the ego, negative for the
        // partner looking back.
        Assert.True(ego[longitudinal] > 0f, "the ego should see a car ahead");
        Assert.True(
            opponent[longitudinal] < 0f,
            "the sparring partner should see a car behind"
        );
        // The two planned-future channels of the opponent slot stay zero on
        // this contract: a controller's plan is private, and the motion
        // that would have to be inferred from is already in the block.
        for (int channel = 12; channel < DirectDriveObservation.OpponentSize; channel++)
        {
            Assert.Equal(0f, ego[DirectDriveObservation.OpponentOffset + channel]);
            Assert.Equal(0f, opponent[DirectDriveObservation.OpponentOffset + channel]);
        }
    }

    [Fact]
    public void TheEgoRunsIntoASlowerCarAndTheContactIsResolved()
    {
        DirectDriveDuelEnvironment environment = new(solo: false);
        (float[] ego, float[] opponent) = Frames();
        // Six metres apart on a straight at speed, with the partner braking
        // as hard as it can and the ego on full throttle: the gap closes
        // inside a second and neither of them can steer out of it.
        environment.ResetScenario(
            "silverstone", 3,
            egoStartS: 40f,
            forwardGapMeters: 6f,
            startSpeedMetersPerSecond: 40f,
            ego, opponent
        );

        bool contacted = false;
        float leadAtContact = float.NaN;
        for (int step = 0; step < 60 && !environment.IsTerminal; step++)
        {
            environment.Step([0f, 1f], [0f, -1f], ego, opponent);
            if (environment.Ego.HitCarThisStep)
            {
                contacted = true;
                leadAtContact = environment.SignedLeadDistanceMeters;
                break;
            }
        }

        Assert.True(contacted, "the cars never touched, so nothing was resolved");
        // Resolved, not passed through: the two bodies are still apart by
        // about their own length at the moment of contact rather than
        // overlapping, and both cars are still on the road with finite
        // states.
        float clearance = environment.Ego.Collision.HalfLengthMeters +
                          environment.Opponent.Collision.HalfLengthMeters;
        Assert.InRange(leadAtContact, 0f, clearance * 1.5f);
        Assert.True(float.IsFinite(environment.Ego.State.Speed));
        Assert.True(float.IsFinite(environment.Opponent.State.Speed));
    }

    [Fact]
    public void ContactIsChargedToTheEgoAndTheRelativeProgressTermIsLive()
    {
        DirectDriveDuelEnvironment environment = new(solo: false);
        (float[] ego, float[] opponent) = Frames();
        environment.ResetScenario(
            "silverstone", 5,
            egoStartS: 40f,
            forwardGapMeters: 6f,
            startSpeedMetersPerSecond: 40f,
            ego, opponent
        );

        bool charged = false;
        bool relativeMoved = false;
        for (int step = 0; step < 60 && !environment.IsTerminal; step++)
        {
            TrainingStepResult result = environment.Step(
                [0f, 1f], [0f, -1f], ego, opponent
            );
            if (result.ContactPenalty < 0f)
                charged = true;
            if (MathF.Abs(result.RelativeProgressReward) > 1e-6f)
                relativeMoved = true;
        }

        Assert.True(charged, "running into somebody cost nothing");
        Assert.True(
            relativeMoved,
            "the relative-progress term read zero with an opponent on track"
        );
    }

    [Fact]
    public void AFinishedEpisodePutsBothCarsBackOnTheRoad()
    {
        DirectDriveDuelEnvironment environment = new(
            solo: false, episodeDurationSeconds: 2f
        );
        (float[] ego, float[] opponent) = Frames();
        environment.ResetTrack("silverstone", 13, ego, opponent);

        while (!environment.IsTerminal)
            environment.Step(Coast, Coast, ego, opponent);

        environment.ResetTrack("silverstone", 17, ego, opponent);
        Assert.False(environment.IsTerminal);
        Assert.Equal(1f, ego[DirectDriveObservation.OpponentOffset]);
        Assert.Equal(1f, opponent[DirectDriveObservation.OpponentOffset]);
        Assert.NotEqual(
            environment.Ego.State.Position,
            environment.Opponent.State.Position
        );
        // And it steps on afterwards, which is the part a batch depends on.
        environment.Step(Coast, Coast, ego, opponent);
        for (int channel = 0; channel < ego.Length; channel++)
        {
            Assert.True(float.IsFinite(ego[channel]), $"ego channel {channel}");
            Assert.True(
                float.IsFinite(opponent[channel]), $"opponent channel {channel}"
            );
        }
    }

    [Fact]
    public void ADuelStepWithoutAnActionForThePartnerIsRefused()
    {
        DirectDriveDuelEnvironment environment = new(solo: false);
        (float[] ego, float[] opponent) = Frames();
        environment.ResetTrack("silverstone", 19, ego, opponent);
        Assert.Throws<ArgumentException>(() => environment.Step(Coast, ego));
    }
}
