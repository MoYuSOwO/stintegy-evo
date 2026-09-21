using StintegyEVO.Core.Cars;
using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// What the wheel costs, and — more to the point — what it does not.
///
/// The charge is the second difference of the steering command: how much
/// the movement changed, not how much there was. That shape is the whole
/// design, because the three things a driver does with a wheel have to
/// price differently. Winding it on through a corner is driving. Holding
/// it there is driving. Sawing at it is the fault, and a single catch is
/// a save rather than a fault, so it pays once rather than continuously.
///
/// Sophy charges the same quantity (arXiv 2511.02094, appendix F) and
/// publishes no coefficient; these tests pin the shape, and the manifest
/// records the three measurements the coefficients were fitted to.
/// </summary>
public sealed class SteeringCostTests
{
    private const float Reversal = 0.8f;
    private const float Change = 0.05f;
    private const int ReversalComponent = 12;
    private const int ChangeComponent = 13;

    private static DirectDriveDuelEnvironment Environment(
        float reversal = Reversal, float change = Change
    ) => new(
        solo: true,
        deltaActions: true,
        steeringReversalPenaltyPerSecond: reversal,
        steeringChangePenaltyPerSecond: change
    );

    private static float[] Frame() =>
        new float[DirectDriveObservation.ObservationSize];

    /// <summary>
    /// Winding the wheel on at a steady rate is free of the reversal
    /// charge. It is what turning in is, and a corner that costs a driver
    /// to enter is a corner they will enter slowly.
    /// </summary>
    [Fact]
    public void WindingTheWheelOnAtASteadyRateCostsNothingButTheTax()
    {
        DirectDriveDuelEnvironment environment = Environment();
        float[] observation = Frame();
        environment.ResetTrack("silverstone", 3, observation);

        // The first decision sets the rate; from the second on, the rate
        // is unchanged and the second difference is zero.
        environment.Step([0.5f, 0f], observation);
        for (int i = 0; i < 8; i++)
        {
            TrainingStepResult result = environment.Step([0.5f, 0f], observation);
            Assert.Equal(0f, result.GetComponent(ReversalComponent), 6);
            Assert.True(
                result.GetComponent(ChangeComponent) < 0f,
                "the movement itself should still be taxed"
            );
        }
    }

    /// <summary>
    /// Holding a corner is free of both. A policy that has found its line
    /// and is asking for nothing should pay nothing for the wheel.
    /// </summary>
    [Fact]
    public void HoldingTheWheelStillCostsNothingAtAll()
    {
        DirectDriveDuelEnvironment environment = Environment();
        float[] observation = Frame();
        environment.ResetTrack("silverstone", 5, observation);

        for (int i = 0; i < 4; i++)
            environment.Step([0.6f, 0f], observation);
        // Asking for no further movement: under the incremental contract
        // this is the wheel held where it is.
        environment.Step([0f, 0f], observation);
        for (int i = 0; i < 8; i++)
        {
            TrainingStepResult result = environment.Step([0f, 0f], observation);
            Assert.Equal(0f, result.GetComponent(ReversalComponent), 6);
            Assert.Equal(0f, result.GetComponent(ChangeComponent), 6);
        }
    }

    /// <summary>
    /// Sawing pays every decision, which is the behaviour this exists to
    /// price, and pays far more than the same amount of steering spent
    /// going one way.
    /// </summary>
    [Fact]
    public void SawingPaysEveryDecisionAndWindingOnDoesNot()
    {
        DirectDriveDuelEnvironment environment = Environment();
        float[] observation = Frame();
        environment.ResetTrack("silverstone", 7, observation);
        float sawn = 0f;
        for (int i = 0; i < 12; i++)
        {
            TrainingStepResult result = environment.Step(
                [i % 2 == 0 ? 0.5f : -0.5f, 0f], observation
            );
            sawn += result.GetComponent(ReversalComponent);
        }

        DirectDriveDuelEnvironment steady = Environment();
        float[] other = Frame();
        steady.ResetTrack("silverstone", 7, other);
        float wound = 0f;
        for (int i = 0; i < 12; i++)
        {
            TrainingStepResult result = steady.Step([0.5f, 0f], other);
            wound += result.GetComponent(ReversalComponent);
        }

        Assert.True(sawn < wound * 5f, $"sawing {sawn:0.0000} vs winding {wound:0.0000}");
        Assert.True(sawn < -0.01f, "sawing at the wheel cost almost nothing");
    }

    /// <summary>
    /// A catch pays for the decision the hand turns in, and not for the
    /// ones after it. Catching a car is what a driver is for, and a charge
    /// that went on being levied while the correction was held would be a
    /// charge for saving the car.
    /// </summary>
    [Fact]
    public void ASingleCatchPaysForOneDecision()
    {
        DirectDriveDuelEnvironment environment = Environment();
        float[] observation = Frame();
        environment.ResetTrack("silverstone", 11, observation);

        for (int i = 0; i < 4; i++)
            environment.Step([0.4f, 0f], observation);
        // The catch: the hand goes the other way, once, and is then held
        // there at the same rate.
        TrainingStepResult caught = environment.Step([-0.9f, 0f], observation);
        TrainingStepResult after = environment.Step([-0.9f, 0f], observation);
        TrainingStepResult later = environment.Step([-0.9f, 0f], observation);

        Assert.True(caught.GetComponent(ReversalComponent) < -0.001f);
        Assert.Equal(0f, after.GetComponent(ReversalComponent), 6);
        Assert.Equal(0f, later.GetComponent(ReversalComponent), 6);
    }

    /// <summary>
    /// Both at zero is the world as it was: the components are present and
    /// exactly nothing, which is what <see cref="SoloFingerprintTests"/>
    /// relies on.
    /// </summary>
    [Fact]
    public void BothPricesAtZeroChargeNothing()
    {
        DirectDriveDuelEnvironment environment = Environment(0f, 0f);
        float[] observation = Frame();
        environment.ResetTrack("silverstone", 13, observation);

        for (int i = 0; i < 12; i++)
        {
            TrainingStepResult result = environment.Step(
                [i % 2 == 0 ? 1f : -1f, 0f], observation
            );
            Assert.Equal(0f, result.GetComponent(ReversalComponent));
            Assert.Equal(0f, result.GetComponent(ChangeComponent));
        }
    }

    /// <summary>
    /// The charge is on the wheel, not on the contract: the same movement
    /// of the command costs the same under either way of asking for it.
    /// </summary>
    [Fact]
    public void TheChargeIsTheSameUnderEitherActionContract()
    {
        DirectDriveDuelEnvironment incremental = new(
            solo: true,
            deltaActions: true,
            steeringReversalPenaltyPerSecond: Reversal,
            steeringChangePenaltyPerSecond: Change
        );
        DirectDriveDuelEnvironment absolute = new(
            solo: true,
            deltaActions: false,
            steeringReversalPenaltyPerSecond: Reversal,
            steeringChangePenaltyPerSecond: Change
        );
        float[] one = Frame();
        float[] two = Frame();
        incremental.ResetTrack("silverstone", 17, one);
        absolute.ResetTrack("silverstone", 17, two);

        float cap = DirectDriveController.CurvatureDeltaCap(
            new CarConfig(), DirectDriveController.DefaultDecisionHz
        );
        // The incremental car asks for a full increment each decision; the
        // absolute car names the commands that produces. Same wheel, same
        // bill.
        float command = 0f;
        for (int i = 0; i < 6; i++)
        {
            TrainingStepResult a = incremental.Step([1f, 0f], one);
            command = MathF.Min(1f, command + cap);
            TrainingStepResult b = absolute.Step([command, 0f], two);
            Assert.Equal(
                a.GetComponent(ReversalComponent),
                b.GetComponent(ReversalComponent),
                5
            );
            Assert.Equal(
                a.GetComponent(ChangeComponent),
                b.GetComponent(ChangeComponent),
                5
            );
        }
    }
}
