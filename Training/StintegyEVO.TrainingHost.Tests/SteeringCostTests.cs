using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// What the wheel costs, and — more to the point — what it does not.
///
/// The charge is the detour the front wheels took: over two decisions they
/// travelled <c>|Δ₁| + |Δ₂|</c> and got <c>|Δ₁ + Δ₂|</c> of the way, and the
/// difference between those is what is billed. That shape is the whole
/// design, because the three things a driver does with a wheel have to price
/// differently. Winding it on through a corner is driving. Holding it there
/// is driving. Sawing at it is the fault, and a single catch is a save rather
/// than a fault, so it pays once rather than continuously.
///
/// The quantity is read at the front wheels, not at the command, which is
/// what makes it independent of how the policy asked — and what makes a slow
/// weave pay, where the command's second difference this replaces let one
/// through for free.
///
/// Sophy names a cost over the acted steering history (arXiv 2511.02094,
/// appendix F) and publishes neither formula nor coefficient. The shape here
/// is inferred from that name rather than copied from it; the manifest records
/// the readings the coefficients were fitted to.
/// </summary>
public sealed class SteeringCostTests
{
    private const float Detour = 1f;
    private const float Travel = 1f;
    private const int DetourComponent = 12;
    private const int TravelComponent = 13;

    private static DirectDriveDuelEnvironment Environment(
        float detour = Detour, float travel = Travel, bool deltaActions = true
    ) => new(
        solo: true,
        deltaActions: deltaActions,
        steeringDetourPenalty: detour,
        steeringTravelPenalty: travel
    );

    private static float[] Frame() =>
        new float[DirectDriveObservation.ObservationSize];

    /// <summary>
    /// Winding the wheel on is free of the detour charge, however hard it is
    /// asked for. Turning in is what a corner is, and a corner that costs a
    /// driver to enter is a corner they will enter slowly.
    /// </summary>
    [Fact]
    public void WindingTheWheelOnCostsNothingButTheTravelTax()
    {
        DirectDriveDuelEnvironment environment = Environment();
        float[] observation = Frame();
        environment.ResetTrack("silverstone", 3, observation);

        bool moved = false;
        for (int i = 0; i < 8; i++)
        {
            TrainingStepResult result = environment.Step([1f, 0f], observation);
            Assert.Equal(0f, result.GetComponent(DetourComponent), 6);
            moved |= result.GetComponent(TravelComponent) < 0f;
        }
        Assert.True(moved, "the wheels never moved, so nothing was tested");
    }

    /// <summary>
    /// A car running straight and settled pays practically nothing: three
    /// orders of magnitude under what the rack costs at its rate limit.
    ///
    /// This is deliberately not the stronger claim that holding a corner is
    /// free. Billing the wheels rather than the command means the charge sees
    /// what the wheels actually do, and holding a line through a corner is
    /// not a still wheel: the lock a corner wants moves with speed, load and
    /// the slip the tyres take up, so the rack works the whole way round. The
    /// detour is what stays zero there, because that work is monotone — which
    /// is the point of charging the detour and not the movement.
    /// </summary>
    [Fact]
    public void ACarRunningStraightAndSettledPaysPracticallyNothing()
    {
        DirectDriveDuelEnvironment environment = Environment();
        float[] observation = Frame();
        environment.ResetTrack("silverstone", 5, observation);

        // Long enough for the start transient to settle out.
        for (int i = 0; i < 46; i++)
            environment.Step([0f, 0f], observation);

        // The rack at its rate limit charges 0.0698 a decision; anything at
        // this scale is the road breathing, not a driver moving the wheel.
        const float Nothing = 5e-4f;
        for (int i = 0; i < 12; i++)
        {
            TrainingStepResult result = environment.Step([0f, 0f], observation);
            Assert.InRange(result.GetComponent(DetourComponent), -Nothing, 0f);
            Assert.InRange(result.GetComponent(TravelComponent), -Nothing, 0f);
        }
    }

    /// <summary>
    /// Sawing pays every decision, which is the behaviour this exists to
    /// price, and pays far more than the same wheel spent going one way.
    /// </summary>
    [Fact]
    public void SawingPaysEveryDecisionAndWindingOnDoesNot()
    {
        DirectDriveDuelEnvironment sawing = Environment();
        float[] observation = Frame();
        sawing.ResetTrack("silverstone", 7, observation);
        float sawn = 0f;
        int decisionsCharged = 0;
        for (int i = 0; i < 12; i++)
        {
            TrainingStepResult result = sawing.Step(
                [i % 2 == 0 ? 1f : -1f, 0f], observation
            );
            sawn += result.GetComponent(DetourComponent);
            if (i >= 2 && result.GetComponent(DetourComponent) < 0f)
                decisionsCharged++;
        }

        DirectDriveDuelEnvironment steady = Environment();
        float[] other = Frame();
        steady.ResetTrack("silverstone", 7, other);
        float wound = 0f;
        for (int i = 0; i < 12; i++)
            wound += steady.Step([1f, 0f], other).GetComponent(DetourComponent);

        Assert.Equal(0f, wound, 6);
        Assert.True(sawn < -0.01f, $"sawing cost almost nothing: {sawn:0.000000}");
        Assert.True(
            decisionsCharged >= 9,
            $"sawing was charged on {decisionsCharged} of 10 decisions"
        );
    }

    /// <summary>
    /// A weave too slow for the two-decision window pays nothing in detour —
    /// that is the known reach of the main blade — and the travel tax is what
    /// stands under it. This is the escape the command's second difference
    /// let through for free, and the reason the tax is still here.
    /// </summary>
    [Fact]
    public void ASlowWeaveEscapesTheDetourAndPaysTheTravelTax()
    {
        DirectDriveDuelEnvironment environment = Environment();
        float[] observation = Frame();
        environment.ResetTrack("silverstone", 23, observation);

        float detour = 0f, travel = 0f;
        for (int i = 0; i < 24; i++)
        {
            // Four decisions one way, four the other: no two neighbouring
            // moves ever oppose except at the turn.
            TrainingStepResult result = environment.Step(
                [i % 8 < 4 ? 1f : -1f, 0f], observation
            );
            detour += result.GetComponent(DetourComponent);
            travel += result.GetComponent(TravelComponent);
        }

        Assert.True(
            travel < -0.05f,
            $"the weave travelled far and paid {travel:0.000000}"
        );
        Assert.True(
            detour > travel,
            "a weave this slow should pay the detour less often than it travels"
        );
    }

    /// <summary>
    /// A catch pays for the decision the hand turns in, and not for the ones
    /// after it. Catching a car is what a driver is for, and a charge that
    /// went on being levied while the correction was held would be a charge
    /// for saving the car.
    /// </summary>
    [Fact]
    public void ASingleCatchPaysForOneDecision()
    {
        DirectDriveDuelEnvironment environment = Environment();
        float[] observation = Frame();
        environment.ResetTrack("silverstone", 11, observation);

        for (int i = 0; i < 6; i++)
            environment.Step([0.4f, 0f], observation);
        // The catch: the hand goes the other way and stays there.
        TrainingStepResult caught = environment.Step([-1f, 0f], observation);
        TrainingStepResult after = environment.Step([-1f, 0f], observation);
        TrainingStepResult later = environment.Step([-1f, 0f], observation);

        Assert.True(
            caught.GetComponent(DetourComponent) < -0.0001f,
            "the turn of the hand was not charged"
        );
        Assert.Equal(0f, after.GetComponent(DetourComponent), 6);
        Assert.Equal(0f, later.GetComponent(DetourComponent), 6);
    }

    /// <summary>
    /// Both prices at zero is the world as it was: the components are present
    /// and exactly nothing, which is what <see cref="SoloFingerprintTests"/>
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
            Assert.Equal(0f, result.GetComponent(DetourComponent));
            Assert.Equal(0f, result.GetComponent(TravelComponent));
        }
    }

    /// <summary>
    /// The charge does not know which contract asked for the movement,
    /// because it never reads the action: it reads the angle the wheels
    /// reached. Two environments given the same wheel by different means are
    /// billed identically, to the bit.
    /// </summary>
    [Fact]
    public void TheChargeReadsTheWheelAndNotTheContract()
    {
        DirectDriveDuelEnvironment incremental = Environment(deltaActions: true);
        DirectDriveDuelEnvironment absolute = Environment(deltaActions: false);
        float[] one = Frame();
        float[] two = Frame();
        incremental.ResetTrack("silverstone", 17, one);
        absolute.ResetTrack("silverstone", 17, two);

        float cap = DirectDriveController.CurvatureDeltaCap(
            new StintegyEVO.Core.Cars.CarConfig(),
            DirectDriveController.DefaultDecisionHz
        );
        // The incremental car asks for a full increment each decision; the
        // absolute car names the commands that produces. Same wheel, same
        // bill.
        float command = 0f;
        for (int i = 0; i < 8; i++)
        {
            TrainingStepResult a = incremental.Step([1f, 0f], one);
            command = MathF.Min(1f, command + cap);
            TrainingStepResult b = absolute.Step([command, 0f], two);
            Assert.Equal(
                a.GetComponent(DetourComponent),
                b.GetComponent(DetourComponent),
                6
            );
            Assert.Equal(
                a.GetComponent(TravelComponent),
                b.GetComponent(TravelComponent),
                6
            );
        }
    }
}
