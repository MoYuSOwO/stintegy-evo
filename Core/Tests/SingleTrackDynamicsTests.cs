using System;
using StintegyEVO.Core.Cars;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The three manoeuvres a chassis is judged on, and the envelope it has to
/// keep while it does them.
///
/// The model these test replaced could not fail any of them, because it had
/// no slip angles: cornering force was handed over on request and the
/// body's attitude was bookkeeping held inside a clamp. A car with real
/// slip angles can understeer, oversteer, and be caught - so it can also be
/// wrong in all three ways, and these are the shapes that say it is not.
/// </summary>
public sealed class SingleTrackDynamicsTests
{
    private const float TestAirTempC = 25f;
    private const float TestTrackTempC = 35f;
    private const float Dt = 1f / 60f;

    private readonly ITestOutputHelper _output;

    public SingleTrackDynamicsTests(ITestOutputHelper output) => _output = output;

    /// <summary>
    /// Steady state on a constant radius: the car settles, and what it
    /// settles at is the cornering envelope the chassis was calibrated to.
    ///
    /// This is the constraint the tyre curve was solved against. The peak of
    /// the curve is one by construction, so an axle at its best slip angle
    /// delivers exactly the grip it has, and the car's limit is the same
    /// three and a half g it was before any of this - which is what makes
    /// every lap time on record still comparable.
    /// </summary>
    [Fact]
    public void SteadyCorneringStillFindsTheCarsOwnEnvelope()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        float best = 0f;
        float bestCurvature = 0f;
        float grip = 0f;

        for (int i = 1; i <= 30; i++)
        {
            float curvature = 0.0005f * i;
            (float lateral, bool spun, float axleGrip) =
                SteadyCircle(car, tires, 60f, curvature);
            if (!spun && lateral > best)
            {
                best = lateral;
                bestCurvature = curvature;
                grip = axleGrip;
            }
        }

        _output.WriteLine(
            $"peak steady lateral {best:0.00} m/s^2 ({best / 9.80665f:0.00} g) " +
            $"at curvature {bestCurvature:0.0000}, axle grip sum {grip:0.00} " +
            $"({grip / 9.80665f:0.00} g), used {best / grip * 100f:0.0}%"
        );
        Assert.True(
            best > 3.0f * 9.80665f && best < 3.9f * 9.80665f,
            $"steady cornering should land in the Formula 2 band the chassis " +
            $"was calibrated to; got {best / 9.80665f:0.00} g"
        );
        // Short of the whole circle, because a tyre's peak is a peak: sit
        // exactly on it and the next disturbance takes force away rather
        // than adding it. The old model could spend the whole circle, and
        // spent 96.6% of it on this same manoeuvre, so the envelope has not
        // moved - which is what keeps every lap time on record comparable.
        Assert.InRange(best / grip, 0.85f, 0.99f);
    }

    /// <summary>
    /// Asked for more corner than it has, the car goes wide instead of
    /// getting it. That is understeer, and before this model there was no
    /// such thing: the tyres handed over whatever was requested until a
    /// circle stopped them, and the car simply cornered harder.
    /// </summary>
    /// <summary>
    /// The number on the sign has to be the number the car does.
    ///
    /// <see cref="TireSlipCurve.SustainablePeakShare"/> is the car's own
    /// account of how much of its friction circle it can actually hold, and
    /// everything that plans a corner speed reads it. It is a measured
    /// figure, not a chosen one, and the thing it is measured against is the
    /// constant-speed skidpad below - so this test exists to make the
    /// measurement compulsory. Change the balance, the tyre curve, the
    /// steering, anything that moves what the car will hold, and this fails
    /// until somebody re-reads the skidpad and writes the new number down.
    ///
    /// The alternative is a sign nobody maintains, which is what the old
    /// model left behind: it advertised the whole circle, which was true
    /// when it could spend the whole circle and a tenth of a lie afterwards.
    /// </summary>
    [Fact]
    public void TheAdvertisedGripMatchesWhatTheSkidpadMeasures()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        const float Speed = 60f;

        float measured = 0f;
        float grip = 0f;
        for (int i = 1; i <= 30; i++)
        {
            (float lateral, bool spun, float axleGrip) =
                SteadyCircle(car, tires, Speed, 0.0005f * i);
            if (!spun && lateral > measured)
            {
                measured = lateral;
                grip = axleGrip;
            }
        }

        float measuredShare = measured / grip;
        float advertised = TireSlipCurve.SustainablePeakShare;
        _output.WriteLine(
            $"skidpad holds {measuredShare:0.000} of the axles' grip; the " +
            $"sign says {advertised:0.000}"
        );
        Assert.InRange(measuredShare, advertised - 0.05f, advertised + 0.05f);
    }

    [Fact]
    public void AskingForMoreThanTheTyresHaveGivesUnderteerRatherThanGrip()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();

        CarState reasonable = CreateState(60f, tires);
        HoldSpeed(reasonable, car, tires, 0.004f, 60f, 180);
        float gentleShare =
            MathF.Abs(reasonable.Telemetry.ActualLateralAccel) /
            MathF.Abs(reasonable.Telemetry.RequestedLateralAccel);
        _output.WriteLine(
            $"gentle: asked {reasonable.Telemetry.RequestedLateralAccel:0.00} " +
            $"got {reasonable.Telemetry.ActualLateralAccel:0.00}"
        );

        // Inside the envelope the driver gets the corner they asked for:
        // the wheels are turned the extra the tyres take, which is what a
        // driver does and what keeps a curvature request meaning something.
        Assert.InRange(gentleShare, 0.95f, 1.05f);

        // Beyond it, more lock buys less and less corner, and eventually
        // none. Swept rather than asserted at one curvature because where
        // the envelope sits is a property of the chassis, not of this test.
        float previous = 0f;
        float best = 0f;
        float turnedOverAt = 0f;
        for (int i = 1; i <= 40; i++)
        {
            float curvature = 0.0005f * i;
            (float lateral, bool spun, _) = SteadyCircle(car, tires, 60f, curvature);
            if (spun)
            {
                _output.WriteLine($"lost it when asked for {curvature:0.0000}");
                break;
            }
            if (lateral < previous && turnedOverAt == 0f)
                turnedOverAt = curvature;
            best = MathF.Max(best, lateral);
            previous = lateral;
        }

        float demandAtPeak = 60f * 60f * (turnedOverAt > 0f ? turnedOverAt : 0.02f);
        _output.WriteLine(
            $"most the car gave: {best:0.00} m/s^2, asked for " +
            $"{demandAtPeak:0.00} by then"
        );
        Assert.True(
            best < demandAtPeak * 0.95f,
            $"past the envelope the corner has to arrive short: gave " +
            $"{best:0.00} where {demandAtPeak:0.00} was asked"
        );
    }

    /// <summary>
    /// The balance, stated as a test because it is the difference between a
    /// car that can be raced and one that cannot: asked for far more corner
    /// than it has, this car washes wide for a long way before it lets go.
    /// A neutral car - both ends peaking together - spins the moment it is
    /// overdriven at all, which is quicker on paper and unraceable in fact.
    /// </summary>
    /// <summary>
    /// The car understeers, in the textbook sense: holding a steady corner
    /// takes more lock than the corner's own geometry needs, and the extra
    /// is the difference between the two ends' slip angles.
    ///
    /// This is the balance the chassis is set up with and the reason the
    /// front peaks later than the rear. It is not a free choice - a neutral
    /// car is quicker on paper - and it is not decoration either: which end
    /// runs out first is what a driver is reading when they decide whether
    /// to keep leaning on it.
    /// </summary>
    [Fact]
    public void HoldingACornerTakesMoreLockThanGeometryAsksFor()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = CreateState(60f, tires);
        HoldSpeed(state, car, tires, 0.005f, 60f, 240);

        float kinematic = car.WheelBaseMeters *
                          MathF.Abs(state.Telemetry.ActualCurvature);
        float actual = MathF.Abs(state.SteerAngleRadians);
        _output.WriteLine(
            $"geometry would need {kinematic:0.0000} rad, the car is holding " +
            $"{actual:0.0000} rad"
        );
        Assert.True(
            car.FrontPeakSlipAngleRatio > 1f,
            "the default chassis should be set up to understeer"
        );
        Assert.True(
            actual > kinematic * 1.05f,
            $"a steady corner should be taking more lock than geometry: " +
            $"{actual:0.0000} against {kinematic:0.0000}"
        );
    }

    /// <summary>
    /// Step steer: the yaw rate rises, settles, and stays settled. A car
    /// that oscillated here would be one whose lateral states are being
    /// integrated too coarsely, which is the failure the subdivided clock
    /// inside the step exists to prevent.
    /// </summary>
    [Fact]
    public void AStepOfSteeringSettlesInsteadOfRinging()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = CreateState(50f, tires);
        DriverInput input = new(0.004f, 0f);

        float peak = 0f;
        float firstSecond = 0f;
        float lateSwing = 0f;
        float settled = 0f;
        for (int i = 0; i < 240; i++)
        {
            StepHoldingSpeed(state, car, tires, input.DesiredCurvature, 50f);
            float yawRate = state.YawRateRadiansPerSecond;
            if (i < 60)
                peak = MathF.Max(peak, yawRate);
            else if (i < 120)
                firstSecond = MathF.Max(firstSecond, yawRate);
            if (i >= 180)
            {
                settled = yawRate;
                lateSwing = MathF.Max(lateSwing, MathF.Abs(yawRate - settled));
            }
        }

        _output.WriteLine(
            $"first peak {peak:0.0000}, second {firstSecond:0.0000}, " +
            $"settled {settled:0.0000}, late swing {lateSwing:0.00000}, " +
            $"sideslip {state.SideslipAngleRadians * 180f / MathF.PI:0.00} deg"
        );
        Assert.True(settled > 0f, "a left step should leave the car turning left");
        // A ringing car is one whose lateral states are being advanced too
        // coarsely, which is what the subdivided clock inside a step exists
        // to prevent. Overshoot is allowed - a real car has some - but each
        // swing has to be smaller than the last.
        Assert.True(
            peak < settled * 1.5f,
            $"the yaw response overshot by more than half: peak {peak:0.0000} " +
            $"against settled {settled:0.0000}"
        );
        Assert.True(
            firstSecond < peak,
            $"the oscillation should decay: {peak:0.0000} then {firstSecond:0.0000}"
        );
        Assert.True(
            lateSwing < settled * 0.02f,
            $"the car should be settled after four seconds, still swinging " +
            $"{lateSwing:0.00000}"
        );
        Assert.True(
            MathF.Abs(state.SideslipAngleRadians) < 0.15f,
            "a moderate corner should not leave the car noticeably sideways"
        );
    }

    /// <summary>
    /// Trail braking: the same corner, entered with the brakes still on,
    /// costs the tyres their circle and the car its cornering. The slip
    /// angles rise because the same force is being asked of less rubber.
    /// </summary>
    [Fact]
    public void BrakingIntoACornerCostsTheCornering()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        DriverInput coasting = new(0.005f, 0f);
        DriverInput trailing = new(0.005f, -18f);

        CarState free = CreateState(60f, tires);
        Settle(free, car, tires, coasting, 60);
        CarState braking = CreateState(60f, tires);
        Settle(braking, car, tires, trailing, 60);

        _output.WriteLine(
            $"coasting lateral {free.Telemetry.ActualLateralAccel:0.00}, " +
            $"braking lateral {braking.Telemetry.ActualLateralAccel:0.00}"
        );
        Assert.True(
            MathF.Abs(braking.Telemetry.ActualLateralAccel) <
            MathF.Abs(free.Telemetry.ActualLateralAccel),
            "braking should take cornering off the same corner"
        );
        Assert.True(
            braking.Telemetry.RearLateralUse + braking.Telemetry.RearLongitudinalUse >
            free.Telemetry.RearLateralUse + free.Telemetry.RearLongitudinalUse,
            "the trail-braked axle should be working harder overall"
        );
    }

    /// <summary>
    /// The wheels take time to get where the driver wants them, and that
    /// time is the point: it is what makes a slide catchable or not, and it
    /// is the first thing in this car that makes the decision rate bite.
    /// </summary>
    [Fact]
    public void TheWheelsCannotArriveInstantly()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = CreateState(40f, tires);
        DriverInput hard = new(car.MaxCurvatureRequest, 0f);

        CarPhysics.Step(state, car, tires, PhysicsInput(hard), Dt);
        float afterOne = state.SteerAngleRadians;

        Assert.True(afterOne > 0f, "the wheels should have started moving");
        Assert.True(
            afterOne <= car.SteerRateLimitRadiansPerSecond * Dt + 1e-6f,
            $"the wheels moved {afterOne:0.0000} rad in one frame, more than " +
            $"the rate limit allows"
        );

        for (int i = 0; i < 120; i++)
            CarPhysics.Step(state, car, tires, PhysicsInput(hard), Dt);
        Assert.True(
            state.SteerAngleRadians > afterOne,
            "held there, the wheels should keep going"
        );
        Assert.True(
            state.SteerAngleRadians <= car.MaxSteerAngleRadians + 1e-6f,
            "and should stop at the lock stops"
        );
    }

    /// <summary>
    /// A slide held past where the model can describe it is a spin, and a
    /// spin hands back a moving car. Flicking through the same angle is
    /// not: a save should be allowed to be spectacular.
    /// </summary>
    [Fact]
    public void OnlyASustainedSlideIsASpin()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        float past = SingleTrackDynamicsLimits.SpinVerdictSideslipRadians * 1.1f;
        float verdict = SingleTrackDynamicsLimits.SpinVerdictHoldSeconds;

        CarState flicked = HeldSideways(car, tires, past, verdict * 0.5f);
        Assert.False(flicked.Spinning, "half the verdict time is a save");
        Assert.Equal(0, flicked.SpinEvents);

        CarState gone = HeldSideways(car, tires, past, verdict * 2f);
        Assert.True(gone.Spinning, "twice the verdict time is a spin");
        Assert.Equal(1, gone.SpinEvents);

        int steps = 0;
        DriverInput desperate = new(car.MaxCurvatureRequest, car.MaxDriveAcceleration);
        float entrySpeed = gone.Speed;
        while (gone.Spinning && steps < 30 * 60)
        {
            float before = gone.Speed;
            CarPhysics.Step(gone, car, tires, PhysicsInput(desperate), Dt);
            Assert.True(
                gone.Speed <= before + 1e-4f,
                "a spinning car must not accelerate on the driver's throttle"
            );
            steps++;
        }

        _output.WriteLine($"spin lasted {steps * Dt:0.00} s, {entrySpeed:0.0} -> {gone.Speed:0.0} m/s");
        Assert.False(gone.Spinning, "the spin should end on its own");
        Assert.True(
            gone.Speed >=
            SingleTrackDynamicsLimits.SpinReleaseSpeedMetersPerSecond - 1f,
            $"handed back moving, not parked: {gone.Speed:0.0} m/s"
        );
        Assert.Equal(1, gone.SpinEvents);
    }


    private static (float Lateral, bool Spun, float Grip) SteadyCircle(
        CarConfig car,
        TireConfig tires,
        float speed,
        float curvature
    )
    {
        CarState state = CreateState(speed, tires);
        float peak = 0f;
        float grip = 0f;
        bool spun = false;
        for (int i = 0; i < 300; i++)
        {
            StepHoldingSpeed(state, car, tires, curvature, speed);
            if (state.Spinning)
                spun = true;
            if (i > 60 && !spun &&
                MathF.Abs(state.Telemetry.ActualLateralAccel) > peak)
            {
                peak = MathF.Abs(state.Telemetry.ActualLateralAccel);
                grip = state.Telemetry.FrontGripAccel + state.Telemetry.RearGripAccel;
            }
        }
        return (peak, spun, grip);
    }

    private static float FirstCurvatureThatSpins(
        CarConfig car,
        TireConfig tires,
        float speed
    )
    {
        for (int i = 1; i <= 60; i++)
        {
            float curvature = 0.0005f * i;
            if (SteadyCircle(car, tires, speed, curvature).Spun)
                return curvature;
        }
        return 0.0005f * 61;
    }

    private static void HoldSpeed(
        CarState state,
        CarConfig car,
        TireConfig tires,
        float curvature,
        float speed,
        int steps
    )
    {
        for (int i = 0; i < steps; i++)
            StepHoldingSpeed(state, car, tires, curvature, speed);
    }

    /// <summary>
    /// A corner taken at a constant speed - a skidpad, which is how a
    /// chassis is actually measured.
    ///
    /// The speed is held rather than chased, because at the limit no
    /// throttle holds it: the cornering scrub alone takes more than the
    /// car makes, so a car left to find its own speed spirals inwards and
    /// every reading is of a different corner. The drive is still applied
    /// so the weight sits where driving through a corner puts it.
    /// </summary>
    private static void StepHoldingSpeed(
        CarState state,
        CarConfig car,
        TireConfig tires,
        float curvature,
        float speed
    )
    {
        float throttle = Math.Clamp((speed - state.Speed) * 4f, -5f, 8f);
        CarPhysics.Step(
            state,
            car,
            tires,
            PhysicsInput(new DriverInput(curvature, throttle)),
            Dt
        );
        if (!state.Spinning)
            state.Speed = speed;
    }

    private static CarState HeldSideways(
        CarConfig car,
        TireConfig tires,
        float sideslip,
        float seconds
    )
    {
        CarState state = CreateState(55f, tires);
        int steps = (int)MathF.Round(seconds / Dt);
        for (int i = 0; i < steps && !state.Spinning; i++)
        {
            state.SideslipAngleRadians = sideslip;
            CarPhysics.Step(
                state, car, tires, PhysicsInput(new DriverInput(0f, 0f)), Dt
            );
        }
        return state;
    }

    private static void Settle(
        CarState state,
        CarConfig car,
        TireConfig tires,
        DriverInput input,
        int steps
    )
    {
        for (int i = 0; i < steps; i++)
            CarPhysics.Step(state, car, tires, PhysicsInput(input), Dt);
    }

    private static CarState CreateState(float speed, TireConfig tires)
    {
        CarState state = new()
        {
            Speed = speed,
            Energy = PowertrainState.Filled(0.6f)
        };
        state.InstallFreshTires(tires);
        return state;
    }

    private static TireConfig WarmTires() => new()
    {
        StartingSurfaceTempC = 90f,
        StartingCoreTempC = 90f
    };

    private static CarPhysicsStepInput PhysicsInput(DriverInput input) =>
        new(input, CarStrategy.Default, TestAirTempC, TestTrackTempC);
}
