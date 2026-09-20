using System.Globalization;
using System.Text;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The car was pulled apart so a petrol or hybrid car could be dropped in
/// beside the electric one. Nothing about the electric car was supposed to
/// change while that happened, and "supposed to" is not a thing a physics
/// model should be trusted about: moving where a number is computed can
/// reorder a multiply and shift the last bit, and a shifted last bit
/// compounds over a race.
///
/// So these trajectories are pinned to every digit a float has. They are
/// recorded from the model as it stood before the powertrain became an
/// interface, and any difference at all — including one the eye would call
/// rounding — is a real change to the car and has to be argued for rather
/// than discovered later.
///
/// The four runs between them touch every path the powertrain owns: the
/// force ceiling at low speed, the power ceiling at high speed, regeneration
/// under braking, the sag of a pack low enough to be limited, and a pack
/// with nothing left in it at all.
///
/// Re-recorded on 2026-09-07 and again on 2026-09-08 when the physics
/// batch was frozen, and here is the argument the comment above demands. The car's lateral force stopped being handed over on
/// request and started coming from slip angles, so these trajectories move
/// for a reason that has nothing to do with the powertrain: the same
/// steering input now draws a different line, the car spends a different
/// amount of its lap cornering, and everything downstream of the path -
/// where it ends up, how fast, how much charge it used getting there -
/// follows. Nothing in the powertrain changed, and that is exactly what
/// these pins are for: if the next re-record cannot point at a change of
/// this size somewhere else, it is a bug.
///
/// Re-recorded again on 2026-09-09, and this time the powertrain is what
/// moved. Two changes, both deliberate. The pack is 1100 MJ instead of
/// 1470, because measurement said the old one let the car go flat out for
/// a whole race and finish with charge to spare — every charge figure in
/// these trajectories therefore falls faster, in exact proportion. And a
/// nearly empty pack no longer fades to nothing: it lands on a limp
/// floor, so the empty-pack run, which used to be a car sitting still, is
/// now a car crawling. Those two account for every digit that moved here;
/// the drive-power figures at full charge are unchanged, which is the
/// check that nothing else was disturbed.
///
/// And once more in the same batch, for the tyres. Lateral tread heat
/// became force times sliding instead of force squared, so temperatures
/// move, grip follows temperature, and the line the car draws follows
/// grip. Sub-limit temperatures were held to within 0.15 C of what they
/// were, which SlipBasedHeatKeepsTheTemperaturesTheCarWasCalibratedTo
/// checks directly, so what moves here is a tenth of a degree compounding
/// over twenty seconds of driving — plus the empty-pack run, which now
/// crawls on the limp floor where it used to sit still.
///
/// And a fourth time, in the same batch, for the thermal time constant.
/// The car could not warm a cold tyre inside an episode — two laps from
/// 25 C reached 59 C against a working floor of 85 — so every thermal
/// mass was divided by the same factor. No equilibrium moved and only the
/// journey to it got shorter, which is checked directly: the same duty
/// cycle at the old constant reads 95.33 at five minutes, 87.22 at twenty
/// and 85.65 at an hour, still descending towards the 84.91 the new one
/// reaches in three. What moves here is twenty seconds of driving during
/// which the tyres now change temperature five times as fast, and grip
/// follows temperature.
///
/// Re-recorded on 2026-09-11, for the rear tyres. They had a slip-angle
/// heater of their own on top of the force-times-sliding heat all four
/// tyres get, billing the same sliding twice. With it, the mixed-driving
/// and sagging-pack runs took a rear tread to 142 C and 154 C inside
/// twenty seconds and held the right rear above 115 C for 3.6 and 5.6
/// seconds; without it they peak at 103 C and 115 C. That is the whole of
/// what moved: rear core temperatures fall by four to seven degrees, and
/// the right-rear wear falls by nearly half and nearly three quarters in
/// those two runs, because the hot-wear multiplier it was running under --
/// five times at 150 C -- is gone. The front tread peaks are within a
/// degree of what they were, and the charge and drive-power figures move
/// only as far as the line the car draws.
///
/// And again the same day, for the rear tyres' wear, which had the same
/// fossil: a rear-only slip-angle wear term beside the force-times-scrub
/// wear all four tyres get. Rear wear in these runs falls by 15 to 32%;
/// front wear moves by under one per cent and the core temperatures by
/// under a twentieth of a degree, and what the line does after that is
/// grip following wear.
///
/// Re-recorded on 2026-09-16 after removing PR#56's pedal reflex and the
/// driver-efficiency/settle inputs from the vehicle boundary. The expected
/// trajectories are the pre-reflex recordings, regenerated with
/// STINTEGY_GOLDEN_DUMP rather than made tolerant: removing those controls is
/// the intentional behavior change, while the remaining vehicle equations
/// retain their default-equivalent factors of 1, 1, and infinity.
///
/// Re-recorded on 2026-09-18 when the pedal trim returned as a device on
/// the car -- the combined-grip limiter, published on CarConfig -- and the
/// old traction control and anti-lock were removed in its favour. Both are
/// intentional behaviour changes; the recordings were regenerated with
/// STINTEGY_GOLDEN_DUMP, not made tolerant.
///
/// Re-recorded on 2026-09-20 for the cornering drag. Two approximations of
/// one mechanism -- a rate times the square of lateral utilisation, and the
/// body's sideslip against its lateral acceleration -- were replaced by the
/// mechanism itself: each axle's lateral force times the sine of the angle
/// that axle is dragged at. These runs saw both of the old terms at once,
/// so what moves here is mostly the double charge coming off: the mixed
/// lap ends at 10.50 m/s where it used to end at 8.80, and the sagging-pack
/// run at 12.47 where it was 7.60. The empty-pack run barely moves (1.57
/// against 1.56) because a car on the limp floor is not cornering hard
/// enough for any of this to matter. Charge and drive power follow the
/// line the car draws, as they always do here; nothing in the powertrain
/// changed.
///
/// What these pins are for, stated once so the next re-record does not
/// have to argue it from the beginning: they guard refactoring. A change
/// that is meant to leave the car alone -- moving code, splitting a class,
/// reordering a sum -- must leave every digit here where it was. A change
/// that is meant to alter the car will move them, and the answer to that
/// is to re-record with the reason written above, not to hold the physics
/// still. What must survive a deliberate change is checked by tests that
/// state a behaviour -- conservation, the warm-up band, not going through
/// a wall -- rather than by these.
/// </summary>
public sealed class PowertrainGoldenTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void AHardLapOfMixedDrivingIsUnchanged()
    {
        Assert.Equal(Expected.MixedDriving, RunTrajectory(30f, 0.8f, CarStrategy.Default, 1200));
    }

    [Fact]
    public void ASaggingPackAtSpeedIsUnchanged()
    {
        // Starts inside the low-charge limiter and fast enough that power,
        // not grip, is what the car runs out of - so the sag term is doing
        // the deciding for the whole run rather than being masked by a
        // force ceiling the car could not reach anyway.
        Assert.Equal(
            Expected.SaggingPack,
            RunTrajectory(60f, 0.15f, CarStrategy.Default, 1200)
        );
    }

    [Fact]
    public void TheTopOfTheOutputLadderIsUnchanged()
    {
        // Attack raises the power ceiling and nothing else, so this run
        // separates from the first one only where power was the limit.
        Assert.Equal(
            Expected.AttackLadder,
            RunTrajectory(
                30f,
                0.8f,
                new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Attack),
                1200
            )
        );
    }

    [Fact]
    public void AnEmptyPackIsUnchanged()
    {
        // Nothing left at all, which is its own branch: the car cannot drive
        // until braking has put something back, and then it can. Worth
        // pinning because it is the one path where the powertrain refuses
        // outright rather than merely holding the car back.
        Assert.Equal(
            Expected.EmptyPack,
            RunTrajectory(40f, 0f, CarStrategy.Default, 1200)
        );
    }

    /// <summary>
    /// First recorded from the model as it stood before the powertrain
    /// became an interface; every re-record since is argued in the class
    /// summary. Regenerate by running these tests with STINTEGY_GOLDEN_DUMP
    /// set to a file path, and only with a reason that is written down.
    /// </summary>
    private static class Expected
    {
        public const string MixedDriving =
            "speed 10.503875\n" +
            "position 119.785484 232.3097\n" +
            "heading 0.8034462\n" +
            "sideslip 0.00399669\n" +
            "yawrate 0.14844604\n" +
            "charge 0.7992362\n" +
            "drivepower 94497.25\n" +
            "regenpower 0\n" +
            "longaccel 9.8545265\n" +
            "lataccel 2.3245997\n" +
            "wear 0.006140316 0.008229952 0.0065681036 0.008362908\n" +
            "coretemp 86.072815 87.0068 86.05677 86.87644";

        public const string SaggingPack =
            "speed 12.469586\n" +
            "position 251.94646 206.88443\n" +
            "heading 0.2675818\n" +
            "sideslip 0.0117959445\n" +
            "yawrate 0.16288356\n" +
            "charge 0.14951797\n" +
            "drivepower 78888.02\n" +
            "regenpower 0\n" +
            "longaccel 6.778231\n" +
            "lataccel 2.536681\n" +
            "wear 0.010479987 0.013529259 0.010252206 0.0121586\n" +
            "coretemp 87.18694 88.355675 86.784485 87.67511";

        public const string AttackLadder =
            "speed 11.185181\n" +
            "position 104.26694 234.18494\n" +
            "heading 1.0435033\n" +
            "sideslip 0.0031593104\n" +
            "yawrate 0.155935\n" +
            "charge 0.7992547\n" +
            "drivepower 101183.15\n" +
            "regenpower 0\n" +
            "longaccel 9.921828\n" +
            "lataccel 1.0404942\n" +
            "wear 0.006109137 0.008212671 0.0064976853 0.00832536\n" +
            "coretemp 85.99094 86.961586 85.97046 86.83836";

        public const string EmptyPack =
            "speed 1.5746721\n" +
            "position 106.40075 84.171135\n" +
            "heading -2.7751317\n" +
            "sideslip 0\n" +
            "yawrate 0.96750796\n" +
            "charge 0.0002377593\n" +
            "drivepower 1341.8473\n" +
            "regenpower 0\n" +
            "longaccel 0.7778206\n" +
            "lataccel 1.5093746\n" +
            "wear 0.0017717066 0.0024435304 0.0015242929 0.0021419\n" +
            "coretemp 84.81786 85.59239 84.38617 85.035385";
    }

    /// <summary>
    /// Drives a car for a fixed number of steps on a demand that swings
    /// through full throttle, trailing brake and both directions of lock, and
    /// reports where it ended up. No track: this is the vehicle model alone,
    /// so nothing in the answer depends on the road code.
    /// </summary>






    private static string RunTrajectory(
        float startSpeed,
        float startCharge,
        CarStrategy strategy,
        int steps
    )
    {
        CarConfig config = new();
        TireConfig tires = new()
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        };
        CarState state = new()
        {
            Speed = startSpeed,
            Energy = PowertrainState.Filled(startCharge)
        };
        state.InstallFreshTires(tires);

        for (int i = 0; i < steps; i++)
        {
            // Two periods that do not divide each other, so the car is never
            // asked for the same combination of grip and power twice.
            float phase = i / 120f;
            float accel = MathF.Sin(phase * 2.1f) * 20f;
            float curvature = MathF.Sin(phase * 0.7f) * 0.02f;
            CarPhysics.Step(
                state,
                config,
                tires,
                new CarPhysicsStepInput(
                    new DriverInput(curvature, accel),
                    strategy,
                    28f
                ),
                Dt
            );
        }

        CarTelemetry telemetry = state.Telemetry;
        StringBuilder report = new();
        Line(report, "speed", state.Speed);
        Line(report, "position", state.Position.X, state.Position.Y);
        Line(report, "heading", state.Heading);
        Line(report, "sideslip", state.SideslipAngleRadians);
        Line(report, "yawrate", state.YawRateRadiansPerSecond);
        Line(report, "charge", state.Energy.Primary);
        Line(report, "drivepower", telemetry.DrivePowerWatts);
        Line(report, "regenpower", telemetry.RegenPowerWatts);
        Line(report, "longaccel", telemetry.ActualLongitudinalAccel);
        Line(report, "lataccel", telemetry.ActualLateralAccel);
        Line(
            report,
            "wear",
            state.FrontLeft.Wear,
            state.FrontRight.Wear,
            state.RearLeft.Wear,
            state.RearRight.Wear
        );
        Line(
            report,
            "coretemp",
            state.FrontLeft.CoreTempC,
            state.FrontRight.CoreTempC,
            state.RearLeft.CoreTempC,
            state.RearRight.CoreTempC
        );
        string result = report.ToString().TrimEnd('\n');
        string? dump = Environment.GetEnvironmentVariable("STINTEGY_GOLDEN_DUMP");
        if (!string.IsNullOrEmpty(dump))
        {
            System.IO.File.AppendAllText(
                dump,
                $"=== {startSpeed} {startCharge} {strategy.PowerRung} {steps}\n" +
                result + "\n"
            );
        }
        return result;
    }

    private static void Line(
        StringBuilder report,
        string label,
        params float[] values
    )
    {
        report.Append(label);
        foreach (float value in values)
        {
            report.Append(' ');
            // Round trip, so a single changed bit shows up as a changed digit
            // instead of hiding under a display format.
            report.Append(value.ToString("R", CultureInfo.InvariantCulture));
        }
        report.Append('\n');
    }
}
