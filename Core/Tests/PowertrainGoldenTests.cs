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
/// Re-recorded on 2026-10-16 for the battery's low end. The pack used to
/// fade as the square of what was left from twenty per cent down, which had
/// a car at fifteen per cent on little more than half its power; it now
/// delivers full power through the usable window, sags to 85 per cent over
/// the last five per cent and limps at empty. Only the two runs that start
/// low move: the sagging-pack run now starts at three per cent (fifteen is
/// full power under the new rule) and the empty-pack run crawls a little
/// faster (2.43 m/s against 1.57) because the empty band blends towards the
/// limp rather than sitting on it. The mixed and Attack runs are unchanged
/// to the last digit, which is the check that nothing else moved.
///
/// Re-recorded on 2026-10-16 for the F1E car, which is now the default: a
/// range extender (a 30 kWh pack topped up by a 300 kW generator), 900 kg,
/// drive on both axles shared by the grip each has left, and tyre heat and
/// wear driven by sliding. Every digit moves because it is a different
/// car. Two things worth saying about the runs: the sagging-pack run now
/// climbs out of the sag, because the generator is refilling the pack
/// while it drives; and the empty-pack run has no fuel either, so it is
/// still a car living on what braking puts back. The Attack run moved to
/// the pure electric car, because the range extender races on one power
/// ceiling and on it Attack would only repeat the mixed run.
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
        // Starts inside the low-voltage sag at the bottom of the usable
        // window and fast enough that power, not grip, is what the car runs
        // out of - so the sag term is doing the deciding for the whole run
        // rather than being masked by a force ceiling the car could not reach
        // anyway. (Fifteen per cent until the sag moved to the last five.)
        Assert.Equal(
            Expected.SaggingPack,
            RunTrajectory(60f, 0.03f, CarStrategy.Default, 1200)
        );
    }

    [Fact]
    public void TheTopOfTheOutputLadderIsUnchanged()
    {
        // Attack raises the power ceiling and nothing else, so this run
        // separates from the first one only where power was the limit. On
        // the pure electric car, which is the one with a ladder of power
        // ceilings; the range extender races on one ceiling and leaves the
        // rungs to the driver.
        Assert.Equal(
            Expected.AttackLadder,
            RunTrajectory(
                30f,
                0.8f,
                new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Attack),
                1200,
                new CarConfig { Powertrain = ElectricPowertrain.Default }
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
            "speed 16.859566\n" +
            "position 220.96376 145.97528\n" +
            "heading -1.551475\n" +
            "sideslip 0.0070686173\n" +
            "yawrate 0.17618407\n" +
            "charge 0.8447239\n" +
            "drivepower 216296.84\n" +
            "regenpower 0\n" +
            "longaccel 11.523183\n" +
            "lataccel 2.7676978\n" +
            "wear 0.008535774 0.012809126 0.003314874 0.008056065\n" +
            "coretemp 89.389496 91.09053 88.14037 89.09536";

        public const string SaggingPack =
            "speed 17.44385\n" +
            "position 288.25803 166.86389\n" +
            "heading -1.524405\n" +
            "sideslip 0.0067390534\n" +
            "yawrate 0.18322404\n" +
            "charge 0.07588903\n" +
            "drivepower 204323.3\n" +
            "regenpower 0\n" +
            "longaccel 11.500067\n" +
            "lataccel 2.983499\n" +
            "wear 0.019311484 0.026255459 0.01445176 0.01985164\n" +
            "coretemp 92.825455 95.00944 90.99691 92.33881";

        public const string AttackLadder =
            "speed 17.366413\n" +
            "position 213.2151 144.39732\n" +
            "heading -1.5888377\n" +
            "sideslip 0.0064561157\n" +
            "yawrate 0.17913395\n" +
            "charge 0.79890907\n" +
            "drivepower 202741.31\n" +
            "regenpower 0\n" +
            "longaccel 11.502533\n" +
            "lataccel 2.896853\n" +
            "wear 0.006286549 0.010922485 0.0022490413 0.005544171\n" +
            "coretemp 88.535706 90.11254 87.43014 88.30958";

        public const string EmptyPack =
            "speed 10.766233\n" +
            "position 83.21116 147.31778\n" +
            "heading 0.5188052\n" +
            "sideslip 0.012505305\n" +
            "yawrate 0.119251505\n" +
            "charge 0.0027031489\n" +
            "drivepower 63189.426\n" +
            "regenpower 0\n" +
            "longaccel 5.7412767\n" +
            "lataccel 0.7238014\n" +
            "wear 0.0005450026 0.0007009862 0.0005355882 0.00068192906\n" +
            "coretemp 85.158325 85.93098 84.82786 85.46673";
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
        int steps,
        CarConfig? car = null
    )
    {
        CarConfig config = car ?? new();
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
