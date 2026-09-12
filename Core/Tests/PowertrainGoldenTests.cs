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
/// Re-recorded on 2026-09-12 for the driver's reflex: the pedals are now
/// trimmed to the share of the tyre the pit wall allots, and these runs
/// sweep throttle and brake across their whole range under a mode that
/// allots 97.7% of it. So they move further than any re-record before.
/// The mixed-driving run ends at 6.7 m/s instead of 12.8 and its sideslip,
/// a thousandth of a radian, is now a ten-thousandth: the car is not being
/// asked for less, it is asking for exactly as much and being handed what
/// the mode allows. Wear falls with it, because a tyre that is never spun
/// up or locked is not being scrubbed.
///
/// And again the same day, for the rear tyres' wear, which had the same
/// fossil: a rear-only slip-angle wear term beside the force-times-scrub
/// wear all four tyres get. Rear wear in these runs falls by 15 to 32%;
/// front wear moves by under one per cent and the core temperatures by
/// under a twentieth of a degree, and what the line does after that is
/// grip following wear.
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
            "speed 6.675524\n" +
            "position 119.51982 217.39804\n" +
            "heading 1.0010049\n" +
            "sideslip -0.00023420347\n" +
            "yawrate -0.0007249452\n" +
            "charge 0.79929245\n" +
            "drivepower 53890.246\n" +
            "regenpower 0\n" +
            "longaccel 8.9404125\n" +
            "lataccel 0.40589708\n" +
            "wear 0.0062936083 0.008368016 0.0065273875 0.008280306\n" +
            "coretemp 85.8954 86.86655 85.7752 86.601074";

        public const string SaggingPack =
            "speed 4.515425\n" +
            "position 228.53156 222.24359\n" +
            "heading 0.7418249\n" +
            "sideslip -0\n" +
            "yawrate -0.848187\n" +
            "charge 0.14948115\n" +
            "drivepower 12868.681\n" +
            "regenpower 0\n" +
            "longaccel 3.015591\n" +
            "lataccel -0.6528294\n" +
            "wear 0.011361726 0.014382173 0.0111212805 0.013062157\n" +
            "coretemp 87.407295 88.59324 87.0251 87.97709";

        public const string AttackLadder =
            "speed 5.7899113\n" +
            "position 113.00772 220.93668\n" +
            "heading 1.1348377\n" +
            "sideslip -1.000687E-05\n" +
            "yawrate 0.032037795\n" +
            "charge 0.7992953\n" +
            "drivepower 42137.805\n" +
            "regenpower 0\n" +
            "longaccel 8.048885\n" +
            "lataccel 0.8672846\n" +
            "wear 0.00625618 0.008330417 0.006495113 0.008250059\n" +
            "coretemp 85.90422 86.87715 85.78806 86.6159";

        public const string EmptyPack =
            "speed 1.5601083\n" +
            "position 101.256935 90.43726\n" +
            "heading 2.8561602\n" +
            "sideslip 0\n" +
            "yawrate 0.97336274\n" +
            "charge 0.00023053592\n" +
            "drivepower 1329.4596\n" +
            "regenpower 0\n" +
            "longaccel 0.7672162\n" +
            "lataccel 1.5045261\n" +
            "wear 0.0020320385 0.002716252 0.0016325803 0.0022563473\n" +
            "coretemp 84.85489 85.65209 84.47284 85.13138";
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
