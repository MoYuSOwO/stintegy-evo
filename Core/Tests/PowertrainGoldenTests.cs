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
    /// Recorded from the model as it stood before the powertrain became an
    /// interface. Regenerate only with a reason that is written down.
    /// </summary>
    private static class Expected
    {
        public const string MixedDriving =
            "speed 13.530974\n" +
            "position 158.78868 219.77843\n" +
            "heading 0.65742946\n" +
            "sideslip -0.00018094783\n" +
            "yawrate 0.20909905\n" +
            "charge 0.79911804\n" +
            "drivepower 130745.53\n" +
            "regenpower 0\n" +
            "longaccel 10.545293\n" +
            "lataccel 2.6851478\n" +
            "wear 0.009863093 0.012245412 0.0118346 0.01550368\n" +
            "coretemp 90.38499 90.669106 90.69098 91.147644";

        public const string SaggingPack =
            "speed 9.31804\n" +
            "position 251.01974 198.16179\n" +
            "heading -0.661389\n" +
            "sideslip 0.0055259233\n" +
            "yawrate 0.10551673\n" +
            "charge 0.14940746\n" +
            "drivepower 58780.914\n" +
            "regenpower 0\n" +
            "longaccel 6.832253\n" +
            "lataccel 2.5289817\n" +
            "wear 0.020538472 0.027574878 0.021284401 0.032688405\n" +
            "coretemp 91.24614 91.5836 91.39253 91.87926";

        public const string AttackLadder =
            "speed 13.880993\n" +
            "position 157.68254 223.63878\n" +
            "heading 0.5272883\n" +
            "sideslip -0.0006604545\n" +
            "yawrate 0.21404772\n" +
            "charge 0.7991027\n" +
            "drivepower 134086.42\n" +
            "regenpower 0\n" +
            "longaccel 10.529843\n" +
            "lataccel 2.730351\n" +
            "wear 0.009899226 0.012295458 0.011910472 0.015613546\n" +
            "coretemp 90.389534 90.674324 90.703835 91.16355";

        public const string EmptyPack =
            "speed 1.5597687\n" +
            "position 105.777306 85.16254\n" +
            "heading -2.640342\n" +
            "sideslip 0\n" +
            "yawrate 1.0515409\n" +
            "charge 0.00023203857\n" +
            "drivepower 1329.17\n" +
            "regenpower 0\n" +
            "longaccel 0.7671234\n" +
            "lataccel 1.6251454\n" +
            "wear 0.0016517526 0.0023394222 0.0021250374 0.003034483\n" +
            "coretemp 89.25825 89.407196 89.856155 90.03744";
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
