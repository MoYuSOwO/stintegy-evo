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
/// Re-recorded once, on 2026-09-07, and here is the argument the comment
/// above demands. The car's lateral force stopped being handed over on
/// request and started coming from slip angles, so these trajectories move
/// for a reason that has nothing to do with the powertrain: the same
/// steering input now draws a different line, the car spends a different
/// amount of its lap cornering, and everything downstream of the path -
/// where it ends up, how fast, how much charge it used getting there -
/// follows. Nothing in the powertrain changed, and that is exactly what
/// these pins are for: if the next re-record cannot point at a change of
/// this size somewhere else, it is a bug.
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
            "speed 14.383302\n" +
            "position 155.72798 224.89479\n" +
            "heading 0.75618255\n" +
            "sideslip -0.0012083551\n" +
            "yawrate 0.2221503\n" +
            "charge 0.79933155\n" +
            "drivepower 139043.88\n" +
            "regenpower 0\n" +
            "longaccel 10.516269\n" +
            "lataccel 2.9792864\n" +
            "wear 0.0022059116 0.003671374 0.004188899 0.006723498\n" +
            "coretemp 89.333855 89.50403 89.754364 90.11687";

        public const string SaggingPack =
            "speed 8.7776785\n" +
            "position 246.60391 202.65977\n" +
            "heading 0.36257264\n" +
            "sideslip 0.002409677\n" +
            "yawrate 0.16459456\n" +
            "charge 0.1495791\n" +
            "drivepower 52192.832\n" +
            "regenpower 0\n" +
            "longaccel 6.4349647\n" +
            "lataccel 2.67442\n" +
            "wear 0.0022874032 0.0038749138 0.0037261834 0.0062735025\n" +
            "coretemp 89.33142 89.51966 89.72158 90.0773";

        public const string AttackLadder =
            "speed 14.582245\n" +
            "position 163.30225 217.36243\n" +
            "heading 0.5354826\n" +
            "sideslip -0.0014949912\n" +
            "yawrate 0.22522388\n" +
            "charge 0.79932326\n" +
            "drivepower 140895.89\n" +
            "regenpower 0\n" +
            "longaccel 10.50243\n" +
            "lataccel 3.047317\n" +
            "wear 0.0021987236 0.0036625804 0.004175864 0.006719463\n" +
            "coretemp 89.33616 89.505806 89.73039 90.092064";

        public const string EmptyPack =
            "speed 1.46651455E-05\n" +
            "position 109.38712 78.010605\n" +
            "heading 1.643721\n" +
            "sideslip 0\n" +
            "yawrate 0\n" +
            "charge 0.00015505234\n" +
            "drivepower 9.388729E-08\n" +
            "regenpower 0\n" +
            "longaccel 7.2123694E-06\n" +
            "lataccel 5.366446\n" +
            "wear 0.0011343651 0.0017199669 0.0013666197 0.0021131495\n" +
            "coretemp 89.18301 89.323746 89.27645 89.43045";
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
