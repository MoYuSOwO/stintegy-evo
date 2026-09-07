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
            "speed 18.824127\n" +
            "position -31.116688 248.81851\n" +
            "heading 1.5834359\n" +
            "sideslip 0.004505396\n" +
            "yawrate 0.24032454\n" +
            "charge 0.7991771\n" +
            "drivepower 196852.44\n" +
            "regenpower 0\n" +
            "longaccel 11.212663\n" +
            "lataccel 4.5334196\n" +
            "wear 0.0047271955 0.007471319 0.0072629647 0.011266288\n" +
            "coretemp 90.13731 90.672844 90.26405 90.91436";

        public const string SaggingPack =
            "speed 12.177776\n" +
            "position -54.470325 198.52911\n" +
            "heading 2.4299796\n" +
            "sideslip 0.0032548904\n" +
            "yawrate 0.15684499\n" +
            "charge 0.14967\n" +
            "drivepower 72625.04\n" +
            "regenpower 0\n" +
            "longaccel 6.389864\n" +
            "lataccel 1.9018174\n" +
            "wear 0.009106642 0.015658716 0.010837926 0.026633725\n" +
            "coretemp 90.428116 91.309296 90.71517 91.82905";

        public const string AttackLadder =
            "speed 18.823484\n" +
            "position -33.95673 248.97173\n" +
            "heading 1.5930907\n" +
            "sideslip 0.004504323\n" +
            "yawrate 0.24031371\n" +
            "charge 0.79916584\n" +
            "drivepower 196841.52\n" +
            "regenpower 0\n" +
            "longaccel 11.212446\n" +
            "lataccel 4.5331087\n" +
            "wear 0.00482691 0.0076248446 0.0074113524 0.011534613\n" +
            "coretemp 90.15514 90.69805 90.28953 90.95318";

        public const string EmptyPack =
            "speed 1.6715203E-05\n" +
            "position 104.57055 80.86308\n" +
            "heading 1.9105024\n" +
            "sideslip 0\n" +
            "yawrate 0\n" +
            "charge 0.00016553534\n" +
            "drivepower 1.2197106E-07\n" +
            "regenpower 0\n" +
            "longaccel 8.220584E-06\n" +
            "lataccel 3.5870439E-12\n" +
            "wear 0.0013105738 0.0020760307 0.0011134592 0.0017888439\n" +
            "coretemp 89.24304 89.43001 89.134186 89.27047";
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
