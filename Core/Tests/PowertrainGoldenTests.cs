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
            "speed 13.40875\n" +
            "position 172.9271 207.5298\n" +
            "heading -0.15525547\n" +
            "sideslip -0.00018292625\n" +
            "yawrate 0.20515737\n" +
            "charge 0.799117\n" +
            "drivepower 129564.68\n" +
            "regenpower 0\n" +
            "longaccel 10.55054\n" +
            "lataccel 2.554152\n" +
            "wear 0.0075465534 0.009750845 0.009860357 0.022910213\n" +
            "coretemp 86.27512 87.162285 89.81326 91.706314";

        public const string SaggingPack =
            "speed 9.058744\n" +
            "position 256.44012 192.68784\n" +
            "heading -0.8529291\n" +
            "sideslip 0.0025448594\n" +
            "yawrate 0.08742024\n" +
            "charge 0.14941761\n" +
            "drivepower 57140.273\n" +
            "regenpower 0\n" +
            "longaccel 6.8621855\n" +
            "lataccel 1.0504482\n" +
            "wear 0.011916569 0.015075512 0.01883422 0.058874045\n" +
            "coretemp 87.686325 88.82643 91.616196 93.5386";

        public const string AttackLadder =
            "speed 13.650756\n" +
            "position 165.57838 217.45422\n" +
            "heading 0.11490971\n" +
            "sideslip -0.0004500112\n" +
            "yawrate 0.20920505\n" +
            "charge 0.79911155\n" +
            "drivepower 131903.16\n" +
            "regenpower 0\n" +
            "longaccel 10.540171\n" +
            "lataccel 2.6822414\n" +
            "wear 0.007575325 0.009807646 0.009863827 0.023253517\n" +
            "coretemp 86.28994 87.194145 89.94575 91.90892";

        public const string EmptyPack =
            "speed 1.5538739\n" +
            "position 106.76541 85.17931\n" +
            "heading -2.4786868\n" +
            "sideslip 0\n" +
            "yawrate 1.2285085\n" +
            "charge 0.00023203176\n" +
            "drivepower 1324.1598\n" +
            "regenpower 0\n" +
            "longaccel 0.7623942\n" +
            "lataccel 1.892148\n" +
            "wear 0.0016786164 0.0023556852 0.0021549093 0.0030735934\n" +
            "coretemp 84.74505 85.53445 91.22737 92.25324";
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
