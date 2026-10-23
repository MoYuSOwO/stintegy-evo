using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// Calibrates the energy and tyre rungs of the circuits named in
/// <c>STINTEGY_CALIBRATE</c> (comma separated) and writes each circuit's
/// folder, with the new rungs in its driver.json, under the probe directory
/// in <c>calibrated/</c>. Run on purpose:
/// <c>STINTEGY_LANE_PROBES=1 STINTEGY_CALIBRATE=silverstone dotnet test -c Release --filter RungCalibration</c>.
/// </summary>
public sealed class RungCalibration(ITestOutputHelper output)
{
    [Fact]
    public void CalibrateTheRungs()
    {
        if (!LaneProbes.Enabled || Environment.GetEnvironmentVariable("STINTEGY_CALIBRATE") is not { Length: > 0 } list) return;
        string? progress = Environment.GetEnvironmentVariable("STINTEGY_CALIBRATE_PROGRESS");
        void Log(string line)
        {
            output.WriteLine(line);
            if (progress is not null)
                File.AppendAllText(progress, line + "\n");
        }

        var calibrator = new RungCalibrator();
        foreach (string id in list.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
        {
            TrackData track = CircuitLibrary.Load(id);
            LaneTrackFile shipped = LaneTrackFile.ForCircuit(id, track);
            RungCalibrator.Result result = calibrator.Calibrate(id, track, shipped, Log);
            LaneTrackFile calibrated = shipped.Clone();
            calibrated.Energy = result.Energy;
            calibrated.Tyres = result.Tyres;
            calibrated.Calibration = result.Notes;
            string folder = Path.Combine(LaneProbes.Dir, "calibrated", id);
            calibrated.SaveCircuit(folder, id);
            Log($"{id}: energy {string.Join(" / ", result.Energy.LiftSeconds().Select(x => x.ToString("F2")))} s  ({result.Notes.Energy})");
            Log($"{id}: tyres  {string.Join(" / ", result.Tyres.Table().Select(s => s.Corner.ToString("F4")))} corner  ({result.Notes.Tyres})");
            Log($"{id}: wrote {folder}");
        }
    }
}

public sealed class RungCalibratorTests
{
    private static RungCalibrator.Sample S(float setting, float lap, float pack = 0f, float wear = 0f) =>
        new(setting, lap, pack, wear, 0, 0f);

    // Lifting longer saves charge and costs time, as on a real circuit.
    private static readonly RungCalibrator.Sample[] Lifts =
    [
        S(0f, 90.0f, 0.020f), S(0.5f, 90.3f, 0.014f), S(1.0f, 90.7f, 0.009f), S(1.5f, 91.2f, 0.005f)
    ];

    [Fact]
    public void NormalSpendsThePackOverTheRaceAndTheRungsStayWithinASpread()
    {
        var calibrator = new RungCalibrator();
        (float[] lift, string note) = calibrator.SolveEnergy(Lifts, raceLaps: 80);
        // Normal drains the usable pack over the race: 0.95 / 80 a lap.
        float drain = Linear(lift[2], Lifts.Select(s => s.Setting).ToArray(), Lifts.Select(s => s.PackPerLap).ToArray());
        Near(0.95f / 80f, drain, 0.0005f);
        for (int i = 1; i < 5; i++)
            Assert.True(lift[i] <= lift[i - 1], note);
        float slowest = Linear(lift[0], Lifts.Select(s => s.Setting).ToArray(), Lifts.Select(s => s.LapSeconds).ToArray());
        float fastest = Linear(lift[4], Lifts.Select(s => s.Setting).ToArray(), Lifts.Select(s => s.LapSeconds).ToArray());
        Assert.True((slowest - fastest) / 90f <= 0.0101f, note);
    }

    [Fact]
    public void ACircuitTheGeneratorCannotKeepUpWithSaysSo()
    {
        (float[] lift, string note) = new RungCalibrator().SolveEnergy(Lifts, raceLaps: 300);
        Assert.Contains("short", note);
        Assert.Equal(1.5f, lift[2]);
    }

    [Fact]
    public void ACircuitWithChargeToSpareNeedsNoLift()
    {
        (float[] lift, string note) = new RungCalibrator().SolveEnergy(Lifts, raceLaps: 20);
        Assert.Contains("surplus", note);
        Assert.Equal(0f, lift[2]);
        Assert.Equal(0f, lift[4]);
    }

    [Fact]
    public void GentlerRungsWearLessAndAttackIsThePlanAsLearned()
    {
        RungCalibrator.Sample[] sweep =
        [
            S(0f, 90.0f, wear: 0.020f), S(0.25f, 90.2f, wear: 0.016f), S(0.5f, 90.4f, wear: 0.012f),
            S(0.75f, 90.6f, wear: 0.009f), S(1f, 90.8f, wear: 0.007f)
        ];
        (float[] g, string note) = new RungCalibrator().SolveTyres(sweep);
        Assert.Equal(0f, g[4]);
        for (int i = 1; i < 5; i++)
            Assert.True(g[i] <= g[i - 1], note);
        Assert.Contains("Attack 1.4", note);
    }

    private static float Linear(float at, float[] x, float[] y)
    {
        for (int i = 1; i < x.Length; i++)
            if (at <= x[i])
                return y[i - 1] + (y[i] - y[i - 1]) * (at - x[i - 1]) / (x[i] - x[i - 1]);
        return y[^1];
    }

    private static void Near(float expected, float actual, float allowed) =>
        Assert.True(MathF.Abs(expected - actual) <= allowed, $"expected about {expected}, got {actual}");
}
