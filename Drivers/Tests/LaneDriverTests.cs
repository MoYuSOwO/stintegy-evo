using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// Probes sweep, learn and write tuning files; they take minutes and are run
/// on purpose: <c>STINTEGY_LANE_PROBES=1 dotnet test --filter ...</c>.
/// </summary>
internal static class LaneProbes
{
    public static bool Enabled => Environment.GetEnvironmentVariable("STINTEGY_LANE_PROBES") == "1";
    public static string Dir
    {
        get
        {
            string dir = Path.Combine(Path.GetTempPath(), "stintegy-lanes");
            Directory.CreateDirectory(dir);
            return dir;
        }
    }
}

public sealed class LaneDriverTests
{
    private readonly ITestOutputHelper _output;

    public LaneDriverTests(ITestOutputHelper output) => _output = output;

    private static LaneTrackFile Silverstone(TrackData track) =>
        LaneTrackFile.ForCircuit("silverstone", track);

    [Fact]
    public void BakesThreeLanesInsideTheRoad()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneSet lanes = LaneSet.Bake(track);
        float minRoom = float.PositiveInfinity;
        float leftOverRight = 0f;
        for (int i = 0; i < lanes.Count; i++)
        {
            float half = lanes.HalfWidth(i);
            foreach (LaneId lane in new[] { LaneId.Free, LaneId.Left, LaneId.Right })
                minRoom = MathF.Min(minRoom, half - MathF.Abs(lanes.Offset(lane, i)));
            leftOverRight += lanes.Offset(LaneId.Left, i) - lanes.Offset(LaneId.Right, i);
        }
        _output.WriteLine($"stations {lanes.Count} min room to edge {minRoom:F2} m, mean left-right gap {leftOverRight / lanes.Count:F2} m");
        Assert.True(minRoom >= 0.8f, $"a lane comes within {minRoom:F2} m of the edge");
        Assert.True(leftOverRight / lanes.Count > 1f);
    }

    [Fact]
    public void TheShippedSilverstoneFolderWasLearnedOnTodaysCar()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = Silverstone(track);
        Assert.True(file.IsCurrent,
            $"learned on {file.LearnedFor?.Physics ?? "an unstamped car"} ({file.LearnedFor?.Date}), the car is now {PhysicsFingerprint.Stock}: relearn it");
    }

    [Fact]
    public void ACircuitFolderReadsBackAsWritten()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = Silverstone(track);
        file.MarkLearnedNow();
        file.Energy = LaneEnergyRungs.From([1f, 0.6f, 0.4f, 0.3f, 0f], raceLaps: 52);
        string folder = Path.Combine(Path.GetTempPath(), "stintegy-circuit-roundtrip", Guid.NewGuid().ToString("N"));
        file.SaveCircuit(folder, "silverstone");
        LaneTrackFile back = LaneTrackFile.LoadCircuit(folder, track);
        Assert.True(back.IsCurrent);
        Assert.Equal(file.Energy, back.Energy);
        Assert.Equal(file.Factors(LaneId.Free), back.Factors(LaneId.Free));
        Directory.Delete(folder, recursive: true);
    }

    [Fact]
    public void TheShippedSilverstoneFileLapsUnder142()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        Assert.NotNull(RuleLineAssets.FindFile("f1e/silverstone/driver.json"));
        LaneLapResult r = LaneLapRunner.Run(track, Silverstone(track));
        _output.WriteLine(r.ToString());
        Assert.True(r.Finished, r.ToString());
        Assert.Equal(0f, r.OffTrackSeconds);
        Assert.InRange(r.FlyingLapSeconds, 95f, 102f);
    }

    [Fact]
    public void ThreeFlyingLapsWithoutASpinAsTheTyresWear()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneLapResult r = LaneLapRunner.Run(track, Silverstone(track), flyingLaps: 3);
        _output.WriteLine(string.Join(' ', r.LapTimes.Select(t => t.ToString("F2"))));
        Assert.Equal(0, r.Spins);
        Assert.Equal(3, r.LapTimes.Count);
        Assert.Equal(0f, r.OffTrackSeconds);
    }

    [Theory]
    [InlineData(1f, 1f)]
    [InlineData(1f, -1f)]
    [InlineData(1f, 0f)]
    public void AWholeLapOnASideLaneOrBetweenThem(float range, float side)
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneLapResult r = LaneLapRunner.Run(track, Silverstone(track), onStep: (car, d, t) =>
        {
            d.TargetRange = range;
            d.TargetSide = side;
        });
        _output.WriteLine($"range {range} side {side}: {r}");
        Assert.True(r.Finished, r.ToString());
        Assert.Equal(0f, r.OffTrackSeconds);
    }

    [Fact]
    public void SwitchingLanesEveryEightSecondsStaysClean()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneLapResult r = LaneLapRunner.Run(track, Silverstone(track), onStep: (car, d, t) =>
        {
            int phase = (int)(t / 8f) % 4;
            d.TargetRange = phase == 0 ? 0f : 1f;
            d.TargetSide = phase == 1 ? 1f : phase == 2 ? 0f : -1f;
        });
        _output.WriteLine(r.ToString());
        Assert.True(r.Finished, r.ToString());
        Assert.True(r.WorstErrorMeters < 3f, r.ToString());
    }

    [Fact]
    public void AnUnlearnedCircuitStillGetsRound()
    {
        TrackData track = TrackFactory.MonzaStyleTestTrack();
        LaneLapResult r = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), startSpeed: 30f);
        _output.WriteLine(r.ToString());
        Assert.True(r.Finished, r.ToString());
    }
}

public sealed class LaneDriverTrace
{
    private readonly ITestOutputHelper _output;

    public LaneDriverTrace(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(1700f, 2050f)]
    public void TraceWindow(float from, float to)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.Fresh();
        int n = 0;
        LaneLapResult result = LaneLapRunner.Run(track, file, onStep: (car, d, t) =>
        {
            float s = car.Progress.CurrentS;
            if (s < from || s > to || n++ % 6 != 0)
                return;
            var tel = d.LastTelemetry;
            var phys = car.State.Telemetry;
            float plannedK = tel.FeedforwardCurvature;
            _output.WriteLine(
                $"t={t:F2} s={s:F0} v={car.State.Speed:F1} tgt={tel.TargetSpeed:F1} e={tel.LateralErrorMeters:+0.00;-0.00} " +
                $"he={tel.HeadingErrorRadians:+0.000;-0.000} kff={plannedK:+0.0000;-0.0000} kcmd={tel.CommandCurvature:+0.0000;-0.0000} " +
                $"kact={phys.ActualCurvature:+0.0000;-0.0000} slip={car.State.SideslipAngleRadians:+0.000;-0.000} " +
                $"steer={car.State.SteerAngleRadians:+0.000;-0.000} ay={phys.ActualLateralAccel:F1}/{phys.RequestedLateralAccel:F1} " +
                $"use f={phys.FrontLateralUse:F2}/{phys.FrontLongitudinalUse:F2} r={phys.RearLateralUse:F2}/{phys.RearLongitudinalUse:F2} " +
                $"rs={phys.RearSlideSeverity:F2} a={tel.CommandAccel:F1}");
        });
        _output.WriteLine(result.ToString());
    }
}

public sealed class LaneLearningRun
{
    private readonly ITestOutputHelper _output;
    public LaneLearningRun(ITestOutputHelper output) => _output = output;

    [Fact]
    public void LearnSilverstone()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var watch = System.Diagnostics.Stopwatch.StartNew();
        var learner = new LaneLearner { Reward = 0.02f };
        var (best, bestLap, history) = learner.Learn(track, LaneTrackFile.Fresh(), iterations: 200);
        foreach (var step in history.Where(h => h.Kept))
            _output.WriteLine($"{step.Iteration,3} {(step.Kept ? "*" : " ")} {step.Result}");
        _output.WriteLine($"best {bestLap}  ({watch.Elapsed.TotalSeconds:F0}s)");
        string dir = LaneProbes.Dir;
        best.Save(Path.Combine(dir, "silverstone.lanes.csv"));
    }
}

public sealed class LaneGainSweep
{
    private readonly ITestOutputHelper _output;
    public LaneGainSweep(ITestOutputHelper output) => _output = output;

    [Fact]
    public void SweepTrackingGains()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.Fresh();
        file.LanesFor(track);
        var rows = new List<(float Gap, string Line)>();
        foreach (float pursuit in new[] { 0.6f, 0.8f, 1.0f })
        foreach (float kPreview in new[] { 0.1f, 0.2f, 0.3f })
        foreach (float vPreview in new[] { 0.0f, 0.15f, 0.3f })
        foreach (float vGain in new[] { 3f, 6f })
        {
            var gains = new LaneDriverGains
            {
                PursuitSeconds = pursuit,
                CurvaturePreviewSeconds = kPreview,
                SpeedPreviewSeconds = vPreview,
                SpeedGain = vGain
            };
            LaneLapResult r = LaneLapRunner.Run(track, file, gains);
            float gap = r.Finished ? r.FlyingLapSeconds - r.PlannedLapSeconds : 99f;
            rows.Add((gap, $"pursuit {pursuit:F1} kprev {kPreview:F1} vprev {vPreview:F2} vgain {vGain:F0}: {r}"));
        }
        foreach (var row in rows.OrderBy(r => r.Gap).Take(15))
            _output.WriteLine($"gap {row.Gap:F2}  {row.Line}");
        _output.WriteLine($"failed {rows.Count(r => r.Gap >= 99f)} of {rows.Count}");
    }
}

public sealed class LaneLearningVariants
{
    private readonly ITestOutputHelper _output;
    public LaneLearningVariants(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(0.30f, 0.92f)]
    [InlineData(0.15f, 0.92f)]
    [InlineData(0.05f, 0.92f)]
    public void LearnWith(float brakePreview, float brakeShare)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var gains = new LaneDriverGains { BrakePreviewSeconds = brakePreview };
        var shares = new LaneShares { Brake = brakeShare };
        var (best, bestLap, history) = new LaneLearner { Reward = 0.02f }
            .Learn(track, LaneTrackFile.Fresh(), 160, gains, shares);
        _output.WriteLine($"brakePreview {brakePreview:F2} brakeShare {brakeShare:F2}: first {history[0].Result.FlyingLapSeconds:F2} best {bestLap}  kept@{string.Join(',', history.Where(h => h.Kept).Select(h => h.Iteration))}");
    }
}

public sealed class LaneEdgeVariants
{
    private readonly ITestOutputHelper _output;
    public LaneEdgeVariants(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(1.15f, 0.3f)]
    [InlineData(0.95f, 0.2f)]
    [InlineData(0.85f, 0.0f)]
    public void LearnWithEdge(float edge, float inner)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var file = LaneTrackFile.Fresh(new LaneBakeOptions { EdgeMargin = edge, MaxInnerMargin = inner });
        var (best, bestLap, history) = new LaneLearner { Reward = 0.02f }.Learn(track, file, 120);
        _output.WriteLine($"edge {edge:F2} inner {inner:F1}: first {history[0].Result} | best {bestLap}");
        string dir = LaneProbes.Dir;
        best.Save(Path.Combine(dir, $"silverstone-edge{edge:F2}.lanes.csv"));
    }
}

public sealed class LaneBrakePreviewSweep
{
    private readonly ITestOutputHelper _output;
    public LaneBrakePreviewSweep(ITestOutputHelper output) => _output = output;

    [Fact]
    public void SweepBrakePreviewOnTheLearnedFile()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        string path = Path.Combine(Path.GetTempPath(), "stintegy-lanes", "silverstone.lanes.csv");
        LaneTrackFile file = LaneTrackFile.Load(path, track);
        foreach (float brake in new[] { 0.3f, 0.2f, 0.1f, 0.05f, 0.0f })
        foreach (float drive in new[] { 0.3f, 0.15f })
        {
            var gains = new LaneDriverGains { BrakePreviewSeconds = brake, SpeedPreviewSeconds = drive };
            _output.WriteLine($"brake {brake:F2} drive {drive:F2}: {LaneLapRunner.Run(track, file, gains)}");
        }
    }
}

public sealed class LaneEdgeFileSweep
{
    private readonly ITestOutputHelper _output;
    public LaneEdgeFileSweep(ITestOutputHelper output) => _output = output;

    [Fact]
    public void SweepOnEdgeFile()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        string path = Path.Combine(Path.GetTempPath(), "stintegy-lanes", "silverstone-edge0.85.lanes.csv");
        LaneTrackFile file = LaneTrackFile.Load(path, track);
        var rows = new List<(float T, string L)>();
        foreach (float bp in new[] { 0.35f, 0.3f, 0.25f, 0.2f })
        foreach (float sp in new[] { 0.3f, 0.4f, 0.5f })
        foreach (float g in new[] { 4f, 6f, 9f })
        foreach (float pursuit in new[] { 0.5f, 0.6f, 0.7f })
        {
            var gains = new LaneDriverGains { BrakePreviewSeconds = bp, SpeedPreviewSeconds = sp, SpeedGain = g, PursuitSeconds = pursuit };
            LaneLapResult r = LaneLapRunner.Run(track, file, gains);
            rows.Add((r.Finished && r.OffTrackSeconds == 0f ? r.FlyingLapSeconds : 999f, $"bp {bp:F2} sp {sp:F2} g {g:F0} pursuit {pursuit:F1}: {r}"));
        }
        foreach (var row in rows.OrderBy(r => r.T).Take(10))
            _output.WriteLine(row.L);
        _output.WriteLine($"clean {rows.Count(r => r.T < 999f)} of {rows.Count}");
    }
}

public sealed class LaneShortfallProbe
{
    private readonly ITestOutputHelper _output;
    public LaneShortfallProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void WhereTheLearnedLapLosesTime()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        string path = Path.Combine(Path.GetTempPath(), "stintegy-lanes", "silverstone.lanes.csv");
        LaneTrackFile file = File.Exists(path) ? LaneTrackFile.Load(path, track) : LaneTrackFile.Fresh();
        LaneSet lanes = file.LanesFor(track);
        // Phase buckets: time lost against the plan while braking, mid-corner, accelerating, straight.
        var lost = new Dictionary<string, float> { ["brake"] = 0, ["corner"] = 0, ["drive"] = 0, ["straight"] = 0 };
        var bySector = new float[24];
        bool flying = false;
        int lastLap = -1;
        LaneLapResult r = LaneLapRunner.Run(track, file, onStep: (car, d, t) =>
        {
            if (lastLap < 0) lastLap = car.Progress.Lap;
            if (car.Progress.Lap != lastLap) { flying = !flying || flying; lastLap = car.Progress.Lap; }
            if (!flying) return;
            float s = car.Progress.CurrentS;
            float plan = MathF.Max(d.PlannedSpeedAt(s), 1f);
            float v = MathF.Max(car.State.Speed, 1f);
            float dt = 1f / 60f;
            float wasted = dt * (1f - v / plan); // time lost this frame vs. driving at plan speed
            var tel = car.State.Telemetry;
            float k = MathF.Abs(d.LastTelemetry.FeedforwardCurvature);
            string phase = d.LastTelemetry.CommandAccel < -2f ? "brake" : k > 0.004f ? (tel.ActualLongitudinalAccel > 1f ? "drive" : "corner") : "straight";
            lost[phase] += wasted;
            bySector[Math.Clamp((int)(s / track.LengthMeters * 24), 0, 23)] += wasted;
        });
        _output.WriteLine(r.ToString());
        foreach (var kv in lost) _output.WriteLine($"{kv.Key,-9} {kv.Value:+0.00;-0.00}s");
        for (int i = 0; i < 24; i++) _output.WriteLine($"sector {i * track.LengthMeters / 24:0000}-{(i + 1) * track.LengthMeters / 24:0000}: {bySector[i]:+0.00;-0.00}s");
    }
}

public sealed class LaneRobustness
{
    private readonly ITestOutputHelper _output;
    public LaneRobustness(ITestOutputHelper output) => _output = output;

    private static LaneTrackFile Learned(TrackData track) =>
        LaneTrackFile.Load(Path.Combine(Path.GetTempPath(), "stintegy-lanes", "silverstone.lanes.csv"), track);

    [Fact]
    public void StartSpeedsAndALongRun()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = Learned(track);
        foreach (float v in new[] { 30f, 40f, 50f })
            _output.WriteLine($"start {v:F0}: {LaneLapRunner.Run(track, file, startSpeed: v)}");
        LaneLapResult run = LaneLapRunner.Run(track, file, flyingLaps: 6);
        _output.WriteLine($"six laps: {string.Join(' ', run.LapTimes.Select(t => t.ToString("F2")))}  spins {run.Spins}{(run.SpinS >= 0 ? $"@{run.SpinS:F0}" : "")} off {run.OffTrackSeconds:F2}");
    }

    [Theory]
    [InlineData(1f, 1f)]
    [InlineData(1f, -1f)]
    [InlineData(1f, 0f)]
    public void WholeLapOnASideLane(float range, float side)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = Learned(track);
        LaneLapResult r = LaneLapRunner.Run(track, file, onStep: (car, d, t) =>
        {
            d.TargetRange = range;
            d.TargetSide = side;
        });
        _output.WriteLine($"range {range} side {side}: {r}");
    }

    [Fact]
    public void SwitchingSidesEveryEightSeconds()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = Learned(track);
        LaneLapResult r = LaneLapRunner.Run(track, file, onStep: (car, d, t) =>
        {
            int phase = (int)(t / 8f) % 4;
            d.TargetRange = phase == 0 ? 0f : 1f;
            d.TargetSide = phase == 1 ? 1f : phase == 2 ? 0f : -1f;
        });
        _output.WriteLine($"switching: {r}");
    }
}

public sealed class LaneSideLearning
{
    private readonly ITestOutputHelper _output;
    public LaneSideLearning(ITestOutputHelper output) => _output = output;

    /// <summary>Starts from the learned free lane, resets the side lanes to 1, learns each, saves.</summary>
    [Fact]
    public void LearnSideLanes()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        string path = Path.Combine(Path.GetTempPath(), "stintegy-lanes", "silverstone.lanes.csv");
        LaneTrackFile file = LaneTrackFile.Load(path, track);
        foreach (LaneId side in new[] { LaneId.Left, LaneId.Right })
        {
            Array.Fill(file.Factors(side), 1f);
            Array.Fill(file.BrakeFactors(side), 1f);
        }
        foreach (LaneId side in new[] { LaneId.Left, LaneId.Right })
        {
            var (best, bestLap, history) = new LaneLearner { Reward = 0.02f }.Learn(track, file, 80, lane: side);
            _output.WriteLine($"{side}: first {history[0].Result} | best {bestLap}");
            file = best;
        }
        file.Save(Path.Combine(Path.GetTempPath(), "stintegy-lanes", "silverstone.lanes.csv"));
    }
}

public sealed class LaneOtherCircuits
{
    private readonly ITestOutputHelper _output;
    public LaneOtherCircuits(ITestOutputHelper output) => _output = output;

    public static IEnumerable<object[]> Circuits() =>
        new[] { "simple", "monaco", "baku", "spa", "monza", "interlagos", "singapore", "portimao", "zandvoort", "shanghai", "sepang" }
            .Select(n => new object[] { n });

    internal static TrackData Build(string name) => name switch
    {
        "simple" => TrackFactory.SimpleTestTrack(),
        "monaco" => TrackFactory.MonacoStyleTestTrack(),
        "baku" => TrackFactory.BakuStyleTestTrack(),
        "spa" => TrackFactory.SpaStyleTestTrack(),
        "monza" => TrackFactory.MonzaStyleTestTrack(),
        "interlagos" => TrackFactory.InterlagosStyleTestTrack(),
        "singapore" => TrackFactory.SingaporeStyleTestTrack(),
        "portimao" => TrackFactory.PortimaoStyleTestTrack(),
        "zandvoort" => TrackFactory.ZandvoortStyleTestTrack(),
        "shanghai" => TrackFactory.ShanghaiStyleTestTrack(),
        "sepang" => TrackFactory.SepangStyleTestTrack(),
        "daytona" => TrackFactory.DaytonaStyleTestTrack(),
        _ => throw new ArgumentOutOfRangeException(nameof(name))
    };

    [Theory]
    [MemberData(nameof(Circuits))]
    public void UnlearnedLap(string name)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = Build(name);
        LaneLapResult r = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), startSpeed: 30f);
        _output.WriteLine($"{name,-11} {track.LengthMeters,6:F0} m  road  {r}");
        LaneLapResult flat = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), new LaneDriverGains { UseRoadAttitude = false }, startSpeed: 30f);
        _output.WriteLine($"{name,-11} {track.LengthMeters,6:F0} m  flat  {flat}");
    }
}

public sealed class LaneLearnOthers
{
    private readonly ITestOutputHelper _output;
    public LaneLearnOthers(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData("portimao")]
    [InlineData("sepang")]
    [InlineData("shanghai")]
    [InlineData("singapore")]
    public void LearnCircuit(string name)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = LaneOtherCircuits.Build(name);
        var (best, bestLap, history) = new LaneLearner { Reward = 0.02f }.Learn(track, LaneTrackFile.Fresh(), 60);
        int firstClean = history.FindIndex(h => h.Result.Finished);
        _output.WriteLine($"{name,-10} first clean at {firstClean}  best {bestLap}");
        string dir = LaneProbes.Dir;
        if (bestLap.Finished)
            best.Save(Path.Combine(dir, $"{name}.lanes.csv"));
    }
}

public sealed class LaneLapGaps
{
    private readonly ITestOutputHelper _output;
    public LaneLapGaps(ITestOutputHelper output) => _output = output;

    [Fact]
    public void WhatTheLapDoesNotCoverYet()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        _output.WriteLine($"standing start: {LaneLapRunner.Run(track, file, startSpeed: 0f)}");
        _output.WriteLine($"cold tyres 25C: {LaneLapRunner.Run(track, file, tireConfig: new StintegyEVO.Core.Cars.TireConfig())}");
        foreach (var tyre in Enum.GetValues<StintegyEVO.Core.Cars.TireUsageMode>())
            _output.WriteLine($"tyre {tyre,-7}: {LaneLapRunner.Run(track, file, strategy: new StintegyEVO.Core.Cars.CarStrategy(tyre, StintegyEVO.Core.Cars.PowerOutputMode.Normal))}");
        foreach (var power in Enum.GetValues<StintegyEVO.Core.Cars.PowerOutputMode>())
            _output.WriteLine($"power {power,-6}: {LaneLapRunner.Run(track, file, strategy: new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Normal, power))}");
    }
}

public sealed class LaneColdTrace
{
    private readonly ITestOutputHelper _output;
    public LaneColdTrace(ITestOutputHelper output) => _output = output;

    [Fact]
    public void ColdTyresIntoTheFirstCorner()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        int n = 0;
        LaneLapResult r = LaneLapRunner.Run(track, file, tireConfig: new StintegyEVO.Core.Cars.TireConfig(), onStep: (car, d, t) =>
        {
            if (n++ % 300 != 0) return;
            var s = car.State;
            var tel = car.State.Telemetry;
            _output.WriteLine(
                $"t={t:F1} s={car.Progress.CurrentS:F0} v={s.Speed:F1} tgt={d.LastTelemetry.TargetSpeed:F1} " +
                $"surf F={s.FrontLeft.SurfaceTempC:F0}/{s.FrontRight.SurfaceTempC:F0} R={s.RearLeft.SurfaceTempC:F0}/{s.RearRight.SurfaceTempC:F0} " +
                $"grip F={tel.FrontGripAccel:F1} R={tel.RearGripAccel:F1} ay={tel.ActualLateralAccel:F1}/{tel.RequestedLateralAccel:F1} " +
                $"use f={tel.FrontLateralUse:F2}/{tel.FrontLongitudinalUse:F2} r={tel.RearLateralUse:F2}/{tel.RearLongitudinalUse:F2} " +
                $"slip={s.SideslipAngleRadians:+0.000;-0.000} e={d.LastTelemetry.LateralErrorMeters:+0.00;-0.00} a={d.LastTelemetry.CommandAccel:F1}");
        });
        _output.WriteLine(r.ToString());
        foreach (float temp in new[] { 25f, 60f, 90f })
        {
            int m = 0;
            LaneLapRunner.Run(track, file, tireConfig: new StintegyEVO.Core.Cars.TireConfig { StartingSurfaceTempC = temp, StartingCoreTempC = temp }, onStep: (car, d, t) =>
            {
                if (m++ == 1)
                    _output.WriteLine($"tyres {temp}C: plan lap {d.PlannedLapSeconds(LaneId.Free):F1}s target here {d.LastTelemetry.TargetSpeed:F1} frontGrip {car.State.Telemetry.FrontGripAccel:F1} surf {car.State.FrontLeft.SurfaceTempC:F0}");
            });
        }
    }
}

public sealed class LaneFreshVsLearned
{
    private readonly ITestOutputHelper _output;
    public LaneFreshVsLearned(ITestOutputHelper output) => _output = output;

    [Fact]
    public void ColdAndAttackWithAndWithoutLearning()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var attack = new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Attack, StintegyEVO.Core.Cars.PowerOutputMode.Normal);
        foreach ((string name, LaneTrackFile file) in new[] { ("unlearned", LaneTrackFile.Fresh()), ("learned", LaneTrackFile.ForCircuit("silverstone", track)) })
        {
            _output.WriteLine($"{name,-9} hot normal : {LaneLapRunner.Run(track, file)}");
            _output.WriteLine($"{name,-9} hot attack : {LaneLapRunner.Run(track, file, strategy: attack)}");
            _output.WriteLine($"{name,-9} cold normal: {LaneLapRunner.Run(track, file, tireConfig: new StintegyEVO.Core.Cars.TireConfig())}");
            _output.WriteLine($"{name,-9} cold stand : {LaneLapRunner.Run(track, file, startSpeed: 0f, tireConfig: new StintegyEVO.Core.Cars.TireConfig())}");
        }
    }
}

public sealed class LaneStandingTrace
{
    private readonly ITestOutputHelper _output;
    public LaneStandingTrace(ITestOutputHelper output) => _output = output;

    [Fact]
    public void StandingStartTrace()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        int n = 0;
        LaneLapResult r = LaneLapRunner.Run(track, LaneTrackFile.ForCircuit("silverstone", track), startSpeed: 0f, onStep: (car, d, t) =>
        {
            if (n++ % 15 != 0 || t > 8f) return;
            var tel = car.State.Telemetry;
            _output.WriteLine($"t={t:F2} s={car.Progress.CurrentS:F1} d={car.Progress.CurrentD:F2} v={car.State.Speed:F2} hdg={car.State.Heading:F3} tgt={d.LastTelemetry.TargetSpeed:F1} a={d.LastTelemetry.CommandAccel:F2} k={d.LastTelemetry.CommandCurvature:F4} e={d.LastTelemetry.LateralErrorMeters:F2} he={d.LastTelemetry.HeadingErrorRadians:F3} gripR={tel.RearGripAccel:F1} latR={tel.RearLateralUse:F2} lap={car.Progress.Lap} dist={car.Progress.TotalDistance:F1}");
        });
        _output.WriteLine(r.ToString());
    }
}

public sealed class LaneSpinTrace
{
    private readonly ITestOutputHelper _output;
    public LaneSpinTrace(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData("cold", 120f, 300f)]
    [InlineData("attack", 820f, 960f)]
    public void TraceSpin(string condition, float from, float to)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var file = LaneTrackFile.Fresh();
        int n = 0;
        var tires = condition == "cold" ? new StintegyEVO.Core.Cars.TireConfig() : null;
        var strategy = condition == "attack"
            ? new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Attack, StintegyEVO.Core.Cars.PowerOutputMode.Normal)
            : (StintegyEVO.Core.Cars.CarStrategy?)null;
        LaneLapResult r = LaneLapRunner.Run(track, file, tireConfig: tires, strategy: strategy, onStep: (car, d, t) =>
        {
            float s = car.Progress.CurrentS;
            if (s < from || s > to || n++ % 6 != 0) return;
            var tel = car.State.Telemetry;
            var dt = d.LastTelemetry;
            _output.WriteLine(
                $"s={s:F0} v={car.State.Speed:F1} tgt={dt.TargetSpeed:F1} a={dt.CommandAccel:+0.0;-0.0}/{tel.ActualLongitudinalAccel:+0.0;-0.0} " +
                $"kff={dt.FeedforwardCurvature:+0.0000;-0.0000} k={dt.CommandCurvature:+0.0000;-0.0000} kact={tel.ActualCurvature:+0.0000;-0.0000} " +
                $"e={dt.LateralErrorMeters:+0.00;-0.00} slip={car.State.SideslipAngleRadians:+0.000;-0.000} steer={car.State.SteerAngleRadians:+0.000;-0.000} " +
                $"grip F{tel.FrontGripAccel:F1} R{tel.RearGripAccel:F1} use f{tel.FrontLateralUse:F2}/{tel.FrontLongitudinalUse:F2} r{tel.RearLateralUse:F2}/{tel.RearLongitudinalUse:F2} " +
                $"cut{tel.CombinedGripLimiterCutAccel:F1} T F{car.State.FrontLeft.SurfaceTempC:F0} R{car.State.RearLeft.SurfaceTempC:F0}");
        });
        _output.WriteLine(r.ToString());
    }
}

public sealed class LaneRaiseCap
{
    private readonly ITestOutputHelper _output;
    public LaneRaiseCap(ITestOutputHelper output) => _output = output;

    [Fact]
    public void ContinueWithAHigherCap()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile start = LaneTrackFile.Load(Path.Combine(LaneProbes.Dir, "silverstone-continued-0.90.lanes.csv"), track);
        var (best, bestLap, history) = new LaneLearner { Reward = 0.02f, MaximumFactor = 1.4f }.Learn(track, start, 200);
        _output.WriteLine($"cap 1.4: start {history[0].Result.FlyingLapSeconds:F2} best {bestLap}");
        best.Save(Path.Combine(LaneProbes.Dir, "silverstone-cap1.4.lanes.csv"));
    }
}

public sealed class LaneContinueLearning
{
    private readonly ITestOutputHelper _output;
    public LaneContinueLearning(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData(0.90f, 150)]
    [InlineData(0.92f, 150)]
    public void ContinueSilverstone(float tractionUse, int iterations)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var gains = new LaneDriverGains { TractionUse = tractionUse };
        LaneTrackFile start = LaneTrackFile.Load(Path.Combine(LaneProbes.Dir, "silverstone.lanes.csv"), track);
        var (best, bestLap, _) = new LaneLearner { Reward = 0.02f }.Learn(track, start, iterations, gains);
        var attack = LaneLapRunner.Run(track, best, gains, strategy: new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Attack, StintegyEVO.Core.Cars.PowerOutputMode.Normal));
        var cold = LaneLapRunner.Run(track, best, gains, startSpeed: 0f, tireConfig: new StintegyEVO.Core.Cars.TireConfig());
        _output.WriteLine($"traction {tractionUse:F2}: best {bestLap} | attack {attack} | cold standing {cold}");
        best.Save(Path.Combine(LaneProbes.Dir, $"silverstone-continued-{tractionUse:F2}.lanes.csv"));
    }
}

public sealed class LaneGripLevels
{
    private readonly ITestOutputHelper _output;
    public LaneGripLevels(ITestOutputHelper output) => _output = output;

    [Fact]
    public void GripReadings()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var file = LaneTrackFile.Fresh();
        void Show(string name, StintegyEVO.Core.Cars.TireConfig? tires = null, float road = 1f, float wear = 0f)
        {
            var seen = new List<string>();
            int n = 0;
            LaneLapRunner.Run(track, file, tireConfig: tires, roadGrip: road, startingWear: wear, onStep: (car, d, t) =>
            {
                if (n++ % 600 == 1 && t < 70f)
                    seen.Add($"{t:F0}s:{d.Grip:F2}");
            });
            _output.WriteLine($"{name,-12} {string.Join(' ', seen)}");
        }
        Show("hot");
        Show("road 0.85", road: 0.85f);
        Show("road 0.70", road: 0.70f);
        Show("cold", new StintegyEVO.Core.Cars.TireConfig());
        Show("wear 0.6", wear: 0.6f);
    }
}

public sealed class LaneLearnAllLevels
{
    private readonly ITestOutputHelper _output;
    public LaneLearnAllLevels(ITestOutputHelper output) => _output = output;

    [Fact]
    public void LearnEveryGripLevelThenVerify()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var watch = System.Diagnostics.Stopwatch.StartNew();
        LaneTrackFile file = LaneTrackFile.Fresh();
        for (int level = LaneTrackFile.TopLevel; level >= 0; level--)
        {
            var (best, bestLap, history) = new LaneLearner { Reward = 0.02f }.Learn(track, file, 150, level: level);
            _output.WriteLine($"grip {LaneTrackFile.GripLevels[level]:F2}: first {history[0].Result.FlyingLapSeconds:F2} best {bestLap}  [{watch.Elapsed.TotalSeconds:F0}s]");
            file = best;
        }
        foreach (LaneId side in new[] { LaneId.Left, LaneId.Right })
        {
            var (best, bestLap, _) = new LaneLearner { Reward = 0.02f }.Learn(track, file, 80, lane: side);
            _output.WriteLine($"{side} lane: best {bestLap}");
            file = best;
        }
        file.Save(Path.Combine(LaneProbes.Dir, "silverstone-levels.lanes.csv"));

        var attack = new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Attack, StintegyEVO.Core.Cars.PowerOutputMode.Normal);
        var cases = new (string Name, Func<LaneLapResult> Run)[]
        {
            ("hot normal", () => LaneLapRunner.Run(track, file)),
            ("hot attack", () => LaneLapRunner.Run(track, file, strategy: attack)),
            ("standing", () => LaneLapRunner.Run(track, file, startSpeed: 0f)),
            ("cold rolling", () => LaneLapRunner.Run(track, file, tireConfig: new StintegyEVO.Core.Cars.TireConfig())),
            ("cold standing", () => LaneLapRunner.Run(track, file, startSpeed: 0f, tireConfig: new StintegyEVO.Core.Cars.TireConfig())),
            ("wear 0.4", () => LaneLapRunner.Run(track, file, startingWear: 0.4f)),
            ("wear 0.8", () => LaneLapRunner.Run(track, file, startingWear: 0.8f)),
            ("road 0.92", () => LaneLapRunner.Run(track, file, roadGrip: 0.92f)),
            ("road 0.78", () => LaneLapRunner.Run(track, file, roadGrip: 0.78f)),
            ("road 0.62", () => LaneLapRunner.Run(track, file, roadGrip: 0.62f)),
            ("left lane", () => LaneLapRunner.Run(track, file, onStep: (c, d, t) => { d.TargetRange = 1f; d.TargetSide = 1f; })),
            ("right lane", () => LaneLapRunner.Run(track, file, onStep: (c, d, t) => { d.TargetRange = 1f; d.TargetSide = -1f; })),
            ("switching", () => LaneLapRunner.Run(track, file, onStep: (c, d, t) =>
            {
                int phase = (int)(t / 8f) % 4;
                d.TargetRange = phase == 0 ? 0f : 1f;
                d.TargetSide = phase == 1 ? 1f : phase == 2 ? 0f : -1f;
            })),
        };
        foreach (var c in cases)
            _output.WriteLine($"{c.Name,-13} {c.Run()}");
        var six = LaneLapRunner.Run(track, file, flyingLaps: 6);
        _output.WriteLine($"six laps      {string.Join(' ', six.LapTimes.Select(x => x.ToString("F2")))} spins {six.Spins} off {six.OffTrackSeconds:F2}");
    }
}

public sealed class LaneQuestions
{
    private readonly ITestOutputHelper _output;
    public LaneQuestions(ITestOutputHelper output) => _output = output;

    [Fact]
    public void GradeBankSlideAndError()
    {
        if (!LaneProbes.Enabled) return;
        foreach (string name in new[] { "silverstone", "spa", "monaco", "interlagos", "baku", "shanghai" })
        {
            TrackData t = name == "silverstone" ? TrackFactory.SilverstoneStyleTestTrack() : LaneOtherCircuits.Build(name);
            float maxGrade = 0, maxBank = 0, maxVert = 0;
            for (float s = 0; s < t.LengthMeters; s += 2f)
            {
                var x = t.Sample(s);
                maxGrade = MathF.Max(maxGrade, MathF.Abs(x.Grade));
                maxBank = MathF.Max(maxBank, MathF.Abs(x.BankSlope));
                maxVert = MathF.Max(maxVert, MathF.Abs(x.VerticalRate));
            }
            _output.WriteLine($"{name,-11} grade max {maxGrade * 100:F1}%  bank max {MathF.Atan(maxBank) * 57.3f:F1} deg  crest/dip rate {maxVert:F4} 1/m");
        }

        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var errors = new List<float>();
        float maxSlip = 0, slipOver5 = 0, slipOver8 = 0;
        int lap0 = -1;
        LaneLapRunner.Run(track, LaneTrackFile.ForCircuit("silverstone", track), onStep: (car, d, time) =>
        {
            if (lap0 < 0) lap0 = car.Progress.Lap;
            if (car.Progress.Lap == lap0) return;
            errors.Add(MathF.Abs(d.LastTelemetry.LateralErrorMeters));
            float slip = MathF.Abs(car.State.SideslipAngleRadians);
            maxSlip = MathF.Max(maxSlip, slip);
            if (slip > 0.05f) slipOver5 += 1f / 60f;
            if (slip > 0.08f) slipOver8 += 1f / 60f;
        });
        errors.Sort();
        float P(float q) => errors[(int)(q * (errors.Count - 1))];
        _output.WriteLine($"lateral error  p50 {P(0.5f):F2}  p90 {P(0.9f):F2}  p99 {P(0.99f):F2}  max {errors[^1]:F2} m;  time over 0.5 m {errors.Count(e => e > 0.5f) / 60f:F1}s, over 1 m {errors.Count(e => e > 1f) / 60f:F1}s");
        _output.WriteLine($"sideslip max {maxSlip * 57.3f:F1} deg; time over 2.9 deg {slipOver5:F1}s, over 4.6 deg {slipOver8:F1}s");
    }
}

public sealed class LaneBaked3d
{
    private readonly ITestOutputHelper _output;
    public LaneBaked3d(ITestOutputHelper output) => _output = output;

    internal static string Baked(string id) =>
        Environment.GetEnvironmentVariable("STINTEGY_LANES3D") is { } dir ? Path.Combine(dir, $"{id}.lanes3d.csv") : "";

    [Fact]
    public void CompareBakedFreeLaneWithK1999()
    {
        if (!LaneProbes.Enabled || !File.Exists(Baked("silverstone"))) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneSet k1999 = LaneSet.Bake(track);
        LaneSet baked = LaneSet.ImportBaked(Baked("silverstone"), track, k1999);
        LaneEnvelope env = LanePlanProbe.HotEnvelope();
        float diff = 0f, worst = 0f;
        for (int i = 0; i < k1999.Count; i++)
        {
            float x = MathF.Abs(baked.Offset(LaneId.Free, i) - k1999.Offset(LaneId.Free, i));
            diff += x; worst = MathF.Max(worst, x);
            if (x > 3f && i % 5 == 0)
                _output.WriteLine($"  station {i} s={i * k1999.Spacing:F0} hw={k1999.HalfWidth(i):F2} k1999={k1999.Offset(LaneId.Free, i):F2} tum={baked.Offset(LaneId.Free, i):F2}");
        }
        _output.WriteLine($"free lane: mean |baked - k1999| {diff / k1999.Count:F2} m, worst {worst:F2} m");
        foreach ((string name, LaneSet set) in new[] { ("k1999", k1999), ("tum3d", baked) })
        {
            var full = LanePlanProbe.Plan(set, env, new LaneShares { Lateral = 1f, Brake = 1f }, 3);
            var margin = LanePlanProbe.Plan(set, env, new LaneShares(), 3);
            var lap = LaneLapRunner.Run(track, LaneTrackFile.For(set));
            _output.WriteLine($"{name,-6} paper full {full.Lap:F1}s margin {margin.Lap:F1}s peak k {full.PeakK:F4} | unlearned lap {lap}");
        }
    }
}

public sealed class LaneRoadTerms
{
    private readonly ITestOutputHelper _output;
    public LaneRoadTerms(ITestOutputHelper output) => _output = output;

    [Theory]
    [InlineData("spa")]
    [InlineData("simple")]
    [InlineData("portimao")]
    [InlineData("zandvoort")]
    [InlineData("monaco")]
    public void WhichRoadTerm(string name)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = LaneOtherCircuits.Build(name);
        foreach ((string label, LaneDriverGains g) in new[]
        {
            ("grade only", new LaneDriverGains { UseBank = false, UseVertical = false }),
            ("bank only", new LaneDriverGains { UseGrade = false, UseVertical = false }),
            ("crest only", new LaneDriverGains { UseGrade = false, UseBank = false }),
            ("no crest", new LaneDriverGains { UseVertical = false }),
        })
        {
            LaneLapResult r = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), g, startSpeed: 30f);
            _output.WriteLine($"{name,-10} {label,-10} {r}");
        }
    }
}

public sealed class LaneBankTrace
{
    private readonly ITestOutputHelper _output;
    public LaneBankTrace(ITestOutputHelper output) => _output = output;

    [Fact]
    public void MonacoBankOnly()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = LaneOtherCircuits.Build("monaco");
        foreach ((string label, LaneDriverGains g) in new[] {
            ("flat", new LaneDriverGains { UseRoadAttitude = false }),
            ("bank", new LaneDriverGains { UseGrade = false, UseVertical = false }) })
        {
            int n = 0;
            LaneLapRunner.Run(track, LaneTrackFile.Fresh(), g, startSpeed: 30f, onStep: (car, d, t) =>
            {
                float s = car.Progress.CurrentS;
                if (s < 700 || s > 810 || n++ % 6 != 0 || t > 60f) return;
                var tel = car.State.Telemetry;
                var x = track.Sample(s);
                _output.WriteLine($"{label} s={s:F0} v={car.State.Speed:F1} tgt={d.LastTelemetry.TargetSpeed:F1} a={d.LastTelemetry.CommandAccel:+0.0;-0.0} k={d.LastTelemetry.CommandCurvature:+0.0000;-0.0000} kff={d.LastTelemetry.FeedforwardCurvature:+0.0000;-0.0000} e={d.LastTelemetry.LateralErrorMeters:+0.00;-0.00} slip={car.State.SideslipAngleRadians:+0.000;-0.000} bank={x.BankSlopeAt(car.Progress.CurrentD):+0.000;-0.000} grade={x.Grade:+0.000;-0.000} use f{tel.FrontLateralUse:F2}/{tel.FrontLongitudinalUse:F2} r{tel.RearLateralUse:F2}/{tel.RearLongitudinalUse:F2}");
            });
        }
    }
}

public sealed class LaneBankVariants
{
    private readonly ITestOutputHelper _output;
    public LaneBankVariants(ITestOutputHelper output) => _output = output;

    [Fact]
    public void BankVariants()
    {
        if (!LaneProbes.Enabled) return;
        foreach (string name in new[] { "monaco", "spa", "portimao", "simple", "zandvoort" })
        {
            TrackData track = LaneOtherCircuits.Build(name);
            var bankOnly = new LaneDriverGains { UseGrade = false, UseVertical = false };
            LaneRoad.BankAddsLoad = true;
            var noCmd = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), bankOnly with { BankInCommand = false }, startSpeed: 30f);
            LaneRoad.BankAddsLoad = false;
            var noLoad = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), bankOnly, startSpeed: 30f);
            var noLoadNoCmd = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), bankOnly with { BankInCommand = false }, startSpeed: 30f);
            LaneRoad.BankAddsLoad = true;
            _output.WriteLine($"{name,-9} cmd flat : {noCmd}");
            _output.WriteLine($"{name,-9} no load  : {noLoad}");
            _output.WriteLine($"{name,-9} both     : {noLoadNoCmd}");
        }
    }
}

public sealed class LaneBankProbe
{
    private readonly ITestOutputHelper _output;
    public LaneBankProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void BankAtTheEdges()
    {
        if (!LaneProbes.Enabled) return;
        foreach (string name in new[] { "simple", "monaco", "spa", "portimao" })
        {
            TrackData t = LaneOtherCircuits.Build(name);
            float centre = 0, edge = 0, curv = 0;
            float atS = 0;
            for (float s = 0; s < t.LengthMeters; s += 2f)
            {
                var x = t.Sample(s);
                centre = MathF.Max(centre, MathF.Abs(x.BankSlope));
                curv = MathF.Max(curv, MathF.Abs(x.BankCurvature));
                float e = MathF.Max(MathF.Abs(x.BankSlopeAt(x.HalfWidth)), MathF.Abs(x.BankSlopeAt(-x.HalfWidth)));
                if (e > edge) { edge = e; atS = s; }
            }
            var w = t.Sample(atS);
            _output.WriteLine($"{name,-9} centre {MathF.Atan(centre) * 57.3f:F1} deg  edge {MathF.Atan(edge) * 57.3f:F1} deg @s={atS:F0} (hw {w.HalfWidth:F1}, slope {w.BankSlope:F3}, curv {w.BankCurvature:F4}, k {w.Curvature:F4})  max bankCurvature {curv:F4}");
        }
    }
}

public sealed class LaneSteerLearning
{
    private readonly ITestOutputHelper _output;
    public LaneSteerLearning(ITestOutputHelper output) => _output = output;

    [Fact]
    public void LearnSteeringOnTheLevelsFile()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.Load(Path.Combine(LaneProbes.Dir, "silverstone-levels.lanes.csv"), track);
        var (best, bestLap, history) = new LaneLearner().LearnSteering(track, file, 60);
        foreach (var h in history.Where(h => h.Kept))
            _output.WriteLine($"{h.Iteration,3} {h.Result}");
        _output.WriteLine($"best {bestLap}");
        best.Save(Path.Combine(LaneProbes.Dir, "silverstone-steer.lanes.csv"));
    }
}

public sealed class LaneLearnTumFree
{
    private readonly ITestOutputHelper _output;
    public LaneLearnTumFree(ITestOutputHelper output) => _output = output;

    [Fact]
    public void LearnLevelsOnTheTumFreeLane()
    {
        if (!LaneProbes.Enabled || !File.Exists(LaneBaked3d.Baked("silverstone"))) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.FromBakedLanes(LaneBaked3d.Baked("silverstone"), track);
        for (int level = LaneTrackFile.TopLevel; level >= 0; level--)
        {
            var (best, bestLap, history) = new LaneLearner { Reward = 0.02f }.Learn(track, file, 150, level: level);
            _output.WriteLine($"tum grip {LaneTrackFile.GripLevels[level]:F2}: first {history[0].Result.FlyingLapSeconds:F2} best {bestLap}");
            file = best;
        }
        file.Save(Path.Combine(LaneProbes.Dir, "silverstone-tumfree.lanes.csv"));
        var attack = new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Attack, StintegyEVO.Core.Cars.PowerOutputMode.Normal);
        _output.WriteLine($"hot      {LaneLapRunner.Run(track, file)}");
        _output.WriteLine($"attack   {LaneLapRunner.Run(track, file, strategy: attack)}");
        _output.WriteLine($"cold     {LaneLapRunner.Run(track, file, startSpeed: 0f, tireConfig: new StintegyEVO.Core.Cars.TireConfig())}");
        _output.WriteLine($"wear 0.8 {LaneLapRunner.Run(track, file, startingWear: 0.8f)}");
        _output.WriteLine($"road .78 {LaneLapRunner.Run(track, file, roadGrip: 0.78f)}");
    }
}

public sealed class LaneSimpleBankTrace
{
    private readonly ITestOutputHelper _output;
    public LaneSimpleBankTrace(ITestOutputHelper output) => _output = output;

    [Fact]
    public void SimpleBankedCorner()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = LaneOtherCircuits.Build("simple");
        int n = 0;
        var r = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), new LaneDriverGains { UseBank = true }, startSpeed: 30f, onStep: (car, d, t) =>
        {
            float s = car.Progress.CurrentS;
            if (s < 1440 || s > 1620 || n++ % 6 != 0 || t > 45f) return;
            var tel = car.State.Telemetry;
            var x = track.Sample(s);
            float bank = x.BankSlopeAt(car.Progress.CurrentD);
            _output.WriteLine($"s={s:F0} v={car.State.Speed:F1} tgt={d.LastTelemetry.TargetSpeed:F1} a={d.LastTelemetry.CommandAccel:+0.0;-0.0}/{tel.ActualLongitudinalAccel:+0.0;-0.0} kff={d.LastTelemetry.FeedforwardCurvature:+0.0000;-0.0000} k={d.LastTelemetry.CommandCurvature:+0.0000;-0.0000} kact={tel.ActualCurvature:+0.0000;-0.0000} e={d.LastTelemetry.LateralErrorMeters:+0.00;-0.00} bank={bank:+0.000;-0.000} ay={tel.ActualLateralAccel:F1}/{tel.RequestedLateralAccel:F1} grip F{tel.FrontGripAccel:F1} R{tel.RearGripAccel:F1} use f{tel.FrontLateralUse:F2}/{tel.FrontLongitudinalUse:F2} r{tel.RearLateralUse:F2}/{tel.RearLongitudinalUse:F2} steer={car.State.SteerAngleRadians:+0.000;-0.000}");
        });
        _output.WriteLine(r.ToString());
    }
}

public sealed class LaneAxleCap
{
    private readonly ITestOutputHelper _output;
    public LaneAxleCap(ITestOutputHelper output) => _output = output;

    [Fact]
    public void AxleCapFlatAndBanked()
    {
        if (!LaneProbes.Enabled) return;
        TrackData sil = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile learned = LaneTrackFile.Load(Path.Combine(LaneProbes.Dir, "silverstone-levels.lanes.csv"), sil);
        foreach (float share in new[] { 0.90f, 0.95f, 1.0f })
        {
            var g = new LaneDriverGains { AxleCapShare = share };
            _output.WriteLine($"silverstone learned, axle cap {share:F2}: {LaneLapRunner.Run(sil, learned, g)}");
        }
        _output.WriteLine($"silverstone learned, envelope cap: {LaneLapRunner.Run(sil, learned, new LaneDriverGains { CapFromAxles = false })}");
        foreach (string name in new[] { "monaco", "spa", "portimao", "simple", "zandvoort", "baku", "interlagos" })
        {
            TrackData track = LaneOtherCircuits.Build(name);
            var flat = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), new LaneDriverGains(), startSpeed: 30f);
            var bank = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), new LaneDriverGains { UseBank = true }, startSpeed: 30f);
            _output.WriteLine($"{name,-10} axle cap, no bank : {flat}");
            _output.WriteLine($"{name,-10} axle cap, bank    : {bank}");
        }
    }
}

public sealed class DaytonaRoadCourse
{
    private readonly ITestOutputHelper _output;
    public DaytonaRoadCourse(ITestOutputHelper output) => _output = output;

    internal static string? DataPath()
    {
        string? dir = AppContext.BaseDirectory;
        for (int i = 0; i < 8 && dir is not null; i++, dir = Path.GetDirectoryName(dir))
        {
            string candidate = Path.Combine(dir, "private", "tracks", "daytona_road.csv");
            if (File.Exists(candidate))
                return candidate;
        }
        return null;
    }

    internal static TrackData Build() => StintegyEVO.Drivers.Tracks.TrackPack.Load(DataPath()!, 5_729f);

    [Fact]
    public void BuildAndDriveOnce()
    {
        if (!LaneProbes.Enabled || DataPath() is null) return;
        TrackData track = Build();
        float maxBank = 0, favourable = 0, adverse = 0, maxK = 0;
        for (float s = 0; s < track.LengthMeters; s += 2f)
        {
            var x = track.Sample(s);
            maxBank = MathF.Max(maxBank, MathF.Abs(x.BankSlope));
            maxK = MathF.Max(maxK, MathF.Abs(x.Curvature));
            if (MathF.Abs(x.Curvature) > 0.002f && MathF.Abs(x.BankSlope) > 0.3f)
            {
                if (x.Curvature * x.BankSlope > 0) favourable += 2; else adverse += 2;
            }
        }
        _output.WriteLine($"length {track.LengthMeters:F0} m, max bank {MathF.Atan(maxBank) * 57.3f:F1} deg, tightest R {1f / maxK:F0} m, banked-turn metres into the turn {favourable:F0} / against {adverse:F0}");
        foreach (float s in new[] { 0f, 400f, 1200f, 2000f, 2600f, 3300f, 4200f, 5300f })
        {
            var x = track.Sample(s);
            _output.WriteLine($"  s={s,5:F0} k={x.Curvature:+0.0000;-0.0000} R={1f / MathF.Max(MathF.Abs(x.Curvature), 1e-5f),6:F0} bank={MathF.Atan(x.BankSlope) * 57.3f:+0.0;-0.0} deg width={x.Width:F1}");
        }
        LaneLapResult r = LaneLapRunner.Run(track, LaneTrackFile.Fresh(), startSpeed: 30f);
        _output.WriteLine($"unlearned: {r}");
    }
}

public sealed class DaytonaLearning
{
    private readonly ITestOutputHelper _output;
    public DaytonaLearning(ITestOutputHelper output) => _output = output;

    [Fact]
    public void LearnDaytonaThenVerify()
    {
        if (!LaneProbes.Enabled || DaytonaRoadCourse.DataPath() is null) return;
        TrackData track = DaytonaRoadCourse.Build();
        LaneTrackFile file = LaneTrackFile.Fresh();
        for (int level = LaneTrackFile.TopLevel; level >= 0; level--)
        {
            var (best, bestLap, history) = new LaneLearner { Reward = 0.02f }.Learn(track, file, 150, level: level);
            _output.WriteLine($"daytona grip {LaneTrackFile.GripLevels[level]:F2}: first {history[0].Result.FlyingLapSeconds:F2} best {bestLap}");
            file = best;
        }
        foreach (LaneId side in new[] { LaneId.Left, LaneId.Right })
        {
            var (best, bestLap, _) = new LaneLearner { Reward = 0.02f }.Learn(track, file, 80, lane: side);
            _output.WriteLine($"daytona {side} lane: {bestLap}");
            file = best;
        }
        string dir = Path.GetDirectoryName(DaytonaRoadCourse.DataPath()!)!;
        file.Save(Path.Combine(dir, "daytona_road.lanes.csv"));
        var attack = new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Attack, StintegyEVO.Core.Cars.PowerOutputMode.Normal);
        _output.WriteLine($"hot      {LaneLapRunner.Run(track, file)}");
        _output.WriteLine($"standing {LaneLapRunner.Run(track, file, startSpeed: 0f)}");
        _output.WriteLine($"attack   {LaneLapRunner.Run(track, file, strategy: attack)}");
        _output.WriteLine($"cold     {LaneLapRunner.Run(track, file, startSpeed: 0f, tireConfig: new StintegyEVO.Core.Cars.TireConfig())}");
        _output.WriteLine($"wear 0.8 {LaneLapRunner.Run(track, file, startingWear: 0.8f)}");
        _output.WriteLine($"road .78 {LaneLapRunner.Run(track, file, roadGrip: 0.78f)}");
        _output.WriteLine($"switch   {LaneLapRunner.Run(track, file, onStep: (c, d, t) => { int p = (int)(t / 8f) % 4; d.TargetRange = p == 0 ? 0f : 1f; d.TargetSide = p == 1 ? 1f : p == 2 ? 0f : -1f; })}");
        var six = LaneLapRunner.Run(track, file, flyingLaps: 6);
        _output.WriteLine($"six laps {string.Join(' ', six.LapTimes.Select(x => x.ToString("F2")))} spins {six.Spins} off {six.OffTrackSeconds:F2}");
    }
}

public sealed class LaneThrottleProbe
{
    private readonly ITestOutputHelper _output;
    public LaneThrottleProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void IsItFlatOnTheStraights()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        int lap0 = -1, n = 0;
        float straight = 0, flat = 0, atTarget = 0;
        LaneLapRunner.Run(track, LaneTrackFile.ForCircuit("silverstone", track), onStep: (car, d, t) =>
        {
            if (lap0 < 0) lap0 = car.Progress.Lap;
            if (car.Progress.Lap == lap0) return;
            var tel = d.LastTelemetry;
            float k = MathF.Abs(tel.FeedforwardCurvature);
            if (k > 0.0015f || tel.CommandAccel <= 0f) return;
            float available = car.State.Telemetry.ActualLongitudinalAccel; // placeholder, replaced below
            straight += 1f / 60f;
            float motor = car.CarConfig.Powertrain.DriveAccelerationLimit(car.State.Energy, car.Strategy, car.State.Speed,
                StintegyEVO.Core.Cars.CarPhysics.TotalMassKg(car.CarConfig, car.State.Energy), car.CarConfig.MaxDriveAcceleration);
            if (tel.CommandAccel >= 0.98f * motor) flat += 1f / 60f;
            if (car.State.Speed >= tel.TargetSpeed - 0.3f) atTarget += 1f / 60f;
            if (n++ % 120 == 0)
                _output.WriteLine($"s={car.Progress.CurrentS,5:F0} v={car.State.Speed:F1} tgt={tel.TargetSpeed:F1} cmd={tel.CommandAccel:F2} motor={motor:F2} pedal={tel.CommandAccel / MathF.Max(motor, 0.1f) * 100f:F0}% grip R={car.State.Telemetry.RearGripAccel:F1}");
        });
        _output.WriteLine($"on straights accelerating: {straight:F1}s, flat out {flat:F1}s ({flat / MathF.Max(straight, 1e-3f) * 100f:F0}%), at or over target {atTarget:F1}s");
    }
}

public sealed class LaneAbilityProbe
{
    private readonly ITestOutputHelper _output;
    public LaneAbilityProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void PaceAndConsistency()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        void Run(string label, StintegyEVO.Core.Drivers.DriverAbilities a)
        {
            var r = LaneLapRunner.Run(track, file, abilities: a, flyingLaps: 4);
            var laps = r.LapTimes;
            float mean = laps.Count > 0 ? laps.Average() : 0f;
            float spread = laps.Count > 1 ? MathF.Sqrt(laps.Select(x => (x - mean) * (x - mean)).Sum() / (laps.Count - 1)) : 0f;
            _output.WriteLine($"{label,-22} laps {string.Join(' ', laps.Select(x => x.ToString("F2")))}  mean {mean:F2} sd {spread:F3}  spins {r.Spins} off {r.OffTrackSeconds:F2} err {r.MeanAbsErrorMeters:F2}/{r.WorstErrorMeters:F2}");
        }
        foreach (float pace in new[] { 100f, 50f, 0f })
            Run($"pace {pace:F0}", new StintegyEVO.Core.Drivers.DriverAbilities { Pace = pace });
        foreach (float c in new[] { 100f, 50f, 0f })
            Run($"consistency {c:F0}", new StintegyEVO.Core.Drivers.DriverAbilities { Consistency = c });
        Run("pace 0 consistency 0", new StintegyEVO.Core.Drivers.DriverAbilities { Pace = 0, Consistency = 0 });
    }
}

public sealed class DaytonaLowGrip
{
    private readonly ITestOutputHelper _output;
    public DaytonaLowGrip(ITestOutputHelper output) => _output = output;

    [Fact]
    public void RelearnTheLowestLevel()
    {
        if (!LaneProbes.Enabled || DaytonaRoadCourse.DataPath() is null) return;
        TrackData track = DaytonaRoadCourse.Build();
        string dir = Path.GetDirectoryName(DaytonaRoadCourse.DataPath()!)!;
        LaneTrackFile file = LaneTrackFile.Load(Path.Combine(dir, "daytona_road.lanes.csv"), track);
        file.ResetLevel(0);
        var (best, bestLap, history) = new LaneLearner { Reward = 0.02f }.Learn(track, file, 150, level: 0);
        int firstClean = history.FindIndex(h => h.Result.Finished);
        _output.WriteLine($"daytona grip 0.55: first clean at {firstClean}, best {bestLap}");
        best.Save(Path.Combine(dir, "daytona_road.lanes.csv"));
        _output.WriteLine($"cold     {LaneLapRunner.Run(track, best, startSpeed: 0f, tireConfig: new StintegyEVO.Core.Cars.TireConfig())}");
        _output.WriteLine($"cold rl  {LaneLapRunner.Run(track, best, tireConfig: new StintegyEVO.Core.Cars.TireConfig())}");
        _output.WriteLine($"hot      {LaneLapRunner.Run(track, best)}");
    }
}

public sealed class LaneControlProbe
{
    private readonly ITestOutputHelper _output;
    public LaneControlProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void CarControlAndReactions()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        void Run(string label, StintegyEVO.Core.Drivers.DriverAbilities a, StintegyEVO.Core.Cars.TireConfig? tires = null)
        {
            float maxSlip = 0f; int events = 0; float recovering = 0f;
            var r = LaneLapRunner.Run(track, file, abilities: a, flyingLaps: 4, tireConfig: tires, onStep: (car, d, t) =>
            {
                maxSlip = MathF.Max(maxSlip, MathF.Abs(car.State.SideslipAngleRadians));
                events = d.RecoveryEvents;
                if (d.IsRecovering) recovering += 1f / 60f;
            });
            float mean = r.LapTimes.Count > 0 ? r.LapTimes.Average() : 0f;
            _output.WriteLine($"{label,-24} laps {string.Join(' ', r.LapTimes.Select(x => x.ToString("F2")))} mean {mean:F2} | catches {events} ({recovering:F1}s) max slip {maxSlip * 57.3f:F1} deg | spins {r.Spins}{(r.SpinS >= 0 ? $"@{r.SpinS:F0}" : "")} off {r.OffTrackSeconds:F2}");
        }
        foreach (float c in new[] { 100f, 50f, 0f })
            Run($"car control {c:F0}", new StintegyEVO.Core.Drivers.DriverAbilities { CarControl = c });
        foreach (float c in new[] { 100f, 50f, 0f })
            Run($"reactions {c:F0}", new StintegyEVO.Core.Drivers.DriverAbilities { Reactions = c });
        // Cold tyres make slides happen, which is where reactions show.
        foreach (float c in new[] { 100f, 0f })
            Run($"cold, reactions {c:F0}", new StintegyEVO.Core.Drivers.DriverAbilities { Reactions = c }, new StintegyEVO.Core.Cars.TireConfig());
        foreach (float c in new[] { 100f, 0f })
            Run($"cold, car control {c:F0}", new StintegyEVO.Core.Drivers.DriverAbilities { CarControl = c }, new StintegyEVO.Core.Cars.TireConfig());
    }
}

public sealed class LaneEnergyProbe
{
    private readonly ITestOutputHelper _output;
    public LaneEnergyProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void EnergyPerLapByRung()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        foreach (var power in Enum.GetValues<StintegyEVO.Core.Cars.PowerOutputMode>())
        {
            var crossings = new List<(float T, float Soc, float Wear)>();
            int lastLap = int.MinValue;
            var r = LaneLapRunner.Run(track, file, flyingLaps: 2,
                strategy: new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Normal, power),
                onStep: (car, d, t) =>
                {
                    int lap = car.Progress.Lap;
                    if (lap != lastLap)
                    {
                        if (lastLap != int.MinValue)
                            crossings.Add((t, car.State.Energy.Primary, car.State.RearLeft.Wear));
                        lastLap = lap;
                    }
                });
            if (crossings.Count >= 2)
            {
                float socPerLap = (crossings[0].Soc - crossings[^1].Soc) / (crossings.Count - 1);
                float lapsOnFull = 1f / MathF.Max(socPerLap, 1e-6f);
                _output.WriteLine($"{power,-7} lap {r.FlyingLapSeconds:F2}s  battery {socPerLap * 100f:F2}%/lap ({socPerLap * 1100f:F1} MJ)  laps on a full pack {lapsOnFull:F1}");
            }
        }
    }
}

public sealed class LaneModeProbe
{
    private readonly ITestOutputHelper _output;
    public LaneModeProbe(ITestOutputHelper output) => _output = output;

    private static (LaneLapResult Lap, float SocPerLap, float WearPerLap) Measure(
        TrackData track, LaneTrackFile file, StintegyEVO.Core.Cars.CarStrategy strategy, float coast)
    {
        var crossings = new List<(float Soc, float Wear)>();
        int lastLap = int.MinValue;
        var r = LaneLapRunner.Run(track, file, flyingLaps: 2, strategy: strategy, onStep: (car, d, t) =>
        {
            d.CoastOverride = coast;
            if (car.Progress.Lap != lastLap)
            {
                if (lastLap != int.MinValue)
                {
                    var s = car.State;
                    crossings.Add((s.Energy.Primary, 0.25f * (s.FrontLeft.Wear + s.FrontRight.Wear + s.RearLeft.Wear + s.RearRight.Wear)));
                }
                lastLap = car.Progress.Lap;
            }
        });
        if (crossings.Count < 2) return (r, float.NaN, float.NaN);
        int n = crossings.Count - 1;
        return (r, (crossings[0].Soc - crossings[^1].Soc) / n, (crossings[^1].Wear - crossings[0].Wear) / n);
    }

    [Fact]
    public void CoastingTradeCurve()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        var attack = new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Normal, StintegyEVO.Core.Cars.PowerOutputMode.Attack);
        var save = new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Normal, StintegyEVO.Core.Cars.PowerOutputMode.Save);
        var baseline = Measure(track, file, attack, 0f);
        _output.WriteLine($"attack, no coast : lap {baseline.Lap.FlyingLapSeconds:F2}s battery {baseline.SocPerLap * 100f:F3}%/lap");
        var s = Measure(track, file, save, 0f);
        _output.WriteLine($"save rung        : +{s.Lap.FlyingLapSeconds - baseline.Lap.FlyingLapSeconds:F2}s for -{(baseline.SocPerLap - s.SocPerLap) * 100f:F3}%/lap  -> {(s.Lap.FlyingLapSeconds - baseline.Lap.FlyingLapSeconds) / MathF.Max((baseline.SocPerLap - s.SocPerLap) * 100f, 1e-4f):F2} s per 0.1%... (per 1% of pack)");
        foreach (float coast in new[] { 0.5f, 1f, 1.5f, 2f, 3f, 4f })
        {
            var c = Measure(track, file, attack, coast);
            float dt = c.Lap.FlyingLapSeconds - baseline.Lap.FlyingLapSeconds;
            float de = (baseline.SocPerLap - c.SocPerLap) * 100f;
            _output.WriteLine($"coast {coast:F1}s        : +{dt:F2}s for -{de:F3}%/lap  -> {dt / MathF.Max(de, 1e-4f):F2} s per 1% of pack  (spins {c.Lap.Spins})");
        }
    }

    [Fact]
    public void TyreRungs()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        foreach (var tyre in Enum.GetValues<StintegyEVO.Core.Cars.TireUsageMode>())
        {
            var m = Measure(track, file, new StintegyEVO.Core.Cars.CarStrategy(tyre, StintegyEVO.Core.Cars.PowerOutputMode.Normal), 0f);
            _output.WriteLine($"tyre {tyre,-7}: lap {m.Lap.FlyingLapSeconds:F2}s wear {m.WearPerLap * 100f:F3}%/lap  spins {m.Lap.Spins}{(m.Lap.SpinS >= 0 ? $"@{m.Lap.SpinS:F0}" : "")} off {m.Lap.OffTrackSeconds:F2} worst {m.Lap.WorstErrorMeters:F2}");
        }
    }
}

public sealed class LaneWearSpinTrace
{
    private readonly ITestOutputHelper _output;
    public LaneWearSpinTrace(ITestOutputHelper output) => _output = output;

    [Fact]
    public void WhatTheCarIsLikeWhenItSpinsOnWornTyres()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        var history = new Queue<(float Slip, string Line)>();
        int lastLap = -1;
        var r = LaneLapRunner.Run(track, file, startSpeed: 30f, flyingLaps: 30, onStep: (car, d, t) =>
        {
            var s = car.State; var tel = s.Telemetry;
            if (car.Progress.Lap != lastLap)
            {
                lastLap = car.Progress.Lap;
                _output.WriteLine($"lap {lastLap,2} t={t,6:F0} wear F {s.FrontLeft.Wear:F2}/{s.FrontRight.Wear:F2} R {s.RearLeft.Wear:F2}/{s.RearRight.Wear:F2} grip-level {d.Grip:F2} axle grip @speed F {tel.FrontGripAccel:F1} R {tel.RearGripAccel:F1} temps F {s.FrontLeft.SurfaceTempC:F0} R {s.RearLeft.SurfaceTempC:F0}");
            }
            float cs = car.Progress.CurrentS;
            if ((car.Progress.Lap == 2 || car.Progress.Lap == 21 || car.Progress.Lap == 23) && cs > 3450f && cs < 3790f && (int)(t * 50) % 15 == 0)
                _output.WriteLine($"   L{car.Progress.Lap} s={cs:F0} v={s.Speed:F1} region={car.Progress.Region} load FL{s.FrontLeft.LoadN:F0} FR{s.FrontRight.LoadN:F0} RL{s.RearLeft.LoadN:F0} RR{s.RearRight.LoadN:F0} sg {s.FrontLeft.SurfaceGrip:F2}/{s.FrontRight.SurfaceGrip:F2}/{s.RearLeft.SurfaceGrip:F2}/{s.RearRight.SurfaceGrip:F2} temp {s.FrontLeft.SurfaceTempC:F0}/{s.FrontRight.SurfaceTempC:F0}/{s.RearLeft.SurfaceTempC:F0}/{s.RearRight.SurfaceTempC:F0} grip F{tel.FrontGripAccel:F1} R{tel.RearGripAccel:F1} err={d.LastTelemetry.LateralErrorMeters:+0.00;-0.00} k want {d.LastTelemetry.WantedCurvature:+0.0000;-0.0000} ff {d.LastTelemetry.FeedforwardCurvature:+0.0000;-0.0000} cap {d.LastTelemetry.CurvatureCap:+0.0000;-0.0000} got {d.LastTelemetry.CommandCurvature:+0.0000;-0.0000} a={d.LastTelemetry.CommandAccel:+0.0;-0.0} tgt={d.LastTelemetry.TargetSpeed:F1} rearUse {tel.RearLateralUse:F2} frontUse {tel.FrontLateralUse:F2}");
            history.Enqueue((s.SideslipAngleRadians * 57.3f, $"  s={car.Progress.CurrentS:F0} v={s.Speed:F1} tgt={d.LastTelemetry.TargetSpeed:F1} a={d.LastTelemetry.CommandAccel:+0.0;-0.0} k={d.LastTelemetry.CommandCurvature:+0.0000;-0.0000} slip={s.SideslipAngleRadians * 57.3f:+0.0;-0.0}deg grip F{tel.FrontGripAccel:F1} R{tel.RearGripAccel:F1} use f{tel.FrontLateralUse:F2}/{tel.FrontLongitudinalUse:F2} r{tel.RearLateralUse:F2}/{tel.RearLongitudinalUse:F2} recovering={d.IsRecovering}"));
            if (history.Count > 360) history.Dequeue();
        });
        var lines = history.ToList();
        int onset = lines.FindIndex(l => MathF.Abs(l.Slip) > 3.5f);
        _output.WriteLine($"result: spins {r.Spins} at s={r.SpinS:F0}; from 1 s before slip passed 3.5 deg:");
        foreach (var line in lines.Skip(Math.Max(0, onset - 60)).Take(120).Where((_, i) => i % 4 == 0)) _output.WriteLine(line.Line);
    }
}

public sealed class LaneRoadGripCornerProbe
{
    private readonly ITestOutputHelper _output;
    public LaneRoadGripCornerProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void TheCornerBeforeTheWornSpinAtEachRoadGrip()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        foreach (float g in new[] { 1.00f, 0.85f, 0.70f })
        {
            var r = LaneLapRunner.Run(track, file, startSpeed: 30f, flyingLaps: 1, roadGrip: g, onStep: (car, d, t) =>
            {
                float cs = car.Progress.CurrentS;
                if (car.Progress.Lap == 1 && cs > 3660f && cs < 3790f && (int)(t * 50) % 25 == 0)
                {
                    var s = car.State; var tel = s.Telemetry;
                    _output.WriteLine($"  g={g:F2} s={cs:F0} v={s.Speed:F1} tgt={d.LastTelemetry.TargetSpeed:F1} gripLvl={d.Grip:F2} grip F{tel.FrontGripAccel:F1} R{tel.RearGripAccel:F1} use f{tel.FrontLateralUse:F2} r{tel.RearLateralUse:F2} err={d.LastTelemetry.LateralErrorMeters:+0.00;-0.00} want {d.LastTelemetry.WantedCurvature:+0.0000;-0.0000} cap {d.LastTelemetry.CurvatureCap:+0.0000;-0.0000} slip={s.SideslipAngleRadians * 57.3f:+0.0;-0.0}");
                }
            });
            _output.WriteLine($"g={g:F2}: {r}");
        }
    }
}

public sealed class LaneSwitchDeviationProbe
{
    private readonly ITestOutputHelper _output;
    public LaneSwitchDeviationProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void WhereTheSwitchingCarLeavesItsLine()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        var r = LaneLapRunner.Run(track, file, onStep: (car, d, t) =>
        {
            int phase = (int)(t / 8f) % 4;
            d.TargetRange = phase == 0 ? 0f : 1f;
            d.TargetSide = phase == 1 ? 1f : phase == 2 ? 0f : -1f;
            float cs = car.Progress.CurrentS;
            if (car.Progress.Lap == 1 && cs > 4980f && cs < 5160f && (int)(t * 50) % 10 == 0)
            {
                var s = car.State; var tel = s.Telemetry; var lt = d.LastTelemetry;
                _output.WriteLine($"  t={t:F1} s={cs:F0} v={s.Speed:F1} tgt={lt.TargetSpeed:F1} plan={d.PlannedSpeedAt(cs):F1} range {lt.Range:F2}->{d.TargetRange:F0} side {lt.Side:+0.00;-0.00}->{d.TargetSide:+0;-0} err={lt.LateralErrorMeters:+0.00;-0.00} ff {lt.FeedforwardCurvature:+0.0000;-0.0000} want {lt.WantedCurvature:+0.0000;-0.0000} cap {lt.CurvatureCap:+0.0000;-0.0000} got {lt.CommandCurvature:+0.0000;-0.0000} a={lt.CommandAccel:+0.0;-0.0} use f{tel.FrontLateralUse:F2} r{tel.RearLateralUse:F2} slip={s.SideslipAngleRadians * 57.3f:+0.0;-0.0}");
            }
        });
        _output.WriteLine(r.ToString());
    }
}

public sealed class LaneLearnSideLanes
{
    private readonly ITestOutputHelper _output;
    public LaneLearnSideLanes(ITestOutputHelper output) => _output = output;

    /// <summary>The free lane was relearned after the TUM import; the side lanes never were.</summary>
    [Fact]
    public void LearnTheSideLanesAtEveryLevel()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        var watch = System.Diagnostics.Stopwatch.StartNew();
        for (int level = LaneTrackFile.TopLevel; level >= 0; level--)
        {
            foreach (LaneId side in new[] { LaneId.Left, LaneId.Right })
            {
                var (best, bestLap, history) = new LaneLearner { Reward = 0.02f }.Learn(track, file, 80, lane: side, level: level);
                _output.WriteLine($"{side} @ {LaneTrackFile.GripLevels[level]:F2}: first {history[0].Result} best {bestLap}  [{watch.Elapsed.TotalSeconds:F0}s]");
                file = best;
            }
        }
        string path = Path.Combine(LaneProbes.Dir, "silverstone-sides.lanes.csv");
        file.Save(path);
        _output.WriteLine($"saved {path}");

        foreach (float side in new[] { -1f, 0f, 1f })
            _output.WriteLine($"hold range 1 side {side:+0;-0}: {LaneLapRunner.Run(track, file, onStep: (car, d, t) => { d.TargetRange = 1f; d.TargetSide = side; })}");
        _output.WriteLine($"switch every 8 s: {LaneLapRunner.Run(track, file, onStep: (car, d, t) => { int phase = (int)(t / 8f) % 4; d.TargetRange = phase == 0 ? 0f : 1f; d.TargetSide = phase == 1 ? 1f : phase == 2 ? 0f : -1f; })}");
        _output.WriteLine($"free:             {LaneLapRunner.Run(track, file)}");
    }
}

public sealed class LaneDriverCostProbe
{
    private readonly ITestOutputHelper _output;
    public LaneDriverCostProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void HowMuchOneCarCosts()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneLapRunner.Run(track, file, flyingLaps: 1);
        foreach (var (label, gains) in new[] { ("default", new LaneDriverGains()), ("turn-aware", new LaneDriverGains { TurnAwareEnvelope = true }) })
        {
            var watch = System.Diagnostics.Stopwatch.StartNew();
            float simSeconds = 0f;
            var r = LaneLapRunner.Run(track, file, gains, flyingLaps: 5, onStep: (car, d, t) => simSeconds = t);
            watch.Stop();
            double perSimSecond = watch.Elapsed.TotalMilliseconds / simSeconds;
            _output.WriteLine($"{label}: {simSeconds:F0} s simulated in {watch.Elapsed.TotalSeconds:F2} s real -> {perSimSecond:F2} ms per simulated second per car; {1000.0 / perSimSecond:F0}x real time for one car, {1000.0 / perSimSecond / 20:F1}x for 20, {1000.0 / perSimSecond / 30:F1}x for 30 (one core)");
        }
    }
}

/// <summary>
/// Silverstone relearned for the F1E car, from the freshly baked lanes:
/// every grip level, free lane then both sides, with the turn-aware
/// envelope, three hot flying laps per candidate and a steering-cap
/// station counted as trouble.
/// </summary>
public sealed class LaneLearnF1E
{
    private readonly ITestOutputHelper _output;
    public LaneLearnF1E(ITestOutputHelper output) => _output = output;

    [Fact]
    public void LearnSilverstoneForTheF1ECar()
    {
        if (!LaneProbes.Enabled || !File.Exists(LaneBaked3d.Baked("silverstone"))) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.FromBakedLanes(LaneBaked3d.Baked("silverstone"), track);
        var gains = new LaneDriverGains { TurnAwareEnvelope = true };
        var learner = new LaneLearner { Reward = 0.02f, FlyingLaps = 3, CapBoundIsTrouble = true };
        var watch = System.Diagnostics.Stopwatch.StartNew();
        _output.WriteLine($"unlearned: {LaneLapRunner.Run(track, file, gains)}");
        for (int level = LaneTrackFile.TopLevel; level >= 0; level--)
        {
            // STINTEGY_LEARN_FREE_ONLY=1 learns the free lane alone, for when the sides are still baking.
            LaneId[] lanes = Environment.GetEnvironmentVariable("STINTEGY_LEARN_FREE_ONLY") == "1"
                ? new[] { LaneId.Free }
                : new[] { LaneId.Free, LaneId.Left, LaneId.Right };
            foreach (LaneId lane in lanes)
            {
                var (best, bestLap, history) = learner.Learn(track, file, lane == LaneId.Free ? 120 : 80, gains, lane: lane, level: level);
                _output.WriteLine($"{lane} @ {LaneTrackFile.GripLevels[level]:F2}: first {history[0].Result.FlyingLapSeconds:F2} best {bestLap}  [{watch.Elapsed.TotalMinutes:F0} min]");
                file = best;
            }
        }
        file.MarkLearnedNow();
        file.Energy = LaneTrackFile.ForCircuit("silverstone", track).Energy;
        string path = Path.Combine(LaneProbes.Dir, "silverstone");
        file.SaveCircuit(path, "silverstone");
        _output.WriteLine($"saved {path}");
        var attack = new StintegyEVO.Core.Cars.CarStrategy(StintegyEVO.Core.Cars.TireUsageMode.Attack, StintegyEVO.Core.Cars.PowerOutputMode.Normal);
        _output.WriteLine($"hot      {LaneLapRunner.Run(track, file, gains, flyingLaps: 3)}");
        _output.WriteLine($"attack   {LaneLapRunner.Run(track, file, gains, strategy: attack, flyingLaps: 3)}");
        _output.WriteLine($"cold     {LaneLapRunner.Run(track, file, gains, startSpeed: 0f, tireConfig: new StintegyEVO.Core.Cars.TireConfig())}");
        _output.WriteLine($"wear 0.8 {LaneLapRunner.Run(track, file, gains, startingWear: 0.8f)}");
        _output.WriteLine($"road .78 {LaneLapRunner.Run(track, file, gains, roadGrip: 0.78f)}");
        foreach (float side in new[] { -1f, 0f, 1f })
            _output.WriteLine($"hold range 1 side {side:+0;-0}: {LaneLapRunner.Run(track, file, gains, onStep: (car, d, t) => { d.TargetRange = 1f; d.TargetSide = side; })}");
        _output.WriteLine($"switch every 8 s: {LaneLapRunner.Run(track, file, gains, onStep: (car, d, t) => { int phase = (int)(t / 8f) % 4; d.TargetRange = phase == 0 ? 0f : 1f; d.TargetSide = phase == 1 ? 1f : phase == 2 ? 0f : -1f; })}");
    }
}
