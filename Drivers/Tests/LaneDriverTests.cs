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

    /// <summary>
    /// The offline 3D bake's output for a circuit: from <c>STINTEGY_LANES3D</c>
    /// when set, else the shipped one in <c>Drivers/Bake/lanes/</c>; empty when
    /// neither has it.
    /// </summary>
    public static string Baked(string id)
    {
        if (Environment.GetEnvironmentVariable("STINTEGY_LANES3D") is { Length: > 0 } dir)
            return Path.Combine(dir, $"{id}.lanes3d.csv");
        DirectoryInfo? at = new(AppContext.BaseDirectory);
        while (at != null && !File.Exists(Path.Combine(at.FullName, "StintegyEVO.sln")))
            at = at.Parent;
        string shipped = at == null ? "" : Path.Combine(at.FullName, "Drivers", "Bake", "lanes", $"{id}.lanes3d.csv");
        return File.Exists(shipped) ? shipped : "";
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
    public void TheShippedSilverstoneFolderLapsCleanly()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        Assert.NotNull(RuleLineAssets.FindFile("f1e/silverstone/driver.json"));
        LaneLapResult r = LaneLapRunner.Run(track, Silverstone(track));
        _output.WriteLine(r.ToString());
        Assert.True(r.Finished, r.ToString());
        Assert.Equal(0f, r.OffTrackSeconds);
        Assert.InRange(r.FlyingLapSeconds, 85f, 115f);
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

public sealed class LaneLearnF1E
{
    private readonly ITestOutputHelper _output;
    public LaneLearnF1E(ITestOutputHelper output) => _output = output;

    [Fact]
    public void LearnSilverstoneForTheF1ECar()
    {
        if (!LaneProbes.Enabled || !File.Exists(LaneProbes.Baked("silverstone"))) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.FromBakedLanes(LaneProbes.Baked("silverstone"), track);
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
