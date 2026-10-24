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
            foreach (LaneId lane in new[] { LaneId.Free, LaneId.Left, LaneId.Right, LaneId.Middle })
                minRoom = MathF.Min(minRoom, half - MathF.Abs(lanes.Offset(lane, i)));
            leftOverRight += lanes.Offset(LaneId.Left, i) - lanes.Offset(LaneId.Right, i);
        }
        _output.WriteLine($"stations {lanes.Count} min room to edge {minRoom:F2} m, mean left-right gap {leftOverRight / lanes.Count:F2} m");
        Assert.True(minRoom >= 0.8f, $"a lane comes within {minRoom:F2} m of the edge");
        Assert.True(leftOverRight / lanes.Count > 1f);
    }

    [Fact]
    public void TheShippedLanesLieLeftToRightInTheirOrder()
    {
        // A bake that took the other side of the road for left put the left
        // lane on the right the whole lap, and every move to make room went
        // into the car it was making room for.
        TrackData track = CircuitLibrary.Load("silverstone");
        LaneSet lanes = Silverstone(track).LanesFor(track);
        int wrong = 0;
        for (int i = 0; i < lanes.Count; i++)
        {
            float left = lanes.Offset(LaneId.Left, i), middle = lanes.Offset(LaneId.Middle, i), right = lanes.Offset(LaneId.Right, i);
            if (!(left > middle && middle > right))
                wrong++;
        }
        Assert.True(wrong == 0, $"left, middle, right out of order at {wrong} of {lanes.Count} stations");
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
        Assert.NotNull(DriverFiles.FindFile("formula_a_mixed/silverstone/driver.json"));
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
        Assert.Equal(0f, r.WallSeconds);
    }

    [Fact]
    public void BakuSwitchWithTheLearnedFile()
    {
        if (!LaneProbes.Enabled) return;
        string folder = Environment.GetEnvironmentVariable("STINTEGY_CIRCUIT_FOLDER") ?? "";
        if (folder.Length == 0) return;
        TrackData track = CircuitLibrary.Load("baku");
        LaneLapResult r = LaneLapRunner.Run(track, LaneTrackFile.LoadCircuit(folder, track), onStep: (car, d, t) =>
        {
            int phase = (int)(t / 8f) % 4;
            d.TargetRange = phase == 0 ? 0f : 1f;
            d.TargetSide = phase == 1 ? 1f : phase == 2 ? 0f : -1f;
        });
        _output.WriteLine(r.ToString());
        Assert.True(r.Finished, r.ToString());
        Assert.Equal(0f, r.OffTrackSeconds);
        Assert.Equal(0f, r.WallSeconds);
    }

    /// <summary>
    /// Full grip is the circuit at its best. On asphalt a tenth slippier
    /// than the average circuit, new tyres in their window still read as
    /// full grip, so the driver uses what it learned at full grip, and the
    /// lap is still clean because the slipperier road is in the envelope.
    /// </summary>
    [Fact]
    public void FullGripIsTheCircuitAtItsBest()
    {
        TrackData track = CircuitLibrary.Load("monaco");
        track.AsphaltGrip = 0.9f;
        var read = new List<float>();
        LaneLapResult r = LaneLapRunner.Run(track, LaneTrackFile.ForCircuit("monaco", track), onStep: (car, d, t) =>
        {
            if (t > 5f)
                read.Add(d.ActualGrip);
        });
        _output.WriteLine($"{r}  grip read {read.Min():F3}..{read.Max():F3}");
        Assert.True(r.Finished, r.ToString());
        Assert.Equal(0, r.Spins);
        Assert.Equal(0f, r.OffTrackSeconds);
        read.Sort();
        Assert.InRange(read[read.Count / 2], 0.95f, 1.05f);
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
