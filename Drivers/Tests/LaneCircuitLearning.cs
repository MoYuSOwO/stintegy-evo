using System;
using System.IO;
using System.Linq;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// Not a test: the way a circuit's lanes are learned, kept beside the tests
/// because it drives the same runner. It does nothing unless asked to.
/// </summary>
public sealed class LaneCircuitLearning(ITestOutputHelper output)
{
    private readonly ITestOutputHelper _output = output;

    /// <summary>
    /// Every lane at every grip level, on a circuit's baked lanes, and the
    /// result saved under <see cref="LaneProbes.Dir"/>. Set
    /// STINTEGY_LANE_PROBES=1, STINTEGY_LEARN_CIRCUIT=&lt;id&gt; and, for a
    /// bake not shipped, STINTEGY_LANES3D=&lt;the bake's out folder&gt;;
    /// STINTEGY_LEARN_TOP_ONLY=1 learns the full-grip level alone.
    /// </summary>
    [Fact]
    public void LearnBakedCircuit()
    {
        if (!LaneProbes.Enabled || Environment.GetEnvironmentVariable("STINTEGY_LEARN_CIRCUIT") is not { Length: > 0 } id) return;
        TrackData track = CircuitLibrary.Load(id);
        string baked = LaneProbes.Baked(id);
        LaneTrackFile file = File.Exists(baked) ? LaneTrackFile.FromBakedLanes(baked, track) : LaneTrackFile.ForCircuit(id, track);
        LaneTrackFile shipped = LaneTrackFile.ForCircuit(id, track);
        file.Energy = shipped.Energy;
        var gains = new LaneDriverGains { TurnAwareEnvelope = true };
        var learner = new LaneLearner { Reward = 0.02f, FlyingLaps = 3, CapBoundIsTrouble = true };
        var watch = System.Diagnostics.Stopwatch.StartNew();
        _output.WriteLine($"{id} from {(File.Exists(baked) ? baked : "the geometric bake")}: {LaneLapRunner.Run(track, file, gains)}");
        // STINTEGY_LEARN_TOP_ONLY=1: full grip only, for comparing two ways of
        // driving, where the lower levels would only say the same again.
        int lowest = Environment.GetEnvironmentVariable("STINTEGY_LEARN_TOP_ONLY") == "1" ? LaneTrackFile.TopLevel : 0;
        for (int level = LaneTrackFile.TopLevel; level >= lowest; level--)
        {
            foreach (LaneId lane in new[] { LaneId.Free, LaneId.Left, LaneId.Right })
            {
                var (best, bestLap, history) = learner.Learn(track, file, lane == LaneId.Free ? 120 : 80, gains, lane: lane, level: level);
                _output.WriteLine($"{lane} @ {LaneTrackFile.GripLevels[level]:F2}: first {history[0].Result.FlyingLapSeconds:F2} kept {history.Count(h => h.Kept)}/{history.Count} best {bestLap}  [{watch.Elapsed.TotalMinutes:F0} min]");
                file = best;
            }
        }
        file.MarkLearnedNow();
        string path = Path.Combine(LaneProbes.Dir, id);
        file.SaveCircuit(path, id);
        _output.WriteLine($"saved {path}");
        _output.WriteLine($"hot      {LaneLapRunner.Run(track, file, gains, flyingLaps: 3)}");
        _output.WriteLine($"attack   {LaneLapRunner.Run(track, file, gains, strategy: new CarStrategy(TireUsageMode.Attack, PowerOutputMode.Normal), flyingLaps: 3)}");
        _output.WriteLine($"standing {LaneLapRunner.Run(track, file, gains, startSpeed: 0f, tireTempC: Tire.BlanketTempC)}");
        foreach (float side in new[] { -1f, 0f, 1f })
            _output.WriteLine($"hold range 1 side {side:+0;-0}: {LaneLapRunner.Run(track, file, gains, onStep: (car, d, t) => { d.TargetRange = 1f; d.TargetSide = side; })}");
        _output.WriteLine($"switch every 8 s: {LaneLapRunner.Run(track, file, gains, onStep: (car, d, t) => { int phase = (int)(t / 8f) % 4; d.TargetRange = phase == 0 ? 0f : 1f; d.TargetSide = phase == 1 ? 1f : phase == 2 ? 0f : -1f; })}");
    }
}
