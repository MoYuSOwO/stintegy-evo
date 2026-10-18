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
    private Action<string>? _progress;

    /// <summary>
    /// Every lane at every grip level, on a circuit's baked lanes, and the
    /// result saved under <see cref="LaneProbes.Dir"/>. Set
    /// STINTEGY_LANE_PROBES=1, STINTEGY_LEARN_CIRCUIT=&lt;id&gt; and, for a
    /// bake not shipped, STINTEGY_LANES3D=&lt;the bake's out folder&gt;;
    /// STINTEGY_LEARN_TOP_ONLY=1 learns the full-grip level alone, and
    /// STINTEGY_LEARN_LOOSE=1 keeps laps that are merely on the road
    /// (LaneLearner.BestMustBeTroubleFree off), for comparison.
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
        // A racing car is where it says it is: half a metre off its lane is
        // trouble, and only a lap with none anywhere is kept. Within that it
        // is pushed: wherever it ran within four tenths, the pace goes up,
        // and a sector pushed past the line goes back. Reaching full lock is
        // not trouble by itself here; at the limit a car touches it in every
        // corner, and counted as trouble it held the whole lap back from the
        // limit with the car on its lane.
        bool loose = Environment.GetEnvironmentVariable("STINTEGY_LEARN_LOOSE") == "1";
        // STINTEGY_LEARN_BY_LAP=1: the old way, a whole lap kept or thrown away.
        bool byLap = Environment.GetEnvironmentVariable("STINTEGY_LEARN_BY_LAP") == "1";
        // STINTEGY_LEARN_PROGRESS=<file>: a line after every lap the learner drives.
        string? progressPath = Environment.GetEnvironmentVariable("STINTEGY_LEARN_PROGRESS");
        var watch0 = System.Diagnostics.Stopwatch.StartNew();
        void Progress(string line)
        {
            if (progressPath is { Length: > 0 })
                File.AppendAllText(progressPath, $"[{watch0.Elapsed.TotalMinutes,5:F1} min] {id} {line}{Environment.NewLine}");
        }
        _progress = Progress;
        var learner = new LaneLearner
        {
            Progress = Progress,
            Reward = 0.02f, FlyingLaps = 3, CapBoundIsTrouble = loose,
            BestMustBeTroubleFree = !loose,
            TroubleErrorMeters = loose ? 1.2f : 0.5f,
            CleanErrorMeters = loose ? 0.6f : 0.4f,
        };
        var watch = System.Diagnostics.Stopwatch.StartNew();
        _output.WriteLine($"{id} from {(File.Exists(baked) ? baked : "the geometric bake")}: {LaneLapRunner.Run(track, file, gains)}");
        // STINTEGY_LEARN_TOP_ONLY=1: full grip only, for comparing two ways of
        // driving, where the lower levels would only say the same again.
        int lowest = Environment.GetEnvironmentVariable("STINTEGY_LEARN_TOP_ONLY") == "1" ? LaneTrackFile.TopLevel : 0;
        for (int level = LaneTrackFile.TopLevel; level >= lowest; level--)
        {
            foreach (LaneId lane in new[] { LaneId.Free, LaneId.Left, LaneId.Right })
            {
                if (byLap)
                {
                    var (best, bestLap, history) = learner.Learn(track, file, lane == LaneId.Free ? 120 : 80, gains, lane: lane, level: level);
                    _output.WriteLine($"{lane} @ {LaneTrackFile.GripLevels[level]:F2}: first {history[0].Result.FlyingLapSeconds:F2} kept {history.Count(h => h.Kept)}/{history.Count} best {bestLap}  [{watch.Elapsed.TotalMinutes:F0} min]");
                    file = best;
                    continue;
                }
                // The pace first, until the lap gets round on the road: the
                // steering learns from a lap's error and a lap off the road or
                // spun says nothing about the lock. Then the steering, where
                // the car is off its lane for want of lock at the right moment
                // rather than for speed; then the pace again on the steering
                // learned, and the steering once more for the corners the new
                // pace changed. The steering is one table a lane, learned at
                // full grip.
                int rounds = lane == LaneId.Free ? 20 : 15;
                file = Pace(track, file, gains, learner, lane, level, rounds, watch);
                if (level == LaneTrackFile.TopLevel)
                {
                    file = Steer(track, file, gains, lane, 15, "steer");
                    file = Pace(track, file, gains, learner, lane, level, rounds, watch);
                    file = Steer(track, file, gains, lane, 10, "steer again");
                }
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

    private LaneTrackFile Pace(TrackData track, LaneTrackFile file, LaneDriverGains gains, LaneLearner learner, LaneId lane, int level, int rounds, System.Diagnostics.Stopwatch watch)
    {
        var (paced, lap, history) = learner.LearnSectors(track, file, rounds, gains, lane: lane, level: level);
        _output.WriteLine($"{lane} @ {LaneTrackFile.GripLevels[level]:F2} pace: first {history[0].Result.FlyingLapSeconds:F2} better {history.Count(h => h.Kept)}/{history.Count} best {lap}  [{watch.Elapsed.TotalMinutes:F0} min]");
        return paced;
    }

    private LaneTrackFile Steer(TrackData track, LaneTrackFile file, LaneDriverGains gains, LaneId lane, int iterations, string label)
    {
        var (steered, lap, history) = new LaneLearner { Progress = _progress }.LearnSteering(track, file, iterations, gains, lane);
        _output.WriteLine($"{lane} {label}: error {history[0].Result.MeanAbsErrorMeters:F2}/{history[0].Result.WorstErrorMeters:F2} -> {lap.MeanAbsErrorMeters:F2}/{lap.WorstErrorMeters:F2} kept {history.Count(h => h.Kept)}/{history.Count}");
        return steered;
    }
}
