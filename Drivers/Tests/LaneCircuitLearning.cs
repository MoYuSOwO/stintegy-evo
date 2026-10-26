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
    private readonly object _gate = new();

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
        bool append = Environment.GetEnvironmentVariable("STINTEGY_LEARN_APPEND_LEVELS") == "1";
        LaneTrackFile file;
        if (append)
        {
            string learned = Path.Combine(LaneProbes.Dir, id);
            file = File.Exists(Path.Combine(learned, "driver.json"))
                ? LaneTrackFile.LoadCircuit(learned, track)
                : (File.Exists(baked) ? LaneTrackFile.FromBakedLanes(baked, track) : LaneTrackFile.ForCircuit(id, track));
        }
        else
        {
            file = File.Exists(baked) ? LaneTrackFile.FromBakedLanes(baked, track) : LaneTrackFile.ForCircuit(id, track);
        }
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
        // STINTEGY_LEARN_PROGRESS=<file>: a line after every lap the learner drives.
        string? progressPath = Environment.GetEnvironmentVariable("STINTEGY_LEARN_PROGRESS");
        var watch0 = System.Diagnostics.Stopwatch.StartNew();
        void Progress(string line)
        {
            if (progressPath is { Length: > 0 })
                lock (_gate)
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
        _output.WriteLine($"{id} from {(append ? "the existing learned file" : File.Exists(baked) ? baked : "the geometric bake")}: {LaneLapRunner.Run(track, file, gains)}");
        // STINTEGY_LEARN_TOP_ONLY=1: full grip only, for comparing two ways of
        // driving, where the lower levels would only say the same again.
        // STINTEGY_LEARN_APPEND_LEVELS=1: keep the full-grip factors and
        // steering already in the learned file, and add the lower levels only.
        int lowest = append ? 0 : (Environment.GetEnvironmentVariable("STINTEGY_LEARN_TOP_ONLY") == "1" ? LaneTrackFile.TopLevel : 0);
        int highest = append ? LaneTrackFile.TopLevel - 1 : LaneTrackFile.TopLevel;
        // The three lanes learn side by side, each on its own copy of the
        // file: a lane's factors and steering are its own, and a car settled
        // on a side lane plans on that lane alone. Each lane still goes down
        // the levels in order, as a level is capped by the ones above it.
        // One after another, Silverstone took nine minutes at full grip.
        LaneId[] lanes = [LaneId.Free, LaneId.Left, LaneId.Right];
        LaneTrackFile[] byLane = new LaneTrackFile[lanes.Length];
        System.Threading.Tasks.Parallel.For(0, lanes.Length, k =>
        {
            LaneId lane = lanes[k];
            LaneTrackFile mine = file.Clone();
            for (int level = highest; level >= lowest; level--)
            {
                // The pace first, until the lap gets round on the road: the
                // steering learns from a lap's error and a lap off the road or
                // spun says nothing about the lock. Then the steering, where
                // the car is off its lane for want of lock at the right moment
                // rather than for speed; then the pace again on the steering
                // learned, and the steering once more for the corners the new
                // pace changed. The steering is one table a lane, learned at
                // full grip.
                int rounds = lane == LaneId.Free ? 20 : 15;
                (mine, bool paceLearned) = Pace(track, mine, gains, learner, lane, level, rounds, watch);
                if (level == LaneTrackFile.TopLevel)
                {
                    (mine, bool steerLearned) = Steer(track, mine, gains, lane, 15, "steer");
                    // Learning is deterministic: from the factors and steering
                    // the first pace began with, the second would drive every
                    // round of it again.
                    if (paceLearned || steerLearned)
                        (mine, _) = Pace(track, mine, gains, learner, lane, level, rounds, watch);
                    else
                        _progress?.Invoke($"{lane} pace again skipped: it would begin where the first did");
                    (mine, _) = Steer(track, mine, gains, lane, 10, "steer again");
                }
            }
            byLane[k] = mine;
        });
        for (int k = 0; k < lanes.Length; k++)
        {
            for (int level = highest; level >= lowest; level--)
            {
                byLane[k].Factors(lanes[k], level).CopyTo(file.Factors(lanes[k], level), 0);
                byLane[k].BrakeFactors(lanes[k], level).CopyTo(file.BrakeFactors(lanes[k], level), 0);
            }
            byLane[k].Steer(lanes[k]).CopyTo(file.Steer(lanes[k]), 0);
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

    private (LaneTrackFile, bool Learned) Pace(TrackData track, LaneTrackFile file, LaneDriverGains gains, LaneLearner learner, LaneId lane, int level, int rounds, System.Diagnostics.Stopwatch watch)
    {
        var (paced, lap, history) = learner.LearnSectors(track, file, rounds, gains, lane: lane, level: level);
        lock (_gate)
            _output.WriteLine($"{lane} @ {LaneTrackFile.GripLevels[level]:F2} pace: first {history[0].Result.FlyingLapSeconds:F2} better {history.Count(h => h.Kept)}/{history.Count} best {lap}  [{watch.Elapsed.TotalMinutes:F0} min]");
        return (paced, history.Skip(1).Any(h => h.Kept));
    }

    private (LaneTrackFile, bool Learned) Steer(TrackData track, LaneTrackFile file, LaneDriverGains gains, LaneId lane, int iterations, string label)
    {
        var (steered, lap, history) = new LaneLearner { Progress = _progress }.LearnSteering(track, file, iterations, gains, lane);
        lock (_gate)
            _output.WriteLine($"{lane} {label}: error {history[0].Result.MeanAbsErrorMeters:F2}/{history[0].Result.WorstErrorMeters:F2} -> {lap.MeanAbsErrorMeters:F2}/{lap.WorstErrorMeters:F2} kept {history.Count(h => h.Kept)}/{history.Count}");
        return (steered, history.Skip(1).Any(h => h.Kept));
    }
}
