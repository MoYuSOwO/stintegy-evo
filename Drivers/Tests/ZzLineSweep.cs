using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Util;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// Probe, not a test: random lines through one stretch of road, each with the
/// car arriving in its own condition, driven on the real physics at a range
/// of corner and braking shares, to see whether what a line needs can be
/// learned stretch by stretch.
/// </summary>
public sealed class ZzLineSweep
{
    private readonly ITestOutputHelper _output;
    public ZzLineSweep(ITestOutputHelper output) => _output = output;

    private static float Env(string name, float fallback) =>
        float.TryParse(Environment.GetEnvironmentVariable(name), NumberStyles.Float, CultureInfo.InvariantCulture, out float v) ? v : fallback;

    private static string F(float v) => v.ToString("F4", CultureInfo.InvariantCulture);

    [Fact]
    public void Corners()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = file.LanesFor(track);
        bool inside = false;
        float from = 0f, sharpest = 0f;
        for (int i = 0; i < lanes.Count; i++)
        {
            float k = MathF.Abs(LaneSet.Curvature(
                lanes.Position(i - 3, lanes.Offset(LaneId.Free, i - 3)),
                lanes.Position(i, lanes.Offset(LaneId.Free, i)),
                lanes.Position(i + 3, lanes.Offset(LaneId.Free, i + 3))));
            if (k > 1f / 400f && !inside) { inside = true; from = i * lanes.Spacing; sharpest = 0f; }
            if (inside) sharpest = MathF.Max(sharpest, k);
            if (k < 1f / 600f && inside)
            {
                inside = false;
                _output.WriteLine($"corner {from,6:F0}..{i * lanes.Spacing,6:F0} m  tightest radius {1f / sharpest,5:F0} m");
            }
        }

        var watch = Stopwatch.StartNew();
        LaneLapResult r = LaneLapRunner.Run(track, file);
        _output.WriteLine($"one out-lap + flying lap: {watch.Elapsed.TotalSeconds:F2} s wall, {r}");
        LaneLapResult open = LaneLapRunner.Run(track, file, new LaneDriverGains { CapFromAxles = false }, flyingLaps: 3);
        _output.WriteLine($"without the axle cap, three laps: {string.Join(" ", open.LapTimes.Select(t => t.ToString("F2")))} {open}");
    }

    private sealed record Stretch(float StartS, float FromS, float ToS, float EndS);

    /// <summary>
    /// The car and the day as the car arrives at the stretch. Front and rear
    /// tyres apart, because the balance between them is what makes a car
    /// push or step out. A lead gap of zero means no car ahead.
    /// </summary>
    private sealed record Condition(
        float RoadGrip, float TrackTempC,
        int Compound, float FrontTempC, float RearTempC, float FrontWear, float RearWear,
        float Fuel, float Charge, PowerOutputMode Power, TireUsageMode TyreMode,
        float EntrySpeed, float EntryOffset, float EntryHeading,
        float LeadGap,
        float Pace, float Consistency, float Precision, float Focus, float Recovery, float Reactions)
    {
        public static readonly string Header =
            "road_grip,track_temp_c,compound,front_temp_c,rear_temp_c,front_wear,rear_wear,fuel,charge,power_mode,tyre_mode," +
            "entry_mps,entry_offset_m,entry_heading_rad,lead_gap_m,pace,consistency,precision,focus,recovery,reactions";

        public string Row() => string.Join(",", new[]
        {
            F(RoadGrip), F(TrackTempC), Compound.ToString(CultureInfo.InvariantCulture), F(FrontTempC), F(RearTempC), F(FrontWear), F(RearWear),
            F(Fuel), F(Charge), ((int)Power).ToString(CultureInfo.InvariantCulture), ((int)TyreMode).ToString(CultureInfo.InvariantCulture),
            F(EntrySpeed), F(EntryOffset), F(EntryHeading), F(LeadGap),
            F(Pace), F(Consistency), F(Precision), F(Focus), F(Recovery), F(Reactions)
        });
    }

    private sealed record Run(float Seconds, bool Spun, float Off, float Wall, float WorstError, float WorstSlip, bool Touched, float Wake)
    {
        /// <summary>Where the car first strayed 0.6 m off its line in the stretch (or spun), NaN if it never did.</summary>
        public float StrayS { get; init; } = float.NaN;

        /// <summary>Whether it was braking when it strayed.</summary>
        public bool StrayBraking { get; init; }

        /// <summary>Per station, as a learning lap keeps it: how far off and how sideways, off the road, braking, on the throttle.</summary>
        public float[] StationError { get; init; } = [];
        public float[] StationSlip { get; init; } = [];
        public bool[] StationOff { get; init; } = [];
        public bool[] StationBraking { get; init; } = [];
        public bool[] StationThrottle { get; init; } = [];
        public float[] StationSeconds { get; init; } = [];
        public float SpinS { get; init; } = float.NaN;

        /// <summary>Kept on the road and on the line; touching the car ahead is counted apart.</summary>
        public bool Clean => !Spun && Off <= 0f && Wall <= 0f && WorstError <= 1.2f;

        public static readonly string Header = "seconds,clean,spun,off_s,wall_s,worst_error_m,worst_slip_rad,touched,wake_mean,stray_s,stray_braking";

        public string Row() => string.Join(",", new[]
        {
            F(Seconds), Clean ? "1" : "0", Spun ? "1" : "0", F(Off), F(Wall), F(WorstError), F(WorstSlip), Touched ? "1" : "0", F(Wake),
            F(StrayS), StrayBraking ? "1" : "0"
        });
    }

    /// <summary>
    /// STINTEGY_SWEEP_LINES lines (line 0 the learned free line itself, car
    /// as it comes). Each is driven first on a coarse ladder of one share for
    /// both corner and braking, then at random corner and braking shares
    /// apart, close round where the ladder found the edge.
    /// </summary>
    private sealed record Setup(
        Stretch Stretch, TrackData Track, LaneTrackFile Shipped, LaneSet Lanes, string[] Compounds,
        float EntrySpeed, float[][] Knots, Condition[] Conditions, int[] Seeds);

    /// <summary>
    /// The stretch, and every line and condition drawn from one seed, so a
    /// later replay meets exactly the same lines. STINTEGY_SWEEP_FULL=1 draws
    /// the car's whole condition; otherwise only the road's grip, the tyres'
    /// heat and wear front and rear, and the speed in, everything else as a
    /// learning lap has it.
    /// </summary>
    private static Setup Build(int lines)
    {
        // Default: Brooklands and Luffield, Wellington's braking zone and both corners.
        var stretch = new Stretch(StartS: Env("STINTEGY_SWEEP_START", 1350f), FromS: Env("STINTEGY_SWEEP_FROM", 1650f),
            ToS: Env("STINTEGY_SWEEP_TO", 2550f), EndS: Env("STINTEGY_SWEEP_END", 2700f));
        const int Knots = 5;
        bool full = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_FULL") == "1";

        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile shipped = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = shipped.LanesFor(track);
        string[] compounds = ((ICarTires)StockCar.Model).Compounds.Select(c => c.Id).ToArray();

        // The speed the learned free line carries where each run starts.
        LaneLapResult reference = LaneLapRunner.Run(track, shipped);
        float entrySpeed = reference.StationSpeed[lanes.Wrap((int)MathF.Round(lanes.StationOf(stretch.StartS)))];

        var random = new Random(20261028);
        float Between(float a, float b) => a + (b - a) * (float)random.NextDouble();
        var knots = new float[lines][];
        var conditions = new Condition[lines];
        var seeds = new int[lines];
        int super = Array.FindIndex(compounds, c => c == DevelopmentTyres.SuperId);
        var asLearned = new Condition(1f, 35f, super, 90f, 90f, 0f, 0f, 0.8f, 0.8f, PowerOutputMode.Normal, TireUsageMode.Normal,
            entrySpeed, 0f, 0f, 0f, 100f, 100f, 100f, 100f, 100f, 100f);
        for (int n = 0; n < lines; n++)
        {
            knots[n] = new float[Knots];
            seeds[n] = random.Next();
            conditions[n] = asLearned;
            if (n == 0)
                continue;
            for (int j = 0; j < Knots; j++)
                knots[n][j] = Between(-1f, 1f);
            conditions[n] = full
                ? new Condition(
                    RoadGrip: Between(0.85f, 1f), TrackTempC: Between(20f, 50f),
                    Compound: random.Next(compounds.Length),
                    FrontTempC: Between(70f, 110f), RearTempC: Between(70f, 110f),
                    FrontWear: Between(0f, 0.5f), RearWear: Between(0f, 0.5f),
                    Fuel: Between(0.05f, 1f), Charge: Between(0.2f, 0.9f),
                    Power: (PowerOutputMode)random.Next(1, 6), TyreMode: (TireUsageMode)random.Next(1, 6),
                    EntrySpeed: entrySpeed * Between(0.85f, 1f), EntryOffset: Between(-0.7f, 0.7f), EntryHeading: Between(-0.03f, 0.03f),
                    LeadGap: random.NextDouble() < 0.5 ? 0f : Between(3f, 30f),
                    Pace: Between(60f, 100f), Consistency: Between(60f, 100f), Precision: Between(60f, 100f),
                    Focus: Between(60f, 100f), Recovery: Between(60f, 100f), Reactions: Between(60f, 100f))
                : asLearned with
                {
                    RoadGrip = Between(0.85f, 1f),
                    FrontTempC = Between(70f, 110f), RearTempC = Between(70f, 110f),
                    FrontWear = Between(0f, 0.5f), RearWear = Between(0f, 0.5f),
                    EntrySpeed = entrySpeed * Between(0.85f, 1f)
                };
        }
        return new Setup(stretch, track, shipped, lanes, compounds, entrySpeed, knots, conditions, seeds);
    }

    private static Run Try(Setup setup, int n, LaneSet drawn, float corner, float brake)
    {
        LaneTrackFile file = setup.Shipped.WithLanes(drawn);
        Prepare(file, setup.Lanes, setup.Stretch, corner, brake);
        return Drive(setup.Track, setup.Shipped, file, drawn, setup.Stretch, setup.Conditions[n], setup.Compounds);
    }

    private static string Header =>
        "line," + string.Join(",", Enumerable.Range(0, 5).Select(j => $"k{j}")) + "," +
        Condition.Header + ",phase,corner_share,brake_share," + Run.Header + "\n";

    /// <summary>
    /// STINTEGY_SWEEP_LINES lines (line 0 the learned free line itself, car
    /// as it comes). Each is driven first on a coarse ladder of one share for
    /// both corner and braking, then at random corner and braking shares
    /// apart, close round where the ladder found the edge.
    /// </summary>
    [Fact]
    public void Sweep()
    {
        if (!LaneProbes.Enabled) return;
        int lines = (int)Env("STINTEGY_SWEEP_LINES", 50);
        int near = (int)Env("STINTEGY_SWEEP_NEAR", 12);
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "line-sweep.csv");
        float[] ladder = [0.70f, 0.79f, 0.88f, 0.97f, 1.06f, 1.15f, 1.24f];
        Setup setup = Build(lines);
        (Stretch stretch, LaneSet lanes, float[][] knots, Condition[] conditions, int[] seeds, float entrySpeed) =
            (setup.Stretch, setup.Lanes, setup.Knots, setup.Conditions, setup.Seeds, setup.EntrySpeed);

        var rows = new List<string>[lines];
        var watch = Stopwatch.StartNew();
        int done = 0;
        Parallel.For(0, lines, new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount }, n =>
        {
            LaneSet drawn = lanes.WithOffsets(LaneId.Free, Draw(lanes, stretch, knots[n]));
            string head = $"{n}," + string.Join(",", knots[n].Select(F)) + "," + conditions[n].Row();
            rows[n] = new List<string>();
            float edge = ladder[0];
            bool clean = true;
            foreach (float share in ladder)
            {
                Run r = Try(setup, n, drawn, share, share);
                rows[n].Add($"{head},ladder,{F(share)},{F(share)},{r.Row()}");
                clean &= r.Clean;
                if (clean)
                    edge = share;
            }
            var local = new Random(seeds[n]);
            for (int j = 0; j < near; j++)
            {
                float corner = Math.Clamp(edge - 0.10f + 0.25f * (float)local.NextDouble(), 0.6f, 1.35f);
                float brake = Math.Clamp(edge - 0.10f + 0.25f * (float)local.NextDouble(), 0.6f, 1.35f);
                Run r = Try(setup, n, drawn, corner, brake);
                rows[n].Add($"{head},near,{F(corner)},{F(brake)},{r.Row()}");
            }
            int now = System.Threading.Interlocked.Increment(ref done);
            if (now % 500 == 0)
                Console.WriteLine($"{now}/{lines} lines, {watch.Elapsed.TotalSeconds:F0} s");
        });

        var csv = new StringBuilder(Header);
        foreach (List<string> line in rows)
            foreach (string row in line)
                csv.Append(row).Append('\n');
        File.WriteAllText(output, csv.ToString());

        // Each line's offsets too, for drawing and for features.
        var offsets = new StringBuilder("line,station,s_m,offset_m,half_width_m\n");
        int first = (int)MathF.Floor(lanes.StationOf(stretch.FromS)), last = (int)MathF.Ceiling(lanes.StationOf(stretch.ToS));
        for (int n = 0; n < lines; n++)
        {
            float[] o = Draw(lanes, stretch, knots[n]);
            for (int i = first; i <= last; i++)
                offsets.Append(CultureInfo.InvariantCulture, $"{n},{i},{i * lanes.Spacing:F1},{o[i]:F3},{lanes.HalfWidth(i):F3}\n");
        }
        File.WriteAllText(Path.ChangeExtension(output, ".offsets.csv"), offsets.ToString());
        _output.WriteLine($"{lines} lines x ({ladder.Length} + {near}) runs in {watch.Elapsed.TotalSeconds:F0} s, entry {entrySpeed:F1} m/s -> {output}");
    }

    /// <summary>
    /// Drives the shares a model picked for lines it never saw: one row of
    /// STINTEGY_SWEEP_PICKS (line,label,corner_share,brake_share) each, the
    /// lines and conditions rebuilt from the same seed and count.
    /// </summary>
    [Fact]
    public void Replay()
    {
        if (!LaneProbes.Enabled) return;
        string? picks = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_PICKS");
        if (picks is null) return;
        Setup setup = Build((int)Env("STINTEGY_SWEEP_LINES", 50));
        var wanted = File.ReadAllLines(picks).Skip(1).Where(l => l.Length > 0).Select(l => l.Split(',')).ToArray();
        var rows = new string[wanted.Length];
        Parallel.For(0, wanted.Length, new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount }, i =>
        {
            int n = int.Parse(wanted[i][0], CultureInfo.InvariantCulture);
            float corner = float.Parse(wanted[i][2], CultureInfo.InvariantCulture);
            float brake = float.Parse(wanted[i][3], CultureInfo.InvariantCulture);
            LaneSet drawn = setup.Lanes.WithOffsets(LaneId.Free, Draw(setup.Lanes, setup.Stretch, setup.Knots[n]));
            Run r = Try(setup, n, drawn, corner, brake);
            rows[i] = $"{n},{wanted[i][1]},{F(corner)},{F(brake)},{r.Row()}";
        });
        File.WriteAllLines(Path.ChangeExtension(picks, ".driven.csv"), new[] { "line,label,corner_share,brake_share," + Run.Header }.Concat(rows));
        _output.WriteLine($"{rows.Length} picks driven");
    }

    /// <summary>How many knots each kind of factor has through a stretch.</summary>
    private const int ProfileKnots = 8;

    /// <summary>
    /// For a learner outside: STINTEGY_SWEEP_JOBS rows of
    /// id, k0..k4, road_grip, front_temp_c, rear_temp_c, front_wear, rear_wear,
    /// entry_share, then the corner, braking and traction factors at
    /// ProfileKnots knots each, set outright through the stretch at every
    /// grip level. One row out per job.
    /// </summary>
    [Fact]
    public void Batch()
    {
        if (!LaneProbes.Enabled) return;
        string? jobs = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_JOBS");
        if (jobs is null) return;
        Setup setup = Build(1);
        Condition learned = setup.Conditions[0];
        var wanted = File.ReadAllLines(jobs).Skip(1).Where(l => l.Length > 0)
            .Select(l => l.Split(',').Select(v => float.Parse(v, CultureInfo.InvariantCulture)).ToArray()).ToArray();
        var rows = new string[wanted.Length];
        Parallel.For(0, wanted.Length, new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount }, i =>
        {
            float[] v = wanted[i];
            float[] knots = v[1..6];
            Condition c = learned with
            {
                RoadGrip = v[6], FrontTempC = v[7], RearTempC = v[8], FrontWear = v[9], RearWear = v[10],
                EntrySpeed = setup.EntrySpeed * v[11]
            };
            float[] profile = v[12..];
            LaneSet drawn = setup.Lanes.WithOffsets(LaneId.Free, Draw(setup.Lanes, setup.Stretch, knots));
            LaneTrackFile file = setup.Shipped.WithLanes(drawn);
            SetProfile(file, setup.Lanes, setup.Stretch, profile);
            Run r = Drive(setup.Track, setup.Shipped, file, drawn, setup.Stretch, c, setup.Compounds);
            rows[i] = $"{(int)v[0]},{r.Row()}";
        });
        File.WriteAllLines(Path.ChangeExtension(jobs, ".driven.csv"), new[] { "id," + Run.Header }.Concat(rows));
        _output.WriteLine($"{rows.Length} jobs driven");
    }

    /// <summary>The stations a profile covers: from the braking zone before the stretch to its end.</summary>
    private static (int First, int Last) ProfileStations(LaneSet lanes, Stretch stretch) =>
        ((int)MathF.Floor(lanes.StationOf(stretch.FromS - 150f)), (int)MathF.Ceiling(lanes.StationOf(stretch.ToS)));

    /// <summary>
    /// The road under the stretch for a learner outside, one row a station
    /// from the braking zone before it to its end: where the centre is, which
    /// way is left, how wide, where the free line runs, and whether the line
    /// is drawn there. Written to STINTEGY_SWEEP_GEOMETRY.
    /// </summary>
    [Fact]
    public void Geometry()
    {
        if (!LaneProbes.Enabled) return;
        string? path = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_GEOMETRY");
        if (path is null) return;
        Setup setup = Build(1);
        LaneSet lanes = setup.Lanes;
        (int first, int last) = ProfileStations(lanes, setup.Stretch);
        var rows = new List<string> { $"# spacing_m={F(lanes.Spacing)} from_s={F(setup.Stretch.FromS)} to_s={F(setup.Stretch.ToS)} entry_mps={F(setup.EntrySpeed)}",
            "station,s_m,x,y,normal_x,normal_y,half_width_m,free_m" };
        for (int i = first; i <= last; i++)
        {
            Vector2 c = lanes.Centre(i), n = lanes.Normal(i);
            rows.Add($"{i},{F(i * lanes.Spacing)},{F(c.X)},{F(c.Y)},{F(n.X)},{F(n.Y)},{F(lanes.HalfWidth(i))},{F(lanes.Offset(LaneId.Free, i))}");
        }
        File.WriteAllLines(path, rows);
        _output.WriteLine($"{last - first + 1} stations -> {path}");
    }

    /// <summary>
    /// For a learner that sets every station: STINTEGY_SWEEP_JOBS rows of
    /// id, k0..k4, road_grip, front_temp_c, rear_temp_c, front_wear, rear_wear,
    /// entry_share, then the corner factors at every station Geometry lists,
    /// then the braking factors, then the traction factors.
    /// </summary>
    [Fact]
    public void BatchPoints()
    {
        if (!LaneProbes.Enabled) return;
        string? jobs = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_JOBS");
        if (jobs is null) return;
        Setup setup = Build(1);
        Condition learned = setup.Conditions[0];
        (int first, int last) = ProfileStations(setup.Lanes, setup.Stretch);
        int count = last - first + 1;
        string[] lines = File.ReadAllLines(jobs).Skip(1).Where(l => l.Length > 0).ToArray();
        var rows = new string[lines.Length];
        Parallel.For(0, lines.Length, new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount }, i =>
        {
            float[] v = lines[i].Split(',').Select(x => float.Parse(x, CultureInfo.InvariantCulture)).ToArray();
            if (v.Length != 12 + 3 * count)
                throw new InvalidDataException($"job {i}: {v.Length} values, wanted {12 + 3 * count}");
            Condition c = learned with
            {
                RoadGrip = v[6], FrontTempC = v[7], RearTempC = v[8], FrontWear = v[9], RearWear = v[10],
                EntrySpeed = setup.EntrySpeed * v[11]
            };
            LaneSet drawn = setup.Lanes.WithOffsets(LaneId.Free, Draw(setup.Lanes, setup.Stretch, v[1..6]));
            LaneTrackFile file = setup.Shipped.WithLanes(drawn);
            for (int level = 0; level < LaneTrackFile.GripLevels.Length; level++)
            {
                float[] corner = file.Factors(LaneId.Free, level), brake = file.BrakeFactors(LaneId.Free, level),
                    traction = file.TractionFactors(LaneId.Free, level);
                for (int k = 0; k < count; k++)
                {
                    int at = setup.Lanes.Wrap(first + k);
                    corner[at] = v[12 + k];
                    brake[at] = v[12 + count + k];
                    traction[at] = v[12 + 2 * count + k];
                }
            }
            float[] steer = file.Steer(LaneId.Free);
            for (int k = 0; k < count; k++)
                steer[setup.Lanes.Wrap(first + k)] = 0f;
            Run r = Drive(setup.Track, setup.Shipped, file, drawn, setup.Stretch, c, setup.Compounds);
            rows[i] = $"{(int)v[0]},{r.Row()}";
        });
        File.WriteAllLines(Path.ChangeExtension(jobs, ".driven.csv"), new[] { "id," + Run.Header }.Concat(rows));
        _output.WriteLine($"{rows.Length} jobs driven");
    }

    /// <summary>
    /// Corner, braking and traction factors through the stretch from the
    /// braking zone before it, each a straight line between evenly spaced
    /// knots, the same at every grip level; the steering learned for the old
    /// line forgotten there.
    /// </summary>
    private static void SetProfile(LaneTrackFile file, LaneSet lanes, Stretch stretch, float[] profile)
    {
        float from = stretch.FromS - 150f, to = stretch.ToS;
        int first = (int)MathF.Floor(lanes.StationOf(from));
        int last = (int)MathF.Ceiling(lanes.StationOf(to));
        float At(int kind, int i)
        {
            float u = Math.Clamp((i * lanes.Spacing - from) / (to - from), 0f, 1f) * (ProfileKnots - 1);
            int k = Math.Min((int)u, ProfileKnots - 2);
            float t = u - k;
            return profile[kind * ProfileKnots + k] * (1f - t) + profile[kind * ProfileKnots + k + 1] * t;
        }
        for (int level = 0; level < LaneTrackFile.GripLevels.Length; level++)
        {
            float[] corner = file.Factors(LaneId.Free, level), brake = file.BrakeFactors(LaneId.Free, level),
                traction = file.TractionFactors(LaneId.Free, level);
            for (int i = first; i <= last; i++)
            {
                corner[lanes.Wrap(i)] = At(0, i);
                brake[lanes.Wrap(i)] = At(1, i);
                traction[lanes.Wrap(i)] = At(2, i);
            }
        }
        float[] steer = file.Steer(LaneId.Free);
        for (int i = first; i <= last; i++)
            steer[lanes.Wrap(i)] = 0f;
    }

    /// <summary>
    /// Why lines go wrong: STINTEGY_SWEEP_LINES random lines on one flat
    /// profile (every factor STINTEGY_CLASSIFY_FACTOR), the car as a
    /// learning lap has it. For each that goes wrong, the half second before
    /// the car first strays 0.6 m off its line: was it over the planned
    /// speed, was the wheel wanted past what the axles give, was it braking.
    /// </summary>
    [Fact]
    public void Classify()
    {
        if (!LaneProbes.Enabled) return;
        int count = (int)Env("STINTEGY_SWEEP_LINES", 300);
        float factor = Env("STINTEGY_CLASSIFY_FACTOR", 1f);
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "classify.csv");
        Setup setup = Build(count);
        Condition learned = setup.Conditions[0];
        float[] flat = Enumerable.Repeat(factor, 3 * ProfileKnots).ToArray();
        var rows = new string[count];
        var traces = new System.Collections.Concurrent.ConcurrentDictionary<int, List<string>>();
        Parallel.For(1, count, n =>
        {
            LaneSet drawn = setup.Lanes.WithOffsets(LaneId.Free, Draw(setup.Lanes, setup.Stretch, setup.Knots[n]));
            LaneTrackFile file = setup.Shipped.WithLanes(drawn);
            SetProfile(file, setup.Lanes, setup.Stretch, flat);
            var trace = new List<(float S, float Err, float Over, bool Bound, float Accel, float Speed, float Plan, float Slip, float Lat, GripUse G, float Wanted, float Cap, float Yaw, float YawRef, float LongAccel, float Here, float Ahead)>();
            float startS = setup.Stretch.StartS;
            Run r = Drive(setup.Track, setup.Shipped, file, drawn, setup.Stretch, learned, setup.Compounds, (car, d, travelled) =>
            {
                LaneDriverTelemetry t = d.LastTelemetry;
                float s = startS + travelled;
                float plan = d.PlannedSpeedAt(car.Progress.CurrentS);
                bool bound = t.CurvatureCap != 0f && MathF.Sign(t.WantedCurvature) == MathF.Sign(t.CurvatureCap) &&
                             MathF.Abs(t.WantedCurvature) > MathF.Abs(t.CurvatureCap) * LaneLapRunner.CapOverreach;
                trace.Add((s, t.LateralErrorMeters, car.State.Speed - plan, bound, t.CommandAccel, car.State.Speed, plan,
                    car.State.SideslipAngleRadians, car.State.Telemetry.ActualLateralAccel, ((ICarGripFeel)car.Car).GripUse(car.State),
                    t.WantedCurvature, t.CurvatureCap, car.State.YawRateRadiansPerSecond, car.State.Telemetry.ReferenceYawRateRadiansPerSecond,
                    car.State.Telemetry.ActualLongitudinalAccel, LineCurvature(drawn, car.Progress.CurrentS), t.FeedforwardCurvature));
            });
            string cls = "clean";
            float at = -1f, over = 0f, boundShare = 0f, accel = 0f;
            if (!r.Clean)
            {
                int k = trace.FindIndex(x => x.S >= setup.Stretch.FromS && MathF.Abs(x.Err) > 0.6f);
                if (k < 0) k = trace.Count - 1;
                var window = trace.Skip(Math.Max(0, k - 30)).Take(Math.Min(30, k) + 1).ToList();
                at = trace[k].S;
                over = window.Max(x => x.Over);
                boundShare = window.Count(x => x.Bound) / (float)window.Count;
                accel = window.Average(x => x.Accel);
                cls = over > 1.0f ? "over speed" : boundShare > 0.3f ? "wheel past the axles" : "neither";
            }
            traces[n] = trace.Where(x => x.S >= setup.Stretch.FromS - 100f && x.S <= setup.Stretch.ToS).Select(x => string.Join(",", new float[]
                { n, r.Clean ? 1 : 0, x.S, x.Err, x.Over, x.Bound ? 1 : 0, x.Accel, x.Speed, x.Plan, x.Slip, x.Lat,
                  x.G.FrontGripAccel, x.G.RearGripAccel, x.G.FrontLateral, x.G.RearLateral, x.G.FrontLongitudinal, x.G.RearLongitudinal,
                  x.Wanted, x.Cap, x.Yaw, x.YawRef, x.LongAccel, x.Here, x.Ahead }
                .Select(v => v.ToString("F4", CultureInfo.InvariantCulture)))).ToList();
            rows[n] = $"{n},{(r.Spun ? "spun" : r.Clean ? "clean" : "trouble")},{cls},{F(at)},{F(over)},{F(boundShare)},{F(accel)},{F(r.WorstError)},{F(r.Seconds)}";
        });
        File.WriteAllLines(output, new[] { "line,outcome,class,first_stray_s,over_plan_mps,wheel_bound_share,mean_accel,worst_error_m,seconds" }.Concat(rows.Skip(1)));
        File.WriteAllLines(Path.ChangeExtension(output, ".traces.csv"),
            new[] { "line,clean,s_m,error_m,over_plan_mps,wheel_bound,command_accel,speed,plan,sideslip,lat_accel,front_grip,rear_grip,front_lat,rear_lat,front_long,rear_long,wanted_k,cap_k,yaw,yaw_ref,long_accel,line_k_here,feedforward_k" }.Concat(traces.Values.SelectMany(t => t)));
        foreach (var g in rows.Skip(1).Select(r => r.Split(',')[2]).GroupBy(x => x))
            _output.WriteLine($"{g.Key,-22} {g.Count()}");
    }

    /// <summary>A whole lap on a side lane, as LaneDriverTests drives it, with every moment off the road and the felt lag then.</summary>
    [Fact]
    public void SideLaneOff()
    {
        if (!LaneProbes.Enabled) return;
        float side = Env("STINTEGY_SIDE", -1f);
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        foreach (bool feel in new[] { false, true })
        {
            var notes = new List<string>();
            float lastS = -100f;
            LaneLapResult r = LaneLapRunner.Run(track, file, new LaneDriverGains { FeelTurnLag = feel }, onStep: (car, d, t) =>
            {
                d.TargetRange = 1f;
                d.TargetSide = side;
                LaneDriverTelemetry tm = d.LastTelemetry;
                if (car.Progress.Region != TrackRegion.RacingSurface && car.Progress.CurrentS - lastS > 20f)
                {
                    lastS = car.Progress.CurrentS;
                    notes.Add($"  off at {car.Progress.CurrentS:F0} m, t {t:F1}, speed {car.State.Speed:F1}, error {tm.LateralErrorMeters:F2}, braking {-tm.CommandAccel:F1}, felt {string.Join("/", d.FeltExtraLags.Select(x => x.ToString("F3")))}");
                }
            });
            _output.WriteLine($"feel {feel}: {r}");
            notes.ForEach(_output.WriteLine);
        }
    }

    /// <summary>
    /// Every step of every line between STINTEGY_TIMELINE_FROM and _TO on
    /// one flat factor: the line's curvature under the car, what the hands
    /// read ahead, what they asked for after the feedback and the cap, the
    /// wheel's angle and the path the car actually made, so where the car
    /// falls behind the line can be read off in time.
    /// </summary>
    [Fact]
    public void Timeline()
    {
        if (!LaneProbes.Enabled) return;
        int count = (int)Env("STINTEGY_SWEEP_LINES", 300);
        float factor = Env("STINTEGY_CLASSIFY_FACTOR", 1f);
        float from = Env("STINTEGY_TIMELINE_FROM", 3600f), to = Env("STINTEGY_TIMELINE_TO", 3760f);
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "timeline.csv");
        Setup setup = Build(count);
        Condition learned = setup.Conditions[0];
        float[] flat = Enumerable.Repeat(factor, 3 * ProfileKnots).ToArray();
        var lines = new List<string>[count];
        Parallel.For(1, count, n =>
        {
            LaneSet drawn = setup.Lanes.WithOffsets(LaneId.Free, Draw(setup.Lanes, setup.Stretch, setup.Knots[n]));
            LaneTrackFile file = setup.Shipped.WithLanes(drawn);
            SetProfile(file, setup.Lanes, setup.Stretch, flat);
            var rows = new List<string>();
            int step = 0;
            Run r = Drive(setup.Track, setup.Shipped, file, drawn, setup.Stretch, learned, setup.Compounds, (car, d, travelled) =>
            {
                step++;
                float s = car.Progress.CurrentS;
                if (s < from || s > to) return;
                LaneDriverTelemetry t = d.LastTelemetry;
                rows.Add(string.Join(",", new float[]
                {
                    n, step / 60f, s, t.LateralErrorMeters, car.State.Speed, t.CommandAccel, LineCurvature(drawn, s), t.FeedforwardCurvature,
                    t.WantedCurvature, t.CommandCurvature, t.CurvatureCap, car.State.SteerAngleRadians, car.State.Telemetry.ActualCurvature,
                    car.State.Telemetry.ActualLateralAccel, car.State.Telemetry.ActualLongitudinalAccel,
                    d.FeltExtraLags[0], d.FeltExtraLags[1], d.FeltExtraLags[2], d.FeltExtraLags[3]
                }.Select(v => v.ToString("F5", CultureInfo.InvariantCulture))));
            });
            rows.Insert(0, $"#{n},{(r.Clean ? 1 : 0)}");
            lines[n] = rows;
        });
        File.WriteAllLines(output, new[] { "line,t,s,error,speed,command_accel,line_k,feedforward_k,wanted_k,command_k,cap_k,steer,actual_k,lat,long,felt0,felt10,felt20,felt30" }
            .Concat(lines.Skip(1).SelectMany(l => l)));
    }

    /// <summary>
    /// Worked out from the tyre's own rule, not driven: in a steady corner
    /// under the brakes, the angle the steering estimate asks for against the
    /// angle the tyres need once braking has taken its share of each axle's
    /// grip, and how much of the corner the estimate's angle gets the car.
    /// </summary>
    [Fact]
    public void SteerShortfall()
    {
        if (!LaneProbes.Enabled) return;
        CarConfig config = StockCar.Config;
        var car = new SingleTrackCar(config, ((ICarTires)StockCar.Model).Compounds);
        float frontBrake = config.FixedFrontBrakeShare ?? 0.5f;
        // The tyre's own curve, internal to Core: reached the long way round, for a probe.
        var inverse = typeof(SteeringEstimate).Assembly.GetType("StintegyEVO.Core.Cars.TireSlipCurve")!
            .GetMethod("InverseEvaluate", new[] { typeof(float), typeof(float) })!;
        float Inverse(float share, float peak = 1f) => (float)inverse.Invoke(null, new object[] { share, peak })!;
        _output.WriteLine("speed braking asked | front long use, rear long use | wheel asked deg, wheel needed deg | corner got share (steady) | at turn-in from straight");
        foreach (float speed in new[] { 40f, 60f, 85f })
            foreach (float braking in new[] { 0f, 10f, 20f, 30f })
                foreach (float asked in new[] { 15f, 25f, 32f })
                {
                    var state = new CarState { Speed = speed };
                    ((ICarStores)car).Fill(state, PowertrainState.Filled(0.8f));
                    ((ICarTires)car).InstallFreshTires(state);
                    ((ICarTires)car).WarmTires(state, 90f, 90f);
                    state.FilteredLongitudinalAccel = -braking;
                    state.FilteredLateralAccel = asked;
                    SteeringEstimate est = ((ICarEnvelope)car).EstimateSteering(state, RoadAttitude.Flat);
                    float l = est.WheelBaseMeters, front = est.RearArmMeters / l, rear = est.FrontArmMeters / l;
                    float uf = Math.Clamp(frontBrake * braking / est.FrontGripAccel, 0f, 0.99f);
                    float ur = Math.Clamp((1f - frontBrake) * braking / est.RearGripAccel, 0f, 0.99f);
                    float cf = MathF.Sqrt(1f - uf * uf), cr = MathF.Sqrt(1f - ur * ur);
                    // The slip angles the tyres need for a pull, with braking's share taken out, as the physics has it.
                    float Needed(float pull) =>
                        Inverse(Math.Min(front * pull / (est.FrontGripAccel * cf), 0.999f), est.FrontPeakSlipAngleRatio) -
                        Inverse(Math.Min(rear * pull / (est.RearGripAccel * cr), 0.999f));
                    float Got(float wheel)
                    {
                        float lo = 0f, hi = asked * 1.5f;
                        for (int i = 0; i < 40; i++)
                        {
                            float mid = 0.5f * (lo + hi);
                            if (MathF.Atan(l * mid / (speed * speed)) + Needed(mid) < wheel) lo = mid; else hi = mid;
                        }
                        return 0.5f * (lo + hi);
                    }
                    float curvature = asked / (speed * speed);
                    float kinematic = MathF.Atan(l * curvature);
                    float estimated = Inverse(front * asked / est.FrontGripAccel, est.FrontPeakSlipAngleRatio) -
                                      Inverse(rear * asked / est.RearGripAccel);
                    float wheelAsked = kinematic + estimated;
                    float wheelNeeded = kinematic + Needed(asked);
                    _output.WriteLine($"{speed,3:F0} {braking,4:F0} {asked,4:F0} | {uf:F2} {ur:F2} | {wheelAsked * 57.2958f:F2} {wheelNeeded * 57.2958f:F2} | {Got(wheelAsked) / asked:F2} | {Got(kinematic) / asked:F2}");
                }
    }

    /// <summary>
    /// The car's turn-in delay measured the way TurnLag measures it (a car
    /// in a settled corner asked for a little more, the area between its
    /// path coming round and an instant answer), but from other starts:
    /// from straight as well as from a corner, asked for a little more or a
    /// lot more, coasting or braking.
    /// </summary>
    [Fact]
    public void TurnDelay()
    {
        if (!LaneProbes.Enabled) return;
        const float dt = 1f / 60f;
        float At(float speed, float settledAccel, float stepAccel, float braking)
        {
            ICar car = new SingleTrackCar(StockCar.Config, ((ICarTires)StockCar.Model).Compounds);
            var state = new CarState { Position = Vector2.Zero, Heading = 0f, Speed = speed };
            (car as ICarStores)?.Fill(state, PowertrainState.Filled(0.8f));
            var tyres = (ICarTires)car;
            var envelope = (ICarEnvelope)car;
            tyres.InstallFreshTires(state);
            tyres.WarmTires(state, 90f, 90f);
            int settle = 90, window = 60;
            var path = new float[window];
            float heading = 0f, start = 0f;
            for (int i = 0; i < settle + window; i++)
            {
                bool stepped = i >= settle;
                float v = MathF.Max(state.Speed, 5f);
                float wanted = (settledAccel + (stepped ? stepAccel : 0f)) / (v * v);
                var input = new CarPhysicsStepInput(
                    new DriverInput(envelope.EstimateSteering(state, RoadAttitude.Flat).AngleFor(wanted), i >= settle - 18 ? braking : 0f),
                    CarStrategy.Default, 25f);
                Vector2 from = state.Position;
                car.Step(state, input, dt);
                Vector2 move = state.Position - from;
                float length = move.Length();
                if (length < 1e-4f) return float.NaN;
                float now = MathF.Atan2(move.Y, move.X);
                float curvature = MathHelper.NormalizeAngle(now - heading) / length;
                heading = now;
                // As sideways pull, so a car slowing under the brakes is not read as turning tighter.
                float pull = curvature * MathF.Max(state.Speed, 5f) * MathF.Max(state.Speed, 5f);
                if (stepped) path[i - settle] = pull; else start = pull;
            }
            float end = 0f;
            for (int k = window / 2; k < window; k++) end += path[k];
            end /= window - window / 2;
            float taken = end - start;
            if (MathF.Abs(taken) < 0.1f * stepAccel) return float.NaN;
            float behind = 0f;
            for (int k = 0; k < window / 2; k++)
                behind += (1f - Math.Clamp((path[k] - start) / taken, 0f, 1f)) * dt;
            return behind;
        }
        var rows = new List<string> { "speed,settled_accel,step_accel,braking,delay_s" };
        foreach (float speed in new[] { 30f, 45f, 60f })
            foreach ((float settled, float step) in new[] { (12f, 2f), (0f, 2f), (0f, 10f), (0f, 20f), (12f, 15f) })
                foreach (float braking in new[] { 0f, -15f })
                {
                    float delay = At(speed, settled, step, braking);
                    rows.Add($"{speed:F0},{settled:F0},{step:F0},{braking:F0},{F(delay)}");
                    _output.WriteLine(rows[^1]);
                }
        File.WriteAllLines(Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "turndelay.csv"), rows);
    }

    /// <summary>
    /// Where the plan asks too much: STINTEGY_SWEEP_LINES random lines on
    /// every factor 1.0, then again with one window of the stretch (40 m at a
    /// time) at 0.95, its corner factor alone, its braking alone, or both.
    /// The window whose easing takes the trouble away is where it comes from.
    /// </summary>
    [Fact]
    public void Ablate()
    {
        if (!LaneProbes.Enabled) return;
        int count = (int)Env("STINTEGY_SWEEP_LINES", 300);
        float width = Env("STINTEGY_ABLATE_WIDTH", 40f);
        float low = Env("STINTEGY_ABLATE_FACTOR", 0.95f);
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "ablate.csv");
        Setup setup = Build(count);
        Condition learned = setup.Conditions[0];
        float[] flat = Enumerable.Repeat(1f, 3 * ProfileKnots).ToArray();
        float from = Env("STINTEGY_ABLATE_FROM", setup.Stretch.FromS), to = Env("STINTEGY_ABLATE_TO", setup.Stretch.ToS);
        var cases = new List<(string Name, float A, float B, bool Corner, bool Brake)> { ("all 1.00", 0f, 0f, false, false), ("all " + F(low), -1e9f, 1e9f, true, true) };
        for (float a = from; a < to; a += width)
            foreach ((string kind, bool c, bool b) in new[] { ("corner", true, false), ("brake", false, true), ("both", true, true) })
                cases.Add(($"{kind} {a:F0}-{a + width:F0}", a, a + width, c, b));
        var rows = new List<string> { "case,from_s,to_s,trouble,spins,first_stray_median_s,clean_mean_s" };
        foreach (var k in cases)
        {
            var runs = new Run[count];
            Parallel.For(1, count, n =>
            {
                LaneSet drawn = setup.Lanes.WithOffsets(LaneId.Free, Draw(setup.Lanes, setup.Stretch, setup.Knots[n]));
                LaneTrackFile file = setup.Shipped.WithLanes(drawn);
                SetProfile(file, setup.Lanes, setup.Stretch, flat);
                int a = (int)MathF.Ceiling(setup.Lanes.StationOf(Math.Max(k.A, setup.Stretch.FromS - 150f)));
                int b = (int)MathF.Floor(setup.Lanes.StationOf(Math.Min(k.B, setup.Stretch.ToS)));
                for (int level = 0; level < LaneTrackFile.GripLevels.Length; level++)
                    for (int i = a; i <= b && k.A < k.B; i++)
                    {
                        if (k.Corner) file.Factors(LaneId.Free, level)[setup.Lanes.Wrap(i)] = low;
                        if (k.Brake) file.BrakeFactors(LaneId.Free, level)[setup.Lanes.Wrap(i)] = low;
                    }
                runs[n] = Drive(setup.Track, setup.Shipped, file, drawn, setup.Stretch, learned, setup.Compounds);
            });
            Run[] driven = runs.Skip(1).ToArray();
            float[] strays = driven.Where(r => !r.Clean && !float.IsNaN(r.StrayS)).Select(r => r.StrayS).OrderBy(x => x).ToArray();
            rows.Add(string.Join(",", k.Name, F(k.A), F(k.B), driven.Count(r => !r.Clean), driven.Count(r => r.Spun),
                strays.Length > 0 ? F(strays[strays.Length / 2]) : "", F(driven.Where(r => r.Clean).Select(r => r.Seconds).DefaultIfEmpty(float.NaN).Average())));
            _output.WriteLine(rows[^1]);
        }
        File.WriteAllLines(output, rows);
    }

    /// <summary>
    /// The lap learner's way of learning, every station of the stretch, but
    /// each round driven on a batch of random lines instead of one: a station
    /// any of them strayed near gives a little, one they all passed cleanly
    /// takes a little more. What it keeps is a table every line that passes
    /// can drive. Ends with an exam on fresh lines beside a flat 0.95 and 1.0.
    /// </summary>
    [Fact]
    public void Robust()
    {
        if (!LaneProbes.Enabled) return;
        int rounds = (int)Env("STINTEGY_ROBUST_ROUNDS", 80);
        int batch = (int)Env("STINTEGY_ROBUST_BATCH", 100);
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "robust.csv");
        var learner = new LaneLearner();
        Setup setup = Build(1);
        Condition learned = setup.Conditions[0];
        (int first, int last) = ProfileStations(setup.Lanes, setup.Stretch);
        int count = last - first + 1;
        float spacing = setup.Lanes.Spacing;
        bool levels = Environment.GetEnvironmentVariable("STINTEGY_ROBUST_LEVELS") == "1";
        var random = new Random(4242);
        float[][] Lines(int n) => Enumerable.Range(0, n).Select(_ => Enumerable.Range(0, 5).Select(_ => (float)(random.NextDouble() * 2.0 - 1.0)).ToArray()).ToArray();

        Table[] Same(float[] c, float[] b, float[] t) => LaneTrackFile.GripLevels.Select(_ => new Table(c, b, t)).ToArray();
        Run[] DriveAll(float[][] knots, Table[] tables, Func<int, Condition> conditionOf, List<string>[]? traces = null)
        {
            var runs = new Run[knots.Length];
            Parallel.For(0, knots.Length, new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount }, i =>
            {
                LaneSet drawn = setup.Lanes.WithOffsets(LaneId.Free, Draw(setup.Lanes, setup.Stretch, knots[i]));
                LaneTrackFile file = setup.Shipped.WithLanes(drawn);
                for (int level = 0; level < LaneTrackFile.GripLevels.Length; level++)
                {
                    float[] fc = file.Factors(LaneId.Free, level), fb = file.BrakeFactors(LaneId.Free, level), ft = file.TractionFactors(LaneId.Free, level);
                    for (int k = 0; k < count; k++)
                    {
                        int at = setup.Lanes.Wrap(first + k);
                        fc[at] = tables[level].Corner[k]; fb[at] = tables[level].Brake[k]; ft[at] = tables[level].Traction[k];
                    }
                }
                float[] steer = file.Steer(LaneId.Free);
                for (int k = 0; k < count; k++)
                    steer[setup.Lanes.Wrap(first + k)] = 0f;
                List<string>? trace = traces is null ? null : traces[i] = new List<string>();
                float startS = setup.Stretch.StartS;
                runs[i] = Drive(setup.Track, setup.Shipped, file, drawn, setup.Stretch, conditionOf(i), setup.Compounds, trace is null ? null : (car, d, travelled) =>
                {
                    float s = startS + travelled;
                    if (s < setup.Stretch.FromS - 100f || s > setup.Stretch.ToS) return;
                    LaneDriverTelemetry tm = d.LastTelemetry;
                    int k = Math.Clamp((int)MathF.Round(s / spacing) - first, 0, count - 1);
                    GripUse g = ((ICarGripFeel)car.Car).GripUse(car.State);
                    float grip = d.Grip;
                    trace.Add(string.Join(",", new float[]
                    {
                        i, s, tm.LateralErrorMeters, car.State.Speed, d.PlannedSpeedAt(car.Progress.CurrentS), tm.CommandAccel,
                        tm.WantedCurvature, tm.CurvatureCap, tm.FeedforwardCurvature, LineCurvature(drawn, car.Progress.CurrentS),
                        car.State.SideslipAngleRadians, car.State.Telemetry.ActualLateralAccel, car.State.Telemetry.ActualLongitudinalAccel,
                        g.FrontGripAccel, g.RearGripAccel, g.FrontLateral, g.RearLateral, grip
                    }.Select(v => v.ToString("F4", CultureInfo.InvariantCulture))));
                });
            });
            return runs;
        }

        static float[] Smooth(float[] x, int half)
        {
            var y = new float[x.Length];
            for (int i = 0; i < x.Length; i++)
            {
                int a = Math.Max(0, i - half), z = Math.Min(x.Length - 1, i + half);
                float sum = 0f;
                for (int j = a; j <= z; j++) sum += x[j];
                y[i] = sum / (z - a + 1);
            }
            return y;
        }

        // The lap learner's rules, over a batch: trouble at a station (off
        // the line, sideways, off the road) charges the 150 m before it and
        // the 20 m after to whatever the driver was doing there, braking to
        // braking, throttle to traction, the rest to corner; a station no line
        // was in trouble near and every line passed cleanly takes a little
        // more. Each round's lessons come from a fresh batch of lines, and the
        // candidate is driven on that same batch and taken 200 m sector by
        // sector the way the lap learner takes it.
        int back = (int)MathF.Ceiling(learner.LookBackMeters / spacing), ahead = (int)MathF.Ceiling(learner.LookAheadMeters / spacing);
        int length = Math.Max(1, (int)MathF.Round(learner.SectorMeters / spacing));
        int sectors = (count + length - 1) / length;
        float[][] check = Lines((int)Env("STINTEGY_ROBUST_CHECK", 1000));
        bool Trouble(Run r, int i) =>
            r.StationError[i] > learner.TroubleErrorMeters || r.StationSlip[i] > learner.TroubleSlipRadians || r.StationOff[i];
        // A spin counts as a whole sector in trouble where it happened.
        (int Stations, float Worst) SectorTrouble(Run[] runs, int sector)
        {
            int stations = 0;
            float worst = 0f;
            foreach (Run r in runs)
            {
                for (int k = sector * length; k < Math.Min(count, (sector + 1) * length); k++)
                {
                    int i = setup.Lanes.Wrap(first + k);
                    if (Trouble(r, i)) stations++;
                    worst = MathF.Max(worst, r.StationError[i]);
                }
                if (r.Spun && (int)MathF.Round(setup.Lanes.StationOf(r.SpinS)) - first is int at && at >= sector * length && at < (sector + 1) * length)
                    stations += length;
            }
            return (stations, worst);
        }
        float SectorSeconds(Run[] runs, int sector)
        {
            float seconds = 0f;
            foreach (Run r in runs)
                for (int k = sector * length; k < Math.Min(count, (sector + 1) * length); k++)
                    seconds += r.StationSeconds[setup.Lanes.Wrap(first + k)];
            return seconds;
        }
        (int Trouble, float Time) Score(Run[] runs) =>
            (runs.Count(r => !r.Clean), runs.Where(r => r.Clean).Select(r => r.Seconds).DefaultIfEmpty(float.PositiveInfinity).Average());

        var troubleLog = new List<string> { "grip,round,line,kind,s_m,worst_error_m,braking,throttle,corner,brake,traction" };
        var log = new List<string> { "grip,round,trouble_before,trouble_after,clean_mean_s_before,step,kept_sectors,corner_mean,brake_mean,traction_mean,corner_min,brake_min,traction_min" };
        Table Learn(Condition cond, string tag)
        {
            var corner = Enumerable.Repeat(0.95f, count).ToArray();
            var brake = Enumerable.Repeat(0.95f, count).ToArray();
            var traction = Enumerable.Repeat(1f, count).ToArray();
            float step = 1f;
            int stale = 0;
            var snapshots = new List<(int Round, Table Table)> { (0, new Table((float[])corner.Clone(), (float[])brake.Clone(), (float[])traction.Clone())) };
            for (int round = 1; round <= rounds; round++)
            {
                if (stale >= learner.Patience)
                {
                    step *= learner.StepDecay;
                    stale = 0;
                }
                // The round's lines, new each round: the lessons are drawn from
                // them, and the candidate is judged on them, as the lap learner
                // learns from a lap and judges on the same lap.
                float[][] lines = Lines(batch);
                Run[] current = DriveAll(lines, Same(corner, brake, traction), _ => cond);
                Run[] runs = current;
                var cs = Enumerable.Repeat(1f, count).ToArray();
                var bs = Enumerable.Repeat(1f, count).ToArray();
                var ts = Enumerable.Repeat(1f, count).ToArray();
                var troubled = new bool[count];
                var unclean = new bool[count];
                foreach (Run r in runs)
                {
                    void Punish(int at, float penalty, bool both)
                    {
                        for (int d = -back; d <= ahead; d++)
                        {
                            int k = at + d - first;
                            if (k < 0 || k >= count) continue;
                            int i = setup.Lanes.Wrap(first + k);
                            troubled[k] = true;
                            bool braking = r.StationBraking[i];
                            bool throttle = r.StationThrottle[i] && !braking;
                            if (both || (!braking && !throttle) || (d >= 0 && !throttle))
                                cs[k] = MathF.Min(cs[k], 1f - penalty);
                            if (both || braking)
                                bs[k] = MathF.Min(bs[k], 1f - penalty);
                            if (both || throttle)
                                ts[k] = MathF.Min(ts[k], 1f - penalty);
                        }
                    }
                    if (r.Spun)
                        Punish((int)MathF.Round(setup.Lanes.StationOf(r.SpinS)), learner.SpinPenalty * step, both: true);
                    for (int k = 0; k < count; k++)
                    {
                        int i = setup.Lanes.Wrap(first + k);
                        if (Trouble(r, i))
                        {
                            bool severe = r.StationError[i] > learner.SevereErrorMeters || r.StationOff[i];
                            Punish(first + k, (severe ? learner.SeverePenalty : learner.Penalty) * step, both: severe);
                        }
                        if (r.StationError[i] >= learner.CleanErrorMeters || r.StationSlip[i] >= learner.CleanSlipRadians)
                            unclean[k] = true;
                    }
                }
                for (int k = 0; k < count; k++)
                {
                    if (troubled[k] || unclean[k]) continue;
                    int i = setup.Lanes.Wrap(first + k);
                    cs[k] = 1f + learner.Reward * step;
                    if (runs.Any(r => r.StationBraking[i])) bs[k] = 1f + learner.Reward * step;
                    if (runs.Any(r => r.StationThrottle[i])) ts[k] = 1f + learner.Reward * step;
                }
                var nc = new float[count];
                var nb = new float[count];
                var nt = new float[count];
                for (int k = 0; k < count; k++)
                {
                    nc[k] = Math.Clamp(corner[k] * cs[k], learner.MinimumFactor, learner.MaximumFactor);
                    nb[k] = Math.Clamp(brake[k] * bs[k], learner.MinimumFactor, learner.MaximumBrakeFactor);
                    nt[k] = Math.Clamp(traction[k] * ts[k], learner.MinimumFactor, learner.MaximumTractionFactor);
                }
                nc = Smooth(nc, learner.SmoothStations);
                nb = Smooth(nb, learner.SmoothStations);
                nt = Smooth(nt, learner.SmoothStations);

                // Sector by sector, as the lap learner takes a candidate: a clean
                // sector stays clean and gets quicker, one in trouble gets less of
                // it, and neither hands the sector after more trouble than it had.
                Run[] tried = DriveAll(lines, Same(nc, nb, nt), _ => cond);
                int kept = 0;
                for (int sector = 0; sector < sectors; sector++)
                {
                    var before = SectorTrouble(current, sector);
                    var after = SectorTrouble(tried, sector);
                    int nextBefore = sector + 1 < sectors ? SectorTrouble(current, sector + 1).Stations : 0;
                    int nextAfter = sector + 1 < sectors ? SectorTrouble(tried, sector + 1).Stations : 0;
                    if (before.Stations == 0 ? nextAfter > nextBefore : after.Stations + nextAfter > before.Stations + nextBefore)
                        continue;
                    bool take = before.Stations == 0
                        ? after.Stations == 0 && SectorSeconds(tried, sector) < SectorSeconds(current, sector)
                        : after.Stations < before.Stations || (after.Stations == before.Stations && after.Worst < before.Worst - 0.05f);
                    if (!take) continue;
                    for (int k = sector * length; k < Math.Min(count, (sector + 1) * length); k++)
                    {
                        corner[k] = nc[k];
                        brake[k] = nb[k];
                        traction[k] = nt[k];
                    }
                    kept++;
                }
                stale = kept == 0 ? stale + 1 : 0;
                if (round % 10 == 0 || round == rounds)
                    snapshots.Add((round, new Table((float[])corner.Clone(), (float[])brake.Clone(), (float[])traction.Clone())));

                (int Trouble, float Time) score = Score(current);
                for (int n = 0; n < current.Length; n++)
                {
                    Run r = current[n];
                    if (r.Clean) continue;
                    int at = Enumerable.Range(0, count).FirstOrDefault(k => r.StationError[setup.Lanes.Wrap(first + k)] > learner.TroubleErrorMeters || r.StationOff[setup.Lanes.Wrap(first + k)], -1);
                    if (r.Spun) at = Math.Clamp((int)MathF.Round(setup.Lanes.StationOf(r.SpinS)) - first, 0, count - 1);
                    if (at < 0) continue;
                    int i = setup.Lanes.Wrap(first + at);
                    troubleLog.Add($"{tag},{round},{n},{(r.Spun ? "spun" : r.Off > 0f ? "off" : "error")},{F((first + at) * spacing)},{F(r.WorstError)},{(r.StationBraking[i] ? 1 : 0)},{(r.StationThrottle[i] ? 1 : 0)},{F(corner[at])},{F(brake[at])},{F(traction[at])}");
                }
                log.Add(tag + "," + string.Join(",", new[] { round, score.Trouble, tried.Count(r => !r.Clean) }.Select(v => v.ToString(CultureInfo.InvariantCulture))
                    .Concat(new[] { score.Time, step }.Select(F)).Append(kept.ToString(CultureInfo.InvariantCulture))
                    .Concat(new[] { corner.Average(), brake.Average(), traction.Average(), corner.Min(), brake.Min(), traction.Min() }.Select(F))));
                Console.WriteLine(log[^1]);
            }
            // The best of the tables it held along the way, on a fixed set of
            // check lines: fewest stations in trouble, then the quickest.
            Table bestTable = snapshots[^1].Table;
            (int Stations, float Time) bestScore = (int.MaxValue, float.PositiveInfinity);
            foreach ((int round, Table table) in snapshots)
            {
                Run[] onCheck = DriveAll(check, Same(table.Corner, table.Brake, table.Traction), _ => cond);
                int stations = Enumerable.Range(0, sectors).Sum(sector => SectorTrouble(onCheck, sector).Stations);
                float time = onCheck.Where(r => r.Clean).Select(r => r.Seconds).DefaultIfEmpty(float.PositiveInfinity).Average();
                log.Add($"{tag},check after round {round},{stations} stations in trouble,{onCheck.Count(r => !r.Clean)} lines,{F(time)}");
                Console.WriteLine(log[^1]);
                if (stations < bestScore.Stations || (stations == bestScore.Stations && time < bestScore.Time))
                {
                    bestScore = (stations, time);
                    bestTable = table;
                }
            }
            return bestTable;
        }

        // One table learned as the lap learner learns it, or one per grip
        // level, each on a road that grippy, the levels read between as the
        // driver reads them.
        Table[] learnedTables = levels
            ? LaneTrackFile.GripLevels.Select(g => Learn(learned with { RoadGrip = g }, F(g))).ToArray()
            : LaneTrackFile.GripLevels.Select(_ => Learn(learned, "as learned")).Take(1).SelectMany(t => Same(t.Corner, t.Brake, t.Traction)).ToArray();

        // The exam: fresh lines, the learned table beside one number for the stretch.
        float[][] exam = Lines(300);
        // On a level-learned table the exam is driven on any day: road grip,
        // tyre temperatures and wear anywhere across what the levels span.
        Condition[] examDays = exam.Select(_ => levels
            ? learned with
            {
                RoadGrip = 0.55f + (float)random.NextDouble() * 0.45f,
                FrontTempC = 70f + (float)random.NextDouble() * 40f, RearTempC = 70f + (float)random.NextDouble() * 40f,
                FrontWear = (float)random.NextDouble() * 0.5f, RearWear = (float)random.NextDouble() * 0.5f
            }
            : learned).ToArray();
        var rows = new List<string> { "table,trouble,spins,clean_mean_s" };
        var perLine = new List<string> { "table,line,k1,k2,k3,k4,k5," + Condition.Header + "," + Run.Header };
        var examTraces = new List<string>[exam.Length];
        float[] Flat(float v) => Enumerable.Repeat(v, count).ToArray();
        foreach ((string name, Table[] tables) in new[]
        {
            ("learned", learnedTables),
            ("flat 0.95", Same(Flat(0.95f), Flat(0.95f), Flat(1f))),
            ("flat 1.00", Same(Flat(1f), Flat(1f), Flat(1f)))
        })
        {
            Run[] runs = DriveAll(exam, tables, i => examDays[i], name == "learned" ? examTraces : null);
            perLine.AddRange(runs.Select((r, i) => $"{name},{i},{string.Join(",", exam[i].Select(F))},{examDays[i].Row()},{r.Row()}"));
            float time = runs.Where(r => r.Clean).Select(r => r.Seconds).DefaultIfEmpty(float.NaN).Average();
            rows.Add($"{name},{runs.Count(r => !r.Clean)},{runs.Count(r => r.Spun)},{F(time)}");
            _output.WriteLine(rows[^1]);
        }
        File.WriteAllLines(output, log.Concat(new[] { "" }).Concat(rows));
        File.WriteAllLines(Path.ChangeExtension(output, ".exam.csv"), perLine);
        File.WriteAllLines(Path.ChangeExtension(output, ".checktrouble.csv"), troubleLog);
        File.WriteAllLines(Path.ChangeExtension(output, ".traces.csv"),
            new[] { "line,s_m,error_m,speed,plan,command_accel,wanted_k,cap_k,feedforward_k,line_k,sideslip,lat_accel,long_accel,front_grip,rear_grip,front_lat,rear_lat,grip_read" }
                .Concat(examTraces.SelectMany(t => t)));
        File.WriteAllLines(Path.ChangeExtension(output, ".table.csv"), new[] { "grip,station,s_m,corner,brake,traction" }
            .Concat(learnedTables.SelectMany((t, level) => Enumerable.Range(0, count)
                .Select(k => $"{F(LaneTrackFile.GripLevels[level])},{first + k},{F((first + k) * spacing)},{F(t.Corner[k])},{F(t.Brake[k])},{F(t.Traction[k])}"))));
    }

    /// <summary>
    /// Where the robust learner's time goes: STINTEGY_SWEEP_LINES lines
    /// prepared (line drawn, file cloned, factors written) and driven, once on
    /// one thread and once on every core.
    /// </summary>
    [Fact]
    public void Timing()
    {
        if (!LaneProbes.Enabled) return;
        int lines = (int)Env("STINTEGY_SWEEP_LINES", 200);
        Setup setup = Build(lines);
        Condition learned = setup.Conditions[0];
        (int first, int last) = ProfileStations(setup.Lanes, setup.Stretch);
        float[] flat = Enumerable.Repeat(0.95f, 3 * ProfileKnots).ToArray();
        double Ms(long ticks) => ticks * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
        foreach (int threads in new[] { 1, Environment.ProcessorCount })
        {
            _placeTicks = _stepTicks = _loopTicks = _steps = _driverTicks = 0;
            long prepare = 0;
            var wall = System.Diagnostics.Stopwatch.StartNew();
            Parallel.For(0, lines, new ParallelOptions { MaxDegreeOfParallelism = threads }, n =>
            {
                long began = System.Diagnostics.Stopwatch.GetTimestamp();
                LaneSet drawn = setup.Lanes.WithOffsets(LaneId.Free, Draw(setup.Lanes, setup.Stretch, setup.Knots[n]));
                LaneTrackFile file = setup.Shipped.WithLanes(drawn);
                SetProfile(file, setup.Lanes, setup.Stretch, flat);
                Interlocked.Add(ref prepare, System.Diagnostics.Stopwatch.GetTimestamp() - began);
                Drive(setup.Track, setup.Shipped, file, drawn, setup.Stretch, learned, setup.Compounds);
            });
            wall.Stop();
            _output.WriteLine($"threads {threads}: {lines} lines in {wall.Elapsed.TotalSeconds:F2} s, {lines / wall.Elapsed.TotalSeconds:F0} lines/s, {_steps / wall.Elapsed.TotalSeconds:F0} steps/s");
            _output.WriteLine($"  per line: prepare {Ms(prepare) / lines:F2} ms, set up the car and race {Ms(_placeTicks) / lines:F2} ms, " +
                              $"steps {Ms(_stepTicks) / lines:F2} ms ({_steps / lines} steps, {Ms(_stepTicks) * 1000 / Math.Max(_steps, 1):F1} us each), " +
                              $"the probe's own bookkeeping {(Ms(_loopTicks - _placeTicks - _stepTicks)) / lines:F2} ms");
            _output.WriteLine($"  of a step: the driver {Ms(_driverTicks) * 1000 / Math.Max(_steps, 1):F1} us, the world {Ms(_stepTicks - _driverTicks) * 1000 / Math.Max(_steps, 1):F1} us");
        }
    }

    /// <summary>A stretch's factors, one per station from the profile's first.</summary>
    private sealed record Table(float[] Corner, float[] Brake, float[] Traction);

    /// <summary>The drawn free line's curvature where the car is.</summary>
    private static float LineCurvature(LaneSet lanes, float s)
    {
        int i = (int)MathF.Round(lanes.StationOf(s));
        return LaneSet.Curvature(lanes.Position(i - 3, lanes.Offset(LaneId.Free, i - 3)), lanes.Position(i, lanes.Offset(LaneId.Free, i)),
            lanes.Position(i + 3, lanes.Offset(LaneId.Free, i + 3)));
    }

    /// <summary>
    /// The free line, moved across the road by a Catmull-Rom curve through
    /// evenly spaced knots, nothing at either end of the stretch. A knot of
    /// +1 puts the line at the left margin there, -1 at the right one.
    /// </summary>
    private static float[] Draw(LaneSet lanes, Stretch stretch, float[] knots)
    {
        var offset = new float[lanes.Count];
        for (int i = 0; i < lanes.Count; i++)
            offset[i] = lanes.Offset(LaneId.Free, i);
        int first = (int)MathF.Ceiling(lanes.StationOf(stretch.FromS));
        int last = (int)MathF.Floor(lanes.StationOf(stretch.ToS));
        int segments = knots.Length + 1;
        float span = stretch.ToS - stretch.FromS;
        for (int i = first; i <= last; i++)
        {
            float u = (i * lanes.Spacing - stretch.FromS) / span * segments;
            int k = Math.Min((int)u, segments - 1);
            float t = u - k;
            float At(int j) => j <= 0 || j > knots.Length ? 0f : knots[j - 1];
            float p0 = At(k - 1), p1 = At(k), p2 = At(k + 1), p3 = At(k + 2);
            float w = 0.5f * (2f * p1 + (-p0 + p2) * t + (2f * p0 - 5f * p1 + 4f * p2 - p3) * t * t + (-p0 + 3f * p1 - 3f * p2 + p3) * t * t * t);
            float room = lanes.HalfWidth(i) - LaneSet.EdgeMarginMeters;
            // Towards a margin from the free line, as far as w asks.
            float free = offset[i];
            float target = w >= 0f ? free + (room - free) * w : free + (room + free) * w;
            offset[i] = Math.Clamp(target, -room, room);
        }
        return offset;
    }

    /// <summary>
    /// What was learned on the free line, taken as it stands: every grip
    /// level's corner factors through the stretch scaled by one share and its
    /// braking factors by another, and the steering learned for the old line
    /// forgotten there.
    /// </summary>
    private static void Prepare(LaneTrackFile file, LaneSet lanes, Stretch stretch, float cornerShare, float brakeShare)
    {
        int first = (int)MathF.Floor(lanes.StationOf(stretch.FromS - 150f));
        int last = (int)MathF.Ceiling(lanes.StationOf(stretch.ToS));
        for (int level = 0; level < LaneTrackFile.GripLevels.Length; level++)
        {
            float[] corner = file.Factors(LaneId.Free, level), brake = file.BrakeFactors(LaneId.Free, level);
            for (int i = first; i <= last; i++)
            {
                corner[lanes.Wrap(i)] *= cornerShare;
                brake[lanes.Wrap(i)] *= brakeShare;
            }
        }
        float[] steer = file.Steer(LaneId.Free);
        for (int i = first; i <= last; i++)
            steer[lanes.Wrap(i)] = 0f;
    }

    private static RaceCar Place(string id, LaneTrackFile file, DriverAbilities abilities, Condition c, string compound, float s, float offset, float heading, float speed, LaneSet lanes)
    {
        int station = (int)MathF.Round(lanes.StationOf(s));
        Vector2 here = lanes.Position(station, lanes.Offset(LaneId.Free, station) + offset);
        Vector2 next = lanes.Position(station + 1, lanes.Offset(LaneId.Free, station + 1) + offset);
        ICar model = new SingleTrackCar(StockCar.Config, ((ICarTires)StockCar.Model).Compounds);
        var driver = new LaneDriver(file, model) { LearnsAsItDrives = false, PlanIntervalSeconds = Env("STINTEGY_PLAN_INTERVAL", 0.1f), Gains = new LaneDriverGains
        {
            CapFromAxles = Environment.GetEnvironmentVariable("STINTEGY_CAP_FROM_AXLES") != "0",
            FeelTurnLag = Environment.GetEnvironmentVariable("STINTEGY_FEEL") != "0",
            CurvaturePreviewSeconds = Environment.GetEnvironmentVariable("STINTEGY_PREVIEW") is { } preview ? float.Parse(preview, CultureInfo.InvariantCulture) : null,
            PursuitSeconds = Env("STINTEGY_PURSUIT", new LaneDriverGains().PursuitSeconds),
            PlanLeadSeconds = Environment.GetEnvironmentVariable("STINTEGY_PLAN_LEAD") is { } planLead ? float.Parse(planLead, CultureInfo.InvariantCulture) : null,
            SteerLeadSeconds = Environment.GetEnvironmentVariable("STINTEGY_STEER_LEAD") is { } steerLead ? float.Parse(steerLead, CultureInfo.InvariantCulture) : null
        } };
        var car = new RaceCar(id, model, new Driver(new DriverProfile(id, abilities), new Timed(driver)),
            new CarState { Position = here, Heading = MathF.Atan2(next.Y - here.Y, next.X - here.X) + heading, Speed = speed })
        {
            Strategy = new CarStrategy(c.TyreMode, c.Power)
        };
        ((ICarTires)model).FitTires(car.State, compound);
        car.State.Chassis().Energy = new PowertrainState(c.Charge, c.Fuel);
        car.State.Chassis().WarmTires(c.FrontTempC, c.FrontTempC);
        TireState[] rear = [car.State.Chassis().RearLeft, car.State.Chassis().RearRight];
        foreach (TireState tyre in rear)
        {
            tyre.SurfaceTempC = c.RearTempC;
            tyre.CoreTempC = c.RearTempC;
        }
        car.State.Chassis().FrontLeft.Wear = c.FrontWear;
        car.State.Chassis().FrontRight.Wear = c.FrontWear;
        foreach (TireState tyre in rear)
            tyre.Wear = c.RearWear;
        return car;
    }

    /// <summary>Where a drive's time goes, summed over every drive since the last reset, in stopwatch ticks.</summary>
    private static long _placeTicks, _stepTicks, _loopTicks, _steps, _driverTicks;

    /// <summary>The driver, timed: how much of a step is the driver thinking rather than the world moving.</summary>
    private sealed class Timed(IDriverController inner) : IDriverController
    {
        public IDriverController Inner => inner;
        public CarNeeds Needs => inner.Needs;
        public void Initialize(in DriverContext context) => inner.Initialize(context);
        public DriverInput GetControl(in DriverContext context, float dt)
        {
            long began = Stopwatch.GetTimestamp();
            DriverInput input = inner.GetControl(context, dt);
            Interlocked.Add(ref _driverTicks, Stopwatch.GetTimestamp() - began);
            return input;
        }
    }

    private static Run Drive(TrackData track, LaneTrackFile shipped, LaneTrackFile file, LaneSet lanes, Stretch stretch, Condition c, string[] compounds,
        Action<RaceCar, LaneDriver, float>? onStep = null)
    {
        long began = System.Diagnostics.Stopwatch.GetTimestamp();
        try
        {
            return DriveTimed(track, shipped, file, lanes, stretch, c, compounds, onStep, began);
        }
        finally
        {
            Interlocked.Add(ref _loopTicks, System.Diagnostics.Stopwatch.GetTimestamp() - began);
        }
    }

    private static Run DriveTimed(TrackData track, LaneTrackFile shipped, LaneTrackFile file, LaneSet lanes, Stretch stretch, Condition c, string[] compounds,
        Action<RaceCar, LaneDriver, float>? onStep, long began)
    {
        var abilities = new DriverAbilities
        {
            Pace = c.Pace, Consistency = c.Consistency, Precision = c.Precision,
            Focus = c.Focus, Recovery = c.Recovery, Reactions = c.Reactions
        };
        string compound = compounds[c.Compound];
        float room = lanes.HalfWidth((int)lanes.StationOf(stretch.StartS)) - LaneSet.EdgeMarginMeters;
        int startStation = (int)MathF.Round(lanes.StationOf(stretch.StartS));
        float free = lanes.Offset(LaneId.Free, startStation);
        float entryOffset = Math.Clamp(free + c.EntryOffset * room, -room, room) - free;
        RaceCar car = Place("sweep", file, abilities, c, compound, stretch.StartS, entryOffset, c.EntryHeading, c.EntrySpeed, lanes);
        var simulation = new RaceSimulation(track, new RaceEnvironment { SurfaceGripScalar = c.RoadGrip, TrackTempC = c.TrackTempC });
        simulation.AddCar(car);
        RaceCar? lead = null;
        if (c.LeadGap > 0f)
        {
            // A car of the same kind and the same driver on the learned free line,
            // just ahead: here only for the air it leaves behind.
            lead = Place("lead", shipped, abilities, c, compound, stretch.StartS + c.LeadGap, 0f, 0f, c.EntrySpeed, lanes);
            simulation.AddCar(lead);
        }

        Interlocked.Add(ref _placeTicks, System.Diagnostics.Stopwatch.GetTimestamp() - began);
        const float dt = 1f / 60f;
        float startDistance = car.Progress.TotalDistance;
        float fromAt = -1f, toAt = -1f, off = 0f, wall = 0f, worstError = 0f, worstSlip = 0f, wake = 0f;
        int wakeSamples = 0;
        bool touched = false;
        float strayS = float.NaN;
        bool strayBraking = false;
        var stationError = new float[lanes.Count];
        var stationSlip = new float[lanes.Count];
        var stationOff = new bool[lanes.Count];
        var stationBraking = new bool[lanes.Count];
        var stationThrottle = new bool[lanes.Count];
        var stationSeconds = new float[lanes.Count];
        var lanesDriver = (LaneDriver)((Timed)car.Driver!.Controller).Inner;
        // With STINTEGY_WARM_LAP=1 the stretch is judged on the car's second
        // time through it, a whole lap on: a driver in a race has been round.
        float warm = Environment.GetEnvironmentVariable("STINTEGY_WARM_LAP") == "1" ? track.LengthMeters : 0f;
        float lead0 = stretch.FromS - stretch.StartS + warm, through = stretch.ToS - stretch.StartS + warm, tail = stretch.EndS - stretch.StartS + warm;
        float previous = 0f;
        for (int i = 0; i < 60 * 240; i++)
        {
            long stepBegan = System.Diagnostics.Stopwatch.GetTimestamp();
            simulation.Step(dt);
            Interlocked.Add(ref _stepTicks, System.Diagnostics.Stopwatch.GetTimestamp() - stepBegan);
            Interlocked.Increment(ref _steps);
            if (car.State.SpinEvents > 0)
                return new Run(float.NaN, true, off, wall, worstError, worstSlip, touched, wakeSamples > 0 ? wake / wakeSamples : 0f)
                {
                    StrayS = float.IsNaN(strayS) ? stretch.StartS + car.Progress.TotalDistance - startDistance - warm : strayS,
                    StrayBraking = float.IsNaN(strayS) ? lanesDriver.LastTelemetry.CommandAccel < -3f : strayBraking,
                    StationError = stationError, StationSlip = stationSlip, StationOff = stationOff,
                    StationBraking = stationBraking, StationThrottle = stationThrottle, StationSeconds = stationSeconds, SpinS = car.Progress.CurrentS
                };
            float travelled = car.Progress.TotalDistance - startDistance;
            float t = (i + 1) * dt;
            {
                LaneDriverTelemetry tel = lanesDriver.LastTelemetry;
                int k = lanes.Wrap((int)MathF.Round(lanes.StationOf(car.Progress.CurrentS)));
                stationError[k] = MathF.Max(stationError[k], MathF.Abs(tel.LateralErrorMeters));
                float sideways = MathF.Atan(car.Car.Spec.RearAxleOffsetMeters * car.State.Telemetry.ActualCurvature);
                stationSlip[k] = MathF.Max(stationSlip[k], MathF.Abs(car.State.SideslipAngleRadians - sideways));
                stationOff[k] |= car.Progress.Region != TrackRegion.RacingSurface;
                stationBraking[k] |= tel.CommandAccel < -3f;
                stationThrottle[k] |= tel.CommandAccel > 1f;
                stationSeconds[k] += dt;
            }
            if (fromAt < 0f && travelled >= lead0)
                fromAt = t - (travelled - lead0) / MathF.Max(travelled - previous, 1e-3f) * dt;
            if (toAt < 0f && travelled >= through)
                toAt = t - (travelled - through) / MathF.Max(travelled - previous, 1e-3f) * dt;
            if (lead is not null && !touched)
            {
                // Overlapping footprints, near enough, in this car's frame.
                Vector2 d = lead.State.Position - car.State.Position;
                float along = MathF.Cos(car.State.Heading) * d.X + MathF.Sin(car.State.Heading) * d.Y;
                float across = -MathF.Sin(car.State.Heading) * d.X + MathF.Cos(car.State.Heading) * d.Y;
                touched = MathF.Abs(along) < car.Collision.LengthMeters && MathF.Abs(across) < car.Collision.WidthMeters;
            }
            if (fromAt >= 0f && toAt < 0f)
            {
                worstError = MathF.Max(worstError, MathF.Abs(lanesDriver.LastTelemetry.LateralErrorMeters));
                if (float.IsNaN(strayS) && (MathF.Abs(lanesDriver.LastTelemetry.LateralErrorMeters) > 0.6f || car.Progress.Region != TrackRegion.RacingSurface))
                {
                    strayS = stretch.StartS + travelled - warm;
                    strayBraking = lanesDriver.LastTelemetry.CommandAccel < -3f;
                }
                float geometric = MathF.Atan(car.Car.Spec.RearAxleOffsetMeters * car.State.Telemetry.ActualCurvature);
                worstSlip = MathF.Max(worstSlip, MathF.Abs(car.State.SideslipAngleRadians - geometric));
                if (car.Progress.Region != TrackRegion.RacingSurface)
                    off += dt;
                wall += car.BoundaryContactSeconds;
                wake += car.State.WakeDownforceLoss;
                wakeSamples++;
            }
            onStep?.Invoke(car, lanesDriver, travelled);
            previous = travelled;
            if (travelled >= tail)
                break;
        }
        return new Run(toAt > 0f ? toAt - fromAt : float.NaN, false, off, wall, worstError, worstSlip, touched, wakeSamples > 0 ? wake / wakeSamples : 0f)
        {
            StrayS = strayS,
            StrayBraking = strayBraking,
            StationError = stationError, StationSlip = stationSlip, StationOff = stationOff,
            StationBraking = stationBraking, StationThrottle = stationThrottle, StationSeconds = stationSeconds
        };
    }
}
