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
        /// <summary>Kept on the road and on the line; touching the car ahead is counted apart.</summary>
        public bool Clean => !Spun && Off <= 0f && Wall <= 0f && WorstError <= 1.2f;

        public static readonly string Header = "seconds,clean,spun,off_s,wall_s,worst_error_m,worst_slip_rad,touched,wake_mean";

        public string Row() => string.Join(",", new[]
        {
            F(Seconds), Clean ? "1" : "0", Spun ? "1" : "0", F(Off), F(Wall), F(WorstError), F(WorstSlip), Touched ? "1" : "0", F(Wake)
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
        var driver = new LaneDriver(file, model) { LearnsAsItDrives = false };
        var car = new RaceCar(id, model, new Driver(new DriverProfile(id, abilities), driver),
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

    private static Run Drive(TrackData track, LaneTrackFile shipped, LaneTrackFile file, LaneSet lanes, Stretch stretch, Condition c, string[] compounds)
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

        const float dt = 1f / 60f;
        float startDistance = car.Progress.TotalDistance;
        float fromAt = -1f, toAt = -1f, off = 0f, wall = 0f, worstError = 0f, worstSlip = 0f, wake = 0f;
        int wakeSamples = 0;
        bool touched = false;
        var lanesDriver = (LaneDriver)car.Driver!.Controller;
        float lead0 = stretch.FromS - stretch.StartS, through = stretch.ToS - stretch.StartS, tail = stretch.EndS - stretch.StartS;
        float previous = 0f;
        for (int i = 0; i < 60 * 60; i++)
        {
            simulation.Step(dt);
            if (car.State.SpinEvents > 0)
                return new Run(float.NaN, true, off, wall, worstError, worstSlip, touched, wakeSamples > 0 ? wake / wakeSamples : 0f);
            float travelled = car.Progress.TotalDistance - startDistance;
            float t = (i + 1) * dt;
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
                float geometric = MathF.Atan(car.Car.Spec.RearAxleOffsetMeters * car.State.Telemetry.ActualCurvature);
                worstSlip = MathF.Max(worstSlip, MathF.Abs(car.State.SideslipAngleRadians - geometric));
                if (car.Progress.Region != TrackRegion.RacingSurface)
                    off += dt;
                wall += car.BoundaryContactSeconds;
                wake += car.State.WakeDownforceLoss;
                wakeSamples++;
            }
            previous = travelled;
            if (travelled >= tail)
                break;
        }
        return new Run(toAt > 0f ? toAt - fromAt : float.NaN, false, off, wall, worstError, worstSlip, touched, wakeSamples > 0 ? wake / wakeSamples : 0f);
    }
}
