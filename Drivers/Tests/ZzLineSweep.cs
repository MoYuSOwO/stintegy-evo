using System;
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
/// Probe, not a test: random lines through one stretch of road, each driven
/// on the real physics at a ladder of grip shares, to see whether what a line
/// needs can be learned stretch by stretch.
/// </summary>
public sealed class ZzLineSweep
{
    private readonly ITestOutputHelper _output;
    public ZzLineSweep(ITestOutputHelper output) => _output = output;

    private static float Env(string name, float fallback) =>
        float.TryParse(Environment.GetEnvironmentVariable(name), NumberStyles.Float, CultureInfo.InvariantCulture, out float v) ? v : fallback;

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

    private static bool UseAbilities => Environment.GetEnvironmentVariable("STINTEGY_SWEEP_ABILITIES") != "0";
    private static bool ForgetSteer => Environment.GetEnvironmentVariable("STINTEGY_SWEEP_KEEP_STEER") != "1";

    private sealed record Stretch(float StartS, float FromS, float ToS, float EndS);

    /// <summary>The car as it arrives: the road's grip today, tyre wear and heat, and its speed.</summary>
    private sealed record CarCondition(float RoadGrip, float Wear, float TyreTempC, float EntrySpeed);

    private sealed record Run(float Seconds, bool Spun, float Off, float Wall, float WorstError, float WorstSlip)
    {
        public bool Clean => !Spun && Off <= 0f && Wall <= 0f && WorstError <= 1.2f;
    }

    /// <summary>
    /// STINTEGY_SWEEP_LINES lines (line 0 the learned free line itself), each
    /// driven at every share on the ladder; one CSV row per line and share.
    /// </summary>
    [Fact]
    public void Sweep()
    {
        if (!LaneProbes.Enabled) return;
        int lines = (int)Env("STINTEGY_SWEEP_LINES", 50);
        string output = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_OUT") ?? Path.Combine(LaneProbes.Dir, "line-sweep.csv");
        // Brooklands and Luffield: Wellington's braking zone and both corners.
        var stretch = new Stretch(StartS: Env("STINTEGY_SWEEP_START", 1350f), FromS: Env("STINTEGY_SWEEP_FROM", 1650f),
            ToS: Env("STINTEGY_SWEEP_TO", 2550f), EndS: Env("STINTEGY_SWEEP_END", 2700f));
        float[] shares = Enumerable.Range(0, 10).Select(i => 0.70f + 0.06f * i).ToArray();
        const int Knots = 5;

        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile shipped = LaneTrackFile.ForCircuit("silverstone", track);
        LaneSet lanes = shipped.LanesFor(track);

        // The speed the learned free line carries where each run starts.
        float entrySpeed = 0f;
        LaneLapRunner.Run(track, shipped);
        LaneLapResult reference = LaneLapRunner.Run(track, shipped);
        entrySpeed = reference.StationSpeed[lanes.Wrap((int)MathF.Round(lanes.StationOf(stretch.StartS)))];

        var random = new Random(20261028);
        var knots = new float[lines][];
        var states = new CarCondition[lines];
        bool vary = Environment.GetEnvironmentVariable("STINTEGY_SWEEP_STATE") == "1";
        float Between(float a, float b) => a + (b - a) * (float)random.NextDouble();
        for (int n = 0; n < lines; n++)
        {
            knots[n] = new float[Knots];
            states[n] = new CarCondition(1f, 0f, 90f, entrySpeed);
            if (n == 0) continue;
            for (int j = 0; j < Knots; j++)
                knots[n][j] = (float)(random.NextDouble() * 2.0 - 1.0);
            if (vary)
                states[n] = new CarCondition(Between(0.85f, 1f), Between(0f, 0.5f), Between(70f, 110f), entrySpeed * Between(0.85f, 1f));
        }

        var rows = new string[lines][];
        var watch = Stopwatch.StartNew();
        int done = 0;
        Parallel.For(0, lines, new ParallelOptions { MaxDegreeOfParallelism = Environment.ProcessorCount }, n =>
        {
            LaneSet drawn = lanes.WithOffsets(LaneId.Free, Draw(lanes, stretch, knots[n]));
            rows[n] = new string[shares.Length];
            for (int g = 0; g < shares.Length; g++)
            {
                LaneTrackFile file = shipped.WithLanes(drawn);
                Prepare(file, lanes, stretch, shares[g]);
                Run r = Drive(track, file, drawn, stretch, states[n]);
                rows[n][g] = string.Join(",",
                    new[] { n.ToString(CultureInfo.InvariantCulture) }
                    .Concat(knots[n].Select(k => k.ToString("F4", CultureInfo.InvariantCulture)))
                    .Concat(new[] { states[n].RoadGrip, states[n].Wear, states[n].TyreTempC, states[n].EntrySpeed }.Select(v => v.ToString("F4", CultureInfo.InvariantCulture)))
                    .Concat(new[]
                    {
                        shares[g].ToString("F2", CultureInfo.InvariantCulture),
                        r.Seconds.ToString("F4", CultureInfo.InvariantCulture),
                        r.Clean ? "1" : "0",
                        r.Spun ? "1" : "0",
                        r.Off.ToString("F3", CultureInfo.InvariantCulture),
                        r.Wall.ToString("F3", CultureInfo.InvariantCulture),
                        r.WorstError.ToString("F3", CultureInfo.InvariantCulture),
                        r.WorstSlip.ToString("F4", CultureInfo.InvariantCulture)
                    }));
            }
            int now = System.Threading.Interlocked.Increment(ref done);
            if (now % 200 == 0)
                Console.WriteLine($"{now}/{lines} lines, {watch.Elapsed.TotalSeconds:F0} s");
        });

        var csv = new StringBuilder("line," + string.Join(",", Enumerable.Range(0, Knots).Select(j => $"k{j}")) +
            ",road_grip,wear,tyre_temp_c,entry_mps,share,seconds,clean,spun,off_s,wall_s,worst_error_m,worst_slip_rad\n");
        foreach (string[] line in rows)
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
        _output.WriteLine($"{lines} lines x {shares.Length} shares in {watch.Elapsed.TotalSeconds:F0} s, entry {entrySpeed:F1} m/s -> {output}");
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
    /// level's corner and braking factors through the stretch scaled by one
    /// share, and the steering learned for the old line forgotten there.
    /// </summary>
    private static void Prepare(LaneTrackFile file, LaneSet lanes, Stretch stretch, float share)
    {
        int first = (int)MathF.Floor(lanes.StationOf(stretch.FromS - 150f));
        int last = (int)MathF.Ceiling(lanes.StationOf(stretch.ToS));
        for (int level = 0; level < LaneTrackFile.GripLevels.Length; level++)
        {
            float[] corner = file.Factors(LaneId.Free, level), brake = file.BrakeFactors(LaneId.Free, level);
            for (int i = first; i <= last; i++)
            {
                corner[lanes.Wrap(i)] *= share;
                brake[lanes.Wrap(i)] *= share;
            }
        }
        if (!ForgetSteer) return;
        float[] steer = file.Steer(LaneId.Free);
        for (int i = first; i <= last; i++)
            steer[lanes.Wrap(i)] = 0f;
    }

    private static Run Drive(TrackData track, LaneTrackFile file, LaneSet lanes, Stretch stretch, CarCondition condition)
    {
        int station = (int)MathF.Round(lanes.StationOf(stretch.StartS));
        Vector2 here = lanes.Position(station, lanes.Offset(LaneId.Free, station));
        Vector2 next = lanes.Position(station + 1, lanes.Offset(LaneId.Free, station + 1));
        ICar model = new SingleTrackCar(StockCar.Config, DevelopmentTyres.Super);
        var driver = new LaneDriver(file, model) { UseAbilities = UseAbilities, LearnsAsItDrives = false };
        var car = new RaceCar("sweep", model, new Driver(new DriverProfile("sweep", new DriverAbilities()), driver),
            new CarState { Position = here, Heading = MathF.Atan2(next.Y - here.Y, next.X - here.X), Speed = condition.EntrySpeed })
        {
            Strategy = CarStrategy.Default
        };
        car.State.Chassis().Energy = PowertrainState.Filled(0.8f);
        car.State.Chassis().WarmTires(condition.TyreTempC, condition.TyreTempC);
        foreach (TireState tyre in new[] { car.State.Chassis().FrontLeft, car.State.Chassis().FrontRight, car.State.Chassis().RearLeft, car.State.Chassis().RearRight })
            tyre.Wear = condition.Wear;
        var simulation = new RaceSimulation(track, new RaceEnvironment { SurfaceGripScalar = condition.RoadGrip });
        simulation.AddCar(car);

        const float dt = 1f / 60f;
        float startDistance = car.Progress.TotalDistance;
        float fromAt = -1f, toAt = -1f, off = 0f, wall = 0f, worstError = 0f, worstSlip = 0f;
        float lead = stretch.FromS - stretch.StartS, through = stretch.ToS - stretch.StartS, tail = stretch.EndS - stretch.StartS;
        float previous = 0f;
        for (int i = 0; i < 60 * 60; i++)
        {
            simulation.Step(dt);
            if (car.State.SpinEvents > 0)
                return new Run(float.NaN, true, off, wall, worstError, worstSlip);
            float travelled = car.Progress.TotalDistance - startDistance;
            float t = (i + 1) * dt;
            if (fromAt < 0f && travelled >= lead)
                fromAt = t - (travelled - lead) / MathF.Max(travelled - previous, 1e-3f) * dt;
            if (toAt < 0f && travelled >= through)
                toAt = t - (travelled - through) / MathF.Max(travelled - previous, 1e-3f) * dt;
            if (fromAt >= 0f && toAt < 0f)
            {
                worstError = MathF.Max(worstError, MathF.Abs(driver.LastTelemetry.LateralErrorMeters));
                float geometric = MathF.Atan(car.Car.Spec.RearAxleOffsetMeters * car.State.Telemetry.ActualCurvature);
                worstSlip = MathF.Max(worstSlip, MathF.Abs(car.State.SideslipAngleRadians - geometric));
                if (car.Progress.Region != TrackRegion.RacingSurface)
                    off += dt;
                wall += car.BoundaryContactSeconds;
            }
            previous = travelled;
            if (travelled >= tail)
                break;
        }
        return new Run(toAt > 0f ? toAt - fromAt : float.NaN, false, off, wall, worstError, worstSlip);
    }
}
