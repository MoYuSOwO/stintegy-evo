using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Mods;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers;
using StintegyEVO.Drivers.Lanes;

// Times LaneDriver.GetControl alone. Each car is seated with a proxy that
// returns whatever the measured driver last decided, so the world advances on
// the real plan while the stopwatch covers only the decision.
static class Program
{
    sealed class Proxy : IDriverController
    {
        public DriverInput Next;
        public CarNeeds Needs => CarNeeds.None;
        public DriverInput GetControl(in DriverContext context, float dt) => Next;
    }

    static readonly string[] Parts = ["shape", "join", "drive", "brake", "rest"];
    static readonly string[] Sections =
    [
        "traffic", "envelope", "attack", "follow", "material", "rest",
        "readGrip+wake", "joint total", "-", "PlanAhead total",
        "before plan", "plan block", "scale+blend", "steer geom", "cap", "tail", "corner limits",
    ];

    static void Main(string[] args)
    {
        string circuitId = args.Length > 0 ? args[0] : "official:silverstone";
        string tablePath = args.Length > 1 ? args[1] : throw new ArgumentException("Give the joint table to plan on.");
        string? geometryPath = args.Length > 2 && args[2] != "-" ? args[2] : null;
        int cars = args.Length > 3 ? int.Parse(args[3]) : 1;
        bool lookahead = args.Length > 4 && args[4] == "look";
        int steps = args.Length > 5 ? int.Parse(args[5]) : 600;
        bool exactBraking = args.Length > 6 && args[6].Contains("exact");
        bool protect = args.Length <= 6 || !args[6].Contains("noprotect");

        var watch = Stopwatch.StartNew();
        TrackData track = GameMods.Registry.Circuits.Get(circuitId).Track;
        Console.WriteLine($"track      {circuitId}  {track.LengthMeters:F0} m   ({watch.ElapsedMilliseconds} ms)");

        LaneTrackFile file = LaneTrackFile.ForCircuit(circuitId, track);
        LaneSet lanes = file.LanesFor(track);
        Console.WriteLine($"lanes      {lanes.Count} stations, {lanes.Spacing:F3} m apart, current={file.IsCurrent}");

        watch.Restart();
        LaneDriver.JointTable table = LaneDriver.LoadJointTable(tablePath, circuitId, geometryPath);
        Console.WriteLine($"table      {watch.ElapsedMilliseconds} ms to load");

        var simulation = new RaceSimulation(track);
        var drivers = new LaneDriver[cars];
        var proxies = new Proxy[cars];
        var raceCars = new RaceCar[cars];
        var profiles = new DriverProfile[cars];
        int start = (int)MathF.Round(lanes.StationOf(track.StartingLineS));
        for (int i = 0; i < cars; i++)
        {
            int station = start + 15 * i;
            Vector2 here = lanes.Position(station, lanes.Offset(LaneId.Free, station));
            Vector2 next = lanes.Position(station + 1, lanes.Offset(LaneId.Free, station + 1));
            ICar model = StockCar.Build(DevelopmentTyres.SuperId);
            var driver = new LaneDriver(file, model)
            {
                JointLookahead = lookahead,
                ApproximateJointBraking = !exactBraking,
                JointProtect = protect,
            };
            driver.UseJointTable(table);
            drivers[i] = driver;
            proxies[i] = new Proxy();
            profiles[i] = new DriverProfile($"bench-{i}", new DriverAbilities());
            var car = new RaceCar($"bench-{i}", model, new Driver(profiles[i], proxies[i]), new CarState
            {
                Position = here,
                Heading = MathF.Atan2(next.Y - here.Y, next.X - here.X),
                Speed = cars > 1 ? 45f - 2f * i : 45f,
            });
            (car.Car as ICarStores)?.Fill(car.State, PowertrainState.Filled(0.8f));
            raceCars[i] = car;
            simulation.AddCar(car);
        }

        // The track view comes from the race itself; the measured drivers are
        // initialized on the same frozen frame they will be asked about.
        var probes = new DriverContext[cars];
        for (int i = 0; i < cars; i++)
        {
            probes[i] = simulation.CaptureFrameContext(raceCars[i]);
            drivers[i].Initialize(in probes[i]);
        }

        const float dt = 1f / 60f;
        // Warm up: JIT, first-use caches, the lanes' own bake.
        for (int i = 0; i < cars; i++)
        {
            var warm = new DriverContext(profiles[i], probes[i].Track, simulation.CaptureFrame(), i)
            { Random = new RandomSource(1) };
            proxies[i].Next = drivers[i].GetControl(in warm, dt);
        }
        simulation.Step(dt);

        var samples = new double[steps];
        var planning = new List<double>();
        var between = new List<double>();
        for (int step = 0; step < steps; step++)
        {
            for (int i = 0; i < cars; i++)
            {
                var context = new DriverContext(profiles[i], probes[i].Track, simulation.CaptureFrame(), i)
                { Random = new RandomSource((ulong)step + 1) };
                int before = drivers[i].JointPlansApplied;
                watch.Restart();
                DriverInput input = drivers[i].GetControl(in context, dt);
                watch.Stop();
                double ms = watch.Elapsed.TotalMilliseconds;
                samples[step] = Math.Max(samples[step], ms);
                (drivers[i].JointPlansApplied > before ? planning : between).Add(ms);
                proxies[i].Next = input;
            }
            simulation.Step(dt);
        }

        Array.Sort(samples);
        double sum = 0;
        foreach (double s in samples) sum += s;
        int plans = Math.Max(1, drivers[0].JointPlansApplied);
        Console.WriteLine();
        Console.WriteLine($"cars={cars} lookahead={lookahead} steps={steps}");
        Console.WriteLine($"  mean   {sum / samples.Length:F3} ms");
        Console.WriteLine($"  p50    {samples[samples.Length / 2]:F3} ms");
        Console.WriteLine($"  p95    {samples[(int)(samples.Length * 0.95)]:F3} ms");
        Console.WriteLine($"  p99    {samples[(int)(samples.Length * 0.99)]:F3} ms");
        Console.WriteLine($"  max    {samples[^1]:F3} ms");
        Console.WriteLine($"  plans  {drivers[0].JointPlansApplied} applied");
        Report("planning steps", planning);
        Report("other steps   ", between);

        Console.WriteLine("  --- PlanAhead and the joint plan ---");
        long[] sectionTicks = drivers[0].JointParts;
        for (int p = 0; p < sectionTicks.Length; p++)
        {
            if (p == 8) continue;
            double ms = sectionTicks[p] * 1000.0 / Stopwatch.Frequency;
            Console.WriteLine($"  {Sections[p],-14} {ms,9:F1} ms total   {ms / plans:F3} ms/plan");
        }
        double laneLoop = (sectionTicks[9] - sectionTicks[6] - sectionTicks[7]) * 1000.0 / Stopwatch.Frequency;
        Console.WriteLine($"  {"lane loop",-14} {laneLoop,9:F1} ms total   {laneLoop / plans:F3} ms/plan");

        Console.WriteLine("  --- inside the search ---");
        long[] ticks = drivers[0].JointTicks, calls = drivers[0].JointCalls;
        for (int p = 0; p < ticks.Length; p++)
        {
            double ms = ticks[p] * 1000.0 / Stopwatch.Frequency;
            Console.WriteLine($"  {Parts[p],-6} {ms,9:F1} ms total   {calls[p],9:N0} calls   {ms / plans:F3} ms/plan");
        }

        long[] counts = drivers[0].JointCounts;
        Console.WriteLine("  --- inside the join ---");
        Console.WriteLine($"  joins        {counts[0],12:N0}   {counts[0] / plans,10:F1} per plan");
        Console.WriteLine($"  rival visits {counts[1],12:N0}   {counts[1] / plans,10:F1} per plan" +
            $"   {(counts[0] == 0 ? 0 : (double)counts[1] / counts[0]):F2} per join");
        Console.WriteLine($"  body tests   {counts[2],12:N0}   {counts[2] / plans,10:F1} per plan" +
            $"   {(counts[1] == 0 ? 0 : (double)counts[2] / counts[1]):F3} per visit");
        Console.WriteLine($"  follow fallback {counts[3],9:N0}   {counts[3] / plans,10:F1} per plan");
    }

    static void Report(string label, List<double> list)
    {
        if (list.Count == 0)
        {
            Console.WriteLine($"  {label}  none");
            return;
        }
        list.Sort();
        double total = 0;
        foreach (double v in list) total += v;
        Console.WriteLine(
            $"  {label}  n={list.Count,4}  mean {total / list.Count,7:F3}  p50 {list[list.Count / 2],7:F3}  " +
            $"p95 {list[(int)(list.Count * 0.95)],7:F3}  max {list[^1],7:F3} ms");
    }
}
