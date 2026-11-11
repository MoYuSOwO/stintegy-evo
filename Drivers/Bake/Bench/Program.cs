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

    static readonly string[] Parts = ["shape", "join", "drive", "brake", "rest", "traffic"];
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
        int cars = args.Length > 3 && args[3] != "consistency" ? int.Parse(args[3]) : 1;
        bool lookahead = args.Length > 4 && args[4] == "look";
        int steps = args.Length > 5 ? int.Parse(args[5]) : 600;
        bool exactBraking = args.Length > 6 && args[6].Contains("exact");
        bool protect = args.Length <= 6 || !args[6].Contains("noprotect");
        float attackBonus = Flag(args, "attack", 1f);
        float blockedSeconds = Flag(args, "blocked", 1f);
        float defenceSeconds = Flag(args, "defence", 1f);
        var watch = Stopwatch.StartNew();
        TrackData track = GameMods.Registry.Circuits.Get(circuitId).Track;
        Console.WriteLine($"track      {circuitId}  {track.LengthMeters:F0} m   ({watch.ElapsedMilliseconds} ms)");

        LaneTrackFile file = LaneTrackFile.ForCircuit(circuitId, track);
        LaneSet lanes = file.LanesFor(track);
        if (args.Length > 3 && args[3] == "consistency")
        {
            ICar probeCar = StockCar.Build(DevelopmentTyres.SuperId);
            foreach (var (exactSpeeds, approximate) in new[] { (false, false), (true, false), (true, true) })
                Console.Write(JointLineBake.Consistency(circuitId, track, lanes, probeCar, tablePath, exactSpeeds, approximate));
            return;
        }
        Console.WriteLine($"lanes      {lanes.Count} stations, {lanes.Spacing:F3} m apart, current={file.IsCurrent}");
        float widest = 0f;
        for (int i = 0; i < lanes.Count; i++)
            widest = MathF.Max(widest, lanes.HalfWidth(i));
        Console.WriteLine($"road       widest half width {widest:F1} m");

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
                AttackBonus = attackBonus,
                BlockedSeconds = blockedSeconds,
                DefenceSeconds = defenceSeconds,
                UseAbilities = !Has(args, "noabil"),
                JointStartOnLine = !Has(args, "measured"),
                JointHoldSeconds = Flag(args, "hold", 0.02f),
                BlendStations = (int)Flag(args, "blend", 0f),
                JointProbe = (Has(args, "bshares") ? LaneDriver.JointProbes.BakeShares : 0) |
                             (Has(args, "bfactor") ? LaneDriver.JointProbes.BakeFactor : 0) |
                             (Has(args, "benv") ? LaneDriver.JointProbes.BakeEnvelope : 0) |
                             (Has(args, "noline") ? LaneDriver.JointProbes.NoLineChange : 0) |
                             (Has(args, "frozen") ? LaneDriver.JointProbes.FrozenLine : 0),
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
        float firstGap = 0f, lastGap = 0f, bestGap = float.PositiveInfinity, maxLateral = 0f, chaseLateral = 0f;
        double ran0 = 0, ran1 = 0;
        StreamWriter? trail = true
            ? new StreamWriter(Path.Combine(Path.GetTempPath(), "bench-trail.csv"))
            : null;
        trail?.WriteLine("step,s0,d0,v0,s1,d1,v1,probeD,probeV,planTarget,curv,rawD,roadK,err,applied,shift,shiftMax,where,made,held,chal1,side1,noplan1,app1,steer1,accel1,speed1,curv1,drawn1,fast1,brake1,block1,timed1,steer0,lat0,over0,want0,up0,low0,ff0,slide0");
        bool passed = false;
        bool applied0 = false, applied1 = false;
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
                bool applied = drivers[i].JointPlansApplied > before;
                if (i == 0)
                    applied0 = applied;
                if (i == 1)
                    applied1 = applied;
                (applied ? planning : between).Add(ms);
                proxies[i].Next = input;
            }
            simulation.Step(dt);
            var frame = simulation.CaptureFrame();
            int at = (int)lanes.StationOf(frame[0].TrackS);
            maxLateral = MathF.Max(maxLateral, MathF.Abs(frame[0].TrackD - lanes.Offset(LaneId.Free, at)));
            ran0 += frame[0].SpeedMetersPerSecond * dt;
            if (cars > 1)
            {
                float gap = MathF.IEEERemainder(frame[0].TrackS - frame[1].TrackS, track.LengthMeters);
                if (step == 0)
                    firstGap = gap;
                lastGap = gap;
                bestGap = MathF.Min(bestGap, gap);
                if (gap > 0f)
                    passed = true;
                ran1 += frame[1].SpeedMetersPerSecond * dt;
                int at1 = (int)lanes.StationOf(frame[1].TrackS);
                chaseLateral = MathF.Max(chaseLateral,
                    MathF.Abs(frame[1].TrackD - lanes.Offset(LaneId.Free, at1)));
            }
            if (trail is not null)
            {
                int probe = at + 10;
                float s1 = cars > 1 ? frame[1].TrackS : 0f;
                float d1 = cars > 1 ? frame[1].TrackD : 0f;
                float v1 = cars > 1 ? frame[1].SpeedMetersPerSecond : 0f;
                trail.WriteLine($"{step},{frame[0].TrackS:F2},{frame[0].TrackD:F3},{frame[0].SpeedMetersPerSecond:F2}," +
                    $"{s1:F2},{d1:F3},{v1:F2}," +
                    $"{drivers[0].DrawnOffset(probe):F3},{drivers[0].DrawnSpeed(probe):F2},{drivers[0].PlannedSpeed(probe):F2},{drivers[0].SteeringCurvature(probe):F6},{drivers[0].RawPlanOffset:F3},{drivers[0].RoadCurvature(at):F6}," +
                    $"{frame[0].TrackD - lanes.Offset(LaneId.Free, at):F3},{(applied0 ? 1 : 0)},{drivers[0].PlanShift.At:F3},{drivers[0].PlanShift.Most:F3},{drivers[0].PlanShiftDetail.Where},{drivers[0].PlanShiftDetail.Made},{drivers[0].PlanShiftDetail.Held}," +
                    (cars > 1
                        ? $"{(drivers[1].DefenceProbe.Challenged ? 1 : 0)},{drivers[1].DefenceProbe.Side},{(drivers[1].DefenceProbe.NoPlan ? 1 : 0)},{(applied1 ? 1 : 0)}," +
                          $"{proxies[1].Next.SteerAngleRadians:F3},{proxies[1].Next.DesiredAccel:F2},{frame[1].SpeedMetersPerSecond:F2},{drivers[1].SteeringCurvature((int)lanes.StationOf(frame[1].TrackS)):F5},{drivers[1].DrawnOffset((int)lanes.StationOf(frame[1].TrackS) + 10):F2}," +
                          $"{drivers[1].RejectProbe.TooFast},{drivers[1].RejectProbe.CannotBrake},{drivers[1].RejectProbe.Blocked},{drivers[1].RejectProbe.Timed}"
                        : "0,0,0,0,0,0,0,0,0,0,0,0,0") +
                    $",{proxies[0].Next.SteerAngleRadians:F4},{frame[0].Motion?.LateralAccel ?? 0f:F3},{frame[0].Motion?.OverLimit ?? 0f:F3},{drivers[0].SteerProbe.Wanted:F5},{drivers[0].SteerProbe.Upper:F5},{drivers[0].SteerProbe.Lower:F5},{drivers[0].SteerProbe.Feedforward:F5},{drivers[0].SteerProbe.Slide:F3}");
            }
        }

        trail?.Dispose();
        Array.Sort(samples);        double sum = 0;
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
        if (cars > 1)
            Console.WriteLine($"  gap    {firstGap,8:F1} m start   {lastGap,8:F1} m end" +
                $"   {bestGap,8:F1} m closest   {maxLateral,5:F1} m lead off line   {chaseLateral,5:F1} m chase off line   {(passed ? "PASSED" : "held")}");
        if (cars > 1)
            Console.WriteLine($"  ran    {ran0,8:F1} m lead   {ran1,8:F1} m chase   {ran0 - ran1,7:F1} m apart");
        Report("planning steps", planning);
        Report("other steps   ", between);

        // What the bake thinks the rest of the lap costs from each lateral
        // position, at the sharpest corner: it must be lowest where the line is.
        int sharpest = 0;
        float most = 0f;
        for (int st = 0; st < lanes.Count; st++)
        {
            float k = MathF.Abs(drivers[0].RoadCurvature(st));
            if (k > most)
                (most, sharpest) = (k, st);
        }
        int cornerRow = drivers[0].RowAt(sharpest);
        float[] places = drivers[0].PlacesAt(cornerRow);
        Console.WriteLine();
        Console.WriteLine($"  --- the bake's value across the road at station {sharpest} " +
            $"(curvature {most:F4}, radius {1f / MathF.Max(1e-6f, most):F0} m), zero heading ---");
        foreach (float at in new[] { 15f, 20f, 25f, 30f })
        {
            float[] profile = drivers[0].ValueProfile(cornerRow, at);
            Console.Write($"  {at,4:F0} m/s ");
            for (int p = 0; p < places.Length; p++)
                Console.Write($"  y {places[p],6:F2} -> {(float.IsFinite(profile[p]) ? profile[p].ToString("F3") : "  inf"),8}");
            Console.WriteLine();
        }


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
        long[] shapeTicks = drivers[0].JointShapeTicks;
        Console.WriteLine("  --- inside a shape ---");
        string[] halves = ["geometry", "fit+join"];
        for (int p = 0; p < shapeTicks.Length; p++)
        {
            double ms = shapeTicks[p] * 1000.0 / Stopwatch.Frequency;
            Console.WriteLine($"  {halves[p],-10} {ms,9:F1} ms total   {ms / plans,9:F3} ms/plan");
        }
        long[] hits = drivers[0].JointGeometryHits;
        Console.WriteLine($"  from file  {hits[0],9:N0} shapes, {hits[1]:N0} joins" +
            $"   of {calls[0]:N0} shapes, {calls[1]:N0} joins");
        long[] loop = drivers[0].JointLoopCounts;
        Console.WriteLine("  --- inside the successor loop ---");
        Console.WriteLine($"  decisions    {loop[0],12:N0}   {loop[0] / plans,10:F1} per plan");
        Console.WriteLine($"  successors   {loop[1],12:N0}   {loop[1] / plans,10:F1} per plan" +
            $"   {(loop[0] == 0 ? 0 : (double)loop[1] / loop[0]):F1} per decision");
        Console.WriteLine($"  slow ways    {loop[2],12:N0}   {loop[2] / plans,10:F1} per plan" +
            $"   {(loop[1] == 0 ? 0 : 100.0 * loop[2] / loop[1]):F1}% of successors");
        Console.WriteLine("  --- inside the join ---");
        Console.WriteLine($"  joins        {counts[0],12:N0}   {counts[0] / plans,10:F1} per plan");
        Console.WriteLine($"  rival visits {counts[1],12:N0}   {counts[1] / plans,10:F1} per plan" +
            $"   {(counts[0] == 0 ? 0 : (double)counts[1] / counts[0]):F2} per join");
        Console.WriteLine($"  body tests   {counts[2],12:N0}   {counts[2] / plans,10:F1} per plan" +
            $"   {(counts[1] == 0 ? 0 : (double)counts[2] / counts[1]):F3} per visit");
        Console.WriteLine($"  follow fallback {counts[3],9:N0}   {counts[3] / plans,10:F1} per plan");
        long[] terms = drivers[0].JointTermCounts;
        Console.WriteLine($"  terms        {terms[0],12:N0}   {terms[0] / plans,10:F1} per plan" +
            $"   {terms[1]:N0} found a rival ({(terms[0] == 0 ? 0 : 100.0 * terms[1] / terms[0]):F1}%)");
    }

    static bool Has(string[] args, string name) =>
        args.Length > 6 && Array.IndexOf(args[6].Split(','), name) >= 0;

    static float Flag(string[] args, string name, float fallback)
    {
        if (args.Length <= 6)
            return fallback;
        foreach (string part in args[6].Split(','))
            if (part.StartsWith(name + "=", StringComparison.Ordinal) &&
                float.TryParse(part[(name.Length + 1)..], out float value))
                return value;
        return fallback;
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
