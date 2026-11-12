using System.Globalization;
using StintegyEVO.Core.Mods;
using StintegyEVO.Drivers.Lanes;
using StintegyEVO.Drivers;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

// Corner lines: a few fixed ways through each corner, each driven with the
// real car, timed to the next corner's claim point.
//   Corners map <official:circuit>      the bends the free line takes, by station

CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

if (args is ["lap", var lapCircuit, ..])
{
    // The driver's own table of where it races at this circuit, read from
    // its notes for the stock car there. Nothing is detected: with no table
    // there is nowhere it races.
    var track = GameMods.Registry.Circuits.Get(lapCircuit).Track;
    LaneTrackFile file = LaneTrackFile.ForCircuit(lapCircuit, track);
    LaneSet lanes = file.LanesFor(track);
    float[] k = CornerMap.FreeCurvature(lanes);
    List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
    var corners = new List<Lap.Corner>();
    for (int i = 0; i < bends.Count; i++)
        corners.Add(new Lap.Corner(i, bends[i], CornerLines.PinsOf(lanes, k, bends, i), 0));
    string? table = file.NotesText?.Invoke("corners.txt");
    Console.WriteLine($"{lapCircuit}: {corners.Count} corners, the driver's table {(table is null ? "is not there" : "is there")}");
    foreach (Lap.Corner c in corners)
        Console.WriteLine($"  {c.Index,2}  stations {c.Pins.Entry,5}/{c.Pins.Apex,5}/{c.Pins.Exit,5}  s {c.Pins.Entry * lanes.Spacing,6:F0}  " +
            $"R {1f / MathF.Abs(k[c.Pins.Apex]),6:F0} m  {(k[c.Pins.Apex] > 0f ? "left " : "right")}");
    Console.WriteLine("where it races:");
    foreach (Lap.Complex place in Lap.Read(table))
    {
        float from = corners[place.First].Pins.Entry * lanes.Spacing, to = corners[place.Last].Pins.Exit * lanes.Spacing;
        Console.WriteLine($"  corners {place.First}-{place.Last}  s {from,6:F0}-{to,6:F0}  {(place.NoOvertake ? "no overtake" : "overtake"),-12}  abreast {place.MaxAbreast}");
    }
    return 0;
}

if (args is ["map", var circuit])
{
    var track = GameMods.Registry.Circuits.Get(circuit).Track;
    LaneSet lanes = LaneTrackFile.ForCircuit(circuit, track).LanesFor(track);
    float[] k = CornerMap.FreeCurvature(lanes);
    Console.WriteLine($"{circuit}: {lanes.Count} stations of {lanes.Spacing:F3} m, {lanes.LengthMeters:F0} m");
    foreach (CornerMap.Bend b in CornerMap.Bends(lanes, k))
        Console.WriteLine(
            $"stations {b.First,5}-{b.Last,5}  s {b.First * lanes.Spacing,6:F0}-{b.Last * lanes.Spacing,6:F0}  " +
            $"apex {b.Apex,5} (s {b.Apex * lanes.Spacing,6:F0})  R {1f / MathF.Abs(k[b.Apex]),6:F0} m  " +
            $"{(k[b.Apex] > 0f ? "left " : "right")}  free {lanes.Offset(LaneId.Free, b.Apex),6:F2} of ±{lanes.HalfWidth(b.Apex):F2}");
    return 0;
}

if (args.Length >= 4 && args[0] == "complex")
{
    // Several bends as one: held at the first's turn-in, every apex and the
    // last's track-out, each place read in its own bend's direction; the
    // lattice draws the way between, the car times it.
    string id = args[1];
    int firstBend = int.Parse(args[2]), lastBend = int.Parse(args[3]);
    var track = GameMods.Registry.Circuits.Get(id).Track;
    LaneTrackFile file = LaneTrackFile.ForCircuit(id, track);
    LaneSet lanes = file.LanesFor(track);
    float[] k = CornerMap.FreeCurvature(lanes);
    List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
    var first = CornerLines.PinsOf(lanes, k, bends, firstBend);
    var last = CornerLines.PinsOf(lanes, k, bends, lastBend);
    var sites = new List<(int Station, int Sign, string Name)> { (first.Entry, first.Sign, "in") };
    for (int b = firstBend; b <= lastBend; b++)
    {
        var p = CornerLines.PinsOf(lanes, k, bends, b);
        sites.Add((p.Apex, p.Sign, $"apex{b}"));
    }
    sites.Add((last.Exit, last.Sign, "out"));
    CornerMap.Bend next = bends[(lastBend + 1) % bends.Count];
    int claim = lanes.Wrap(first.Entry - CornerLines.BeforeStations);
    var marks = new List<float> { claim * lanes.Spacing };
    marks.AddRange(sites.Skip(1).Select(s => s.Station * lanes.Spacing));
    marks.Add(next.First * lanes.Spacing);
    Console.WriteLine($"bends {firstBend}-{lastBend}: claim {claim}; held at " + string.Join(", ", sites.Select(s => $"{s.Name} {s.Station}")) + $"; next turns in at {next.First}");
    float[] freeLine = Enumerable.Range(0, lanes.Count).Select(i => lanes.Offset(LaneId.Free, i)).ToArray();
    Console.WriteLine("free line: " + string.Join("", sites.Select(s => CornerLines.PlaceName(lanes, s.Sign, s.Station, freeLine[s.Station]))));

    // Places at every site, the turn-in no nearer the inside than the first
    // apex, the track-out no nearer it than the last.
    var combos = new List<int[]>();
    int n = sites.Count;
    for (int code = 0; code < (int)Math.Pow(3, n); code++)
    {
        var places = new int[n];
        for (int i = 0, c = code; i < n; i++, c /= 3)
            places[i] = c % 3;
        if (places[0] >= places[1] && places[n - 1] >= places[n - 2])
            combos.Add(places);
    }
    var claimRun = CornerLines.Drive(track, file, lanes, freeLine, 1f, marks.ToArray());
    float claimSpeed = claimRun.Speeds[0];
    int span = lanes.Wrap(last.Exit + CornerLines.AfterStations - claim);
    int rows = (span + 4) / 5;
    var car = StockCar.Build(DevelopmentTyres.SuperId);
    var clock = System.Diagnostics.Stopwatch.StartNew();
    var drawn = new float[combos.Count][];
    Parallel.For(0, combos.Count, c =>
    {
        var held = sites.Select((s, i) => (s.Station, CornerLines.PlaceOffset(lanes, s.Sign, s.Station, combos[c][i]))).ToList();
        var way = JointLineBake.PinnedLine(track, lanes, car, claim, rows, claimSpeed, held);
        if (way is null)
            return;
        var line = (float[])freeLine.Clone();
        for (int j = 0; j < way.Value.Offset.Length; j++)
            line[lanes.Wrap(claim + j)] = way.Value.Offset[j];
        drawn[c] = line;
    });
    Console.WriteLine($"{combos.Count} combinations, {drawn.Count(d => d is not null)} drawn by the lattice in {clock.Elapsed.TotalSeconds:F1}s");
    var timed = new List<(string Code, CornerLines.Run Run)>();
    var lock_ = new object();
    Parallel.For(0, combos.Count, c =>
    {
        if (drawn[c] is null)
            return;
        var r = CornerLines.Drive(track, file, lanes, drawn[c], 1f, marks.ToArray());
        string code = string.Concat(combos[c].Select(p => "内中外"[p]));
        lock (lock_)
            timed.Add((code, r));
    });
    float freeTime = claimRun.Times[^1] - claimRun.Times[0];
    Console.WriteLine($"free line {freeTime:F3}s from the claim point to the next turn-in");
    Console.WriteLine("line      to next   vs free   v at next   off  spins");
    foreach (var (code, r) in timed.OrderBy(t => t.Run.Times[^1] - t.Run.Times[0]))
    {
        float time = r.Times[^1] - r.Times[0];
        Console.WriteLine($"{code,-8} {time,8:F3} {(time - freeTime) * 1000f,+8:F0} ms {r.Speeds[^1],8:F1}  {r.Lap.OffTrackSeconds,5:F2} {r.Lap.Spins,4}");
    }
    return 0;
}

if (args.Length >= 3 && args[0] == "lines")
{
    string id = args[1];
    int index = int.Parse(args[2]);
    float[] grips = args.Length > 3 ? args[3].Split(',').Select(float.Parse).ToArray() : [1f];
    bool parallel = !args.Contains("serial");
    var track = GameMods.Registry.Circuits.Get(id).Track;
    LaneTrackFile file = LaneTrackFile.ForCircuit(id, track);
    LaneSet lanes = file.LanesFor(track);
    float[] k = CornerMap.FreeCurvature(lanes);
    List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
    CornerMap.Bend bend = bends[index];
    CornerMap.Bend next = bends[(index + 1) % bends.Count];
    CornerLines.Pins pins = CornerLines.PinsOf(lanes, k, bends, index);
    int claim = lanes.Wrap(pins.Entry - CornerLines.BeforeStations);
    float[] marks = CornerLines.Marks(lanes, claim, pins, next);
    Console.WriteLine($"bend {index}: turning {bend.First}-{bend.Last}; claim {claim}, entry {pins.Entry}, apex {pins.Apex}, exit {pins.Exit}; next turns in at {next.First}");
    Console.WriteLine($"free line at entry/apex/exit: {CornerLines.PlaceOf(lanes, k, pins, pins.Entry)}{CornerLines.PlaceOf(lanes, k, pins, pins.Apex)}{CornerLines.PlaceOf(lanes, k, pins, pins.Exit)}");

    bool grid = args.Contains("grid");
    float[] freeLine = Enumerable.Range(0, lanes.Count).Select(i => lanes.Offset(LaneId.Free, i)).ToArray();
    var drawClock = System.Diagnostics.Stopwatch.StartNew();
    var shapes = new Dictionary<string, float[]>();
    if (grid)
    {
        // The lattice draws them: entered at the speed the free line is driven at the claim point.
        float claimSpeed = CornerLines.Drive(track, file, lanes, freeLine, 1f, marks).Speeds[0];
        int span = lanes.Wrap(pins.Exit + CornerLines.AfterStations - claim);
        int rows = (span + 4) / 5;
        var car = StockCar.Build(DevelopmentTyres.SuperId);
        var combos = CornerLines.Combinations().ToList();
        var drawn = new float[combos.Count][];
        Parallel.For(0, combos.Count, c =>
        {
            var (e, a, x) = combos[c];
            var held = new List<(int, float)>
            {
                (pins.Entry, CornerLines.PlaceAt(lanes, pins, pins.Entry, e)),
                (pins.Apex, CornerLines.PlaceAt(lanes, pins, pins.Apex, a)),
                (pins.Exit, CornerLines.PlaceAt(lanes, pins, pins.Exit, x))
            };
            var way = JointLineBake.PinnedLine(track, lanes, car, claim, rows, claimSpeed, held);
            if (way is null)
                return;
            var line = (float[])freeLine.Clone();
            for (int j = 0; j < way.Value.Offset.Length; j++)
                line[lanes.Wrap(claim + j)] = way.Value.Offset[j];
            drawn[c] = line;
        });
        for (int c = 0; c < combos.Count; c++)
        {
            string code = CornerLines.Code(combos[c].Item1, combos[c].Item2, combos[c].Item3);
            if (drawn[c] is { } line)
                shapes[code] = line;
            else
                Console.WriteLine($"{code}: the lattice has no way through these places");
        }
    }
    else
        foreach ((int e, int a, int x) in CornerLines.Combinations())
            shapes[CornerLines.Code(e, a, x)] = CornerLines.Draw(lanes, k, pins, e, a, x);
    Console.WriteLine($"{shapes.Count} lines drawn by the {(grid ? "lattice" : "curvature relaxation")} in {drawClock.Elapsed.TotalSeconds:F2}s");

    var jobs = new List<(string Code, float[] Line, float Grip)>();
    foreach (float grip in grips)
    {
        jobs.Add(("free", freeLine, grip));
        foreach (var (code, line) in shapes)
            jobs.Add((code, line, grip));
    }

    var results = new CornerLines.Run[jobs.Count];
    var clock = System.Diagnostics.Stopwatch.StartNew();
    if (parallel)
        Parallel.For(0, jobs.Count, j => results[j] = CornerLines.Drive(track, file, lanes, jobs[j].Line, jobs[j].Grip, marks));
    else
        for (int j = 0; j < jobs.Count; j++)
            results[j] = CornerLines.Drive(track, file, lanes, jobs[j].Line, jobs[j].Grip, marks);
    Console.WriteLine($"{jobs.Count} laps in {clock.Elapsed.TotalSeconds:F1}s");

    string csv = Path.Combine(Path.GetTempPath(), $"corner-lines-{index}.csv");
    using (var w = new StreamWriter(csv))
    {
        w.WriteLine("code,grip,mark,s,t,v");
        for (int j = 0; j < jobs.Count; j++)
            for (int m = 0; m < marks.Length; m++)
                w.WriteLine($"{jobs[j].Code},{jobs[j].Grip},{m},{marks[m]:F1},{results[j].Times[m]:F4},{results[j].Speeds[m]:F3}");
    }

    using (var w = new StreamWriter(Path.Combine(Path.GetTempPath(), $"corner-shapes-{index}.csv")))
    {
        var codes = jobs.Where(j => j.Grip == grips[0]).ToList();
        w.WriteLine("station,half," + string.Join(',', codes.Select(c => c.Code)));
        for (int i = lanes.Wrap(pins.Entry - CornerLines.BeforeStations - 20); i != lanes.Wrap(pins.Exit + CornerLines.AfterStations + 20); i = lanes.Wrap(i + 1))
            w.WriteLine($"{i},{lanes.HalfWidth(i):F2}," + string.Join(',', codes.Select(c => (pins.Sign * -c.Line[i]).ToString("F2"))));
    }

    int exitMark = 2, endMark = marks.Length - 1;
    foreach (float grip in grips)
    {
        Console.WriteLine($"\ngrip {grip}: time from the claim point (station {claim}), seconds; speed m/s");
        Console.WriteLine("line   to exit   v exit   to next   v next    lap      off  spins  worst err    tightest");
        CornerLines.Run? free = null;
        for (int j = 0; j < jobs.Count; j++)
        {
            if (jobs[j].Grip != grip)
                continue;
            CornerLines.Run r = results[j];
            free ??= r;
            var tight = CornerLines.Tightest(lanes, jobs[j].Line, lanes.Wrap(pins.Entry - CornerLines.BeforeStations), lanes.Wrap(pins.Exit + CornerLines.AfterStations));
            float toExit = r.Times[exitMark] - r.Times[0];
            float toNext = r.Times[endMark] - r.Times[0];
            float freeNext = free.Times[endMark] - free.Times[0];
            Console.WriteLine(
                $"{jobs[j].Code,-5} {toExit,8:F3} {r.Speeds[exitMark],8:F2} {toNext,9:F3} {r.Speeds[endMark],8:F2}  " +
                $"{r.Lap.FlyingLapSeconds,7:F3} {r.Lap.OffTrackSeconds,6:F2} {r.Lap.Spins,5}  {r.Lap.WorstErrorMeters,5:F2}@{r.Lap.WorstErrorS:F0}" +
                $"  R {tight.Radius,5:F0}@{tight.At,-5}  {(toNext - freeNext) * 1000f,+7:F0} ms vs free");
        }
    }
    Console.WriteLine($"\ntrail: {csv}");
    return 0;
}

if (args is ["solo", var soloCircuit, var soloAdvantages])
{
    var soloTrack = GameMods.Registry.Circuits.Get(soloCircuit).Track;
    LaneTrackFile soloFile = LaneTrackFile.ForCircuit(soloCircuit, soloTrack);
    foreach (float a in soloAdvantages.Split(',').Select(float.Parse))
    {
        LaneLapResult lap = LaneLapRunner.Run(soloTrack, soloFile, model: Duel.CarWithGrip(a), compound: DevelopmentTyres.SuperId);
        Console.WriteLine($"grip +{a * 100f:F0}%: {lap}");
    }
    return 0;
}

if (args.Length >= 2 && args[0] == "lap")
{
    // The whole lap drawn by the lattice with nothing held, from a straight
    // back to it, then driven by the car beside the shipped free line.
    string id = args[1];
    var track = GameMods.Registry.Circuits.Get(id).Track;
    LaneTrackFile file = LaneTrackFile.ForCircuit(id, track);
    LaneSet lanes = file.LanesFor(track);
    float[] k = CornerMap.FreeCurvature(lanes);
    List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
    LaneLapResult solo = LaneLapRunner.Run(track, file);
    int from = lanes.Wrap(bends[0].First - 60);
    int rows = (lanes.Count - 1) / 5;
    var clock = System.Diagnostics.Stopwatch.StartNew();
    var way = JointLineBake.PinnedLine(track, lanes, StockCar.Build(DevelopmentTyres.SuperId), from, rows, solo.StationSpeed[from], []);
    if (way is null)
    {
        Console.WriteLine("no lattice way round");
        return 1;
    }
    float[] line = Enumerable.Range(0, lanes.Count).Select(i => lanes.Offset(LaneId.Free, i)).ToArray();
    for (int j = 0; j < way.Value.Offset.Length; j++)
        line[lanes.Wrap(from + j)] = way.Value.Offset[j];
    Console.WriteLine($"lattice lap from station {from}: {rows} rows in {clock.Elapsed.TotalSeconds:F1}s; the lattice's own time {way.Value.Seconds:F3}s");
    LaneLapResult drawn = LaneLapRunner.Run(track, file.WithLanes(lanes.WithOffsets(LaneId.Free, line)), flyingLaps: 2);
    Console.WriteLine($"shipped free line: {solo}");
    Console.WriteLine($"lattice lap:       {drawn}  laps {string.Join(", ", drawn.LapTimes.Select(x => x.ToString("F3")))}");
    string csv = Path.Combine(Path.GetTempPath(), "lattice-lap.csv");
    using (var w = new StreamWriter(csv))
    {
        w.WriteLine("station,free,lattice,half,v_free,v_lattice");
        for (int i = 0; i < lanes.Count; i++)
            w.WriteLine($"{i},{lanes.Offset(LaneId.Free, i):F3},{line[i]:F3},{lanes.HalfWidth(i):F2},{solo.StationSpeed[i]:F2},{drawn.StationSpeed[i]:F2}");
    }
    Console.WriteLine($"trail: {csv}");
    return 0;
}
if (args.Length >= 2 && args[0] == "dive")
{
    // A dive into Brooklands from the Wellington Straight, alone: the car
    // held to the inside from before the braking to a late turn-in, then
    // the lattice's fastest way through the apex and out. Driven at the
    // braking the speed plan gives it, and later and harder.
    string id = args[1];
    var track = GameMods.Registry.Circuits.Get(id).Track;
    LaneTrackFile file = LaneTrackFile.ForCircuit(id, track);
    LaneSet lanes = file.LanesFor(track);
    float[] k = CornerMap.FreeCurvature(lanes);
    List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
    int corner = args.Length > 2 ? int.Parse(args[2]) : 4;
    LaneLapResult solo = LaneLapRunner.Run(track, file);
    CornerLines.Pins pins = CornerLines.PinsOf(lanes, k, bends, corner);
    CornerLines.Pins before = CornerLines.PinsOf(lanes, k, bends, corner - 1);
    int braking = before.Exit;
    for (int i = before.Exit; i != bends[corner].First; i = lanes.Wrap(i + 1))
        if (solo.StationSpeed[i] > solo.StationSpeed[braking])
            braking = i;
    int claim = lanes.Wrap(Math.Min(pins.Entry, braking) - CornerLines.BeforeStations);
    int span = lanes.Wrap(pins.Exit + CornerLines.AfterStations - claim);
    float[] freeLine = Enumerable.Range(0, lanes.Count).Select(i => lanes.Offset(LaneId.Free, i)).ToArray();
    var car = StockCar.Build(DevelopmentTyres.SuperId);
    int next = bends[(corner + 1) % bends.Count].Apex;
    Console.WriteLine($"bend {corner}: free line brakes from {braking} at {solo.StationSpeed[braking]:F1} m/s; entry {pins.Entry}, apex {pins.Apex}, exit {pins.Exit}; claim {claim}; timed to the next apex {next}");

    float[]? Draw(int turnIn, int exitPlace)
    {
        var held = new List<(int, float)>();
        if (turnIn >= 0)
            for (int st = lanes.Wrap(braking - 20); lanes.Wrap(turnIn - st) < lanes.Count / 2 && st != turnIn; st = lanes.Wrap(st + 5))
                held.Add((st, CornerLines.PlaceOffset(lanes, pins.Sign, st, 0)));
        else
            held.Add((pins.Entry, CornerLines.PlaceOffset(lanes, pins.Sign, pins.Entry, 0)));
        held.Add((pins.Apex, CornerLines.PlaceOffset(lanes, pins.Sign, pins.Apex, 0)));
        held.Add((pins.Exit, CornerLines.PlaceOffset(lanes, pins.Sign, pins.Exit, exitPlace)));
        var way = JointLineBake.PinnedLine(track, lanes, car, claim, (span + 4) / 5, solo.StationSpeed[claim], held);
        if (way is null)
            return null;
        var line = (float[])freeLine.Clone();
        for (int j = 0; j < way.Value.Offset.Length; j++)
            line[lanes.Wrap(claim + j)] = way.Value.Offset[j];
        return line;
    }

    var shapes = new List<(string Name, float[]? Line)> { ("free line", freeLine) };
    var wanted = new List<(string Name, int TurnIn, int Exit)>();
    foreach (int x in new[] { 1, 2 })
    {
        wanted.Add(($"内内{"内中外"[x]} (one entry pin)", -1, x));
        foreach (int metres in new[] { 60, 40, 20 })
            wanted.Add(($"V {metres} m before apex, out {"内中外"[x]}", lanes.Wrap(pins.Apex - metres / 2), x));
    }
    var drawn = new float[]?[wanted.Count];
    Parallel.For(0, wanted.Count, w => drawn[w] = Draw(wanted[w].TurnIn, wanted[w].Exit));
    for (int w = 0; w < wanted.Count; w++)
        shapes.Add((wanted[w].Name, drawn[w]));

    float[] factors = [1f, 1.1f, 1.2f];
    var jobs = new List<(string Name, float[] Line, float Factor)>();
    foreach (var (name, line) in shapes)
    {
        if (line is null)
        {
            Console.WriteLine($"{name}: no lattice way");
            continue;
        }
        foreach (float f in factors)
            jobs.Add((name, line, f));
    }
    var results = new Dive.Result[jobs.Count];
    Parallel.For(0, jobs.Count, j => results[j] = Dive.Run(track, file, lanes, jobs[j].Line, jobs[j].Factor, claim, pins, next));
    Dive.Result baseline = results[0];
    Console.WriteLine("line                             brake x   brakes from     v there   v slowest   to next    vs free    wide   off  spins");
    for (int j = 0; j < jobs.Count; j++)
    {
        Dive.Result r = results[j];
        float shift = lanes.Wrap(r.BrakeFrom - baseline.BrakeFrom + lanes.Count / 2) - lanes.Count / 2;
        Console.WriteLine($"{jobs[j].Name,-32} {jobs[j].Factor,6:F1}   {r.BrakeFrom,5} ({shift * lanes.Spacing,+5:F0} m) {r.BrakeSpeed,8:F1} {r.Slowest,10:F1} {r.Seconds,9:F3} {(r.Seconds - baseline.Seconds) * 1000f,+8:F0} ms {r.Wide,6:F2} {r.Off,5:F2} {r.Spins,4}");
    }
    return 0;
}

if (args.Length >= 2 && args[0] == "battle")
{
    // Wellington Straight into Brooklands, two cars by the rules agreed. On
    // the straight a car holds its place across the road or changes it once,
    // quickly. The chaser sits in the tow on the leader's path and pulls out
    // once, about two seconds before the turn-in, onto its corner line. As
    // the leader reaches the braking point the apex is given the way the
    // FIA's driving standards guidelines give it, on where the two will be
    // at the apex: the car without it takes a line that leaves the other
    // room, or tucks in behind.
    //
    // The decisions are the cars': the leader reads the chaser's best attack
    // against its outside line, from the gap and the straight's gain table,
    // and covers the inside when that would earn the apex within the margin
    // its nerve allows. The chaser reads the leader's rears a tier off the
    // truth and attacks on its best line when that earns the apex by the
    // margin its nerve asks for. Both read tables baked offline.
    string id = args[1];
    var track = GameMods.Registry.Circuits.Get(id).Track;
    LaneTrackFile file = LaneTrackFile.ForCircuit(id, track);
    LaneSet lanes = file.LanesFor(track);
    float[] k = CornerMap.FreeCurvature(lanes);
    List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
    LaneLapResult solo = LaneLapRunner.Run(track, file);
    CornerLines.Pins loop = CornerLines.PinsOf(lanes, k, bends, 3);
    CornerLines.Pins bk = CornerLines.PinsOf(lanes, k, bends, 4);
    int braking = loop.Exit;
    for (int i = loop.Exit; i != bends[4].First; i = lanes.Wrap(i + 1))
        if (solo.StationSpeed[i] > solo.StationSpeed[braking])
            braking = i;
    int claim = lanes.Wrap(bk.Entry - CornerLines.BeforeStations);
    int windowEnd = lanes.Wrap(bk.Exit + CornerLines.AfterStations);
    int luffield = bends[5].Apex;
    int decide = lanes.Wrap(bk.Entry - 5);
    int pullAt = lanes.Wrap(bk.Entry - (int)MathF.Round(Battle.PullSeconds * 85f / lanes.Spacing));
    float[] free = Enumerable.Range(0, lanes.Count).Select(i => lanes.Offset(LaneId.Free, i)).ToArray();
    var car = StockCar.Build(DevelopmentTyres.SuperId);
    int span = lanes.Wrap(windowEnd - claim);

    var clock = System.Diagnostics.Stopwatch.StartNew();
    var lines = new System.Collections.Concurrent.ConcurrentDictionary<string, float[]>();
    Parallel.ForEach(CornerLines.Combinations().ToList(), c =>
    {
        var (e, a, x) = c;
        var held = new List<(int, float)>
        {
            (bk.Entry, CornerLines.PlaceOffset(lanes, bk.Sign, bk.Entry, e)),
            (bk.Apex, CornerLines.PlaceOffset(lanes, bk.Sign, bk.Apex, a)),
            (bk.Exit, CornerLines.PlaceOffset(lanes, bk.Sign, bk.Exit, x))
        };
        var way = JointLineBake.PinnedLine(track, lanes, car, claim, (span + 4) / 5, solo.StationSpeed[claim], held);
        if (way is null)
            return;
        var line = (float[])free.Clone();
        for (int j = 0; j < way.Value.Offset.Length; j++)
            line[lanes.Wrap(claim + j)] = way.Value.Offset[j];
        lines[CornerLines.Code(e, a, x)] = line;
    });
    Console.WriteLine($"Brooklands {bk.Entry}/{bk.Apex}/{bk.Exit}; braking {braking}; apex given at {decide}; {lines.Count} lines in {clock.Elapsed.TotalSeconds:F1}s");

    // Each line driven alone, by the chaser and by the leader at every tier
    // of its rears, timed at every station from the straight's start to Luffield's apex.
    int tableFrom = loop.Exit;
    float[] markS = Enumerable.Range(0, lanes.Wrap(luffield - tableFrom) + 1).Select(j => lanes.Wrap(tableFrom + j) * lanes.Spacing).ToArray();
    var leaderTables = Enumerable.Range(0, Battle.RearTiers.Length)
        .Select(_ => new System.Collections.Concurrent.ConcurrentDictionary<string, CornerLines.Run>()).ToArray();
    var chaserTables = new System.Collections.Concurrent.ConcurrentDictionary<string, CornerLines.Run>();
    var jobs = lines.Keys.SelectMany(code => new (string code, int tier)[] { (code, -1) }.Concat(Enumerable.Range(0, Battle.RearTiers.Length).Select(tier => (code, tier)))).ToList();
    Parallel.ForEach(jobs, job =>
    {
        ICar model = job.tier < 0 || job.tier == 0 ? StockCar.Build(DevelopmentTyres.SuperId) : Duel.CarWithRear(Battle.RearTiers[job.tier]);
        var run = CornerLines.Drive(track, file, lanes, lines[job.code], 1f, markS, model);
        if (job.tier < 0)
            chaserTables[job.code] = run;
        else
            leaderTables[job.tier][job.code] = run;
    });

    // The leader's one move on the straight: out of the Loop it holds the
    // outside place, or covers the inside early, joining an inside line
    // before the turn-in. One such path per line that turns in from there,
    // so it can cover onto whichever of them its rears can drive.
    float[] holdPath = lines[Battle.Hold];
    var coverPaths = new Dictionary<string, float[]>();
    foreach (string code in lines.Keys.Where(c => c[0] == '内'))
    {
        var cover = (float[])lines[code].Clone();
        int c0 = lanes.Wrap(loop.Exit + 40), join = lanes.Wrap(bk.Entry - 10);
        for (int i = c0; i != join; i = lanes.Wrap(i + 1))
        {
            float place = CornerLines.PlaceOffset(lanes, bk.Sign, i, 0);
            cover[i] = free[i] + (place - free[i]) * Battle.Ease(lanes.Wrap(i - c0) / 75f);
        }
        for (int i = join; i != bk.Entry; i = lanes.Wrap(i + 1))
        {
            float place = CornerLines.PlaceOffset(lanes, bk.Sign, i, 0);
            cover[i] = place + (lines[code][i] - place) * Battle.Ease(lanes.Wrap(i - join) / 10f);
        }
        coverPaths[code] = cover;
    }

    var ctx = new Battle.Context
    {
        Track = track, File = file, Lanes = lanes, Solo = solo, Lines = lines, Free = free,
        HoldPath = holdPath, CoverPaths = coverPaths, LeaderTables = leaderTables, ChaserTables = chaserTables, TableFrom = tableFrom, Claim = claim,
        Start = lanes.Wrap(bends[2].First - 60), Release = loop.Apex, PushFrom = loop.Exit, PullAt = pullAt, Braking = braking,
        DefendAt = lanes.Wrap(loop.Exit + 40), Decide = decide, Entry = bk.Entry, Apex = bk.Apex, Exit = bk.Exit, Luffield = luffield, WindowEnd = windowEnd,
        InsideDirection = -bk.Sign,
    };

    // What the straight is worth, baked once: the chaser held 0.3 s behind
    // through Village and the Loop, then its own pace in the tow, with and
    // without a push, behind a leader at each tier of its rears. The gap in
    // seconds at the straight's start and at the pull-out point.
    Console.WriteLine($"straight, chaser's gap in seconds at its start (station {loop.Exit}) and at the pull-out point (station {pullAt}):");
    foreach (int tier in new[] { 0, 2, 3 })
    {
        foreach (bool push in new[] { false, true })
        {
            Battle.Result d = Battle.Run(ctx, new Battle.Policy { Tier = tier, Push = push });
            ctx.Gain[tier, push ? 1 : 0] = d.TimeGap[4] - d.TimeGap[5];
            Console.WriteLine($"  rears {Battle.RearTiers[tier]:P0}, push {push,-5}  {d.TimeGap[4]:F2}s -> {d.TimeGap[5]:F2}s  (closes {ctx.Gain[tier, push ? 1 : 0]:F2}s)");
        }
    }
    ctx.Gain[1, 0] = (ctx.Gain[0, 0] + ctx.Gain[2, 0]) / 2f;
    ctx.Gain[1, 1] = (ctx.Gain[0, 1] + ctx.Gain[2, 1]) / 2f;

    Console.WriteLine($"lines alone, seconds from the braking point to Luffield's apex (leader at each tier / chaser):");
    foreach (string code in lines.Keys.OrderBy(c => c))
        Console.WriteLine($"  {code}  {string.Join(" ", Enumerable.Range(0, Battle.RearTiers.Length).Select(t => ctx.LeaderSeconds(t, code, decide, luffield).ToString("F3")))} / {ctx.ChaserSeconds(code, decide, luffield):F3}");

    // The sweep: the leader's rears, the chaser's push, how much margin each
    // asks for before it moves, and how far off the truth the chaser reads
    // the leader's rears. The leader reads the chaser's push the wrong way
    // round in half of them.
    var cases = new List<Battle.Policy>();
    foreach (int tier in new[] { 0, 2, 3 })
        foreach (bool push in new[] { false, true })
            foreach (float nerve in new[] { 0f, 2f })
                foreach (float cover in new[] { 0f, 3f })
                    foreach (int error in new[] { 0, -1 })
                        foreach (bool misread in new[] { false, true })
                            cases.Add(new Battle.Policy { Tier = tier, Push = push, AttackMargin = nerve, DefendWithin = cover, ReadError = error, MisreadsPush = misread, AutoAttack = true, AutoDefend = true });

    var results = new Battle.Result[cases.Count];
    Parallel.For(0, cases.Count, r => results[r] = Battle.Run(ctx, cases[r]));
    Console.WriteLine("\nrears  push  nerve  cover  reads   leader reads  threat   leader        attack  apex given        gap m at Luffield  closest  result");
    int contacts = 0, spins = 0, passes = 0;
    for (int r = 0; r < cases.Count; r++)
    {
        Battle.Policy p = cases[r];
        Battle.Result d = results[r];
        if (d.Contact) contacts++;
        if (d.Spins > 0) spins++;
        if (d.Gap[3] < 0f) passes++;
        Console.WriteLine($"  {Battle.RearTiers[p.Tier],5:P0}  {p.Push,-5}  {p.AttackMargin,4:F0}m  {p.DefendWithin,4:F0}m  {(p.ReadError == 0 ? "true" : "a tier off"),-11}  {(p.MisreadsPush ? "wrong" : "right"),-5}  " +
            $"{(float.IsNegativeInfinity(d.Threat) ? "    none" : $"{d.Threat,7:F1}m"),7}  {(d.Covered ? "covers" : "holds"),-6}  {d.LeaderLine,-7}  {(d.Attack ?? "follows"),-7}  {d.Decision,-16}  " +
            $"{d.Gap[3],8:F1}   {(float.IsInfinity(d.Closest) ? "   -" : d.Closest.ToString("F2")),6}   " +
            $"{(d.Contact ? "CONTACT " : "")}{(d.Spins > 0 ? $"spins {d.Spins} " : "")}{(d.Gap[3] < 0f ? "PASSED" : "behind")}");
    }
    Console.WriteLine($"{cases.Count} races: {passes} passes, {contacts} with contact, {spins} with a spin");
    return 0;
}

if (args.Length >= 2 && args[0] == "pass")
{
    // Wellington Straight into Brooklands: the leader on worn rears, the
    // chaser close behind on a push, alongside by the braking point. Each
    // takes its own lattice line through Brooklands; who comes out ahead?
    string id = args[1];
    var track = GameMods.Registry.Circuits.Get(id).Track;
    LaneTrackFile file = LaneTrackFile.ForCircuit(id, track);
    LaneSet lanes = file.LanesFor(track);
    float[] k = CornerMap.FreeCurvature(lanes);
    List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
    Duel.LeaderRear = args.FirstOrDefault(a => a.StartsWith("rear=")) is { } rearArg ? float.Parse(rearArg[5..]) : 0.8f;
    string? only = args.FirstOrDefault(a => a.StartsWith("only="))?[5..];
    if (args.Contains("trace"))
        Duel.Trace = (800, 960);
    LaneLapResult solo = LaneLapRunner.Run(track, file);
    CornerLines.Pins loop = CornerLines.PinsOf(lanes, k, bends, 3);
    CornerLines.Pins brooklands = CornerLines.PinsOf(lanes, k, bends, 4);
    int braking = loop.Exit;
    for (int i = loop.Exit; i != bends[4].First; i = lanes.Wrap(i + 1))
        if (solo.StationSpeed[i] > solo.StationSpeed[braking])
            braking = i;
    int claim = lanes.Wrap(brooklands.Entry - CornerLines.BeforeStations);
    int span = lanes.Wrap(brooklands.Exit + CornerLines.AfterStations - claim);
    float[] freeLine = Enumerable.Range(0, lanes.Count).Select(i => lanes.Offset(LaneId.Free, i)).ToArray();
    var car = StockCar.Build(DevelopmentTyres.SuperId);
    int early = lanes.Wrap(loop.Exit + 10);
    float[]? Lattice(int e, int a, int x, int from = -1)
    {
        from = from < 0 ? claim : from;
        int rowsFrom = (lanes.Wrap(brooklands.Exit + CornerLines.AfterStations - from) + 4) / 5;
        var held = new List<(int, float)>
        {
            (brooklands.Entry, CornerLines.PlaceAt(lanes, brooklands, brooklands.Entry, e)),
            (brooklands.Apex, CornerLines.PlaceAt(lanes, brooklands, brooklands.Apex, a)),
            (brooklands.Exit, CornerLines.PlaceAt(lanes, brooklands, brooklands.Exit, x))
        };
        var way = JointLineBake.PinnedLine(track, lanes, car, from, rowsFrom, solo.StationSpeed[from], held);
        if (way is null)
            return null;
        var line = (float[])freeLine.Clone();
        for (int j = 0; j < way.Value.Offset.Length; j++)
            line[lanes.Wrap(from + j)] = way.Value.Offset[j];
        return line;
    }
    var attackers = new (string Code, int E, int A, int X)[] { ("内内中", 0, 0, 1), ("内内外", 0, 0, 2), ("内中外", 0, 1, 2) };
    var defenders = new (string Code, int E, int A, int X)[] { ("外内外", 2, 0, 2), ("外中外", 2, 1, 2), ("外外外", 2, 2, 2) };
    var shapes = new Dictionary<string, float[]?>();
    Parallel.ForEach(attackers.Select(c => ("A" + c.Code, c.E, c.A, c.X, claim)).Concat(attackers.Select(c => ("E" + c.Code, c.E, c.A, c.X, early)))
        .Concat(defenders.Select(c => ("D" + c.Code, c.E, c.A, c.X, claim))), c =>
    {
        float[]? line = Lattice(c.Item2, c.Item3, c.Item4, c.Item5);
        lock (shapes)
            shapes[c.Item1] = line;
    });
    Console.WriteLine($"Brooklands {brooklands.Entry}/{brooklands.Apex}/{brooklands.Exit}, claim {claim}; braking from {braking}; leader rears {Duel.LeaderRear:P0}");
    foreach (var (code, line) in shapes.OrderBy(s => s.Key))
        Console.WriteLine($"  {code}: {(line is null ? "no lattice way" : "drawn")}");

    int[] marks = [bends[2].Apex, loop.Entry, loop.Apex, loop.Exit, braking, brooklands.Apex, brooklands.Exit, bends[5].Apex];
    string[] names = ["Village", "Loop in", "Loop apex", "Aintree out", "braking", "Brooklands apex", "Brooklands out", "Luffield apex"];
    var runs = new List<(string Name, Duel.Attack? Attack)> { ("both on the free line", null) };
    // The free line in the tow, then an eased S across onto the corner line
    // at its turn-in, over a length of straight: drawn as it would be in
    // the race, nothing of the straight baked.
    float Ease(float x) { x = Math.Clamp(x, 0f, 1f); return x * x * (3f - 2f * x); }
    foreach (int metres in new[] { 100, 150, 200, 300 })
        foreach (var a in attackers.Take(2))
            foreach (var d in defenders.Take(2))
            {
                if (shapes["A" + a.Code] is not { } al || shapes["D" + d.Code] is not { } dl)
                    continue;
                var s = (float[])freeLine.Clone();
                int entry = brooklands.Entry, length = metres / 2;
                for (int i = lanes.Wrap(entry - length); i != lanes.Wrap(entry + 1); i = lanes.Wrap(i + 1))
                {
                    float u = 1f - lanes.Wrap(entry - i) / (float)length;
                    s[i] = freeLine[i] + (al[entry] - freeLine[entry]) * Ease(u);
                }
                for (int i = lanes.Wrap(entry + 1); i != lanes.Wrap(brooklands.Exit + CornerLines.AfterStations); i = lanes.Wrap(i + 1))
                    s[i] = al[i];
                LaneSet sLanes = lanes.WithOffsets(LaneId.Free, s);
                runs.Add(($"S {metres} m {a.Code} vs {d.Code}", new Duel.Attack(file.WithLanes(sLanes), sLanes,
                    file.WithLanes(lanes.WithOffsets(LaneId.Free, dl)), 0)));
            }
    foreach (string when in new[] { "A", "E" })
        foreach (var a in attackers)
            foreach (var d in defenders)
            {
                if (shapes[when + a.Code] is not { } al || shapes["D" + d.Code] is not { } dl)
                    continue;
                LaneSet chaserLanes = lanes.WithOffsets(LaneId.Free, al);
                // Across late, out of the tow on its own pull-out; or across from the Aintree exit, the line its own move.
                runs.Add(($"{(when == "A" ? "late" : "early")} {a.Code} vs {d.Code}", new Duel.Attack(file.WithLanes(chaserLanes), chaserLanes,
                    file.WithLanes(lanes.WithOffsets(LaneId.Free, dl)), when == "A" ? -brooklands.Sign : 0)));
            }
    if (only is not null)
        runs = runs.Where((r, i) => i == 0 || only.Split('+').Any(o => r.Name.Contains(o))).ToList();
    var results = new Duel.Result[runs.Count];
    Parallel.For(0, runs.Count, r => results[r] = Duel.Run(track, file, lanes, solo, bends[2].First - 60, marks, braking,
        0.3f, bends[2].Apex, loop.Apex, loop.Exit, 0f, push: true, attack: runs[r].Attack));
    Console.WriteLine("nose-to-tail gap, m (time gap, s); negative is the chaser ahead");
    for (int r = 0; r < runs.Count; r++)
    {
        Duel.Result d = results[r];
        Console.Write($"  {runs[r].Name,-28}");
        for (int m = 4; m < marks.Length; m++)
            Console.Write($"  {names[m]} {d.Gap[m],6:F1} ({d.TimeGap[m]:+0.00;-0.00})");
        Console.WriteLine($"   {(d.Contact ? "CONTACT" : "clean")}; pulled out {d.PulledOutAt}");
        int cut = d.Note.IndexOf(" [");
        Console.WriteLine(cut < 0 ? d.Note : d.Note[..cut]);
    }
    return 0;
}

if (args.Length >= 2 && args[0] == "duel")
{
    string id = args[1];
    float[] advantages = args.Length > 2 ? args[2].Split(',').Select(float.Parse).ToArray() : [0f, 0.03f, 0.06f];
    var track = GameMods.Registry.Circuits.Get(id).Track;
    LaneTrackFile file = LaneTrackFile.ForCircuit(id, track);
    LaneSet lanes = file.LanesFor(track);
    float[] k = CornerMap.FreeCurvature(lanes);
    List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
    string scenario = args.Length > 3 ? args[3] : "hangar";
    if (args.FirstOrDefault(a => a.StartsWith("rear=")) is { } rearArg)
        Duel.LeaderRear = float.Parse(rearArg[5..]);
    if (Duel.LeaderRear != 1f)
    {
        var worn = LaneLapRunner.Run(track, file, model: Duel.CarWithRear(Duel.LeaderRear), compound: DevelopmentTyres.SuperId);
        Console.WriteLine($"leader on rears at {Duel.LeaderRear:P0} of new grip, alone: {worn}");
    }
    LaneLapResult solo = LaneLapRunner.Run(track, file);
    // Hangar: Maggotts, Becketts and Chapel (7-11) as one, out onto the
    // Hangar Straight into Stowe (12); the gap is opened from Becketts'
    // apex and the chaser drives its own pace from Chapel's turn-in.
    // Wellington: Village (2) into the Loop and Aintree (3), out onto the
    // Wellington Straight into Brooklands (4); the gap is opened from
    // Village's apex, its own pace from the Loop's apex.
    bool hangar = scenario == "hangar";
    int corner = hangar ? 11 : 3;
    CornerLines.Pins exit = CornerLines.PinsOf(lanes, k, bends, corner);
    CornerMap.Bend after = bends[corner + 1];
    int braking = exit.Exit;
    for (int i = exit.Exit; i != after.First; i = lanes.Wrap(i + 1))
        if (solo.StationSpeed[i] > solo.StationSpeed[braking])
            braking = i;
    int start = hangar ? bends[7].First - 60 : bends[2].First - 60;
    int backOff = hangar ? bends[9].Apex : bends[2].Apex;
    int release = hangar ? exit.Entry : exit.Apex;
    int[] marks = hangar
        ? [bends[7].Apex, bends[10].Apex, exit.Entry, exit.Apex, exit.Exit, lanes.Wrap(exit.Exit + 100), lanes.Wrap(exit.Exit + 200), braking]
        : [bends[2].Apex, exit.Entry, exit.Apex, exit.Exit, lanes.Wrap(exit.Exit + 100), lanes.Wrap(exit.Exit + 200), braking];
    string[] names = hangar
        ? ["Maggotts", "Becketts", "Chapel in", "Chapel apex", "Chapel out", "+200 m", "+400 m", "braking"]
        : ["Village", "Loop in", "Loop apex", "Aintree out", "+200 m", "+400 m", "braking"];
    Console.WriteLine($"start {start}; back off from {backOff}, own pace from {release}; out {exit.Exit}; braking from {braking} at {solo.StationSpeed[braking]:F1} m/s; solo lap {solo.FlyingLapSeconds:F3}s");

    // Every chaser keeps 0.3 s to the back-off point, then its own gap, then
    // drives its own pace, held back only from running into the leader. Out
    // from behind on the straight only, when it is closing fast enough to
    // reach the leader before the braking.
    var ways = new (string Name, float Seconds, bool Push)[]
    {
        ("tight: 0.3s", 0.3f, false),
        ("back off to 0.5s", 0.5f, false),
        ("back off to 0.8s", 0.8f, false),
        ("tight, push on the straight", 0.3f, true),
        ("back off 0.5s, push on the straight", 0.5f, true),
    };
    float[] freeLine = Enumerable.Range(0, lanes.Count).Select(i => lanes.Offset(LaneId.Free, i)).ToArray();
    float[] markS = marks.Select(m => m * lanes.Spacing).ToArray();
    Console.WriteLine($"alone, seconds from {names[0]} to each mark:");
    foreach (float a in advantages)
    {
        var alone = CornerLines.Drive(track, file, lanes, freeLine, 1f, markS, Duel.CarWithGrip(a));
        Console.WriteLine($"  grip +{a * 100f:F0}%: " + string.Join("  ", Enumerable.Range(0, marks.Length).Select(m => $"{names[m]} {alone.Times[m] - alone.Times[0]:F3}")));
    }
    var runs = new List<(float Advantage, int Way)>();
    foreach (float a in advantages)
        for (int w = 0; w < ways.Length; w++)
            runs.Add((a, w));
    var results = new Duel.Result[runs.Count];
    Parallel.For(0, runs.Count, r =>
        results[r] = Duel.Run(track, file, lanes, solo, start, marks, braking, ways[runs[r].Way].Seconds, backOff, release, exit.Exit, runs[r].Advantage, ways[runs[r].Way].Push));

    for (int r = 0; r < runs.Count; r++)
    {
        if (r == 0 || runs[r].Advantage != runs[r - 1].Advantage)
            Console.WriteLine($"\nchaser grip +{runs[r].Advantage * 100f:F0}%: nose-to-tail gap, m (time gap, s) at each mark");
        Duel.Result d = results[r];
        Console.Write($"  {ways[runs[r].Way].Name,-40}");
        for (int m = 0; m < marks.Length; m++)
            Console.Write($"  {names[m]} {d.Gap[m],6:F1} ({d.TimeGap[m]:F2})");
        Console.WriteLine($"   {(d.Gap[^1] < -2f * 4.8f ? "AHEAD" : d.Gap[^1] < 0f ? "ALONGSIDE" : "behind")} at braking, dv {d.SpeedGain:+0.0;-0.0} m/s; pulled out {(d.PulledOutAt >= 0 ? d.PulledOutAt.ToString() : "no")}; held back {d.HeldSeconds:F1}s, {d.HeldAfter:F2}s after release{(d.Contact ? "; CONTACT" : "")}");
        if (args.Contains("notes")) Console.WriteLine("     " + d.Note);
    }
    return 0;
}

Console.Error.WriteLine("Usage: Corners map <official:circuit>");
Console.Error.WriteLine("       Corners lap <official:circuit>");
Console.Error.WriteLine("       Corners lines <official:circuit> <bend> [grip,grip,...] [serial] [grid]");
return 2;

static class Lap
{
    /// <summary>One corner of the lap: the bend the free line takes, and where its lines are pinned.</summary>
    public sealed record Corner(int Index, CornerMap.Bend Bend, CornerLines.Pins Pins, int Braking);

    /// <summary>Corners raced as one: one move covers them, and a pass is judged at the last of them.</summary>
    public sealed record Complex(int First, int Last, bool NoOvertake, int MaxAbreast);

    /// <summary>
    /// The driver's own table of where it races at a circuit, as it wrote it.
    /// One line a place: the corners it covers, numbered as the map prints
    /// them, then whether a pass may be made there and how many cars may run
    /// abreast: <c>first-last [no] [abreast=N]</c>. Nothing is detected and
    /// nothing is filled in: no text is nowhere it races.
    /// </summary>
    public static List<Complex> Read(string? text)
    {
        var places = new List<Complex>();
        if (text is null)
            return places;
        foreach (string raw in text.Split('\n'))
        {
            string line = raw.Trim();
            if (line.Length == 0 || line.StartsWith('#'))
                continue;
            string[] parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            string[] range = parts[0].Split('-');
            int a = int.Parse(range[0]), b = range.Length == 2 ? int.Parse(range[1]) : a;
            bool no = parts.Contains("no");
            int abreast = parts.FirstOrDefault(p => p.StartsWith("abreast=")) is { } ab ? int.Parse(ab[8..]) : 2;
            places.Add(new Complex(a, b, no, abreast));
        }
        return places.OrderBy(p => p.First).ToList();
    }

    /// <summary>The complex a corner belongs to.</summary>
    public static Complex Of(IReadOnlyList<Complex> complexes, int corner) => complexes.First(c => corner >= c.First && corner <= c.Last);
}

static class CornerLines
{
    /// <summary>How far before the bend's first turning station a car claims its line: 150 m.</summary>
    public const int BeforeStations = 75;

    /// <summary>How far past the bend's last turning station a line may still differ from the free one: 200 m.</summary>
    public const int AfterStations = 100;

    public sealed record Run(LaneLapResult Lap, float[] Times, float[] Speeds);

    /// <summary>Entry, apex and exit, each inside (0), middle (1) or outside (2), none of them nearer the outside at the apex than at either end.</summary>
    public static IEnumerable<(int, int, int)> Combinations()
    {
        for (int a = 0; a < 3; a++)
            for (int e = a; e < 3; e++)
                for (int x = a; x < 3; x++)
                    yield return (e, a, x);
    }

    public static string Code(int e, int a, int x) => $"{"内中外"[e]}{"内中外"[a]}{"内中外"[x]}";

    /// <summary>The checkpoints timed: claim, apex, exit, then every 25 m to where the next bend turns in.</summary>
    public readonly record struct Pins(int Entry, int Apex, int Exit, int Sign);

    /// <summary>
    /// Where the free line turns in and tracks out round a bend: the station
    /// nearest the outside before the apex and after it, no further back
    /// than the last bend's apex nor on past the next one's.
    /// </summary>
    public static Pins PinsOf(LaneSet lanes, float[] k, List<CornerMap.Bend> bends, int index)
    {
        CornerMap.Bend bend = bends[index];
        int sign = MathF.Sign(k[bend.Apex]);
        int before = lanes.Wrap(bend.Apex - bends[(index - 1 + bends.Count) % bends.Count].Apex);
        int after = lanes.Wrap(bends[(index + 1) % bends.Count].Apex - bend.Apex);
        float Out(int i) => sign * lanes.Offset(LaneId.Free, i) / Edge(lanes, i);
        int entry = bend.Apex - 5, exit = bend.Apex + 5;
        for (int j = 5; j < Math.Min(90, before); j++)
            if (Out(bend.Apex - j) > Out(entry) + 1e-3f)
                entry = bend.Apex - j;
        for (int j = 5; j < Math.Min(125, after); j++)
            if (Out(bend.Apex + j) > Out(exit) + 1e-3f)
                exit = bend.Apex + j;
        return new Pins(lanes.Wrap(entry), bend.Apex, lanes.Wrap(exit), sign);
    }

    public static float[] Marks(LaneSet lanes, int claim, Pins pins, CornerMap.Bend next)
    {
        var marks = new List<float> { claim, pins.Apex, pins.Exit };
        int stations = lanes.Wrap(next.First - pins.Exit);
        int every = (int)MathF.Round(25f / lanes.Spacing);
        for (int i = every; i < stations; i += every)
            marks.Add(lanes.Wrap(pins.Exit + i));
        marks.Add(next.First);
        return marks.Select(m => m * lanes.Spacing).ToArray();
    }

    private static float Edge(LaneSet lanes, int station) => lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters;

    /// <summary>Offset of a place at a station of the bend: inside, the road's middle, or outside.</summary>
    /// <summary>Offset of a place in a bend turning <paramref name="sign"/>: inside (0), the road's middle (1), outside (2).</summary>
    public static float PlaceOffset(LaneSet lanes, int sign, int station, int place)
    {
        float inside = -sign * Edge(lanes, station);
        return place switch { 0 => inside, 1 => 0f, _ => -inside };
    }

    public static char PlaceName(LaneSet lanes, int sign, int station, float d)
    {
        float inside = PlaceOffset(lanes, sign, station, 0);
        float t = (d - inside) / (-2f * inside);
        return t < 1f / 3f ? '内' : t < 2f / 3f ? '中' : '外';
    }

    private static float Place(LaneSet lanes, float[] k, Pins pins, int station, int place)
    {
        // A bend turning left (positive curvature) has its inside at negative offsets.
        float inside = -pins.Sign * Edge(lanes, station);
        return place switch { 0 => inside, 1 => 0f, _ => -inside };
    }

    public static float PlaceAt(LaneSet lanes, Pins pins, int station, int place) => Place(lanes, null!, pins, station, place);

    public static char PlaceOf(LaneSet lanes, float[] k, Pins pins, int station)
    {
        float d = lanes.Offset(LaneId.Free, station);
        float inside = Place(lanes, k, pins, station, 0);
        float t = (d - inside) / (-2f * inside); // 0 inside, 1 outside
        return t < 1f / 3f ? '内' : t < 2f / 3f ? '中' : '外';
    }

    /// <summary>
    /// The least-curvature line through the three places, the free line
    /// outside the window round the bend: relaxed as the lanes are, coarse to
    /// fine, with the three held stations on every level's grid.
    /// </summary>
    public static float[] Draw(LaneSet lanes, float[] k, Pins pins, int e, int a, int x)
    {
        int n = lanes.Count;
        int from = pins.Entry - BeforeStations;
        int span = lanes.Wrap(pins.Exit + AfterStations - from);
        var line = Enumerable.Range(0, n).Select(i => lanes.Offset(LaneId.Free, i)).ToArray();
        var held = new Dictionary<int, float>
        {
            [lanes.Wrap(pins.Entry - from)] = Place(lanes, k, pins, pins.Entry, e),
            [lanes.Wrap(pins.Apex - from)] = Place(lanes, k, pins, pins.Apex, a),
            [lanes.Wrap(pins.Exit - from)] = Place(lanes, k, pins, pins.Exit, x)
        };
        float D(int j) => line[lanes.Wrap(from + j)];
        void Set(int j, float d) => line[lanes.Wrap(from + j)] = d;
        System.Numerics.Vector2 P(int j) => lanes.Position(from + j, D(j));

        // Start from straight pieces between the held places, then relax.
        var knots = new List<int> { 0 };
        knots.AddRange(held.Keys.Order());
        knots.Add(span);
        foreach (var (j, d) in held)
            Set(j, d);
        for (int q = 0; q + 1 < knots.Count; q++)
            for (int j = knots[q] + 1; j < knots[q + 1]; j++)
                Set(j, D(knots[q]) + (D(knots[q + 1]) - D(knots[q])) * (j - knots[q]) / (knots[q + 1] - knots[q]));

        for (int step = 16; step >= 1; step /= 2)
        {
            // This level's grid: every step-th station and the held ones, with
            // fixed free-line points a step beyond either end.
            var grid = new List<int> { -2 * step, -step };
            grid.AddRange(Enumerable.Range(0, span + 1).Where(j => j % step == 0 || held.ContainsKey(j) || j == span));
            grid.Add(span + step);
            grid.Add(span + 2 * step);
            for (int pass = 0; pass < 600; pass++)
            {
                for (int m = 3; m < grid.Count - 3; m++)
                {
                    int j = grid[m];
                    if (held.ContainsKey(j))
                        continue;
                    var p0 = P(grid[m - 2]);
                    var p1 = P(grid[m - 1]);
                    var p2 = P(j);
                    var p3 = P(grid[m + 1]);
                    var p4 = P(grid[m + 2]);
                    float kPrev = LaneSet.Curvature(p0, p1, p2);
                    float kNext = LaneSet.Curvature(p2, p3, p4);
                    float lPrev = (p2 - p1).Length(), lNext = (p3 - p2).Length();
                    float target = (lNext * kPrev + lPrev * kNext) / MathF.Max(lPrev + lNext, 1e-4f);
                    float kHere = LaneSet.Curvature(p1, p2, p3);
                    float kProbe = LaneSet.Curvature(p1, p2 + lanes.Normal(from + j) * 0.01f, p3);
                    float slope = (kProbe - kHere) / 0.01f;
                    if (MathF.Abs(slope) < 1e-7f)
                        continue;
                    float edge = Edge(lanes, from + j);
                    Set(j, Math.Clamp(D(j) + (target - kHere) / slope * 0.8f, -edge, edge));
                }
            }
            // Fill between this level's points for the next.
            for (int m = 2; m < grid.Count - 3; m++)
                for (int j = grid[m] + 1; j < grid[m + 1]; j++)
                    Set(j, D(grid[m]) + (D(grid[m + 1]) - D(grid[m])) * (j - grid[m]) / (grid[m + 1] - grid[m]));
        }
        return line;
    }

    /// <summary>The sharpest the line turns between two stations, as a radius in metres, and where.</summary>
    public static (float Radius, int At) Tightest(LaneSet lanes, float[] line, int from, int to)
    {
        float most = 0f;
        int at = from;
        for (int i = from; i != lanes.Wrap(to); i = lanes.Wrap(i + 1))
        {
            float c = MathF.Abs(LaneSet.Curvature(
                lanes.Position(i - 1, line[lanes.Wrap(i - 1)]),
                lanes.Position(i, line[i]),
                lanes.Position(i + 1, line[lanes.Wrap(i + 1)])));
            if (c > most)
            {
                most = c;
                at = i;
            }
        }
        return (1f / MathF.Max(most, 1e-6f), at);
    }

    public static Run Drive(StintegyEVO.Core.Track.TrackData track, LaneTrackFile file, LaneSet lanes, float[] line, float grip, float[] marks, ICar? model = null)
    {
        var times = new float[marks.Length];
        var speeds = new float[marks.Length];
        var seen = new int[marks.Length];
        float length = track.LengthMeters;
        float previousS = float.NaN, previousT = 0f;
        LaneLapResult lap = LaneLapRunner.Run(track, file.WithLanes(lanes.WithOffsets(LaneId.Free, line)), roadGrip: grip, model: model, flyingLaps: marks.Any(m => m < marks[0]) ? 2 : 1,
            onStep: (car, _, t) =>
            {
                float s = car.Progress.CurrentS;
                if (!float.IsNaN(previousS))
                {
                    // Where the car was and is, in one stretch across the seam.
                    float s1 = s < previousS - 0.5f * length ? s + length : s;
                    for (int m = 0; m < marks.Length; m++)
                    {
                        foreach (float c in (ReadOnlySpan<float>)[marks[m], marks[m] + length])
                        {
                            if (c <= previousS || c > s1)
                                continue;
                            // Every mark is passed once on the out-lap and once on the flying lap.
                            if (++seen[m] == (marks[m] < marks[0] ? 3 : 2))
                            {
                                float u = (c - previousS) / MathF.Max(s1 - previousS, 1e-4f);
                                times[m] = previousT + u * (t - previousT);
                                speeds[m] = car.State.Speed;
                            }
                        }
                    }
                }
                previousS = s;
                previousT = t;
            });
        if (seen.Where((c, m) => c < (marks[m] < marks[0] ? 3 : 2)).Any())
            times[0] = float.NaN;
        return new Run(lap, times, speeds);
    }
}

static class CornerMap
{
    public readonly record struct Bend(int First, int Apex, int Last);

    /// <summary>The free line's signed curvature at every station, smoothed over ten metres either side.</summary>
    public static float[] FreeCurvature(LaneSet lanes)
    {
        int n = lanes.Count;
        var raw = new float[n];
        for (int i = 0; i < n; i++)
            raw[i] = LaneSet.Curvature(
                lanes.Position(i - 2, lanes.Offset(LaneId.Free, i - 2)),
                lanes.Position(i, lanes.Offset(LaneId.Free, i)),
                lanes.Position(i + 2, lanes.Offset(LaneId.Free, i + 2)));
        int half = Math.Max(1, (int)MathF.Round(10f / lanes.Spacing));
        var smooth = new float[n];
        for (int i = 0; i < n; i++)
        {
            float sum = 0f;
            for (int j = -half; j <= half; j++)
                sum += raw[lanes.Wrap(i + j)];
            smooth[i] = sum / (2 * half + 1);
        }
        return smooth;
    }

    /// <summary>
    /// Stretches turning tighter than <paramref name="radius"/> one way, joined
    /// across gaps shorter than <paramref name="joinMeters"/>.
    /// </summary>
    public static List<Bend> Bends(LaneSet lanes, float[] k, float radius = 600f, float joinMeters = 30f)
    {
        int n = lanes.Count;
        float limit = 1f / radius;
        // Start the walk on a straight so no bend is cut by the seam.
        int start = 0;
        while (start < n && MathF.Abs(k[start]) >= limit)
            start++;
        var bends = new List<Bend>();
        int join = (int)MathF.Round(joinMeters / lanes.Spacing);
        int? first = null;
        int last = 0, sign = 0;
        for (int step = 0; step <= n; step++)
        {
            int i = lanes.Wrap(start + step);
            bool turning = MathF.Abs(k[i]) >= limit;
            int s = MathF.Sign(k[i]);
            if (turning && first is null)
            {
                first = start + step;
                sign = s;
            }
            else if (turning && s != sign)
            {
                bends.Add(Close(first!.Value, last));
                first = start + step;
                sign = s;
            }
            if (turning)
                last = start + step;
            else if (first is not null && start + step - last > join)
            {
                bends.Add(Close(first.Value, last));
                first = null;
            }
        }
        if (first is not null)
            bends.Add(Close(first.Value, last));
        return bends;

        Bend Close(int a, int b)
        {
            int apex = a;
            for (int i = a; i <= b; i++)
                if (MathF.Abs(k[lanes.Wrap(i)]) > MathF.Abs(k[lanes.Wrap(apex)]))
                    apex = i;
            return new Bend(lanes.Wrap(a), lanes.Wrap(apex), lanes.Wrap(b));
        }
    }
}

static class Duel
{
    /// <summary>A pass tried into the bend after the straight: each car's own line through it, and the side the chaser goes.</summary>
    public sealed record Attack(LaneTrackFile ChaserFile, LaneSet ChaserLine, LaneTrackFile LeaderFile, int Side);

    public sealed record Result(float[] Gap, float[] TimeGap, float SpeedGain, int PulledOutAt, float HeldSeconds, bool Contact, float HeldAfter = 0f, string Note = "");

    /// <summary>The stock car on the development compound with <paramref name="advantage"/> more grip.</summary>
    /// <summary>The leader's rear tyres' grip, as a share of new: below one they are worn and it cannot put its power down.</summary>
    public static float LeaderRear = 1f;

    /// <summary>Stations to trace the two cars over, every five: none when the first is negative.</summary>
    public static (int From, int To) Trace = (-1, -1);

    public static ICar CarWithRear(float rear)
    {
        TireCompound super = StockCar.Loaded.Compounds.First(c => c.Id == DevelopmentTyres.SuperId);
        var numbers = System.Text.Json.Nodes.JsonNode.Parse(System.Text.Json.JsonSerializer.Serialize(super.Rear))!;
        numbers[nameof(TireConfig.BaseMu)] = super.Rear.BaseMu * rear;
        TireConfig worn = System.Text.Json.JsonSerializer.Deserialize<TireConfig>(numbers)!;
        return StockCar.Loaded.Definition.Build([new TireCompound(super.Id, super.Name, super.Front, worn)]);
    }

    public static ICar CarWithGrip(float advantage)
    {
        TireCompound super = StockCar.Loaded.Compounds.First(c => c.Id == DevelopmentTyres.SuperId);
        if (advantage == 0f)
            return StockCar.Build(DevelopmentTyres.SuperId);
        TireConfig Scale(TireConfig t)
        {
            var numbers = System.Text.Json.Nodes.JsonNode.Parse(System.Text.Json.JsonSerializer.Serialize(t))!;
            numbers[nameof(TireConfig.BaseMu)] = t.BaseMu * (1f + advantage);
            return System.Text.Json.JsonSerializer.Deserialize<TireConfig>(numbers)!;
        }
        TireConfig front = Scale(super.Front);
        TireConfig rear = ReferenceEquals(super.Front, super.Rear) ? front : Scale(super.Rear);
        return StockCar.Loaded.Definition.Build([new TireCompound(super.Id, super.Name, front, rear)]);
    }

    public static Result Run(TrackData track, LaneTrackFile file, LaneSet lanes, LaneLapResult solo, int start, int[] marks,
        int braking, float followSeconds, int backOff, int release, int straightFrom, float advantage, bool push = false, Attack? attack = null)
    {
        var simulation = new RaceSimulation(track);
        float length = track.LengthMeters, spacing = lanes.Spacing;
        float leaderS = start * spacing;
        var (leader, _) = LaneGrid.Seat(simulation, attack?.LeaderFile ?? file, lanes, "leader", LeaderRear == 1f ? StockCar.Build(DevelopmentTyres.SuperId) : CarWithRear(LeaderRear),
            DevelopmentTyres.SuperId, leaderS, 0f, solo.StationSpeed[start]);
        float body = leader.Car.Spec.Body?.LengthMeters ?? 5f;
        float chaserS = leaderS - body - 25f;
        int chaserStation = lanes.Wrap((int)MathF.Round(chaserS / spacing));

        ICar model = CarWithGrip(advantage);
        var inner = new LaneDriver(attack?.ChaserFile ?? file, model) { LearnsAsItDrives = false };
        var follow = new Follow(inner, lanes, attack?.ChaserLine ?? lanes, attack?.Side ?? 0, followSeconds, backOff, release, straightFrom, braking);
        System.Numerics.Vector2 here = lanes.Position(chaserStation, lanes.Offset(LaneId.Free, chaserStation));
        System.Numerics.Vector2 next = lanes.Position(chaserStation + 1, lanes.Offset(LaneId.Free, chaserStation + 1));
        var chaser = new RaceCar("chaser", model, new Driver(new DriverProfile("chaser", new DriverAbilities()), follow), new CarState
        {
            Position = here,
            Heading = MathF.Atan2(next.Y - here.Y, next.X - here.X),
            Speed = solo.StationSpeed[chaserStation]
        }) { Strategy = CarStrategy.Default };
        ((ICarTires)model).FitTires(chaser.State, DevelopmentTyres.SuperId);
        ((ICarStores)model).Fill(chaser.State, PowertrainState.Filled(0.8f));
        ((ICarTires)model).WarmTires(chaser.State, 90f, 90f);
        simulation.AddCar(chaser);

        // Every position unwrapped from the leader's start.
        float[] markU = marks.Select(m => leaderS + lanes.Wrap(m - start) * spacing).ToArray();
        var cars = new[] { leader, chaser };
        var u = new[] { leaderS, leaderS + MathF.IEEERemainder(chaserS - leaderS, length) };
        var last = cars.Select(c => c.Progress.CurrentS).ToArray();
        var at = new float[2, marks.Length];
        var speed = new float[2, marks.Length];
        var gap = new float[marks.Length];
        for (int m = 0; m < marks.Length; m++)
            at[0, m] = at[1, m] = float.NaN;
        bool contact = false;
        string note = "";
        var slowest = new[] { float.PositiveInfinity, float.PositiveInfinity };
        var trace = new System.Text.StringBuilder();
        int tracedAt = -1;
        const float dt = 1f / 60f;
        for (int step = 0; step < 60 * 60; step++)
        {
            int chaserAt = lanes.Wrap((int)lanes.StationOf(chaser.Progress.CurrentS));
            bool pushing = push && lanes.Wrap(chaserAt - straightFrom) < lanes.Wrap(braking - straightFrom);
            chaser.Strategy = chaser.Strategy.WithButton(0, pushing);
            simulation.Step(dt);
            float t = (step + 1) * dt;
            for (int c = 0; c < 2; c++)
            {
                float s = cars[c].Progress.CurrentS;
                float moved = MathF.IEEERemainder(s - last[c], length);
                float u0 = u[c];
                u[c] += moved;
                last[c] = s;
                for (int m = 0; m < marks.Length; m++)
                    if (float.IsNaN(at[c, m]) && u0 < markU[m] && u[c] >= markU[m])
                    {
                        at[c, m] = t - dt * (u[c] - markU[m]) / MathF.Max(moved, 1e-4f);
                        speed[c, m] = cars[c].State.Speed;
                        if (c == 1)
                            gap[m] = u[0] - u[1] - body;
                        if (c == 1)
                            note += $" [{m}: v {cars[1].State.Speed:F1}/{cars[0].State.Speed:F1} grip {inner.Grip:F3}/{inner.ActualGrip:F3} range {inner.Range:F2} plan {inner.PlannedSpeedAt(cars[1].Progress.CurrentS):F1}]";
                    }
            }
            for (int c = 0; c < 2; c++)
                if (marks.Length >= 4 && u[c] >= markU[^4])
                    slowest[c] = MathF.Min(slowest[c], cars[c].State.Speed);
            if (Trace.From >= 0)
            {
                int st = lanes.Wrap((int)lanes.StationOf(chaser.Progress.CurrentS));
                if (st % 5 == 0 && st != tracedAt && lanes.Wrap(st - Trace.From) <= lanes.Wrap(Trace.To - Trace.From))
                {
                    tracedAt = st;
                    trace.AppendLine($"    {st,5}  gap {u[0] - u[1] - body,6:F1}  v {chaser.State.Speed,5:F1}/{leader.State.Speed,5:F1}  plan {inner.PlannedSpeedAt(chaser.Progress.CurrentS),5:F1}  d {chaser.Progress.CurrentD,5:F2}/{leader.Progress.CurrentD,5:F2}  line {(attack?.ChaserLine ?? lanes).Offset(LaneId.Free, st),5:F2}  accel {follow.LastAsked,6:F1}->{follow.LastGiven,6:F1}  push {(chaser.Strategy.Holding(0) ? 1 : 0)}");
                }
            }
            float along = u[0] - u[1];
            float across = MathF.Abs(MathF.IEEERemainder(0f, 1f) + (leader.Progress.CurrentD - chaser.Progress.CurrentD));
            if (MathF.Abs(along) < body && across < 1.9f)
                contact = true;
            if (!float.IsNaN(at[1, marks.Length - 1]) && !float.IsNaN(at[0, marks.Length - 1]))
                break;
        }
        var timeGap = new float[marks.Length];
        for (int m = 0; m < marks.Length; m++)
            timeGap[m] = at[1, m] - at[0, m];
        int lastMark = marks.Length - 1;
        note = trace + $"leader spins {((ICarMotion)leader.Car).Motion(leader.State).Spins}, slowest {slowest[0]:F1} m/s; chaser spins {((ICarMotion)model).Motion(chaser.State).Spins}, slowest {slowest[1]:F1} m/s" + note;
        return new Result(gap, timeGap, speed[1, lastMark] - speed[0, lastMark], follow.PulledOutAt, follow.HeldSeconds, contact, follow.HeldAfterRelease, note);
    }

    /// <summary>
    /// The chaser: the lane driver, its pedal held back to a time gap on the
    /// car ahead until <c>release</c>, then free; on the straight, close
    /// behind, it pulls out beside the leader's line and stays there.
    /// </summary>
    private sealed class Follow(LaneDriver inner, LaneSet lanes, LaneSet line, int attackSide, float seconds, int backOff, int release, int straightFrom, int braking) : IDriverController
    {
        private bool _released, _backing;

        /// <summary>How far before the braking the chaser pulls out if it is close: the last part of the straight.</summary>
        public static float PullOutMeters = 150f;

        private static float Ease(float x)
        {
            x = Math.Clamp(x, 0f, 1f);
            return x * x * (3f - 2f * x);
        }

        /// <summary>Centre to centre across, beside the leader: a car's width and a metre.</summary>
        private const float Beside = 2.9f;
        public int PulledOutAt { get; private set; } = -1;
        public float HeldSeconds { get; private set; }
        public float HeldAfterRelease { get; private set; }

        public CarNeeds Needs => ((IDriverController)inner).Needs;

        public void Initialize(in DriverContext context) => inner.Initialize(in context);

        public DriverInput GetControl(in DriverContext context, float dt)
        {
            DriverInput input = inner.GetControl(in context, dt);
            LastAsked = input.DesiredAccel;
            RaceCarSnapshot me = context.Car, ahead = context.Frame[0];
            int station = lanes.Wrap((int)lanes.StationOf(me.TrackS));
            float length = lanes.LengthMeters;
            float gap = MathF.IEEERemainder(ahead.TrackS - me.TrackS, length) - 5f;
            if (!_backing && lanes.Wrap(station - backOff) < 50)
                _backing = true;
            if (!_released && lanes.Wrap(station - release) < 50)
                _released = true;
            float v = me.SpeedMetersPerSecond, vAhead = ahead.SpeedMetersPerSecond;
            float cap = float.PositiveInfinity;
            bool beside = MathF.Abs(ahead.TrackD - me.TrackD) > 2.1f;
            if (!_released)
                cap = 1.2f * (gap - (_backing ? seconds : 0.3f) * v) + 1.8f * (vAhead - v);
            else if (!beside && gap < 6f && gap > 0f)
                cap = 1.5f * (gap - 3f) + 2f * (vAhead - v);
            if (cap < input.DesiredAccel)
            {
                input = input with { DesiredAccel = cap };
                HeldSeconds += dt;
                if (_released)
                    HeldAfterRelease += dt;
            }

            // Out from behind on the straight, as late as it can: when it
            // would otherwise run into the leader, or in the last stretch
            // before the braking with the leader close enough to reach. It
            // goes to whichever side of the leader has a car's width of road
            // and more, stays there into the braking zone, and comes back
            // to its line well past it, so the speed it plans to the braking
            // point is the straight's and not a kink's.
            bool straight = lanes.Wrap(station - straightFrom) < lanes.Wrap(braking - straightFrom);
            float toBraking = lanes.Wrap(braking - station) * lanes.Spacing;
            bool late = toBraking < PullOutMeters && gap < 25f;
            bool tooClose = gap < 8f && v > vAhead;
            if (_released && PulledOutAt < 0 && straight && gap > -5f && (late || tooClose) && (attackSide != 0 || ReferenceEquals(line, lanes)))
            {
                float leftRoom = lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters - ahead.TrackD;
                float rightRoom = ahead.TrackD + lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters;
                float side = attackSide != 0 ? attackSide : leftRoom >= rightRoom ? 1f : -1f;
                if ((side > 0f ? leftRoom : rightRoom) >= Beside)
                {
                    PulledOutAt = station;
                    // Onto its own line for the bend by the braking point when it has one; else beside until well past it.
                    int end = lanes.Wrap(braking + (attackSide != 0 ? 10 : 150)), count = lanes.Wrap(end - station);
                    var offsets = new float[count];
                    float aim = ahead.TrackD + side * Beside;
                    for (int j = 0; j < count; j++)
                    {
                        int i = lanes.Wrap(station + j);
                        float edge = lanes.HalfWidth(i) - LaneSet.EdgeMarginMeters;
                        float free = line.Offset(LaneId.Free, i);
                        float want = Math.Clamp(aim, -edge, edge) - free;
                        // Eased in and out, an S on the road: no kink for the speed plan to lift for.
                        float into = Ease(j / 40f), outOf = Ease((count - j) / (attackSide != 0 ? 60f : 75f));
                        offsets[j] = want * MathF.Min(into, outOf);
                    }
                    inner.DriveLine(station, offsets);
                }
            }            LastGiven = input.DesiredAccel;
            return input;
        }

        public float LastAsked { get; private set; }
        public float LastGiven { get; private set; }
    }
}

static class Dive
{
    /// <param name="BrakeFrom">The station the car's speed first falls at past the claim point, on the flying lap.</param>
    /// <param name="Seconds">From the claim point to the next bend's apex.</param>
    /// <param name="Wide">The most the car ran away from its line between the claim point and the exit, metres.</param>
    public sealed record Result(int BrakeFrom, float BrakeSpeed, float Slowest, float Seconds, float Wide, float Off, int Spins);

    public static Result Run(TrackData track, LaneTrackFile file, LaneSet lanes, float[] line, float factor, int claim, CornerLines.Pins pins, int next)
    {
        int count = Math.Max(lanes.Wrap(pins.Exit + 40 - claim), lanes.Wrap(next - claim) + 5);
        int toNext = lanes.Wrap(next - claim);
        var zeros = new float[count];
        int passes = 0, brakeFrom = -1;
        float lastT = 0f, lastV = 0f, start = float.NaN, end = float.NaN;
        float brakeSpeed = 0f, slowest = float.PositiveInfinity, wide = 0f, off = 0f;
        bool inWindow = false;
        LaneLapResult result = LaneLapRunner.Run(track, file.WithLanes(lanes.WithOffsets(LaneId.Free, line)), onStep: (car, driver, t) =>
        {
            int station = lanes.Wrap((int)lanes.StationOf(car.Progress.CurrentS));
            int into = lanes.Wrap(station - claim);
            if (into < 3 && !inWindow)
            {
                // Into the window: the braking taken at the factor from here to past the exit.
                inWindow = true;
                passes++;
                if (factor != 1f)
                    driver.DriveLine(station, zeros, factor);
                if (passes == 2)
                    start = lastT;
            }
            else if (inWindow && into >= count && into < lanes.Count / 2 + count)
            {
                inWindow = false;
                if (factor != 1f)
                    driver.ClearLine();
            }
            if (passes == 2 && inWindow)
            {
                float v = car.State.Speed;
                if (brakeFrom < 0 && v < lastV - 0.05f)
                {
                    brakeFrom = station;
                    brakeSpeed = lastV;
                }
                slowest = MathF.Min(slowest, v);
                wide = MathF.Max(wide, MathF.Abs(car.Progress.CurrentD - line[station]));
                if (car.Progress.Region != TrackRegion.RacingSurface)
                    off += 1f / 60f;
                if (float.IsNaN(end) && into >= toNext)
                    end = t;
            }
            lastT = t;
            lastV = car.State.Speed;
        }, flyingLaps: 1);
        return new Result(brakeFrom, brakeSpeed, slowest, end - start, wide, off, result.Spins);
    }
}

static class Battle
{
    /// <summary>
    /// From a car's front axle back to its mirrors. An assumption: the
    /// guidelines judge an inside pass on the attacker's front axle reaching
    /// the defender's mirrors at the apex; where the mirrors sit is the car's.
    /// </summary>
    public const float AxleToMirror = 1.8f;

    /// <summary>The least a car's width and a little apart two lines may come and both be driven side by side, centre to centre.</summary>
    public const float Room = 2.6f;

    /// <summary>How long a pull-out takes: one quick move across, about two seconds.</summary>
    public const float PullSeconds = 2f;

    /// <summary>The leader's rear grip in each tier a rival may read it in, as a share of new: tier 0 is new.</summary>
    public static readonly float[] RearTiers = [1f, 0.95f, 0.9f, 0.85f];

    public const string Hold = "外内外", Cover = "内内外";

    public static float Ease(float x)
    {
        x = Math.Clamp(x, 0f, 1f);
        return x * x * (3f - 2f * x);
    }

    /// <summary>
    /// What each car will do. The leader holds its place outside, covers the
    /// inside once at <see cref="Context.DefendAt"/>, or decides there: it
    /// covers when the chaser's best attack, as it reads it, would come
    /// within <see cref="DefendWithin"/> metres of earning the apex. The
    /// chaser attacks on a given line, or decides two seconds before the
    /// turn-in: it goes on its best line when that would earn the apex by
    /// <see cref="AttackMargin"/> metres or more, as it reads the leader. It
    /// reads the leader's rears <see cref="ReadError"/> tiers off the truth.
    /// </summary>
    public sealed record Policy
    {
        public string? Attack { get; init; }
        public bool AutoAttack { get; init; }
        public float AttackMargin { get; init; }
        public bool CoverAlways { get; init; }
        public bool AutoDefend { get; init; }
        public float DefendWithin { get; init; }
        public bool Push { get; init; } = true;
        public int Tier { get; init; }
        public int ReadError { get; init; }
        /// <summary>The leader reads the chaser's push the wrong way round.</summary>
        public bool MisreadsPush { get; init; }
        public int ReadTier => Math.Clamp(Tier + ReadError, 0, RearTiers.Length - 1);
    }

    public sealed class Context
    {
        public required TrackData Track;
        public required LaneTrackFile File;
        public required LaneSet Lanes;
        public required LaneLapResult Solo;
        public required System.Collections.Concurrent.ConcurrentDictionary<string, float[]> Lines;
        public required float[] Free, HoldPath;
        /// <summary>The leader's path covering the inside onto each line that turns in from there.</summary>
        public required Dictionary<string, float[]> CoverPaths;

        /// <summary>The leader's fastest line turning in from the inside that it can drive on its rears, or none.</summary>
        public string? CoverLine(int tier) => CoverPaths.Keys.Where(code => LeaderCan(tier, code))
            .OrderBy(code => LeaderSeconds(tier, code, Decide, Luffield)).FirstOrDefault();
        /// <summary>The leader's timetables on each line, one set per tier of its rears.</summary>
        public required System.Collections.Concurrent.ConcurrentDictionary<string, CornerLines.Run>[] LeaderTables;
        public required System.Collections.Concurrent.ConcurrentDictionary<string, CornerLines.Run> ChaserTables;
        public required int TableFrom, Claim, Start, Release, PushFrom, Braking, Decide, Entry, Apex, Exit, Luffield, WindowEnd, InsideDirection, DefendAt, PullAt;
        /// <summary>Seconds the chaser closes in the tow from the exit before the straight to the pull-out point: by leader tier, without and with a push.</summary>
        public float[,] Gain = new float[RearTiers.Length, 2];

        /// <summary>Time on a line alone at a station, interpolated, from its timetable.</summary>
        private float At(CornerLines.Run run, float station)
        {
            float j = Lanes.Wrap((int)MathF.Floor(station) - TableFrom) + (station - MathF.Floor(station));
            int a = Math.Clamp((int)j, 0, run.Times.Length - 1), b = Math.Min(a + 1, run.Times.Length - 1);
            return run.Times[a] + (run.Times[b] - run.Times[a]) * (j - a);
        }

        public float LeaderSeconds(int tier, string code, float from, float to) => At(LeaderTables[tier][code], to) - At(LeaderTables[tier][code], from);
        public float ChaserSeconds(string code, float from, float to) => At(ChaserTables[code], to) - At(ChaserTables[code], from);
        public float ChaserSpeed(string code, int station) => ChaserTables[code].Speeds[Math.Clamp(Lanes.Wrap(station - TableFrom), 0, ChaserTables[code].Speeds.Length - 1)];
        public float LeaderSpeed(int tier, string code, int station) => LeaderTables[tier][code].Speeds[Math.Clamp(Lanes.Wrap(station - TableFrom), 0, LeaderTables[tier][code].Speeds.Length - 1)];

        public bool LeaderCan(int tier, string code) => Drives(LeaderTables[tier][code]);
        public bool ChaserCan(string code) => Drives(ChaserTables[code]);
        /// <summary>Driven alone without a spin, and timed in order at every station.</summary>
        private static bool Drives(CornerLines.Run run)
        {
            if (run.Lap.Spins != 0 || float.IsNaN(run.Times[0]))
                return false;
            for (int i = 1; i < run.Times.Length; i++)
                if (!(run.Times[i] > run.Times[i - 1]))
                    return false;
            return true;
        }

        /// <summary>The leader's lines that turn in from where it already is.</summary>
        public IEnumerable<string> LeaderOptions(int tier, string line) => Lines.Keys.Where(c => c[0] == line[0] && LeaderCan(tier, c)).OrderBy(c => c);

        /// <summary>The chaser's lines that turn in from somewhere else than the leader does.</summary>
        public IEnumerable<string> Attacks(string leaderLine) => Lines.Keys.Where(c => c[0] != leaderLine[0] && ChaserCan(c)).OrderBy(c => c);

        /// <summary>Whether a line takes its apex inside of the leader's.</summary>
        public bool Inside(string attack, float[] leaderPath) => (Lines[attack][Apex] - leaderPath[Apex]) * InsideDirection > 0f;

        /// <summary>
        /// How far behind the leader's centre the chaser's will be at the
        /// apex, each on its line from where it is, driven as alone; less
        /// whatever either is carrying above its alone speed now, held to the
        /// braking point.
        /// </summary>
        public float Along(int tier, string leaderLine, float leaderAt, float leaderSpeed, string attack, float chaserAt, float chaserSpeed)
        {
            float late = ChaserSeconds(attack, chaserAt, Apex) - LeaderSeconds(tier, leaderLine, leaderAt, Apex);
            float toBraking = Lanes.Wrap(Braking - (int)chaserAt) * Lanes.Spacing;
            float carried = toBraking < Lanes.LengthMeters / 2f
                ? ((chaserSpeed - ChaserSpeed(attack, (int)chaserAt)) - (leaderSpeed - LeaderSpeed(tier, leaderLine, (int)leaderAt))) * toBraking / MathF.Max(chaserSpeed, 1f)
                : 0f;
            return late * ChaserSpeed(attack, Apex) - carried;
        }

        /// <summary>By how many metres an attack would earn the apex: positive earns it.</summary>
        public float Margin(string attack, float[] leaderPath, float along) => (Inside(attack, leaderPath) ? AxleToMirror : 0f) - along;

        /// <summary>
        /// How close across the road the two come where they are alongside:
        /// each driven as alone from where it is now, the stations from the
        /// turn-in to past the exit where the two are within a car's length
        /// of each other in time at the speed there. Infinite where they are
        /// never alongside.
        /// </summary>
        public float Room(int tier, string leaderLine, float leaderAt, string chaserLine, float chaserAt, float body)
        {
            float closest = float.PositiveInfinity;
            for (int i = Lanes.Wrap(Entry - 10); i != Lanes.Wrap(Exit + 20); i = Lanes.Wrap(i + 1))
            {
                float apart = (ChaserSeconds(chaserLine, chaserAt, i) - LeaderSeconds(tier, leaderLine, leaderAt, i)) * ChaserSpeed(chaserLine, i);
                if (MathF.Abs(apart) < body)
                    closest = MathF.Min(closest, MathF.Abs(Lines[leaderLine][i] - Lines[chaserLine][i]));
            }
            return closest;
        }

        /// <summary>The leader's fastest line from where it is that leaves an attack room, or none.</summary>
        public string? LeavesRoom(int tier, string line, float mine, string attack, float theirs, float body) =>
            LeaderOptions(tier, line)
                .Select(o => (Code: o, Room: Room(tier, o, mine, attack, theirs, body), Seconds: LeaderSeconds(tier, o, mine, Luffield)))
                .Where(o => o.Room >= Battle.Room)
                .OrderBy(o => o.Seconds)
                .Select(o => o.Code)
                .FirstOrDefault();
    }

    public sealed class Referee
    {
        public bool Decided, Earned, PulledOut, Considered, AttackerTucks, Covered;
        public string? AttackerLine, Attack;
        public int PulledAt = -1;
        public float Along = float.NaN, Threat = float.NaN, Read = float.NaN;
        public string LeaderLine = Hold;
        public float[]? LeaderTarget;
    }

    public sealed record Result(string Decision, string LeaderLine, string? Attack, bool Covered, float Threat, float Read, int PulledAt, float Along,
        float[] Gap, float[] TimeGap, float Closest, bool Contact, int Spins);

    public static Result Run(Context c, Policy p)
    {
        LaneSet lanes = c.Lanes;
        var simulation = new RaceSimulation(c.Track);
        LaneTrackFile pathFile = c.File.WithLanes(lanes.WithOffsets(LaneId.Free, c.HoldPath));
        var referee = new Referee { LeaderLine = Hold, LeaderTarget = c.HoldPath, Attack = p.Attack };

        ICar leaderModel = p.Tier == 0 ? StockCar.Build(DevelopmentTyres.SuperId) : Duel.CarWithRear(RearTiers[p.Tier]);
        var defender = new Defender(new LaneDriver(pathFile, leaderModel) { LearnsAsItDrives = false }, c, referee, p);
        RaceCar leader = Seat(simulation, "leader", leaderModel, defender, lanes, c.HoldPath, c.Start, c.Solo.StationSpeed[c.Start]);
        float body = leader.Car.Spec.Body?.LengthMeters ?? 4.8f;
        int behind = lanes.Wrap(c.Start - (int)MathF.Round((body + 25f) / lanes.Spacing));
        ICar chaserModel = StockCar.Build(DevelopmentTyres.SuperId);
        var attacker = new Attacker(new LaneDriver(pathFile, chaserModel) { LearnsAsItDrives = false }, c, referee, p, body);
        RaceCar chaser = Seat(simulation, "chaser", chaserModel, attacker, lanes, c.HoldPath, behind, c.Solo.StationSpeed[behind]);

        float length = c.Track.LengthMeters, spacing = lanes.Spacing;
        float origin = c.Start * spacing;
        // Metres apart at the braking point, apex, exit and Luffield; seconds at those and at the straight's start and the pull-out point.
        int[] marks = [c.Braking, c.Apex, c.Exit, c.Luffield, c.PushFrom, c.PullAt];
        float[] markU = marks.Select(m => origin + lanes.Wrap(m - c.Start) * spacing).ToArray();
        var cars = new[] { leader, chaser };
        var u = new[] { origin, origin - lanes.Wrap(c.Start - behind) * spacing };
        var last = cars.Select(x => x.Progress.CurrentS).ToArray();
        var gap = Enumerable.Repeat(float.NaN, marks.Length).ToArray();
        var at = new float[2, marks.Length];
        for (int m = 0; m < marks.Length; m++)
            at[0, m] = at[1, m] = float.NaN;
        float closest = float.PositiveInfinity;
        bool contact = false;
        const float dt = 1f / 60f;
        for (int step = 0; step < 60 * 60 && float.IsNaN(gap[3]); step++)
        {
            int here = lanes.Wrap((int)lanes.StationOf(chaser.Progress.CurrentS));
            chaser.Strategy = chaser.Strategy.WithButton(0, p.Push && lanes.Wrap(here - c.PushFrom) < lanes.Wrap(c.Braking - c.PushFrom));
            simulation.Step(dt);
            float t = (step + 1) * dt;
            for (int x = 0; x < 2; x++)
            {
                float s = cars[x].Progress.CurrentS;
                float moved = MathF.IEEERemainder(s - last[x], length);
                float u0 = u[x];
                u[x] += moved;
                last[x] = s;
                for (int m = 0; m < marks.Length; m++)
                    if (float.IsNaN(at[x, m]) && u0 < markU[m] && u[x] >= markU[m])
                        at[x, m] = t - dt * (u[x] - markU[m]) / MathF.Max(moved, 1e-4f);
            }
            for (int m = 0; m < marks.Length; m++)
                if (float.IsNaN(gap[m]) && u[0] >= markU[m] && u[1] >= markU[m])
                    gap[m] = u[0] - u[1] - body;
            float along = u[0] - u[1];
            float across = MathF.Abs(leader.Progress.CurrentD - chaser.Progress.CurrentD);
            if (MathF.Abs(along) < body)
            {
                closest = MathF.Min(closest, across);
                if (across < (leader.Car.Spec.Body?.WidthMeters ?? 1.9f))
                    contact = true;
            }
        }
        var timeGap = new float[marks.Length];
        for (int m = 0; m < marks.Length; m++)
            timeGap[m] = at[1, m] - at[0, m];
        int spins = ((ICarMotion)leader.Car).Motion(leader.State).Spins + ((ICarMotion)chaserModel).Motion(chaser.State).Spins;
        string decision = !referee.PulledOut ? "-" : !referee.Decided ? "?" : (referee.Earned ? "chaser" : "leader") +
            (referee.AttackerTucks ? ", tucks" : referee.AttackerLine is { } beside && beside != referee.Attack ? $", to {beside}" : "");
        return new Result(decision, referee.LeaderLine, referee.PulledOut ? referee.Attack : null, referee.Covered, referee.Threat, referee.Read,
            referee.PulledAt, referee.Along, gap, timeGap, closest, contact, spins);
    }

    private static RaceCar Seat(RaceSimulation simulation, string name, ICar model, IDriverController controller, LaneSet lanes, float[] path, int station, float speed)
    {
        System.Numerics.Vector2 here = lanes.Position(station, path[station]);
        System.Numerics.Vector2 next = lanes.Position(station + 1, path[lanes.Wrap(station + 1)]);
        var car = new RaceCar(name, model, new Driver(new DriverProfile(name, new DriverAbilities()), controller), new CarState
        {
            Position = here,
            Heading = MathF.Atan2(next.Y - here.Y, next.X - here.X),
            Speed = speed
        }) { Strategy = CarStrategy.Default };
        ((ICarTires)model).FitTires(car.State, DevelopmentTyres.SuperId);
        ((ICarStores)model).Fill(car.State, PowertrainState.Filled(0.8f));
        ((ICarTires)model).WarmTires(car.State, 90f, 90f);
        simulation.AddCar(car);
        return car;
    }

    /// <summary>
    /// One quick move from where a car is onto a line: from <paramref name="from"/>
    /// it eases across over <paramref name="over"/> stations onto
    /// <paramref name="target"/>, then keeps to it to the window's end; as
    /// offsets from <paramref name="path"/>, the line the car's lanes are.
    /// </summary>
    private static float[] Move(LaneSet lanes, float[] path, int from, float offsetNow, int over, float[] target, int end)
    {
        int count = lanes.Wrap(end - from);
        var relative = new float[count];
        float held = offsetNow - path[from];
        for (int j = 0; j < count; j++)
        {
            int i = lanes.Wrap(from + j);
            float now = path[i] + held;
            float there = j < over ? now + (target[i] - now) * Ease(j / (float)over) : target[i];
            relative[j] = there - path[i];
        }
        return relative;
    }

    /// <summary>
    /// Off the back of a car ahead that is in the way across the road: the
    /// pedal held to what closes to three metres no faster than it can stop;
    /// when <paramref name="tucked"/>, held in behind whatever the road
    /// across, short of being beside it already.
    /// </summary>
    private static DriverInput KeepOff(DriverInput input, float gap, RaceCarSnapshot me, RaceCarSnapshot ahead, bool tucked)
    {
        float across = MathF.Abs(ahead.TrackD - me.TrackD);
        float closing = me.SpeedMetersPerSecond - ahead.SpeedMetersPerSecond;
        bool inTheWay = across < 2.2f || tucked && gap > -2f;
        if (!inTheWay || gap < -2f || gap > 80f)
            return input;
        float safe = 3f + MathF.Max(0f, closing) * 0.8f;
        float cap = 1.5f * (gap - safe) - 2f * closing;
        return cap < input.DesiredAccel ? input with { DesiredAccel = cap } : input;
    }

    private sealed class Defender(LaneDriver inner, Context c, Referee referee, Policy p) : IDriverController
    {
        private bool _defended;
        public CarNeeds Needs => ((IDriverController)inner).Needs;
        public void Initialize(in DriverContext context) => inner.Initialize(in context);

        public DriverInput GetControl(in DriverContext context, float dt)
        {
            DriverInput input = inner.GetControl(in context, dt);
            LaneSet lanes = c.Lanes;
            RaceCarSnapshot me = context.Car, other = context.Frame[1];
            int station = lanes.Wrap((int)lanes.StationOf(me.TrackS));
            const float body = 4.8f;
            // Never into the back of the other once it is ahead and in the way.
            float ahead = MathF.IEEERemainder(other.TrackS - me.TrackS, lanes.LengthMeters) - body;
            input = KeepOff(input, ahead, me, other, false);

            // Its one move on the straight, early, before the chaser could pull out.
            if (!_defended && lanes.Wrap(station - c.DefendAt) < 5)
            {
                _defended = true;
                bool cover = p.CoverAlways;
                if (!cover && p.AutoDefend)
                {
                    referee.Threat = Threat(me, other, -ahead - body);
                    cover = referee.Threat >= -p.DefendWithin;
                }
                if (cover && c.CoverLine(p.Tier) is { } onto)
                {
                    referee.Covered = true;
                    referee.LeaderLine = onto;
                    referee.LeaderTarget = c.CoverPaths[onto];
                    inner.DriveLine(station, Move(lanes, c.HoldPath, station, me.TrackD, 10, c.CoverPaths[onto], c.WindowEnd));
                }
            }

            if (referee.Decided || lanes.Wrap(station - c.Decide) >= 5)
                return input;
            referee.Decided = true;
            if (!referee.PulledOut || referee.Attack is not { } attack)
                return input;
            // The apex as the guidelines give it, from where the two will be
            // at it: the referee reads the leader as it truly is.
            string line = referee.LeaderLine;
            float[] path = referee.LeaderTarget!;
            float mine = lanes.StationOf(me.TrackS), theirs = lanes.StationOf(other.TrackS);
            float along = c.Along(p.Tier, line, mine, me.SpeedMetersPerSecond, attack, theirs, other.SpeedMetersPerSecond);
            referee.Along = along;
            referee.Earned = c.Margin(attack, path, along) >= 0f;
            // The attacker's lines from where it already is that keep a car's
            // width from the leader's wherever the two are alongside.
            string? Alongside(string leaderLine) => c.Lines.Keys
                .Where(a => a[0] == attack[0] && c.ChaserCan(a))
                .Select(a => (Code: a, Room: c.Room(p.Tier, leaderLine, mine, a, theirs, body), Seconds: c.ChaserSeconds(a, theirs, c.Luffield)))
                .Where(a => a.Room >= Room)
                .OrderBy(a => a.Seconds)
                .Select(a => a.Code)
                .FirstOrDefault();
            string? pick = referee.Earned ? c.LeavesRoom(p.Tier, line, mine, attack, theirs, body) : null;
            if (pick is not null)
            {
                referee.LeaderLine = pick;
                referee.LeaderTarget = c.Lines[pick];
                if (pick != line)
                    inner.DriveLine(station, Move(lanes, c.HoldPath, station, me.TrackD, 15, c.Lines[pick], c.WindowEnd));
            }
            else
            {
                // The apex is the leader's, or the attack cuts across every
                // line it has: the attacker keeps beside it on a line that
                // leaves it its own, or tucks in behind.
                referee.AttackerLine = Alongside(line);
                referee.AttackerTucks = referee.AttackerLine is null;
            }
            return input;
        }

        /// <summary>
        /// The chaser's best attack against holding the outside, as the
        /// leader reads it from the defence point: the chaser where it is
        /// now less what the tow and a push close on the straight to the
        /// pull-out point, then each attack line against its own outside
        /// line. Metres by which it would earn the apex.
        /// </summary>
        private float Threat(RaceCarSnapshot me, RaceCarSnapshot other, float metresBehind)
        {
            LaneSet lanes = c.Lanes;
            float seconds = metresBehind / MathF.Max(me.SpeedMetersPerSecond, 1f);
            float atPull = MathF.Max(0f, seconds - c.Gain[p.Tier, p.Push != p.MisreadsPush ? 1 : 0]);
            float leaderAt = c.PullAt;
            float metres = atPull * c.LeaderSpeed(p.Tier, Hold, c.PullAt);
            float chaserAt = lanes.Wrap((int)MathF.Round(c.PullAt - metres / lanes.Spacing));
            float best = float.NegativeInfinity;
            foreach (string a in c.Attacks(Hold))
            {
                float along = c.Along(p.Tier, Hold, leaderAt, c.LeaderSpeed(p.Tier, Hold, c.PullAt), a, chaserAt, c.ChaserSpeed(a, (int)chaserAt));
                // An attack the leader could not leave room for is no attack it must give way to.
                if (c.Margin(a, c.HoldPath, along) >= 0f && c.LeavesRoom(p.Tier, Hold, leaderAt, a, chaserAt, 4.8f) is null)
                    continue;
                best = MathF.Max(best, c.Margin(a, c.HoldPath, along));
            }
            return best;
        }
    }

    private sealed class Attacker(LaneDriver inner, Context c, Referee referee, Policy p, float body) : IDriverController
    {
        private bool _released, _yielded, _followed;
        public CarNeeds Needs => ((IDriverController)inner).Needs;
        public void Initialize(in DriverContext context) => inner.Initialize(in context);

        public DriverInput GetControl(in DriverContext context, float dt)
        {
            DriverInput input = inner.GetControl(in context, dt);
            LaneSet lanes = c.Lanes;
            RaceCarSnapshot me = context.Car, ahead = context.Frame[0];
            int station = lanes.Wrap((int)lanes.StationOf(me.TrackS));
            float gap = MathF.IEEERemainder(ahead.TrackS - me.TrackS, lanes.LengthMeters) - body;
            float v = me.SpeedMetersPerSecond, vAhead = ahead.SpeedMetersPerSecond;
            if (!_released && lanes.Wrap(station - c.Release) < 50)
                _released = true;

            // Held to 0.3 s through Village and the Loop; then its own pace,
            // kept off the leader's gearbox while behind it, and behind it
            // into the bend when the apex is the leader's.
            bool yields = referee.Decided && referee.PulledOut && referee.AttackerTucks;
            if (!_released)
            {
                float cap = 1.2f * (gap - 0.3f * v) + 1.8f * (vAhead - v);
                if (cap < input.DesiredAccel)
                    input = input with { DesiredAccel = cap };
            }
            else
                input = KeepOff(input, gap, me, ahead, yields);

            // In the tow: where the leader moves across, it follows, once.
            if (referee.Covered && !_followed && !referee.PulledOut)
            {
                _followed = true;
                inner.DriveLine(station, Move(lanes, c.HoldPath, station, me.TrackD, 40, referee.LeaderTarget!, c.WindowEnd));
            }

            // Without the apex: beside the leader on a line that leaves it
            // room, or in behind it, onto the line it takes.
            if (referee.AttackerLine is { } alongside && !_yielded)
            {
                _yielded = true;
                if (alongside != referee.Attack)
                    inner.DriveLine(station, Move(lanes, c.HoldPath, station, me.TrackD, 15, c.Lines[alongside], c.WindowEnd));
            }
            if (yields && !_yielded)
            {
                _yielded = true;
                inner.DriveLine(station, Move(lanes, c.HoldPath, station, me.TrackD, 20, referee.LeaderTarget!, c.WindowEnd));
            }

            // Out of the tow once, quickly, onto its line for the bend: the
            // move starts two seconds of road before the turn-in and is done
            // at it. Only from close enough behind to have a chance.
            if (_released && !referee.Considered)
            {
                float toEntry = lanes.Wrap(c.Entry - station) * lanes.Spacing;
                if (toEntry < lanes.LengthMeters / 2f && toEntry <= PullSeconds * v)
                {
                    referee.Considered = true;
                    string? attack = p.AutoAttack ? Choose(me, ahead) : p.Attack;
                    if (attack is not null && gap < 20f)
                    {
                        referee.Attack = attack;
                        referee.PulledOut = true;
                        referee.PulledAt = station;
                        inner.DriveLine(station, Move(lanes, c.HoldPath, station, me.TrackD, lanes.Wrap(c.Entry - station), c.Lines[attack], c.WindowEnd));
                    }
                }
            }
            return input;
        }

        /// <summary>
        /// Its best attack as it reads the leader: the line that would earn
        /// the apex by the most, against the line the leader is on, the
        /// leader's rears read in the tier it reads them in; taken when it
        /// earns by the margin its nerve asks for and the leader could leave
        /// it room, else none.
        /// </summary>
        private string? Choose(RaceCarSnapshot me, RaceCarSnapshot ahead)
        {
            LaneSet lanes = c.Lanes;
            string line = referee.LeaderLine;
            float[] path = referee.LeaderTarget!;
            float mine = lanes.StationOf(me.TrackS), theirs = lanes.StationOf(ahead.TrackS);
            string? best = null;
            float bestMargin = float.NegativeInfinity;
            foreach (string a in c.Attacks(line))
            {
                float along = c.Along(p.ReadTier, line, theirs, ahead.SpeedMetersPerSecond, a, mine, me.SpeedMetersPerSecond);
                float margin = c.Margin(a, path, along);
                if (c.LeavesRoom(p.ReadTier, line, theirs, a, mine, body) is null)
                    continue;
                if (margin > bestMargin)
                    (best, bestMargin) = (a, margin);
            }
            referee.Read = bestMargin;
            return bestMargin >= p.AttackMargin ? best : null;
        }
    }
}
