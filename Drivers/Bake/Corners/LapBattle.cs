using System.Collections.Concurrent;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Mods;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers;
using StintegyEVO.Drivers.Lanes;

/// <summary>
/// A whole lap raced from the driver's own table. Every place the table
/// names is baked, whether or not a pass may be made there: the lattice
/// lines through it, each timed alone at every tier of the leader's rear
/// grip, and the straight into it timed with the chaser in the tow. Two cars
/// then run the lap. At a place where a pass may be made, each decides from
/// those tables whether to attack, to cover, or to do neither. At a place
/// where none may, neither attacks; the two run side by side on the lines
/// the road leaves them when they arrive together, and one behind the other
/// when they do not.
/// </summary>
static class LapBattle
{
    /// <summary>One place of the lap, with everything the decision reads and everything the cars drive.</summary>
    public sealed class Baked
    {
        public required Lap.Complex Complex { get; init; }
        public required CornerBattle.Place Place { get; init; }
        /// <summary>The next place's claim, or just past this place's window where it is the last.</summary>
        public required int NextClaim;
        /// <summary>The station the straight before this place starts at, where the tow's gain is timed from.</summary>
        public required int StraightFrom;
        /// <summary>How many cars the road through this place can take abreast, as the table says.</summary>
        public int Abreast => Complex.MaxAbreast;
    }

    /// <summary>Bakes every place the driver's table names, then races the lap once per policy.</summary>
    public static int Run(string[] args)
    {
        string id = args[1];
        var track = GameMods.Registry.Circuits.Get(id).Track;
        LaneTrackFile file = LaneTrackFile.ForCircuit(id, track);
        LaneSet lanes = file.LanesFor(track);
        float[] k = CornerMap.FreeCurvature(lanes);
        List<CornerMap.Bend> bends = CornerMap.Bends(lanes, k);
        var corners = new List<Lap.Corner>();
        for (int i = 0; i < bends.Count; i++)
            corners.Add(new Lap.Corner(i, bends[i], CornerLines.PinsOf(lanes, k, bends, i), 0));
        List<Lap.Complex> table = Lap.Read(file.NotesText?.Invoke("corners.txt"));
        if (table.Count == 0)
        {
            Console.WriteLine($"{id}: the driver's table names nowhere it races.");
            return 0;
        }
        LaneLapResult solo = LaneLapRunner.Run(track, file);
        float[] free = Enumerable.Range(0, lanes.Count).Select(i => lanes.Offset(LaneId.Free, i)).ToArray();
        var car = StockCar.Build(DevelopmentTyres.SuperId);
        Console.WriteLine($"{id}: {corners.Count} corners, {table.Count} places in the driver's table, solo lap {solo.FlyingLapSeconds:F3}s");

        var places = new List<Baked>();
        foreach (Lap.Complex complex in table)
        {
            var clock = System.Diagnostics.Stopwatch.StartNew();
            Baked? baked = Bake(track, file, lanes, bends, corners, complex, solo, free, car);
            Console.WriteLine($"  corners {complex.First}-{complex.Last}: {(baked is null ? "no line drawn" : $"{baked.Place.Lines.Count} lines")}, " +
                $"{(complex.NoOvertake ? "no overtake" : "overtake")}, abreast {complex.MaxAbreast}, {clock.Elapsed.TotalSeconds:F0}s");
            if (baked is not null)
                places.Add(baked);
        }
        if (places.Count == 0)
            return 0;

        // What each straight is worth, measured once: the chaser held behind
        // through the place before, then its own pace in the tow, with and
        // without a push, behind a leader at each tier of its rears. The
        // defence reads this; without it a cover is decided on a straight
        // that closes nothing.
        foreach (Baked baked in places)
            MeasureStraight(track, file, lanes, solo, baked);

        // The sweep: the leader's rears, the chaser's push, how much margin
        // each asks for before it moves, how far off the truth the chaser
        // reads the leader's rears, and whether the leader reads the push
        // the wrong way round.
        var policies = new List<CornerBattle.Policy>();
        foreach (int tier in new[] { 0, 2, 3 })
            foreach (bool push in new[] { false, true })
                foreach (float nerve in new[] { 0f, 2f })
                    foreach (int error in new[] { 0, -1 })
                        foreach (bool misread in new[] { false, true })
                            policies.Add(new CornerBattle.Policy
                            {
                                Tier = tier, Push = push, AttackMargin = nerve, DefendWithin = nerve,
                                ReadError = error, MisreadsPush = misread, AutoAttack = true, AutoDefend = true,
                            });

        int passes = 0, covers = 0, contacts = 0, spins = 0;
        foreach (CornerBattle.Policy policy in policies)
        {
            List<Outcome> raced = Race(track, file, lanes, solo, places, free, policy);
            passes += raced.Count(o => o.Passed);
            covers += raced.Count(o => o.Verdict.Covered && !o.Passed);
            contacts += raced.Count(o => o.Contact);
            spins += raced.Count(o => o.Spins > 0);
            Console.WriteLine($"rears {CornerBattle.RearTiers[policy.Tier]:P0} push {policy.Push,-5} nerve {policy.AttackMargin:F0}m " +
                $"reads {(policy.ReadError == 0 ? "true" : "off"),-4} {(policy.MisreadsPush ? "misread" : "read"),-7}: " +
                string.Join("  ", raced.Select(o => $"{o.Place.Complex.First}:{(o.Passed ? "PASS" : o.Verdict.PulledOut ? "att" : "-")}" +
                    $"{(o.Verdict.Covered ? "/cov" : "")}{(o.Contact ? "!" : "")}")));
        }
        Console.WriteLine($"{policies.Count} races over {places.Count} places: {passes} passes, {covers} covers that held, {contacts} contacts, {spins} spins");
        return 0;
    }

    /// <summary>
    /// The lines through one place, and the tables of what each costs. Every
    /// pin of the place is held, at the inside, the middle or the outside,
    /// and a line that turns in later than it takes the apex, or tracks out
    /// later than it takes the apex, is not drawn: that is a line that crosses
    /// itself. A place of one corner has three pins and fourteen lines; a
    /// place of four has six and three hundred and twenty four, and that is
    /// what it is.
    /// </summary>
    private static Baked? Bake(TrackData track, LaneTrackFile file, LaneSet lanes, List<CornerMap.Bend> bends,
        List<Lap.Corner> corners, Lap.Complex complex, LaneLapResult solo, float[] free, ICar car)
    {
        var first = corners[complex.First].Pins;
        var last = corners[complex.Last].Pins;
        int claim = lanes.Wrap(first.Entry - CornerLines.BeforeStations);
        // The window ends where the next place turns in. The 200 m past an
        // exit is only there so a line can rejoin the free one, and the next
        // place's own claim already covers that road; drawing both over it
        // would put the car in two places at once.
        int nextEntry = corners[(complex.Last + 1) % corners.Count].Pins.Entry;
        int rejoins = lanes.Wrap(last.Exit + CornerLines.AfterStations);
        int windowEnd = lanes.Wrap(rejoins - nextEntry) < lanes.Wrap(nextEntry - rejoins) ? nextEntry : rejoins;
        if (lanes.Wrap(last.Exit - claim) > lanes.Wrap(windowEnd - claim))
        {
            Console.WriteLine($"    corners {complex.First}-{complex.Last}: the next place turns in at station {nextEntry}, before this one exits at {last.Exit}; not drawn.");
            return null;
        }
        int rows = (lanes.Wrap(windowEnd - claim) + 4) / 5;

        var sites = new List<(int Station, int Sign)> { (first.Entry, first.Sign) };
        for (int b = complex.First; b <= complex.Last; b++)
            sites.Add((corners[b].Pins.Apex, corners[b].Pins.Sign));
        sites.Add((last.Exit, last.Sign));
        var combos = new List<int[]>();
        for (int code = 0; code < (int)Math.Pow(3, sites.Count); code++)
        {
            var pins = new int[sites.Count];
            for (int i = 0, c = code; i < sites.Count; i++, c /= 3)
                pins[i] = c % 3;
            if (pins[0] >= pins[1] && pins[^1] >= pins[^2])
                combos.Add(pins);
        }

        var lines = new ConcurrentDictionary<string, float[]>();
        Parallel.ForEach(combos, combo =>
        {
            var held = sites.Select((s, i) => (s.Station, CornerLines.PlaceOffset(lanes, s.Sign, s.Station, combo[i]))).ToList();
            var way = JointLineBake.PinnedLine(track, lanes, car, claim, rows, solo.StationSpeed[claim], held);
            if (way is null)
                return;
            var line = (float[])free.Clone();
            for (int j = 0; j < way.Value.Offset.Length; j++)
                line[lanes.Wrap(claim + j)] = way.Value.Offset[j];
            lines[string.Concat(combo.Select(p => "内中外"[p]))] = line;
        });
        if (lines.IsEmpty)
            return null;

        // Tables from the straight before the place to past its exit, so a
        // decision at the defence point and one at the braking point read
        // the same run.
        int straightFrom = corners[(complex.First - 1 + corners.Count) % corners.Count].Pins.Exit;
        int tableFrom = lanes.Wrap(Math.Min(straightFrom, claim) - 20);
        int markCount = lanes.Wrap(windowEnd - tableFrom) + 1;
        float[] marks = Enumerable.Range(0, markCount).Select(j => lanes.Wrap(tableFrom + j) * lanes.Spacing).ToArray();
        var leaderTables = Enumerable.Range(0, CornerBattle.RearTiers.Length).Select(_ => new ConcurrentDictionary<string, CornerLines.Run>()).ToArray();
        var chaserTables = new ConcurrentDictionary<string, CornerLines.Run>();
        var jobs = lines.Keys.SelectMany(code => new (string code, int tier)[] { (code, -1) }
            .Concat(Enumerable.Range(0, CornerBattle.RearTiers.Length).Select(tier => (code, tier)))).ToList();
        Parallel.ForEach(jobs, job =>
        {
            ICar model = job.tier < 0 || job.tier == 0 ? StockCar.Build(DevelopmentTyres.SuperId) : Duel.CarWithRear(CornerBattle.RearTiers[job.tier]);
            var run = CornerLines.Drive(track, file, lanes, lines[job.code], 1f, marks, model);
            if (job.tier < 0)
                chaserTables[job.code] = run;
            else
                leaderTables[job.tier][job.code] = run;
        });

        string hold = lines.Keys.Where(c => c[0] == '外').OrderBy(c => c.Count(ch => ch == '外')).Last();
        int braking = claim;
        for (int i = claim; i != first.Entry; i = lanes.Wrap(i + 1))
            if (solo.StationSpeed[i] > solo.StationSpeed[braking])
                braking = i;
        int pullAt = lanes.Wrap(first.Entry - (int)MathF.Round(CornerBattle.PullSeconds * 85f / lanes.Spacing));
        int defendAt = lanes.Wrap(straightFrom + 40);

        // The leader's one move on the straight: from the outside place,
        // across early onto the inside, joining the inside line before the
        // turn-in. One such path per line that turns in from there, so it
        // can cover onto whichever of them its rears can drive.
        var coverPaths = new Dictionary<string, float[]>();
        foreach (string code in lines.Keys.Where(c => c[0] == '内'))
        {
            var cover = (float[])lines[code].Clone();
            int join = lanes.Wrap(first.Entry - 10);
            for (int i = defendAt; i != join; i = lanes.Wrap(i + 1))
            {
                float onto = CornerLines.PlaceOffset(lanes, first.Sign, i, 0);
                cover[i] = free[i] + (onto - free[i]) * CornerBattle.Ease(lanes.Wrap(i - defendAt) / 75f);
            }
            for (int i = join; i != first.Entry; i = lanes.Wrap(i + 1))
            {
                float onto = CornerLines.PlaceOffset(lanes, first.Sign, i, 0);
                cover[i] = onto + (lines[code][i] - onto) * CornerBattle.Ease(lanes.Wrap(i - join) / 10f);
            }
            coverPaths[code] = cover;
        }

        var place = new CornerBattle.Place
        {
            Lanes = lanes, Lines = lines, Free = free, HoldPath = lines[hold], Hold = hold, CoverPaths = coverPaths,
            LeaderTables = leaderTables.Select(AsRuns).ToArray(), ChaserTables = AsRuns(chaserTables),
            TableFrom = tableFrom, Claim = claim, Decide = lanes.Wrap(first.Entry - 5), Entry = first.Entry,
            Apex = last.Apex, Exit = last.Exit, WindowEnd = windowEnd, DefendAt = defendAt, PullAt = pullAt,
            Braking = braking, InsideDirection = -last.Sign,
        };
        return new Baked { Complex = complex, Place = place, NextClaim = lanes.Wrap(windowEnd + 10), StraightFrom = straightFrom };
    }

    /// <summary>
    /// The straight into a place, driven once at each tier and with and
    /// without a push. The chaser is held a tenth behind until the straight
    /// starts, then runs its own pace in the tow. The seconds it closes
    /// between the straight's start and the pull-out point is what the
    /// defence reads, so the defence is decided on a straight that has been
    /// driven, not on one assumed to close nothing.
    /// </summary>
    private static void MeasureStraight(TrackData track, LaneTrackFile file, LaneSet lanes, LaneLapResult solo, Baked baked)
    {
        CornerBattle.Place place = baked.Place;
        Console.Write($"  straight into {baked.Complex.First}-{baked.Complex.Last}, chaser closes");
        foreach (int tier in new[] { 0, 2, 3 })
        {
            foreach (bool push in new[] { false, true })
            {
                float[] gap = TimeTheStraight(track, file, lanes, solo, baked, tier, push);
                place.Gain[tier, push ? 1 : 0] = gap[0] - gap[1];
                Console.Write($"  {CornerBattle.RearTiers[tier]:P0} {(push ? "push" : "tow"),-4} {place.Gain[tier, push ? 1 : 0]:F2}s");
            }
        }
        place.Gain[1, 0] = (place.Gain[0, 0] + place.Gain[2, 0]) / 2f;
        place.Gain[1, 1] = (place.Gain[0, 1] + place.Gain[2, 1]) / 2f;
        Console.WriteLine();
    }

    /// <summary>Seconds the chaser is behind at the straight's start and at the pull-out point.</summary>
    private static float[] TimeTheStraight(TrackData track, LaneTrackFile file, LaneSet lanes, LaneLapResult solo, Baked baked, int tier, bool push)
    {
        CornerBattle.Place place = baked.Place;
        var simulation = new RaceSimulation(track);
        LaneTrackFile pathFile = file.WithLanes(lanes.WithOffsets(LaneId.Free, place.Free));
        int start = lanes.Wrap(baked.StraightFrom - 60);
        ICar leaderModel = tier == 0 ? StockCar.Build(DevelopmentTyres.SuperId) : Duel.CarWithRear(CornerBattle.RearTiers[tier]);
        var leaderDriver = new LaneDriver(pathFile, leaderModel) { LearnsAsItDrives = false };
        var held = new HeldBehind(new LaneDriver(pathFile, StockCar.Build(DevelopmentTyres.SuperId)) { LearnsAsItDrives = false }, lanes, baked.StraightFrom);
        RaceCar leader = Seat(simulation, "leader", leaderModel, leaderDriver, lanes, place.Free, start, solo.StationSpeed[start]);
        float body = leader.Car.Spec.Body?.LengthMeters ?? 4.8f;
        int behind = lanes.Wrap(start - (int)MathF.Round((body + 8f) / lanes.Spacing));
        RaceCar chaser = Seat(simulation, "chaser", StockCar.Build(DevelopmentTyres.SuperId), held, lanes, place.Free, behind, solo.StationSpeed[behind]);

        float length = track.LengthMeters, origin = start * lanes.Spacing;
        var u = new[] { origin, origin - lanes.Wrap(start - behind) * lanes.Spacing };
        var last = new[] { leader.Progress.CurrentS, chaser.Progress.CurrentS };
        int[] marks = [baked.StraightFrom, place.PullAt];
        float[] at = [float.NaN, float.NaN, float.NaN, float.NaN];
        const float dt = 1f / 60f;
        for (int step = 0; step < 60 * 40 && float.IsNaN(at[3]); step++)
        {
            int here = lanes.Wrap((int)lanes.StationOf(chaser.Progress.CurrentS));
            bool onStraight = lanes.Wrap(here - baked.StraightFrom) < lanes.Wrap(place.PullAt - baked.StraightFrom);
            chaser.Strategy = chaser.Strategy.WithButton(0, push && onStraight);
            simulation.Step(dt);
            float t = (step + 1) * dt;
            for (int x = 0; x < 2; x++)
            {
                float s = (x == 0 ? leader : chaser).Progress.CurrentS;
                u[x] += MathF.IEEERemainder(s - last[x], length);
                last[x] = s;
                for (int m = 0; m < marks.Length; m++)
                    if (float.IsNaN(at[x * 2 + m]) && lanes.Wrap((int)lanes.StationOf(s) - marks[m]) < 3)
                        at[x * 2 + m] = t;
            }
        }
        return [at[2] - at[0], at[3] - at[1]];
    }

    /// <summary>Holds a tenth of a second behind the car ahead until the straight starts, then drives its own pace.</summary>
    private sealed class HeldBehind(LaneDriver inner, LaneSet lanes, int releaseAt) : IDriverController
    {
        public CarNeeds Needs => ((IDriverController)inner).Needs;
        public void Initialize(in DriverContext context) => inner.Initialize(in context);

        public DriverInput GetControl(in DriverContext context, float dt)
        {
            DriverInput input = inner.GetControl(in context, dt);
            int here = lanes.Wrap((int)lanes.StationOf(context.Car.TrackS));
            if (lanes.Wrap(here - releaseAt) < lanes.Count / 2)
                return input;
            RaceCarSnapshot ahead = context.Frame[0];
            float gap = MathF.IEEERemainder(ahead.TrackS - context.Car.TrackS, lanes.LengthMeters) - context.Car.LengthMeters;
            float cap = 1.2f * (gap - 0.1f * context.Car.SpeedMetersPerSecond) + 1.8f * (ahead.SpeedMetersPerSecond - context.Car.SpeedMetersPerSecond);
            return cap < input.DesiredAccel ? input with { DesiredAccel = cap } : input;
        }
    }

    private static IReadOnlyDictionary<string, CornerBattle.Run> AsRuns(ConcurrentDictionary<string, CornerLines.Run> table) =>
        table.ToDictionary(kv => kv.Key, kv => new CornerBattle.Run(kv.Value.Times, kv.Value.Speeds, kv.Value.Lap.Spins));

    public sealed record Outcome(Baked Place, CornerBattle.Verdict Verdict, bool Passed, bool Contact, int Spins, float GapAtExit);

    /// <summary>
    /// The lap, once. The two start on the straight before the first place,
    /// the chaser a car's length and a little behind, and run until the last
    /// place is judged. The defence at each place reads the straight's gain
    /// measured before the sweep, not one timed during the race it decides.
    /// </summary>
    private static List<Outcome> Race(TrackData track, LaneTrackFile file, LaneSet lanes, LaneLapResult solo,
        List<Baked> places, float[] free, CornerBattle.Policy policy)
    {
        var simulation = new RaceSimulation(track);
        LaneTrackFile pathFile = file.WithLanes(lanes.WithOffsets(LaneId.Free, free));
        int start = lanes.Wrap(places[0].Place.Claim - 30);
        ICar leaderModel = policy.Tier == 0 ? StockCar.Build(DevelopmentTyres.SuperId) : Duel.CarWithRear(CornerBattle.RearTiers[policy.Tier]);
        var leaderDriver = new LaneDriver(pathFile, leaderModel) { LearnsAsItDrives = false };
        var chaserDriver = new LaneDriver(pathFile, StockCar.Build(DevelopmentTyres.SuperId)) { LearnsAsItDrives = false };
        var outcomes = new List<Outcome>();
        var referee = new Referee(places, policy, outcomes);
        RaceCar leader = Seat(simulation, "leader", leaderModel, new Switching(leaderDriver, referee, true), lanes, free, start, solo.StationSpeed[start]);
        float body = leader.Car.Spec.Body?.LengthMeters ?? 4.8f;
        int behind = lanes.Wrap(start - (int)MathF.Round((body + 8f) / lanes.Spacing));
        ICar chaserModel = StockCar.Build(DevelopmentTyres.SuperId);
        RaceCar chaser = Seat(simulation, "chaser", chaserModel, new Switching(chaserDriver, referee, false), lanes, free, behind, solo.StationSpeed[behind]);
        referee.Bind(leader, chaser, body, lanes);

        float length = track.LengthMeters;
        float origin = start * lanes.Spacing;
        var u = new[] { origin, origin - lanes.Wrap(start - behind) * lanes.Spacing };
        var last = new[] { leader.Progress.CurrentS, chaser.Progress.CurrentS };
        const float dt = 1f / 60f;
        for (int step = 0; step < 60 * 400 && referee.Open; step++)
        {
            int here = lanes.Wrap((int)lanes.StationOf(chaser.Progress.CurrentS));
            Baked? current = referee.Current;
            chaser.Strategy = chaser.Strategy.WithButton(0, policy.Push && current is not null
                && lanes.Wrap(here - current.StraightFrom) < lanes.Wrap(current.Place.Braking - current.StraightFrom));
            simulation.Step(dt);
            for (int x = 0; x < 2; x++)
            {
                RaceCar car = x == 0 ? leader : chaser;
                float s = car.Progress.CurrentS;
                u[x] += MathF.IEEERemainder(s - last[x], length);
                last[x] = s;
            }
            referee.Mark(u[0] - u[1], MathF.Abs(leader.Progress.CurrentD - chaser.Progress.CurrentD), (step + 1) * dt);
        }
        return outcomes;
    }

    private static RaceCar Seat(RaceSimulation simulation, string name, ICar model, IDriverController controller, LaneSet lanes, float[] path, int station, float speed)
    {
        System.Numerics.Vector2 here = lanes.Position(station, path[station]);
        System.Numerics.Vector2 next = lanes.Position(station + 1, path[lanes.Wrap(station + 1)]);
        var car = new RaceCar(name, model, new Driver(new DriverProfile(name, new DriverAbilities()), controller), new CarState
        {
            Position = here,
            Heading = MathF.Atan2(next.Y - here.Y, next.X - here.X),
            Speed = speed,
        }) { Strategy = CarStrategy.Default };
        ((ICarTires)model).FitTires(car.State, DevelopmentTyres.SuperId);
        ((ICarStores)model).Fill(car.State, PowertrainState.Filled(0.8f));
        ((ICarTires)model).WarmTires(car.State, 90f, 90f);
        simulation.AddCar(car);
        return car;
    }

    /// <summary>
    /// One decision shared by both cars at the place they are in. It owns the
    /// verdict; the two drivers only ask it what was decided and drive it.
    /// </summary>
    private sealed class Referee(List<Baked> places, CornerBattle.Policy policy, List<Outcome> outcomes)
    {
        private int _at;
        private float _closest = float.PositiveInfinity, _gapAtExit = float.NaN;
        private bool _contact;
        private int _spins;
        public Baked? Current => _at < places.Count ? places[_at] : null;
        public bool Open => _at < places.Count;
        public CornerBattle.Policy Policy => policy;
        public CornerBattle.Verdict Verdict { get; } = new();
        private RaceCar _leader = null!, _chaser = null!;
        private LaneSet _lanes = null!;
        private float _body;

        public void Bind(RaceCar leader, RaceCar chaser, float body, LaneSet lanes)
        {
            _leader = leader;
            _chaser = chaser;
            _body = body;
            _lanes = lanes;
        }

        /// <summary>The gap and the room, read after each step.</summary>
        public void Mark(float along, float across, float time)
        {
            Baked? baked = Current;
            if (baked is null)
                return;
            if (MathF.Abs(along) < _body)
            {
                _closest = MathF.Min(_closest, across);
                if (across < _leader.Car.Spec.Body?.WidthMeters)
                    _contact = true;
            }
            int leaderAt = _lanes.Wrap((int)_lanes.StationOf(_leader.Progress.CurrentS));
            if (float.IsNaN(_gapAtExit) && leaderAt == baked.Place.Exit)
                _gapAtExit = along - _body;
        }

        /// <summary>The place is over once its apex has been given and the leader has reached the exit.</summary>
        public void Advance(int station)
        {
            Baked? baked = Current;
            if (baked is null || !Verdict.Decided || _lanes.Wrap(station - baked.Place.Exit) >= 5)
                return;
            bool passed = Verdict.PulledOut && Verdict.Along < 0f;
            outcomes.Add(new Outcome(baked, Verdict, passed, _contact, _spins, _gapAtExit));
            _at++;
            Verdict.Defended = Verdict.Considered = Verdict.Decided = Verdict.PulledOut = Verdict.Earned = Verdict.Covered = Verdict.AttackerTucks = false;
            Verdict.Threat = float.NegativeInfinity;
            Verdict.Read = float.NegativeInfinity;
            Verdict.Along = float.NaN;
            Verdict.PulledAt = -1;
            Verdict.Attack = Verdict.AttackerLine = null;
            _closest = float.PositiveInfinity;
            _gapAtExit = float.NaN;
            _contact = false;
            _spins = 0;
            if (Current is { } next)
            {
                Verdict.LeaderLine = next.Place.Hold;
                Verdict.LeaderTarget = next.Place.HoldPath;
            }
        }
    }

    /// <summary>
    /// A driver that follows the free line except through a place, where the
    /// decision moves it. The decision is the same one the single corner
    /// uses; this only asks it at the station it is due and drives what it
    /// says.
    /// </summary>
    private sealed class Switching(LaneDriver inner, Referee referee, bool leader) : IDriverController
    {
        private bool _moved, _yielded;
        public CarNeeds Needs => ((IDriverController)inner).Needs;
        public void Initialize(in DriverContext context) => inner.Initialize(in context);

        public DriverInput GetControl(in DriverContext context, float dt)
        {
            DriverInput input = inner.GetControl(in context, dt);
            Baked? baked = referee.Current;
            if (baked is null)
                return input;
            CornerBattle.Place place = baked.Place;
            LaneSet lanes = place.Lanes;
            int station = lanes.Wrap((int)lanes.StationOf(context.Car.TrackS));
            if (leader)
                referee.Advance(station);
            baked = referee.Current;
            if (baked is null)
                return input;
            place = baked.Place;
            if (lanes.Wrap(station - place.Claim) > lanes.Wrap(place.WindowEnd - place.Claim))
            {
                _moved = _yielded = false;
                return input;
            }
            RaceCarSnapshot me = context.Car;
            RaceCarSnapshot other = context.Frame[leader ? 1 : 0];
            float body = me.LengthMeters;
            CornerBattle.Verdict verdict = referee.Verdict;

            if (leader && !verdict.Defended && lanes.Wrap(station - place.DefendAt) < 5)
            {
                float behind = MathF.IEEERemainder(me.TrackS - other.TrackS, lanes.LengthMeters);
                if (!baked.Complex.NoOvertake && CornerBattle.Defend(place, referee.Policy, verdict, me, MathF.Max(0f, behind)) && verdict.LeaderTarget is { } cover)
                    inner.DriveLineNow(station, Move(lanes, place.Free, station, me.TrackD, 10, cover, place.WindowEnd));
                else if (verdict.LeaderLine.Length == 0)
                {
                    verdict.LeaderLine = place.Hold;
                    verdict.LeaderTarget = place.HoldPath;
                }
            }

            if (leader && !verdict.Decided && lanes.Wrap(station - place.Decide) < 5)
            {
                CornerBattle.Judge(place, referee.Policy, verdict, me, other, body);
                if (verdict.LeaderTarget is { } onto && verdict.LeaderLine != place.Hold)
                    inner.DriveLineNow(station, Move(lanes, place.Free, station, me.TrackD, 15, onto, place.WindowEnd));
            }

            if (!leader && !_moved)
            {
                float toEntry = lanes.Wrap(place.Entry - station) * lanes.Spacing;
                if (toEntry < lanes.LengthMeters / 2f && toEntry <= CornerBattle.PullSeconds * me.SpeedMetersPerSecond)
                {
                    _moved = true;
                    if (!baked.Complex.NoOvertake && CornerBattle.Choose(place, referee.Policy, verdict, me, other, body) is { } attack)
                        inner.DriveLineNow(station, Move(lanes, place.Free, station, me.TrackD, lanes.Wrap(place.Entry - station), place.Lines[attack], place.WindowEnd));
                }
            }

            // Without the apex: beside the leader on the line that leaves it
            // room, or in behind it. With the apex earned and the leader
            // moved aside, stay on the attack.
            if (!leader && !_yielded && verdict.Decided)
            {
                if (verdict.AttackerLine is { } beside && beside != verdict.Attack)
                {
                    _yielded = true;
                    inner.DriveLineNow(station, Move(lanes, place.Free, station, me.TrackD, 15, place.Lines[beside], place.WindowEnd));
                }
                else if (verdict.AttackerTucks && verdict.LeaderTarget is { } behind)
                {
                    _yielded = true;
                    inner.DriveLineNow(station, Move(lanes, place.Free, station, me.TrackD, 20, behind, place.WindowEnd));
                }
            }
            return input;
        }

        private static float[] Move(LaneSet lanes, float[] path, int from, float offsetNow, int over, float[] target, int end)
        {
            int count = Math.Max(1, lanes.Wrap(end - from));
            var relative = new float[count];
            float held = offsetNow - path[from];
            for (int j = 0; j < count; j++)
            {
                int i = lanes.Wrap(from + j);
                float now = path[i] + held;
                float there = j < over ? now + (target[i] - now) * CornerBattle.Ease(j / (float)Math.Max(over, 1)) : target[i];
                relative[j] = there - path[i];
            }
            return relative;
        }
    }
}
