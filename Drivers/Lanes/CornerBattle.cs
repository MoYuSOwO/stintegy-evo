using StintegyEVO.Core.Racing;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// The attack and the defence at one place, decided from tables baked
/// offline. The tables say what each line costs alone, at every tier of the
/// leader's rear grip; the decision reads those and nothing else, because a
/// driver in a race cannot run the other car's lap to find out.
/// </summary>
/// <remarks>
/// <para>
/// A place is the stretch one move covers: the turn-in of its first corner
/// to the track-out of its last, with the apex it is judged at. The lines
/// through it are named by the place each pin is held at, inside, middle or
/// outside, in order along the road. A line's first letter is where it turns
/// in, and that is the place a car is already in when it chooses.
/// </para>
/// <para>
/// The apex is given the way the driving standards give it. The car whose
/// front axle reaches the other's mirrors at the apex has earned it; the car
/// without it takes a line that leaves the other a car's width wherever the
/// two are alongside, or tucks in behind. Both read the other's line and
/// pace from the tables, and either may read them wrong.
/// </para>
/// </remarks>
public static class CornerBattle
{
    /// <summary>
    /// From a car's front axle back to its mirrors. The guidelines judge an
    /// inside pass on the attacker's front axle reaching the defender's
    /// mirrors at the apex; where the mirrors sit is the car's.
    /// </summary>
    public const float AxleToMirror = 1.8f;

    /// <summary>The least, centre to centre, two lines may come and both be driven side by side: a car's width and a little apart.</summary>
    public const float Room = 2.6f;

    /// <summary>How long a pull-out takes: one quick move across, about two seconds.</summary>
    public const float PullSeconds = 2f;

    /// <summary>The leader's rear grip in each tier a rival may read it in, as a share of new. Tier 0 is new.</summary>
    public static readonly float[] RearTiers = [1f, 0.95f, 0.9f, 0.85f];

    /// <summary>A smoothstep, held at its ends.</summary>
    public static float Ease(float x)
    {
        x = Math.Clamp(x, 0f, 1f);
        return x * x * (3f - 2f * x);
    }

    /// <summary>
    /// What each car will do. The leader holds the place it is in, covers the
    /// inside once at the defence point, or decides there: it covers when the
    /// chaser's best attack, as it reads it, would earn the apex within
    /// <see cref="DefendWithin"/> metres. The chaser attacks on a given line,
    /// or decides about <see cref="PullSeconds"/> before the turn-in: it goes
    /// when its best line would earn the apex by <see cref="AttackMargin"/>
    /// metres or more, as it reads the leader. It reads the leader's rears
    /// <see cref="ReadError"/> tiers off the truth, and the leader may read
    /// the chaser's push the wrong way round.
    /// </summary>
    public sealed record Policy
    {
        /// <summary>The line the chaser attacks on, when it does not choose; none when it follows.</summary>
        public string? Attack { get; init; }
        public bool AutoAttack { get; init; }
        /// <summary>Metres by which an attack must earn the apex before the chaser takes it.</summary>
        public float AttackMargin { get; init; }
        public bool CoverAlways { get; init; }
        public bool AutoDefend { get; init; }
        /// <summary>Metres within which the chaser's best attack, as the leader reads it, makes the leader cover.</summary>
        public float DefendWithin { get; init; }
        public bool Push { get; init; } = true;
        /// <summary>The tier of the leader's rear grip, into <see cref="RearTiers"/>.</summary>
        public int Tier { get; init; }
        /// <summary>How many tiers off the truth the chaser reads the leader's rears.</summary>
        public int ReadError { get; init; }
        /// <summary>The leader reads the chaser's push the wrong way round.</summary>
        public bool MisreadsPush { get; init; }

        /// <summary>The tier the chaser's reading of the leader falls in.</summary>
        public int ReadTier => Math.Clamp(Tier + ReadError, 0, RearTiers.Length - 1);
    }

    /// <summary>
    /// One place, with the lines through it and the tables of what each costs
    /// alone. The tables are timed from <see cref="TableFrom"/>, one sample a
    /// station, in order along the road.
    /// </summary>
    public sealed class Place
    {
        public required LaneSet Lanes { get; init; }

        /// <summary>The line through the place, one offset from the centreline per station, by the code that names its pins.</summary>
        public required IReadOnlyDictionary<string, float[]> Lines { get; init; }

        /// <summary>The free line, one offset from the centreline per station.</summary>
        public required float[] Free { get; init; }

        /// <summary>The line the leader holds when it does not cover: the one it was already on.</summary>
        public required float[] HoldPath { get; init; }

        /// <summary>The code of <see cref="HoldPath"/>, so a reading can name the line the leader is on.</summary>
        public required string Hold { get; init; }

        /// <summary>The leader's path covering the inside onto each line that turns in from there.</summary>
        public required IReadOnlyDictionary<string, float[]> CoverPaths { get; init; }

        /// <summary>The leader's timetable on each line, one set per tier of its rears.</summary>
        public required IReadOnlyList<IReadOnlyDictionary<string, Run>> LeaderTables { get; init; }

        /// <summary>The chaser's timetable on each line, on fresh tyres.</summary>
        public required IReadOnlyDictionary<string, Run> ChaserTables { get; init; }

        /// <summary>Seconds the chaser closes in the tow from the straight's start to the pull-out point: by leader tier, without and with a push.</summary>
        public float[,] Gain { get; } = new float[RearTiers.Length, 2];

        /// <summary>The station the tables' first sample is timed at.</summary>
        public required int TableFrom { get; init; }
        public required int Claim, Decide, Entry, Apex, Exit, WindowEnd, DefendAt, PullAt, Braking, InsideDirection;

        /// <summary>The leader's fastest line turning in from the inside that it can drive on its rears, or none.</summary>
        public string? CoverLine(int tier) => CoverPaths.Keys.Where(code => LeaderCan(tier, code))
            .OrderBy(code => LeaderSeconds(tier, code, Decide, Apex)).FirstOrDefault();

        /// <summary>Seconds on a line between two stations, interpolated from its timetable.</summary>
        public float LeaderSeconds(int tier, string code, float from, float to) => At(LeaderTables[tier][code], to) - At(LeaderTables[tier][code], from);
        public float ChaserSeconds(string code, float from, float to) => At(ChaserTables[code], to) - At(ChaserTables[code], from);
        public float ChaserSpeed(string code, int station) => Speed(ChaserTables[code], station);
        public float LeaderSpeed(int tier, string code, int station) => Speed(LeaderTables[tier][code], station);

        public bool LeaderCan(int tier, string code) => LeaderTables[tier].TryGetValue(code, out Run run) && Drives(run);
        public bool ChaserCan(string code) => ChaserTables.TryGetValue(code, out Run run) && Drives(run);

        /// <summary>The leader's lines that turn in from where it already is.</summary>
        public IEnumerable<string> LeaderOptions(int tier, string line) => Lines.Keys.Where(c => c[0] == line[0] && LeaderCan(tier, c));

        /// <summary>The chaser's lines that turn in from somewhere else than the leader does.</summary>
        public IEnumerable<string> Attacks(string leaderLine) => Lines.Keys.Where(c => c[0] != leaderLine[0] && ChaserCan(c));

        /// <summary>Whether a line takes its apex inside of the leader's path.</summary>
        public bool Inside(string attack, float[] leaderPath) => (Lines[attack][Apex] - leaderPath[Apex]) * InsideDirection > 0f;

        /// <summary>
        /// How far behind the leader's centre the chaser's will be at the
        /// apex, each on its line from where it is, driven as alone, less
        /// whatever either carries above its alone speed now, held to the
        /// braking point. Positive means the chaser arrives later.
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

        /// <summary>Metres by which an attack would earn the apex: positive earns it.</summary>
        public float Margin(string attack, float[] leaderPath, float along) => (Inside(attack, leaderPath) ? AxleToMirror : 0f) - along;

        /// <summary>
        /// How close across the road the two come where they are alongside:
        /// each driven as alone from where it is now, over the stations from
        /// the turn-in to past the exit where the two are within a car's
        /// length of each other. Infinite where they are never alongside.
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
                .Select(o => (Code: o, Room: Room(tier, o, mine, attack, theirs, body), Seconds: LeaderSeconds(tier, o, mine, Apex)))
                .Where(o => o.Room >= CornerBattle.Room)
                .OrderBy(o => o.Seconds)
                .Select(o => o.Code)
                .FirstOrDefault();

        private float At(Run run, float station)
        {
            float j = Lanes.Wrap((int)MathF.Floor(station) - TableFrom) + (station - MathF.Floor(station));
            int a = Math.Clamp((int)j, 0, run.Times.Length - 1), b = Math.Min(a + 1, run.Times.Length - 1);
            return run.Times[a] + (run.Times[b] - run.Times[a]) * (j - a);
        }

        private float Speed(Run run, int station) => run.Speeds[Math.Clamp(Lanes.Wrap(station - TableFrom), 0, run.Speeds.Length - 1)];

        /// <summary>Driven alone without a spin, and timed in order at every station.</summary>
        private static bool Drives(Run run)
        {
            if (run.Spins != 0 || run.Times.Length == 0 || float.IsNaN(run.Times[0]))
                return false;
            for (int i = 1; i < run.Times.Length; i++)
                if (!(run.Times[i] > run.Times[i - 1]))
                    return false;
            return true;
        }
    }

    /// <summary>One line driven alone: the time and the speed at each station from the table's start, and the spins it took.</summary>
    public readonly record struct Run(float[] Times, float[] Speeds, int Spins);

    /// <summary>What was decided at a place, and what came of it.</summary>
    public sealed class Verdict
    {
        public bool Defended, Considered, Decided, PulledOut, Earned, Covered, AttackerTucks;
        /// <summary>Metres by which the chaser's best attack would earn the apex, as the leader reads it from the defence point.</summary>
        public float Threat = float.NegativeInfinity;
        /// <summary>Metres by which the chaser's best attack would earn the apex, as the chaser reads it.</summary>
        public float Read = float.NegativeInfinity;
        /// <summary>How far behind the leader the chaser is at the apex, as the tables give it.</summary>
        public float Along = float.NaN;
        public int PulledAt = -1;
        /// <summary>The line the leader ends on, and the one the attacker moves to when it cannot stay on its attack.</summary>
        public string LeaderLine = "";
        public string? Attack, AttackerLine;
        public float[]? LeaderTarget;
    }

    /// <summary>
    /// The leader's one move on the straight, taken once as it passes the
    /// defence point. It covers the inside when it always does, or when the
    /// chaser's best attack, read from the gap and the straight's gain table,
    /// would earn the apex within the margin its nerve allows. The cover is
    /// onto the inside line its rears can drive. Returns whether it moved.
    /// </summary>
    public static bool Defend(Place place, Policy policy, Verdict verdict, RaceCarSnapshot me, float metresBehind)
    {
        verdict.Defended = true;
        bool cover = policy.CoverAlways;
        if (!cover && policy.AutoDefend)
        {
            verdict.Threat = Threat(place, policy, me.SpeedMetersPerSecond, metresBehind);
            cover = verdict.Threat >= -policy.DefendWithin;
        }
        if (cover && place.CoverLine(policy.Tier) is { } onto)
        {
            verdict.Covered = true;
            verdict.LeaderLine = onto;
            verdict.LeaderTarget = place.CoverPaths[onto];
            return true;
        }
        return false;
    }

    /// <summary>
    /// The apex, given as the guidelines give it, from where the two are as
    /// the leader reaches the braking point. The leader reads itself as it
    /// truly is. Where the attack earns the apex, the leader moves to the
    /// fastest line from where it is that leaves the attack room. Where it
    /// does not, the attacker keeps beside it on a line that leaves it its
    /// own, or tucks in behind. An attack the leader could not leave room for
    /// is no attack it must give way to, and that is judged here too.
    /// </summary>
    public static void Judge(Place place, Policy policy, Verdict verdict, RaceCarSnapshot me, RaceCarSnapshot other, float body)
    {
        verdict.Decided = true;
        if (!verdict.PulledOut || verdict.Attack is not { } attack)
            return;
        LaneSet lanes = place.Lanes;
        string line = verdict.LeaderLine;
        float[] path = verdict.LeaderTarget ?? place.HoldPath;
        float mine = lanes.StationOf(me.TrackS), theirs = lanes.StationOf(other.TrackS);
        float along = place.Along(policy.Tier, line, mine, me.SpeedMetersPerSecond, attack, theirs, other.SpeedMetersPerSecond);
        verdict.Along = along;
        verdict.Earned = place.Margin(attack, path, along) >= 0f;
        string? pick = verdict.Earned ? place.LeavesRoom(policy.Tier, line, mine, attack, theirs, body) : null;
        if (pick is not null)
        {
            verdict.LeaderLine = pick;
            verdict.LeaderTarget = place.Lines[pick];
        }
        else
        {
            verdict.AttackerLine = Alongside(place, policy, line, attack, mine, theirs, body);
            verdict.AttackerTucks = verdict.AttackerLine is null;
        }
    }

    /// <summary>
    /// The chaser's line, chosen once about two seconds before the turn-in:
    /// the one that earns the apex by the most, as it reads the leader, taken
    /// when that earns by the margin its nerve asks for and the leader could
    /// leave it room. None when it follows.
    /// </summary>
    public static string? Choose(Place place, Policy policy, Verdict verdict, RaceCarSnapshot me, RaceCarSnapshot ahead, float body)
    {
        verdict.Considered = true;
        string? attack = policy.AutoAttack ? Best(place, policy, verdict, me, ahead, body) : policy.Attack;
        if (attack is null)
            return null;
        verdict.Attack = attack;
        verdict.PulledOut = true;
        verdict.PulledAt = place.Lanes.Wrap((int)place.Lanes.StationOf(me.TrackS));
        return attack;
    }

    /// <summary>
    /// The chaser's best attack against the line the leader is on, as the
    /// leader reads it from the defence point: the chaser where it is now,
    /// less what the tow and a push close on the straight to the pull-out
    /// point, then each attack line against the line the leader holds.
    /// Metres by which it would earn the apex; negative infinity where none
    /// would, or where every one that would is an attack the leader could
    /// not leave room for.
    /// </summary>
    private static float Threat(Place place, Policy policy, float mySpeed, float metresBehind)
    {
        LaneSet lanes = place.Lanes;
        string held = place.Hold;
        float seconds = metresBehind / MathF.Max(mySpeed, 1f);
        float atPull = MathF.Max(0f, seconds - place.Gain[policy.Tier, policy.Push != policy.MisreadsPush ? 1 : 0]);
        float metres = atPull * place.LeaderSpeed(policy.Tier, held, place.PullAt);
        float chaserAt = lanes.Wrap((int)MathF.Round(place.PullAt - metres / lanes.Spacing));
        float best = float.NegativeInfinity;
        foreach (string a in place.Attacks(held))
        {
            float along = place.Along(policy.Tier, held, place.PullAt, place.LeaderSpeed(policy.Tier, held, place.PullAt), a, chaserAt, place.ChaserSpeed(a, (int)chaserAt));
            if (place.Margin(a, place.HoldPath, along) >= 0f && place.LeavesRoom(policy.Tier, held, place.PullAt, a, chaserAt, 4.8f) is null)
                continue;
            best = MathF.Max(best, place.Margin(a, place.HoldPath, along));
        }
        return best;
    }

    private static string? Best(Place place, Policy policy, Verdict verdict, RaceCarSnapshot me, RaceCarSnapshot ahead, float body)
    {
        LaneSet lanes = place.Lanes;
        string line = verdict.LeaderLine;
        float[] path = verdict.LeaderTarget ?? place.HoldPath;
        float mine = lanes.StationOf(me.TrackS), theirs = lanes.StationOf(ahead.TrackS);
        string? best = null;
        float bestMargin = float.NegativeInfinity;
        foreach (string a in place.Attacks(line))
        {
            float margin = place.Margin(a, path, place.Along(policy.ReadTier, line, theirs, ahead.SpeedMetersPerSecond, a, mine, me.SpeedMetersPerSecond));
            if (place.LeavesRoom(policy.ReadTier, line, theirs, a, mine, body) is null)
                continue;
            if (margin > bestMargin)
                (best, bestMargin) = (a, margin);
        }
        verdict.Read = bestMargin;
        return bestMargin >= policy.AttackMargin ? best : null;
    }

    /// <summary>The attacker's fastest line from where it already is that leaves the leader a car's width, or none.</summary>
    private static string? Alongside(Place place, Policy policy, string leaderLine, string attack, float leaderAt, float chaserAt, float body) =>
        place.Lines.Keys
            .Where(a => a[0] == attack[0] && place.ChaserCan(a))
            .Select(a => (Code: a, Room: place.Room(policy.Tier, leaderLine, leaderAt, a, chaserAt, body), Seconds: place.ChaserSeconds(a, chaserAt, place.Apex)))
            .Where(a => a.Room >= Room)
            .OrderBy(a => a.Seconds)
            .Select(a => a.Code)
            .FirstOrDefault();
}
