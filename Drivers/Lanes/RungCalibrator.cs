using System.Globalization;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// Sets one circuit's energy and tyre rungs so that each does on this
/// circuit what its name says, by driving it.
///
/// The targets are the player's, not the driver's: the driver only carries
/// out the rung it is given.
/// <list type="bullet">
/// <item>Energy: Normal, from a full pack, spends it over the race distance
/// and no faster. The five rungs span at most one per cent of a lap.</item>
/// <item>Tyres: the five rungs span at most one per cent of a lap; Attack
/// wears about 1.4 times Normal and Protect about 0.7, so a race of Attack
/// costs about one more stop than a race of Protect.</item>
/// </list>
/// Where a circuit cannot meet a target with driving alone (a pack the
/// generator cannot keep up with even lifting everywhere, a tyre that will
/// not wear less however gently it is driven) the result says so rather
/// than pretending: that is the car's business, not the driver's.
/// </summary>
public sealed class RungCalibrator
{
    /// <summary>The widest the five rungs may spread, as a share of a lap.</summary>
    public float Spread { get; init; } = 0.01f;

    /// <summary>The share of the pack Normal spends over a race: all of it, short of the last few per cent where it sags.</summary>
    public float UsablePack { get; init; } = 0.95f;

    /// <summary>Wear of each tyre rung, Protect to Attack, relative to Normal.</summary>
    public float[] WearRatios { get; init; } = [0.7f, 0.837f, 1f, 1.183f, 1.4f];

    /// <summary>Flying laps measured per setting, after an out lap and one lap to settle.</summary>
    public int MeasuredLaps { get; init; } = 3;

    public float[] Lifts { get; init; } = [0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f, 1.25f, 1.5f];

    /// <summary>How gently to drive, from the plan as learned (0) to twice as far below it as the general Protect (1).</summary>
    public float[] Gentleness { get; init; } = [0f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, 1f];

    public static readonly TyreStyle AsLearned = new(1f, 1f, 0.9f, 1f);
    public static readonly TyreStyle Gentlest = new(0.9462f, 0.9624f, 0.7710f, 0.9874f);

    public static TyreStyle Style(float gentleness) => new(
        Lerp(AsLearned.Corner, Gentlest.Corner, gentleness),
        Lerp(AsLearned.Brake, Gentlest.Brake, gentleness),
        Lerp(AsLearned.Traction, Gentlest.Traction, gentleness),
        Lerp(AsLearned.Cap, Gentlest.Cap, gentleness));

    /// <summary>The fewest laps that cover a Grand Prix: 305 km, or 260 at Monaco.</summary>
    public static int RaceLaps(string id, TrackData track)
    {
        float distance = id == "monaco" ? 260_000f : 305_000f;
        return (int)MathF.Ceiling(distance / track.LengthMeters);
    }

    public readonly record struct Sample(float Setting, float LapSeconds, float PackPerLap, float WearPerLap, int Spins, float OffSeconds);

    public sealed record Result(
        LaneEnergyRungs Energy,
        LaneTyreRungs Tyres,
        LaneCalibration Notes,
        IReadOnlyList<Sample> EnergySweep,
        IReadOnlyList<Sample> TyreSweep);

    public Result Calibrate(string id, TrackData track, LaneTrackFile shipped, Action<string>? log = null)
    {
        LaneTrackFile file = shipped.Clone();
        file.Energy = null;
        file.Tyres = null;
        var gains = new LaneDriverGains { TurnAwareEnvelope = true };
        int raceLaps = RaceLaps(id, track);

        var energy = new List<Sample>();
        foreach (float lift in Lifts)
        {
            Sample s = Measure(track, file, gains, new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Normal), lift, lift);
            energy.Add(s);
            log?.Invoke($"{id} lift {lift:F2}s  lap {s.LapSeconds:F2}  pack {s.PackPerLap * 100f:+0.00;-0.00}%/lap  spins {s.Spins} off {s.OffSeconds:F2}");
        }
        (float[] lifts, string energyNote) = SolveEnergy(energy, raceLaps);

        var tyres = new List<Sample>();
        foreach (float g in Gentleness)
        {
            TyreStyle style = Style(g);
            var styled = new LaneDriverGains { TurnAwareEnvelope = true, TyreStyleTable = [style, style, style, style, style] };
            Sample s = Measure(track, file, styled, new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Normal), null, g);
            tyres.Add(s);
            log?.Invoke($"{id} gentleness {g:F3}  lap {s.LapSeconds:F2}  wear {s.WearPerLap * 100f:F3}%/lap  spins {s.Spins} off {s.OffSeconds:F2}");
        }
        (float[] gentleness, string tyreNote) = SolveTyres(tyres);

        var notes = new LaneCalibration
        {
            Date = DateTime.Now.ToString("yyyy-MM-dd", CultureInfo.InvariantCulture),
            Physics = StockCar.Loaded.Fingerprint,
            Energy = energyNote,
            Tyres = tyreNote
        };
        return new Result(
            LaneEnergyRungs.From(lifts, raceLaps),
            LaneTyreRungs.From(gentleness.Select(Style).ToArray()),
            notes, energy, tyres);
    }

    /// <summary>
    /// Lift seconds, Save to Attack. Normal is where the pack lasts the race;
    /// the others are spaced evenly in lap time around it, a quarter of the
    /// spread apart, Attack no harder than not lifting at all.
    /// </summary>
    public (float[] Lifts, string Note) SolveEnergy(IReadOnlyList<Sample> sweep, int raceLaps)
    {
        float target = UsablePack / raceLaps;
        float[] lift = sweep.Select(s => s.Setting).ToArray();
        float[] drain = sweep.Select(s => s.PackPerLap).ToArray();
        float[] time = sweep.Select(s => s.LapSeconds).ToArray();
        float fastest = time[0];
        float step = Spread * fastest / 4f;
        string note;
        float normal;
        if (drain[0] <= target)
        {
            normal = 0f;
            note = $"surplus: flat out spends {drain[0] * raceLaps * 100f:F0}% of the pack over {raceLaps} laps; Normal does not need to lift";
        }
        else if (drain[^1] > target)
        {
            normal = lift[^1];
            note = $"short: even lifting {lift[^1]:F2} s spends {drain[^1] * raceLaps * 100f:F0}% of the pack over {raceLaps} laps; the generator cannot keep up";
        }
        else
        {
            normal = Inverse(lift, drain, target);
            note = $"Normal lifts {normal:F2} s and spends {UsablePack * 100f:F0}% of the pack over {raceLaps} laps";
        }
        float tNormal = Interpolate(lift, time, normal);
        // Attack and Push sit faster than Normal, no faster than not lifting.
        float faster = MathF.Min(step, (tNormal - fastest) / 2f);
        float[] targets = [tNormal + 2f * step, tNormal + step, tNormal, tNormal - faster, tNormal - 2f * faster];
        float[] result = targets.Select(t => Math.Clamp(Inverse(lift, time, t), 0f, lift[^1])).ToArray();
        result[2] = normal;
        float spread = (Interpolate(lift, time, result[0]) - Interpolate(lift, time, result[4])) / fastest;
        note += $"; Save to Attack {spread * 100f:F2}% of a lap";
        return (result, note);
    }

    /// <summary>
    /// Gentleness, Protect to Attack. Attack is the plan as learned; the
    /// others wear at the target ratios to Normal, unless that would spread
    /// the rungs past the allowed share of a lap, in which case the gentle
    /// end stops there and the note says what it reached.
    /// </summary>
    public (float[] Gentleness, string Note) SolveTyres(IReadOnlyList<Sample> sweep)
    {
        float[] g = sweep.Select(s => s.Setting).ToArray();
        float[] wear = sweep.Select(s => s.WearPerLap).ToArray();
        float[] time = sweep.Select(s => s.LapSeconds).ToArray();
        float attackWear = wear[0];
        float normalWear = attackWear / WearRatios[4];
        float[] result = WearRatios.Select(r => Math.Clamp(Inverse(g, wear, normalWear * r), 0f, g[^1])).ToArray();
        result[4] = 0f;
        string note = "";
        float limit = time[0] * (1f + Spread);
        if (Interpolate(g, time, result[0]) > limit)
        {
            float most = Math.Clamp(Inverse(g, time, limit), 0f, g[^1]);
            float scale = most / MathF.Max(result[0], 1e-6f);
            result = result.Select(x => x * scale).ToArray();
            note = "the wear targets would spread past a lap's share, so the gentle end stops there; ";
        }
        float n = Interpolate(g, wear, result[2]);
        float spread = (Interpolate(g, time, result[0]) - time[0]) / time[0];
        note += $"wear to Normal: Protect {Interpolate(g, wear, result[0]) / n:F2}, Attack {attackWear / n:F2}; Protect to Attack {spread * 100f:F2}% of a lap";
        if (Interpolate(g, wear, g[^1]) > normalWear * WearRatios[0] && result[0] >= g[^1])
            note += "; the gentlest style still wears more than the Protect target";
        return (result, note);
    }

    private Sample Measure(TrackData track, LaneTrackFile file, LaneDriverGains gains, CarStrategy strategy, float? lift, float setting)
    {
        var crossings = new List<(float T, float Pack, float Wear)>();
        int lastLap = int.MinValue;
        LaneLapResult r = LaneLapRunner.Run(track, file, gains, flyingLaps: MeasuredLaps + 1, startingCharge: 0.6f, strategy: strategy,
            onStep: (car, d, t) =>
            {
                if (lift is { } l)
                    d.CoastOverride = l;
                if (car.Progress.Lap == lastLap)
                    return;
                lastLap = car.Progress.Lap;
                CarState s = car.State;
                float wear = (s.Chassis().FrontLeft.Wear + s.Chassis().FrontRight.Wear + s.Chassis().RearLeft.Wear + s.Chassis().RearRight.Wear) / 4f;
                crossings.Add((t, s.Chassis().Energy.Primary, wear));
            });
        // Crossings: the start, the end of the out lap, the end of the
        // settling lap, then the measured laps.
        var a = crossings[2];
        var b = crossings[^1];
        int laps = Math.Max(1, crossings.Count - 3);
        return new Sample(setting, (b.T - a.T) / laps, (a.Pack - b.Pack) / laps, (b.Wear - a.Wear) / laps, r.Spins, r.OffTrackSeconds);
    }

    private static float Lerp(float a, float b, float t) => a + (b - a) * t;

    /// <summary>y at x on the piecewise line through the points, held at the ends.</summary>
    private static float Interpolate(float[] x, float[] y, float at)
    {
        if (at <= x[0])
            return y[0];
        for (int i = 1; i < x.Length; i++)
        {
            if (at <= x[i])
                return Lerp(y[i - 1], y[i], (at - x[i - 1]) / MathF.Max(x[i] - x[i - 1], 1e-6f));
        }
        return y[^1];
    }

    /// <summary>
    /// x where the line through the points reaches y, for y that rises or
    /// falls with x: the first crossing, held at the ends.
    /// </summary>
    private static float Inverse(float[] x, float[] y, float target)
    {
        for (int i = 1; i < x.Length; i++)
        {
            float lo = MathF.Min(y[i - 1], y[i]), hi = MathF.Max(y[i - 1], y[i]);
            if (target >= lo && target <= hi)
            {
                float span = y[i] - y[i - 1];
                return MathF.Abs(span) < 1e-9f ? x[i - 1] : Lerp(x[i - 1], x[i], (target - y[i - 1]) / span);
            }
        }
        bool rising = y[^1] > y[0];
        return (target < y[0]) == rising ? x[0] : x[^1];
    }
}
