using System.IO;
using System;
using System.Collections.Generic;
using System.Linq;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

// Temporary: measures the tyre model against the targets. Delete after use.
public sealed class TyreModelProbe
{
    private readonly ITestOutputHelper _output;
    public TyreModelProbe(ITestOutputHelper output) => _output = output;

    private static TireConfig Tyres(float c) => new() { StartingSurfaceTempC = c, StartingCoreTempC = c };

    private LaneLapResult Lap(TireConfig tyres, float wear = 0f)
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        return LaneLapRunner.Run(track, file, flyingLaps: 1, tireConfig: tyres, startingWear: wear);
    }

    [Fact]
    public void ColdSweep()
    {
        if (!LaneProbes.Enabled) return;
        float baseOut = Lap(Tyres(90f)).OutLapSeconds;
        _output.WriteLine($"90 C out {baseOut:F2}");
        foreach (float k in new[] { 0.0006f, 0.0001f, 0.00008f, 0.00006f, 0.00004f })
            foreach (float c in new[] { 25f, 70f })
            {
                var r = Lap(new TireConfig { StartingSurfaceTempC = c, StartingCoreTempC = c, ColdGripLossPerCSquared = k });
                _output.WriteLine($"k {k:F5} start {c} C: out {r.OutLapSeconds:F2} (+{r.OutLapSeconds - baseOut:F2}s, {(r.OutLapSeconds / baseOut - 1) * 100:F1}%)  spins {r.Spins} off {r.OffTrackSeconds:F2}");
            }
    }

    [Theory]
    [InlineData(TireUsageMode.Protect, 0.85f)]
    [InlineData(TireUsageMode.Normal, 0.977f)]
    [InlineData(TireUsageMode.Attack, 0.977f)]
    [InlineData(TireUsageMode.Attack, 1.0f)]
    public void HeatSpan(TireUsageMode mode, float usage)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        float maxSurf = 0f, maxRearSurf = 0f; int lastLap = int.MinValue; var line = new List<string>();
        var r = LaneLapRunner.Run(track, file, flyingLaps: 6, tireConfig: new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f, AccelerationUsage = usage },
            strategy: new CarStrategy(mode, PowerOutputMode.Normal), onStep: (car, d, t) =>
            {
                var s = car.State;
                var ty = new[] { s.FrontLeft, s.FrontRight, s.RearLeft, s.RearRight };
                maxSurf = MathF.Max(maxSurf, ty.Max(q => q.SurfaceTempC));
                maxRearSurf = MathF.Max(maxRearSurf, MathF.Max(s.RearLeft.SurfaceTempC, s.RearRight.SurfaceTempC));
                if (car.Progress.Lap != lastLap) { lastLap = car.Progress.Lap; line.Add($"{ty.Average(q => q.SurfaceTempC):F0}/{ty.Max(q => q.CoreTempC):F0}/{ty.Max(q => q.Wear) * 100:F1}%"); }
            });
        _output.WriteLine($"{mode} usage {usage}: laps {string.Join(" ", r.LapTimes.Select(l => l.ToString("F2")))}  peak surface {maxSurf:F0} (rear {maxRearSurf:F0})  spins {r.Spins} off {r.OffTrackSeconds:F2}");
        _output.WriteLine("   avg surf/max core/wear at line: " + string.Join("  ", line));
    }

    // Attack for `attackLaps` laps, then Normal; per lap: time, hottest core, mean core per tyre.
    private void Scenario(string label, TireConfig tyres, int attackLaps, int totalLaps)
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        int lastLap = int.MinValue, lapIndex = -1;
        var core = new double[4]; var surf = new double[4]; double n = 0; var rows = new List<string>();
        var r = LaneLapRunner.Run(track, file, flyingLaps: totalLaps, tireConfig: tyres,
            strategy: new CarStrategy(attackLaps > 0 ? TireUsageMode.Attack : TireUsageMode.Normal, PowerOutputMode.Normal), onStep: (car, d, t) =>
            {
                var s = car.State;
                var ty = new[] { s.FrontLeft, s.FrontRight, s.RearLeft, s.RearRight };
                for (int i = 0; i < 4; i++) { core[i] += ty[i].CoreTempC; surf[i] += ty[i].SurfaceTempC; }
                n++;
                if (car.Progress.Lap != lastLap)
                {
                    lastLap = car.Progress.Lap; lapIndex++;
                    if (lapIndex > 0)
                        rows.Add($"{car.Strategy.TireMode,-6} core FL {core[0] / n:F0} FR {core[1] / n:F0} RL {core[2] / n:F0} RR {core[3] / n:F0}  surf max {surf.Max() / n:F0}  wear {ty.Max(q => q.Wear) * 100:F0}%");
                    Array.Clear(core); Array.Clear(surf); n = 0;
                    if (lapIndex >= attackLaps) car.Strategy = new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Normal);
                }
            });
        _output.WriteLine($"{label}: spins {r.Spins} off {r.OffTrackSeconds:F2}");
        for (int i = 0; i < r.LapTimes.Count && i + 1 < rows.Count; i++)
            _output.WriteLine($"  lap {i + 1,2} {r.LapTimes[i]:F2}  {rows[i + 1]}");
    }

    [Fact]
    public void OverheatBaseline()
    {
        if (!LaneProbes.Enabled) return;
        var hot = new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f };
        Scenario("all Normal", hot, 0, 8);
        Scenario("Attack 6 then Normal", hot, 6, 10);
    }

    [Theory]
    [InlineData(TireUsageMode.Protect)]
    [InlineData(TireUsageMode.Normal)]
    [InlineData(TireUsageMode.Attack)]
    public void UseSpread(TireUsageMode mode)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        var uses = new List<float>(); float tCorner = 0f;
        LaneLapRunner.Run(track, file, flyingLaps: 2, tireConfig: Tyres(90f), strategy: new CarStrategy(mode, PowerOutputMode.Normal), onStep: (car, d, t) =>
        {
            if (t < 110f) return;
            var tel = car.State.Telemetry;
            float u = MathF.Max(MathF.Sqrt(tel.FrontLateralUse * tel.FrontLateralUse + tel.FrontLongitudinalUse * tel.FrontLongitudinalUse),
                                MathF.Sqrt(tel.RearLateralUse * tel.RearLateralUse + tel.RearLongitudinalUse * tel.RearLongitudinalUse));
            uses.Add(u);
        });
        uses.Sort();
        float P(float q) => uses[(int)(q * (uses.Count - 1))];
        int n = uses.Count;
        _output.WriteLine($"{mode}: combined use p50 {P(0.5f):F3} p75 {P(0.75f):F3} p90 {P(0.9f):F3} p97 {P(0.97f):F3} max {uses[^1]:F3}  share>0.9 {uses.Count(x => x > 0.9f) * 100f / n:F1}%  share>0.97 {uses.Count(x => x > 0.97f) * 100f / n:F1}%");
    }

    [Theory]
    [InlineData(0.90f)]
    [InlineData(0.94f)]
    [InlineData(0.97f)]
    public void CapShare(float cap)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        var uses = new List<float>(); int lastLap = int.MinValue; var rows = new List<string>();
        var r = LaneLapRunner.Run(track, file, new LaneDriverGains { AxleCapShare = cap }, flyingLaps: 6, tireConfig: Tyres(90f), strategy: new CarStrategy(TireUsageMode.Attack, PowerOutputMode.Normal), onStep: (car, d, t) =>
        {
            var s = car.State; var tel = s.Telemetry;
            uses.Add(MathF.Max(MathF.Sqrt(tel.FrontLateralUse * tel.FrontLateralUse + tel.FrontLongitudinalUse * tel.FrontLongitudinalUse),
                               MathF.Sqrt(tel.RearLateralUse * tel.RearLateralUse + tel.RearLongitudinalUse * tel.RearLongitudinalUse)));
            if (car.Progress.Lap != lastLap) { lastLap = car.Progress.Lap; var ty = new[] { s.FrontLeft, s.FrontRight, s.RearLeft, s.RearRight }; rows.Add($"{ty.Max(q => q.CoreTempC):F0}/{ty.Max(q => q.Wear) * 100:F0}%"); }
        });
        uses.Sort();
        _output.WriteLine($"cap {cap}: laps {string.Join(" ", r.LapTimes.Select(l => l.ToString("F2")))}  use p97 {uses[(int)(0.97 * (uses.Count - 1))]:F3} max {uses[^1]:F3}  spins {r.Spins} off {r.OffTrackSeconds:F2}");
        _output.WriteLine("   max core/wear at line: " + string.Join("  ", rows));
    }

    private static float UseP97(List<float> uses) { uses.Sort(); return uses[(int)(0.97 * (uses.Count - 1))]; }

    private static float CombinedUse(CarTelemetry tel) =>
        MathF.Max(MathF.Sqrt(tel.FrontLateralUse * tel.FrontLateralUse + tel.FrontLongitudinalUse * tel.FrontLongitudinalUse),
                  MathF.Sqrt(tel.RearLateralUse * tel.RearLateralUse + tel.RearLongitudinalUse * tel.RearLongitudinalUse));

    [Theory]
    [InlineData(0.90f)]
    [InlineData(0.93f)]
    [InlineData(0.96f)]
    public void LearnWithCap(float cap)
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        var gains = new LaneDriverGains { TurnAwareEnvelope = true, AxleCapShare = cap };
        var learner = new LaneLearner { Reward = 0.02f, FlyingLaps = 3, CapBoundIsTrouble = true };
        var (best, bestLap, history) = learner.Learn(track, file, 80, gains, lane: LaneId.Free, level: LaneTrackFile.TopLevel);
        int kept = history.Count(h => h.Kept);
        float[] f = best.Factors(LaneId.Free, LaneTrackFile.TopLevel);
        _output.WriteLine($"cap {cap}: first {history[0].Result} | best {bestLap} | kept {kept}/{history.Count}  factor p50 {f.OrderBy(x => x).ElementAt(f.Length / 2):F3} max {f.Max():F3}");
        foreach (TireUsageMode mode in new[] { TireUsageMode.Normal, TireUsageMode.Attack })
        {
            var uses = new List<float>(); int lastLap = int.MinValue; var rows = new List<string>();
            var r = LaneLapRunner.Run(track, best, gains, flyingLaps: 5, tireConfig: Tyres(90f), strategy: new CarStrategy(mode, PowerOutputMode.Normal), onStep: (car, d, t) =>
            {
                uses.Add(CombinedUse(car.State.Telemetry));
                if (car.Progress.Lap != lastLap) { lastLap = car.Progress.Lap; var s = car.State; var ty = new[] { s.FrontLeft, s.FrontRight, s.RearLeft, s.RearRight }; rows.Add($"{ty.Max(q => q.CoreTempC):F0}"); }
            });
            _output.WriteLine($"   {mode}: laps {string.Join(" ", r.LapTimes.Select(l => l.ToString("F2")))} use p97 {UseP97(uses):F3} spins {r.Spins} off {r.OffTrackSeconds:F2} core {string.Join(" ", rows)}");
        }
    }

    // Compact: per lap "time/max core/max surface-avg".
    private string Compact(TireConfig tyres, int attackLaps, int totalLaps)
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        int lastLap = int.MinValue, lapIndex = -1; var cores = new List<string>();
        double surf = 0, n = 0;
        var r = LaneLapRunner.Run(track, file, flyingLaps: totalLaps, tireConfig: tyres,
            strategy: new CarStrategy(attackLaps > 0 ? TireUsageMode.Attack : TireUsageMode.Normal, PowerOutputMode.Normal), onStep: (car, d, t) =>
            {
                var s = car.State; var ty = new[] { s.FrontLeft, s.FrontRight, s.RearLeft, s.RearRight };
                surf += ty.Max(q => q.SurfaceTempC); n++;
                if (car.Progress.Lap == lastLap) return;
                lastLap = car.Progress.Lap; lapIndex++;
                if (lapIndex > 1) cores.Add($"{ty.Max(q => q.CoreTempC):F0}/{surf / n:F0}");
                surf = 0; n = 0;
                if (lapIndex >= attackLaps) car.Strategy = new CarStrategy(TireUsageMode.Normal, PowerOutputMode.Normal);
            });
        var laps = r.LapTimes.Select((l, i) => $"{l:F2}({(i < cores.Count ? cores[i] : "-")})");
        return $"{string.Join(" ", laps)}  spins {r.Spins} off {r.OffTrackSeconds:F2}";
    }

    [Theory]
    [InlineData(0.15f, 0.95f, 0f)]
    [InlineData(0.3f, 0.95f, 0f)]
    [InlineData(0.6f, 0.95f, 0f)]
    [InlineData(1.2f, 0.95f, 0f)]
    [InlineData(0.6f, 0.97f, 0f)]
    [InlineData(1.2f, 0.97f, 0f)]
    public void EdgeHeatSweep(float rate, float start, float coreLoss) => EdgeHeat(rate, start, coreLoss, 0.00025f);

    [Theory]
    [InlineData(1.2f, 0.95f, 0f, 0f)]
    [InlineData(2f, 0.97f, 0f, 0f)]
    [InlineData(2f, 0.97f, 0f, 0.00025f)]
    public void EdgeHeatNoHotLoss(float rate, float start, float coreLoss, float hot) => EdgeHeat(rate, start, coreLoss, hot);

    [Theory]
    [InlineData(2f, 0.0001f, 111f, 0.01f)]
    [InlineData(2f, 0.0001f, 111f, 0.02f)]
    [InlineData(2f, 0.0001f, 112f, 0.03f)]
    [InlineData(1.5f, 0.00025f, 110f, 0.02f)]
    [InlineData(1.5f, 0.0001f, 110f, 0.02f)]
    [InlineData(1.5f, 0.0001f, 110f, 0.04f)]
    public void CoreOverheatSweep(float rate, float hot, float threshold, float loss) => EdgeHeat(rate, 0.97f, loss, hot, threshold);

    [Theory]
    [InlineData(12f, 2f, 112f, 0.03f)]
    [InlineData(24f, 2f, 112f, 0.03f)]
    [InlineData(36f, 2f, 112f, 0.03f)]
    [InlineData(36f, 2.5f, 113f, 0.03f)]
    public void CoreCapacitySweep(float ratio, float rate, float threshold, float loss)
    {
        if (!LaneProbes.Enabled) return;
        _ = ratio;
        try
        {
            _output.WriteLine($"core capacity {ratio}");
            EdgeHeat(rate, 0.97f, loss, 0.0001f, threshold);
            float baseOut = Lap(Tyres(90f)).OutLapSeconds;
            foreach (float c in new[] { 25f, 70f })
            {
                var r = Lap(new TireConfig { StartingSurfaceTempC = c, StartingCoreTempC = c, NearLimitHeatRate = rate, NearLimitHeatStartShare = 0.97f, CoreOverheatGripLossPerC = loss, HotGripLossPerCSquared = 0.0001f, CoreOverheatTempC = threshold });
                _output.WriteLine($"  start {c} C out +{r.OutLapSeconds - baseOut:F2}s  spins {r.Spins} off {r.OffTrackSeconds:F2}");
            }
        }
        finally { }
    }

    [Theory]
    [InlineData(0.00008f)]
    [InlineData(0.00007f)]
    public void ColdAtCapacity24(float cold)
    {
        if (!LaneProbes.Enabled) return;
        ;
        try
        {
            TireConfig T(float c) => new() { StartingSurfaceTempC = c, StartingCoreTempC = c, ColdGripLossPerCSquared = cold, NearLimitHeatRate = 2f, NearLimitHeatStartShare = 0.97f, CoreOverheatGripLossPerC = 0.03f, HotGripLossPerCSquared = 0.0001f, CoreOverheatTempC = 112f };
            float baseOut = Lap(T(90f)).OutLapSeconds;
            foreach (float c in new[] { 25f, 70f })
            {
                var r = Lap(T(c));
                _output.WriteLine($"cold {cold}: start {c} C out +{r.OutLapSeconds - baseOut:F2}s ({(r.OutLapSeconds / baseOut - 1) * 100:F1}%) spins {r.Spins} off {r.OffTrackSeconds:F2}");
            }
        }
        finally { }
    }

    [Fact]
    public void Defaults()
    {
        if (!LaneProbes.Enabled) return;
        var hot = new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f };
        _output.WriteLine($"Normal x8       : {Compact(hot, 0, 8)}");
        _output.WriteLine($"Attack x7 +N x3 : {Compact(hot, 7, 10)}");
        _output.WriteLine($"blankets: Attack x7 +N x3 : {Compact(new TireConfig(), 7, 10)}");
        float baseOut = Lap(Tyres(90f)).OutLapSeconds;
        foreach (float c in new[] { 25f, 70f })
        {
            var r = Lap(Tyres(c));
            _output.WriteLine($"start {c} C out +{r.OutLapSeconds - baseOut:F2}s ({(r.OutLapSeconds / baseOut - 1) * 100:F1}%) spins {r.Spins} off {r.OffTrackSeconds:F2}");
        }
        float fresh = Lap(Tyres(90f)).FlyingLapSeconds;
        foreach (float w in new[] { 0.7f, 0.8f })
            _output.WriteLine($"wear {w}: +{Lap(Tyres(90f), w).FlyingLapSeconds - fresh:F2}s");
    }

    [Fact]
    public void WarmUpTime()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        foreach (TireUsageMode mode in new[] { TireUsageMode.Normal, TireUsageMode.Attack })
            foreach (float c in new[] { 70f, 25f })
            {
                float avgAt = -1f, allAt = -1f, coreAt = -1f;
                var r = LaneLapRunner.Run(track, file, flyingLaps: 2, tireConfig: Tyres(c), strategy: new CarStrategy(mode, PowerOutputMode.Normal), onStep: (car, d, t) =>
                {
                    var s = car.State; var ty = new[] { s.FrontLeft, s.FrontRight, s.RearLeft, s.RearRight };
                    if (avgAt < 0f && ty.Average(q => q.SurfaceTempC) >= 85f) avgAt = t;
                    if (allAt < 0f && ty.Min(q => q.SurfaceTempC) >= 85f) allAt = t;
                    if (coreAt < 0f && ty.Average(q => q.CoreTempC) >= 85f) coreAt = t;
                });
                _output.WriteLine($"{mode} from {c} C: avg surface 85 at {avgAt:F0}s ({avgAt / 101f:F2} lap), all four {allAt:F0}s ({allAt / 101f:F2} lap), avg core 85 at {coreAt:F0}s  out lap {r.OutLapSeconds:F2}");
            }
    }

    [Theory]
    [InlineData(0.0001f)]
    [InlineData(0.00025f)]
    public void HotLossWithFastReplan(float hot)
    {
        if (!LaneProbes.Enabled) return;
        var watch = System.Diagnostics.Stopwatch.StartNew();
        var t = new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f, HotGripLossPerCSquared = hot };
        _output.WriteLine($"hot {hot}");
        _output.WriteLine($"  Normal x7       : {Compact(t, 0, 7)}");
        _output.WriteLine($"  Attack x7 +N x3 : {Compact(t, 7, 10)}");
        _output.WriteLine($"  {watch.Elapsed.TotalSeconds:F1} s for 17 laps");
    }

    // Relearn the free lane at every grip level from neutral factors, on the
    // shipped lane positions (the side lanes are still the geometric ones).
    [Fact]
    public void RelearnSilverstone()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track).Clone();
        for (int level = 0; level <= LaneTrackFile.TopLevel; level++)
        {
            Array.Fill(file.Factors(LaneId.Free, level), 1f);
            Array.Fill(file.BrakeFactors(LaneId.Free, level), 1f);
        }
        var gains = new LaneDriverGains { TurnAwareEnvelope = true };
        var learner = new LaneLearner { Reward = 0.02f, FlyingLaps = 3, CapBoundIsTrouble = true };
        var watch = System.Diagnostics.Stopwatch.StartNew();
        _output.WriteLine($"neutral: {LaneLapRunner.Run(track, file, gains)}");
        for (int level = LaneTrackFile.TopLevel; level >= 0; level--)
        {
            var (best, bestLap, history) = learner.Learn(track, file, 120, gains, lane: LaneId.Free, level: level);
            _output.WriteLine($"Free @ {LaneTrackFile.GripLevels[level]:F2}: first {history[0].Result.FlyingLapSeconds:F2} kept {history.Count(h => h.Kept)}/{history.Count} best {bestLap}  [{watch.Elapsed.TotalMinutes:F0} min]");
            file = best;
        }
        file.MarkLearnedNow();
        string path = Path.Combine(LaneProbes.Dir, "silverstone");
        file.SaveCircuit(path, "silverstone");
        _output.WriteLine($"saved {path}");
        var attack = new CarStrategy(TireUsageMode.Attack, PowerOutputMode.Normal);
        _output.WriteLine($"hot      {LaneLapRunner.Run(track, file, gains, flyingLaps: 3)}");
        _output.WriteLine($"attack   {LaneLapRunner.Run(track, file, gains, strategy: attack, flyingLaps: 3)}");
        _output.WriteLine($"cold     {LaneLapRunner.Run(track, file, gains, startSpeed: 0f, tireConfig: new TireConfig { StartingSurfaceTempC = 25f, StartingCoreTempC = 25f })}");
        _output.WriteLine($"blankets {LaneLapRunner.Run(track, file, gains, startSpeed: 0f, tireConfig: new TireConfig())}");
        _output.WriteLine($"wear 0.7 {LaneLapRunner.Run(track, file, gains, startingWear: 0.7f)}");
        _output.WriteLine($"wear 0.8 {LaneLapRunner.Run(track, file, gains, startingWear: 0.8f)}");
        _output.WriteLine($"road .78 {LaneLapRunner.Run(track, file, gains, roadGrip: 0.78f)}");
    }

    private void EdgeHeat(float rate, float start, float coreLoss, float hot, float threshold = 115f)
    {
        if (!LaneProbes.Enabled) return;
        var t = new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f, NearLimitHeatRate = rate, NearLimitHeatStartShare = start, CoreOverheatGripLossPerC = coreLoss, HotGripLossPerCSquared = hot, CoreOverheatTempC = threshold };
        _output.WriteLine($"  rate {rate} start {start} hot {hot} core>{threshold} loss {coreLoss}/C");
        _output.WriteLine($"  Normal x7      : {Compact(t, 0, 7)}");
        _output.WriteLine($"  Attack x6 +N x3: {Compact(t, 6, 9)}");
    }

    [Fact]
    public void WearSweep()
    {
        if (!LaneProbes.Enabled) return;
        float fresh = Lap(Tyres(90f)).FlyingLapSeconds;
        foreach (float lin in new[] { 0.03f, 0.04f, 0.05f })
        {
            var r = Lap(new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f, WearLinearGripLoss = lin }, 0.7f);
            _output.WriteLine($"linear {lin:F2} wear 0.7: +{r.FlyingLapSeconds - fresh:F2}s ({(r.FlyingLapSeconds / fresh - 1) * 100:F1}%)");
        }
        foreach (float cliff in new[] { 0.06f, 0.08f, 0.10f, 0.12f })
            foreach (float w in new[] { 0.8f, 0.9f })
            {
                var t = new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f, WearLinearGripLoss = 0.04f, WearCliffGripLoss = cliff };
                float at7 = Lap(t, 0.7f).FlyingLapSeconds;
                var r = Lap(t, w);
                _output.WriteLine($"linear 0.04 cliff {cliff:F2} wear {w:F1}: +{r.FlyingLapSeconds - fresh:F2}s total, +{r.FlyingLapSeconds - at7:F2}s past 0.7  spins {r.Spins} off {r.OffTrackSeconds:F2}");
            }
    }

    private (LaneLapResult R, List<(float T, float Wear, float Surf, float Core, float Fuel)> X) Laps(
        float startC, int laps, TireUsageMode mode, float wear = 0f, float startSpeed = 40f)
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneTrackFile file = LaneTrackFile.ForCircuit("silverstone", track);
        var x = new List<(float, float, float, float, float)>();
        int lastLap = int.MinValue;
        var r = LaneLapRunner.Run(track, file, flyingLaps: laps, tireConfig: Tyres(startC), startingWear: wear,
            startSpeed: startSpeed, strategy: new CarStrategy(mode, PowerOutputMode.Normal), onStep: (car, d, t) =>
            {
                if (car.Progress.Lap == lastLap) return;
                lastLap = car.Progress.Lap;
                var s = car.State;
                var ty = new[] { s.FrontLeft, s.FrontRight, s.RearLeft, s.RearRight };
                x.Add((t, ty.Max(q => q.Wear), ty.Average(q => q.SurfaceTempC), ty.Average(q => q.CoreTempC), s.Energy.Secondary * 120f));
            });
        return (r, x);
    }

    [Fact]
    public void WarmUp()
    {
        if (!LaneProbes.Enabled) return;
        foreach (float c in new[] { 90f, 70f, 25f })
        {
            var (r, x) = Laps(c, 3, TireUsageMode.Normal);
            _output.WriteLine($"start {c} C: out {r.OutLapSeconds:F2}  laps {string.Join(" ", r.LapTimes.Select(l => l.ToString("F2")))}  spins {r.Spins} off {r.OffTrackSeconds:F2}");
            _output.WriteLine("   surface at line: " + string.Join(" ", x.Select(q => q.Surf.ToString("F0"))));
        }
    }

    [Fact]
    public void WearToSeconds()
    {
        if (!LaneProbes.Enabled) return;
        foreach (float w in new[] { 0f, 0.2f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f })
        {
            var (r, _) = Laps(90f, 1, TireUsageMode.Normal, w);
            _output.WriteLine($"wear {w:F1}: lap {r.FlyingLapSeconds:F2}  spins {r.Spins} off {r.OffTrackSeconds:F2}");
        }
    }

    [Theory]
    [InlineData(TireUsageMode.Normal, 30)]
    [InlineData(TireUsageMode.Attack, 12)]
    [InlineData(TireUsageMode.Protect, 30)]
    public void Stint(TireUsageMode mode, int laps)
    {
        if (!LaneProbes.Enabled) return;
        var (r, x) = Laps(90f, laps, mode);
        _output.WriteLine($"{mode}: spins {r.Spins} off {r.OffTrackSeconds:F2}");
        for (int i = 1; i < x.Count; i++)
            _output.WriteLine($"  lap {i,2}: {x[i].T - x[i - 1].T:F2}s  wear {x[i].Wear * 100f:F1}%  surface {x[i].Surf:F0} core {x[i].Core:F0} C  fuel {x[i].Fuel:F0} kg");
    }
}
