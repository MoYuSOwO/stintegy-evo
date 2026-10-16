using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

public sealed class LanePlanProbe
{
    private readonly ITestOutputHelper _output;
    public LanePlanProbe(ITestOutputHelper output) => _output = output;

    internal static (float Lap, float PeakK) Plan(LaneSet lanes, LaneEnvelope env, LaneShares shares, int chord = 3)
    {
        int n = lanes.Count;
        var k = new float[n];
        var ds = new float[n];
        var f = new float[n];
        Array.Fill(f, 1f);
        Vector2 P(int i) => lanes.Position(i, lanes.Offset(LaneId.Free, i));
        float peak = 0f;
        for (int i = 0; i < n; i++)
        {
            k[i] = MathF.Abs(LaneSet.Curvature(P(i - chord), P(i), P(i + chord)));
            ds[i] = (P(i + 1) - P(i)).Length();
            peak = MathF.Max(peak, k[i]);
        }
        var v = new float[n];
        LaneSpeedPlanner.PlanClosed(env, k, ds, f, f, ReadOnlySpan<LaneRoad>.Empty, shares, v);
        float lap = 0f;
        for (int i = 0; i < n; i++)
            lap += ds[i] / MathF.Max(v[i], 1f);
        return (lap, peak);
    }

    internal static LaneEnvelope HotEnvelope()
    {
        var state = new CarState { Energy = PowertrainState.Filled(0.8f) };
        foreach (TireState t in new[] { state.FrontLeft, state.FrontRight, state.RearLeft, state.RearRight })
        {
            t.SurfaceTempC = 95f;
            t.CoreTempC = 95f;
        }
        return LaneEnvelope.From(state, new CarConfig(), new TireConfig(), CarStrategy.Default);
    }

    [Fact]
    public void SmoothingAndCoupling()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneEnvelope env = HotEnvelope();
        foreach (int passes in new[] { 12, 50, 200 })
        {
            LaneSet lanes = LaneSet.Bake(track, new LaneBakeOptions { PassesPerLevel = passes });
            foreach (int chord in new[] { 2, 3, 5, 8 })
            {
                var on = Plan(lanes, env, new LaneShares { Lateral = 1f, Brake = 1f }, chord);
                var off = Plan(lanes, env, new LaneShares { Lateral = 1f, Brake = 1f, BrakingCoupling = false }, chord);
                _output.WriteLine($"passes {passes,3} chord {chord}: coupled {on.Lap:F1}s uncoupled {off.Lap:F1}s peak {on.PeakK:F4}");
            }
        }
    }

    [Fact]
    public void ConvergenceAndRoom()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneEnvelope env = HotEnvelope();
        var full = new LaneShares { Lateral = 1f, Brake = 1f };
        foreach ((int passes, float edge, float inner) in new[] { (200, 1.25f, 0.6f), (500, 1.25f, 0.6f), (1000, 1.25f, 0.6f), (500, 1.05f, 0.0f), (500, 1.05f, 0.3f) })
        {
            var watch = System.Diagnostics.Stopwatch.StartNew();
            LaneSet lanes = LaneSet.Bake(track, new LaneBakeOptions { PassesPerLevel = passes, EdgeMargin = edge, MaxInnerMargin = inner });
            long ms = watch.ElapsedMilliseconds;
            var a = Plan(lanes, env, full, 3);
            var m = Plan(lanes, env, new LaneShares(), 3);
            _output.WriteLine($"passes {passes,4} edge {edge:F2} inner {inner:F1}: full {a.Lap:F1}s margin {m.Lap:F1}s peak {a.PeakK:F4} bake {ms} ms");
        }
    }

    [Fact]
    public void CompareLinesAndMargins()
    {
        if (!LaneProbes.Enabled) return;
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        LaneSet lanes = LaneSet.Bake(track);
        LaneEnvelope env = HotEnvelope();

        GeometricRacingLine tum = RuleLineAssets.LoadOrBake(RuleTrackPack.Silverstone, track, 0.95f);
        var tumOffsets = new float[lanes.Count];
        for (int i = 0; i < lanes.Count; i++)
        {
            // GeometricRacingLine offsets are TrackD, the same normal the lanes use.
            tumOffsets[i] = tum.At(track.Sample, track.WrapS, i * lanes.Spacing).OffsetMeters;
        }
        LaneSet tumLanes = lanes.WithFreeOffsets(tumOffsets);
        var centre = new float[lanes.Count];
        LaneSet centreLanes = lanes.WithFreeOffsets(centre);

        var margin = new LaneShares();
        var full = new LaneShares { Lateral = 1f, Brake = 1f };
        foreach ((string name, LaneSet set) in new[] { ("k1999", lanes), ("tum", tumLanes), ("centre", centreLanes) })
        {
            foreach (int chord in new[] { 2, 3, 5 })
            {
                var a = Plan(set, env, margin, chord);
                var b = Plan(set, env, full, chord);
                _output.WriteLine($"{name,-7} chord {chord}: margin {a.Lap:F1}s  full {b.Lap:F1}s  peak k {a.PeakK:F4}");
            }
        }
    }
}
