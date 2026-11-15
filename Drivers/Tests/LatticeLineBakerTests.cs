using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using StintegyEVO.Drivers.Lanes;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// The offline lattice baker: a pinned stretch, then a closed lap driven
/// against the geometric lanes and the shipped 3D bake.
/// </summary>
public sealed class LatticeLineBakerTests
{
    private readonly ITestOutputHelper _output;

    public LatticeLineBakerTests(ITestOutputHelper output) => _output = output;

    private static ICar Car() => StockCar.Build(DevelopmentTyres.SuperId);

    [Fact]
    public void APinnedStretchStaysInsideTheRoadAndOnTheKerb()
    {
        TrackData track = GameMods.Registry.Circuits.Get("silverstone").Track;
        LaneSet lanes = LaneSet.Bake(track);
        int apex = 0;
        float sharpest = 0f;
        for (int i = 0; i < lanes.Count; i++)
        {
            float k = LaneSet.Curvature(
                lanes.Position(i - 3, lanes.Offset(LaneId.Free, i - 3)),
                lanes.Position(i, lanes.Offset(LaneId.Free, i)),
                lanes.Position(i + 3, lanes.Offset(LaneId.Free, i + 3)));
            if (MathF.Abs(k) > sharpest)
                (sharpest, apex) = (MathF.Abs(k), i);
        }

        var options = new LatticeBakeOptions
        {
            RowStations = 5,
            EvenPlaces = 7,
            SearchBandMeters = 0f,
            Spans = [1, 2, 4, 8],
            MaxLateralJumps = [4f, 7f, 10f, 14f],
            Slopes = [-0.16f, -0.06f, 0f, 0.06f, 0.16f],
            SpeedStep = 2f,
            ExactSpeeds = true,
            PolishBandMeters = 0.25f,
        };
        int from = lanes.Wrap(apex - 12 * options.RowStations);
        int rows = 18;
        int apexRow = (int)MathF.Round(lanes.Wrap(apex - from) / (float)options.RowStations);
        Assert.InRange(apexRow, 1, rows - 1);
        // The free line's own corner decides the inside: positive curvature turns left,
        // and that inside is the negative offset.
        float sign = MathF.Sign(LaneSet.Curvature(
            lanes.Position(apex - 3, lanes.Offset(LaneId.Free, apex - 3)),
            lanes.Position(apex, lanes.Offset(LaneId.Free, apex)),
            lanes.Position(apex + 3, lanes.Offset(LaneId.Free, apex + 3))));
        if (sign == 0f)
            sign = 1f;
        float Inside(int station)
        {
            float edge = lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters;
            return sign > 0f ? -edge : edge;
        }
        int apexStation = lanes.Wrap(from + apexRow * options.RowStations);
        float kerb = Inside(apexStation);
        float startY = lanes.Offset(LaneId.Free, from);
        float endY = lanes.Offset(LaneId.Free, lanes.Wrap(from + rows * options.RowStations));

        (float Lo, float Hi) Corridor(int station)
        {
            float edge = lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters;
            float lo = -edge, hi = edge;
            if (sign > 0f)
                hi = MathF.Min(hi, edge - 1.5f);
            else
                lo = MathF.Max(lo, -edge + 1.5f);
            lo = MathF.Min(lo, MathF.Min(startY, endY) - 0.3f);
            hi = MathF.Max(hi, MathF.Max(startY, endY) + 0.3f);
            return (lo, hi);
        }
        bool Allowed(int station, float y)
        {
            (float lo, float hi) = Corridor(station);
            if (y < lo - 1e-3f || y > hi + 1e-3f)
                return false;
            return station != apexStation || MathF.Abs(y - kerb) <= 0.85f;
        }

        var watch = System.Diagnostics.Stopwatch.StartNew();
        var way = LatticeLineBaker.BakeStretch(track, lanes, Car(), from, rows, 55f, v => 80f / MathF.Max(v, 1f),
            allowedOffset: Allowed, options: options);
        watch.Stop();
        Assert.NotNull(way);
        var (offset, speed, seconds) = way.Value;
        _output.WriteLine($"stretch {watch.Elapsed.TotalSeconds:F2}s  apex row {apexRow} kerb {kerb:F2}  line {seconds:F2}s");

        Assert.Equal(rows * options.RowStations + 1, offset.Length);
        Assert.Equal(offset.Length, speed.Length);
        Assert.True(seconds > 1f && seconds < 60f, seconds.ToString("F2"));
        float maxStep = 0f, maxBend = 0f;
        for (int k = 0; k < offset.Length; k++)
        {
            int station = lanes.Wrap(from + k);
            float edge = lanes.HalfWidth(station) - LaneSet.EdgeMarginMeters;
            Assert.InRange(offset[k], -edge - 0.05f, edge + 0.05f);
            (float lo, float hi) = Corridor(station);
            Assert.InRange(offset[k], lo - 0.35f, hi + 0.35f);
            if (k + 1 < offset.Length)
                maxStep = MathF.Max(maxStep, MathF.Abs(offset[k + 1] - offset[k]));
            if (k > 0 && k + 1 < offset.Length)
                maxBend = MathF.Max(maxBend, MathF.Abs(offset[k + 1] - 2f * offset[k] + offset[k - 1]));
            Assert.True(speed[k] > 5f && speed[k] <= LaneSpeedPlanner.TopSpeed + 0.1f);
        }
        int at = apexRow * options.RowStations;
        _output.WriteLine($"apex offset {offset[at]:F2} vs kerb {Inside(lanes.Wrap(from + at)):F2}  step {maxStep:F2} bend {maxBend:F2}");
        Assert.InRange(offset[at], kerb - 0.55f, kerb + 0.55f);
        Assert.True(maxStep < 0.85f, $"step {maxStep:F2}");
        Assert.True(maxBend < 0.35f, $"bend {maxBend:F2}");
    }

    [Fact]
    public void AClosedLapIsSmoothAndFasterThanTheGeometricLine()
    {
        TrackData track = GameMods.Registry.Circuits.Get("interlagos").Track;
        ICar car = Car();
        var options = new LatticeBakeOptions
        {
            RowStations = 5,
            EvenPlaces = 3,
            SearchBandMeters = 3f,
            Spans = [2, 4, 8],
            MaxLateralJumps = [8f, 12f, 16f],
            Slopes = [-0.1f, -0.04f, 0f, 0.04f, 0.1f],
            SpeedStep = 2f,
            ExactSpeeds = true,
            UseTurnLag = true,
            PolishBandMeters = 0.25f,
            ClosurePasses = 2,
            SideRoomMeters = 3f,
        };

        var watch = System.Diagnostics.Stopwatch.StartNew();
        LaneSet lattice = LatticeLineBaker.BakeLaneSet(track, car, options);
        watch.Stop();
        LaneSet geometric = LaneSet.Bake(track);
        _output.WriteLine($"full-lap bake {watch.Elapsed.TotalSeconds:F2}s  stations {lattice.Count}");

        float offsetJump = 0f, curvatureJump = 0f;
        float K(LaneSet lanes, int i) => LaneSet.Curvature(
            lanes.Position(i - 3, lanes.Offset(LaneId.Free, i - 3)),
            lanes.Position(i, lanes.Offset(LaneId.Free, i)),
            lanes.Position(i + 3, lanes.Offset(LaneId.Free, i + 3)));
        int n = lattice.Count;
        for (int i = 0; i < n; i++)
        {
            float edge = lattice.HalfWidth(i) - LaneSet.EdgeMarginMeters;
            Assert.InRange(lattice.Offset(LaneId.Free, i), -edge - 0.05f, edge + 0.05f);
            Assert.InRange(lattice.Offset(LaneId.Left, i), -edge - 0.05f, edge + 0.05f);
            Assert.InRange(lattice.Offset(LaneId.Right, i), -edge - 0.05f, edge + 0.05f);
        }
        for (int i = 0; i < n; i++)
        {
            float step = lattice.Offset(LaneId.Free, i + 1) - lattice.Offset(LaneId.Free, i);
            float prev = lattice.Offset(LaneId.Free, i) - lattice.Offset(LaneId.Free, i - 1);
            offsetJump = MathF.Max(offsetJump, MathF.Abs(step - prev));
            curvatureJump = MathF.Max(curvatureJump, MathF.Abs(K(lattice, i + 1) - K(lattice, i)));
        }
        float offSum = 0f, signed = 0f, offMax = 0f;
        for (int i = 0; i < n; i++)
        {
            float d = lattice.Offset(LaneId.Free, i) - geometric.Offset(LaneId.Free, i);
            offSum += MathF.Abs(d);
            signed += d;
            offMax = MathF.Max(offMax, MathF.Abs(d));
        }
        _output.WriteLine($"seam second-difference {offsetJump:F4} m  max curvature step {curvatureJump:F5}  " +
            $"off geometric mean {offSum / n:F2} signed {signed / n:F2} max {offMax:F2} m");
        Assert.True(offsetJump < 0.25f, $"offset kink {offsetJump:F3} m");
        Assert.True(curvatureJump < 0.012f, $"curvature kink {curvatureJump:F4}");

        (LaneLapResult Result, float Steer) Drive(LaneSet lanes)
        {
            double rough = 0;
            int samples = 0;
            float previous = 0f;
            bool have = false;
            LaneLapResult result = LaneLapRunner.Run(track, LaneTrackFile.For(lanes), onStep: (_, driver, _) =>
            {
                float wheel = driver.LastTelemetry.WantedCurvature;
                if (have)
                {
                    rough += MathF.Abs(wheel - previous);
                    samples++;
                }
                previous = wheel;
                have = true;
            });
            return (result, samples == 0 ? 0f : (float)(rough / samples));
        }

        (LaneLapResult geoLap, float geoSteer) = Drive(geometric);
        (LaneLapResult latLap, float latSteer) = Drive(lattice);
        _output.WriteLine($"geometric {geoLap}  steer {geoSteer:F5}");
        _output.WriteLine($"lattice    {latLap}  steer {latSteer:F5}");

        string shipped = LaneProbes.Baked("interlagos");
        if (shipped.Length > 0)
        {
            LaneSet tum = LaneTrackFile.FromBakedLanes(shipped, track).LanesFor(track);
            (LaneLapResult tumLap, float tumSteer) = Drive(tum);
            _output.WriteLine($"tum 3d     {tumLap}  steer {tumSteer:F5}");
        }

        Assert.True(latLap.Finished, latLap.ToString());
        Assert.Equal(0, latLap.Spins);
        Assert.Equal(0f, latLap.OffTrackSeconds);
        Assert.Equal(0f, latLap.WallSeconds);
        Assert.True(geoLap.Finished, geoLap.ToString());
        Assert.True(latLap.FlyingLapSeconds < geoLap.FlyingLapSeconds,
            $"lattice {latLap.FlyingLapSeconds:F3}s geometric {geoLap.FlyingLapSeconds:F3}s");
    }
}
