using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Track;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Drivers.Tests;

public sealed class SilverstonePaceProbe
{
    private readonly ITestOutputHelper _output;

    public SilverstonePaceProbe(ITestOutputHelper output) => _output = output;

    [Fact]
    public void ReportTheCarSpeedPlanOnTheGeometricLine()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        GeometricRacingLine line = RuleLineAssets.LoadOrBake(RuleTrackPack.Silverstone, track, 0.95f);
        var car = new CarConfig();
        var tires = new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f };
        var state = new CarState { Energy = PowertrainState.Filled(0.8f) };
        foreach (TireState tire in new[] { state.FrontLeft, state.FrontRight, state.RearLeft, state.RearRight })
        {
            tire.SurfaceTempC = 90f;
            tire.CoreTempC = 90f;
        }

        float grip = tires.GetAccelerationUsage(CarStrategy.Default);
        foreach (float v in new[] { 12f, 40f, 70f })
        {
            CarPerformanceLimits lim = CarPhysics.EstimatePerformanceLimits(
                state, car, tires, CarStrategy.Default, v, curvature: 0f, grip
            );
            _output.WriteLine(
                $"envelope v={v:0} ay={lim.LateralAccelerationLimit:F1} " +
                $"brake={lim.MaximumBrakeDeceleration:F1} drive={lim.MaximumDriveAcceleration:F1}"
            );
        }

        LineSpeedProfile withRoad = Plan(line, track, state, car, tires, grip, includeRoad: true, gg: 0.95f);
        LineSpeedProfile lineOnly = Plan(line, track, state, car, tires, grip, includeRoad: false, gg: 0.95f);
        LineSpeedProfile lineFull = Plan(line, track, state, car, tires, grip, includeRoad: false, gg: 1.00f);

        int roadWins = 0;
        int samples = 0;
        float sumLineK = 0f;
        float sumRoadK = 0f;
        for (float s = 0f; s < track.LengthMeters; s += 8f)
        {
            float lineK = MathF.Abs(line.PathCurvatureAt(track.WrapS, s));
            float roadK = MathF.Abs(track.Sample(s).Curvature);
            sumLineK += lineK;
            sumRoadK += roadK;
            if (roadK > lineK + 1e-4f)
                roadWins++;
            samples++;
        }

        _output.WriteLine($"geometric line has baked v*: {line.HasBakedSpeed}");
        _output.WriteLine($"car plan, max(line, road) κ, 95% gg: {withRoad.EstimateLapSeconds():F1}s peak {withRoad.PeakSpeed():F1}");
        _output.WriteLine($"car plan, line κ only, 95% gg: {lineOnly.EstimateLapSeconds():F1}s peak {lineOnly.PeakSpeed():F1} meanκ {lineOnly.MeanAbsKappa():F4}");
        _output.WriteLine($"car plan, line κ only, 100% gg: {lineFull.EstimateLapSeconds():F1}s peak {lineFull.PeakSpeed():F1}");
        _output.WriteLine($"road κ tighter than line at {roadWins}/{samples}, mean line κ {sumLineK / samples:F4} mean road κ {sumRoadK / samples:F4}");
        _output.WriteLine($"peak |line κ| {PeakLineK(line, track):F4} peak |road κ| {PeakRoadK(track):F4}");
        Assert.False(line.HasBakedSpeed);
        Assert.True(lineOnly.EstimateLapSeconds() > 80f && lineOnly.EstimateLapSeconds() < 160f);
    }

    private static LineSpeedProfile Plan(
        GeometricRacingLine line,
        TrackData track,
        CarState state,
        CarConfig car,
        TireConfig tires,
        float grip,
        bool includeRoad,
        float gg
    )
    {
        LineSpeedProfile profile = LineSpeedProfile.FromLine(line, track, includeRoad);
        profile.Recompute(state, car, tires, grip, gg);
        return profile;
    }

    private static float PeakLineK(GeometricRacingLine line, TrackData track)
    {
        float peak = 0f;
        for (float s = 0f; s < track.LengthMeters; s += 2f)
            peak = MathF.Max(peak, MathF.Abs(line.PathCurvatureAt(track.WrapS, s)));
        return peak;
    }

    private static float PeakRoadK(TrackData track)
    {
        float peak = 0f;
        for (float s = 0f; s < track.LengthMeters; s += 2f)
            peak = MathF.Max(peak, MathF.Abs(track.Sample(s).Curvature));
        return peak;
    }
}
