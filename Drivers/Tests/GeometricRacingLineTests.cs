using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class GeometricRacingLineTests
{
    [Fact]
    public void OffsetsStayInsideTheRoad()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        GeometricRacingLine line = GeometricRacingLine.Bake(track, 0.95f);
        for (float s = 0; s < track.LengthMeters; s += 25f)
        {
            GeometricRacingLine.Sample sample = line.At(track.Sample, track.WrapS, s);
            TrackSample road = track.Sample(sample.S);
            Assert.InRange(sample.OffsetMeters, -road.HalfWidth + 0.5f, road.HalfWidth - 0.5f);
        }
    }

    [Fact]
    public void BankedSpeedwayPrefersTheSteeperSide()
    {
        TrackData track = TrackFactory.BankedSpeedwayTestTrack();
        GeometricRacingLine line = GeometricRacingLine.Bake(track, 0.95f);
        float s = 350f + 0.5f * MathF.PI * 90f;
        GeometricRacingLine.Sample sample = line.At(track.Sample, track.WrapS, s);
        TrackSample road = track.Sample(s);
        Assert.True(
            MathF.Abs(road.BankSlope) > 0.10f,
            $"expected a banked turn at s={s:0}, bank={road.BankSlope:0.000}"
        );

        float onLine = MathF.Abs(road.BankSlopeAt(sample.OffsetMeters));
        float onCentre = MathF.Abs(road.BankSlopeAt(0f));
        Assert.True(
            onLine >= onCentre + 0.02f,
            $"high line should see more bank: d={sample.OffsetMeters:0.00} bank={onLine:0.000} vs centre {onCentre:0.000}"
        );
    }

    [Fact]
    public void BakedCurvatureDoesNotInventTighterCornersThanTheRoad()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        GeometricRacingLine line = GeometricRacingLine.Bake(track, 0.95f);
        float peakLine = 0f;
        float peakRoad = 0f;
        for (float s = 0; s < track.LengthMeters; s += 2f)
        {
            peakLine = MathF.Max(peakLine, MathF.Abs(line.PathCurvatureAt(track.WrapS, s)));
            peakRoad = MathF.Max(peakRoad, MathF.Abs(track.Sample(s).Curvature));
        }

        Assert.True(peakRoad > 0.04f, $"road peak κ {peakRoad:0.000}");
        Assert.True(
            peakLine < peakRoad + 0.022f,
            $"line peak κ {peakLine:0.000} vs road {peakRoad:0.000}"
        );
    }
}
