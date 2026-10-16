using System;
using StintegyEVO.Core.Track;
using StintegyEVO.GodotApp.LowPoly;
using Xunit;

namespace StintegyEVO.Core.Tests;

public class TrackSurfaceGeometryTests
{
    private static readonly Lazy<TrackData> Circuit = new(TrackFactory.SilverstoneStyleTestTrack);

    [Fact]
    public void SurfaceClosesAtLapSeamAndWrapsNegativeDistances()
    {
        var track = Circuit.Value;
        var surface = new TrackSurfaceGeometry(track);
        Near.Equal(surface.Height(0f, 0f), surface.Height(track.LengthMeters, 0f));
        Near.Equal(surface.Height(track.LengthMeters - 5f, 2f), surface.Height(-5f, 2f));
        Assert.InRange(MathF.Abs(surface.Height(0.01f, 0f) - surface.Height(track.LengthMeters - 0.01f, 0f)), 0f, 0.01f);
    }

    [Fact]
    public void HeightGradientMatchesTheSurfaceUsedByPhysics()
    {
        var track = Circuit.Value;
        var surface = new TrackSurfaceGeometry(track);
        float min = float.PositiveInfinity, max = float.NegativeInfinity;
        for (float s = 10f; s < track.LengthMeters - 10f; s += 73f)
        {
            float gradient = (surface.Height(s + 0.25f, 0f) - surface.Height(s - 0.25f, 0f)) / 0.5f;
            Assert.InRange(MathF.Abs(gradient - track.Sample(s).Grade), 0f, 0.002f);
            min = MathF.Min(min, surface.Height(s, 0f));
            max = MathF.Max(max, surface.Height(s, 0f));
        }
        // A flat former airfield: some metres, not tens.
        Assert.InRange(max - min, 3f, 20f);
    }

    [Fact]
    public void RoadEdgesUseTheSameBankAndCrownAsPhysics()
    {
        var track = Circuit.Value;
        var surface = new TrackSurfaceGeometry(track);
        var sample = track.Sample(175f);
        float d = sample.HalfWidth;
        float expectedRise = sample.BankSlope * d + sample.BankCurvature * d * d;
        Near.Equal(expectedRise, surface.Height(175f, d) - surface.Height(175f, 0f));
        var point = surface.Point(175f, d);
        Near.Equal(sample.LeftEdge.X, point.X);
        Near.Equal(sample.LeftEdge.Y, point.Z);
        Near.Equal(surface.Height(175f, d), point.Y);
    }
}
