using System;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Singapore and Portimão exist as a pair: the street circuit that is flat
/// and the steep circuit that is wide. Between them and Baku, the
/// street-circuit family keeps its dimensions separated — and Portimão's
/// climb is the one that finally takes the training range past Monaco's
/// 8.6 percent, retiring that exam on purpose.
/// </summary>
public sealed class StreetAndGradientTests
{
    [Fact]
    public void SingaporeIsNarrowAndHonestlyFlat()
    {
        TrackData track = TrackFactory.SingaporeStyleTestTrack();
        float narrowest = 999f, steepest = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
        {
            narrowest = MathF.Min(narrowest, track.Sample(s).Width);
            steepest = MathF.Max(
                steepest, MathF.Abs(track.Sample(s + 0.5f).Grade));
        }
        Assert.InRange(narrowest, 10.5f, 11.5f);
        // Marina Bay is flat and pretending otherwise would paper over the
        // gradient hole with fiction. Bridges only.
        Assert.True(steepest < 0.03f, $"Singapore climbs {steepest * 100:0.0}%");
    }

    [Fact]
    public void PortimaoTakesTheTrainingRangePastMonaco()
    {
        TrackData track = TrackFactory.PortimaoStyleTestTrack();
        float up = 0f, down = 0f, height = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
        {
            float grade = track.Sample(s + 0.5f).Grade;
            height += grade;
            up = MathF.Max(up, grade);
            down = MathF.Min(down, grade);
        }
        Assert.InRange(height, -0.05f, 0.05f);
        // The climb must clear Monaco's 8.6 percent or the circuit has
        // failed at its one job; the plunge is allowed to be wilder.
        Assert.InRange(up, 0.087f, 0.12f);
        Assert.InRange(-down, 0.10f, 0.18f);
    }
}
