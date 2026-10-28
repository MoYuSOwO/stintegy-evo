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
        TrackData track = TestTracks.SingaporeStyleTestTrack();
        float narrowest = 999f, steepest = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
        {
            narrowest = MathF.Min(narrowest, track.Sample(s).Width);
            steepest = MathF.Max(
                steepest, MathF.Abs(track.Sample(s + 0.5f).Grade));
        }
        Assert.InRange(narrowest, 8.5f, 10.5f);
        // Marina Bay is flat and pretending otherwise would paper over the
        // gradient hole with fiction. Bridges only.
        Assert.True(steepest < 0.03f, $"Singapore climbs {steepest * 100:0.0}%");
    }

    [Fact]
    public void PortimaoTakesTheTrainingRangePastMonaco()
    {
        TrackData track = TestTracks.PortimaoStyleTestTrack();
        float up = 0f, down = 0f, height = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
        {
            float grade = track.Sample(s + 0.5f).Grade;
            height += grade;
            up = MathF.Max(up, grade);
            down = MathF.Min(down, grade);
        }
        Assert.InRange(height, -0.05f, 0.05f);
        // The rollercoaster: steep both ways, by the survey.
        Assert.InRange(up, 0.05f, 0.18f);
        Assert.InRange(-down, 0.05f, 0.18f);
    }
}
