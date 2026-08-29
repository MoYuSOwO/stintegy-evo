using System;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Zandvoort is here for one reason: it is the only modern Grand Prix
/// venue that is actually banked. Every other road circuit in this
/// simulation carries two and a half degrees of drainage crossfall and
/// teaches a car that corners are flat. If these pins break, the training
/// set has quietly stopped containing a banked road course.
/// </summary>
public sealed class ZandvoortTests
{
    [Fact]
    public void TheTwoBankedCornersAreBanked()
    {
        TrackData track = TrackFactory.ZandvoortStyleTestTrack();

        // Hugenholtz, the slow hairpin, and Arie Luyendijk, the fast final
        // corner. Both were rebanked to about eighteen degrees for the 2021
        // return; the model puts a little more on at the apex because the
        // corner's own lean is added to the crossfall underneath.
        float hugenholtz = Degrees(track, 890f);
        float luyendijk = Degrees(track, 3_900f);

        Assert.InRange(MathF.Abs(hugenholtz), 15f, 24f);
        Assert.InRange(MathF.Abs(luyendijk), 12f, 24f);
        // They turn opposite ways, so the road leans opposite ways too.
        Assert.True(
            hugenholtz * luyendijk < 0f,
            $"Hugenholtz {hugenholtz:0.0} deg and Arie Luyendijk " +
            $"{luyendijk:0.0} deg should lean into corners that turn " +
            "opposite ways"
        );
    }

    [Fact]
    public void TheRestOfTheLapIsAnOrdinaryRoadCircuit()
    {
        TrackData track = TrackFactory.ZandvoortStyleTestTrack();
        int metres = (int)track.LengthMeters;
        int steep = 0;
        for (int s = 0; s < metres; s++)
            if (MathF.Abs(Degrees(track, s)) > 6f)
                steep++;

        // A couple of corners' worth, not a speedway. Anything much more
        // than this means the banking has leaked along the lap.
        Assert.InRange(steep, 40, 500);
    }

    [Fact]
    public void ItClimbsTheDunesAndComesBackDown()
    {
        TrackData track = TrackFactory.ZandvoortStyleTestTrack();
        int metres = (int)track.LengthMeters;
        float height = 0f, lowest = 0f, highest = 0f;
        for (int s = 0; s < metres; s++)
        {
            height += track.Sample(s + 0.5f).Grade;
            lowest = MathF.Min(lowest, height);
            highest = MathF.Max(highest, height);
        }

        // Something over fifteen metres across a four kilometre lap, and
        // back to where it started, like every other closed circuit here.
        Assert.InRange(highest - lowest, 10f, 25f);
        Assert.InRange(height, -0.05f, 0.05f);
    }

    private static float Degrees(TrackData track, float s) =>
        MathF.Atan(track.Sample(s).BankSlope) * 180f / MathF.PI;
}
