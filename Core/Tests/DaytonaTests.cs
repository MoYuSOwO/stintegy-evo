using System;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Daytona is in this project for one measurement: whether a policy taught
/// on twenty degrees of banking can drive on thirty-one. Every other
/// circuit here is either flat or inside the training range, so if these
/// pins break the banking dimension has stopped being tested and nothing
/// else will say so.
/// </summary>
public sealed class DaytonaTests
{
    [Fact]
    public void TheTurnsAreBankedAndTheStraightsAreNot()
    {
        TrackData track = TrackFactory.DaytonaStyleTestTrack();
        int metres = (int)track.LengthMeters;
        float steepest = 0f, shallowest = 1f;
        int banked = 0;
        for (int s = 0; s < metres; s++)
        {
            float bank = MathF.Abs(track.Sample(s).BankSlope);
            steepest = MathF.Max(steepest, bank);
            shallowest = MathF.Min(shallowest, bank);
            if (Degrees(bank) > 25f)
                banked++;
        }

        Assert.InRange(Degrees(steepest), 30f, 32f);
        Assert.InRange(Degrees(shallowest), 2f, 4f);
        // Two turns of a thousand feet is 1,916 metres of arc, and the
        // part of each transition already past twenty-five degrees adds
        // about another 130. If this drifts far either way the banking has
        // leaked onto a straight or stopped reaching the middle of a turn.
        Assert.InRange(banked, (int)(metres * 0.44f), (int)(metres * 0.56f));
    }

    [Fact]
    public void TheBankingWindsOnRatherThanArriving()
    {
        // Reading the banking off the curvature instead puts it on in a
        // metre, and a car meets a thirty-one degree corner's lateral
        // gravity in one wheel rotation.
        TrackData track = TrackFactory.DaytonaStyleTestTrack();
        int metres = (int)track.LengthMeters;
        float worst = 0f;
        for (int s = 0; s < metres; s++)
        {
            worst = MathF.Max(
                worst,
                MathF.Abs(track.Sample(s + 1).BankSlope -
                          track.Sample(s).BankSlope)
            );
        }
        Assert.True(
            worst < 0.02f,
            $"banking changes by {worst:0.0000} per metre at its sharpest"
        );
    }

    private static float Degrees(float tangent) =>
        MathF.Atan(tangent) * 180f / MathF.PI;
}
