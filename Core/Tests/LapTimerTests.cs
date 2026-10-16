using StintegyEVO.GodotApp.LowPoly;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The HUD's lap timer, which is presentation code with no engine in it:
/// laps timed from coarse samples must still be right to the millisecond,
/// and the running delta must say quicker when the lap is quicker.
/// </summary>
public sealed class LapTimerTests
{
    private const float Lap = 1000f;

    /// <summary>Drives at a steady speed, sampled every <paramref name="step"/> seconds, from just behind the line.</summary>
    private static LapTimer Drive(float speed, float seconds, float step, LapTimer? timer = null, float startTime = 0f, float startDistance = -10f)
    {
        timer ??= new LapTimer(Lap);
        for (float t = 0f; t <= seconds; t += step)
            timer.Sample(startTime + t, startDistance + speed * t);
        return timer;
    }

    [Fact]
    public void ALapReadFromCoarseSamplesIsRightToTheMillisecond()
    {
        // 50 m/s round a kilometre is twenty seconds, sampled every 0.37 s.
        LapTimer timer = Drive(speed: 50f, seconds: 65f, step: 0.37f);
        Assert.Equal(3, timer.Completed);
        Assert.Equal(20f, timer.LastLap, 3);
        Assert.Equal(20f, timer.BestLap, 3);
    }

    [Fact]
    public void NothingIsTimedBeforeTheLineIsCrossed()
    {
        LapTimer timer = new(Lap);
        timer.Sample(0f, -30f);
        timer.Sample(0.5f, -5f);
        Assert.Equal(0, timer.Completed);
        Assert.Equal(0f, timer.Current(0.5f));
        Assert.Null(timer.Delta(0.5f, -5f));
    }

    [Fact]
    public void TheDeltaSaysQuickerWhenTheLapIsQuicker()
    {
        // Two laps at 50 m/s set a best of twenty seconds; the third is driven faster.
        LapTimer timer = Drive(speed: 50f, seconds: 40.2f, step: 0.1f);
        Assert.Equal(20f, timer.BestLap, 3);
        float t0 = 40.2f, d0 = -10f + 50f * 40.2f;
        float t = t0, d = d0;
        for (int i = 0; i < 50; i++)
        {
            t += 0.1f;
            d += 55f * 0.1f;
            timer.Sample(t, d);
        }
        float? delta = timer.Delta(t, d);
        Assert.NotNull(delta);
        Assert.True(delta < -0.1f, $"a quicker lap reads quicker: {delta}");
    }
}
