using StintegyEVO.TrainingHost.Adapter;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// The hidden curriculum (freeze design, section 2): the draws have the
/// designed shares, switching it on moves no other draw, it stays out of
/// evaluation, and its noise touches only the perception channels.
/// </summary>
public sealed class HiddenCurriculumTests
{
    [Fact]
    public void TheLimiterDrawHasTheDesignedShares()
    {
        int full = 0, partial = 0, off = 0, noisy = 0;
        const int samples = 10_000;
        for (int i = 0; i < samples; i++)
        {
            float pick = (i + 0.5f) / samples;
            var draw = HiddenCurriculum.FromUniforms(pick, 0.3f, pick, 0.3f);
            if (draw.LimiterStrength == 1f) full++;
            else if (draw.LimiterStrength == 0f) off++;
            else
            {
                partial++;
                Assert.InRange(draw.LimiterStrength, 0.5f, 1f);
            }
            if (draw.NoiseScale > 0f) noisy++;
        }
        Assert.Equal(0.70, full / (double)samples, 2);
        Assert.Equal(0.25, partial / (double)samples, 2);
        Assert.Equal(0.05, off / (double)samples, 2);
        Assert.Equal(0.30, noisy / (double)samples, 2);
    }

    [Fact]
    public void TheTireStressDrawHasTheDesignedShareAndBand()
    {
        int nominal = 0;
        const int samples = 10_000;
        for (int i = 0; i < samples; i++)
        {
            float pick = (i + 0.5f) / samples;
            float stress = HiddenCurriculum.TireStressFromUniforms(pick, pick);
            if (stress == 1f) nominal++;
            else Assert.InRange(stress, 0.85f, 1.15f);
        }
        Assert.Equal(0.70, nominal / (double)samples, 2);
    }

    [Fact]
    public void TheDrawnTireStressReachesTheTyresAndNotTheObservation()
    {
        DirectDriveDuelEnvironment plain = new(solo: true);
        DirectDriveDuelEnvironment hidden = new(solo: true, hiddenCurriculum: true);
        float[] a = new float[DirectDriveObservation.ObservationSize];
        float[] b = new float[DirectDriveObservation.ObservationSize];
        for (long seed = 0; seed < 200; seed++)
        {
            hidden.ResetTrack("silverstone", seed, b);
            if (hidden.Curriculum.TireStressScale == 1f ||
                hidden.Curriculum.NoiseScale > 0f ||
                hidden.Curriculum.LimiterStrength != 1f)
                continue;
            Assert.Equal(hidden.Curriculum.TireStressScale, hidden.Ego.TireConfig.TireStressScale);
            plain.ResetTrack("silverstone", seed, a);
            Assert.Equal(a, b);
            return;
        }
        Assert.Fail("no episode drew only a tyre stress scale");
    }

    [Fact]
    public void EvaluationIsNominal()
    {
        DirectDriveDuelEnvironment environment = new(solo: true);
        float[] observation = new float[DirectDriveObservation.ObservationSize];
        for (long seed = 0; seed < 40; seed++)
        {
            environment.ResetTrack("silverstone", seed, observation);
            Assert.Equal(1f, environment.Curriculum.LimiterStrength);
            Assert.Equal(0f, environment.Curriculum.NoiseScale);
            Assert.Equal(1f, environment.Ego.CarConfig.CombinedGripLimiterStrength);
            Assert.Equal(1f, environment.Ego.TireConfig.TireStressScale);
        }
    }

    [Fact]
    public void TheCurriculumMovesNoOtherDraw()
    {
        DirectDriveDuelEnvironment plain = new(solo: true, randomiseEpisodeStart: true);
        DirectDriveDuelEnvironment hidden = new(
            solo: true, randomiseEpisodeStart: true, hiddenCurriculum: true);
        float[] a = new float[DirectDriveObservation.ObservationSize];
        float[] b = new float[DirectDriveObservation.ObservationSize];
        bool sawCurriculum = false;
        for (long seed = 0; seed < 40; seed++)
        {
            plain.Reset(seed, a);
            hidden.Reset(seed, b);
            Assert.Equal(plain.TrackFamily, hidden.TrackFamily);
            Assert.Equal(plain.EgoStartS, hidden.EgoStartS);
            Assert.Equal(plain.EgoStrategy, hidden.EgoStrategy);
            Assert.Equal(
                plain.Ego.State.FrontLeft.Wear, hidden.Ego.State.FrontLeft.Wear);
            sawCurriculum |= hidden.Curriculum.LimiterStrength < 1f ||
                             hidden.Curriculum.NoiseScale > 0f;
        }
        Assert.True(sawCurriculum, "forty seeds drew nothing but nominal");
    }

    [Fact]
    public void NoiseTouchesOnlyThePerceptionChannels()
    {
        DirectDriveDuelEnvironment plain = new(solo: true);
        DirectDriveDuelEnvironment hidden = new(solo: true, hiddenCurriculum: true);
        float[] a = new float[DirectDriveObservation.ObservationSize];
        float[] b = new float[DirectDriveObservation.ObservationSize];
        var noisy = new HashSet<int>(HiddenCurriculum.NoisyChannels.Select(c => c.Channel));
        bool checkedOne = false;
        for (long seed = 0; seed < 200 && !checkedOne; seed++)
        {
            hidden.ResetTrack("silverstone", seed, b);
            if (hidden.Curriculum.NoiseScale == 0f || hidden.Curriculum.LimiterStrength != 1f)
                continue;
            plain.ResetTrack("silverstone", seed, a);
            bool anyNoise = false;
            for (int c = 0; c < a.Length; c++)
            {
                if (noisy.Contains(c))
                    anyNoise |= a[c] != b[c];
                else
                    Assert.Equal(a[c], b[c]);
            }
            Assert.True(anyNoise, "a noisy episode left every perception channel clean");
            checkedOne = true;
        }
        Assert.True(checkedOne, "no noisy episode with a full-strength limiter was drawn");
    }
}
