using System;
using StintegyEVO.TrainingHost.Environment;
using Xunit;

namespace StintegyEVO.TrainingHost.Tests;

/// <summary>
/// What an episode is allowed to start as.
///
/// The distribution exists because a policy trained only on fresh warm
/// tyres has never met the car it fails in. What matters about it is that
/// its edges come from the model rather than from the current exam, and
/// that the tails are thin but not empty.
/// </summary>
public sealed class EpisodeStartDistributionTests
{
    private static readonly EpisodeStartDistribution Default = new();

    /// <summary>
    /// The band a race spends its life in gets most of the draws, and
    /// everything else gets the rest. The point of the tail is not to make
    /// the policy good at driving a ruined tyre — it is to stop a ruined
    /// tyre being outside the distribution when one turns up.
    /// </summary>
    [Fact]
    public void MostDrawsLandInTheBandARaceActuallyLivesIn()
    {
        int normal = 0;
        const int draws = 10_000;
        for (int i = 0; i < draws; i++)
        {
            float band = (i + 0.5f) / draws;
            EpisodeStart start = Default.Draw(band, 0.5f, 0.5f, 0.5f);
            if (start.Wear <= Default.NormalWearMax + 1e-5f &&
                start.SurfaceTempC >= Default.NormalTempMinC - 1e-3f)
            {
                normal++;
            }
        }
        Assert.InRange(normal / (float)draws, 0.65f, 0.75f);
    }

    /// <summary>
    /// The extreme end reaches the model's own limits, and those limits
    /// are the model's rather than the exam's.
    ///
    /// The temperature ceiling is not a taste: grip falls off above the
    /// ideal window as the square of the excess and is clamped at a floor,
    /// and those two numbers put the last informative temperature at about
    /// 140 C. Above it every temperature is the same car.
    /// </summary>
    [Fact]
    public void TheTailReachesTheModelsOwnEdges()
    {
        EpisodeStart hottest = Default.Draw(0.99f, 1f, 1f, 1f);
        EpisodeStart coldest = Default.Draw(0.99f, 0f, 0f, 0f);

        Assert.Equal(
            EpisodeStartLimits.MaxCalibratedTempC, hottest.SurfaceTempC, 3
        );
        Assert.Equal(EpisodeStartLimits.MaxCalibratedWear, hottest.Wear, 3);
        Assert.Equal(EpisodeStartLimits.AmbientTempC, coldest.SurfaceTempC, 3);
        Assert.Equal(0f, coldest.Wear, 3);
    }

    /// <summary>
    /// Cold has to be reachable. An out lap, a restart and the first corner
    /// after a safety car are all cold, and cold rubber slides for entirely
    /// different reasons than worn rubber does.
    /// </summary>
    [Fact]
    public void ColdRubberIsInsideTheDistribution()
    {
        EpisodeStart cold = Default.Draw(0.99f, 0f, 0f, 0.5f);
        Assert.True(
            cold.SurfaceTempC <= EpisodeStartLimits.AmbientTempC + 1e-3f,
            $"the cold tail has to reach the air, got {cold.SurfaceTempC:F1} C"
        );
        Assert.Equal(cold.SurfaceTempC, cold.CoreTempC, 3);
    }

    /// <summary>
    /// Every draw is somewhere a car can physically be. A start outside
    /// that is not a hard case, it is a car the model has no opinion about.
    /// </summary>
    [Theory]
    [InlineData(0.0f)]
    [InlineData(0.35f)]
    [InlineData(0.69f)]
    [InlineData(0.71f)]
    [InlineData(1.0f)]
    public void EveryDrawIsSomewhereACarCanBe(float band)
    {
        for (int i = 0; i <= 10; i++)
        {
            float u = i / 10f;
            EpisodeStart start = Default.Draw(band, u, u, u);
            Assert.InRange(start.Wear, 0f, EpisodeStartLimits.MaxCalibratedWear);
            Assert.InRange(
                start.SurfaceTempC,
                EpisodeStartLimits.AmbientTempC,
                EpisodeStartLimits.MaxCalibratedTempC
            );
            Assert.InRange(start.Charge, 0f, 1f);
        }
    }

    /// <summary>
    /// The ranges are configurable, because a range that can only be
    /// changed by editing the engine is a range nobody will run an
    /// experiment against.
    /// </summary>
    [Fact]
    public void TheRangesAreConfigurable()
    {
        EpisodeStartDistribution narrow = Default with
        {
            ExtremeWearMax = 0.5f,
            ExtremeTempMinC = 60f
        };
        EpisodeStart start = narrow.Draw(0.99f, 1f, 0f, 0.5f);
        Assert.Equal(0.5f, start.Wear, 3);
        Assert.Equal(60f, start.SurfaceTempC, 3);
    }
}
