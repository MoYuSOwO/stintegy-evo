using StintegyEVO.GodotApp.LowPoly;
using Xunit;

namespace StintegyEVO.Core.Tests;

public class PresentationTimingTests
{
    [Theory]
    [InlineData(30)]
    [InlineData(60)]
    [InlineData(120)]
    [InlineData(144)]
    public void FixedStepsPreserveRealTimeAtDifferentFrameRates(int fps)
    {
        var budget = new FixedStepBudget();
        int steps = 0;
        for (int frame = 0; frame < fps * 10; frame++)
        {
            budget.Advance(1.0 / fps);
            steps += budget.TakeSteps();
        }
        Assert.Equal(600, steps);
    }

    [Fact]
    public void LongStallHasBoundedCatchUpAndPauseDropsDebt()
    {
        var budget = new FixedStepBudget();
        budget.Advance(10);
        Assert.Equal(6, budget.TakeSteps());
        Assert.Equal(0, budget.TakeSteps());
        budget.Advance(0.01);
        budget.Reset();
        Assert.Equal(0, budget.TakeSteps());
    }

    [Fact]
    public void BatchedDeliveryDoesNotResetInterpolatedMotion()
    {
        var timeline = new SnapshotTimeline<double>(0);
        int produced = 0;
        // Deliver irregular batches ahead of playback, as a background worker does.
        for (int frame = 0; frame < 1000; frame++)
        {
            double time = frame / 144.0;
            if (frame % 5 == 0)
                for (int n = 0; n < 3; n++)
                {
                    double snapshotTime = ++produced / 60.0;
                    timeline.Add(snapshotTime, snapshotTime * 50);
                }
            var (previous, next, fraction) = timeline.Sample(time);
            Assert.Equal(time * 50, previous + (next - previous) * fraction, 5);
        }
    }
}
