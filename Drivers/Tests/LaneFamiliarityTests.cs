using StintegyEVO.Core.Content;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class LaneFamiliarityTests
{
    [Fact]
    public void AScareHoldsBackOnlyWhatTheDriverWasDoingOnTheWayIn()
    {
        LaneFamiliarity f = LaneFamiliarity.NewCircuit(2000f, adaptability: 1f);
        int at = f.SegmentOf(1000f);
        f.Scare(1000f, LaneFamiliarity.Pedal.Braking, big: false);

        Assert.True(f.Brake[at] < 1f);
        Assert.True(f.Brake[at - 1] < 1f);
        Assert.Equal(1f, f.Brake[at + 1]);
        Assert.Equal(1f, f.Corner[at]);
        Assert.Equal(1f, f.Traction[at]);
    }

    [Fact]
    public void NoScareTakesItPastTheFloorAndNoCleanLapPastTheNotes()
    {
        LaneFamiliarity f = LaneFamiliarity.NewCircuit(2000f, adaptability: 1f);
        int at = f.SegmentOf(500f);
        for (int k = 0; k < 100; k++)
            f.Scare(500f, LaneFamiliarity.Pedal.Corner, big: true);
        Assert.Equal(LaneFamiliarity.Floor, f.Corner[at], 4);

        float held = f.Corner[at];
        f.Clean(at, adaptability: 0.5f);
        Assert.True(f.Corner[at] > held);
        for (int k = 0; k < 500; k++)
            f.Clean(at, adaptability: 1f);
        Assert.True(f.Corner[at] <= 1f);
    }

    [Fact]
    public void AQuickLearnerGetsBackSoonerThanASlowOne()
    {
        LaneFamiliarity quick = LaneFamiliarity.NewCircuit(2000f, 1f), slow = LaneFamiliarity.NewCircuit(2000f, 1f);
        quick.Scare(500f, LaneFamiliarity.Pedal.Throttle, big: true);
        slow.Scare(500f, LaneFamiliarity.Pedal.Throttle, big: true);
        int at = quick.SegmentOf(500f);
        quick.Clean(at, adaptability: 1f);
        slow.Clean(at, adaptability: 0f);
        Assert.True(quick.Traction[at] > slow.Traction[at]);
    }

    [Fact]
    public void ANewCircuitStartsShortOfTheNotesForADriverSlowToAdapt()
    {
        LaneFamiliarity slow = LaneFamiliarity.NewCircuit(3000f, adaptability: 0f);
        LaneFamiliarity quick = LaneFamiliarity.NewCircuit(3000f, adaptability: 1f);
        Assert.True(slow.Corner[0] < quick.Corner[0]);
        Assert.True(slow.Corner[0] >= LaneFamiliarity.Floor);
    }

    [Fact]
    public void WhatADriverLearnedComesBackFromTheSave()
    {
        string root = Path.Combine(Path.GetTempPath(), "stintegy-progress-" + Guid.NewGuid().ToString("N"));
        try
        {
            var progress = new StintegyEVO.Core.Saves.SaveFolder(root).Slot(
                new StintegyEVO.Core.Saves.SaveKey(LanesPack.Id, "driver_a", ResourceId.Parse(StockCar.Id), ResourceId.Parse("silverstone")));
            LaneFamiliarity f = LaneFamiliarity.NewCircuit(5891f, adaptability: 0.4f);
            f.Scare(2000f, LaneFamiliarity.Pedal.Braking, big: true);
            f.Laps = 7;
            progress.Write("familiarity.json", f.ToJson());

            LaneFamiliarity? back = LaneFamiliarity.FromJson(progress.Read("familiarity.json")!);
            Assert.NotNull(back);
            Assert.True(back!.Fits(5891f));
            Assert.Equal(f.Laps, back.Laps);
            Assert.Equal(f.Brake, back.Brake);
            Assert.False(back.Fits(7004f));
        }
        finally
        {
            if (Directory.Exists(root))
                Directory.Delete(root, recursive: true);
        }
    }
}
