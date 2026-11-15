using StintegyEVO.Core.Cars;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class BrakeFeelAbilityTests
{
    private static GripUse Axles(float front, float rear, float frontLat = 0f, float rearLat = 0f) =>
        new(front, rear, frontLat, rearLat, 0f, 0f, 0f);

    [Fact]
    public void BrakeCircle_UsesStyleBrakeWithoutFamiliarity()
    {
        float noStyle = LaneDriver.BrakeCircle(0.98f, null, 0.97f, 1f, 0f, 0f);
        float styled = LaneDriver.BrakeCircle(0.98f, 1f, 0.97f, 0.5f, 0.02f, 0.05f);
        Assert.Equal(0.97f, noStyle, 3);
        Assert.Equal(1.10f, styled, 3);
    }

    [Fact]
    public void StraightBrakeRoom_IsTheAxleBarrelNotRawMaxBrake()
    {
        float room = LaneDriver.DistributedBrakeRoom(Axles(30f, 25f), 0f, 0.47f, 0.97f, 0.57f, 57f);
        Assert.InRange(room, 51.0f, 51.1f);
        Assert.True(room < 57f);
    }

    [Fact]
    public void HereCorner_KeepsMoreBrakeThanAnEntryPreview()
    {
        GripUse tyres = Axles(30f, 25f, 0.05f, 0.05f);
        float here = LaneDriver.DistributedBrakeRoom(tyres, 0f, 0.47f, 0.97f, 0.57f, 57f);
        float preview = LaneDriver.DistributedBrakeRoom(tyres, 40f, 0.47f, 0.97f, 0.57f, 57f);
        Assert.True(here > preview + 15f, $"here {here:F2} should not spend the preview corner {preview:F2}");
    }

    [Fact]
    public void ColdAxles_FallBackToMaxBrake()
    {
        float frontCold = LaneDriver.DistributedBrakeRoom(Axles(0.4f, 25f), 0f, 0.47f, 0.97f, 0.57f, 57f);
        float rearCold = LaneDriver.DistributedBrakeRoom(Axles(30f, 0.5f), 0f, 0.47f, 0.97f, 0.57f, 57f);
        Assert.Equal(57f, frontCold);
        Assert.Equal(57f, rearCold);
    }

    [Fact]
    public void TechnicalOffset_ShrinksTheBarrel()
    {
        GripUse tyres = Axles(30f, 25f);
        float balanced = LaneDriver.DistributedBrakeRoom(tyres, 0f, 0.47f, 0.97f, 0.57f, 57f);
        float forward = LaneDriver.DistributedBrakeRoom(tyres, 0f, 0.47f, 0.97f, 0.64f, 57f);
        Assert.True(forward < balanced - 4f, $"forward {forward:F2} balanced {balanced:F2}");
    }
}
