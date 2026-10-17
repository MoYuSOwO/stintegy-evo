using StintegyEVO.Core.Cars;
using StintegyEVO.Drivers.Lanes;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class TurnLagTests
{
    private static readonly TireConfig Warm = DevelopmentTyres.Super;

    /// <summary>
    /// The game's car answers the wheel within a few hundredths of a second
    /// once it is in a corner: well under the 0.15 s the driver used to read
    /// ahead by, which put it half a metre wide out of every corner.
    /// </summary>
    [Fact]
    public void TheGamesCarAnswersTheWheelInHundredthsOfASecond()
    {
        TurnLag lag = TurnLag.Measure(new CarConfig(), Warm);
        foreach (float speed in new[] { 15f, 30f, 50f, 80f })
            Assert.InRange(lag.Seconds(speed), 0.003f, 0.1f);
    }

    /// <summary>A car harder to turn about itself is read as slower to take up a turn.</summary>
    [Fact]
    public void ASlowerCarIsReadFurtherAhead()
    {
        var quick = new CarConfig();
        var slow = new CarConfig { YawInertiaKgM2 = quick.YawInertiaKgM2 * 3f };
        TurnLag a = TurnLag.Measure(quick, Warm), b = TurnLag.Measure(slow, Warm);
        foreach (float speed in new[] { 30f, 50f, 70f })
            Assert.True(b.Seconds(speed) > a.Seconds(speed) * 1.3f, $"at {speed} m/s: {b.Seconds(speed):F3} s against {a.Seconds(speed):F3} s");
    }
}
