using Official.SingleTrack;
using StintegyEVO.Core.Cars;
using Xunit;

namespace Official.SingleTrack.Tests;

/// <summary>
/// The aero is trimmed by its own number, not by where the weight sits: a
/// car whose downforce is moved forward carries more of its load on the
/// front at speed, and none of it standing still.
/// </summary>
public sealed class AeroBalanceTests
{
    [Fact]
    public void MovingTheDownforceForwardLoadsTheFrontAtSpeedOnly()
    {
        float FrontShare(float? aeroBalance, float speed)
        {
            var car = new CarConfig { Aero = new StandardAero { FrontDownforceShare = aeroBalance } };
            var state = new CarState { Speed = speed }.WithEnergy(PowertrainState.Filled(0.8f));
            state.InstallFreshTires(new TireConfig());
            CarPhysics.Step(state, car, new TireConfig(),
                new CarStepInput(new DriverInput(0f, 0f), CarStrategy.Default, 25f, 35f), 1f / 60f);
            float front = state.Chassis().FrontLeft.LoadN + state.Chassis().FrontRight.LoadN;
            return front / (front + state.Chassis().RearLeft.LoadN + state.Chassis().RearRight.LoadN);
        }

        float trimmedWithWeight = FrontShare(null, 60f);
        float trimmedForward = FrontShare(0.6f, 60f);
        Assert.True(trimmedForward > trimmedWithWeight + 0.03f,
            $"front share {trimmedForward:P1} with the aero forward, {trimmedWithWeight:P1} with the weight");

        // Standing still there is no downforce to move.
        Assert.InRange(FrontShare(0.6f, 0.5f) - FrontShare(null, 0.5f), -0.002f, 0.002f);
    }
}
