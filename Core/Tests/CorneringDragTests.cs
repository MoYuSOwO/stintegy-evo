using System;
using StintegyEVO.Core.Cars;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// What a corner costs in speed, and that the bill is written once.
///
/// The car used to pay two approximations of the same thing: a rate times
/// the square of lateral utilisation, which is the steady-state answer as a
/// curve fit, and the body's sideslip against its lateral acceleration,
/// which is that answer seen from the chassis. Both are now replaced by the
/// mechanism itself — each axle's lateral force times the sine of the angle
/// it is being dragged at — and these tests are about the difference that
/// makes rather than about the size of the number.
/// </summary>
public sealed class CorneringDragTests
{
    private const float Dt = 1f / 60f;

    private static TireConfig WarmTires() => new()
    {
        StartingSurfaceTempC = 90f,
        StartingCoreTempC = 90f
    };

    private static CarState Rolling(float speed)
    {
        CarState state = new()
        {
            Speed = speed,
            Energy = PowertrainState.Filled(0.8f)
        };
        state.InstallFreshTires(WarmTires());
        return state;
    }

    private static void Drive(
        CarState state, CarConfig car, TireConfig tires, float curvature
    )
    {
        CarPhysics.Step(
            state,
            car,
            tires,
            new CarPhysicsStepInput(
                new DriverInput(curvature, 0f),
                CarStrategy.Default,
                AirTempC: 25f,
                TrackTempC: 35f
            ),
            Dt
        );
    }

    /// <summary>
    /// The one this change exists for.
    ///
    /// A driver who saws at the wheel is working the front tyres at a large
    /// slip angle the whole time while the car goes, on average, straight
    /// ahead. The old accounting could not see it: utilisation averages out
    /// over the sawing, and the body never has time to develop a sideslip
    /// angle for the other term to read. So shaking the wheel was nearly
    /// free, and a policy that had found that out drove like it — which is
    /// how the hole was found, by watching one drive in the viewer.
    ///
    /// Measured here against a car driving straight over the same five
    /// seconds: shaking costs 0.34 m/s, where before this change it cost
    /// 0.10. The threshold is set below the first and above the second, so
    /// this test states the difference rather than pinning today's number.
    /// </summary>
    [Fact]
    public void ShakingTheWheelCostsSpeedThatDrivingStraightDoesNot()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState straight = Rolling(45f);
        CarState shaken = Rolling(45f);

        const float amplitude = 0.008f;
        for (int i = 0; i < 300; i++)
        {
            // Four physics steps of lock each way, which is one decision at
            // the rate the learned driver thinks: a driver shaking the
            // wheel, not a numerical flutter.
            float sawn = (i / 4) % 2 == 0 ? amplitude : -amplitude;
            Drive(straight, car, tires, 0f);
            Drive(shaken, car, tires, sawn);
        }

        float extra = straight.Speed - shaken.Speed;
        Assert.True(
            extra > 0.2f,
            $"sawing at the wheel cost only {extra:0.000} m/s over five " +
            "seconds, which is the hole this accounting was meant to close"
        );
    }

    /// <summary>
    /// The size of the bill in a corner a car actually holds, stated so
    /// that a change to it has to be argued.
    ///
    /// Held at a corner the tyres can carry, the induced drag is the
    /// lateral acceleration times the sine of the slip angle the axles
    /// settle at — a few tenths of a metre per second squared at half the
    /// circle, rising towards a couple at the limit, where the peak slip
    /// angle is eight degrees and its sine is 0.14. The retired flat rate
    /// charged 1.15 at full utilisation whatever the angle was.
    /// </summary>
    [Fact]
    public void ASteadyCornerIsChargedTheSineOfTheAngleItIsHeldAt()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = Rolling(45f);
        for (int i = 0; i < 240; i++)
            Drive(state, car, tires, 0.006f);

        CarTelemetry telemetry = state.Telemetry;
        float lateral = MathF.Abs(telemetry.ActualLateralAccel);
        Assert.True(lateral > 5f, "the fixture stopped cornering");
        Assert.True(
            telemetry.InducedDragAccel > 0f,
            "a car in a corner is paying nothing for it"
        );
        // Between the whole of the peak angle's sine and a tenth of it: the
        // car is somewhere on the curve, and the drag is the force times
        // that angle's sine rather than a rate times anything squared.
        float atPeak = lateral * MathF.Sin(0.13962634f);
        Assert.InRange(telemetry.InducedDragAccel, atPeak * 0.1f, atPeak * 1.2f);
    }

    /// <summary>
    /// A car sliding is charged at least what the body angle alone used to
    /// say, because the axles are dragged at least as far as the body is.
    /// This is the half of the old accounting that was doing real work, and
    /// losing it would make sliding cheap.
    /// </summary>
    [Fact]
    public void ASlideCostsAtLeastWhatTheBodyAngleSaid()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = Rolling(50f);
        for (int i = 0; i < 40; i++)
            Drive(state, car, tires, 0.02f);

        CarTelemetry telemetry = state.Telemetry;
        float bodyReading = MathF.Abs(
            telemetry.ActualLateralAccel *
            MathF.Sin(state.SideslipAngleRadians)
        );
        Assert.True(
            MathF.Abs(state.SideslipAngleRadians) > 0.2f,
            "the fixture stopped sliding"
        );
        Assert.True(
            telemetry.InducedDragAccel >= bodyReading * 0.95f,
            $"a slide at {state.SideslipAngleRadians:0.000} rad was charged " +
            $"{telemetry.InducedDragAccel:0.000} against the body reading's " +
            $"{bodyReading:0.000}"
        );
    }

    /// <summary>
    /// The published envelope quotes what the physics charges.
    ///
    /// A speed plan reads <see cref="CarPhysics.EstimatePerformanceLimits"/>
    /// and the car pays <see cref="CarPhysics.Step"/>; when those two price
    /// a corner differently, every plan is wrong by the difference and
    /// nothing downstream can tell. The estimate cannot know the angles, so
    /// it solves them from the force each axle is asked for — the same
    /// curve, read backwards — and lands within a few percent of what the
    /// car settles at.
    /// </summary>
    [Theory]
    [InlineData(0.002f)]
    [InlineData(0.004f)]
    [InlineData(0.006f)]
    public void ThePublishedEnvelopeQuotesWhatTheCarIsCharged(float curvature)
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = Rolling(45f);
        for (int i = 0; i < 240; i++)
            Drive(state, car, tires, curvature);

        CarPerformanceLimits limits = CarPhysics.EstimatePerformanceLimits(
            state, car, tires, CarStrategy.Default, state.Speed, curvature
        );
        float charged = state.Telemetry.LossAccel;
        Assert.InRange(limits.LossAcceleration, charged * 0.95f, charged * 1.05f);
    }

    /// <summary>
    /// A crawling car pays nothing for its wheels being turned.
    ///
    /// Below the speed where the model carries a sideslip at all, the car
    /// follows its wheels: there is no angle between the rubber and its
    /// travel, so there is no induced drag to charge. Reading the angles
    /// down there anyway bills a crawling car the whole of its grip, and a
    /// car nosed into a barrier can then never pull away from it — a
    /// deadlock this model has been dug out of twice already.
    /// </summary>
    [Fact]
    public void ACrawlingCarIsNotChargedForItsWheelsBeingTurned()
    {
        CarConfig car = new();
        TireConfig tires = WarmTires();
        CarState state = Rolling(2f);
        for (int i = 0; i < 30; i++)
        {
            CarPhysics.Step(
                state,
                car,
                tires,
                new CarPhysicsStepInput(
                    new DriverInput(car.MaxCurvatureRequest, 3f),
                    CarStrategy.Default,
                    AirTempC: 25f,
                    TrackTempC: 35f
                ),
                Dt
            );
        }

        Assert.Equal(0f, state.Telemetry.InducedDragAccel, 4);
        Assert.True(
            state.Speed > 2f,
            $"a car at walking pace on full lock lost speed ({state.Speed:0.00} m/s)"
        );
    }
}
