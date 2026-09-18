using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Car-to-car contact as a rigid-body collision: the impulse acts at the
/// contact point, so an off-centre hit turns the cars, and afterwards the
/// cars keep pointing where they were pointing. The new direction of travel
/// is sideslip for the slip-angle dynamics to deal with.
///
/// Every scenario checks the same physical bookkeeping. Linear and angular
/// momentum are conserved exactly, because the two impulses are equal,
/// opposite and act at one point. Kinetic energy, counting rotation, never
/// rises, because restitution is below one and friction only removes it.
/// The cars end up apart, and neither heading is touched.
/// </summary>
public sealed class CarContactDynamicsTests
{
    private static readonly CarConfig Config = new();

    /// <summary>
    /// Two cars side by side on a straight, one drifting into the other's
    /// flank. Level with each other, the push is through both centres and
    /// turns neither car.
    /// </summary>
    [Fact]
    public void SideBySideSqueezePushesWithoutTurning()
    {
        RaceCar inside = Car("inside", new Vector2(0f, 0f), 0f, 40f, Slip(40f, 2f));
        RaceCar outside = Car("outside", new Vector2(0f, 1.75f), 0f, 40f);

        Collide(inside, outside);

        Assert.InRange(MathF.Abs(inside.State.YawRateRadiansPerSecond), 0f, 1e-3f);
        Assert.InRange(MathF.Abs(outside.State.YawRateRadiansPerSecond), 0f, 1e-3f);
        Assert.True(outside.State.Velocity.Y > 0f, "the outside car should be pushed away");
    }

    /// <summary>
    /// The same squeeze with the outside car a metre and a half further up the
    /// straight. The shared stretch of flank is ahead of the inside car's
    /// centre and behind the outside car's, and the two are pushed in opposite
    /// directions, so both turn the same way: the inside car is pushed right
    /// at its front, the outside car left at its rear, both clockwise.
    /// </summary>
    [Fact]
    public void StaggeredSqueezeTurnsBothCars()
    {
        RaceCar inside = Car("inside", new Vector2(0f, 0f), 0f, 40f, Slip(40f, 2f));
        RaceCar outside = Car("outside", new Vector2(1.5f, 1.75f), 0f, 40f);

        Collide(inside, outside);

        Assert.True(
            inside.State.YawRateRadiansPerSecond < -0.01f,
            $"inside yaw rate {inside.State.YawRateRadiansPerSecond}"
        );
        Assert.True(
            outside.State.YawRateRadiansPerSecond < -0.01f,
            $"outside yaw rate {outside.State.YawRateRadiansPerSecond}"
        );
    }

    /// <summary>
    /// A rear-end hit a metre off the centre line. The car in front is shoved
    /// forward and turned about the corner that was hit, which is how a
    /// clipped rear bumper sends a car round.
    /// </summary>
    [Fact]
    public void OffsetRearEndHitTurnsTheCarInFront()
    {
        RaceCar front = Car("front", new Vector2(0f, 0f), 0f, 20f);
        RaceCar rear = Car("rear", new Vector2(-4.6f, 1.0f), 0f, 35f);

        Collide(rear, front);

        Assert.True(front.State.Speed > 20f, "the car in front should be pushed");
        Assert.True(rear.State.Speed < 35f, "the car behind should be slowed");
        // Pushed forward at a point to its left: clockwise.
        Assert.True(
            front.State.YawRateRadiansPerSecond < -0.01f,
            $"front yaw rate {front.State.YawRateRadiansPerSecond}"
        );
    }

    /// <summary>
    /// Two cars whose lines cross at the apex: one on the racing line, one
    /// diving down the inside at an angle and putting its nose into the
    /// other's flank.
    /// </summary>
    [Fact]
    public void CrossingAtTheApexTurnsTheCarThatWasHit()
    {
        RaceCar onLine = Car("on-line", new Vector2(0f, 0f), 0f, 30f);
        float angle = 0.6f;
        RaceCar diving = Car(
            "diving",
            new Vector2(-2.9f, -2.2f),
            angle,
            34f
        );
        Assert.True(CarContactResolver.AreOverlapping(onLine, diving), "scenario must start in contact");

        Collide(diving, onLine);

        Assert.True(MathF.Abs(onLine.State.YawRateRadiansPerSecond) > 0.01f);
        Assert.True(onLine.State.Velocity.Y > 0f, "the hit car should be pushed off its line");
    }

    /// <summary>
    /// A car already sliding is hit on the side the slide is heading. The
    /// contact adds to the slide rather than erasing it: the old resolver
    /// snapped the heading onto the velocity and zeroed sideslip and yaw
    /// rate, a free recovery for any car that touched another.
    /// </summary>
    [Fact]
    public void AHitCarKeepsItsSlide()
    {
        float slip = 0.3f;
        RaceCar sliding = Car("sliding", new Vector2(0f, 0f), 0f, 30f, slip, yawRate: 0.5f);
        // Coming from the right and behind, pushing towards the car's left,
        // which is the way a positive sideslip is already carrying it.
        RaceCar hitter = Car("hitter", new Vector2(-1.0f, -1.8f), 0f, 30f, Slip(30f, 6f));

        Collide(hitter, sliding);

        Assert.True(
            MathF.Abs(sliding.State.SideslipAngleRadians) >= slip,
            $"sideslip {sliding.State.SideslipAngleRadians} fell below {slip}"
        );
        Assert.NotEqual(0f, sliding.State.YawRateRadiansPerSecond);
    }

    /// <summary>
    /// A stopped car T-boned at speed, then left to the full simulation for
    /// three seconds. The contact leaves it moving sideways, sideslip near a
    /// right angle, and the slip-angle dynamics have to take that without
    /// help: every state stays finite and the cars do not end up inside
    /// each other.
    /// </summary>
    [Fact]
    public void ASidewaysCarFromAContactIsTakenOverByTheDynamics()
    {
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        TrackSample at = track.Sample(track.StartingLineS + 300f);
        Vector2 left = new(-MathF.Sin(at.Heading), MathF.Cos(at.Heading));
        RaceCar stopped = new(
            "stopped", Config, new TireConfig(), null,
            new CarState { Position = at.Center, Heading = at.Heading }
        );
        RaceCar hitter = new(
            "hitter", Config, new TireConfig(), null,
            new CarState
            {
                Position = at.Center - left * 3.2f,
                Heading = at.Heading + MathF.PI / 2f,
                Speed = 20f
            }
        );
        var simulation = new RaceSimulation(track);
        simulation.AddCar(stopped);
        simulation.AddCar(hitter);

        bool touched = false;
        for (int i = 0; i < 180; i++)
        {
            simulation.Step(1f / 60f);
            touched |= stopped.HitCarThisStep;
            foreach (RaceCar car in new[] { stopped, hitter })
            {
                Assert.True(float.IsFinite(car.State.Speed));
                Assert.True(float.IsFinite(car.State.Heading));
                Assert.True(float.IsFinite(car.State.SideslipAngleRadians));
                Assert.True(float.IsFinite(car.State.YawRateRadiansPerSecond));
                Assert.True(float.IsFinite(car.State.Position.X) && float.IsFinite(car.State.Position.Y));
            }
            Assert.False(CarContactResolver.AreOverlapping(stopped, hitter), $"overlap at step {i}");
        }
        Assert.True(touched, "the hitter never reached the stopped car");
    }

    private static void Collide(RaceCar a, RaceCar b)
    {
        Assert.True(CarContactResolver.AreOverlapping(a, b), "scenario must start in contact");
        float headingA = a.State.Heading;
        float headingB = b.State.Heading;
        Vector2 velocityA = a.State.Velocity;
        Vector2 velocityB = b.State.Velocity;
        float yawA = a.State.YawRateRadiansPerSecond;
        float yawB = b.State.YawRateRadiansPerSecond;
        float energyBefore = KineticEnergy(a) + KineticEnergy(b);

        CarContactResolver.Resolve(new[] { a, b });

        Assert.False(CarContactResolver.AreOverlapping(a, b), "cars left overlapping");
        Assert.Equal(headingA, a.State.Heading);
        Assert.Equal(headingB, b.State.Heading);

        float m = Config.MassKg;
        float inertia = Config.YawInertiaKgM2;
        Vector2 momentumBefore = m * (velocityA + velocityB);
        Vector2 momentumAfter = m * (a.State.Velocity + b.State.Velocity);
        float scale = m * (velocityA.Length() + velocityB.Length());
        Assert.InRange((momentumAfter - momentumBefore).Length() / scale, 0f, 1e-4f);

        // About the origin, with the positions the impulse acted at (the
        // resolver moves the cars apart before it applies the impulse).
        float angularBefore =
            m * Cross(a.State.Position, velocityA) + inertia * yawA +
            m * Cross(b.State.Position, velocityB) + inertia * yawB;
        float angularAfter =
            m * Cross(a.State.Position, a.State.Velocity) +
            inertia * a.State.YawRateRadiansPerSecond +
            m * Cross(b.State.Position, b.State.Velocity) +
            inertia * b.State.YawRateRadiansPerSecond;
        float angularScale = m * 5f * (velocityA.Length() + velocityB.Length());
        Assert.InRange(MathF.Abs(angularAfter - angularBefore) / angularScale, 0f, 1e-4f);

        float energyAfter = KineticEnergy(a) + KineticEnergy(b);
        Assert.True(
            energyAfter <= energyBefore * (1f + 1e-5f),
            $"kinetic energy rose from {energyBefore} to {energyAfter}"
        );
    }

    private static float KineticEnergy(RaceCar car) =>
        0.5f * Config.MassKg * car.State.Speed * car.State.Speed +
        0.5f * Config.YawInertiaKgM2 *
        car.State.YawRateRadiansPerSecond * car.State.YawRateRadiansPerSecond;

    private static float Cross(Vector2 a, Vector2 b) => a.X * b.Y - a.Y * b.X;

    /// <summary>The sideslip that gives a lateral speed on top of a forward one.</summary>
    private static float Slip(float forward, float lateral) => MathF.Atan2(lateral, forward);

    private static RaceCar Car(
        string id,
        Vector2 position,
        float heading,
        float forwardSpeed,
        float sideslip = 0f,
        float yawRate = 0f
    )
    {
        float speed = forwardSpeed / MathF.Cos(sideslip);
        return new RaceCar(
            id,
            Config,
            new TireConfig(),
            null,
            new CarState
            {
                Position = position,
                Heading = heading,
                Speed = speed,
                SideslipAngleRadians = sideslip,
                YawRateRadiansPerSecond = yawRate
            }
        );
    }
}
