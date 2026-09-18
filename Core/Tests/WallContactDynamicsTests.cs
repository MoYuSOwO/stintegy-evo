using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// A wall contact as an impulse at the corner that reached the wall. After
/// the contact the car keeps pointing where it was pointing and keeps its
/// slide; the wall no longer straightens a car for free.
///
/// The wall has a history, which is why these tests exist. PR #52 fixed a
/// car pinned by its own rotation: the wall refused the rotation and with
/// it the translation that would have freed the car. A literal past-peak
/// lockout once broke wall escapes too. So every scenario here checks, as
/// well as its own point, that the car stays finite, never ends up through
/// the wall, and gets away.
/// </summary>
public sealed class WallContactDynamicsTests
{
    private static readonly TrackData Track = TrackFactory.SimpleTestTrack();

    /// <summary>
    /// A pose relative to the left wall, the side where the track's lateral
    /// coordinate is positive (which is not necessarily the side the
    /// sample's normal points to), with the heading turned towards that wall
    /// by <paramref name="headingIntoWall"/>.
    /// </summary>
    /// <summary>
    /// The sign of a curvature that turns towards the left wall at
    /// <paramref name="s"/>; its negative turns away from it.
    /// </summary>
    private static float TowardsLeftWall(float s)
    {
        TrackSample at = Track.Sample(s);
        Vector2 left = Track.Project(at.Center + at.Normal).D > 0f ? at.Normal : -at.Normal;
        Vector2 forward = new(MathF.Cos(at.Heading), MathF.Sin(at.Heading));
        return forward.X * left.Y - forward.Y * left.X > 0f ? 1f : -1f;
    }

    private static (Vector2 Position, float Heading) NearLeftWall(
        float s,
        float offsetFromLeftWall,
        float headingIntoWall
    )
    {
        TrackSample at = Track.Sample(s);
        float wall = TrackBoundaryResolver.GetWallLimits(at).LeftWallD;
        Vector2 left = Track.Project(at.Center + at.Normal).D > 0f ? at.Normal : -at.Normal;
        float towardsLeft = TowardsLeftWall(s);
        return (
            at.Center + left * (wall - offsetFromLeftWall),
            at.Heading + towardsLeft * headingIntoWall
        );
    }

    /// <summary>
    /// One impulse, looked at directly: the heading is untouched, the slide
    /// is not zeroed, the corner's lever arm turns the car, and kinetic
    /// energy counting rotation does not rise.
    /// </summary>
    [Fact]
    public void AWallImpulseKeepsTheHeadingAndTheSlide()
    {
        (Vector2 position, float pointing) = NearLeftWall(100f, 1.2f, 0.35f);
        CarState state = new()
        {
            // Placed so its leading corner is just through the left wall.
            Position = position,
            Heading = pointing,
            Speed = 30f,
            SideslipAngleRadians = 0.12f,
            YawRateRadiansPerSecond = 0.3f
        };
        CarCollisionConfig collision = new();
        CarConfig car = new();
        Assert.False(
            TrackBoundaryResolver.IsInsideTrackWalls(Track, state, collision),
            "scenario must start through the wall"
        );
        float heading = state.Heading;
        float energyBefore = KineticEnergy(state, car);

        TrackBoundaryContact? contact =
            TrackBoundaryResolver.ResolveCurrent(Track, state, collision, car);

        Assert.True(contact.HasValue);
        Assert.True(TrackBoundaryResolver.IsInsideTrackWalls(Track, state, collision));
        Assert.Equal(heading, state.Heading);
        Assert.NotEqual(0f, state.SideslipAngleRadians);
        Assert.NotEqual(0.3f, state.YawRateRadiansPerSecond);
        Assert.True(
            KineticEnergy(state, car) <= energyBefore * (1f + 1e-5f),
            $"kinetic energy rose from {energyBefore} to {KineticEnergy(state, car)}"
        );
    }

    /// <summary>
    /// Pressed along the wall: a car running beside the barrier and
    /// steering gently into it the whole time. It scrapes, and it keeps
    /// going; scraping costs speed, not the ability to move.
    /// </summary>
    [Fact]
    public void SlidingAlongTheWallKeepsMoving()
    {
        (RaceCar car, _) = Run(
            s: 100f,
            offsetFromLeftWall: 1.2f,
            headingIntoWall: 0.02f,
            speed: 30f,
            command: _ => new DriverInput(0.01f * TowardsLeftWall(100f), 2f),
            seconds: 4f,
            out int contactFrames
        );

        Assert.True(contactFrames > 0, "never reached the wall");
        Assert.True(car.Progress.TotalDistance > 60f, $"covered {car.Progress.TotalDistance:F1} m");
    }

    /// <summary>
    /// A shallow graze with the wheel straight. The car touches, is turned a
    /// little by the corner that touched, and carries on.
    /// </summary>
    [Fact]
    public void AShallowGrazeCarriesOn()
    {
        (RaceCar car, float firstContactYaw) = Run(
            s: 100f,
            offsetFromLeftWall: 2.2f,
            headingIntoWall: 0.08f,
            speed: 35f,
            command: _ => new DriverInput(0f, 2f),
            seconds: 3f,
            out int contactFrames
        );

        Assert.True(contactFrames > 0, "never reached the wall");
        Assert.True(MathF.Abs(firstContactYaw) > 1e-3f, "the corner should turn the car");
        Assert.True(car.Progress.TotalDistance > 60f, $"covered {car.Progress.TotalDistance:F1} m");
    }

    /// <summary>
    /// A car at a crawl against the wall, with the kind of yaw rate the
    /// kinematic regime reports while it lays the heading onto the
    /// direction of travel. That rate is not a rotation the body carries,
    /// so the wall must not turn it into speed: the car may not come off
    /// the wall faster than it arrived, and nothing about it spins up.
    /// </summary>
    [Fact]
    public void AKinematicYawRateIsNotTurnedIntoSpeed()
    {
        (Vector2 position, float pointing) = NearLeftWall(100f, 1.2f, 0.35f);
        CarState state = new()
        {
            Position = position,
            Heading = pointing,
            Speed = 0.5f,
            YawRateRadiansPerSecond = 10f * TowardsLeftWall(100f)
        };
        CarCollisionConfig collision = new();
        Assert.False(TrackBoundaryResolver.IsInsideTrackWalls(Track, state, collision));

        TrackBoundaryResolver.ResolveCurrent(Track, state, collision, new CarConfig());

        Assert.True(TrackBoundaryResolver.IsInsideTrackWalls(Track, state, collision));
        Assert.InRange(state.Speed, 0f, 0.5f + 1e-4f);
        Assert.Equal(0f, state.YawRateRadiansPerSecond);
    }

    /// <summary>
    /// A hard hit at a sharp angle, then the driver steers away and drives
    /// off. The car model has no reverse gear, so leaving the wall means
    /// turning away from it under power. It must get clear of the wall and
    /// under way again.
    /// </summary>
    [Fact]
    public void AHardHitAtAnAngleCanDriveAway()
    {
        (RaceCar car, _) = Run(
            s: 100f,
            offsetFromLeftWall: 4f,
            headingIntoWall: 0.6f,
            speed: 25f,
            command: hit => hit
                ? new DriverInput(-0.15f * TowardsLeftWall(100f), 3f)
                : new DriverInput(0f, 0f),
            seconds: 8f,
            out int contactFrames
        );

        Assert.True(contactFrames > 0, "never reached the wall");
        float wall = TrackBoundaryResolver.GetWallLimits(Track.Project(car.State.Position).Sample).LeftWallD;
        Assert.True(
            wall - car.Progress.CurrentD > 3f,
            $"still {wall - car.Progress.CurrentD:F2} m from the wall it hit"
        );
        Assert.True(car.State.Speed > 3f, $"speed {car.State.Speed:F2}");
    }

    /// <summary>
    /// Pinned nose-first against the wall at a standstill, at an angle, and
    /// then given lock one way or the other and power. This is the shape of
    /// the old rotation deadlock: every step asks for a rotation that swings
    /// a corner into the barrier. Either way, the car has to get out.
    /// </summary>
    [Theory]
    [InlineData(-0.2f)]
    [InlineData(0.2f)]
    public void ACarPinnedNoseFirstGetsOut(float curvature)
    {
        (RaceCar car, _) = Run(
            s: 100f,
            offsetFromLeftWall: 2.3f,
            headingIntoWall: 0.6f,
            speed: 0f,
            command: _ => new DriverInput(curvature, 3f),
            seconds: 10f,
            out _
        );

        Assert.True(car.Progress.TotalDistance > 10f, $"covered {car.Progress.TotalDistance:F1} m");
        Assert.True(car.State.Speed > 3f, $"speed {car.State.Speed:F2}");
    }

    /// <summary>
    /// A known limit, kept here so it is not forgotten: a car stopped with
    /// its nose into the wall at more than about half a right angle cannot
    /// leave. Forward is into the wall, every turn swings a front corner
    /// into it, and the car model has no reverse gear. This was as true
    /// before the wall stopped aligning cars with itself as after: the same
    /// poses were stuck on the old resolver, frame for frame. Getting out
    /// needs reverse, which is a vehicle capability, not a wall rule.
    /// </summary>
    [Fact(Skip = "Known limit: no reverse gear. A car stopped nose-in at 0.9 rad or more stays pinned, on the old wall resolver as on this one.")]
    public void ACarSquareOnToTheWallCanReverseOut()
    {
        (RaceCar car, _) = Run(
            s: 100f,
            offsetFromLeftWall: 2.45f,
            headingIntoWall: MathF.PI / 2f,
            speed: 0f,
            command: _ => new DriverInput(-0.2f, 3f),
            seconds: 10f,
            out _
        );

        Assert.True(car.Progress.TotalDistance > 10f, $"covered {car.Progress.TotalDistance:F1} m");
    }


    /// <summary>
    /// Drives one car from a pose relative to the left wall, applying a
    /// command that may change once the car has touched the wall, and
    /// checks at every frame that the state is finite and inside the walls.
    /// Returns the car and the yaw rate it had just after its first contact.
    /// </summary>
    private static (RaceCar Car, float FirstContactYaw) Run(
        float s,
        float offsetFromLeftWall,
        float headingIntoWall,
        float speed,
        Func<bool, DriverInput> command,
        float seconds,
        out int contactFrames
    )
    {
        (Vector2 position, float pointing) = NearLeftWall(s, offsetFromLeftWall, headingIntoWall);
        RaceCar car = TestControlFixtures.ExternalCar(
            "wall",
            new CarState
            {
                Position = position,
                Heading = pointing,
                Speed = speed,
                Energy = PowertrainState.Filled(0.9f)
            },
            command(false)
        );
        var simulation = new RaceSimulation(Track);
        simulation.AddCar(car);

        contactFrames = 0;
        bool hit = false;
        float firstContactYaw = 0f;
        for (int i = 0; i < (int)(seconds * 60f); i++)
        {
            simulation.Step(1f / 60f);
            if (car.LastBoundaryContact.HasValue)
            {
                if (!hit)
                    firstContactYaw = car.State.YawRateRadiansPerSecond;
                hit = true;
                contactFrames++;
            }
            car.ExternalInput = command(hit);

            Assert.True(float.IsFinite(car.State.Speed), $"speed at frame {i}");
            Assert.True(float.IsFinite(car.State.Heading), $"heading at frame {i}");
            Assert.True(float.IsFinite(car.State.SideslipAngleRadians), $"sideslip at frame {i}");
            Assert.True(float.IsFinite(car.State.YawRateRadiansPerSecond), $"yaw at frame {i}");
            Assert.True(
                TrackBoundaryResolver.IsInsideTrackWalls(Track, car.State, car.Collision),
                $"through the wall at frame {i}"
            );
        }
        return (car, firstContactYaw);
    }

    private static float KineticEnergy(CarState state, CarConfig car) =>
        0.5f * car.MassKg * state.Speed * state.Speed +
        0.5f * car.YawInertiaKgM2 *
        state.YawRateRadiansPerSecond * state.YawRateRadiansPerSecond;
}
