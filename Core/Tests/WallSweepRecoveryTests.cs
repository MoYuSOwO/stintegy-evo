using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// A car whose corner is against the wall has to be able to leave it.
///
/// The sweep that keeps a car out of the wall used to move position and
/// heading together, as one fraction of the step. When the step's rotation
/// turned a corner into the barrier at once, that fraction was zero and
/// the translation went with it -- including a translation that was taking
/// the car away from the wall. The velocity response did not help either:
/// it only acts when the centre of the car is moving into the wall, and
/// here it was moving out. So the car was put back where it started, the
/// next step asked for the same rotation, and the reference-line driver
/// sat at one spot on the test track for 5223 frames with its wheels
/// turning at six metres a second.
///
/// The state below is that spot, frozen at 70 s on the corrected thermal
/// physics (Training/diagnostics/2026-09-11-thermal-adaptation, evidence/
/// regression). It is recorded as numbers rather than re-driven, so the
/// test goes on meaning what it means when the car or the driver changes;
/// the preconditions check that it still describes a corner rotating into
/// a wall while the body moves clear, and say so if the track moves under
/// it.
/// </summary>
public sealed class WallSweepRecoveryTests
{
    private static readonly CarCollisionConfig Collision = new();

    [Fact]
    public void ARotationTheWallRefusesDoesNotCancelATranslationItAllows()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        CarState start = FrozenStart();
        CarState target = FrozenTarget();

        CarState translationOnly = FrozenTarget();
        translationOnly.Heading = start.Heading;
        CarState rotationOnly = FrozenTarget();
        rotationOnly.Position = start.Position;
        Assert.True(
            TrackBoundaryResolver.IsInsideTrackWalls(track, start, Collision) &&
            !TrackBoundaryResolver.IsInsideTrackWalls(track, target, Collision) &&
            TrackBoundaryResolver.IsInsideTrackWalls(track, translationOnly, Collision) &&
            !TrackBoundaryResolver.IsInsideTrackWalls(track, rotationOnly, Collision),
            "the frozen state no longer describes a rotation into the wall " +
            "with a clear translation; the track has moved under it and it " +
            "needs recapturing"
        );
        float freeTranslation = Vector2.Distance(start.Position, target.Position);

        TrackBoundaryContact? contact =
            TrackBoundaryResolver.ResolveSweep(track, start, target, Collision);

        Assert.True(contact.HasValue, "the rotation did reach the wall");
        Assert.True(
            TrackBoundaryResolver.IsInsideTrackWalls(track, target, Collision),
            "the resolved pose is through the wall"
        );
        float moved = Vector2.Distance(start.Position, target.Position);
        Assert.True(
            moved > freeTranslation * 0.99f,
            $"the car should keep the translation the wall allows: moved " +
            $"{moved:0.00000} m of {freeTranslation:0.00000} m"
        );
    }

    [Fact]
    public void ACarPinnedByItsOwnRotationDrivesAwayFromTheWall()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        CarState state = FrozenStart();
        state.Energy = PowertrainState.Filled(0.9f);
        state.InstallFreshTires(new TireConfig
        {
            StartingSurfaceTempC = 90f,
            StartingCoreTempC = 90f
        });
        RaceCar car = new(
            "pinned",
            new CarConfig(),
            new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f },
            new ReferenceLineDriver(),
            state
        );
        RaceSimulation simulation = new(track);
        simulation.AddCar(car);

        int contactFrames = 0;
        for (int i = 0; i < 60 * 10; i++)
        {
            simulation.Step(1f / 60f);
            if (car.LastBoundaryContact.HasValue)
                contactFrames++;
            Assert.True(
                TrackBoundaryResolver.IsInsideTrackWalls(track, car.State, car.Collision),
                $"through the wall at frame {i}"
            );
        }

        // Ten seconds from rest against the wall. Stuck, this was 600
        // contact frames and no distance at all.
        Assert.True(
            car.Progress.TotalDistance > 30f,
            $"the car should be under way again, covered {car.Progress.TotalDistance:0.0} m " +
            $"with {contactFrames} contact frames"
        );
        Assert.True(
            contactFrames < 60,
            $"the car should come off the wall within a second, took {contactFrames} frames"
        );
    }

    private static CarState FrozenStart() => new()
    {
        Position = new Vector2(539.8531f, -23.242304f),
        Heading = -3.0143077f,
        Speed = 6.2218313f,
        SideslipAngleRadians = 2.465298E-06f,
        YawRateRadiansPerSecond = 0.99164987f,
        SteerAngleRadians = 0.40112853f
    };

    // The same car one 1/120 s substep later, as the vehicle model predicted
    // it before the wall had its say.
    private static CarState FrozenTarget() => new()
    {
        Position = new Vector2(539.8017f, -23.249168f),
        Heading = -3.0021334f,
        Speed = 6.2255144f,
        SideslipAngleRadians = -0.0012807623f,
        YawRateRadiansPerSecond = 0.32306945f
    };
}
