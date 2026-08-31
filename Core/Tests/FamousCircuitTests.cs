using System;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The four circuits added for the second round of coverage and testing:
/// Baku to teach narrow-between-walls, Spa to examine gradient past the
/// training range, Monza to examine braking from real speed, Interlagos
/// for anticlockwise rhythm. Each pin here is the one property the
/// circuit was brought in for; lose it and the circuit is scenery.
/// </summary>
public sealed class FamousCircuitTests
{
    [Fact]
    public void BakuIsTheNarrowestRoadInTheSet()
    {
        TrackData track = TrackFactory.BakuStyleTestTrack();
        float narrowest = 999f, steepest = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
        {
            narrowest = MathF.Min(narrowest, track.Sample(s).Width);
            steepest = MathF.Max(steepest, track.Sample(s + 0.5f).Grade);
        }
        // The castle squeeze, tighter than Monaco's 10.5.
        Assert.InRange(narrowest, 7.2f, 8.2f);
        // Bound for the training set, so its climb must stay under the
        // training maximum of 7.3 percent — Monaco's 8.6 remains a
        // gradient the policy has never seen.
        Assert.True(
            steepest < 0.073f,
            $"Baku climbs at {steepest * 100f:0.0}% and would spoil " +
            "Monaco as a gradient exam"
        );
    }

    [Fact]
    public void SpaClimbsPastTheTrainingRange()
    {
        TrackData track = TrackFactory.SpaStyleTestTrack();
        float steepest = 0f;
        for (int s = 0; s < (int)track.LengthMeters; s++)
            steepest = MathF.Max(steepest, track.Sample(s + 0.5f).Grade);
        // The compression climb: well past the training maximum of 7.3
        // and past Monaco's 8.6, which is what it is held out for.
        Assert.InRange(steepest, 0.10f, 0.14f);
    }

    [Fact]
    public void AllFourCloseAndTheAnalyticDriverLapsThemCleanly()
    {
        (string, Func<TrackData>)[] tracks =
        [
            ("baku", TrackFactory.BakuStyleTestTrack),
            ("spa", TrackFactory.SpaStyleTestTrack),
            ("monza", TrackFactory.MonzaStyleTestTrack),
            ("interlagos", TrackFactory.InterlagosStyleTestTrack),
        ];
        foreach ((string name, Func<TrackData> make) in tracks)
        {
            TrackData track = make();
            float height = 0f;
            for (int s = 0; s < (int)track.LengthMeters; s++)
                height += track.Sample(s + 0.5f).Grade;
            Assert.InRange(height, -0.05f, 0.05f);

            TrackSample start = track.Sample(0f);
            RaceCar car = new(
                name,
                new CarConfig(),
                new TireConfig
                {
                    StartingSurfaceTempC = 90f,
                    StartingCoreTempC = 90f
                },
                new ReferenceLineDriver(),
                new CarState
                {
                    Position = start.RefPosition,
                    Heading = start.RefHeading,
                    Speed = 0f,
                    Energy = PowertrainState.Filled(0.8f)
                }
            );
            RaceSimulation simulation = new(track);
            simulation.AddCar(car);
            bool touched = false;
            for (int i = 0; i < 200 * 120; i++)
            {
                simulation.Step(1f / 120f);
                if (car.LastBoundaryContact.HasValue)
                    touched = true;
            }
            Assert.False(touched, $"the analytic driver hit a wall at {name}");
            Assert.True(
                car.Progress.TotalDistance > track.LengthMeters,
                $"{name}: only {car.Progress.TotalDistance:0} m " +
                "in two hundred seconds"
            );
        }
    }
}
