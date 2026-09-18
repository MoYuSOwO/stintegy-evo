using System;
using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The README's shortest path to a car that moves, kept honest: the example
/// controller and host loop below are the ones the README prints, and this
/// test drives them round a circuit. If the README's code stops compiling or
/// stops going round, this fails.
/// </summary>
public sealed class ReadmeExampleTests
{
    [Fact]
    public void TheReadmeExampleDrivesALap()
    {
        // --- README example: host loop ---
        TrackData track = TrackFactory.SilverstoneStyleTestTrack();
        var simulation = new RaceSimulation(track);
        var profile = new DriverProfile("example", new DriverAbilities());
        TrackSample start = track.Sample(track.StartingLineS);
        var car = new RaceCar(
            "car-1",
            new CarConfig(),
            new TireConfig(),
            new Driver(profile, new CentrelineFollower()),
            new CarState { Position = start.Center, Heading = start.Heading }
        );
        simulation.AddCar(car);
        for (int i = 0; i < 60 * 240; i++)   // four simulated minutes
            simulation.Step(1f / 60f);
        // --- end of README example ---

        Assert.True(
            car.Progress.TotalDistance > track.LengthMeters,
            $"covered {car.Progress.TotalDistance:F0} m of a " +
            $"{track.LengthMeters:F0} m lap"
        );
        Assert.Equal(0, car.State.SpinEvents);
    }

    // --- README example: controller ---
    /// <summary>
    /// Aims at a point on the centreline ahead and slows for the curvature
    /// it can see coming. Not a racing driver; a demonstration of the seam.
    /// </summary>
    private sealed class CentrelineFollower : IDriverController
    {
        public DriverInput GetControl(in DriverContext context, float dt)
        {
            RaceCarSnapshot car = context.Car;
            float lookahead = 12f + 0.6f * car.SpeedMetersPerSecond;
            Vector2 aim = context.Track.Sample(car.TrackS + lookahead).Center;

            // Pure pursuit: the arc through the aim point, in the car's frame.
            Vector2 offset = aim - car.Position;
            float left = -MathF.Sin(car.HeadingRadians) * offset.X +
                         MathF.Cos(car.HeadingRadians) * offset.Y;
            float curvature = 2f * left / MathF.Max(offset.LengthSquared(), 1f);

            // Slow for the tightest bend in the next 200 m.
            float tightest = 0f;
            for (float d = 0f; d <= 200f; d += 10f)
                tightest = MathF.Max(
                    tightest,
                    MathF.Abs(context.Track.Sample(car.TrackS + d).Curvature)
                );
            float target = MathF.Min(60f, MathF.Sqrt(14f / MathF.Max(tightest, 1e-4f)));
            float accel = Math.Clamp(2f * (target - car.SpeedMetersPerSecond), -20f, 8f);
            return new DriverInput(curvature, accel);
        }
    }
    // --- end of README example ---
}
