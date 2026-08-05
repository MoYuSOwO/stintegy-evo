using System;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;
using Xunit.Abstractions;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// End to end: a faster car behind a slower one, with nothing anywhere in the
/// code telling either of them to overtake.
/// </summary>
public class OvertakingTests(ITestOutputHelper output)
{
    private const float Dt = 1f / 60f;
    private const float StartingGapMeters = 25f;

    [Theory]
    [InlineData("Silverstone", 60)]
    [InlineData("Shanghai", 150)]
    public void AFasterCarGetsPastASlowerOneOnItsOwn(string circuit, int seconds)
    {
        TrackData track = circuit == "Shanghai"
            ? TrackFactory.ShanghaiStyleTestTrack()
            : TrackFactory.SilverstoneStyleTestTrack();
        RaceSimulation simulation = new(track);

        ReferenceLineDriver leaderDriver = new(new DriverProfile(
            "slow",
            new DriverAbilities { Pace = 55f, Consistency = 100f },
            randomSeed: 11UL
        ));
        RaceCar leader = CreateCar(
            "leader", track, 245f, 40f, leaderDriver,
            new CarStrategy(TireUsageMode.Protect, BatteryOutputMode.Save));
        ReferenceLineDriver chaserDriver = new(new DriverProfile(
            "fast",
            new DriverAbilities { Pace = 100f, Consistency = 100f },
            randomSeed: 12UL
        ));
        RaceCar chaser = CreateCar(
            "chaser", track, 245f - StartingGapMeters, 40f, chaserDriver,
            new CarStrategy(TireUsageMode.Attack, BatteryOutputMode.Attack));

        simulation.AddCar(leader);
        simulation.AddCar(chaser);

        int offLine = 0;
        int frames = 0;
        int contacts = 0;
        bool touching = false;
        float widestOffset = 0f;
        for (int step = 0; step < 60 * seconds; step++)
        {
            simulation.Step(Dt);
            frames++;
            if (chaserDriver.LastTacticalOffsetMeters != 0f)
                offLine++;

            TrackPose pose = track.Project(chaser.State.Position);
            widestOffset = MathF.Max(
                widestOffset,
                MathF.Abs(pose.D - pose.Sample.RefOffset)
            );
            bool overlapping = CarContactResolver.AreOverlapping(chaser, leader);
            if (overlapping && !touching)
                contacts++;
            touching = overlapping;
        }

        // Both cars start with zero distance travelled, so the gap covered is
        // how much further the chaser went. Getting past means clearing the
        // ground it started behind, not merely closing on it.
        float gained = chaser.Progress.TotalDistance -
                       leader.Progress.TotalDistance;
        output.WriteLine(
            $"started {StartingGapMeters:F0} m back, gained {gained:F1} m over {seconds} s");
        output.WriteLine(
            $"off line on {offLine * 100.0 / frames:F1}% of frames, " +
            $"widest {widestOffset:F2} m, {contacts} contact(s)");
        output.WriteLine($"last phase {chaserDriver.LastTacticalPhase}");

        Assert.True(
            gained > StartingGapMeters,
            $"the chaser gained {gained:F1} m, so it never got past"
        );
        Assert.Equal(0, contacts);
    }

    private static RaceCar CreateCar(
        string id,
        TrackData track,
        float startS,
        float speed,
        IRaceDriver driver,
        CarStrategy strategy
    )
    {
        TrackSample sample = track.Sample(startS);
        return new RaceCar(
            id,
            new CarConfig(),
            new TireConfig
            {
                StartingSurfaceTempC = 88f,
                StartingCoreTempC = 86f
            },
            driver,
            new CarState
            {
                Position = sample.RefPosition,
                Heading = sample.RefHeading,
                Speed = speed,
                BatterySoc = 0.9f
            }
        )
        {
            Strategy = strategy
        };
    }
}
