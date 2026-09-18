using System.Reflection;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class DriverAbilityTests
{
    public static IEnumerable<object[]> InvalidRatings()
    {
        foreach (PropertyInfo property in typeof(DriverAbilities).GetProperties())
        foreach (float value in new[] { -1f, 101f, float.NaN, float.PositiveInfinity, float.NegativeInfinity })
            yield return new object[] { property.Name, value };
    }

    [Theory]
    [MemberData(nameof(InvalidRatings))]
    public void AllRatingsMustBeFiniteAndOnTheManagerScale(string property, float value)
    {
        DriverAbilities ratings = new();
        typeof(DriverAbilities).GetProperty(property)!.SetValue(ratings, value);
        Assert.Throws<ArgumentOutOfRangeException>(() => new DriverProfile("driver", ratings));
    }

    [Fact]
    public void IdentityAndSeedDoNotDependOnAControllerImplementation()
    {
        Assert.Throws<ArgumentException>(() => new DriverProfile(" ", new()));
        Assert.Throws<ArgumentNullException>(() => new DriverProfile("driver", null!));
        Assert.Equal(1UL, new DriverProfile("driver", new(), 0).RandomSeed);
        DriverProfile profile = new("driver", new DriverAbilities { Pace = 72f }, 42);
        var controller = new FixedController();
        Driver driver = new(profile, controller);
        Assert.Same(profile, driver.Profile);
        Assert.Same(controller, driver.Controller);
        Assert.Throws<ArgumentNullException>(() => new Driver(null!, controller));
        Assert.Throws<ArgumentNullException>(() => new Driver(profile, null!));
    }

    [Fact]
    public void AbilitiesAreDeliveredButDoNotSecretlyChangePhysicalResults()
    {
        TrackData track = TrackFactory.SimpleTestTrack();
        TrackSample at = track.Sample(100f);
        (RaceSimulation Race, RaceCar Car, FixedController Controller) Entry(float ability)
        {
            FixedController controller = new();
            DriverProfile profile = new("driver", new DriverAbilities
            {
                Pace = ability, Consistency = ability, CarControl = ability,
                TireManagement = ability, Adaptability = ability,
                Reactions = ability, Awareness = ability, Overtaking = ability, Defending = ability
            }, 42);
            RaceCar car = new("car", new(), new(), new Driver(profile, controller),
                new CarState { Position = at.Center, Heading = at.Heading, Speed = 10f });
            RaceSimulation race = new(track);
            race.AddCar(car);
            return (race, car, controller);
        }
        var low = Entry(0f);
        var high = Entry(100f);
        low.Race.Step(0.5f);
        high.Race.Step(0.5f);
        Assert.Equal(0f, low.Controller.Profile!.Abilities.Pace);
        Assert.Equal(100f, high.Controller.Profile!.Abilities.Pace);
        Assert.Equal(low.Car.State.Position, high.Car.State.Position);
        Assert.Equal(low.Car.State.Speed, high.Car.State.Speed);
        Assert.Equal(low.Car.State.FrontLeft.Wear, high.Car.State.FrontLeft.Wear);
        Assert.Equal(low.Car.State.Energy, high.Car.State.Energy);
    }

    private sealed class FixedController : IDriverController
    {
        public DriverProfile? Profile { get; private set; }
        public DriverInput GetControl(in DriverContext context, float dt)
        {
            Profile = context.Profile;
            return new(0f, 2f);
        }
    }
}
