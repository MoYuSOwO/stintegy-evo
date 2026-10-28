using System;
using System.Linq;
using Official.SingleTrack;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Content;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The game's own car is a pack, found and loaded the way anyone's is: by
/// its namespaced id, from its folder under data/, through ICar.
/// </summary>
public sealed class CarLibraryTests
{
    [Fact]
    public void TheGamesOwnCarLoadsByItsIdLikeAnyPack()
    {
        LoadedCar car = CarLibrary.Load("official:formula_a_mixed");

        Assert.Equal(new ResourceId("official", "formula_a_mixed"), car.Id);
        Assert.Equal("Formula A (mixed)", car.Manifest.Name);
        Assert.Matches("^[0-9a-f]{12}$", car.Fingerprint);
        Assert.Single(car.Car.Ladders);
        // A bare name is the game's own, and the same pack is loaded once.
        Assert.Same(car, CarLibrary.Load("formula_a_mixed"));
        Assert.Contains(car.Id, CarLibrary.Available());
        Assert.Same(car, CarLibrary.Origin(car.Car));
        Assert.Null(CarLibrary.Origin(new SingleTrackCar(new CarConfig(), new TireConfig())));
    }

    /// <summary>
    /// A car is known by what it is, not by the bytes it was built into:
    /// the same car is the same fingerprint every time it is asked, a new
    /// version of it is another car, and so is the same pack under someone
    /// else's name.
    /// </summary>
    [Fact]
    public void ACarIsKnownByWhatItIsNotByItsBuild()
    {
        LoadedCar car = CarLibrary.Load("official:formula_a_mixed");
        Assert.Equal(car.Fingerprint, CarLibrary.Fingerprint(car.Id, car.Manifest, car.Car));
        Assert.NotEqual(car.Fingerprint,
            CarLibrary.Fingerprint(car.Id, car.Manifest with { Version = car.Manifest.Version + 1 }, car.Car));
        Assert.NotEqual(car.Fingerprint,
            CarLibrary.Fingerprint(new ResourceId("someone", car.Id.Name), car.Manifest, car.Car));
    }

    [Fact]
    public void TheGamesOwnCarIsNotPartOfTheGame()
    {
        // Its code lives in its pack's assembly, which nothing in the game
        // references: the only way in is the one every pack has.
        Assert.Contains("official.formula_a_mixed",
            AppDomain.CurrentDomain.GetAssemblies().Select(a => a.GetName().Name));
        Assert.DoesNotContain(
            typeof(ICar).Assembly.GetReferencedAssemblies(),
            name => name.Name == "official.formula_a_mixed");
    }

    [Theory]
    [InlineData("official:formula_a_mixed", "official", "formula_a_mixed")]
    [InlineData("formula_a_mixed", "official", "formula_a_mixed")]
    [InlineData("someone:their_car_2", "someone", "their_car_2")]
    public void IdsReadWithOrWithoutTheirNamespace(string text, string space, string name)
    {
        ResourceId id = ResourceId.Parse(text);
        Assert.Equal(space, id.Namespace);
        Assert.Equal(name, id.Name);
        Assert.Equal($"{space}:{name}", id.ToString());
    }

    [Theory]
    [InlineData("Official:Car")]
    [InlineData("official:")]
    [InlineData(":car")]
    [InlineData("official:my-car")]
    public void AnIdIsLowerCaseLettersDigitsAndUnderscores(string text)
    {
        Assert.False(ResourceId.TryParse(text, out _));
    }
}
