using StintegyEVO.Core.Cars;

namespace StintegyEVO.Drivers;

/// <summary>
/// The car the driver is learned on, tested on and races in the tools: the
/// game's own, loaded from its pack the way the game loads it, so what is
/// learned is learned on the car that is raced and there is one definition
/// of it, not a second copy in the chassis's defaults.
/// </summary>
public static class StockCar
{
    public const string Id = "official:" + DriverFiles.StockCar;

    private static readonly Lazy<CarConfig> ConfigValue = new(() =>
        CarLibrary.Load(Id).Car is SingleTrackCar car
            ? car.Config
            : throw new InvalidOperationException($"{Id} is not a single-track car; the driver drives only those."));

    public static CarConfig Config => ConfigValue.Value;

    private static readonly Lazy<ICar> ModelValue = new(() => CarLibrary.Load(Id).Car);

    /// <summary>The stock car as the game loads it, for a driver to plan with.</summary>
    public static ICar Model => ModelValue.Value;

    /// <summary>How the stock car's grip feels: where its tyres peak, and what they hold.</summary>
    public static ICarGripFeel Feel => Model as ICarGripFeel
        ?? throw new InvalidOperationException($"{Id} does not say how its grip feels; the driver needs that.");
}
