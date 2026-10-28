using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Mods;

namespace StintegyEVO.Drivers;

/// <summary>
/// The car the driver's own tools learn on and lap in: the game's own, taken
/// from the registry the way the game takes it, so what is learned is
/// learned on the car that is raced. The driver itself never asks for it:
/// in a race it drives whatever car it is seated in.
/// </summary>
public static class StockCar
{
    public const string Id = "official:formula_a_mixed";

    /// <summary>The stock car as the registry has it: where its notes are kept, and what it is now.</summary>
    public static ModCar Loaded => GameMods.Registry.Cars.Get(Id);

    /// <summary>The stock car as the registry has it, for a tool to plan with.</summary>
    public static ICar Model => Loaded.Car;

    /// <summary>A stock car of its own, built by its mod's definition with the compounds it runs.</summary>
    public static ICar Build() => Loaded.Definition.Build(Loaded.Compounds);

    /// <summary>A stock car of its own that runs <paramref name="compound"/> and no other.</summary>
    public static ICar Build(string compound) => Loaded.Definition.Build([
        Loaded.Compounds.FirstOrDefault(c => c.Id == compound)
            ?? throw new ArgumentException($"{Id} does not run '{compound}'.", nameof(compound))]);

    /// <summary>How the stock car's grip feels: where its tyres peak, and what they hold.</summary>
    public static ICarGripFeel Feel => Model as ICarGripFeel
        ?? throw new InvalidOperationException($"{Id} does not say how its grip feels; the driver needs that.");
}
