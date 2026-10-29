using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// What a mod's code registers for each car its <c>mod.json</c> lists: a way
/// to build it. The game's own car is a mod's like any other and is found
/// the same way.
/// </summary>
public interface ICarDefinition
{
    /// <param name="compounds">
    /// The compounds the pack's <c>car.json</c> declares, loaded from their
    /// tyre packs, in the order declared. The car runs these and no others;
    /// its code says nothing about tyres' numbers.
    /// </param>
    ICar Build(IReadOnlyList<TireCompound> compounds);
}

/// <summary>
/// A car's <c>car.json</c>: what it is called, what it is built on and what
/// it runs. The code that builds it is registered by its mod under the same
/// name (<see cref="ICarDefinition"/>).
/// </summary>
public sealed record CarManifest
{
    public const string FileName = "car.json";
    public const int CurrentFormat = 1;

    public int Format { get; init; } = CurrentFormat;

    /// <summary>The car's name within its mod's namespace.</summary>
    public string Id { get; init; } = "";

    public string Name { get; init; } = "";

    /// <summary>
    /// The car's own version: its author raises it whenever the car is
    /// changed in a way that changes how it drives. Everything learned on
    /// the car -- a driver's notes -- is learned on a version, and is stale
    /// on another.
    /// </summary>
    public int Version { get; init; } = 1;

    /// <summary>The car's model, relative to its folder in the mod; none to be drawn the game's way.</summary>
    public string? Model { get; init; }

    /// <summary>
    /// The tyres the car runs: whole tyre packs (<c>official:formula_a</c>)
    /// or single compounds of them (<c>official:formula_a/medium</c>). At
    /// least one; a car that does not say what it runs does not run.
    /// </summary>
    public IReadOnlyList<string> Tyres { get; init; } = [];

    /// <summary>
    /// The chassis the car is built on (<c>official:single_track</c>), a mod's
    /// registered chassis; none for a car with physics of its own.
    /// </summary>
    public string? Chassis { get; init; }

    private static readonly JsonSerializerOptions Json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        WriteIndented = true
    };

    public static CarManifest Parse(string json) =>
        JsonSerializer.Deserialize<CarManifest>(json, Json)
        ?? throw new InvalidDataException("car.json is empty.");
}
