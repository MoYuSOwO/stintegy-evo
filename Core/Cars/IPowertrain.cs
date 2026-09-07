using System;
using System.Collections.Generic;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// One thing a car spends over a race and cannot get back for free.
/// </summary>
/// <param name="Id">Stable name, for saved files and telemetry columns.</param>
/// <param name="Label">What to write on the gauge.</param>
/// <param name="FullMassKg">
/// What a full store of it weighs. Zero for a battery, which weighs the same
/// empty as full; a hundred kilograms or so for a fuel tank, which is the
/// whole reason a petrol car gets quicker as the race goes on.
/// </param>
public readonly record struct PowertrainResourceInfo(
    string Id,
    string Label,
    float FullMassKg
);

/// <summary>
/// A row of settings the strategist chooses between, from the most
/// conservative to the most aggressive.
///
/// The rungs are named by whoever owns the ladder, because the same five
/// positions mean different words on different machinery - a battery's
/// output settings, an engine's maps, a tyre's usage - while being the same
/// decision underneath: spend more of something now to go quicker now.
/// </summary>
public sealed class ModeLadder
{
    /// <summary>
    /// The fewest positions worth calling a ladder. One rung is not a choice,
    /// it is a fixed value wearing a control's clothes.
    /// </summary>
    public const int MinimumRungs = 2;

    public ModeLadder(string label, params string[] rungs)
    {
        if (rungs is null || rungs.Length < MinimumRungs)
        {
            throw new ArgumentException(
                $"A ladder needs at least {MinimumRungs} rungs; got " +
                $"{rungs?.Length ?? 0}.",
                nameof(rungs)
            );
        }

        Label = label;
        Rungs = rungs;
    }

    public string Label { get; }
    public IReadOnlyList<string> Rungs { get; }
    public int RungCount => Rungs.Count;

    /// <summary>
    /// Where this ladder sits when nobody has chosen: the middle, which on an
    /// odd-length ladder is the honest middle and on an even one is the lower
    /// of the two - a car that has not been told what to do should not be
    /// spending.
    /// </summary>
    public int DefaultRung => (RungCount + 1) / 2;

    /// <summary>
    /// The name of a rung, counted from one as the settings themselves are.
    /// </summary>
    public string RungLabel(int ordinal)
    {
        return Rungs[Clamp(ordinal) - 1];
    }

    /// <summary>
    /// The nearest rung this ladder actually has. Called wherever a setting
    /// arrives from somewhere that did not know how long this ladder is.
    /// </summary>
    public int Clamp(int ordinal)
    {
        return Math.Clamp(ordinal, 1, RungCount);
    }
}

/// <summary>
/// What turns the wheels, and what it costs to do so.
///
/// The abstraction is not "a battery, generalised". It is the observation
/// that a strategy table is the same game whatever is under the engine cover:
/// some stores that run down, and some ladders that trade a store against
/// pace. A hybrid picks an engine map and a deployment setting; a petrol car
/// picks a map and watches a tank; an electric car picks an output setting
/// and watches a pack. Same instrument, same decision, different plumbing -
/// so the plumbing is what lives behind this interface and nothing else does.
///
/// Everything here speaks in watts, joules and kilograms. Acceleration is the
/// chassis's word, not the powertrain's: the powertrain does not know what
/// the car weighs or how much grip it has, and is told both when it is asked
/// what it can do.
/// </summary>
public interface IPowertrain
{
    /// <summary>
    /// What this powertrain spends, in the order its stores are stored. The
    /// count of these is how many slots of a <see cref="PowertrainState"/>
    /// mean anything, and it may not exceed
    /// <see cref="PowertrainState.Capacity"/> - a store nothing can hold is
    /// a store that silently never runs down.
    /// </summary>
    IReadOnlyList<PowertrainResourceInfo> Resources { get; }

    /// <summary>
    /// The output setting the strategist chooses. Named by the machinery -
    /// output modes on a pack, engine maps on a motor - and as long as the
    /// machinery says it is. Five is what the shipped car happens to have;
    /// nothing in the car, the panel or the strategy assumes it, so an engine
    /// with seven maps declares seven and everything counts to seven.
    /// </summary>
    ModeLadder OutputLadder { get; }

    /// <summary>
    /// The most driving acceleration this powertrain will allow right now.
    ///
    /// It is handed the chassis's own force ceiling rather than being asked
    /// only about power, because a powertrain that is running out holds back
    /// everything the car can do and not only the part of it that was power
    /// limited: a motor cannot make its rated torque on a sagging pack, and
    /// an engine starved of fuel does not make its rated torque either.
    /// </summary>
    float DriveAccelerationLimit(
        in PowertrainState energy,
        in CarStrategy strategy,
        float speed,
        float massKg,
        float chassisForceCeiling
    );

    /// <summary>
    /// Spends what a step of driving costs and takes back what a step of
    /// braking recovers, and reports the bill.
    /// </summary>
    PowertrainSettlement Settle(
        in PowertrainState energy,
        float driveAccel,
        float brakeAccel,
        float speed,
        float massKg,
        float dt
    );

    /// <summary>
    /// Power the brakes are putting back into the stores, for telemetry. Zero
    /// for machinery that cannot recover anything.
    /// </summary>
    float RecoveredPowerWatts(float brakeAccel, float speed, float massKg);

    /// <summary>
    /// What is left to burn, in kilograms. Zero for anything whose stores do
    /// not weigh anything - which is every electric car and no petrol one.
    /// </summary>
    float ConsumableMassKg(in PowertrainState energy);

    /// <summary>
    /// How much of its rated output the powertrain can actually deliver,
    /// zero to one. One when nothing is holding it back; less when a store is
    /// low enough to sag. This is a gauge reading, not a control: the limit
    /// itself is already applied inside
    /// <see cref="DriveAccelerationLimit"/>.
    /// </summary>
    float OutputAvailability(in PowertrainState energy);
}
