using System;
using System.Collections.Generic;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// The physical resource category. Encoding it for a controller, network or
/// transport protocol is an adapter's responsibility, not part of the powertrain.
/// </summary>
public enum PowertrainResourceClass
{
    None = 0,
    Battery = 1,
    Fuel = 2,
}

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
    float FullMassKg,
    PowertrainResourceClass Class = PowertrainResourceClass.Battery
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
        : this(IdOf(label), label, rungs)
    {
    }

    /// <param name="id">Stable name, for a saved strategy or a telemetry column; never shown.</param>
    public ModeLadder(string id, string label, string[] rungs)
    {
        Id = string.IsNullOrWhiteSpace(id) ? throw new ArgumentException("A ladder needs an id.", nameof(id)) : id;
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

    public string Id { get; }
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

    private static string IdOf(string label) =>
        string.Join('_', label.ToLowerInvariant().Split(' ', StringSplitOptions.RemoveEmptyEntries));

    /// <summary>
    /// The nearest rung this ladder actually has. Called wherever a setting
    /// arrives from somewhere that did not know how long this ladder is.
    /// </summary>
    public int Clamp(int ordinal)
    {
        return Math.Clamp(ordinal, 1, RungCount);
    }

    /// <summary>
    /// Where a named rung sits on the ladder's travel: the travel runs from
    /// zero, the most conservative end, to one, the most aggressive, and the
    /// names are marks on it, evenly spaced. A setting may sit anywhere
    /// along it; the names are where the machinery's own settings are.
    /// </summary>
    public float Mark(int ordinal) => (Clamp(ordinal) - 1) / (float)(RungCount - 1);

    /// <summary>Where the ladder sits when nobody has chosen: the mark of <see cref="DefaultRung"/>.</summary>
    public float DefaultSetting => Mark(DefaultRung);

    /// <summary>A setting on the travel, as rungs up from the bottom one: fractional between two marks.</summary>
    public float RungsUp(float setting) => Settle(setting) * (RungCount - 1);

    /// <summary>The named rung nearest a setting, counted from one.</summary>
    public int Nearest(float setting) => Clamp((int)MathF.Round(RungsUp(setting)) + 1);

    /// <summary>
    /// A setting brought onto the travel: clamped to its ends, and the middle
    /// for one that is not a number at all.
    /// </summary>
    public static float Settle(float setting) => float.IsFinite(setting) ? Math.Clamp(setting, 0f, 1f) : 0.5f;
}
