using System.Collections.Generic;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// One store, and how much of it is left.
/// </summary>
public readonly record struct DashboardResource(
    string Id,
    string Label,
    float Fraction,
    float RemainingMassKg
);

/// <summary>
/// One corner, and what state it is in. Tyres get their own shape rather than
/// being squeezed into a fraction, because a tyre that is cold is not the
/// same problem as a tyre that is worn and the strategist has to tell them
/// apart.
/// </summary>
/// <param name="Window">Where this tyre works, so a screen can say whether
/// a temperature is good without knowing anything about the rubber.</param>
public readonly record struct DashboardTire(
    string Label,
    float LifeFraction,
    float SurfaceTempC,
    float CoreTempC,
    float LoadN,
    TireWindow Window
);

/// <summary>
/// The temperatures a tyre is happy between, and the core temperature past
/// which it is cooking. Every car's tyre has one; what the numbers are is
/// the tyre's to say.
/// </summary>
public readonly record struct TireWindow(
    string Compound,
    float IdealSurfaceLowC,
    float IdealSurfaceHighC,
    float CoreOverheatC
);

/// <summary>
/// One ladder and where it is set: zero to one along its travel, and the
/// name of the mark nearest it.
/// </summary>
public readonly record struct DashboardMode(
    string Label,
    string Nearest,
    float Setting
);

/// <summary>
/// What the strategy table shows: the stores that are running down, the
/// ladders they are being spent on, and whether the car can still deliver
/// what the top of a ladder promises.
///
/// Every word on it comes from the car. Nothing here knows that the default
/// car is electric, so a petrol car fitted tomorrow reads out its tank and
/// its engine maps through this same instrument without a line of it
/// changing - which is the entire point of the powertrain being an interface
/// rather than a battery with the serial numbers filed off.
///
/// The lists are kept and refilled rather than rebuilt, because this is read
/// once a frame while a race is running.
/// </summary>
public sealed class CarDashboard
{
    private readonly List<DashboardResource> _resources = new();
    private readonly List<DashboardTire> _tires = new();
    private readonly List<DashboardMode> _modes = new();

    public IReadOnlyList<DashboardResource> Resources => _resources;
    public IReadOnlyList<DashboardTire> Tires => _tires;
    public IReadOnlyList<DashboardMode> Modes => _modes;

    /// <summary>
    /// How much of its rated output the powertrain can actually give right
    /// now, zero to one. Below one means something is holding the car back -
    /// a pack too low to hold its voltage, a tank too low to feed the pump -
    /// and it is worth showing, because a driver who cannot see it thinks
    /// the car has gone wrong.
    /// </summary>
    public float OutputAvailability { get; private set; } = 1f;

    /// <summary>
    /// What the car weighs at this moment, stores included.
    /// </summary>
    public float MassKg { get; private set; }

    /// <summary>Asks the car to write itself out.</summary>
    public void Refresh(ICar car, CarState state, CarStrategy strategy)
    {
        Clear();
        car.Describe(this, state, strategy);
    }

    /// <summary>Empties the instrument before a car writes into it.</summary>
    public void Clear()
    {
        _resources.Clear();
        _tires.Clear();
        _modes.Clear();
        OutputAvailability = 1f;
        MassKg = 0f;
    }

    public void AddResource(DashboardResource resource) => _resources.Add(resource);

    public void AddTire(DashboardTire tire) => _tires.Add(tire);

    /// <summary>
    /// One ladder and where it is set, brought onto its travel so a
    /// strategy from anywhere reads out as the setting the car will get.
    /// </summary>
    public void AddMode(ModeLadder ladder, float setting)
    {
        setting = ModeLadder.Settle(setting);
        _modes.Add(new DashboardMode(ladder.Label, ladder.RungLabel(ladder.Nearest(setting)), setting));
    }

    public void SetOutputAvailability(float availability) => OutputAvailability = availability;

    public void SetMass(float massKg) => MassKg = massKg;
}
