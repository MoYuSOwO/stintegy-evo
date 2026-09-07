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
public readonly record struct DashboardTire(
    string Label,
    float LifeFraction,
    float SurfaceTempC,
    float CoreTempC,
    float LoadN
);

/// <summary>
/// One ladder and where it is set.
/// </summary>
public readonly record struct DashboardMode(
    string Label,
    string Rung,
    int Ordinal,
    int RungCount
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

    public void Refresh(CarConfig config, CarState state, CarStrategy strategy)
    {
        IPowertrain powertrain = config.Powertrain;
        PowertrainState energy = state.Energy;

        _resources.Clear();
        IReadOnlyList<PowertrainResourceInfo> declared = powertrain.Resources;
        for (int i = 0; i < declared.Count && i < PowertrainState.Capacity; i++)
        {
            PowertrainResourceInfo info = declared[i];
            float fraction = energy[i];
            _resources.Add(
                new DashboardResource(
                    info.Id,
                    info.Label,
                    fraction,
                    info.FullMassKg * fraction
                )
            );
        }

        _tires.Clear();
        AddTire("FL", state.FrontLeft);
        AddTire("FR", state.FrontRight);
        AddTire("RL", state.RearLeft);
        AddTire("RR", state.RearRight);

        _modes.Clear();
        AddMode(TireLadder.Usage, (int)strategy.TireMode);
        AddMode(powertrain.OutputLadder, strategy.PowerRung);

        OutputAvailability = powertrain.OutputAvailability(energy);
        MassKg = CarPhysics.TotalMassKg(config, energy);
    }

    private void AddTire(string label, TireState tire)
    {
        _tires.Add(
            new DashboardTire(
                label,
                1f - tire.Wear,
                tire.SurfaceTempC,
                tire.CoreTempC,
                tire.LoadN
            )
        );
    }

    private void AddMode(ModeLadder ladder, int ordinal)
    {
        // Clamped, so a strategy carried over from a car with a longer
        // ladder reads out as the setting it will actually get.
        ordinal = ladder.Clamp(ordinal);
        _modes.Add(
            new DashboardMode(
                ladder.Label,
                ladder.RungLabel(ordinal),
                ordinal,
                ladder.RungCount
            )
        );
    }
}
