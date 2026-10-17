using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// How a tyre was used over one step, as the chassis measured it.
/// </summary>
/// <param name="SizedLoadN">What its wheel carries with the car standing.</param>
/// <param name="LateralUse">Share of its grip taken sideways, scrub included.</param>
/// <param name="LateralSlipWork">Share of its grip times how fast it is being dragged sideways.</param>
/// <param name="LongitudinalUse">Share of its grip taken driving or braking.</param>
/// <param name="BrakeUse">The braking part of that.</param>
/// <param name="CombinedDemandUse">How full the car's friction circles are, the busier axle's.</param>
/// <param name="LateralHeatScale">This axle's share of the car's lateral heat.</param>
/// <param name="LateralWearScale">This axle's share of the car's lateral wear.</param>
/// <param name="OverLimit">How far past its grip the axle is being driven, as costed.</param>
/// <param name="CoolingAirSpeed">The speed of the air reaching the tyre, after any car in front.</param>
/// <param name="WakeDownforceLoss">How much of the car's downforce a car in front is taking, as the tyres feel it.</param>
public readonly record struct TireWork(
    float SizedLoadN,
    float LateralUse,
    float LateralSlipWork,
    float LongitudinalUse,
    float BrakeUse,
    float CombinedDemandUse,
    float LateralHeatScale,
    float LateralWearScale,
    float OverLimit,
    float AirTempC,
    float TrackTempC,
    float Speed,
    float CoolingAirSpeed,
    float WakeDownforceLoss
);

/// <summary>The tyres a car has on, axle by axle.</summary>
public sealed record TireSet(Tire Front, Tire Rear)
{
    public Tire For(bool front) => front ? Front : Rear;

    public Tire For(WheelId wheel) =>
        wheel is WheelId.FrontLeft or WheelId.FrontRight ? Front : Rear;

    /// <summary>One compound all round, by a name nobody chose: for tests and probes.</summary>
    public static TireSet Of(TireConfig config) => TireCompound.Of(config).Set;

    public void Install(CarState state)
    {
        Front.Reset(state.FrontLeft);
        Front.Reset(state.FrontRight);
        Rear.Reset(state.RearLeft);
        Rear.Reset(state.RearRight);
    }
}

/// <summary>
/// A compound a car can be given: what its tyre pack calls it and the
/// numbers it puts on each axle (most compounds put the same on both). Its
/// id is its full name, the tyre pack's and its own:
/// <c>official:formula_a/medium</c>.
/// </summary>
public sealed class TireCompound
{
    /// <summary>A compound in no pack, the same on both axles: for tests and probes.</summary>
    public const string Unpacked = "unpacked";

    public TireCompound(string id, string name, TireConfig front, TireConfig rear)
    {
        Id = string.IsNullOrWhiteSpace(id) ? throw new ArgumentException("A compound needs an id.", nameof(id)) : id;
        Name = string.IsNullOrWhiteSpace(name) ? id : name;
        Front = front ?? throw new ArgumentNullException(nameof(front));
        Rear = rear ?? throw new ArgumentNullException(nameof(rear));
        var frontTire = new Tire(front, Id);
        Set = new TireSet(frontTire, ReferenceEquals(front, rear) ? frontTire : new Tire(rear, Id));
    }

    public string Id { get; }
    public string Name { get; }
    public TireConfig Front { get; }
    public TireConfig Rear { get; }

    /// <summary>The tyres this compound puts on a car.</summary>
    public TireSet Set { get; }

    public static TireCompound Of(TireConfig config, string id = Unpacked) =>
        new(id, id, config ?? throw new ArgumentNullException(nameof(config)), config);
}
