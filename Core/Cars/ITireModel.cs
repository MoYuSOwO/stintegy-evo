using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// A tyre: how much it grips, and what being used does to it.
///
/// The chassis decides how hard each tyre is pressed and how it is being
/// used -- loads, slip, how much of its grip each direction is taking -- and
/// hands that over; the tyre answers with the force it can give and keeps
/// its own temperature and wear. A car can fit a different tyre on each axle.
/// </summary>
public interface ITireModel
{
    /// <summary>Where this tyre works and what it is called.</summary>
    TireWindow Window { get; }

    /// <summary>
    /// The most force the tyre can give at this load, as it is now: warm or
    /// cold, fresh or worn. <paramref name="sizedLoadN"/> is the load its own
    /// wheel carries with the car standing, which the tyre was chosen for.
    /// </summary>
    float GripForce(TireState tire, float loadN, float sizedLoadN);

    /// <summary>
    /// The force this tyre gives at its present load against what it gives at
    /// the load it was sized for: what its work, and so its heat and wear, is
    /// counted against.
    /// </summary>
    float ForceScale(TireState tire, float sizedLoadN);

    /// <summary>One step of being used: the tread and the carcass heat or cool, and the rubber wears.</summary>
    void Update(TireState tire, in TireWork work, float dt);

    /// <summary>How much of its grip the strategy's tyre rung lets the driver use.</summary>
    float GripUsage(CarStrategy strategy);

    /// <summary>A new tyre on this corner.</summary>
    void Reset(TireState tire);
}

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

/// <summary>The tyres a car runs, axle by axle.</summary>
public sealed record TireSet(ITireModel Front, ITireModel Rear)
{
    public ITireModel For(bool front) => front ? Front : Rear;

    public ITireModel For(WheelId wheel) =>
        wheel is WheelId.FrontLeft or WheelId.FrontRight ? Front : Rear;

    /// <summary>The same tyre all round.</summary>
    public static TireSet Same(ITireModel tire) => new(tire, tire);

    /// <summary>The game's own tyre, built to these numbers, all round.</summary>
    public static TireSet Of(TireConfig config) => Same(new StandardTire(config));

    public void Install(CarState state)
    {
        Front.Reset(state.FrontLeft);
        Front.Reset(state.FrontRight);
        Rear.Reset(state.RearLeft);
        Rear.Reset(state.RearRight);
    }
}
