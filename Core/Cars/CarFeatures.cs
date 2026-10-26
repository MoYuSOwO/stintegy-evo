using System;
using System.Collections.Generic;

namespace StintegyEVO.Core.Cars;

// What a car may have beyond what every car has. Every car is a rigid body
// on the road (CarState's motion, ICar's Step and Spec); anything more --
// tyres to change, stores to spend, a button to press, an envelope a driver
// can plan against -- is something a kind of car says it has by
// implementing the interface for it, and only then. Whoever wants one asks
// the car for it by type (car is ICarTires tires) and copes, or refuses the
// car, when it is not there.

/// <summary>A car with tyres that can be changed, heat and wear.</summary>
public interface ICarTires
{
    /// <summary>
    /// The compounds this car can be given, from the tyre packs its pack
    /// declares; the first is what it is given when nobody says otherwise.
    /// No car runs a compound it does not list.
    /// </summary>
    IReadOnlyList<TireCompound> Compounds { get; }

    /// <summary>
    /// A new set of <paramref name="compound"/> (its full id) on every
    /// corner, off the blankets. A compound the car does not list is refused.
    /// </summary>
    void FitTires(CarState state, string compound);

    /// <summary>A new set of what is on, or of the first compound if nothing is.</summary>
    void InstallFreshTires(CarState state);

    /// <summary>
    /// Every tyre brought to these temperatures, in place of the blankets':
    /// for a host or a test that wants a car on tyres already warm, or
    /// stone cold. Which tyres are on and how worn they are is unchanged.
    /// </summary>
    void WarmTires(CarState state, float surfaceTempC, float coreTempC);

    /// <summary>The compound on the car, by its full id; none before it has been given any.</summary>
    string? Compound(CarState state);

    /// <summary>The tyre on this corner as it is now, and where it works.</summary>
    TireReading Tire(CarState state, WheelId wheel);

    /// <summary>
    /// A planning copy of the car given these tyres: the compound and what
    /// each of the four read, front left, front right, rear left, rear
    /// right. For a driver who keeps its own copy of the car to ask the
    /// envelope about; the race's own car is never given tyres this way.
    /// </summary>
    void Assume(CarState plan, string? compound, ReadOnlySpan<TireReading> tires);
}

/// <summary>One tyre as it is now: its temperatures, wear, load and grip, and the window it works in.</summary>
public readonly record struct TireReading(float SurfaceTempC, float CoreTempC, float Wear, float LoadN, float SurfaceGrip)
{
    public TireWindow Window { get; init; }
}

/// <summary>A car that carries stores it spends as it goes: charge, fuel.</summary>
public interface ICarStores
{
    /// <summary>What it runs on and what the strategist spends: the stores, the output ladder and what can be delivered.</summary>
    IPowertrain Powertrain { get; }

    /// <summary>How much of each store is left, in the order <see cref="IPowertrain.Resources"/> declares them.</summary>
    PowertrainState Stores(CarState state);

    /// <summary>Every store set to these shares of full: for a host filling the car before it goes out.</summary>
    void Fill(CarState state, PowertrainState stores);
}

/// <summary>A button on a car, as the car names it.</summary>
/// <param name="Id">Stable name, for a saved strategy or a key binding.</param>
public readonly record struct CarButton(string Id, string Label);

/// <summary>
/// A button as it stands: whether pressing it now would do anything, whether
/// it is doing something, and the car's own words for it.
/// </summary>
/// <param name="SecondsLeft">How much use is left in it, in seconds; infinite for a button that never runs out.</param>
/// <param name="SecondsToWait">How long before it can be used again; zero when it can.</param>
/// <param name="Status">What the car would show on its wheel, short.</param>
public readonly record struct CarButtonReading(
    bool Ready, bool Running, bool Locked, float SecondsLeft, float SecondsToWait, string Status);

/// <summary>
/// A car with buttons a driver holds down: a push-to-pass, an overtake
/// mode. The strategy says which are held (<see cref="CarStrategy.Holding"/>);
/// whether holding one does anything is the car's to say.
/// </summary>
public interface ICarButtons
{
    /// <summary>The buttons, in the order a strategy holds them.</summary>
    IReadOnlyList<CarButton> Buttons { get; }

    CarButtonReading Read(CarState state, int button);

    /// <summary>
    /// Race control's hold, for the start and every restart: no button does
    /// anything until the car next crosses the line. Anything running when
    /// it is set ends there.
    /// </summary>
    void HoldUntilLine(CarState state);

    /// <summary>The car has crossed the line: race control's hold, if any, is lifted.</summary>
    void CrossedLine(CarState state);

    /// <summary>
    /// A planning copy of the car with this button doing what it was read
    /// doing: for a driver asking the envelope what the car can do while a
    /// push runs, or does not.
    /// </summary>
    void Assume(CarState plan, int button, bool running);
}

/// <summary>
/// A car that can say what it could do: the envelope a driver or an offline
/// bake plans against. Asking changes nothing.
/// </summary>
public interface ICarEnvelope
{
    /// <summary>
    /// What this car can do right now at a speed and round a curvature.
    /// </summary>
    /// <param name="gripUsage">The share of the friction circle to plan against, usually <see cref="GripUsage"/>.</param>
    CarPerformanceLimits EstimatePerformanceLimits(
        CarState state,
        CarStrategy strategy,
        float speed,
        float curvature,
        float gripUsage = 1f,
        float assumedLongitudinalAcceleration = 0f,
        float frontBrakeBiasOffset = 0f
    );

    /// <summary>The steady corner the car can hold at this speed, read with that corner's own load on the outside wheels.</summary>
    float EstimateCorneringLimit(CarState state, CarStrategy strategy, float speed, float gripUsage = 1f);

    /// <summary>
    /// The road-wheel angle that would turn this car onto a curvature, as it
    /// is now, on this road: the steering a driver who knew the car exactly
    /// would do. The car obeys the angle it is given, not this.
    /// </summary>
    SteeringEstimate EstimateSteering(CarState state, RoadAttitude road);

    /// <summary>How much of the tyres' grip the strategy's tyre rung lets the driver use.</summary>
    float GripUsage(CarStrategy strategy);

    /// <summary>The limits a controller is told about, as they stand.</summary>
    CarCapabilities Capabilities(CarState state, CarStrategy strategy);

    /// <summary>How the car carries its weight and shares its drive and its brakes between the axles.</summary>
    AxleLayout Layout { get; }
}

/// <summary>
/// How a car is laid out between its axles, as its driver knows it: where
/// the weight sits, where the drive goes and how the brakes are shared.
/// </summary>
/// <param name="FrontWeightShare">The share of the car's weight on the front axle, standing.</param>
/// <param name="FrontDriveShare">The share of the drive the front axle is given.</param>
/// <param name="DriveFollowsGrip">
/// Whether the car moves its drive between the axles by the grip each has
/// left, each held to its own motor's share; otherwise the drive is split
/// as it is and runs out where the first axle does.
/// </param>
/// <param name="FixedFrontBrakeShare">
/// The share of the braking at the front, where the brakes are fixed to a
/// balance the driver dials (<see cref="DriverInput.FrontBrakeBiasOffset"/>
/// moves it); none where the car shares its brakes by grip itself.
/// </param>
public readonly record struct AxleLayout(
    float FrontWeightShare, float FrontDriveShare, bool DriveFollowsGrip, float? FixedFrontBrakeShare);

/// <summary>
/// Each axle's grip, as a driver feels it through the seat and the wheel:
/// how much there is, and how much of it is being used, zero none and one
/// all the axle has.
/// </summary>
/// <param name="FrontGripAccel">What the front axle could give, as an acceleration of the whole car.</param>
/// <param name="RearGripAccel">What the rear axle could give, the same way.</param>
/// <param name="RearSlideSeverity">How far the rear is past its peak and sliding, zero when it is not.</param>
public readonly record struct GripUse(
    float FrontGripAccel, float RearGripAccel,
    float FrontLateral, float RearLateral, float FrontLongitudinal, float RearLongitudinal, float RearSlideSeverity);

/// <summary>
/// A car whose grip a driver can feel being used, and whose tyres say where
/// their peak is: what a driver needs to drive at the limit and not past it.
/// </summary>
public interface ICarGripFeel
{
    /// <summary>How much of each axle's grip was used in the last step.</summary>
    GripUse GripUse(CarState state);

    /// <summary>
    /// The share of its peak grip a tyre holds in a steady corner: what a
    /// driver can lean on without the tyre letting go.
    /// </summary>
    float SustainablePeakShare { get; }

    /// <summary>The slip angle at which the tyres' grip peaks; past it they are sliding.</summary>
    float PeakSlipAngleRadians { get; }

    /// <summary>The slip angle at which a tyre gives <paramref name="share"/> of its peak grip, on the way up to it.</summary>
    float SlipAngleAtShare(float share);

    /// <summary>
    /// How much the tyres on the car lose of their grip per unit of load as
    /// the load grows: zero is grip in proportion to load, one is none gained.
    /// </summary>
    float LoadSensitivity(CarState state);
}

/// <summary>
/// What a driver needs of a car before it will drive it, and what it will
/// use if the car has it. Each is a car feature, named by its interface
/// (<see cref="ICarTires"/>, <see cref="ICarEnvelope"/> and the rest).
/// </summary>
public sealed record CarNeeds
{
    public static readonly CarNeeds None = new([], []);

    public CarNeeds(IReadOnlyList<Type> required, IReadOnlyList<Type> optional)
    {
        Required = Checked(required, nameof(required));
        Optional = Checked(optional, nameof(optional));
    }

    /// <summary>Without these the driver cannot drive the car, and will not be put in it.</summary>
    public IReadOnlyList<Type> Required { get; }

    /// <summary>These the driver uses when the car has them, and drives without when it has not.</summary>
    public IReadOnlyList<Type> Optional { get; }

    /// <summary>What of <see cref="Required"/> this car does not have; empty when it has them all.</summary>
    public IReadOnlyList<Type> MissingFrom(ICar car)
    {
        ArgumentNullException.ThrowIfNull(car);
        var missing = new List<Type>();
        foreach (Type need in Required)
            if (!need.IsInstanceOfType(car))
                missing.Add(need);
        return missing;
    }

    private static IReadOnlyList<Type> Checked(IReadOnlyList<Type> features, string name)
    {
        ArgumentNullException.ThrowIfNull(features, name);
        foreach (Type feature in features)
            if (feature is null || !feature.IsInterface)
                throw new ArgumentException("A need is a car feature's interface.", name);
        return features;
    }
}
