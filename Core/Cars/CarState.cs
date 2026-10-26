using System;
using System.Numerics;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// The part of a car's state that belongs to the kind of car it is, and
/// that nothing outside that car reads. The race copies it with the rest of
/// the state -- a step tried and undone, a frame held back -- without
/// knowing what is in it.
/// </summary>
public interface ICarStateExtension
{
    ICarStateExtension Clone();

    /// <summary>Takes on <paramref name="other"/>'s values; it is always the same kind.</summary>
    void CopyFrom(ICarStateExtension other);
}

/// <summary>
/// One car's running state. What every car has is here: a rigid body's
/// motion on the road, the referee's count of its spins, the air it sits in
/// and what it last reported. Anything that belongs to the kind of car it is
/// -- tyres, stores, buttons -- is the car's own (<see cref="Own"/>), read
/// through the car's features (<see cref="ICarTires"/> and the rest).
/// </summary>
public sealed class CarState
{
    public Vector2 Position { get; set; }
    public float Heading { get; set; }
    /// <summary>
    /// Signed velocity-direction offset relative to the body heading.
    /// Negative values represent the tail stepping outward in a left turn.
    /// </summary>
    public float SideslipAngleRadians { get; set; }
    public float YawRateRadiansPerSecond { get; set; }
    public float Speed { get; set; }

    /// <summary>
    /// Where the front wheels are actually pointed, relative to the body.
    /// A state rather than a number worked out from the driver's request,
    /// because the wheels take time to get there and the time is the point:
    /// a slide is caught by lock that arrives soon enough, and not caught by
    /// the same lock arriving late.
    /// </summary>
    public float SteerAngleRadians { get; set; }

    /// <summary>
    /// How long the car has been sideways past the angle the referee
    /// watches. Reset the moment it comes back under, so a flick that is
    /// caught costs nothing.
    /// </summary>
    public float SideslipHoldSeconds { get; set; }

    /// <summary>
    /// True while the car is spinning: the driver's hands are off it and
    /// the physics is playing a scripted rotation until it hands back.
    /// </summary>
    public bool Spinning { get; set; }

    /// <summary>
    /// How long the current spin has been running. Zero when not spinning.
    /// </summary>
    public float SpinSeconds { get; set; }

    /// <summary>
    /// How many times this car has been declared lost. A scoreline: a
    /// graduating driver has none of these in a session.
    /// </summary>
    public int SpinEvents { get; set; }

    public float FilteredLongitudinalAccel { get; set; }
    public float FilteredLateralAccel { get; set; }

    /// <summary>
    /// How much of its own speed the air around this car is already carrying,
    /// because a car in front has dragged it along.
    ///
    /// This lives on the car rather than being worked out where it is used
    /// because two very different things read it. The physics needs it to know
    /// what the car is doing now; the speed planner needs it to know what the
    /// car could do next, and the planner is handed a state, not a grid. Put
    /// here, a car that has caught a tow plans on having it, which is the whole
    /// point: the run to the corner is quicker, so the road in front becomes
    /// somewhere a pass can be made, without anything in the racecraft being
    /// told that a tow exists.
    ///
    /// Set once a frame from one frozen picture of where every car is, so it
    /// does not depend on the order the cars are stepped in.
    /// </summary>
    public float AirVelocityDeficit { get; set; }

    /// <summary>
    /// Additional fraction of aerodynamic downforce made unavailable by the
    /// turbulent and rotating part of a leading car's wake. The loss caused
    /// directly by reduced air speed is not included here.
    /// </summary>
    public float WakeDownforceLoss { get; set; }

    /// <summary>
    /// Share of oncoming-air speed missing at this car's working aero
    /// surfaces. The body's own deficit is <see cref="AirVelocityDeficit"/>;
    /// this one decays much faster with distance, because the wake rises off
    /// the road while wings and floor stay close to it.
    /// </summary>
    public float DownforceVelocityDeficit { get; set; }

    public CarTelemetry Telemetry { get; internal set; }

    public Vector2 Forward => new(MathF.Cos(Heading), MathF.Sin(Heading));
    public Vector2 Left => new(-MathF.Sin(Heading), MathF.Cos(Heading));
    public float VelocityHeading => MathHelper.NormalizeAngle(Heading + SideslipAngleRadians);
    public Vector2 VelocityForward => new(MathF.Cos(VelocityHeading), MathF.Sin(VelocityHeading));
    public Vector2 Velocity => VelocityForward * Speed;

    public void Normalize()
    {
        Speed = Math.Max(0f, Speed);
        Heading = MathHelper.NormalizeAngle(Heading);
        SideslipAngleRadians = MathHelper.NormalizeAngle(SideslipAngleRadians);
        AirVelocityDeficit = Math.Clamp(AirVelocityDeficit, 0f, 1f);
        WakeDownforceLoss = Math.Clamp(WakeDownforceLoss, 0f, 1f);
        DownforceVelocityDeficit = Math.Clamp(DownforceVelocityDeficit, 0f, 1f);
    }

    /// <summary>The car's own state, which only that car reads (<see cref="ICarStateExtension"/>).</summary>
    public ICarStateExtension? Own { get; set; }

    public CarState Clone()
    {
        CarState clone = new()
        {
            Position = Position,
            Heading = Heading,
            SideslipAngleRadians = SideslipAngleRadians,
            YawRateRadiansPerSecond = YawRateRadiansPerSecond,
            Speed = Speed,
            SteerAngleRadians = SteerAngleRadians,
            SideslipHoldSeconds = SideslipHoldSeconds,
            Spinning = Spinning,
            SpinSeconds = SpinSeconds,
            SpinEvents = SpinEvents,
            FilteredLongitudinalAccel = FilteredLongitudinalAccel,
            FilteredLateralAccel = FilteredLateralAccel,
            AirVelocityDeficit = AirVelocityDeficit,
            WakeDownforceLoss = WakeDownforceLoss,
            DownforceVelocityDeficit = DownforceVelocityDeficit,
            Telemetry = Telemetry,
            Own = Own?.Clone()
        };
        return clone;
    }

    public void CopyFrom(CarState other)
    {
        Position = other.Position;
        Heading = other.Heading;
        SideslipAngleRadians = other.SideslipAngleRadians;
        YawRateRadiansPerSecond = other.YawRateRadiansPerSecond;
        Speed = other.Speed;
        SteerAngleRadians = other.SteerAngleRadians;
        SideslipHoldSeconds = other.SideslipHoldSeconds;
        Spinning = other.Spinning;
        SpinSeconds = other.SpinSeconds;
        SpinEvents = other.SpinEvents;
        FilteredLongitudinalAccel = other.FilteredLongitudinalAccel;
        FilteredLateralAccel = other.FilteredLateralAccel;
        AirVelocityDeficit = other.AirVelocityDeficit;
        WakeDownforceLoss = other.WakeDownforceLoss;
        DownforceVelocityDeficit = other.DownforceVelocityDeficit;
        Telemetry = other.Telemetry;
        if (other.Own is null)
            Own = null;
        else if (Own is not null && Own.GetType() == other.Own.GetType())
            Own.CopyFrom(other.Own);
        else
            Own = other.Own.Clone();
    }
}
