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
/// One car's running state. What every car has is here, and it is what the
/// world reads and writes: a rigid body's place and motion on the road --
/// where it is, which way it points, how fast and in which direction it
/// moves, how fast it turns -- which collisions with walls and other cars
/// change. Anything that belongs to the kind of car it is -- its wheels'
/// angle, its spins, the air it sits in, its tyres, stores, buttons -- is
/// the car's own (<see cref="Own"/>), read through the car's features
/// (<see cref="ICarMotion"/>, <see cref="ICarTires"/> and the rest).
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
        if (other.Own is null)
            Own = null;
        else if (Own is not null && Own.GetType() == other.Own.GetType())
            Own.CopyFrom(other.Own);
        else
            Own = other.Own.Clone();
    }
}
