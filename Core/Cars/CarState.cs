using System;
using System.Numerics;
using StintegyEVO.Core.Util;

namespace StintegyEVO.Core.Cars;

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
    public float BatterySoc { get; set; } = 1f;

    public float FilteredLongitudinalAccel { get; set; }
    public float FilteredLateralAccel { get; set; }

    public TireState FrontLeft { get; } = new();
    public TireState FrontRight { get; } = new();
    public TireState RearLeft { get; } = new();
    public TireState RearRight { get; } = new();

    public CarTelemetry Telemetry { get; internal set; }

    public Vector2 Forward => new(MathF.Cos(Heading), MathF.Sin(Heading));
    public Vector2 Left => new(-MathF.Sin(Heading), MathF.Cos(Heading));
    public float VelocityHeading => MathHelper.NormalizeAngle(Heading + SideslipAngleRadians);
    public Vector2 VelocityForward => new(MathF.Cos(VelocityHeading), MathF.Sin(VelocityHeading));
    public Vector2 Velocity => VelocityForward * Speed;

    public void Normalize()
    {
        Speed = Math.Max(0f, Speed);
        BatterySoc = Math.Clamp(BatterySoc, 0f, 1f);
        Heading = MathHelper.NormalizeAngle(Heading);
        SideslipAngleRadians = MathHelper.NormalizeAngle(SideslipAngleRadians);
    }

    public TireState GetTire(WheelId wheel)
    {
        return wheel switch
        {
            WheelId.FrontLeft => FrontLeft,
            WheelId.FrontRight => FrontRight,
            WheelId.RearLeft => RearLeft,
            WheelId.RearRight => RearRight,
            _ => FrontLeft
        };
    }

    public void InstallFreshTires(TireConfig config)
    {
        FrontLeft.Reset(config);
        FrontRight.Reset(config);
        RearLeft.Reset(config);
        RearRight.Reset(config);
    }

    public CarState Clone()
    {
        CarState clone = new()
        {
            Position = Position,
            Heading = Heading,
            SideslipAngleRadians = SideslipAngleRadians,
            YawRateRadiansPerSecond = YawRateRadiansPerSecond,
            Speed = Speed,
            BatterySoc = BatterySoc,
            FilteredLongitudinalAccel = FilteredLongitudinalAccel,
            FilteredLateralAccel = FilteredLateralAccel,
            Telemetry = Telemetry
        };
        clone.FrontLeft.CopyFrom(FrontLeft);
        clone.FrontRight.CopyFrom(FrontRight);
        clone.RearLeft.CopyFrom(RearLeft);
        clone.RearRight.CopyFrom(RearRight);
        return clone;
    }

    public void CopyFrom(CarState other)
    {
        Position = other.Position;
        Heading = other.Heading;
        SideslipAngleRadians = other.SideslipAngleRadians;
        YawRateRadiansPerSecond = other.YawRateRadiansPerSecond;
        Speed = other.Speed;
        BatterySoc = other.BatterySoc;
        FilteredLongitudinalAccel = other.FilteredLongitudinalAccel;
        FilteredLateralAccel = other.FilteredLateralAccel;
        Telemetry = other.Telemetry;
        FrontLeft.CopyFrom(other.FrontLeft);
        FrontRight.CopyFrom(other.FrontRight);
        RearLeft.CopyFrom(other.RearLeft);
        RearRight.CopyFrom(other.RearRight);
    }
}
