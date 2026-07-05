using System;
using StintegyEVO.Core.Car;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;

public readonly record struct SpeedPlanningTireFriction(
    float LatFrontLeft,
    float LatFrontRight,
    float LatRearLeft,
    float LatRearRight,
    float LongFrontLeft,
    float LongFrontRight,
    float LongRearLeft,
    float LongRearRight
)
{
    public static SpeedPlanningTireFriction FromConfig(CarConfig carConfig)
    {
        return new SpeedPlanningTireFriction(
            carConfig.Tires[(int)TireType.FrontLeft].LatPeakFriction,
            carConfig.Tires[(int)TireType.FrontRight].LatPeakFriction,
            carConfig.Tires[(int)TireType.RearLeft].LatPeakFriction,
            carConfig.Tires[(int)TireType.RearRight].LatPeakFriction,
            carConfig.Tires[(int)TireType.FrontLeft].LongPeakFriction,
            carConfig.Tires[(int)TireType.FrontRight].LongPeakFriction,
            carConfig.Tires[(int)TireType.RearLeft].LongPeakFriction,
            carConfig.Tires[(int)TireType.RearRight].LongPeakFriction
        );
    }

    public static SpeedPlanningTireFriction FromCurrentFrame(CarLogic carLogic)
    {
        return new SpeedPlanningTireFriction(
            carLogic.TireFrontLeft.CurrLatPeakFriction,
            carLogic.TireFrontRight.CurrLatPeakFriction,
            carLogic.TireRearLeft.CurrLatPeakFriction,
            carLogic.TireRearRight.CurrLatPeakFriction,
            carLogic.TireFrontLeft.CurrLongPeakFriction,
            carLogic.TireFrontRight.CurrLongPeakFriction,
            carLogic.TireRearLeft.CurrLongPeakFriction,
            carLogic.TireRearRight.CurrLongPeakFriction
        );
    }

    internal float GetLatPeak(TireType type)
    {
        return type switch
        {
            TireType.FrontLeft => LatFrontLeft,
            TireType.FrontRight => LatFrontRight,
            TireType.RearLeft => LatRearLeft,
            TireType.RearRight => LatRearRight,
            _ => throw new ArgumentOutOfRangeException(nameof(type))
        };
    }

    internal float GetLongPeak(TireType type)
    {
        return type switch
        {
            TireType.FrontLeft => LongFrontLeft,
            TireType.FrontRight => LongFrontRight,
            TireType.RearLeft => LongRearLeft,
            TireType.RearRight => LongRearRight,
            _ => throw new ArgumentOutOfRangeException(nameof(type))
        };
    }
}

public readonly record struct SpeedPlanningState(
    float Mass,
    float BatterySoc,
    float TrackFrictionMultiplier,
    float DirtyAirFactor,
    SpeedPlanningTireFriction TireFriction
)
{
    public static SpeedPlanningState FromConfig(CarConfig carConfig)
    {
        return new SpeedPlanningState(
            Mass: carConfig.Chassis.DryMass,
            BatterySoc: 1.0f,
            TrackFrictionMultiplier: 1.0f,
            DirtyAirFactor: 0.0f,
            TireFriction: SpeedPlanningTireFriction.FromConfig(carConfig)
        );
    }

    public static SpeedPlanningState FromCurrentFrame(
        CarSensor carSensor,
        CarLogic carLogic,
        TrackData track,
        float dirtyAirFactor = 0.0f
    )
    {
        float mass = carSensor.Mass > 0.0f ? carSensor.Mass : carLogic.Config.Chassis.DryMass;
        return new SpeedPlanningState(
            Mass: mass,
            BatterySoc: Math.Clamp(carLogic.Battery.SocPct, 0.0f, 1.0f),
            TrackFrictionMultiplier: MathF.Max(0.0f, track.FrictionMultiplier),
            DirtyAirFactor: dirtyAirFactor,
            TireFriction: SpeedPlanningTireFriction.FromCurrentFrame(carLogic)
        );
    }
}
