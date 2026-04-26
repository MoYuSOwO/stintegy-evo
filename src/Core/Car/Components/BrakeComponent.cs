using Godot;
using PloyRacing.Core.Car.Configs;
using System;

namespace PloyRacing.Core.Car.Components;

public class BrakeComponent(BrakeConfig config, float envTemp, float initBiasFront)
{
    public readonly BrakeConfig Config = config;

    private CarBrakeTemp temp = new()
    {
        FrontLeft = envTemp,
        FrontRight = envTemp,
        RearLeft = envTemp,
        RearRight = envTemp
    };
    public CarBrakeTemp Temp => temp;
    private float biasFront = Mathf.Clamp(initBiasFront, BrakeConfig.MinBiasFront, BrakeConfig.MinBiasFront);
    public float BiasFront
    {
        get => biasFront;
        set => biasFront = Mathf.Clamp(value, BrakeConfig.MinBiasFront, BrakeConfig.MinBiasFront);
    }

    public CarBrakeForce GetBrakeDemand(float brakePedal)
    {
        CarBrakeForce demand = new();
        if (brakePedal <= 0.01f) return demand;

        float totalRequestForce = brakePedal * Config.MaxBrakeForce;
        float baseFront = (totalRequestForce * biasFront) / 2f;
        float baseRear  = (totalRequestForce * (1.0f - biasFront)) / 2f;

        demand.FrontLeft = baseFront * CalculateEfficiency(temp.FrontLeft, envTemp);
        demand.FrontRight = baseFront * CalculateEfficiency(temp.FrontRight, envTemp);
        demand.RearLeft = baseRear  * CalculateEfficiency(temp.RearLeft, envTemp);
        demand.RearRight = baseRear  * CalculateEfficiency(temp.RearRight, envTemp);

        return demand;
    }

    public void UpdateThermodynamics(
        CarBrakeForce actualAppliedForce, float envTemp,
        bool lockFL, bool lockFR, bool lockRL, bool lockRR,
        float speedMs, float dt
    )
    {
        // 动态撞风散热：只要车在跑，就在散热，无论踩没踩刹车
        float dynamicCooling = Config.CoolingRateBase + (speedMs * Config.CoolingRateAir);

        // 如果车轮抱死 (isLocked == true)，刹车盘停止摩擦，发热功率瞬间为 0！
        float heatFL = lockFL ? 0f : (actualAppliedForce.FrontLeft * speedMs);
        float heatFR = lockFR ? 0f : (actualAppliedForce.FrontRight * speedMs);
        float heatRL = lockRL ? 0f : (actualAppliedForce.RearLeft * speedMs);
        float heatRR = lockRR ? 0f : (actualAppliedForce.RearRight * speedMs);

        temp.FrontLeft = UpdateDiscTemp(temp.FrontLeft, envTemp, heatFL, dynamicCooling, dt);
        temp.FrontRight = UpdateDiscTemp(temp.FrontRight, envTemp, heatFR, dynamicCooling, dt);
        temp.RearLeft = UpdateDiscTemp(temp.RearLeft, envTemp, heatRL, dynamicCooling, dt);
        temp.RearRight = UpdateDiscTemp(temp.RearRight, envTemp, heatRR, dynamicCooling, dt);
    }

    private float UpdateDiscTemp(float currentTemp, float envTemp, float heatPower, float coolingRate, float dt)
    {
        float tempRise = (heatPower / Config.SpecificHeat) * dt;
        float heatLoss = (currentTemp - envTemp) * coolingRate * dt;
        return Mathf.Min(Math.Max(envTemp, currentTemp + tempRise - heatLoss), BrakeConfig.AbsoluteMaxTemp + 0.5f);
    }

    public float CalculateEfficiency(float tireTemp, float envTemp)
    {
        if (temp.IsFailure) return BrakeConfig.FatalFailureEfficiency;
        else if (tireTemp >= Config.OptimalMinTemp && tireTemp <= Config.OptimalMaxTemp) return 1.0f;
        else if (tireTemp < Config.OptimalMinTemp)
        {
            float progress = (tireTemp - envTemp) / Math.Max(Config.OptimalMinTemp - envTemp, 1f);
            return Config.ColdEfficiency + progress * (1.0f - Config.ColdEfficiency);
        }
        else
        {
            float excess = tireTemp - Config.OptimalMaxTemp;
            float drop = excess * 0.002f; 
            return Math.Max(Config.OverheatEfficiency, 1.0f - drop);
        } 
    }
}