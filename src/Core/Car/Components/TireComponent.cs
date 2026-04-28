using Godot;
using PloyRacing.Core.Car.Configs;
using System;

namespace PloyRacing.Core.Car.Components;

public struct BasePressureCondition
{
    public float temp;
    public float pressure;
}

public class TireComponent(TireConfig config, float initEnvTemp)
{
    private const int SubSteps = 50;

    public readonly TireConfig Config = config;
    private BasePressureCondition condition = new()
    {
        temp = initEnvTemp,
        pressure = Mathf.Clamp(
            config.InitPressure,
            TireConfig.AbsoluteMinPressure,
            TireConfig.AbsoluteMaxPressure
        )
    };
    private float _pressure = Mathf.Clamp(
        config.InitPressure,
        TireConfig.AbsoluteMinPressure,
        TireConfig.AbsoluteMaxPressure
    );

    // 实时状态
    public float SurfaceTemp { get; private set; } = initEnvTemp;
    public float CoreTemp { get; private set; } = initEnvTemp;
    public float Pressure 
    { 
        get => _pressure; 
        set
        {
            condition.temp = CoreTemp;
            condition.pressure = value;
            _pressure = Mathf.Clamp(
                value,
                TireConfig.AbsoluteMinPressure,
                TireConfig.AbsoluteMaxPressure
            );
        }
    }
    public float Wear { get; private set; } = 0.0f;
    public float WheelAngularVel { get; private set; } = 0f;

    public float GripFactor => GetTempGripFactor() * GetWearGripFactor() * GetPressureGripFactor();
    public float CurrLatPeakFriction => Config.LatPeakFriction * GripFactor;
    public float CurrLongPeakFriction => Config.LongPeakFriction * GripFactor;

    public TireOutput UpdateAndGetTire(
        float powerTorque, float fz, 
        Vector2 v, float dt, float envTemp
    )
    {
        float radius = Config.Radius;
        float carSpeedLong = v.X;

        if (fz <= 0.1f)
        {
            WheelAngularVel = carSpeedLong / radius;
            return new()
            { 
                Force = Vector2.Zero,
                IsSliding = false,
                IsLockedUp = false 
            };
        }

        // 1. 轮速子步积分
        float subDt = dt / SubSteps;
        float peakLongGrip = fz * CurrLongPeakFriction;
        for (int i = 0; i < SubSteps; i++)
        {
            float wheelSpeed = WheelAngularVel * radius;

            // 防震荡
            if (Mathf.Abs(carSpeedLong) < 0.5f && Mathf.Abs(powerTorque) < 5.0f)
            {
                WheelAngularVel = carSpeedLong / radius;
                break;
            }

            float slipRatio = CalculateSlipRatio(wheelSpeed, carSpeedLong);
            float actualLongForce = CalculatePacejkaLong(slipRatio, Config.LongStiffness, peakLongGrip);

            // 地面力对轮心的力矩（注意符号：若驱动，地面力向后推车，对轮子产生反向扭矩）
            float groundTorque = actualLongForce * radius;
            // 驱动扭矩
            float netTorque = powerTorque - groundTorque;
            float angularAccel = netTorque / Config.Inertia;
            WheelAngularVel += angularAccel * subDt;
        }

        // 2. 最终状态计算
        float finalWheelSpeed = WheelAngularVel * radius;
        float finalSlipRatio = CalculateSlipRatio(finalWheelSpeed, carSpeedLong);
        float finalPeakLongGrip = fz * CurrLongPeakFriction;
        float finalLongForce = CalculatePacejkaLong(finalSlipRatio, Config.LongStiffness, finalPeakLongGrip);

        // 3. 横向力（Pacejka）
        float peakLatGrip = fz * CurrLatPeakFriction;
        float slipAngle = Mathf.Atan2(v.Y, Mathf.Abs(carSpeedLong) + 0.01f);
        float latStiffness = Config.LatStiffness * GetPressureStiffnessFactor();
        float rawFy = CalculatePacejkaLat(slipAngle, latStiffness, peakLatGrip);

        // 4. 摩擦椭圆约束
        float longRatio = Mathf.Clamp(Mathf.Abs(finalLongForce) / peakLongGrip, 0f, 1f);
        float allowedLat = peakLatGrip * Mathf.Sqrt(1f - longRatio * longRatio);
        float actualFy = Mathf.Clamp(rawFy, -allowedLat, allowedLat);

        // 5. 发热与磨损
        float slipPowerLong = Mathf.Abs(finalLongForce) * Mathf.Abs(finalWheelSpeed - carSpeedLong);
        float slipPowerLat = Mathf.Abs(actualFy) * Mathf.Abs(v.Y);
        UpdateHeatAndWear(fz, v.Length(), slipPowerLong, slipPowerLat, envTemp, dt);

        // 6. 判定滑动状态
        bool isBraking = (finalLongForce * carSpeedLong) < -0.1f;
        bool isLockedUp = isBraking && (Mathf.Abs(finalWheelSpeed) < 0.2f);
        bool isSliding = Mathf.Abs(finalSlipRatio) > 0.05f || (Mathf.Abs(rawFy) > allowedLat + 0.1f);

        return new TireOutput
        {
            Force = new Vector2(finalLongForce, actualFy),
            IsSliding = isSliding,
            IsLockedUp = isLockedUp
        };
    }

    private static float CalculateSlipRatio(float wheelSpeed, float carSpeed)
    {
        const float eps = 0.01f;
        if (wheelSpeed > carSpeed) // 驱动打滑
        {
            float denom = Mathf.Max(Mathf.Abs(wheelSpeed), eps);
            return (wheelSpeed - carSpeed) / denom;
        }
        else if (wheelSpeed < carSpeed) // 制动打滑
        {
            float denom = Mathf.Max(Mathf.Abs(carSpeed), eps);
            return -(carSpeed - wheelSpeed) / denom;
        }
        return 0f;
    }

    private float CalculatePacejkaLong(float slipRatio, float stiffness, float peakGrip)
    {
        float x = slipRatio * stiffness;
        float sinTerm = Mathf.Sin(TireConfig.LongShape * Mathf.Atan(x - Config.LongDrop * (x - Mathf.Atan(x))));
        return peakGrip * sinTerm;
    }

    private float CalculatePacejkaLat(float alpha, float stiffness, float peakGrip)
    {
        float x = alpha * stiffness;
        float sinTerm = Mathf.Sin(TireConfig.LatShape * Mathf.Atan(x - Config.LatDrop * (x - Mathf.Atan(x))));
        return -peakGrip * sinTerm;
    }

    private void UpdateHeatAndWear(float fz, float speedMs, float longSlipPower, float latSlipPower, float envTemp, float dt)
    {
        // 摩擦能量注入表层
        float longEnergy = longSlipPower * dt;
        float latEnergy = latSlipPower * dt;
        float totalFrictionEnergy = longEnergy + latEnergy;
        float rollingEnergy = fz * speedMs * Config.DefaultRollingResCoef * dt;

        float surfaceMass = Config.Mass * TireConfig.SurfaceMassRatio;
        float coreMass = Config.Mass - surfaceMass;

        // 表层升温
        float tempRise = totalFrictionEnergy / (surfaceMass * TireConfig.SpecificHeat);
        SurfaceTemp += tempRise;

        // 内核滚动摩擦升温
        CoreTemp += rollingEnergy / (coreMass * TireConfig.SpecificHeat);

        // 内部热传导（表层->内核）
        float internalTransfer = (SurfaceTemp - CoreTemp) * TireConfig.InternalHeatTransCoef * dt;
        SurfaceTemp -= internalTransfer / (surfaceMass * TireConfig.SpecificHeat);
        CoreTemp += internalTransfer / (coreMass * TireConfig.SpecificHeat);

        // 风冷散热
        float coolingRate = TireConfig.BaseCoolingCoef + TireConfig.AirCoolingCoef * speedMs;
        float envTransfer = (SurfaceTemp - envTemp) * coolingRate * dt;
        SurfaceTemp -= envTransfer / (surfaceMass * TireConfig.SpecificHeat);

        // 磨损计算（基于摩擦功）
        float longWear = longEnergy * Config.BaseLongWearRate;
        float latWear = latEnergy * Config.BaseLatWearRate;
        // 可增加温度惩罚
        float thermalMult = 1.0f;
        if (SurfaceTemp > Config.OptimalMaxTemp)
            thermalMult += (SurfaceTemp - Config.OptimalMaxTemp) * Config.ThermalWearCoef;
        Wear = Mathf.Clamp(Wear + (longWear + latWear) * thermalMult, 0f, 1f);

        // 胎压更新（理想气体方程）
        Pressure = condition.pressure * (CoreTemp + 273.15f) / (condition.temp + 273.15f);
    }

    private float GetPressureStiffnessFactor()
    {
        float ratio = 1.0f;
        if (Pressure > Config.OptimalMaxPressure || Pressure < Config.OptimalMinPressure)
        {
            ratio = Pressure / ((Config.OptimalMinPressure + Config.OptimalMaxPressure) / 2.0f);
        }
        return 1.0f + (ratio - 1.0f) * TireConfig.StiffnessSensitivity;
    }

    private float GetTempGripFactor()
    {
        if (SurfaceTemp >= Config.OptimalMinTemp && SurfaceTemp <= Config.OptimalMaxTemp) return 1.0f;
        if (SurfaceTemp < Config.OptimalMinTemp)
        {
            float diff = Config.OptimalMinTemp - SurfaceTemp;
            return Mathf.Max(0.6f, 1.0f - (diff * TireConfig.ColdSensitivity));
        }
        else
        {
            float diff = SurfaceTemp - Config.OptimalMaxTemp;
            return Mathf.Max(0.3f, 1.0f - (diff * diff * TireConfig.HotSensitivity));
        }
    }

    private float GetWearGripFactor()
    {
        if (Wear <= 0.6f) return 1.0f - (Wear * (0.1f / 0.6f));
        if (Wear <= 0.8f) return 0.9f - (((Wear - 0.6f) / 0.2f) * 0.2f);
        if (Wear <= 0.85f) return 0.7f - (((Wear - 0.8f) / 0.05f) * 0.35f);
        
        float progress = (Wear - 0.85f) / 0.15f;
        return Mathf.Max(0.35f, 0.7f - (progress * progress * 0.35f));
    }

    private float GetPressureGripFactor()
    {
        if (Pressure >= Config.OptimalMinPressure && Pressure <= Config.OptimalMaxPressure) return 1.0f;
        if (Pressure < Config.OptimalMinPressure)
        {
            float diff = Config.OptimalMinPressure - Pressure;
            return Mathf.Max(0.8f, 1.0f - (diff * TireConfig.LowPressureSensitivity));
        }
        else
        {
            float diff = Pressure - Config.OptimalMaxPressure;
            return Mathf.Max(0.5f, 1.0f - (diff * diff * TireConfig.HighPressureSensitivity));
        }
    }
}