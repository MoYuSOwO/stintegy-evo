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

    public float GripFactor => GetTempGripFactor() * GetWearGripFactor() * GetPressureGripFactor();
    public float CurrLatPeakFriction => Config.LatPeakFriction * GripFactor;
    public float CurrLongPeakFriction => Config.LongPeakFriction * GripFactor;

    public TireOutput UpdateAndGetTire(
        float demandFx, float fz, 
        Vector2 v, float dt, float envTemp
    )
    {
        if (fz <= 0.1f)
        {
            return new()
            { 
                Force = Vector2.Zero,
                IsSliding = false,
                IsLockedUp = false 
            };
        }

        // 1. 结算当前环境系数带来的抓地力惩罚
        float currLatMaxGrip = fz * CurrLatPeakFriction;
        float currLongMaxGrip = fz * CurrLongPeakFriction;

        // 2. 纵向拟合 (无滑移率版本的 Peak & Sliding 模型)
        float frictionFx = CalculateLongitudinalForce(demandFx, currLongMaxGrip);
        float actualFx = frictionFx;
        float pressureRatio = Mathf.Clamp(Config.OptimalMinPressure / Mathf.Max(Pressure, 0.1f), 0.8f, 3.0f);
        float rollingResCoef = Config.DefaultRollingResCoef * pressureRatio;
        if (Mathf.Abs(v.X) > 0.1f)
        {
            float rollingDragForce = Mathf.Sign(v.X) * fz * rollingResCoef;
            actualFx -= rollingDragForce; 
        }

        // 3. 横向 Pacejka
        float absVLong = Mathf.Abs(v.X);
        float slipAngle = Mathf.Atan2(v.Y, absVLong + 0.01f);

        float latStiff = Config.LatStiffness * GetPressureStiffnessFactor();
        float rawFy = CalculatePacejkaLat(slipAngle, latStiff, currLatMaxGrip);

        // 4. 摩擦椭圆：纵向吃肉，横向喝汤
        float xRatio = Mathf.Clamp(Mathf.Abs(frictionFx) / currLongMaxGrip, 0f, 1f);
        float availableFy = currLatMaxGrip * Mathf.Sqrt(1.0f - (xRatio * xRatio));
        float actualFy = Mathf.Clamp(rawFy, -availableFy, availableFy);

        float wastedForce = Mathf.Max(0f, Mathf.Abs(demandFx) - Mathf.Abs(frictionFx));

        bool isBraking = (demandFx * v.X) < -0.1f; 
        bool isLockedUp = isBraking && (wastedForce > 0f);
        
        bool isSliding = (wastedForce > 0f) || (Mathf.Abs(rawFy) > availableFy);

        UpdateThermodynamics(fz, v, actualFy, wastedForce, isLockedUp, envTemp, dt);

        return new TireOutput
        {
            Force = new Vector2(actualFx, actualFy),
            IsSliding = isSliding,
            IsLockedUp = isLockedUp
        };
    }

    private float CalculateLongitudinalForce(float demand, float maxGrip)
    {
        float absDemand = Mathf.Abs(demand);
        float u = absDemand / Mathf.Max(maxGrip, 1.0f);

        if (u <= 1.0f) return demand; // 未突破极限

        float limitCurve;
        if (u <= TireConfig.LongOptimalSlipRatio)
        {
            // 弹性拉伸区 (1.0 -> 1.15)：爆发峰值抓地力
            float t = (u - 1.0f) / (TireConfig.LongOptimalSlipRatio - 1.0f);
            limitCurve = 1.0f + (Config.LongPeakFriction - 1.0f) * Mathf.Sin(t * Mathf.Pi / 2.0f);
        }
        else
        {
            // 彻底滑动区：抓地力指数级跌落
            float over = u - TireConfig.LongOptimalSlipRatio;
            float decay = Mathf.Exp(-2.0f * over);
            limitCurve = TireConfig.LongSlideMultiplier + (Config.LongPeakFriction - TireConfig.LongSlideMultiplier) * decay;
        }

        return Mathf.Sign(demand) * (maxGrip * limitCurve);
    }

    private float CalculatePacejkaLat(float alpha, float stiffness, float peakGrip)
    {
        float x = alpha * stiffness;
        return -peakGrip * Mathf.Sin(TireConfig.LatShape * Mathf.Atan(x - Config.LatDrop * (x - Mathf.Atan(x))));
    }

    private void UpdateThermodynamics(float fz, Vector2 v, float actualFy, float wastedForce, bool isLockedUp, float envTemp, float dt)
    {
        float speedMs = v.Length();

        // 更新胎压和滚阻影响
        float pressureRatio = Mathf.Clamp(Config.OptimalMinPressure / Math.Max(Pressure, 0.1f), 0.8f, 3.0f);
        float RollingResCoef = Config.DefaultRollingResCoef * pressureRatio;

        // 极其真实的热力学辨别：
        float deltaVSlip = 0f;
        if (wastedForce > 0f)
        {
            if (isLockedUp)
            {
                // 刹车抱死：轮胎停转，以车身全速在地上死拖！
                deltaVSlip = Mathf.Abs(v.X); 
            }
            else
            {
                // 引擎空转：车在走，轮子转得更快。滑移速度与多出的力成正比。
                deltaVSlip = Mathf.Min(speedMs, wastedForce * TireConfig.WastedForceToSlipVCoef + (Mathf.Abs(v.X) < 0.5f ? TireConfig.StartingBurnoutBaseSlipV : 0f));
            }
        }

        float frictionPowerLong = wastedForce * deltaVSlip; // 纵向滑移生热
        float frictionPowerLat = Mathf.Abs(actualFy) * Mathf.Abs(v.Y); // 横向侧滑生热

        // 能量守恒
        float frictionEnergy = (frictionPowerLong + frictionPowerLat) * dt;
        float rollingEnergy = fz * speedMs * RollingResCoef * dt;

        float surfaceMass = Config.Mass * TireConfig.SurfaceMassRatio;
        float coreMass = Config.Mass - surfaceMass;

        // 表层温度：摩擦能量注入
        float tempRise = frictionEnergy / (surfaceMass * TireConfig.SpecificHeat);
        
        // 极端打滑时的 Ablation (消融) 处理：超过最大升温限制的能量，直接转化为剥落轮胎的物理损耗，不再升温
        float ablationEnergy = 0f;
        float maxTempRisePerStep = TireConfig.MaxTempRiseRate * dt;
         // 参照你旧代码
        if (tempRise > maxTempRisePerStep)
        {
            ablationEnergy = (tempRise - maxTempRisePerStep) * (surfaceMass * TireConfig.SpecificHeat);
            tempRise = maxTempRisePerStep;
        }

        SurfaceTemp += tempRise;
        CoreTemp += rollingEnergy / (coreMass * TireConfig.SpecificHeat);

        // 内部热传导 (表层传给内核)
        float internalTransfer = (SurfaceTemp - CoreTemp) * TireConfig.InternalHeatTransCoef * dt;
        SurfaceTemp -= internalTransfer / (surfaceMass * TireConfig.SpecificHeat);
        CoreTemp += internalTransfer / (coreMass * TireConfig.SpecificHeat);

        // 外部环境散热 (撞风)
        float coolingRate = TireConfig.BaseCoolingCoef + TireConfig.AirCoolingCoef * speedMs;
        float envTransfer = (SurfaceTemp - envTemp) * coolingRate * dt;
        SurfaceTemp -= envTransfer / (surfaceMass * TireConfig.SpecificHeat);

        // 磨损计算 (常规摩擦 + 极端烧胎的消融)
        float thermalMult = 1.0f;
        if (SurfaceTemp > Config.OptimalMaxTemp) 
            thermalMult += (SurfaceTemp - Config.OptimalMaxTemp) * TireConfig.ThermalCoef;

        float wearFromFriction = frictionEnergy * Config.WearCoef * thermalMult;
        float wearFromAblation = ablationEnergy * Config.WearCoef * TireConfig.AblationCoef * thermalMult;
        Wear = Mathf.Clamp(Wear + wearFromFriction + wearFromAblation, 0f, 1f);

        // 理想气体方程更新胎压
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