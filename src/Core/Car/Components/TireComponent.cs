using Godot;
using StintegyEVO.Core.Car.Configs;

namespace StintegyEVO.Core.Car.Components;

public struct BasePressureCondition
{
    public float Temp;
    public float Pressure;
}

public class TireComponent(TireConfig config, float initEnvTemp)
{
    private const int SubSteps = 50;

    public readonly TireConfig Config = config;
    private BasePressureCondition condition = new()
    {
        Temp = initEnvTemp,
        Pressure = Mathf.Clamp(
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

    public float SurfaceTemp { get; private set; } = initEnvTemp;
    public float CoreTemp { get; private set; } = initEnvTemp;
    public float Pressure 
    { 
        get => _pressure; 
        set
        {
            condition.Temp = CoreTemp;
            condition.Pressure = value;
            _pressure = Mathf.Clamp(
                value,
                TireConfig.AbsoluteMinPressure,
                TireConfig.AbsoluteMaxPressure
            );
        }
    }
    public float Wear { get; private set; } = 0.0f;
    public float WheelAngularVel { get; private set; } = 0f;
    private float _longForceState;
    private float _latForceState;

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
            _longForceState = 0f;
            _latForceState = 0f;
            return new()
            { 
                Force = Vector2.Zero,
                IsSliding = false,
                IsLockedUp = false 
            };
        }

        // 1. Wheel speed substep integration
        float subDt = dt / SubSteps;
        float peakLongGrip = fz * CurrLongPeakFriction;
        for (int i = 0; i < SubSteps; i++)
        {
            float appliedTorque = GetAppliedWheelTorque(powerTorque, carSpeedLong, WheelAngularVel);
            float wheelSpeed = WheelAngularVel * radius;
            float slipRatio = CalculateSlipRatio(wheelSpeed, carSpeedLong);
            float targetLongForce = CalculateBlendedLongForce(appliedTorque, radius, slipRatio, carSpeedLong, peakLongGrip);
            float actualLongForce = RelaxForce(
                _longForceState,
                targetLongForce,
                TireConfig.LongRelaxationLength,
                Mathf.Max(Mathf.Abs(wheelSpeed), Mathf.Abs(carSpeedLong)),
                subDt
            );
            actualLongForce = Mathf.Clamp(actualLongForce, -peakLongGrip, peakLongGrip);
            _longForceState = actualLongForce;

            float groundTorque = actualLongForce * radius;
            float netTorque = appliedTorque - groundTorque;
            float angularAccel = netTorque / Config.Inertia;
            float nextAngularVel = WheelAngularVel + angularAccel * subDt;
            WheelAngularVel = ClampWheelStopDeadband(nextAngularVel, powerTorque);
        }

        // 2. Final longitudinal force
        float finalWheelSpeed = WheelAngularVel * radius;
        float finalSlipRatio = CalculateSlipRatio(finalWheelSpeed, carSpeedLong);
        float finalLongForce = _longForceState;

        // 3. Lateral force
        float peakLatGrip = fz * CurrLatPeakFriction;
        float slipAngle = CalculateSlipAngle(v);
        float latStiffness = Config.LatStiffness * GetPressureStiffnessFactor();
        float pacejkaFy = CalculatePacejkaLat(slipAngle, latStiffness, peakLatGrip);
        float lowSpeedFy = Mathf.Clamp(-v.Y * fz * TireConfig.LowSpeedLatDamping, -peakLatGrip, peakLatGrip);
        float latBlend = CalculateSpeedBlend(Mathf.Abs(carSpeedLong));
        float rawFy = Mathf.Lerp(lowSpeedFy, pacejkaFy, latBlend);

        // 4. Friction Ellipse Constraint
        float longRatio = Mathf.Clamp(Mathf.Abs(finalLongForce) / peakLongGrip, 0f, 1f);
        float allowedLat = peakLatGrip * Mathf.Sqrt(1f - longRatio * longRatio);
        float targetFy = Mathf.Clamp(rawFy, -allowedLat, allowedLat);
        float actualFy = RelaxForce(_latForceState, targetFy, TireConfig.LatRelaxationLength, v.Length(), dt);
        actualFy = Mathf.Clamp(actualFy, -allowedLat, allowedLat);
        _latForceState = actualFy;

        // 5. Heat & Wear
        float slipPowerLong = Mathf.Abs(finalLongForce) * Mathf.Abs(finalWheelSpeed - carSpeedLong);
        float slipPowerLat = Mathf.Abs(actualFy) * Mathf.Abs(v.Y);
        UpdateHeatAndWear(fz, v.Length(), slipPowerLong, slipPowerLat, envTemp, dt);

        // 6. Judge
        bool isBraking = (finalLongForce * carSpeedLong) < -0.1f;
        bool isLockedUp = isBraking && (Mathf.Abs(finalWheelSpeed) < 0.2f);
        bool isSliding = Mathf.Abs(finalSlipRatio) > 0.12f || (Mathf.Abs(rawFy) > allowedLat + 0.1f);

        return new TireOutput
        {
            Force = new Vector2(finalLongForce, actualFy),
            SlipRatio = finalSlipRatio,
            SlipAngle = slipAngle,
            IsSliding = isSliding,
            IsLockedUp = isLockedUp
        };
    }

    private static float CalculateSlipRatio(float wheelSpeed, float carSpeed)
    {
        float denom = Mathf.Max(Mathf.Max(Mathf.Abs(wheelSpeed), Mathf.Abs(carSpeed)), TireConfig.MinStableSlipSpeed);
        return (wheelSpeed - carSpeed) / denom;
    }

    private static float CalculateSlipAngle(Vector2 tireVelocity)
    {
        return Mathf.Atan2(tireVelocity.Y, Mathf.Max(Mathf.Abs(tireVelocity.X), TireConfig.MinStableSlipSpeed));
    }

    private static float GetAppliedWheelTorque(float requestedTorque, float carSpeedLong, float wheelAngularVel)
    {
        if (requestedTorque >= 0f) return requestedTorque;

        float brakeMagnitude = -requestedTorque;
        float direction = 0f;
        if (Mathf.Abs(wheelAngularVel) > TireConfig.BrakeLockAngularVel)
        {
            direction = Mathf.Sign(wheelAngularVel);
        }
        else if (Mathf.Abs(carSpeedLong) > TireConfig.MinStableSlipSpeed)
        {
            direction = Mathf.Sign(carSpeedLong);
        }

        return direction == 0f ? 0f : -direction * brakeMagnitude;
    }

    private static float ClampWheelStopDeadband(float wheelAngularVel, float requestedTorque)
    {
        if (requestedTorque > 0f) return wheelAngularVel;
        return wheelAngularVel < TireConfig.WheelStopDeadbandAngularVel ? 0f : wheelAngularVel;
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

    private float CalculateBlendedLongForce(float powerTorque, float radius, float slipRatio, float carSpeedLong, float peakGrip)
    {
        float torqueForce = powerTorque / radius;
        float lowSpeedForce = Mathf.Clamp(
            torqueForce + slipRatio * peakGrip * TireConfig.LowSpeedLongSlipStiffness,
            -peakGrip,
            peakGrip
        );
        float pacejkaForce = CalculatePacejkaLong(slipRatio, Config.LongStiffness, peakGrip);
        float blendSpeed = Mathf.Abs(carSpeedLong);

        return Mathf.Lerp(lowSpeedForce, pacejkaForce, CalculateSpeedBlend(blendSpeed));
    }

    private static float CalculateSpeedBlend(float speed)
    {
        float t = Mathf.InverseLerp(TireConfig.MinStableSlipSpeed, TireConfig.FullPacejkaSpeed, speed);
        return Mathf.SmoothStep(0f, 1f, t);
    }

    private static float RelaxForce(float current, float target, float relaxationLength, float speed, float dt)
    {
        float effectiveSpeed = Mathf.Max(speed, TireConfig.MinStableSlipSpeed);
        float tau = relaxationLength / effectiveSpeed;
        float alpha = 1.0f - Mathf.Exp(-dt / tau);
        return current + (target - current) * alpha;
    }

    private void UpdateHeatAndWear(float fz, float speedMs, float longSlipPower, float latSlipPower, float envTemp, float dt)
    {
        // Friction energy
        float longEnergy = longSlipPower * dt;
        float latEnergy = latSlipPower * dt;
        float totalFrictionEnergy = longEnergy + latEnergy;
        float rollingEnergy = fz * speedMs * Config.DefaultRollingResCoef * dt;
        float surfaceFrictionEnergy = totalFrictionEnergy * TireConfig.SurfaceFrictionHeatFraction;
        float coreFrictionEnergy = totalFrictionEnergy * TireConfig.CoreFrictionHeatFraction;

        float surfaceMass = Config.Mass * TireConfig.SurfaceMassRatio;
        float coreMass = Config.Mass - surfaceMass;

        // Surface temp rise
        float tempRise = surfaceFrictionEnergy / (surfaceMass * TireConfig.SpecificHeat);
        SurfaceTemp += tempRise;

        // Friction heat also reaches the tire body. The remaining energy is
        // dissipated into the road surface, wear debris and rubber damage.
        CoreTemp += (rollingEnergy + coreFrictionEnergy) / (coreMass * TireConfig.SpecificHeat);

        // Internal heat conduction (surface to core)
        float internalTransfer = (SurfaceTemp - CoreTemp) * TireConfig.InternalHeatTransCoef * dt;
        SurfaceTemp -= internalTransfer / (surfaceMass * TireConfig.SpecificHeat);
        CoreTemp += internalTransfer / (coreMass * TireConfig.SpecificHeat);

        // Air cooling
        float coolingRate = TireConfig.BaseCoolingCoef + TireConfig.AirCoolingCoef * speedMs;
        float envTransfer = (SurfaceTemp - envTemp) * coolingRate * dt;
        SurfaceTemp -= envTransfer / (surfaceMass * TireConfig.SpecificHeat);

        // Wear calculation (based on friction work)
        float longWear = longEnergy * Config.BaseLongWearRate;
        float latWear = latEnergy * Config.BaseLatWearRate;
        float rollingWear = rollingEnergy * Config.RollingWearEnergyRate;
        // Overheat penalty
        float thermalMult = 1.0f;
        if (SurfaceTemp > Config.OptimalMaxTemp)
            thermalMult += (SurfaceTemp - Config.OptimalMaxTemp) * Config.ThermalWearCoef;
        Wear = Mathf.Clamp(Wear + (longWear + latWear + rollingWear) * thermalMult, 0f, 1f);

        // Tire pressure update (ideal gas equation)
        Pressure = condition.Pressure * (CoreTemp + 273.15f) / (condition.Temp + 273.15f);
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
