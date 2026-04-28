using Godot;
using PloyRacing.Core.Car.Configs;
using System;

namespace PloyRacing.Core.Car.Components;

public class BatteryComponent(BatteryConfig config, float initialKWh)
{
    private readonly BatteryConfig Config = config;

    public float CurrentKWh { get; private set; } = Math.Clamp(initialKWh, 0f, config.CapacityKWh);
    public float SocPct => CurrentKWh / Config.CapacityKWh;

    public void Consume(float mechanicalPowerWatts, float dt)
    {
        if (mechanicalPowerWatts <= 0) return;
        float elecPowerWatts = mechanicalPowerWatts / Config.DriveEfficiency;
        float energyKWh = BatteryConfig.WsToKWh(elecPowerWatts * dt);
        CurrentKWh = Math.Max(0f, CurrentKWh - energyKWh);
    }

    public void Regen(float mechanicalPowerWatts, float dt)
    {
        if (mechanicalPowerWatts <= 0) return;
        float elecPowerWatts = mechanicalPowerWatts * Config.RegenEfficiency;
        float energyKWh = BatteryConfig.WsToKWh(elecPowerWatts * dt);
        CurrentKWh = Math.Min(Config.CapacityKWh, CurrentKWh + energyKWh);
    }

    public float Charge(float seconds)
    {
        if (CurrentKWh >= Config.CapacityKWh) return 0f;
        float maxEnergy = Config.MaxChargePowerKW * (seconds / 3600f);
        float actual = Math.Min(maxEnergy, Config.CapacityKWh - CurrentKWh);
        CurrentKWh += actual;
        return actual;
    }

    public void Reset()
    {
        CurrentKWh = Config.CapacityKWh;
    }
}