using Godot;
using PloyRacing.Core.Car.Configs;
using System;

namespace PloyRacing.Core.Car.Components;

public class FuelComponent(FuelConfig config, float initialL)
{
    public readonly FuelConfig Config = config;

    public float CurrentL { get; private set; } = Math.Clamp(initialL, 0f, config.CapacityL);

    public float CurrentFuelMassKg => CurrentL * FuelConfig.FuelDensity;
    public float FullFuelMassKg => Config.CapacityL * FuelConfig.FuelDensity;
    public float FuelPct => CurrentL / Config.CapacityL;

    public void UpdateFuel(float rpmRatio, float throttle, float dt)
    {
        if (CurrentL <= 0f) return;

        float burnRate = Config.BasicBurnRate + (Config.MaxBurnRate * (float)Math.Pow(rpmRatio, 1.1) * throttle);
        
        CurrentL -= burnRate * dt;
        CurrentL = Math.Max(0f, CurrentL);
    }

    public void Refuel(float amountL)
    {
        CurrentL = Math.Min(Config.CapacityL, CurrentL + amountL);
    }
}