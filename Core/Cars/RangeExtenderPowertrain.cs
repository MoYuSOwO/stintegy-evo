using System;
using System.Collections.Generic;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// Electric drive with an engine that only makes electricity.
///
/// The wheels are driven by the motors alone and the motors draw on the
/// pack alone; the engine never touches a wheel. It turns a generator at a
/// steady output for as long as there is fuel and the pack has room, so the
/// pack is a buffer between a steady supply and a demand that swings from
/// full power to full regeneration twice a corner.
///
/// The buffer is the whole point. A pack this size holds several laps of the
/// difference between the two, so a lap at full power never runs it short
/// and nothing about the car changes on a straight: the generator's output
/// is spent in the background. What runs out over a race is the fuel, which
/// is what the race-long budget is kept in.
/// </summary>
public sealed class RangeExtenderPowertrain : IPowertrain
{
    private const float Epsilon = 1e-5f;
    private const float FuelEnergyJoulesPerKg = 43_000_000f;

    public static readonly RangeExtenderPowertrain Default = new();

    private static readonly ModeLadder Ladder = new(
        "Power",
        "Save",
        "Eco",
        "Normal",
        "Push",
        "Attack"
    );

    // Built on first use: the allowance is an init property, set after any constructor runs.
    public IReadOnlyList<PowertrainResourceInfo> Resources => _resources ??=
    [
        new("battery", "Charge", 0f, PowertrainResourceClass.Battery),
        new("fuel", "Fuel", FuelAllowanceKg, PowertrainResourceClass.Fuel),
    ];
    private PowertrainResourceInfo[]? _resources;

    public ModeLadder OutputLadder => Ladder;

    public float BatteryCapacityJoules { get; init; } = 30f * 3_600_000f;
    public float BatteryDriveEfficiency { get; init; } = 0.92f;
    public float RegenEfficiency { get; init; } = 0.85f;
    public float RegenPowerCapWatts { get; init; } = 600_000f;
    public float MinPowerSpeed { get; init; } = 8f;

    /// <summary>The race's power: every rung, until the rungs become driver behaviour.</summary>
    public float RaceDrivePowerLimitWatts { get; init; } = 600_000f;

    /// <summary>Electrical output of the generator while it runs.</summary>
    public float GeneratorPowerWatts { get; init; } = 300_000f;

    /// <summary>Fuel to electricity at the generator's terminals.</summary>
    public float GeneratorEfficiency { get; init; } = 0.36f;

    /// <summary>The generator stops above this charge, so regeneration always has somewhere to go.</summary>
    public float GeneratorStopCharge { get; init; } = 0.95f;

    /// <summary>The event's fuel allowance, which is what the race-long budget is.</summary>
    public float FuelAllowanceKg { get; init; } = 112f;

    // Below this the pack's voltage sags, and past empty there is only a limp.
    public float LowVoltageStart { get; init; } = 0.05f;
    public float LowVoltageOutputFraction { get; init; } = 0.85f;
    public float EmptyBand { get; init; } = 0.005f;
    public float LowSocPowerFloor { get; init; } = 0.08f;

    public float OutputAvailability(in PowertrainState energy)
    {
        float charge = energy.Primary;
        if (charge >= LowVoltageStart)
            return 1f;
        float sag = LowVoltageOutputFraction +
                    (1f - LowVoltageOutputFraction) *
                    Math.Clamp(charge / Math.Max(LowVoltageStart, Epsilon), 0f, 1f);
        if (charge >= EmptyBand)
            return sag;
        float into = Math.Clamp(charge / Math.Max(EmptyBand, Epsilon), 0f, 1f);
        return LowSocPowerFloor + (sag - LowSocPowerFloor) * into;
    }

    public float DriveAccelerationLimit(
        in PowertrainState energy,
        in CarStrategy strategy,
        float speed,
        float massKg,
        float chassisForceCeiling
    )
    {
        float power = strategy.DrivePowerLimitWattsOverride ?? RaceDrivePowerLimitWatts;
        float powerLimitedAccel = power / (massKg * Math.Max(speed, MinPowerSpeed));
        return Math.Min(chassisForceCeiling, powerLimitedAccel) *
               OutputAvailability(energy);
    }

    public PowertrainSettlement Settle(
        in PowertrainState energy,
        float driveAccel,
        float brakeAccel,
        float speed,
        float massKg,
        float dt
    )
    {
        float drivePower = driveAccel <= 0f
            ? 0f
            : massKg * driveAccel * speed /
              Math.Max(BatteryDriveEfficiency, Epsilon);
        float regenPower = RecoveredPowerWatts(brakeAccel, speed, massKg);

        float fuelJoules = energy.Secondary * FuelAllowanceKg * FuelEnergyJoulesPerKg;
        float generatorPower = energy.Primary < GeneratorStopCharge && fuelJoules > 0f
            ? GeneratorPowerWatts
            : 0f;
        float fuelBurn = generatorPower / Math.Max(GeneratorEfficiency, Epsilon) * dt;
        if (fuelBurn > fuelJoules)
        {
            generatorPower *= fuelJoules / Math.Max(fuelBurn, Epsilon);
            fuelBurn = fuelJoules;
        }

        float netEnergy = (drivePower - regenPower - generatorPower) * dt;
        PowertrainState spent = energy;
        spent.Primary = Math.Clamp(
            spent.Primary - netEnergy / Math.Max(BatteryCapacityJoules, Epsilon),
            0f,
            1f
        );
        spent.Secondary = Math.Clamp(
            spent.Secondary - fuelBurn / Math.Max(FuelAllowanceKg * FuelEnergyJoulesPerKg, Epsilon),
            0f,
            1f
        );
        return new PowertrainSettlement(spent, drivePower);
    }

    public float RecoveredPowerWatts(float brakeAccel, float speed, float massKg)
    {
        if (brakeAccel <= 0f || speed <= 0f)
            return 0f;
        float brakePower = massKg * brakeAccel * speed;
        return Math.Min(brakePower * RegenEfficiency, RegenPowerCapWatts);
    }

    public float ConsumableMassKg(in PowertrainState energy)
    {
        return Math.Clamp(energy.Secondary, 0f, 1f) * FuelAllowanceKg;
    }
}
