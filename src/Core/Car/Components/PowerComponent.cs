using Godot;
using StintegyEVO.Core.Car.Configs;

namespace StintegyEVO.Core.Car.Components;

public class PowerComponent(PowerConfig config)
{
    public readonly PowerConfig Config = config;

    public PowerOutput UpdateAndGetDriveForce(float input, float carSpeed, float battery)
    {
        // Regen & Brake
        if (input < -PowerConfig.InputDeadZone)
        {
            float brake = -input;

            // No regen when battery fulls
            if (battery > 0.999f)
            {
                return new()
                {
                    Drive = -Config.MaxBrakeForce * brake,
                    Regen = 0
                };
            }

            // Battery is not fulled
            else
            {
                // Regen only
                if (Config.MaxBrakeForce * brake < Config.MaxRegenForce)
                {
                    float regenForce = Config.MaxBrakeForce * brake;
                    float speedFactor = Mathf.Clamp(carSpeed / 2f, 0f, 1f);
                    return new()
                    {
                        Drive = -regenForce * speedFactor,
                        Regen = regenForce * speedFactor
                    };
                }
                
                // Regen & Brake
                else
                {
                    float brakeForce = Config.MaxBrakeForce * brake;
                    float regenForce = Config.MaxRegenForce;
                    float speedFactor = Mathf.Clamp(carSpeed / 2f, 0f, 1f);
                    return new()
                    {
                        Drive = -brakeForce * speedFactor,
                        Regen = regenForce * speedFactor
                    };
                } 
            }
        }

        // Drive
        else if (input > PowerConfig.InputDeadZone)
        {
            float throttle = input;
            float maxForceAtSpeed = Config.CalcMaxDriveForceAtSpeed(carSpeed);
            return new()
            {
                Drive = throttle * maxForceAtSpeed,
                Regen = 0
            };
        }

        // Coasting
        else
        {
            if (battery > 0.999f)
            {
                return new()
                {
                    Drive = 0,
                    Regen = 0
                };
            }
            float speedFactor = Mathf.Clamp(carSpeed / 2f, 0f, 1f);
            return new()
            {
                Drive = -Config.BaseRegenForce * speedFactor,
                Regen = Config.BaseRegenForce * speedFactor
            };
        }
    }
}