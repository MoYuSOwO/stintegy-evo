using Godot;
using PloyRacing.Core.Car.Configs;

namespace PloyRacing.Core.Car.Components;

public class PowerComponent(PowerConfig config)
{
    public readonly PowerConfig Config = config;

    public PowerOutput UpdateAndGetDriveForce(float input, float carSpeed, float battery)
    {
        // 动能回收 + 刹车
        if (input < -PowerConfig.InputDeadZone)
        {
            float brake = -input;

            // 满电不能回收
            if (battery > 0.999f)
            {
                return new()
                {
                    Drive = -Config.MaxBrakeForce * brake,
                    Regen = 0
                };
            }

            // 不满电
            else
            {
                // 仅动能回收
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
                
                // 动能回收力不够
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

        // 驱动模式
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

        // 滑行
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