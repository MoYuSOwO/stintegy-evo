using Godot;
using PloyRacing.Core.Car.Configs;

namespace PloyRacing.Core.Car.Components;

public class AeroComponent(AeroConfig config)
{
    public readonly AeroConfig Config = config;

    public AeroOutput UpdateAero(float speed, float dirtyAirFactor = 0f)
    {
        // 动态压力： 0.5 * rho * v^2
        float dynamicPressure = 0.5f * AeroConfig.AirDensity * (speed * speed);

        // 乱流影响：跟车时，阻力变小 (尾流效应)，下压力暴跌 (失去抓地力)
        float dragMultiplier = 1.0f - (dirtyAirFactor * 0.2f); // 阻力最多减少 20%
        float dfMultiplier = 1.0f - (dirtyAirFactor * 0.4f);   // 下压力最多暴跌 40%

        // 计算阻力 Fd = 0.5 * rho * v^2 * Cd * A
        float drag = dynamicPressure * Config.BaseDragCoef * Config.FrontalArea * dragMultiplier;

        // 计算总下压力
        float totalDownforce = dynamicPressure * Config.DownforceCoef * Config.FrontalArea * dfMultiplier;

        return new AeroOutput
        {
            DragForce = drag,
            DownforceFront = totalDownforce * Config.AeroBalanceFront,
            DownforceRear = totalDownforce * (1.0f - Config.AeroBalanceFront)
        };
    }
}