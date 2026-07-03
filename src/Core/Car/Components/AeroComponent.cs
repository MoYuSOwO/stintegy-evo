using Godot;
using StintegyEVO.Core.Car.Configs;

namespace StintegyEVO.Core.Car.Components;

public class AeroComponent(AeroConfig config)
{
    public readonly AeroConfig Config = config;

    public AeroOutput CalculateAero(float speed, float dirtyAirFactor = 0f)
    {
        return CalculateAero(Config, speed, dirtyAirFactor);
    }

    public static AeroOutput CalculateAero(AeroConfig config, float speed, float dirtyAirFactor = 0f)
    {
        // Dynamic pressure： 0.5 * rho * v^2
        float dynamicPressure = 0.5f * AeroConfig.AirDensity * (speed * speed);

        // Turbulence effects: When following another vehicle, drag decreases (wake effect), downforce drops sharply (loss of grip).
        float dragMultiplier = 1.0f - (dirtyAirFactor * 0.2f); // drag decreases at most 20%
        float dfMultiplier = 1.0f - (dirtyAirFactor * 0.4f);   // downforce decreases at most 40%

        // Calculate drag: Fd = 0.5 * rho * v^2 * Cd * A
        float drag = dynamicPressure * config.BaseDragCoef * config.FrontalArea * dragMultiplier;

        float totalDownforce = dynamicPressure * config.DownforceCoef * config.FrontalArea * dfMultiplier;

        return new AeroOutput
        {
            DragForce = drag,
            DownforceFront = totalDownforce * config.AeroBalanceFront,
            DownforceRear = totalDownforce * (1.0f - config.AeroBalanceFront)
        };
    }
}
