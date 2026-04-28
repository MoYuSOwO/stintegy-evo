using Godot;
using StintegyEVO.Core.Car.Configs;

namespace StintegyEVO.Core.Car.Components;

public class AeroComponent(AeroConfig config)
{
    public readonly AeroConfig Config = config;

    public AeroOutput CalculateAero(float speed, float dirtyAirFactor = 0f)
    {
        // Dynamic pressure： 0.5 * rho * v^2
        float dynamicPressure = 0.5f * AeroConfig.AirDensity * (speed * speed);

        // Turbulence effects: When following another vehicle, drag decreases (wake effect), downforce drops sharply (loss of grip).
        float dragMultiplier = 1.0f - (dirtyAirFactor * 0.2f); // drag decreases at most 20%
        float dfMultiplier = 1.0f - (dirtyAirFactor * 0.4f);   // downforce decreases at most 40%

        // Calculate drag: Fd = 0.5 * rho * v^2 * Cd * A
        float drag = dynamicPressure * Config.BaseDragCoef * Config.FrontalArea * dragMultiplier;

        float totalDownforce = dynamicPressure * Config.DownforceCoef * Config.FrontalArea * dfMultiplier;

        return new AeroOutput
        {
            DragForce = drag,
            DownforceFront = totalDownforce * Config.AeroBalanceFront,
            DownforceRear = totalDownforce * (1.0f - Config.AeroBalanceFront)
        };
    }
}