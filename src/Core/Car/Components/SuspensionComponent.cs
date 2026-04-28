using Godot;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Util;

namespace StintegyEVO.Core.Car.Components;

public class SuspensionComponent(SuspensionConfig config, CarLoad initialLoad)
{
    public readonly SuspensionConfig Config = config;
    private CarLoad _load = initialLoad;
    public CarLoad Load => _load;

    public CarLoad CalculateSteadyStateLoad(
        float mass, float staticWeightDistFront, float cgHeight, float wheelBase, float width,
        float accelLong, float accelLat, float downforceFront, float downforceRear
    )
    {
        float totalWeight = mass * GeomUtil.g;
        float staticFront = totalWeight * staticWeightDistFront;
        float staticRear  = totalWeight * (1.0f - staticWeightDistFront);

        // Theoretical transfer
        float deltaWeightLong = (mass * accelLong * cgHeight) / wheelBase;
        float deltaWeightLat = (mass * accelLat * cgHeight) / width;

        // Roll Balance
        float deltaLatFront = deltaWeightLat * Config.FrontRollBalance;
        float deltaLatRear  = deltaWeightLat * (1.0f - Config.FrontRollBalance);

        // Calculate steady-state target load
        float targetFL = (staticFront / 2f) - (deltaWeightLong / 2f) + deltaLatFront + (downforceFront / 2f);
        float targetFR = (staticFront / 2f) - (deltaWeightLong / 2f) - deltaLatFront + (downforceFront / 2f);
        float targetRL = (staticRear / 2f)  + (deltaWeightLong / 2f) + deltaLatRear  + (downforceRear / 2f);
        float targetRR = (staticRear / 2f)  + (deltaWeightLong / 2f) - deltaLatRear  + (downforceRear / 2f);

        // Clamp
        targetFL = Mathf.Max(0.1f, targetFL);
        targetFR = Mathf.Max(0.1f, targetFR);
        targetRL = Mathf.Max(0.1f, targetRL);
        targetRR = Mathf.Max(0.1f, targetRR);

        return new CarLoad()
        {
            FrontLeft = targetFL,
            FrontRight = targetFR,
            RearLeft = targetRL,
            RearRight = targetRR
        };
    }

    public CarLoad UpdateAndGetLoad(
        float mass, float staticWeightDistFront, float cgHeight, float wheelBase, float width,
        float accelLong, float accelLat, float downforceFront, float downforceRear, float dt
    )
    {
        CarLoad target = CalculateSteadyStateLoad(
            mass, staticWeightDistFront, cgHeight, wheelBase, width,
            accelLong, accelLat, downforceFront, downforceRear
        );

        // damping simulation (Low-Pass Filter)
        float alpha = 1.0f - (float) Mathf.Exp(-Config.DampingSpeed * dt);
        _load.FrontLeft += (target.FrontLeft - _load.FrontLeft) * alpha;
        _load.FrontRight += (target.FrontRight - _load.FrontRight) * alpha;
        _load.RearLeft += (target.RearLeft - _load.RearLeft) * alpha;
        _load.RearRight += (target.RearRight - _load.RearRight) * alpha;

        return _load;
    }
}