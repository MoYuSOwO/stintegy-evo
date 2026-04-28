using Godot;
using PloyRacing.Core.Car.Configs;
using PloyRacing.Util;

namespace PloyRacing.Core.Car.Components;

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

        // 理论总转移量
        float deltaWeightLong = (mass * accelLong * cgHeight) / wheelBase;
        float deltaWeightLat = (mass * accelLat * cgHeight) / width;

        // Roll Balance 魔法分配
        float deltaLatFront = deltaWeightLat * Config.FrontRollBalance;
        float deltaLatRear  = deltaWeightLat * (1.0f - Config.FrontRollBalance);

        // 计算稳态目标载荷
        float targetFL = (staticFront / 2f) - (deltaWeightLong / 2f) + deltaLatFront + (downforceFront / 2f);
        float targetFR = (staticFront / 2f) - (deltaWeightLong / 2f) - deltaLatFront + (downforceFront / 2f);
        float targetRL = (staticRear / 2f)  + (deltaWeightLong / 2f) + deltaLatRear  + (downforceRear / 2f);
        float targetRR = (staticRear / 2f)  + (deltaWeightLong / 2f) - deltaLatRear  + (downforceRear / 2f);

        // 防止车辆腾空导致载荷变负
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

        // 避震筒阻尼模拟 (Low-Pass Filter)
        // 让载荷平滑过渡，消除 Pacejka 因为载荷突变导致的数值震荡
        float alpha = 1.0f - (float) Mathf.Exp(-Config.DampingSpeed * dt);
        _load.FrontLeft += (target.FrontLeft - _load.FrontLeft) * alpha;
        _load.FrontRight += (target.FrontRight - _load.FrontRight) * alpha;
        _load.RearLeft += (target.RearLeft - _load.RearLeft) * alpha;
        _load.RearRight += (target.RearRight - _load.RearRight) * alpha;

        return _load;
    }
}