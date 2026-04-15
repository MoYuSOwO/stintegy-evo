using Godot;
using PloyRacing.Core.Car.Configs;
using PloyRacing.Util;

namespace PloyRacing.Core.Car.Components;

public class SuspensionComponent(SuspensionConfig config, CarLoad initialLoad)
{
    public readonly SuspensionConfig Config = config;
    private CarLoad load = initialLoad;

    public CarLoad UpdateLoads(
        float mass, float staticWeightDistFront, float cgHeight, float wheelBase, float width,
        float accelLong, float accelLat, float downforceFront, float downforceRear, float dt
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
        float targetFL = (staticFront / 2f) - (deltaWeightLong / 2f) - deltaLatFront + (downforceFront / 2f);
        float targetFR = (staticFront / 2f) - (deltaWeightLong / 2f) + deltaLatFront + (downforceFront / 2f);
        float targetRL = (staticRear / 2f)  + (deltaWeightLong / 2f) - deltaLatRear  + (downforceRear / 2f);
        float targetRR = (staticRear / 2f)  + (deltaWeightLong / 2f) + deltaLatRear  + (downforceRear / 2f);

        // 防止车辆腾空导致载荷变负
        targetFL = Mathf.Max(0.1f, targetFL);
        targetFR = Mathf.Max(0.1f, targetFR);
        targetRL = Mathf.Max(0.1f, targetRL);
        targetRR = Mathf.Max(0.1f, targetRR);

        // 避震筒阻尼模拟 (Low-Pass Filter)
        // 让载荷平滑过渡，消除 Pacejka 因为载荷突变导致的数值震荡
        float alpha = 1.0f - (float) Mathf.Exp(-Config.DampingSpeed * dt);
        load.FrontLeft += (targetFL - load.FrontLeft) * alpha;
        load.FrontRight += (targetFR - load.FrontRight) * alpha;
        load.RearLeft += (targetRL - load.RearLeft) * alpha;
        load.RearRight += (targetRR - load.RearRight) * alpha;

        return load;
    }
}