using Godot;
using PloyRacing.Core.Car.Configs;
using System;

namespace PloyRacing.Core.Car.Components;

public class DifferentialComponent(DifferentialConfig config)
{
    public readonly DifferentialConfig Config = config;

    public DifferentialOutput DistributeForce(
        float inputForce, float gripLeft, float gripRight,
        float carSpeed, float angularVelocity, float carWidth
    )
    {
        DifferentialOutput result = new();
        float halfForce = inputForce * 0.5f;

        // 核心参数：根据是在加速还是减速，决定当前的锁止率 (0.0 ~ 1.0)
        float currentRamp = (inputForce >= 0) ? Config.PowerRamp : Config.CoastRamp;

        float forceL = halfForce;
        float forceR = halfForce;

        // 左轮打滑，按比例转移力
        if (Math.Abs(forceL) > gripLeft)
        {
            float overflow = Math.Abs(forceL) - gripLeft;
            float transfer = overflow * currentRamp * Math.Sign(forceL);
            forceL = gripLeft * Math.Sign(forceL);
            forceR += transfer;
        }
        // 右轮打滑，按比例转移力
        else if (Math.Abs(forceR) > gripRight)
        {
            float overflow = Math.Abs(forceR) - gripRight;
            float transfer = overflow * currentRamp * Math.Sign(forceR);
            forceR = gripRight * Math.Sign(forceR);
            forceL += transfer;
        }

        // 兜底安全限制，确保绝不超过抓地极限
        result.ForceLeft = Math.Clamp(forceL, -gripLeft, gripLeft);
        result.ForceRight = Math.Clamp(forceR, -gripRight, gripRight);

        // 终极魔法：推头惩罚现在是“连续”的！
        // Ramp 为 0 时 (开放)，惩罚为 0，车极其灵活；
        // Ramp 为 1 时 (锁死)，惩罚拉满，车变成推土机。
        float speedForMath = Math.Max(carSpeed, 1.0f);
        result.YawPenalty = -angularVelocity * (Config.LockedStiffness / (2.0f * speedForMath)) * (carWidth * carWidth) * currentRamp;

        return result;
    }
}