using Godot;
using PloyRacing.Core.Car.Configs;

namespace PloyRacing.Core.Car.Components;

public class PowerComponent(PowerConfig config)
{
    public readonly PowerConfig Config = config;
    public int CurrentGear { get; private set; } = 1;
    public float CurrentRPM { get; private set; } = config.IdleRPM;
    public float RPMRatio => (CurrentRPM - Config.IdleRPM) / (Config.RedlineRPM - Config.IdleRPM + 0.01f);
    private float shiftTimer = 0.0f;
    private float clutchKick = 0.0f;
    private float revLimiterTimer = 0.0f;

    public float CalculateDriveForce(float throttle, float carSpeed, float dt)
    {
        UpdateGearAndFakeRPM(throttle, carSpeed, dt);

        if (shiftTimer > 0) return 0f;

        float currentGearMaxSpeed = Config.GetMaxSpeedAtGear(CurrentGear);
        if (CurrentRPM >= Config.RedlineRPM || carSpeed >= currentGearMaxSpeed)
        {
            if (revLimiterTimer <= 0f)
            {
                revLimiterTimer = 0.06f;
            }
        }
        if (revLimiterTimer > 0)
        {
            revLimiterTimer -= dt;
            CurrentRPM -= 200.0f * dt; 
            return 0f;
        }

        float engineBaseForce = Config.BaseForce * throttle;
        float currentGearMultiplier = Config.GetMultiplierAtGear(CurrentGear);
        float multipliedForce = engineBaseForce * currentGearMultiplier;

        float curveFactor = Config.GetForceRatioAtRPMRatio(RPMRatio);

        return multipliedForce * curveFactor;
    }

    private void UpdateGearAndFakeRPM(float throttle, float carSpeed, float dt)
    {
        if (carSpeed < 0.1f) {
            CurrentGear = 1;
            CurrentRPM = Config.IdleRPM;
            clutchKick = 0.0f;
            shiftTimer = 0.0f;
            return;
        }

        // 升档
        if (CurrentGear < Config.MaxGear) {
            if (carSpeed > Config.GetMaxSpeedAtGear(CurrentGear)) {
                CurrentGear++;
                shiftTimer = 0.2f;
                clutchKick = 1.0f;
            }
        }

        // 降档 (迟滞回线，防止跳档)
        if (CurrentGear > 1) {
            if (carSpeed < Config.GetMaxSpeedAtGear(CurrentGear - 1) * 0.8f) {
                CurrentGear--;
                shiftTimer = 0.1f;
                clutchKick = 1.0f;
            }
        }

        // 动力切断
        if (shiftTimer > 0) {
            shiftTimer -= dt;
        } else {
            shiftTimer = 0.0f;
        }

        // RPM
        float gearMaxSpeed = Config.GetMaxSpeedAtGear(CurrentGear);
        float ratio = carSpeed / gearMaxSpeed;
        float baseRPM = Mathf.Max(Config.IdleRPM, ratio * Config.RedlineRPM);

        // 负载虚位
        float loadFactor = (throttle - 0.3f) * 1500.0f;
        if (CurrentGear > 1) loadFactor *= 0.5f;
        
        // 高频噪声
        float time = (float)(Time.GetTicksUsec() / 1000000.0f);
        float noiseMag = 20.0f + (throttle * 50.0f);
        float vibration = (float) Mathf.Sin(time * 60.0f) * noiseMag;
        vibration += (float) ((GD.Randf() - 0.5f) * 30.0f);

        // 换挡冲击
        if (clutchKick > 0) {
            clutchKick -= dt * 5.0f;
        }
        float kickRPM = clutchKick * 500.0f;
        float targetRPM = baseRPM + loadFactor + vibration + kickRPM;

        // 平滑
        float smoothness;
        if (targetRPM > CurrentRPM) {
            smoothness = 15.0f;
        } else {
            smoothness = 5.0f;
        }

        // 限制红线
        CurrentRPM += (targetRPM - CurrentRPM) * smoothness * dt;
        CurrentRPM = Mathf.Max(Config.IdleRPM, CurrentRPM);
    }
}