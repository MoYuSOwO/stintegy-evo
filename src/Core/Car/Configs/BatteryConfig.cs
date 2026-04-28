using Godot;

namespace PloyRacing.Core.Car.Configs;

[GlobalClass]
public partial class BatteryConfig : Resource
{
    [Export] public float MaxChargePowerKW { get; set; } = 150f; // 最大充电功率 (千瓦)

    [Export] public float CapacityKWh { get; set; } = 80f; // 电池最大电量 (千瓦时)

    [Export(PropertyHint.Range, "0.5,1.0")]
    public float DriveEfficiency { get; set; } = 0.92f; // 驱动时电能→机械能效率

    [Export(PropertyHint.Range, "0.3,0.9")]
    public float RegenEfficiency { get; set; } = 0.7f; // 动能回收时机械能→电能效率

    public static float KWhToWs(float KWh)
    {
        return KWh * 3600000f;
    }

    public static float WsToKWh(float Ws)
    {
        return Ws / 3600000f;
    }
}