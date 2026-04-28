using Godot;

namespace StintegyEVO.Core.Car.Configs;

[GlobalClass]
public partial class BatteryConfig : Resource
{
    [Export] public float MaxChargePowerKW { get; set; } = 150f;

    [Export] public float CapacityKWh { get; set; } = 80f;

    [Export(PropertyHint.Range, "0.5,1.0")]
    public float DriveEfficiency { get; set; } = 0.92f;

    [Export(PropertyHint.Range, "0.3,0.9")]
    public float RegenEfficiency { get; set; } = 0.7f;

    public static float KWhToWs(float KWh)
    {
        return KWh * 3600000f;
    }

    public static float WsToKWh(float Ws)
    {
        return Ws / 3600000f;
    }
}