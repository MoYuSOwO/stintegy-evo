using System;

namespace StintegyEVO.Core.Cars;

public enum BatteryOutputMode
{
    Save = 1,
    Eco = 2,
    Normal = 3,
    Push = 4,
    Attack = 5
}

public enum TireUsageMode
{
    Protect = 1,
    Light = 2,
    Normal = 3,
    Push = 4,
    Attack = 5
}

public enum WheelId
{
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight
}

internal static class CarModeIndex
{
    public static int ToIndex(this BatteryOutputMode mode)
    {
        return Math.Clamp((int)mode, 1, 5) - 1;
    }

    public static int ToIndex(this TireUsageMode mode)
    {
        return Math.Clamp((int)mode, 1, 5) - 1;
    }
}
