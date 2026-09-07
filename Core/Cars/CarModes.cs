using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// How hard the strategist is asking the powertrain to run, from the setting
/// that hoards to the setting that spends.
///
/// Deliberately not named after a battery. The five positions are output
/// settings on a pack, engine maps on a petrol car and deployment settings on
/// a hybrid, and in all three cases they are the same choice: give up some of
/// what is left in the store to go quicker now. What each rung is called on a
/// given car comes from that car's powertrain, through its
/// <see cref="ModeLadder"/>.
/// </summary>
public enum PowerOutputMode
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

/// <summary>
/// The tyre half of the strategy table. Unlike the power ladder this one is
/// the same on every car, because every car that races has tyres and spends
/// them the same way, so its rungs are named here rather than by the
/// machinery.
/// </summary>
public static class TireLadder
{
    public static readonly ModeLadder Usage = new(
        "Tyres",
        "Protect",
        "Light",
        "Normal",
        "Push",
        "Attack"
    );
}

internal static class CarModeIndex
{
    public static int ToIndex(this PowerOutputMode mode)
    {
        return Math.Clamp((int)mode, 1, 5) - 1;
    }

    public static int ToIndex(this TireUsageMode mode)
    {
        return Math.Clamp((int)mode, 1, 5) - 1;
    }
}
