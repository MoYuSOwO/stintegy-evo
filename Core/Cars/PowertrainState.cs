using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// How much of each of its consumables a car has left, as a fraction of full.
///
/// Two of them, because two is what a car can physically carry and still be
/// one car: a pack, a tank, or - the interesting case - both, which is what
/// makes a hybrid's strategy a real decision rather than a slider. Which slot
/// means what is the powertrain's business to say, through its resource list.
/// Nothing outside a powertrain should read <see cref="Primary"/> and assume
/// it is charge.
///
/// Fields on a struct rather than an array because the speed planner clones a
/// car state for every candidate it considers, and a lookahead that allocates
/// per candidate is a lookahead that runs out of time.
/// </summary>
public struct PowertrainState : IEquatable<PowertrainState>
{
    public float Primary;
    public float Secondary;

    public PowertrainState(float primary, float secondary)
    {
        Primary = primary;
        Secondary = secondary;
    }

    /// <summary>
    /// Every store the same fraction full. What a car starts a race on, and
    /// what a test means when it says the car sets off with eighty per cent.
    /// </summary>
    public static PowertrainState Filled(float fraction)
    {
        return new PowertrainState(fraction, fraction);
    }

    public static PowertrainState Full => Filled(1f);

    public float this[int index]
    {
        readonly get => index == 0 ? Primary : Secondary;
        set
        {
            if (index == 0)
                Primary = value;
            else
                Secondary = value;
        }
    }

    /// <summary>Number of stores this type can hold, full or not.</summary>
    public const int Capacity = 2;

    public void Clamp()
    {
        Primary = Math.Clamp(Primary, 0f, 1f);
        Secondary = Math.Clamp(Secondary, 0f, 1f);
    }

    public readonly bool Equals(PowertrainState other)
    {
        return Primary.Equals(other.Primary) &&
               Secondary.Equals(other.Secondary);
    }

    public readonly override bool Equals(object? obj)
    {
        return obj is PowertrainState other && Equals(other);
    }

    public readonly override int GetHashCode()
    {
        return HashCode.Combine(Primary, Secondary);
    }

    public readonly override string ToString()
    {
        return $"({Primary:0.###}, {Secondary:0.###})";
    }
}

/// <summary>
/// What one step took out of the stores, and what it cost to take it. The
/// power drawn is larger than the power that reached the road, because
/// nothing converts for free - it is reported so telemetry can show the bill
/// rather than the delivery.
/// </summary>
public readonly record struct PowertrainSettlement(
    PowertrainState Energy,
    float DrawnPowerWatts
);
