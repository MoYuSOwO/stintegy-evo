using System;

namespace StintegyEVO.Core.Cars;

/// <summary>
/// The game's randomness, as a car or a driver is handed it: a seed, split
/// by name or number into sources of their own, each of which draws the
/// same numbers every time it is asked.
///
/// A race is seeded once (<c>RaceSimulation.Random</c>) and splits that seed
/// for each car, for the car and the driver in it, and for each step, so
/// what one of them draws never moves what another does, and a race run
/// again on the same seed comes out the same. Nothing is consumed: a driver
/// that looks ahead by stepping a copy of its car draws what the car will
/// draw, and the race goes on as if it had not looked. A component never
/// reaches for <see cref="System.Random"/> or the clock; what it needs by
/// chance it takes from the source it is given, and splits it further for
/// its own parts (<see cref="For(string)"/>).
/// </summary>
public readonly record struct RandomSource(ulong Seed)
{
    /// <summary>A source of its own for the part called <paramref name="name"/>.</summary>
    public RandomSource For(string name)
    {
        ArgumentNullException.ThrowIfNull(name);
        // FNV-1a over the characters: the same on every run and every
        // machine, which string.GetHashCode is not.
        ulong hash = 14695981039346656037UL;
        foreach (char c in name)
        {
            hash = (hash ^ c) * 1099511628211UL;
        }
        return For(hash);
    }

    /// <summary>A source of its own for the <paramref name="index"/>th of something: a step, a wheel.</summary>
    public RandomSource For(long index) => For(unchecked((ulong)index));

    private RandomSource For(ulong key) =>
        new(Mix(Seed ^ Mix(unchecked(key + 0x9E3779B97F4A7C15UL))));

    /// <summary>The numbers this source draws, from the first; the same every time.</summary>
    public RandomStream Draw() => new(Seed);

    internal static ulong Mix(ulong z)
    {
        unchecked
        {
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }
    }
}

/// <summary>The numbers one <see cref="RandomSource"/> draws, in order.</summary>
public struct RandomStream(ulong seed)
{
    private ulong _state = seed;

    /// <summary>Any 64 bits.</summary>
    public ulong NextUInt64()
    {
        _state = unchecked(_state + 0x9E3779B97F4A7C15UL);
        return RandomSource.Mix(_state);
    }

    /// <summary>At least zero and less than one, evenly.</summary>
    public float NextFloat() => (NextUInt64() >> 40) * (1f / (1 << 24));

    /// <summary>At least <paramref name="min"/> and less than <paramref name="max"/>, evenly.</summary>
    public float NextFloat(float min, float max) => min + (max - min) * NextFloat();

    /// <summary>Spread about zero as a bell, one standard deviation wide.</summary>
    public float NextNormal()
    {
        double u = ((NextUInt64() >> 11) + 1) * (1.0 / (1UL << 53));
        double v = (NextUInt64() >> 11) * (1.0 / (1UL << 53));
        return (float)(Math.Sqrt(-2.0 * Math.Log(u)) * Math.Cos(2.0 * Math.PI * v));
    }
}
