using System;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// A small direct-mapped cache for the planner's quantized performance
/// estimates. Collisions only evict an older estimate; the stored key is
/// always checked, so a collision can cause recomputation but never return a
/// value for the wrong speed or curvature.
/// </summary>
internal sealed class PerformanceLookupCache
{
    private const int DefaultCapacity = 16_384;
    private readonly long[] _keys;
    private readonly float[] _values;
    private readonly int[] _generations;
    private readonly int _mask;
    private int _generation = 1;

    public PerformanceLookupCache(int capacity = DefaultCapacity)
    {
        if (capacity <= 0 || (capacity & (capacity - 1)) != 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(capacity),
                "Cache capacity must be a positive power of two."
            );
        }

        _keys = new long[capacity];
        _values = new float[capacity];
        _generations = new int[capacity];
        _mask = capacity - 1;
    }

    public bool TryGetValue(long key, out float value)
    {
        int index = Index(key);
        if (_generations[index] == _generation && _keys[index] == key)
        {
            value = _values[index];
            return true;
        }

        value = 0f;
        return false;
    }

    public void Set(long key, float value)
    {
        int index = Index(key);
        _keys[index] = key;
        _values[index] = value;
        _generations[index] = _generation;
    }

    public void Clear()
    {
        _generation = unchecked(_generation + 1);
        if (_generation != 0)
            return;

        Array.Clear(_generations);
        _generation = 1;
    }

    private int Index(long key)
    {
        uint speed = (uint)((ulong)key >> 32);
        uint curvature = (uint)key;
        uint hash = speed * 0x9E3779B1u ^ curvature * 0x85EBCA77u;
        hash ^= hash >> 16;
        return (int)(hash & _mask);
    }
}
