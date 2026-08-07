using StintegyEVO.Core.Drivers;
using Xunit;

namespace StintegyEVO.Core.Tests;

public sealed class PerformanceLookupCacheTests
{
    [Fact]
    public void StoredValueCanBeReadBack()
    {
        PerformanceLookupCache cache = new(capacity: 8);

        cache.Set(42L, 3.25f);

        Assert.True(cache.TryGetValue(42L, out float value));
        Assert.Equal(3.25f, value);
    }

    [Fact]
    public void CollisionNeverReturnsTheDisplacedKeysValue()
    {
        PerformanceLookupCache cache = new(capacity: 1);
        cache.Set(1L, 10f);
        cache.Set(2L, 20f);

        Assert.False(cache.TryGetValue(1L, out _));
        Assert.True(cache.TryGetValue(2L, out float value));
        Assert.Equal(20f, value);
    }

    [Fact]
    public void ClearInvalidatesStoredValues()
    {
        PerformanceLookupCache cache = new(capacity: 8);
        cache.Set(7L, 11f);

        cache.Clear();

        Assert.False(cache.TryGetValue(7L, out _));
    }
}
