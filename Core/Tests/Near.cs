using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// Equality for numbers that come out of arithmetic: within a per cent of
/// each other, or within an absolute floor near zero. The digits a different
/// CPU produces may move; what the test is about may not.
/// </summary>
internal static class Near
{
    public static void Equal(float expected, float actual, float rel = 0.01f, float abs = 1e-3f)
    {
        float allowed = MathF.Max(abs, rel * MathF.Max(MathF.Abs(expected), MathF.Abs(actual)));
        Assert.True(
            MathF.Abs(expected - actual) <= allowed,
            $"expected about {expected}, got {actual} (allowed {allowed})");
    }

    public static void Equal(double expected, double actual, double rel = 0.01, double abs = 1e-3)
    {
        double allowed = Math.Max(abs, rel * Math.Max(Math.Abs(expected), Math.Abs(actual)));
        Assert.True(
            Math.Abs(expected - actual) <= allowed,
            $"expected about {expected}, got {actual} (allowed {allowed})");
    }
}
