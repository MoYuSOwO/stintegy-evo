using StintegyEVO.Drivers.Lanes;

namespace StintegyEVO.Drivers.Racecraft;

/// <summary>
/// What the racecraft layer gives back each time it draws, one number per
/// control point of the line and one for the brakes, each between -1 and 1,
/// and what that drawing is to the driver: offsets from the racing line and
/// the share of the next braking zone's braking to take it at.
/// </summary>
public static class RacecraftAction
{
    /// <summary>How far off the racing line the furthest drawing reaches, either way.</summary>
    public const float MaxOffsetMeters = 8f;

    /// <summary>The brake factor the brake number runs between, -1 to 1.</summary>
    public const float MinBrakeFactor = 0.85f;
    public const float MaxBrakeFactor = 1.03f;

    public static int Points => DrawnLine.PointMeters.Length;
    public static int Size => Points + 1;

    /// <summary>The drawing <paramref name="action"/> asks for: its offsets into <paramref name="offsets"/>, and its brake factor.</summary>
    public static float Decode(ReadOnlySpan<float> action, Span<float> offsets)
    {
        if (action.Length != Size)
            throw new ArgumentException($"A racecraft action has {Size} numbers, not {action.Length}.", nameof(action));
        for (int k = 0; k < Points; k++)
            offsets[k] = MaxOffsetMeters * Math.Clamp(action[k], -1f, 1f);
        float brake = 0.5f * (Math.Clamp(action[Points], -1f, 1f) + 1f);
        return MinBrakeFactor + (MaxBrakeFactor - MinBrakeFactor) * brake;
    }

    /// <summary>The action that draws the racing line and brakes as planned.</summary>
    public static void RacingLine(Span<float> action)
    {
        action[..Points].Clear();
        action[Points] = (2f * (1f - MinBrakeFactor) / (MaxBrakeFactor - MinBrakeFactor)) - 1f;
    }
}
