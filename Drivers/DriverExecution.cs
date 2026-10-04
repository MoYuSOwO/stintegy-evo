using StintegyEVO.Core.Drivers;

namespace StintegyEVO.Drivers;

/// <summary>
/// What a rating changes about an otherwise identical plan.
///
/// Pace is only a cornering share: two percent from a rating of zero to a
/// rating of one hundred. Throttle and brake stay at whatever the tyres will
/// take. A slower corner carries itself down the following straight.
/// Consistency would move the brake marker and the line, and car control is
/// how much of a slide is given back to the tyre. Those two are reserved at
/// their means; the per-segment noise is not sampled yet.
/// </summary>
public readonly record struct DriverExecution
{
    /// <summary>Share of the planned corner that comes back out of the tyres. 0.95..1.</summary>
    public float PaceEfficiency { get; init; }

    /// <summary>Metres added to the station the speed table is read at. Positive is later.</summary>
    public float BrakeMarkerErrorMeters { get; init; }

    /// <summary>Metres added to the racing-line offset. Positive is to the left.</summary>
    public float LateralBiasMeters { get; init; }

    /// <summary>How completely a slide is handed back. One gives the excess back.</summary>
    public float ControlAuthority { get; init; }

    public static DriverExecution Full { get; } = new()
    {
        PaceEfficiency = 1f,
        ControlAuthority = 1f
    };

    public static DriverExecution From(DriverAbilities abilities)
    {
        float pace = Score(abilities.Pace);
        float control = Score(abilities.CarControl);
        return new DriverExecution
        {
            PaceEfficiency = 0.95f + 0.05f * pace,
            BrakeMarkerErrorMeters = 0f,
            LateralBiasMeters = 0f,
            ControlAuthority = control
        };
    }

    private static float Score(float rating) => Math.Clamp(rating * 0.01f, 0f, 1f);
}
