using System;
using StintegyEVO.Core.Cars;

namespace StintegyEVO.Core.Racing;

/// <summary>
/// The air one car leaves behind it, as another car meets it: the world's
/// own rule, the one the race works each car's air out by, for anyone who
/// wants to know what the air will be before a car gets there.
/// </summary>
public static class WakeAir
{
    /// <summary>
    /// Downforce recovers quickly once a car moves sideways out of the wake;
    /// experiments find it recovers faster laterally than drag.
    /// </summary>
    private const float DirtyAirLateralRecovery = 2f;

    /// <summary>
    /// How fast the wake widens with distance, as a half angle. A turbulent
    /// wake spreads into a cone of a few degrees, which is why sitting exactly
    /// behind matters when close and matters much less far back: the hole is
    /// narrow and strong at a car length and broad and weak at fifty metres.
    /// </summary>
    private const float WakeSpreadPerMeter = 0.123f;

    /// <summary>Beyond this there is nothing left worth computing.</summary>
    public const float ReachMeters = 80f;

    private static float GaussianFalloff(float gap, float decayLengthMeters)
    {
        float scale = MathF.Max(decayLengthMeters, 1e-3f);
        float normalized = gap / scale;
        return MathF.Exp(-normalized * normalized);
    }

    /// <summary>
    /// The air <paramref name="gapMeters"/> behind a car leaving
    /// <paramref name="wake"/>, nose to tail, and <paramref name="sidewaysMeters"/>
    /// to one side of its middle, before the follower's own sensitivity
    /// (<see cref="Felt"/>). Clean air touching or beyond the wake's reach.
    /// </summary>
    public static CarAir Behind(CarWake wake, float aheadWidthMeters, float gapMeters, float sidewaysMeters)
    {
        float gap = gapMeters;
        if (gap <= 0f || gap >= ReachMeters)
            return CarAir.Clean;

        // The wake's two faces part company with distance: it rises
        // off the road as it ages, so the follower's body stays sunk
        // in slowed air - the drag relief keeps a long hyperbolic
        // tail - while wings and floor climb out of it within a
        // couple of car lengths, so everything the downforce model
        // reads decays on the short Gaussians.
        float towDeficit = wake.VelocityDeficit /
                           (1f + gap / MathF.Max(
                               wake.TowHalfDistanceMeters,
                               1e-3f
                           ));
        float downforceDeficit = wake.VelocityDeficit *
                                 GaussianFalloff(
                                     gap,
                                     wake.DownforceDecayLengthMeters
                                 );
        float downforceLoss = wake.DownforceDisruption *
                              GaussianFalloff(
                                  gap,
                                  wake.DirtyAirDecayLengthMeters
                              );

        // Across the wake the deficit falls away from the middle, and
        // the middle is wider the further back it is read. Downforce
        // recovers faster sideways than the tow does.
        float halfWidth = aheadWidthMeters * 0.5f +
                          gap * WakeSpreadPerMeter;
        float sideways = MathF.Abs(sidewaysMeters) /
                         MathF.Max(halfWidth, 0.1f);
        towDeficit *= MathF.Exp(-sideways * sideways);
        float lateralRecovery = MathF.Exp(
            -DirtyAirLateralRecovery * sideways * sideways
        );
        downforceDeficit *= lateralRecovery;
        downforceLoss *= lateralRecovery;
        return new CarAir(towDeficit, downforceDeficit, downforceLoss);
    }

    /// <summary>
    /// The air as a car with <paramref name="dirtyAirSensitivity"/> feels it:
    /// the drag relief of a tow is universal; how much of the disturbed air
    /// reaches the working surfaces is the follower's own trait.
    /// </summary>
    public static CarAir Felt(CarAir air, float dirtyAirSensitivity)
    {
        float sensitivity = MathF.Max(0f, dirtyAirSensitivity);
        return new CarAir(
            air.VelocityDeficit,
            MathF.Min(1f, air.DownforceVelocityDeficit * sensitivity),
            MathF.Min(1f, air.DownforceLoss * sensitivity));
    }

    /// <summary>The stronger of two airs, part by part: two cars in line ahead punch one hole in the air, not two.</summary>
    public static CarAir Strongest(CarAir a, CarAir b) => new(
        MathF.Max(a.VelocityDeficit, b.VelocityDeficit),
        MathF.Max(a.DownforceVelocityDeficit, b.DownforceVelocityDeficit),
        MathF.Max(a.DownforceLoss, b.DownforceLoss));
}
