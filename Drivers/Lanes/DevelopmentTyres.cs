using System;
using StintegyEVO.Core.Cars;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// The tyre the driver is worked on: official:formula_a/super, the medium's
/// grip with no window to leave, no carcass to cook and next to no wear, so
/// a lap that goes wrong went wrong in the driving, not in the tyres.
/// </summary>
public static class DevelopmentTyres
{
    public const string SuperId = "official:formula_a/super";

    private static readonly Lazy<TireConfig> SuperConfig = new(() => TireLibrary.Resolve(SuperId)[0].Front);

    public static TireConfig Super => SuperConfig.Value;
}
