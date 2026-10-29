using Official.FormulaAMixed;
using Official.SingleTrack;
using StintegyEVO.Core.Mods;

namespace Official;

/// <summary>
/// The game's own mod: its chassis and the car built on it. Its circuits and
/// tyres are data, listed in its mod.json.
/// </summary>
public sealed class OfficialMod : IMod
{
    public void Register(ModContext context)
    {
        context.Registry.Chassis.Register("single_track", SingleTrackCar.Declaration);
        context.Registry.Cars.Register("formula_a_mixed", new FormulaAMixedCar());
    }
}
