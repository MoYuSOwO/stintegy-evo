using StintegyEVO.Core.Cars;

namespace StintegyEVO.Drivers;

public sealed class SilverstoneRuleController : RuleLineController
{
    public SilverstoneRuleController(CarConfig? car = null, TireConfig? tires = null)
        : base(RuleTrackPack.Silverstone, car, tires)
    {
    }
}

public sealed class SimpleCircuitRuleController : RuleLineController
{
    public SimpleCircuitRuleController(CarConfig? car = null, TireConfig? tires = null)
        : base(RuleTrackPack.Simple, car, tires)
    {
    }
}
