using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;

namespace StintegyEVO.TrainingHost.Environment;

/// <summary>
/// Wraps a driver so it decides at a fixed rate and holds its input in
/// between, instead of replanning every physics step.
///
/// This is how the analytic driver earns its place as a sparring partner.
/// Sony found that an agent trained only against copies of itself was
/// ill-prepared for opponents that brake a fraction of a second earlier
/// than it would, and that a mixed population — past agents together with
/// the game's relatively slower built-in AI — was what worked. The
/// reference driver is our slower AI, and running it at a tenth of the
/// physics rate is the same knob turned twice: on Shanghai it takes a duel
/// from 2,024 steps a second to 5,568, and it makes the opponent ten
/// percent slower and coarser in its inputs, which is the half of the
/// bargain we actually wanted.
/// </summary>
internal sealed class HeldDecisionDriver(IRaceDriver inner, float decisionHz)
    : IRaceDriver
{
    private readonly float _period = 1f / decisionHz;
    private float _secondsSinceDecision = float.MaxValue;
    private DriverInput _held;

    public float TireEnergyEfficiency => inner.TireEnergyEfficiency;

    public void Initialize(in RaceDriverInitContext context)
    {
        _secondsSinceDecision = float.MaxValue;
        _held = default;
        inner.Initialize(in context);
    }

    public DriverInput GetControl(in RaceDriverFrameContext context, float dt)
    {
        _secondsSinceDecision += dt;
        if (_secondsSinceDecision < _period)
            return _held;

        _secondsSinceDecision = 0f;
        _held = inner.GetControl(in context, dt);
        return _held;
    }
}
