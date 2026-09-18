using StintegyEVO.Core.Cars;

namespace StintegyEVO.TrainingHost.Environment;

/// <summary>
/// The budget re-core (PLAN_zh.md §7, freeze design section 3): the power
/// rungs are target lines for the charge, not caps on the motor.
///
/// Each rung with a line asks for a share of the pack left at the flag, and
/// the line runs straight from full at the start to that share at the
/// finish, by race progress. The driver is charged through a potential,
/// <c>phi = -lambda * max(0, target - remaining)</c>, shaped every step as
/// <c>phi' - phi</c> (see <see cref="DefaultGamma"/>): overspend and the bill arrives at once,
/// recover and it is refunded. Summed over a race that telescopes to the
/// deficit at the flag, which is why the flag is a true terminal
/// (<see cref="TrainingTerminalReason.Finished"/>): shaping alone leaves the
/// optimum unchanged, and it is the settled bill at the finish that makes
/// the line an instruction.
/// </summary>
public static class EnergyBudget
{
    /// <summary>
    /// Race distance, a world constant: 33 laps of Silverstone, the length at
    /// which the current Normal consumption (2.754% of the pack a lap on
    /// world-v3) finishes with 9% left. Other circuits spend energy at other
    /// rates, so Normal finishes elsewhere with more or less; managing that
    /// is the pit wall's job, and the two-dimensional episode start keeps
    /// the whole (progress, charge) plane in the training distribution.
    /// </summary>
    public const float DefaultRaceKilometres = 194.4f;

    /// <summary>
    /// The price of a unit of deficit, in reward per unit of pack fraction.
    ///
    /// Dimensions first. Gate 1 measured the most that energy can buy: at
    /// Silverstone's dearest braking zone, spending a megajoule rather than
    /// lifting saves 3.3 s a lap. At the lap's mean speed of 57.7 m/s and
    /// 0.02 reward a metre, that is 3.8 reward per MJ, or 4190 per unit of
    /// the 1100 MJ pack. Three times the most that overspending can earn,
    /// the same margin the mode-excess penalty was priced at, so that no
    /// megajoule is worth taking from the line: 12,500.
    ///
    /// Watch condition: three times is meant to make the line bite, and it
    /// may bite too hard. If an early from-scratch parent drives Normal the
    /// way Save should be driven -- finishing well above 9% while giving up
    /// lap time it did not need to -- lambda is lowered and reconsidered.
    /// </summary>
    public const float DefaultLambda = 12_500f;

    /// <summary>
    /// The shaping's discount: one, so the step reward is <c>phi' - phi</c>.
    ///
    /// PLAN §7 writes the textbook <c>gamma * phi' - phi</c> with the
    /// learner's gamma. Tried first, it paid a car for standing in deficit:
    /// with phi negative and unchanged, <c>gamma * phi - phi</c> is
    /// <c>(1 - gamma) * |phi|</c> every step, and a randomised start fifteen
    /// points under the line has phi near -1900, so about +13 a step against
    /// progress's +1. Measured on a smoke run: +12,240 of budget reward
    /// against +10.7 of progress over 3,200 lane-steps, critic loss 770. The
    /// optimum is unchanged in theory; the value scale is not, and it
    /// drowns everything else.
    ///
    /// With one, a standing deficit earns and costs nothing, only a change
    /// in it is priced (lambda times the step's overspend, a few hundredths
    /// of reward), and an episode's shaping sums exactly to the change in
    /// deficit across it, settled at the flag. What departs from the
    /// invariant form is a running term <c>(1 - gamma) * phi'</c>, which is
    /// an incentive to get back to the line and is in keeping with what
    /// the line asks.
    /// </summary>
    public const float DefaultGamma = 1f;

    /// <summary>
    /// The Normal line's finish share, which the two-dimensional start and
    /// the nominal start are both drawn about.
    /// </summary>
    public const float NormalFinishShare = 0.09f;

    /// <summary>
    /// What a rung asks to be left at the flag, or null for no line. Push is
    /// not given one (gate 1: overspending buys almost no time on a lap of
    /// one's own; its value is in wheel-to-wheel windows), and Attack
    /// manages nothing, by design.
    /// </summary>
    public static float? FinishShare(int rung) => rung switch
    {
        1 => 0.15f,
        2 => 0.10f,
        3 => NormalFinishShare,
        _ => null
    };

    public static float? Target(float progress, int rung)
    {
        float? finish = FinishShare(rung);
        if (finish is null)
            return null;
        float p = Math.Clamp(progress, 0f, 1f);
        return 1f - (1f - finish.Value) * p;
    }

    /// <summary>Remaining minus target; zero when the rung has no line.</summary>
    public static float Deviation(float progress, float remaining, int rung) =>
        Target(progress, rung) is float target ? remaining - target : 0f;

    public static float Potential(float progress, float remaining, int rung, float lambda) =>
        Target(progress, rung) is float target
            ? -lambda * MathF.Max(0f, target - remaining)
            : 0f;

    /// <summary>
    /// The progress at which a charge sits exactly on the Normal line.
    /// </summary>
    public static float ProgressOnNormalLine(float remaining) =>
        Math.Clamp((1f - remaining) / (1f - NormalFinishShare), 0f, 1f);
}
