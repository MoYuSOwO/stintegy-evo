using System;
using StintegyEVO.Core.Cars;

namespace Official.SingleTrack;

/// <summary>
/// The push-to-pass allowance, as the series writes it: a budget of seconds
/// for the race, a cap on any one push, and a wait after each before the
/// next. What a push does to the car is the powertrain's and the aero's
/// business; this is only when it may run.
/// </summary>
public sealed record PushToPassRules
{
    /// <summary>Seconds of push for the whole race.</summary>
    public float TotalSeconds { get; init; } = 200f;

    /// <summary>The longest one push may run before it switches itself off.</summary>
    public float MaxActivationSeconds { get; init; } = 15f;

    /// <summary>How long after a push ends, however it ended, before the next may start.</summary>
    public float CooldownSeconds { get; init; } = 30f;
}

/// <summary>
/// Where a car's push-to-pass stands. A value, so the planner's clones of a
/// car carry it without allocating.
///
/// A push runs while the button is held and nothing stops it: not locked,
/// not cooling down, seconds left, and under the cap. Releasing ends it;
/// so does the cap or the budget running out, and then the button has to
/// come up before it counts again, so holding it does not chain one capped
/// push into the next. Every end starts the cooldown, a two-second push as
/// much as a full one.
/// </summary>
public struct PushToPassState
{
    public float UsedSeconds;
    public float ActivationSeconds;
    public float CooldownSeconds;
    public bool Active;

    /// <summary>
    /// Held off by race control: at the start and at every restart, until
    /// the car crosses the line. The race sets it; the line clears it.
    /// </summary>
    public bool Locked;

    /// <summary>A push the cap or the budget ended, still held: it needs letting go first.</summary>
    public bool AwaitingRelease;

    public readonly float RemainingSeconds(PushToPassRules rules) =>
        MathF.Max(0f, rules.TotalSeconds - UsedSeconds);

    /// <summary>One step of the button and the clock.</summary>
    public static PushToPassState Advance(PushToPassState state, PushToPassRules rules, bool requested, float dt)
    {
        if (!requested)
        {
            state.AwaitingRelease = false;
            if (state.Active)
                state = End(state, rules);
        }

        if (!state.Active)
        {
            state.CooldownSeconds = MathF.Max(0f, state.CooldownSeconds - dt);
            if (requested && !state.AwaitingRelease && !state.Locked &&
                state.CooldownSeconds <= 0f && state.RemainingSeconds(rules) > 0f)
            {
                state.Active = true;
                state.ActivationSeconds = 0f;
            }
        }

        if (state.Active && state.Locked)
            state = End(state, rules);

        if (state.Active)
        {
            float run = MathF.Min(dt, state.RemainingSeconds(rules));
            state.UsedSeconds += run;
            state.ActivationSeconds += run;
            if (state.ActivationSeconds >= rules.MaxActivationSeconds - 1e-4f ||
                state.RemainingSeconds(rules) <= 0f)
            {
                state = End(state, rules);
                state.AwaitingRelease = true;
            }
        }
        return state;
    }

    private static PushToPassState End(PushToPassState state, PushToPassRules rules)
    {
        state.Active = false;
        state.ActivationSeconds = 0f;
        state.CooldownSeconds = rules.CooldownSeconds;
        return state;
    }
}
