using StintegyEVO.Drivers.Racecraft;
using Xunit;

namespace StintegyEVO.Drivers.Tests;

/// <summary>
/// A racecraft duel driven on the racing line, the way a layer that has
/// learned nothing yet would be left to: it runs to its end, its rows stay
/// numbers, and the car behind sits behind the slower one without hitting it.
/// </summary>
public sealed class RacecraftDuelTests
{
    [Fact]
    public void OnTheRacingLineADuelRunsItsTimeBehindTheCarAheadWithoutTouchingIt()
    {
        var duel = new RacecraftDuel("silverstone", GameMods.Registry.Circuits.Get("silverstone").Track);
        var row = new float[RacecraftDuel.ObservationSize];
        var action = new float[RacecraftDuel.ActionSize];
        duel.Reset(new Random(3), row);
        RacecraftAction.RacingLine(action);
        float contact = 0f, passes = 0f;
        int steps = 0;
        RacecraftStep step;
        do
        {
            step = duel.Step(action, row);
            Assert.All(row, x => Assert.True(float.IsFinite(x)));
            contact += step.Contact;
            passes += step.Passes;
            steps++;
        } while (!step.Ended);
        Assert.True(step.OutOfTime, "the duel ended before its time");
        Assert.Equal((int)(RacecraftDuel.EpisodeSeconds / RacecraftDuel.StepSeconds), steps);
        Assert.Equal(0f, contact);
        Assert.Equal(0f, passes);
    }
}
