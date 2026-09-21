using StintegyEVO.GodotApp.Scenery;
using Xunit;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The scenery plan's file format.
///
/// This tests presentation code from the Core suite, which needs a word:
/// <c>SceneryPlan</c> is the one part of the scenery pipeline with no
/// Godot in it — a file format and the arithmetic of unrolling a row of
/// trees — and this is the solution's only test project. What is guarded
/// is the thing a person edits by hand: that what the editor writes can be
/// read back, and that a row written once arrives as a row.
/// </summary>
public sealed class SceneryPlanTests
{
    private const string Sample = """
        {
          "track": "silverstone",
          "props": [
            { "prop": "grandstand", "s": 120.0, "d": -46.0, "yaw": 3.142, "scale": 1.0 },
            { "prop": "pine", "s": 300.0, "d": 68.0, "scale": 1.15,
              "repeat": { "count": 3, "step_s": 26.0, "step_d": 2.5 } },
            { "prop": "post", "s": 10.0, "d": 5.0, "align": "world", "height": 1.5 }
          ]
        }
        """;

    [Fact]
    public void APlanReadsBackWhatWasWritten()
    {
        SceneryPlan plan = SceneryPlan.Parse(Sample);

        Assert.Equal("silverstone", plan.Track);
        Assert.Equal(3, plan.Props.Count);
        Assert.Equal("grandstand", plan.Props[0].Prop);
        Assert.Equal(-46f, plan.Props[0].D);
        Assert.Equal(1.15f, plan.Props[1].Scale, 3);
        Assert.True(plan.Props[0].AlignToTrack);
        Assert.False(plan.Props[2].AlignToTrack);
        Assert.Equal(1.5f, plan.Props[2].Height);
    }

    [Fact]
    public void ARowIsWrittenOnceAndArrivesAsARow()
    {
        SceneryPlan plan = SceneryPlan.Parse(Sample);
        SceneryPlacement[] placed = [.. plan.Placements()];

        // One grandstand, three pines, one post.
        Assert.Equal(5, placed.Length);
        SceneryPlacement[] pines = [.. placed.Where(p => p.Prop == "pine")];
        Assert.Equal(3, pines.Length);
        Assert.Equal(300f, pines[0].S);
        Assert.Equal(326f, pines[1].S);
        Assert.Equal(352f, pines[2].S);
        Assert.Equal(68f, pines[0].D);
        Assert.Equal(73f, pines[2].D);
        // And an unrolled prop carries no repeat of its own, or the next
        // reader would unroll it again.
        Assert.All(placed, p => Assert.Null(p.Repeat));
    }

    [Fact]
    public void WhatTheEditorWritesIsWhatTheParserReads()
    {
        SceneryPlan plan = SceneryPlan.Parse(Sample);
        SceneryPlan again = SceneryPlan.Parse(plan.ToJson());

        Assert.Equal(plan.Track, again.Track);
        Assert.Equal(plan.Props.Count, again.Props.Count);
        for (int i = 0; i < plan.Props.Count; i++)
        {
            SceneryPlacement before = plan.Props[i];
            SceneryPlacement after = again.Props[i];
            Assert.Equal(before.Prop, after.Prop);
            Assert.Equal(before.S, after.S, 2);
            Assert.Equal(before.D, after.D, 2);
            Assert.Equal(before.Yaw, after.Yaw, 3);
            Assert.Equal(before.Scale, after.Scale, 3);
            Assert.Equal(before.Height, after.Height, 2);
            Assert.Equal(before.AlignToTrack, after.AlignToTrack);
            Assert.Equal(before.Repeat?.Count, after.Repeat?.Count);
            Assert.Equal(before.Repeat?.StepS, after.Repeat?.StepS);
        }
    }

    [Fact]
    public void APlanFromTheFutureLosesOnlyWhatItAddedAndNotTheRest()
    {
        // A key this version has never heard of is ignored rather than
        // fatal: a plan is content, and content outlives a reader.
        SceneryPlan plan = SceneryPlan.Parse("""
            {
              "track": "monaco",
              "lighting": "evening",
              "props": [ { "prop": "pine", "s": 5.0, "d": 3.0, "sway": 0.4 } ]
            }
            """);

        Assert.Equal("monaco", plan.Track);
        SceneryPlacement only = Assert.Single(plan.Props);
        Assert.Equal("pine", only.Prop);
        Assert.Equal(5f, only.S);
    }

    [Fact]
    public void ATrackIsNamedByItsPlanFile()
    {
        Assert.Equal(
            "silverstone",
            SceneryPlan.TrackOf("res://Levels/scenery/silverstone.json")
        );
        Assert.Equal("monaco", SceneryPlan.TrackOf("monaco.json"));
    }
}
