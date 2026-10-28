using System;
using System.Numerics;
using StintegyEVO.Core.Mods;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Tests;

/// <summary>
/// The roads tests drive on. The game's circuits come from the registry, as
/// the game takes them. The shapes only a test needs -- the simple layout
/// and its mirror, the banked short track, the banked and flat sweepers and
/// the Daytona-style superspeedway -- are built here, in code, for the tests
/// alone: never registered, never shipped.
/// </summary>
internal static class TestTracks
{
    private const float GrandPrixTestBufferMeters = 5f;

    private static TrackData Circuit(string name) => GameMods.Registry.Circuits.Get("official:" + name).Track;

    public static TrackData SilverstoneStyleTestTrack() => Circuit("silverstone");
    public static TrackData MonacoStyleTestTrack() => Circuit("monaco");
    public static TrackData ZandvoortStyleTestTrack() => Circuit("zandvoort");
    public static TrackData BakuStyleTestTrack() => Circuit("baku");
    public static TrackData SpaStyleTestTrack() => Circuit("spa");
    public static TrackData MonzaStyleTestTrack() => Circuit("monza");
    public static TrackData InterlagosStyleTestTrack() => Circuit("interlagos");
    public static TrackData SingaporeStyleTestTrack() => Circuit("singapore");
    public static TrackData PortimaoStyleTestTrack() => Circuit("portimao");
    public static TrackData ShanghaiStyleTestTrack() => Circuit("shanghai");
    public static TrackData SepangStyleTestTrack() => Circuit("sepang");

    public static TrackData SimpleTestTrack(bool isLeft = false) {
        float baseWidth = 18.0f;
        TrackBuilder builder = new(new(), baseWidth, 5, 5);
        if (isLeft) {
            builder.AddStraight(550)
                    .AddTurn(180, 20.0f, 25.0f)
                    .AddStraight(120, baseWidth)
                    .AddTurn(-45, 30.0f)
                    .AddTurn(45, 30.0f)
                    .AddStraight(5)
                    .AddTurn(45, 30.0f)
                    .AddTurn(-45, 30.0f)
                    .AddStraight(500, 12.0f)
                    .AddTurn(180, 80.0f, 12.0f)
                    .AddStraight(20)
                    .AddTurn(90, 15.0f)
                    .CloseLoop()
            ;
        } else {
            builder.AddStraight(550)
                    .AddTurn(-180, 20.0f, 25.0f)
                    .AddStraight(120, baseWidth)
                    .AddTurn(45, 30.0f)
                    .AddTurn(-45, 30.0f)
                    .AddStraight(5)
                    .AddTurn(-45, 30.0f)
                    .AddTurn(45, 30.0f)
                    .AddStraight(500, 12.0f)
                    .AddTurn(-180, 80.0f, 12.0f)
                    .AddStraight(20)
                    .AddTurn(-90, 15.0f)
                    .CloseLoop()
            ;
        }

        builder.WithSurface(SimpleTestTrackSurface);
        return builder.Build(
            new()
            {
                StartingLineIdx = 300,
                GridCount = 30,
                IsFirstGridLeft = true,
                GridOffset = 5f
            }
        );
    }

    /// <summary>
    /// A banked short track, which is where banking stops being a drainage
    /// detail and becomes the corner. The turns run to about thirty-one
    /// degrees at the wall against eighteen at the apron, so grip grows
    /// with the load the corner itself presses down, and running high buys
    /// bank at the price of distance. The ninety-metre turns are what make
    /// that matter: a gentler speedway's corners are not the limit for
    /// these cars, and banking a corner nobody was slowing for changes
    /// nothing. No Grand Prix circuit in this file offers the trade; this
    /// one exists so the physics that models it has somewhere to be true.
    /// </summary>
    public static TrackData BankedSpeedwayTestTrack()
    {
        TrackBuilder builder = new(
            new Vector2(0f, 0f),
            startWidth: 15f,
            startLeftBuffer: 6f,
            startRightBuffer: 6f
        );
        builder
            .AddStraight(350f)
            .AddTurn(180f, 90f)
            .AddStraight(350f)
            .AddTurn(180f, 90f)
            .CloseLoop()
            .WithSurface(CrossSections.Speedway);
        return builder.Build(
            new TrackGridConfig
            {
                StartingLineIdx = 100,
                GridCount = 20,
                IsFirstGridLeft = true,
                GridOffset = 5f
            }
        );
    }

    // Where each feature of the simple test layout starts, measured from
    // the start/finish line. The layout is written a few lines above; these
    // follow from it, and a test pins them against the curvature that is
    // actually built so they cannot drift apart unnoticed.
    private const float SimpleStartStraightEnd = 550f;
    private const float SimpleTurn1End = 613f;      // 180 degrees at R20
    private const float SimpleBackStraightStart = 832f;
    private const float SimpleBackStraightEnd = 1332f;
    private const float SimpleBigTurnEnd = 1584f;   // 180 degrees at R80

    /// <summary>
    /// Curvature at which the added bank is fully committed: loose enough
    /// that both banked corners here get essentially all of it, tight enough
    /// that a straight gets essentially none.
    /// </summary>
    private const float SimpleExtraBankReferenceCurvature = 0.006f;

    /// <summary>
    /// The simple test layout given some relief: the start/finish straight
    /// climbs hard, the plateau carries through the first-corner hairpin and
    /// the esses, and the back straight gives every metre of it back. The
    /// hairpin and the long left before the final corner are banked well
    /// past what a road circuit carries, so this layout exercises gradient
    /// and bank together — which is the point of a test track, as against
    /// the Grand Prix circuits, where the surface is modelled after the real
    /// places and is correspondingly mild.
    /// </summary>
    private static TrackSurface SimpleTestTrackSurface(
        TrackSurfaceContext context
    )
    {
        const float summit = 30f;
        TrackSurface section = TrackElevation.ProfileByDistance(
            [
                (0f, 0f),
                (SimpleStartStraightEnd, summit),
                (SimpleBackStraightStart, summit),
                (SimpleBackStraightEnd, 0f)
            ],
            CrossSections.RoadCircuit
        )(context);

        float extra = SimpleExtraBank(
            context.DistanceMeters,
            context.LapLengthMeters
        );
        if (extra <= 0f)
            return section;

        // Added in the direction the corner leans, so the bank helps the turn
        // whichever way it goes -- but weighted by how committed that lean
        // is, not merely by its sign. Taking the sign alone put the whole
        // seventeen degrees on whichever side of zero the curvature happened
        // to be, and flipped all of it in a single metre where the hairpin
        // handed over to the esses.
        float lean = TrackSurfaces.CornerLean(
            context.CentrelineCurvature,
            SimpleExtraBankReferenceCurvature
        );
        return section with
        {
            BankSlope = section.BankSlope + extra * lean
        };
    }

    /// <summary>
    /// How much bank the two chosen corners get beyond the drainage
    /// crossfall, easing in and out over their entries and exits so the
    /// road never steps.
    /// </summary>
    private static float SimpleExtraBank(float distance, float lapLength)
    {
        const float hairpinBank = 0.30f;   // about seventeen degrees
        const float sweeperBank = 0.22f;   // about twelve degrees
        return MathF.Max(
            hairpinBank * TrackSurfaces.SectionWeight(
                distance, SimpleStartStraightEnd, SimpleTurn1End, 25f, lapLength
            ),
            sweeperBank * TrackSurfaces.SectionWeight(
                distance, SimpleBackStraightEnd, SimpleBigTurnEnd, 45f, lapLength
            )
        );
    }


    /// <summary>
    /// A training circuit built for one hole in the coverage, found by
    /// measuring rather than by guessing.
    ///
    /// Every banked corner the policy had ever driven was a hairpin. Twenty
    /// degrees at Zandvoort sits on a twenty-five metre radius and nineteen
    /// on the simple layouts sits on twenty, and both are taken at walking
    /// pace by racing standards. Meanwhile every fast corner on every road
    /// circuit here carries the same two and a half degrees of drainage
    /// crossfall. So the joint distribution had banking and it had speed and
    /// it never had the two together — and when Daytona asked for thirty-one
    /// degrees at four hundred and fifty metres of radius, the policy spent a
    /// sixth of the lap off the road.
    ///
    /// The corners here run from sixty metres of radius to four hundred, and
    /// the banking rises with the radius rather than sitting on the slowest
    /// corner: nothing at a hairpin, the full eighteen degrees by two hundred
    /// and fifty metres. Eighteen is deliberately inside what has already
    /// been trained on. The point is not to show the policy a steeper road,
    /// it is to show it a fast one that leans — so that Daytona is left
    /// extrapolating in the bank angle alone rather than in the whole
    /// combination.
    ///
    /// Corners turn both ways for the same reason the simple layout comes in
    /// a mirrored pair: a circuit that only ever leans one way teaches the
    /// sign along with the lesson.
    /// </summary>
    public static TrackData BankedSweeperTestTrack()
    {
        // One copy of the geometry. The builder walks it and the banking is
        // derived from the same walk, so a corner cannot be banked at an arc
        // length it does not occupy — which is what happens when the layout
        // is written once for the shape and again for the surface.
        (float Straight, float TurnDegrees, float Radius)[] layout =
            SweeperLayout;

        var corners = new (float Start, float End, float Bank)[layout.Length];
        float distance = 0f;
        for (int i = 0; i < layout.Length; i++)
        {
            distance += layout[i].Straight;
            float arc = layout[i].Radius *
                        MathF.Abs(layout[i].TurnDegrees) * MathF.PI / 180f;
            corners[i] = (
                distance,
                distance + arc,
                BankFor(layout[i].Radius) *
                MathF.CopySign(1f, layout[i].TurnDegrees)
            );
            distance += arc;
        }

        TrackBuilder builder = new(
            Vector2.Zero,
            startWidth: 14f,
            startLeftBuffer: GrandPrixTestBufferMeters,
            startRightBuffer: GrandPrixTestBufferMeters
        );
        builder.WithSurface(context =>
            BankedSweeperSurface(context, corners)
        );
        foreach ((float straight, float turn, float radius) in layout)
            builder.AddStraight(straight).AddTurn(turn, radius);
        return builder.CloseLoop().Build(new TrackGridConfig());
    }

    /// <summary>
    /// The banked sweeper's layout with the banking left off — nothing but
    /// the road circuit's drainage crossfall over the same sixty-to-four-
    /// hundred-metre corners.
    ///
    /// It exists because the flat oval collapsed while everything else
    /// improved: from eight seconds off the analytic lap to grinding the
    /// wall, one evaluation at a time, as training attention moved to
    /// banked sweepers and walled streets. A fast corner that is merely
    /// flat had nothing anchoring it — the sweeper's big corners all lean,
    /// the street circuits' are all tight — and the skill quietly drained
    /// away. That hole touches shipped content, not just the synthetic
    /// oval: Monza's long flat sweeps are the same family, and Monza
    /// oscillated in step with the oval's slide.
    ///
    /// Sharing the layout is deliberate: paired with the banked original,
    /// the two circuits differ in exactly one variable, which is what made
    /// the flat/banked Daytona comparison worth having.
    /// </summary>
    public static TrackData FlatSweeperTestTrack()
    {
        TrackBuilder builder = new(
            Vector2.Zero,
            startWidth: 14f,
            startLeftBuffer: GrandPrixTestBufferMeters,
            startRightBuffer: GrandPrixTestBufferMeters
        );
        builder.WithSurface(CrossSections.RoadCircuit);
        foreach ((float straight, float turn, float radius) in SweeperLayout)
            builder.AddStraight(straight).AddTurn(turn, radius);
        return builder.CloseLoop().Build(new TrackGridConfig());
    }

    /// <summary>
    /// One copy of the sweeper's geometry, shared by the banked original
    /// and the flat variant so the pair can never drift apart.
    /// </summary>
    private static readonly (float Straight, float TurnDegrees, float Radius)[]
        SweeperLayout =
    [
        (700f, 120f, 300f),
        (200f, -60f, 140f),
        (150f, 150f, 200f),
        (300f, -70f, 90f),
        (200f, 140f, 400f),
        (250f, 80f, 60f),
    ];

    private const float BankedSweeperMaxBankTangent = 0.325f;  // eighteen degrees
    private const float BankedSweeperFlatRadiusMeters = 60f;
    private const float BankedSweeperFullRadiusMeters = 250f;
    private const float BankedSweeperTransitionMeters = 60f;

    /// <summary>
    /// How much a corner of this radius leans. A hairpin gets nothing,
    /// because hairpin banking is the one thing the training set already has
    /// too much of; anything from two hundred and fifty metres upward gets
    /// the full eighteen degrees. Smoothstepped so the middle radii do not
    /// sit on a corner of the ramp.
    /// </summary>
    private static float BankFor(float radius)
    {
        float ramp = Math.Clamp(
            (radius - BankedSweeperFlatRadiusMeters) /
            (BankedSweeperFullRadiusMeters - BankedSweeperFlatRadiusMeters),
            0f,
            1f
        );
        return BankedSweeperMaxBankTangent * ramp * ramp * (3f - 2f * ramp);
    }

    /// <summary>
    /// The road circuit's crossfall everywhere, with each corner's banking
    /// wound on over sixty metres at either end of the arc it occupies.
    ///
    /// By arc length rather than by curvature, for the reason Daytona is:
    /// curvature steps at a segment boundary, so banking read off it arrives
    /// as most of a degree per metre and the car meets a banked corner's
    /// lateral gravity inside one wheel rotation.
    /// </summary>
    private static TrackSurface BankedSweeperSurface(
        TrackSurfaceContext context,
        (float Start, float End, float Bank)[] corners
    )
    {
        TrackSurface section = CrossSections.RoadCircuit(context);
        float extra = 0f;
        foreach ((float start, float end, float bank) in corners)
        {
            extra += bank * TrackSurfaces.SectionWeight(
                context.DistanceMeters,
                start,
                end,
                BankedSweeperTransitionMeters,
                context.LapLengthMeters
            );
        }
        return extra == 0f
            ? section
            : section with { BankSlope = section.BankSlope + extra };
    }

    /// <summary>
    /// A superspeedway on the Daytona pattern: two long straights joined by
    /// two constant-radius turns banked at thirty-one degrees.
    ///
    /// It exists to test one thing the rest of the track set cannot. The
    /// steepest banking anywhere else here is Zandvoort's Hugenholtz at
    /// twenty degrees, and it is in the training set, so a policy held out
    /// on Sepang, Monaco and a flat oval is never asked about banking it has
    /// not already seen. Thirty-one degrees is half as much again as
    /// anything trained on, and the difference is not decorative: the
    /// lateral the tyres are asked for falls by a seventh at that angle
    /// while the load on them rises, which is the whole reason a
    /// superspeedway corner is taken flat out.
    ///
    /// The geometry is Daytona's to about two parts in a thousand — four
    /// kilometres and change, turns of a thousand feet — and the banking is
    /// its published thirty-one degrees in the turns and three on the
    /// straights. The width is assigned rather than sourced: eighteen
    /// metres, at the generous end of what a superspeedway runs, chosen so
    /// that a policy failing here is failing at the banking and not at the
    /// room, the way Monaco's ten and a half metres confounds its own test.
    /// </summary>
    public static TrackData DaytonaStyleTestTrack()
    {
        TrackBuilder builder = new(
            Vector2.Zero,
            startWidth: DaytonaWidthMeters,
            startLeftBuffer: GrandPrixTestBufferMeters,
            startRightBuffer: GrandPrixTestBufferMeters
        );
        builder.WithSurface(DaytonaSurface);
        return builder
            .AddStraight(DaytonaStraightMeters)
            .AddTurn(180f, DaytonaTurnRadiusMeters)
            .AddStraight(DaytonaStraightMeters)
            .AddTurn(180f, DaytonaTurnRadiusMeters)
            .CloseLoop()
            .Build(new TrackGridConfig());
    }

    private const float DaytonaWidthMeters = 18f;
    private const float DaytonaStraightMeters = 1_050f;
    private const float DaytonaTurnRadiusMeters = 305f;      // a thousand feet
    private const float DaytonaTurnBankTangent = 0.6009f;    // thirty-one degrees
    private const float DaytonaStraightBankTangent = 0.0524f; // three degrees

    /// <summary>
    /// Thirty-one degrees through the turns, three down the straights, with
    /// a hundred metres of transition at each end of each turn.
    ///
    /// The transition is the point of doing this by arc length. Reading the
    /// banking off the curvature is tempting on an oval — the turns are
    /// exactly where the road bends — but the hyperbolic tangent saturates
    /// within a fraction of the corner's curvature, so the bank arrives as a
    /// step of a degree per metre and the car meets the lateral gravity of a
    /// thirty-one degree corner in the space of one wheel rotation. A real
    /// superspeedway spends a hundred metres or so winding the banking on,
    /// and so does this.
    ///
    /// The turn boundaries are computed from the layout above rather than
    /// written down, so there is no second copy of the geometry to get out
    /// of step with the first.
    ///
    /// A superspeedway leans one way, toward the infield, rather than
    /// crowning to shed water off both edges the way a road circuit does, so
    /// this does not build on <see cref="CrossSections.RoadCircuit"/>. Both
    /// turns bend the same way, so the lean never changes sign.
    /// </summary>
    private static TrackSurface DaytonaSurface(TrackSurfaceContext context)
    {
        float turnLength = MathF.PI * DaytonaTurnRadiusMeters;
        float firstTurnStart = DaytonaStraightMeters;
        float firstTurnEnd = firstTurnStart + turnLength;
        float secondTurnStart = firstTurnEnd + DaytonaStraightMeters;
        float secondTurnEnd = secondTurnStart + turnLength;

        float inTurn = MathF.Max(
            TrackSurfaces.SectionWeight(
                context.DistanceMeters,
                firstTurnStart,
                firstTurnEnd,
                DaytonaTransitionMeters,
                context.LapLengthMeters
            ),
            TrackSurfaces.SectionWeight(
                context.DistanceMeters,
                secondTurnStart,
                secondTurnEnd,
                DaytonaTransitionMeters,
                context.LapLengthMeters
            )
        );
        return new TrackSurface(
            BankSlope: DaytonaStraightBankTangent +
                       (DaytonaTurnBankTangent - DaytonaStraightBankTangent) *
                       inTurn
        );
    }

    private const float DaytonaTransitionMeters = 100f;
}
