using System.Numerics;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Circuits;

/// <summary>
/// Every circuit the project ships, as it is made: which source, which
/// length, how wide, how high, how banked, where the grid is. Moved here
/// from Core's TrackFactory; the game reads the folders these write.
/// </summary>
internal static class Layouts
{
    private const float GrandPrixTestBufferMeters = 5f;

    /// <summary>
    /// Run-off on a permanent circuit, either side, where the layout leaves
    /// room for it: gravel and asphalt run tens of metres deep at the
    /// corners and a few metres along the straights, and ten metres is the
    /// least of it. Where the circuit passes close to itself it is cut back
    /// to what fits between (Centreline).
    /// </summary>
    private const float PermanentCircuitRunoffMeters = 10f;

    /// <summary>
    /// A street circuit's barriers stand on the kerb line: the pavement is
    /// behind them, not beside the road. Half a metre is the kerb and the
    /// gap to the barrier's face.
    /// </summary>
    private const float StreetBarrierMeters = 0.5f;
    private const float GrandPrixTestTrackWidthMeters = 15f;


    // FIA Arena Grand Prix layout: the source centerline and widths come from
    // the TUM FTM open racetrack database and are scaled to the FIA-published
    // 5.891 km centreline length.
    public static TrackData SilverstoneStyleTestTrack()
    {
        // The source draws Abbey as a kink: some 35 degrees of its 57 in
        // 25 m, a 33 m radius in a corner taken at 285 km/h, where the
        // bacinger/f1-circuits survey turns the same 57 degrees over about
        // 100 m. Free to use the whole road, a line cut it; a lane kept to
        // the inside strip could not, and the racing line bake circled
        // there until it ran out of time. The turn is spread over 50 m
        // either side, which moves the centre by 1.5 m at most and opens
        // the tightest of the corner to a 69 m radius.
        TrackBuilder builder = Centreline.FromClosedCenterline(
            SourceRepairs.SpreadBend(Sources.Silverstone, 320f, 520f, 50f),
            5_891f,
            PermanentCircuitRunoffMeters,
            PermanentCircuitRunoffMeters
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(CrossSections.RoadCircuit);
        // The line sits 98 m along the source centreline rather than at its
        // start, because the file's first row is only where the data begins.
        // Behind that point the Hamilton straight runs 186 m to the exit of
        // Club, which is 23 boxes and no more; a full 30 would need 242 m and
        // put the back of the grid in the corner. Ahead of it, 279 m to Abbey.
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 98,
            GridCount = 23,
            IsFirstGridLeft = true,
            GridOffset = 4.75f
        });
    }

    // FIA Monaco Grand Prix layout. The public GeoJSON centreline is projected
    // to local metres and scaled to the FIA-published 3.337 km length.
    public static TrackData MonacoStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Monaco,
            3_337f,
            StreetBarrierMeters,
            StreetBarrierMeters
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(CrossSections.RoadCircuit);
        // Eight to fifteen metres of road, the narrowest on the calendar, about
        // ten on the grid, which is what the 3.5 m stagger has always been for.
        // The back of the grid does
        // run into Sainte Devote's approach: the straight behind the line is only
        // 198 m and 30 boxes want 242 m. That is Monaco, and it is authored.
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 0,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 3.5f
        });
    }

    // Circuit Zandvoort, the one modern Grand Prix venue that is actually
    // banked. It was rebanked for the 2021 return: Hugenholtz, the slow
    // hairpin, and Arie Luyendijk, the fast final corner, both carry
    // eighteen degrees, which is speedway territory and nothing like the
    // couple of degrees of drainage crossfall the rest of the calendar has.
    // That is why it is here — every other road circuit teaches the car
    // that a corner is flat.
    private const float ZandvoortLengthMeters = 4_259f;
    // The circuit's own figures: Hugenholtz 35 per cent (about 19 degrees),
    // Arie Luyendyk 32 per cent (about 18).
    private const float ZandvoortHugenholtzBankTangent = 0.35f;
    private const float ZandvoortLuyendykBankTangent = 0.32f;

    // Where the two banked corners are, in metres from the start line. The
    // source centreline carries a point every thirty-six metres, which over
    // fourteen turns is too coarse to pick a corner out by its shape alone,
    // so these are placed by the layout's structure instead: Hugenholtz is
    // the tight hairpin after the opening sequence, and Arie Luyendijk is
    // the last corner before the pit straight.
    private const float ZandvoortHugenholtzStart = 860f;
    private const float ZandvoortHugenholtzEnd = 980f;
    private const float ZandvoortLuyendijkStart = 3_853f;
    private const float ZandvoortLuyendijkEnd = 3_948f;

    /// <summary>
    /// The narrow-street training circuit, and the counterpart of the
    /// banked sweeper: that one existed because every banked corner in the
    /// training set was slow, this one exists because every narrow road in
    /// it was gentle. Monaco — ten and a half metres wide, 8.6 percent,
    /// walls at the white line — failed while Shanghai, its equal for width
    /// and corner radius, lapped cleanly; what training never contained was
    /// narrow, climbing and walled at once. Baku is that, from life: the
    /// castle squeeze is the tightest point in the sport at 7.6 metres,
    /// tighter than Monaco, so Monaco's width becomes a question the
    /// policy has seen — while its gradient stays past the training
    /// range, because this circuit's climb tops out under seven percent.
    /// The two-kilometre seafront straight is the longest in the training
    /// set into the bargain, which the top-speed work will not mind.
    /// </summary>
    public static TrackData BakuStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Baku,
            6_003f,
            StreetBarrierMeters,
            StreetBarrierMeters,
            controlSpacingMeters: 12f
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(CrossSections.RoadCircuit);
        // About ten metres of road on the grid: Monaco's stagger.
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 0,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 3.5f
        });
    }

    /// <summary>
    /// Held out for one number: the climb after the compression, authored
    /// past eleven percent against a training maximum around seven. It is
    /// the gradient exam Monaco sets, separated from Monaco's confounders —
    /// this road is wide and the walls are back from the line.
    /// </summary>
    public static TrackData SpaStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Spa,
            7_004f,
            PermanentCircuitRunoffMeters,
            PermanentCircuitRunoffMeters,
            controlSpacingMeters: 12f
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(CrossSections.RoadCircuit);
        return builder.Build(new TrackGridConfig
        {
            // On the climb from the Bus Stop to La Source, which is where
            // the line is; the source line starts on the run to Blanchimont,
            // and a grid there stood its back rows in the bend.
            StartingLineIdx = 1580,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 5f
        });
    }

    /// <summary>
    /// Held out as the road-circuit speed exam: long flat runs broken by
    /// chicanes, which asks for the late hard braking from very high speed
    /// that the pure oval never does.
    /// </summary>
    public static TrackData MonzaStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Monza,
            5_793f,
            PermanentCircuitRunoffMeters,
            PermanentCircuitRunoffMeters,
            controlSpacingMeters: 12f
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(CrossSections.RoadCircuit);
        // Twelve metres of road at the grid: the columns come in to suit it.
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 0,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 4.5f
        });
    }

    /// <summary>
    /// Held out for rhythm: anticlockwise where most of the set turns
    /// right, short, and rolling the whole way round.
    /// </summary>
    public static TrackData InterlagosStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Interlagos,
            4_309f,
            PermanentCircuitRunoffMeters,
            PermanentCircuitRunoffMeters,
            controlSpacingMeters: 12f
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(CrossSections.RoadCircuit);
        // Twelve metres of road at the grid, as at Monza.
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 0,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 4.5f
        });
    }

    /// <summary>
    /// The second narrow-street circuit, flat where Baku climbs. Between
    /// them the street-circuit family separates its own dimensions: Baku
    /// carries narrow-with-gradient, this carries narrow-with-nothing-else,
    /// and Portimão carries gradient-with-width. Monaco, held out, is the
    /// combination of all three.
    /// </summary>
    public static TrackData SingaporeStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Singapore,
            4_928f,
            StreetBarrierMeters,
            StreetBarrierMeters,
            controlSpacingMeters: 12f
        );
        builder.WithSurface(TrackElevation.Profile(
            ElevationTables.SingaporeHeights,
            CrossSections.RoadCircuit
        ));
        // The line moves 136 m along for the same reason as Silverstone's:
        // the source file's first row left only 121 m behind it, and boxes 16
        // and back were lying in a 33 m corner. From here the straight runs
        // 257 m behind the line, which holds all 30. Eleven metres of road, so
        // the columns stand closer together than anywhere but Monaco.
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 136,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 4.0f
        });
    }

    /// <summary>
    /// The gradient trainer. Baku's climb is deliberately capped under the
    /// old training maximum so that Monaco's 8.6 percent stayed an exam;
    /// this circuit retires that exam on purpose, taking the training range
    /// past eleven percent on a road wide enough that gradient is the only
    /// question being asked. Monaco then sits inside the envelope on every
    /// axis and remains unseen only as a layout — which is the kind of
    /// unseen the policy has been passing all along.
    /// </summary>
    public static TrackData PortimaoStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Portimao,
            4_653f,
            PermanentCircuitRunoffMeters,
            PermanentCircuitRunoffMeters,
            controlSpacingMeters: 12f
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(CrossSections.RoadCircuit);
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 0,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 5f
        });
    }

    public static TrackData ZandvoortStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Zandvoort,
            ZandvoortLengthMeters,
            PermanentCircuitRunoffMeters,
            PermanentCircuitRunoffMeters,
            controlSpacingMeters: 12f
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(ZandvoortSurface);
        // About ten metres of road at the grid: Monaco's stagger.
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 0,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 3.5f
        });
    }

    /// <summary>
    /// The road circuit's drainage crossfall everywhere, with the two banked
    /// corners laid over it. The extra bank is added in the direction the
    /// corner already leans and weighted by how committed that lean is, so
    /// it eases in and out with the corner rather than switching on at a
    /// distance.
    /// </summary>
    private static TrackSurface ZandvoortSurface(TrackSurfaceContext context)
    {
        TrackSurface section = CrossSections.RoadCircuit(context);
        float extra = MathF.Max(
            ZandvoortHugenholtzBankTangent * TrackSurfaces.SectionWeight(
                context.DistanceMeters,
                ZandvoortHugenholtzStart,
                ZandvoortHugenholtzEnd,
                35f,
                context.LapLengthMeters
            ),
            ZandvoortLuyendykBankTangent * TrackSurfaces.SectionWeight(
                context.DistanceMeters,
                ZandvoortLuyendijkStart,
                ZandvoortLuyendijkEnd,
                45f,
                context.LapLengthMeters
            )
        );
        if (extra <= 0f)
            return section;

        float lean = TrackSurfaces.CornerLean(
            context.CentrelineCurvature,
            ZandvoortBankReferenceCurvature
        );
        return section with { BankSlope = section.BankSlope + extra * lean };
    }

    private const float ZandvoortBankReferenceCurvature = 0.006f;

    // FIA Shanghai Grand Prix layout, including both snail complexes and the
    // 1.2 km back straight, scaled to the published 5.451 km length.
    public static TrackData ShanghaiStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Shanghai,
            5_451f,
            PermanentCircuitRunoffMeters,
            PermanentCircuitRunoffMeters,
            controlSpacingMeters: 12f
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(CrossSections.RoadCircuit);
        // Eleven and a half metres of road at the grid.
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 0,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 4.25f
        });
    }

    // Sepang Grand Prix layout: a non-overlapping high-speed direction-change
    // benchmark replacing Suzuka, whose grade-separated crossover cannot be
    // represented by the current two-dimensional TrackData topology.
    public static TrackData SepangStyleTestTrack()
    {
        TrackBuilder builder = Centreline.FromClosedCenterline(
            Sources.Sepang,
            5_543f,
            PermanentCircuitRunoffMeters,
            PermanentCircuitRunoffMeters
        );
        // Heights come from the survey (Elevation); the layout gives the cross-section.
        builder.WithSurface(CrossSections.RoadCircuit);
        return builder.Build(new TrackGridConfig
        {
            StartingLineIdx = 0,
            GridCount = 30,
            IsFirstGridLeft = true,
            GridOffset = 5f
        });
    }
}
