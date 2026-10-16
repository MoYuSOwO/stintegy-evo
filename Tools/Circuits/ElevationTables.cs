namespace StintegyEVO.Circuits;

/// <summary>
/// Height profiles, as (fraction of a lap, metres). Approximations of each
/// circuit's documented character, not survey data.
/// </summary>
internal static class ElevationTables
{
    /// <summary>
    /// Zandvoort is built through coastal dunes and rides over them. The
    /// lap climbs away from Tarzan into the Hunserug, crests before
    /// Scheivlak and drops through it — the corner is famous for arriving
    /// blind over the top — runs low along the back of the circuit, and
    /// climbs again through the last sequence to the banked final corner.
    /// Something over fifteen metres between the highest and lowest points,
    /// which is a great deal for a circuit this short.
    /// </summary>
    /// <summary>
    /// Baku is a seafront street circuit and mostly flat; the exception is
    /// the climb along the old city wall into the castle section, which is
    /// also where the road narrows to the tightest squeeze in the sport.
    /// The grade tops out under seven percent on purpose: this circuit is
    /// bound for the training set, and the training set's steepest climb
    /// must stay below Monaco's 8.6 so that Monaco remains a gradient the
    /// policy has never been asked for.
    /// </summary>
    public static readonly (float, float)[] BakuHeights =
    [
        (0.00f, 0f), (0.20f, 0f), (0.30f, 2f), (0.34f, 6f),
        (0.39f, 24f), (0.44f, 26f), (0.52f, 14f), (0.62f, 4f),
        (0.75f, 1f), (0.90f, 0f)
    ];

    /// <summary>
    /// Authored in the style of the Ardennes original: a long descent to
    /// the valley floor, the climb back along the top, and then the
    /// signature — a fast drop followed by a compression and a climb far
    /// steeper than anything in the training set. Held out precisely for
    /// that climb: eleven-plus percent against a training maximum around
    /// seven.
    /// </summary>
    public static readonly (float, float)[] SpaHeights =
    [
        (0.00f, 55f), (0.08f, 47f), (0.16f, 29f), (0.26f, 12f),
        (0.36f, 4f), (0.44f, 0f), (0.52f, 7f), (0.62f, 20f),
        (0.72f, 34f), (0.82f, 50f), (0.87f, 40f), (0.90f, 27f),
        (0.92f, 25f), (0.955f, 48f)
    ];

    /// <summary>Monza is a park and the park is flat.</summary>
    public static readonly (float, float)[] MonzaHeights =
    [
        (0.00f, 0f), (0.25f, 2f), (0.55f, -1f), (0.80f, 1f)
    ];

    /// <summary>
    /// Rolling and anticlockwise, cut into a hillside: down from the
    /// start, a low back stretch, and the long climb home.
    /// </summary>
    public static readonly (float, float)[] InterlagosHeights =
    [
        (0.00f, 12f), (0.10f, 4f), (0.22f, 0f), (0.36f, 6f),
        (0.50f, 15f), (0.64f, 23f), (0.78f, 30f), (0.90f, 24f)
    ];

    /// <summary>
    /// Marina Bay is famously flat — the only relief is the pair of bridge
    /// crossings, a few metres each. Authored honestly: this circuit is in
    /// the set for its walls and its width, and pretending it climbs would
    /// cover a hole with fiction.
    /// </summary>
    public static readonly (float, float)[] SingaporeHeights =
    [
        (0.00f, 0f), (0.18f, 1f), (0.30f, 4f), (0.36f, 1f),
        (0.55f, 0f), (0.72f, 3f), (0.80f, 0f)
    ];

    /// <summary>
    /// The Algarve rollercoaster. Authored to what the venue is famous
    /// for: blind crests and plunges, with the drop into turn one and the
    /// climb to the final corner both past ten percent. This is the
    /// gradient trainer — the one dimension of Monaco that Baku's capped
    /// climb deliberately leaves out — carried on a road wide enough that
    /// nothing else is being tested at the same time.
    /// </summary>
    public static readonly (float, float)[] PortimaoHeights =
    [
        (0.000f, 32f), (0.035f, 13f), (0.090f, 6f), (0.140f, 14f),
        (0.190f, 3f), (0.270f, 11f), (0.330f, 2f), (0.420f, 9f),
        (0.500f, 0f), (0.600f, 8f), (0.660f, 2f), (0.760f, 13f),
        (0.820f, 6f), (0.880f, 9f), (0.930f, 26f)
    ];

    public static readonly (float, float)[] ZandvoortHeights =
    [
        (0.00f, 4f), (0.10f, 2f), (0.22f, 11f), (0.32f, 16f),
        (0.40f, 6f), (0.52f, 1f), (0.64f, 0f), (0.76f, 5f),
        (0.86f, 12f), (0.94f, 9f)
    ];

    /// <summary>
    /// Monaco climbs harder than anywhere else on the calendar: out of
    /// Sainte Dévote up Beau Rivage to Casino, then down through Mirabeau
    /// and the hairpin to Portier and the tunnel at sea level, flat along
    /// the harbour, and back up through Anthony Noghes to the line. About
    /// forty metres between the highest and lowest points.
    /// </summary>
    public static readonly (float, float)[] MonacoHeights =
    [
        (0.00f, 8f), (0.05f, 4f), (0.10f, 14f), (0.17f, 32f),
        (0.24f, 38f), (0.31f, 30f), (0.37f, 16f), (0.43f, 4f),
        (0.50f, 0f), (0.62f, 0f), (0.72f, 1f), (0.82f, 2f),
        (0.90f, 4f), (0.96f, 7f)
    ];

    /// <summary>
    /// Shanghai's opening complex climbs into the first corner and unwinds
    /// downhill through the snail; the rest of the lap, including the long
    /// back straight, is close to level. Around ten metres in all.
    /// </summary>
    public static readonly (float, float)[] ShanghaiHeights =
    [
        (0.00f, 0f), (0.04f, 6f), (0.08f, 7f), (0.13f, 2f),
        (0.22f, 0f), (0.38f, -1f), (0.52f, -2f), (0.66f, -1f),
        (0.80f, 0f), (0.92f, 0f)
    ];

    /// <summary>
    /// Silverstone is a converted airfield and the flattest circuit here.
    /// What little there is falls away through the Village loop and rises
    /// again toward Stowe, inside about eight metres.
    /// </summary>
    public static readonly (float, float)[] SilverstoneHeights =
    [
        (0.00f, 3f), (0.12f, 1f), (0.24f, -2f), (0.38f, -3f),
        (0.52f, 0f), (0.66f, 2f), (0.78f, 4f), (0.90f, 4f)
    ];

    /// <summary>
    /// Sepang rises through the first sector and falls away again to the
    /// second long straight, with something under twenty metres between the
    /// extremes.
    /// </summary>
    public static readonly (float, float)[] SepangHeights =
    [
        (0.00f, 0f), (0.08f, 5f), (0.18f, 12f), (0.28f, 15f),
        (0.40f, 9f), (0.52f, 2f), (0.64f, -2f), (0.76f, -1f),
        (0.88f, 0f)
    ];
}
