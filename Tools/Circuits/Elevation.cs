using System.Globalization;
using System.Numerics;
using System.Text;
using System.Text.Json;

namespace StintegyEVO.Circuits;

/// <summary>
/// Heights from public survey data instead of hand-written profiles.
///
/// Each real circuit's line as bacinger/f1-circuits draws it (longitude and
/// latitude) is sampled every 20 m and each sample's height read from the
/// SRTM 30 m elevation model through the public Open Topo Data service.
/// <c>fetch-elevation</c> does that once and keeps the result under
/// Sources/elevation; writing a circuit only reads it, so no build needs
/// the network.
///
/// The bacinger line is not our centreline (it starts elsewhere, may run
/// the other way, and is a little longer or shorter), so it is lined up
/// with ours by the one thing both share: where they turn. The heights are
/// then smoothed, because SRTM is a surface model that carries trees and
/// buildings as well as the ground and is good to a few metres, not to the
/// last one.
/// </summary>
internal static class Elevation
{
    public static readonly Dictionary<string, string> Codes = new()
    {
        ["silverstone"] = "gb-1948",
        ["shanghai"] = "cn-2004",
        ["sepang"] = "my-1999",
        ["monaco"] = "mc-1929",
        ["zandvoort"] = "nl-1948",
        ["baku"] = "az-2016",
        ["spa"] = "be-1925",
        ["monza"] = "it-1922",
        ["interlagos"] = "br-1940",
        ["singapore"] = "sg-2008",
        ["portimao"] = "pt-2008",
    };

    /// <summary>
    /// Highest to lowest, where the series publishes it (formula1.com,
    /// "Highs and lows"). The survey's shape is kept and its total scaled to
    /// this, since a surface model reads a few metres high or low where
    /// there are trees, walls and water beside the road.
    /// </summary>
    public static readonly Dictionary<string, float> PublishedClimbMeters = new()
    {
        ["spa"] = 102.2f,
        ["interlagos"] = 43f,
        ["monaco"] = 42f,
    };

    /// <summary>
    /// The steepest stretch of road, where it is published: Raidillon's 17
    /// per cent (lapmeta), the Subida dos Boxes' 10 (circuit guides).
    /// Smoothing a 30 m survey over 200 m takes the top off a short, steep
    /// climb -- Raidillon came out at 13.6 -- so the steepest stretch of the
    /// survey has its grade drawn in towards its middle until the middle is
    /// this steep, with the heights at either end of it, and so everywhere
    /// else, left where they were.
    ///
    /// Portimão is not here. Its "up to 12 per cent" is a ticket page's,
    /// names no stretch, and drawn onto the survey's steepest descent it
    /// rounds the crest above it to 460 m, which a car crossing it at 300
    /// km/h would leave the ground on.
    /// </summary>
    public static readonly Dictionary<string, float> PublishedSteepestGrade = new()
    {
        ["spa"] = 0.17f,
        ["interlagos"] = 0.10f,
    };

    /// <summary>
    /// Circuits whose survey is not the road. Marina Bay runs between tower
    /// blocks, and SRTM, which sees the roofs, makes a 5 m circuit climb 28;
    /// its authored profile is kept.
    /// </summary>
    public static readonly HashSet<string> Unusable = ["singapore"];

    private const float SampleMeters = 20f;
    private const string Dataset = "srtm30m";
    private const double EarthRadius = 6_371_000.0;

    /// <summary>Samples either side the median is taken over: 100 m in all.</summary>
    private const int MedianHalfWidth = 2;

    /// <summary>The moving average over the line: about 200 m.</summary>
    private const int AverageHalfWidth = 5;

    /// <summary>The last moving average, over the metre rows, that rounds off the interpolation.</summary>
    private const int RowAverageHalfWidth = 20;

    public static string Folder => Path.Combine(Sources.Folder, "elevation");

    public static string Notes(string id) =>
        $"Heights: SRTM 30 m (public domain) through Open Topo Data, sampled every {SampleMeters:F0} m along " +
        $"{Codes[id]} from bacinger/f1-circuits, lined up by curvature and smoothed over about 200 m" +
        (PublishedClimbMeters.TryGetValue(id, out float m) ? $", scaled to the published {m:F1} m highest to lowest" : "") +
        (PublishedSteepestGrade.TryGetValue(id, out float g) ? $", its steepest stretch drawn to the published {g * 100f:F0}%." : ".");

    public static async Task Fetch(IEnumerable<string> ids)
    {
        Directory.CreateDirectory(Folder);
        using var http = new HttpClient();
        http.DefaultRequestHeaders.UserAgent.ParseAdd("StintegyEVO-circuits/1");
        foreach (string id in ids)
        {
            string code = Codes[id];
            string geojson = await http.GetStringAsync(
                $"https://raw.githubusercontent.com/bacinger/f1-circuits/master/circuits/{code}.geojson");
            List<(double Lat, double Lon)> ring = Ring(geojson);
            List<(double Lat, double Lon, double S)> samples = Resample(ring, SampleMeters);

            var heights = new double[samples.Count];
            for (int start = 0; start < samples.Count; start += 100)
            {
                int count = Math.Min(100, samples.Count - start);
                string locations = string.Join('|', Enumerable.Range(start, count).Select(i =>
                    string.Create(CultureInfo.InvariantCulture, $"{samples[i].Lat:F6},{samples[i].Lon:F6}")));
                string reply = await http.GetStringAsync($"https://api.opentopodata.org/v1/{Dataset}?locations={locations}");
                using JsonDocument doc = JsonDocument.Parse(reply);
                int k = 0;
                foreach (JsonElement result in doc.RootElement.GetProperty("results").EnumerateArray())
                {
                    JsonElement e = result.GetProperty("elevation");
                    heights[start + k++] = e.ValueKind == JsonValueKind.Number ? e.GetDouble() : double.NaN;
                }
                await Task.Delay(1100); // the public service allows one call a second
            }

            var text = new StringBuilder();
            text.Append($"# {id}: {code} from bacinger/f1-circuits, every {SampleMeters:F0} m; {Dataset} via api.opentopodata.org\n");
            text.Append("s_m,lat,lon,elevation_m\n");
            for (int i = 0; i < samples.Count; i++)
                text.Append(string.Create(CultureInfo.InvariantCulture,
                    $"{samples[i].S:F1},{samples[i].Lat:F6},{samples[i].Lon:F6},{heights[i]:F1}\n"));
            File.WriteAllText(Path.Combine(Folder, id + ".csv"), text.ToString());
            Console.WriteLine($"{id,-12} {code}  {samples.Count} samples  {heights.Min():F0}..{heights.Max():F0} m");
        }
    }

    /// <summary>The height of every row of this centreline, or null when the circuit has no survey.</summary>
    public static float[]? ForTrack(string id, IReadOnlyList<Vector2> centre, out string report)
    {
        report = "";
        string path = Path.Combine(Folder, id + ".csv");
        if (!File.Exists(path) || Unusable.Contains(id))
            return null;
        (List<Vector2> points, List<double> raw, _, _) = ReadSurvey(path);
        double[] heights = Smooth(FillGaps(raw.ToArray()));

        // Line the survey's line up with ours by where they turn.
        (int shift, bool reversed, float score) = Align(centre, points);

        int n = centre.Count;
        int m = heights.Length;
        var z = new float[n];
        for (int i = 0; i < n; i++)
        {
            double f = (double)i / n;
            double g = reversed ? (shift / (double)AlignBins - f) : (f + shift / (double)AlignBins);
            g -= Math.Floor(g);
            z[i] = (float)CatmullRom(heights, g * m);
        }
        z = RowAverage(z, RowAverageHalfWidth);
        float surveyed = z.Max() - z.Min();
        string scaled = "";
        if (PublishedClimbMeters.TryGetValue(id, out float published))
        {
            float low = z.Min();
            float k = published / MathF.Max(surveyed, 1e-3f);
            for (int i = 0; i < n; i++)
                z[i] = low + (z[i] - low) * k;
            scaled = $", scaled from {surveyed:F1} m to the published {published:F1} m";
        }

        if (PublishedSteepestGrade.TryGetValue(id, out float steepest))
        {
            float before = SteepestGrade(z);
            if (Steepen(z, steepest))
                scaled += $", steepest drawn from {before * 100f:F1}% to the published {steepest * 100f:F0}%";
        }

        float maxGrade = SteepestGrade(z);
        report = $"survey {raw.Min():F0}..{raw.Max():F0} m, aligned {(reversed ? "reversed" : "forward")} at {shift * 100f / AlignBins:F1}% (match {score:F2}), " +
                 $"climb {z.Max() - z.Min():F1} m{scaled}, steepest {maxGrade * 100f:F1}%";
        return z;
    }

    /// <summary>
    /// The real circuit's line, from bacinger/f1-circuits as the survey
    /// sampled it, on the game's plane about its own middle; and where on it
    /// each row of our centreline is, found by lining the two up by where
    /// they turn. Null when the circuit has no survey file.
    /// </summary>
    public static SurveyLine? Line(string id, IReadOnlyList<Vector2> centre)
    {
        string path = Path.Combine(Folder, id + ".csv");
        if (!File.Exists(path))
            return null;
        (List<Vector2> points, _, double lat0, double lon0) = ReadSurvey(path);
        (int shift, bool reversed, _) = Align(centre, points);
        var along = new double[points.Count + 1];
        for (int i = 0; i < points.Count; i++)
            along[i + 1] = along[i] + Vector2.Distance(points[i], points[(i + 1) % points.Count]);
        int n = centre.Count;
        var at = new Vector2[n];
        for (int i = 0; i < n; i++)
        {
            double f = (double)i / n;
            double g = reversed ? (shift / (double)AlignBins - f) : (f + shift / (double)AlignBins);
            double s = (g - Math.Floor(g)) * along[^1];
            int k = Array.BinarySearch(along, s);
            if (k < 0)
                k = ~k - 1;
            k = Math.Clamp(k, 0, points.Count - 1);
            float t = (float)((s - along[k]) / Math.Max(along[k + 1] - along[k], 1e-9));
            at[i] = Vector2.Lerp(points[k], points[(k + 1) % points.Count], t);
        }
        return new SurveyLine(lat0, lon0, at);
    }

    /// <summary>The real line about (<see cref="Lat0"/>, <see cref="Lon0"/>), and where each of our rows is on it.</summary>
    public sealed record SurveyLine(double Lat0, double Lon0, Vector2[] AtRow)
    {
        public Vector2 Project(double lat, double lon) => Elevation.Project(lat, lon, Lat0, Lon0);
    }

    private static (List<Vector2> Points, List<double> Heights, double Lat0, double Lon0) ReadSurvey(string path)
    {
        var rows = File.ReadAllLines(path).Where(l => l.Length > 0 && !l.StartsWith('#')).Skip(1)
            .Select(l => l.Split(',').Select(f => double.Parse(f, CultureInfo.InvariantCulture)).ToArray()).ToList();
        double lat0 = rows.Average(r => r[1]);
        double lon0 = rows.Average(r => r[2]);
        return (rows.Select(r => Project(r[1], r[2], lat0, lon0)).ToList(), rows.Select(r => r[3]).ToList(), lat0, lon0);
    }

    private const int AlignBins = 2000;

    private static (int Shift, bool Reversed, float Score) Align(IReadOnlyList<Vector2> ours, IReadOnlyList<Vector2> theirs) =>
        Align(TurnProfile(ours, AlignBins), TurnProfile(theirs, AlignBins));

    private static float SteepestGrade(float[] z)
    {
        int n = z.Length;
        float steepest = 0f;
        for (int i = 0; i < n; i++)
            steepest = MathF.Max(steepest, MathF.Abs(z[(i + 1) % n] - z[(i - 1 + n) % n]) / 2f);
        return steepest;
    }

    /// <summary>
    /// The steepest stretch, up or down -- the run of road around the
    /// steepest row that keeps at least a quarter of its grade -- drawn in
    /// towards its middle until the middle reaches <paramref name="target"/>.
    /// Each row's grade above the stretch's edge is raised to a power and
    /// scaled back to the height the stretch had, so the climb's two ends do
    /// not move, its grade runs on unbroken into the road either side, and
    /// nothing outside it changes. False when the survey is already this steep.
    /// </summary>
    private static bool Steepen(float[] z, float target)
    {
        int n = z.Length;
        var grade = new float[n];
        for (int i = 0; i < n; i++)
            grade[i] = z[(i + 1) % n] - z[i];
        int peak = 0;
        for (int i = 1; i < n; i++)
            if (MathF.Abs(grade[i]) > MathF.Abs(grade[peak]))
                peak = i;
        float sign = MathF.Sign(grade[peak]);
        float top = MathF.Abs(grade[peak]);
        if (top >= target)
            return false;

        int first = peak, last = peak;
        while (sign * grade[(first - 1 + n) % n] > 0.25f * top && (peak - first) < n / 4)
            first--;
        while (sign * grade[(last + 1) % n] > 0.25f * top && (last - peak) < n / 4)
            last++;
        int count = last - first + 1;
        var g = new float[count];
        float climb = 0f;
        for (int k = 0; k < count; k++)
        {
            g[k] = sign * grade[(first + k + n) % n];
            climb += g[k];
        }

        // Only the grade above the stretch's edge is drawn in, so where the
        // stretch begins and ends the grade runs on unbroken: a step in the
        // grade is a kink in the road, felt as a jolt and seen by the bake
        // as a spike in the vertical bend.
        float edge = 0.25f * top;
        float[] Drawn(float power)
        {
            var d = new float[count];
            float sum = 0f;
            for (int k = 0; k < count; k++)
                sum += d[k] = MathF.Pow(MathF.Max(g[k] - edge, 0f) / (top - edge), power);
            float scale = (climb - count * edge) / sum;
            for (int k = 0; k < count; k++)
                d[k] = edge + d[k] * scale;
            return d;
        }

        float low = 1f, high = 8f;
        for (int iteration = 0; iteration < 40; iteration++)
        {
            float mid = 0.5f * (low + high);
            if (Drawn(mid).Max() < target) low = mid; else high = mid;
        }
        float[] drawn = Drawn(high);
        if (drawn.Max() < target * 0.98f)
            return false;
        float height = z[(first + n) % n];
        for (int k = 0; k < count - 1; k++)
        {
            height += sign * drawn[k];
            z[(first + k + 1 + n) % n] = height;
        }
        return true;
    }

    private static List<(double Lat, double Lon)> Ring(string geojson)
    {
        using JsonDocument doc = JsonDocument.Parse(geojson);
        JsonElement geometry = doc.RootElement.GetProperty("features")[0].GetProperty("geometry");
        JsonElement coords = geometry.GetProperty("coordinates");
        if (geometry.GetProperty("type").GetString() == "MultiLineString")
            coords = coords[0];
        var ring = coords.EnumerateArray().Select(c => (Lat: c[1].GetDouble(), Lon: c[0].GetDouble())).ToList();
        if (ring.Count > 1 && ring[0] == ring[^1])
            ring.RemoveAt(ring.Count - 1);
        return ring;
    }

    // Onto the game's plane, y to the south like the centrelines (Sources),
    // so the two lines turn the same way when they are lined up.
    internal static Vector2 Project(double lat, double lon, double lat0, double lon0) => new(
        (float)((lon - lon0) * Math.PI / 180.0 * EarthRadius * Math.Cos(lat0 * Math.PI / 180.0)),
        (float)(-(lat - lat0) * Math.PI / 180.0 * EarthRadius));

    private static List<(double Lat, double Lon, double S)> Resample(List<(double Lat, double Lon)> ring, float step)
    {
        double lat0 = ring.Average(p => p.Lat), lon0 = ring.Average(p => p.Lon);
        var xy = ring.Select(p => Project(p.Lat, p.Lon, lat0, lon0)).ToList();
        int n = xy.Count;
        var along = new double[n + 1];
        for (int i = 0; i < n; i++)
            along[i + 1] = along[i] + Vector2.Distance(xy[i], xy[(i + 1) % n]);
        int count = (int)Math.Round(along[n] / step);
        double spacing = along[n] / count;
        var result = new List<(double, double, double)>(count);
        int k = 0;
        for (int j = 0; j < count; j++)
        {
            double s = j * spacing;
            while (k < n - 1 && along[k + 1] <= s)
                k++;
            double t = (s - along[k]) / Math.Max(along[k + 1] - along[k], 1e-9);
            var a = ring[k];
            var b = ring[(k + 1) % n];
            result.Add((a.Lat + (b.Lat - a.Lat) * t, a.Lon + (b.Lon - a.Lon) * t, s));
        }
        return result;
    }

    private static double[] FillGaps(double[] h)
    {
        double fallback = h.Where(v => !double.IsNaN(v)).DefaultIfEmpty(0).Average();
        return h.Select(v => double.IsNaN(v) ? fallback : v).ToArray();
    }

    private static double[] Smooth(double[] h)
    {
        int n = h.Length;
        var median = new double[n];
        for (int i = 0; i < n; i++)
        {
            var window = new double[2 * MedianHalfWidth + 1];
            for (int k = -MedianHalfWidth; k <= MedianHalfWidth; k++)
                window[k + MedianHalfWidth] = h[((i + k) % n + n) % n];
            Array.Sort(window);
            median[i] = window[MedianHalfWidth];
        }
        var average = new double[n];
        for (int i = 0; i < n; i++)
        {
            double sum = 0;
            for (int k = -AverageHalfWidth; k <= AverageHalfWidth; k++)
                sum += median[((i + k) % n + n) % n];
            average[i] = sum / (2 * AverageHalfWidth + 1);
        }
        return average;
    }

    private static float[] RowAverage(float[] z, int half)
    {
        int n = z.Length;
        var result = new float[n];
        double sum = 0;
        for (int k = -half; k <= half; k++)
            sum += z[((k % n) + n) % n];
        for (int i = 0; i < n; i++)
        {
            result[i] = (float)(sum / (2 * half + 1));
            sum += z[(i + half + 1) % n] - z[((i - half) % n + n) % n];
        }
        return result;
    }

    private static double CatmullRom(double[] h, double x)
    {
        int n = h.Length;
        int i = (int)Math.Floor(x);
        double t = x - i;
        double p0 = h[((i - 1) % n + n) % n], p1 = h[(i % n + n) % n], p2 = h[((i + 1) % n + n) % n], p3 = h[((i + 2) % n + n) % n];
        return 0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t + (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t);
    }

    /// <summary>Signed turning per bin of the lap, smoothed: the shape of a circuit, independent of where it starts.</summary>
    private static float[] TurnProfile(IReadOnlyList<Vector2> points, int bins)
    {
        int n = points.Count;
        var along = new double[n + 1];
        for (int i = 0; i < n; i++)
            along[i + 1] = along[i] + Vector2.Distance(points[i], points[(i + 1) % n]);
        Vector2 At(double f)
        {
            double s = (f - Math.Floor(f)) * along[n];
            int lo = 0, hi = n;
            while (hi - lo > 1) { int mid = (lo + hi) / 2; if (along[mid] <= s) lo = mid; else hi = mid; }
            float t = (float)((s - along[lo]) / Math.Max(along[lo + 1] - along[lo], 1e-9));
            return Vector2.Lerp(points[lo], points[(lo + 1) % n], t);
        }
        var heading = new double[bins];
        for (int b = 0; b < bins; b++)
        {
            Vector2 d = At((b + 1.0) / bins) - At((double)b / bins);
            heading[b] = Math.Atan2(d.Y, d.X);
        }
        var turn = new float[bins];
        for (int b = 0; b < bins; b++)
        {
            double dh = heading[(b + 1) % bins] - heading[b];
            while (dh > Math.PI) dh -= 2 * Math.PI;
            while (dh < -Math.PI) dh += 2 * Math.PI;
            turn[b] = (float)dh;
        }
        // Smooth over 1% of the lap so the two lines need not agree to the metre.
        var smooth = new float[bins];
        const int half = 10;
        for (int b = 0; b < bins; b++)
        {
            float sum = 0f;
            for (int k = -half; k <= half; k++)
                sum += turn[((b + k) % bins + bins) % bins];
            smooth[b] = sum;
        }
        return smooth;
    }

    private static (int Shift, bool Reversed, float Score) Align(float[] ours, float[] theirs)
    {
        int bins = ours.Length;
        double norm = Math.Sqrt(ours.Sum(v => (double)v * v) * theirs.Sum(v => (double)v * v));
        (int, bool, float) best = (0, false, float.MinValue);
        foreach (bool reversed in new[] { false, true })
        {
            for (int shift = 0; shift < bins; shift++)
            {
                double sum = 0;
                for (int b = 0; b < bins; b++)
                {
                    // Forward: our bin b is their bin b + shift. Reversed: their bin shift - b, turning the other way.
                    float t = reversed ? -theirs[((shift - b - 1) % bins + bins) % bins] : theirs[(b + shift) % bins];
                    sum += ours[b] * t;
                }
                float score = (float)(sum / norm);
                if (score > best.Item3)
                    best = (shift, reversed, score);
            }
        }
        return best;
    }
}
