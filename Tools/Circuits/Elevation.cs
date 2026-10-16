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
        (PublishedClimbMeters.TryGetValue(id, out float m) ? $", scaled to the published {m:F1} m highest to lowest." : ".");

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
        var points = new List<Vector2>();
        var raw = new List<double>();
        double lat0 = 0, lon0 = 0;
        var rows = File.ReadAllLines(path).Where(l => l.Length > 0 && !l.StartsWith('#')).Skip(1)
            .Select(l => l.Split(',').Select(f => double.Parse(f, CultureInfo.InvariantCulture)).ToArray()).ToList();
        lat0 = rows.Average(r => r[1]);
        lon0 = rows.Average(r => r[2]);
        foreach (double[] r in rows)
        {
            points.Add(Project(r[1], r[2], lat0, lon0));
            raw.Add(r[3]);
        }
        double[] heights = Smooth(FillGaps(raw.ToArray()));

        // Line the survey's line up with ours by where they turn.
        const int bins = 2000;
        float[] ours = TurnProfile(centre, bins);
        float[] theirs = TurnProfile(points, bins);
        (int shift, bool reversed, float score) = Align(ours, theirs);

        int n = centre.Count;
        int m = heights.Length;
        var z = new float[n];
        for (int i = 0; i < n; i++)
        {
            double f = (double)i / n;
            double g = reversed ? (shift / (double)bins - f) : (f + shift / (double)bins);
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

        float maxGrade = 0f;
        for (int i = 0; i < n; i++)
            maxGrade = MathF.Max(maxGrade, MathF.Abs(z[(i + 1) % n] - z[(i - 1 + n) % n]) / 2f);
        report = $"survey {raw.Min():F0}..{raw.Max():F0} m, aligned {(reversed ? "reversed" : "forward")} at {shift * 100f / bins:F1}% (match {score:F2}), " +
                 $"climb {z.Max() - z.Min():F1} m{scaled}, steepest {maxGrade * 100f:F1}%";
        return z;
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

    private static Vector2 Project(double lat, double lon, double lat0, double lon0) => new(
        (float)((lon - lon0) * Math.PI / 180.0 * EarthRadius * Math.Cos(lat0 * Math.PI / 180.0)),
        (float)((lat - lat0) * Math.PI / 180.0 * EarthRadius));

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
