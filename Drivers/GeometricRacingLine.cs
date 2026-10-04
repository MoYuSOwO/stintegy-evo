using System.Numerics;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers;

/// <summary>
/// A closed offset from the road centreline. This is a driver asset, not track
/// geometry: Core stores no preferred line.
///
/// Track-specific drivers load a statically baked CSV (TUMRT 3D time-optimal
/// line plus v*). GeometricBake remains the fallback when no CSV exists.
/// </summary>
public sealed class GeometricRacingLine
{
    public readonly record struct Sample(
        float S,
        float OffsetMeters,
        Vector2 Position,
        float Heading,
        float PathCurvature
    );

    private readonly float[] _offset;
    private readonly float[] _curvature;
    private readonly float[]? _speed;
    private readonly float[]? _speedKappa;
    private readonly float _length;

    private GeometricRacingLine(
        float[] offset,
        float[] curvature,
        float length,
        float[]? speed = null,
        float[]? speedKappa = null
    )
    {
        _offset = offset;
        _curvature = curvature;
        _length = length;
        _speed = speed;
        _speedKappa = speedKappa;
    }

    public float LengthMeters => _length;
    public bool HasBakedSpeed => _speed is { Length: > 0 };

    public static GeometricRacingLine Bake(TrackData track, float vehicleHalfWidthMeters) =>
        Bake(track.Sample, track.LengthMeters, track.WrapS, vehicleHalfWidthMeters);

    public static GeometricRacingLine Bake(DriverTrackView track, float vehicleHalfWidthMeters) =>
        Bake(track.Sample, track.LengthMeters, track.WrapS, vehicleHalfWidthMeters);

    public static GeometricRacingLine Bake(
        Func<float, TrackSample> sample,
        float lengthMeters,
        Func<float, float> wrap,
        float vehicleHalfWidthMeters
    )
    {
        ArgumentNullException.ThrowIfNull(sample);
        ArgumentNullException.ThrowIfNull(wrap);
        int count = Math.Max(32, (int)MathF.Round(lengthMeters));
        float[] raw = new float[count];
        for (int i = 0; i < count; i++)
        {
            float s = wrap(i * lengthMeters / count);
            raw[i] = OffsetAt(sample, wrap, s, vehicleHalfWidthMeters);
        }

        float[] smooth = new float[count];
        for (int i = 0; i < count; i++)
        {
            float sum = 0f;
            for (int k = -10; k <= 10; k++)
                sum += raw[(i + k + count) % count];
            TrackSample here = sample(wrap(i * lengthMeters / count));
            float half = UsableHalf(here, vehicleHalfWidthMeters);
            smooth[i] = Math.Clamp(sum / 21f, -half, half);
        }

        float[] rawKappa = new float[count];
        float ds = lengthMeters / count;
        int chord = Math.Max(4, (int)MathF.Round(10f / ds));
        for (int i = 0; i < count; i++)
        {
            Vector2 behind = PositionAt(sample, wrap, smooth, lengthMeters, wrap((i - chord) * ds));
            Vector2 here = PositionAt(sample, wrap, smooth, lengthMeters, wrap(i * ds));
            Vector2 ahead = PositionAt(sample, wrap, smooth, lengthMeters, wrap((i + chord) * ds));
            rawKappa[i] = PathCurvature(behind, here, ahead);
        }

        float[] kappa = new float[count];
        int roadWindow = Math.Max(4, (int)MathF.Round(20f / ds));
        for (int i = 0; i < count; i++)
        {
            float sum = 0f;
            for (int k = -5; k <= 5; k++)
                sum += rawKappa[(i + k + count) % count];
            float smoothed = sum / 11f;
            float roadPeak = 0f;
            for (int k = -roadWindow; k <= roadWindow; k++)
            {
                TrackSample around = sample(wrap((i + k) * ds));
                roadPeak = MathF.Max(roadPeak, MathF.Abs(around.Curvature));
            }

            float cap = roadPeak + 0.02f;
            kappa[i] = Math.Clamp(smoothed, -cap, cap);
        }

        return new GeometricRacingLine(smooth, kappa, lengthMeters);
    }

    public static GeometricRacingLine FromBakedCsv(
        string path,
        Func<Vector2, TrackPose> project,
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        float lengthMeters
    )
    {
        string[] lines = File.ReadAllLines(path);
        if (lines.Length < 3)
            throw new InvalidOperationException($"Racing line CSV is empty: {path}");

        string[] header = lines[0].Split(',');
        int xCol = Column(header, "x_m");
        int yCol = Column(header, "y_m");
        int sCol = Column(header, "s_m");
        int nCol = Column(header, "offset_m");
        int kCol = Column(header, "kappa_radpm");
        bool hasSn = sCol >= 0 && nCol >= 0;
        bool hasXy = xCol >= 0 && yCol >= 0;
        int rows = lines.Length - 1;
        float[] sRaw = new float[rows];
        float[] nRaw = new float[rows];
        float[] kRaw = new float[rows];
        for (int i = 0; i < rows; i++)
        {
            string[] cols = lines[i + 1].Split(',');
            // Project TUM xy onto this track so s/n match the live car.
            // CSV s/n is TUM's own stationing and can fall out of phase.
            if (hasXy)
            {
                var xy = new Vector2(Parse(cols[xCol]), Parse(cols[yCol]));
                TrackPose pose = project(xy);
                sRaw[i] = pose.S;
                nRaw[i] = pose.D;
            }
            else if (hasSn)
            {
                sRaw[i] = Parse(cols[sCol]);
                nRaw[i] = Parse(cols[nCol]);
            }
            else
            {
                sRaw[i] = i * lengthMeters / rows;
                nRaw[i] = 0f;
            }

            kRaw[i] = kCol >= 0 ? Parse(cols[kCol]) : 0f;
        }

        int count = Math.Max(32, (int)MathF.Round(lengthMeters));
        float[] offset = new float[count];
        float[] kappa = new float[count];
        float[] sUnwrapped = UnwrapClosed(sRaw, lengthMeters);
        for (int i = 0; i < count; i++)
        {
            float s = i * lengthMeters / count;
            offset[i] = SampleClosed(sUnwrapped, nRaw, s, lengthMeters);
            TrackSample road = sample(wrap(s));
            float half = MathF.Max(0.4f, road.HalfWidth - 0.95f - 0.2f);
            offset[i] = Math.Clamp(offset[i], -half, half);
        }

        const int offsetSmooth = 6;
        float[] nSmooth = new float[count];
        for (int i = 0; i < count; i++)
        {
            float sum = 0f;
            for (int k = -offsetSmooth; k <= offsetSmooth; k++)
                sum += offset[(i + k + count) % count];
            TrackSample road = sample(wrap(i * lengthMeters / count));
            float half = MathF.Max(0.4f, road.HalfWidth - 0.95f - 0.2f);
            nSmooth[i] = Math.Clamp(sum / (2 * offsetSmooth + 1), -half, half);
        }

        float ds = lengthMeters / count;
        int chord = Math.Max(4, (int)MathF.Round(16f / ds));
        int capWindow = Math.Max(4, (int)MathF.Round(40f / ds));
        float[] roadCap = new float[count];
        for (int i = 0; i < count; i++)
        {
            float peak = 0.01f;
            for (int k = -capWindow; k <= capWindow; k++)
                peak = MathF.Max(peak, MathF.Abs(sample(wrap((i + k) * ds)).Curvature));
            roadCap[i] = peak + 0.003f;
        }

        float[] rawKappa = new float[count];
        for (int pass = 0; pass < 16; pass++)
        {
            FillPathCurvature(sample, wrap, nSmooth, lengthMeters, ds, chord, rawKappa);
            bool hot = false;
            float[] next = (float[])nSmooth.Clone();
            for (int i = 0; i < count; i++)
            {
                if (MathF.Abs(rawKappa[i]) <= roadCap[i])
                    continue;
                hot = true;
                int prev = (i - 1 + count) % count;
                int nxt = (i + 1) % count;
                float mid = 0.5f * (nSmooth[prev] + nSmooth[nxt]);
                float half = MathF.Max(0.4f, sample(wrap(i * ds)).HalfWidth - 0.95f - 0.2f);
                next[i] = Math.Clamp(nSmooth[i] + 0.55f * (mid - nSmooth[i]), -half, half);
            }

            nSmooth = next;
            if (!hot)
                break;
        }

        FillPathCurvature(sample, wrap, nSmooth, lengthMeters, ds, chord, rawKappa);
        const int kappaSmooth = 4;
        for (int i = 0; i < count; i++)
        {
            float sum = 0f;
            float cap = 0f;
            for (int k = -kappaSmooth; k <= kappaSmooth; k++)
            {
                int j = (i + k + count) % count;
                sum += rawKappa[j];
                cap = MathF.Max(cap, roadCap[j]);
            }

            kappa[i] = Math.Clamp(sum / (2 * kappaSmooth + 1), -cap, cap);
        }

        float[] speedKappa = new float[count];
        if (kCol >= 0)
        {
            for (int i = 0; i < count; i++)
                speedKappa[i] = MathF.Abs(SampleClosed(sUnwrapped, kRaw, i * ds, lengthMeters));
        }
        else
        {
            for (int i = 0; i < count; i++)
                speedKappa[i] = MathF.Abs(kappa[i]);
        }

        return new GeometricRacingLine(
            nSmooth,
            kappa,
            lengthMeters,
            speed: null,
            speedKappa
        );
    }

    public float SpeedAt(Func<float, float> wrap, float s)
    {
        if (_speed is null)
            return 0f;
        return Interpolate(_speed, wrap(s));
    }

    public float SpeedAheadMin(Func<float, float> wrap, float s, float horizonMeters)
    {
        if (_speed is null)
            return 0f;
        float min = SpeedAt(wrap, s);
        if (horizonMeters <= 0.5f)
            return min;
        for (float d = 4f; d <= horizonMeters; d += 4f)
            min = MathF.Min(min, SpeedAt(wrap, s + d));
        return min;
    }

    public Sample At(Func<float, TrackSample> sample, Func<float, float> wrap, float s)
    {
        float wrapped = wrap(s);
        float offset = Interpolate(_offset, wrapped);
        Vector2 position = PositionAt(sample, wrap, wrapped);
        float ds = 16f;
        Vector2 behind = PositionAt(sample, wrap, wrap(wrapped - ds));
        Vector2 ahead = PositionAt(sample, wrap, wrap(wrapped + ds));
        Vector2 tangent = ahead - behind;
        float heading = tangent.LengthSquared() < 1e-8f
            ? sample(wrapped).Heading
            : MathF.Atan2(tangent.Y, tangent.X);
        return new Sample(wrapped, offset, position, heading, Interpolate(_curvature, wrapped));
    }

    public float PathCurvatureAt(Func<float, float> wrap, float s) =>
        Interpolate(_curvature, wrap(s));

    public float SpeedKappaAt(Func<float, float> wrap, float s)
    {
        if (_speedKappa is null)
            return MathF.Abs(PathCurvatureAt(wrap, s));
        return Interpolate(_speedKappa, wrap(s));
    }

    private float Interpolate(float[] values, float wrapped)
    {
        float t = wrapped / _length * values.Length;
        int i0 = ((int)MathF.Floor(t) % values.Length + values.Length) % values.Length;
        int i1 = (i0 + 1) % values.Length;
        float u = t - MathF.Floor(t);
        return values[i0] + (values[i1] - values[i0]) * u;
    }

    private Vector2 PositionAt(Func<float, TrackSample> sample, Func<float, float> wrap, float s)
    {
        float wrapped = wrap(s);
        TrackSample road = sample(wrapped);
        return road.Center + road.Normal * Interpolate(_offset, wrapped);
    }

    private static Vector2 PositionAt(
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        float[] offset,
        float length,
        float s
    )
    {
        float wrapped = wrap(s);
        TrackSample road = sample(wrapped);
        float t = wrapped / length * offset.Length;
        int i0 = ((int)MathF.Floor(t) % offset.Length + offset.Length) % offset.Length;
        int i1 = (i0 + 1) % offset.Length;
        float u = t - MathF.Floor(t);
        float n = offset[i0] + (offset[i1] - offset[i0]) * u;
        return road.Center + road.Normal * n;
    }

    private static float OffsetAt(
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        float s,
        float vehicleHalfWidthMeters
    )
    {
        TrackSample here = sample(s);
        float half = UsableHalf(here, vehicleHalfWidthMeters);
        float far = MeanCurvature(sample, wrap, s, 35f, 110f);
        float near = MeanCurvature(sample, wrap, s, 6f, 40f);
        float farTurn = MathF.Tanh(far / 0.012f);
        float nearTurn = MathF.Tanh(near / 0.014f);

        // Far window parks the car on the outside of the next corner; the
        // near window cuts back in around the apex. Together that is the
        // cheap geometric out-in-out, not a constant inside offset.
        float geometric = half * Math.Clamp(-0.50f * farTurn + 0.78f * nearTurn, -0.88f, 0.88f);

        // Drainage banks on a GP road are a couple of degrees and must not
        // steal the line. Real banking (Zandvoort, a speedway) is an order
        // larger; only then walk toward the steeper edge.
        float bankMix = Smooth01((MathF.Abs(here.BankSlope) - 0.10f) / 0.18f);
        float high = MathF.Abs(here.BankSlopeAt(half));
        float low = MathF.Abs(here.BankSlopeAt(-half));
        float highSide = high >= low ? 1f : -1f;
        if (MathF.Abs(high - low) < 1e-4f)
            highSide = here.BankSlope != 0f ? MathF.Sign(here.BankSlope) : 0f;
        float banked = highSide * half * 0.55f;
        return Math.Clamp(
            geometric * (1f - bankMix) + banked * bankMix,
            -half,
            half
        );
    }

    private static float MeanCurvature(
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        float s,
        float fromMeters,
        float toMeters
    )
    {
        const int n = 6;
        float sum = 0f;
        for (int k = 0; k < n; k++)
        {
            float d = fromMeters + (toMeters - fromMeters) * (k + 0.5f) / n;
            sum += sample(wrap(s + d)).Curvature;
        }
        return sum / n;
    }

    private static float UsableHalf(TrackSample here, float vehicleHalfWidthMeters)
    {
        return MathF.Max(0.4f, here.HalfWidth - vehicleHalfWidthMeters - 0.35f);
    }

    private static void FillPathCurvature(
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        float[] offset,
        float lengthMeters,
        float ds,
        int chord,
        float[] destination
    )
    {
        int count = offset.Length;
        for (int i = 0; i < count; i++)
        {
            Vector2 behind = PositionAt(sample, wrap, offset, lengthMeters, wrap((i - chord) * ds));
            Vector2 here = PositionAt(sample, wrap, offset, lengthMeters, wrap(i * ds));
            Vector2 ahead = PositionAt(sample, wrap, offset, lengthMeters, wrap((i + chord) * ds));
            destination[i] = PathCurvature(behind, here, ahead);
        }
    }

    private static float PathCurvature(Vector2 behind, Vector2 here, Vector2 ahead)
    {
        Vector2 u = here - behind;
        Vector2 v = ahead - here;
        float a = u.Length();
        float b = v.Length();
        float c = (ahead - behind).Length();
        if (a < 1e-3f || b < 1e-3f || c < 1e-3f)
            return 0f;
        float twiceArea = u.X * v.Y - u.Y * v.X;
        return 2f * twiceArea / (a * b * c);
    }

    private static float Smooth01(float x)
    {
        float t = Math.Clamp(x, 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    private static int Column(string[] header, string name)
    {
        for (int i = 0; i < header.Length; i++)
        {
            if (string.Equals(header[i].Trim(), name, StringComparison.OrdinalIgnoreCase))
                return i;
        }

        return -1;
    }

    private static float Parse(string text) =>
        float.Parse(text, System.Globalization.CultureInfo.InvariantCulture);

    private static float[] UnwrapClosed(float[] s, float length)
    {
        float[] unwrapped = new float[s.Length];
        unwrapped[0] = s[0];
        float laps = 0f;
        for (int i = 1; i < s.Length; i++)
        {
            float delta = s[i] - s[i - 1];
            if (delta < -0.5f * length)
                laps += length;
            else if (delta > 0.5f * length)
                laps -= length;
            unwrapped[i] = s[i] + laps;
        }

        return unwrapped;
    }

    private static float SampleClosed(float[] s, float[] values, float query, float length)
    {
        float start = s[0];
        float q = query;
        while (q < start)
            q += length;
        while (q > start + length)
            q -= length;
        int i = 0;
        while (i + 1 < s.Length && s[i + 1] < q)
            i++;
        int j = (i + 1) % s.Length;
        float s1 = s[i];
        float s2 = i + 1 < s.Length ? s[i + 1] : s[0] + length;
        float span = s2 - s1;
        float u = span <= 1e-6f ? 0f : Math.Clamp((q - s1) / span, 0f, 1f);
        return values[i] + (values[j] - values[i]) * u;
    }
}
