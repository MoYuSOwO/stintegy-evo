using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers;

/// <summary>
/// Velocity on a closed path. Kappa is taken from the path once; the speed
/// table is recomputed from the live car envelope so tyre, energy and
/// strategy changes move the plan.
/// </summary>
internal sealed class LineSpeedProfile
{
    private readonly float[] _speed;
    private readonly float[] _kAbs;
    private readonly float _length;
    private readonly float _ds;

    private LineSpeedProfile(float[] speed, float[] kAbs, float length, float ds)
    {
        _speed = speed;
        _kAbs = kAbs;
        _length = length;
        _ds = ds;
    }

    public static LineSpeedProfile FromLine(GeometricRacingLine line, DriverTrackView track) =>
        FromLine(line, track.Sample, track.WrapS, line.LengthMeters, includeRoadCurvature: false, windowMeters: 2f);

    public static LineSpeedProfile FromLine(
        GeometricRacingLine line,
        TrackData track,
        bool includeRoadCurvature = false,
        float windowMeters = 2f,
        bool usePathCurvature = false,
        float curvatureScale = 1f
    ) =>
        FromLine(
            line,
            track.Sample,
            track.WrapS,
            line.LengthMeters,
            includeRoadCurvature,
            windowMeters,
            usePathCurvature,
            curvatureScale
        );

    public static LineSpeedProfile FromLine(
        GeometricRacingLine line,
        DriverTrackView track,
        bool includeRoadCurvature,
        float windowMeters = 2f,
        bool usePathCurvature = false,
        float curvatureScale = 1f
    ) =>
        FromLine(
            line,
            track.Sample,
            track.WrapS,
            line.LengthMeters,
            includeRoadCurvature,
            windowMeters,
            usePathCurvature,
            curvatureScale
        );

    private static LineSpeedProfile FromLine(
        GeometricRacingLine line,
        Func<float, TrackSample> sample,
        Func<float, float> wrap,
        float lengthMeters,
        bool includeRoadCurvature,
        float windowMeters = 2f,
        bool usePathCurvature = false,
        float curvatureScale = 1f
    )
    {
        int count = Math.Max(32, (int)MathF.Round(line.LengthMeters));
        float ds = line.LengthMeters / count;
        int window = Math.Max(0, (int)MathF.Round(windowMeters / ds));
        float[] kAbs = new float[count];
        for (int i = 0; i < count; i++)
        {
            float linePeak = 0f;
            float roadPeak = 0f;
            for (int k = -window; k <= window; k++)
            {
                float s = wrap((i + k) * ds);
                float kappa = usePathCurvature
                    ? MathF.Abs(line.PathCurvatureAt(wrap, s))
                    : line.SpeedKappaAt(wrap, s);
                linePeak = MathF.Max(linePeak, kappa);
                if (includeRoadCurvature)
                    roadPeak = MathF.Max(roadPeak, MathF.Abs(sample(s).Curvature));
            }

            kAbs[i] = MathF.Max(linePeak, MathF.Max(roadPeak, 1e-4f)) *
                      Math.Clamp(curvatureScale, 0.5f, 1.2f);
        }

        return new LineSpeedProfile(new float[count], kAbs, line.LengthMeters, ds);
    }

    public void Recompute(
        CarState plan,
        CarConfig car,
        TireConfig tires,
        float gripUsage,
        float ggUsage,
        Func<float, float>? ggAt = null,
        float topSpeedMetersPerSecond = 80f,
        float brakeUsage = 1f
    )
    {
        GgDiagram gg = GgDiagram.From(plan, car, tires, gripUsage, ggUsage);
        int count = _speed.Length;
        for (int i = 0; i < count; i++)
        {
            float localGg = ggAt?.Invoke(i * _ds) ?? ggUsage;
            float scale = localGg / MathF.Max(gg.GgUsage, 0.05f);
            float aeroA = gg.AeroA * scale;
            float aeroB = gg.AeroB * scale;
            float denom = _kAbs[i] - MathF.Max(aeroB, 0f);
            float vMax = denom > 1e-5f
                ? MathF.Sqrt(MathF.Max(aeroA, 1f) / denom)
                : 95f;
            _speed[i] = MathF.Min(MathF.Max(topSpeedMetersPerSecond, 1f), vMax);
        }

        for (int pass = 0; pass < 2; pass++)
        {
            for (int i = count - 1; i >= 0; i--)
            {
                int next = (i + 1) % count;
                float v = _speed[next];
                float share = MathF.Min(1f, v * v * _kAbs[i] / gg.Ay(v));
                float ax = gg.Brake(v) *
                           Math.Clamp(brakeUsage, 0.2f, 1f) *
                           MathF.Sqrt(MathF.Max(0f, 1f - share * share));
                _speed[i] = MathF.Min(
                    _speed[i],
                    MathF.Sqrt(MathF.Max(0f, v * v + 2f * ax * _ds))
                );
            }
        }

        for (int pass = 0; pass < 2; pass++)
        {
            for (int i = 0; i < count; i++)
            {
                int prev = (i - 1 + count) % count;
                float v = _speed[prev];
                float share = MathF.Min(1f, v * v * _kAbs[prev] / gg.Ay(v));
                float ax = gg.Drive(v) * MathF.Sqrt(MathF.Max(0f, 1f - share * share));
                _speed[i] = MathF.Min(
                    _speed[i],
                    MathF.Sqrt(MathF.Max(0f, v * v + 2f * ax * _ds))
                );
            }
        }
    }

    public float At(Func<float, float> wrap, float s)
    {
        float wrapped = wrap(s);
        float t = wrapped / _length * _speed.Length;
        int i0 = ((int)MathF.Floor(t) % _speed.Length + _speed.Length) % _speed.Length;
        int i1 = (i0 + 1) % _speed.Length;
        float u = t - MathF.Floor(t);
        return _speed[i0] + (_speed[i1] - _speed[i0]) * u;
    }

    public float MinAhead(Func<float, float> wrap, float s, float horizonMeters)
    {
        float min = At(wrap, s);
        int steps = Math.Max(1, (int)MathF.Ceiling(horizonMeters / MathF.Max(_ds, 0.5f)));
        for (int k = 1; k <= steps; k++)
            min = MathF.Min(min, At(wrap, wrap(s + k * _ds)));
        return min;
    }

    public float EstimateLapSeconds()
    {
        float seconds = 0f;
        for (int i = 0; i < _speed.Length; i++)
            seconds += _ds / MathF.Max(_speed[i], 1f);
        return seconds;
    }

    public float PeakSpeed()
    {
        float peak = 0f;
        for (int i = 0; i < _speed.Length; i++)
            peak = MathF.Max(peak, _speed[i]);
        return peak;
    }

    public float MeanAbsKappa()
    {
        float sum = 0f;
        for (int i = 0; i < _kAbs.Length; i++)
            sum += _kAbs[i];
        return sum / _kAbs.Length;
    }

    public float MinAlong(
        ReadOnlySpan<StanleyPathPredictor.Point> path,
        float stepMeters,
        CarState plan,
        CarConfig car,
        TireConfig tires,
        float gripUsage,
        float ggUsage
    )
    {
        GgDiagram gg = GgDiagram.From(plan, car, tires, gripUsage, ggUsage);
        float minV = 95f;
        for (int i = path.Length - 1; i >= 0; i--)
        {
            float absK = MathF.Max(MathF.Abs(path[i].PathCurvature), 1e-4f);
            float denom = absK - MathF.Max(gg.AeroB, 0f);
            float vCorner = denom > 1e-5f
                ? MathF.Sqrt(MathF.Max(gg.AeroA, 1f) / denom)
                : 95f;
            vCorner = MathF.Min(95f, vCorner);
            if (i == path.Length - 1)
            {
                minV = vCorner;
                continue;
            }

            float share = MathF.Min(1f, minV * minV * absK / gg.Ay(minV));
            float ax = gg.Brake(minV) * MathF.Sqrt(MathF.Max(0f, 1f - share * share));
            float fromNext = MathF.Sqrt(MathF.Max(0f, minV * minV + 2f * ax * stepMeters));
            minV = MathF.Min(vCorner, fromNext);
        }

        return minV;
    }

    private readonly struct GgDiagram
    {
        public readonly float AeroA;
        public readonly float AeroB;
        public readonly float GgUsage;
        private readonly float _brake12;
        private readonly float _brake40;
        private readonly float _brake70;
        private readonly float _drive12;
        private readonly float _drive40;
        private readonly float _drive70;

        private GgDiagram(
            float aeroA,
            float aeroB,
            float ggUsage,
            CarPerformanceLimits slow,
            CarPerformanceLimits mid,
            CarPerformanceLimits fast
        )
        {
            AeroA = aeroA;
            AeroB = aeroB;
            GgUsage = ggUsage;
            _brake12 = slow.MaximumBrakeDeceleration;
            _brake40 = mid.MaximumBrakeDeceleration;
            _brake70 = fast.MaximumBrakeDeceleration;
            _drive12 = slow.MaximumDriveAcceleration;
            _drive40 = mid.MaximumDriveAcceleration;
            _drive70 = fast.MaximumDriveAcceleration;
        }

        public static GgDiagram From(
            CarState plan,
            CarConfig car,
            TireConfig tires,
            float gripUsage,
            float ggUsage
        )
        {
            CarPerformanceLimits slow = Limits(plan, car, tires, gripUsage, 12f, 0f);
            CarPerformanceLimits mid = Limits(plan, car, tires, gripUsage, 40f, 0f);
            CarPerformanceLimits fast = Limits(plan, car, tires, gripUsage, 70f, 0f);
            CarPerformanceLimits slowBrake = Limits(
                plan, car, tires, gripUsage, 12f, -slow.MaximumBrakeDeceleration
            );
            CarPerformanceLimits midBrake = Limits(
                plan, car, tires, gripUsage, 40f, -mid.MaximumBrakeDeceleration
            );
            CarPerformanceLimits fastBrake = Limits(
                plan, car, tires, gripUsage, 70f, -fast.MaximumBrakeDeceleration
            );
            float aySlow = MathF.Max(slow.LateralAccelerationLimit * ggUsage, 1f);
            float ayFast = MathF.Max(fast.LateralAccelerationLimit * ggUsage, 1f);
            float aeroB = (ayFast - aySlow) / (70f * 70f - 12f * 12f);
            float aeroA = aySlow - aeroB * 12f * 12f;
            return new GgDiagram(
                aeroA,
                aeroB,
                ggUsage,
                new CarPerformanceLimits(
                    slow.LateralAccelerationLimit,
                    slow.MaximumDriveAcceleration,
                    slowBrake.MaximumBrakeDeceleration,
                    slow.LossAcceleration
                ),
                new CarPerformanceLimits(
                    mid.LateralAccelerationLimit,
                    mid.MaximumDriveAcceleration,
                    midBrake.MaximumBrakeDeceleration,
                    mid.LossAcceleration
                ),
                new CarPerformanceLimits(
                    fast.LateralAccelerationLimit,
                    fast.MaximumDriveAcceleration,
                    fastBrake.MaximumBrakeDeceleration,
                    fast.LossAcceleration
                )
            );
        }

        public float Ay(float speed) =>
            MathF.Max(AeroA + AeroB * speed * speed, 1f);

        public float Brake(float speed) =>
            GgUsage * MathF.Max(Lerp(speed, _brake12, _brake40, _brake70), 1f);

        public float Drive(float speed) =>
            GgUsage * MathF.Max(Lerp(speed, _drive12, _drive40, _drive70), 1f);

        private static CarPerformanceLimits Limits(
            CarState plan,
            CarConfig car,
            TireConfig tires,
            float gripUsage,
            float speed,
            float assumedAx
        ) =>
            CarPhysics.EstimatePerformanceLimits(
                plan,
                car,
                tires,
                CarStrategy.Default,
                speed,
                curvature: 0f,
                gripUsage,
                assumedLongitudinalAcceleration: assumedAx
            );

        private static float Lerp(float speed, float at12, float at40, float at70)
        {
            float vv = speed * speed;
            if (speed <= 12f)
                return at12;
            if (speed <= 40f)
                return at12 + (at40 - at12) * (vv - 144f) / (1600f - 144f);
            if (speed >= 70f)
                return at70;
            return at40 + (at70 - at40) * (vv - 1600f) / (4900f - 1600f);
        }
    }
}
