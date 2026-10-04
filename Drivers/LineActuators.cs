namespace StintegyEVO.Drivers;

/// <summary>
/// The steering and throttle the car should hold to stay on a geometric
/// line at this car's envelope. Kappa is the value that puts n(T) on the
/// line, clipped by the road edges. Drive/brake is whatever longitudinal
/// remains at that kappa.
/// </summary>
internal static class LineActuators
{
    public const float HorizonSeconds = 0.50f;
    public const float EdgeMarginMeters = 0.30f;
    public const float MaxHeadingRadians = 0.40f;

    public readonly record struct Steer(
        float Curvature,
        float PredictedN,
        float NLow,
        float NHigh
    );

    /// <summary>
    /// Frenet with left-positive n, headingError = path heading − car heading,
    /// positive kappa increases n:
    ///   ṅ = −v sin(ψ),  ψ̇ = v (pathK − kappa),
    ///   n(T) ≈ n − v sin(ψ) T + ½ v² cos(ψ) (kappa − pathK) T².
    /// </summary>
    public static Steer CurvatureForLine(
        float n,
        float nLine,
        float headingError,
        float speed,
        float pathKappa,
        float halfWidth,
        float carHalfWidth,
        float maxKappa
    )
    {
        Band band = RoadBand(nLine, halfWidth, carHalfWidth);
        (float A, float B) = Predict(n, headingError, speed);
        float extra = MathF.Abs(B) < 1e-6f ? 0f : (band.Want - A) / B;
        extra = ClampExtra(extra, A, B, band.Low, band.High);
        float kappa = Math.Clamp(pathKappa + extra, -maxKappa, maxKappa);
        extra = kappa - pathKappa;
        return new Steer(kappa, A + B * extra, band.Low, band.High);
    }

    /// <summary>
    /// Keep a tracking command, but drop any extra curvature that would put
    /// n(T) past the road or past the line on the kerb side.
    /// </summary>
    public static float LimitToBand(
        float requested,
        float n,
        float nLine,
        float headingError,
        float speed,
        float pathKappa,
        float halfWidth,
        float carHalfWidth,
        float maxKappa
    )
    {
        Band band = RoadBand(nLine, halfWidth, carHalfWidth);
        (float A, float B) = Predict(n, headingError, speed);
        float extra = requested - pathKappa;
        extra = ClampExtra(extra, A, B, band.Low, band.High);
        return Math.Clamp(pathKappa + extra, -maxKappa, maxKappa);
    }

    public static float RemainingAccel(
        float speed,
        float target,
        float speedGain,
        float gradeCompensation,
        float maxDrive,
        float maxBrake,
        float loss,
        bool allowDrive
    )
    {
        float error = target - speed;
        float net = error < -1.0f
            ? -maxBrake + gradeCompensation
            : speedGain * error + gradeCompensation;
        float drive = allowDrive ? maxDrive : 0f;
        return Math.Clamp(net + loss, -maxBrake, drive);
    }

    internal static void RoadLimits(
        float nLine,
        float halfWidth,
        float carHalfWidth,
        out float nLow,
        out float nHigh
    )
    {
        Band band = RoadBand(nLine, halfWidth, carHalfWidth);
        nLow = band.Low;
        nHigh = band.High;
    }

    private static (float A, float B) Predict(float n, float headingError, float speed)
    {
        float T = HorizonSeconds;
        float v = MathF.Max(speed, 1f);
        float psi = Math.Clamp(headingError, -MaxHeadingRadians, MaxHeadingRadians);
        float cos = MathF.Max(MathF.Cos(psi), 0.50f);
        return (n - v * MathF.Sin(psi) * T, 0.5f * v * v * cos * T * T);
    }

    private static Band RoadBand(float nLine, float halfWidth, float carHalfWidth)
    {
        float margin = carHalfWidth + EdgeMarginMeters;
        float nLo = -halfWidth + margin;
        float nHi = halfWidth - margin;
        if (nLo > nHi)
        {
            float mid = 0.5f * (nLo + nHi);
            nLo = nHi = mid;
        }

        if (nLine - nLo < 1.5f)
            nLo = MathF.Max(nLo, nLine - 0.15f);
        if (nHi - nLine < 1.5f)
            nHi = MathF.Min(nHi, nLine + 0.15f);
        if (nLo > nHi)
        {
            float mid = 0.5f * (nLo + nHi);
            nLo = nHi = mid;
        }

        return new Band(nLo, nHi, Math.Clamp(nLine, nLo, nHi));
    }

    private readonly record struct Band(float Low, float High, float Want);

    private static float ClampExtra(float extra, float A, float B, float nLo, float nHi)
    {
        if (MathF.Abs(B) < 1e-6f)
            return 0f;
        float lo = (nLo - A) / B;
        float hi = (nHi - A) / B;
        if (lo > hi)
            (lo, hi) = (hi, lo);
        return Math.Clamp(extra, lo, hi);
    }
}
