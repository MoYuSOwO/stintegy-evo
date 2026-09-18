using StintegyEVO.TrainingHost.Adapter;

namespace StintegyEVO.TrainingHost.Environment;

/// <summary>
/// The hidden randomisation curriculum (freeze design, section 2): things a
/// training episode varies and never tells the policy.
///
/// Mass sits at the clean end, because the clean parent is the ceiling a
/// policy is built towards; a long tail keeps states and distortions a
/// deployed driver will meet inside the training distribution, so they can
/// be recovered from. Each is drawn once per episode and held for all of
/// it, because a deployed driver's distortion is a fixed trait, and the
/// training distribution has to contain that constancy.
///
/// Nothing drawn here is written into the observation or any descriptor.
/// </summary>
public static class HiddenCurriculum
{
    /// <summary>Share of episodes with the combined-grip limiter at full strength.</summary>
    public const float LimiterFullShare = 0.70f;

    /// <summary>
    /// Share with the limiter off entirely. Rare, and there on purpose: an
    /// always-fitted limiter keeps the car out of the slide-past-the-peak
    /// family of states, and a policy that never sees them cannot catch one.
    /// </summary>
    public const float LimiterOffShare = 0.05f;

    /// <summary>The rest draw a partial strength from [0.5, 1).</summary>
    public const float LimiterPartialMinimum = 0.5f;

    /// <summary>Share of episodes whose perception is noise-free.</summary>
    public const float CleanPerceptionShare = 0.70f;

    /// <summary>
    /// The perception channels that take noise, each with its largest
    /// per-episode standard deviation in the channel's own scaled units.
    /// Tyre temperatures (scaled by 150 C) by up to 1.5 C; tyre loads (a
    /// share of static corner load), the two axle friction-circle uses and
    /// the three aero readings by up to 0.02. Geometry and the ego's own
    /// motion take none: they are what the car's position and inertial
    /// sensing give, not what it has to infer. Bias is not drawn: whether a
    /// constant bias reads as personality is untested, and waits for its
    /// own probe.
    /// </summary>
    public static readonly (int Channel, float SigmaMax)[] NoisyChannels = BuildNoisyChannels();

    public readonly record struct Draw(float LimiterStrength, float NoiseScale);

    /// <summary>
    /// One episode's draw from four uniforms in [0, 1). The noise scale is a
    /// fraction of each channel's own maximum.
    /// </summary>
    public static Draw FromUniforms(float limiterPick, float limiterLevel, float noisePick, float noiseLevel)
    {
        float strength;
        if (limiterPick < LimiterFullShare)
            strength = 1f;
        else if (limiterPick < 1f - LimiterOffShare)
            strength = LimiterPartialMinimum + (1f - LimiterPartialMinimum) * limiterLevel;
        else
            strength = 0f;

        float scale = noisePick < CleanPerceptionShare
            ? 0f
            // (0, 1]: a noisy episode is never silently a clean one.
            : 1f - noiseLevel;
        return new Draw(strength, scale);
    }

    private static (int, float)[] BuildNoisyChannels()
    {
        List<(int, float)> channels = [];
        int tyre = DirectDriveObservation.TireAndBatteryOffset;
        const float temperature = 1.5f / 150f;
        const float small = 0.02f;
        for (int wheel = 0; wheel < 4; wheel++)
        {
            int first = tyre + wheel * 4;
            channels.Add((first, temperature));      // surface temperature
            channels.Add((first + 1, temperature));  // core temperature
            channels.Add((first + 3, small));        // load
        }
        int road = DirectDriveObservation.RoadAndLimitsOffset;
        channels.Add((road + 9, small));             // front axle use
        channels.Add((road + 10, small));            // rear axle use
        int aero = DirectDriveObservation.AeroOffset;
        for (int i = 0; i < DirectDriveObservation.AeroSize; i++)
            channels.Add((aero + i, small));
        return channels.ToArray();
    }
}
