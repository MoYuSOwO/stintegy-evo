using System;

namespace StintegyEVO.Core.Drivers;

/// <summary>
/// Seeded, slowly varying execution state. No random value is sampled at the
/// physics-frame rate: form changes by session, lap, track segment, or a
/// discrete loss-of-control event.
/// </summary>
internal sealed class DriverPerformanceState
{
    private const float SegmentLengthMeters = 40f;
    private const float MaximumAbilityVolatility = 0.12f;
    private const float ErrorFilterTimeSeconds = 0.35f;
    private const float MaximumLapBrakeBiasFraction = 0.025f;
    private const float MaximumSegmentBrakeJitterFraction = 0.035f;
    private const float MaximumBrakeBiasErrorFraction = 0.07f;

    private readonly DriverAbilities _abilities;
    private readonly DriverRandom _random;
    private int _lap = int.MinValue;
    private int _segment = int.MinValue;
    private float _targetBrakeMarkerErrorMeters;
    private float _targetLateralErrorMeters;
    private float _targetLocalSpeedErrorFraction;
    private float _lapBrakeBiasFraction;
    private float _targetFrontBrakeBiasOffset;
    private float _gripEstimateBias;
    private bool _gripInitialized;

    public DriverPerformanceState(DriverProfile profile, string carId)
    {
        _abilities = profile.Abilities;
        ulong seed = profile.RandomSeed ^ StableHash(profile.Id) ^ RotateLeft(StableHash(carId), 23);
        _random = new DriverRandom(seed);
    }

    public int PlanningRevision { get; private set; }
    public float SessionForm { get; private set; }
    public float LapForm { get; private set; }
    public float SegmentForm { get; private set; }
    public float PlanningPace { get; private set; } = 1f;
    public float EffectivePace { get; private set; } = 1f;
    public float EffectiveControl { get; private set; } = 1f;
    public float EffectiveTireManagement { get; private set; } = 0.8f;
    public float EffectiveAdaptability { get; private set; } = 1f;
    /// <summary>
    /// The share of a corner this driver gets back out of the tyres.
    ///
    /// The bottom of the range is what sets how much a rating is worth, and it
    /// is set from lap time rather than from anything about a tyre, because lap
    /// time is the only part of it anybody can check. Across the whole scale it
    /// is two per cent, which on a hundred second circuit is a couple of
    /// seconds: about the spread of a real single-seater grid from the front
    /// row to the back, and a little more than a set of tyres is worth, so a
    /// good driver is an advantage without being the whole race.
    ///
    /// Worth remembering when handing out ratings that this is the full scale.
    /// A grid drawn from a narrow band of it will be a grid of drivers nobody
    /// can tell apart.
    /// </summary>
    public float PaceEfficiency => Lerp(0.95f, 1f, EffectivePace);
    public float BrakeMarkerErrorMeters { get; private set; }
    public float LateralTargetErrorMeters { get; private set; }
    public float LocalSpeedErrorFraction { get; private set; }
    public float FrontBrakeBiasOffset { get; private set; }
    public float EstimatedGrip { get; private set; }
    public float EstimatedGripScale { get; private set; } = 1f;
    public float ActualGrip { get; private set; }
    public float TireEnergyEfficiency => TireEnergyFactor(EffectiveTireManagement);
    public bool IsRecovering { get; private set; }
    public float ControlGainScale { get; private set; } = 1f;

    public void Initialize(int lap, float s, float actualGrip)
    {
        SessionForm = _random.NextNormal();
        LapForm = _random.NextNormal();
        SegmentForm = _random.NextNormal();
        _lap = lap;
        _segment = SegmentIndex(s);
        UpdateLapAbilities();
        UpdateSegmentAbilities();
        ActualGrip = MathF.Max(actualGrip, 1e-3f);
        EstimatedGrip = ActualGrip;
        EstimatedGripScale = 1f;
        _gripInitialized = true;
    }

    public void Update(int lap, float s, float actualGrip, float dt)
    {
        if (!_gripInitialized)
            Initialize(lap, s, actualGrip);

        if (lap != _lap)
        {
            _lap = lap;
            LapForm = 0.6f * LapForm + MathF.Sqrt(1f - 0.6f * 0.6f) * _random.NextNormal();
            UpdateLapAbilities();
            PlanningRevision++;
        }

        int segment = SegmentIndex(s);
        if (segment != _segment)
        {
            _segment = segment;
            SegmentForm = 0.25f * SegmentForm + MathF.Sqrt(1f - 0.25f * 0.25f) * _random.NextNormal();
            UpdateSegmentAbilities();
        }

        float response = 1f - MathF.Exp(-MathF.Max(0f, dt) / ErrorFilterTimeSeconds);
        BrakeMarkerErrorMeters = Lerp(BrakeMarkerErrorMeters, _targetBrakeMarkerErrorMeters, response);
        LateralTargetErrorMeters = Lerp(LateralTargetErrorMeters, _targetLateralErrorMeters, response);
        LocalSpeedErrorFraction = Lerp(LocalSpeedErrorFraction, _targetLocalSpeedErrorFraction, response);
        FrontBrakeBiasOffset = Lerp(
            FrontBrakeBiasOffset,
            _targetFrontBrakeBiasOffset,
            response
        );

        UpdateGripEstimate(actualGrip, dt);
    }

    public void UpdateControlEvent(bool unstable)
    {
        if (unstable && !IsRecovering)
        {
            IsRecovering = true;
            float eventForm = 0.25f * SessionForm + 0.35f * LapForm + 0.90f * _random.NextNormal();
            EffectiveControl = SampleAbility(_abilities.CarControl, eventForm);
            float errorRoom = 1f - EffectiveControl;
            ControlGainScale = Math.Clamp(
                Lerp(0.72f, 1f, EffectiveControl) + _random.NextNormal() * 0.18f * errorRoom,
                0.55f,
                1.12f
            );
        }
        else if (!unstable && IsRecovering)
        {
            IsRecovering = false;
            ControlGainScale = 1f;
        }
    }

    private void UpdateLapAbilities()
    {
        float lapAbilityForm = 0.38f * SessionForm + 0.92f * LapForm;
        PlanningPace = SampleAbility(_abilities.Pace, lapAbilityForm);
        EffectiveControl = SampleAbility(_abilities.CarControl, lapAbilityForm);
        float longRunForm = 0.6f * SessionForm + 0.8f * LapForm;
        EffectiveTireManagement = SampleAbility(_abilities.TireManagement, longRunForm);
        EffectiveAdaptability = SampleAbility(_abilities.Adaptability, longRunForm);

        // Brake allocation is an execution skill rather than another manager-
        // facing rating. Car control sets the typical error; consistency decides
        // how much it wanders from one track segment to the next.
        float brakeControlError = MathF.Pow(1f - EffectiveControl, 1.25f);
        _lapBrakeBiasFraction = _random.NextNormal() *
                                MaximumLapBrakeBiasFraction *
                                brakeControlError;

        float adaptabilityError = MathF.Pow(1f - EffectiveAdaptability, 1.2f);
        _gripEstimateBias = _random.NextNormal() * 0.03f * adaptabilityError;
    }

    private void UpdateSegmentAbilities()
    {
        float segmentAbilityForm = 0.35f * SessionForm + 0.55f * LapForm + 0.76f * SegmentForm;
        EffectivePace = SampleAbility(_abilities.Pace, segmentAbilityForm);

        float consistencyError = MathF.Pow(1f - Score(_abilities.Consistency), 1.5f);
        _targetBrakeMarkerErrorMeters = _random.NextNormal() * 1.5f * consistencyError;
        _targetLateralErrorMeters = _random.NextNormal() * 0.25f * consistencyError;
        _targetLocalSpeedErrorFraction = _random.NextNormal() * 0.012f * consistencyError;

        float brakeControlError = MathF.Pow(1f - EffectiveControl, 1.25f);
        float brakeJitterScale = Lerp(0.2f, 1f, consistencyError);
        _targetFrontBrakeBiasOffset = Math.Clamp(
            _lapBrakeBiasFraction +
            _random.NextNormal() *
            MaximumSegmentBrakeJitterFraction *
            brakeControlError *
            brakeJitterScale,
            -MaximumBrakeBiasErrorFraction,
            MaximumBrakeBiasErrorFraction
        );
    }

    private void UpdateGripEstimate(float actualGrip, float dt)
    {
        ActualGrip = MathF.Max(actualGrip, 1e-3f);
        if (EffectiveAdaptability >= 0.999f)
        {
            EstimatedGrip = ActualGrip;
            EstimatedGripScale = 1f;
            return;
        }

        float target = ActualGrip * (1f + _gripEstimateBias);
        float responseTime = Lerp(5f, 0.5f, EffectiveAdaptability);
        float response = 1f - MathF.Exp(-MathF.Max(0f, dt) / responseTime);
        EstimatedGrip = Lerp(EstimatedGrip, target, response);
        EstimatedGripScale = Math.Clamp(EstimatedGrip / ActualGrip, 0.9f, 1.1f);
    }

    private float SampleAbility(float rating, float form)
    {
        float skill = Score(rating);
        float consistency = Score(_abilities.Consistency);
        float volatility = MaximumAbilityVolatility *
                           MathF.Pow(1f - consistency, 1.5f) *
                           Lerp(1.2f, 0.9f, skill);
        return Math.Clamp(skill + volatility * Math.Clamp(form, -3f, 3f), 0f, 1f);
    }

    private static float TireEnergyFactor(float skill)
    {
        const float neutralSkill = 0.8f;
        return skill <= neutralSkill
            ? Lerp(1.04f, 1f, skill / neutralSkill)
            : Lerp(1f, 0.97f, (skill - neutralSkill) / (1f - neutralSkill));
    }

    private static int SegmentIndex(float s) => (int)MathF.Floor(MathF.Max(0f, s) / SegmentLengthMeters);
    private static float Score(float rating) => Math.Clamp(rating * 0.01f, 0f, 1f);
    private static float Lerp(float from, float to, float t) => from + (to - from) * Math.Clamp(t, 0f, 1f);

    private static ulong StableHash(string value)
    {
        const ulong offset = 14695981039346656037UL;
        const ulong prime = 1099511628211UL;
        ulong hash = offset;
        foreach (char character in value)
        {
            hash ^= character;
            hash *= prime;
        }
        return hash;
    }

    private static ulong RotateLeft(ulong value, int count) =>
        (value << count) | (value >> (64 - count));

    private sealed class DriverRandom
    {
        private ulong _state;
        private bool _hasSpare;
        private float _spare;

        public DriverRandom(ulong seed)
        {
            _state = seed == 0UL ? 0x9E3779B97F4A7C15UL : seed;
        }

        public float NextNormal()
        {
            if (_hasSpare)
            {
                _hasSpare = false;
                return _spare;
            }

            float u1 = MathF.Max(NextUnit(), 1e-7f);
            float u2 = NextUnit();
            float magnitude = MathF.Sqrt(-2f * MathF.Log(u1));
            float angle = 2f * MathF.PI * u2;
            _spare = Math.Clamp(magnitude * MathF.Sin(angle), -3f, 3f);
            _hasSpare = true;
            return Math.Clamp(magnitude * MathF.Cos(angle), -3f, 3f);
        }

        private float NextUnit()
        {
            ulong value = NextUInt64();
            return (float)((value >> 40) * (1.0 / (1UL << 24)));
        }

        private ulong NextUInt64()
        {
            ulong x = _state;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            _state = x;
            return x * 2685821657736338717UL;
        }
    }
}
