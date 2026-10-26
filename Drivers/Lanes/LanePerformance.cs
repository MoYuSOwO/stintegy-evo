using StintegyEVO.Core.Drivers;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// What a driver's ratings do to an otherwise identical plan, carried over
/// from the historical driver's performance state. Nothing is sampled at the
/// physics rate: form moves by session, by lap and by forty-metre segment,
/// from the driver's own seed, so the same driver on the same day drives the
/// same race.
///
/// Pace is how much of the plan the driver takes. Consistency is how far the
/// brake marker, the line and the speed wander from one segment to the next,
/// and how much a driver's form swings from lap to lap. Every driver keeps a
/// floor of that wandering, the best included, so no two races are the same
/// frame for frame.
/// </summary>
public sealed class LanePerformance
{
    public const float SegmentLengthMeters = 40f;
    private const float MaximumAbilityVolatility = 0.12f;
    private const float ErrorFilterSeconds = 0.35f;

    /// <summary>Pace across the whole scale: a rating of zero takes this share of the plan.</summary>
    public const float SlowestPaceShare = 0.92f;

    /// <summary>The wandering every driver keeps, as a share of a zero-consistency driver's.</summary>
    public const float NoiseFloor = 0.08f;

    /// <summary>
    /// What the best driver keeps of every misjudgement a driver with no feel
    /// makes: the grip each axle has, the lock a corner wants, the brake
    /// balance, the limit and the day's grip. Nobody reads a tyre exactly.
    /// </summary>
    public const float FeelFloor = 0.15f;

    // Feel for the grip each axle has left, as a share of it: how far one
    // segment's reading wanders, and the lap's lean on top, at no feel.
    private const float AxleGripMisreading = 0.05f;
    private const float LapAxleGripMisreading = 0.025f;

    // The lock a corner wants, as a share of it: a driver who knew the car
    // exactly would turn the wheel just so; a real one is a little over or
    // under and the line takes up the rest.
    private const float SteeringMisjudgement = 0.05f;

    // Tyre management: the wear a driver's hands add or save beyond what the
    // physics resolves, from a tenth less at the top of the scale to two
    // fifths more at the bottom, the default rating (80) the driver the
    // tyre was calibrated with.
    private const float CalibratedTyreManagement = 0.8f;
    private const float WearExcessPerRating = 0.5f;

    // And how closely the rung the pit wall asks for is the rung driven: a
    // lap's drift off it in rungs, most at no management, and a lean toward
    // driving harder than asked, which is what drivers do.
    private const float RungDriftWithNoManagement = 0.6f;
    private const float RungPushWithNoManagement = 0.3f;

    private const float BrakeMarkerSpreadMeters = 4.5f;
    private const float LateralSpreadMeters = 0.35f;
    private const float SpeedSpread = 0.012f;
    private const float MaximumExtraBrakePreview = 0.12f;

    private readonly DriverAbilities _abilities;
    private readonly Random _random;
    private int _lap = int.MinValue;
    private int _segment = int.MinValue;
    private float _sessionForm, _lapForm, _segmentForm;
    private float _targetMarker, _targetLateral, _targetSpeed, _targetOveruse, _targetBrakeBias;
    private float _lapBrakeBias, _gripBias;
    private float _lapFrontRead, _lapRearRead, _targetFrontRead, _targetRearRead, _targetSteer;

    // Car control, as the historical driver had it: how near the limit a
    // driver leaves the tyre wanders from one segment to the next, by up to
    // this much for a driver with no feel at all, and a driver with less feel
    // sits a little further from it on average, because they cannot tell
    // where it is. The wandering is what puts them over it now and then.
    private const float LimitJitter = 0.06f;
    private const float LimitMarginWithNoFeel = 0.03f;
    private const float MaximumLimitError = 0.15f;

    // And now and then a driver without the feel for it gets a corner wrong:
    // in too fast, by a few percent to a tenth. How often is what car
    // control buys; a driver with all of it never does.
    private const float MistakesPerSegmentWithNoFeel = 0.012f;
    private const float SmallestMistake = 0.04f;
    private const float LargestMistake = 0.10f;

    // Brake balance, also from the historical driver: a lap's bias error and
    // a segment's on top of it, as a share of the braking moved forward.
    private const float LapBrakeBiasError = 0.025f;
    private const float SegmentBrakeBiasJitter = 0.035f;
    private const float MaximumBrakeBiasError = 0.07f;

    // Adaptability: how long the driver takes to notice the grip has changed,
    // and how far off their reading of it can be on a given lap.
    private const float SlowestGripResponseSeconds = 5f;
    private const float FastestGripResponseSeconds = 0.5f;
    private const float GripMisreading = 0.03f;
    private float _perceivedGrip = float.NaN;

    public LanePerformance(DriverProfile profile)
    {
        ArgumentNullException.ThrowIfNull(profile);
        _abilities = profile.Abilities;
        _random = new Random(unchecked((int)(profile.RandomSeed ^ (ulong)StableHash(profile.Id))));
        _sessionForm = Normal();
        _lapForm = Normal();
        _segmentForm = Normal();
    }

    /// <summary>Share of the plan taken this lap.</summary>
    public float PaceShare { get; private set; } = 1f;

    /// <summary>Metres the speed table is read ahead of the car: positive brakes later.</summary>
    public float BrakeMarkerMeters { get; private set; }

    /// <summary>Metres added to the lane offset, positive the track's positive side.</summary>
    public float LateralOffsetMeters { get; private set; }

    /// <summary>Fraction added to the target speed through this segment.</summary>
    public float SpeedError { get; private set; }

    /// <summary>The consistency error before the floor, 0 for a perfect rating.</summary>
    public float ConsistencyError => MathF.Max(NoiseFloor, MathF.Pow(1f - Score(_abilities.Consistency), 1.2f));

    /// <summary>
    /// The extra look-ahead into braking zones a slower driver takes: they
    /// brake earlier as well as corner slower, which is where a lap is lost.
    /// </summary>
    public float ExtraBrakePreviewSeconds { get; private set; }

    /// <summary>Car control as drawn this lap, 0..1.</summary>
    public float Control { get; private set; } = 1f;

    /// <summary>How long a slide takes to be felt fully: a filter on what the driver perceives.</summary>
    public float PerceptionSeconds => 0.25f + (0.04f - 0.25f) * Control;

    /// <summary>
    /// From the moment a slide starts to the moment the hands move. Reactions,
    /// not control: the slow driver is clean until something happens, then late.
    /// </summary>
    public float ReactionSeconds => 0.30f + (0.05f - 0.30f) * Score(_abilities.Reactions);

    /// <summary>
    /// How much more of the tyre than the plan this driver is using in this
    /// segment: 0 at full control; for less feel, a little less on average
    /// and wandering either side of it.
    /// </summary>
    public float LimitOveruse { get; private set; }

    /// <summary>How much faster than the plan this segment is being taken, by mistake: 0 nearly always.</summary>
    public float CornerOverstep { get; private set; }

    /// <summary>
    /// How long the brake balance takes to follow what the axles want, s: a
    /// driver with feel turns the dial with the corner coming, one without
    /// is a beat behind every change. Never instant: the floor is a driver's.
    /// </summary>
    public float BrakeDialSeconds => 0.06f + (0.35f - 0.06f) * (1f - Control);

    /// <summary>The brake balance this driver has set, as a share moved forward of the best split.</summary>
    public float FrontBrakeBiasOffset { get; private set; }

    /// <summary>
    /// How much more grip than is there the driver feels at each axle, as a
    /// share: positive and the tyre is asked for more than it has.
    /// </summary>
    public float FrontGripRead { get; private set; }
    public float RearGripRead { get; private set; }

    /// <summary>How much more lock than the corner wants the hands put on, as a share.</summary>
    public float SteeringRead { get; private set; }

    /// <summary>The share more wear this driver's hands put on the tyres than the reference driver's.</summary>
    public float TyreWearExcess => (CalibratedTyreManagement - Score(_abilities.TireManagement)) * WearExcessPerRating;

    /// <summary>
    /// This lap, how far off the asked-for rung the driving is, in rungs:
    /// positive harder. The tyre rung and the energy rung each their own.
    /// </summary>
    public float TyreRungDrift { get; private set; }
    public float EnergyRungDrift { get; private set; }

    /// <summary>Adaptability as drawn this lap, 0..1.</summary>
    public float Adaptation { get; private set; } = 1f;

    /// <summary>
    /// The grip the driver believes the car has, for a grip that really is
    /// <paramref name="actual"/>: late to follow it when it changes, and on
    /// some laps read a little high or low.
    /// </summary>
    public float PerceiveGrip(float actual, float dt)
    {
        float target = actual * (1f + _gripBias);
        if (float.IsNaN(_perceivedGrip))
            _perceivedGrip = target;
        float seconds = SlowestGripResponseSeconds + (FastestGripResponseSeconds - SlowestGripResponseSeconds) * Adaptation;
        _perceivedGrip += (target - _perceivedGrip) * (1f - MathF.Exp(-MathF.Max(0f, dt) / seconds));
        return _perceivedGrip;
    }

    /// <summary>How firmly this catch is made, drawn when the slide is felt.</summary>
    public float RecoveryGain { get; private set; } = 1f;

    /// <summary>A slide was felt: draw how well this one is handled.</summary>
    public void BeginRecovery()
    {
        float eventForm = 0.25f * _sessionForm + 0.35f * _lapForm + 0.9f * Normal();
        float control = Sample(_abilities.CarControl, eventForm);
        RecoveryGain = Math.Clamp(0.72f + 0.28f * control + Normal() * 0.18f * (1f - control), 0.55f, 1.12f);
    }

    /// <summary>How much of the pace scale this lap's draw sits at, 0..1.</summary>
    public float PaceDraw { get; private set; } = 1f;

    public void Update(int lap, float s, float dt)
    {
        if (lap != _lap)
        {
            _lapForm = _lap == int.MinValue ? _lapForm : 0.6f * _lapForm + 0.8f * Normal();
            _lap = lap;
            float pace = Sample(_abilities.Pace, 0.38f * _sessionForm + 0.92f * _lapForm);
            PaceDraw = pace;
            Control = Sample(_abilities.CarControl, 0.38f * _sessionForm + 0.92f * _lapForm);
            PaceShare = SlowestPaceShare + (1f - SlowestPaceShare) * pace;
            ExtraBrakePreviewSeconds = (1f - pace) * MaximumExtraBrakePreview;
            float lapFeel = Feel(MathF.Pow(1f - Control, 1.25f));
            _lapBrakeBias = Normal() * LapBrakeBiasError * lapFeel;
            _lapFrontRead = Normal() * LapAxleGripMisreading * lapFeel;
            _lapRearRead = Normal() * LapAxleGripMisreading * lapFeel;

            float unmanaged = Feel(1f - Score(_abilities.TireManagement));
            TyreRungDrift = RungPushWithNoManagement * (1f - Score(_abilities.TireManagement)) + Normal() * RungDriftWithNoManagement * unmanaged * 0.5f;
            EnergyRungDrift = RungPushWithNoManagement * (1f - Score(_abilities.TireManagement)) + Normal() * RungDriftWithNoManagement * unmanaged * 0.5f;

            float longRun = 0.6f * _sessionForm + 0.8f * _lapForm;
            Adaptation = Sample(_abilities.Adaptability, longRun);
            _gripBias = Normal() * GripMisreading * Feel(MathF.Pow(1f - Adaptation, 1.2f));
        }

        int segment = (int)MathF.Floor(MathF.Max(0f, s) / SegmentLengthMeters);
        if (segment != _segment)
        {
            _segmentForm = 0.25f * _segmentForm + 0.97f * Normal();
            _segment = segment;
            float error = ConsistencyError;
            _targetMarker = Normal() * BrakeMarkerSpreadMeters * error;
            _targetLateral = Normal() * LateralSpreadMeters * error;
            _targetSpeed = Normal() * SpeedSpread * error;
            float limitError = Feel(MathF.Pow(1f - Control, 1.25f));
            _targetFrontRead = _lapFrontRead + Normal() * AxleGripMisreading * limitError;
            _targetRearRead = _lapRearRead + Normal() * AxleGripMisreading * limitError;
            _targetSteer = Normal() * SteeringMisjudgement * limitError;
            float wander = 0.4f + 0.6f * Math.Clamp(error, 0f, 1f);
            _targetOveruse = Math.Clamp(
                -LimitMarginWithNoFeel * (1f - Control) + Normal() * LimitJitter * limitError * wander,
                -MaximumLimitError,
                MaximumLimitError);
            // From the rating, not the day's draw of it: a perfect rating
            // still wanders a hair below one on a bad lap, and that is not a
            // driver who gets corners wrong.
            float mistakeChance = MistakesPerSegmentWithNoFeel * MathF.Pow(1f - Score(_abilities.CarControl), 1.25f);
            // Drawn only for a driver who can make one, so a perfect driver's
            // day is the same draw for draw as it was before mistakes existed.
            CornerOverstep = mistakeChance > 0f && (float)_random.NextDouble() < mistakeChance
                ? SmallestMistake + (LargestMistake - SmallestMistake) * (float)_random.NextDouble()
                : 0f;
            _targetBrakeBias = Math.Clamp(
                _lapBrakeBias + Normal() * SegmentBrakeBiasJitter * limitError * (0.2f + 0.8f * Math.Clamp(error, 0f, 1f)),
                -MaximumBrakeBiasError,
                MaximumBrakeBiasError);
        }

        float response = 1f - MathF.Exp(-MathF.Max(0f, dt) / ErrorFilterSeconds);
        BrakeMarkerMeters += (_targetMarker - BrakeMarkerMeters) * response;
        LateralOffsetMeters += (_targetLateral - LateralOffsetMeters) * response;
        SpeedError += (_targetSpeed - SpeedError) * response;
        LimitOveruse += (_targetOveruse - LimitOveruse) * response;
        FrontBrakeBiasOffset += (_targetBrakeBias - FrontBrakeBiasOffset) * response;
        FrontGripRead += (_targetFrontRead - FrontGripRead) * response;
        RearGripRead += (_targetRearRead - RearGripRead) * response;
        SteeringRead += (_targetSteer - SteeringRead) * response;
    }

    /// <summary>A rating as drawn on the day: a low consistency lets it swing with form.</summary>
    private float Sample(float rating, float form)
    {
        float skill = Score(rating);
        float volatility = MaximumAbilityVolatility *
                           ConsistencyError *
                           (1.2f - 0.3f * skill);
        return Math.Clamp(skill + volatility * Math.Clamp(form, -3f, 3f), 0f, 1f);
    }

    private float Normal()
    {
        float u1 = MathF.Max(1e-7f, (float)_random.NextDouble());
        float u2 = (float)_random.NextDouble();
        return Math.Clamp(MathF.Sqrt(-2f * MathF.Log(u1)) * MathF.Cos(2f * MathF.PI * u2), -3f, 3f);
    }

    private static float Score(float rating) => Math.Clamp(rating * 0.01f, 0f, 1f);

    /// <summary>A misjudgement's size, from a driver's share of the worst: never below the floor.</summary>
    private static float Feel(float share) => FeelFloor + (1f - FeelFloor) * Math.Clamp(share, 0f, 1f);

    private static uint StableHash(string value)
    {
        uint hash = 2166136261;
        foreach (char c in value)
            hash = (hash ^ c) * 16777619;
        return hash;
    }
}
