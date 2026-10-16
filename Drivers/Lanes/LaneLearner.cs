using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>
/// Simplix's per-section learning, offline. Lap after lap on the real
/// physics: where the car got into trouble, the plan spends less from the
/// braking zone through that point; where it was clean, a little more. The
/// best clean lap's factors are what the tuning file keeps.
/// </summary>
public sealed class LaneLearner
{
    public float TroubleErrorMeters { get; init; } = 1.2f;
    public float TroubleSlipRadians { get; init; } = 0.09f;
    public float CleanErrorMeters { get; init; } = 0.6f;
    public float CleanSlipRadians { get; init; } = 0.05f;
    public float LookBackMeters { get; init; } = 150f;
    public float LookAheadMeters { get; init; } = 20f;
    public float Penalty { get; init; } = 0.03f;
    public float SpinPenalty { get; init; } = 0.06f;
    public float Reward { get; init; } = 0.01f;
    public float MaximumFactor { get; init; } = 1.2f;
    public float MinimumFactor { get; init; } = 0.6f;
    public int SmoothStations { get; init; } = 8;

    /// <summary>Learn braking separately from cornering, as Simplix does.</summary>
    public bool LearnBraking { get; init; } = true;
    public float MaximumBrakeFactor { get; init; } = 1.08f;

    /// <summary>After this many laps without a new best, go back to the best and take smaller steps.</summary>
    public int Patience { get; init; } = 8;

    /// <summary>
    /// Also drive every candidate from a rolling start on cold tyres. The hot
    /// lap is what is timed; the cold one only has to come round clean, and
    /// wherever it does not, the factors there come down.
    /// </summary>
    public bool CheckColdTyres { get; init; } = false;
    public float StepDecay { get; init; } = 0.6f;

    /// <summary>
    /// Flying laps in a row each candidate has to drive clean, and is timed
    /// on the mean of. One lap is a lap on tyres just come up to temperature;
    /// the second and third are on the tyres the race is actually driven on.
    /// </summary>
    public int FlyingLaps { get; init; } = 1;

    /// <summary>
    /// Count a station where the driver wanted more wheel than the axles
    /// allowed as trouble. A plan the car can only follow at the very edge of
    /// its steering is one that a few percent less grip puts on the grass.
    /// </summary>
    public bool CapBoundIsTrouble { get; init; } = false;

    public sealed record Step(int Iteration, LaneLapResult Result, bool Kept);

    public (LaneTrackFile Best, LaneLapResult BestLap, List<Step> History) Learn(
        TrackData track,
        LaneTrackFile start,
        int iterations,
        LaneDriverGains? gains = null,
        LaneShares? shares = null,
        LaneId lane = LaneId.Free,
        int level = -1
    )
    {
        // Each grip level is learned on a road worth that much: the driver
        // reads its grip as that share and so drives off that level's factors.
        if (level < 0)
            level = LaneTrackFile.TopLevel;
        float roadGrip = LaneTrackFile.GripLevels[level];
        // A side lane is learned by driving it: the whole lap held on it.
        Action<Core.Racing.RaceCar, LaneDriver, float>? hold = lane == LaneId.Free
            ? null
            : (car, driver, t) =>
            {
                driver.TargetRange = 1f;
                driver.TargetSide = lane == LaneId.Left ? 1f : -1f;
            };
        LaneLapResult Drive(LaneTrackFile file) => LaneLapRunner.Run(track, file, gains, shares, onStep: hold, roadGrip: roadGrip, flyingLaps: FlyingLaps);
        LaneLapResult? DriveCold(LaneTrackFile file) => CheckColdTyres
            ? LaneLapRunner.Run(track, file, gains, shares, onStep: hold, tireConfig: new Core.Cars.TireConfig())
            : null;

        LaneTrackFile current = start.Clone();
        LaneSet lanes = current.LanesFor(track);
        LaneTrackFile best = current.Clone();
        LaneLapResult bestLap = Drive(best);
        LaneLapResult? bestCold = DriveCold(best);
        var history = new List<Step> { new(0, bestLap, true) };
        float bestTime = Clean(bestLap) && (bestCold?.Finished ?? true)
            ? Pace(bestLap)
            : float.PositiveInfinity;
        LaneLapResult last = bestLap;
        LaneLapResult? lastCold = bestCold;
        float step = 1f;
        int stale = 0;
        int failures = 0;

        for (int iteration = 1; iteration <= iterations; iteration++)
        {
            if (stale >= Patience)
            {
                current = best.Clone();
                lanes = current.LanesFor(track);
                last = bestLap;
                lastCold = bestCold;
                step *= StepDecay;
                stale = 0;
            }
            Adjust(lanes, current, last, lastCold, step, lane, level);
            // Three laps in a row that never got round: nothing local is
            // working, so come down everywhere until there is a lap to tune.
            failures = last.Finished ? 0 : failures + 1;
            if (failures >= 3)
            {
                foreach (float[] table in new[] { current.Factors(lane, level), current.BrakeFactors(lane, level) })
                    for (int i = 0; i < table.Length; i++)
                        table[i] = MathF.Max(MinimumFactor, table[i] * 0.97f);
                failures = 0;
            }
            last = Drive(current);
            lastCold = DriveCold(current);
            bool kept = Clean(last) &&
                        (lastCold?.Finished ?? true) && (lastCold?.OffTrackSeconds ?? 0f) == 0f &&
                        Pace(last) < bestTime;
            if (kept)
            {
                bestTime = Pace(last);
                best = current.Clone();
                bestLap = last;
                bestCold = lastCold;
                stale = 0;
            }
            else
            {
                stale++;
            }
            history.Add(new Step(iteration, last, kept));
        }

        return (best, bestLap, history);
    }

    /// <summary>Every lap driven, round without a spin and on the road.</summary>
    private bool Clean(LaneLapResult lap) =>
        lap.Finished && lap.OffTrackSeconds == 0f && lap.LapTimes.Count >= FlyingLaps;

    /// <summary>What a candidate is timed on: the mean of its flying laps.</summary>
    private static float Pace(LaneLapResult lap) =>
        lap.LapTimes.Count > 0 ? lap.LapTimes.Average() : lap.FlyingLapSeconds;

    private void Adjust(LaneSet lanes, LaneTrackFile file, LaneLapResult lap, LaneLapResult? cold, float step, LaneId lane, int level)
    {
        int n = lanes.Count;
        float[] lateral = file.Factors(lane, level);
        float[] brake = file.BrakeFactors(lane, level);
        var lateralScale = new float[n];
        var brakeScale = new float[n];
        Array.Fill(lateralScale, 1f);
        Array.Fill(brakeScale, 1f);
        var troubled = new bool[n];
        int back = (int)MathF.Ceiling(LookBackMeters / lanes.Spacing);
        int ahead = (int)MathF.Ceiling(LookAheadMeters / lanes.Spacing);
        bool sawLap = lap.StationError.Length == n && lap.FlyingLapSeconds > 0f && LearnBraking;

        // Trouble is charged to whichever the driver was doing on the way in:
        // stations it was braking at give up braking, the rest give up corner.
        void Punish(int at, float penalty, bool both)
        {
            for (int k = -back; k <= ahead; k++)
            {
                int i = lanes.Wrap(at + k);
                troubled[i] = true;
                bool braking = sawLap && lap.StationBraking[i];
                if (both || !sawLap || !braking || k >= 0)
                    lateralScale[i] = MathF.Min(lateralScale[i], 1f - penalty);
                if (both || braking)
                    brakeScale[i] = MathF.Min(brakeScale[i], 1f - penalty);
            }
        }

        if (lap.Spins > 0 || lap.FlyingLapSeconds <= 0f)
        {
            if (lap.SpinS >= 0f)
                Punish((int)MathF.Round(lanes.StationOf(lap.SpinS)), SpinPenalty * step, both: true);
        }

        // The cold lap only ever takes away: where it spun, and wherever its
        // flying lap was in trouble.
        if (cold is not null)
        {
            if (cold.SpinS >= 0f)
                Punish((int)MathF.Round(lanes.StationOf(cold.SpinS)), SpinPenalty * step, both: true);
            if (cold.StationError.Length == n && cold.FlyingLapSeconds > 0f)
            {
                for (int i = 0; i < n; i++)
                {
                    if (cold.StationError[i] > TroubleErrorMeters || cold.StationSlip[i] > TroubleSlipRadians ||
                        (cold.StationOff.Length == n && cold.StationOff[i]))
                        Punish(i, Penalty * step, both: true);
                }
            }
        }

        if (lap.StationError.Length == n && lap.FlyingLapSeconds > 0f)
        {
            for (int i = 0; i < n; i++)
            {
                if (lap.StationError[i] > TroubleErrorMeters || lap.StationSlip[i] > TroubleSlipRadians ||
                    (lap.StationOff.Length == n && lap.StationOff[i]) ||
                    (CapBoundIsTrouble && lap.StationCapBound.Length == n && lap.StationCapBound[i]))
                    Punish(i, Penalty * step, both: !LearnBraking);
            }
            for (int i = 0; i < n; i++)
            {
                if (!troubled[i] &&
                    lap.StationError[i] < CleanErrorMeters &&
                    lap.StationSlip[i] < CleanSlipRadians)
                {
                    lateralScale[i] = 1f + Reward * step;
                    if (LearnBraking && lap.StationBraking[i])
                        brakeScale[i] = 1f + Reward * step;
                }
            }
        }

        Apply(lanes, lateral, lateralScale, MinimumFactor, MaximumFactor);
        if (LearnBraking)
            Apply(lanes, brake, brakeScale, MinimumFactor, MaximumBrakeFactor);
    }

    private void Apply(LaneSet lanes, float[] factors, float[] scale, float minimum, float maximum)
    {
        int n = factors.Length;
        var next = new float[n];
        for (int i = 0; i < n; i++)
            next[i] = Math.Clamp(factors[i] * scale[i], minimum, maximum);

        // Smoothed, so the plan does not learn a sawtooth.
        for (int i = 0; i < n; i++)
        {
            float sum = 0f;
            for (int k = -SmoothStations; k <= SmoothStations; k++)
                sum += next[lanes.Wrap(i + k)];
            factors[i] = sum / (2 * SmoothStations + 1);
        }
    }

    /// <summary>
    /// Learns the steering feedforward the way the factors are learned: lap
    /// after lap, wherever the car sits off its lane, a little curvature is
    /// added the distance ahead of that point the car needs to act on it, so
    /// it turns in earlier or later rather than correcting after the fact.
    /// Kept only while the error falls and the lap stays clean and no slower.
    /// </summary>
    public (LaneTrackFile Best, LaneLapResult BestLap, List<Step> History) LearnSteering(
        TrackData track,
        LaneTrackFile start,
        int iterations,
        LaneDriverGains? gains = null,
        LaneId lane = LaneId.Free,
        float leadSeconds = 0.6f,
        float gain = 0.4f,
        float maximumCorrection = 0.01f
    )
    {
        Action<Core.Racing.RaceCar, LaneDriver, float>? hold = lane == LaneId.Free
            ? null
            : (car, driver, t) =>
            {
                driver.TargetRange = 1f;
                driver.TargetSide = lane == LaneId.Left ? 1f : -1f;
            };
        LaneLapResult Drive(LaneTrackFile file) => LaneLapRunner.Run(track, file, gains, onStep: hold);

        LaneTrackFile best = start.Clone();
        LaneSet lanes = best.LanesFor(track);
        LaneLapResult bestLap = Drive(best);
        var history = new List<Step> { new(0, bestLap, true) };
        int n = lanes.Count;
        float worstAllowed = bestLap.WorstErrorMeters + 0.05f;
        // A step size per station: where a correction overshoots and the error
        // changes side, that stretch takes smaller steps from then on, and the
        // rest of the lap is not held back by it.
        var local = new float[n];
        Array.Fill(local, gain);

        for (int iteration = 1; iteration <= iterations; iteration++)
        {
            LaneTrackFile candidate = best.Clone();
            float[] steer = candidate.Steer(lane);
            var delta = new float[n];
            for (int i = 0; i < n; i++)
            {
                float e = bestLap.StationSignedError.Length == n ? bestLap.StationSignedError[i] : 0f;
                if (e == 0f)
                    continue;
                float v = MathF.Max(bestLap.StationSpeed[i], 5f);
                float lead = MathF.Max(10f, v * leadSeconds);
                int at = lanes.Wrap(i - (int)MathF.Round(lead / lanes.Spacing));
                // Positive error is clockwise of the lane: more counter-clockwise
                // curvature beforehand pulls it back.
                delta[at] += local[i] * 2f * e / (lead * lead);
            }
            for (int i = 0; i < n; i++)
            {
                float sum = 0f;
                for (int k = -SmoothStations; k <= SmoothStations; k++)
                    sum += delta[lanes.Wrap(i + k)];
                steer[i] = Math.Clamp(steer[i] + sum / (2 * SmoothStations + 1), -maximumCorrection, maximumCorrection);
            }

            LaneLapResult lap = Drive(candidate);
            bool kept = lap.Finished && lap.OffTrackSeconds == 0f &&
                        lap.MeanAbsErrorMeters < bestLap.MeanAbsErrorMeters &&
                        lap.WorstErrorMeters <= worstAllowed &&
                        lap.FlyingLapSeconds <= bestLap.FlyingLapSeconds + 0.05f;

            // Wherever the error swapped sides by more than a hand's width, the
            // step there was too big: halve it, and its neighbours'.
            if (lap.StationSignedError.Length == n && bestLap.StationSignedError.Length == n)
            {
                for (int i = 0; i < n; i++)
                {
                    float before = bestLap.StationSignedError[i], after = lap.StationSignedError[i];
                    if (before * after < 0f && MathF.Abs(after) > 0.15f)
                    {
                        for (int k = -2 * SmoothStations; k <= 2 * SmoothStations; k++)
                            local[lanes.Wrap(i + k)] *= 0.7f;
                    }
                }
            }

            if (kept)
            {
                best = candidate;
                bestLap = lap;
            }
            else
            {
                for (int i = 0; i < n; i++)
                    local[i] *= 0.8f;
            }
            history.Add(new Step(iteration, lap, kept));
        }

        return (best, bestLap, history);
    }
}
