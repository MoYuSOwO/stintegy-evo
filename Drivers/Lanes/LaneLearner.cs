using StintegyEVO.Core.Cars;
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
    /// <summary>
    /// A slide: the car's sideslip, net of the turn's geometry, past the
    /// angle a tyre's grip peaks at. It was 0.09 rad, set when the hands
    /// held the tyres to nine tenths and a car at its limit stood at three
    /// or four degrees; held to what the physics says a tyre holds, the
    /// rear stands at five to seven in every fast corner with no slide at
    /// all (RearSlideSeverity nought), and the learning slowed them.
    /// </summary>
    public float TroubleSlipRadians { get; init; } = Tire.PeakSlipAngleRadians;
    public float CleanErrorMeters { get; init; } = 0.6f;
    /// <summary>Under the angle a tyre gives what it holds at: room left to go faster.</summary>
    public float CleanSlipRadians { get; init; } = Tire.SlipAngleAtShare(Tire.SustainablePeakShare);
    public float LookBackMeters { get; init; } = 150f;
    public float LookAheadMeters { get; init; } = 20f;
    public float Penalty { get; init; } = 0.03f;
    public float SpinPenalty { get; init; } = 0.06f;
    /// <summary>
    /// What a station off the road, or this far off its lane, gives up in
    /// one round, from both the corner and the braking. Zandvoort's left lane
    /// ran 10 m wide out of a corner it braked into and needed the corner 20
    /// to 30% slower, and at the 3% of a small miss it took longer than the
    /// learning had; giving up only the braking there spun the car.
    /// </summary>
    public float SeverePenalty { get; init; } = 0.1f;
    public float SevereErrorMeters { get; init; } = 2f;
    public float Reward { get; init; } = 0.01f;
    public float MaximumFactor { get; init; } = 1.2f;
    public float MinimumFactor { get; init; } = 0.6f;
    public int SmoothStations { get; init; } = 8;

    /// <summary>Learn braking separately from cornering, as Simplix does.</summary>
    public bool LearnBraking { get; init; } = true;
    public float MaximumBrakeFactor { get; init; } = 1.08f;

    /// <summary>After this many laps without a new best, go back to the best and take smaller steps.</summary>
    public int Patience { get; init; } = 8;

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

    /// <summary>
    /// Keep a candidate only if its lap is nowhere in trouble, by the same
    /// measure Adjust punishes (an error past TroubleErrorMeters, a slip past
    /// TroubleSlipRadians), not merely on the road. Kept on the road alone,
    /// the fastest lap was the one that ran three metres off its lane where
    /// the tracker had nothing left: quick alone, and in traffic a car that
    /// is not where it says it is. Held to the trouble line, the margin a
    /// corner needs is learned there, corner by corner.
    /// </summary>
    public bool BestMustBeTroubleFree { get; init; } = false;

    public sealed record Step(int Iteration, LaneLapResult Result, bool Kept);

    /// <summary>Told one line after every lap the learner drives, for a log to follow it by.</summary>
    public Action<string>? Progress { get; init; }

    /// <summary>How long a stretch of road is judged on its own by <see cref="LearnSectors"/>.</summary>
    public float SectorMeters { get; init; } = 200f;

    /// <summary>
    /// The pace is kept stretch by stretch rather than lap by lap, as the
    /// autonomous racing teams that learn a pace map do (a performance level
    /// per sector, raised, held or lowered on what the car did there). Keeping
    /// a whole lap kept a candidate only if the whole lap was better, so one
    /// corner in trouble threw away what every other corner had learned that
    /// lap, and of 121 laps it kept two to twenty.
    /// Here each sector keeps the candidate's factors if it and the sector
    /// after it (where a faster exit arrives) are out of trouble and it is
    /// quicker, or if it was in trouble and no longer is; the rest go back.
    /// The combination is then driven, so what the next lessons are drawn
    /// from is a lap the factors really make, and the best of those that is
    /// nowhere in trouble is what is returned.
    /// </summary>
    public (LaneTrackFile Best, LaneLapResult BestLap, List<Step> History) LearnSectors(
        TrackData track,
        LaneTrackFile start,
        int iterations,
        LaneDriverGains? gains = null,
        LaneShares? shares = null,
        LaneId lane = LaneId.Free,
        int level = -1
    )
    {
        if (level < 0)
            level = LaneTrackFile.TopLevel;
        float roadGrip = LaneTrackFile.GripLevels[level];
        Action<Core.Racing.RaceCar, LaneDriver, float>? hold = lane == LaneId.Free
            ? null
            : (car, driver, t) =>
            {
                driver.TargetRange = 1f;
                driver.TargetSide = lane == LaneId.Left ? 1f : -1f;
            };
        LaneLapResult Drive(LaneTrackFile file) => LaneLapRunner.Run(track, file, gains, shares, onStep: hold, roadGrip: roadGrip, flyingLaps: FlyingLaps);

        LaneTrackFile current = start.Clone();
        LaneSet lanes = current.LanesFor(track);
        int n = lanes.Count;
        int length = Math.Max(1, (int)MathF.Round(SectorMeters / lanes.Spacing));
        int sectors = (n + length - 1) / length;

        LaneLapResult lap = Drive(current);
        var history = new List<Step> { new(0, lap, Clean(lap) && TroubleFree(lap)) };
        LaneTrackFile best = current.Clone();
        LaneLapResult bestLap = lap;
        (int Trouble, float Time) bestScore = Score(lap);
        float bestTime = Clean(lap) && TroubleFree(lap) ? Pace(lap) : float.PositiveInfinity;
        float step = 1f;
        int stale = 0;
        int stillFor = 0;

        for (int iteration = 1; iteration <= iterations; iteration++)
        {
            if (stale >= Patience)
            {
                step *= StepDecay;
                stale = 0;
            }
            LaneTrackFile candidate = current.Clone();
            Adjust(lanes, candidate, lap, null, step, lane, level);
            LaneLapResult tried = Drive(candidate);

            int kept = 0;
            if (tried.Finished && !lap.Finished)
            {
                // Round at all is better than not round: the whole candidate,
                // and its lessons from then on.
                current = candidate;
                kept = sectors;
            }
            else if (tried.Finished && tried.StationError.Length == n && lap.StationError.Length == n)
            {
                float[] lateral = current.Factors(lane, level), brake = current.BrakeFactors(lane, level);
                float[] newLateral = candidate.Factors(lane, level), newBrake = candidate.BrakeFactors(lane, level);
                for (int k = 0; k < sectors; k++)
                {
                    int next = (k + 1) % sectors;
                    var before = SectorTrouble(lap, k, length, n);
                    var after = SectorTrouble(tried, k, length, n);
                    int nextBefore = SectorTrouble(lap, next, length, n).Stations;
                    int nextAfter = SectorTrouble(tried, next, length, n).Stations;
                    // The sector after is where a faster exit arrives: a clean
                    // sector may not make it worse for its own sake. One in
                    // trouble may hand it on, no more of it than it sheds: a
                    // car slowed for a corner it left the road at can still
                    // go off a little further on, and that one is learned next
                    // round, where vetoing it kept the first for good.
                    if (before.Stations == 0 ? nextAfter > nextBefore : after.Stations + nextAfter > before.Stations + nextBefore)
                        continue;
                    bool take = before.Stations == 0
                        // Clean: stays clean, and is quicker.
                        ? after.Stations == 0 &&
                          SectorSeconds(lanes, tried, k, length, n) < SectorSeconds(lanes, lap, k, length, n)
                        // In trouble: less of it. A step that only eases the
                        // trouble is kept, or the next one starts from where
                        // this one did and the sector never gets out of it.
                        : after.Stations < before.Stations ||
                          (after.Stations == before.Stations && after.Worst < before.Worst - 0.05f);
                    if (!take)
                        continue;
                    for (int i = k * length; i < Math.Min(n, (k + 1) * length); i++)
                    {
                        lateral[i] = newLateral[i];
                        brake[i] = newBrake[i];
                    }
                    kept++;
                }
            }
            else if (!tried.Finished)
            {
                // A lap that did not get round says where it went wrong, not
                // what was right: take its lesson onto the current factors.
                Adjust(lanes, current, tried, null, step, lane, level);
            }

            lap = Drive(current);
            // Best is the lap with the fewest stations in trouble, then the
            // quickest: a run that never gets wholly clean still keeps the
            // closest it came rather than going back to where it started.
            (int Trouble, float Time) score = Score(lap);
            bool better = score.Trouble < bestScore.Trouble ||
                          (score.Trouble == bestScore.Trouble && score.Time < bestScore.Time);
            if (better)
            {
                bestScore = score;
                best = current.Clone();
                bestLap = lap;
                if (score.Trouble == 0)
                    bestTime = score.Time;
            }
            stale = kept > 0 ? 0 : stale + 1;
            history.Add(new Step(iteration, lap, better));
            // The same lap again and again: the sectors still being kept
            // change nothing the car does. Rounds past that are time spent
            // learning nothing, and the next phase is where the change is.
            int steady = history.Count >= 2 && SameLap(history[^2].Result, lap) ? stillFor + 1 : 0;
            stillFor = steady;
            Progress?.Invoke(
                $"{lane} pace {iteration}/{iterations}: sectors kept {kept}/{sectors}, " +
                $"lap {(lap.Finished ? Pace(lap).ToString("F2") : "unfinished")} err {lap.WorstErrorMeters:F2} " +
                $"{(TroubleFree(lap) ? "clean" : "trouble " + TroubleCauses(lap))}, best {(float.IsFinite(bestTime) ? bestTime.ToString("F2") : "none")}");
            if (stillFor >= SteadyRounds)
            {
                Progress?.Invoke($"{lane} pace: the lap has not changed for {SteadyRounds} rounds, done");
                break;
            }
        }

        return (best, bestLap, history);
    }

    /// <summary>How many stations each kind of trouble was found at, for the log.</summary>
    private string TroubleCauses(LaneLapResult lap)
    {
        int error = 0, slip = 0, off = 0, cap = 0;
        for (int i = 0; i < lap.StationError.Length; i++)
        {
            if (lap.StationError[i] > TroubleErrorMeters) error++;
            if (lap.StationSlip.Length > i && lap.StationSlip[i] > TroubleSlipRadians) slip++;
            if (lap.StationOff.Length > i && lap.StationOff[i]) off++;
            if (CapBoundIsTrouble && lap.StationCapBound.Length > i && lap.StationCapBound[i]) cap++;
        }
        return $"(error {error}, slip {slip}, off {off}, lock {cap})";
    }

    /// <summary>Rounds in a row the lap may come back the same before a phase stops.</summary>
    public int SteadyRounds { get; init; } = 3;

    private static bool SameLap(LaneLapResult a, LaneLapResult b) =>
        a.Finished == b.Finished &&
        MathF.Abs(a.FlyingLapSeconds - b.FlyingLapSeconds) < 0.005f &&
        MathF.Abs(a.WorstErrorMeters - b.WorstErrorMeters) < 0.005f;

    /// <summary>Stations in trouble, every one if the lap did not get round on the road, then the pace.</summary>
    private (int Trouble, float Time) Score(LaneLapResult lap)
    {
        if (!OnTheRoad(lap) || lap.StationError.Length == 0)
            return (int.MaxValue, float.PositiveInfinity);
        int trouble = 0;
        for (int i = 0; i < lap.StationError.Length; i++)
            if (StationInTrouble(lap, i))
                trouble++;
        return (trouble, Pace(lap));
    }

    private bool StationInTrouble(LaneLapResult lap, int i) =>
        lap.StationError[i] > TroubleErrorMeters ||
        (lap.StationSlip.Length > i && lap.StationSlip[i] > TroubleSlipRadians) ||
        (lap.StationOff.Length > i && lap.StationOff[i]) ||
        (CapBoundIsTrouble && lap.StationCapBound.Length > i && lap.StationCapBound[i]);

    /// <summary>How many of a sector's stations are in trouble, and its worst error.</summary>
    private (int Stations, float Worst) SectorTrouble(LaneLapResult lap, int sector, int length, int n)
    {
        int stations = 0;
        float worst = 0f;
        for (int i = sector * length; i < Math.Min(n, (sector + 1) * length); i++)
        {
            if (StationInTrouble(lap, i))
                stations++;
            worst = MathF.Max(worst, lap.StationError[i]);
        }
        return (stations, worst);
    }

    private static float SectorSeconds(LaneSet lanes, LaneLapResult lap, int sector, int length, int n)
    {
        float seconds = 0f;
        for (int i = sector * length; i < Math.Min(n, (sector + 1) * length); i++)
            seconds += lanes.Spacing / MathF.Max(lap.StationSpeed.Length > i ? lap.StationSpeed[i] : 0f, 1f);
        return seconds;
    }

    /// <summary>Every lap driven, round without a spin and on the road.</summary>
    private bool Clean(LaneLapResult lap) =>
        OnTheRoad(lap) && (!BestMustBeTroubleFree || TroubleFree(lap));

    private bool OnTheRoad(LaneLapResult lap) =>
        lap.Finished && lap.OffTrackSeconds == 0f && lap.LapTimes.Count >= FlyingLaps;

    private bool TroubleFree(LaneLapResult lap)
    {
        if (lap.WorstErrorMeters > TroubleErrorMeters)
            return false;
        for (int i = 0; i < lap.StationError.Length; i++)
            if (StationInTrouble(lap, i))
                return false;
        return true;
    }

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
                {
                    bool severe = lap.StationError[i] > SevereErrorMeters || (lap.StationOff.Length == n && lap.StationOff[i]);
                    Punish(i, (severe ? SeverePenalty : Penalty) * step, both: severe || !LearnBraking);
                }
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
        int dropped = 0;

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
            Progress?.Invoke(
                $"{lane} steer {iteration}/{iterations}: err {lap.MeanAbsErrorMeters:F2}/{lap.WorstErrorMeters:F2} " +
                $"{(kept ? "kept" : "dropped")}, best {bestLap.MeanAbsErrorMeters:F2}/{bestLap.WorstErrorMeters:F2}");
            // Corrections no longer taken: each step is smaller than the
            // last, so a few in a row dropped is the table settled.
            dropped = kept ? 0 : dropped + 1;
            if (dropped >= SteadyRounds + 1)
            {
                Progress?.Invoke($"{lane} steer: {dropped} corrections in a row dropped, done");
                break;
            }
        }

        return (best, bestLap, history);
    }
}
