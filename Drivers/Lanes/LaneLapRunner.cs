using System.Numerics;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers.Lanes;

/// <summary>What one flying lap did, station by station where it matters.</summary>
public sealed class LaneLapResult
{
    public float OutLapSeconds { get; init; }
    public float FlyingLapSeconds { get; init; }
    public float PlannedLapSeconds { get; init; }
    public int Spins { get; init; }
    public float SpinS { get; init; } = -1f;
    public float MeanAbsErrorMeters { get; init; }
    public float WorstErrorMeters { get; init; }
    public float WorstErrorS { get; init; }
    public float OffTrackSeconds { get; init; }
    public float WallSeconds { get; init; }
    public bool Finished => FlyingLapSeconds > 0f && Spins == 0;
    public IReadOnlyList<float> LapTimes { get; init; } = [];

    /// <summary>Per station: worst |lateral error| on the flying lap.</summary>
    public float[] StationError { get; init; } = [];

    /// <summary>Per station: worst body sideslip on the flying lap.</summary>
    public float[] StationSlip { get; init; } = [];

    /// <summary>Per station: mean speed on the flying lap.</summary>
    public float[] StationSpeed { get; init; } = [];

    /// <summary>Per station: mean signed lateral error, positive clockwise of the lane.</summary>
    public float[] StationSignedError { get; init; } = [];

    /// <summary>Per station: whether the car's centre was off the racing surface there.</summary>
    public bool[] StationOff { get; init; } = [];

    /// <summary>Per station: whether the driver was braking there.</summary>
    public bool[] StationBraking { get; init; } = [];

    /// <summary>
    /// Stations where the driver wanted more wheel than the axles would give:
    /// the car had stopped following its plan, whether or not it had run
    /// wide yet.
    /// </summary>
    public bool[] StationCapBound { get; init; } = [];

    /// <summary>Per station: speed below plan, m/s, at the worst.</summary>
    public float[] StationShortfall { get; init; } = [];

    public override string ToString() =>
        $"flying {FlyingLapSeconds:F2}s (plan {PlannedLapSeconds:F2}s) out {OutLapSeconds:F1}s " +
        $"spins {Spins}{(SpinS >= 0f ? $"@{SpinS:F0}" : "")} " +
        $"err mean {MeanAbsErrorMeters:F2} worst {WorstErrorMeters:F2}@{WorstErrorS:F0} " +
        $"off {OffTrackSeconds:F2}s wall {WallSeconds:F2}s";
}

/// <summary>
/// Drives one rolling out-lap and one flying lap on the free lane. Shared by
/// the probes and by the offline learner.
/// </summary>
public static class LaneLapRunner
{
    /// <summary>How far past the steering cap the wanted wheel has to be to count as bound.</summary>
    public const float CapOverreach = 1.05f;

    public static LaneLapResult Run(
        TrackData track,
        LaneTrackFile file,
        LaneDriverGains? gains = null,
        LaneShares? shares = null,
        float startSpeed = 40f,
        DriverAbilities? abilities = null,
        Action<RaceCar, LaneDriver, float>? onStep = null,
        int flyingLaps = 1,
        TireConfig? tireConfig = null,
        CarStrategy? strategy = null,
        float roadGrip = 1f,
        float startingWear = 0f,
        float startingCharge = 0.8f
    )
    {
        LaneSet lanes = file.LanesFor(track);
        float startS = track.StartingLineS;
        int station = (int)MathF.Round(lanes.StationOf(startS));
        float offset = lanes.Offset(LaneId.Free, station);
        Vector2 here = lanes.Position(station, offset);
        Vector2 next = lanes.Position(station + 1, lanes.Offset(LaneId.Free, station + 1));
        float heading = MathF.Atan2(next.Y - here.Y, next.X - here.X);

        var carConfig = new CarConfig();
        var tires = tireConfig ?? new TireConfig { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f };
        var driver = new LaneDriver(file, carConfig, tires)
        {
            Gains = gains ?? new LaneDriverGains(),
            Shares = shares ?? new LaneShares()
        };
        var profile = new DriverProfile("lanes", abilities ?? new DriverAbilities());
        var car = new RaceCar(
            "lanes",
            carConfig,
            tires,
            new Driver(profile, driver),
            new CarState
            {
                Position = here,
                Heading = heading,
                Speed = startSpeed,
                Energy = PowertrainState.Filled(startingCharge)
            }
        )
        {
            Strategy = strategy ?? CarStrategy.Default
        };
        if (startingWear > 0f)
        {
            foreach (TireState tyre in new[] { car.State.FrontLeft, car.State.FrontRight, car.State.RearLeft, car.State.RearRight })
                tyre.Wear = startingWear;
        }
        var simulation = new RaceSimulation(track, new RaceEnvironment { SurfaceGripScalar = roadGrip });
        simulation.AddCar(car);

        const float dt = 1f / 60f;
        float length = track.LengthMeters;
        float previous = car.Progress.TotalDistance;
        float? firstCrossing = null;
        float lastCrossing = 0f;
        var lapTimes = new List<float>();
        float flying = -1f;
        int count = lanes.Count;
        var stationError = new float[count];
        var stationSlip = new float[count];
        var stationShortfall = new float[count];
        var stationBraking = new bool[count];
        var stationOff = new bool[count];
        var stationCapBound = new bool[count];
        var signedSum = new float[count];
        var signedCount = new int[count];
        var speedSum = new float[count];
        double errorSum = 0;
        int samples = 0;
        float worst = 0f, worstS = 0f, off = 0f, wall = 0f, spinS = -1f;
        float planned = 0f;

        for (int i = 0; i < 60 * (200 + 130 * flyingLaps); i++)
        {
            simulation.Step(dt);
            float t = (i + 1) * dt;
            onStep?.Invoke(car, driver, t);
            if (car.State.SpinEvents > 0)
            {
                spinS = car.Progress.CurrentS;
                break;
            }

            float distance = car.Progress.TotalDistance;
            int before = (int)MathF.Floor(previous / length);
            int after = (int)MathF.Floor(distance / length);
            if (after > before && distance > previous)
            {
                float share = (after * length - previous) / (distance - previous);
                float at = i * dt + share * dt;
                if (firstCrossing is null)
                {
                    firstCrossing = at;
                    planned = driver.PlannedLapSeconds(
                        driver.TargetRange < 0.5f ? LaneId.Free : driver.TargetSide >= 0f ? LaneId.Left : LaneId.Right);
                }
                else
                {
                    float lapTime = at - (lapTimes.Count == 0 ? firstCrossing.Value : lastCrossing);
                    lapTimes.Add(lapTime);
                    lastCrossing = at;
                    if (flying < 0f)
                        flying = lapTime;
                    if (lapTimes.Count >= flyingLaps)
                    {
                        previous = distance;
                        break;
                    }
                }
            }
            previous = distance;

            if (firstCrossing is not null)
            {
                LaneDriverTelemetry tel = driver.LastTelemetry;
                int k = lanes.Wrap((int)MathF.Round(lanes.StationOf(car.Progress.CurrentS)));
                float error = MathF.Abs(tel.LateralErrorMeters);
                stationError[k] = MathF.Max(stationError[k], error);
                signedSum[k] += tel.LateralErrorMeters;
                speedSum[k] += car.State.Speed;
                signedCount[k]++;
                stationSlip[k] = MathF.Max(stationSlip[k], MathF.Abs(car.State.SideslipAngleRadians));
                float plan = driver.PlannedSpeedAt(car.Progress.CurrentS);
                stationShortfall[k] = MathF.Max(stationShortfall[k], plan - car.State.Speed);
                if (tel.CommandAccel < -3f)
                    stationBraking[k] = true;
                if (tel.CurvatureCap != 0f && MathF.Sign(tel.WantedCurvature) == MathF.Sign(tel.CurvatureCap) &&
                    MathF.Abs(tel.WantedCurvature) > MathF.Abs(tel.CurvatureCap) * CapOverreach)
                    stationCapBound[k] = true;
                errorSum += error;
                samples++;
                if (error > worst)
                {
                    worst = error;
                    worstS = car.Progress.CurrentS;
                }
                if (car.Progress.Region != TrackRegion.RacingSurface)
                {
                    off += dt;
                    stationOff[k] = true;
                }
                wall += car.BoundaryContactSeconds;
            }
        }

        return new LaneLapResult
        {
            OutLapSeconds = firstCrossing ?? -1f,
            FlyingLapSeconds = flying,
            PlannedLapSeconds = planned,
            Spins = car.State.SpinEvents,
            SpinS = spinS,
            MeanAbsErrorMeters = samples > 0 ? (float)(errorSum / samples) : 0f,
            WorstErrorMeters = worst,
            WorstErrorS = worstS,
            OffTrackSeconds = off,
            WallSeconds = wall,
            StationError = stationError,
            StationSlip = stationSlip,
            StationShortfall = stationShortfall,
            StationBraking = stationBraking,
            StationOff = stationOff,
            StationCapBound = stationCapBound,
            StationSpeed = speedSum.Select((sum, i) => signedCount[i] > 0 ? sum / signedCount[i] : 0f).ToArray(),
            StationSignedError = signedSum.Select((sum, i) => signedCount[i] > 0 ? sum / signedCount[i] : 0f).ToArray(),
            LapTimes = lapTimes
        };
    }
}
