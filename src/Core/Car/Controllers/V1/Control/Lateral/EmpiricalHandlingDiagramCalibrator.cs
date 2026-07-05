using System;
using System.Collections.Concurrent;
using System.Globalization;
using System.Text;
using Godot;
using StintegyEVO.Core.Car;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Track;
using StintegyEVO.Nodes.Race;

namespace StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;

public static class EmpiricalHandlingDiagramCalibrator
{
    private static readonly ConcurrentDictionary<string, EmpiricalHandlingDiagramModel> Cache = new();

    public static EmpiricalHandlingDiagramModel Calibrate(
        CarConfig carConfig,
        TrackData track,
        EmpiricalHandlingDiagramCalibrationConfig config
    )
    {
        ArgumentNullException.ThrowIfNull(carConfig);
        ArgumentNullException.ThrowIfNull(track);
        ArgumentNullException.ThrowIfNull(config);
        Validate(config);

        string cacheKey = CreateCacheKey(carConfig, config);
        return Cache.GetOrAdd(cacheKey, _ => CalibrateCore(carConfig, track, config));
    }

    private static EmpiricalHandlingDiagramModel CalibrateCore(
        CarConfig carConfig,
        TrackData track,
        EmpiricalHandlingDiagramCalibrationConfig config
    )
    {
        NormalEquations equations = new(config.Ridge);
        int sampleCount = 0;

        for (
            float targetSpeed = config.MinimumFitSpeedMetersPerSecond;
            targetSpeed <= config.MaximumFitSpeedMetersPerSecond + 1e-4f;
            targetSpeed += config.SpeedStepMetersPerSecond
        )
        {
            for (int i = -config.SteeringSamplesPerSide; i <= config.SteeringSamplesPerSide; i++)
            {
                if (i == 0)
                    continue;

                float steeringInput = config.MaximumCalibrationSteeringInput * i / config.SteeringSamplesPerSide;
                if (!TrySimulateSteadyCorner(
                    carConfig,
                    track,
                    config,
                    targetSpeed,
                    steeringInput,
                    out CalibrationSample sample
                ))
                {
                    continue;
                }

                float ay = sample.LateralAcceleration;
                float speed = MathF.Max(MathF.Abs(sample.LongitudinalSpeed), 1.0f);
                float deltaAck = EmpiricalHandlingDiagramModel.CalculateKinematicSteeringAngle(
                    carConfig.Chassis.WheelBase,
                    ay,
                    speed
                );
                float deltaDev = sample.SteeringAngle - deltaAck;
                float z = deltaDev / ay;
                float ay2 = ay * ay;
                equations.Add(
                    ay2 * speed,
                    ay2,
                    speed,
                    1.0f,
                    z
                );
                sampleCount++;
            }
        }

        if (sampleCount < 8 || !equations.TrySolve(out float[] p))
            return EmpiricalHandlingDiagramModel.Zero;

        return new EmpiricalHandlingDiagramModel(
            KAy3V: p[0],
            KAy3: p[1],
            KAyV: p[2],
            KAy: p[3],
            FitSampleCount: sampleCount
        );
    }

    private static bool TrySimulateSteadyCorner(
        CarConfig carConfig,
        TrackData track,
        EmpiricalHandlingDiagramCalibrationConfig config,
        float targetSpeed,
        float steeringInput,
        out CalibrationSample sample
    )
    {
        sample = default;

        CarLogic logic = new(carConfig, track, DummyEnvironment.Instance);
        float dt = config.SimulationDtSeconds;
        float mass = carConfig.Chassis.DryMass;
        float inertia = MathF.Max(carConfig.Chassis.DryI, 1.0f);
        float steeringAngle = steeringInput * carConfig.Chassis.MaxSteerAngle;
        Vector2 globalVelocity = new(targetSpeed, 0.0f);
        Vector2 globalAcceleration = Vector2.Zero;
        float rotation = 0.0f;
        float yawRate = targetSpeed * MathF.Tan(steeringAngle) / MathF.Max(carConfig.Chassis.WheelBase, 1e-3f);
        int settleSteps = Math.Max(1, Mathf.RoundToInt(config.SettleSeconds / dt));
        int measureSteps = Math.Max(1, Mathf.RoundToInt(config.MeasureSeconds / dt));
        int totalSteps = settleSteps + measureSteps;

        RunningAverage vx = new();
        RunningAverage ay = new();
        RunningAverage speedError = new();
        int slideSamples = 0;

        for (int step = 0; step < totalSteps; step++)
        {
            Vector2 localVelocity = globalVelocity.Rotated(-rotation);
            Vector2 localAcceleration = globalAcceleration.Rotated(-rotation);
            float input = Mathf.Clamp(
                config.SpeedHoldGain * (targetSpeed - localVelocity.X),
                -config.MaximumBrakeInput,
                config.MaximumThrottleInput
            );

            PhysicsOutput output = logic.Tick(
                dt,
                input,
                steeringInput,
                localVelocity,
                localAcceleration,
                yawRate,
                mass
            );

            Vector2 totalLocalForce =
                output.DragForce +
                output.FrontLeft.Force +
                output.FrontRight.Force +
                output.RearLeft.Force +
                output.RearRight.Force;
            float yawMoment =
                Cross(output.FrontLeft.Pos, output.FrontLeft.Force) +
                Cross(output.FrontRight.Pos, output.FrontRight.Force) +
                Cross(output.RearLeft.Pos, output.RearLeft.Force) +
                Cross(output.RearRight.Pos, output.RearRight.Force);

            globalAcceleration = totalLocalForce.Rotated(rotation) / mass;
            globalVelocity += globalAcceleration * dt;
            yawRate += yawMoment / inertia * dt;
            rotation = WrapAngle(rotation + yawRate * dt);

            if (step < settleSteps)
                continue;

            Vector2 measuredLocalVelocity = globalVelocity.Rotated(-rotation);
            Vector2 measuredLocalAcceleration = globalAcceleration.Rotated(-rotation);
            vx.Add(measuredLocalVelocity.X);
            ay.Add(measuredLocalAcceleration.Y);
            speedError.Add(MathF.Abs(targetSpeed - measuredLocalVelocity.X));
            if (AnyTireSliding(output.Params))
                slideSamples++;
        }

        if (vx.Count == 0)
            return false;

        float meanVx = vx.Value;
        float meanAy = ay.Value;
        float absAy = MathF.Abs(meanAy);
        float slideFraction = slideSamples / (float)vx.Count;
        if (!float.IsFinite(meanVx) || !float.IsFinite(meanAy))
            return false;
        if (meanVx < config.MinimumFitSpeedMetersPerSecond * 0.75f)
            return false;
        if (speedError.Value > config.MaximumMeanSpeedErrorMetersPerSecond)
            return false;
        if (absAy < config.MinimumFitLateralAccelerationMetersPerSecondSquared)
            return false;
        if (absAy > config.MaximumFitLateralAccelerationMetersPerSecondSquared)
            return false;
        if (slideFraction > config.MaximumSlideFraction)
            return false;

        sample = new CalibrationSample(meanVx, meanAy, steeringAngle);
        return true;
    }

    private static bool AnyTireSliding(IntermediateParams p)
    {
        return p.FrontLeft.IsSliding ||
            p.FrontRight.IsSliding ||
            p.RearLeft.IsSliding ||
            p.RearRight.IsSliding;
    }

    private static float Cross(Vector2 a, Vector2 b)
    {
        return a.X * b.Y - a.Y * b.X;
    }

    private static float WrapAngle(float angle)
    {
        while (angle <= -Mathf.Pi)
            angle += Mathf.Tau;
        while (angle > Mathf.Pi)
            angle -= Mathf.Tau;
        return angle;
    }

    private static string CreateCacheKey(CarConfig car, EmpiricalHandlingDiagramCalibrationConfig config)
    {
        StringBuilder key = new();
        Append(key, car.Chassis.DryMass);
        Append(key, car.Chassis.DryI);
        Append(key, car.Chassis.MaxSteerAngle);
        Append(key, car.Chassis.CgHeight);
        Append(key, car.Chassis.WheelBase);
        Append(key, car.Chassis.Width);
        Append(key, car.Chassis.WeightDistFront);
        Append(key, car.Power.MaxDriveForce);
        Append(key, car.Power.MaxPower);
        Append(key, car.Power.MaxBrakeForce);
        Append(key, car.Aero.BaseDragCoef);
        Append(key, car.Aero.FrontalArea);
        Append(key, car.Aero.DownforceCoef);
        Append(key, car.Aero.AeroBalanceFront);
        Append(key, car.Distributor.FrontBias);
        Append(key, car.Distributor.VectoringStrength);
        Append(key, car.Distributor.FrontVectoringScale);
        foreach (TireConfig tire in car.Tires)
        {
            Append(key, (int)tire.Type);
            Append(key, tire.Radius);
            Append(key, tire.Mass);
            Append(key, tire.Inertia);
            Append(key, tire.LatStiffness);
            Append(key, tire.LatDrop);
            Append(key, tire.LatPeakFriction);
            Append(key, tire.LongStiffness);
            Append(key, tire.LongDrop);
            Append(key, tire.LongPeakFriction);
            Append(key, tire.InitPressure);
        }
        Append(key, config.MinimumFitSpeedMetersPerSecond);
        Append(key, config.MaximumFitSpeedMetersPerSecond);
        Append(key, config.SpeedStepMetersPerSecond);
        Append(key, config.SteeringSamplesPerSide);
        Append(key, config.MaximumCalibrationSteeringInput);
        Append(key, config.MinimumFitLateralAccelerationMetersPerSecondSquared);
        Append(key, config.MaximumFitLateralAccelerationMetersPerSecondSquared);
        return key.ToString();
    }

    private static void Append(StringBuilder builder, float value)
    {
        builder.Append(value.ToString("R", CultureInfo.InvariantCulture)).Append('|');
    }

    private static void Append(StringBuilder builder, int value)
    {
        builder.Append(value.ToString(CultureInfo.InvariantCulture)).Append('|');
    }

    private static void Validate(EmpiricalHandlingDiagramCalibrationConfig config)
    {
        if (config.MinimumFitSpeedMetersPerSecond <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Minimum fit speed must be positive.");
        if (config.MaximumFitSpeedMetersPerSecond < config.MinimumFitSpeedMetersPerSecond)
            throw new ArgumentOutOfRangeException(nameof(config), "Maximum fit speed must be at least the minimum fit speed.");
        if (config.SpeedStepMetersPerSecond <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Speed step must be positive.");
        if (config.SteeringSamplesPerSide <= 0)
            throw new ArgumentOutOfRangeException(nameof(config), "Steering samples per side must be positive.");
        if (config.SimulationDtSeconds <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Simulation dt must be positive.");
        if (config.MeasureSeconds <= 0.0f)
            throw new ArgumentOutOfRangeException(nameof(config), "Measure time must be positive.");
    }

    private readonly record struct CalibrationSample(
        float LongitudinalSpeed,
        float LateralAcceleration,
        float SteeringAngle
    );

    private struct RunningAverage
    {
        private float _sum;

        public int Count { get; private set; }
        public readonly float Value => Count == 0 ? 0.0f : _sum / Count;

        public void Add(float value)
        {
            _sum += value;
            Count++;
        }
    }

    private sealed class NormalEquations
    {
        private readonly double[,] _a = new double[4, 4];
        private readonly double[] _b = new double[4];

        public NormalEquations(float ridge)
        {
            for (int i = 0; i < 4; i++)
                _a[i, i] = ridge;
        }

        public void Add(float f0, float f1, float f2, float f3, float target)
        {
            double[] f = [f0, f1, f2, f3];
            for (int r = 0; r < 4; r++)
            {
                _b[r] += f[r] * target;
                for (int c = 0; c < 4; c++)
                    _a[r, c] += f[r] * f[c];
            }
        }

        public bool TrySolve(out float[] result)
        {
            result = new float[4];
            double[,] a = (double[,])_a.Clone();
            double[] b = (double[])_b.Clone();

            for (int pivot = 0; pivot < 4; pivot++)
            {
                int best = pivot;
                double bestAbs = Math.Abs(a[pivot, pivot]);
                for (int r = pivot + 1; r < 4; r++)
                {
                    double abs = Math.Abs(a[r, pivot]);
                    if (abs <= bestAbs)
                        continue;
                    best = r;
                    bestAbs = abs;
                }

                if (bestAbs < 1e-12)
                    return false;

                if (best != pivot)
                {
                    for (int c = pivot; c < 4; c++)
                        (a[pivot, c], a[best, c]) = (a[best, c], a[pivot, c]);
                    (b[pivot], b[best]) = (b[best], b[pivot]);
                }

                double pivotValue = a[pivot, pivot];
                for (int c = pivot; c < 4; c++)
                    a[pivot, c] /= pivotValue;
                b[pivot] /= pivotValue;

                for (int r = 0; r < 4; r++)
                {
                    if (r == pivot)
                        continue;
                    double factor = a[r, pivot];
                    for (int c = pivot; c < 4; c++)
                        a[r, c] -= factor * a[pivot, c];
                    b[r] -= factor * b[pivot];
                }
            }

            for (int i = 0; i < 4; i++)
            {
                if (!double.IsFinite(b[i]))
                    return false;
                result[i] = (float)b[i];
            }

            return true;
        }
    }
}
