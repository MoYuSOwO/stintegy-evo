using Godot;
using GdUnit4;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;
using StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;
using StintegyEVO.Core.Track;
using static GdUnit4.Assertions;

namespace StintegyEVO.Tests.Controllers.V1.Control.Lateral;

[TestSuite]
public sealed class EmpiricalHandlingDiagramLateralControllerTests
{
    [TestCase]
    [RequireGodotRuntime]
    public void FeedForwardUsesHandlingDiagramDeviation()
    {
        CarConfig car = new();
        EmpiricalHandlingDiagramModel model = new(
            KAy3V: 0.0f,
            KAy3: 0.0f,
            KAyV: 0.0f,
            KAy: 0.01f,
            FitSampleCount: 12
        );
        float ay = 6.0f;
        float speed = 20.0f;

        float steering = model.PredictSteeringAngle(car, ay, speed);
        float expected = EmpiricalHandlingDiagramModel.CalculateKinematicSteeringAngle(
            car.Chassis.WheelBase,
            ay,
            speed
        ) + 0.01f * ay;

        AssertThat(steering).IsEqualApprox(expected, 0.0001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void SteersBackTowardPathWhenVehicleIsLeftOfPath()
    {
        EmpiricalHandlingDiagramLateralController controller = new(model: EmpiricalHandlingDiagramModel.Zero);
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            Position = new Vector2(10.0f, 2.0f),
            Rotation = 0.0f,
            LinearVelocity = new Vector2(20.0f, 0.0f)
        };

        LateralControlOutput output = controller.Control(profile, sensor, car, track, dt: 1.0f / 60.0f);

        AssertThat(output.ProfileDistance).IsEqualApprox(10.0f, 0.001f);
        AssertThat(output.NearestProfileIndex).IsEqual(1);
        AssertThat(output.LateralError).IsGreater(0.0f);
        AssertThat(output.FeedbackSteeringAngle).IsLess(0.0f);
        AssertThat(output.SteeringAngle).IsLess(0.0f);
        AssertThat(Mathf.Abs(output.SteeringAngle)).IsLessEqual(car.Chassis.MaxSteerAngle + 0.001f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void PositiveSlipAndYawErrorCounterSteerOnStraightPath()
    {
        EmpiricalHandlingDiagramLateralController controller = new(model: EmpiricalHandlingDiagramModel.Zero);
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            Position = new Vector2(10.0f, 0.0f),
            Rotation = 0.0f,
            AngularVelocity = 0.8f,
            LinearVelocity = new Vector2(20.0f, 4.0f)
        };

        LateralControlOutput output = controller.Control(profile, sensor, car, track, dt: 1.0f / 60.0f);

        AssertThat(output.Beta).IsGreater(0.0f);
        AssertThat(output.YawRateError).IsGreater(0.0f);
        AssertThat(output.StabilityRisk).IsGreater(0.0f);
        AssertThat(output.StabilitySteeringAngle).IsLess(0.0f);
        AssertThat(output.SteeringAngle).IsLess(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void ConflictingSlipAndYawErrorPrioritizesYawDamping()
    {
        EmpiricalHandlingDiagramLateralController controller = new(model: EmpiricalHandlingDiagramModel.Zero);
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            Position = new Vector2(10.0f, 0.0f),
            Rotation = 0.0f,
            AngularVelocity = 1.5f,
            LinearVelocity = new Vector2(20.0f, -8.0f)
        };

        LateralControlOutput output = controller.Control(profile, sensor, car, track, dt: 1.0f / 60.0f);

        AssertThat(output.Beta).IsLess(0.0f);
        AssertThat(output.YawRateError).IsGreater(0.0f);
        AssertThat(output.StabilityRisk).IsGreater(0.9f);
        AssertThat(output.YawDampingSteeringAngle).IsLess(0.0f);
        AssertThat(output.BetaDampingSteeringAngle).IsGreater(0.0f);
        AssertThat(Mathf.Abs(output.BetaDampingSteeringAngle)).IsLess(Mathf.Abs(output.YawDampingSteeringAngle));
        AssertThat(output.StabilitySteeringAngle).IsLess(0.0f);
        AssertThat(output.SteeringAngle).IsLess(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void ConflictingPathFeedbackYieldsToYawDamping()
    {
        EmpiricalHandlingDiagramLateralController controller = new(model: EmpiricalHandlingDiagramModel.Zero);
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            Position = new Vector2(10.0f, 2.0f),
            Rotation = 0.0f,
            AngularVelocity = -0.25f,
            LinearVelocity = new Vector2(20.0f, 0.0f)
        };

        LateralControlOutput output = controller.Control(profile, sensor, car, track, dt: 1.0f / 60.0f);

        AssertThat(output.FeedbackSteeringAngle).IsLess(0.0f);
        AssertThat(Mathf.Abs(output.FeedbackSteeringAngle)).IsLess(0.02f);
        AssertThat(output.YawDampingSteeringAngle).IsGreater(0.0f);
        AssertThat(output.SteeringAngle).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void LookaheadGeometryCanOverrideConflictingFeedForward()
    {
        EmpiricalHandlingDiagramModel model = new(
            KAy3V: 0.0f,
            KAy3: 0.0f,
            KAyV: 0.0f,
            KAy: 0.02f,
            FitSampleCount: 12
        );
        EmpiricalHandlingDiagramLateralController controller = new(model: model);
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        SpeedProfile profile = CreateCurvedProfile(speed: 5.0f, curvature: -0.03f);
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            Position = new Vector2(0.0f, -4.0f),
            Rotation = 0.0f,
            LinearVelocity = new Vector2(5.0f, 0.0f)
        };

        LateralControlOutput output = controller.Control(profile, sensor, car, track, dt: 1.0f / 60.0f);

        AssertThat(output.FeedForwardSteeringAngle).IsLess(0.0f);
        AssertThat(output.FeedbackSteeringAngle).IsGreater(0.0f);
        AssertThat(output.SteeringAngle).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void FeedForwardLateralDemandIsCappedByPlannedLookaheadSpeed()
    {
        EmpiricalHandlingDiagramLateralController controller = new(model: EmpiricalHandlingDiagramModel.Zero);
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        SpeedProfile profile = CreateCurvedProfile(speed: 8.0f, curvature: 0.05f);
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            Position = Vector2.Zero,
            Rotation = 0.0f,
            LinearVelocity = new Vector2(20.0f, 0.0f)
        };

        LateralControlOutput output = controller.Control(profile, sensor, car, track, dt: 1.0f / 60.0f);

        AssertThat(output.TargetSpeed).IsEqualApprox(8.0f, 0.001f);
        AssertThat(output.TargetLateralAcceleration).IsEqualApprox(8.0f * 8.0f * 0.05f, 0.001f);
    }


    private static SpeedProfile CreateStraightProfile()
    {
        SpeedProfilePoint[] points = new SpeedProfilePoint[6];
        for (int i = 0; i < points.Length; i++)
        {
            float distance = i * 20.0f;
            points[i] = new SpeedProfilePoint(
                SampleIndex: i,
                Position: new Vector2(distance, 0.0f),
                Heading: 0.0f,
                Curvature: 0.0f,
                Distance: distance,
                Speed: 20.0f,
                AccelerationToNext: 0.0f,
                TimeFromStart: i,
                MaxSpeed: 30.0f,
                MaxAcceleration: 5.0f,
                MaxDeceleration: 8.0f,
                LateralAcceleration: 0.0f
            );
        }

        return new SpeedProfile(points);
    }

    private static SpeedProfile CreateCurvedProfile(float speed, float curvature)
    {
        SpeedProfilePoint[] points = new SpeedProfilePoint[6];
        for (int i = 0; i < points.Length; i++)
        {
            float distance = i * 20.0f;
            points[i] = new SpeedProfilePoint(
                SampleIndex: i,
                Position: new Vector2(distance, 0.0f),
                Heading: 0.0f,
                Curvature: curvature,
                Distance: distance,
                Speed: speed,
                AccelerationToNext: 0.0f,
                TimeFromStart: i,
                MaxSpeed: speed,
                MaxAcceleration: 5.0f,
                MaxDeceleration: 8.0f,
                LateralAcceleration: speed * speed * Mathf.Abs(curvature)
            );
        }

        return new SpeedProfile(points);
    }
}
