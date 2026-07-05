using Godot;
using GdUnit4;
using StintegyEVO.Core.Car;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Car.Controllers.V1.Control.Lateral;
using StintegyEVO.Core.Car.Controllers.V1.Control.Longitudinal;
using StintegyEVO.Core.Car.Controllers.V1.SpeedPlanning;
using StintegyEVO.Core.Track;
using StintegyEVO.Nodes.Race;
using static GdUnit4.Assertions;

namespace StintegyEVO.Tests.Controllers.V1.Control.Longitudinal;

[TestSuite]
public sealed class StabilityAwareSpeedControllerTests
{
    [TestCase]
    [RequireGodotRuntime]
    public void StabilityRiskTurnsAccelerationDemandIntoBraking()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(12.0f, 0.0f)
        };
        LateralControlOutput lateral = new(
            SteeringInput: 0.0f,
            SteeringAngle: 0.0f,
            FeedForwardSteeringAngle: 0.0f,
            FeedbackSteeringAngle: 0.0f,
            StabilitySteeringAngle: 0.0f,
            YawDampingSteeringAngle: 0.0f,
            BetaDampingSteeringAngle: 0.0f,
            LookaheadDistance: 20.0f,
            LookaheadPoint: new Vector2(20.0f, 0.0f),
            TargetSpeed: 30.0f,
            TargetLateralAcceleration: 0.0f,
            NearestProfileIndex: 0,
            ProfileDistance: 0.0f,
            LateralError: 0.0f,
            HeadingError: 0.0f,
            Beta: 0.3f,
            BetaReference: 0.0f,
            YawRate: 1.5f,
            YawRateReference: 0.0f,
            YawRateError: 1.5f,
            StabilityRisk: 1.0f
        );

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.StabilityRisk).IsEqualApprox(1.0f, 0.001f);
        AssertThat(output.RequestedAcceleration).IsLess(0.0f);
        AssertThat(output.Input).IsLess(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void LowSpeedTireSlideDoesNotPreventLaunch()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(0.2f, 0.0f),
            Params = new IntermediateParams
            {
                FrontLeft = new TireOutput { IsSliding = true },
                FrontRight = new TireOutput { IsSliding = true },
                RearLeft = new TireOutput { IsSliding = true },
                RearRight = new TireOutput { IsSliding = true }
            }
        };
        LateralControlOutput lateral = new(
            SteeringInput: 0.0f,
            SteeringAngle: 0.0f,
            FeedForwardSteeringAngle: 0.0f,
            FeedbackSteeringAngle: 0.0f,
            StabilitySteeringAngle: 0.0f,
            YawDampingSteeringAngle: 0.0f,
            BetaDampingSteeringAngle: 0.0f,
            LookaheadDistance: 20.0f,
            LookaheadPoint: new Vector2(20.0f, 0.0f),
            TargetSpeed: 30.0f,
            TargetLateralAcceleration: 0.0f,
            NearestProfileIndex: 0,
            ProfileDistance: 0.0f,
            LateralError: 0.0f,
            HeadingError: 0.0f,
            Beta: 0.0f,
            BetaReference: 0.0f,
            YawRate: 0.0f,
            YawRateReference: 0.0f,
            YawRateError: 0.0f,
            StabilityRisk: 0.0f
        );

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.StabilityRisk).IsEqualApprox(0.0f, 0.001f);
        AssertThat(output.LimitedAcceleration).IsGreater(0.0f);
        AssertThat(output.Input).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void LowSpeedLateralStabilityRiskDoesNotForceBrake()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(4.0f, 0.0f)
        };
        LateralControlOutput lateral = new(
            SteeringInput: 0.0f,
            SteeringAngle: 0.0f,
            FeedForwardSteeringAngle: 0.0f,
            FeedbackSteeringAngle: 0.0f,
            StabilitySteeringAngle: 0.0f,
            YawDampingSteeringAngle: 0.0f,
            BetaDampingSteeringAngle: 0.0f,
            LookaheadDistance: 20.0f,
            LookaheadPoint: new Vector2(20.0f, 0.0f),
            TargetSpeed: 30.0f,
            TargetLateralAcceleration: 0.0f,
            NearestProfileIndex: 0,
            ProfileDistance: 0.0f,
            LateralError: 0.0f,
            HeadingError: 0.0f,
            Beta: 0.5f,
            BetaReference: 0.0f,
            YawRate: 1.0f,
            YawRateReference: 0.0f,
            YawRateError: 1.0f,
            StabilityRisk: 1.0f
        );

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.StabilityRisk).IsEqualApprox(0.0f, 0.001f);
        AssertThat(output.RequestedAcceleration).IsGreater(0.0f);
        AssertThat(output.Input).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void RecoverySpeedTireSlideDoesNotForceStabilityBrake()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(5.5f, 0.0f),
            Params = new IntermediateParams
            {
                FrontLeft = new TireOutput { IsSliding = true },
                FrontRight = new TireOutput { IsSliding = true },
                RearLeft = new TireOutput { IsSliding = true },
                RearRight = new TireOutput { IsSliding = true }
            }
        };
        LateralControlOutput lateral = CreateStraightLateralOutput(nearestProfileIndex: 0);

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.StabilityRisk).IsEqualApprox(0.0f, 0.001f);
        AssertThat(output.LimitedAcceleration).IsGreater(0.0f);
        AssertThat(output.Input).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void CurrentSpeedPointDoesNotHoldLaunchTargetAtZero()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateLaunchProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(0.2f, 0.0f)
        };
        LateralControlOutput lateral = CreateStraightLateralOutput(nearestProfileIndex: 0);

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.TargetSpeed).IsGreater(1.0f);
        AssertThat(output.RequestedAcceleration).IsGreater(0.0f);
        AssertThat(output.Input).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void LargeTrackingErrorDoesNotCutThrottleByItself()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(8.0f, 0.0f)
        };
        LateralControlOutput lateral = new(
            SteeringInput: 0.0f,
            SteeringAngle: 0.0f,
            FeedForwardSteeringAngle: 0.0f,
            FeedbackSteeringAngle: 0.0f,
            StabilitySteeringAngle: 0.0f,
            YawDampingSteeringAngle: 0.0f,
            BetaDampingSteeringAngle: 0.0f,
            LookaheadDistance: 20.0f,
            LookaheadPoint: new Vector2(20.0f, 0.0f),
            TargetSpeed: 30.0f,
            TargetLateralAcceleration: 0.0f,
            NearestProfileIndex: 0,
            ProfileDistance: 0.0f,
            LateralError: 5.5f,
            HeadingError: 0.55f,
            Beta: 0.0f,
            BetaReference: 0.0f,
            YawRate: 0.0f,
            YawRateReference: 0.0f,
            YawRateError: 0.0f,
            StabilityRisk: 0.0f
        );

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.TrackingRisk).IsGreater(0.9f);
        AssertThat(output.RequestedAcceleration).IsGreater(0.0f);
        AssertThat(output.Input).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void TinyFutureSpeedDipDoesNotSuppressThrottle()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateProfileWithTinyFutureSpeedDip();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(10.0f, 0.0f)
        };
        LateralControlOutput lateral = CreateStraightLateralOutput(nearestProfileIndex: 0);

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.TargetSpeed).IsGreater(10.0f);
        AssertThat(output.RequestedAcceleration).IsGreater(0.0f);
        AssertThat(output.Input).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void TrackBoundaryExcessBrakesEvenWhenPathTrackingLooksFine()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(14.0f, 0.0f)
        };
        LateralControlOutput lateral = CreateStraightLateralOutput(
            nearestProfileIndex: 0,
            trackBoundaryExcess: 3.0f
        );

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.TrackingRisk).IsEqualApprox(1.0f, 0.001f);
        AssertThat(output.RequestedAcceleration).IsLess(-1.0f);
        AssertThat(output.Input).IsLess(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void TrackBoundaryExcessAllowsCrawlBelowRecoverySpeed()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(1.0f, 0.0f)
        };
        LateralControlOutput lateral = CreateStraightLateralOutput(
            nearestProfileIndex: 0,
            trackBoundaryExcess: 3.0f
        );

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.TrackingRisk).IsEqualApprox(1.0f, 0.001f);
        AssertThat(output.RequestedAcceleration).IsGreater(0.0f);
        AssertThat(output.Input).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void ModerateTrackingErrorCapsAccelerationWithoutBraking()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(6.0f, 0.0f)
        };
        LateralControlOutput lateral = CreateStraightLateralOutput(
            nearestProfileIndex: 0,
            lateralError: 3.5f,
            headingError: 0.2f
        );

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.TrackingRisk).IsGreater(0.0f);
        AssertThat(output.RequestedAcceleration).IsGreater(0.0f);
        AssertThat(output.LimitedAcceleration).IsGreater(0.0f);
        AssertThat(output.Input).IsGreater(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void OverspeedIntoTightCurveKeepsBrakeCapacityFromActualYaw()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateConstantCurvatureProfile(curvature: 0.05f, speed: 10.0f);
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(24.0f, 0.0f)
        };
        LateralControlOutput lateral = new(
            SteeringInput: 0.0f,
            SteeringAngle: 0.0f,
            FeedForwardSteeringAngle: 0.0f,
            FeedbackSteeringAngle: 0.0f,
            StabilitySteeringAngle: 0.0f,
            YawDampingSteeringAngle: 0.0f,
            BetaDampingSteeringAngle: 0.0f,
            LookaheadDistance: 30.0f,
            LookaheadPoint: new Vector2(30.0f, 0.0f),
            TargetSpeed: 10.0f,
            TargetLateralAcceleration: 24.0f * 24.0f * 0.05f,
            NearestProfileIndex: 0,
            ProfileDistance: 0.0f,
            LateralError: 0.0f,
            HeadingError: 0.0f,
            Beta: 0.0f,
            BetaReference: 0.0f,
            YawRate: 0.0f,
            YawRateReference: 24.0f * 0.05f,
            YawRateError: -24.0f * 0.05f,
            StabilityRisk: 0.0f
        );

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.LateralSpeedLimit).IsLess(24.0f);
        AssertThat(output.MaximumDeceleration).IsGreater(1.0f);
        AssertThat(output.LimitedAcceleration).IsLess(-1.0f);
        AssertThat(output.Input).IsLess(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void HighActualYawDoesNotRemoveBrakeAuthority()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateStraightProfile();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(12.0f, 0.0f)
        };
        LateralControlOutput lateral = new(
            SteeringInput: 0.0f,
            SteeringAngle: 0.0f,
            FeedForwardSteeringAngle: 0.0f,
            FeedbackSteeringAngle: 0.0f,
            StabilitySteeringAngle: 0.0f,
            YawDampingSteeringAngle: 0.0f,
            BetaDampingSteeringAngle: 0.0f,
            LookaheadDistance: 20.0f,
            LookaheadPoint: new Vector2(20.0f, 0.0f),
            TargetSpeed: 30.0f,
            TargetLateralAcceleration: 0.0f,
            NearestProfileIndex: 0,
            ProfileDistance: 0.0f,
            LateralError: 0.0f,
            HeadingError: 0.0f,
            Beta: 0.0f,
            BetaReference: 0.0f,
            YawRate: 2.0f,
            YawRateReference: 0.0f,
            YawRateError: 2.0f,
            StabilityRisk: 1.0f
        );

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.MaximumDeceleration).IsGreater(1.0f);
        AssertThat(output.LimitedAcceleration).IsLess(-1.0f);
        AssertThat(output.Input).IsLess(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void BrakesForMinimumSpeedInsideLookaheadWindow()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateProfileWithMidWindowSlowPoint();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(12.0f, 0.0f)
        };
        LateralControlOutput lateral = CreateStraightLateralOutput(nearestProfileIndex: 0);

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.TargetSpeed).IsGreater(6.0f);
        AssertThat(output.RequestedAcceleration).IsLess(0.0f);
        AssertThat(output.Input).IsLess(0.0f);
    }

    [TestCase]
    [RequireGodotRuntime]
    public void BrakesForSlowPointBeyondShortSpeedLookaheadWhenDistanceRequiresIt()
    {
        CarConfig car = new();
        TrackData track = TrackFactory.SimpleOvalTrack(width: 80.0f, height: 30.0f, trackWidth: 18.0f);
        CarLogic logic = new(car, track, DummyEnvironment.Instance);
        StabilityAwareSpeedController controller = new();
        SpeedProfile profile = CreateProfileWithLateSlowPoint();
        CarSensor sensor = new()
        {
            Mass = car.Chassis.DryMass,
            LinearVelocity = new Vector2(40.0f, 0.0f)
        };
        LateralControlOutput lateral = CreateStraightLateralOutput(nearestProfileIndex: 0);

        StabilityAwareSpeedControlOutput output = controller.Control(profile, lateral, sensor, logic, track);

        AssertThat(output.TargetSpeed).IsGreater(39.0f);
        AssertThat(output.RequestedAcceleration).IsLess(-3.0f);
        AssertThat(output.Input).IsLess(0.0f);
    }

    private static SpeedProfile CreateStraightProfile()
    {
        SpeedProfilePoint[] points = new SpeedProfilePoint[8];
        for (int i = 0; i < points.Length; i++)
        {
            float distance = i * 10.0f;
            points[i] = new SpeedProfilePoint(
                SampleIndex: i,
                Position: new Vector2(distance, 0.0f),
                Heading: 0.0f,
                Curvature: 0.0f,
                Distance: distance,
                Speed: 30.0f,
                AccelerationToNext: 2.0f,
                TimeFromStart: i,
                MaxSpeed: 40.0f,
                MaxAcceleration: 6.0f,
                MaxDeceleration: 9.0f,
                LateralAcceleration: 0.0f
            );
        }

        return new SpeedProfile(points);
    }

    private static SpeedProfile CreateLaunchProfile()
    {
        SpeedProfilePoint[] points = new SpeedProfilePoint[8];
        for (int i = 0; i < points.Length; i++)
        {
            float distance = i * 10.0f;
            float speed = i == 0 ? 0.0f : 30.0f;
            points[i] = new SpeedProfilePoint(
                SampleIndex: i,
                Position: new Vector2(distance, 0.0f),
                Heading: 0.0f,
                Curvature: 0.0f,
                Distance: distance,
                Speed: speed,
                AccelerationToNext: 2.0f,
                TimeFromStart: i,
                MaxSpeed: 40.0f,
                MaxAcceleration: 6.0f,
                MaxDeceleration: 9.0f,
                LateralAcceleration: 0.0f
            );
        }

        return new SpeedProfile(points);
    }

    private static SpeedProfile CreateProfileWithLateSlowPoint()
    {
        SpeedProfilePoint[] points = new SpeedProfilePoint[4];
        float[] distances = [0.0f, 70.0f, 140.0f, 180.0f];
        float[] speeds = [50.0f, 50.0f, 50.0f, 10.0f];
        for (int i = 0; i < points.Length; i++)
        {
            points[i] = new SpeedProfilePoint(
                SampleIndex: i,
                Position: new Vector2(distances[i], 0.0f),
                Heading: 0.0f,
                Curvature: 0.0f,
                Distance: distances[i],
                Speed: speeds[i],
                AccelerationToNext: 0.0f,
                TimeFromStart: i,
                MaxSpeed: speeds[i],
                MaxAcceleration: 6.0f,
                MaxDeceleration: 9.0f,
                LateralAcceleration: 0.0f
            );
        }

        return new SpeedProfile(points);
    }

    private static SpeedProfile CreateProfileWithTinyFutureSpeedDip()
    {
        SpeedProfilePoint[] points = new SpeedProfilePoint[5];
        float[] distances = [0.0f, 10.0f, 25.0f, 40.0f, 60.0f];
        float[] speeds = [30.0f, 30.0f, 9.9f, 30.0f, 30.0f];
        for (int i = 0; i < points.Length; i++)
        {
            points[i] = new SpeedProfilePoint(
                SampleIndex: i,
                Position: new Vector2(distances[i], 0.0f),
                Heading: 0.0f,
                Curvature: 0.0f,
                Distance: distances[i],
                Speed: speeds[i],
                AccelerationToNext: 2.0f,
                TimeFromStart: i,
                MaxSpeed: 40.0f,
                MaxAcceleration: 6.0f,
                MaxDeceleration: 9.0f,
                LateralAcceleration: 0.0f
            );
        }

        return new SpeedProfile(points);
    }

    private static SpeedProfile CreateProfileWithMidWindowSlowPoint()
    {
        SpeedProfilePoint[] points = new SpeedProfilePoint[5];
        float[] distances = [0.0f, 15.0f, 30.0f, 45.0f, 60.0f];
        float[] speeds = [30.0f, 24.0f, 6.0f, 24.0f, 30.0f];
        for (int i = 0; i < points.Length; i++)
        {
            points[i] = new SpeedProfilePoint(
                SampleIndex: i,
                Position: new Vector2(distances[i], 0.0f),
                Heading: 0.0f,
                Curvature: 0.0f,
                Distance: distances[i],
                Speed: speeds[i],
                AccelerationToNext: 0.0f,
                TimeFromStart: i,
                MaxSpeed: speeds[i],
                MaxAcceleration: 6.0f,
                MaxDeceleration: 9.0f,
                LateralAcceleration: 0.0f
            );
        }

        return new SpeedProfile(points);
    }

    private static LateralControlOutput CreateStraightLateralOutput(
        int nearestProfileIndex,
        float lateralError = 0.0f,
        float headingError = 0.0f,
        float trackBoundaryExcess = 0.0f
    )
    {
        return new LateralControlOutput(
            SteeringInput: 0.0f,
            SteeringAngle: 0.0f,
            FeedForwardSteeringAngle: 0.0f,
            FeedbackSteeringAngle: 0.0f,
            StabilitySteeringAngle: 0.0f,
            YawDampingSteeringAngle: 0.0f,
            BetaDampingSteeringAngle: 0.0f,
            LookaheadDistance: 20.0f,
            LookaheadPoint: new Vector2(20.0f, 0.0f),
            TargetSpeed: 30.0f,
            TargetLateralAcceleration: 0.0f,
            NearestProfileIndex: nearestProfileIndex,
            ProfileDistance: nearestProfileIndex * 10.0f,
            LateralError: lateralError,
            HeadingError: headingError,
            Beta: 0.0f,
            BetaReference: 0.0f,
            YawRate: 0.0f,
            YawRateReference: 0.0f,
            YawRateError: 0.0f,
            StabilityRisk: 0.0f,
            TrackBoundaryExcess: trackBoundaryExcess
        );
    }

    private static SpeedProfile CreateConstantCurvatureProfile(float curvature, float speed)
    {
        SpeedProfilePoint[] points = new SpeedProfilePoint[8];
        for (int i = 0; i < points.Length; i++)
        {
            float distance = i * 10.0f;
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
                MaxAcceleration: 6.0f,
                MaxDeceleration: 9.0f,
                LateralAcceleration: speed * speed * curvature
            );
        }

        return new SpeedProfile(points);
    }
}
