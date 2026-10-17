using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Drivers;

/// <summary>
/// Follows one baked path. The speed table is that path's own curvature,
/// run through this car's envelope, so the speed at a station is already
/// the speed that can still brake for the corner ahead. Curvature is the
/// line's, plus whatever lateral and heading error the road will allow.
/// The bake's v* is ignored: it belongs to a different car.
/// </summary>
public class RuleLineController : IDriverController
{
    public const float LaunchStraightenSpeed = 15f;
    private const float DynamicYawClearSpeed = 8f;
    private const int RecomputeEverySteps = 6;

    private readonly RuleTrackPack _pack;
    private RuleTrackPack _active;
    private readonly CarConfig _car;
    private readonly TireConfig _tires;
    private readonly CarState _plan = new();
    private GeometricRacingLine? _line;
    private LineSpeedProfile? _speed;
    private DriverTrackView? _track;
    private DriverExecution _execution = DriverExecution.Full;
    private float _commandCurvature;
    private bool _clearedLaunch;
    private int _recomputeAge = RecomputeEverySteps;

    public RuleLineController(CarConfig? car = null, TireConfig? tires = null)
        : this(null, car, tires)
    {
    }

    public RuleLineController(RuleTrackPack? pack, CarConfig? car = null, TireConfig? tires = null)
    {
        _pack = pack ?? RuleTrackPack.Silverstone;
        _active = _pack;
        _car = car ?? new CarConfig();
        _tires = tires ?? new TireConfig();
    }

    public GeometricRacingLine? Line => _line;

    public void Initialize(in DriverContext context)
    {
        _track = context.Track;
        _active = _pack.Matches(context.Track.LengthMeters)
            ? _pack
            : RuleTrackPack.For(context.Track.LengthMeters);
        float halfWidth = MathF.Max(0.4f, context.Car.WidthMeters * 0.5f);
        _line = RuleLineAssets.LoadOrBake(_active, context.Track, halfWidth);
        _execution = DriverExecution.From(context.Profile.Abilities);
        _speed = LineSpeedProfile.FromLine(
            _line,
            context.Track,
            includeRoadCurvature: false,
            windowMeters: _active.KappaWindowMeters,
            usePathCurvature: true,
            curvatureScale: _active.CurvatureScale
        );
        HydratePlan(context.Car);
        _speed.Recompute(
            _plan,
            _car,
            _tires,
            CarPhysics.GripUsage(context.Car.Strategy),
            CornerShare(),
            topSpeedMetersPerSecond: _active.TopSpeedMetersPerSecond,
            brakeUsage: _active.BrakeUsage
        );
    }

    public DriverInput GetControl(in DriverContext context, float dt)
    {
        DriverTrackView track = _track ?? context.Track;
        GeometricRacingLine line = _line ?? GeometricRacingLine.Bake(track, 0.95f);
        RaceCarSnapshot car = context.Car;
        HydratePlan(car);

        float speed = car.SpeedMetersPerSecond;
        float grip = CarPhysics.GripUsage(car.Strategy);
        if (!_clearedLaunch)
        {
            if (speed < DynamicYawClearSpeed)
            {
                TrackSample road = track.Sample(car.TrackS);
                return new DriverInput(
                    road.Curvature,
                    Math.Clamp(4.5f, 0f, car.Capabilities.AvailableDriveAcceleration)
                );
            }

            if (speed < LaunchStraightenSpeed)
                return HoldRoad(car, track, speed, grip);

            _clearedLaunch = true;
        }

        LineSpeedProfile profile = _speed ?? LineSpeedProfile.FromLine(
            line,
            track,
            includeRoadCurvature: false,
            windowMeters: _active.KappaWindowMeters,
            usePathCurvature: true,
            curvatureScale: _active.CurvatureScale
        );
        if (++_recomputeAge >= RecomputeEverySteps)
        {
            profile.Recompute(
                _plan,
                _car,
                _tires,
                grip,
                CornerShare(),
                topSpeedMetersPerSecond: _active.TopSpeedMetersPerSecond,
                brakeUsage: _active.BrakeUsage
            );
            _recomputeAge = 0;
        }

        return FollowLine(car, track, line, profile, speed, grip, dt);
    }

    private DriverInput FollowLine(
        RaceCarSnapshot car,
        DriverTrackView track,
        GeometricRacingLine line,
        LineSpeedProfile profile,
        float speed,
        float grip,
        float dt
    )
    {
        float s = car.TrackS;
        FrenetTracking.Sample tracking = FrenetTracking.AtOffset(
            car,
            line,
            track,
            s,
            car.TrackD,
            car.VelocityHeadingRadians,
            speed,
            _execution.LateralBiasMeters,
            lateralGain: 0.55f,
            headingGain: 0.35f,
            lateralDamping: 0.22f,
            previewSeconds: 0.15f,
            maximumPreviewMeters: 8f
        );
        CarPerformanceLimits straight = CarPhysics.EstimatePerformanceLimits(
            _plan,
            _car,
            _tires,
            car.Strategy,
            speed,
            curvature: 0f,
            grip
        );
        float desired = FrenetTracking.ClampToEnvelope(
            tracking.UnconstrainedCurvature,
            speed,
            straight.LateralAccelerationLimit,
            car.Capabilities.MaxCurvatureRequest,
            CornerShare()
        );
        float wheelBase = MathF.Max(car.WheelBaseMeters, 1.5f);
        float step = car.Capabilities.SteerRateLimitRadiansPerSecond /
                     wheelBase *
                     MathF.Max(dt, 1f / 120f);
        _commandCurvature += Math.Clamp(desired - _commandCurvature, -step, step);
        float kappa = _commandCurvature;
        float slip = MathF.Abs(car.SideslipAngleRadians);
        float unwind = Math.Clamp((slip - 0.22f) / 0.08f, 0f, 1f) * _execution.ControlAuthority;
        if (unwind > 0f)
            kappa += (tracking.LineCurvature - kappa) * unwind;

        float vStar = profile.At(track.WrapS, s + _execution.BrakeMarkerErrorMeters);
        CarPerformanceLimits now = CarPhysics.EstimatePerformanceLimits(
            _plan,
            _car,
            _tires,
            car.Strategy,
            speed,
            kappa,
            grip
        );
        float error = vStar - speed;
        float accel = error < -1f
            ? -now.MaximumBrakeDeceleration
            : _active.SpeedGain * error;
        accel = Math.Clamp(accel, -now.MaximumBrakeDeceleration, now.MaximumDriveAcceleration);
        if (slip > 0.28f)
            accel = MathF.Min(accel, 0f);
        return new DriverInput(kappa, accel);
    }

    private float CornerShare() => _active.GgUsage * _execution.PaceEfficiency;

    private DriverInput HoldRoad(
        RaceCarSnapshot car,
        DriverTrackView track,
        float speed,
        float grip
    )
    {
        FrenetTracking.Sample tracking = FrenetTracking.AtRoad(car, track, speed);
        CarPerformanceLimits circle = CarPhysics.EstimatePerformanceLimits(
            _plan,
            _car,
            _tires,
            car.Strategy,
            speed,
            curvature: 0f,
            grip
        );
        float curvature = FrenetTracking.ClampToEnvelope(
            tracking.UnconstrainedCurvature,
            speed,
            circle.LateralAccelerationLimit,
            car.Capabilities.MaxCurvatureRequest
        );
        CarPerformanceLimits now = CarPhysics.EstimatePerformanceLimits(
            _plan,
            _car,
            _tires,
            car.Strategy,
            speed,
            curvature,
            grip
        );
        return new DriverInput(
            curvature,
            Math.Clamp(
                MathF.Min(now.MaximumDriveAcceleration, 2.8f),
                0f,
                car.Capabilities.AvailableDriveAcceleration
            )
        );
    }

    private void HydratePlan(RaceCarSnapshot car)
    {
        _plan.Position = car.Position;
        _plan.Heading = car.HeadingRadians;
        _plan.SideslipAngleRadians = car.SideslipAngleRadians;
        _plan.YawRateRadiansPerSecond = car.YawRateRadiansPerSecond;
        _plan.Speed = car.SpeedMetersPerSecond;
        _plan.SteerAngleRadians = car.SteerAngleRadians;
        _plan.Spinning = car.Spinning;
        _plan.SpinEvents = car.SpinEvents;
        _plan.AirVelocityDeficit = car.AirVelocityDeficit;
        _plan.DownforceVelocityDeficit = car.DownforceVelocityDeficit;
        _plan.WakeDownforceLoss = car.WakeDownforceLoss;
        _plan.ActiveAero = car.ActiveAero;
        if (car.Resources.Length > 0)
            _plan.Energy = PowertrainState.Filled(car.Resources[0].Fraction);
        CopyTire(_plan.FrontLeft, car.FrontLeft);
        CopyTire(_plan.FrontRight, car.FrontRight);
        CopyTire(_plan.RearLeft, car.RearLeft);
        CopyTire(_plan.RearRight, car.RearRight);
    }

    private static void CopyTire(TireState dst, TireSnapshot src)
    {
        dst.SurfaceTempC = src.SurfaceTempC;
        dst.CoreTempC = src.CoreTempC;
        dst.Wear = src.Wear;
        dst.LoadN = src.LoadN;
        dst.SurfaceGrip = src.SurfaceGrip;
    }
}
