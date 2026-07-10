using System;
using System.Globalization;
using System.IO;
using System.Text;
using TheStint.Core.Cars;
using TheStint.Core.Racing;
using TheStint.Core.Track;

namespace StintegyEVO.Presentation.Race;

internal sealed class RaceCsvTelemetryRecorder : IDisposable
{
    private const float CurvatureProbeMeters = 1f;

    private readonly StreamWriter _writer;

    public RaceCsvTelemetryRecorder(string path)
    {
        string? directory = Path.GetDirectoryName(path);
        if (!string.IsNullOrEmpty(directory))
            Directory.CreateDirectory(directory);

        _writer = new StreamWriter(
            path,
            append: false,
            encoding: new UTF8Encoding(encoderShouldEmitUTF8Identifier: false)
        );
        _writer.WriteLine(
            "time_s,lap,total_distance_m,s_m,d_m,lateral_error_m,heading_error_rad," +
            "ref_curvature_1pm,preview_curvature_1pm,curvature_gradient_1pm2," +
            "curvature_step_delta_1pm,desired_curvature_1pm,curvature_correction_1pm," +
            "correction_decay_m,correction_envelope_max_curvature_1pm,speed_plan_ms," +
            "global_profile_speed_mps," +
            "target_speed_mps,actual_speed_mps,a_ref_mps2,loss_compensation_mps2," +
            "speed_feedback_mps2," +
            "command_accel_mps2,actual_accel_mps2,actual_lateral_accel_mps2," +
            "actual_curvature_1pm,front_lateral_use,rear_lateral_use,over_limit,wall_contact," +
            "physics_loss_mps2,rolling_loss_mps2,aero_loss_mps2,cornering_scrub_mps2," +
            "sideslip_loss_mps2,traction_control_cut_mps2," +
            "front_longitudinal_use,rear_longitudinal_use,drive_power_kw,regen_power_kw," +
            "battery_soc,air_temp_c,track_temp_c," +
            "front_surface_temp_c,rear_surface_temp_c," +
            "front_core_temp_c,rear_core_temp_c,front_wear,rear_wear," +
            "fl_surface_temp_c,fr_surface_temp_c,rl_surface_temp_c,rr_surface_temp_c," +
            "fl_core_temp_c,fr_core_temp_c,rl_core_temp_c,rr_core_temp_c," +
            "fl_wear,fr_wear,rl_wear,rr_wear," +
            "fl_load_n,fr_load_n,rl_load_n,rr_load_n," +
            "sideslip_angle_rad,rear_slide_severity,reference_yaw_rate_radps," +
            "yaw_rate_radps,yaw_accel_radps2"
        );
    }

    public void Write(
        float raceTimeSeconds,
        RaceCar car,
        TrackData track,
        RaceEnvironment environment
    )
    {
        if (car.Driver is not ReferenceLineDriver driver)
            return;

        ReferenceLineDriverTelemetry control = driver.LastTelemetry;
        CarTelemetry physics = car.State.Telemetry;
        float s = car.Progress.CurrentS;
        float previousCurvature = track.Sample(s - CurvatureProbeMeters).RefCurvature;
        float currentCurvature = track.Sample(s).RefCurvature;
        float nextCurvature = track.Sample(s + CurvatureProbeMeters).RefCurvature;
        float curvatureGradient = (nextCurvature - previousCurvature) /
                                  (2f * CurvatureProbeMeters);
        float curvatureStepDelta = MathF.Max(
            MathF.Abs(currentCurvature - previousCurvature),
            MathF.Abs(nextCurvature - currentCurvature)
        );
        float speed = car.State.Speed;
        float rollingLoss = speed > 0.01f ? car.CarConfig.RollingDragAccel : 0f;
        float aeroLoss = speed > 0.01f
            ? car.CarConfig.AeroDragAccelPerSpeedSquared * speed * speed
            : 0f;
        float corneringScrub = MathF.Max(
            0f,
            physics.LossAccel - rollingLoss - aeroLoss - physics.SideslipLossAccel
        );
        float frontSurfaceTemp = Average(car.State.FrontLeft.SurfaceTempC, car.State.FrontRight.SurfaceTempC);
        float rearSurfaceTemp = Average(car.State.RearLeft.SurfaceTempC, car.State.RearRight.SurfaceTempC);
        float frontCoreTemp = Average(car.State.FrontLeft.CoreTempC, car.State.FrontRight.CoreTempC);
        float rearCoreTemp = Average(car.State.RearLeft.CoreTempC, car.State.RearRight.CoreTempC);
        float frontWear = Average(car.State.FrontLeft.Wear, car.State.FrontRight.Wear);
        float rearWear = Average(car.State.RearLeft.Wear, car.State.RearRight.Wear);

        object[] values =
        [
            raceTimeSeconds,
            car.Progress.Lap,
            car.Progress.TotalDistance,
            s,
            car.Progress.CurrentD,
            control.LateralErrorMeters,
            control.HeadingErrorRadians,
            control.ReferenceCurvature,
            control.PreviewCurvature,
            curvatureGradient,
            curvatureStepDelta,
            control.DesiredCurvature,
            control.CurvatureCorrection,
            control.CorrectionDecayDistanceMeters,
            control.CorrectionEnvelopeMaximumCurvature,
            control.CorrectionSpeedPlanningMilliseconds,
            control.GlobalProfileTargetSpeed,
            control.TargetSpeed,
            car.State.Speed,
            control.ReferenceAcceleration,
            control.LossCompensationAcceleration,
            control.SpeedFeedbackAcceleration,
            control.DesiredAcceleration,
            physics.ActualLongitudinalAccel,
            physics.ActualLateralAccel,
            physics.ActualCurvature,
            physics.FrontLateralUse,
            physics.RearLateralUse,
            physics.OverLimit,
            car.LastBoundaryContact.HasValue ? 1 : 0,
            physics.LossAccel,
            rollingLoss,
            aeroLoss,
            corneringScrub,
            physics.SideslipLossAccel,
            physics.TractionControlCutAccel,
            physics.FrontLongitudinalUse,
            physics.RearLongitudinalUse,
            physics.DrivePowerWatts * 0.001f,
            physics.RegenPowerWatts * 0.001f,
            car.State.BatterySoc,
            environment.AirTempC,
            environment.TrackTempC,
            frontSurfaceTemp,
            rearSurfaceTemp,
            frontCoreTemp,
            rearCoreTemp,
            frontWear,
            rearWear,
            car.State.FrontLeft.SurfaceTempC,
            car.State.FrontRight.SurfaceTempC,
            car.State.RearLeft.SurfaceTempC,
            car.State.RearRight.SurfaceTempC,
            car.State.FrontLeft.CoreTempC,
            car.State.FrontRight.CoreTempC,
            car.State.RearLeft.CoreTempC,
            car.State.RearRight.CoreTempC,
            car.State.FrontLeft.Wear,
            car.State.FrontRight.Wear,
            car.State.RearLeft.Wear,
            car.State.RearRight.Wear,
            car.State.FrontLeft.LoadN,
            car.State.FrontRight.LoadN,
            car.State.RearLeft.LoadN,
            car.State.RearRight.LoadN,
            physics.SideslipAngleRadians,
            physics.RearSlideSeverity,
            physics.ReferenceYawRateRadiansPerSecond,
            physics.YawRateRadiansPerSecond,
            physics.YawAccelerationRadiansPerSecondSquared
        ];

        for (int i = 0; i < values.Length; i++)
        {
            if (i > 0)
                _writer.Write(',');
            if (values[i] is float value)
                _writer.Write(value.ToString("G9", CultureInfo.InvariantCulture));
            else
                _writer.Write(Convert.ToString(values[i], CultureInfo.InvariantCulture));
        }
        _writer.WriteLine();
    }

    private static float Average(float left, float right)
    {
        return (left + right) * 0.5f;
    }

    public void Dispose()
    {
        _writer.Dispose();
    }
}
