using System;
using System.Globalization;
using System.IO;
using System.Text;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.GodotApp.Race;

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
            "predicted_path_length_m,predicted_path_max_curvature_1pm," +
            "predicted_terminal_lateral_error_m,dynamic_prediction_length_m," +
            "joins_reference_line,reference_line_join_curvature_delta_1pm," +
            "path_prediction_ms,rolling_speed_plan_ms," +
            "traffic_constraint,traffic_opponent,traffic_constraint_distance_m," +
            "traffic_target_speed_mps,traffic_conflict_time_s,traffic_clearance_m," +
            "reference_path_speed_mps," +
            "target_speed_mps,actual_speed_mps,a_ref_mps2,loss_compensation_mps2," +
            "grade_compensation_mps2," +
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
            "yaw_rate_radps,yaw_accel_radps2," +
            "driver_pace,driver_consistency,driver_control,driver_tire_management," +
            "driver_adaptability,session_form,lap_form,segment_form,planning_pace," +
            "effective_pace,pace_efficiency,effective_control,effective_tire_management," +
            "tire_energy_efficiency,effective_adaptability,actual_grip,estimated_grip," +
            "estimated_grip_scale,brake_marker_error_m,lateral_target_error_m," +
            "local_speed_error_fraction,front_brake_bias_offset,control_severity," +
            "control_curvature_correction," +
            "driver_recovering"
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
        float metAir = 1f - Math.Clamp(car.State.AirVelocityDeficit, 0f, 1f);
        float aeroLoss = speed > 0.01f
            ? car.CarConfig.AeroDragAccelPerSpeedSquared * speed * speed *
              metAir * metAir *
              (1f - car.CarConfig.OvertakeAssistDragReduction *
               Math.Clamp(car.State.OvertakeAssist, 0f, 1f))
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
            control.PredictedPathLengthMeters,
            control.PredictedPathMaximumCurvature,
            control.PredictedTerminalLateralErrorMeters,
            control.DynamicPredictionLengthMeters,
            control.JoinsReferenceLine ? 1 : 0,
            control.ReferenceLineJoinCurvatureDelta,
            control.PathPredictionMilliseconds,
            control.RollingSpeedPlanningMilliseconds,
            control.TrafficConstraintKind,
            control.TrafficOpponentId ?? string.Empty,
            control.TrafficConstraintDistanceMeters,
            control.TrafficTargetSpeedMetersPerSecond,
            control.TrafficConflictTimeSeconds,
            control.TrafficCurrentClearanceMeters,
            control.ReferencePathTargetSpeed,
            control.TargetSpeed,
            car.State.Speed,
            control.ReferenceAcceleration,
            control.LossCompensationAcceleration,
            control.GradeCompensationAcceleration,
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
            physics.YawAccelerationRadiansPerSecondSquared,
            control.PaceRating,
            control.ConsistencyRating,
            control.CarControlRating,
            control.TireManagementRating,
            control.AdaptabilityRating,
            control.SessionForm,
            control.LapForm,
            control.SegmentForm,
            control.PlanningPace,
            control.EffectivePace,
            control.PaceEfficiency,
            control.EffectiveControl,
            control.EffectiveTireManagement,
            control.TireEnergyEfficiency,
            control.EffectiveAdaptability,
            control.ActualGrip,
            control.EstimatedGrip,
            control.EstimatedGripScale,
            control.BrakeMarkerErrorMeters,
            control.LateralTargetErrorMeters,
            control.LocalSpeedErrorFraction,
            control.FrontBrakeBiasOffset,
            control.ControlSeverity,
            control.ControlCurvatureCorrection,
            control.IsRecovering ? 1 : 0
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
