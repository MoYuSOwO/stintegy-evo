using System;
using System.Globalization;
using System.IO;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;

namespace StintegyEVO.GodotApp.Race;

/// <summary>
/// Writes controller-independent physical telemetry.  The presentation layer
/// does not know how a car was controlled; it records the input that Core
/// applied and the resulting vehicle state instead.
/// </summary>
internal sealed class RaceCsvTelemetryRecorder : IDisposable
{
    private readonly StreamWriter _writer;

    public RaceCsvTelemetryRecorder(string path)
    {
        string? directory = Path.GetDirectoryName(path);
        if (!string.IsNullOrWhiteSpace(directory))
            Directory.CreateDirectory(directory);

        _writer = new StreamWriter(path, false);
        _writer.WriteLine(
            "time_s,car_id,controller_status,lap,total_distance_m,s_m,d_m,x_m,y_m,heading_rad," +
            "speed_mps,desired_curvature_1pm,desired_accel_mps2,front_brake_bias_offset," +
            "actual_accel_mps2,actual_lateral_accel_mps2,actual_curvature_1pm," +
            "front_lateral_use,rear_lateral_use,front_longitudinal_use,rear_longitudinal_use," +
            "over_limit,loss_accel_mps2,drive_power_kw,regen_power_kw,traction_control_cut_mps2," +
            "sideslip_angle_rad,rear_slide_severity,yaw_rate_radps,yaw_accel_radps2," +
            "primary_energy,air_temp_c,track_temp_c,fl_surface_temp_c,fr_surface_temp_c," +
            "rl_surface_temp_c,rr_surface_temp_c,fl_core_temp_c,fr_core_temp_c,rl_core_temp_c," +
            "rr_core_temp_c,fl_wear,fr_wear,rl_wear,rr_wear,fl_load_n,fr_load_n,rl_load_n,rr_load_n," +
            "boundary_contact,hit_car"
        );
    }

    public void Write(
        float raceTimeSeconds,
        RaceCar car,
        TrackData track,
        RaceEnvironment environment
    )
    {
        TrackPose pose = track.Project(car.State.Position);
        CarTelemetry telemetry = car.State.Telemetry;
        DriverInput input = telemetry.Input;
        CarState state = car.State;

        WriteValues(
            raceTimeSeconds,
            car.Id,
            car.Driver == null ? "No controller" : "Controller attached",
            car.Progress.Lap,
            car.Progress.TotalDistance,
            pose.S,
            pose.D,
            state.Position.X,
            state.Position.Y,
            state.Heading,
            state.Speed,
            input.DesiredCurvature,
            input.DesiredAccel,
            input.FrontBrakeBiasOffset,
            telemetry.ActualLongitudinalAccel,
            telemetry.ActualLateralAccel,
            telemetry.ActualCurvature,
            telemetry.FrontLateralUse,
            telemetry.RearLateralUse,
            telemetry.FrontLongitudinalUse,
            telemetry.RearLongitudinalUse,
            telemetry.OverLimit,
            telemetry.LossAccel,
            telemetry.DrivePowerWatts * 0.001f,
            telemetry.RegenPowerWatts * 0.001f,
            telemetry.TractionControlCutAccel,
            telemetry.SideslipAngleRadians,
            telemetry.RearSlideSeverity,
            telemetry.YawRateRadiansPerSecond,
            telemetry.YawAccelerationRadiansPerSecondSquared,
            state.Energy.Primary,
            environment.AirTempC,
            environment.TrackTempC,
            state.FrontLeft.SurfaceTempC,
            state.FrontRight.SurfaceTempC,
            state.RearLeft.SurfaceTempC,
            state.RearRight.SurfaceTempC,
            state.FrontLeft.CoreTempC,
            state.FrontRight.CoreTempC,
            state.RearLeft.CoreTempC,
            state.RearRight.CoreTempC,
            state.FrontLeft.Wear,
            state.FrontRight.Wear,
            state.RearLeft.Wear,
            state.RearRight.Wear,
            state.FrontLeft.LoadN,
            state.FrontRight.LoadN,
            state.RearLeft.LoadN,
            state.RearRight.LoadN,
            car.LastBoundaryContact.HasValue ? 1 : 0,
            car.HitCarThisStep ? 1 : 0
        );
    }

    private void WriteValues(params object[] values)
    {
        for (int i = 0; i < values.Length; i++)
        {
            if (i > 0)
                _writer.Write(',');

            if (values[i] is float value)
                _writer.Write(value.ToString("G9", CultureInfo.InvariantCulture));
            else
            {
                string text = Convert.ToString(values[i], CultureInfo.InvariantCulture) ?? "";
                if (text.IndexOfAny([',', '"', '\r', '\n']) >= 0)
                    _writer.Write("\"" + text.Replace("\"", "\"\"") + "\"");
                else
                    _writer.Write(text);
            }
        }
        _writer.WriteLine();
    }

    public void Dispose() => _writer.Dispose();
}
