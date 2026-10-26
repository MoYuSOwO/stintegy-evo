using System.Collections.Generic;
using System.Linq;
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
    }

    private const string CommonColumns =
        "time_s,car_id,controller_status,lap,total_distance_m,s_m,d_m,x_m,y_m,heading_rad," +
        "speed_mps,desired_steer_angle_rad,desired_accel_mps2,front_brake_bias_offset," +
        "actual_accel_mps2,actual_lateral_accel_mps2,actual_curvature_1pm,front_steer_angle_rad," +
        "over_limit,loss_accel_mps2,drive_power_kw,regen_power_kw," +
        "sideslip_angle_rad,yaw_rate_radps,yaw_accel_radps2," +
        "primary_energy,air_temp_c,track_temp_c,fl_surface_temp_c,fr_surface_temp_c," +
        "rl_surface_temp_c,rr_surface_temp_c,fl_core_temp_c,fr_core_temp_c,rl_core_temp_c," +
        "rr_core_temp_c,fl_wear,fr_wear,rl_wear,rr_wear,fl_load_n,fr_load_n,rl_load_n,rr_load_n," +
        "boundary_contact,hit_car";

    // The car's own instruments follow the common columns, named by the
    // first car written; another kind of car fills the ones it shares by
    // name and leaves the rest empty.
    private string[]? _channelIds;
    private float[] _readings = [];

    public void Write(
        float raceTimeSeconds,
        RaceCar car,
        TrackData track,
        RaceEnvironment environment
    )
    {
        ICar model = car.Car;
        if (_channelIds is null)
        {
            _channelIds = model.TelemetryChannels.Select(c => c.Id).ToArray();
            _writer.WriteLine(CommonColumns + string.Concat(_channelIds.Select(id => "," + id)));
        }
        IReadOnlyList<TelemetryChannel> channels = model.TelemetryChannels;
        if (_readings.Length < channels.Count)
            _readings = new float[channels.Count];
        model.ReadTelemetry(car.State, _readings.AsSpan(0, channels.Count));
        var own = new object[_channelIds.Length];
        for (int i = 0; i < own.Length; i++)
        {
            own[i] = "";
            for (int k = 0; k < channels.Count; k++)
                if (channels[k].Id == _channelIds[i])
                    own[i] = _readings[k];
        }

        TrackPose pose = track.Project(car.State.Position);
        CarTelemetry telemetry = car.State.Telemetry;
        DriverInput input = telemetry.Input;
        CarState state = car.State;
        // Stores and tyres only where the car has them; blank where it has not.
        object primary = car.Car is ICarStores stores ? stores.Stores(state).Primary : "";
        var tires = new object[16];
        Array.Fill(tires, "");
        if (car.Car is ICarTires tyres)
        {
            WheelId[] wheels = [WheelId.FrontLeft, WheelId.FrontRight, WheelId.RearLeft, WheelId.RearRight];
            for (int w = 0; w < 4; w++)
            {
                TireReading tire = tyres.Tire(state, wheels[w]);
                tires[w] = tire.SurfaceTempC;
                tires[4 + w] = tire.CoreTempC;
                tires[8 + w] = tire.Wear;
                tires[12 + w] = tire.LoadN;
            }
        }

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
            input.SteerAngleRadians,
            input.DesiredAccel,
            input.FrontBrakeBiasOffset,
            telemetry.ActualLongitudinalAccel,
            telemetry.ActualLateralAccel,
            telemetry.ActualCurvature,
            // The road wheels' angle beside the corner asked for and the one
            // driven: the three together show the steering's answer, which
            // once wound the lock off near the limit unseen.
            state.SteerAngleRadians,
            telemetry.OverLimit,
            telemetry.LossAccel,
            telemetry.DrivePowerWatts * 0.001f,
            telemetry.RegenPowerWatts * 0.001f,
            telemetry.SideslipAngleRadians,
            telemetry.YawRateRadiansPerSecond,
            telemetry.YawAccelerationRadiansPerSecondSquared,
            primary,
            environment.AirTempC,
            environment.TrackTempC,
            tires[0], tires[1], tires[2], tires[3],
            tires[4], tires[5], tires[6], tires[7],
            tires[8], tires[9], tires[10], tires[11],
            tires[12], tires[13], tires[14], tires[15],
            car.LastBoundaryContact.HasValue ? 1 : 0,
            car.HitCarThisStep ? 1 : 0
        );
        WriteOwn(own);
    }

    private void WriteOwn(object[] values)
    {
        if (values.Length == 0)
        {
            _writer.WriteLine();
            return;
        }
        _writer.Write(',');
        WriteRow(values, endLine: true);
    }

    private void WriteValues(params object[] values) => WriteRow(values, endLine: false);

    private void WriteRow(object[] values, bool endLine)
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
        if (endLine)
            _writer.WriteLine();
    }

    public void Dispose() => _writer.Dispose();
}
