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
            "actual_curvature_1pm,front_lateral_use,rear_lateral_use,over_limit,wall_contact"
        );
    }

    public void Write(float raceTimeSeconds, RaceCar car, TrackData track)
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
            car.LastBoundaryContact.HasValue ? 1 : 0
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

    public void Dispose()
    {
        _writer.Dispose();
    }
}
