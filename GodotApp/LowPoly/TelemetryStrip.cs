using System;
using System.Collections.Generic;
using Godot;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Racing;

namespace StintegyEVO.GodotApp.LowPoly;

/// <summary>
/// Rolling throttle / brake / steer traces for the selected car.
/// </summary>
public partial class TelemetryStrip : Control
{
    public const float WindowSeconds = 8f;
    private const float Header = 22f;

    /// <summary>
    /// The steer trace is the wheel in the driver's hands, not the front
    /// wheels: a single-seater turns its road wheels only a few degrees in a
    /// quick corner, which drew a flat line. Twelve to one puts the slowest
    /// hairpins at about half a turn, where a formula car's wheel is about
    /// at lock. Display only; the physics steers the front wheels.
    /// </summary>
    private const float SteeringRatio = 12f;
    private const float SteerScaleDegrees = 180f;

    private readonly List<(float T, float Throttle, float Brake, float Steer)> _samples = [];
    private readonly Label[] _captions = new Label[3];
    private static readonly Color Throttle = Color.FromHtml("#3e6653");
    private static readonly Color Brake = LowPolyMesh.Vermilion;
    private static readonly Color Steer = Color.FromHtml("#456583");
    private static readonly Color Rule = Color.FromHtml("#d2d2c5");
    private static readonly Color Muted = Color.FromHtml("#647269");

    public TelemetryStrip()
    {
        MouseFilter = MouseFilterEnum.Ignore;
        ClipContents = true;
        string[] names = ["THR", "BRK", "STR"];
        for (int i = 0; i < names.Length; i++)
        {
            _captions[i] = new Label
            {
                Text = names[i],
                MouseFilter = MouseFilterEnum.Ignore
            };
            _captions[i].AddThemeFontSizeOverride("font_size", 10);
            _captions[i].AddThemeColorOverride("font_color", Muted);
            AddChild(_captions[i]);
        }
    }

    public void Push(float time, RaceCar car)
    {
        // Pedal positions, not accelerations: the throttle as a share of what
        // the motor can give at this speed, so flat out reads full at 75 m/s
        // as it does at 20, and the brake as a share of the car's full brake.
        float accel = car.LastInput.DesiredAccel;
        // What the car says it can give, not one chassis's numbers; a car
        // that says nothing is read against its brakes.
        float brakeFull = MathF.Max(0.1f, car.Car.Spec.MaximumBrakeDeceleration);
        float motor = car.Car is ICarEnvelope envelope
            ? MathF.Max(0.1f, envelope.Capabilities(car.State, car.Strategy).AvailableDriveAcceleration)
            : brakeFull;
        float steerDegrees = car.State.SteerAngleRadians * SteeringRatio * 180f / MathF.PI;
        _samples.Add((
            time,
            Mathf.Clamp(accel / motor, 0f, 1f),
            Mathf.Clamp(-accel / brakeFull, 0f, 1f),
            Mathf.Clamp(steerDegrees / SteerScaleDegrees, -1f, 1f)
        ));
        _captions[2].Text = $"STR {steerDegrees:+0;-0}°\n±{SteerScaleDegrees:0}°";
        float floor = time - WindowSeconds;
        int drop = 0;
        while (drop < _samples.Count && _samples[drop].T < floor)
            drop++;
        if (drop > 0)
            _samples.RemoveRange(0, drop);
        QueueRedraw();
    }

    // Laid out on entering the scene as well as on a resize: a size set
    // before the strip is in the tree sends no resize, and left every
    // caption stacked in the top corner.
    public override void _Notification(int what)
    {
        if (what != NotificationResized && what != NotificationReady)
            return;
        float band = (Size.Y - Header) / 3f;
        for (int i = 0; i < _captions.Length; i++)
            _captions[i].Position = new Vector2(0, Header + i * band + 6);
    }

    public override void _Draw()
    {
        float band = (Size.Y - Header) / 3f;
        DrawChannel(0, band, sample => sample.Throttle, 0f, 1f, Throttle, midline: false);
        DrawChannel(1, band, sample => sample.Brake, 0f, 1f, Brake, midline: false);
        DrawChannel(2, band, sample => sample.Steer, -1f, 1f, Steer, midline: true);
    }

    private void DrawChannel(
        int index,
        float band,
        Func<(float T, float Throttle, float Brake, float Steer), float> channel,
        float min,
        float max,
        Color color,
        bool midline
    )
    {
        float top = Header + index * band;
        var area = new Rect2(62, top + 4, Size.X - 70, band - 8);
        DrawLine(new Vector2(0, top), new Vector2(Size.X, top), Rule, 1f);
        if (midline)
        {
            float mid = area.Position.Y + area.Size.Y * 0.5f;
            DrawLine(new Vector2(area.Position.X, mid), new Vector2(area.End.X, mid), Rule, 1f);
        }
        if (_samples.Count < 2)
            return;
        float t1 = _samples[^1].T;
        float t0 = t1 - WindowSeconds;
        var points = new Vector2[_samples.Count];
        for (int i = 0; i < _samples.Count; i++)
        {
            var sample = _samples[i];
            float x = area.Position.X + (sample.T - t0) / WindowSeconds * area.Size.X;
            float unit = (channel(sample) - min) / (max - min);
            points[i] = new Vector2(x, area.End.Y - unit * area.Size.Y);
        }
        DrawPolyline(points, color, 1.6f, true);
    }
}
