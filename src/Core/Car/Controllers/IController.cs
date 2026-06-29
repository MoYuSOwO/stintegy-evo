using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Godot;
using StintegyEVO.Core.Track;

namespace StintegyEVO.Core.Car.Controllers;

public struct CarSensor
{
    public float Mass;
    public Vector2 LinearVelocity;
    public float AngularVelocity;
    public Vector2 Position;
    public float Rotation;
    public Vector2 LocalAccel;
    public IntermediateParams Params;
}

public interface IController
{
    public float Input { get; }
    public float Steer { get; }

    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }

    public abstract void Init(CarLogic carLogic, TrackData track);
    public abstract void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track);
    public abstract void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track);
}

public interface IControllerTelemetry
{
    public IEnumerable<string> TelemetryColumns { get; }
    public void AppendTelemetryValues(StringBuilder builder);
}

public readonly record struct ControllerDebugPathStyle(Color Color, float Width, int ZIndex)
{
    public static ControllerDebugPathStyle Default => new(Color.FromHtml("#65c4ff"), 1.0f, 50);
}

public interface IControllerDebugPaths
{
    public int DebugPathLineCount { get; }
    public int GetDebugPathPointCount(int lineIndex);
    public Vector2 GetDebugPathPoint(int lineIndex, int pointIndex);
    public ControllerDebugPathStyle GetDebugPathStyle(int lineIndex) => ControllerDebugPathStyle.Default;
    public long GetDebugPathVersion(int lineIndex) => -1;
}

public static class TelemetryCsv
{
    public static void Append(StringBuilder builder, float value)
    {
        AppendSeparator(builder);
        Span<char> buffer = stackalloc char[32];
        if (value.TryFormat(buffer, out int written, "0.######", CultureInfo.InvariantCulture))
            builder.Append(buffer[..written]);
        else
            builder.Append(value.ToString("0.######", CultureInfo.InvariantCulture));
    }

    public static void Append(StringBuilder builder, int value)
    {
        AppendSeparator(builder);
        Span<char> buffer = stackalloc char[16];
        if (value.TryFormat(buffer, out int written, provider: CultureInfo.InvariantCulture))
            builder.Append(buffer[..written]);
        else
            builder.Append(value.ToString(CultureInfo.InvariantCulture));
    }

    public static void Append(StringBuilder builder, bool value)
    {
        AppendSeparator(builder);
        builder.Append(value ? '1' : '0');
    }

    private static void AppendSeparator(StringBuilder builder)
    {
        if (builder.Length > 0)
            builder.Append(',');
    }
}

public static class Controllers
{
    public static readonly IController Dummy = DummyController.Instance;

    public static IController CreateDefault() => Dummy;
}

public class DummyController : IController
{
    public static readonly DummyController Instance = new();

    private DummyController() {}

    public float Input => 0.5f;
    public float Steer => 0.3f;

    public float FuelSaveFactor { get; set; }
    public float TireSaveFactor { get; set; }

    public void Init(CarLogic carLogic, TrackData track)
    {
    }

    public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }

    public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
    {
    }
}
