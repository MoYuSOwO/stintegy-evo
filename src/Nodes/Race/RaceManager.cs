using Godot;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Track;
using StintegyEVO.Nodes.Track;
using StintegyEVO.Nodes.Car;
using System;
using System.Collections.Generic;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Car;
using System.Globalization;

namespace StintegyEVO.Nodes.Race;

public partial class RaceManager : Node2D
{
	[Export] public TrackRenderer? Renderer { get; set; }
	[Export] public bool ShowFrameStats { get; set; }
	private readonly List<RaceCar> cars = [];

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		if (ShowFrameStats)
			AddChild(new FrameTimeMonitor());

		Init(TrackFactory.SimpleTestTrack(), [new CarConfig()]);
	}

	public void Init(TrackData trackData, IList<CarConfig> carConfigs)
	{
		if (Renderer == null) throw new ArgumentNullException("Renderer", "Race have not fully initialized!"); 
		Renderer.Init(trackData);
		foreach (var config in carConfigs)
		{
			RaceCar car = new();
			Vector2 startPos = new();
			float startRotation = 0.0f;
			if (config.IsGrid)
			{
				var grid = trackData.Grids[config.CarGridOrPitIdx];
				startPos = grid.Position;
				int idx = grid.Index;
				startRotation = trackData[idx].Tangent.Angle();
			}
			ConfigureTelemetry(car);
			car.Init(new CarLogic(config, trackData, DummyEnvironment.Instance), Controllers.CreateDefault(), startPos, startRotation, this);
			AddChild(car);
		}
	}

	private static void ConfigureTelemetry(RaceCar car)
	{
		string? telemetryName = System.Environment.GetEnvironmentVariable("STINTEGY_TELEMETRY_NAME");
		if (string.IsNullOrWhiteSpace(telemetryName)) return;

		car.TelemetryName = telemetryName;
		car.TelemetryFrameStride = ParseEnvInt("STINTEGY_TELEMETRY_STRIDE", 1);
	}

	private static int ParseEnvInt(string name, int fallback)
	{
		string? value = System.Environment.GetEnvironmentVariable(name);
		return int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsed) ? Mathf.Max(parsed, 1) : fallback;
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}
}
