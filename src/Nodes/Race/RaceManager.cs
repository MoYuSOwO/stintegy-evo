using Godot;
using PloyRacing.Core.Car.Configs;
using PloyRacing.Core.Track;
using PloyRacing.Nodes.Track;
using PloyRacing.Nodes.Car;
using System;
using System.Collections.Generic;
using PloyRacing.Nodes.Car.Controllers;

namespace PloyRacing.Nodes.Race;

public partial class RaceManager : Node2D
{
	[Export] public TrackRenderer? Renderer { get; set; }
	private readonly List<RaceCar> cars = [];

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
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
				startPos = trackData.GridPosToVector2(config.CarGridOrPitIdx);
				int idx = trackData.GridPosToIdx(config.CarGridOrPitIdx);
				startRotation = trackData.Nodes[idx].Tangent.Angle();
			}
			car.Init(config, trackData, DummyEnvironment.Instance, DummyController.Instance, startPos, startRotation);
			AddChild(car);
		}
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}
}
