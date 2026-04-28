using Godot;
using StintegyEVO.Core.Car.Configs;
using StintegyEVO.Core.Track;
using StintegyEVO.Nodes.Track;
using StintegyEVO.Nodes.Car;
using System;
using System.Collections.Generic;
using StintegyEVO.Core.Car.Controllers;
using StintegyEVO.Core.Car;

namespace StintegyEVO.Nodes.Race;

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
				var grid = trackData.Grids[config.CarGridOrPitIdx];
				startPos = grid.Position;
				int idx = grid.Index;
				startRotation = trackData[idx].Tangent.Angle();
			}
			car.Init(new CarLogic(config, trackData, DummyEnvironment.Instance), DummyController.Instance, startPos, startRotation, this);
			AddChild(car);
		}
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}
}
