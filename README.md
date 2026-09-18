<p align="center">
  <img src="logo.svg" alt="StintegyEVO" width="500"/>
</p>

# StintegyEVO — Racing Simulation Core

[中文正式版](README_zh.md)

StintegyEVO is a real-time racing simulation: vehicle and tyre physics, energy and wear, tracks, race stepping, and a Godot presentation layer. It ships the world, not the drivers. Anything that decides how a car is driven, whether rule-based, learned or human, plugs in from outside through one small interface.

**Open source, not community-driven.** The code is public under the AGPL and you are welcome to read, run, fork and build on it. The project is developed by its maintainer on their own schedule; issues and pull requests may be read, but there is no promise that they will be answered, reviewed or merged.

> We do not compete on who drives faster. We compete on who calculates better, while keeping the world model honest.

## Quick start: a car that moves

You need the [.NET 8 SDK](https://dotnet.microsoft.com/download). For the 3D view you also need [Godot 4.6 .NET](https://godotengine.org/download).

```bash
git clone https://github.com/MoYuSOwO/stintegy-evo.git
```

```bash
cd stintegy-evo && dotnet test Core/Tests/StintegyEVO.Core.Tests.csproj -c Release
```

The world never drives itself. To see a car go round, give it a controller. This one aims at a point on the centreline ahead and slows for the curvature it can see coming:

```csharp
sealed class CentrelineFollower : IDriverController
{
    public DriverInput GetControl(in DriverContext context, float dt)
    {
        RaceCarSnapshot car = context.Car;
        float lookahead = 12f + 0.6f * car.SpeedMetersPerSecond;
        Vector2 aim = context.Track.Sample(car.TrackS + lookahead).Center;

        // Pure pursuit: the arc through the aim point, in the car's frame.
        Vector2 offset = aim - car.Position;
        float left = -MathF.Sin(car.HeadingRadians) * offset.X +
                     MathF.Cos(car.HeadingRadians) * offset.Y;
        float curvature = 2f * left / MathF.Max(offset.LengthSquared(), 1f);

        // Slow for the tightest bend in the next 200 m.
        float tightest = 0f;
        for (float d = 0f; d <= 200f; d += 10f)
            tightest = MathF.Max(
                tightest,
                MathF.Abs(context.Track.Sample(car.TrackS + d).Curvature)
            );
        float target = MathF.Min(60f, MathF.Sqrt(14f / MathF.Max(tightest, 1e-4f)));
        float accel = Math.Clamp(2f * (target - car.SpeedMetersPerSecond), -20f, 8f);
        return new DriverInput(curvature, accel);
    }
}
```

Put it in a car and step the world:

```csharp
TrackData track = TrackFactory.SilverstoneStyleTestTrack();
var simulation = new RaceSimulation(track);
var profile = new DriverProfile("example", new DriverAbilities());
TrackSample start = track.Sample(track.StartingLineS);
var car = new RaceCar(
    "car-1",
    new CarConfig(),
    new TireConfig(),
    new Driver(profile, new CentrelineFollower()),
    new CarState { Position = start.Center, Heading = start.Heading }
);
simulation.AddCar(car);
for (int i = 0; i < 60 * 240; i++)   // four simulated minutes
    simulation.Step(1f / 60f);
```

The example is deliberately slow. It completes a lap of the 5.9 km circuit inside those four minutes with no spins. This exact code is a test, [`Core/Tests/ReadmeExampleTests.cs`](Core/Tests/ReadmeExampleTests.cs), so it cannot silently stop working.

To watch it, open the project in Godot. The main scene, `Levels/lowpoly.tscn`, is a stationary preview until a host hands it a world: build the simulation as above and call `RaceView3D.BindSimulation(simulation)` before the view enters the scene tree. [`Tools/Tests/CompositionSmoke.cs`](Tools/Tests/CompositionSmoke.cs) wires both a controller-driven world and a host-driven one into the 3D view.

## What is here

- **World and physics:** vehicle state, slip-angle tyres, powertrain resources, heat and wear, grip, road attitude and banking, collisions, wakes, track limits and walls, deterministic stepping.
- **Domain contracts:** stable identities, driver profiles with 0..100 ratings, car capabilities, typed resource slots, track geometry, immutable frame snapshots, and the controller interface.
- **Presentation:** Godot 3D views, cameras, HUD and CSV recording of physical state. The old 2D `RaceView` / `Levels/root.tscn` is retained for compatibility only.

What is **not** here: rule-based or learned drivers, racing-line solvers, observation vectors, neural networks, training programs and model files. The world does not prefer a line, and it does not hide a driver model in its physics.

## How the pieces meet

- **Controllers.** `IDriverController.GetControl` reads a `DriverContext` (the driver's profile, a read-only track view and a frozen frame) and returns a `DriverInput`: desired curvature, desired longitudinal acceleration and an optional brake-bias offset. A car may also have no driver; the host then sets `RaceCar.ExternalInput`, and a zero command coasts rather than freezing the car.
- **Freeze, collect, advance.** In every substep all controllers read the same frozen frame, every command is collected and validated, and only then does physics advance. No controller sees a world another controller has already changed.
- **Devices on the car.** Everything between a command and the tyres is a published device with its parameters on `CarConfig`. A car without a device has that device's parameters at zero; the interface does not change.
  - The **combined-grip limiter** is the electronic stability device this class of racing permits. It trims each axle's drive and braking so that the tyre's combined use stays inside the share of the friction circle the team's tyre rung authorises, and above 10 m/s it cuts drive on an axle that is past its peak slip angle. It never steers. `CombinedGripLimiterStrength = 0` means the device is not fitted.
  - **Drag reduction** removes part of the aero drag and restores part of the downforce lost in a leading car's wake. An external race host sets its activation; Core does not decide eligibility.
- **Tracks are geometry.** A track provides a centreline, widths, run-off, curvature, elevation and banking, surfaces and a starting grid. It does not provide a racing line.

The contracts are stated in the XML documentation on these types and are held by the tests in `Core/Tests`.

## History

This is a narrower product boundary than the project's earlier prototype, which carried its own analytic and learned drivers and a training stack. That history remains in Git: baseline `d85e164` and the `era/world-v2` line. Current behaviour differs from it on purpose. Driver-efficiency inputs are gone. The old traction control and anti-lock are replaced by the combined-grip limiter. Drag reduction is activated by the host instead of by a one-second gap at the line. Old results are evidence about an old contract; do not read them as claims about this one.

## Verification

```bash
python3 Tools/verify_boundary.py --godot /path/to/godot-dotnet
```

This runs the source-boundary check, the Core tests, both app builds, the headless 3D composition and motion smokes, and a telemetry check. Add `--render` for real rendered captures.

## Licence

- **Code** is licensed under the [GNU Affero General Public License v3.0](LICENSE).
- **Content and assets** follow the separate [content licensing notice](CONTENT_LICENSE.md), which covers project assets, third-party material and official distributions.
- **Third-party track data** under `Core/Track/Data/` keeps its original LGPL-3.0 or MIT terms; see [`Core/Track/Data/README.md`](Core/Track/Data/README.md).
