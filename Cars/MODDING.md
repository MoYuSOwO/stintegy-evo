# Making a car

A car is a pack: a folder with a `car.json`, a .NET assembly and, if you
like, a model. The game's own car is made exactly this way — it is in
`Cars/official/formula_a_mixed/` — and uses nothing a pack of yours cannot.

```
data/someone/cars/my_car/
  car.json
  someone.my_car.dll
  car.glb            (optional)
```

## Names

Every car has an id, `namespace:name`: `official:formula_a_mixed`,
`someone:my_car`. The namespace is yours — your name or your team's — and
keeps your cars apart from everyone else's. Both halves are lower case
letters, digits and underscores, and they are the folder names:
`data/<namespace>/cars/<name>/`. A bare name means the game's own
(`formula_a_mixed` is `official:formula_a_mixed`).

## car.json

```json
{
  "format": 1,
  "id": "my_car",
  "name": "My Car",
  "assembly": "someone.my_car.dll",
  "model": "car.glb"
}
```

`id` must be the folder's name. `definition` names the class to build when
the assembly holds more than one.

## The code

Reference `StintegyEVO.Core` and nothing else — not Godot: a car is
physics, and the game draws it. Implement `ICarDefinition`:

```csharp
public sealed class MyCar : ICarDefinition
{
    public ICar Build() => new SingleTrackCar(chassis, new TireSet(frontTyre, rearTyre));
}
```

What you get to work with is the game's own:

- `SingleTrackCar` — the chassis every car in the game runs on. Give it a
  `CarConfig` and tyres, or derive from it and override what your car does
  differently; every member is virtual.
- `ITireModel` (`StandardTire` is the game's), one per axle if you want.
- `IAero` (`StandardAero`), `IPowertrain` (`RangeExtenderPowertrain`,
  `ElectricPowertrain`).
- Or implement `ICar` yourself, from nothing.

What the strategist can set is yours to declare: `ICar.Ladders`, up to
four — an output setting, an engine map, a hybrid's deployment — each with
its own rungs and names. The tyre ladder is the game's and every car has
it. What the car's dashboard shows and what its telemetry records are
yours too (`Describe`, `TelemetryChannels`).

The driver is not part of a car and cannot be changed by one: every car is
driven by the same drivers, through `ICar`.

Copy `Cars/official/formula_a_mixed/formula_a_mixed.csproj` for the build
settings: Core is referenced but not copied, because the game supplies it.

## The model

glTF (`.glb`/`.gltf`), in metres, Y up, the car facing glTF's forward
(+Z), its origin on the ground under the centre of mass. Nodes called
`wheel_fl`, `wheel_fr`, `wheel_rl` and `wheel_rr` are the wheels, and the
front two steer. No model, or one that will not load, and the car is drawn
the game's way with its wheels where your `CarSpec` puts them. For why only
glTF, see `Circuits/MODDING.md`.

## Where it goes

`data/` inside the game's user directory, which is where players put
packs. Cars beside the game's executable are found first, so a pack cannot
replace the game's own. While developing, point the game at any folder:

```sh
STINTEGY_DATA=/path/to/data <game>
```

## Trust

A pack is code, and loading it runs it, with everything the game can do.
That is true of every game's code mods. Install packs from people you
trust.

Anything learned on a car — a driver's lines — is learned on that code. A
pack's fingerprint is a hash of its assembly, so rebuilding it marks what
was learned on the old one as stale.
