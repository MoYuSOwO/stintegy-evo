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
  "version": 1,
  "assembly": "someone.my_car.dll",
  "chassis": "official:single_track",
  "model": "car.glb",
  "tyres": ["official:formula_a"]
}
```

`id` must be the folder's name. `version` is yours to raise whenever you
change how the car drives: what a driver learned on the car was learned on a
version, and is out of date on another (building the same pack again changes
nothing). `definition` names the class to build when the assembly holds more
than one. `chassis` is the chassis pack the car is built on (see
[The chassis](#the-chassis)), loaded before the car; a car with physics of
its own names none. `tyres` is what the car runs (see
[Tyres](#tyres)); a car that names none does not load.

## The code

Reference `StintegyEVO.Core` and the chassis you build on, and nothing
else — not Godot: a car is physics, and the game draws it. Implement
`ICarDefinition`:

```csharp
public sealed class MyCar : ICarDefinition
{
    // The compounds car.json declares, loaded from their tyre packs.
    public ICar Build(IReadOnlyList<TireCompound> compounds) => new SingleTrackCar(chassis, compounds);
}
```

What you get to work with is the game's own:

- `IAero` (`StandardAero`), `IPowertrain` (`RangeExtenderPowertrain`,
  `ElectricPowertrain`), from Core.
- A chassis pack: the game's own, `official:single_track` (below), or
  anyone's.
- Or implement `ICar` yourself, from nothing: a car with physics nobody
  else uses keeps them in its own pack and names no chassis. Its estimates —
  `EstimatePerformanceLimits`, `EstimateCorneringLimit` — are what the
  drivers plan every corner and every braking point with: if they are not
  what the car really does, the car still runs, and is driven badly.

The tyres are not yours to write: every car runs the game's one tyre
(`Tire`), a chassis of your own included, which tells it how hard each
tyre is pressed and how it is used and is told the grip it has.

What the strategist can set is yours to declare: `ICar.Ladders`, up to
four — an output setting, an engine map, a hybrid's deployment — each with
its own rungs and names. The tyre ladder is the game's and every car has
it. A ladder is set anywhere from zero to one along its travel, and its
rungs are evenly spaced marks on it (`ModeLadder.Mark`): your powertrain is
handed that setting, should give exactly the named setting on a mark, and
decides for itself what lies between two. What the car's dashboard shows and what its telemetry records are
yours too (`Describe`, `TelemetryChannels`).

The driver is not part of a car and cannot be changed by one: every car is
driven by the same drivers, through `ICar`. Drivers are packs of their own
(`data/<namespace>/drivers/<name>/`, a `driver.json` and an assembly with one
`IDriverDefinition`). What a driver learns about your car at a circuit is
about your car, so it is kept in your pack, under the driver and the circuit
by their ids: `drivers/<driver namespace>/<driver name>/<circuit
namespace>/<circuit name>/`, with a `notes.json` saying which version of the
car it was learned on. Ship those folders and the drivers know your car.

Copy `Cars/official/formula_a_mixed/formula_a_mixed.csproj` for the build
settings: Core is referenced but not copied, because the game supplies it.

## The chassis

A chassis is physics a car is built on, shared by every car that names
it: a folder with a `chassis.json` and the assembly cars build against.
It is code and no car; what it offers is what its assembly makes public.

```
data/official/chassis/single_track/
  chassis.json
  official.single_track.dll
```

```json
{
  "format": 1,
  "id": "single_track",
  "name": "Single-track chassis",
  "version": 1,
  "assembly": "official.single_track.dll"
}
```

`version` is the chassis's to raise whenever a change to it changes how a
car on it drives, which marks what was learned on every car on it as
stale. The game's own is `SingleTrackCar` (namespace
`Official.SingleTrack`, source in `Chassis/official/single_track/`): a
single-track model with load transfer, slip-angle tyres, a combined-grip
limiter and whatever powertrain is fitted. Give it a `CarConfig` and its
compounds, or derive from it and override what your car does
differently; every member is virtual. Reference its project the way
`formula_a_mixed.csproj` does, not copied: its pack ships it.

## Tyres

The tyre model is the game's and the same on every car, because the
drivers read it: how hot is too hot, how much grip is left, what pushing
costs must mean the same whatever they drive. What differs is the
compound, and compounds come in tyre packs: a folder with a `tyres.json`
and nothing else.

```
data/someone/tyres/their_tyres/
  tyres.json
```

```json
{
  "format": 1,
  "id": "their_tyres",
  "name": "Their tyres",
  "compounds": [
    {
      "id": "soft",
      "name": "Soft",
      "tyre": {
        "baseMu": 2.5,
        "loadSensitivity": 0.3,
        "idealSurfaceTempLowC": 80,
        "idealSurfaceTempHighC": 110,
        "coldGripLossPerCSquared": 0.00008,
        "hotGripLossPerCSquared": 0.00012,
        "coreOverheatTempC": 108,
        "heatScale": 1.2,
        "wearScale": 1.6,
        "wearCliffGripLoss": 0.15
      }
    }
  ]
}
```

Ten numbers a compound, every one written out:

| number | what it is |
|---|---|
| `baseMu` | friction at the load the tyre was sized for, warm and new |
| `loadSensitivity` | how fast friction falls as the load rises (0 to under 1; a slick is about 0.3) |
| `idealSurfaceTempLowC`, `idealSurfaceTempHighC` | the tread's working window |
| `coldGripLossPerCSquared`, `hotGripLossPerCSquared` | how fast grip falls below and above it |
| `coreOverheatTempC` | where the carcass starts to cost grip |
| `heatScale` | how hot it runs for the same work: every heat source at once |
| `wearScale` | how fast it wears for the same work, which brings the cliff forward too |
| `wearCliffGripLoss` | how much grip the cliff takes |

Everything else — how heat and wear split between their causes, the shape
of the cliff, the cooling — is the model's own and not set by any pack.
Give `"front"` and `"rear"` instead of `"tyre"` for a compound with its
own numbers on each axle. How warm a fresh set starts is the series' (the
blankets), not the compound's. The tyre is sized for each car by the car:
its wheels' share of the car's weight.

A car runs what its `car.json` declares, a whole pack
(`"official:formula_a"`) or single compounds (`"official:formula_a/medium"`),
and nothing else. A compound's full id is the pack's and its own:
`someone:their_tyres/soft`.

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

Anything learned on a car — a driver's lines — is learned on that code and
those tyres. A car's fingerprint is a hash of its version, its chassis's
version, what it says of itself and the tyre packs it runs, so raising
either version, or changing its tyres, marks what was learned on the old
one as stale.
