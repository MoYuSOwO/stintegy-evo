# Making a mod

Everything the game races with comes from mods: circuits, tyres, chassis,
cars and drivers. The game's own content is a mod like anyone's — it is in
`mods/official/` — and uses nothing a mod of yours cannot.

A mod is a folder, or the same folder zipped into a `.stmod` file; the game
reads both the same way and never unpacks anything.

```
someone/                     (or someone.stmod)
  mod.json
  someone.dll                (code, if the mod has any)
  someone.pdb                (optional)
  data/
    circuits/their_ring/     circuit.json, track.csv, scenery.json
    tyres/their_tyres.json
    cars/my_car/             car.json, car.glb
```

## mod.json

```json
{
  "format": 1,
  "id": "someone",
  "name": "Someone's cars",
  "version": "1.2.0",
  "depends": { "official": ">=1.0.0" },
  "assembly": "someone.dll",
  "entrypoint": "Someone.SomeoneMod",
  "circuits": ["data/circuits/their_ring"],
  "tyres": ["data/tyres/their_tyres.json"],
  "cars": ["data/cars/my_car"],
  "props": [],
  "scenery": [{ "circuit": "official:silverstone", "file": "scenery/greener_silverstone.json" }]
}
```

- `id` is the mod's namespace, and the name of its folder or file. Lower
  case letters, digits and underscores.
- `version` is `major.minor.patch`. A dependency names the least version it
  takes, `">=1.2.0"`, and the major version must match: 1.5 meets it, 2.0
  does not.
- Only what `mod.json` lists is there. The game does not look through a
  mod's folders, so source files, notes and work files beside the listed
  ones are ignored.
- A mod of data alone has no `assembly` or `entrypoint`.

## Names, and what is yours

Everything registered has an id, `namespace:name`: `official:silverstone`,
`someone:my_car`. A mod registers only in its own namespace, and an id is
registered once: no mod can replace, change or remove what another
registered. To change the game's chassis, derive from it in your own code
and register the result as your own; to change a circuit, copy it and
publish it under your id. A bare name means the game's own
(`formula_a_mixed` is `official:formula_a_mixed`).

The one exception is scenery: any mod may add scenery to any circuit, its
own or another's (`"scenery"` above). It only adds; it never takes away. If
the circuit is not installed, the scenery is left out and the log says so.

## How the game loads mods

1. **Found** in the game's `mods/` folder and the player's `user://mods/`.
   The game's own come first: a player's mod with the same id is not loaded.
   While developing, add folders with `STINTEGY_MODS=/path/a:/path/b`.
2. **Ordered** so that each mod loads after what it depends on. A missing
   dependency, the wrong version or a circle stops the game here, naming
   the mod.
3. **Code loaded**, once, from its bytes. A mod's code may use another mod's
   only if it depends on that mod.
4. **Data read** by the game's own readers: every circuit, tyre pack, car
   declaration, prop and scenery file listed.
5. **Registered**: the entrypoint (`IMod.Register`) adds the chassis, cars'
   code and drivers.
6. **Checked**: every car is built and checked (below). Anything wrong
   stops the game at the start, not in a race.
7. **Frozen**: the registry is read only from here on.

A mod is code, and loading it runs it, with everything the game can do.
That is true of every game's code mods. Install mods from people you trust.

## Data

**Circuits.** A folder with a `circuit.json` and its `track.csv`, one row a
metre, written by `Tools/Circuits` for the game's own (see
`Circuits/README.md`). The road is read the first time a circuit is used.
Its scenery is `scenery.json` beside it (see `Circuits/SCENERY.md`).

**Tyres.** The tyre model is the game's and the same on every car, because
the drivers read it: how hot is too hot, how much grip is left, what pushing
costs must mean the same whatever they drive. What differs is the compound,
and compounds come in tyre packs:

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

Everything else is the model's own. Give `"front"` and `"rear"` instead of
`"tyre"` for a compound with its own numbers on each axle. A compound's full
id is its pack's and its own: `someone:their_tyres/soft`.

## Code

Reference `StintegyEVO.Core`, and the mods your code builds on, and nothing
else — not Godot: a mod offers parts, and the game draws them. Copy
`mods/official/official.csproj` for the build settings: Core is referenced
but not copied, because the game supplies it. The entrypoint is a class with
a constructor that takes nothing:

```csharp
public sealed class SomeoneMod : IMod
{
    public void Register(ModContext context)
    {
        context.Registry.Cars.Register("my_car", new MyCar());
        context.Registry.Drivers.Register("steady", new SteadyDriver());
    }
}
```

The game makes it once, calls `Register` once and keeps it. A mod listens
to nothing and changes nothing in the game: it offers standard parts, and
the game asks for them by id when it needs them. `context.Resources` reads
the mod's own files, packed or not; `context.Log` tells the player
something while the mods load.

### A chassis

Physics cars are built on, shared by every car that names it:

```csharp
context.Registry.Chassis.Register("my_chassis", new ChassisDeclaration("My chassis", Version: 1));
```

Raise `Version` whenever a change to it changes how a car on it drives:
that marks what was learned on every car on it as stale. The game's own is
`official:single_track` (`SingleTrackCar`, namespace `Official.SingleTrack`):
a single-track model with load transfer, slip-angle tyres, a combined-grip
limiter and whatever powertrain is fitted. Give it a `CarConfig` and its
compounds, or derive from it and override what your car does differently;
every member is virtual. Depend on `official` to build on it.

### A car

A car is two halves with one name: its `car.json`, listed in `mod.json`,
and the code that builds it, registered under the same name.

```json
{
  "format": 1,
  "id": "my_car",
  "name": "My Car",
  "version": 1,
  "chassis": "official:single_track",
  "model": "car.glb",
  "tyres": ["official:formula_a"]
}
```

```csharp
public sealed class MyCar : ICarDefinition
{
    // The compounds car.json declares, from their tyre packs.
    public ICar Build(IReadOnlyList<TireCompound> compounds) => new SingleTrackCar(Config(), compounds);
}
```

`version` is yours to raise whenever you change how the car drives.
`chassis` is the chassis it is built on; a car with physics of its own
names none. `tyres` is what it runs: whole packs (`"official:formula_a"`)
or single compounds (`"official:formula_a/medium"`), and nothing else. A
car may use another mod's chassis or tyres only if its mod depends on that
mod.

Every car is built once, when the mods load, and refused there, naming
what is wrong, if: it has no code or code with no `car.json`; its chassis or
tyres are not registered; it runs other compounds than it declares; it
declares itself wrongly (below); or it has more than four ladders of its
own.

A car's fingerprint is a hash of its id and version, its chassis's version,
everything it declares of itself and the tyre packs it runs. What a driver
learns on the car is learned on that fingerprint, and is stale on another.

## What the world reads from a car

The world reads only what the car declares, and every number is the car's
to write: none is filled in for it. All of it is in `ICar.Spec`
(`CarSpec`), except the car's mass:

| declared | the world uses it for |
|---|---|
| `WheelBaseMeters` | the steering estimate drivers plan with |
| `FrontAxleOffsetMeters`, `RearAxleOffsetMeters`, `TrackWidthMeters` | where each wheel is: which surface each one is on |
| `YawInertiaKgM2` | how a blow off-centre turns the car |
| `MaximumBrakeDeceleration` | the hardest stop drivers plan around |
| `KinematicBelowSpeed` | below this speed a blow moves the car but cannot spin it |
| `Body.LengthMeters`, `Body.WidthMeters` | the box other cars and the walls meet |
| `Body.Restitution`, `Body.Friction` | how it gives and slides against another car (the pair takes the smaller) |
| `Body.WallRestitution`, `Body.WallFriction` | how it gives and slides against a barrier |
| `Wake.VelocityDeficit`, `Wake.TowHalfDistanceMeters` | the tow it gives the car behind, and how far it reaches |
| `Wake.DownforceDisruption`, `Wake.DownforceDecayLengthMeters`, `Wake.DirtyAirDecayLengthMeters` | the disturbed air it leaves for the car behind |
| `DirtyAirSensitivity` | how much of the air in front reaches its own working surfaces |
| `TotalMassKg(state)` | how a blow changes its speed; asked for at every contact |

Lengths, the inertia and the brake must be above zero; the speed, the
sensitivity, the frictions and the wake's deficits must not be below it;
the restitutions are from zero to one. A car declaring one wrongly is
refused, by the field, when the mods load and when it is put in a race.

The world turns a car about its centre of mass with the inertia declared,
which does not change as the car burns fuel or drains a battery, while
`TotalMassKg` does. A car whose mass changes a great deal through a race
declares the inertia it should have at the weight it is mostly raced at.

How many passes the contact solver makes is the world's, not the car's.

## What the world gives a car, and what it does to it

**Each step** the world calls `Step(state, input, dt)`, with every car's
input settled before any car moves. The input is all the car is told, and a
car depends on nothing it is not handed:

| `CarStepInput` | what it is |
|---|---|
| `DriverInput` | where the driver turns the road wheels and what it asks of the pedals; for a car with axles, how it moves brakes and drive between them (`Axles`) and how hard its hands are on the tyres (`TyreWearExcess`) |
| `Strategy` | where the strategist has set the tyre ladder, each of the car's ladders, and which buttons are held |
| `AirTempC`, `TrackTempC` | the day |
| `RoadAttitude` | the road's grade and bank under the car |
| `SurfaceGrip` | what the road is worth under each wheel: clean surface, kerb, grass |
| `SurfaceAbrasion` | how hard the asphalt is on rubber |
| `Air` | the tow and the disturbed air the cars ahead leave it in |
| `Random` | its randomness for this step (below) |
| `Impacts` | every blow since its last step (below) |

`dt` is the step's length; the world steps cars in substeps of at most
1/120 s.

**What the car writes** is its own state. `CarState` holds the body the
world sees — position, heading, speed, sideslip, yaw rate — and the car
keeps whatever else it needs in its own part of the state (`state.Own`, an
`ICarStateExtension` it copies and clones itself). One `ICar` serves every
car of its kind on the grid, so it keeps nothing of a race in itself.

**What the world writes.** At a contact with a wall or another car, the
world moves the car out and changes its body's motion: position, heading
(only below `KinematicBelowSpeed`, where the car follows its path), speed,
sideslip and yaw rate. Nothing else of the car's is touched. It then tells
the car what hit it, at its next step, in `Impacts`: for each blow, a car or
a wall, how hard (the impulse in N·s; divided by the mass, the speed it
changed), where on the body (metres from the centre of mass, forward and to
the left) and which way it pushed. That is a channel, not a damage model:
what a blow breaks is the car's to decide.

**Randomness** is the game's. A race is seeded once and splits the seed for
each car by its id, for the car and its driver apart, and for each step: the
car takes its own from `input.Random`, the driver from `context.Random`.
Split it further for the parts that want one (`input.Random.For("wing")`)
and draw from it (`.Draw().NextFloat()`). Nothing is used up, so a driver
that looks ahead by stepping a copy of its car sees the chances the car
will have, and the same seed gives the same race. Never use `System.Random`
or the clock.

**Other calls** the race makes into a car, by the features it has:

| call | when |
|---|---|
| `ICarTires.InstallFreshTires` | when it is put in a race, unless the host says not |
| `ICarTires.FitTires`, `WarmTires` | when the host changes or warms its tyres (a pit stop, a grid on blankets) |
| `ICarStores.Fill` | when the host fills its stores |
| `ICarButtons.HoldUntilLine`, `CrossedLine` | when race control holds its buttons, and when it crosses the line |
| `ICarTires.Assume`, `ICarButtons.Assume` | on a copy, when a driver plans ahead |

Everything else is read: what each feature says of the car
(`ICarTires.Tire`, `ICarStores.Stores`, `ICarEnvelope`, `ICarMotion`,
`ICarAxles.GripUse`, `ICarGripFeel`), the dashboard (`Describe`) and the
recorder's channels (`TelemetryChannels`, `ReadTelemetry`).

## What a car has

Every car is a body on the road (`ICar`). Anything more it declares by
implementing a feature: changeable tyres (`ICarTires`), stores it spends
(`ICarStores`), buttons a driver holds (`ICarButtons`), an envelope to plan
against (`ICarEnvelope`), its motion for the screen and the feel of the
car (`ICarMotion`), two axles to move brakes and drive between
(`ICarAxles`), and the feel of its grip (`ICarGripFeel`). The frame carries
a feature's readings only for a car that has it.

What the strategist can set is yours to declare: `ICar.Ladders`, up to four
— an output setting, an engine map, a hybrid's deployment. The tyre ladder
is the game's and every car has it. A ladder is set anywhere from zero to
one along its travel, its rungs are evenly spaced marks on it
(`ModeLadder.Mark`), and a car should give exactly the named setting on a
mark and decides for itself what lies between two.

The estimates in `ICarEnvelope` are what the drivers plan every corner and
every braking point with: if they are not what the car really does, the car
still runs, and is driven badly.

## A driver

A driver is registered code: an `IDriverDefinition` that says what it
needs of a car and seats itself in one.

```csharp
public sealed class SteadyDriver : IDriverDefinition
{
    // Required: refused without it. Optional: used where the car has it.
    public CarNeeds Needs => new([typeof(ICarEnvelope)], [typeof(ICarAxles)]);

    public IDriverController Seat(DriverSeat seat) => new Steady(seat);
}
```

Every feature a driver reads goes in `Needs`, required or optional. The
game puts a driver only in a car that has every required one, and refuses
the seat, before the race, naming what is missing (`ModDrivers.Check`,
`ModDrivers.Seat`). An optional one the driver must do without where the
car has none.

Seated, the driver is given the person driving, the car, the circuit, what
it has learned about that car there (`DriverSeat.Notes`) and, where the
game keeps it, this person's own experience of it (`DriverSeat.Progress`).
Each decision it reads one `DriverContext` — its profile, a read-only track
view and a frozen frame of every car — and returns a `DriverInput`. Every
driver reads the same frozen frame; no driver sees a world another has
already changed.

## The model

glTF (`.glb`/`.gltf`), in metres, Y up, the car facing glTF's forward
(+Z), its origin on the ground under the centre of mass. Nodes called
`wheel_fl`, `wheel_fr`, `wheel_rl` and `wheel_rr` are the wheels, and the
front two steer. No model, or one that will not load, and the car is drawn
the game's way with its wheels where your `CarSpec` puts them. For why only
glTF, see `Circuits/MODDING.md`.

## Building

Build your project into the mod's folder, beside its `mod.json`, as
`official.csproj` does. The folder is the mod; zip it into
`<id>.stmod` to hand it to someone.
