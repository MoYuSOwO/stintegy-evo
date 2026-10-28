# Scenery plans

One file per circuit, in its folder: `silverstone/scenery.json` is Silverstone's.
It says which props stand where, and it is meant to be read and edited by
a person.

```json
{
  "track": "silverstone",
  "props": [
    { "prop": 2, "s": 120.0, "d": -46.0, "yaw": 3.142, "scale": 1.0 },
    { "prop": 1, "s": 300.0, "d": 68.0, "scale": 1.15,
      "repeat": { "count": 14, "step_s": 26.0, "step_d": 2.5 } },
    { "prop": "res://mods/official/data/circuits/silverstone/props/marquee.tscn",
      "s": 210.0, "d": -38.0 }
  ]
}
```

## Naming a prop

Three ways, and a plan may use all three at once.

| written | means |
|---|---|
| `"prop": 2` | the game's own prop number two, through `Assets/Scenery/catalogue.json` |
| `"prop": "pine"` | `pine.tscn`, `pine.glb`, `pine.obj` … in `Assets/Scenery`, by name |
| `"prop": "res://mods/official/data/circuits/silverstone/props/marquee.tscn"` | that file, wherever it is — a circuit bringing scenery of its own |

Models in a format Godot has to import — glTF, Wavefront, COLLADA, FBX —
work anywhere a name or a path does, but must be imported once first (see
`Assets/Scenery/README.md`). Godot's own `.tscn` needs nothing.

The numbers are the stable handle on the props that ship with the game:
re-model the tree or rename its file, and every circuit that asked for a 1
still gets a tree. A circuit that brings its own asset names the file and
needs no number, no registration and no change to the shared library.

## The other keys

| key | meaning |
|---|---|
| `s` | metres along the centreline from the start line |
| `d` | metres to the side: **positive is left** of the direction of travel |
| `yaw` | radians, turned from the road's direction (or from the world, with `"align": "world"`) |
| `scale` | multiplier on the prop's own size |
| `height` | metres above the ground, when a prop should not sit on it |
| `repeat` | a row of the same prop: `count`, and `step_s` / `step_d` / `step_yaw` added each time |

Positions are in the circuit's own frame rather than the world's, so a
circuit whose geometry is re-cut keeps its scenery beside the same corners
instead of scattering it across the county. Height is not authored: the
ground is where the terrain says it is.

## Many props at once

A circuit needs thousands of things standing round it, and nobody places a
wood one tree at a time. Three ways to place many:

**Along the circuit** — a prop every so many metres between two stations,
on one side, the other, or both, each nudged a little so a row looks
planted rather than ruled:

```json
{ "prop": "hedge", "d": 40, "along": { "from": 200, "to": 900, "every": 10,
  "side": "both", "d_jitter": 0.6, "s_jitter": 2, "scale_jitter": 0.1, "yaw_jitter": 0.2 } }
```

**An area** — so many per hundred square metres anywhere in a band beside
the circuit, between two stations and two offsets. A wood is an area of
trees; a car park is an area of cars (`"yaw": "track"` lines them up with
the road instead of turning each at random):

```json
{ "prop": 1, "area": { "s": [1200, 1600], "d": [75, 175], "per_100m2": 0.55, "scale": [0.8, 1.5] } }
```

**Farmland** — one block per circuit: a patchwork of fields from `from` to
`to` metres off the circuit, in parcels of about `field` metres on a grid
turned by `angle`, each a colour from the palette or, a `meadow` share of
the time, left as grass; with `hedge` along the parcel edges and now and
then a `tree`. Fields are laid in the world, not along the road, because
they were there first:

```json
"farmland": { "from": 190, "to": 1100, "field": 150, "angle": 0.42, "meadow": 0.22,
  "palette": ["#cdb866", "#a9bd70", "#9c8461"], "hedge": 4, "tree": 3 }
```

Plain ground colour — a car park's tarmac, a campsite's mown grass — is a
patch, between two stations and two offsets:

```json
"patches": [ { "color": "#6f7577", "s": [60, 420], "d": [72, 150] } ]
```

All of these are seeded by the plan itself, so the same file grows the
same wood every time, and none of them can put anything on the circuit:
whatever would stand inside the barrier of *any* part of the lap — not only
the part it was placed beside — is left out and counted in the log
(`… kept off the circuit`).

## How the circuit looks

`name` is the circuit as the screen shows it. `environment` is its sky,
sun, haze and ground; every key is optional and one left out keeps the
default, so a plan names only what makes its circuit different:

```json
"name": "Monaco",
"environment": {
  "sky_top": "#5d93cf", "sky_horizon": "#e3e6dd", "sun_pitch": 55, "sun_yaw": 20,
  "sun_color": "#fff2d6", "sun_energy": 1.3, "fog_density": 0.0002,
  "grass": "#7f9a5e", "mown": ["#93ad68", "#86a05f"],
  "barrier": ["#e8e4d6", "#dcd7c8"], "tyre_wall": ["#4a5254", "#565e60"],
  "hills_near": "#7f9566", "hills_far": "#8ea2a8", "clouds": 12
}
```

| key | meaning |
|---|---|
| `sky_top`, `sky_horizon`, `sky_ground` | sky colours overhead, at the horizon, and below it |
| `sun_pitch`, `sun_yaw` | degrees the sun shines down from, and the direction it comes from |
| `sun_color`, `sun_energy`, `ambient_energy` | the sun's colour and strength, and the sky's fill light |
| `fog_color`, `fog_density` | the haze the distance fades into (the overview thins it) |
| `road`, `grass`, `plain` | asphalt, the meadow, and the plain beyond it |
| `mown` | run-off bands, alternating |
| `barrier`, `tyre_wall` | barrier faces on straights, and round the outside of tight corners |
| `hills_near`, `hills_far`, `clouds` | horizon colours and how many clouds |

## Circuit furniture

The fence, marshal posts, distance boards, start gantry and start-straight
flags are placed from the circuit's own shape, so a plan need say nothing
about them. `furniture` switches any of them off (`false`), changes how
often they come, or names where they stand; sides are `"left"` / `"right"`
of the direction of travel:

```json
"furniture": {
  "fence": { "on": true, "spacing": 6 },
  "marshals": { "spacing": 250, "at": [ { "s": 1200, "side": "left" } ] },
  "boards": { "auto": true, "at": [ { "s": 3150, "side": "right" } ] },
  "gantry": { "s": 5884 },
  "flags": false
}
```

| key | meaning |
|---|---|
| `fence` | catch fencing behind the barrier; `spacing` between posts |
| `marshals` | a post every `spacing` metres, sides alternating; `at` places them instead |
| `boards` | 150/100/50 boards before braking zones found from the geometry (`auto`), plus any corner in `at` (the station the braking ends) |
| `gantry` | the start gantry; `s` moves it (default: a few metres before the line) |
| `flags` | flags either side of the start line, `range` metres each way, every `spacing` |

## Mods

A player adding scenery does not edit these files: they drop a folder into
the game's `mods` directory, and it is read at startup and drawn after the
circuit's own. See `MODDING.md`. The one difference that matters is the
model format — a mod's models are read at runtime and must be glTF,
because the importer that handles everything else is part of the editor
and is not in a shipped game.

## Writing one

By hand: the format is seven keys, and F5 shows the result. A prop that is
in the wrong place is one number away from the right one.

**None of this is physical.** A plan cannot put anything in the car's way:
the barrier is the only boundary the simulation knows, and scenery lives
entirely in the renderer.
