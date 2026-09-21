# Scenery plans

One file per circuit, named for it: `silverstone.json` is Silverstone's.
It says which props stand where, and it is meant to be read and edited by
a person.

```json
{
  "track": "silverstone",
  "props": [
    { "prop": 2, "s": 120.0, "d": -46.0, "yaw": 3.142, "scale": 1.0 },
    { "prop": 1, "s": 300.0, "d": 68.0, "scale": 1.15,
      "repeat": { "count": 14, "step_s": 26.0, "step_d": 2.5 } },
    { "prop": "res://Assets/TrackScenery/silverstone/marquee.tscn",
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
| `"prop": "res://Assets/TrackScenery/silverstone/marquee.tscn"` | that file, wherever it is — a circuit bringing scenery of its own |

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
