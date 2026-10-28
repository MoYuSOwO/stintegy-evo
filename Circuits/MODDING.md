# Adding scenery to a circuit

You do not need the game's source, its editor, or a build. A mod of scenery
is a folder (or the same folder zipped into a `.stmod`) with a `mod.json`,
its models and a plan.

```
greener_silverstone/
  mod.json
  props/oak.glb
  scenery/silverstone.json
```

```json
{
  "format": 1,
  "id": "greener_silverstone",
  "name": "Greener Silverstone",
  "version": "1.0.0",
  "scenery": [{ "circuit": "official:silverstone", "file": "scenery/silverstone.json" }]
}
```

`"circuit"` is the full id of the circuit the plan is for, the game's own or
anyone's. The plan is the format every circuit's own scenery uses (see
`SCENERY.md` beside this file):

```json
{
  "track": "silverstone",
  "props": [
    { "prop": "props/oak.glb", "s": 240.0, "d": -30.0, "scale": 1.4 },
    { "prop": "props/oak.glb", "s": 400.0, "d": 36.0,
      "repeat": { "count": 12, "step_s": 22.0, "step_d": 1.5 } },
    { "prop": 1, "s": 900.0, "d": -44.0 }
  ]
}
```

- A path is a model **in your own mod**, beside the plan, so the mod can be
  renamed, moved or zipped without editing anything.
- `"prop": 1` is the game's own prop number one, and `"official:grandstand"`
  is a prop some mod registered: a mod may use anyone's props and ship only
  the models it adds. To offer props to others, list a file in `"props"`
  that names each by an id and its model:
  `{ "props": { "oak": { "model": "props/oak.glb", "scale": 1.0 } } }`.
- Your plan adds to the circuit's scenery; it never replaces or removes
  any of it. This is the one thing a mod may add to what is not its own.
- If the circuit is not installed, your plan is left out and the log says
  so; the game starts as usual.

## Where the folder goes

`mods/` inside the game's user directory — the writable one the game makes
on your platform. While developing, add any folder of mods:

```sh
STINTEGY_MODS=/path/to/mods <game>
```

## Models: use glTF

`.glb` (or a `.gltf` with everything inside it) — every tool exports them,
they carry their materials, and the game reads them from your mod's files
itself.

**Other formats do not work from a mod, and this is not a preference.**
`.obj`, `.fbx`, `.dae`, Blender files and Godot's own scenes are turned
into engine resources by Godot's *editor* before a build; a shipped game
has no editor in it, and a mod is never imported. Export to glTF from
whatever you model in — it is one menu item in Blender — and it will load.

A model that will not parse costs your mod that prop and nothing else: the
game says so in its log and carries on.

## What scenery is, and is not

Scenery is drawn. That is the whole of it.

Nothing in a mod can be hit, driven into, or noticed by the simulation in
any way. The barrier is the only boundary the car knows, and it comes from
the circuit itself. Put a grandstand in the middle of the track and the
cars will drive straight through it — you will have decorated the racing
line, not blocked it.

This is deliberate. It means a mod cannot change a race, cannot break a
replay, and cannot make one player's result differ from another's.

## Sizes and orientation

Model with the feet at the origin, facing +Z, in metres. The plan places a
prop by its station along the circuit and its distance to the side;
whatever the ground is doing there, the prop stands on it.
