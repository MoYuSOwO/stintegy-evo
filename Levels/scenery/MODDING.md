# Adding scenery to a circuit

You do not need the game's source, its editor, or a build. A mod is a
folder with models and a text file.

```
mods/greener-silverstone/
  props/oak.glb
  scenery/silverstone.json
```

`scenery/silverstone.json` is the same format the game's own circuits use
(see `README.md` beside this file):

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

- Paths are relative to **your own folder**, so the mod can be renamed,
  moved or zipped without editing anything.
- `"prop": 1` is the game's own prop number one. A mod may use the built-in
  props freely and only ship the models it actually adds.
- Your file adds to the circuit's scenery; it does not replace it.

## Where the folder goes

`mods/` inside the game's user directory — the writable one the game makes
on your platform. While developing, point the game at any folder instead:

```sh
STINTEGY_MODS=/path/to/mods <game>
```

## Models: use glTF

`.glb` and `.gltf` — every tool exports them, they carry their materials,
and the game reads them itself when it starts.

**Other formats do not work from a mod folder, and this is not a
preference.** `.obj`, `.fbx`, `.dae` and Blender files are converted by
Godot's *editor* before a build; a shipped game has no editor in it and
therefore no converter. Export to glTF from whatever you model in — it is
one menu item in Blender — and it will load.

`.tscn` and `.scn` also load, if you happen to be building your mod
against the project.

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
