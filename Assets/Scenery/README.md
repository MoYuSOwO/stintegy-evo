# The prop library

Anything in this directory is a prop a circuit's scenery plan can stand
beside the road. A prop exists because its file does: nothing registers it,
nothing lists it, nothing compiles it in. Drop `oak.glb` in here and any
plan may ask for `"prop": "oak"`.

`catalogue.json` gives the props that ship with the game their numbers:

```json
{ "props": { "1": "pine", "2": "grandstand" } }
```

A plan that asks for `"prop": 1` gets whatever the catalogue says 1 is, so
the default scenery can be re-modelled or renamed without rewriting a
single circuit. Numbering is for the shared props only — a circuit with
scenery of its own names the file instead, from anywhere in the project,
and never appears here.

Accepted: `.tscn`, `.scn`, `.glb`, `.gltf`, looked up in that order by name.

Two kinds, and the difference is only about how they are drawn:

- **A single mesh** — a scene whose only node is a `MeshInstance3D`, like
  `pine.tscn`. Eight or more of these in one plan are drawn as one
  instanced batch, so an avenue of forty trees costs the renderer one
  draw.
- **A scene** — anything with parts, like `grandstand.tscn`. Instanced as
  a scene, which is what a scene is for.

Model them with their feet at the origin and facing +Z. A plan places a
prop by its station along the circuit and its offset to the side; the
ground under it comes from the terrain, never from the file.

**Props are scenery.** Nothing here is known to the physics: the car
cannot hit a tree, and the barrier remains the only boundary in the world.
Keep them low-poly — this circuit is drawn in flat-shaded facets, and a
photogrammetry oak would look stranger than the cone does.
