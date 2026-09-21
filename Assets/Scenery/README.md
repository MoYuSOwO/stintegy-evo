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

## Formats

Accepted, tried in this order by name: `.tscn`, `.scn`, `.glb`, `.gltf`,
`.obj`, `.dae`, `.fbx`, `.blend`, `.res`. Two of them are here as working
examples of the two kinds: `crate.glb` imports as a scene, `barrel.obj` as
a bare mesh, and the loader takes either.

**Anything that is not Godot's own format has to be imported once.** A
`.glb` dropped in here and run straight away is invisible — the game only
sees resources the editor has imported, and importing is what writes the
`.import` file beside the model. Opening the project in Godot does it, or,
without opening anything:

```sh
/Applications/Godot_mono.app/Contents/MacOS/Godot --headless --path . --import
```

After that it loads like anything else. `.tscn` and `.scn` need none of
this, which is why the props that ship here are written in them.

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
