# Low-poly 3D prototype

This presentation worktree starts at master `1c97008`. It uses the existing Core simulation and 20 rule drivers. It does not include the uncommitted learned-driver work in the other checkout.

## Run

From this worktree:

```sh
dotnet build StintegyEVO.csproj -p:Optimize=true
/Applications/Godot_mono.app/Contents/MacOS/Godot --path .
```

Import `project.godot` with Godot **4.6.3 .NET** for normal editor use. The default scene is `Levels/lowpoly.tscn`. It starts paused on the grid: use RUN or Space to start. A loading label remains responsive while the existing reference-line solver prepares Silverstone. The original `Levels/root.tscn` remains available for 2D debugging.

## Controls

| Input | Action |
|---|---|
| 1 | Oblique car-follow camera |
| 2 | Whole-circuit aerial camera |
| 3 | Elevated side-on global camera |
| Scroll | Zoom current camera |
| Right drag | Orbit current camera |
| Left / Right | Select previous / next car |
| Click running-order row | Select that car |
| F | Toggle follow / aerial |
| Space or Pause button | Pause / resume simulation |
| Q / E | Lower / raise selected car's tire mode |
| A / D | Lower / raise selected car's power mode |
| H | Hide / show HUD |

A full-circuit view necessarily renders individual cars very small at real-world scale. Use Follow to inspect cars and wheel-to-wheel spacing; the circuit map locates the selected car.

## Design references and how they informed this prototype

- [Art of Rally official site](https://www.artofrally.com/): primary visual reference for recognizable vehicle silhouettes and a spacious composition. Interpretation for this circuit game: quiet terrain, legible tarmac, restrained team colors, a camera with room ahead of the car. This is not a reproduction of its assets.
- [Blender: Shade Smooth & Flat](https://docs.blender.org/UATEST/manual/en/dev/modeling/meshes/editing/face/shading.html): face normals define hard facets. The procedural meshes use independent face normals and deliberate proportions; no noisy normal-map texture is needed to suggest detail.
- [Godot: Using the SurfaceTool](https://docs.godotengine.org/en/stable/tutorials/3d/procedural_geometry/surfacetool.html): mesh attributes and normals. Our small mesh builder writes vertices, per-face normals and colors directly to ArrayMesh, with Godot's clockwise winding.
- [Godot: Environment and post-processing](https://docs.godotengine.org/en/stable/tutorials/3d/environment_and_post_processing.html): directional illumination, ambient fill and tone mapping. This version uses one warm sun and a cool neutral fill; bloom, depth-of-field and screen-space effects are unnecessary for the first prototype.
- [Godot: MultiMeshInstance3D](https://docs.godotengine.org/en/stable/classes/class_multimeshinstance3d.html): repeated trees are instanced in separate spatial clusters so they can be culled as groups.

Our art decisions: grey-green landscape, charcoal asphalt, ivory and vermilion kerbs, simple grandstands and garages. HUD uses flat off-white panels, dark type and a single red selection accent. No generated image assets, decorative gradients, glass cards or fictional telemetry. Buildings are illustrative props, not surveyed Silverstone architecture.

## Geometry and performance boundaries

`TrackSurfaceGeometry` reconstructs height by integrating Core's Grade values with numerical seam correction. Bank and crown use the same polynomial as Core. Core's Silverstone elevation profile is already an approximation, not a surveyed height field. World axes are `(Core X, height, Core Y)`.

Static meshes are built once in chunks. Cars interpolate the last two completed simulation poses on rendering frames. Core runs in one background task at a time at a fixed 1/60 s step. Presentation reads Core only between steps; car transforms and map positions are copied. Strategy commands apply between steps, and pausing lets at most the current in-flight step finish. There is no accumulated catch-up queue. The HUD updates five times per second. No Godot collision shapes or rigid bodies are added. The selected car's front wheels visualize the actual curvature request.

`CORE` in the HUD measures one simulation step, not rendering cost. `SIM` estimates the maximum real-time factor from that cost (capped at 1x), not a race-speed setting. The inherited master rule drivers were measured at roughly 250 ms per 20-car step on this development machine; the worker keeps cameras/UI responsive but does not make the simulation real-time. This prototype deliberately does not substitute scripted movement or the learned-driver branch. Use actual graphical runs for GPU/frame-rate comparisons; a headless run does not establish rendering performance.

## Verification

```sh
dotnet test Core/Tests/StintegyEVO.Core.Tests.csproj -c Release --filter FullyQualifiedName~TrackSurfaceGeometryTests
/Applications/Godot_mono.app/Contents/MacOS/Godot --path . --script res://Tools/lowpoly_smoke.gd
```

The smoke script captures three camera views, a compact window and running cars under `.tmp/lowpoly/`, and exercises pause, selection, zoom and resume. Numerical tests guard against reversed transverse height, flat elevation and a discontinuity at the lap seam.
