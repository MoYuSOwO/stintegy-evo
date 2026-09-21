# Scenery plans

One file per circuit, named for it: `silverstone.json` is Silverstone's.
It says which props stand where, and it is meant to be read and edited by
a person.

```json
{
  "track": "silverstone",
  "props": [
    { "prop": "grandstand", "s": 120.0, "d": -46.0, "yaw": 3.142, "scale": 1.0 },
    { "prop": "pine", "s": 300.0, "d": 68.0, "scale": 1.15,
      "repeat": { "count": 14, "step_s": 26.0, "step_d": 2.5 } }
  ]
}
```

| key | meaning |
|---|---|
| `prop` | a file in `Assets/Scenery`, by name |
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

## Writing one

Two ways, and the second is the one to use.

1. **By hand.** The format is six keys; a text editor and F5 will do it.
2. **By pointing.** Run the game, press **F2**, and the viewer becomes the
   editor: click the ground to place, `[` and `]` to change prop, `Z` to
   undo, `ctrl+S` to write the plan back to this directory. The prop lands
   where the cursor is and the file records it as a station and an offset.

The Godot editor is deliberately not the answer here. The circuit does not
exist in it — road, kerbs, barriers and terrain are all built at load from
the track model — so placing props there means placing them against an
empty grid and running the game to find out where they went. The viewer
already draws the circuit, so the viewer is where scenery is placed.

**None of this is physical.** A plan cannot put anything in the car's way:
the barrier is the only boundary the simulation knows, and scenery lives
entirely in the renderer.
