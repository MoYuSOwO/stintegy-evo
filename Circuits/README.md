# Circuits

One folder per circuit. Everything about a circuit that is not a car or a
driver lives in its folder; the game reads it as it is and changes nothing.

```
Circuits/silverstone/
  circuit.json      name, start line, grid, where the data came from
  track.csv         the road, one row a metre
  scenery.json      which props stand where (see SCENERY.md)
  props/            models this circuit brings of its own
  LICENSE.*         the terms of the source data, where it has any
```

Circuits kept out of the repository go in `private/circuits/<id>/`, laid out
the same way; the game looks there after `Circuits/`.

## `track.csv`

The road, finished: closed, starting at row 0, one row a metre. The rows
are spaced as near a metre as a lap of a whole number of metres allows (the
lap is spread evenly over every row), and the game refuses a file whose rows
are not.

| column | meaning |
|---|---|
| `x_m`, `y_m` | the middle of the road, metres |
| `z_m` | height, metres; the gradient is read off it |
| `width_m` | edge to edge |
| `runoff_left_m`, `runoff_right_m` | run-off beyond each edge, to the wall |
| `bank_slope` | tangent of the road's tilt across, positive leaning left |
| `crown` | how much the tilt changes per metre across: the crown that sheds water |

Nothing in the game scales, smooths or resamples it. Heading and curvature
come from the points, gradient and vertical bend from the heights; that is
all that is computed.

## `circuit.json`

```json
{
  "format": 1,
  "id": "silverstone",
  "name": "Silverstone (Arena Grand Prix)",
  "track": "track.csv",
  "lengthMeters": 5891,
  "startLineMeters": 98,
  "grid": { "count": 23, "firstLeft": true, "offsetMeters": 4.75 },
  "source": { "centreline": "...", "license": "LGPL-3.0", "licenseFile": "LICENSE.LGPL-3.0", "notes": "..." }
}
```

`lengthMeters` is the number of rows. `startLineMeters` is the row the
start/finish line crosses. The grid has `count` boxes, pole on the left
when `firstLeft`, alternating sides `offsetMeters` from the middle of the
road. The size of a box and the distance between them are the series'
rules, the same at every circuit, and are not written here.

## Making one

The folders are written by `Tools/Circuits`, which holds the source
centrelines and everything done to them: smoothing, scaling to the
published length, centring, run-off, height and banking.

```bash
dotnet run --project Tools/Circuits -- export
dotnet run --project Tools/Circuits -- check
```

`check` builds every circuit again and compares it with its folder.
