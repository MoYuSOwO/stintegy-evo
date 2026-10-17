# Circuits

One folder per circuit. Everything about a circuit that is not a car or a
driver lives in its folder; the game reads it as it is and changes nothing.

```
Circuits/silverstone/
  circuit.json      name, start line, grid, where the data came from
  track.csv         the road, one row a metre
  pitlane.csv       the pit lane, one row a metre, where the circuit has one
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
| `x_m`, `y_m` | the middle of the road, metres: x to the east, **y to the south** (the game's plane, as Godot's 2D one); a map's y, to the north, goes in negated, or the circuit comes out mirrored |
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

## The pit lane

Every real circuit has a pit lane; one with none, such as the test
layouts, simply has no `pitLane` in its `circuit.json`. Where there is one:

```json
"pitLane": {
  "track": "pitlane.csv",
  "lengthMeters": 1133,
  "entryMeters": 5270,
  "exitMeters": 687,
  "speedLimitKph": 80,
  "speedLimitFromMeters": 45,
  "speedLimitToMeters": 1088,
  "boxes": { "count": 12, "firstMeters": 667.5, "spacingMeters": 14, "offsetMeters": -2.5 }
}
```

`entryMeters` and `exitMeters` are the rows of `track.csv` where the lane
leaves the track and comes back: the lane's first row is that entry row's
point of the centreline, and its last row the exit row's, and the game
refuses a lane whose ends are not those points. The limit holds
between the two lines, measured along the lane. A box is where a car
stops: `count` of them from `firstMeters` along the lane, `spacingMeters`
apart, `offsetMeters` to the left of the lane's middle (negative is right).

`pitlane.csv` is the lane from end to end, any shape it has, `lengthMeters`
+ 1 rows a metre apart, in the same plane as the track:

| column | meaning |
|---|---|
| `x_m`, `y_m` | the middle of the lane, metres, as in `track.csv` |
| `z_m` | height, metres |
| `width_m` | wall to wall: a pit lane has no run-off |

## Making one

The folders are written by `Tools/Circuits`, which holds the source
centrelines and everything done to them: smoothing, scaling to the
published length, centring, run-off, height and banking.

```bash
dotnet run --project Tools/Circuits -- export
dotnet run --project Tools/Circuits -- check
```

`check` builds every circuit again and compares it with its folder.

The pit lanes are the real ones, from OpenStreetMap (the Formula One lane
where a circuit has more than one). The ways each circuit uses are named in
`Tools/Circuits/Program.cs`; `fetch-pitlanes` reads them through the Overpass
API into `Tools/Circuits/Sources/pitlanes/`, and `export` lays each on our
road (see `Tools/Circuits/Pits.cs`):

```bash
dotnet run --project Tools/Circuits -- fetch-pitlanes
```
