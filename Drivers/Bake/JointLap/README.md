# Joint racing line bake

Run from the repository root in Release:

```powershell
dotnet run --project Drivers/Bake/JointLap -c Release -- official:silverstone .tmp/joint-silverstone five 48
dotnet run --project Drivers/Bake/JointLap -c Release -- geometry official:silverstone .tmp/joint-silverstone/five.table .tmp/joint-silverstone/five.geometry
```

The bake uses the stock car and development compound. The final argument is
the common start and finish speed in m/s. The default uses the compact
speed-chain evaluator. Append `--exact` for station-by-station timing.

`five.csv` contains the selected closed lap's positions, lateral offsets, speeds,
and cumulative time. `five.table` is the five-place state table for the race.

Set `STINTEGY_CIRCUIT=official:silverstone`, `STINTEGY_JOINT_TABLE` to the absolute
path of `five.table`, and `STINTEGY_JOINT_GEOMETRY` to the absolute path of
`five.geometry`. Run `res://Tools/Demo/lane_driver_demo.tscn` in
Godot .NET for a rolling two-car test. Use the same stock lane data as the bake.
The two drivers share the loaded table and use live grip and the current race
snapshot to choose a line and speed together. The opponent check projects
current speed and lateral offset through the candidate's first edge.
The regular lane driver remains the default outside this demo. The joint demo
does not yet provide a fresh following plan when every candidate is rejected.
It also does not search across the start/finish seam.

The geometry file stores every legal edge regardless of its speeds. It also
stores the joined curvature of the supplied value table's continuations. The
same base edges can be reused with another value table on the same lattice.
If its continuations differ, their joined curvature is calculated on demand.
