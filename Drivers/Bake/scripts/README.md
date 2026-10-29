# Bake scripts

How the lanes in `Drivers/Bake/lanes/` were made. Git Bash on Windows; `STINTEGY_WORK` is any scratch folder.

1. Export a circuit and the car's grip table for the optimiser:
   `dotnet run -c Release --project Drivers/Bake/ExportTracks -- "$STINTEGY_WORK/raw" spa monaco ...`
2. Bake: `Drivers/Bake/scripts/bake.sh spa` (about 10 to 90 minutes a circuit; IPOPT gives up a lane
   after 30 minutes and tries it again from a lane that baked). Copy
   `$STINTEGY_WORK/bake-spa/spa.lanes3d.csv` to `Drivers/Bake/lanes/`.
3. The lanes are the whole of the driver's circuit file: nothing is learned beforehand any more. A baked
   `lanes.csv` goes into the car's pack with its `lanes.json` and `notes.json` stamp.

What is in the repository now (2026-10-18):
- `lanes/`: the 11 circuits baked on the current physics. Silverstone is baked on the repaired
  Abbey (#95), and its left lane did not bake (IPOPT stalls near The Loop, s about 1048 m), so the
  driver falls back to its own geometric left lane there.
