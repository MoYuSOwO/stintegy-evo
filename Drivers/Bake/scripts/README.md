# Bake and learn scripts

How the lanes in `Drivers/Bake/lanes/` and the learned folders in `Drivers/Cars/formula_a_mixed/<circuit>/`
were made. Git Bash on Windows; `STINTEGY_WORK` is any scratch folder.

1. Export a circuit and the car's grip table for the optimiser:
   `dotnet run -c Release --project Drivers/Bake/ExportTracks -- "$STINTEGY_WORK/raw" spa monaco ...`
2. Bake: `Drivers/Bake/scripts/bake.sh spa` (about 10 to 90 minutes a circuit; IPOPT gives up a lane
   after 30 minutes and tries it again from a lane that baked). Copy
   `$STINTEGY_WORK/bake-spa/spa.lanes3d.csv` to `Drivers/Bake/lanes/`.
3. Build `dotnet build Drivers/Tests -c Release`, then learn:
   `Drivers/Bake/scripts/learn-all.sh mytag spa monaco ...` (full grip, about 50 minutes for 8
   circuits at once). Watch `$STINTEGY_WORK/learn-mytag/all.log`. Copy each
   `$STINTEGY_WORK/learn-mytag/stintegy-lanes/<circuit>/` to `Drivers/Cars/formula_a_mixed/<circuit>/`,
   keeping a circuit's `energy` section in `driver.json` if it had one.

Do not build Drivers/Tests in Release while a learning run is using it.

What is in the repository now (2026-10-18):
- `lanes/`: the 11 circuits baked on the current physics. Silverstone is baked on the repaired
  Abbey (#95), and its left lane did not bake (IPOPT stalls near The Loop, s about 1048 m), so the
  driver falls back to its own geometric left lane there.
- `Drivers/Cars/formula_a_mixed/`: the run called ilc5, full grip only, super tyre, for spa, monaco,
  silverstone, monza, interlagos, zandvoort, singapore and baku. Silverstone's was learned before
  the Abbey repair and should be learned again on the lanes here.
