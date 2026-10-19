#!/usr/bin/env bash
# bake.sh <circuit> [raw dir] : bake the three lanes of one circuit with TUM's 3D optimiser.
# Needs: STINTEGY_WORK (a scratch folder) and a Python venv with TUM's requirements
# at $STINTEGY_WORK/tumvenv (or STINTEGY_TUMVENV). Raw input from ExportTracks
# (default $STINTEGY_WORK/raw). Output: $STINTEGY_WORK/bake-<circuit>/<circuit>.lanes3d.csv,
# log: $STINTEGY_WORK/bake-<circuit>.log. Copy the csv to Drivers/Bake/lanes/ to ship it.
set -u
W=${STINTEGY_WORK:?set STINTEGY_WORK to a scratch folder}
VENV=${STINTEGY_TUMVENV:-$W/tumvenv}
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
id=$1; raw=${2:-$W/raw}; out=$W/bake-$id
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
cd "$REPO/Drivers/Bake"
"$VENV/Scripts/python.exe" bake_lanes_3d.py "$id" --raw "$(cygpath -w "$raw")" --out "$(cygpath -w "$out")" --warm > "$W/bake-$id.log" 2>&1
echo "BAKED $id: $(grep -E 'bake lap|refused|wrote' "$W/bake-$id.log" | tr '\n' ' ')"
