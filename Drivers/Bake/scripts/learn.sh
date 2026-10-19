#!/usr/bin/env bash
# learn.sh <tag> <circuit> [lanes dir] : learn one circuit from its baked lanes
# (LaneCircuitLearning.LearnBakedCircuit, full grip only unless TOP_ONLY=0).
# Progress: $STINTEGY_WORK/learn-<tag>/<circuit>.progress; the learned folder is saved
# under $STINTEGY_WORK/learn-<tag>/stintegy-lanes/<circuit>/ -- copy it to
# Drivers/Cars/<car>/<circuit>/ to ship it. Build Drivers/Tests -c Release first.
set -u
W=${STINTEGY_WORK:?set STINTEGY_WORK to a scratch folder}
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
tag=$1; id=$2; lanes=${3:-$REPO/Drivers/Bake/lanes}
dir=$W/learn-$tag; mkdir -p "$dir"; rm -f "$dir/$id.progress"
cd "$REPO"
STINTEGY_LEARN_TOP_ONLY=${TOP_ONLY:-1} TMP="$(cygpath -w "$dir")" TEMP="$(cygpath -w "$dir")" \
STINTEGY_LEARN_PROGRESS="$(cygpath -w "$dir/$id.progress")" STINTEGY_LANE_PROBES=1 \
STINTEGY_LEARN_CIRCUIT=$id STINTEGY_LANES3D="$(cygpath -w "$lanes")" \
  dotnet test Drivers/Tests -c Release --no-build --filter "FullyQualifiedName~LearnBakedCircuit" \
  --logger "console;verbosity=detailed" > "$dir/$id.log" 2>&1
echo "LEARNED $tag $id: $(grep -E "^ $id from|^ hot|^ attack|^ standing|^ saved|^ hold|^ switch|已通过|失败" "$dir/$id.log" | tr '\n' ' ')"
