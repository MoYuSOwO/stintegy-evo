#!/usr/bin/env bash
# learn-all.sh <tag> <circuit>... : learn several circuits at once (PAR at a time, default 8),
# with one combined progress log, $STINTEGY_WORK/learn-<tag>/all.log, to watch.
set -u
W=${STINTEGY_WORK:?set STINTEGY_WORK to a scratch folder}
here=$(cd "$(dirname "$0")" && pwd)
tag=$1; shift
dir=$W/learn-$tag; mkdir -p "$dir"
for id in "$@"; do touch "$dir/$id.progress"; done
(cd "$dir" && tail -n +1 -F "${@/%/.progress}" > all.log 2>/dev/null &)
for id in "$@"; do
  while [ "$(jobs -r | wc -l)" -ge "${PAR:-8}" ]; do sleep 10; done
  "$here/learn.sh" "$tag" "$id" >> "$dir/status.txt" 2>&1 &
done
wait
