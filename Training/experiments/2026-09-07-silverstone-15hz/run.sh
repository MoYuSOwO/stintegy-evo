#!/usr/bin/env bash
# Three seeds, one after another. Serial because the machine saturates:
# two arms in parallel each ran at 39% of a solo arm, so sharing costs more
# wall clock in total than queueing does, and queueing shows the first
# curve in a third of the time.
set -u
cd /Users/jayhuang/Code/stintegy-evo/Training/python
PY=/Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python3
export STINTEGY_HOST_BIN=/Users/jayhuang/Code/stintegy-evo/Training/StintegyEVO.TrainingHost/bin/Release/net8.0/StintegyEVO.TrainingHost
for SEED in 1 2 3; do
  echo "=== seed $SEED starting $(date '+%F %T') ==="
  "$PY" -u train.py --solo --track silverstone \
      --batch 64 --steps 400000 --seed "$SEED" --tag "15hz-s$SEED" \
      > "run-15hz-s$SEED.log" 2>&1
  echo "=== seed $SEED finished $(date '+%F %T') rc=$? ==="
done
echo "=== all seeds done $(date '+%F %T') ==="
