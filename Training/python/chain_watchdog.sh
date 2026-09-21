#!/bin/zsh
# The parent chain's watchdog: it lights the next leg itself.
#
# The chain's standing order is that it does not stop, and its gate is
# mechanical (chain_gate.py). What used to sit between a leg ending and the
# next one starting was a session waking up to read the log -- which once did
# not happen for four and a half hours (2026-09-20-parent3i/manifest.md). So
# this waits on the running leg, runs the gate, and either lights the next leg
# on the same recipe or holds and says why.
#
#   ./chain_watchdog.sh parent3i 43170
#
# It holds -- writing chain-watchdog.hold and exiting -- when the gate reads
# FLAT (no line broken; the order says report before the next leg), when the
# leg did not finish cleanly, when the checkpoint is missing, or when a leg is
# already running. Everything it does goes to chain-watchdog.log.

set -u
# The bake's own worktree, where the logs and checkpoints are; it is a
# different checkout from the one this script is read out of, and stays at the
# bake commit, so nothing is written into it but the run's own files.
GATE="${0:A:h}/chain_gate.py"
cd "${CHAIN_DIR:-${0:A:h}}"

PYTHON=/Users/jayhuang/Code/stintegy-evo/Training/.venv/bin/python
LOG="$PWD/chain-watchdog.log"
HOLD="$PWD/chain-watchdog.hold"
TAGS=(parent3i parent3j parent3k parent3l parent3m parent3n parent3o parent3p)

tag=$1
pid=$2

say() { print -r -- "$(date '+%F %T')  $*" >> $LOG }

hold() {
  say "HOLD: $*"
  print -r -- "$(date '+%F %T')  $*" > $HOLD
  exit 1
}

say "watching $tag (pid $pid)"

while true; do
  while kill -0 $pid 2>/dev/null; do sleep 20; done
  say "$tag (pid $pid) exited"

  verdict=$($PYTHON $GATE run-$tag.log run-parent3*.log 2>&1)
  status=$?
  say "gate on $tag:"
  print -r -- "$verdict" | sed 's/^/    /' >> $LOG
  (( status == 0 )) || hold "gate on $tag returned $status; the chain waits for a ruling"

  index=${TAGS[(i)$tag]}
  next=${TAGS[$((index + 1))]}
  [[ -n "$next" ]] || hold "no tag after $tag; extend TAGS to carry on"

  [[ -f checkpoints/latest$tag.pt ]] || hold "checkpoints/latest$tag.pt is missing"

  if pgrep -f "train.py --solo --track silverstone" > /dev/null; then
    hold "a leg is already running; not lighting $next on top of it"
  fi

  nohup $PYTHON -u train.py --solo --track silverstone --batch 64 --seed 1 \
    --tag $next --steps 10000000 --eval-every 25000 --eval-batch 6 \
    --fixed-alpha 0.0011 --resume checkpoints/latest$tag.pt \
    > run-$next.log 2>&1 &
  disown
  sleep 20
  pid=$(pgrep -f "tag $next")
  [[ -n "$pid" ]] || hold "$next did not start; see run-$next.log"
  say "lit $next (pid $pid, ppid $(ps -o ppid= -p $pid | tr -d ' ')), resumed from latest$tag.pt"
  tag=$next
done
