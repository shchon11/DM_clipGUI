#!/bin/bash
# Scaling benchmark of a full from-scratch run (2026-09-24 night bag, zeroshot, the 25 regression windows)
# for several worker counts, each inside a systemd scope capped like the 32 GB vehicle PC.
#
#   tools/bench_scaling.sh OUTROOT [WORKERS ...]          (default workers: 4 8 12 16)
#
# Environment (defaults in brackets):
#   PY     python of the venv            [/hdd/DM_calib/nontarget_cal/.venv/bin/python]
#   BAG    the bag directory             [/media/shchon11/Seonghyun/Data Machine/Calibration/rec_20260924_204902]
#   SLICE  systemd user slice            [calib.slice]  its own MemoryMax must be >= MEM, else it caps first
#   MEM    MemoryMax of each run's scope [32G]          (the vehicle PC's RAM; no swap)
#   HEAD   admission headroom inside it  [6]            (GB left for OS + clip GUI, as on the vehicle PC)
#   KEEP   1 = keep the heavy work data  [0]            (extract/, thermal16/, lo/, tracks/ are deleted after
#                                                        the regression; events, tasks, logs, outputs stay)
# Per run OUTROOT/wN/: events.jsonl, stderr.log, mem.log (scope memory every 10 s: time, current GB, anon GB,
# peak GB), disk.log (work dir GB every 60 s), regression.md/json, stage_times.md, projection.txt.
# Needs ~75 GB free on the work disk per run (deleted between runs unless KEEP=1).
set -u
OUTROOT=${1:?usage: bench_scaling.sh OUTROOT [WORKERS ...]}
shift
WORKERS=("$@"); [ ${#WORKERS[@]} -eq 0 ] && WORKERS=(4 8 12 16)
CODE=$(cd "$(dirname "$0")/.." && pwd)
PY=${PY:-/hdd/DM_calib/nontarget_cal/.venv/bin/python}
BAG=${BAG:-"/media/shchon11/Seonghyun/Data Machine/Calibration/rec_20260924_204902"}
SLICE=${SLICE:-calib.slice}
MEM=${MEM:-32G}
HEAD=${HEAD:-6}
WIN="S01:120:160,S02:180:220,S03:300:340,S04:380:420,S05:560:600,S06:660:700,S07:700:740,S08:760:800,S09:840:880,S10:955:995,W00:0:18,W01:72:120,W02:160:180,W03:220:260,W04:260:300,W05:340:380,W06:420:460,W07:460:500,W08:500:530,W09:540:560,W10:600:623,W11:740:760,W12:800:840,W13:880:920,W14:920:955"
mkdir -p "$OUTROOT"

for W in "${WORKERS[@]}"; do
  R="$OUTROOT/w$W"
  if [ -e "$R/work" ]; then echo "$R/work exists: skipped (delete it to re-run)"; continue; fi
  mkdir -p "$R"
  # the vehicle PC has no cgroup limit: keep its 6 GB OS/GUI headroom inside the emulating scope
  printf 'resources: {cgroup_headroom_gb: %s}\n' "$HEAD" > "$R/bench.yaml"
  echo "[$(date +%T)] w$W: start (scope MemoryMax=$MEM in $SLICE)"
  systemd-run --user --scope -q --slice="$SLICE" -p MemoryMax="$MEM" -p MemorySwapMax=0 \
    env PYTHONPATH="$CODE" NONTARGET_MAX_PROCS="$W" "$PY" -m nontarget_cal run --bags "$BAG" \
      --out "$R/out" --workdir "$R/work" --mode zeroshot --sensors rgb,thermal --windows "$WIN" \
      --config "$R/bench.yaml" > "$R/events.jsonl" 2> "$R/stderr.log" &
  RUN=$!
  # samplers: scope memory (the scope's cgroup, found from the run's pid) and work-dir disk use
  (
    CG=""
    while kill -0 $RUN 2>/dev/null; do
      if [ -z "$CG" ]; then
        P=$(pgrep -f -- "--workdir $R/work" | head -1)
        [ -n "$P" ] && CG=/sys/fs/cgroup$(sed -n 's/^0:://p' /proc/$P/cgroup 2>/dev/null)
      fi
      if [ -n "$CG" ] && [ -r "$CG/memory.current" ]; then
        cur=$(cat "$CG/memory.current"); anon=$(awk '$1=="anon"{print $2}' "$CG/memory.stat")
        peak=$(cat "$CG/memory.peak" 2>/dev/null || echo 0)
        echo "$(date +%s) $(awk -v a=$cur -v b=$anon -v c=$peak 'BEGIN{printf "%.2f %.2f %.2f", a/1e9, b/1e9, c/1e9}')"
      fi
      sleep 10
    done
  ) > "$R/mem.log" &
  ( while kill -0 $RUN 2>/dev/null; do echo "$(date +%s) $(du -s -B1G "$R/work" 2>/dev/null | cut -f1)"; sleep 60; done ) > "$R/disk.log" &
  wait $RUN
  echo "exit $?" >> "$R/stderr.log"
  echo "[$(date +%T)] w$W: run done ($(tail -1 "$R/stderr.log"))"
  systemd-run --user --scope -q --slice="$SLICE" -p MemoryMax="$MEM" \
    "$PY" "$CODE/tests/regression.py" --out "$R/out" --work "$R/work" --components --json "$R/regression.json" \
      --md "$R/regression.md" >> "$R/stderr.log" 2>&1
  PYTHONPATH="$CODE" "$PY" "$CODE/tools/stage_times.py" "$R/work" > "$R/stage_times.md" 2>&1
  PYTHONPATH="$CODE" "$PY" "$CODE/tools/stage_times.py" --tasks "$R/work" >> "$R/stage_times.md" 2>&1
  PYTHONPATH="$CODE" "$PY" "$CODE/tools/project_runtime.py" "$R/work" --cpus 20 --ram-gb 32 --workers 6 8 10 12 \
    > "$R/projection.txt" 2>&1
  echo "[$(date +%T)] w$W: $(grep -m1 -o 'RGB vs[^)]*)' "$R/regression.md"); $(grep -m1 -o 'Thermal vs.*' "$R/regression.md")"
  if [ "${KEEP:-0}" != "1" ]; then
    rm -rf "$R/work/extract" "$R/work/thermal16" "$R/work/tracks" "$R/work/lo" "$R/work/thermal/tracks" "$R/work/thermal/edges"
  fi
done
echo "summary: $OUTROOT/w*/stage_times.md, regression.md, mem.log (max anon: sort -k3 -n | tail -1)"
