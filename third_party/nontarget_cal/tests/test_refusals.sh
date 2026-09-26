#!/bin/bash
# The refusal paths of `nontarget_cal check` on the 2026-09-24 bag (each ~30 s):
#   1. a name map with two cameras swapped (front5 <-> front8)  -> ambiguous_camera_names (geometry contradicts)
#   2. no name map at all (known maps disabled)                   -> names identified from geometry, check passes
#   3. an absurd rotation requirement                             -> not_enough_rotation
#   4. an absurd disk margin                                      -> insufficient_disk
# Prints the last JSON line (refusal / check_done) and the exit code of each.
HERE=$(cd "$(dirname "$0")/.." && pwd)
PY=${PY:-$HERE/.venv/bin/python}
BAG="/media/shchon11/Seonghyun/Data Machine/Calibration/rec_20260924_204902"
T=${1:-/hdd/DM_calib/nt_regress/refusals}
mkdir -p "$T"
python3 - "$HERE" "$T" <<'PY'
import sys, yaml
here, t = sys.argv[1], sys.argv[2]
m = yaml.safe_load(open(f"{here}/nontarget_cal/data/name_maps/camera_name_map_20260924.yaml"))
c = m["cameras"]
c["camera_26076474"]["old"], c["camera_26075999"]["old"] = "camera_front8", "camera_front5"
yaml.safe_dump(m, open(f"{t}/swapped_map.yaml", "w"))
open(f"{t}/no_maps.yaml", "w").write("paths:\n  known_name_maps: []\n")
open(f"{t}/rot.yaml", "w").write("preflight:\n  rgb:\n    min_rotation_deg: 100000\n")
open(f"{t}/disk.yaml", "w").write("preflight:\n  disk_margin_gb: 100000\n")
PY
run() { name=$1; shift; rm -rf "$T/$name"; "$PY" -m nontarget_cal check --bags "$BAG" --sensors rgb,thermal --workdir "$T/$name" "$@" > "$T/$name.jsonl" 2> "$T/$name.err"; echo "== $name exit $?"; tail -1 "$T/$name.jsonl" | cut -c1-400; }
run swapped --name-map "$T/swapped_map.yaml"
run geometry --config "$T/no_maps.yaml"
run rotation --config "$T/rot.yaml"
run disk --config "$T/disk.yaml"
