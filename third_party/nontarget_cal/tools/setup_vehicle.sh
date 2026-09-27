#!/bin/bash
# One-time setup on the vehicle PC (Ubuntu 22.04 / ROS 2 Humble / Python 3.10). No ROS needed at run time.
#   bash tools/setup_vehicle.sh [--cpu-torch]
# Creates .venv next to this repository and installs nontarget_cal (command: .venv/bin/nontarget_cal).
set -e
HERE=$(cd "$(dirname "$0")/.." && pwd)
cd "$HERE"
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -U "pip>=24" "setuptools>=64,<80" wheel
if [ "$1" = "--cpu-torch" ]; then
  .venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
fi
.venv/bin/pip install -r requirements.txt
.venv/bin/pip install --no-deps -e .
.venv/bin/nontarget_cal --help > /dev/null && echo "nontarget_cal installed: $HERE/.venv/bin/nontarget_cal"
.venv/bin/python - <<'PY'
import os, torch, cv2, numpy, scipy, kiss_icp, rosbags
print("cpus", os.cpu_count(), "| torch", torch.__version__, "cuda", torch.cuda.is_available(),
      "| opencv", cv2.__version__, "| numpy", numpy.__version__, "| kiss_icp", kiss_icp.__version__)
PY
