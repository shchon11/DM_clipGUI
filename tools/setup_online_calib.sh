#!/bin/bash
# 차량 PC 에 온라인(타깃 없는) 캘리브레이션 도구 nontarget_cal 을 설치한다. 한 번만 (코드를 다시 동기화했으면 다시).
#
#   bash tools/setup_online_calib.sh [--cpu-torch] [--src 경로] [--venv 경로]
#
#   --src   설치할 nontarget_cal 코드 (기본: 이 리포의 third_party/nontarget_cal — tools/sync_nontarget_cal.sh 로 넣은 사본)
#   --venv  가상환경 위치 (기본: ~/.local/share/dm_clip_gui/nontarget_cal/venv). GUI 의 기본값과 같다.
#   --cpu-torch  torch 를 CPU 판으로 (도구는 GPU 를 쓰지 않는다 — README §6). 인터넷이 필요하다.
#
# 코드는 venv 안으로 복사 설치한다(-e 아님) — 리포를 pull 해도 돌고 있는 캘리브레이션의 코드가 바뀌지 않고,
# 어떤 커밋이 설치됐는지는 venv/NONTARGET_CAL_VERSION 에 남는다 (GUI 가 보여 준다).
# ROS 는 필요 없다 (bag 은 rosbags 로 직접 읽는다). Ubuntu 22.04 · Python 3.10 에서 확인.
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
SRC="$HERE/third_party/nontarget_cal"
VENV="$HOME/.local/share/dm_clip_gui/nontarget_cal/venv"
CPU_TORCH=0
while [ $# -gt 0 ]; do
  case "$1" in
    --cpu-torch) CPU_TORCH=1 ;;
    --src) SRC=$2; shift ;;
    --venv) VENV=$2; shift ;;
    *) echo "모르는 인자: $1" >&2; exit 2 ;;
  esac
  shift
done
[ -f "$SRC/pyproject.toml" ] || { echo "nontarget_cal 코드가 없습니다: $SRC (tools/sync_nontarget_cal.sh 먼저)" >&2; exit 1; }

mkdir -p "$(dirname "$VENV")"
python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install -U "pip>=24" "setuptools>=64,<80" wheel
if [ "$CPU_TORCH" = 1 ]; then
  "$VENV/bin/pip" install torch --index-url https://download.pytorch.org/whl/cpu
fi
"$VENV/bin/pip" install -r "$SRC/requirements.txt"
# 복사 설치 — 빌드 디렉터리를 임시로 (원본 사본을 더럽히지 않게)
BUILD=$(mktemp -d)
trap 'rm -rf "$BUILD"' EXIT
cp -r "$SRC/." "$BUILD/"
"$VENV/bin/pip" install --no-deps --force-reinstall "$BUILD"
{
  echo "installed_at: $(date -Iseconds)"
  echo "installed_from: $SRC"
  if [ -f "$SRC/VENDORED_FROM" ]; then grep -v '^#' "$SRC/VENDORED_FROM"; fi
  if git -C "$SRC" rev-parse HEAD >/dev/null 2>&1 && [ ! -f "$SRC/VENDORED_FROM" ]; then
    echo "commit: $(git -C "$SRC" rev-parse HEAD)"
  fi
} > "$VENV/NONTARGET_CAL_VERSION"
"$VENV/bin/nontarget_cal" --help > /dev/null
echo "설치 완료: $VENV/bin/nontarget_cal"
cat "$VENV/NONTARGET_CAL_VERSION"
"$VENV/bin/python" - <<'PY'
import os, torch, cv2, numpy, scipy, kiss_icp, rosbags
print("cpus", os.cpu_count(), "| torch", torch.__version__, "cuda", torch.cuda.is_available(),
      "| opencv", cv2.__version__, "| numpy", numpy.__version__, "| kiss_icp", kiss_icp.__version__)
PY
