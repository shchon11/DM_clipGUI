#!/usr/bin/env bash
# clip — DM Clip GUI 런처.
#
#   1) ROS + 워크스페이스 소싱
#   2) net_probe(IP별 트래픽 카운터)에 cap_net_raw가 없으면 sudo setcap 한 번
#      (colcon build로 바이너리가 새로 만들어졌을 때만 비밀번호를 묻는다)
#   3) GUI는 일반 사용자 권한으로 실행
#
# GUI 자체를 sudo로 띄우지 않는 이유: 클립/설정 파일이 root 소유가 되고, DDS 공유메모리
# 세그먼트도 root 소유가 되어 일반 사용자로 도는 카메라 드라이버와 충돌한다.
#
# ~/.bashrc:  alias clip='~/DM_clipGUI/scripts/clip_launch.sh'
# 워크스페이스 경로가 다르면:  DM_CLIP_WS=/path/to/ws clip

# (set -u 사용 금지: ROS setup.bash가 nounset에 안전하지 않음)
WS="${DM_CLIP_WS:-$HOME/DM_clipGUI}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
else
  echo "워크스페이스가 빌드되지 않음: $WS/install 없음 (colcon build 먼저)" >&2
  exit 1
fi

NP="$(readlink -f "$WS/install/clip_recorder/lib/clip_recorder/net_probe" 2>/dev/null || true)"
if [ -n "$NP" ] && [ -x "$NP" ]; then
  if ! getcap "$NP" 2>/dev/null | grep -q cap_net_raw; then
    echo "net_probe 패킷 캡처 권한 부여 (빌드 후 한 번): sudo setcap cap_net_raw+ep $NP"
    if ! sudo setcap cap_net_raw+ep "$NP"; then
      echo "권한 부여 실패 — IP별 트래픽 없이 NIC 합계만 표시됩니다" >&2
    fi
  fi
else
  echo "net_probe 없음 ($NP) — IP별 트래픽은 표시되지 않습니다" >&2
fi

# PTP 점검 — GUI 로 센서를 켜기 전에 시간 동기 체인(Orin GNSS → PC → 카메라)을 눈에 띄게 확인한다.
# alias ptp 와 같은 스크립트(ptp_setup.py status). 준비가 안 됐으면 그 자리에서 ptp(세우기)를 돌릴지 묻는다.
# 건너뛰기: DM_SKIP_PTP=1 dm
PTP_SCRIPT="$WS/scripts/ptp_setup.py"
if [ -f "$PTP_SCRIPT" ] && [ -z "$DM_SKIP_PTP" ]; then
  echo "================ PTP 점검 (ptp status) ================"
  if python3 "$PTP_SCRIPT" status --no-sudo; then
    echo "================ PTP 준비됨 → GUI 를 엽니다 ================"
  elif [ -t 0 ]; then
    echo "======================================================="
    read -r -p "PTP 가 준비 안 됐습니다. 지금 ptp 로 세울까요? [Y/n] " answer
    if [ "${answer:-Y}" != "n" ] && [ "${answer:-Y}" != "N" ]; then
      python3 "$PTP_SCRIPT" start || read -r -p "PTP 세우기 실패 — 그래도 GUI 를 열까요? [Enter] " _
    fi
  else
    echo "PTP 준비 안 됨 — 터미널에서 ptp 를 실행하세요 (GUI 는 그대로 엽니다)" >&2
  fi
fi

# ros2 run 을 거치지 않고 GUI 를 바로 exec 한다. ros2 run 은 받은 SIGTERM 을 자식에게 넘기지 않아서
# 이 PID 로 보낸 종료 신호가 GUI 에 닿지 않았다. GUI 는 Ctrl+C · SIGTERM · SIGHUP 을 받으면
# 녹화 정리 → 센서 → 레코더 순으로 내리고 끈다 (한 번 더 누르면 기다리지 않고 강제 종료).
#
# GUI 본체는 소스(scripts/clip_gui.py)를 바로 띄운다. install/.../clip_gui 는 CMake 가 이름을 바꿔 설치하느라(RENAME)
# 심볼릭 링크가 아니라 복사본이라 colcon build 를 다시 하기 전까지 예전 코드다 — 2026-09-23 GUI 를 다시 켜도 수동 녹화
# 자동 진단 · 새 진단 창이 안 나왔다 (설치 사본이 소스보다 365줄 뒤처짐). 함께 import 하는 모듈(sensor_stage 등)은 소스로
# 링크돼 있어 최신이라, 일부만 옛 코드로 도는 걸 알아채기 어려웠다.
SRC_GUI="$WS/scripts/clip_gui.py"
if [ -f "$SRC_GUI" ]; then
  exec python3 "$SRC_GUI" "$@"
fi
GUI="$WS/install/clip_recorder/lib/clip_recorder/clip_gui"
if [ -x "$GUI" ]; then
  exec "$GUI" "$@"
fi
exec ros2 run clip_recorder clip_gui "$@"
