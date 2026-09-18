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

# ros2 run 을 거치지 않고 GUI 를 바로 exec 한다. ros2 run 은 받은 SIGTERM 을 자식에게 넘기지 않아서
# 이 PID 로 보낸 종료 신호가 GUI 에 닿지 않았다. GUI 는 Ctrl+C · SIGTERM · SIGHUP 을 받으면
# 녹화 정리 → 센서 → 레코더 순으로 내리고 끈다 (한 번 더 누르면 기다리지 않고 강제 종료).
GUI="$WS/install/clip_recorder/lib/clip_recorder/clip_gui"
if [ -x "$GUI" ]; then
  exec "$GUI" "$@"
fi
exec ros2 run clip_recorder clip_gui "$@"
