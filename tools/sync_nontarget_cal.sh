#!/bin/bash
# nontarget_cal(타깃 없는 카메라/열화상 ↔ LiDAR 캘리브레이션) 코드를 third_party/nontarget_cal 로 복사한다.
#
#   tools/sync_nontarget_cal.sh [원본 리포 경로] [커밋/브랜치]
#     원본 기본값: /hdd/DM_calib/nontarget_cal   커밋 기본값: HEAD
#
# - git 에 커밋된 파일만 가져온다 (git archive). 작업 중인 수정 · .venv · 결과 · 가중치는 절대 안 들어온다.
# - 가져온 커밋 해시 · 날짜 · 원본 경로를 third_party/nontarget_cal/VENDORED_FROM 에 남긴다.
# - 원본 리포는 읽기만 한다.
# 복사한 뒤 차량 PC 에서 tools/setup_online_calib.sh 로 venv 에 다시 설치해야 새 코드가 돈다.
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
SRC=${1:-/hdd/DM_calib/nontarget_cal}
REF=${2:-HEAD}
DST="$HERE/third_party/nontarget_cal"

git -C "$SRC" rev-parse --verify "$REF^{commit}" >/dev/null
COMMIT=$(git -C "$SRC" rev-parse "$REF^{commit}")
SUBJECT=$(git -C "$SRC" log -1 --format=%s "$COMMIT")
CDATE=$(git -C "$SRC" log -1 --format=%cI "$COMMIT")
if [ -n "$(git -C "$SRC" status --porcelain --untracked-files=no)" ] && [ "$REF" = "HEAD" ]; then
  echo "주의: $SRC 에 커밋 안 된 수정이 있습니다 — 커밋된 $COMMIT 만 가져옵니다" >&2
fi

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
git -C "$SRC" archive --format=tar "$COMMIT" | tar -x -C "$TMP"
rm -rf "$DST"
mkdir -p "$(dirname "$DST")"
mv "$TMP" "$DST"
trap - EXIT
cat > "$DST/VENDORED_FROM" <<INFO
# DM_clipGUI 가 차량에 가져가는 nontarget_cal 사본 — 직접 고치지 말고 원본에서 고친 뒤 다시 동기화한다
source: $SRC
commit: $COMMIT
commit_date: $CDATE
subject: "$(echo "$SUBJECT" | sed 's/"/\\"/g')"
synced_at: $(date -Iseconds)
INFO
# 원본에 아직 안 들어간 차량 쪽 수정 (third_party/nontarget_cal_patches/*.patch) 을 다시 적용한다
PATCH_DIR="$HERE/third_party/nontarget_cal_patches"
FAILED=0
for p in "$PATCH_DIR"/*.patch; do
  [ -e "$p" ] || continue
  n=$(basename "$p")
  if (cd "$DST" && patch -p1 --dry-run -s < "$p" >/dev/null 2>&1); then
    (cd "$DST" && patch -p1 -s < "$p")
    echo "local_patch: $n" >> "$DST/VENDORED_FROM"
    echo "로컬 패치 적용: $n"
  elif (cd "$DST" && patch -p1 -R --dry-run -s < "$p" >/dev/null 2>&1); then
    echo "로컬 패치 $n 은 원본에 이미 들어 있음 — $p 를 지우세요"
  else
    echo "오류: 로컬 패치 $n 이 새 원본에 맞지 않습니다 — 원본에 같은 수정이 들어갔는지 확인하고 패치를 고치거나 지우세요" >&2
    FAILED=1
  fi
done
echo "nontarget_cal $COMMIT → $DST"
exit $FAILED
