# 실제 solver 스트림 검증

2026-09-26–27. 합성 리플레이를 사용하지 않고 `nontarget_cal run`과 Xvfb/GLX 뷰어를 동시에 실행했다.
전체 clip GUI, 센서, 레코더, GPU는 실행하지 않았다. [실행 ID·게이트 증거](img/calib_live_runs.json),
[warm 캡처 메타데이터](img/calib_live_capture.json), [zero-shot 캡처 메타데이터](img/calib_live_zero_capture.json).

## 연결 위치와 규약

- 도구 `viz.py`/`viz_preview.py`: 크기1 비동기 mailbox, 프로세스 간 파일 잠금, 완전 스냅샷,
  원자적 NPZ/JPG, 선택 카메라 제어, 자산 보존/로그 회전. 새 의존성은 없다.
- `lo/kiss.py`, `lo/refine.py`, `lo/maps.py`: 이미 계산된 sweep·현재 refinement 상태·지도 샘플.
- `rgb/rigba.py`, `thermal/tba.py`: 기존 LM의 초기/채택 상태 콜백.
  각 `solve.py`는 현재 T/K/D·실제 잔차/비용/시간 추정과 정확한 최종값을 발행한다.
- `pipeline.py`, `viz_pipeline.py`, `worker.py`, `cli.py`: 실행 ID·구간/bag 문맥, 켜기/끄기,
  캐시 복원, 실제 검증 게이트. 절반/held-out 추정이 production 포즈를 덮지 않는다.
- GUI `scripts/calib_viz/`: 현재 렌즈 투영, stream tail/재시작, 선택 제어와 표시.
  `scripts/online_calib.py`: 최신 run_start 이후 오류/경고만 사용한다.

`calib-viz/1`에 run_id, 현재 K/D, pass/iteration, per-camera cost/timing 필드를 추가했다.
하위 호환을 유지하며 원자적 완전 상태는 앞 이벤트 없이 복구할 수 있다.
상태 최대4Hz·지도1Hz·영상1Hz, 자산 기본120초/최대512개와 현재 참조, 로그16MiB를 적용한다.
상세 내용은 [스트림 규약](calib_viz_stream.md)을 따른다.

## 실행 범위

입력은 `/media/shchon11/Seonghyun/Data Machine/Calibration/rec_20260924_204902`.
원본 `/hdd/DM_calib/nt_regress/full/{work,out}`는 읽기 전용이다. 추출 메타데이터를 복사하고
원본 카메라/LiDAR/트랙 파일에는 읽기 전용 참조를 사용했다. 원본 주요 파일의 크기·mtime도 전후 동일했다.
요청 경로 `/hdd/DM_calib/nt_viz_test`는 이 세션에서 생성이 거부되어,
GUI의 `.omx/nt_viz_test/`를 사용했다(버전 관리 제외).

| 실행 | 데이터·설정 | 실제 결과 |
|---|---|---|
| warm | S01 120–160초, S02 180–220초; RGB14+열화상2; 이전 full/out 초기값 | 실제 전체 단계 완료. 초기 LO273초·RGB270초. SIGTERM 종료 후 캐시로 재개, 재개 실행618.6초 |
| zero-shot | S01 120–160초; RGB14; `--force`; 카메라당 최대400트랙 |244.5초. 설계 원점에서 시작해 실제 포즈·렌즈 이동을 캡처 |

Warm은 S01 KISS/정밀화/지도 생성을 새로 수행하고 S02 LO를 재사용했다. 실시간 캡처는 초기 실행과
재개 실행 양쪽에서 얻었으며 각 PNG의 `run_id`·`seq`를 JSON에 기록했다. 재개 시 지도는
`map_source=cached-lo-sample`로 복원하며 새 계산으로 표시하지 않는다.

이것은 제한된 자원에서 전체 연결 경로를 검증한 smoke 실행이다. Warm은 LO 정밀화1회, RGB6회,
열화상5회/1round, 열화상 최소 구간2개, 검증 영상3장으로 줄였다. Zero-shot은 RGB3회,
첫 pass4회, 검증 영상2장이다. solver의 기존 A 단계·바깥 반복은 유지했다.
게이트 임계값은 바꾸지 않았다. 25구간 전체 정확도 회귀 결과로 해석하면 안 된다.
처음에는 최대3 worker, 재개는2 worker, zero-shot은1 worker로 실행했다.
별도 동일성 검사는 한 번에 한 numerical process로 진행해 전체 동시 수를4 이하로 제한했다.

Warm의 실제 게이트는14대 통과·2대 실패다. 우측 후방 카메라는 광축 방향1σ 62mm가60mm를 넘었고,
좌측 열화상은 투표 peak −1.25/0.26px로 실패했다. 이 두 카메라만 빨간색이다.
Zero-shot의 축소 실행은 물리 배치 규칙2개가 실패했다. 실패 결과를 합격으로 바꾸지 않았다.

## 화면

| 상태 | 파일 | 증거 |
|---|---|---|
| 새로 계산 중인 KISS LO | [mid_lo](img/calib_live_mid_lo.png) | 실제 sweep 지도·궤적 성장 |
| warm RGB accepted LM | [mid_rgb](img/calib_live_mid_rgb.png) | A 반복2, 실제 포즈 변화 |
| 현재 RGB 추정 투영 | [projection](img/calib_live_projection.png) | 실제 영상, LiDAR 투영, KLT 관측 |
| 열화상 위치 풀이 | [mid_thermal](img/calib_live_mid_thermal.png) | 실제 시간 offset·행 readout·투영 |
| warm 최종 검증 | [final](img/calib_live_final.png) |14대 통과·2대 실패, 실제 검증1σ |
| zero-shot 중간 | [zero_converging](img/calib_live_zero_rgb_converging.png) | 원점에서 벗어나는 실제 B 반복 포즈 |
| zero-shot 최종 | [zero_final](img/calib_live_zero_final.png) | 실제 최종값과 실패한 종합 게이트 |

뷰어는 Xvfb/llvmpipe, 최대12FPS, 지도30k점, 투영5k점, 관측100개로 실행했다.
캡처는 `scripts/calib_live_capture.py`가 실제 tail과 `snapshot_applied`/render tick에서 저장했다.
재시작 후에도 PNG와 스냅샷의 실행 ID를 보존한다. 초반 열화상 대기 프레임과 잘못된 센서 선택 캡처는
검토 후 제거하고 실제 자산이 도착한 화면으로 교체했다.

## 자동 검사와 수치 동일성

- 도구 `.venv/bin/python -m pytest -q tests`: **47 passed**.
- GUI `QT_QPA_PLATFORM=offscreen OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python3 -m pytest -q test tests`:
  **116 passed**. Vendoring 후 다시 통과했다. 공유 부하에서228.35초 소요.
- 신규 모듈·GUI 변경 파일 Pyflakes, Python 문법 검사, `git diff --check` 통과.
  별도 타입 검사 설정은 없다. 원래 thermal solver의 미사용 import/변수 경고4개는 수치 경로를 유지하기 위해 그대로 뒀다.
- `tests/test_equivalence.py`: 원본 S01 RGB BA의 T/intrinsics 차이0, 관측800,876개 일치.
  KLT는 원본 및 저장본과 동일(1,200프레임, 30,245트랙, 657,036관측).
  LO 정밀화는 기존 최적화의 반올림 차이 `max |dT|=6.946e-13`로 기존 허용치 통과.
  [원본 비교](img/calib_live_original_equivalence.json).
- `tests/test_refusals.sh`: 이름표 모순, 이름 자동 식별, 회전 부족, 디스크 부족4종의 종료 코드·JSON 결과를
  직접 확인했다. [기대값 비교](img/calib_live_refusals.json).
- 실제 S01 첫20 sweep KISS를 off/on/on/off로 수행한 모든 수치 배열이 동일했다.
  [원시 배열 동일성](img/calib_live_lo_identity.json). 이 짧은 실행의 시간 차이는 성능 보장으로 사용하지 않는다.

실제 S01 전체 LO refinement, RGB14대, 열화상2대의 viz off/on 출력에서 수치 JSON과 NPZ 배열 payload가
바이트 단위로 일치했다. `wall_s`와 ZIP 메타데이터만 비교에서 제외했다. RGB4회 균형 실행의 수치 SHA-256은
`5a144d4532ca77579d28ec82ba03c25df4192f3190ece2fd09e97f2401d9170e`로 모두 같다.
RGB는 초기 회전8회+바깥3패스×5회, 열화상은 초기8회+1패스×5회, LO는399 sweep/401 knot의1회 정밀화다.

SIFT는 새로운 S01 계산1회를 commit1af77de의 기준
`nt_regress/crosstime/cache_snap/S01.npz`와 비교했다. `la/lb/dist/dts/imp` 다섯 배열이 동일하며,
8,813쌍·269 impostor 값, 후보14,773개·채택8,071개·연결 landmark5,995개다. 실행1359.477초.
SIFT에는 새 emitter 훅이 없으므로 중복 viz-on 재계산을 주장하지 않는다.
측정 중 바뀐 시각화 코드와 별개로 해당 SIFT 소스는 그대로였음을 별도 해시로 확인했다.

[전체 동일성·소스 해시·발행 간격 증거](img/calib_live_equivalence.json).
최종 실제 실행의 서로 다른 JPG 발행 간격은 모두1초 이상이며 최솟값은 **1.000975초**였다.
동일 자산을 반복 참조한 스냅샷은 새 영상으로 세지 않았다.

## 오버헤드: 2% 런타임 목표는 확인하지 못함

이 작업의 다른 계산·GUI 테스트·뷰어를 모두 끝낸 뒤 기본 round-robin 모드에서 고정 순서
off→on→on→off를 실행했다. 다른 사용자의 작업은 유지됐고16코어 머신의 load average는 약14.6–21로 변했다.

| 고정 실행 순서 | Wall 초 | Process CPU 초 |
|---|---:|---:|
| off |222.387|256.228|
| on |238.858|185.451|
| on |151.959|208.932|
| off |113.431|207.925|

평균 off167.909초, on195.408초로 **관측된 wall 차이는 +16.377%**다.
따라서 **<2% 런타임 목표를 달성했다고 주장하지 않는다**. 공유 부하가 크게 달라 이 차이 전체를
emitter의 인과적 오버헤드라고 해석할 수도 없다. 결과가 나쁘다는 이유로 추가 균형 실행을 반복하지 않았다.
앞선 GUI/실행과 겹친 비교는 이 표에 섞지 않았다.

별도 기본 round-robin on 계측1회는 wall326.682초, process CPU206.716초였으며 수치 해시는 그대로였다.
동기 observer28회는 CPU0.5633초/wall0.5888초, 비동기 I/O14회는 CPU1.0280초/wall44.7994초였다.
계측한 두 스레드의 CPU 합은1.5913초, worker CPU의 **0.7698%**다.
비동기 wall 시간은 대기와 solver와의 중첩을 포함하므로 CPU 비율이나 I/O wall 합을 런타임 오버헤드로 바꾸지 않는다.

계측 함수는 [프로파일러](artifacts/calib_live_profile.py)에 보존했다(경로만 CLI 인수로 정리).
`tests/viz_equivalence.py --scratch NEW_DIR --repeats 2 --sift-golden-only`로 원래 검사를 재현할 수 있다.
프로파일러는 새 출력 경로만 허용한다.

```bash
OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MKL_NUM_THREADS=4 \
NONTARGET_LO_THREADS=4 NONTARGET_TIE_THREADS=1 NONTARGET_DEVICE=cpu \
OPENCV_FOR_THREADS_NUM=1 CUDA_VISIBLE_DEVICES='' \
/hdd/DM_calib/nontarget_cal/.venv/bin/python docs/artifacts/calib_live_profile.py \
  --scratch /tmp/calib-live-new-profile --baseline docs/img/calib_live_profile_baseline.json
```

## 제한

- 지도는 구간별 독립 `lidar_window:<bag>/<window>` 좌표다. 전역 등록을 한 것처럼 이어 붙이지 않는다.
- 프러스텀은 개략적인 표시이며 영상 투영은 실제 현재 K/D와 왜곡 모델을 사용한다.
- 미리보기는 각 창의 실제 중간 프레임을 사용한다. KLT는 실제 관측이며 검증된 점별 LiDAR 대응선이 아니다.
  열화상 행 시각 보정은 현재 추정으로3회 고정점 반복한다. 영상은 최대1Hz이며 I/O 지연이 추가될 수 있다.
- 다중 bag은 문맥·좌표 분리 및 순서 테스트로 검증했다. 이번 실제 bag 실행은 한 개 bag이다.
- 수치 미제공 값은 비워 둔다. 반복 중 RMS와 완료 결과의 median은 `metric_source`로 구분한다.

## 커밋과 재현

GUI 코드 커밋은 `69c543f`, vendoring 커밋은 `a0001bd`이다.

원본 도구 `.git`은 읽기 전용 마운트라 `git add`가 `index.lock: Read-only file system`으로 거부됐다.
원본 소스 수정은 보존하고, 같은 기반 커밋996d2f8의 별도 checkout에서 다음 커밋을 만들었다.

- `e47725c`: 비동기 producer, 원자적/다중 프로세스 기록, 실제 미리보기.
- `15e24a9`: 인코딩 시간 변화에도 실제 영상 발행 간격1초 보장.
- `7fbc5ff32494f40d40d4b58069e31367c09d0d90`: LO·RGB·열화상·검증 훅, CLI·캐시 복원, 동일성 검사.

`tools/sync_nontarget_cal.sh`는 이 실제 커밋을 사용했다. 원본 작업 파일·커밋·vendor 추적 파일98개가
바이트 단위로 일치한다. [검증된 bundle](artifacts/nontarget_cal_live.bundle), [사본 증거](img/calib_live_vendor.json).
**원본 저장소 branch는 아직996d2f8이다.** VENDORED_FROM도 `upstream_commit_applied: false`를 명시한다.
원본 Git 쓰기가 가능해진 뒤 다음처럼 커밋을 가져올 수 있다. `reset --mixed`는 작업 파일을 지우지 않고
branch·index를 bundle의 커밋으로 맞춘다. 원본 HEAD가 이후 변경됐다면 아래 대신 먼저 병합을 검토한다.

```bash
git -C /hdd/DM_calib/nontarget_cal fetch \
  /hdd/DM_calib/DM_clipGUI_viz/docs/artifacts/nontarget_cal_live.bundle HEAD
git -C /hdd/DM_calib/nontarget_cal reset --mixed 7fbc5ff32494f40d40d4b58069e31367c09d0d90
# 원본 출처의 VENDORED_FROM으로 다시 기록하려면:
bash tools/sync_nontarget_cal.sh /hdd/DM_calib/nontarget_cal HEAD
```

실행 설정은 [warm](artifacts/calib_live_warm.yaml), [zero-shot](artifacts/calib_live_zero.yaml)에 보존했다.
다음 명령은 캐시가 없으면 실제 추출부터 수행한다. 기록된 시간은 앞서 설명한 캐시 재사용 조건이다.

```bash
/hdd/DM_calib/nontarget_cal/.venv/bin/nontarget_cal run \
  --bags '/media/shchon11/Seonghyun/Data Machine/Calibration/rec_20260924_204902' \
  --out /path/to/test/out --workdir /path/to/test/work \
  --windows S01:120:160,S02:180:220 --mode warm \
  --init /hdd/DM_calib/nt_regress/full/out --config docs/artifacts/calib_live_warm.yaml

xvfb-run -a -s '-screen 0 1920x1280x24 +extension GLX' \
  env QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  python3 scripts/calib_live_capture.py --stream /path/to/test/work/viz --output docs/img
```

GitHub/원격 저장소에는 push하지 않았다.
