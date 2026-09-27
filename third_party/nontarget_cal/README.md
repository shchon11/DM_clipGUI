# nontarget_cal — 타깃 없는 카메라/열화상 ↔ LiDAR 캘리브레이션 (DM 리그)

주행 bag 하나(또는 여러 개)만 넣으면 **보드 없이** 14대 RGB 카메라와 2대 열화상 카메라(FLIR A70)의
LiDAR(Ouster `os_lidar`) 기준 외부 파라미터와 내부 파라미터를 구하고, 팀 형식 YAML, 투영 이미지,
카메라별 지표 표, 보고서를 만든다.

방법은 이미 검증된 두 작업을 그대로 옮긴 것이다(수치 코드는 바꾸지 않았다):

- RGB: `/hdd/DM_calib/lidar_odo` — LiDAR 오도메트리(KISS-ICP + 연속시간 다중 스윕 정밀화)로 궤적을 구해
  **고정**하고, KLT 트랙 번들 조정 + 특징점–LiDAR 평면 구속. 렌즈(fx=fy, cx, cy, k1, k2)도 같이 푼다.
- 열화상: `/hdd/DM_calib/thermal_lo` — 같은 궤적으로 열화상 KLT 번들 조정 + LO로 누적한 근거리 LiDAR
  에지 정렬 항 + 좌우 열화상 교차 관측. 창(구간)별 시간 오프셋과 행 판독 시간을 같이 푼다.

## 1. 설치 (차량 PC: Ubuntu 22.04, ROS 2 Humble, Python 3.10)

ROS는 필요 없다(bag은 `rosbags`로 직접 읽는다). GPU는 선택: 있으면 RGB·열화상 KLT 추적의 Lucas-Kanade를 CUDA로
돌린다(torch에 들어 있는 Triton 커널, `rgb.tracks.device: auto`; CUDA가 없으면 OpenCV OpenCL, 그것도 없으면 CPU).
번들 조정·LiDAR 오도메트리의 계산 핵심은 `numba`로
컴파일한다(없으면 검증된 torch/numpy 코드가 돈다).

```bash
cd /path/to/nontarget_cal
python3 -m venv --system-site-packages .venv      # 시스템 numpy/opencv가 있으면 재사용
.venv/bin/pip install -U "pip>=24" "setuptools>=64,<80"
.venv/bin/pip install -r requirements.txt         # torch는 CPU판으로 충분: --index-url https://download.pytorch.org/whl/cpu
.venv/bin/pip install --no-deps -e .
.venv/bin/nontarget_cal --help
```

- 검증한 버전: numpy 2.2.6, scipy 1.15.3, opencv 4.13, torch 2.10, rosbags 0.11.0, **kiss-icp 1.3.0(고정)**,
  numba 0.61.2(+ llvmlite 0.44.0; 우분투 apt의 numba 0.55는 numpy 2와 안 맞으므로 venv에 pip로 설치), triton 3.6.0
  (torch 2.10 휠에 포함; CUDA LK 커널은 처음 한 번 컴파일되어 `~/.triton/cache`에 남는다).
- 이 개발 박스에는 `/hdd/DM_calib/nontarget_cal/.venv`에 설치되어 있다.
- 설정은 한 파일: `nontarget_cal/config/default.yaml`(경로, 임계값, 토픽 이름, 스레드 수). 바꿀 부분만
  적은 YAML을 `--config my.yaml`로 주면 덮어쓴다. 물리 배치 규칙은 `nontarget_cal/config/layout_rules.yaml`.
- 이 차량 전용 데이터는 패키지 안 `nontarget_cal/data/`에 있다: 차체 마스크(`masks/`), 설계 각도·기준 리그
  (`rig_design.yaml`), 알려진 토픽 이름 대응표(`name_maps/`), Ouster 링 시프트.

## 2. 명령 하나

```bash
nontarget_cal run --bags BAG [BAG ...] --out 결과폴더 [--mode warm|zeroshot] [--init 이전결과폴더] \
                  [--sensors rgb,thermal] [--workdir 작업폴더]
nontarget_cal check --bags BAG [BAG ...]          # 사전 점검만 (약 30초)
nontarget_cal report 결과폴더                      # 보고서 다시 출력
```

- `--mode zeroshot`(기본, `--init`이 없을 때): 보드·이전 값 없이 설계 각도에서 시작(2단계 풀이).
- `--mode warm --init DIR`: 이전 결과에서 시작. DIR = 이 도구의 결과 폴더, 팀 deliverable 폴더
  (`extrinsic/`+`intrinsic/`), `lidar_odo/final` 형식 JSON 폴더, `thermal_lo_calib.yaml`. 쉼표로 여러 개.
- `--workdir`: 큰 중간 파일이 쌓이는 곳(기본: `결과폴더_work`). **여유 공간이 큰 디스크**로 지정.
- `--name-map FILE`: 카메라 토픽 이름 대응표. 없으면 (1) 토픽이 원래 이름인지, (2) 알려진 대응표가 맞는지
  보고, (3) 둘 다 아니면 **영상 기하로 식별**한다. 어느 경우든 영상 기하(카메라 간 상대 회전 + 기준선
  방향)로 **검증**하고, 모순되거나 애매하면 멈춘다(조용히 추측하지 않음).
- `--windows S01:120:160,S02:180:220`: 구간을 직접 지정(초, bag의 첫 `/gps/fix` 기준). 없으면 자동 선택.
- `--force`: 데이터 양·동기·노출 거절과 bag 간 불일치를 무시하고 진행(디스크 부족은 무시 불가).
- 여러 bag: 창을 모두 모아 한 번에 푼다(카메라 파라미터 공유). 그 전에 **bag별로 따로 풀어 비교**하고,
  경험적 반복성을 넘게 다른 bag은 "리그가 바뀌었나?"로 표시하고 제외한다(`--force`면 포함).

### 재시작·중단

단계마다 작업 폴더에 결과가 캐시된다. 같은 명령을 다시 실행하면 끝난 단계는 건너뛴다. 중간에 꺼지거나 bag
디스크가 빠져도 이미 끝난 단계·창은 남아 있고, 쓰다 만 창은 `.partial`로 남았다가 다시 만들어진다.

### GUI 연동

- **stdout = 한 줄에 JSON 하나**(진행 이벤트). 사람용 로그는 stderr와 `작업폴더/log.txt`.
- 이벤트 종류: `run_start`, `stage_start`, `stage_skip`, `stage_progress`(done/total), `stage_end`(wall_s,
  disk_gb, workdir_free_gb), `warning`(code, msg, msg_ko), `task_failed`, `refusal`, `error`, `run_end`.
- 종료 코드: 0 정상, **2 거절**(마지막 JSON 줄의 `code`, `msg`(영어), `msg_ko`(한국어)), 1 오류.
- 거절 코드: `not_enough_motion`, `not_enough_rotation`, `not_enough_thermal_windows`, `insufficient_disk`,
  `sync_broken`, `exposure_absurd`, `ambiguous_camera_names`, `unknown_camera_names`, `missing_topic`,
  `bag_disagrees`, `no_good_windows`, `bad_init`.

### 실제 계산의 3D 라이브 스트림

`run`은 기본으로 `<workdir>/viz`에 `calib-viz/1` 스트림을 발행한다. `--viz-dir DIR`로 위치를 바꾸고,
`--no-viz` 또는 설정 `viz.enabled: false`로 끈다. GUI 없이도 실행할 수 있다. 수치 계산은 발행 결과를
읽지 않으며, 발행 오류는 한 번 기록하고 계산을 계속한다. 새 의존성은 없다.

- `manifest.json`은 실제 카메라 모델과 실행 ID, `events.jsonl`은 최신 상태를 복원할 수 있는 완전 스냅샷이다.
  워커 간 파일 잠금으로 JSONL과 상태를 직렬화하고, 작은 NPZ/JPG는 임시 파일에서 원자적으로 교체한다.
- KISS-ICP sweep, LO refinement 반복, 지도 생성 중 이미 계산된 점·궤적을 샘플링한다.
  지도는 `lidar_window:<bag>/<window>` 좌표이며 독립 창을 이어 붙이지 않는다.
- RGB/열화상 LM의 초기값과 **채택된 반복 해**에서 포즈·K/D·재투영 RMS·비용을 발행한다.
  카메라 비용은 Huber 영상 잔차만, `objective_cost`는 ties/prior를 포함한 solver 목적함수다.
  열화상은 창별 시간 오프셋과 행 판독 시간을 포함한다. 검증용 절반/held-out 풀이가 최종 리그를 덮지 않는다.
- 영상은 실제 노출 시각으로 이동 보정한 LiDAR 점과 KLT 관측을 사용한다. 선택한 카메라를
  `control.json`의 `camera`로 전달하며, 선택이 없으면 순환한다. solver 콜백은 이미지 인코딩을 기다리지 않는다.
- 전역 발행 상한은 상태 4 Hz, 지도 1 Hz, 영상 1 Hz다. 작은 메일박스에서 밀린 상태를 합치고,
  자산은 기본 120초 보존, 이벤트는 16 MiB에서 최신 완전 상태로 회전한다.
  재시작은 새 `run_id`를 사용하고, 이전 실행 워커의 쓰기는 거부한다.

GUI의 `docs/calib_viz_stream.md`가 상세 좌표·전송 규약이다. 합성 리플레이는 GUI 데모 전용이며
이 도구는 합성 중간 해나 불확실도를 발행하지 않는다. 다음 검사는 실제 수치 배열의 on/off 동일성도 확인한다.

```bash
.venv/bin/python -m pytest -q tests
# 실제 S01 입력: 원본 스크립트와의 비교 / viz on-off 비교 (출력은 별도 scratch)
.venv/bin/python tests/test_equivalence.py --scratch /path/to/scratch/original
.venv/bin/python tests/viz_equivalence.py --help
```

## 3. 파이프라인 (단계별로 캐시됨)

| 단계 | 하는 일 |
|---|---|
| names | 토픽 이름 → 표준 이름(camera_front1..9, top, side_*, rear_*, thermal_left/right) + 기하 검증 |
| preflight | 움직인 시간, 누적 회전·회전 수, 속도, Ouster timestamp_mode(없으면 헤더-녹화시각 차), 카메라 간 시각 차(PTP), 노출, 열화상 fps·중복, 근거리 구조물, 시간·디스크 추정, 여유 공간 |
| windows / extract | 움직인 부분을 최대 40 s 창으로 나누고(회전 많은 창 우선), 창마다 한 번의 범위 질의로 RGB JPEG(원본 그대로), LiDAR 스윕(점별 시각 포함), 열화상 16비트 PNG(창·카메라별 한 파일 `thermal16/<창>/<카메라>.pack`에 이어 붙임), 정차 순간 1개 추출. bag 전체 INS는 작은 토픽만 한 번 훑어서 저장 |
| lo | 창이 추출되는 대로 바로: KISS-ICP → 연속시간 정밀화(5회) → 5 cm 지도 캐시 → 품질 검사(1 s 간격 스윕 불일치, 지도 두께). 불량 창은 제외 |
| rgb | 카메라·창별 KLT 트랙(창이 추출되는 대로; 끝나면 그 창의 RGB 영상은 10장에 1장만 남김) → (zeroshot) 회전 많은 5개 창으로 설계 각도에서 2단계 풀이 → bag별 풀이·비교 → 전체 풀이 |
| thermal | 회전 ≥ 120° 창에서 열화상 KLT → 렌즈·행 판독(25 m 이상 점, 회전만) → 근거리 에지 준비 → 위치 풀이 → 좌우 교차 관측 → 최종 풀이(회전, 위치, fx, fy/fx, 창별 시간 오프셋) |
| validation | 서로 겹치지 않는 두 절반으로 따로 풀어 비교(경험적 1σ), 반대쪽 절반에서 트랙 재투영(held-out), LiDAR 에지 거리, 투표(voting) 게이트, 물리 배치 규칙 |
| outputs | YAML, camera_info, 이미지, 지표 표, 보고서, zip |

## 4. 데이터 수집 요구사항

근거: `lidar_odo/README.md` §3.3·§6, `handeye/README.md` §4(오차 ∝ 1/√누적회전), `thermal_lo/README.md`.

| 항목 | 권장 | 최소(미만이면 거절) | 이유 |
|---|---|---|---|
| RGB: 움직인 시간 | **3–5분 이상** | 80 s | 40 s 한 창: 광축 방향 ±2.5–3 cm, 200 s: ±1–1.5 cm, 450 s: 카메라 간 1–2 mm |
| RGB: 누적 회전 | **1000–2000° 이상**(좌·우 회전 모두) | 250° | 위치(레버암)는 회전에서만 보인다. 250°에서 약 21 mm, 2165°에서 약 7 mm |
| 열화상 | **약 10분**, 회전 ≥ 120°인 40 s 창 10개 이상, 2–10 m 근거리 구조물(연석, 주차 차량, 기둥, 벽) | 회전 창 4개 | 열화상 특징점은 대부분 20–30 m라 광축 방향 위치를 근거리 에지가 정한다 |
| 속도 | 5–40 km/h, 직진과 회전이 섞이게 | – | 정지 구간은 쓰지 않음(투영 이미지용 정차 1회는 있으면 좋음) |
| 환경 | 건물·가로수·주차 차량이 있는 시가지, 평평한 노면이 보이는 곳 | – | 노면 평면이 카메라 높이를 정한다(움직임만으로는 높이를 알 수 없음) |
| 시각 동기 | 카메라·LiDAR 모두 **PTP**. Ouster `timestamp_mode = TIME_FROM_PTP_1588`, `/ouster/metadata`도 녹화 | 헤더–녹화시각 차 ±0.5 s 이내, 카메라 간 < 2 ms | 동기가 깨지면 궤적과 영상을 맞출 수 없다 |
| 노출 | 자동 노출. 야간에 수십 us 같은 고정 노출 금지 | 야간 < 500 us 또는 > 35 ms면 거절 | 검은 영상에는 특징점이 없다 |
| 토픽 | `/ouster/points`, `/gps/fix`, `/gps/odom`(또는 `/imu/data`+`/gps/vel`), 각 카메라 `image_rgb/compressed`+`image_raw/metadata`, 열화상 `image_raw`+`metadata` | 필수 토픽 없으면 거절 | |

- **렌즈를 만지거나(초점·조리개) 카메라를 다시 조이면 반드시 새로 수집해서 다시 돌린다.** 이전 결과는 그 순간부터 틀리다.
- 카메라 토픽 이름이 바뀌었으면 결과 폴더의 `summary.json` → `names`와 경고를 꼭 확인한다.

## 5. 결과 읽는 법

```
결과폴더/
  extrinsic/<camera>.yaml    x_cam = R @ x_lidar + t (lidar -> camera), parent_frame os_lidar (x = 차량 후방)
  intrinsic/<camera>.yaml    RGB: equidistant(KB, D=[k1..k4]); 열화상: plumb_bob(k1 k2)
  camera_info/<camera>.yaml  ROS camera_info 형식 (distortion_model equidistant / plumb_bob)
  calib_json/<camera>.json   online_calib 형식 (RGB)
  rig.yaml                   front5 광학 좌표계 기준 리그, 차량 좌표(V: 전/좌/상, 원점 Ouster) 위치
  nontarget_cal.zip          YAML만 묶은 것 (팀 배포용)
  images/parked/…            정차 장면 LiDAR 투영 (색 = 거리, 빨강 가까움 → 파랑 40 m)
  images/driving/…           주행 중 투영(가장 빠른 순간, 급회전, 중간 속도): 점마다 자기 측정 시각의 LO 자세로 옮겨 움직임 보정
  metrics.md / metrics.json  카메라별 지표 표
  report.md                  요약 보고서 (판정, 경고, 배치 검사, 단계별 시간·디스크)
  summary.json               모든 수치
```

**지표 표의 열**(카메라별):

| 열 | 뜻 | 이번 방법의 전형적 값 |
|---|---|---|
| rot 1σ [deg] | 두 절반 풀이의 회전 차이/√2 | 0.02–0.12°(앞줄), rear 더 큼 |
| pos 1σ [mm] | 위치 차이/√2 | 리그 전체↔LiDAR 공통 오프셋 1–2 cm가 바닥 |
| along-axis 1σ [mm] | 카메라 광축 방향 위치 차이/√2 | 5 min 이상에서 약 1 cm |
| focal 1σ [px] | 초점거리 차이/√2 | 약 1 px |
| track reproj [px] | 최종 풀이의 트랙 재투영 중앙값 | 0.8–1.2 px |
| held-out reproj [px] | 한 절반의 보정을 고정하고 다른 절반에서 잰 재투영 | 약 0.9 px |
| LiDAR edge [px] | RGB: 주행 중 LiDAR 깊이 에지 ↔ 영상 에지 거리 중앙값(야간엔 12–16 px로 판별력 없음, parked 값 참고). 열화상: 근거리 에지 점-선 거리 | 열화상 약 1 px |
| vote | 영상을 ±6 px 옮겨가며 에지 일치를 센 투표의 최고점이 (0,0) 근처인지 | pass |

- 1σ는 **절반 데이터 풀이의 산포**라 보수적이다(전체 결과는 약 √2배 좋음). 솔버가 내는 형식 σ는 10–100배
  낙관적이라 싣지 않는다.
- `report.md`의 판정이 "확인 필요"면 이유가 적혀 있다(1σ가 기준 초과, 배치 규칙 실패, 투표 실패).
- 주점(cx, cy)과 회전은 서로 섞인다(10–17 px ≈ 0.5–1°). 투영 정확도에는 영향 없음.
- 카메라 높이는 노면 평면 구속이 정한다. 노면이 거의 안 보이는 데이터에서는 높이가 약해진다.

## 6. 전형적 시간·디스크·메모리

`check`의 `estimate`가 실행 전에 시간·디스크를 계산해 알려 주고, 여유 공간이 모자라면 시작하지 않는다.
단계별 측정값은 결과 폴더 `report.md`의 "단계별 시간·디스크" 표와 `작업폴더/events.jsonl`에 있다
(`tools/stage_times.py 작업폴더`: 단계별 시작·끝; `--tasks`: 작업별 CPU 합).

측정(16코어 공용 개발 박스, 작업 폴더 = HDD, bag = USB 외장 SSD, 다른 에이전트의 풀이가 함께 돌던 상태):
2026-09-24 야간 bag, zeroshot, RGB 14대 + 열화상 2대. 공용 박스의 벽시계 시간은 HDD 경합(IO pressure 50–70 %)과
작업 수 제한(3–4)에 크게 흔들리므로, 기계와 무관한 **작업별 CPU 합**(`tools/stage_times.py --tasks`)으로 비교한다.

| 창 25개(15분 주행), CPU·분 | 속도 개선 전 | 0bb9956(이전 단계) | 지금 |
|---|---|---|---|
| 추출(디스크 위주, 벽시계 약 15–20분) | – | 14 | 14 (같은 코드) |
| LiDAR 오도메트리(KISS + 정밀화 + 지도) | – | 162 | 162 (같은 코드, 결과 4e-13 m 이내 동일) |
| RGB KLT 추적(카메라·창 350개) | 517 | 355 | **119** (CUDA LK) |
| 열화상 추적(32개) | – | 16.5 | 13.8 |
| 열화상 렌즈·에지·위치·연결·최종(+절반) | – | 46 | 35 |
| RGB 풀이(zero-shot 2 + 최종) | – | 21 | 21 |
| 검증 풀이·지표(RGB 절반·held-out·에지, 열화상) | – | 30 | 약 22 (RGB 에지 7배) |
| **합** | – | **646** | **약 390** |
| 작업 폴더 디스크(끝) | 112 GB | 65 GB | 65 GB |

창 4개 전체 실행(처음부터, 작업 3개): CPU 합 127 → 89 CPU·분(RGB 추적 58.6 → 21.4), 벽시계는 23분(작업 4개,
한가할 때) / 43분(작업 3개, HDD 경합) — 공용 박스 벽시계는 비교 기준이 못 된다. 디스크 20 → 13 GB.

**작업 하나의 최대 메모리(비공유 RssAnon, 25창)**: LO 5.7, RGB 최종 풀이 8.4, RGB 절반 4.4, 열화상 위치 7.1, 열화상 최종
7.7(절반 4.5/4.0), 열화상 에지 검증 6.3, 열화상 연결 2.0, RGB 추적(카메라 2대, CUDA) 1.3, 열화상 추적 1.3 GB.
지도는 memory-map(파일 페이지, 회수 가능)이라 RSS에는 더 크게 보인다(RGB 최종 약 13 GB 파일 페이지).
GPU: 추적 작업 하나에 약 330 MB(CUDA 컨텍스트 + Triton + 영상 피라미드); 작업 12개여도 4 GB 미만.

**차량 PC 예상(논리 CPU 20개, RAM 32 GB, RTX 3080 Ti, NVMe)** — `tools/project_runtime.py 작업폴더 --cpus 20
--ram-gb 32 --workers 6 8 10 12`(측정한 작업별 CPU 시간으로 의존성 그래프를 스케줄링; 코어 환산 14개, nice 가중,
추출은 측정 벽시계): 작업 6–12개 모두 **약 37–39분**(추출이 NVMe에서 10분이면 약 34분); 이전 단계 코드의 같은
예측은 54분. 임계 경로: 추출(디스크) → 마지막 창의 LO(약 25분) → RGB 사슬(zero-shot 1·2 → 최종 ∥ 절반 → 에지 →
held-out)과 열화상 사슬(렌즈 → 에지 → 위치 → 연결 → 최종 ∥ 절반 → 검증), 두 사슬 모두 LO 뒤 약 12–13분.
기계가 CPU로 꽉 차므로(합 약 390 CPU·분 ÷ 14코어 = 28분) 작업 수를 8보다 늘려도 빨라지지 않는다.

**32 GB 계획**: 작업 수 자동(`resources.max_procs: auto` = 논리 CPU / 2.5 = 8, RAM 예산 26 GB 안). 새 작업은
작업별 메모리 추정(`resources.task_mem_gb`, 위 측정값 이상으로 잡음: LO 5.5, RGB 풀이 1.5 + 0.3×창 → 최종 9.0,
열화상 풀이 1.5 + 0.45×창 → 8.25, 열화상 에지 검증 7.0)의 합이 RAM − 6 GB(`mem_headroom_gb`: OS·클립 GUI) 안이고,
운영체제가 알려 주는 여유 RAM이 6 GB 이상 남을 때만 시작한다. 가장 무거운 겹침(RGB 최종 + 절반 2개 + 열화상 최종
= 추정 28 GB)은 예산을 넘으므로 순서대로 돈다(실측 합 약 25 GB) → 32 GB에서 OOM 없음. 컨테이너나 systemd
slice(MemoryMax)처럼 cgroup 메모리 제한 안에서 돌면 그 제한을 RAM으로 보고 여유는 `cgroup_headroom_gb`(1 GB).
이 공용 박스처럼 작업 수를 제한하려면 `NONTARGET_MAX_PROCS=3`(또는 설정 `resources.max_procs: 3`).

**처음부터 다시 재는 방법(한가한 박스)**: `tools/bench_scaling.sh 결과폴더 4 8 12 16` — 작업 수별로 25창 전체 실행을
systemd scope(MemoryMax 32G, swap 없음, 여유 6 GB = 차량 PC와 같은 조건) 안에서 하고, 메모리·디스크 표본,
회귀, 단계별 시간, 차량 PC 예상을 남긴다(`SLICE`의 MemoryMax가 32G 이상이어야 함).

속도 관련 설정(`config/default.yaml`):

| 키 | 기본 | 뜻 |
|---|---|---|
| `resources.max_procs` | auto | 작업 프로세스 수: 논리 CPU / `cpus_per_worker`(2.5), 최대 `max_procs_cap`(12), RAM 예산 안. 환경 변수 `NONTARGET_MAX_PROCS`가 우선 |
| `resources.mem_headroom_gb` | 6 | OS·GUI 몫. 작업 시작은 작업별 메모리 추정(`task_mem_gb`)의 합이 RAM − 이 값 안일 때만 |
| `resources.ba_kernel` | numba | 번들 조정의 정규방정식·비용을 numba로 컴파일한 한 번의 루프로(같은 식; 결과 차이 1e-14 상대, 10–25배). `torch` = 검증된 코드 그대로 |
| `resources.lo_kernel` | numba | LO 정밀화의 쌍별 항을 GIL 없이(4e-10 mm 차이, 반복당 2.5배) |
| `resources.tie_cache_gb` | 3 | 풀이 라운드 사이 LiDAR 평면 후보 캐시(결과 동일, 지도 다시 읽기 없음) |
| `rgb.tracks.device` / `thermal.tracks.device` | auto | Lucas-Kanade를 CUDA(Triton 커널, OpenCV와 같은 알고리즘)로: 추적 CPU 48 → 16 ms/프레임(1920×1200), 점 차이 중앙값 0.0002 px·p99 0.04 px, 상태 판정 0.003 % 달라짐. CUDA가 없으면 OpenCL(NVIDIA에서는 드라이버가 기다리며 CPU를 돌려 CPU 이득 없음), 그다음 CPU. `cpu` = 검증된 트랙 비트 동일 |
| `resources.tracks_nice` | 5 | 추적 작업의 OS nice: CPU가 모자라면 LO(두 풀이 사슬의 관문)가 먼저 |
| `validation.rgb_halves_start` | start | RGB 절반 풀이 2개를 최종 풀이와 같은 시작점에서 최종과 나란히(`final` = 검증된 순서: 최종 결과에서, 최종 뒤). 검증의 반복성 숫자만 달라짐 |
| `resources.cgroup_headroom_gb` | 1 | cgroup 메모리 제한 안에서 돌 때의 여유(OS·GUI는 cgroup 밖) |
| `resources.track_cams_per_task` | 2 | 창 하나의 카메라 2대를 한 작업에서 스레드로(트랙 동일) |
| `rgb.solve.step_tol`, `thermal.solve_defaults.step_tol` | 켬 | 받아들인 LM 한 걸음이 모든 카메라 변수에서 이 값보다 작으면 멈춤(25창: 카메라 0.14 mm, 0.0005° 이내 변화). `null` = 검증된 규칙만 |
| `lo.kiss.voxel` | 1.0 | KISS-ICP 복셀(검증 값 0.5): KISS 2.1배, 정밀화 뒤 궤적 차이 중앙값 0.9 mm(1 s 상대 운동 0.07 mm) |
| `extract.keep_rgb_every` | 10 | 추적이 끝난 창의 RGB 영상은 10장에 1장만 남김(디스크 약 40 GB 절약). 0 = 모두 보관 |
| `rgb.tracks.frame_step` | 1 | 2 = 15 Hz로 추적(빠른 설정에서 사용) |
| `resources.ba_device` | cpu | `auto`/`cuda`면 번들 조정을 GPU에서(이 박스에서 이득 없음) |
| `rgb.solve.reuse_pass2_as_final` | false | 창이 5개 이하일 때 세 번째 풀이를 생략. **정확도와 바꾸는 옵션이라 기본은 끔** |

**빠른 설정**: `--config fast`(= `nontarget_cal/config/fast.yaml`: 15 Hz 추적 + 회전이 많은 창 최대 16개).
15 Hz 추적만 쓴 25창 실행(OpenCL LK 시절, 추출·LO 재사용)이 RGB 14/14, 열화상 2/2로 3σ 안(`speed/full_t15`).

속도 개선 요약(모두 측정; 수치가 바뀌는 것은 차이를 적음):

| 무엇 | 효과 | 결과 차이 |
|---|---|---|
| 추출 → LO·RGB 추적 스트리밍, 창의 LO가 끝나면 그 창 열화상 추적, 작업 슬롯 우선순위(추출 > LO > 나머지, 공정 분배) | 단계 겹침 | 없음 |
| 번들 조정 numba 커널(RGB·열화상) | 반복당 18 → 2 s(RGB 25창), 20 → 2 s(열화상) | 1e-14 상대 |
| LM 걸음 크기 정지 | 반복 약 30 % 감소 | ≤ 0.14 mm, 0.0005° |
| LiDAR 평면 연결: 셀 색인 파일, 지도 memory-map, 라운드 간 후보 캐시, O(n) 복셀 중복 제거 | 25창 1회 220–270 → 17–50 s(캐시), 첫 회 약 2배 | 없음(동일) |
| LO: 스윕 한 번 읽기, 정밀화 쌍 항 numba, 트리·world 스레드, float32 저장·메모리 반환 | 정밀화 반복 36 → 14 s, 메모리 7.8 → 5.5 GB | 4e-10 mm |
| KISS 복셀 1.0 | KISS 2.1배 | 궤적 0.9 mm 중앙값 |
| OpenCL LK(이전 단계), 영상 디코딩 스레드, 마스크 셀 건너뛰기, 카메라 2대/작업 | RGB 추적 CPU 517 → 355 CPU·분 | 25창 카메라 ≤ 3.2 mm, held-out −0.04 % |
| CUDA LK(Triton, RGB·열화상), 추적 nice 5 | RGB 추적 CPU 355 → 119 CPU·분, 열화상 추적 LK 25 → 3 ms/프레임 | 점 0.0002 px 중앙값; 창 4개 RGB 14/14·열화상 2/2, 창 25개 열화상 2/2 3σ 안 |
| 열화상: 프레임 팩 파일, 에지 준비의 보정 무관 부분을 LO 직후 캐시, z-buffer numba, 연결 창별 병렬, 에지 연관 스레드, 렌즈·held-out 풀이의 쓰지 않는 LiDAR 보고 생략 | 열화상 풀이 사슬 CPU 46 → 35 CPU·분 | 없음(에지·연결 파일 동일) |
| RGB LiDAR 에지 검증: 카메라 스레드 3개, 스윕 한 번만 읽기 | 7배 | 없음(같은 숫자) |
| RGB 절반 풀이를 최종 풀이와 나란히 | LO 뒤 꼬리 약 5분 단축(차량 PC 예상) | 검증 반복성 숫자만 |
| 추적 후 RGB 영상 솎기 | 디스크 112 → 65 GB | 검증 에지 지표만(쓰는 프레임이 달라짐) |

## 7. 회귀 테스트

- `tests/test_equivalence.py`: 원래 스크립트(lidar_odo, online_calib)와 이 패키지 함수를 같은 입력에 돌려
  비교(LO 정밀화, RGB BA, KLT 트랙). 결과: 세 가지 모두 **비트 단위로 동일**(속도 최적화 뒤 LO 정밀화는 1e-10 mm 차이).
  `kernels`: numba 커널 대 검증된 numpy/torch(LO 정밀화 6e-12 m, RGB BA 5e-15), 복셀 중복 제거 동일;
  `ties`: 후보 캐시를 쓴 평면 연결이 캐시 없는 것과 동일(랜드마크를 옮기고 일부 지운 3 라운드);
  `lk`: CUDA LK 대 OpenCV CPU LK(같은 프레임 쌍·점: 0.0002 px 중앙값, 상태 0.003 %; 트랙 657036 → 659245 관측).
- 2026-09-24 야간 bag 결과: `tests/regression_reduced_20260925.md`(창 4개), `tests/regression_full_20260925.md`(창 25개):
  RGB 14/14, 열화상 2/2가 검증된 결과와 경험적 반복성 3σ 안.
- `tests/test_refusals.sh`: 이름 대응표 모순, 기하만으로 이름 식별, 회전 부족, 디스크 부족 거절 확인.
- `tests/regression.py OUT_DIR`: 2026-09-24 야간 bag 결과를 검증된 결과(`lidar_odo/final`,
  `thermal_lo/thermal_lo_calib.yaml`)와 비교해 카메라별 차이를 경험적 반복성과 함께 표로 낸다.
- `tests/run_regression.sh reduced|full`: 축소판(창 4개, 모든 단계) / 전체(창 25개) 실행.

## 8. 선택 기능: 교차 카메라·교차 시간 랜드마크 연결 (`rgb.crosstime.enabled`, 기본 끔)

같은 구조물을 전방 카메라가 보고 몇 초 뒤 측면·후방 카메라가 보는 것을 이용해, 서로 다른 카메라의 KLT 랜드마크를
3D 위치(깊이 비례 게이트) + 양쪽 트랙 재투영 + SIFT 기술자로 연결해 공동 BA에 넣는다(`nontarget_cal/rgb/crosstime.py`,
창별 캐시로 중단 후 이어서 실행). 2026-09-24 bag, 25개 창, 절반 비교 1σ 중앙값(`tests/crosstime_eval_20260926.txt`):

| 그룹 | 없음 | 느슨한 연결 | 엄격한 연결(합친 점 재투영 1/2 px) |
|---|---|---|---|
| 전방+top 회전 / 광축 | 0.035° / 7.6 mm | 0.015° / 2.9 mm | 0.018° / 5.8 mm |
| 측면 위치 / 광축 | 8.3 / 3.7 mm | 9.8 / 6.0 mm | 9.7 / 5.3 mm |
| 후방 위치 / 광축 | 31.5 / 15.6 mm | 17.9 / 10.3 mm | 26.4 / 15.3 mm |
| 트랙 재투영 중앙값 | 0.84 px | **1.07 px** | 0.86 px |
| held-out 재투영 | 변화 없음 | 변화 없음 | 변화 없음 |

후방 카메라는 다른 카메라와 연결되는 점이 매우 적어(수백 개) 목표였던 측면·후방 개선이 일관되지 않고, 느슨한 연결은
잘못 합쳐진 점 때문에 잔차가 커진다. 그래서 기본은 끔. 비용: 연결 계산 0.5–3시간(디스크 위주) + BA 약 15 %.

**학습 기반 매처로 검증 (`rgb.crosstime.matcher`, 2026-09-26, `tests/crosstime_eval_learned_20260926.txt`)**:
SIFT 기술자 거리 대신, 두 트랙의 KLT 점을 가상 핀홀 카메라로 재샘플링한 160×160 크롭(어안 왜곡 제거, 세계 수직 정렬,
같은 실측 스케일) 쌍에 학습 매처(XFeat+LighterGlue 등)로 대략 확인(중심 전달 ≤ 3 px) 후, 중심 템플릿의 서브픽셀 NCC가
1 px 이내일 때만 연결한다(`nontarget_cal/rgb/lmatch.py`). 매처 비교(S03·W05 후보 1866쌍): SuperPoint/ALIKED/DISK+LightGlue,
XFeat(+LighterGlue/MNN) 모두 같은 판정(검증률·2 px 이동 오수락 3.8–3.9 %)이었고 정밀도는 정렬된 크롭의 NCC에서 나온다
(모델 없는 `ncc`도 같음). 라이선스: SuperPoint 가중치는 비상업용, ALIKED BSD-3, DISK·XFeat·LightGlue Apache-2.0.
전체 평가는 xfeat_lighterglue(Apache-2.0, GPU 167 MB):

| 그룹 | 없음 | SIFT 느슨 | SIFT 엄격 | 학습 느슨 | 학습 엄격 |
|---|---|---|---|---|---|
| 전방+top 회전 / 광축 | 0.035° / 7.6 mm | 0.015° / 2.9 mm | 0.018° / 5.8 mm | 0.017° / 4.6 mm | 0.021° / 5.7 mm |
| 측면 위치 / 광축 | 8.3 / 3.7 mm | 9.8 / 6.0 mm | 9.7 / 5.3 mm | 10.2 / 6.5 mm | 11.7 / 6.8 mm |
| 후방 위치 / 광축 | 31.5 / 15.6 mm | 17.9 / 10.3 mm | 26.4 / 15.3 mm | 16.2 / 10.9 mm | 28.7 / 14.9 mm |
| 트랙 재투영 | 0.84 px | 1.07 px | 0.86 px | 0.90 px | 0.87 px |
| 후방 연결 트랙 (L/R) | – | 451/301 | 32/14 | 52/23 | 4/2 |

잘못 합친 점은 대부분 걸러져 잔차가 1.07 → 0.90 px로 줄고 전방 회전·후방 이득은 유지되지만, 0.84–0.86 px 목표에는 못 미치고
측면은 개선되지 않으며 후방 연결은 여전히 적다(전방↔후방은 물체의 반대 면을 봄). 검증된 연결도 기하 잔차가 약 1 px 남아
남은 불일치는 모델(카메라별 시간 오프셋 등) 쪽으로 보인다. 그래서 기본은 계속 끔. 켤 때는 `matcher: xfeat_lighterglue`
(선택 패키지 `lightglue`, XFeat 저장소 경로 `NONTARGET_XFEAT_DIR`; GPU 있으면 사용, 없으면 CPU). 연결 계산 약 0.8시간(추정) + keys 풀이.

## 9. 선택 기능: 카메라별 시간 오프셋 (`rgb.solve.time`, 기본 끔) 과 남은 잔차의 원인

RGB 번들 조정에 카메라별 시간 오프셋 dt(와 행 판독 시간 rs)를 넣을 수 있다: 촬영 시각 = header + 노출/2 + dt_c
(+ rs_c·(v/H − 1/2)). 궤적은 고정이므로 게이지 없이 각 dt가 LiDAR 시계에 대해 관측된다(약한 사전 50 ms). 풀이 사이에
LO 자세 표를 새 시각에서 다시 계산하고(`solve.retime`), 풀이 안에서는 LO 각속도·속도로 1차 근사한다(해석 자코비안,
수치 미분과 2e-7 일치). 결과 파일에 `time_dt_s`/`time_rs_s`로 남고 절반·held-out 평가로 이어진다. 끄면 기존과 비트 단위로 같다.

2026-09-24 야간 bag, 25개 창(`tests/time_offset_eval_20260927.txt`): 14대 모두 스탬프가 같고(PTP, 노출 15.7 ms, 글로벌 셔터),
추정 dt = 공통 **+0.5 ms**, 카메라별 차이 ±0.3 ms, 절반 A−B 중앙값 0.3 ms(최대 0.7 ms) → 0과 구별되지 않음. 트랙 재투영
0.843 → 0.843 px, held-out·1σ·LiDAR 평면 거리 변화 없음, 검증 결과 대비 최대 0.25σ. 16 ms를 일부러 틀리게 주면 재투영이
1.13 px로 커지고 dt가 이를 되찾으므로(−16.8 ms) 민감도는 충분하다. **시간 오프셋 오차는 없다 → 기본은 끔**(비용 약 +5 %).

남은 약 0.85 px 잔차의 원인(같은 파일 4절, `tests/resid_*.py`):
- **KLT 특징점 표류·국소화 잡음(주원인)**: 잔차가 트랙 중심에서 멀어질수록 커지고(0.68 → 1.13 px, 깊이와 무관), 트랙을 따라
  매끄럽다(133 ms 지연 상관 0.7–0.8). 프레임 간 LK의 전·후방 폐합 오차: 전방 1 s 0.3 px, 2–4 s 0.4–0.5 px, 후방 수 px.
  코너 세기 하위 20 % 1.00 px ↔ 상위 20 % 0.67 px, 어두운 곳 0.89 ↔ 밝은 곳 0.73 px.
- **궤적(LO) 오차, 프레임별**: 동시에 찍힌 모든 카메라에 공통인 6자유도 자세 보정을 프레임마다 맞추면 0.843 → 0.761 px
  (무작위 묶음 0.841), 제곱합 기준 약 0.36 px. 대부분 롤(LiDAR x축) 약 0.045° rms, 주행 속도 3 m/s 이상에서 잔차가 커짐
  (0.66 → 0.94 px). IMU의 10 Hz 사이 회전과 양의 상관(0.2–0.3). (IMU의 x축 각속도 부호가 LiDAR와 반대임에 주의.)
- **렌즈 모델**: 영상 칸별 평균 잔차 0.02–0.04 px, 분산의 0.1 % → 충분. **시간 오프셋·롤링 셔터**: 없음.
- 교차 연결 트랙의 약 1 px 불일치도 같은 원인(각 트랙의 KLT 표류 + 시점에 따른 특징 중심 차이)이다. 개선하려면 키프레임
  템플릿 기준 추적(표류 제거)이나 트랙 시간 폭 제한, 또는 BA 안에서 프레임별(30 Hz) 궤적 보정/IMU 결합 LO가 필요하다.
  → 시험 결과는 10절: 트랙 시간 폭 제한(2 s)이 정확도를 올려 기본이 되었다.

## 10. KLT 트랙 시간 폭 제한 (`rgb.solve.seg_span_s: 2.0`, 기본 켬) 과 잔차 연구

9절의 원인 분석대로 남은 잔차의 주원인은 KLT 표류다. 긴 트랙 하나의 표류(전방 2–4 s에 0.3–0.5 px, 후방 수 px)는
랜드마크 하나가 흡수해야 하므로 백색 잡음이 아니라 창들의 움직임에 따라 정해지는 **편향**으로 외부 파라미터에 들어간다.
`seg_span_s`는 BA에 쓰는 트랙 선택(최소 길이, 트랙당 최대 40 관측) 뒤 각 트랙을 round(길이 / seg_span_s)개의 같은 시간
조각으로 잘라 각 조각을 별도 랜드마크로 둔다(`nontarget_cal/rgb/data.py`). 첫 번째 보드 없는 패스(설계 각도 시작)는 전체
트랙을 그대로 쓰고, 영점 패스 2·최종·절반 풀이에 적용된다. 검증의 held-out 재투영은 비교 기준이 바뀌지 않도록 전체 트랙,
연구 옵션 없이 계산한다. `null`이면 이전(검증된) 방식과 같다.

2026-09-24 야간 bag, 25개 창, 절반 비교 1σ 그룹 중앙값(`tests/resid_eval_20260927.txt`; 회전 ° / 위치 mm / 광축 mm):

| 그룹 | 기존 | 1 s | **2 s (기본)** | 3 s | 2 s + 프레임별 자세 보정 |
|---|---|---|---|---|---|
| 전방+top 회전/위치/광축 | 0.035 / 13.8 / 7.6 | 0.039 / 5.9 / 2.2 | **0.029 / 7.7 / 2.4** | 0.034 / 9.8 / 4.1 | 0.027 / 7.8 / 2.8 |
| 측면 회전/위치/광축 | 0.042 / 8.3 / 3.7 | 0.030 / 8.7 / 6.4 | **0.045 / 7.1 / 4.2** | 0.043 / 6.5 / 3.3 | 0.038 / 6.3 / 2.1 |
| 후방 회전/위치/광축 | 0.251 / 31.5 / 15.6 | 0.054 / 13.3 / 9.2 | **0.082 / 6.3 / 2.5** | 0.100 / 7.1 / 4.9 | 0.082 / 5.9 / 2.4 |
| 트랙 재투영 (px) | 0.843 | 0.574 | **0.671** | 0.750 | 0.592 |
| held-out (전체 트랙, px) | 0.849 | 0.849 | **0.848** | 0.848 | 0.848 |
| 랜드마크–LiDAR 평면 (후방) mm | 26.4 (35.1) | 21.6 (21.4) | **25.9 (24.0)** | 27.4 (25.6) | 25.9 (23.6) |
| 검증 결과 대비 최대 이동 | – | 1.79σ | **1.15σ** | 1.28σ | 1.14σ |

- 2 s: 후방 위치 1σ 31.5 → 6.3 mm, 광축 15.6 → 2.5 mm, 회전 0.25 → 0.08°, 전방+top 광축 7.6 → 2.4 mm. held-out은 그대로
  (전체 트랙을 설명하는 능력이 떨어지지 않음), 어떤 카메라도 검증 결과에서 1.15σ 이상 움직이지 않음(중앙값 2.7 mm;
  가장 큰 rear_left 22 mm는 기존 후방 1σ 31–39 mm 안). 측면(카메라 2대, 분할 1회)은 잡음 범위 안에서 변화 없음.
  잔차는 트랙 중심에서의 시간과 무관해짐(0.68 → 1.13 px 대신 0.62 → 0.69 px).
- 1 s는 전방에 가장 좋지만 기선이 짧아 측면 광축(6.4 mm)·후방(13 / 9 mm)이 나빠지고 결과가 더 움직인다. 3 s는 표류의
  절반이 남는다. 2 s가 모든 그룹에서 최선이거나 그에 가깝다.
- 비용: 랜드마크 0.49 M → 0.89 M, 최대 메모리 8.2 → 9.2 GB; 같은 부하의 2창 풀이 324 → 350 s(+8 %).
- 1σ는 변형마다 절반 분할 1회에서 나온 값(카메라별 표본 1개)이다. 다른 bag에서 다시 확인할 것.

함께 시험했지만 **기본 끔**으로 둔 연구 옵션:
- `rgb.solve.frame_corr`(`rgb/framecorr.py`): 동시에 찍힌 프레임마다 LiDAR 자세 6자유도 보정(사전 0.2° / 20 mm, 1 s
  고역 통과, 창별 평균 0). 잔차 0.843 → 0.764 px(2 s와 함께 0.671 → 0.592 px)이지만 1σ·held-out·평면 거리는 그대로 →
  궤적 오차는 외부 파라미터에서 평균되어 정확도를 제한하지 않는다. 2 s와 함께일 때 측면 광축 4.2 → 2.1 mm는 분할 1회의
  잡음 범위이고 단독으로는 일관되지 않아, 다른 bag에서 재확인 후보.
- `rgb.solve.obs_weight`(`rgb/obsweight.py`): 트랙 중심에서의 시간에 따른 경험적 가중치. 잔차 0.812 px, 1σ 변화 없음.
- `rgb.tracks.anchor`(`rgb/tracks_anchor.py`): 키프레임 템플릿 기준 재측정 KLT. 전·후방 폐합 오차가 오히려 커지고
  (front5 60–120 프레임 0.27 → 0.71 px, 야간 템플릿 외관 변화·어안 스케일 변화), 추적 CPU 약 3배, 잔차 0.795 → 0.776 px뿐.
