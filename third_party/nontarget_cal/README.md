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

ROS는 필요 없다(bag은 `rosbags`로 직접 읽는다). GPU도 필요 없다(CPU만으로 돈다; 병목은 CPU).

```bash
cd /path/to/nontarget_cal
python3 -m venv --system-site-packages .venv      # 시스템 numpy/opencv가 있으면 재사용
.venv/bin/pip install -U "pip>=24" "setuptools>=64,<80"
.venv/bin/pip install -r requirements.txt         # torch는 CPU판으로 충분: --index-url https://download.pytorch.org/whl/cpu
.venv/bin/pip install --no-deps -e .
.venv/bin/nontarget_cal --help
```

- 검증한 버전: numpy 2.2.6, scipy 1.15.3, opencv 4.13, torch 2.10, rosbags 0.11.0, **kiss-icp 1.3.0(고정)**.
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

## 3. 파이프라인 (단계별로 캐시됨)

| 단계 | 하는 일 |
|---|---|
| names | 토픽 이름 → 표준 이름(camera_front1..9, top, side_*, rear_*, thermal_left/right) + 기하 검증 |
| preflight | 움직인 시간, 누적 회전·회전 수, 속도, Ouster timestamp_mode(없으면 헤더-녹화시각 차), 카메라 간 시각 차(PTP), 노출, 열화상 fps·중복, 근거리 구조물, 시간·디스크 추정, 여유 공간 |
| windows / extract | 움직인 부분을 최대 40 s 창으로 나누고(회전 많은 창 우선), 창마다 한 번의 범위 질의로 RGB JPEG(원본 그대로), LiDAR 스윕(점별 시각 포함), 열화상 16비트 PNG, 정차 순간 1개 추출. bag 전체 INS는 작은 토픽만 한 번 훑어서 저장 |
| lo | 창마다 KISS-ICP → 연속시간 정밀화(5회) → 5 cm 지도 캐시 → 품질 검사(1 s 간격 스윕 불일치, 지도 두께). 불량 창은 제외 |
| rgb | 카메라·창별 KLT 트랙 → (zeroshot) 회전 많은 5개 창으로 설계 각도에서 2단계 풀이 → bag별 풀이·비교 → 전체 풀이 |
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

## 6. 전형적 시간·디스크·메모리 (16코어 개발 박스, 작업 폴더 = HDD)

`check`의 `estimate`가 실행 전에 시간·디스크를 계산해 알려 주고, 여유 공간이 모자라면 시작하지 않는다.
단계별 측정값은 결과 폴더 `report.md`의 "단계별 시간·디스크" 표와 `작업폴더/events.jsonl`에 있다.

| | 2분 40초 주행(40 s 창 4개, RGB 14대 + 열화상 2대, zeroshot) | 15분 주행(창 25개) |
|---|---|---|
| 전체 시간 | **약 46분** (최적화 전 약 1시간 55분) | 약 3–3.5시간(단계별 측정 합: 추출 27 + LO 75 + 추적 88 + RGB 40 ∥ 열화상 52분 + 검증) |
| 작업 폴더 디스크 | 20 GB (40 s 창당 약 4.3 GB RGB + 0.8 GB 열화상) | 약 110 GB |
| 메모리 | 최대 약 10 GB | 최대 약 25 GB (RGB 전체 풀이 + 절반 풀이 + 열화상이 동시에 돌 때) |

- 병목은 CPU다. 가장 큰 단일 비용은 KLT 추적(카메라·창 하나에 CPU 약 55초, 14대 × 창 수)이다.
- 차량 PC 예상: 같은 16코어급 CPU + NVMe SSD면 위와 같거나 더 빠르다(여기서는 작업 폴더가 HDD라 추적이 디스크를
  기다린다). 8코어면 약 1.8배. GPU(RTX 3080 Ti)는 쓰지 않는다: 번들 조정을 GPU로 옮겨 측정했더니 이득이 없었다
  (float64가 소비자 GPU에서 1/64 속도이고 풀이가 지연시간 위주). `resources.ba_device: auto`로 켤 수는 있다.
- 메모리가 32 GB보다 작으면 `resources.max_procs: 3`, `resources.tie_threads: 1`.

속도 관련 설정(`config/default.yaml`):

| 키 | 기본 | 뜻 |
|---|---|---|
| `resources.max_procs` | 4 | 동시에 도는 작업 프로세스 수(이 공용 박스 제한). 차량 전용 PC면 코어 수/4까지 올려도 된다 |
| `resources.lo_threads` | 3 | LiDAR 오도메트리 정밀화 한 개 안의 스레드 수(결과는 스레드 수와 무관하게 동일) |
| `resources.tie_threads` | 3 | LiDAR 평면 연결을 창별로 병렬 처리(결과 동일) |
| `resources.ba_device` | cpu | `auto`/`cuda`면 번들 조정을 GPU에서(결과 차이 1e-11 mm, 이 박스에서 이득 없음) |
| `rgb.solve.reuse_pass2_as_final` | false | 창이 5개 이하일 때 세 번째 풀이를 생략(약 10분 절약, 카메라가 전방 0.1–0.6 mm, 후방 3–6 mm 달라짐). **정확도와 바꾸는 옵션이라 기본은 끔** |

## 7. 회귀 테스트

- `tests/test_equivalence.py`: 원래 스크립트(lidar_odo, online_calib)와 이 패키지 함수를 같은 입력에 돌려
  비교(LO 정밀화, RGB BA, KLT 트랙). 결과: 세 가지 모두 **비트 단위로 동일**(속도 최적화 뒤 LO 정밀화는 1e-10 mm 차이).
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
