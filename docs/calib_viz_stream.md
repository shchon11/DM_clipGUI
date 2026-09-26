# 온라인 캘리브레이션 시각화 스트림 v1

뷰어는 `scripts/calib_viz/viewer.py`의 `CalibrationVizWidget`이며,
`CalibTab`의 기본 **3D 보정 상태** 화면에 연결되어 있다. 작업 선택 시 새 작업 설정은
접히고, **카메라별 결과 / 작업 로그** 보조 탭에서 상세 수치와 로그를 확인한다.

## 실제 작업과 데모의 구분

- 실제 작업: 기존 `online_calib.Progress`의 JSON 이벤트에서 단계, 구간 k/N, ETA(발행 시),
  경고를 읽는다. 카메라×구간 작업 수나 scan 수를 구간 수로 표시하지 않는다.
- `run_start`마다 이전 시도의 경고·오류·작업 실패·거절·종료 판정을 지운다. JSONL 읽기 위치는
  유지하므로 같은 작업 폴더에서 실패 후 이어서 실행해도 이전 오류를 현재 오류로 표시하지 않는다.
  구형 도구가 재개 시 생략하는 완료된 names/windows 캐시 단계는 유지한다.
- `work/viz`가 없으면 `job.py`가 도구의 설계 초기값 또는 `--init`의 실제 warm-start 값을
  표시한다. 중간 포즈와 1σ/재투영 오차는 만들지 않는다. 완료 시 결과 YAML과
  `summary.json`/`metrics.json`을 읽어 **최종값으로 바로 전환**한다.
- zero-shot RGB는 도구의 `nominal_cams(square=True)`와 동일하게 LiDAR 원점에서 시작한다.
  따라서 초기 RGB 프러스텀은 겹친다. 설계 도면의 장착 위치로 대체하지 않는다.
  초기 차량 축은 명목 장착 축, 최종 차량 축은 `rig.yaml`의 실측 축이다.
- 실행 기록의 도구 venv 설정을 우선 읽는다. 사용자 실행 파일의 설정을 확인할 수 없으면
  초기값을 미표시하고 이유를 알린다. 확인된 최종 출력은 여전히 표시한다.
- `work/viz/manifest.json`과 정상 스냅샷이 뒤늦게 생기면 해당 스트림을 읽는다.
  진행 상태는 작업 이벤트를 따르며, 완료된 결과는 오래된 중간 스트림보다 우선한다.
  실제 작업에 합성 리플레이 manifest/스냅샷을 연결하면 거부한다.
- 작업의 최신 `run_start.run_id`가 있으면 같은 ID의 스트림만 표시한다. `--no-viz` 재시도로
  이전 `viz/`가 남아 있어도 현재 중간값으로 표시하지 않고 초기값/진행 상태를 사용한다.
  run_id를 발행하지 않는 구형 도구는 기존 연결 동작을 유지한다.
- 실제 작업의 영상/지도는 발행된 스트림 자산만 사용한다. 새 도구는 LO와 solver의 실제 상태를
  발행한다. 구형 도구가 발행하지 않는 영상/지도는 대기 화면으로 남는다.
- 데모만 실측 최종값을 향하는 **합성 중간 포즈·지표**를 생성한다. 화면에 항상
  `실제 데이터 · 합성 수렴 리플레이`를 표시한다.

## 실행

기존 선택 의존성은 `scripts/calib_viz/requirements.txt`를 따른다. 새 의존성은 없다.
ROS 설치 시 CMake가 `calib_viz/`를 GUI 모듈과 함께 설치한다.

```bash
# 센서/레코더를 실행하지 않고 실제 완료 결과를 온라인 보정 탭에서 보기
python3 scripts/calib_tab_demo.py --workdir /hdd/DM_calib/nt_regress/full/work

# 데모 전용 합성 수렴 (기본: 모든 사용 가능한 구간 순회)
python3 scripts/calib_viz_demo.py --replay /hdd/DM_calib/nt_regress/full/work --speed 60 --max-fps 20 --max-points 30000

# solver가 발행한 스트림만 보기
python3 scripts/calib_viz_demo.py --stream /path/to/run/work/viz

# 실제 도구: 기본은 <workdir>/viz, config의 viz.enabled=false로 완전히 비활성화
/hdd/DM_calib/nontarget_cal/.venv/bin/nontarget_cal run ... --viz-dir /path/to/run/work/viz
```

`--out`으로 별도 결과 폴더를 지정한다. 리플레이는 기본 `/tmp/calib-viz-*`에만 쓰고
종료 시 정리한다. 원본은 읽기 전용이다. `--viz-dir`은 출력 보존용이다.
`--screenshots docs/img --fractions 0.06,0.55,1`로 캡처하고, 특정 센서/구간 캡처에는
`--camera thermal_right --window S01`을 더한다. `--window`는 스크린샷 모드에만 적용된다.

직접 임베드할 때는 `set_job(job, progress)` / `update_job(job, progress)` 또는
`set_stream(workdir / "viz")`를 사용한다. 탭/호스트 종료 시 `shutdown()`을 호출한다.
현재 `clip_gui` 종료 경로에서도 뷰어 worker/timer를 정리한다.

## 파일과 원자적 발행

```text
work/viz/
  manifest.json              # 원자적 교체, 256 KiB 이하
  events.jsonl               # 한 줄 = 완전한 상태 스냅샷, 잠금으로 직렬화한 발행
  control.json               # 선택 카메라 힌트, 뷰어가 원자적 교체 (선택 사항)
  assets/
    map_000123.npz
    camera_front5_000123.jpg
    camera_front5_000123.npz
```

1. 작은 배열/이미지를 같은 디렉터리의 고유한 `.tmp` 이름으로 쓴다.
2. close 후 `os.replace(tmp, final)`로 게시한다. 게시된 자산 이름은 재사용하지 않는다.
3. 자산이 모두 게시된 뒤 UTF-8 JSON 한 줄과 마지막 `\n`을 `events.jsonl`에 append한다.
4. 부분 줄은 다음 poll까지 보류한다. 잘못된 줄은 건너뛰고 다음 정상 스냅샷을 처리한다.
   중복/역순 seq는 같은 run_id/파일 세대 내에서 무시한다. 파일 교체/축소(다시 커진 copy-truncate
   포함) 시 reader가 재시작한다. append 방식의 재시작도 run_id가 바뀌면 seq=0부터 받는다.
5. 새 실행은 새 `run_id`, 새 자산 이름, 원자적으로 교체한 새 이벤트 파일을 사용한다.
   로그 회전은 가장 최신 **완전 스냅샷**을 새 파일에 먼저 넣어 교체한다.
   현재 manifest와 다른 run_id의 이벤트는 무시한다. 작업을 재개할 때 실패한 시도의
   포즈·지표 이력·영상·지도를 새 시도의 상태로 이어 붙이지 않는다.

JSON의 NaN/Infinity와 NPZ의 pickle/object 배열은 금지한다. 모든 자산 경로는 viz 아래의 상대 경로이며
절대 경로, `..` 또는 symlink를 통한 디렉터리 탈출은 거절한다. JPG/PNG는 RGB 미리보기로 읽는다.

## manifest.json

```json
{
  "schema": "calib-viz/1",
  "run_id": "20260926-run-001",
  "mode": "live",
  "parent_frame": "os_lidar",
  "display_frame": "vehicle",
  "R_lidar_V": [[-1,0,0],[0,-1,0],[0,0,1]],
  "map_frame": "vehicle_aligned_window:S01",
  "cameras": {
    "camera_front5": {
      "sensor": "rgb", "model": "equidistant", "width": 1920, "height": 1200,
      "K": [[800,0,960],[0,800,600],[0,0,1]], "D": [0,0,0,0]
    }
  }
}
```

위 수치는 스키마 설명용이며 실제 캘리브레이션 값이 아니다. 실제 리플레이는 `rig.yaml`의 측정
`R_lidar_V`와 카메라별 intrinsic/extrinsic YAML을 그대로 읽는다.
필수 RGB 이름은 `camera_front1..9`, `camera_top`, `camera_side_left/right`,
`camera_rear_left/right`, 열화상 이름은 `thermal_left/right`이다. 부분 장치 구성도 표시할 수 있다.
manifest의 `cameras`는 이번 실행에 참여하는 카메라만 포함하는 권위 있는 목록이다.
리그 부제의 RGB/열화상/전체 카메라 수와 선택 목록은 이 구성에서 계산하며,
목록에 없는 카메라 선택은 거부한다. manifest 도착 전 선택은 허용하되 도착 후 유효한 센서로 맞춘다.
`T_cam_lidar_final`, `source_*`, `provenance`, `rate_limits`, `preview_cameras`는 리플레이 부가 정보다.
실시간 manifest에 정답 포즈는 필요 없다.

## events.jsonl: 완전 스냅샷

각 스냅샷은 이전 이벤트를 버려도 화면을 복원할 수 있어야 한다. 예:

```json
{
  "schema": "calib-viz/1", "run_id": "20260926-run-001", "seq": 123, "t": 1790400000.5,
  "source_time_s": 200.5, "progress": 0.45,
  "stage": "rgb_ba", "window": 12, "total_windows": 25, "eta_s": 160.0,
  "bag_id": "rec_20260924_204902", "window_id": "S12",
  "solver_pass": "B2", "iteration": 3, "total_iterations": 12, "purpose": "calibration",
  "cameras": {
    "camera_front5": {
      "T_cam_lidar": [[0,1,0,0],[0,0,-1,-0.35],[-1,0,0,-0.77],[0,0,0,1]],
      "sigma_rot_deg": 0.12, "sigma_pos_mm": 21.4, "reprojection_px": 1.7,
      "K": [[801,0,960],[0,801,600],[0,0,1]], "D": [0.001,0,0,0], "model": "equidistant",
      "cost": 142.5, "cost_source": "huber_reprojection_only",
      "state": "converging", "gate_pass": null, "validation_vote": null,
      "informational_checks": [], "metric_source": "solver"
    }
  },
  "gate": {"status": "pending", "pass": null, "source": "solver.validation", "warning": ""},
  "map_frame": "vehicle_aligned_window:S01",
  "assets": {
    "map": "assets/map_000123.npz",
    "matching": {
      "camera_front5": {
        "image": "assets/camera_front5_000123.jpg",
        "points": "assets/camera_front5_000123.npz",
        "width": 960, "height": 600,
        "source_stamp_ns": 1790400000000000000,
        "capture_stamp_ns": 1790400000000000000,
        "lidar_stamp_ns": 1790400000000000000,
        "source_window": "S01", "points_frame": "os_lidar_at_image_capture"
      }
    }
  }
}
```

| 필드 | 의미/단위 |
|---|---|
| `seq` | 현재 파일 세대에서 단조 증가하는 정수 |
| `run_id` | 실행 시도 ID. 재시작/이어서 실행은 새 ID. 이전 v1 이벤트는 생략 가능 |
| `t`, `source_time_s` | UNIX 초 / 실행 시작 이후 원본 시간 초. 애니메이션은 뷰어 monotonic clock 사용 |
| `progress` | 전체 진행률 0..1. 표시용이며 게이트 판정 근거가 아님 |
| `stage` | `extract`, `lidar_odometry`, `rgb_ba`, `thermal`, `validation` |
| `window`, `total_windows` | 완료/현재 처리 창 수와 전체 창 수. 공동 BA에서는 처리된 창 수 유지 |
| `bag_id`, `window_id`, `windows` | 실제 bag/대표 구간 ID와 solve에 참여한 구간 목록. 서로 다른 bag의 S01은 별개 |
| `windows` (camera) | 해당 센서 solve에 실제 사용한 구간 목록. 전역/선택 센서 문맥에 구간 수와 최대 3개 ID를 표시 |
| `solver_pass`, `solve_name`, `purpose` | A/B1/B2/B3 패스, solve 출력 식별자, 용도. 검증용 부분 solve를 최종 해로 오인하지 않게 표시 |
| `solver_pass`, `solve_name`, `purpose`, `iteration`, `total_iterations` (camera) | 해당 카메라 추정치를 만든 solve의 문맥. 선택 센서 지표 위에 표시. 병렬 작업의 최상위 문맥과 다를 수 있음. 최종 판정은 `solver_pass=final`, 반복 필드는 null |
| `iteration`, `total_iterations`, `accepted` | LM 반복 인덱스(0=초기, 이후 1부터)·반복 상한·채택 상태. 최종 검증 등 반복 밖에서는 null 허용. 시간 제한으로 일부 중간 반복은 생략됨 |
| `objective_cost` | 해당 solve의 전체 목적함수. 카메라별 픽셀 재투영 비용과 구분 |
| `eta_s` | 화면 기준 남은 초, 불명확하면 null. 리플레이는 압축된 시간 |
| `T_cam_lidar` | 4×4 강체 변환, 이동 성분 **m**, 행 우선 중첩 배열 |
| `sigma_rot_deg`, `sigma_pos_mm` | 1σ 회전 deg / 위치 mm. 없으면 null; 계산 방식은 `metric_source`로 명시 |
| `reprojection_px` | **원본 영상 해상도 기준** 재투영 오차 px. 없으면 null |
| `reprojection_median_px` | 실제 관측 재투영 오차 중앙값. live `reprojection_px`는 RMS |
| `K`, `D`, `model`, `width`, `height` (camera) | 선택적인 현재 intrinsic override. 없으면 manifest 초기값. 원본 영상 해상도 |
| `cost`, `cost_source` (camera) | 현재 Huber 재투영 비용. `huber_reprojection_only`는 depth 페널티·tie·prior 제외 |
| `time_offset_s` (camera) | 열화상 구간별 시간 offset의 현재 평균(초, 부호 유지) |
| `time_offsets_s`, `time_offset_by_window_s`, `rs_s`, `row_readout_s` (camera) | 구간별 시간 offset 목록/매핑과 현재 rolling shutter 행 readout 시간(초). `row_readout_s`는 `rs_s`의 명시적 이름이며 선택 센서에 µs로 표시 |
| `state` | `pending`, `converging`, `converged`, `failed` |
| `gate_pass` | 도구의 실제 카메라 게이트 bool/null. true=청록, false=빨강, null=회색 미판정 |
| `validation_vote` | 별도 에지/투표 진단 bool/null. 이 값만으로 실패 색상을 정하지 않음 |
| `informational_checks` | 게이트에 포함되지 않는 참고 검사 설명 목록. 센서 행의 옅은 ⓘ와 툴팁 |
| `gate_reasons` | 실제 카메라 게이트 실패 이유 목록 |
| `gate` | solver의 종합 판정 그대로. 전체 배치 규칙 실패를 모든 카메라 실패로 전파하지 않음 |

개별 solve 종료 시 `solver_pass=result`, `iteration/total_iterations/accepted=null`,
`state=converged`, `metric_source=solver.result_residuals`로 정확한 계산 최종값을 먼저 발행한다.
이는 아직 게이트 통과를 뜻하지 않는다. 전역 검증 완료의 `solver_pass=final` 및 실제 gate가 뒤따른다.
종료 상태는 로컬 샘플링 간격에 관계없이 enqueue하지만 writer의 전체 4 Hz 제한은 유지한다.

프러스텀은 현재 K에서 계산한 짧은 광학 축/사각 피라미드이며, D까지 적용한 광각 렌즈의 정확한
가시체적 경계는 아니다. 영상 투영은 매 스냅샷의 실제 K/D/포즈를 사용한다.
회전/이동을 강체 보간하고 최근 14번의 표시 포즈 잔상을 유지한다. 센서별 재투영 이력은 80개,
선택 센서의 세 지표 이력은 각 100개로 제한한다. 선택한 카메라 영상이 없으면 대기 상태를 표시한다.

## 좌표계: 반드시 지킬 계약

`x_cam = R_cam_lidar @ x_lidar + t_cam_lidar`이며 OpenCV 광학 축은 x 오른쪽, y 아래, z 앞이다.
Ouster `os_lidar`의 +x는 차량 **뒤**다. `R_lidar_V`는 **차량에서 LiDAR로** 회전시키므로:

```text
R_V_cam = R_lidar_V.T @ R_cam_lidar.T
C_V     = -R_V_cam @ t_cam_lidar
x_V     = R_lidar_V.T @ x_lidar
```

차량 좌표 V는 x 전방, y 좌측, z 위, 원점 Ouster다. 실제 데이터의 앞 카메라 x는 약 +0.76m,
측면 좌측 y는 약 +0.54m다. 상단 카메라 z≈−0.18m는 앞줄 z≈−0.37m보다 높지만 Ouster 아래다.
차량 외형은 개략적인 크기 기준이며 측정 CAD가 아니다.

지도 NPZ의 `points`와 `trajectory`는 같은 **map_frame** 안의 m 단위 float32 `(N,3)`이다.
지도 뷰와 차량 리그 뷰는 별도의 좌표계를 표시한다. 각 창에서 odometry가 재시작되면
`vehicle_aligned_window:<id>`를 바꿔야 한다. 독립 창의 점군을 그냥 이어 붙이면 안 된다.
전역 지도 훅에서는 창 사이 등록을 완료한 뒤, 명시한 하나의 고정 map_frame으로 발행한다.
실제 LO producer는 회전 변환을 추가하지 않은 원본 LiDAR odometry 축의
`lidar_window:<bag_id>/<window_id>`를 쓴다. 지도와 궤적은 같은 창의 동일 좌표이며,
리그의 차량 좌표와 별개다. 다른 bag 또는 창의 map_frame으로 이동하면 이전 자산을
실패 대체용으로 유지하지 않는다. 공동 solve의 영상이 지도와 다른 창에서 왔으면
영상의 `source_window`를 별도로 표시한다.
동일 입력의 작업 재개는 manifest의 선택적 `input_identity`가 일치하면 안전한 이전 지도 자산만
남길 수 있다. 이때 `map_source=previous-attempt-cache`, `previous_map_run_id`,
`map_window/map_source_window`와 원래 `map_frame`을 상태에 기록하고 카메라·게이트·진행률은 초기화한다.
스트림이 없는 구형 LO 캐시에서 실제 deskew된 sweep 3개로 만든 지도는
`map_source=cached-lo-sample`, 새 계산으로 갱신한 지도는 `map_source=solver`다.

## 영상·투영 자산

카메라 NPZ:

- `points_lidar`: `(N,3)` float32, **해당 이미지 노출 시점 LiDAR 좌표**의 점. deskew와 이동 보정은 producer 책임.
- `tracks_uv`, `tracks_prev_uv`: 선택 `(M,2)` float32, 동일 track ID의 실제 관측 영상 좌표.
  미리보기 해상도 좌표이며, 대응이 없으면 빈 배열. 합성 정답 매칭을 만들지 않는다.
- `mask`: 선택 `(height,width)` uint8, **>127이 유효**. 미리보기와 같은 크기.

K와 D는 원본 해상도다. 뷰어는 RGB의 Kannala–Brandt equidistant 4계수 투영을 직접 계산하고
미리보기 크기로 uv를 비례 축소한다. **현재 결과의 열화상은 실제 YAML에서 `plumb_bob` 5계수**이므로
그 모델로 투영한다. 실제 소스와 다른 모델을 일괄 적용하지 않는다.
점 색상은 광학 z 깊이 0..35m를 표시한다. 초기 추정 비교의 선은 **동일 LiDAR 점의 초기→현재 투영 이동**이며
관측 특징과의 정답 대응선이 아니다. 관측 특징은 별도의 녹색 사각형/궤적으로 표시한다.
영상 하단의 입력 헤더차는 이미지 원본 헤더와 LiDAR sweep 시작 시각의 차이다.
노출 offset/이동 보정 후의 잔차나 센서 동기화 품질을 뜻하지 않는다.
실제 RGB 미리보기는 카메라 원본 header를 `source_stamp_ns`, 유효 노출 중간시각+현재 offset을
`capture_stamp_ns`에 기록한다. 열화상은 clock smoothing과 현재 offset을 사용하며,
현재 포즈/렌즈/행 readout으로 3회 고정점 행 보정을 적용한다. 이 경우
`points_frame=os_lidar_at_image_row_capture`, `row_readout_s`, `row_projection_iterations=3`을 명시한다.
포즈/시간 offset이 갱신된 뒤 다음 이미지까지 자산 지연이 있을 수 있다.
영상 샘플링·발행은 최대 1Hz이며, I/O 부하에 따라 지연은 더 길어질 수 있다.

### 선택 카메라 제어 (선택 사항)

```json
{"schema":"calib-viz-control/1","run_id":"20260926-run-001","camera":"camera_front5","enabled":true,"updated_at":1790400000.5}
```

뷰어 I/O 스레드는 live manifest의 카메라 선택이 바뀔 때 `control.json`을 원자적으로 교체하며
1초에 1회 이하로 제한한다. 읽기 전용 폴더나 구형 도구에서도 표시는 계속한다.
producer는 현재 run_id와 일치하는 선택을 사용하고, 선택이 없으면 카메라를 순환한다.
이 파일은 미리보기 선택 힌트이며 계산 파라미터/solver 동작은 바꾸지 않는다. 데모는 쓰지 않는다.

## 발행 훅과 성능 계약

`nontarget_cal`은 다음 실제 계산 경계에서 **비동기 best-effort**로 발행한다.

1. 준비 단계: 카메라 모델/K/D/원본 크기와 `R_lidar_V`, run_id를 manifest로 기록.
2. RGB/thermal A 및 B 반복의 초기값/채택된 LM 상태: 현재 extrinsic/intrinsic, 실제 잔차/비용,
   단계/구간/패스/반복 스냅샷. 중간 covariance를 계산하지 않았다면 1σ는 빈 값이다.
3. LiDAR odometry/refinement 콜백: 이미 계산한 지도/궤적에서 작은 샘플 복사. 영상 선택 콜백: 축소 JPG,
   노출 시점으로 보정된 점, 실제 관측 tracks/mask를 내보냄.
4. 실제 검증 완료: 종합 게이트 및 카메라별 판정을 그대로 기록. 검증 시작 시 최종 판정을 미리 보내지 않음.

발행 측 전역 상한: 스냅샷 4Hz, 지도 1Hz/70k 점, 영상 1Hz/긴 변960px/카메라당7k 점.
설정으로 더 낮출 수 있으며 기본 지도30k·영상5k 점이다. 영상은 선택 센서 한 대, 선택이 없으면 순환한다.
JSON은256KiB, 압축/해제 NPZ는32MiB, 이미지3840×2160을 넘지 않는다.
임베드 뷰어는 최대20FPS/지도30k점, 영상 투영5k/특징100개로 제한한다. 숨겨진 뷰어는 렌더와 I/O를 쉬며, 선택한 카메라 영상만 읽는다. I/O worker 하나, 최신 mailbox 하나, 자산 캐시5개를 사용한다.
producer는 프로세스마다 크기1 mailbox와 I/O 스레드 하나를 사용하고, 밀리면 최신 상태로 합친다.
solver iteration은 이미지 인코딩/파일 flush를 기다리지 않는다. 여러 프로세스의 쓰기는 파일 잠금으로 직렬화한다.
영상 원본 미리보기 캐시는 최대16개 LRU이며, 점군 전체나 전체 KLT 배열을 보존하지 않는다.
I/O 실패 시 시각화만 건너뛰고 solver를 계속 진행한다. 뷰어는 producer에 ACK를 보내지 않는다.
공유 CPU/디스크/GPU의 간접 자원 경쟁을 완전히 없애지는 못한다.

live producer는 최신120초 정도의 자산을 보존한 뒤 지우고, 로그는16MiB 정도에서 회전시킨다.
삭제된 과거 자산을 만난 뷰어는 마지막 정상 영상을 유지하며 다음 스냅샷으로 넘어간다.
상태 읽기는 단일 I/O 스레드, 표시 전달은 크기1 mailbox다. 화면 갱신은 기본40 FPS이며 렌더링이
느리면15 FPS까지 단계적으로 낮추고 지도 점 상한도 낮춘다. UI 중지 버튼은 화면만 멈추고 solver는 건드리지 않는다.

## 리플레이의 진실 범위

리플레이는 실제 최종16센서 포즈/지표, `events.jsonl`의 단계 시작·검증 완료·실행 시간,
모든 사용 가능한 구간의 LiDAR/odometry·RGB·16bit 열화상·특징 tracks·마스크를 사용한다.
7시간52분33초의 원본 시간은60배속에서 약7분52초다. 재시작/긴 대기도 원본 타이밍에 포함된다.
중간 포즈와 불확실도는 설계 초기 오차(약7–10°/수cm)를 최종값으로 감쇠시킨 **합성 과정**이다.
실제 solver의 중간 해나 covariance를 복원한 것이 아니다.

지도는 **25개 구간 각각에서 최대12개 LiDAR sweep을 뽑아 순차 표시**한다.
창별 odometry 원점이 달라 전역으로 합치지 않는다. 현재 구간 ID와 `map_frame`을 명시하고,
다른 창의 지도/영상을 이어 표시하지 않는다. 원시 NPY는 mmap으로 샘플링하며,
거대한 `lo/map_*.npz`는 읽지 않는다. 활성 구간의 궤적은 최대4096포즈,
특징 NPZ는 해제 크기32MiB 이하만 읽고 샘플 관측만 보존한다.

`prepare()`는 메타데이터만 읽고, 현재 재생 구간·시점의 프레임을 요청할 때 자산을 생성한다.
디스크에는 최근3세트만 남기고 이벤트는 실행당4096개로 제한한다. 뷰어는 RGB14대+열화상2대
모두 선택 가능하며 선택한 영상만 디코딩한다. 해당 구간에 영상이 없으면 대기 화면이다.
매우 빠른 배속에서도 모든 구간을 최소 한 번 발행하고 발행 간격은 최소250ms다.
따라서 I/O나 발행 상한 때문에 실제 재생시간은 요청한 배속보다 길어질 수 있다.
야간 RGB는 gamma0.55, 열화상은2–98 percentile+inferno 표시 변환을 적용한다.
열화상은 측정 time offset과 중앙 행 시점을 사용하되 clock smoothing/행별 rolling shutter 보정은 생략했다.
RGB/열화상 특징과 LiDAR 점은 각각 실제 데이터지만 **검증된 점별 영상↔LiDAR 대응 집합은 아니다**.
카메라 색상은 명시된 카메라 게이트가 있으면 그대로, 기존 결과는 도구의 카메라별 실패 목록과
검증된 지표를 사용한다. 게이트 기록이 없는 이전 형식만 도구와 같은 임계값으로 복원한다.
RGB 야간 vote는 참고용 ⓘ다. 종합 게이트 통과를 빨간 카메라로 뒤집지 않는다.

## 검증·스크린샷

```bash
QT_QPA_PLATFORM=offscreen OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q test tests
xvfb-run -a -s '-screen 0 1800x1200x24' \
  env LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=2 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=xcb \
  python3 scripts/calib_viz_demo.py --replay /hdd/DM_calib/nt_regress/full/work \
  --screenshots docs/img --max-fps 20 --max-points 30000
```

Qt `offscreen` 플랫폼에서 OpenGL이 안 되면 위 Xvfb/GLX 경로를 사용한다. 현재 머신에서는 Xvfb 렌더링이
확인됐으나 NVIDIA 드라이버 접근이 안 돼 RTX3080Ti 하드웨어 성능은 검증하지 않았다.
`calib_viz_capture.json`의 FPS는 Qt 타이머 횟수가 아닌 rig OpenGL `frameSwapped` 횟수다.

사용 API 참고: [PyQtGraph GLViewWidget](https://pyqtgraph.readthedocs.io/en/pyqtgraph-0.13.3/api_reference/3dgraphics/glviewwidget.html).
