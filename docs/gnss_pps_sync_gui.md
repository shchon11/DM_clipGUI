# GNSS PPS 동기화 — GUI 대응 명세

> FLIR 리그가 "PC가 PTP grandmaster + PTP scheduled action 트리거" 에서
> "GNSS가 grandmaster + PPS가 GPIO 직결 트리거" 로 바뀐다. 그에 맞춰 GUI가
> 무엇을 노출하고 무엇을 고쳐야 하는지에 대한 명세.
>
> 짝 문서: `FLIR_control/docs/gnss_pps_sync.md` (카메라 쪽 전환·검증 절차).
> 그쪽의 T0~T6 단계 번호를 여기서 그대로 참조한다.
>
> 작성: 2026-09-21. 하드웨어 배선 완료 시점 기준. 실측 미검증.

## 0. 원칙

**모든 설정은 사람이 GUI에서 보고 바꿀 수 있어야 한다.** 이 리포의 구조상 그
수단은 대부분 `config/sensors.yaml` 에 필드를 올리는 것이다 — 코드를 고치는 게
아니라. `sensors.yaml` 머리말이 이미 그렇게 선언하고 있다:

> 각 센서군(group)은 ... `fields` : 설정 탭 맨 위 '주요 설정' 에 한국어 이름으로 올릴 키

그리고 `전체 설정` 은 원본 params YAML의 모든 키를 자동으로 읽어 보여주므로
(`scripts/param_doc.py`), **새로 생기는 파라미터는 아무것도 안 해도 '전체 설정'
에는 뜬다.** `fields` 에 올리는 기준은 "자주 바꾸고, 잘못 바꾸면 위험해서 설명이
필요한 것" 이다. 아래 §4의 선별은 그 기준을 따른다.

예외적으로 코드를 고쳐야 하는 것은 §2 (경로), §3 (역할 모델), §6 (검증 패널)
세 가지다.

## 1. 요약 — 무엇이 바뀌나

| 층위 | 기존 | 새 구성 |
|------|------|---------|
| 트리거 | PTP scheduled action (카메라 1대가 sender) | 외부 PPS → GPIO (8대 전부 slave, master 없음) |
| 시각 | PC가 GM, 카메라가 slave | GNSS가 GM, PC·카메라 모두 slave |
| GUI 역할 칸 | "PTP 동기" (sender/receiver/none) | 트리거 소스 모델로 재정의 (§3) |

## 2. 블로커 ① — `sensors.yaml` 의 FLIR 경로가 존재하지 않는다

`config/sensors.yaml` 의 FLIR 그룹은 전부 `~/FLIR_control` 을 가리킨다:

```yaml
workspaces:
  - "~/FLIR_control/scripts/setup_flir_env.bash"
  - "~/FLIR_control/install/setup.bash"
workdir: "~/FLIR_control"
inventory: "~/FLIR_control/src/flir_spinnaker_camera/config/multicam_cameras.yaml"
params:    "~/FLIR_control/src/flir_spinnaker_camera/config/flir_camera.yaml"
```

**그런데 이 PC에 `~/FLIR_control` 은 없다.** 실제 워크스페이스는
`~/projects/DM/FLIR_control_master` 다. `sensors.yaml` 머리말대로 GUI는 죽지
않고 FLIR 그룹을 "설정 없음" 으로 표시하며 기동 버튼을 잠근다 — 즉 **오늘 배선을
끝내도 GUI에서 카메라를 못 띄운다.**

`~/projects/DM/FLIR_control` (master 아님)도 있지만 이쪽을 쓰면 안 된다.
열화상·라이다 params가 없다:

| 파일 | `FLIR_control` | `FLIR_control_master` |
|------|----------------|------------------------|
| `flir_camera.yaml` | O | O |
| `multicam_cameras.yaml` | O | O |
| `multicam_thermal_cameras.yaml` | **X** | O |
| `thermal_camera.yaml` | **X** | O |
| `lidar_driver_params.yaml` | **X** | O |

**조치 (택1):**

- (A) `ln -s ~/projects/DM/FLIR_control_master ~/FLIR_control` — `sensors.yaml`
  무수정. 가장 빠르고, 여러 체크아웃 사이를 옮겨 다니기도 쉽다.
- (B) `sensors.yaml` 의 경로 7곳(FLIR 그룹 + ouster `params`)을
  `~/projects/DM/FLIR_control_master` 로 변경.

(A)를 권한다. 단 심볼릭 링크는 GUI에서 안 보이는 상태라, 어느 체크아웃을 쓰는지
**GUI 상단이나 설정 탭에 실제 해석 경로를 표시**하는 편이 안전하다 (§6에 포함).

## 3. 블로커 ② — `_fix_sync_roles` 가 GPIO slave 를 전부 해제한다

[`scripts/sensor_config.py:532-538`](../scripts/sensor_config.py#L532-L538):

```python
# GPIO 트리거는 배선이 필요한 역할이라 master 로 올리지 않는다 — slave 를 자유 실행으로.
slaves = [e for e in entries if _role(e, "hardware_trigger_role") == "slave"]
if slaves and not any(_role(e, "hardware_trigger_role") == "master" for e in entries):
    for e in slaves:
        e["hardware_trigger_role"] = "none"
```

기존 모델에서는 옳은 방어였다 — 카메라 master가 안 보이면 slave는 오지 않을
트리거를 기다리니까. **그런데 새 구성은 master가 영원히 없다.** 트리거 발생원이
카메라가 아니라 외부 GNSS 박스다. 이 코드가 그대로면 기동할 때마다 8대 전부
`none` 으로 되돌아가 자유 실행하고, 노트에는 "GPIO 트리거 master 가 안 보여 …
자유 실행으로 띄웁니다" 가 찍힌다. **동기는 안 맞는데 에러는 안 나는 상태** 라
제일 까다로운 실패 모드다.

**조치:** 트리거 발생원이 외부임을 선언할 수 있어야 한다. `sensors.yaml` 의
subset에 플래그를 추가하고 `_fix_sync_roles` 가 그걸 존중하게 한다.

```yaml
# sensors.yaml, visible subset
external_trigger: true     # PPS 등 외부 발생원. master 없이 slave 만 있어도 정상
```

```python
# sensor_config.py
if slaves and not sub.get("external_trigger") and not any(... == "master" ...):
    ...  # 기존 동작 유지
```

`external_trigger: true` 일 때는 slave를 해제하지 않고, 대신 **master 역할을 UI에서
고를 수 없게** 한다 (외부 발생원이 있는데 카메라가 또 쏘면 충돌).

## 4. 역할 칸 재정의 — "PTP 동기" → "동기 방식"

현재 `감지된 장비` 표의 역할 칸은 PTP action 전용이다:

- [`sensor_config.py:384`](../scripts/sensor_config.py#L384) `PTP_ROLES = ["sender", "receiver", "none"]`
- [`sensor_config.py:414-415`](../scripts/sensor_config.py#L414-L415) 새 카메라 기본값 `hardware_trigger_role: "none", ptp_action_role: "receiver"`
- [`sensor_stage.py:1495-1556`](../scripts/sensor_stage.py#L1495-L1556) 칸 제목 `"PTP 동기"`, 카메라별 콤보

새 구성에서 sender/receiver 개념은 사라진다. 두 가지 축을 한 칸에 담아야 한다.

**제안: 칸 제목을 `동기 방식` 으로 바꾸고 선택지를 역할 쌍으로 표현한다.**

| UI 표시 | `hardware_trigger_role` | `ptp_action_role` |
|---------|------------------------|-------------------|
| `PPS 트리거` | `slave` | `none` |
| `PTP 액션 (보내기)` | `none` | `sender` |
| `PTP 액션 (받기)` | `none` | `receiver` |
| `자유 실행` | `none` | `none` |

이렇게 하면 두 역할이 동시에 켜지는 잘못된 조합을 UI에서 원천 차단할 수 있다
(카메라 노드도 이 조합을 예외로 막는다 —
`flir_spinnaker_camera_node.cpp:734-740`). 또 기존 PTP action 구성으로
되돌아갈 수 있는 길도 남는다 — 배선 문제로 오늘 PPS가 안 되면 GUI에서 바로
되돌릴 수 있어야 한다.

**기본값:** `external_trigger: true` 인 subset의 새 카메라는 `PPS 트리거`.

**일괄 지정이 필요하다.** 8대를 한 대씩 바꾸는 건 실수를 부른다. 표 머리 또는
툴바에 **"전체 동기 방식 지정"** 을 둔다.

## 5. `sensors.yaml` 신규·변경 필드

### 5.1 새로 올릴 필드 (visible subset)

```yaml
      - subset: visible
        section: "동기"
        key: "ptp.enable"
        label: "PTP 시각 동기"
        type: bool
        default: true
        help: "카메라 내부 시계를 PTP grandmaster(GNSS)에 맞춥니다. PPS 트리거를 써도 이게 꺼져 있으면 타임스탬프가 자유 실행 카운터라 절대 시각을 알 수 없습니다. 트리거(노출 정렬)와 시각(타임스탬프)은 별개입니다."

      - subset: visible
        section: "동기"
        key: "hardware_trigger.slave.trigger_source"
        label: "PPS 입력 라인"
        type: enum
        choices: ["Line0", "Line3"]
        default: "Line3"
        help: "Line3 은 비절연 입력이라 지연·지터가 최소입니다 (PPS 정밀도를 살리려면 이쪽). Line0 은 옵토 절연이라 전기적으로 안전하지만 커플러 지연이 µs 단위로 붙습니다. 어느 쪽이 맞았는지는 동기 검증의 지터 폭이 알려줍니다."

      - subset: visible
        section: "동기"
        key: "hardware_trigger.slave.trigger_activation"
        label: "트리거 엣지"
        type: enum
        choices: ["RisingEdge", "FallingEdge", "AnyEdge", "LevelHigh", "LevelLow"]
        default: "RisingEdge"
        help: "PPS 신호의 어느 엣지에 노출을 시작할지. 프레임이 0장이면 여기부터 의심합니다."
```

### 5.2 `ResolveHeaderStamp` 수정 후 추가 (짝 문서 §6)

카메라 노드에 `timestamp.utc_offset_ns` / `timestamp.subtract_exposure` 가
생기면 같이 올린다. **이 둘은 값을 잘못 넣으면 전체 데이터셋의 시각이 조용히
틀어지므로 반드시 `fields` 에 설명과 함께 올린다.**

```yaml
      - subset: visible
        section: "동기"
        key: "timestamp.utc_offset_ns"
        label: "TAI→UTC 보정"
        type: int
        default: 0
        unit: "ns"
        help: "PTP 시간축은 TAI라 UTC보다 37초 앞섭니다(2026년). GNSS grandmaster를 쓰면서 시스템 시계가 UTC면 37000000000 을 넣습니다. 0인지 37초인지는 추측하지 말고 [동기 검증] 의 정수초 잔차로 확인하세요."

      - subset: visible
        section: "동기"
        key: "timestamp.subtract_exposure"
        label: "노출 시간 보정"
        type: bool
        default: false
        help: "카메라가 노출 '끝' 에 타임스탬프를 찍으면 header.stamp 가 노출 시간만큼 늦습니다. 켜면 노출 시작 시점으로 되돌립니다. [동기 검증] 의 정수초 잔차가 노출 시간과 비슷하게 나오면 켜세요."
```

### 5.3 의미가 바뀌어 손봐야 할 기존 항목

| 위치 | 현재 | 문제 |
|------|------|------|
| `ptp_master_interface` (`sensors.yaml:340-346`) | `default: "enp3s0f1"` | GNSS가 GM이면 PC가 GM이 되면 안 된다. **기본값을 비우고**(`empty_value: "none"` 이미 있음) help를 "GNSS 등 외부 grandmaster를 쓰면 반드시 비웁니다" 로 |
| `camera.ExposureTime` help (`sensors.yaml:158`) | "카메라는 노출이 끝날 때 타임스탬프를 찍어서…" | **이 서술이 §5.2 `subtract_exposure` 의 근거다.** 실측(T4)으로 확인되면 서로 링크. 부정되면 양쪽 다 수정 |
| `ptp_action.rate_hz` (`sensors.yaml:134-141`) | "PTP 동기 프레임레이트" | PPS 구성에서는 쓰이지 않는다. 동기 방식이 `PPS 트리거` 면 폼에서 비활성화 (`enabled_when`) |
| `camera.DeviceLinkThroughputLimit` help (`sensors.yaml:186`) | "PTP 동기면 모든 카메라가 같은 순간에…" | PPS 트리거도 동일하게 동시 버스트다. "PTP 동기면" → "동기 트리거면" |
| ouster `timestamp_mode` help (`sensors.yaml:~390`) | "TIME_FROM_ROS_TIME 이어야 카메라 header.stamp 와 같은 시계를 씁니다." | §7 참조 — 전제가 바뀐다 |

`camera.ExposureTime` 의 기존 help는 현장 경험에서 나온 서술로 보이며, 짝 문서
T4의 예상 결과("잔차 ≈ 노출 시간이면 노출 종료 래치")와 일치한다. 즉 T4에서
`subtract_exposure` 가 필요한 쪽으로 나올 가능성이 높다.

## 6. 신규 — 동기 검증 패널

**이게 이번 작업의 핵심이다.** 지금 동기 검증은 터미널에서
`python3 scripts/check_multicam_sync.py` 를 직접 돌려야 한다. "모든 것은 GUI에서"
원칙에 맞지 않고, 무엇보다 **운행 직전에 동기가 맞는지 확인할 방법이 없다.**

`감지된 장비` 탭 또는 녹화 탭에 **[동기 검증]** 버튼을 추가한다. 누르면 20초간
수집 후 표로 보여준다.

| 열 | 내용 | 판정 |
|----|------|------|
| 카메라 | 네임스페이스 | — |
| 프레임 수 / 레이트 | 수집 기간 실측 | 기대 레이트와 다르면 경고 |
| 라운드 스프레드 | 짝 문서 T3 | < 50 µs 정상 / 50 µs~1 ms 경고 / > 1 ms 실패 |
| 정수초 잔차 (중앙값) | 짝 문서 T4 | 0 근처 또는 노출 시간 근처면 정상. **37초 근처면 TAI 경고** |
| 잔차 지터 (표준편차) | 짝 문서 T4 | < 10 µs 정상 / > 1 ms 실패 |

**구현:** `scripts/check_multicam_sync.py` 의 라운드 그룹핑 로직을 그대로 쓰되,
정수초 잔차 계산을 더한다. 짝 문서 §8에 `scripts/check_pps_phase.py` 신규 작성이
산출물로 잡혀 있으므로, **그 스크립트를 FLIR 리포에 만들고 GUI는 그것을 호출해
파싱하는 방식**이 중복이 없다 (`bag_diagnostics.py` 가 이미 외부 도구를 호출해
결과를 표로 보여주는 패턴을 쓴다 — 그 구조를 따른다).

판정 임계값은 하드코딩하지 말고 `sensors.yaml` 에 둔다. PPS냐 PTP action이냐에
따라 기준이 100배 다르기 때문이다 (짝 문서 T3 표).

```yaml
    sync_check:
      spread_warn_us: 50
      spread_fail_us: 1000
      jitter_warn_us: 10
      jitter_fail_us: 1000
```

**추가로 표시할 것:**

- **PTP 상태** — `pmc -u -b 0 -s /tmp/ptp4l-flir 'GET PORT_DATA_SET'` 의 portState와
  `TIME_STATUS_NP` 의 master_offset. 짝 문서 T0에 해당하며, **노드를 띄우기 전에**
  확인해야 하는 값이다. 기동 버튼 옆에 신호등으로 두면 좋다.
- **FLIR 워크스페이스 실제 경로** — §2(A)의 심볼릭 링크를 쓰면 어느 체크아웃인지
  안 보인다. 해석된 절대 경로를 설정 탭에 표시한다.

## 7. Ouster 연동

`sensors.yaml` 의 ouster `timestamp_mode` 는 이미 네 선택지를 노출하고 있고,
그중 `TIME_FROM_SYNC_PULSE_IN` / `TIME_FROM_PTP_1588` 이 이번 구성과 직결된다.
GUI 수정은 **help 문구뿐**이다.

현재 help: "TIME_FROM_ROS_TIME 이어야 카메라 header.stamp 와 같은 시계를 씁니다."

이 서술은 "카메라 header.stamp = 호스트 도착 시각" 을 전제한다. 새 구성에서는
카메라 header가 GNSS 시각이 되므로 전제가 무너진다. 선택지별로:

| 모드 | 새 구성에서 |
|------|-------------|
| `TIME_FROM_ROS_TIME` | 시스템 시계가 GNSS에 규율되면 여전히 맞다. 단 규율 전에는 어긋남 |
| `TIME_FROM_PTP_1588` | GNSS PTP가 라이다까지 도달하면 **가장 정확** |
| `TIME_FROM_SYNC_PULSE_IN` | PPS를 라이다에도 물렸다면 이것 |

배선에 여유가 있으면 라이다도 GNSS에 직접 물리는 쪽이 낫다 (짝 문서 §2).
어느 쪽을 택하든 help를 실제 배선에 맞게 고친다.

## 8. 작업 순서

`~/FLIR_control` 경로(§2)가 먼저다 — 안 고치면 GUI에서 아무것도 못 띄운다.

| # | 작업 | 선행 | 비고 |
|---|------|------|------|
| 1 | `~/FLIR_control` 심볼릭 링크 (§2) | — | 1분. 오늘 제일 먼저 |
| 2 | `external_trigger` 플래그 + `_fix_sync_roles` (§3) | — | 없으면 매 기동마다 동기 해제 |
| 3 | `ptp_master_interface` 기본값 비우기 (§5.3) | — | 설정 한 줄 |
| 4 | `sensors.yaml` 동기 필드 3개 (§5.1) | — | 설정만 |
| 5 | 역할 칸 재정의 + 일괄 지정 (§4) | 2 | UI 작업 |
| 6 | 동기 검증 패널 (§6) | FLIR `check_pps_phase.py` | 이번 작업의 핵심 |
| 7 | `timestamp.*` 필드 (§5.2) | FLIR `ResolveHeaderStamp` 수정 | T4 확정 후 |
| 8 | help 문구 정리 (§5.3, §7) | 실측 | 마지막 |

1~4는 설정·소폭 수정이라 배선 전에 끝낼 수 있다. 5~6이 실제 개발 분량이고,
7~8은 짝 문서의 실측(T4) 결과에 종속된다.

## 9. 미결

짝 문서 §3의 분기가 GUI에도 그대로 영향을 준다.

- **PPS 주파수** — 1 Hz 직결이면 `camera.AcquisitionFrameRate` / `ptp_action.rate_hz`
  관련 필드와 대역폭 help(§5.3)의 수치 전제가 전부 달라진다.
- **GNSS PTP NIC** — 카메라망에 같이 물리면 `ptp_master_interface` 를 비우는 것으로
  끝난다. 별도 NIC면 PC가 boundary clock이 되어야 하고, ptp4l 2포트 설정을 GUI가
  어떻게 다룰지 별도 설계가 필요하다.
- **GPIO 입력 라인** — §5.1의 `Line0`/`Line3` 선택지는 실측 지터로 확정한다.
