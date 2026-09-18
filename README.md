# clip_recorder

ROS 2 Humble용 **블랙박스형 클립 레코더**.
항상 최근 `pre_sec`초를 RAM에 담아두고 있다가, 트리거가 오면 `[사건 - pre_sec, 사건 + post_sec]` 구간을
rosbag2 bag으로 저장한다. 사고·이상 상황 직후 "방금 그 장면"을 남기는 용도.

```
      ┌──────── pre_sec (10 s) ────────┐┌── post_sec (5 s) ──┐
──────┼────────────────────────────────┼┼────────────────────┼──────▶ 시간
   버퍼 시작                          사건(트리거)          저장 시작
```

---

## 1. 빌드 / 실행

```bash
cd ~/ros2_ws
colcon build --packages-select clip_recorder
source install/setup.bash

ros2 launch clip_recorder clip_recorder.launch.py     # config/params.yaml 사용
# 또는
ros2 run clip_recorder clip_recorder --ros-args -p pre_sec:=10.0 -p post_sec:=5.0
```

> **실행 순서**: 레코더는 다른 노드(센서 드라이버 등)가 **다 뜬 뒤에** 실행한다.
> 토픽 발견 시점에 퍼블리셔가 없으면 `/tf_static` 같은 transient_local 토픽을 못 받을 수 있다.

---

## 2. 트리거

두 가지 방법이 있고, **실차에서는 2번(토픽)을 권장**한다.

### 2-1. 서비스 (테스트용)

```bash
ros2 service call /clip_recorder/trigger_clip std_srvs/srv/Trigger
```

요청이 **도착한 시각**이 사건 시각이 된다. `ros2 service call` CLI는 매번 파이썬 기동 + 노드 생성 +
DDS 디스커버리를 거치므로 **수백 ms ~ 1 s 이상 늦게** 도착한다 (토픽이 많을수록 심해짐).
실측: CLI 250~500 ms, 상시 연결 클라이언트 0.3 ms.

### 2-2. 스탬프 토픽 (실차용)

`/clip_recorder/trigger` (`std_msgs/msg/Header`)

| 필드 | 의미 |
|---|---|
| `stamp` | 사건이 **실제로 일어난 시각**. 0이면 수신 시각 사용 |
| `frame_id` | 선택 라벨. 클립 폴더명에 `_라벨`로 붙음 (영숫자/`_`/`-`만, 32자) |

늦게 도착해도 버퍼에서 `[stamp - pre_sec, stamp + post_sec]`를 정확히 잘라낸다.
`trigger_slack_sec`(기본 2 s)만큼 늦는 것까지 pre 구간이 온전하다.
창이 이미 다 지난 아주 늦은 트리거는 기다리지 않고 즉시 저장한다.

터미널에서 바로 쏘려면 (퍼블리셔를 열어두고 **Enter**로 트리거, 라벨 입력 후 Enter면 라벨 포함, `q` 종료):

```bash
ros2 run clip_recorder clip_trigger
ros2 run clip_recorder clip_trigger --label hard_brake   # 기본 라벨
```

버튼/조이스틱/GPIO 노드를 직접 만들 때도 같은 원리 — **퍼블리셔를 미리 만들어두고** 쏘는 것이 핵심:

```python
from std_msgs.msg import Header
pub = node.create_publisher(Header, '/clip_recorder/trigger', 10)   # 노드 시작 시 한 번

# 버튼이 눌렸을 때
h = Header()
h.stamp = node.get_clock().now().to_msg()
h.frame_id = 'hard_brake'
pub.publish(h)
```

호스트가 여러 대면 호스트마다 레코더를 띄워도 publish 한 번으로 전부 동시에 찍힌다.

트리거가 거부되는 경우: 이미 클립 진행 중이거나 bag 쓰기가 끝나지 않았을 때
(`busy: clip in progress or bag write pending`).

---

## 3. 출력

```
<output_dir>/
├── clip_20260826_153920_hard_brake/     # 클립: clip_YYYYmmdd_HHMMSS[_라벨]
│   ├── clip_20260826_153920_hard_brake_0.db3
│   └── metadata.yaml
└── rec_20260919_041500_drive1/          # 수동 녹화: rec_YYYYmmdd_HHMMSS[_라벨]
    ├── rec_20260919_041500_drive1_0.db3 #   시작부터 중지까지 한 파일 (나누지 않음)
    └── metadata.yaml
```

**수동 녹화** — 클립(트리거 앞뒤 창)과 별개로, 시작부터 중지까지 전부 **bag 하나로** 쓴다
(중간에 나누지 않는다 — `record_split_sec` 기본 0. CLI 로 레코더를 띄울 때 >0 을 주면 그 초마다 나눈다).
GUI 녹화 탭의 [⏺ 수동 녹화] 버튼, 또는:

```bash
ros2 topic pub --once /clip_recorder/record std_msgs/msg/String "{data: 'start|drive1'}"   # 라벨 선택
ros2 topic pub --once /clip_recorder/record std_msgs/msg/String "{data: stop}"
ros2 service call /clip_recorder/start_recording std_srvs/srv/Trigger   # 서비스로도
ros2 service call /clip_recorder/stop_recording std_srvs/srv/Trigger
```

- 링 버퍼와 **같은 구독**을 쓴다. `ros2 bag record` 를 따로 띄우면 카메라 15대·라이다가 모든 메시지를 한 번
  더 보내야 한다 (카메라 노드 CPU 가 이 PC 에서 가장 빠듯하다). 받은 버퍼를 **복사 없이** bag writer 에
  넘기고, 디스크 쓰기는 rosbag2 캐시 스레드(`record_cache_mb`, 기본 256 MB)가 한다.
- 녹화 중에도 클립 트리거가 된다 (클립은 따로 `clip_…` 폴더).
- 시작할 때 latched 토픽(`/tf_static`, `/ouster/metadata` 등)의 마지막 값을 맨 앞에 넣는다 — 한 번만 발행돼서
  안 넣으면 녹화에 없다.
- 이벤트(`~/clip_event`): `rec_started|<uri>`, `rec_closing|<uri>`, `rec_stopped|<uri>|<msgs>|<초>|<MB>`,
  `rec_busy|…`, `rec_error|…`. 상태는 `/diagnostics` 의 `rec_active` · `rec_sec` · `rec_mb` 로도 나간다.
- 레코더가 SIGINT 로 내려가면 녹화 중이던 bag 을 닫고(캐시 flush) 내려간다.

```bash
ros2 bag info clips/clip_20260826_153920_hard_brake
ros2 bag play clips/clip_20260826_153920_hard_brake
```

`output_dir`이 상대 경로면 **노드를 실행한 위치 기준**이므로 실차에서는 절대 경로를 쓴다.
mcap이 더 빠르고 작다: `sudo apt install ros-humble-rosbag2-storage-mcap` 후 `storage_id: "mcap"`.

---

## 4. 파라미터 (`config/params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `pre_sec` | 10.0 | 사건 이전 보존 구간 (초) |
| `post_sec` | 5.0 | 사건 이후 녹화 구간 (초) |
| `trigger_slack_sec` | 2.0 | 트리거 지연 허용치. 버퍼는 `pre_sec + 이 값`을 보관 (메모리도 그만큼 더 씀) |
| `max_buffer_mb` | 4096.0 | 링 버퍼 상한. 초과 시 오래된 것부터 버림 → **pre 구간이 조용히 짧아짐**. §6으로 측정해서 정할 것 |
| `queue_depth` | 50 | 구독 QoS history depth. 이미지처럼 큰 토픽은 5~10 권장 |
| `output_dir` | `"clips"` | 저장 경로 |
| `storage_id` | `"sqlite3"` | `"mcap"` 가능 |
| `topics` | (미지정) | 녹화할 토픽 목록. **키를 아예 안 쓰면** 전체 (exclude 제외) |
| `exclude` | `[/rosout, /parameter_events]` | 제외 토픽 (정확히 일치) |
| `topic_qos` | (미지정) | 토픽별 QoS 강제. `"/토픽 <auto\|reliable\|best_effort> <auto\|volatile\|transient_local>"` |
| `status_period_sec` | 5.0 | 버퍼 상태 리포트 주기 (로그 + `/diagnostics`). 0이면 끔 |

> ⚠️ params.yaml에 **빈 리스트(`topics: []`, `topic_qos: []`)를 쓰면 노드가 죽는다.**
> ROS 2 파라미터 YAML 파서는 빈 시퀀스의 타입을 추론하지 못해 노드 생성 단계에서
> `InvalidParameterValueException: parameter_value_from failed for parameter 'topics': No parameter value set`
> 를 던진다 (declare_parameter 이전이라 코드로는 못 막는다). "전체"를 원하면 **그 키를 지우거나 주석 처리**할 것.
> 실행 중 `ros2 param set`으로는 빈 배열을 넣어도 정상 동작한다.

`topics`, `exclude`, `topic_qos`는 **실행 중에도 변경 가능** (2초 내 반영, 구독 자동 재구성):

```bash
ros2 param set /clip_recorder topics '["/cam0/compressed","/lidar/points","/tf","/tf_static"]'
ros2 param set /clip_recorder topic_qos '["/cam0/compressed best_effort volatile"]'
```

QoS `auto`는 현재 퍼블리셔에 맞춘다 (rosbag2 record와 같은 방식).
**센서 스트림은 `best_effort`로 강제하는 것을 권장** — reliable로 붙으면 레코더가 잠깐 밀릴 때
DDS 재전송이 퍼블리셔(카메라 드라이버)까지 늦출 수 있다. 레코더는 구경만 해야 한다.

---

## 5. 버퍼 상태 리포트

`status_period_sec`마다 이런 로그가 찍히고 같은 내용이 `/diagnostics`로 나간다:

```
status: 812.3 MB / 4096.0 MB cap, 12.0 s of 12.0 s retained, 67.7 MB/s in, 812.3 MB needed
         ─┬───    ─┬────        ─┬──    ─┬──                ─┬─────          ─┬──────
          │        │             │       │                   │                └ 이 유입 속도로 12초를 담는 데 필요한 메모리
          │        │             │       │                   └ 녹화 중인 토픽 전체 유입량
          │        │             │       └ 보관 목표 = pre_sec + trigger_slack_sec
          │        │             └ 실제 담긴 시간. 목표보다 작으면 상한에 걸려 잘리는 중
          │        └ max_buffer_mb
          └ 현재 버퍼 사용량
```

상한에 걸리면 WARN으로 바뀌고 가장 무거운 토픽 Top 5를 같이 찍는다.

---

## 6. 상한(max_buffer_mb) 정하기 — `buffer_probe`

```bash
# 터미널 1: 실차 노드 전부 띄운 상태에서 레코더 실행
ros2 run clip_recorder clip_recorder
# 터미널 2: 측정 (실제 주행 상황에서 몇 분)
ros2 run clip_recorder buffer_probe --duration 300
```

```
[    5s] in    67.7 MB/s  need     812 MB  buf    812/4096 MB  span 12.0/12.0 s
...
측정 60회, 300초
유입량      평균 65.2 MB/s   피크 71.9 MB/s
보관 목표   12.0 s (pre_sec + trigger_slack_sec)
필요 메모리 평균 782 MB   피크 863 MB
현재 상한   4096 MB   (초과 없음)

▶ 추천 max_buffer_mb: 1280   (피크 863 MB × 1.3)
  이 PC RAM 31.3 GB 의 4%

무거운 토픽 Top 10 (평균 MB/s, 비중, 12초 보관 시 MB):
     18.2 MB/s   27.9%      218 MB   /cam3/image_raw
     ...
```

- **추천 max_buffer_mb** 를 그대로 params.yaml에 넣는다.
- **상한 초과 N회** 가 뜨면 현재 설정으로는 pre 구간이 잘리고 있다는 뜻.
- 추천값이 RAM의 50%를 넘으면 상한을 올릴 게 아니라 **Top 목록의 무거운 토픽을 빼야** 한다.
  raw 이미지가 상위에 있으면 그게 범인 — `compressed`만 남긴다.

참고 규모: 1080p raw 1대 = 180 MB/s (20대면 10초에 36 GB → 불가).
JPEG compressed 1대 ≈ 6 MB/s (20대 + 라이다 10초 ≈ 1.5 GB → 가능).

---

## 7. 알아둘 제약 (현재 버전)

| 상황 | 동작 |
|---|---|
| 저장 중(bag 쓰는 동안) 새 트리거 | **거부**. 1~2 GB 쓰기는 수십 초 걸릴 수 있어 그 사이 사건은 놓침 |
| post 구간 중 새 트리거 | 거부 (연장 아님) |
| post 구간 중 Ctrl+C | 그 클립은 **유실** |
| 디스크 부족 | 확인 안 함. 쓰기 실패 시 에러 로그만 |
| 오래된 클립 삭제 | 없음 (수동) |
| 이 PC에 없는 메시지 타입 | 해당 토픽만 건너뛰고 WARN (노드는 계속 동작) |
| 토픽 발견 시 퍼블리셔 없음 | volatile/best_effort로 구독 고정 → transient_local 데이터 못 받음 (레코더를 나중에 띄울 것) |
| 타임스탬프 | 메시지 **수신 시각** 기준 (header.stamp 아님). rosbag2 record와 동일 |

### 개선 예정
1. 저장 큐 (쓰는 중에도 트리거 수락) + 재트리거 시 post 연장 + mcap 기본화
2. 디스크 용량 가드 + 오래된 클립 순환 삭제
3. 서비스/타이머 콜백 그룹 분리 (부하 시 트리거 지터 감소)
4. 토픽/QoS 선택 GUI

---

## 8. 구현 메모

- 메시지는 **역직렬화하지 않고** serialized 그대로 `std::deque`에 보관 (`rclcpp::GenericSubscription`).
  CPU 부담이 거의 없고, 어떤 타입이든 녹화 가능.
- transient_local 토픽(`/tf_static` 등)은 최신 샘플을 따로 들고 있다가 모든 클립 앞에 붙인다.
  버퍼에서 밀려난 지 오래여도 클립 재생 시 t=0부터 존재.
- bag 쓰기는 별도 스레드. 그동안 구독은 계속된다.
- **Humble `rosbag2_cpp::Writer::write(shared_ptr<SerializedMessage>)`는 메시지 버퍼의 소유권을 가져간다**
  (헤더 주석: "the serialized data will no longer be managed by message"). 링 버퍼가 같은 메시지를
  계속 참조하므로 반드시 **복사본**을 넘겨야 한다. 원본을 넘기면 세그폴트.
- post 타이머는 노드 클록 기반이라 `use_sim_time`에서도 동작.
- 모든 콜백이 기본 MutuallyExclusive 그룹 → 사실상 단일 스레드 처리 (충분히 빠르지만 부하 시 개선 여지).

---

## 9. QoS 기초

QoS(Quality of Service)는 "이 토픽을 **얼마나 성실하게, 얼마나 오래, 얼마나 쌓아서** 전달할 건가"를
퍼블리셔와 구독자가 각각 정하는 설정이다. 둘이 맞아야 연결된다.

### Reliability — 잃어버려도 되나?

| 값 | 뜻 | 비유 |
|---|---|---|
| `reliable` | 못 받은 메시지는 재전송해서 반드시 전달 | 등기 우편 |
| `best_effort` | 한 번 보내고 끝. 떨어지면 없어짐 | 라이브 방송 |

센서 스트림은 보통 `best_effort` (프레임 하나 빠져도 다음 게 온다). 명령/상태/이벤트는 `reliable`.
reliable 구독자가 밀리면 DDS가 재전송·백로그를 쌓고, 그 부담이 **퍼블리셔(드라이버)까지 느려지게** 한다.

### Durability — 늦게 온 구독자에게 예전 것을 주나?

| 값 | 뜻 |
|---|---|
| `volatile` | 구독 시작 이후에 나온 것만 받음 |
| `transient_local` | 퍼블리셔가 마지막 메시지를 들고 있다가 늦게 온 구독자에게도 줌 ("latched") |

`/tf_static`, `robot_description`, 맵처럼 한 번 발행되고 안 바뀌는 것들이 `transient_local`.

### History — 얼마나 쌓아두나?

| 값 | 뜻 |
|---|---|
| `keep_last` + `depth` N | 콜백이 아직 안 가져간 메시지를 N개까지 큐에 보관, 넘치면 오래된 것부터 버림 |
| `keep_all` | 무제한 (메모리 위험, 거의 안 씀) |

`depth`는 "콜백이 처리하기 전에 미들웨어가 들고 있는 큐 길이". 이미지 토픽에 depth 50이면 콜백이
밀리는 순간 50프레임 × 6 MB = 300 MB를 DDS가 따로 들고 있게 된다.

### 둘이 맞아야 연결된다

규칙 하나: **구독자가 퍼블리셔보다 더 까다로우면 연결이 안 된다.** 에러 없이 조용히 데이터가 안 온다.

| 퍼블리셔 ↓ / 구독자 → | reliable | best_effort |
|---|---|---|
| reliable | ✅ | ✅ (재전송 없이) |
| best_effort | ❌ 연결 안 됨 | ✅ |

| 퍼블리셔 ↓ / 구독자 → | transient_local | volatile |
|---|---|---|
| transient_local | ✅ (과거 것도) | ✅ (새 것만) |
| volatile | ❌ 연결 안 됨 | ✅ |

레코더의 `auto`는 퍼블리셔 QoS를 보고 따라간다. `best_effort` + `volatile`은 어떤 퍼블리셔와도 연결되는
안전한 조합 (단, 과거 메시지는 못 받음). 토픽의 퍼블리셔 QoS는 `ros2 topic info -v /토픽` 으로 확인.

### 나머지 (거의 기본값으로 둠)

- **Deadline** — N ms마다 메시지가 와야 함, 안 오면 콜백으로 알림
- **Lifespan** — 메시지 유효기간
- **Liveliness** — 퍼블리셔 생존 확인 방식

### 프리셋

| 프리셋 | 내용 | 용도 |
|---|---|---|
| `sensor_data` | best_effort, volatile, keep_last 5 | 카메라, 라이다, IMU |
| `default` | reliable, volatile, keep_last 10 | 일반 토픽 |
| (latched 관습) | reliable, transient_local, keep_last 1 | tf_static, 파라미터성 정보 |

### 레코더 권장

| 토픽 종류 | topic_qos | 이유 |
|---|---|---|
| 카메라/라이다 (클립 용도) | `best_effort volatile`, depth 5~10 | 시스템에 영향 없이 구경만 |
| `/tf_static` 류 | `reliable transient_local` (auto가 알아서) | 과거 샘플 필요 |
| 작은 상태 토픽 | `auto` | |

---

## 10. 드랍 없는 센서 로깅 가이드

> 전제: **스위치 → 랜카드 구간은 무손실**이라고 가정한다. 즉 네트워크 장비에서의 패킷 손실은 고려하지
> 않고, 랜카드에 도착한 이후 **호스트 안에서** 잃는 경우만 다룬다.

### QoS만으로는 안 된다

QoS는 전송 구간 하나만 다룬다. 실제 드랍은 아래 사슬의 여러 단계에서 나므로, 전부 맞춰야 한다.

```
드라이버 ─publish→ [퍼블리셔 큐] ─DDS/UDP→ [커널 소켓 버퍼] → [구독자 큐] ─콜백→ [rosbag 캐시] ─쓰기→ 디스크
             ①              ②                 ③                ④              ⑤           ⑥
```

| 단계 | 어디서 잃나 | 막는 방법 |
|---|---|---|
| ① 퍼블리셔 큐 | 퍼블리셔 `depth`가 5인데 구독자가 6프레임 밀리면, reliable이어도 **재전송할 원본이 없음** | 드라이버 쪽도 reliable + depth 크게 |
| ② UDP 조각 | 큰 메시지(이미지)는 여러 UDP 조각으로 쪼개짐. 조각 하나만 잃어도 메시지 전체 폐기. 같은 PC에서도 loopback으로 가면 발생 | 같은 PC면 **공유메모리 전송**, 다른 PC면 ③ |
| ③ 커널 소켓 버퍼 | 랜카드에 도착했지만 프로세스가 가져가기 전에 커널 수신 버퍼가 넘침 (기본 200 KB 정도) | `sysctl` 로 버퍼 확대 (아래) |
| ④ 구독자 큐 | 콜백이 느리면 depth 넘친 것부터 버림 | depth 크게, 콜백 가볍게 |
| ⑤ rosbag 캐시 | 캐시가 차면 **버린다** (로그에 "dropped" 뜸) | 캐시 크게 (`--max-cache-size`) |
| ⑥ 디스크 | 쓰기 속도 < 유입 속도면 캐시가 차서 ⑤로 | NVMe, mcap, 압축 최소 |

### 설정 체크리스트

**1) 구독자(레코더) QoS**: `reliable` + `keep_last`, depth는 **1~2초 분량** (30 Hz 카메라면 30~60).

**2) 퍼블리셔(드라이버) QoS도 같이**: 구독자만 reliable로 해봤자 드라이버가 best_effort면 연결이 안 되고,
reliable이라도 depth가 작으면 재전송할 원본이 없다. 먼저 확인:

```bash
ros2 topic info -v /cam0/image_raw     # Publishers: 아래 QoS profile
```

드라이버가 `qos_overrides` 파라미터를 지원하면 (대부분의 표준 드라이버):

```yaml
cam0_driver:
  ros__parameters:
    qos_overrides:
      /cam0/image_raw:
        publisher:
          reliability: reliable
          history: keep_last
          depth: 60
```

지원 안 하면 드라이버 소스나 자체 QoS 파라미터를 봐야 한다 (ZED wrapper는 자체 파라미터 있음).

**3) 커널 소켓 버퍼** (다른 PC에서 받을 때 필수, 같은 PC라도 권장):

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=67108864
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728   # UDP 조각 재조립 버퍼
# 영구 적용: /etc/sysctl.d/60-ros2.conf 에 기록
```

DDS 쪽도 소켓 버퍼를 크게 요청해야 한다 (FastDDS XML `sendBufferSize/receiveBufferSize`,
CycloneDDS `SocketReceiveBufferSize`).

**4) 같은 PC면 공유메모리**: FastDDS는 같은 호스트 간 SHM이 기본. CycloneDDS는 iceoryx 연동 필요.
UDP 조각 유실(②)이 사라진다.

**5) 디스크/포맷**: NVMe, `storage_id: mcap`, 압축은 없거나 zstd 가벼운 레벨, rosbag 캐시 크게.
sqlite3는 고속 쓰기에 약하다.

### 물리 법칙

reliable + 큰 큐로 만들면: **구독자가 못 따라가면 퍼블리셔가 멈춘다** (`publish()` 블록 또는 큐 대기).

> 소비자가 생산자보다 느리면 "드랍"과 "생산자 정지" 중 하나는 반드시 일어난다.

"드랍 없이"를 고르는 것은 "레코더가 밀리면 카메라 드라이버가 멈춰도 된다"를 고르는 것이다.
인지 스택이 그 카메라를 같이 쓰면 위험하다. 따라서 진짜 답은 QoS가 아니라
**레코더가 유입량을 항상 감당하게 만드는 것** — 대역폭, CPU, 디스크 전부.

### 실차 구성

1. **대역폭부터 가능하게**: 20대 raw = 3.6 GB/s는 어떤 설정으로도 불가. 하드웨어 인코더로 압축(JPEG/H.264)해서
   100~200 MB/s 수준으로.
2. **링크 용량은 손실이 아니라 상한**: 스위치가 무손실이어도 1 GbE는 ~110 MB/s가 끝. 넘치면 송신 측이
   막히거나 버린다. 카메라가 붙은 PC마다 레코더를 하나씩 두거나 10 GbE.
3. **전부 남기는 것과 클립은 다른 일**: 전부 남겨야 하면 수동 녹화(§3 — 같은 레코더가 같은 구독으로 쓴다)를
   쓰고, 클립은 "사건 하이라이트" 용도로 같이 둔다. 클립 레코더의 RAM 링은 디스크 속도와
   무관하게 안 잃지만 애초에 10초짜리다.

### 설정을 믿지 말고 세라

드랍이 없는지는 측정으로만 알 수 있다.

- bag 메시지 수 vs 기대값(주파수 × 시간): `ros2 bag info`
- `header.stamp` 간격이 프레임 주기의 2배 이상 벌어지는 곳 = 드랍 구간
- rosbag2 로그의 "dropped" 문구
- `ros2 topic hz` 로 레코더 PC에서 실제 수신 주파수 확인 (드라이버 주파수와 비교)

(bag을 넣으면 토픽별 드랍 수/구간을 뽑는 검사 툴은 추가 예정)

---

## 11. GUI — `clip_gui`

```bash
clip                                  # 권장: 런처 (아래 참고). ~/.bashrc 에 alias 등록됨
ros2 run clip_recorder clip_gui       # 직접 실행도 가능
```

`clip`(= `scripts/clip_launch.sh`)은 ROS/워크스페이스를 소싱하고, IP별 트래픽 카운터 `net_probe`에
패킷 캡처 권한(cap_net_raw)이 **없을 때만** `sudo setcap`을 한 번 실행한 뒤, GUI를 일반 사용자로 띄운다.
즉 `colcon build` 직후 한 번만 비밀번호를 묻는다. GUI 안에서도 "권한 부여" 버튼(pkexec 인증 창)으로 같은 일을 할 수 있다.
GUI 자체를 sudo로 띄우지 않는 이유: 클립/설정이 root 소유가 되고 DDS 공유메모리가 root 소유가 되어
일반 사용자 노드(카메라 드라이버)와 충돌한다.

레코더를 조종하는 PyQt5 프론트엔드. 링 버퍼/트리거/저장은 그대로 C++ `clip_recorder`가 하고,
GUI는 파라미터·토픽으로 조종한다 (이미 떠 있는 외부 레코더에 붙을 수도 있다).

**끄기** — 창을 닫으면 센서가 떠 있을 때 [센서 종료하고 닫기 / 취소] 를 묻는다. 센서를 둔 채 닫는 선택지는
없다: 창이 닫히면 런치가 SIGKILL 되고 로그 파이프가 끊긴 노드들도 곧 쓰러져, "계속 돈다"가 아니라 지저분하게
죽을 뿐이었다. **터미널에서 Ctrl+C**(또는 SIGTERM, 터미널을 닫아 생기는 SIGHUP)를 받으면 묻지 않고
수동 녹화 정리(파일 닫기) → 센서 종료 → 레코더 종료 순으로 내리고 끈다. 터미널에 진행 상황이 `[clip_gui] …` 로
나오고, 기다리기 싫으면 Ctrl+C 를 한 번 더 누르면 남은 프로세스를 바로 정리하고 끝낸다. 이렇게 하려고
센서 런치와 레코더는 `setsid` 로 따로 세션에 띄운다 — 예전에는 같은 프로세스 그룹이라 Ctrl+C 가 GUI 와 런치에
동시에 가서, GUI 가 먼저 죽고 로그 파이프가 끊긴 런치가 정리 도중 쓰러지며 카메라 노드를 고아로 남기곤 했다.
`clip` 런처는 `ros2 run` 을 거치지 않고 GUI 를 바로 exec 한다 (`ros2 run` 은 받은 SIGTERM 을 GUI 에 넘기지 않는다).

창은 탭 두 개다 — **[센서 기동]** (§12) 과 **[녹화]**. 센서를 띄우는 것이 먼저이고
(아직 안 뜬 센서의 토픽은 목록에 있을 수가 없다), 기동이 끝나면 자동으로 녹화 탭으로 넘어가면서
토픽 선택 창이 열린다. 이미 떠 있는 센서에 그냥 붙으려면 [녹화] 탭 → 파일 → 토픽 선택.

**흐름**
1. **[센서 기동] 탭**: 감지 → 설정 → 기동 (§12).
2. **토픽 선택 창**: 현재 토픽이 전부 뜨고, 이전 세션 선택이 체크된 상태. 토픽별 QoS 콤보
   (기본 best_effort/volatile, transient_local 퍼블리셔가 감지된 토픽은 자동으로 transient_local).
   프로파일 저장/불러오기 가능. 설정은 `~/.config/dm_clip_gui/last_session.yaml`에 자동 저장.
3. **[녹화] 탭**: 레코더 자동 실행(없을 때), 트리거 버튼 + 단축키(기본 F9, 설정에서 변경), 라벨 입력.
   맨 위에 센서 상태 한 줄이 계속 떠 있어서, 녹화 중에 센서가 죽으면 여기서 먼저 보인다.
   - 링 버퍼: 메모리/보관 초/유입 + **디스크 여유** (현재 유입 기준 클립 몇 개 더 저장 가능한지, 부족 시 경고).
     **보관 구간 막대가 다 차야 클립 녹화 버튼이 눌린다** — 덜 찬 채로 받으면 클립 앞부분(pre)이 잘린다.
     차는 동안은 주황(`채우는 중 9초 남음`, 버튼은 `준비 중 9초`), 다 차면 초록, 메모리 상한에 걸려 영영 못 채우면
     빨강(그때는 경고와 함께 받는다). 레코더는 2초마다 알려주고, 그 사이는 GUI 가 실시간으로 채워 0.1초마다
     부드럽게 오른다. 레코더를 다시 띄우면 버퍼가 비므로 처음부터 다시 찬다.
   - 네트워크: 스위치 업링크 NIC 합계 (x / 10 Gbps) + **송신 IP별** Mbps·포트 사용률·pps.
     이름은 [센서 기동] 탭이 감지한 결과로 붙는다 — `back_left · BFS`, `thermal_left · A70`,
     `Ouster LiDAR OS-2-128`, `GNSS RT2000` (카메라는 GUI 에서 정한 이름 = 녹화되는 토픽 이름).
     라이다·GNSS 는 GigE 디스커버리에 안 잡혀서 예전에는 포트로만 추정했다 (`? Ouster LiDAR 데이터`,
     라이다 패킷은 MTU 보다 커서 단편화된 뒤쪽 조각이 `? UDP` 로도 보였다). 정체 모를 IP 가 트래픽을
     내면 센서 감지를 다시 돌린다 (1분에 한 번까지). 그래도 모르는 IP 는 목적지 포트로 정체를 추정한다 (OxTS NCOM 3000, PTP, DDS, Ouster/Velodyne 라이다 포트 등 →
     `? UDP/3000 브로드캐스트` 식). **장치 칸을 더블클릭하면 이름을 직접 지정**할 수 있다 (설정에 저장).
     `127.0.0.1`/이 PC 주소는 자동으로 "이 PC"로 표시. 5분 이상 조용한 IP는 목록에서 빠진다.
     **열 제목(장치 · IP · 대역폭)을 누르면 정렬** — 한 번 더 누르면 반대로 (▲/▼). 기본은 대역폭 ▼,
     `ui.net_sort` 에 저장.
   - GNSS: 상태 한 줄 + 위치·정확도·위성 한 줄 + **미니 지도** (현재 위치 파랑, 지금 트리거하면 담길
     pre 구간 궤적 주황, 끊기면 마지막 위치 회색, **[GNSS · 지도] 창에서 체크한 클립들의 경로**를 창과 같은
     색으로). 어떤 클립을 보일지는 그 창에서 고른다. 차를 따라가고, 마지막 위치를 기억해 다음 실행 때 거기서
     시작한다 (실시간 위치가 없으면 클립 경로들이 다 보이게 맞춘다). [크게 ↗] 는 GNSS · 지도 창.
   - 녹화 칸: **[📁 저장 위치]**(맨 왼쪽) · 라벨 · [● 클립(n초) 녹화 (F9)] · [⏺ 수동 녹화]. 저장 위치를 누르면
     폴더를 고르고, 설정에 바로 저장돼 GUI 를 다시 켜도 유지된다. 떠 있는 레코더에도 바로 알려(`output_dir`
     파라미터) 레코더를 다시 띄우지 않고 다음 클립 · 녹화부터 새 위치에 쓴다 (진행 중인 수동 녹화는 원래 위치).
   - 녹화 칸 아래 **상태 띠**: 대기 중(회색) · 클립 녹화 중(노랑) · 디스크 기록 중(파랑) · 진단 중(보라),
     수동 녹화 중엔 그 위에 빨간 띠가 하나 더 (폴더 · 경과 · 용량 · 유입 MB/s).
   - 하단 로그창 (레코더 /rosout + 클립 진행 상황, 센서 기동 탭 런치 로그와 같은 어두운 배경). 예전의 토픽별 Hz/대역폭 표는 없앴다 —
     대역폭은 네트워크 표가 장치별로, 수신 Hz 는 오른쪽 미리보기가 센서별로 보여준다.
   - **오른쪽 전체: 센서 미리보기** (§13).
4. **클립 녹화 완료 → 자동 진단** (`bag_diagnostics`) → 결과 창 + `<클립>/diagnostics.txt/.json` 저장.
   GNSS 품질(fix 비율, 정확도, 점프, 공백)도 포함되고 궤적은 `<클립>/gnss_track.json`에 캐시.
   **유실은 센서가 매긴 번호로 센다** — 도착 시각만 보면 드라이버·DDS 가 잠깐 멈췄다 몰아 보낸 것을 유실로
   착각한다 (Ouster 가 매 바퀴 5~25 ms 멈췄다 몰려 와서 "912프레임 유실"로 나오던 오탐):

   | 토픽 | 유실 기준 |
   |---|---|
   | Ouster `lidar_packets` | 패킷 안의 frame_id · measurement_id (패킷당 열 수는 데이터에서 알아냄, metadata 불필요) |
   | Ouster `imu_packets` | 패킷 안의 센서 시각 |
   | FLIR 카메라 (`image_raw` · `camera_info` · `image_rgb/compressed` · metadata) | `image_raw/metadata` 의 `camera_frame_id` (같은 stamp 로 형제 토픽에도 적용) |
   | 그 밖 | 도착 간격. 긴 간격 뒤에 몰려 와서 시간선을 따라잡으면 지터로 본다 (빠진 거라면 영영 한 주기 밀린다) |

   **보내는 쪽이 원래 고르지 않은 스트림** — GNSS RT2000(`/imu/data` · `/gps/nav_status` · `/gps/pos_type`)은
   12 ms 격자에서 4칸마다 한 칸을 비우고 보낸다 (12·12·12·24 ms → 실제 67.7 Hz). GNSS NIC 에 도착하는 패킷부터
   67개/초이고 NIC · 소켓 드롭 0 이라 녹화에서 빠진 게 아니다. 예전에는 빈 칸마다 "1프레임 유실"로 세서 클립마다
   FAIL(470프레임 남짓)이 났다. 이제 **한 칸짜리 빈 자리가 규칙적(빈 자리 사이 간격이 80% 이상 최빈값 ±1)이고
   클립 내내 같은 비율이면** 장비 출력 패턴으로 보고 WARN(`보내는 간격이 고르지 않음 … 실제 67.7 Hz`)으로만 알린다.
   같은 비율의 무작위 유실은 이 규칙성이 40% 남짓이라 걸리지 않는다. 리듬을 깨는 빈 칸(4칸 주기인데 1~2칸 만에
   또 빔)과 두 칸 이상 빈 곳은 그대로 유실로 센다 — 이 스트림에서 메시지를 1~20개 지운 시험에서 전부 FAIL.

   지연: 라이다 스캔 토픽(points · *_image · scan)은 stamp 가 한 바퀴의 시작이라 한 바퀴(100 ms)를 빼고 판정한다.
   "백로그"는 지연이 클립 동안 **계속 늘어날** 때만이다 (일정하게 늦은 건 처리 지연). 파일이 여러 개로 나뉜
   bag(`ros2 bag record --max-bag-*` 등)도 이어서 읽는다.
5. **수동 녹화 / 클립(n초) 녹화** — 녹화 탭에 버튼 둘: [● 클립(30초) 녹화 (F9)] 는 트리거 앞뒤
   pre+post 초(설정에 따라 숫자가 바뀐다), [⏺ 수동 녹화] 는 누를 때부터 다시 누를 때까지 (§3). 녹화 중엔 버튼이 [■ 녹화 중지 0:12:34] 가 되고 아래 줄에
   폴더 · 경과 · 용량 · 유입 MB/s, 디스크 칸에 "지금 녹화 속도로 약 N분 더"가 나온다. 디스크 여유가 1 GB 밑으로
   내려가면 자동으로 멈춘다. 레코더 정지 · 설정 변경(레코더 재시작) · GUI 종료 때 녹화 중이면 먼저 묻고,
   파일을 닫은 뒤 진행한다.
   GUI 가 띄운 레코더를 멈출 때는 레코더 바이너리에 직접 SIGINT 를 보낸다 — `ros2 run` 은 받은 시그널을
   넘기지 않아서, 예전처럼 `ros2 run` 에 SIGTERM 을 보내면 레코더가 고아로 남아 계속 돌았다.
6. **궤적 지도 (누적)**: 여러 클립을 체크해서 OSM 위에 겹쳐 본다. 진단이 끝난 클립은 자동으로 추가·체크.
   지도는 인터랙티브(휠 확대/축소, 드래그 이동, 전국~골목 z3~19, 순수 PyQt5 `map_widget.py` — QtWebEngine 불필요).
   타일은 인터넷에서 받아 `~/.cache/dm_clip_gui/tiles`에 캐시 (오프라인이면 캐시된 곳만 보임). PNG 내보내기 가능.
   바탕 지도는 기본으로 **흑백 · 흐리게** 그려서 궤적(흰 테두리를 두른 굵은 선, 선명한 색)이 도로·녹지와
   헷갈리지 않는다. 지도 오른쪽 위 **◐** 버튼으로 원래 색 지도와 바꿀 수 있다.
7. **GNSS · 지도 창** (상태 + 지도 + 클립 궤적이 한 창): 솔루션(pos_type / GGA 품질: RTK FIXED·FLOAT·DGPS·SPS),
   위성 수, HDOP, 정확도, 속도, fix 수신율 + 지도에 **현재 위치**와 **pre 구간(pre_sec 초) 궤적** —
   지금 트리거하면 클립에 담길 경로가 주황선으로 보인다. 왼쪽 목록에서 클립을 체크하면 누적 궤적이 같이 표시된다.
   성능: 수신은 10 Hz로 스로틀되고 그리기는 2 Hz 타이머에서만 — RT2000 100 Hz 출력에도 GUI가 막히지 않는다.

**IP별 트래픽 측정 권한** — 언매니지드 스위치에는 포트 카운터가 없으므로 PC의 업링크 포트에서
AF_PACKET으로 세는 작은 도우미(`net_probe`)를 쓴다. raw 소켓 권한이 필요한데, `clip` 런처나 GUI의
"권한 부여" 버튼이 알아서 처리한다. 수동으로 하려면:

```bash
sudo setcap cap_net_raw+ep ~/DM_clipGUI/build/clip_recorder/net_probe
```

(`colcon build`로 바이너리가 다시 만들어지면 권한이 사라진다 — 런처가 감지해서 다시 묻는다.)

**진단 단독 실행**
```bash
python3 scripts/bag_diagnostics.py clips/clip_XXXX          # 드랍/손상/GNSS 판정
python3 scripts/gnss_tools.py clips/clip_XXXX --map out.png  # 궤적 지도만
```

## 12. 센서 기동 — `[센서 기동]` 탭

리그의 센서 런처가 워크스페이스별로 흩어져 있어서 (카메라는 `~/FLIR_control`, GNSS는 `~/rt2000_ws`,
라이다는 ROS 언더레이) 터미널을 서너 개 띄워야 했다. 이 탭이 그걸 한 창에 모은다:

```
감지  ->  설정  ->  기동  ->  (녹화 탭) 토픽 선택  ->  녹화
```

**화면** — 왼쪽은 센서 **종류**별 카드(가시광 Blackfly / 열화상 A70 / Ouster / GNSS),
오른쪽은 고른 카드의 상세다.

```
┌ ↻ 다시 감지   22:10 감지 · 3종 11대          ▶ 선택한 센서 기동 (3)   ■ 전체 중지 ┐
├┃☑ ● 가시광 Blackfly [실행 중]┬ 가시광 Blackfly [실행 중]           ▶ 기동   ■ 중지 ┤
│┃   8 대 감지                 │ [ 감지된 장비 8 | 설정 ●1 ]                         │
│┃☑ ● 열화상 A70  [기동 중 1/2]│   이름 ▲ / 시리얼 / IP / PTP 동기 / 모델 / NIC      │
│┃   2 대 감지                 ├─────────────────────────────────────────────────────┤
│ ☐ ○ GNSS RT2000     [정지]   │ 런치 로그                                           │
│    0 대 감지                 │                                                     │
└─────────────────────────────┴──────────────────────────────────────────────────────┘
```

- **카드 왼쪽 색 띠와 제목 옆 점 = 실행 상태**: 초록 실행 중 · 주황 기동 중(점이 깜빡임, 알약에 `기동 중 5/15`) ·
  빨강 실패 · 없음(회색 점) 정지. 목록만 훑어도 어느 센서가 도는지 보인다.
- **감지된 장비 표의 행마다 점 = 그 장비의 상태** (이름 칸 앞, 마우스를 올리면 설명):

  | 점 | 상태 | 판단 근거 |
  |---|---|---|
  | ○ 빈 주황 | 대기 | 이번 기동에 들어갔지만 아직 로그가 없음 — 차례를 기다리는 중 |
  | ● 주황 깜빡임 | 기동 중 | 그 카메라 노드의 로그가 나오기 시작함 (카메라를 열고 설정하는 중) |
  | ● 초록 | 실행 중 | 그 노드가 `Camera acquisition started` 를 찍음 (영상 수신 시작) |
  | ● 빨강 | 실패 | `process has died` · 기동 제한 시간 안에 안 뜸 · 떴던 토픽이 사라짐 |
  | ○ 빈 회색 | 정지 / 미포함 | 센서군이 안 돎, 또는 못 여는 장비라 이번 기동에서 뺌 |

  카메라 노드는 뜨자마자(카메라를 열기 전에) 발행자를 만들어서 "토픽이 있다"만으로는 영상이 나오는지 모른다.
  그래서 `sensors.yaml` 의 `ready_log` 가 그 노드 로그(로거 이름 `<네임스페이스>.flir_camera`)에 찍혀야 실행 중으로 보고,
  카드의 `기동 중 N/M` 과 기동 완료 판정도 같은 기준이다. 다 뜬 뒤에도 토픽을 계속 확인해서, 도중에 노드가
  죽거나 토픽이 사라지면 그 행이 빨개지고 카드 알약이 `실행 중 · 1대 실패` 가 된다.
  라이다 · GNSS 는 장비 하나가 곧 센서군이라 행의 점이 카드와 같다.
- 카드의 체크 = 이번에 띄울 대상. 감지할 때 **보이는 종류는 자동으로 체크**되고, 손으로 바꾼 건 그대로 둔다.
  실행 중에는 체크를 못 바꾼다 (카메라는 한 프로세스라 재기동이 필요하다).
- 기동한 센서가 전부 올라오면 툴바에 초록 **[모두 실행 중 · 녹화로 →]** 버튼이 뜨고, 녹화 탭으로 넘어간다.
- **칸 경계는 전부 끌어서 크기를 바꿀 수 있다** — 카드 목록 ↔ 상세, 상세 ↔ 로그, 녹화 탭의 박스들 전부.
  위치는 `last_session.yaml`의 `ui.splitters`에 저장돼 다음 실행에도 유지된다.
- 화면이 작으면(예: 800x600) 창이 화면에 맞게 줄어들고, 설정은 세로로만 스크롤된다.

**감지** — 센서마다 찾는 방법이 다르다. 전부 읽기만 하고 아무것도 바꾸지 않는다.

| 센서 | 방법 |
|---|---|
| 가시광 Blackfly / 열화상 A70 | GVCP 디스커버리를 **NIC마다** 제한 브로드캐스트(255.255.255.255)로 쏜다 (root 불필요) |
| Ouster | 설정에 적힌 주소로 TCP 80 연결 → `api/v1/sensor/metadata` 로 시리얼·펌웨어. 그리고 센서가 스스로 알리는 **mDNS `_roger._tcp` 를 모든 NIC 에서** 듣는다 (`avahi-browse`) — 라이다를 다른 NIC 로 옮겨 꽂아도 어디 붙었는지 안다 |
| GNSS RT2000 | UDP 3000을 잠깐 듣는다 (NCOM 브로드캐스트, 첫 바이트 `0xE7`) |

**라이다를 USB 이더넷 어댑터 등 새 NIC 에 꽂았을 때** — Ouster 는 DHCP 가 없는 link-local(169.254.x.x) 장비다.
새 NIC 에는 NetworkManager 가 기본 DHCP 프로필을 걸어 45초 기다리다 실패하고 연결을 끊기를 되풀이한다 (IPv6 주소까지
사라진다). 그래서 PC 가 라이다에 영영 못 붙고, `169.254.0.0/16` 경로도 엉뚱한 NIC(카메라망)로 나 있다.

- 감지가 이 상황을 알아본다: mDNS 로 라이다가 보이는데 그 NIC 에 PC 쪽 IPv4 가 없으면 **"PC 쪽 NIC 설정 필요"**,
  라이다가 아예 안 보이면 **링크는 올라왔는데 IPv4 가 없는 유선 NIC** 를 후보로 보여준다.
- Ouster 카드의 **[<NIC> NIC 설정]** (또는 툴바 **IP 자동 맞춤**이 켜져 있으면 기동할 때 자동)이 그 NIC 에
  NetworkManager 프로필 `LiDAR link-local (<NIC>)` 를 만든다 (`ipv4.method link-local`, 기본 경로 없음,
  autoconnect-priority 50 — 기본 DHCP 프로필보다 우선). sudo 는 필요 없다 (데스크톱 세션 사용자는 polkit 허용).
  NIC 가 `169.254.x.y/16` 을 받으면 그 경로(metric ~100)가 카메라망의 link-local 경로(metric 1000)보다 앞서
  라이다로 간다. 다음에 꽂을 때부터는 NetworkManager 가 알아서 이 프로필을 쓴다.
  되돌리기: `nmcli connection delete 'LiDAR link-local (<NIC>)'`.
- 찾은 라이다 주소가 설정(`sensor_hostname`)과 다르면 **그 기동에서만** 찾은 주소로 넘긴다 (로그에 `[GUI]` 줄).
- 2026-09-19 실측: USB 어댑터(`enxc84d44268cd7`, r8152, 1 Gb/s)로 OS-2-128 1024x10 — points 9.9 Hz,
  NIC 수신 드롭 0.

**Blackfly와 A70은 따로 센다.** 같은 GigE 카메라이고 같은 런치로 뜨지만 모델도 노드맵도 다르므로
카드·대수·설정이 전부 갈려 있다.

**보이는 대수만 센다.** 리그 대수는 고정이 아니라서 "8대여야 하는데 몇 대 빠졌다" 같은 판정은 하지 않는다.
인벤토리(`multicam_cameras.yaml` / `multicam_thermal_cameras.yaml`)는 이름(namespace)을 붙이는 데만 쓴다.
인벤토리에 없는 새 카메라도 문제가 아니다 — 기동할 때 GUI 가 만드는 인벤토리 사본에
`camera_<시리얼>` / `thermal_<시리얼>` 이름으로 들어간다 (아래 "저장된 인벤토리와 …").
따로 표시하는 건 보이긴 하는데 이대로는 못 여는 카메라 두 가지다:
- **IP 안 맞음** — IP 미설정(새 카메라가 스스로 잡은 링크로컬 `169.254.x.x`), 호스트와 다른 서브넷,
  다른 카메라와 **IP 충돌**. 아래 IP 맞추기로 해결.
- **업데이터 모드** — GVCP 모델명이 `Updater` 로 오는 카메라 (펌웨어 업데이트가 중간에 끊긴 상태 등).
  카메라 전원을 다시 넣어야 한다. 종류는 인벤토리의 시리얼로 정해 해당 카드에 표시하고, 기동에서는 뺀다.
GigE 감지는 유선 NIC만 훑는다 (WiFi 너머에 카메라가 있을 일은 없다).

> 예전 감지는 서브넷 브로드캐스트(`192.168.1.255`)로만 물어봐서, IP 가 아직 없는 새 카메라(169.254.x.x)는
> 못 들었다 — 8대 꽂았는데 6대로 보이던 원인. 지금은 Spinnaker 처럼 제한 브로드캐스트로 묻고, 다른 서브넷
> 장비가 브로드캐스트로 돌려주는 답도 받는다. 결과도 IP 가 아니라 MAC 으로 묶어서, 두 대가 같은 IP 를 써도
> 둘 다 보이고 "IP 충돌"로 표시된다.

**IP 맞추기** — 장비 탭의 [IP 할당], 또는 툴바의 **IP 자동 맞춤**(기본 켬, 기동 직전에 자동으로):

- 한 대씩 GVCP `FORCEIP` 로 주소를 주고, 다시 찾아서 **정말 그 주소로 바뀌었는지 확인**한다 (2번까지 재시도).
  한 대씩 하는 이유: A70 두 대를 동시에 바꾸다 같은 IP 에 둘 다 앉은 적이 있다.
- 주소는 인벤토리(또는 GUI)에 예약된 게 있으면 그것, 없으면 `force_ip_base`(가시광 `.1`, 열화상 `.11`)부터 빈 주소.
  **두 인벤토리에 예약된 주소는 지금 안 꽂힌 카메라 것이라도 피한다** (나중에 그 카메라를 꽂았을 때 충돌 방지).
  주기 직전에 그 주소로 ping 이 오면(감지 안 되는 다른 장비) 건너뛴다.
- 준 주소는 GUI 설정(`cameras.<시리얼>.force_ip_address`)에 남고 인벤토리 사본에도 실린다.
- ForceIP 는 **임시**다 — 카메라 전원을 다시 넣으면 풀린다. 그러면 다시 감지에서 "IP 안 맞음"으로 뜨고,
  자동 맞춤이 같은 주소로 다시 맞춘다. 실행 중인 카메라군은 건드리지 않는다.

**설정** — 설정 탭은 위에서 아래로 세 단이다. 값은 **그 종류의 모든 카메라에 일괄 적용**된다
(런치가 params 파일 하나를 모든 카메라에 먹이는 구조라서 그렇다).

```
[ 설정 검색 — exposure, 대역폭, ptp …        ]  ● 원본과 다른 값 2개  [모두 원본값으로]
┌ 주요 설정 — 가시광 Blackfly ────────────────────────────────────────┐
│ 영상        픽셀 포맷 · 자유 실행 프레임레이트 · PTP 동기 프레임레이트 │
│ 노출 · 게인  노출 auto · 노출 시간 · 게인 auto · 게인                  │
│ 네트워크    카메라당 링크 대역폭 · GigE 패킷 크기                      │
│ 출력        RGB 압축 · JPEG 품질 · image_raw                          │
│ 호스트 처리 ⚠ 디모자이킹 · ⚠ 수신 버퍼 처리                           │
├ 카메라 공통 옵션 (런치 인자) ── 순차 기동 · PTP grandmaster NIC · … ──┤
전체 설정   flir_camera.yaml 의 모든 키와 런치 인자        모두 펼치기 · 모두 접기
│ ▸ Base node configuration (5)   ▸ Exposure (1)   ▸ White balance (5) …│
│ ▸ 런치 인자 — multicam.launch.py (31)                                 │
│ ▸ 직접 추가 — 원본에 없는 키                                          │
```

- **주요 설정** — `sensors.yaml` 의 `fields` 에 한국어 이름·단위·범위·설명과 함께 올린 키. 자주 바꾸고
  잘못 바꾸면 위험한 것들이다. 칸에 마우스를 올리면 왜 그 값인지가 나온다. ⚠ 가 붙은 디모자이킹
  (`color_processing`)·수신 버퍼(`buffer_handling_mode`)는 리그에서 가장 빠듯한 CPU 예산과 얽혀 있다 —
  `hq_linear` 는 예산을 넘겨 `NewestOnly` 가 늦은 프레임을 경고 없이 버렸다 (카메라마다 1.7~3.0% 손실).
- **전체 설정** — 원본 params YAML 을 **파일에서 직접 읽어** 모든 키를 보여준다 (`scripts/param_doc.py`).
  파일의 `# ---- 제목 ----` 머리와 빈 줄 뒤 소제목이 접히는 상자가 되고, 키 바로 위 주석·줄 끝 주석이
  툴팁이 되고, 주석의 `Off | Once | Continuous` / `{NORMAL, EXTENDED}` / `- LEGACY` 목록은 고르기 칸이 된다.
  리포 YAML 에 키가 늘면 GUI 도 그대로 늘어난다.
  - **회색 키** = 원본에서 주석 처리된 키 (`# camera.GammaEnable: false`). 예시값이 채워져 있고 **[지정]** 을
    켜야 파일에 실린다. 끄면 키를 아예 넣지 않는다 (카메라에 저장된 값 그대로).
  - **잠긴 칸** = 런치나 GUI 가 따로 정하는 키. 바꿔 봐야 소용없어서 잠그고 이유를 툴팁에 단다 —
    `camera_serial` · `frame_id` · `ptp_action.role`(런치가 카메라마다 덮어씀), `network.force_ip.*`(GUI 가
    기동 전에 맞춤), `params_file` · `enable_*_cameras` · `auto_update_*`(GUI 가 넘김) 등.
    목록은 `sensors.yaml` 의 `managed_params` / `managed_launch_args`.
  - **런치 인자 전부** — 런치 파일의 `DeclareLaunchArgument` 를 ast 로 읽는다 (실행하지 않는다).
  - 숫자·목록은 글자 칸이다 (범위를 모른다 — `group_mask` 는 2^32-1). 숫자가 아닌 걸 적으면 칸이 빨개지고
    저장하지 않는다. ROS 파라미터는 타입이 엄격해서 **원래 타입으로 되돌려 넣는다** (`18000` → `18000.0`,
    `[0, 0.1]` → `[0.0, 0.1]`).
- **직접 추가** — 원본에 없는 키. 카메라 노드는 `camera.*` / `stream.*` / `tl_device.*` 를 GenICam 노드
  이름으로 그대로 적용하므로 YAML 에 없는 노드도 넣을 수 있다. 값은 YAML 로 (`1.0` · `true` · `"Off"`).
- **검색** — 이름·키·설명·섹션 제목에서 찾는다 (띄어 쓰면 모든 단어). 맞는 줄만 남기고 상자를 펼친다.
  아무것도 없으면 그 글자를 '직접 추가' 칸에 넣어 준다.
- 목록을 휠로 내리다가 지나가는 콤보·스핀 칸의 값이 바뀌지 않는다 — 칸을 한 번 눌러야 휠이 먹는다.

조건이 붙은 필드(노출 시간은 노출 auto = Off 일 때만, 게인은 게인 auto = Off 일 때만, 고정 간격 기동은
순차 기동이 꺼졌을 때만)는 조건이 안 맞으면 폼에서 회색이 될 뿐 아니라 **생성 파일·런치 인자에서도 빠진다**
— 남겨 두면 카메라 노드가 "node is not writable in the current camera state" 로 죽는다 (노출 auto = Once 로
두었을 때 12대가 실제로 그랬다).

원본 params YAML(`flir_camera.yaml` 등)은 **절대 수정하지 않는다.** 왜 그 값인지가 파일 안 주석에
기록돼 있는 것들이 있기 때문이다 (`color_processing: ipp`, `buffer_handling_mode` 등).
대신 원본 전체 + 바꾼 키만 덮어쓴 **사본**을 만들어 런치에 넘긴다:

```
~/.config/dm_clip_gui/sensors/flir_cameras__visible.yaml
~/.config/dm_clip_gui/sensors/flir_cameras__thermal.yaml
~/.config/dm_clip_gui/sensors/ouster.yaml
~/.config/dm_clip_gui/sensors/gnss_rt2000.yaml
```

매 기동마다 원본을 다시 읽어 만들므로 리포의 원본이 바뀌면 그대로 따라간다.
원본과 다른 값은 폼에서 **굵게**(● 표시) 나오고, 툴팁에 원본 값이 뜬다. 원본과 같은 값으로 되돌리면
오버라이드에서 빠진다 — 그래야 리포 원본이 바뀌었을 때 그대로 따라간다. 상자 머리와 탭 제목에 바꾼 개수가 뜬다.

**[기본값으로 저장]** (설정 탭) — GUI 에서 바꾼 값을 **리포 원본에 써서 아예 기본값으로** 만든다. GUI 없이
`ros2 launch` 로 띄워도 같은 값이 되고, '원본과 다른 값' 은 0 이 된다 (`scripts/defaults_writer.py`).

| 무엇 | 어디에 | 어떻게 |
|---|---|---|
| params | 원본 params YAML (`flir_camera.yaml` 등) | 그 키의 줄만 고친다 — 주석 · 순서 유지. 주석 처리된 키는 주석을 풀고, '지정 안 함' 은 주석 처리, 없던 키는 끝에 모아 붙임 |
| 런치 옵션 (레지스트리 default 가 있는 것) | `config/sensors.yaml` 의 그 필드 `default` | GUI 는 그 값을 원본으로 본다 |
| 그 밖의 런치 인자 | 런치 파일의 `DeclareLaunchArgument(default_value=…)` | 글자 상수일 때만 (경로 계산식이면 거절) |

params 는 GUI 가 실제로 넘기는 사본과 원본의 차이를 옮긴다 — 그래서 딸린 설정도 같이 간다 (프레임레이트를
지정하면 `AcquisitionFrameRateEnable: true`, 노출 auto 를 켜면 수동 `ExposureTime` 줄은 주석 처리 — 안 그러면
GUI 없이 띄울 때 노드가 "not writable" 로 죽는다). 누르면 바뀌는 파일 · 값을 보여주고 확인을 받는다. 쓰기 전에
원본을 `~/.config/dm_clip_gui/backups/` 에 복사하고, 쓴 결과를 다시 읽어 값이 맞을 때만 저장한다 (git 으로도 되돌릴 수 있다).

**[이름·역할을 인벤토리에 저장]** (감지된 장비 탭) — GUI 에서 정한 카메라 이름 · PTP 역할 · IP 를 리포 인벤토리
(`multicam_cameras.yaml` / `multicam_thermal_cameras.yaml`)에 쓴다. 인벤토리에 없던 카메라는 MAC 과 함께 추가된다.

**빈 런치 인자** — `ros2 launch` 는 `key:=` (빈 값)를 malformed 로 거부해서 기동 자체가 실패한다. 그래서 칸을
비우면 레지스트리의 `empty_value` 로 넘기고(`ptp_master_interface` 는 `none` → ptp4l 안 띄움), 그런 값이 없는
인자는 넘기지 않는다 (런치 기본값이 쓰인다는 `[GUI]` 로그를 남긴다).

**카메라별 설정 (이름 · PTP 역할)** — [감지된 장비] 탭의 표에서 정한다 (열 순서: 이름 · 시리얼 · IP ·
PTP 동기 · 모델 · NIC). 이름은 더블클릭 또는 오른쪽 라이브 칸, PTP 역할은 콤보.
**열 제목을 누르면 그 열로 정렬**(▲ 오름차순), 한 번 더 누르면 ▼ 내림차순, 또 누르면 ▲… 이름·시리얼은
숫자를 숫자로 비교하고(`front_right2` < `front_right10`), IP 는 주소 순이다. 정렬은 카드마다
`ui.device_sort` 에 저장돼 다음 실행에도 유지된다.

**어떤 그림이 어떤 카메라인지 보며 이름 짓기** — 카메라군이 실행 중이면 표에서 행을 고를 때 오른쪽에
**그 카메라의 라이브**(그림 + 실측 Hz)가 나온다. 고른 한 대만 구독하고(8 fps 로 디코드), 다른 카드·탭으로
가면 구독을 내린다. 실행 중에도 이름을 바꿀 수 있고 **다음 기동부터** 적용된다 (라이브 칸에
"지금은 /back_left/ 로 발행 중 — 다음 기동부터 /rear_left/" 처럼 표시). 실행 전이거나 이번 기동에 없는
카메라도 이름은 미리 정해 둘 수 있다. 값은 GUI 설정(`last_session.yaml`의 `sensors.<군>.cameras.<시리얼>`)에 저장되고
**리포 인벤토리 YAML 은 바뀌지 않는다.** 우선순위는 GUI 에서 정한 값 > 리포 인벤토리 > 자동 이름
(`camera_<시리얼>` / `thermal_<시리얼>` — 꽂힌 조합과 무관하게 고정). 이름은 ROS 네임스페이스로 쓰이므로
영문자로 시작하는 영문·숫자·밑줄만 되고, 센서군 안에서 겹치면 거부된다. 실행 중에는 잠긴다.

**저장된 인벤토리와 지금 꽂힌 카메라가 다를 때** — 기동할 때마다 **지금 보이는 카메라만** 담은 인벤토리
사본을 만들어 `cameras_file` / `thermal_cameras_file` 로 넘긴다 (`~/.config/dm_clip_gui/sensors/*__inventory.yaml`).
저장된 인벤토리를 그대로 쓰면 두 가지가 터진다:

- 꺼져 있는 카메라까지 노드가 떠서 `camera_serial was not found` 로 죽는다 (`camera_start_stagger` 만큼씩 기동도 늦어짐).
- **안 보이는 카메라가 PTP sender 면, 보이는 receiver 는 오지 않을 트리거를 기다리며 프레임이 0장이다**
  (`Failed waiting for EventData on NEW_BUFFER_DATA` 만 반복). 그래서 보이는 카메라 중 sender 가 없으면
  첫 카메라를 **이번 기동에서만** sender 로 쓴다. sender 가 둘 이상이면 하나만 남긴다.
  GPIO 트리거는 배선이 필요한 역할이라 master 로 올리지 않고, master 가 없으면 slave 를 자유 실행으로 띄운다.

한 대도 안 보이는 종류는 기동에서 빠지고, 못 여는 카메라(IP 안 맞음 · 업데이터 모드)도 뺀다
(IP 안 맞음은 툴바의 IP 자동 맞춤이 기동 전에 고친다).
무엇을 빼고 무엇을 바꿨는지는 런치 로그 맨 위에 `[GUI]` 줄로 남는다. 사본을 넘길 때 런치의 인벤토리 자동 갱신은
끈다(`auto_update_cameras_file:=false` 등) — 감지·IP·이름·sender 는 GUI 가 하고, 켜 두면 GUI 가 일부러 뺀
카메라(예: 업데이터 모드)를 런치가 다시 넣어 노드가 죽는다. 리포 인벤토리도 기동할 때마다 늘어나지 않는다.

**기동** — 센서군당 자식 프로세스 하나:

```bash
bash -c 'source <ws1>; source <ws2>; exec ros2 launch <pkg> <launch> arg:=val ...'
```

- 셸에서 소싱해 둔 워크스페이스가 새어 들어오지 않게 경로 변수(`AMENT_PREFIX_PATH`, `PYTHONPATH`,
  `LD_LIBRARY_PATH` 등)를 비우고 `sensors.yaml` 이 선언한 워크스페이스만 소싱한다. `~/.bashrc` 가
  `~/flir_ouster_ws` 를 소싱하고 있어서, 안 비우면 `ouster_ros` 가 언더레이(0.14.2)가 아니라 그 빌드(0.14.0)로 뜬다.
  `ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY` 는 그대로 넘긴다 (레코더와 같아야 서로 보인다).
- **기동 완료 판정**은 이번에 띄운 카메라의 네임스페이스마다 토픽이 떴는가로 본다 (이름을 바꿔도 정확히 맞는다).
  **발행자가 있는 토픽만** 센다. 예전에는 구독만 있는 토픽도 셌는데, 한 번 띄웠다 내리면 레코더·미리보기가 그
  토픽을 계속 구독하고 있어서 다시 기동하자마자 "실행 중"이 되고 녹화 탭으로 넘어가 버렸다 (GUI 를 껐다 켜야
  정상이 되던 증상). 기동 직전에 이미 있던 발행자(강제 종료된 노드는 DDS 임대 시간 ~20초 동안 남는다)도 세지 않는다.
- 모든 센서군에 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 강제한다. `~/flir_ouster_ws`는 cyclonedds가
  기본이라 섞이면 레코더가 토픽을 아예 못 찾는다.
- **기동 완료 판정**은 "토픽이 하나 떴다"가 아니라 **감지된 대수만큼 네임스페이스가 떴는가**다
  (서브넷 불일치 장비는 열 수 없으니 세지 않는다). 카메라는 한 대씩 올라오므로, 첫 대만 보고 완료라고 하면
  나머지가 아직 Init 중인데 녹화가 시작된다. 상태 알약에 **`기동 중 5/15`** 처럼 몇 대째인지 나온다.
  제한 시간은 전체가 아니라 **진척 기준**이다 — 90초 동안 한 대도 새로 안 뜨면 `실패`로 표시하고
  무엇이 모자란지 로그에 남긴다 (순차 기동은 15대에 2분 반이 걸려서 전체 90초 제한이면 멀쩡한데 실패가 된다).
  실패 뒤에라도 다 뜨면 `실행 중`으로 바뀐다.
- 종료는 SIGINT (`ros2 launch`가 자식 노드를 정리하는 유일한 신호) → 10초 기다린 뒤, 런치가 내려간 뒤에도
  남은 자식 노드까지 찾아 정리한다 (카메라를 쥔 채 고아로 남는 노드가 없게). 여러 센서군을 한꺼번에 끌 때
  (창 닫기 · Ctrl+C) 는 SIGINT 를 동시에 보내고 기다린다 — 하나씩 하면 센서군마다 종료 시간이 쌓인다.
  런치는 `setsid` 로 따로 세션에 뜬다 (§11 끄기).

**순차 기동** (카메라 공통 옵션, 기본 켬 — `multicam.launch.py sequential_start:=true`) — 앞 카메라 노드가
`Camera acquisition started` 를 찍은 뒤에 다음 노드를 띄운다. 이유: 카메라 노드마다 리그 전체를 열거하는데
(15대면 약 13초) 그동안 모든 카메라를 쥐고 있어서, 고정 간격(`camera_start_stagger`)으로 띄우면 자기 카메라를
설정하던 노드가 `-1005`/`-1010` 로 밀려 죽었다 (15대 기동에서 front1 · thermal_right · thermal_left 가 매번).
한 대가 `sequential_start_timeout`(45초) 안에 안 뜨거나 죽으면 기다리지 않고 다음 대로 넘어간다.

| 가시광 13 + 열화상 2 | 고정 간격 3초 (이전 기동 로그) | 순차 기동 (2026-09-19 실측) |
|---|---|---|
| 기동한 카메라 | 12/15 (3대 죽음) | **15/15** |
| 마지막 카메라까지 | 약 1분 | **148초** (대당 7~13초, 뒤로 갈수록 빨라짐) |
- 카메라는 가시광·열화상이 **한 프로세스**다. 따로 띄우면 양쪽이 `ptp4l`을 각각 올려 같은 NIC에서
  충돌하고, ForceIP 시퀀스가 동시에 돌아 서로의 IP를 덮어쓴다 (A70 두 대가 같은 IP에 앉아 스트림이
  죽은 적이 있다). UI의 "띄울 대상" 체크박스가 `enable_visible_cameras` / `enable_thermal_cameras`로 간다.
- 센서군을 프로세스로 갈라둔 덕에, 라이다가 센서에 못 붙어 `launch.events.Shutdown`을 쏴도
  그 폭발이 라이다 프로세스 안에 갇힌다 (한 런치에 합치면 카메라까지 같이 죽는다).

**이미 떠 있는 레코더에 붙을 때** — 토픽을 아직 안 고른 상태(빈 목록)면 그 레코더의 토픽 설정을
건드리지 않는다. 빈 목록을 보내면 레코더는 "전체 녹화"로 받아들여 기존 선택을 덮어쓰기 때문이다.

**센서 목록을 고치려면** `config/sensors.yaml` 하나만 고친다 — 소싱할 워크스페이스, 런치, 감지 방법,
주요 설정에 올릴 키, 잠글 키가 전부 거기 있다. 나머지 설정은 원본 YAML·런치 파일에서 읽으므로
코드는 건드릴 필요가 없다.

**단독 실행** (GUI 없이 감지/설정만 확인)
```bash
python3 scripts/sensor_discovery.py              # 전체 감지
python3 scripts/sensor_discovery.py flir_cameras # 센서군 하나만
python3 scripts/sensor_config.py                 # params 경로 / 런치 인자 확인
python3 scripts/param_doc.py <params.yaml | *.launch.py>   # '전체 설정' 이 파일에서 읽는 것 (섹션 · 주석 키 · 선택지)
```

## 13. 센서 미리보기 — [녹화] 탭 오른쪽

센서마다 작은 타일: 최근 그림(카메라 · 열화상 · 라이다) 또는 값(GNSS) + **실측 Hz**.
종류별 섹션(가시광 / 열화상 / LiDAR / GNSS) 머리에 대수 · 평균 Hz · 끊김 수가 나온다.

| 센서 | 그림 | Hz |
|---|---|---|
| 가시광 (BFS) | `image_rgb/compressed` **스냅샷** | `camera_info` (프레임마다 오는 작은 메시지) |
| 열화상 (A70) | `image_raw` mono16 스냅샷 → 컬러맵 + 중앙/최소/최대 °C | `camera_info` |
| LiDAR | `signal_image` 계속 (한 대, 256 KB × 10 Hz) | 같은 토픽 |
| GNSS | `nav_status` · `pos_type` · `fix` · `vel` 값 | `nav_status` (위치 해가 없어도 NCOM 이 흐르면 뜬다) |

**[그림] 설정으로 예산을 고른다** (기본 `실시간 5 fps`):

| 설정 | 동작 | 2026-09-19 실측 (가짜 리그 14+2대 30 Hz, 보이는 타일 약 8개) |
|---|---|---|
| 실시간 15 / 10 / 5 fps | 화면에 보이는 카메라 타일만 이미지를 **계속** 받고, 초당 N장 최신 것만 디코드 | GUI 92% / 83% / 76% (한 코어 기준), 발행 쪽 +2% |
| 1초 · 빠르게 2초 · 보통 4초 · 느리게 8초 | **스냅숏**: 구독을 만들어 한 장 받고 바로 내림, 동시에 최대 4개 | GUI 51% / … / 49% |

- 예전엔 스냅숏(최대 8초 · 동시 2개)만 있었다. 카메라 노드마다 리더가 하나 더 붙는 게 부담이었는데 — 그땐 카메라
  노드의 디모자이킹 + JPEG 이 CPU 를 거의 다 먹었다. 이제 그 일을 GPU 가 하고 점보 프레임으로 커널 수신도 줄어
  (CPU 유휴 3.5% → 77%) 실시간이 기본이다. 예전 설정(느리게 등)은 한 번 `실시간 5 fps` 로 올리고, 그 뒤 고른 값은 그대로 둔다.
- GUI 는 파이썬이라 한 코어가 한계다 — 15 fps 는 GUI 가 한 코어의 90% 를 넘어 반응이 굼떠질 수 있다.
  "느리게" 에서도 약 49% 인 건 Hz 용 `camera_info`(16대 × 30 Hz) 수신 몫이다.
- Hz 는 모드와 상관없이 실시간이다 (`camera_info` 는 수백 바이트).
- 받은 건 직렬화된 바이트 그대로 두고, 그릴 때만 디코드한다. JPEG 은 디코드 단계에서
  줄인다 (`QImageReader.setScaledSize` — libjpeg DCT 스케일링).
- 녹화 탭이 안 보이거나 [켜기]를 끄면 구독을 전부 내린다 — 데이터가 아예 안 흐른다. 스크롤해서 안 보이는
  타일은 실시간이어도 받지 않는다.
- 구독 생성/해제는 전부 ROS 스레드에서 한다 (`RosWorker.call_in_ros_thread`).

타일 크기(작게/보통/크게)는 고를 수 있고, LiDAR 띠와 GNSS 는 한 줄 전체를 쓴다.
14대면 "작게"가 한눈에 들어온다. 설정은 `ui.preview_*` 로 저장된다.

**순서 바꾸기** — 타일을 끌어서 같은 섹션 안의 다른 타일 위에 놓는다 (오른쪽 절반이면 그 뒤로).
순서는 `ui.preview_order` 에 **카메라 시리얼 기준**(`cam:<시리얼>`)으로 저장되어 다음 실행에도 유지된다.
센서가 바뀌어도:
- 이름을 바꿔도 시리얼이 같으니 자리가 그대로다.
- 잠깐 뺀 카메라는 저장된 순서에 남아 있다가, 다시 꽂으면 원래 바로 앞에 있던 카메라 뒤로 돌아온다.
- 새 카메라는 섹션 끝에 붙는다. [순서 초기화] 는 이름순으로 되돌린다.

## 파일

```
clip_recorder/
├── src/clip_recorder.cpp          레코더 노드 (~/clip_event 로 클립 라이프사이클 발행)
├── src/net_probe.c                송신 IP별 트래픽 카운터 (setcap 필요, §11)
├── scripts/clip_gui.py            GUI (ros2 run clip_recorder clip_gui)
├── scripts/clip_launch.sh         `clip` 런처: net_probe 권한 확인/부여 후 GUI 실행
├── scripts/bag_diagnostics.py     클립 진단 엔진 (드랍/손상/GNSS) — CLI 겸용
├── scripts/gnss_tools.py          GNSS 품질 + 궤적 캐시 + PNG 지도
├── scripts/map_widget.py          인터랙티브 OSM 슬리피 맵 위젯 (순수 PyQt5)
├── scripts/net_tools.py           NIC 통계 + GigE Vision 디스커버리
├── scripts/sensor_stage.py        [센서 기동] 탭 UI (§12)
├── scripts/ui_theme.py            공통 테마 (Fusion + 라이트 팔레트 + QSS, 화면 크기 맞춤)
├── scripts/preview_panel.py       [녹화] 탭 센서 미리보기 (스냅샷 + 실측 Hz, §13)
├── scripts/param_doc.py           원본 params YAML · 런치 파일에서 '전체 설정' 을 뽑는 파서 (§12)
├── scripts/defaults_writer.py     [기본값으로 저장] — GUI 의 차이를 원본 YAML · 런치 파일 · 인벤토리에 쓴다 (§12)
├── scripts/sensor_discovery.py    센서 감지 (GVCP / Ouster TCP / NCOM) — CLI 겸용
├── scripts/sensor_config.py       원본 params + 오버라이드 -> 생성 params 파일 — CLI 겸용
├── scripts/sensor_launcher.py     센서군 런치 프로세스 + 기동 완료 판정
├── scripts/buffer_probe.py        버퍼 필요량 측정 툴 (ros2 run clip_recorder buffer_probe)
├── scripts/clip_trigger.py        키보드 트리거 (ros2 run clip_recorder clip_trigger)
├── config/params.yaml             파라미터 (주석 참고)
├── config/sensors.yaml            센서군 레지스트리 (센서 추가/교체 시 여기만 수정, §12)
├── config/fastdds_shm.xml         Fast DDS SHM 프로파일 (대형 이미지 무손실 전송, §10)
├── launch/clip_recorder.launch.py
├── CMakeLists.txt / package.xml
└── README.md
```
