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
└── clip_20260826_153920_hard_brake/     # clip_YYYYmmdd_HHMMSS[_라벨]
    ├── clip_20260826_153920_hard_brake_0.db3
    └── metadata.yaml
```

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
3. **연속 로깅과 클립은 다른 일**: 전부 남겨야 하면 연속 녹화(`ros2 bag record` 또는 별도 레코더)를 따로
   돌리고, 이 클립 레코더는 "사건 하이라이트" 용도로 같이 둔다. 클립 레코더의 RAM 링은 디스크 속도와
   무관하게 안 잃지만 애초에 10초짜리다.

### 설정을 믿지 말고 세라

드랍이 없는지는 측정으로만 알 수 있다.

- bag 메시지 수 vs 기대값(주파수 × 시간): `ros2 bag info`
- `header.stamp` 간격이 프레임 주기의 2배 이상 벌어지는 곳 = 드랍 구간
- rosbag2 로그의 "dropped" 문구
- `ros2 topic hz` 로 레코더 PC에서 실제 수신 주파수 확인 (드라이버 주파수와 비교)

(bag을 넣으면 토픽별 드랍 수/구간을 뽑는 검사 툴은 추가 예정)

---

## 파일

```
clip_recorder/
├── src/clip_recorder.cpp          레코더 노드
├── scripts/buffer_probe.py        버퍼 필요량 측정 툴 (ros2 run clip_recorder buffer_probe)
├── scripts/clip_trigger.py        키보드 트리거 (ros2 run clip_recorder clip_trigger)
├── config/params.yaml             파라미터 (주석 참고)
├── launch/clip_recorder.launch.py
├── CMakeLists.txt / package.xml
└── README.md
```
