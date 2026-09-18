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
GUI는 파라미터·토픽으로 조종한다 (GUI가 죽어도 녹화는 안 죽고, 이미 떠 있는 외부 레코더에 붙을 수도 있다).

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
   - 링 버퍼: 메모리/보관 초/유입 + **디스크 여유** (현재 유입 기준 클립 몇 개 더 저장 가능한지, 부족 시 경고)
   - 네트워크: 스위치 업링크 NIC 합계 (x / 10 Gbps) + **송신 IP별** Mbps·포트 사용률·pps.
     카메라는 GigE Vision 디스커버리 + 드라이버 `camera_serial`로 이름이 자동으로 붙고, 그 외 IP는
     목적지 포트로 정체를 추정한다 (OxTS NCOM 3000, PTP, DDS, Ouster/Velodyne 라이다 포트 등 →
     `? UDP/3000 브로드캐스트` 식). **장치 칸을 더블클릭하면 이름을 직접 지정**할 수 있다 (설정에 저장).
     `127.0.0.1`/이 PC 주소는 자동으로 "이 PC"로 표시. 5분 이상 조용한 IP는 목록에서 빠진다.
   - GNSS: pos_type / nav_status / fix 상태 / 위치 / 수평정확도 / 위성 수·HDOP(GGA) 실시간.
   - 토픽별 Hz와 대역폭(B/s·KB/s·MB/s 자동 단위), 하단 로그창 (레코더 /rosout + 클립 진행 상황).
4. **클립 저장 완료 → 자동 진단** (`bag_diagnostics`) → 결과 창 + `<클립>/diagnostics.txt/.json` 저장.
   GNSS 품질(fix 비율, 정확도, 점프, 공백)도 포함되고 궤적은 `<클립>/gnss_track.json`에 캐시.
5. **궤적 지도 (누적)**: 여러 클립을 체크해서 OSM 위에 겹쳐 본다. 진단이 끝난 클립은 자동으로 추가·체크.
   지도는 인터랙티브(휠 확대/축소, 드래그 이동, 전국~골목 z3~19, 순수 PyQt5 `map_widget.py` — QtWebEngine 불필요).
   타일은 인터넷에서 받아 `~/.cache/dm_clip_gui/tiles`에 캐시 (오프라인이면 캐시된 곳만 보임). PNG 내보내기 가능.
6. **GNSS · 지도 창** (상태 + 지도 + 클립 궤적이 한 창): 솔루션(pos_type / GGA 품질: RTK FIXED·FLOAT·DGPS·SPS),
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

**감지** — 센서마다 찾는 방법이 다르다. 전부 읽기만 하고 아무것도 바꾸지 않는다.

| 센서 | 방법 |
|---|---|
| 가시광 Blackfly / 열화상 A70 | GVCP 디스커버리 브로드캐스트를 **NIC마다** 쏜다 (root 불필요) |
| Ouster | 설정에 적힌 주소로 TCP 80 연결 → `api/v1/sensor/metadata` 로 시리얼·펌웨어 |
| GNSS RT2000 | UDP 3000을 잠깐 듣는다 (NCOM 브로드캐스트, 첫 바이트 `0xE7`) |

**Blackfly와 A70은 따로 센다.** 같은 GigE 카메라이고 같은 런치로 뜨지만 모델도 노드맵도 다르므로,
대수·상태·설정이 전부 갈려 있다 — `열화상 A70 2/2 · 가시광 Blackfly 0/8 (8대 안 보임)` 처럼.
합쳐서 `2/10` 이라고 하면 어느 쪽이 빠졌는지 안 보인다.

감지 결과는 인벤토리(`multicam_cameras.yaml` / `multicam_thermal_cameras.yaml`)의 시리얼과 대조해
네 가지로 나온다: `정상` / `설정에 없음`(새로 꽂은 장비) / `안 보임`(전원·케이블) /
`서브넷 불일치`(ForceIP 필요).

**설정** — 센서군마다 자주 만지는 키만 폼으로 노출한다 (프레임레이트, 노출/게인, A70 `IRFrameRate`·
`IRFormat`, 라이다 `lidar_mode`·`timestamp_mode`, GNSS 포트 등). 값은 **그 센서군의 모든 카메라에
일괄 적용**된다 — 런치가 params 파일 하나를 모든 카메라에 먹이는 구조라서 그렇다.

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
원본과 다른 값은 폼에서 **굵게** 표시되고, 툴팁에 원본 값이 뜬다.

**기동** — 센서군당 자식 프로세스 하나:

```bash
bash -c 'source <ws1>; source <ws2>; exec ros2 launch <pkg> <launch> arg:=val ...'
```

- 모든 센서군에 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 강제한다. `~/flir_ouster_ws`는 cyclonedds가
  기본이라 섞이면 레코더가 토픽을 아예 못 찾는다.
- **기동 완료 판정**은 "토픽이 하나 떴다"가 아니라 **감지된 대수만큼 네임스페이스가 떴는가**다.
  카메라는 `camera_start_stagger` 만큼 한 대씩 올라오므로, 첫 대만 보고 완료라고 하면 나머지가
  아직 Init 중인데 녹화가 시작된다. 90초 안에 못 채우면 `실패`로 표시하고 무엇이 모자란지 로그에 남긴다.
- 종료는 SIGINT → 5초 후 SIGKILL (`ros2 launch`가 자식 노드를 정리하는 유일한 신호).
- 카메라는 가시광·열화상이 **한 프로세스**다. 따로 띄우면 양쪽이 `ptp4l`을 각각 올려 같은 NIC에서
  충돌하고, ForceIP 시퀀스가 동시에 돌아 서로의 IP를 덮어쓴다 (A70 두 대가 같은 IP에 앉아 스트림이
  죽은 적이 있다). UI의 "띄울 대상" 체크박스가 `enable_visible_cameras` / `enable_thermal_cameras`로 간다.
- 센서군을 프로세스로 갈라둔 덕에, 라이다가 센서에 못 붙어 `launch.events.Shutdown`을 쏴도
  그 폭발이 라이다 프로세스 안에 갇힌다 (한 런치에 합치면 카메라까지 같이 죽는다).

**센서 목록을 고치려면** `config/sensors.yaml` 하나만 고친다 — 소싱할 워크스페이스, 런치, 감지 방법,
폼에 노출할 키가 전부 거기 있다. 코드는 건드릴 필요가 없다.

**단독 실행** (GUI 없이 감지/설정만 확인)
```bash
python3 scripts/sensor_discovery.py              # 전체 감지
python3 scripts/sensor_discovery.py flir_cameras # 센서군 하나만
python3 scripts/sensor_config.py                 # params 경로 / 런치 인자 확인
```

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
