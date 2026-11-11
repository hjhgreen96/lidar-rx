# 아키텍처

이 문서는 저장소 전체가 어떻게 나뉘어 있고, 데이터가 어떤 모양으로 바뀌며 흘러가는지를 설명한다.
모듈 하나하나의 안쪽은 `docs/modules/`에 따로 있다.

- [1. 이 프로젝트가 하는 일](#1-이-프로젝트가-하는-일)
- [2. 계층별 전체 그림](#2-계층별-전체-그림)
- [3. 저장소 구조](#3-저장소-구조)
- [4. 크레이트와 의존 방향](#4-크레이트와-의존-방향)
- [5. 경계 세 개와 디자인 패턴](#5-경계-세-개와-디자인-패턴)
- [6. 데이터가 바뀌는 모양](#6-데이터가-바뀌는-모양)
- [7. 스레드 구조](#7-스레드-구조)
- [8. 시간 다루기](#8-시간-다루기)
- [9. 이상 상황을 다루는 방식](#9-이상-상황을-다루는-방식)
- [10. 테스트와 검증](#10-테스트와-검증)
- [11. 남은 것](#11-남은-것)

---

## 1. 이 프로젝트가 하는 일

- 회전형 LiDAR(Velodyne VLP-16): 초당 약 754개 UDP 패킷, 패킷 하나에 레이저 16개 × 24번 발사 = 거리값 384개
- 범위: 패킷이 로봇 컴퓨터의 리눅스에 들어와 ROS 2 포인트 클라우드 메시지로 발행될 때까지, 커널 수신 경로 포함

```text
 VLP-16 ──UDP──► [리눅스 커널] ──► [사용자 공간 드라이버] ──► [ROS 2] ──► RViz, 다른 노드
 (센서)           NIC·IP·UDP·소켓     해석·시각·프레임 조립       PointCloud2
                  + 관찰용 커널 모듈
```

센서는 없으므로, ROS 공식 서버의 실제 VLP-16 캡처 파일(pcap)을 원래 간격대로 다시 보내서(replay)
센서를 대신한다. 모든 결과는 정답과 대조한다: 참조 구현 두 개(공식 ROS 드라이버, `velodyne_decoder`),
센서 사양서의 타이밍 표, 정답을 넣어 보낸 드리프트, 커널 카운터.

## 2. 계층별 전체 그림

```text
 ┌─ 센서 대신 ───────────────────────────────────────────────────────────────────────────┐
 │  data/vlp16.pcap ──► pcap-replay: 원래 간격(1.327 ms) 또는 정해진 드리프트로 UDP 송신        │
 └──────────────────────────────────────────┬───────────────────────────────────────────┘
                                            │ loopback(WSL) 또는 Ethernet(보드)
 ┌─ 리눅스 커널 ─────────────────────────────▼───────────────────────────────────────────┐
 │  NIC → 드라이버 → IP ──► kmod/lidar_mon: IP 계층에서 패킷을 세고 손실 판정              │
 │                    → UDP → 소켓 수신 큐 → 커널 수신 시각·소켓 드롭 수 읽기               │
 │                                          실험 1~4 버퍼·지연·버릴 곳·스케줄링            │
 └──────────────────────────────────────────┬───────────────────────────────────────────┘
                                            │ recvmsg
 ┌─ 사용자 공간: lidar-rx ───────────────────▼───────────────────────────────────────────┐
 │  lidar-io    입력 출처 → Packet         UdpSource(실시간) / PcapSource(파일)            │
 │  vlp16       Packet → 포인트 → 한 바퀴 Frame                                            │
 │  clock       센서 시계 → 호스트 시계 추정 → Frame::stamp                                 │
 │  rx          수신 루프, 소비자마다 스레드 + 채널                                          │
 │   └ 소비자: StatsConsumer, Ros2Publisher (cargo feature ros2)                            │
 └──────────────────────────────────────────┬───────────────────────────────────────────┘
                                            │ ROS 2 토픽 /velodyne_points
 ┌─ 운영 ────────────────────────────────────▼───────────────────────────────────────────┐
 │  deploy/: systemd 서비스(수신, 녹화), 준비 알림·watchdog, sysctl, 모듈 자동 적재             │
 └───────────────────────────────────────────────────────────────────────────────────────┘
   보드: Arty Z7-20(Zynq-7000, ARM Cortex-A9), 크로스 빌드, 실제 NIC 수신 (ROS 2 없이 --forward로 PC에 전달)
```

## 3. 저장소 구조

```text
lidar-rx/
├── Cargo.toml              Rust workspace (크레이트 목록, rust-version 1.85, resolver 3)
├── .github/workflows/ci.yml  push·PR마다 fmt, clippy, 테스트 (10절)
├── crates/
│   ├── lidar-core/         경계(인터페이스)와 공통 타입만. 구현 없음
│   ├── vlp16/              VLP-16 해석기: 바이트 → 포인트 → 프레임
│   │   ├── tests/golden.rs     참조 구현(velodyne_decoder)과 포인트 단위 대조
│   │   └── examples/           inspect_pcap(프레임별 요약), count_points(패킷 묶음별 포인트 수)
│   ├── lidar-io/           입력 출처 구현: pcap 파일, UDP 소켓(리눅스)
│   ├── clock/              센서 시계 → 호스트 시계 추정 (하한선)
│   ├── pcap-replay/        [실행 파일] 센서 대신 보내기
│   └── rx/                 [실행 파일 lidar-rx] 모든 것을 조립해 돌림
├── kmod/                   [C 커널 모듈 lidar_mon] IP 계층에서 패킷을 셈, /proc/lidar_mon
├── deploy/                 systemd 서비스 두 개(수신, 녹화), 실행 인자, 모듈 자동 적재·sysctl 설정, 설치·제거 스크립트
├── testdata/
│   └── golden_vlp16.csv    참조 구현 출력 (패킷 0..152, 54,952 포인트)
├── tools/                  데이터·골든 만들기, 검증·실험 스크립트, 실행 기록, ROS 2 빌드, WSL 커널 빌드
├── docs/                   이 문서들, receive_path.md(수신 경로 정리), rust_notes.md(C와 비교), boot_flow.md(보드 부팅)
│   ├── modules/            모듈별 설명
│   ├── runs/               단계별 실행 기록 (명령과 실제 출력, 자동 생성)
│   └── media/              데모 영상, 단계별 터미널 영상, 실험 그래프
└── data/                   (git 제외) 샘플 pcap. tools/fetch_data.sh로 받음
```

## 4. 크레이트와 의존 방향

Rust에서 크레이트는 따로 컴파일되는 라이브러리(또는 실행 파일) 단위다. 크레이트 A가 B를 쓰려면
A의 `Cargo.toml`에 B를 적어야 하므로, 누가 누구를 아는지가 파일에 명시된다.

```text
                         ┌────────────────┐
                         │   lidar-core   │  Packet, PacketSource, SensorDecoder, Frame, 시각 타입
                         └──▲─────▲────▲──┘
              ┌─────────────┘     │    └─────────────┐
       ┌──────┴──────┐     ┌──────┴──────┐     ┌──────┴──────┐
       │  lidar-io   │     │    vlp16    │     │    clock    │
       │ pcap, UDP   │     │ 해석·조립    │     │ 시계 추정    │
       └──▲───────▲──┘     └──▲───────▲──┘     └──────▲──────┘
          │       │           │       │               │
          │  ┌────┴───────────┴──┐    │               │
          │  │   pcap-replay     │    │               │
          │  └───────────────────┘    │               │
          └──────────┐   ┌────────────┘               │
                  ┌──┴───┴───────────────────────────┴──┐
                  │ rx (lidar-rx)    + r2r (feature ros2)│   모든 것을 조립하는 유일한 곳
                  └─────────────────────────────────────┘
```

규칙은 하나다: 아래쪽 라이브러리들은 서로를 모르고, `lidar-core`만 안다.

- `vlp16`은 `lidar-io`를 모른다. 그래서 해석기는 패킷이 소켓에서 왔는지 파일에서 왔는지 모르고, 바이트 배열만
  넣어 테스트할 수 있다. (`vlp16`의 테스트·예제만 dev-dependency로 `lidar-io`를 씀)
- `clock`은 `vlp16`을 모른다. (센서 시각, 호스트 시각) 쌍만 받으므로 다른 센서에도 그대로 쓸 수 있다.
- 둘을 엮는 것은 맨 위의 실행 파일뿐이다. 조립하는 곳이 한 군데라 바꿀 곳도 한 군데다.
- 외부 크레이트는 꼭 필요한 두 가지만: `libc`(리눅스 빌드에서만 - `lidar-io`의 UDP 수신 `recvmsg` 제어 메시지,
  `rx`의 수신 스레드 CPU 고정·실시간 정책), `r2r`(ROS 2 바인딩, `rx`의 `ros2` feature). 나머지는 표준 라이브러리만 써서
  모든 줄을 따라 읽을 수 있다.

## 5. 경계 세 개와 디자인 패턴

"경계"는 바뀔 수 있는 것과 안 바뀌는 것 사이의 선이다. 경계마다 바뀌는 쪽을 trait 뒤에 숨기면,
바뀌어도 반대쪽 코드는 그대로다.

```text
     바뀌는 것: 입력 출처            바뀌는 것: 센서 모델            바뀌는 것: 소비자
  ┌──────────────────────┐      ┌──────────────────────┐      ┌──────────────────────┐
  │ UdpSource  PcapSource│      │ Vlp16Decoder  (…)    │      │ Stats  Ros2Publisher │
  └──────────┬───────────┘      └──────────┬───────────┘      └──────────▲───────────┘
   ══════════╪═══ 경계 1 ════════════════════╪═══ 경계 2 ════════════════════╪═══ 경계 3 ═══
             ▼                               ▼                               │
       PacketSource::recv()  ──Packet──► SensorDecoder::push() ──Frame──► FrameConsumer::consume()
                                                          (시계 추정이 stamp를 채움)
```

| 경계 | 패턴 | 쉽게 말하면 | 코드 | 이렇게 안 하면 |
|---|---|---|---|---|
| 1. 입력 출처 | **Adapter** | 모양이 다른 것(소켓, 파일)을 같은 모양의 플러그로 맞춤 | `lidar_core::PacketSource`, `lidar_io::{UdpSource, PcapSource}` | 해석기 안에 "소켓이면…, 파일이면…" 분기가 생기고 출처가 늘 때마다 해석기를 고침 |
| 2. 센서 모델 | **Strategy** | 같은 일(해석)을 하는 방법을 갈아 끼움 | `lidar_core::SensorDecoder`, `vlp16::Vlp16Decoder` | 위쪽 코드가 VLP-16 패킷 구조를 알게 됨 |
| 3. 소비자 | **Observer** (발행-구독) | 프레임이 나오면 구독한 쪽들에 알림. 발행하는 쪽은 누가 받는지 모름 | `rx::consumer::{FrameConsumer, Subscriber}` - 소비자마다 스레드 + bounded 채널 | 소비자를 붙일 때마다 수신 코드를 고침. 느린 소비자가 수신을 막음 |
| 전체 흐름 | **Pipes and Filters** | 수신 → 해석 → 조립 → 시계 → 소비를 단계로 나누고 각 단계를 따로 테스트 | `decode` → `assemble` → `clock` → 소비자 | 한 함수에서 다 하면 단계별 검증이 안 됨 |

실제 효과: ROS 2 발행을 붙일 때 수신 루프(`pipeline.rs`)는 한 줄도 바뀌지 않았다 - `main.rs`의 소비자 목록에
하나를 더했을 뿐이다. 보드에서는 그 하나를 빼고 같은 수신 코드를 돌린다.

### Strategy를 Rust로 표현하는 세 가지 방법

| 방법 | 모양 | 언제 |
|---|---|---|
| 제네릭 | `fn run<D: SensorDecoder>(d: D)` | 컴파일할 때 모델이 정해짐. 호출이 직접 연결돼 가장 빠름 |
| trait 객체 | `&mut dyn SensorDecoder` | 실행 중에 설정으로 고름. 호출 한 번에 간접 점프 한 번 |
| enum | `enum Decoder { Vlp16(..), .. }` | 모델 목록이 닫혀 있음. `match`에서 빠진 모델을 컴파일러가 잡음 |

754 패킷/초에서는 세 방법의 호출 비용 차이가 측정되지 않는다. 그래서 성능이 아니라 "설정으로 고르나, 외부에서
모델을 추가하나"로 고른다. `rx::pipeline::run`은 trait 객체(`&mut dyn PacketSource`, `&mut dyn SensorDecoder`)로 받는다.

## 6. 데이터가 바뀌는 모양

```text
 [소켓 recvmsg / pcap 레코드]  Ethernet 14 + IPv4 20 + UDP 8 + 페이로드 1206 = 1248 B
        │ 소켓: 커널이 헤더를 벗겨 줌 + 제어 메시지(커널 수신 시각, 드롭 수)
        │ pcap: lidar_io::net::parse_udp가 헤더 세 겹을 벗김
        ▼
 lidar_core::Packet          { payload (1206 B), received_at: Option<HostTime> }
        │ vlp16::packet::PacketView::parse   검증만 하고 복사하지 않음
        │ vlp16::decode::decode_packet
        ▼
 vlp16::decode::DecodedPacket { timestamp_us, firing_azimuths[24], points: Vec<RawPoint> (최대 384) }
        │ vlp16::assemble::FrameAssembler::push(decoded, received_at)   (약 76패킷마다 한 번 프레임)
        ▼
 lidar_core::Frame           { sensor_stamp, stamp: None, frame_id, points (약 2만 7천),
        │                      packet_times: [(센서 시각, 수신 시각)] × 약 76, stats }
        │ clock::ClockEstimator::stamp_frame   packet_times로 추정을 갱신하고 stamp를 채움 (threaded: stage 스레드)
        ▼
 lidar_core::Frame           { …, stamp: Some(HostTime) }  → Arc로 소비자들에게
        │ rx::ros2::to_point_cloud
        ▼
 sensor_msgs/PointCloud2     header{stamp, frame_id}, x y z intensity ring time, 22 B/포인트
```

- `RawPoint`와 `Point`를 따로 둔 이유: 패킷 안의 위치(발사 번호, 레이저 번호)는 해석·검증에는 필요하지만
  소비자에게는 필요 없다. 골든 테스트는 `RawPoint`의 (발사, ring)으로 참조 구현과 짝을 짓는다.
- `packet_times`를 프레임에 싣는 이유: 센서 시각은 해석기만, 수신 시각은 입력 출처만 안다. 프레임에 함께 실으면
  시계 추정이 해석기 뒤에 붙는 독립된 단계가 된다 (해석기 인터페이스를 바꾸지 않음).

## 7. 스레드 구조

```text
 threaded (기본)                                         inline (비교용)
 ┌─ 수신 스레드 ────────────────────────────┐            ┌─ 한 스레드 ─────────────────────────┐
 │ recv → 해석 → 조립 → 시계                 │            │ recv → … → 소비자 직접 호출          │
 │     └─ 프레임 완성 시 Arc<Frame> try_send ─┼──┐         │  소비자가 느리면 그동안 recv가 멈춤  │
 └──────────────────────────────────────────┘  │         │  → 소켓 버퍼가 차서 커널이 버림      │
          ┌────────────────────────────────────┤         └─────────────────────────────────────┘
          ▼ sync_channel(4)                    ▼ sync_channel(4)
   [consumer-stats 스레드]               [consumer-ros2 스레드]
   차 있으면 그 소비자 몫만 버리고 셈 (queue-drops) → 무엇을 버릴지를 커널이 아니라 설계가 정함
```

## 8. 시간 다루기

```text
 센서 시계 (SensorTime, µs)       호스트 시계 (HostTime, ns)           상대 시각 (Point::time, s)
 ─────────────────────────       ─────────────────────────           ──────────────────────────
 패킷 안에 "정시 이후 µs"로 옴     로봇 컴퓨터의 CLOCK_REALTIME          프레임 대표 시각 기준
 매우 일정 (간격이 µs까지 같음)    커널 수신 시각, ROS 2 메시지 시각       포인트마다 하나
 1시간마다 0으로 돌아감 → 펼침      센서 시계와 기준점·빠르기가 다름         대부분 −0.1 ~ 0 s

     포인트 측정 시각 = 패킷 타임스탬프 + 발사 오프셋(사양서 타이밍 표)          [vlp16]
     프레임 대표 시각 = 마지막 패킷 타임스탬프 (참조 구현들과 같은 기준)          [vlp16]
     Frame::stamp    = 대표 시각을 하한선 모델로 호스트 시계에 옮긴 값            [clock]
```

- 왜 센서 시계를 기준으로 하나: 공식 드라이버의 기본 방식은 `recv` 순간의 호스트 시각을 쓰는데, 그러면 수신
  흔들림이 포인트 시각에 그대로 들어간다. 센서 시계는 흔들리지 않으므로 기준으로 삼고, 호스트 시계와의 관계만
  추정한다.
- 왜 하한선인가: 수신 지연은 음수가 될 수 없어 점들이 실제 관계 위쪽에만 흩어진다. 최소제곱은 평균 지연만큼
  밀린다 (원본 캡처에서 234 µs). 자세히: `docs/modules/clock.md`.

| 시험 | 넣은 것 / 원래 값 | 추정 |
|---|---|---|
| 정답을 아는 드리프트 (loopback) | +100 ppm (센서 시계가 100 ppm 느림) | **+100.01 ~ +100.02 ppm** (4회, 기록: +100.019) |
| 원본 캡처 파일 | (센서 ↔ 캡처 PC) | −53.457 ppm, 최소제곱은 234 µs 위 |
| 원래 간격대로 다시 보냄 | 위와 같아야 함 | −53.1 ~ −53.5 ppm (기록: −53.527) |

## 9. 이상 상황을 다루는 방식

라이브러리는 로그를 찍지 않는다. 오류는 타입으로, 이상 상황은 숫자(통계)로 알리고, 무엇을 할지는 쓰는 쪽(rx)이
정한다.

| 상황 | 감지 위치 | 알림 방법 | rx 동작 |
|---|---|---|---|
| 길이·플래그가 틀린 패킷 | `PacketView::parse` | `DecodeError::Malformed` + `malformed_packets` | 세고 다음 패킷으로 |
| 이중 반환 모드 | `decode_packet` | `DecodeError::Unsupported` | 끝냄 (설정 문제) |
| 제품 ID가 예상과 다름 | `Vlp16Decoder::push` | `product_id_mismatches` (해석은 계속) | 요약에 표시. 샘플은 0x21로 찍혀 있음 |
| 패킷 손실 | 센서 시각 간격 | `missing_packets` (프레임별·누적) | 상태 줄에 표시 |
| 그중 소켓 버퍼 넘침 | 소켓 `SO_RXQ_OVFL` | `PacketSource::kernel_drops` | 상태 줄 `kernel-drops` |
| IP 계층 도착 여부 | 커널 모듈 `lidar_mon` | `/proc/lidar_mon`의 `seen` | 요약 `seen − received here` (IP 계층 이후에서 사라진 수) |
| 소비자가 느림 | `Subscriber::offer` | 소비자별 `queue-drops` | 그 소비자 몫만 버림 |
| 순서 바뀜·중복 | 센서 시각이 조금 뒤로 감 | `reordered_packets`, 그 패킷은 버림 | 세기만 |
| 흐름 재시작 (재생 반복 등) | 시각이 크게 뒤로 가거나 1초 넘게 끊김 | `restarts`, 모으던 프레임 버림 | 세기만 |
| 반 바퀴 넘는 손실 | 손실 구간 × 회전 속도 ≥ 180° | `gap_cuts`, 그 자리에서 프레임을 끊음 (두 바퀴가 섞이지 않게) | 세기만 |
| 정시 직후 늦게 온 지난 시간 패킷 | 원시 타임스탬프가 30분 넘게 앞으로 뜀 | 지난 시간으로 펼침 → 순서 바뀜으로 처리 | 세기만 |
| 호스트 시계 계단 (NTP) | 시계 추정 잔차 | `ClockStats::resets`, 추정 다시 시작 | 요약에 표시 |
| 한 바퀴가 아닌 프레임 | 시작·끝이 자르는 각도가 아님 | `starts_at_cut`, `ends_at_cut` | 통계에서 뺌 |
| 센서가 조용함 | 입력 출처 | `Received::Timeout` | 상태 줄 "sensor quiet" (재시작하지 않음) |

## 10. 테스트와 검증

테스트하기 쉬운 것은 4~5절의 구조 덕분이다.

| 구조 | 가능해진 테스트 | 위치 |
|---|---|---|
| `vlp16`은 입력 출처를 모름 | 합성 바이트 배열로 단위 테스트 | `crates/vlp16/src/*.rs` |
| Adapter (`PacketSource`) | 실제 pcap으로 해석 전체를 소켓 없이 검증 | `crates/vlp16/tests/golden.rs` |
| 해석과 조립 분리 | 포인트는 참조와, 프레임 경계는 합성 패킷으로 따로 | `golden.rs`, `assemble.rs` |
| `clock`은 (센서, 호스트) 쌍만 받음 | 정답을 아는 합성 데이터 (기울기, 계단, 혼잡) | `crates/clock/src/tests.rs` |
| `schedule`은 시계를 읽지 않음 | 송신 시각 계산을 시간 없이 검증 | `crates/pcap-replay/src/schedule.rs` |

```text
 cargo test --workspace   (67개)
   lidar-io     pcap 4 (깨진 파일은 패닉 없이 오류로), net 4 (깨진 길이는 패닉 없이 건너뜀), udp 4 (loopback 소켓: 커널 수신 시각, 돌려준 배열에 다음 패킷, 타임아웃, SO_RXQ_OVFL),
                forward 2 (받은 패킷은 그대로, 같은 바이트를 다른 주소로 / 넘긴 패킷마다 드라이버에 넘긴 시각)
   vlp16        units 2, packet 2, timing 4, calibration 2, geometry 3, decode 4, assemble 8 (돌려받은 배열을 다시 씀), golden 1
   clock        fit 3, estimator 6
   pcap-replay  schedule 3, pacer 1, echo 1 (돌아온 패킷을 보낸 시각과 짝짓고 그대로 넘김)
   rx           histogram 3, kmod(/proc 읽기) 1, sched 2 (CPU 고정을 커널에서 다시 읽음), WorkConsumer 1,
                notify 3 (유닉스 소켓으로 READY·WATCHDOG, watchdog 간격), signal 1 (SIGTERM → 플래그),
                recycle 2 (마지막으로 놓는 쪽만 돌려보냄 / inline·threaded 모두 해석기로 돌아옴)
   (+ 예제 stall_detect, push_cost(rx), refit_cost(clock): 테스트가 아니라 측정 도구, clippy만 거침)
```

커널 모듈은 커널 버전에 묶여 있어 `cargo test`에 넣지 않고, 실제로 적재해 사용자 공간과 대조한다 (아래 표).

CI (`.github/workflows/ci.yml`): push·PR마다 Ubuntu 22.04, Rust 1.85.1(`rust-version`과 같게 고정)에서 샘플 데이터를 받고
`cargo fmt --check`, `cargo clippy -D warnings`, `cargo test --workspace`를 돌린다. ROS 2 feature와 커널 모듈은
그 환경에 ROS 2·맞는 커널 트리가 없어 넣지 않는다.

실제 프로그램을 돌려 정답과 대조한 검증 (`tools/`, 결과는 `docs/modules/tools.md`):

| 검증 | 정답 | 결과 |
|---|---|---|
| 골든 테스트 | velodyne_decoder 3.1.0 출력 | 54,952 포인트 전부 일치 (수평 0.0013 mm, 방위각 0.0051°, 시각 0.23 µs 안) |
| `known_drift.sh` | replay에 넣은 +100 ppm | +100.01 ~ +100.02 ppm (4회) |
| `compare_official.sh` | 공식 velodyne 드라이버 출력 | 93묶음 중 84개 같음. 나머지 9개는 공식 드라이버가 블록 0을 버린 만큼 적음 (예측 285 = 실제 285) |
| `ros2 topic hz`, `ros2_count.py` | 10 Hz, 공식과 같은 필드 구성 | 10.005 ~ 10.013 Hz, 발행 QoS RELIABLE, 필드·point_step 같음 |
| 커널 모듈 대조 | replay가 보낸 7,093개 | 모듈이 본 수 = lidar-rx가 받은 수 = 7,093 |
| `exp1_rcvbuf.sh` | 버린 수 네 군데(모듈, SO_RXQ_OVFL, 트레이스포인트 2개) / 커널 판정식 | 모든 행에서 넷이 같음, 용량 = ⌊버퍼 ÷ 2304⌋ |
| `exp3_inline_threaded.sh` | 처리가 주기보다 길 때의 손실 | inline: 보낸 패킷의 약 76 ~ 78%(보드 82%)를 못 받고 온전한 한 바퀴가 거의 없음 (68개 중 1개) / threaded: 7,093개 모두 받고 프레임 단위로 버림, 받은 프레임은 온전 (6회) |
| `exp4_sched.sh` | 같은 CPU 부하에서 큐 대기의 꼬리 | 일반 p99 약 5 ms → SCHED_FIFO 31 ~ 38 µs (6회). 최대값은 WSL에서 들쭉날쭉 → 보드에서 다시 |
| `stall_detect` 예제 | SCHED_FIFO로도 남는 최대값의 원인 | 0.4 ms 넘는 최대값은 모두 그 순간 수신 CPU 전체가 멈춘 시간 (가상 CPU, Windows가 바쁘면 ~12 ms씩 초당 60번) |
| `deploy/` 서비스 (WSL의 systemd) | 준비 알림, 재시작, watchdog, 정상 종료 | READY 뒤에 active·녹화 시작, 센서가 조용하면 재시작 0, `kill -9` → 재시작, `kill -STOP` → watchdog → 재시작, 수신 재시작 중에도 녹화 프로세스 그대로, SIGTERM에 요약·bag 메타데이터, 서비스로 받은 7,093개 = 모듈이 본 수 |

명령과 실제 출력은 [docs/runs/](runs/README.md)에 단계별로 있다.

## 11. 남은 것

| 부분 | 남은 것 |
|---|---|
| `deploy/` | 녹화 디스크가 찰 때의 정책 (오래된 파일 지우기 또는 녹화 중지) ([deploy.md](modules/deploy.md)) |
| 보드 | 녹화 중 한 번 본 0.2 s 멈춤의 원인 미확정 ([5-board.md](runs/5-board.md) 11절) |
| 센서 모델 | VLP-16만 구현. 다른 모델은 `SensorDecoder` 구현 추가 ([vlp16.md](modules/vlp16.md)) |
