# `rx` - 수신 프로그램 (`lidar-rx`)

크레이트들을 엮는 유일한 곳이다. 입력 출처·해석기·시계 추정·소비자를 골라 조립하고 돌린다.

```text
 crates/rx/src/
 ├── main.rs       인자 → 출처·해석기·시계·소비자 조립 → pipeline::run → 요약. 루프의 순간들을 systemd·신호에 이음
 ├── pipeline.rs   수신 루프, threaded/inline, stage 스레드(시계 추정 + 소비자 전달), LoopHooks
 ├── consumer.rs   경계 3: FrameConsumer trait, Subscriber(스레드 + bounded 채널), StatsConsumer, WorkConsumer(실험 3)
 ├── sched.rs      수신 스레드 하나에만 CPU 고정·SCHED_FIFO, 커널에서 다시 읽어 확인 (실험 4)
 ├── notify.rs     systemd 알림: READY·WATCHDOG·STATUS·STOPPING (4단계)
 ├── signal.rs     SIGTERM·SIGINT → 루프를 끝내고 요약을 남긴 뒤 정상 종료 (4단계)
 ├── status.rs     정해진 간격(기본 1초)마다 상태 한 줄
 ├── histogram.rs  지연 분포를 고정 메모리로 모음 (큐 대기 시간, 프레임 나이)
 ├── kmod.rs       /proc/lidar_mon 읽기 (커널 모듈이 있으면 요약에 대조)
 └── ros2.rs       (feature ros2) Ros2Publisher: Frame → sensor_msgs/PointCloud2

  ┌ PacketSource ┐   ┌ SensorDecoder ┐   ┌ ClockEstimator ┐   ┌ FrameConsumer들 ─────┐
  │ UdpSource    │──►│ Vlp16Decoder  │──►│ stamp_frame    │──►│ StatsConsumer        │
  │ PcapSource   │   └───────────────┘   └────────────────┘   │ Ros2Publisher (ros2) │
  └──────────────┘                                            └──────────────────────┘
       경계 1                 경계 2                                    경계 3
  ◄──────────── 수신 스레드 ─────────────►◄───── stage 스레드 (threaded) ─────►◄ 소비자마다 스레드 ►

  ForwardSource: 어떤 출처든 감싸서, 받은 패킷을 그대로 다른 주소로도 보냄 (--forward)
```

threaded에서 시계 추정과 소비자 전달은 stage 스레드가 한다 (2026-11-11, 보드에서 발견해 옮김). 수신 스레드는
실시간 정책을 받는 스레드라 패킷마다 상한이 정해진 일만 남긴다: recv → 해석 → 완성된 프레임을 채널로 넘김.
시계 추정은 1초마다 창 전체(30 s ≈ 22,600점)를 다시 맞추는데, 보드(Cortex-A9)에서 이 일이 10 ms 넘게 걸려
수신 스레드에서 하면 그동안 패킷이 소켓 큐에서 기다렸다 ([clock.md](clock.md) "비용", [5-board.md](../runs/5-board.md)).

## 사용법

```bash
cargo build --release -p rx                                     # → target/release/lidar-rx (ROS 2 없이)
target/release/lidar-rx --pcap data/vlp16.pcap                  # 파일을 끝까지 (시계 추정은 캡처 시각으로)
target/release/lidar-rx --udp 0.0.0.0:2368 --duration 30        # 실시간 수신 30초
target/release/lidar-rx --udp 0.0.0.0:2368 --rcvbuf 65536 --mode inline

# ROS 2 발행: 빌드도 실행도 ROS 2 환경을 불러온 셸에서 (실행 파일이 ROS 2 라이브러리에 동적 링크됨)
source /opt/ros/humble/setup.bash
tools/build_rx_ros2.sh                                          # → target/ros2/release/lidar-rx
target/ros2/release/lidar-rx --udp 0.0.0.0:2368 --ros2

tools/rviz_demo.sh                                              # replay + 위 + RViz를 한 번에
```

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `--udp ADDR:PORT` / `--pcap FILE` | (필수) | 입력 출처 |
| `--rcvbuf BYTES` | 시스템 기본 | 소켓 수신 버퍼 요청값 (커널은 두 배로 잡음) |
| `--mode` | threaded | `threaded` / `inline` |
| `--queue N` | 4 | threaded에서 소비자마다 채널 길이 (프레임 수) |
| `--duration S` | 끝까지 | 이만큼 지나면 끝냄 |
| `--status S` | 1 | 상태 한 줄 간격 |
| `--ros2` | 끔 | ROS 2 발행 소비자 추가 (`ros2` feature로 빌드해야 함) |
| `--stall-ms T` | 없음 | 실험 1: 패킷을 `--stall-after N`개(기본 2000) 받은 뒤 한 번, T ms 동안 소켓을 읽지 않음 |
| `--work-ms D` | 없음 | 실험 3: 프레임마다 D ms 동안 CPU를 쓰는 소비자(`WorkConsumer`)를 붙임 |
| `--cpu C` | 없음 | 실험 4: 수신 스레드를 CPU C에 고정 |
| `--rt-prio P` | 없음 | 실험 4: 수신 스레드만 SCHED_FIFO 우선순위 P(1~99)로. root 또는 `CAP_SYS_NICE`/`RLIMIT_RTPRIO` 필요 |
| `--lock-memory` | 없음 | `mlockall(MCL_CURRENT \| MCL_FUTURE)`: 실행 코드·스택·힙과 앞으로 받을 메모리를 RAM에 고정해 수신 스레드가 SD 카드 읽기(큰 페이지 폴트)를 기다리지 않게. `RLIMIT_MEMLOCK`(systemd `LimitMEMLOCK=`)이 프로세스 크기보다 커야 함. `MCL_FUTURE`는 수신 스레드가 프레임마다 큰 배열을 새로 받던 때 보드(musl)에서 p99 0.3 → 2.0 ms였고, 배열을 다시 쓰게 한 뒤로 차이 없음 (아래 "다 쓴 배열 돌려받기", [5-board.md](../runs/5-board.md) 11절). 보드에서 켬(시작 때 5.0 MB, 돌면서 7.7 MB), `--ros2`에서는 64 MB를 넘어 실패 |
| `--forward ADDR:PORT` | 없음 | 받은 패킷을 그대로 이 주소로도 보냄 (`lidar_io::ForwardSource`). 보드 → PC의 RViz (보드에는 ROS 2가 없음) |
| `--forward-tx-log FILE` | 없음 | 진단 (`--forward`와 같이): 넘긴 패킷마다 커널 수신 시각과 NIC 드라이버에 넘긴 시각(`SO_TIMESTAMPING` 송신 시각, 오류 큐로 받음)을 끝날 때 FILE로 + "커널 도착 → 드라이버" 분포. 패킷마다 기록을 쌓고 recvmsg를 한 번 더 부르므로 `--duration`이 있는 진단 실행에만. 보드: p50 0.12 ms, p99 0.27 ms, 최대 0.77 ms |

## 수신 루프 - `pipeline.rs`

```text
 소비자·stage 스레드 만들기 → 수신 스레드 실행 조건(sched) → hooks.ready()  (systemd: READY=1)
 loop
   hooks.should_stop()이면 끝 (SIGTERM)
   source.recv()
     Packet  → decoder.push
                 Ok(Some(frame)) → outputs.emit(frame)
                                     threaded: stage 채널에 넣기만 (차 있으면 버리고 셈. 파일 입력이면 기다림)
                                               stage 스레드: frame.settle_point_times → clock.stamp_frame → Arc → 소비자별 채널
                                     inline:   clock.stamp_frame → 소비자 직접 호출
                 Ok(None)        → (프레임 모으는 중)
                 Err(Malformed)  → 해석기가 셈, 다음 패킷
                 Err(Unsupported)→ 끝냄 (설정 문제)
     Timeout → 상태를 "sensor quiet"로 (0.5초마다: 패킷이 없어도 루프는 돈다)
     End     → 루프 끝
   hooks.alive()                                                         (systemd: WATCHDOG=1, 1초마다)
   status가 찍을 때가 되면 한 줄 → hooks.status(line)                      (systemd: STATUS=…)
 hooks.loop_end()   ← 루프가 끝난 바로 그 순간 (main.rs는 여기서 /proc/lidar_mon을 읽고 STOPPING=1)
 decoder.flush() → emit
 stage·소비자 채널을 닫고 스레드가 끝나길 기다림 → 시계 추정기를 돌려받아 요약
```

```rust
pub struct RunOptions {
    pub mode: Mode,                     // Threaded | Inline
    pub duration: Option<Duration>,     // None = 출처가 끝날 때까지
    pub status_every: Duration,         // 상태 한 줄 간격
    pub stall: Option<Stall>,           // 실험 1: 한 번 일부러 쉬기
    pub receive_thread: ThreadSetup,    // 실험 4: 수신 스레드의 CPU 고정·실시간 정책 (sched.rs)
}

pub fn run(source: &mut dyn PacketSource, decoder: &mut dyn SensorDecoder, clock: &mut ClockEstimator,
           consumers: Vec<Box<dyn FrameConsumer>>, options: &RunOptions,
           hooks: &mut dyn LoopHooks) -> Result<Outcome, Box<dyn Error>>

pub trait LoopHooks {            // 모두 기본 구현(아무것도 안 함)이 있다
    fn ready(&mut self);         // 패킷을 받을 준비가 됨
    fn alive(&mut self);         // 루프가 한 바퀴 돎
    fn status(&mut self, line: &str);
    fn should_stop(&self) -> bool;
    fn loop_end(&mut self);      // 수신을 멈춘 바로 그 순간
}
```

- 실행 방식 설정을 `RunOptions` 하나로 묶었다. 인자가 여덟 개로 늘어 순서를 헷갈리기 쉬웠고(clippy
  `too_many_arguments`), 설정이 늘어도 함수 모양이 바뀌지 않는다.
- `LoopHooks`: 루프는 자기 순간들을 알리기만 하고, 그것을 systemd에 보낼지·신호를 볼지·커널 모듈을 읽을지는 `main.rs`가
  정한다. 그래서 루프 코드는 systemd를 모른다 (systemd 없이 돌리면 알림은 아무것도 안 함).
- `loop_end`: 커널 모듈 카운터의 "끝 값"은 수신이 멈춘 순간에 읽어야 한다. 소비자 스레드가 끝나길 기다린 뒤에
  읽으면 그 사이에 도착한(그러나 받지 않은) 패킷까지 `seen`에 들어가 `seen − received`가 0이 아니게 나온다.
- 상태 줄의 값(분위수, 소비자별 버린 수)은 줄을 찍을 때만 모은다. 전에는 패킷마다 모았다.

### threaded와 inline

```text
 threaded (기본)                                  inline (비교용, 실험 3)
 ┌ 수신 스레드 ───────────────────┐               ┌ 한 스레드 ──────────────────────────┐
 │ recv → 해석 → emit             │               │ recv → 해석 → 시계 → 소비자 직접 호출 │
 └───────────┬───────────────────┘               └─────────────────────────────────────┘
             │ Frame을 try_send [채널 4]            소비자가 느리면 그동안 recv가 멈춤
 ┌ stage 스레드 ┴──────────────────┐               → 소켓 버퍼가 차서 커널이 아무 패킷이나 버림
 │ 점 시각 맞춤 → 시계 → Arc<Frame>│               → 구멍 나거나 손실 지점에서 끊긴 프레임
 └───────────┬────────────────────┘
             │ Arc<Frame>을 소비자마다 try_send
     ┌───────┴────────┐
     ▼                ▼
 [채널 4]→stats   [채널 4]→ros2
 차 있으면 그 채널 몫만 버리고 셈 (queue-drops stage / stats / ros2)
```

### 다 쓴 배열 돌려받기

```text
 수신 스레드                                      stage 스레드 · 소비자 스레드
 ┌───────────────────────────────────────┐       ┌───────────────────────────────────────┐
 │ recv ─ 페이로드 ─ 해석 ─ 프레임 조립 ───┼─Frame─► Arc<Frame> 공유 … 마지막으로 놓는 쪽이 │
 │  ▲ source.recycle   ▲ decoder.recycle  │       │ Arc::into_inner로 꺼내 돌려보냄         │
 │  └ 이 패킷 끝나면    └ 패킷마다 채널 확인 ◄┼─Frame─┤ (Recycle, 채널 4. 차면 거기서 해제)      │
 └───────────────────────────────────────┘       └───────────────────────────────────────┘
```

수신 스레드는 패킷마다 페이로드(1,206 B)와 해석한 포인트(384개 자리) 배열을, 프레임마다 점 배열(약 0.7 MB)을
쓴다. 매번 새로 할당하면 보드의 C 라이브러리(musl)가 큰 것은 `mmap`으로 새 페이지를 받아 오고, 작은 것도 묶음째
돌려줬다 다시 받는다 → 수신 스레드가 그 페이지를 처음 쓸 때마다 page fault (커널에 들어가 페이지를 채움).
그래서 셋 다 다 쓴 것을 돌려받아 다시 쓴다.

- 페이로드: 루프가 해석을 마치면 `source.recycle(packet)` → `UdpSource`가 다음 패킷을 그 배열에 복사
- 패킷 포인트: 해석기가 지난 패킷의 배열을 들고 있다가 다음 패킷에 씀 (`vlp16::decode::decode_packet_reusing`)
- 프레임 점 배열: 프레임은 `Arc`로 여러 스레드가 쥐므로, 마지막으로 놓는 스레드가 `Arc::into_inner`로 꺼내
  수신 스레드로 돌려보낸다 (`consumer::Recycle`). 수신 스레드는 패킷마다 그 채널을 기다리지 않고 비워
  `decoder.recycle`로 넘기고, 조립기는 다음 프레임을 거기에 모은다. inline이거나 stage 채널이 차서 버린 프레임은 그
  자리에서 바로 돌려준다. 새로 할당하는 것은 돌아온 것이 없을 때뿐이라 도는 배열 수는 처리 중인 프레임 수 + 1을 넘지 않는다
  (요약 줄 `new point arrays`: 보드 286프레임에 1)

보드 실측 (세 바퀴, [5-board.md](../runs/5-board.md) 11절): 수신 스레드 page fault 48,504 → 556 (남은 것은 시작 직후뿐,
그 뒤 초당 0), 수신 스레드의 메모리 시스템 호출 10 s에 0, 수신 스레드 CPU 20.7 → 17.4 %. 대기 p99는 0.27 ~ 0.28 ms로
비슷하다 (전에도 보이지 않던 비용). 대신 `--lock-memory`가 앞으로 받을 메모리까지 고정해도(`MCL_FUTURE`) 느려지지 않게 됐다.

threaded에서는 수신 스레드가 소켓을 계속 비우고, 밀리면 완성된 프레임 단위로 버린다. 무엇을 어디서 버릴지
커널에 맡기지 않는다. 입력이 파일(`--pcap`)이면 stage 채널에서는 버리지 않고 기다린다: 파일은
실시간이 아니고, 버리면 시계 추정 결과가 실행마다 달라진다 (느린 qemu-arm에서도 threaded = inline 결과로 확인).

## 소비자 - `consumer.rs` (Observer)

```rust
pub trait FrameConsumer: Send {
    fn name(&self) -> &'static str;
    fn consume(&mut self, frame: &Frame);
    fn summary(&self) -> String;
}
```

- `Subscriber::spawn(consumer, queue)`: 소비자를 자기 스레드로 옮기고 앞에 `sync_channel(queue)`를 둔다.
- `Subscriber::offer`: `try_send` - 기다리지 않는다. 차 있으면 지금 들어온(가장 새) 프레임을 버리고 `dropped += 1`.
  채널에 먼저 들어 있던 것이 남으므로, 길이 1이어도 소비자가 받는 것은 가장 최신이 아니라 자리가 빈 뒤 처음 온 프레임이다.
  늘 최신 것만 원하면 오래된 것을 새것으로 바꾸는 한 칸짜리 자리(ROS 2 QoS `KEEP_LAST`가 이쪽)가 맞다 - 아직 없음.
- 프레임은 `Arc`로 공유: 소비자가 둘이어도 포인트 배열(약 0.6 MB)을 복사하지 않는다.
- Rust에서 콜백 등록 방식 대신 채널을 쓰는 이유: 콜백은 수신 스레드에서 실행되므로 느린 콜백이 수신을 막고,
  여러 스레드가 소비자 상태를 공유하면 수명·잠금 문제가 생긴다. 채널은 소유권을 넘기므로 공유가 없다.

`StatsConsumer`: 프레임 수, 한 바퀴 범위(`is_full_turn`)인 프레임 수와 그 점 수, 손실 패킷(프레임별 값의 합 - 자르는
각도에 걸쳐 빠진 패킷은 두 프레임에 다 세므로 해석기의 누적값보다 클 수 있음), 그리고 프레임 나이 -
프레임 대표 시각(호스트 시계로 바꾼 값)에서 이 소비자가 받은 순간까지. 시작 조각·끝 조각은 뺀다.

`WorkConsumer` (`--work-ms D`, 실험 3): 프레임마다 D ms 동안 바쁜 대기로 CPU를 쓴다 - 뒤따르는 인식 알고리즘 같은 무거운
처리를 모사한다. 처리한 프레임 수, 그중 온전한 한 바퀴(`is_complete`: 0°에서 0°까지, 빠진 패킷 없음) 수, **처리를
시작할 때 그 데이터의 나이**를 센다. 빠진 패킷 수만 보면 안 된다: 반 바퀴 넘게 빠져 그 자리에서 끊은 프레임은 빠진 패킷이
0이지만 한 바퀴의 조각이다. inline에서는 이 처리 동안 수신이 멈추고, threaded에서는 이 소비자의 채널만 찬다. 결과는
[experiments.md](experiments.md#실험-3---버리는-위치-inline-vs-threaded).

```text
consumer work: queue drops 28; 68 frames processed (66 complete: full turn, no missing packets); data age when processing started p50 … p99 598016 us, …
```

## 수신 스레드의 실행 조건 - `sched.rs` (실험 4)

```text
 pipeline::run
   소비자 스레드들을 만듦 (Subscriber::spawn)
   sched::apply(--cpu, --rt-prio)        ← 그다음에. 새 스레드는 만든 스레드의 정책을 물려받기 때문
     pthread_setschedparam(self, SCHED_FIFO, P)   ← 먼저: 일반 정책인 채로 바쁜 CPU로 옮기면 옮겨 가지 못하고 기다림
     sched_setaffinity(0, {C})           pid 0 = 부른 스레드 (리눅스는 스레드마다 따로 가짐)
     pthread_getschedparam + sched_getaffinity로 다시 읽음 → 요약 "receive thread: SCHED_FIFO priority 50, CPUs 2"
   수신 루프 …
```

- 수신 스레드 하나에만 준다. 프로세스 전체(`chrt`, systemd의 `CPUSchedulingPolicy=`)로 주면 소비자·ROS 2/DDS 스레드까지
  실시간이 되어, 그들이 CPU를 오래 쓸 때 다른 일을 굶긴다.
- `--mode inline`이면 소비자도 수신 스레드에서 돌므로 같은 정책·CPU를 쓴다 (실험 4는 threaded로 잼).
- 실패하면 오류로 끝낸다 - 정책이 적용됐다고 믿고 측정하지 않도록. 무엇이 실패했는지를 붙여 알린다:
  `lidar-rx: --rt-prio 50 (needs root, CAP_SYS_NICE or RLIMIT_RTPRIO): Operation not permitted (os error 1)`.
- `sched_param`은 C 라이브러리마다 필드가 달라(musl에는 더 있음) 0으로 채운 뒤 우선순위만 쓴다 (보드는 musl로 빌드).
- 결과는 [experiments.md](experiments.md#실험-4---부하-속-수신-지연-스케줄링-정책).

`examples/stall_detect.rs`: 실험 4 재검용. 한 CPU에 고정한 스레드가 시계를 쉬지 않고 읽어, 그 CPU가 멈춘 구간(gap)을
센다 - 커널의 `hwlat` 트레이서와 같은 생각 (이 WSL 커널에는 빠져 있음). 요약의 `queue wait … max at <시각>`과 맞춰 보면
최대 지연이 "CPU가 멈춘 시간"인지 가를 수 있다.

## ROS 2 발행 - `ros2.rs` (feature `ros2`)

```text
 Frame ──► sensor_msgs/PointCloud2  (토픽 velodyne_points, 노드 lidar_rx)
   header.stamp    = Frame::stamp (추정 전이면 마지막 패킷의 커널 수신 시각)
   header.frame_id = "velodyne"
   height 1, width = 포인트 수, point_step 22, is_dense true, little-endian
   fields: x f32 @0, y f32 @4, z f32 @8, intensity f32 @12, ring u16 @16, time f32 @18   (공식 드라이버와 같음)
 QoS: 시스템 기본(reliable, 깊이 10) - REP 2003 (발행은 reliable, 쓰는 쪽이 best effort로 구독)
```

- `r2r`은 빌드할 때 설치된 ROS 2의 메시지 정의로 바인딩을 만든다 → `tools/build_rx_ros2.sh`
  (`source /opt/ros/humble/setup.bash` 후 빌드). 실행할 때도 ROS 2 라이브러리 경로가 필요하므로 같은 환경에서 실행.
- 빌드 폴더를 `target/ros2/`로 따로 둔다. 같은 `target/`이면 ROS 2 없이 하는 일반 빌드(`cargo build -p rx`)가 같은
  실행 파일을 덮어써, `--ros2`를 줬을 때 오류가 난다 (실제로 겪음).

### RViz로 보기 - `tools/rviz_demo.sh`

```text
 pcap-replay (--pace sensor, 끝없이) ─UDP─► target/ros2/release/lidar-rx --ros2 ─/velodyne_points─► rviz2
                                                                         RViz 설정: tools/rviz/lidar.rviz
```

| RViz 설정 (`tools/rviz/lidar.rviz`) | 값 | 이유 |
|---|---|---|
| Fixed Frame | `velodyne` | 메시지의 `frame_id`와 같게. 좌표계 변환(TF)이 없어도 바로 그려짐 |
| PointCloud2 토픽 | `/velodyne_points` | |
| Reliability | Best Effort | REP 2003: 쓰는 쪽은 best effort. 발행이 reliable이라 연결됨 |
| 색 | intensity (반사율) | 벽·바닥 재질 차이가 보임 |
| Grid / Axes | 1 m 격자, 센서 축 | x(빨강) = 앞, y(초록) = 왼쪽 - 좌표 변환이 맞는지 눈으로 확인 |

RViz 오른쪽 아래의 fps(약 30)는 화면을 다시 그리는 횟수로, 설정의 Frame Rate 30을 따른다. 데이터 주기(10 Hz)와는
별개라, 새 포인트 클라우드 하나가 약 세 번 그려진 뒤 다음 것으로 바뀐다.

Global Status: Warn (`No tf data`) - 정상. RViz는 좌표계들의 관계(TF 트리)를 보고 각 메시지를 화면 기준
좌표계(Fixed Frame)로 옮겨 그린다. 여기에는 TF를 보내는 노드가 없어서 경고가 뜬다. 메시지의 `frame_id`와 Fixed
Frame이 둘 다 `velodyne`라 변환 없이 그대로 그려진다.

```text
 실제 로봇이라면:   map ─► odom ─► base_link ─► velodyne
                   (위치 추정)  (휠 오도메트리)  (센서 장착 위치: URDF → robot_state_publisher)
                   Fixed Frame을 odom이나 map으로 두고, 로봇이 움직여도 포인트가 세상에 고정돼 보임
 여기:             velodyne 하나뿐 → Fixed Frame = velodyne
```

이 샘플의 실제 장착 위치(높이·자세)는 알 수 없으므로, 경고를 없애려고 가짜 `base_link → velodyne` 변환을 넣지
않는다. 포인트별 측정 시각(`time` 필드)이 필요한 이유도 이 TF와 이어진다: 로봇이 움직이는 동안 찍힌 포인트를
`odom`으로 옮길 때, 포인트마다 그 시각의 로봇 자세를 써야 장면이 휘지 않는다.

## systemd 서비스로 돌 때 - `notify.rs`, `signal.rs` (4단계)

서비스 파일과 설치는 [deploy.md](deploy.md). 프로그램 쪽에서 한 일은 둘이다.

```text
 notify.rs  NOTIFY_SOCKET(유닉스 데이터그램)에 "KEY=VALUE" 한 덩어리를 보냄. 라이브러리 없이 직접
            READY=1      hooks.ready(): 소켓·버퍼·소비자 스레드·수신 스레드 설정이 다 끝난 뒤 (Type=notify)
            WATCHDOG=1   hooks.alive(): WATCHDOG_USEC의 절반(1초)마다. 루프가 돌 때만 보내므로 루프가 멈추면 끊김
            STATUS=…     hooks.status(): 상태 줄 그대로 → systemctl status에 보임
            STOPPING=1   hooks.loop_end()
            NOTIFY_SOCKET이 없으면(systemd 밖) 아무것도 안 함. '@'로 시작하면 추상 소켓 이름

 signal.rs  SIGTERM·SIGINT → 처리기는 원자 플래그만 세움 → 루프가 한 바퀴마다 보고 끝냄 → 요약을 journal에 남김
            SA_RESTART를 주지 않아 기다리던 poll이 EINTR로 바로 깨고, 아니어도 0.5초 타임아웃 안에 알아챔
```

- 센서가 조용한 것과 프로세스가 멈춘 것을 가른다. 수신은 0.5초 타임아웃으로 기다리므로 패킷이 없어도 루프가 돌고
  watchdog 신호가 나간다. 상태는 "sensor quiet"가 된다. 루프가 정말 멈추면(교착, `kill -STOP`) 신호가 끊겨 systemd가
  2초 뒤 SIGABRT로 죽이고 다시 띄운다.
- 소비자 스레드는 watchdog에 넣지 않았다. 센서가 조용하면 소비자는 할 일이 없어 쉬는 것이 정상이고, 소비자가 느리거나
  멈추면 그 채널이 차서 `queue-drops`로 보인다. 느린 소비자 때문에 수신까지 재시작하면 받을 수 있던 데이터를 잃는다.

## 수신 경로 측정 (3단계에서 추가)

```text
 패킷마다:  큐 대기 시간 = 지금(CLOCK_REALTIME) − 커널 수신 시각(SO_TIMESTAMPNS)
           → histogram.rs (1 µs 단위 ~1 ms, 그 위는 2의 거듭제곱 구간을 64등분, 메모리 고정)
           → 상태 줄 wait-p99, 요약 queue wait p50/p99/max

 --stall-ms T:  N개 받은 뒤 T ms 쉼 → 다시 읽을 때 "커널 수신 시각 ≤ 다시 읽기 시작한 순간"인 패킷 수 = 큐 용량
                요약: stall: … N packets were queued when reading resumed (oldest waited … us)

 /proc/lidar_mon이 있으면 (커널 모듈 적재됨):  시작과 루프가 끝난 순간(loop_end)에 읽어
                요약: kernel module (IP layer): seen …, lost …; seen − received here = …
```

`--stall-ms`의 소켓 드롭 수(`SO_RXQ_OVFL`)도 같은 원칙으로, 쉬기 직전의 값과 큐에 남아 있던 패킷을 다 꺼낸 직후의
값을 짝지어 뺀다 (`StallReport::kernel_drops_during`). 넘친 뒤 처음 큐에 들어간 패킷에 누적값이 붙어 오므로 그 패킷을
받은 순간이 "끝 값"이다. 실행이 끝날 때 읽으면 그 뒤의 드롭까지 섞인다. 커널이 주는 값은 u32 누적값이라 넘침(wrap)을
고려해 u32로 뺀다.

`queue wait`은 녹화 파일(`--pcap`)에서는 재지 않는다: 그때 수신 시각은 캡처한 날의 시각이라 지금과 비교하면 의미가
없다 (10초 넘게 차이 나는 값은 버림). 실험 결과는 `docs/modules/experiments.md`.

## 상태 줄과 요약

상태 줄 (README 데모 2부: replay를 `--drift-ppm 100`으로 다시 시작한 뒤, `docs/media/demo_console.tsv`):

```text
rx: 753 pkt/s 10.0 frm/s | missing 0 reorder 0 restart 1 kernel-drops 0 wait-p99 59 us | clock +100.00 ppm (lsq +99.83) p99 22 us | queue-drops stats 0 ros2 0
```

요약 (끝날 때 한 번, `tools/known_drift.sh 100 3`의 첫 시험 - 3회 반복 21,279 패킷, 반복 사이 시각이 이어지므로 restart 0):

```text
--- summary
decoder: 21279 packets, 286 frames, malformed 0, product-id mismatches 21279, missing 0, reordered 0, restarts 0
socket: kernel drops (SO_RXQ_OVFL) 0
clock: skew +100.019 ppm (least squares +99.971), least squares sits 2.2 us above the envelope; ...
consumer stats: queue drops 0; 286 frames (284 full turns, ... points each), missing packets 0; frame age at consumer p50 166 us, ...
```

| 항목 | 어디서 | 뜻 |
|---|---|---|
| missing | 해석기 (센서 시각 간격) | 센서가 보냈는데 안 온 패킷 (어디서 사라졌든) |
| kernel-drops | 소켓 `SO_RXQ_OVFL` | 그중 수신 버퍼가 넘쳐 커널이 버린 것 |
| queue-drops | stage 채널, 소비자 채널 | 다 받았지만 stage나 소비자가 느려 넘기지 못한 프레임 (`stage`는 2026-11-11부터) |
| frame age | StatsConsumer | 대표 시각 → 소비자 도착. 해석·채널 대기 포함 |

missing − kernel-drops = 소켓에 들어오기 전에 사라진 것 (네트워크·NIC·IP 계층·UDP). 이것을 다시 IP 계층 전후로
가르는 것이 커널 모듈(`kmod/lidar_mon`, 3단계)이다: 모듈이 IP 계층에서 본 수(`seen`)와 lidar-rx가 받은 수의 차이
= IP 계층 이후에서 사라진 수, 모듈의 `lost` = IP 계층에 오기 전에 사라진 수.
