# Rust 노트: C에서 사람이 챙길 부분을 언어가 챙기는 지점

이 저장소에는 C 커널 모듈(`kmod/lidar_mon.c`)과 Rust 사용자 공간 코드(`crates/`)가 같이 있다. 같은 일을 두 언어로
한 곳이 있어서 나란히 비교할 수 있다. 항목마다 문제 → C라면 → 여기서는(코드 위치) → 남는 한계 순서로 적는다.

| # | 상황 | C라면 | 여기서는 |
|---|---|---|---|
| 1 | 바이트를 구조체로 읽기 | packed struct 포인터 캐스팅. 정렬·엔디언·길이를 사람이 챙김 | 길이를 타입으로 확정한 뒤 `from_le_bytes` |
| 2 | 잘못된 입력 | 반환값을 무시해도 진행됨 | `Result`. 무시하면 경고, `?`로 위로 넘김 |
| 3 | 단위가 다른 같은 정수 | 둘 다 `uint16_t` → 바꿔 넣어도 모름 | Newtype → 바꿔 넣으면 컴파일 에러 |
| 4 | 버퍼를 가리키는 포인터 | 버퍼가 덮어써진 뒤에도 쓸 수 있음 | 빌림의 수명 → 컴파일 에러 |
| 5 | 프레임을 다른 스레드로 | 누가 해제하나, 동시에 고치면 레이스 | `Arc`로 읽기 공유, `Send` 검사 |
| 6 | 채널이 찼을 때 | 큐 구현에 따라 막힘 또는 덮어쓰기 | `try_send`의 `Full`을 받아 정책을 코드로 |
| 7 | 경계(인터페이스) | 함수 포인터 구조체 | trait과 `dyn` |
| 8 | 상태 | 정수 상태 변수 + switch | 데이터를 품은 enum + 빠짐없는 `match` |
| 9 | 운영체제 호출 | 어디서나 | `unsafe`를 몇 파일 안에 가두고 이유를 적음 |
| 10 | 신호 처리기 | 처리기 안에서 할 수 있는 일을 사람이 지킴 | **똑같이 사람이 지킴** (언어가 못 막는 것) |

## 1. 바이트를 구조체로 읽기

문제. VLP-16 패킷은 1206 B이고, 블록 12개(각 100 B)와 타임스탬프 4 B로 되어 있다. 값은 little-endian이다.

C라면. 받은 버퍼를 packed struct 포인터로 캐스팅한다. 길이가 1206보다 짧으면 버퍼 밖을 읽고, 엔디언은
호스트가 little-endian일 때만 맞고, packed가 아닌 struct로 캐스팅하면 ARM에서 정렬되지 않은 접근이 된다.

**lidar-rx** (`crates/vlp16/src/packet.rs`):

```rust
pub struct PacketView<'a> {
    bytes: &'a [u8; PAYLOAD_LEN],          // 길이가 타입에 들어 있다: 정확히 1206 B
}

pub fn parse(bytes: &'a [u8]) -> Result<Self, PacketError> {
    let bytes: &[u8; PAYLOAD_LEN] = bytes
        .try_into()                         // 길이가 다르면 여기서 Err(Length)
        .map_err(|_| PacketError::Length(bytes.len()))?;
    // 블록 플래그 0xFFEE, 방위각 < 36000, 반환 모드를 확인한 뒤에만 PacketView를 돌려줌
}

u16::from_le_bytes([self.bytes[2], self.bytes[3]])   // 블록의 방위각. 엔디언을 코드에 적음, 정렬과 무관
```

- 검사를 한 곳에 모은다: `parse`를 통과한 `PacketView`는 길이·플래그·방위각 범위가 보장된 값이라, 그 뒤의
  읽는 메서드들은 실패하지 않는다. "확인했다"가 주석이 아니라 타입으로 남는다.
- 같은 일을 커널 모듈은 `skb_header_pointer`로 한다 (`kmod/lidar_mon.c`). 패킷 데이터가 한 덩어리가 아닐 수 있어
  필요한 바이트만 지역 버퍼로 꺼내고, 모자라면 NULL을 돌려준다. 그 NULL 검사는 사람이 빠뜨리지 않아야 한다.

## 2. 잘못된 입력도 값

- 해석기는 `Result<Option<Frame>, DecodeError>`를 돌려준다 (`crates/lidar-core/src/decoder.rs`).
  `DecodeError::Malformed`(모양이 틀린 패킷)는 수신 루프가 세고 다음 패킷으로 가고, `Unsupported`(지원 안 하는
  반환 모드)는 멈춘다 (`crates/rx/src/pipeline.rs`). 어떻게 할지는 해석기가 아니라 루프가 정한다.
- `Result`를 쓰지 않고 버리면 컴파일러가 경고한다 (`#[must_use]`). `?`는 오류를 그대로 위로 넘긴다.
- 패킷 손실은 오류가 아니다. 손실은 센서에서는 정상적으로 일어나는 일이라 프레임 통계(`missing_packets`,
  `is_complete()`)로 알리고, 어떻게 할지는 소비자가 정한다.
- 녹화 파일의 IPv4·UDP 헤더 길이가 깨져 있으면 패닉 없이 그 레코드를 건너뛰는지(`None`) 테스트로 고정했다
  (`crates/lidar-io/src/net.rs`). C였다면 그 길이를 믿고 버퍼 밖을 읽는 자리다.

## 3. 단위를 타입으로 (Newtype)

```rust
pub struct Azimuth(pub u16);       // 0.01° 단위, 0..36000       crates/vlp16/src/units.rs
pub struct RawDistance(pub u16);   // 2 mm 단위, 0이면 반사 없음
```

- 둘 다 `u16`이라 그대로 쓰면 서로 바꿔 넣어도 모른다. 감싸 두면 거리 자리에 방위각을 넣는 순간 컴파일 에러다.
- 단위 변환(`degrees()`, `meters()`)과 0°를 넘는 차이 계산(`forward_to`, `rem_euclid`)이 타입에 붙어 있어 한 곳에만 있다.
- 실행 비용은 없다. 메모리 표현은 `u16`과 같다.

## 4. 빌린 데이터의 수명

- `PacketView<'a>`는 패킷 바이트를 빌린다 (`&'a`). 복사하지 않고, 원래 바이트보다 오래 살 수 없다.
- C에서는 다음 `recv`가 같은 버퍼를 덮어쓴 뒤에도 옛 포인터를 계속 쓸 수 있고, 그 버그는 값이 이상해질 때에야
  보인다. Rust에서는 빌린 값이 살아 있는 동안 원래 버퍼를 고칠 수 없어서 컴파일 단계에서 막힌다.

## 5. 프레임을 다른 스레드로: 소유권과 Arc

```rust
let frame = Arc::new(frame);                       // 한 번만 만든다           crates/rx/src/pipeline.rs
self.tx.try_send(Arc::clone(frame))                // 소비자마다 참조 수만 늘림  crates/rx/src/consumer.rs
pub trait FrameConsumer: Send { fn consume(&mut self, frame: &Frame); … }
```

- 프레임은 소비자 수와 무관하게 한 번만 만들고, 소비자는 같은 프레임을 읽기만 한다. 마지막 참조가 사라질 때
  해제된다. 누가 해제할지를 사람이 정하지 않는다.
- `FrameConsumer: Send`: 소비자를 스레드로 옮길 수 있는 타입인지 컴파일러가 확인한다.
- 스레드 사이에 공유하는 것은 셋뿐이다: 읽기 전용 `Arc<Frame>`, 종료 플래그(`AtomicBool`), 그리고 stage 스레드가
  상태 줄용으로 적어 두는 사본 하나(`Arc<Mutex<StageView>>`, `crates/rx/src/pipeline.rs`). 가변인 것은 잠금이나
  원자 연산 없이는 만질 수 없게 타입이 막는다 (`Mutex` 없이 `&mut`을 두 스레드에 줄 수 없음).
- 실시간 정책을 받는 수신 스레드는 그 `Mutex`를 `try_lock`으로만 읽는다. 잠겨 있으면 지난 사본을 쓴다: 우선순위가
  낮은 스레드가 쥔 잠금을 실시간 스레드가 기다리는 일(우선순위 역전)을 만들지 않으려고. 이 판단은 언어가 해 주지 않는다.
- 커널 모듈은 같은 문제를 다르게 푼다: 카운터는 CPU별(`DEFINE_PER_CPU`)로 두어 잠금 없이 올리고, 순서 상태(마지막
  타임스탬프)만 spinlock으로 지킨다. `/proc` 쪽은 `spin_lock_bh`로 softirq를 막고 잡는다. 이 규칙은 컴파일러가
  확인해 주지 않는다.

## 6. 채널이 찼을 때의 정책을 코드로

```rust
let (tx, rx) = sync_channel::<Arc<Frame>>(queue);  // 길이 제한 있는 채널 (기본 4)
match self.tx.try_send(Arc::clone(frame)) {
    Ok(()) => {}
    Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => self.dropped += 1,
}
```

- `try_send`는 막히지 않는다. 느린 소비자가 수신 스레드를 붙잡지 않는다 (실험 3).
- 결과 타입이 "가득 참"과 "받는 쪽이 사라짐"을 따로 알려 줘서, 둘 다 처리하지 않으면 `match`가 컴파일되지 않는다.
- 지금 정책은 새로 온 프레임을 버림이다. 오래된 것을 버리고 늘 최신 것을 두려면(ROS 2 `KEEP_LAST` 방식) 한 칸짜리
  자리를 따로 만들어야 하고, 아직 구현하지 않았다.

## 7. 경계는 trait, 고르는 것은 dyn

| trait | 위치 | 구현 |
|---|---|---|
| `PacketSource` | `crates/lidar-core/src/source.rs` | `UdpSource`, `PcapSource`, 그리고 출처를 감싸는 `ForwardSource`(Decorator) |
| `SensorDecoder` | `crates/lidar-core/src/decoder.rs` | `Vlp16Decoder` |
| `FrameConsumer` | `crates/rx/src/consumer.rs` | 통계, 처리 모사(실험 3), ROS 2 발행 |
| `LoopHooks` | `crates/rx/src/pipeline.rs` | systemd 알림·종료 신호 (`main.rs`). 모든 메서드에 기본 구현이 있어 테스트는 빈 훅을 씀 |

- 입력 출처는 실행 인자로 고르므로 `Box<dyn PacketSource>`(실행 중 선택, 가상 함수 표로 간접 호출)를 쓴다.
  제네릭으로 쓰면 컴파일 때 출처마다 코드가 따로 만들어져 호출이 직접적이지만, 고르는 코드가 복잡해진다.
  754 pkt/s에서 호출 비용 차이는 재 볼 만한 크기가 아니라서 성능이 아니라 "실행 중에 고르는가"로 정했다.
- Rust에는 상속이 없다. 공통 동작은 trait의 기본 메서드로, 조합은 구조체에 trait 객체를 담아서 한다.

## 8. 상태는 데이터를 품은 enum

```rust
pub enum Received { Packet(Packet), Timeout, End }       // crates/lidar-core/src/source.rs
enum StallState {                                        // crates/rx/src/pipeline.rs (실험 1의 일부러 쉬기)
    Waiting(Stall),
    Draining { report: StallReport, resumed_ns: i64 },
    Done(StallReport),
}
```

- `Timeout`(센서가 조용함, 계속 기다림)과 `End`(파일 끝, 그만)가 타입으로 구분된다. watchdog을 센서 침묵과 떼어 놓은
  설계가 이 구분 위에 있다.
- `StallState`는 상태마다 필요한 데이터만 가진다. 전이 함수가 `self`를 받아 새 상태를 돌려주므로 옛 상태를 실수로
  다시 쓸 수 없고, `match`에서 상태 하나를 빠뜨리면 컴파일되지 않는다.

## 9. unsafe 가두기

| 파일 | 왜 unsafe인가 |
|---|---|
| `crates/lidar-io/src/udp.rs` (`mod sys`) | `setsockopt`, `poll`, `recvmsg`, 제어 메시지를 훑는 `CMSG_*` 매크로, `struct timespec` 읽기 |
| `crates/rx/src/sched.rs` | `pthread_setschedparam`(SCHED_FIFO), `sched_setaffinity`(CPU 고정), 다시 읽기 |
| `crates/rx/src/signal.rs` | `sigaction` |
| `crates/rx/examples/stall_detect.rs` | 측정 도구의 실시간 정책·CPU 고정 (`sched.rs`와 같은 순서) |

- 운영체제 호출은 이 파일들 안에만 있고, 바깥에는 안전한 함수만 보인다. 블록마다 `// SAFETY:` 주석으로 그 호출이
  안전하려면 무엇이 지켜져야 하는지 적었다 (예: 포인터가 가리키는 버퍼가 호출 동안 살아 있고, 길이를 정확히 넘김).
- 제어 메시지의 데이터는 정렬이 보장되지 않아 `read_unaligned`로 읽고, 길이를 먼저 확인한다. `struct timespec`은
  64비트에서 16 B, 32비트 ARM(보드)에서는 8 B일 수 있어 길이로 구분한다.
- systemd 알림(`crates/rx/src/notify.rs`)은 표준 라이브러리의 유닉스 소켓만으로 해서 unsafe가 없다.

## 10. 언어가 막아 주지 못하는 것

- 신호 처리기: 처리기는 프로그램이 어느 지점에 있든 끼어들어 돈다. 그 안에서 메모리를 할당하거나 잠금을 잡으면
  교착이 날 수 있다. Rust는 처리기 안에서 `println!`을 불러도 막지 않는다. 그래서 처리기는 원자 플래그 하나만 세우고,
  나머지는 수신 루프가 플래그를 보고 한다 (`crates/rx/src/signal.rs`). 이 규칙은 C에서와 똑같이 사람이 지킨다.
- unsafe 블록 안의 약속: `SAFETY` 주석에 적은 조건은 컴파일러가 확인하지 않는다.
- 논리 오류: 실험 3의 지표 오류(끊긴 조각을 온전한 프레임으로 셈), 조립기의 손실 나누기 버그, CPU 고정과
  실시간 정책의 순서 버그는 모두 컴파일을 통과했다. 테스트와 측정으로 찾았다.
- 실행 비용: 프레임 조립(`take`)과 시계 다시 맞추기는 PC에서 1 ms 미만이었는데 보드(Cortex-A9)에서는 7 ms, 20 ms였다.
  둘 다 실시간 수신 스레드에서 돌아 큐 대기 p99가 6~7 ms가 됐다. 포인트마다 부르던 삼각함수 때문에 수신 스레드가 CPU를
  22% 써서, 같은 CPU에 일반 부하가 있으면 굶었다. 안전성과 상관없는 문제라 언어는 아무 말도 하지 않고, 타깃에서 재야
  보인다 ([5-board.md](runs/5-board.md)).

## 11. 빌드 구성: cfg와 feature

- `#[cfg(target_os = "linux")]`: UDP 수신, systemd 알림, 신호, 스레드 설정은 리눅스에서만 빌드된다. pcap 읽기와 해석기는
  어디서나 빌드된다.
- cargo feature `ros2`: ROS 2 발행(`r2r`)은 빌드할 때 설치된 ROS 2를 읽어 바인딩을 만들고 ROS 2 공유 라이브러리에
  동적 링크된다. 그래서 기본은 끄고, CI와 보드용 정적 빌드(`armv7-unknown-linux-musleabihf`)에서는 뺀다.
  Observer 구조라 ROS 2 소비자를 빼도 수신 루프는 그대로다.

## 12. 테스트가 쉬운 이유

- 단위 테스트는 같은 파일의 `#[cfg(test)] mod tests`에 둔다. 해석기는 바이트 배열만 받으므로 합성 패킷으로 시험한다.
- 참조 구현 출력과 포인트 단위로 맞추는 골든 테스트(`crates/vlp16/tests/golden.rs`), loopback 소켓을 실제로 여는
  테스트(`udp.rs`), 자기 자신에게 SIGTERM을 보내는 테스트(`signal.rs`)까지 `cargo test` 하나로 돈다 (67개).
