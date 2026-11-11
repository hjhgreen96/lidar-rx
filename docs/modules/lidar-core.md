# `lidar-core` - 경계와 공통 타입

구현이 하나도 없는 크레이트다. 다른 크레이트들이 서로 주고받는 모양만 정한다.

```text
 crates/lidar-core/src/
 ├── lib.rs       공개할 것 목록
 ├── source.rs    경계 1: Packet, Received, PacketSource
 ├── decoder.rs   경계 2: SensorDecoder, DecodeError, DecoderCounters
 ├── frame.rs     경계 2와 3 사이의 데이터: Frame, Point, PacketTime, FrameStats
 └── time.rs      두 시계: HostTime, SensorTime
```

## 따로 둔 이유

`vlp16`과 `lidar-io`가 서로를 모르게 하려면, 둘이 함께 쓰는 타입(`Packet`)을 둘 중 어디에도 둘 수 없다.
그래서 둘 다 아는 제3의 크레이트에 둔다. 이 크레이트는 아무것에도 의존하지 않으므로 누구나 쓸 수 있다.

```text
   lidar-io ──► lidar-core ◄── vlp16          ✔ 서로 모름
   lidar-io ──────────────────► vlp16          ✘ 입력 출처가 센서 모델을 알게 됨
```

## 경계 1 - `PacketSource`

```rust
pub trait PacketSource {
    fn recv(&mut self) -> std::io::Result<Received>;
    /// 커널이 버린 패킷 수 (소켓이면 수신 버퍼 넘침, 파일이면 None)
    fn kernel_drops(&self) -> Option<u64> { None }
    /// 다 쓴 패킷을 돌려줌 → 출처가 그 배열에 다음 패킷을 받음 (기본은 버림)
    fn recycle(&mut self, _packet: Packet) {}
}

pub enum Received {
    Packet(Packet),
    Timeout,   // 정해진 시간 동안 안 옴. 출처는 살아 있음
    End,       // 더 없음 (파일 끝)
}
```

"패킷이 없다"를 세 가지로 구분:

| 결과 | 뜻 | 쓰는 쪽(rx)이 할 일 |
|---|---|---|
| `Ok(Received::Timeout)` | 센서가 조용함 | 계속 기다림. 상태를 "센서 조용함"으로. systemd watchdog 신호는 보냄 - 루프는 살아 있으므로 |
| `Ok(Received::End)` | 파일이 끝남 | 남은 프레임을 `flush`하고 끝냄 |
| `Err(io::Error)` | 소켓·파일 오류 | 오류를 출력하고 프로세스를 끝냄. systemd가 재시작 |

`Timeout`이 따로 있어야 "센서가 꺼진 것"과 "프로그램이 멈춘 것"을 구분할 수 있다. 막혀서 영원히
기다리는 `recv`라면 둘이 구분되지 않는다.

`Packet`:

| 필드 | 타입 | 뜻 |
|---|---|---|
| `payload` | `Vec<u8>` | UDP 페이로드. Ethernet·IP·UDP 헤더는 이미 벗겨짐 |
| `received_at` | `Option<HostTime>` | 소켓이면 커널 수신 시각(`SO_TIMESTAMPNS`), pcap이면 캡처 시각 |

## 경계 2 - `SensorDecoder`

```rust
pub trait SensorDecoder {
    fn push(&mut self, packet: &Packet) -> Result<Option<Frame>, DecodeError>;
    fn flush(&mut self) -> Option<Frame>;
    fn counters(&self) -> DecoderCounters;   // packets, malformed, missing, reordered, restarts, frames
    fn recycle(&mut self, _frame: Frame) {}  // 다 쓴 프레임을 돌려줌 → 그 배열에 다음 프레임을 모음 (기본은 버림)
}
```

`counters`는 모델과 무관한 항목만 담는다. 그래서 위쪽 코드(rx의 상태 줄)가 어떤 모델인지 모르고도 손실·재시작을
보일 수 있다. 모델 고유 항목(VLP-16의 제품 ID 불일치 등)은 각 해석기의 자체 통계에 둔다.

- `push`는 패킷 하나를 넣는다. 그 패킷으로 프레임이 완성되면 `Some(frame)`.
  한 패킷은 한 바퀴의 작은 조각이라 완성되는 프레임은 많아야 하나다.
- 상태를 갖는다(`&mut self`): 프레임을 모으는 중이기 때문.
- `flush`는 입력이 끝났을 때 모으던 것을 내보낸다.
- `recycle`(두 경계 모두): 수신 스레드가 패킷·프레임마다 배열을 새로 할당하지 않게 다 쓴 것을 돌려받는 자리. 기본
  구현이 "버림"이라 구현하지 않아도 된다 (pcap 출처는 구현하지 않음). 왜 필요한지는 [rx.md](rx.md) "다 쓴 배열 돌려받기"

`DecodeError`는 종류마다 할 일이 달라서 둘로 나눴다.

| 종류 | 예 | 할 일 |
|---|---|---|
| `Malformed` | 길이가 1206이 아님, 플래그가 FF EE가 아님 | 이 패킷만 버리고 계속 |
| `Unsupported` | 이중 반환 모드 | 같은 패킷이 계속 올 것이므로 멈추고 설정을 고침 |

패킷 손실은 오류가 아니다. 프레임의 `stats.missing_packets`로 알린다.

## 데이터 - `Frame`, `Point`

ROS 2 `sensor_msgs/PointCloud2`와 같은 구성으로 잡아, ROS 2로 내보낼 때 필드를 그대로 옮기면 된다.

```text
 Frame                                       PointCloud2
 ├─ sensor_stamp : SensorTime  (센서 시계)    
 ├─ stamp        : Option<HostTime> ───────► header.stamp     (clock이 채움)
 ├─ frame_id     : String     ───────────────► header.frame_id
 ├─ points       : Vec<Point> ───────────────► data (포인트마다 아래 필드)
 │    x, y, z    : f32 [m]                       x, y, z       float32
 │    intensity  : f32                           intensity     float32
 │    ring       : u16                           ring          uint16
 │    time       : f32 [s, sensor_stamp 기준]     time          float32
 ├─ packet_times : Vec<PacketTime>            (ROS 메시지에는 없음. 시계 추정의 입력)
 │    sensor      : SensorTime   그 패킷의 센서 시각
 │    received_at : Option<HostTime>  그 패킷의 커널 수신 시각
 ├─ stats        : FrameStats                 (ROS 메시지에는 없음. 로그·통계용)
 │    missing_packets, starts_at_cut, ends_at_cut   →  is_full_turn() = 양 끝 모두 자르는 각도
 │                                                   is_complete()  = 그리고 빠진 패킷 0 (온전한 한 바퀴)
 └─ pending_time_shift : f32 [s]              (ROS 메시지에는 없음) 0이 아니면 points의 time이 아직 그만큼
                                               덜 옮겨짐 → settle_point_times()가 옮기고 0으로
```

`pending_time_shift`가 있는 이유 (2026-11-11): 해석기는 실시간 수신 스레드에서 돈다. 포인트 시각을 대표 시각 기준으로
옮기려면 점 배열(약 27,000개, 0.65 MB)을 한 번 훑어야 하는데, 보드(Cortex-A9)에서 이 일이 1.9 ms라 그동안 패킷이 소켓
큐에서 기다렸다. 그래서 해석기는 옮길 값만 적고, 수신 루프(`rx`)가 소비자에게 넘기기 전에 다른 스레드에서
`settle_point_times`를 부른다. 소비자가 받는 프레임은 늘 0이다.

`packet_times`를 프레임에 실은 이유: 시계 추정에는 패킷마다의 (센서 시각, 수신 시각) 쌍이 필요하다. 센서 시각은
해석기만 알고(바이트 안에 있고, 정시 넘김을 펼쳐야 함), 수신 시각은 입력 출처만 안다. 프레임에 함께 실으면 시계
추정이 해석기 뒤에 붙는 한 단계가 되어, 해석기 인터페이스(`SensorDecoder`)를 바꾸지 않아도 된다.

## 시각 타입 - `HostTime`, `SensorTime`

| 타입 | 단위 | 시계 | 출처 |
|---|---|---|---|
| `HostTime(i64)` | ns, 1970-01-01부터 | 호스트 `CLOCK_REALTIME` | 커널 수신 시각, pcap 캡처 시각, ROS 2 시각 |
| `SensorTime(i64)` | µs, 기준점 의미 없음 | 센서 내부 시계 | 패킷 타임스탬프를 정시 넘김까지 펼친 값 |

둘을 같은 `i64`로 두면 섞어 써도 컴파일러가 모른다. 다른 타입이라 `SensorTime`을 `HostTime` 자리에
넣으면 컴파일 에러가 난다. 둘을 잇는 것은 `clock` 크레이트의 추정뿐이다.
