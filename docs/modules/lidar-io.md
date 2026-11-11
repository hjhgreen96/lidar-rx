# `lidar-io` - 입력 출처 구현

`lidar_core::PacketSource`의 구체 구현(어댑터)들이 들어 있다: pcap 파일과 실시간 UDP 소켓, 그리고 출처를 감싸는
`ForwardSource`.

```text
 crates/lidar-io/src/
 ├── lib.rs           공개할 것 목록
 ├── pcap.rs          pcap 파일 형식 읽기 (PcapReader, Record)
 ├── net.rs           Ethernet → IPv4 → UDP 헤더 벗기기 (parse_udp)
 ├── pcap_source.rs   위 둘을 묶어 PacketSource로 (PcapSource)
 ├── udp.rs           실시간 UDP 수신 (UdpSource, 리눅스 전용): 커널 수신 시각, 소켓 드롭 수, 타임아웃
 └── forward.rs       어떤 출처든 감싸서 받은 패킷을 그대로 다른 주소로도 보냄 (ForwardSource, Decorator)

 pcap 파일 ──► PcapReader::next_record ──► Record { captured_at, data(Ethernet 프레임) }
                                                    │ parse_udp
                                                    ▼
                                            UdpDatagram { 포트, payload }
                                                    │ 목적지 포트 == 2368 만
                                                    ▼
                         PcapSource::recv ──► Received::Packet(Packet { payload, received_at })
```

## pcap 파일 형식 - `pcap.rs`

tcpdump·Wireshark가 쓰는 파일 형식이다. 머리에 전역 헤더가 하나 있고, 그 뒤로 (레코드 헤더 + 캡처한
바이트)가 반복된다.

```text
 ┌──────────────┬──────────────┬───────────────────┬──────────────┬───────────────────┬─ …
 │ 전역 헤더 24B │ 레코드 헤더 16B│ 캡처한 바이트       │ 레코드 헤더 16B│ 캡처한 바이트       │
 └──────────────┴──────────────┴───────────────────┴──────────────┴───────────────────┴─ …

 전역 헤더                                   레코드 헤더
 ┌────────┬───────┬────────┬───────┬───────┬────────┐   ┌────────┬────────┬────────┬────────┐
 │ magic  │version│thiszone│sigfigs│snaplen│linktype│   │ ts_sec │ts_frac │incl_len│orig_len│
 │   4    │ 2 + 2 │   4    │   4   │   4   │   4    │   │   4    │   4    │   4    │   4    │
 └────────┴───────┴────────┴───────┴───────┴────────┘   └────────┴────────┴────────┴────────┘
                                             1=Ethernet    초      µs/ns   저장 길이  원래 길이
```

| magic (파일 첫 4 바이트) | 뜻 |
|---|---|
| `D4 C3 B2 A1` | little-endian 파일, 시각은 µs |
| `4D 3C B2 A1` | little-endian 파일, 시각은 ns |
| `A1 B2 C3 D4` | big-endian 파일, 시각은 µs |
| `A1 B2 3C 4D` | big-endian 파일, 시각은 ns |

파일을 만든 컴퓨터의 바이트 순서로 저장되므로, magic을 두 가지 순서로 읽어 보고 맞는 쪽으로 이후 값을
모두 읽는다 (`PcapReader::new`, `u32_at`).

- `incl_len < orig_len`이면 캡처할 때 snaplen 때문에 잘린 것이다. 샘플 파일은 잘린 레코드가 없다.
- 레코드 헤더를 읽다가 파일이 끝나면 정상 종료(`None`), 레코드 중간에서 끝나면 잘린 파일이라 오류다
  (`read_exact_or_eof`).
- 캡처 시각은 캡처한 컴퓨터의 커널이 찍은 값이다. 실시간 수신의 `SO_TIMESTAMPNS`와 같은 성질이라
  시계 추정 분석에 쓸 수 있다.

깨진 파일을 받아도 패닉 없이 오류로 - 파일 안의 값을 믿고 그대로 쓰면, 깨진 파일 하나로 프로그램이 죽거나
메모리를 수 GB 잡을 수 있다:

| 필드 | 확인 | 안 하면 |
|---|---|---|
| `incl_len` | ≤ 262,144 B (tcpdump·Wireshark의 최대 snaplen) | 4 GB 버퍼를 잡으려다 메모리 부족으로 죽음 |
| `ts_frac` | µs 파일이면 < 1,000,000, ns 파일이면 < 1,000,000,000 | 초 경계를 넘는 틀린 시각이 시계 추정에 들어감 |

테스트 `implausible_fields_are_errors_not_panics`.

## Ethernet / IPv4 / UDP - `net.rs`

pcap에는 링크 계층(Ethernet)부터 저장돼 있다. 실시간 소켓에서는 커널이 이 헤더들을 벗겨 주지만,
파일에서는 직접 벗겨야 한다.

```text
 한 Ethernet 프레임 = Ethernet 14 B + IPv4 20 B + UDP 8 B + VLP-16 페이로드 1206 B = 1248 B

 Ethernet (14 B)
 ┌────────────────┬────────────────┬─────────┐
 │ 목적지 MAC (6)  │ 출발지 MAC (6)  │ type (2)│   type 0x0800 = IPv4
 └────────────────┴────────────────┴─────────┘   type 0x8100 = VLAN 태그 4 B가 끼어 있음 → 그 뒤 type을 다시 읽음

 IPv4 (IHL × 4 B, 옵션이 없으면 20 B)
  0        1        2        3        4        5        6        7        8        9       10~11   12~15   16~19
 ┌────────┬────────┬─────────────────┬─────────────────┬─────────────────┬────────┬────────┬───────┬───────┬───────┐
 │ver│IHL │  TOS   │   전체 길이       │   식별자          │ flags│fragment  │  TTL   │프로토콜 │체크섬  │출발 IP │도착 IP │
 └────────┴────────┴─────────────────┴─────────────────┴─────────────────┴────────┴────────┴───────┴───────┴───────┘
  4 │ 5 → 20 B                                            MF 플래그·offset ≠ 0 → 조각         17 = UDP

 UDP (8 B)
 ┌──────────────┬──────────────┬──────────────┬──────────────┐
 │ 출발 포트 (2) │ 도착 포트 (2) │ 길이 (2)      │ 체크섬 (2)    │   도착 포트 2368 = VLP-16 데이터
 └──────────────┴──────────────┴──────────────┴──────────────┘
```

`parse_udp`가 확인하는 것:

| 확인 | 이유 |
|---|---|
| type이 IPv4 (VLAN 태그 하나는 건너뜀) | 다른 프로토콜(ARP, IPv6)은 대상이 아님 |
| IP 버전 4, IHL ≥ 5 | IP 헤더 길이가 옵션에 따라 다르므로 고정 20으로 가정하지 않음 |
| 조각(fragment)이 아님 | 조각난 IP 패킷은 첫 조각에만 UDP 헤더가 있음. VLP-16 패킷(1248 B)은 MTU 1500보다 작아 조각나지 않음 |
| 프로토콜 17 (UDP) | |
| IP 전체 길이까지만 봄 | 짧은 프레임은 Ethernet이 뒤에 패딩을 붙일 수 있음 |
| IP 전체 길이 ≥ IP 헤더 길이 | 깨진 길이로 범위를 거꾸로 자르면 패닉 → 건너뜀 (`corrupt_lengths_are_skipped_not_panicking`) |
| UDP 길이 ≥ 8, 데이터가 그만큼 있음 | 잘린 패킷 |

네트워크 헤더는 big-endian(네트워크 바이트 순서)이고, VLP-16 페이로드 안쪽은 little-endian이다.
한 패킷 안에 두 순서가 섞여 있다.

## `PcapSource`

- 목적지 포트가 지정한 값(VLP-16 데이터는 2368)인 UDP 패킷만 `Received::Packet`으로 내보낸다.
  위치 패킷(포트 8308) 등은 건너뛴다.
- 파일을 가능한 한 빨리 읽는다. 원래 간격대로 보내는 것은 `pcap-replay`의 일이다.
  해석 결과는 간격과 무관하므로 테스트에는 빠른 쪽이 좋다.
- 파일 끝이면 `Received::End`. `Timeout`은 나오지 않는다.

## `UdpSource` - 실시간 수신 (`udp.rs`, 리눅스 전용)

```text
 UdpSource::bind(주소, UdpOptions { recv_buffer_bytes, timeout })
   socket()·bind()                  표준 라이브러리 UdpSocket
   setsockopt SO_RCVBUF             요청값 (커널은 먼저 net.core.rmem_max로 자르고 그다음 두 배로 잡음
                                    → 실제 상한 = 2 × rmem_max)
   setsockopt SO_TIMESTAMPNS = 1    패킷마다 커널 수신 시각을 붙여 달라
   setsockopt SO_RXQ_OVFL = 1       패킷마다 이 소켓의 누적 드롭 수를 붙여 달라

 UdpSource::recv
   poll(소켓, 타임아웃 0.5 s)        아무것도 안 오면 → Received::Timeout (신호로 일찍 깨도, EINTR)
   recvmsg(소켓, 페이로드 버퍼, 제어 메시지 버퍼, MSG_DONTWAIT)
                                    기다리지 않고 읽음. 비어 있으면 → Received::Timeout
                                    (poll 뒤에도 비어 있을 수 있음: 커널이 읽는 순간 체크섬 오류로 버리면.
                                     막혀서 기다리면 센서가 조용할 때 루프가 서서 watchdog이 서비스를 죽임)
     ┌ 페이로드 ─────────────────► Packet::payload  (돌려받은 배열에 복사: 아래)
     └ 제어 메시지(cmsg) 목록
         SOL_SOCKET / SCM_TIMESTAMPNS ─► Packet::received_at   struct timespec (초, 나노초)
         SOL_SOCKET / SO_RXQ_OVFL     ─► kernel_drops()        u32 누적값
```

`recycle(packet)`: 수신 루프가 해석을 마친 패킷을 돌려주면 그 페이로드 배열을 들고 있다가 다음 패킷을 거기에 복사한다.
패킷마다(초당 754번) 1,206 B를 새로 할당하지 않게 ([rx.md](rx.md) "다 쓴 배열 돌려받기"). `ForwardSource`는 감싼 출처에
그대로 넘긴다. pcap 출처는 하지 않는다 (기본 구현: 버림).

### `recvmsg`와 제어 메시지

`recv`는 데이터만 주지만 `recvmsg`는 데이터와 함께 제어 메시지(cmsg)를 준다. 커널이 패킷에 대해 알려 줄
부가 정보가 여기에 담긴다.

```text
 msghdr
 ├── msg_iov      ──► [페이로드 버퍼 2048 B]
 └── msg_control  ──► [ cmsghdr | 데이터 | 패딩 ][ cmsghdr | 데이터 | 패딩 ] …
                        len·level·type           CMSG_FIRSTHDR / CMSG_NXTHDR로 차례로 훑음
                                                  CMSG_DATA로 데이터 위치
```

- 제어 메시지 버퍼는 `[u64; 16]`으로 잡는다: `cmsghdr`가 `size_t` 정렬을 요구하기 때문.
- 데이터는 정렬이 보장되지 않을 수 있어 `read_unaligned`로 읽는다.
- 데이터 길이 = `cmsg_len − 헤더 길이`. 커널이 준 값이라도 헤더보다 짧으면(깨진 메시지) 빼기가 음수로 넘어가므로
  `checked_sub`로 확인하고 그 자리에서 훑기를 멈춘다.
- `struct timespec`은 64비트 시스템에서 16 B(i64, i64)지만, 32비트 ARM(보드)의 옛 형식은 8 B(i32, i32)라
  데이터 길이로 구분한다 (`parse_timespec`).
- `libc`를 부르는 unsafe 코드는 모두 `sys` 모듈 안에 있고, 각 unsafe 블록 위에 안전한 이유를 적었다.

### 커널 수신 시각을 쓰는 이유

```text
 패킷 도착 ─► [커널이 시각 찍음: SO_TIMESTAMPNS] ─► 소켓 큐 ─► … 스레드가 깨어남 … ─► recv 반환 [여기서 시계를 읽으면]
                                                              └──── 스케줄링 지연 ────┘ 이것까지 섞임
```

### `SO_RXQ_OVFL`의 성질 (테스트로 확인)

값은 그 패킷이 큐에 들어갈 때의 누적 드롭 수이고, 0이면 아예 붙지 않는다.

```text
 버퍼 4096 B 요청(실제 8192 B)에 1206 B 패킷 64개를 읽지 않고 보냄
   큐에 들어간 몇 개  ── 넘치기 전에 들어감 → 드롭 수가 안 붙음 → 다 읽어도 kernel_drops() = 0
   나머지            ── 커널이 버림
 그 뒤 하나 더 보냄    ── 넘친 뒤 처음 들어간 패킷 → 드롭 수가 붙음 → kernel_drops() = 64 − 받은 수
```

그래서 측정할 때는 송신을 멈춘 뒤 패킷 하나를 더 받아 마지막 값을 확정하거나, 시작 값과 끝 값의 차이로 본다.

| 테스트 | 확인하는 것 |
|---|---|
| `receives_payload_with_kernel_timestamp` | 페이로드, 커널 수신 시각이 보내기 전·받은 후 사이 |
| `receives_the_next_packet_into_a_payload_given_back` | 돌려준 페이로드 배열에 다음 패킷이 담김 (배열 위치 그대로) |
| `times_out_when_quiet` | 30 ms 동안 없으면 `Timeout` |
| `reports_socket_drops_on_the_first_packet_queued_after_overflow` | 위 그림 |

## `ForwardSource` - 받은 패킷을 다른 주소로도 (`forward.rs`)

```text
 UdpSource ──► ForwardSource::recv() ──► 수신 루프 (받은 것 그대로)
                     └─ send_to(같은 페이로드) ──► 다른 컴퓨터 (보드 → PC의 lidar-rx --ros2 → RViz)
```

- 보드(32비트 ARM)에는 ROS 2가 없다 ([architecture.md](../architecture.md)). 보드가 실제 NIC로 받은 패킷을 PC로 넘겨
  PC에서 그린다 (`lidar-rx --forward ADDR:PORT`, [5-board.md](../runs/5-board.md)).
- `PacketSource`를 그대로 구현하므로 수신 루프는 감싼 출처인지 모른다. `kernel_drops`는 감싼 출처의 값.
- 보내기는 non-blocking이고 실패는 세기만 한다(`ForwardCounts`): 보여 주기용 경로가 끊겨도 수신은 멈추지 않는다.
- 세는 값은 `Rc<Cell<u64>>`다: 출처는 수신 스레드 한 곳에서만 쓰고, `Box<dyn PacketSource>`로 넘긴 뒤에도 요약에서
  읽어야 해서. `Rc`는 `Send`가 아니므로 다른 스레드로 넘기면 컴파일이 안 된다.

| 테스트 | 확인하는 것 |
|---|---|
| `passes_packets_through_and_sends_the_same_bytes` | 받은 패킷은 그대로, 같은 바이트가 나감, 패킷이 아닌 결과는 보내지 않음, `kernel_drops`는 감싼 출처의 값 |
| `records_when_each_forwarded_packet_was_handed_to_the_driver` | 송신 시각 기록(아래): loopback으로 3개를 넘기면 3개 모두 보낸 순서대로 받은 시각과 짝지어지고, 송신 시각이 보낸 동안 안에 있음 |

### 진단: 넘긴 패킷마다 드라이버에 넘긴 시각 (`with_tx_timestamps`, `lidar-rx --forward-tx-log`)

```text
 setsockopt(SO_TIMESTAMPING, TX_SOFTWARE | SOFTWARE | OPT_ID | OPT_TSONLY)
 send_to ─► … ─► 드라이버 xmit의 skb_tx_timestamp() ─► 그 시각이 이 소켓의 오류 큐로 (OPT_ID: 몇 번째 보낸 것인지)
 recvmsg(MSG_ERRQUEUE | MSG_DONTWAIT) ─► (번호, 시각)   ← recv()를 부를 때마다 비움 (패킷이 없을 때도)
```

- 받은 패킷의 커널 수신 시각(`SO_TIMESTAMPNS`)과 같은 보드 시계라 패킷마다 "커널 도착 → 드라이버에 넘김"이 바로 나온다.
  보드: p50 0.12 ms, p99 0.27 ms, 최대 0.77 ms (7,093개 모두)
- `OPT_TSONLY`: 오류 큐에 패킷 내용을 복사하지 않고 시각만. `OPT_ID`: 성공한 보내기마다 0부터 번호 → 기록의 위치와 같음
- 32비트 보드에서는 시각이 옛 형식(`i32` 둘 × 3)으로 올 수 있어 길이(24 / 48 B)로 구분 (수신 시각과 같은 방식)
- 패킷마다 기록을 쌓고 수신 스레드에서 recvmsg를 한 번 더 부르므로 길이가 정해진 진단 실행에만 쓴다
