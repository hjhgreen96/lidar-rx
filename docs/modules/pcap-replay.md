# `pcap-replay` - 센서 대신 보내기

녹화된 VLP-16 패킷을 원래 간격대로(또는 정해진 드리프트를 넣어) UDP로 다시 보낸다. 표준 라이브러리 소켓만
써서 Windows에서도 빌드된다 (보드로 보낼 때는 Windows에서 직접 실행: WSL은 NAT라 송신 타이밍에 지연이 더해짐).

```text
 crates/pcap-replay/src/
 ├── main.rs       인자, 파일 읽기, 송신 루프(--gap 구간은 건너뜀), 요약
 ├── schedule.rs   패킷마다 "시작으로부터 몇 ns 뒤에 보낼지" (시계를 읽지 않는 순수 계산)
 ├── pacer.rs      그 시각까지 기다리기 (sleep + 바쁜 대기), 늦음 분포 (고정 크기 칸)
 └── echo.rs       --echo: 받는 쪽이 되돌려 보낸 패킷으로 왕복 시간 (보낸·받은 시각 모두 이 PC 시계)

 pcap ─PcapSource─► (캡처 시각, 페이로드) 전부 메모리 ─Schedule─► send_at_ns(k, i), payload(k, i)
                                                                     │
                                                         Pacer::wait_until ─► send_to
```

## 사용법

```bash
pcap-replay data/vlp16.pcap                                   # 원래 캡처 간격으로 한 번, 127.0.0.1:2368
pcap-replay data/vlp16.pcap --pace sensor --drift-ppm 100 --loops 3
pcap-replay data/vlp16.pcap --to 192.168.2.99:2368 --loops 0  # 보드로 끝없이
```

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `--to` | 127.0.0.1:2368 | 받을 주소 |
| `--pace` | capture | `capture`: 원래 캡처 시각 간격 / `sensor`: 센서 타임스탬프 간격 |
| `--drift-ppm` | 0 | `--pace sensor`에서만: 간격에 (1 + X ppm)을 곱함 |
| `--speed` | 1.0 | 재생 속도 배율 |
| `--loops` | 1 | 반복 횟수, 0이면 끝없이 |
| `--gap AT:FOR` | 없음 | 시작 AT 초부터 FOR 초 동안 보낼 패킷을 보내지 않음 (선을 뽑은 것처럼: 센서는 계속 돌고 그동안의 패킷만 사라짐) |
| `--echo ADDR:PORT` | 없음 | 이 주소에서 되돌아온 패킷을 받아 왕복 시간을 잼. 패킷마다 기록하므로 `--loops N`이 있어야 함 |
| `--echo-relay ADDR:PORT` | 없음 | `--echo`로 받은 것을 그대로 이 주소로 다시 보냄 (보드가 넘긴 것 → WSL의 RViz) |
| `--echo-csv FILE` | 없음 | 왕복 시간을 패킷마다: `send_s`(송신 시작 기준 초), `rtt_us` |

## 왕복 시간 - `echo.rs` (`--echo`)

```text
 송신 루프: 보내기 직전에 (내용 해시 → 보낸 시각)을 표에 적음 ──send_to──► 보드 lidar-rx
                                                                          │ --forward (받은 그대로)
 Echo 스레드: recv ◄──────────────────────────────────────────────────────┘
             해시로 표에서 찾아 왕복 = 지금 − 보낸 시각
```

보낸 시각과 받은 시각을 이 PC의 한 시계로 재므로 두 기기의 시계를 맞출 필요가 없다. 받는 쪽이 패킷을 바꾸지 않고 넘기므로
내용이 곧 이름이다 (반복마다 센서 타임스탬프를 다시 쓰므로 반복이 달라도 겹치지 않음). 테스트 `matches_returned_packets_and_relays_them`:
loopback 소켓 하나가 보드 대신 받은 것을 되돌려 보내고, 모르는 패킷 하나를 더 보냄 → 3개 짝지음, 모르는 것 1, 넘긴 것 4.

보드에서 잰 값 (Windows → 보드 → Windows, [5-board.md](../runs/5-board.md) 11절): 가운데값 0.37 ~ 0.41 ms, p99 13 ~ 14 ms.
큰 값은 보드가 아니라 PC(Windows) 안에서 생긴다: USB 어댑터로 받은 패킷을 NDIS가 CPU 0 전용·보통 우선순위 작업 스레드로
넘기는데, 그 스레드가 CPU 0의 다른 스레드 뒤에서 기다린다 (NIC 단계에서 잰 왕복은 p99 2.6 ms, 5-board.md 11절).
이 프로그램 자신도 원인이 된다: 제때 보내려고 CPU 하나를 쉬지 않고 쓰므로(`pacer.rs`의 바쁜 대기), CPU 0에 올라가면
왕복 가운데값이 0.37 → 12 ~ 13 ms가 된다. Windows에서 왕복을 잴 때는 CPU 0에서 빼서 실행하거나(PowerShell:
`$p = Start-Process pcap-replay.exe … -PassThru; $p.ProcessorAffinity = 0xFFFE` → p99 13 → 7 ~ 11 ms), 반대로 CPU 0에 두고
우선순위를 낮춘다(`$p.ProcessorAffinity = 1; $p.PriorityClass = 'BelowNormal'` → p99 1.8 ms, 대신 송신 늦음 p99 0 → 182 µs).
Windows에서는 되돌려 보낼 곳(`--echo-relay`)이 닫혀 있으면 다음 `recv`가 `ConnectionReset`으로 실패하므로 그 오류는 넘어간다.

## 두 가지 간격

```text
 --pace capture   캡처 PC가 받은 시각 간격 그대로
                  → 원래 수신 흔들림(최대 약 1 ms)과 원래 두 시계의 관계(−53 ppm)까지 재현
 --pace sensor    센서 타임스탬프 간격 × (1 + drift)
                  → 흔들림 없는 깨끗한 간격. 받는 쪽 시계 추정기가 drift를 되찾는지 보는 "정답을 아는 시험"
```

## 반복 재생

9.4 s 파일을 이어 붙인다. 받는 쪽이 시각이 되돌아간 것(재시작)으로 보지 않도록 **센서 타임스탬프를 이어지게
다시 쓴다** (`PacketView::set_timestamp_us`).

```text
 반복 k의 패킷 i:  센서 시각 = 원래 시각 + k × 주기       주기 = 파일 길이(센서 시각) + 패킷 간격 1327 µs
                   정시(3,600,000,000 µs)를 넘으면 0부터 다시 (센서와 같게)
```

`--pace capture`로 반복하면 반복이 이어지는 곳마다 틈이 생긴다: 보내는 간격은 캡처 시계 기준 길이로,
타임스탬프는 센서 시계 기준 길이로 이어지는데 두 길이가 −53 ppm × 9.4 s ≈ 0.5 ms 다르다. 그래서 원래 관계를
재현하는 시험은 반복 없이 한 번만 보낸다 (실제로 3회 반복하면 −49 ppm으로 흐려짐).

## 제때 보내기 - `pacer.rs`

1.327 ms 간격은 OS의 sleep 정밀도와 비슷한 크기다.

```text
 나쁜 방법:  매번 sleep(간격)                → sleep은 늘 조금 늦게 깸 → 늦음이 쌓여 점점 밀림
 이 방법:    마감 = 시작 + 예정 오프셋 (절대 시각)
            마감 − 여유까지 sleep, 남은 구간은 시계를 보며 바쁜 대기

   ──────── sleep ────────┬─ spin ─┐
                          │  여유   │ 마감
  ────────────────────────┴────────┴────► 시간
```

늦게 깨도 다음 마감은 원래 계획대로라 늦음이 쌓이지 않는다. 여유: 리눅스 200 µs(고해상도 타이머), Windows 2 ms
(기본 타이머가 거칢).

늦음 기록은 메모리가 고정이다. 패킷마다 늦음을 목록에 쌓으면 `--loops 0`(끝없이)으로 돌릴 때 메모리가 계속
는다 (하루 6500만 개). 그래서 1 µs 단위 칸 10,001개(0 ~ 10 ms, 그보다 늦은 것은 마지막 칸)에 개수만 세고, 최대값은
따로 둔다. 끝날 때 칸을 훑어 중앙값·p99를 낸다 (`Pacer::lateness`, 테스트 `lateness_summary_from_buckets`).

WSL loopback 결과 (3회 반복, 21,279 패킷, [실행 기록](../runs/2-rx-clock-ros2.md)): 계획 28.242 s → 실제 28.242 s,
늦음 p50 0 µs, p99 9 µs, 최대 1.4 ms (100 µs 넘게 늦은 것 28개 - WSL은 가상 머신이라 가끔 vCPU가 멈춤).
최대값과 100 µs 넘은 수는 실행마다 다르다 (다른 실행에서 최대 2.9 ms, 57개).
보드로 보낼 때는 보드에서 `tcpdump`로 받은 간격을 원본과 비교해 송신 쪽부터 검증한다.
