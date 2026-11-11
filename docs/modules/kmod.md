# `kmod/lidar_mon` - IP 계층에서 패킷을 세는 커널 모듈

```text
 kmod/
 ├── lidar_mon.c   모듈 전체 (약 300줄)
 └── Makefile      실행 중인 WSL 커널용 빌드 (보드용은 5단계)
```

실행 기록(입력·출력): [docs/runs/3-kmod.md](../runs/3-kmod.md) · 영상: [docs/media/run3_kmod.mp4](../media/run3_kmod.mp4)

- [1. 필요한 이유](#1-필요한-이유)
- [2. 어디에 붙나: netfilter PRE_ROUTING](#2-어디에-붙나-netfilter-pre_routing)
- [3. 훅 함수가 하는 일](#3-훅-함수가-하는-일)
- [4. 손실 판정 (정수 계산)](#4-손실-판정-정수-계산)
- [5. 동시성: CPU별 카운터와 잠금](#5-동시성-cpu별-카운터와-잠금)
- [6. 적재와 해제](#6-적재와-해제)
- [7. /proc/lidar_mon](#7-proclidar_mon)
- [8. 빌드·사용](#8-빌드사용)
- [9. eBPF였다면](#9-ebpf였다면)

---

## 1. 필요한 이유

사용자 공간에서는 "패킷이 안 왔다"만 보인다. 어디서 사라졌는지는 모른다. IP 계층에 관측점을 하나 두면
손실을 둘로 가를 수 있다.

```text
 센서 ── 케이블 ── NIC ── 드라이버 ── IP ──┬── UDP ── 소켓 수신 큐 ── lidar-rx
                                          │
                                   [lidar_mon이 셈]

 lidar_mon도 못 봄     → IP 계층 이전에서 사라짐 (케이블, NIC rx ring, 드라이버)
 lidar_mon은 봤는데     → IP 계층 이후에서 사라짐 (대부분 소켓 수신 버퍼 넘침)
 lidar-rx는 못 받음
```

| 이미 있는 카운터 | 한계 |
|---|---|
| `SO_RXQ_OVFL` (소켓 드롭 수) | 소켓 버퍼 넘침만. 그 앞에서 사라진 것은 모름 |
| `/proc/net/snmp`, `ethtool -S` | 장치·시스템 전체 합계. 이 흐름만 가를 수 없음 |
| lidar-rx의 `missing` (센서 타임스탬프 간격) | 어디서 사라졌든 합계 |
| **lidar_mon** | 이 흐름이 IP 계층에 도착했는지. 그리고 lidar-rx가 재시작하는 동안에도 계속 셈 |

## 2. 어디에 붙나: netfilter PRE_ROUTING

netfilter는 커널 네트워크 경로의 정해진 지점에 함수(훅)를 걸 수 있게 해 주는 틀이다 (iptables·nftables도 이 위에 있다).

```text
 수신 경로 (IPv4)

 드라이버 → netif_receive_skb → ip_rcv ─► [PRE_ROUTING] ─► 라우팅 결정 ─► [LOCAL_IN] ─► udp_rcv ─► 소켓
                                  │            │ 우선순위 순서로 훅 실행
                                  │            ├ NF_IP_PRI_FIRST (INT_MIN)  ← lidar_mon
                                  │            ├ RAW (−300), conntrack (−200), mangle (−150)
                                  │            └ NAT (−100) …
                                  └ IP 헤더 검사(버전, 길이, 헤더 체크섬)는 여기서 끝남
```

- PRE_ROUTING: IP 헤더 검사를 통과한 직후, 라우팅·방화벽보다 먼저. "IP 계층에 도착했다"에 가장 가깝다.
- `NF_IP_PRI_FIRST`: 같은 지점의 다른 훅(conntrack, 방화벽)보다 먼저 돈다 → 방화벽이 버린 패킷도 센다.
- 네트워크 네임스페이스마다 따로 등록한다. 여기서는 기본 네임스페이스(`init_net`)만 본다 → 컨테이너 안의
  트래픽은 보지 않는다.

## 3. 훅 함수가 하는 일

```text
 lm_hook(skb)                                         ← softirq 문맥: 잠들면 안 됨
   IP 헤더 읽기 (skb_header_pointer)
   UDP가 아니면                       → 통과
   조각난 패킷의 뒷조각 (offset ≠ 0)   → 통과 (UDP 헤더가 없어 포트를 모름)
   UDP 헤더 읽기, 목적지 포트 ≠ 2368   → 통과
   첫 조각                            → fragments++ , 통과
   seen++
   UDP 길이 ≠ 8 + 1206                → malformed++ , 통과
   페이로드 1200..1204 읽기 (타임스탬프 4 B + 반환 모드 1 B)
   lm_track(타임스탬프, 이중 반환인가, 지금 시각)   ← 손실·순서·재시작 판정
   return NF_ACCEPT                    ← 항상 통과. 관찰만 함
```

| 원리 | 코드에서 |
|---|---|
| **softirq 문맥**: 패킷 수신은 인터럽트 뒤의 softirq에서 처리된다. 잠들 수 없다 → 대기하는 메모리 할당(`GFP_KERNEL`), mutex 금지 | 훅 안에서 할당 없음, 잠금은 spinlock만 |
| **패킷이 한 덩어리라는 보장이 없음**: skb의 데이터는 선형 영역 + 페이지 조각으로 나뉠 수 있다 | `skb_header_pointer(skb, 오프셋, 길이, 임시 버퍼)`: 붙어 있으면 그 자리 포인터, 아니면 임시 버퍼에 복사 |
| **IP 헤더 길이는 가변**: 옵션이 있으면 20 B보다 큼 | UDP 위치 = `ihl × 4` |
| **정렬**: 4 B 값이 4의 배수 주소에 있다는 보장이 없음 | 바이트로 읽어 직접 조립: `t[0] \| t[1] << 8 \| …` (little-endian) |

## 4. 손실 판정 (정수 계산)

규칙은 사용자 공간 해석기(`vlp16::assemble`)와 같다. 커널은 부동소수점을 쓰지 않으므로 정수로 계산한다.

```text
 dt = 이번 타임스탬프 − 지난 타임스탬프 (µs)       정시 넘김(30분 넘게 뒤로 감)이면 + 3,600,000,000
                                                   30분 넘게 앞으로 감 = 정시 직후 늦게 온 지난 시간 패킷이면
                                                   − 3,600,000,000 (→ 조금 뒤로 간 것 = 순서 바뀜)

 −10 ms < dt ≤ 0         reordered++             (지난 기준은 그대로)
 dt ≤ −10 ms, dt > 1 s   restarts++              (재생 반복, 센서 재시작)
 그 밖                    n = (dt·1000 + 간격/2) / 간격      ← round(dt / 간격)을 정수로
                         lost += n − 1  (n > 1일 때)
 간격 = 1,327,104 ns (단일 반환: 발사 24회 × 55.296 µs), 이중 반환이면 663,552 ns (반환 모드 0x39)
```

- 도착 시각(`ktime_get_real_ns`)은 포트가 맞는 패킷에서만 읽는다. 훅은 이 컴퓨터에 오는 모든 IPv4 패킷에서
  불리므로, 거르기 전에 읽으면 상관없는 패킷마다 시계를 읽게 된다.
- 64비트 나눗셈은 `div64_u64`로 한다. 32비트 ARM(보드)에는 64비트 나눗셈 명령이 없어 `/` 연산자를 쓰면 링크
  에러가 난다 (`__aeabi_uldivmod`). 커널은 이를 위한 함수를 따로 둔다.

## 5. 동시성: CPU별 카운터와 잠금

훅은 여러 CPU에서 동시에 돌 수 있다 (패킷마다 처리하는 CPU가 다를 수 있음).

```text
                CPU 0                   CPU 1                 /proc 읽기 (프로세스 문맥)
  seen 등   [lm_cpu (CPU 0 몫)]     [lm_cpu (CPU 1 몫)]  ──►  모든 CPU 몫을 더함
            this_cpu로 자기 몫만      잠금 없음               u64_stats_fetch로 일관된 값

  순서 상태  ┌────────── lm (하나) : last_ts, lost, reordered, restarts ──────────┐
  (지난 패킷  spin_lock (훅, softirq)                    spin_lock_bh (읽기)
   과 비교)  └──────────────────────────────────────────────────────────────────┘
```

| 원리 | 코드에서 |
|---|---|
| **CPU별 카운터**: 단순 개수는 CPU마다 따로 세고 읽을 때 더한다 → 잠금도, 캐시 라인 경쟁도 없음 | `DEFINE_PER_CPU(struct lm_cpu_stats, lm_cpu)`, `this_cpu_ptr` |
| **32비트에서 64비트 값 읽기**: 32비트 CPU(보드의 Cortex-A9)는 u64를 두 번에 나눠 읽는다 → 쓰는 중에 읽으면 위·아래 절반이 다른 시점 값(찢어짐) | `u64_stats_sync`: 쓰는 쪽은 `update_begin/end`, 읽는 쪽은 `fetch_begin/retry`로 다시 읽음. 64비트에서는 비용 없이 사라짐. 6.1 전 커널을 32비트 단일 코어로 빌드하면 `fetch_begin`이 선점만 막아 읽는 도중 softirq(훅)가 끼어들 수 있으므로 `_irq` 판을 쓴다 (`lm_fetch_begin` 매크로, 5.15·6.1·6.2·6.6 소스로 확인) |
| **비교가 필요한 상태는 하나**: "지난 패킷"과 비교하므로 CPU별로 나눌 수 없음 → spinlock | `lm.lock` |
| **`spin_lock_bh` (읽기 쪽)**: /proc 읽기는 프로세스 문맥. 잠금을 잡은 채로 같은 CPU에 softirq(훅)가 끼어들어 같은 잠금을 잡으려 하면 영원히 기다린다(교착) → 읽는 동안 그 CPU의 softirq를 막음 | `lm_show` |
| 훅 쪽은 `spin_lock`만: 이미 softirq 안이라 같은 CPU에서 또 softirq가 끼어들 수 없음 | `lm_track` |

## 6. 적재와 해제

```text
 lm_init                                     lm_exit
   CPU별 u64_stats_init                        nf_unregister_net_hook   ← 먼저 훅을 뗌
   proc_create("/proc/lidar_mon")              proc_remove
   nf_register_net_hook ── 실패하면 proc_remove로 되돌리고 오류 반환
```

- 해제 순서: 훅을 먼저 떼야 새 패킷이 더는 이 모듈로 들어오지 않는다.
- 떼는 함수는 기다리지 않는다 (6.18 `net/netfilter/core.c`로 확인): `nf_unregister_net_hook`은 훅 목록에서
  빼기만 하고, 옛 목록은 `call_rcu`로 나중에 해제한다. 그래서 이미 훅에 들어간 다른 CPU가 잠깐 더 실행할 수 있다.
- 그래도 안전한 이유:

  ```text
   CPU 1: rcu_read_lock ── lm_hook 실행 중 ───────── rcu_read_unlock
   CPU 0:        rmmod → lm_exit → 훅 목록에서 뺌 → proc_remove → (모듈 로더) synchronize_rcu ─ 기다림 ─► 모듈 메모리 해제
                                                                     └ CPU 1의 읽기 구간이 끝나야 넘어감
  ```

  훅은 RCU 읽기 구간 안에서 돌고, 훅이 쓰는 데이터(`lm`, `lm_cpu`)는 모두 모듈 안의 정적 변수다. 모듈 로더가 모듈
  메모리를 풀기 전에 `synchronize_rcu()`로 진행 중인 읽기 구간이 끝나길 기다린다 (`kernel/module/main.c`의
  `free_module`). 만약 훅이 `kmalloc`한 메모리를 쓴다면, exit 함수에서 `synchronize_net()`으로 기다린 뒤 풀어야 한다.
- 커널 API는 버전마다 바뀐다: procfs는 5.6부터 `struct proc_ops`를 쓴다 (그 전은 `file_operations`).
  보드의 PYNQ v3(5.15 또는 6.6)도 5.6 이후라 그대로 빌드된다. 5.6 미만이면 컴파일 단계에서 `#error`로 알린다.

## 7. `/proc/lidar_mon`

```text
$ cat /proc/lidar_mon
port 2368                         감시하는 UDP 목적지 포트 (모듈 인자 port=)
seen 7093                         포트가 맞는 UDP 패킷 수
lost 0                            타임스탬프 간격으로 판정한 손실
reordered 0                       조금 뒤로 간 패킷
restarts 0                        흐름 재시작
malformed 0                       길이가 VLP-16 데이터 패킷이 아님
fragments 0                       조각난 IP 패킷 (VLP-16은 1248 B라 조각나지 않음)
last_timestamp_us 268471207       마지막 패킷의 센서 타임스탬프 (정시 이후 µs)
last_arrival_ns 1794193170238482910  마지막 패킷이 훅에 도착한 시각 (CLOCK_REALTIME)
```

"이름 값" 줄 형식이라 셸에서도(`awk`), 프로그램에서도 쉽게 읽는다. **lidar-rx는 모듈이 적재돼 있으면 시작과
수신 루프가 끝난 순간에 자동으로 읽어** 요약에 `seen − received here`를 보인다 (`crates/rx/src/kmod.rs`).

## 8. 빌드·사용

```bash
make -C kmod                                    # 0단계의 커널 트리로 빌드 (W=1, 경고 없음)
sudo insmod kmod/lidar_mon.ko                   # port=2368 기본. 다른 포트: sudo insmod kmod/lidar_mon.ko port=2370
cat /proc/lidar_mon
target/release/lidar-rx --udp 127.0.0.1:2368 --duration 11 &
target/release/pcap-replay data/vlp16.pcap
cat /proc/lidar_mon                             # seen 7093
sudo rmmod lidar_mon
```

결과 (WSL loopback, [실행 기록](../runs/3-kmod.md)): 보낸 7,093 = 모듈이 본 7,093 = lidar-rx가 받은 7,093,
손실·순서 바뀜·재시작 0.

컴파일러 (`kmod/Makefile`): 모듈은 커널을 빌드한 것과 같은 컴파일러로 빌드해야 한다. WSL에서는 `CROSS_COMPILE`이
비어 있으므로 `gcc-13`(0단계에서 커널 트리를 만든 것), 보드용 크로스 빌드에서는 `$(CROSS_COMPILE)gcc`를 쓴다.
다른 것을 쓰려면 `make -C kmod KCC=…`.

```bash
make -C kmod KDIR=<보드 커널 트리> ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-   # 크로스 빌드 (보드에서는 tools/board/build_kmod.sh로 직접 빌드)
```

## 9. eBPF였다면

같은 관찰을 eBPF(커널 안에서 안전하게 검증된 작은 프로그램을 돌리는 기능)로도 할 수 있다. 여기서 모듈을 택한
이유는 커널 API와 실행 문맥(softirq, CPU별 데이터, 잠금, RCU, 모듈 수명)을 직접 다뤄 보기 위해서다. 실제 운영이라면
잘못돼도 커널을 죽이지 않고(검증기가 막음) 다시 빌드 없이 여러 커널에서 돌아가는 eBPF가 나은 경우가 많다.
