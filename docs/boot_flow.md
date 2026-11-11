# 부팅 흐름: BootROM → U-Boot → 커널 → systemd → lidar-rx

Arty Z7-20(Zynq-7020: Cortex-A9 2코어 + FPGA)에 PYNQ-Z1 v3.0.1 이미지를 SD 카드로 올려, 시리얼 콘솔(115200)과 JTAG로
실제로 본 순서다. 이미지는 직접 만들지 않았다(PYNQ 기성품). 원본 로그는 [5-board_boot.log](runs/5-board_boot.log).

## 1. 한 장 그림

```text
 전원 (USB 5 V)
   │
 ① BootROM (칩 안, 바꿀 수 없음)  부팅 모드 핀(JP4)을 읽음 → BOOT_MODE 레지스터: JTAG 0, SD 5   ← JTAG로 읽어 확인
   │  SD의 첫 파티션(FAT)에서 BOOT.BIN을 찾아 FSBL을 칩 안 메모리(OCM)에 올림. 실패하면 멈추고 오류 코드를 남김
   ▼
 ② FSBL (BOOT.BIN 안, 98 KB)  PS 초기화: DDR, 클럭, MIO 핀 배치 → U-Boot를 DDR에 올려 넘김. 이 이미지의 FSBL은 출력이 없음
   ▼
 ③ U-Boot 2022.01 (BOOT.BIN 안, 964 KB)
   │  환경 변수: FAT에 없음 → 기본값. 2초 기다림(아무 키나 누르면 멈춤)
   │  mmc 0:1에서 boot.scr(2,776 B, U-Boot 스크립트) 실행
   │  image.ub(6.5 MB, 366 ms) = FIT 이미지: kernel-0 6.1 MiB + fdt-0 19.3 KiB, 각각 sha1 검사 OK
   │  device tree를 메모리에 놓고 "Starting kernel ..."
   ▼
 ④ Linux 5.15.19-xilinx-v2022.1  (커널 시작 기준 시각)
   │  cmdline: root=/dev/mmcblk0p2 rw rootfstype=ext4 rootwait     ← SD 두 번째 파티션이 루트
   │  DT를 보고 드라이버를 붙임: ethernet@e000b000 "cdns,zynq-gem" → macb (2.9 s), PHY RTL8211E 연결 (13.9 s)
   │  루트 파일시스템 마운트 → /sbin/init = systemd (커널 단계 3.9 s)
   ▼
 ⑤ systemd 249
   │  6.6 s   systemd-modules-load: lidar_mon 적재 (/etc/modules-load.d/lidar_mon.conf)
   │  15.2 s  lidar-rx.service 시작 → 소켓·버퍼·스레드 준비 뒤 READY=1 (Type=notify)
   │  33.7 s  multi-user.target (networking.service가 링크를 15 s 기다림)
   │  73.5 s  bootpy.service: PYNQ base overlay를 PL에 올림 (fpga_manager "writing base.bin") → DONE LED, LED 깜박임
   │  83.5 s  bootpy 끝 (CPU 34.7 s 사용)
   ▼
 수신 중 (상태 줄 "sensor quiet" → 패킷이 오면 754 pkt/s)
```

## 2. SD 카드 내용

| 파티션 | 파일 | 내용 | 읽는 주체 |
|---|---|---|---|
| 1 (FAT16, 130 MB, `/boot`) | `BOOT.BIN` (1,087,800 B) | 부트 이미지: FSBL + U-Boot + U-Boot용 DT (아래) | BootROM |
| | `boot.scr` | U-Boot 스크립트: 어디서 무엇을 올릴지 | U-Boot |
| | `image.ub` | FIT 이미지: 커널 + 커널용 DT (+ 해시) | U-Boot |
| | `boot.py` | PYNQ가 부팅 끝에 실행하는 파이썬 (overlay 올리기) | `bootpy.service` |
| 2 (ext4, 7.2 GB) | 루트 파일시스템 (Ubuntu 22.04 기반) | `/usr/local/bin/lidar-rx`, `/etc/systemd/system/lidar-rx.service`, `/lib/modules/…/updates/lidar_mon.ko` | 커널, systemd |

BOOT.BIN의 파티션 표를 직접 읽은 결과 (Zynq-7000 부트 이미지 형식, `XNLX` 머리):

| 이미지 | 크기 | 올리는 곳 | 대상 |
|---|---|---|---|
| `zynq_fsbl.elf` | 98,312 B | 0x0000_0000 (칩 안 메모리) | PS |
| `u-boot.elf` | 963,744 B | 0x0400_0000 (DDR) | PS |
| `system.dtb` | 19,768 B | 0x0010_0000 (DDR) | PS (U-Boot가 쓰는 DT) |

비트스트림이 없다 (PL로 가는 파티션이 없음). 많은 Zynq 설계는 FSBL이 BOOT.BIN의 비트스트림으로 PL을 먼저 설정하지만,
이 이미지는 리눅스가 다 뜬 뒤 `bootpy.service`가 fpga_manager로 올린다. 그래서 DONE LED는 부팅이 끝날 무렵에 켜진다.
DT는 두 개: BOOT.BIN 안의 DT는 U-Boot가, image.ub 안의 DT는 커널이 사용.

## 3. device tree → 드라이버

```text
 /proc/device-tree/axi/ethernet@e000b000
   compatible = "cdns,zynq-gem", "cdns,gem"     ← 이 문자열로 짝을 찾음
   status     = "okay"                           (ethernet@e000c000은 "disabled" → 드라이버가 안 붙음)
   phy-mode   = "rgmii-id"
        │  macb 드라이버의 일치 표(macb_dt_ids)에 "cdns,zynq-gem"이 있음 (커널 소스 macb_main.c, 지금은 "deprecated" 표시)
        ▼
 /sys/bus/platform/drivers/macb/e000b000.ethernet  → /sys/class/net/eth0
 dmesg: macb e000b000.ethernet eth0: Cadence GEM rev 0x00020118 at 0xe000b000 irq 34
```

주소(`e000b000`)는 Zynq PS 안의 GEM0 레지스터 주소이고, `irq 34`가 `/proc/interrupts`의 eth0 줄이다.

## 4. 부팅 순서 속 lidar-rx 서비스

- `lidar_mon`: `/etc/modules-load.d/lidar_mon.conf` → `systemd-modules-load.service`가 sysinit 단계에서 올림 (6.6 s). WSL에서는 이
  서비스가 가상화 조건으로 건너뛰어 수동으로 올렸는데, 보드에서는 그대로 돈다.
- `lidar-rx.service`: `multi-user.target`이 원함(`WantedBy`). 기본 의존성으로 sysinit 뒤에 시작해 15.2 s에 준비 완료.
  `network-online.target`은 기다리지 않는다: `0.0.0.0`에 바인드하므로 링크 전에도 소켓을 열 수 있고, 기다리면 15 s 늦어진다.
- `rmem_max`는 `/etc/sysctl.d/90-lidar-rx.conf`로 부팅 때 올라간다.
- 재부팅만으로 셋이 다 올라오는 것을 확인했다 ([5-board.md](runs/5-board.md) 9절).

## 5. 실물에서 알게 된 것

- BootROM이 실패하면 조용하다: JP4가 JTAG면 아무것도 출력하지 않고 JTAG를 기다린다. SD를 못 읽어도 출력 없이 멈추고,
  원인은 `REBOOT_STATUS` 레지스터의 BootROM 오류 코드로만 남는다 (JTAG로 읽음).
- JTAG는 PL 전용이 아니다: JTAG로 PS 쪽 메모리에도 프로그램을 올려 돌릴 수 있다 (xsct). 다만 전원을 끄면 사라지고,
  리눅스를 띄우려면 커널·루트 파일시스템을 따로 만들어야 한다.
- 전원: USB 하나로 쓰면 `reboot` 때 리셋이 반복되다 멈췄다. `REBOOT_STATUS`의 POR 비트가 1(전원이 실제로 떨어짐)이었고,
  반복 끝에 BootROM이 SD 부팅에 실패한 채 멈춤(오류 코드 0x200A). USB를 뽑았다 꽂는 냉부팅은 매번 정상.
- RTC가 없다: 꺼졌다 켜지면 시계가 마지막으로 저장한 시각으로 돌아간다 (실제로 10분 늦어 있었음). 네트워크가 없으면
  NTP로 맞지 않는다. lidar-rx의 큐 대기는 같은 시계끼리 빼므로 영향이 없지만, 로그 시각을 PC와 맞춰 볼 때는 주의.
- 부팅 직후 1분 넘게 다른 일이 CPU를 쓴다: `bootpy.service`가 CPU 34.7 s, `jupyter.service`가 52 s 동안 뜬다. 로봇이라면
  부팅 직후의 수신 지연을 따로 봐야 하는 이유다 (수신 스레드는 SCHED_FIFO라 영향이 작다).
