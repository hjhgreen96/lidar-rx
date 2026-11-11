# `deploy/` - systemd 서비스로 돌리기 (4단계)

lidar-rx를 터미널에서 띄우는 프로그램이 아니라 리눅스가 관리하는 서비스로 만든다. 부팅 때 뜨고, 준비가 끝났을 때
"시작됨"이 되고, 죽거나 멈추면 다시 뜨고, 멈출 때는 요약을 남기고 끝난다. 녹화는 별도 서비스로 두고 ROS 2 토픽으로만
주고받는다. 커널 쪽 설정(모듈 자동 적재, 소켓 버퍼 상한)도 배포의 일부로 함께 설치한다.

```text
 deploy/
 ├── lidar-rx.service          수신 서비스: Type=notify, WatchdogSec=2s, Restart=on-failure, User=lidar, LimitRTPRIO=50
 ├── lidar-record.service      녹화 서비스: ros2 bag record /velodyne_points, After=/Wants=lidar-rx (BindsTo 없음)
 ├── lidar-rx.env              실행 인자 (LIDAR_RX_ARGS) → /etc/lidar-rx/
 ├── modules-load.d/lidar_mon.conf   부팅 때 lidar_mon 적재 (systemd-modules-load)
 ├── modprobe.d/lidar_mon.conf       모듈 인자 port=2368
 ├── sysctl.d/90-lidar-rx.conf       net.core.rmem_max ≥ 1 MB (지금 값이 모자랄 때만 설치)
 ├── install.sh / uninstall.sh       설치·되돌리기 (root)
 └── status.sh                       두 서비스 상태를 한눈에 (sudo 없이)
```

```bash
tools/build_rx_ros2.sh && make -C kmod                 # 일반 사용자로 빌드
sudo deploy/install.sh                                  # 설치만
sudo systemctl enable --now lidar-rx lidar-record       # 켜기 (부팅 때도)
deploy/status.sh                                        # 상태
journalctl -u lidar-rx -f                               # 로그 (상태 줄 10초마다, 끝날 때 요약)
sudo deploy/uninstall.sh --purge                        # 되돌리기 (--purge: 사용자 lidar, 녹화 파일까지)
```

실행 기록: [docs/runs/4-systemd.md](../runs/4-systemd.md) · [터미널 영상](../media/run4_systemd.mp4)

## 서비스 두 개와 의존 관계

```text
 부팅 ─► systemd-modules-load (lidar_mon 적재) ─► sysinit.target ─► … ─► lidar-rx ──READY=1──► lidar-record
                                                                          │                     │
                                                                          └── /velodyne_points ─┘  (ROS 2 토픽만)
```

| 서비스 | 하는 일 | 의존 설정 | 왜 |
|---|---|---|---|
| `lidar-rx` | 수신, 해석, 시계 추정, ROS 2 발행 | 순서를 따로 쓰지 않음 | 모듈 적재 서비스는 `Before=sysinit.target`이고 일반 서비스는 기본 의존성으로 `sysinit.target` 뒤라 이미 보장됨. `Requires=`도 안 씀: 모듈은 관찰용이라 없어도 수신은 돌아야 함. `network-online.target`도 안 기다림: `0.0.0.0`에 바인드하면 링크 전에도 소켓을 엶 |
| `lidar-record` | `ros2 bag record`로 녹화 | `After=` + `Wants=lidar-rx.service`, `BindsTo=` 없음 | lidar-rx가 READY를 보낸 뒤에 시작. 생명주기는 묶지 않음: 수신이 재시작돼도 녹화 프로세스는 그대로 있고 발행자가 돌아오면 다시 받음 |

## 설계 판단

| 설정 | 하는 일 | 판단 |
|---|---|---|
| `Type=notify` + `READY=1` | 소켓·버퍼·소비자 스레드·수신 스레드 설정이 끝난 뒤 "준비됨" | 프로세스가 떴다고 준비된 것이 아니다. `systemctl start`가 이것을 기다리고, 녹화 서비스도 이 뒤에 시작 |
| `WatchdogSec=2s` + `WATCHDOG=1` | 수신 루프가 돌 때만 1초마다 신호 | **센서가 조용한 것과 프로세스가 멈춘 것을 가른다**: 수신은 0.5초 타임아웃으로 기다려 패킷이 없어도 루프가 돌고 신호가 나감 → 상태만 "sensor quiet". 루프가 멈추면 신호가 끊겨 SIGABRT(코어 덤프) → 재시작 |
| 소비자 스레드는 watchdog에 안 넣음 | | 소비자는 센서가 조용하면 쉬는 것이 정상이고, 느리거나 멈추면 채널이 차서 `queue-drops`로 보임. 느린 소비자 때문에 수신까지 재시작하면 받을 수 있던 데이터를 잃음. 대신 아예 멈춘 소비자도 재시작되지 않는다 (아래 한계) |
| `STATUS=` | 상태 줄을 그대로 | `systemctl status`에서 수신률·손실·시계 추정을 바로 봄 |
| SIGTERM 처리 | 루프를 끝내고 소비자를 정리하고 요약을 journal에 | 처리기는 원자 플래그만 세움. `TimeoutStopSec=5s` 안에 못 끝나면 SIGKILL |
| `Restart=on-failure`, `RestartSec=500ms` | 비정상 종료·watchdog 만료 때 재시작 | 정상 종료(`systemctl stop`)는 재시작 안 함 |
| `StartLimitIntervalSec=30`, `StartLimitBurst=10` | 30초에 10번 시작하면 멈춤 | 곧바로 다시 죽는 문제(설정 오류, 포트 사용 중)는 재시작으로 안 풀리므로 실패 상태로 둠 |
| `User=lidar` + `LimitRTPRIO=50` | root가 아닌 전용 사용자 | 수신 스레드 하나만 코드에서 SCHED_FIFO 50으로 올리므로 그 우선순위까지만 허락. `CPUAffinity=`·`Nice=`는 안 씀: 프로세스의 모든 스레드(소비자, ROS 2/DDS)에 걸림 |
| `LimitMEMLOCK=64M` | `--lock-memory`(mlockall)가 프로세스 메모리를 고정할 수 있게 | 기본 8 MB는 스레드 스택만으로 찰 수 있음. 보드는 5.0 ~ 7.7 MB 고정. `--ros2`면 DDS 메모리까지 64 MB를 넘어 실패하므로 PC 설정(`lidar-rx.env`)에서는 `--lock-memory`를 켜지 않음 |
| `EnvironmentFile=` | 실행 인자, ROS 2 환경 | 셸의 `source`에 기대지 않음: 설치할 때 빈 환경에서 `setup.bash`를 불러 필요한 변수만 파일로 남김 |
| 두 서비스 같은 사용자 | | Fast DDS의 공유 메모리 파일은 만든 사용자 권한으로 생김 |
| 녹화 종료 | `systemctl stop` → SIGTERM | rosbag2(Humble 0.15.7 이후, 설치된 것은 0.15.17)는 SIGTERM도 Ctrl+C처럼 받아 메타데이터를 씀. 10초 안에 안 끝나면 SIGKILL → 그때는 `ros2 bag reindex` |
| 녹화 용량 | 60초마다 파일을 나눔 | 프레임 약 0.6 MB × 10 Hz ≈ 시간당 22 GB. **디스크가 찰 때의 정책(오래된 것 지우기)은 아직 없음** |

## 커널 쪽 설정

- 모듈 자동 적재: `lidar_mon.ko`를 `/lib/modules/<커널>/updates/`에 넣고 `depmod`, `/etc/modules-load.d/`에 이름,
  `/etc/modprobe.d/`에 인자(`port=2368`). 부팅 때 `systemd-modules-load`가 `modprobe`로 올린다.
  - WSL에서는 건너뛴다: 이 서비스에 `ConditionVirtualization=!container`가 있는데, WSL에서 systemd는 자신을 컨테이너
    안으로 본다(`systemd-detect-virt` → `wsl`). 그래서 install.sh가 이것을 알리고 `modprobe`로 직접 올린다. 같은
    설정으로 `systemd-modules-load`를 직접 실행하면 모듈이 올라가는 것은 확인했다. 부팅 때 자동 적재는 보드(5단계)에서 확인.
  - 커널이 바뀌면 그 커널용으로 다시 빌드·설치해야 한다 (`updates/`는 커널 버전별 폴더).
- 소켓 버퍼 상한: `--rcvbuf 1048576`이 실제로 잡히려면 `net.core.rmem_max ≥ 1 MB` (커널은 요청을 rmem_max로 자른 뒤 두
  배). 기본값(약 208 KB)이면 약 0.25초만 버틴다. install.sh는 지금 값이 모자랄 때만 sysctl.d 파일을 넣고 바로 적용한다.
  WSL은 이미 4 MB라 낮추지 않는다 (uninstall이 되돌릴 값을 기록해 둠).

## 확인한 것 (WSL, 2026-11-10)

| 확인 | 결과 |
|---|---|
| 시작 | `systemctl start`가 READY=1을 기다림: 실행 20 ~ 74 ms 뒤 active (ROS 2 노드 생성 포함). 녹화 서비스는 그 뒤에 시작 (13 ms 뒤, 기록 전 검증 실행) |
| 수신 스레드 | 사용자 lidar로 돌면서 `SCHED_FIFO priority 50, CPUs 1` (커널에서 다시 읽은 값, `LimitRTPRIO`) |
| 소켓 버퍼 | `1048576` 요청 → `2097152 B` |
| 센서가 조용 (11초) | 재시작 0, 상태 `rx: 0 pkt/s … (sensor quiet)` |
| 녹화 중 lidar-rx 재시작 | 녹화 서비스 PID 그대로, 녹화 계속 |
| `kill -9` | `code=killed, status=9/KILL` → 0.67초 뒤 재시작 (restart counter 1) |
| `kill -STOP` | 프로세스는 살아 있지만 신호가 끊김 → `Watchdog timeout (limit 2s)!` → SIGABRT → `code=dumped, status=6/ABRT` → 재시작 (counter 2). 멈춘 뒤 다시 뜨기까지 약 3초 |
| 녹화 정지 (SIGTERM) | `metadata.yaml`이 써지고 `ros2 bag info`가 읽음 (18.1초, 116개). 19초 × 10 Hz보다 적은 것은 재시작 세 번과, 새로 뜬 발행자를 녹화 쪽이 다시 찾는 동안 빠진 프레임 |
| lidar-rx 정지 (SIGTERM) | `stopped by signal` + 요약이 journal에: 7,093개 받음, 커널 모듈이 IP 계층에서 본 수와 차이 0, ROS 2로 96프레임 발행 |
| 의존 관계 | lidar-record: `After=lidar-rx.service`, `Wants=lidar-rx.service`, `BindsTo=` 비어 있음. systemd-modules-load: `Before=… sysinit.target` |
| 되돌리기 | 사용자·파일·서비스·모듈 모두 지워짐 (실험이 같은 포트 2368을 쓰므로 WSL에서는 지워 둠) |

- 요약의 커널 모듈 `restarts 1`은 맞는 값이다: 모듈은 계속 떠 있어서 앞의 replay(두 바퀴)가 끝난 시각에서 파일 처음으로
  돌아간 것을 봤다. lidar-rx는 새로 떴으므로 그 이전을 모른다 (`restarts 0`).
- WSL에서 코어 덤프는 `/wsl-capture-crash`(Windows 쪽)로 간다. 보드에서는 `kernel.core_pattern`이나 `systemd-coredump`로 정한다.

## 한계

- 멈춘 소비자는 자동으로 복구되지 않는다. watchdog은 수신 루프만 본다. 예를 들어 ROS 2 발행이 멈추면 수신은 계속 돌고
  상태 줄의 `queue-drops`만 늘며, 녹화에는 아무것도 들어가지 않는다. 고치려면 소비자마다 마지막으로 처리를 끝낸 시각을 두고,
  프레임이 계속 들어오는데 오래 진전이 없으면 watchdog 신호를 멈추면 된다. 다만 "느린 것"과 "멈춘 것"의 경계를 정해야 한다
- 지원 안 하는 반환 모드(dual) 패킷이 오면 lidar-rx는 오류로 끝난다. 재시작해도 같으므로 30초에 10번을 넘기면
  systemd가 실패 상태로 둔다 (`StartLimitBurst`). 센서 설정 문제는 재시작으로 안 풀린다는 판단
- 모듈은 설치한 커널용이다 (vermagic). 커널이 바뀌면 다시 빌드해서 다시 설치해야 한다 (자동으로 하려면 DKMS).
  모듈 적재가 실패해도 설치는 경고만 하고 계속한다: 모듈은 관찰용이라 없어도 수신은 돈다
- 소켓 버퍼 상한은 설치할 때의 값으로 판단한다. 그 값이 `sysctl -w`로 잠깐 올린 것이면 재부팅 뒤 내려간다

## 보드에서 (5단계, 2026-11-11)

같은 `install.sh`를 보드(Arty Z7, PYNQ Linux 5.15)에서 돌렸다 ([5-board.md](../runs/5-board.md)). ROS 2가 없으므로 실행 인자에서
`--ros2`를 빼고 녹화 서비스는 설치하지 않는다. 실제 하드웨어라 `systemd-modules-load`가 모듈을 올렸고, `rmem_max`
180,224 → 1,048,576. 전원을 다시 넣은 뒤 부팅만으로 모듈·서비스(수신 스레드 SCHED_FIFO 50 / CPU 1)·소켓 버퍼 상한이
올라오는 것을 확인했다.

## 남은 것

- 녹화 디스크 정책 (오래된 파일 지우기 또는 녹화 중지)
