# `tools/` - 데이터, 검증 스크립트, 커널 빌드

- Rust 라이브러리가 아닌 준비, 검증, 실험, 데모 스크립트

```text
 tools/
 ├── fetch_data.sh          샘플 pcap 받기 + 해시 확인 + 제품 ID 고친 사본 만들기
 ├── patch_product_id.py    사본 만들기 (fetch_data.sh가 부름)
 ├── make_golden.py         참조 구현 출력 → testdata/golden_vlp16.csv
 ├── requirements.txt       velodyne_decoder==3.1.0, numpy → tools/.venv (아래)
 │
 ├── known_drift.sh         정답을 아는 시계 시험: replay(+X ppm) → lidar-rx가 X를 되찾는가 (loopback)
 ├── build_rx_ros2.sh       ROS 2 발행을 켠 lidar-rx 빌드 → target/ros2/release/lidar-rx (일반 빌드와 폴더 분리)
 ├── rviz_demo.sh           replay + lidar-rx --ros2 + RViz를 한 번에 (RViz 창을 닫으면 모두 끝남, 뒤 인자는 replay로)
 ├── board_demo.sh          보드 데모를 녹화 없이 직접 보기: 보드 서비스 확인 → Windows replay(--echo-relay로 WSL에 넘김) → RViz 창 + 보드 로그.
 │                          RViz를 닫으면 다 정리 (README "보드에서 실행")
 ├── rviz/lidar.rviz        RViz 설정: Fixed Frame velodyne, /velodyne_points(best effort), 반사율 색
 ├── ros2_count.py          PointCloud2 토픽의 메시지 수·포인트 수·주기·필드 확인 (rclpy)
 ├── compare_official.sh    공식 velodyne 드라이버와 포인트 수 비교 (같은 pcap, 같은 필터)
 ├── official_drop_check.py 그 차이가 공식 드라이버의 "블록 0 건너뜀"으로 설명되는지 확인
 ├── exp1_rcvbuf.sh         실험 1: 수신 버퍼 크기별 큐 용량 + 버린 수 네 군데 대조 (sudo, 커널 모듈·트레이싱)
 ├── exp3_inline_threaded.sh 실험 3: 프레임마다 D ms 처리하는 소비자를 붙여 inline·threaded 비교 (일반 사용자)
 ├── exp4_sched.sh          실험 4: 수신 스레드 CPU 고정 + 같은/다른 CPU 부하 × 일반/SCHED_FIFO (sudo)
 │
 ├── board/                 보드(Arty Z7) - 5단계 (docs/runs/5-board.md). 크로스 빌드 설정은 저장소 맨 위 .cargo/config.toml
 │   ├── bundle.sh          ARM용 lidar-rx를 크로스 빌드해 deploy/와 함께 target/board/lidar-rx-board.tar.gz로 묶음
 │   ├── board_info.sh      보드에서 sudo로: 이미지·커널 버전·설정, 헤더 유무, 실시간 설정, NIC·인터럽트, 추적 도구,
 │                          부트 파티션 파일을 한 파일로 (모듈을 보드에서 빌드할지 크로스 빌드할지 정하는 근거)
 │   └── build_kmod.sh      보드에서 sudo로: 커널 헤더 트리에 빌드 도구(fixdep, modpost)를 처음 한 번 만들고 lidar_mon 빌드, vermagic 비교
 │
 └── wsl-kernel/
     ├── build.sh           실행 중인 WSL 커널과 같은 커널 트리 빌드 (모듈 빌드용)
     └── smoke/             그 트리로 만든 모듈이 실제로 올라가는지 보는 최소 모듈
```

## 검증 스크립트

| 스크립트 | 확인하는 것 | 결과 (2026-11-09, [실행 기록](../runs/2-rx-clock-ros2.md)) |
|---|---|---|
| `known_drift.sh 100 3` | `--pace sensor --drift-ppm 100`으로 보내면 추정 기울기 = +100 ppm | +100.019 ppm (여러 번 돌려 +100.01 ~ +100.02) |
| 〃 (두 번째 시험) | `--pace capture` 한 번: 원래 캡처의 관계(파일 분석 −53.457 ppm) 재현 | −53.527 ppm (여러 번 돌려 −53.1 ~ −53.5) |
| `ros2_count.py` | lidar-rx `--ros2` 출력: 10 Hz, 필드 구성, QoS 호환 | 약 10 Hz, best effort 구독자와 연결됨 (`ros2 topic hz`: 10.005 ~ 10.013 Hz) |
| `compare_official.sh` | 공식 드라이버와 76패킷 묶음별 포인트 수 | 93묶음 중 84개 같음, 9개는 공식이 블록 0을 버린 만큼(285개) 적음 - 예측과 정확히 일치 |

`compare_official.sh`는 시작 전과 끝난 뒤(중간에 실패해도, `trap`) 공식 드라이버 노드를 정리한다. 전에 띄운 노드가
남아 있으면 같은 토픽에 두 노드가 발행해 포인트 수가 두 배로 세어진다. `official_drop_check.py`는 공식 쪽 메시지가
0개이거나 묶음 수가 다르면 "일치"로 넘어가지 않고 실패한다.

## Python 환경 - `tools/.venv`

- 골든 파일 만들기(`make_golden.py`)에만 필요

```bash
sudo apt install -y python3-venv
python3 -m venv tools/.venv
tools/.venv/bin/pip install -r tools/requirements.txt
```

`ros2_count.py`는 ROS 2의 `rclpy`를 쓰므로 venv가 아니라 시스템 Python(`source /opt/ros/humble/setup.bash` 후
`python3`)으로 돈다. `official_drop_check.py`는 표준 라이브러리만 써서 어느 쪽이든 된다.

## 데이터 흐름

```text
 download.ros.org/data/velodyne/vlp16.pcap
        │ fetch_data.sh (sha256 84727cd4…a6 확인)
        ▼
 data/vlp16.pcap ───────────────────────────────► vlp16 골든 테스트, inspect_pcap 예제
        │ patch_product_id.py (페이로드 1205번 바이트 0x21 → 0x22)
        ▼
 data/vlp16_pid22.pcap
        │ make_golden.py (velodyne_decoder 3.1.0)
        ▼
 testdata/golden_vlp16.csv (git에 포함) ──────────► vlp16 골든 테스트
```

## 제품 ID를 고친 사본이 필요한 이유

샘플 pcap의 패킷은 제품 ID가 `0x21`(HDL-32E)로 찍혀 있지만, 패킷 간격·구조는 VLP-16이다.
참조 구현 `velodyne_decoder`는 이 ID 때문에:

- 자동 감지로는 HDL-32E로 해석해 ring 32개, 수직각 −30° 같은 틀린 결과를 내고,
- 모델을 VLP-16으로 지정하면 "Expected model ID 34, got 33" 오류로 거부한다.

그래서 비교용 사본에서만 그 바이트 하나를 바꾼다. lidar-rx 해석기는 원본을 그대로 읽고, 제품 ID 불일치는
세기만 한다 (`DecoderStats::product_id_mismatches`).

## 골든 파일 형식 - `make_golden.py`

```text
# velodyne_decoder 3.1.0, model VLP16, min_range 0.10 m, max_range 200.0 m, cut_angle None, ...
# packet,firing,ring,x,y,z,intensity,t_rel_frame_s,t_from_packet_us
0,0,8,2.72694564,1.18061984,0.0511370078,9,-0.100857817,2.1834
```

| 열 | 뜻 |
|---|---|
| `packet` | pcap 안 데이터 패킷 순서 (0부터) |
| `firing` | 패킷 안 발사 번호 0..23 |
| `ring` | 0..15 |
| `x, y, z` | m, ROS 좌표 |
| `intensity` | 반사율 |
| `t_rel_frame_s` | 참조 프레임의 대표 시각(마지막 패킷) 기준, s - 참조 출력 그대로 |
| `t_from_packet_us` | 그 포인트 패킷의 타임스탬프 기준, µs - 위 값에서 계산 |

참조 구현은 포인트에 패킷 번호를 주지 않고 프레임 안 발사 번호(`column`)만 준다. 그래서 프레임마다
어느 패킷들인지를 가정하지 않고 데이터로 구한다:

```text
 마지막 패킷 = 프레임 대표 시각(정시 이후 µs)과 타임스탬프가 같은 패킷
 패킷 수     = 발사 번호 개수 ÷ 24
 첫 패킷     = 마지막 − 패킷 수 + 1     → 앞 프레임 바로 다음인지 확인 (빈틈이 있으면 중단)
```

이렇게 확인해 보니 첫 프레임만 패킷 77개(0..76), 그 뒤는 76개씩(77..152, …)이었다.

### 확인된 참조 구현의 성질

- 3.1.0으로 고정: 3.0.0은 첫 포인트를 중복으로 내고 `column`이 전부 0으로 나온다.
- `t_from_packet_us`와 사양서 발사 오프셋의 차이는 −0.23 ~ +0.13 µs. 참조가 절대 시각(약 14억 초)을
  double로 다뤄 생기는 반올림이다 (그 크기에서 double의 해상도가 약 0.24 µs). 프레임마다 차이의 중앙값이
  달라지는 것도 같은 이유.

## WSL 커널 빌드 - `wsl-kernel/build.sh`

### 전체 빌드가 필요한 이유

커널 모듈은 커널이 내보낸 함수(예: `_printk`)를 부른다. 이 커널은 모듈 버전 검사(`CONFIG_MODVERSIONS`)가
켜져 있어, 모듈이 기대하는 함수 모양(체크섬)과 실행 중 커널의 것이 다르면 적재를 거부한다. 그 체크섬 목록
(`Module.symvers`)은 커널 전체를 빌드해야 나온다. 한 번만 하면 된다.

```text
 ./build.sh config
   1. github.com/microsoft/WSL2-Linux-Kernel 의 linux-msft-wsl-6.18.54.1 태그 압축 파일을 ~/src에 받음
   2. /proc/config.gz (실행 중 커널의 설정) → .config
   3. make olddefconfig
   4. 실행 중 커널 설정과 비교해 달라진 줄 출력
   5. make kernelrelease == uname -r 인지 확인 (다르면 중단)
 ./build.sh build    전체 빌드, 로그는 ~/src/…/build.log
```

### 맞춰야 했던 것

| 문제 | 원인 | 해결 |
|---|---|---|
| 버전 끝에 `+`가 붙음 | git 트리에서 `v6.18.54.1` 태그를 찾는데 이 저장소의 태그 이름은 `linux-msft-wsl-…`라 못 찾음 (`scripts/setlocalversion`) | `make LOCALVERSION=` 그리고 git 없이 압축 파일로 받음 |
| 컴파일러 차이 | 실행 중 커널은 GCC 13.2, Ubuntu 22.04 기본은 11.4 | `ubuntu-toolchain-r/test` PPA의 gcc-13 (13.4) |
| 빌드 마지막에 WSL VM이 재시작됨 | BTF 생성(pahole)이 디버그 정보가 든 vmlinux를 스레드 16개로 처리 → 메모리 16 GB 바닥 | BTF 단계만 `JOBS=2` |
| 소스를 /mnt/d에 두면 깨짐 | NTFS는 대소문자를 구분하지 않는데 커널에는 이름이 대소문자만 다른 파일이 있음 | ~/src (ext4) |

`config` 단계에서 나온 설정 차이는 컴파일러·어셈블러·링커 버전(`CONFIG_GCC_VERSION`, `CONFIG_AS_VERSION`,
`CONFIG_LD_VERSION`)과 `asm goto` 출력 기능 플래그(GCC 13.2에는 버그가 있어 꺼져 있었음)뿐이다. 모두 구조체 같은
타입 정의를 바꾸지 않으므로 체크섬에 영향이 없다.

### 확인 - `wsl-kernel/smoke/`

```bash
make -C tools/wsl-kernel/smoke
sudo insmod tools/wsl-kernel/smoke/smoke.ko && sudo rmmod smoke
dmesg | tail -3       # smoke: loaded / smoke: unloaded
```

2026-11-08에 확인: vermagic `6.18.54.1-microsoft-standard-WSL2 SMP preempt mod_unload modversions`가
실행 중 커널과 같고, 체크섬 불일치(`disagrees about version of symbol`)나 BTF 경고 없이 올라가고 내려감.
