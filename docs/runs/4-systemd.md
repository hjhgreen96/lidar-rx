## 설치

deploy/install.sh가 전용 사용자 lidar, 실행 파일, 실행 인자·ROS 2 환경 파일, 커널 모듈과 부팅 때 적재 설정, 서비스 두 개를 설치한다. 소켓 버퍼 상한(net.core.rmem_max)은 지금 값이 모자랄 때만 올린다.

```console
$ sudo deploy/install.sh
== user lidar
== /usr/local/bin/lidar-rx (target/ros2/release/lidar-rx), /etc/lidar-rx/
== kernel module lidar_mon (loaded at boot by systemd-modules-load)
systemd-modules-load skipped (virtualization: wsl); loading lidar_mon with modprobe now
== socket receive buffer limit
net.core.rmem_max is already 4194304 (>= 1048576): not lowering it, sysctl.d file not installed
== services
installed. start: sudo systemctl enable --now lidar-rx lidar-record
```

```console
$ grep -v "^#" /etc/lidar-rx/lidar-rx.env; cat /sys/module/lidar_mon/parameters/port
LIDAR_RX_ARGS=--udp 0.0.0.0:2368 --rcvbuf 1048576 --status 10 --cpu 1 --rt-prio 50 --ros2
2368
```

## 켜기 - READY=1을 받아야 시작됨

Type=notify: lidar-rx가 소켓·버퍼·소비자 스레드·수신 스레드 설정을 끝내고 READY=1을 보낼 때까지 systemctl이 기다린다. 녹화 서비스는 After=lidar-rx.service라 그 뒤에 시작한다. 서비스는 root가 아닌 lidar 사용자로 돌지만 LimitRTPRIO=50 덕에 수신 스레드 하나만 SCHED_FIFO 50이 된다.

```console
$ sudo systemctl enable --now lidar-rx lidar-record 2>&1 | grep -v "^Created symlink"; deploy/status.sh
lidar-rx      active/running  pid 520876  restarts 0
lidar-record  active/running  pid 520888  restarts 0
lidar-rx became active 20 ms after exec (READY=1)
  status: receiving
```

```console
$ sudo journalctl -u lidar-rx --since @$(cat /tmp/lidar-stage4-t0) -o cat --no-pager
Starting lidar-rx: VLP-16 LiDAR receiver...
lidar-rx: systemd watchdog: notifying every 1s
lidar-rx: listening on 0.0.0.0:2368, socket receive buffer 2097152 B (requested Some(1048576))
lidar-rx: receive thread: SCHED_FIFO priority 50, CPUs 1 (read back from the kernel)
Started lidar-rx: VLP-16 LiDAR receiver.
```

## 센서가 조용할 때 - 재시작 없음

패킷이 없어도 수신 루프는 0.5초 타임아웃마다 돌며 WATCHDOG=1을 보낸다. 멈춘 것이 아니라 조용한 것이므로 재시작하지 않고 상태만 바뀐다 (상태 줄은 10초마다).

```console
$ sleep 11; deploy/status.sh
lidar-rx      active/running  pid 520876  restarts 0
lidar-record  active/running  pid 520888  restarts 0
lidar-rx became active 20 ms after exec (READY=1)
  status: rx: 0 pkt/s 0.0 frm/s (sensor quiet) | missing 0 reorder 0 restart 0 kernel-drops 0 | clock (collecting) | queue-drops stats 0 ros2 0
```

## 받으면서 녹화 - 수신 재시작에도 녹화 유지

replay가 두 바퀴(약 19초)를 보낸다. 그동안 lidar-rx를 재시작한다. 녹화 서비스는 순서만 묶였고(BindsTo는 안 씀) 같은 프로세스로 남아, 발행자가 돌아오면 다시 받는다.

```console
$ target/release/pcap-replay data/vlp16.pcap --pace sensor --loops 2 > /dev/null 2>&1 & sleep 11; deploy/status.sh
lidar-rx      active/running  pid 520876  restarts 0
lidar-record  active/running  pid 520888  restarts 0
lidar-rx became active 20 ms after exec (READY=1)
  status: rx: 668 pkt/s 8.9 frm/s | missing 0 reorder 0 restart 0 kernel-drops 0 wait-p99 55 us | clock +0.01 ppm (lsq -0.15) p99 13 us | queue-drops stats 0 ros2 0
```

```console
$ sudo systemctl restart lidar-rx; deploy/status.sh
lidar-rx      active/running  pid 520972  restarts 0
lidar-record  active/running  pid 520888  restarts 0
lidar-rx became active 20 ms after exec (READY=1)
  status: receiving
```

## 죽거나 멈추면 - 재시작, watchdog

kill -9: Restart=on-failure로 0.5초 뒤 다시 띄운다. kill -STOP: 프로세스는 살아 있지만 루프가 멈춰 WATCHDOG=1이 끊긴다 → 2초 뒤 systemd가 SIGABRT로 죽이고(코어 덤프) 다시 띄운다.

```console
$ sudo kill -9 $(systemctl show lidar-rx -p MainPID --value); sleep 2; deploy/status.sh
lidar-rx      active/running  pid 521002  restarts 1
lidar-record  active/running  pid 520888  restarts 0
lidar-rx became active 19 ms after exec (READY=1)
  status: receiving
```

```console
$ sudo kill -STOP $(systemctl show lidar-rx -p MainPID --value); sleep 5; deploy/status.sh
lidar-rx      active/running  pid 521034  restarts 2
lidar-record  active/running  pid 520888  restarts 0
lidar-rx became active 216 ms after exec (READY=1)
  status: receiving
```

```console
$ sudo journalctl -u lidar-rx --since @$(cat /tmp/lidar-stage4-t0) -o short-monotonic --no-pager | grep -E "code=|Watchdog|SIGABRT|restart counter" | cut -c1-150
[131216.633014] 20221421-p03 systemd[1]: lidar-rx.service: Main process exited, code=killed, status=9/KILL
[131217.306820] 20221421-p03 systemd[1]: lidar-rx.service: Scheduled restart job, restart counter is at 1.
[131220.556586] 20221421-p03 systemd[1]: lidar-rx.service: Watchdog timeout (limit 2s)!
[131220.556997] 20221421-p03 systemd[1]: lidar-rx.service: Killing process 521002 (lidar-rx) with signal SIGABRT.
[131220.764659] 20221421-p03 systemd[1]: lidar-rx.service: Main process exited, code=dumped, status=6/ABRT
[131221.306794] 20221421-p03 systemd[1]: lidar-rx.service: Scheduled restart job, restart counter is at 2.
```

## 녹화 끝내기 - SIGTERM에도 메타데이터 기록

systemctl stop은 SIGTERM을 보낸다. rosbag2(Humble 0.15.7 이후)는 SIGTERM도 Ctrl+C처럼 받아 남은 메시지와 메타데이터를 쓰고 끝낸다. 메시지 수는 replay 19초 × 10 Hz보다 적다: 재시작 세 번 동안 빠진 프레임이다.

```console
$ sudo systemctl stop lidar-record
```

```console
$ sudo bash -c "source /opt/ros/humble/setup.bash > /dev/null; ros2 bag info /var/lib/lidar/bags/*/" | grep -E "^(Files|Bag size|Duration|Messages)"
Files:             20261110-132104_0.db3
Bag size:          64.2 MiB
Duration:          18.107650831s
Messages:          116
```

## 수신 끝내기 - 요약 출력, 커널 모듈과 수 일치

lidar-rx를 다시 띄워 replay 한 바퀴를 끊김 없이 받은 뒤 멈춘다. SIGTERM을 받으면 루프를 끝내고 소비자를 정리한 뒤 요약을 journal에 남긴다. 커널 모듈이 IP 계층에서 본 수와 대조한다.

```console
$ sudo systemctl restart lidar-rx
```

```console
$ target/release/pcap-replay data/vlp16.pcap --pace sensor 2>&1 | grep sent
pcap-replay: sent 7093 packets in 9.412 s (planned 9.412 s); lateness p50 0 us, p99 7 us, max 121 us, >100 us: 1
```

```console
$ sudo systemctl stop lidar-rx
```

```console
$ sudo journalctl -u lidar-rx -n 14 -o cat --no-pager | grep -E "^(lidar-rx: stopped|decoder|kernel module|consumer ros2)"
lidar-rx: stopped by signal
decoder: 7093 packets, 96 frames, malformed 0, product-id mismatches 7093, missing 0, reordered 0, restarts 0
kernel module (IP layer, /proc/lidar_mon): seen 7093, lost 0, reordered 0, restarts 1; seen − received here = 0
consumer ros2: queue drops 0; published 96, failed 0, stamped by receive time before the clock estimate 20
```

## 의존 관계

녹화 서비스는 lidar-rx 뒤에 시작하고(After), 같이 켜지지만(Wants) 같이 죽지는 않는다(BindsTo 없음). 모듈 적재 서비스는 sysinit.target 전이고 일반 서비스는 그 뒤라 순서는 따로 쓰지 않았다. WSL에서는 systemd가 자신을 컨테이너 안으로 보고 모듈 적재 서비스를 건너뛴다 (보드에서는 돈다).

```console
$ systemctl show lidar-record -p After -p Wants -p BindsTo | sed "s/ systemd-journald.socket//"
Wants=lidar-rx.service
BindsTo=
After=lidar-rx.service sysinit.target basic.target system.slice
```

```console
$ systemctl show systemd-modules-load -p Before -p ConditionResult | sed "s/ shutdown.target//"; systemd-detect-virt
Before=shutdown.target systemd-sysctl.service sysinit.target
ConditionResult=no
wsl
```

## 되돌리기

실험 스크립트가 같은 포트(2368)를 쓰므로 서비스를 지우고 원래대로 둔다.

```console
$ sudo deploy/uninstall.sh --purge
uninstalled
```

_기록한 날: 2026-11-10 13:21, 커널 6.18.54.1-microsoft-standard-WSL2_
