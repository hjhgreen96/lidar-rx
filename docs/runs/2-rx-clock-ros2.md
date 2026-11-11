## 단위 테스트 (시계 추정, 입출력, replay 일정)

```console
$ cargo test -q -p clock -p lidar-io -p pcap-replay 2>&1 | grep -E "test result"
test result: ok. 9 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.09s
test result: ok. 11 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.03s
test result: ok. 4 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

## 녹화 파일 해석 + 원래 캡처의 시계 관계

pcap의 캡처 시각을 호스트 시각으로 써서, 원래 캡처 PC와 센서 사이의 시계 관계를 추정한다 (−53 ppm).

```console
$ cargo run -q --release -p rx -- --pcap data/vlp16.pcap 2>&1 | grep -E "^(decoder|clock|consumer)"
decoder: 7093 packets, 96 frames, malformed 0, product-id mismatches 7093, missing 0, reordered 0, restarts 0
clock: skew -53.457 ppm (least squares -53.352), least squares sits 233.9 us above the envelope; residual above envelope p50 231.6 us, p99 337.4 us, max 1218.6 us; 6787 samples over 9.01 s; resets 0
consumer stats: queue drops 0; 96 frames (94 full turns, 27034 points each), missing packets 0
```

## 정답을 아는 시계 시험 (UDP loopback)

replay가 센서 시각 간격 × (1 + 100 ppm)으로 보내면(센서 시계가 100 ppm 느린 상황) lidar-rx의 추정 기울기는 +100 ppm이어야 한다(양수 = 호스트 시계가 더 빠름). 두 번째는 원래 캡처 간격 그대로 → 원래 관계(약 −53 ppm) 재현.

```console
$ tools/known_drift.sh 100 3 2>&1 | grep -E "^===|^pcap-replay: sent|^clock:"
=== sensor pace, drift +100 ppm, 3 loops
pcap-replay: sent 21279 packets in 28.242 s (planned 28.242 s); lateness p50 0 us, p99 9 us, max 1383 us, >100 us: 28
clock: skew +100.019 ppm (least squares +99.971), least squares sits 2.2 us above the envelope; residual above envelope p50 1.0 us, p99 14.2 us, max 1387.3 us; 21113 samples over 28.02 s; resets 0
=== capture pace, 1 loop
pcap-replay: sent 7093 packets in 9.412 s (planned 9.412 s); lateness p50 0 us, p99 10 us, max 221 us, >100 us: 16
clock: skew -53.527 ppm (least squares -53.241), least squares sits 236.1 us above the envelope; residual above envelope p50 233.3 us, p99 357.7 us, max 1220.0 us; 6787 samples over 9.01 s; resets 0
```

## ROS 2 발행 확인

lidar-rx --ros2로 /velodyne_points를 발행하는 동안 다른 프로세스에서 QoS와 주기를 본다.

```console
$ source /opt/ros/humble/setup.bash; target/ros2/release/lidar-rx --udp 127.0.0.1:2368 --ros2 --duration 10 2>/dev/null & target/release/pcap-replay data/vlp16.pcap --pace sensor > /dev/null 2>&1 & sleep 4; ros2 topic info -v /velodyne_points | grep -E "Node name|Topic type|Endpoint type|Reliability|Durability"; timeout 4 ros2 topic hz /velodyne_points | tail -n 4; wait
Node name: lidar_rx
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
  Reliability: RELIABLE
  Durability: VOLATILE
average rate: 10.013
	min: 0.099s max: 0.101s std dev: 0.00060s window: 12
average rate: 10.005
	min: 0.099s max: 0.101s std dev: 0.00063s window: 22
```

RViz 화면은 README의 데모 영상 (`docs/media/demo.mp4`, `tools/rviz_demo.sh`).

## 공식 velodyne 드라이버와 포인트 수 비교

같은 pcap·같은 필터(0.1 ~ 200 m)로 공식 드라이버(ROS 2 Humble 바이너리)를 돌리고, 공식처럼 76패킷 묶음으로 센다. 차이는 공식 드라이버가 0°를 넘는 패킷의 블록 0을 버린 만큼인지 예측과 대조한다.

```console
$ tools/compare_official.sh 2>&1 | tail -n 7
first 3: [27282, 27287, 27300], last 3: [27297, 27299, 27325]
--- explained by the official driver skipping block 0?
groups: ours 93, official 93
packets where 0° falls between block 0 and 1: 9
predicted total drop 285, actual total difference 285
groups with a difference (index, predicted, actual): [(19, 32, 32), (22, 32, 32), (34, 32, 32), (44, 32, 32), (48, 32, 32), (52, 31, 31), (62, 32, 32), (72, 31, 31), (75, 31, 31)]
all groups match the prediction
```

_기록한 날: 2026-11-09 13:35, 커널 6.18.54.1-microsoft-standard-WSL2_
