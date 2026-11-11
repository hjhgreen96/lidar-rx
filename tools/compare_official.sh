#!/bin/bash
# 공식 velodyne 드라이버(ROS 2 Humble 바이너리)와 lidar-rx 해석기의 포인트 수를 같은 pcap·같은 필터로 비교한다.
#
#   tools/compare_official.sh
#
# 공식 드라이버는 패킷 76개마다 프레임을 자르므로, lidar-rx 쪽도 76패킷 묶음으로 센다(count_points 예제).
# 차이가 나면, 공식 드라이버가 "블록 0 → 1 사이에서 0°를 넘는 패킷의 블록 0"을 버리는 것으로 설명되는지 확인한다
# (velodyne_pointcloud rawdata.cpp unpack_vlp16: last_azimuth_diff가 패킷마다 0으로 시작해 `continue`).
set -eo pipefail
source /opt/ros/humble/setup.bash
export PATH=$HOME/.cargo/bin:$PATH
cd "$(dirname "$0")/.."
CAL=/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml
[ -f "$CAL" ] || { echo "official driver not installed: sudo apt install ros-humble-velodyne" >&2; exit 1; }
OUT=$(mktemp -d)

# `ros2 run`은 노드를 자식 프로세스로 띄우고 SIGTERM을 넘기지 않는다 → 노드 이름으로 끝낸다.
# 전에 남은 노드가 있으면 /velodyne_points 발행자가 둘이 되어 수가 두 배로 세어지므로 시작 전에도 정리.
stop_nodes() { pkill -f velodyne_transform_node || true; pkill -f velodyne_driver_node || true; }
stop_nodes
trap stop_nodes EXIT

ros2 run velodyne_pointcloud velodyne_transform_node --ros-args \
  -p model:=VLP16 -p calibration:=$CAL -p min_range:=0.1 -p max_range:=200.0 -p organize_cloud:=false \
  > "$OUT/transform.log" 2>&1 &
python3 tools/ros2_count.py /velodyne_points 14 reliable "$OUT/official.txt" &
CNT=$!
sleep 1.5
ros2 run velodyne_driver velodyne_driver_node --ros-args \
  -p model:=VLP16 -p pcap:=$PWD/data/vlp16.pcap -p read_once:=true -p rpm:=600.0 \
  > "$OUT/driver.log" 2>&1 &
wait $CNT
stop_nodes

echo "--- lidar-rx decoder, same 76-packet groups"
cargo run -q --release -p vlp16 --example count_points -- data/vlp16.pcap 76 "$OUT/mine.txt"
echo "--- explained by the official driver skipping block 0?"
python3 tools/official_drop_check.py data/vlp16.pcap "$OUT/mine.txt" "$OUT/official.txt"
