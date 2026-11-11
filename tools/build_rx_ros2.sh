#!/bin/bash
# ROS 2 발행을 켠 lidar-rx 빌드 → target/ros2/release/lidar-rx
#
# - r2r은 빌드할 때 설치된 ROS 2의 메시지 정의를 읽어 Rust 바인딩을 만들므로, ROS 2 환경을 불러온 셸에서 빌드한다.
# - 빌드 폴더를 따로 둔다(target/ros2). 같은 target/에 두면 ROS 2 없이 하는 일반 빌드가 같은 실행 파일을
#   덮어써서, --ros2를 줬을 때 "built without the ros2 feature" 오류가 난다.
# (setup.bash가 정의 안 된 변수를 참조하므로 set -u는 쓰지 않음)
set -eo pipefail
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/.."
cargo build --release -p rx --features ros2 --target-dir target/ros2 "$@"
echo "built: target/ros2/release/lidar-rx"
