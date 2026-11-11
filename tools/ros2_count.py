"""PointCloud2 토픽을 정해진 시간 동안 구독해 메시지 수와 포인트 수를 센다.

공식 velodyne 드라이버와 lidar-rx의 출력을 같은 기준으로 비교하려고 쓴다.
ROS 2 환경을 불러온 셸에서: python3 tools/ros2_count.py [토픽=/velodyne_points] [초=15] [sensor|reliable] [저장할 파일]
(저장할 파일을 주면 메시지마다 포인트 수를 한 줄씩 쓴다)

구독 QoS 기본은 REP 2003대로 센서 데이터용(best effort). 발행자가 reliable이어도 연결된다.
메시지를 하나도 놓치지 않고 세야 할 때(출력 비교)는 reliable.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/velodyne_points'
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    qos = qos_profile_sensor_data
    if len(sys.argv) > 3 and sys.argv[3] == 'reliable':
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    rclpy.init()
    node = Node('cloud_counter')
    widths, stamps, arrivals = [], [], []
    first = {}

    def on_cloud(msg):
        widths.append(msg.width * msg.height)
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        arrivals.append(time.time())
        first.setdefault('fields', [(f.name, f.offset, f.datatype) for f in msg.fields])
        first.setdefault('frame_id', msg.header.frame_id)
        first.setdefault('point_step', msg.point_step)

    node.create_subscription(PointCloud2, topic, on_cloud, qos)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    n = len(widths)
    print(f'{topic}: {n} messages in {seconds:.0f} s')
    if n:
        print(f'  frame_id {first["frame_id"]}, point_step {first["point_step"]}, fields {first["fields"]}')
        print(f'  points per message: min {min(widths)}, median {sorted(widths)[n // 2]}, max {max(widths)}, total {sum(widths)}')
        print(f'  first 3: {widths[:3]}, last 3: {widths[-3:]}')
        if n > 1:
            span = arrivals[-1] - arrivals[0]
            print(f'  rate {(n - 1) / span:.2f} Hz')
        lag = [a - s for a, s in zip(arrivals, stamps)]
        print(f'  arrival - header.stamp: median {sorted(lag)[n // 2] * 1e3:.2f} ms')
    if len(sys.argv) > 4:
        with open(sys.argv[4], 'w') as f:
            f.writelines(f'{w}\n' for w in widths)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
