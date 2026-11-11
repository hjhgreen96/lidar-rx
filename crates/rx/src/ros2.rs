//! ROS 2 발행 소비자 (cargo feature `ros2`).
//!
//! `Frame` → `sensor_msgs/PointCloud2`. 필드 구성은 공식 velodyne 드라이버와 같다.
//!
//! ```text
//!  포인트 하나 = 22 B (little-endian)
//!  ┌──────┬──────┬──────┬───────────┬──────┬──────┐
//!  │ x f32│ y f32│ z f32│intensity  │ ring │ time │
//!  │  0   │  4   │  8   │ f32  12   │u16 16│f32 18│
//!  └──────┴──────┴──────┴───────────┴──────┴──────┘
//!  header.stamp    = Frame::stamp (시계 추정으로 바꾼 호스트 시각). 추정이 아직 없으면 마지막 패킷의 수신 시각
//!  header.frame_id = "velodyne"
//! ```
//!
//! QoS는 REP 2003대로 시스템 기본(reliable, 깊이 10). 센서 데이터를 쓰는 쪽이 best effort로 구독하면
//! 그대로 연결된다 (reliable 발행자는 두 종류 구독자 모두와 연결됨).

use crate::consumer::FrameConsumer;
use lidar_core::{Frame, HostTime};
use r2r::builtin_interfaces::msg::Time;
use r2r::sensor_msgs::msg::{PointCloud2, PointField};
use r2r::std_msgs::msg::Header;

/// sensor_msgs/PointField의 자료형 번호
const UINT16: u8 = 4;
const FLOAT32: u8 = 7;
const POINT_STEP: u32 = 22;

pub struct Ros2Publisher {
    _node: r2r::Node,
    publisher: r2r::Publisher<PointCloud2>,
    published: u64,
    failed: u64,
    fallback_stamps: u64,
}

impl Ros2Publisher {
    pub fn new(node_name: &str, topic: &str) -> Result<Self, r2r::Error> {
        let ctx = r2r::Context::create()?;
        let mut node = r2r::Node::create(ctx, node_name, "")?;
        let publisher = node.create_publisher::<PointCloud2>(topic, r2r::QosProfile::default())?;
        Ok(Ros2Publisher {
            _node: node,
            publisher,
            published: 0,
            failed: 0,
            fallback_stamps: 0,
        })
    }
}

impl FrameConsumer for Ros2Publisher {
    fn name(&self) -> &'static str {
        "ros2"
    }

    fn consume(&mut self, frame: &Frame) {
        let stamp = match frame.stamp {
            Some(s) => Some(s),
            None => {
                self.fallback_stamps += 1;
                frame.packet_times.last().and_then(|p| p.received_at)
            }
        };
        match self.publisher.publish(&to_point_cloud(frame, stamp)) {
            Ok(()) => self.published += 1,
            Err(_) => self.failed += 1,
        }
    }

    fn summary(&self) -> String {
        format!(
            "published {}, failed {}, stamped by receive time before the clock estimate {}",
            self.published, self.failed, self.fallback_stamps
        )
    }
}

fn to_point_cloud(frame: &Frame, stamp: Option<HostTime>) -> PointCloud2 {
    let field = |name: &str, offset: u32, datatype: u8| PointField {
        name: name.into(),
        offset,
        datatype,
        count: 1,
    };
    let mut data = Vec::with_capacity(frame.points.len() * POINT_STEP as usize);
    for p in &frame.points {
        data.extend_from_slice(&p.x.to_le_bytes());
        data.extend_from_slice(&p.y.to_le_bytes());
        data.extend_from_slice(&p.z.to_le_bytes());
        data.extend_from_slice(&p.intensity.to_le_bytes());
        data.extend_from_slice(&p.ring.to_le_bytes());
        data.extend_from_slice(&p.time.to_le_bytes());
    }
    let width = frame.points.len() as u32;
    PointCloud2 {
        header: Header {
            stamp: to_ros_time(stamp.unwrap_or(HostTime(0))),
            frame_id: frame.frame_id.clone(),
        },
        height: 1,
        width,
        fields: vec![
            field("x", 0, FLOAT32),
            field("y", 4, FLOAT32),
            field("z", 8, FLOAT32),
            field("intensity", 12, FLOAT32),
            field("ring", 16, UINT16),
            field("time", 18, FLOAT32),
        ],
        is_bigendian: false,
        point_step: POINT_STEP,
        row_step: POINT_STEP * width,
        data,
        is_dense: true,
    }
}

fn to_ros_time(t: HostTime) -> Time {
    Time {
        sec: t.0.div_euclid(1_000_000_000) as i32,
        nanosec: t.0.rem_euclid(1_000_000_000) as u32,
    }
}
