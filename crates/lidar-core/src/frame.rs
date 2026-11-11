//! 해석기와 소비자 사이를 오가는 공통 데이터: 한 바퀴분 포인트 묶음.
//!
//! 구성은 ROS 2 `sensor_msgs/PointCloud2`와 같게 잡았다:
//! 헤더(시각 + 좌표계 이름) + 포인트마다 (x, y, z, intensity, ring, time).

use crate::{HostTime, SensorTime};

/// 포인트 하나. 좌표계는 `Frame::frame_id`, 축 방향은 ROS 규약(REP-103: x 앞, y 왼쪽, z 위).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    /// m
    pub x: f32,
    /// m
    pub y: f32,
    /// m
    pub z: f32,
    /// 반사 세기 (VLP-16은 0..255)
    pub intensity: f32,
    /// 수직각 오름차순 레이저 번호 (VLP-16: 0 = −15°, 15 = +15°)
    pub ring: u16,
    /// 측정 시각, `Frame::sensor_stamp` 기준 상대값 (s). PointCloud2의 `time` 필드와 같은 뜻.
    /// 해석기가 내보낸 직후의 프레임은 아직 `Frame::pending_time_shift`만큼 덜 옮겨져 있을 수 있다 (`settle_point_times`).
    pub time: f32,
}

/// 한 바퀴분 포인트 묶음.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// 대표 시각 (센서 시계). 기본은 프레임의 마지막 패킷 시각 - 공식 드라이버와 같은 기준.
    pub sensor_stamp: SensorTime,
    /// 같은 순간의 호스트 시계 값. 시계 추정(`clock`)이 채우기 전에는 `None`.
    pub stamp: Option<HostTime>,
    /// 좌표계 이름 (ROS `frame_id`)
    pub frame_id: String,
    pub points: Vec<Point>,
    /// 이 프레임에 발사를 보탠 패킷들의 두 시각, 도착 순서대로. 경계에 걸친 패킷은 양쪽 프레임에 모두 있다.
    /// 시계 추정(`clock`)의 입력이다.
    pub packet_times: Vec<PacketTime>,
    pub stats: FrameStats,
    /// 포인트 `time`을 `sensor_stamp` 기준으로 만들려면 더 빼야 하는 값 (s). 0이면 이미 맞춰져 있다.
    /// 해석기는 실시간 수신 스레드에서 점 배열(약 27,000개)을 훑지 않으려고 이 값만 적어 두고, 수신 루프가
    /// 소비자에게 넘기기 전에 다른 스레드에서 `settle_point_times`를 부른다 (보드에서 이 훑기가 1.9 ms였다).
    pub pending_time_shift: f32,
}

impl Frame {
    /// 포인트 `time`을 `sensor_stamp` 기준으로 맞춘다. 해석기가 미뤄 둔 경우에만 일하고, 두 번 불러도 같다.
    pub fn settle_point_times(&mut self) {
        if self.pending_time_shift != 0.0 {
            let shift = self.pending_time_shift;
            self.points.iter_mut().for_each(|p| p.time -= shift);
            self.pending_time_shift = 0.0;
        }
    }
}

/// 패킷 하나가 찍힌 센서 시각과, 호스트가 받은 시각.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PacketTime {
    pub sensor: SensorTime,
    pub received_at: Option<HostTime>,
}

/// 프레임이 어떻게 만들어졌는지. 소비자가 "믿을 만한 프레임인가"를 판단하는 근거.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FrameStats {
    /// 센서 타임스탬프 간격으로 판정한, 이 프레임 범위에서 빠진 패킷 수. 빠진 구간에 자르는 각도가 있으면
    /// 앞뒤 두 프레임에 각도로 나눠 센다 (자르는 각도에 걸친 패킷은 양쪽 모두에서 빠졌으므로 양쪽에 셈).
    /// 반 바퀴 넘게 빠져 그 자리에서 끊은 경우는 끊기 전 프레임에 모두 센다.
    pub missing_packets: u32,
    /// 자르는 각도에서 시작했는가. 시작 직후·흐름 재시작 직후의 첫 프레임은 한 바퀴의 중간부터라 `false`.
    pub starts_at_cut: bool,
    /// 자르는 각도에서 끝났는가. 입력이 끝나 `flush`로 내보낸 마지막 프레임은 `false`.
    pub ends_at_cut: bool,
}

impl FrameStats {
    /// 자르는 각도에서 시작해 자르는 각도에서 끝났는가 (한 바퀴 범위인가. 그 안에 빠진 패킷이 있을 수 있음)
    pub fn is_full_turn(&self) -> bool {
        self.starts_at_cut && self.ends_at_cut
    }

    /// 한 바퀴 범위이고 빠진 패킷도 없는가 - 온전한 한 바퀴
    pub fn is_complete(&self) -> bool {
        self.is_full_turn() && self.missing_packets == 0
    }
}
