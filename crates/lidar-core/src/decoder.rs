//! 경계 2 - 센서 모델 (Strategy 패턴).
//!
//! "패킷 바이트 → 포인트 → 프레임"의 방법은 센서 모델마다 다르다. 위쪽 코드는 `SensorDecoder`만 알고,
//! 어떤 모델의 해석기가 들어 있는지는 모른다. 모델별 해석 방식을 갈아 끼울 수 있게 하는 것이 Strategy다.

use crate::{Frame, Packet};
use std::fmt;

pub trait SensorDecoder {
    /// 패킷 하나를 넣는다. 이 패킷으로 프레임 하나가 완성되면 그 프레임을 돌려준다.
    ///
    /// 한 패킷은 한 바퀴의 작은 조각(VLP-16은 약 4.8°)이라, 패킷 하나로 완성되는 프레임은 많아야 하나다.
    fn push(&mut self, packet: &Packet) -> Result<Option<Frame>, DecodeError>;

    /// 입력이 끝났을 때, 모으던 중인 프레임을 마저 내보낸다 (없으면 `None`).
    fn flush(&mut self) -> Option<Frame>;

    /// 지금까지 센 것들. 모델과 무관한 항목만 (모델 고유 항목은 각 해석기의 자체 통계로).
    fn counters(&self) -> DecoderCounters;

    /// 다 쓴 프레임을 돌려준다. 해석기는 그 배열을 다음 프레임을 모을 때 다시 쓸 수 있다
    /// (프레임마다 새로 할당하지 않게). 기본은 버림.
    fn recycle(&mut self, _frame: Frame) {}
}

/// 해석기 공통 카운터 (누적).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DecoderCounters {
    pub packets: u64,
    pub malformed_packets: u64,
    /// 센서 시각 간격으로 판정한 손실
    pub missing_packets: u64,
    pub reordered_packets: u64,
    pub restarts: u64,
    pub frames: u64,
}

/// 해석기가 위쪽 코드에 알리는 오류. 종류마다 위쪽 코드가 할 일이 다르다.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// 이 패킷 하나가 깨졌음 (길이·플래그가 안 맞음). → 세고 버린 뒤 다음 패킷으로 계속.
    Malformed(String),
    /// 지원하지 않는 설정의 패킷 (예: 이중 반환 모드). → 계속해도 같은 패킷이 오므로 멈추고 설정을 고침.
    Unsupported(String),
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DecodeError::Malformed(why) => write!(f, "malformed packet: {why}"),
            DecodeError::Unsupported(why) => write!(f, "unsupported packet: {why}"),
        }
    }
}

impl std::error::Error for DecodeError {}
