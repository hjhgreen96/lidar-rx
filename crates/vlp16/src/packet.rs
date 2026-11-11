//! VLP-16 데이터 패킷(UDP 페이로드 1206 B)의 바이트 배치와, 검증된 패킷을 복사 없이 읽는 창.
//!
//! ```text
//!  오프셋  크기  내용
//!  0       100   블록 0  ┐
//!  100     100   블록 1  │  블록 = 플래그 FF EE (2) + 방위각 (2, 0.01°)
//!  …             …       │        + 채널 32개 × [거리 (2, 2 mm) + 반사율 (1)]
//!  1100    100   블록 11 ┘
//!  1200    4     타임스탬프: 정시 이후 µs (이 패킷 첫 발사의 시각)
//!  1204    1     반환 모드: 0x37 strongest, 0x38 last, 0x39 dual
//!  1205    1     제품 ID: 0x22 = VLP-16
//! ```
//!
//! 여러 바이트 값은 모두 little-endian (낮은 자리 바이트가 앞).

use crate::units::{Azimuth, RawDistance};
use std::fmt;

pub const PAYLOAD_LEN: usize = 1206;
pub const BLOCKS: usize = 12;
pub const BLOCK_LEN: usize = 100;
pub const CHANNELS_PER_BLOCK: usize = 32;
const CHANNEL_LEN: usize = 3;
const BLOCK_FLAG: [u8; 2] = [0xFF, 0xEE];
const TIMESTAMP_AT: usize = 1200;
const RETURN_MODE_AT: usize = 1204;
const PRODUCT_ID_AT: usize = 1205;

/// 사양서상 VLP-16의 제품 ID
pub const PRODUCT_ID_VLP16: u8 = 0x22;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReturnMode {
    /// 가장 센 반사 하나
    Strongest,
    /// 가장 늦은 반사 하나
    Last,
    /// 두 가지 모두 → 블록 두 개가 같은 발사를 나눠 담음 (패킷 하나가 담는 시간이 절반)
    Dual,
}

impl ReturnMode {
    fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x37 => Some(ReturnMode::Strongest),
            0x38 => Some(ReturnMode::Last),
            0x39 => Some(ReturnMode::Dual),
            _ => None,
        }
    }
}

/// 패킷이 VLP-16 데이터 패킷의 모양이 아닐 때.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PacketError {
    Length(usize),
    BlockFlag { block: usize, found: [u8; 2] },
    Azimuth { block: usize, raw: u16 },
    ReturnMode(u8),
}

impl fmt::Display for PacketError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PacketError::Length(n) => write!(f, "length {n} B (expected {PAYLOAD_LEN})"),
            PacketError::BlockFlag { block, found } => {
                write!(
                    f,
                    "block {block}: flag {:02X} {:02X} (expected FF EE)",
                    found[0], found[1]
                )
            }
            PacketError::Azimuth { block, raw } => {
                write!(f, "block {block}: azimuth {raw} >= 36000")
            }
            PacketError::ReturnMode(b) => write!(f, "unknown return mode 0x{b:02X}"),
        }
    }
}

impl std::error::Error for PacketError {}

/// 검증을 마친 패킷. 원래 바이트를 빌려 쓰기만 하므로 복사가 없다.
///
/// `parse`를 통과한 뒤에는 길이·플래그·방위각 범위가 보장되므로, 읽는 메서드들은 실패하지 않는다.
#[derive(Clone, Copy, Debug)]
pub struct PacketView<'a> {
    bytes: &'a [u8; PAYLOAD_LEN],
}

impl<'a> PacketView<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<Self, PacketError> {
        let bytes: &[u8; PAYLOAD_LEN] = bytes
            .try_into()
            .map_err(|_| PacketError::Length(bytes.len()))?;
        let view = PacketView { bytes };

        for i in 0..BLOCKS {
            let block = &bytes[i * BLOCK_LEN..][..BLOCK_LEN];
            if block[..2] != BLOCK_FLAG {
                return Err(PacketError::BlockFlag {
                    block: i,
                    found: [block[0], block[1]],
                });
            }
            let azimuth = view.block(i).azimuth();
            if azimuth.0 >= Azimuth::FULL_TURN {
                return Err(PacketError::Azimuth {
                    block: i,
                    raw: azimuth.0,
                });
            }
        }
        if ReturnMode::from_byte(bytes[RETURN_MODE_AT]).is_none() {
            return Err(PacketError::ReturnMode(bytes[RETURN_MODE_AT]));
        }
        Ok(view)
    }

    /// 이 패킷 첫 발사의 시각, 센서 시계로 정시 이후 µs (1시간마다 0으로 돌아감).
    pub fn timestamp_us(&self) -> u32 {
        u32::from_le_bytes(self.bytes[TIMESTAMP_AT..][..4].try_into().unwrap())
    }

    pub fn return_mode(&self) -> ReturnMode {
        ReturnMode::from_byte(self.bytes[RETURN_MODE_AT]).unwrap() // parse에서 확인함
    }

    pub fn product_id(&self) -> u8 {
        self.bytes[PRODUCT_ID_AT]
    }

    /// 타임스탬프만 바꾼 사본을 위해 (pcap-replay가 반복 재생할 때 시각을 이어 붙임).
    /// 길이가 맞지 않으면 아무것도 하지 않고 `false`.
    pub fn set_timestamp_us(payload: &mut [u8], timestamp_us: u32) -> bool {
        if payload.len() != PAYLOAD_LEN {
            return false;
        }
        payload[TIMESTAMP_AT..][..4].copy_from_slice(&timestamp_us.to_le_bytes());
        true
    }

    pub fn block(&self, i: usize) -> Block<'a> {
        let bytes: &'a [u8; PAYLOAD_LEN] = self.bytes;
        Block {
            bytes: &bytes[i * BLOCK_LEN..][..BLOCK_LEN],
        }
    }
}

/// 블록 하나 (100 B).
#[derive(Clone, Copy, Debug)]
pub struct Block<'a> {
    bytes: &'a [u8],
}

impl Block<'_> {
    /// 이 블록 첫 발사가 시작될 때의 방위각
    pub fn azimuth(&self) -> Azimuth {
        Azimuth(u16::from_le_bytes([self.bytes[2], self.bytes[3]]))
    }

    /// 채널 `c` (0..32). 단일 반환에서 0..15는 블록의 첫 발사, 16..31은 두 번째 발사.
    pub fn channel(&self, c: usize) -> Channel {
        let at = 4 + c * CHANNEL_LEN;
        Channel {
            distance: RawDistance(u16::from_le_bytes([self.bytes[at], self.bytes[at + 1]])),
            reflectivity: self.bytes[at + 2],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Channel {
    pub distance: RawDistance,
    pub reflectivity: u8,
}

#[cfg(test)]
pub(crate) mod test_packet {
    //! 테스트용 합성 패킷.
    use super::*;

    /// 블록 b의 방위각 = start + b × step (0.01° 단위), 모든 채널 거리 = distance_raw, 반사율 = 채널 번호.
    pub fn build(timestamp_us: u32, start: u16, step: u16, distance_raw: u16) -> Vec<u8> {
        let mut p = vec![0u8; PAYLOAD_LEN];
        for b in 0..BLOCKS {
            let block = &mut p[b * BLOCK_LEN..][..BLOCK_LEN];
            block[..2].copy_from_slice(&BLOCK_FLAG);
            let az = ((start as u32 + b as u32 * step as u32) % 36000) as u16;
            block[2..4].copy_from_slice(&az.to_le_bytes());
            for c in 0..CHANNELS_PER_BLOCK {
                let at = 4 + c * CHANNEL_LEN;
                block[at..at + 2].copy_from_slice(&distance_raw.to_le_bytes());
                block[at + 2] = c as u8;
            }
        }
        p[TIMESTAMP_AT..TIMESTAMP_AT + 4].copy_from_slice(&timestamp_us.to_le_bytes());
        p[RETURN_MODE_AT] = 0x37;
        p[PRODUCT_ID_AT] = PRODUCT_ID_VLP16;
        p
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_fields() {
        let bytes = test_packet::build(123_456, 35_990, 20, 500);
        let p = PacketView::parse(&bytes).unwrap();
        assert_eq!(p.timestamp_us(), 123_456);
        assert_eq!(p.return_mode(), ReturnMode::Strongest);
        assert_eq!(p.product_id(), PRODUCT_ID_VLP16);
        assert_eq!(p.block(0).azimuth(), Azimuth(35_990));
        assert_eq!(p.block(1).azimuth(), Azimuth(10)); // 0°를 넘어감
        let ch = p.block(3).channel(17);
        assert_eq!(ch.distance, RawDistance(500));
        assert_eq!(ch.reflectivity, 17);
    }

    #[test]
    fn rejects_bad_packets() {
        assert_eq!(
            PacketView::parse(&[0u8; 10]).unwrap_err(),
            PacketError::Length(10)
        );

        let mut bytes = test_packet::build(0, 0, 20, 500);
        bytes[5 * BLOCK_LEN] = 0x00;
        assert!(matches!(
            PacketView::parse(&bytes),
            Err(PacketError::BlockFlag { block: 5, .. })
        ));

        let mut bytes = test_packet::build(0, 0, 20, 500);
        bytes[2..4].copy_from_slice(&36_000u16.to_le_bytes());
        assert!(matches!(
            PacketView::parse(&bytes),
            Err(PacketError::Azimuth { block: 0, .. })
        ));

        let mut bytes = test_packet::build(0, 0, 20, 500);
        bytes[RETURN_MODE_AT] = 0x99;
        assert_eq!(
            PacketView::parse(&bytes).unwrap_err(),
            PacketError::ReturnMode(0x99)
        );
    }
}
