//! 경계 1 - 입력 출처 (Adapter 패턴).
//!
//! 실시간 UDP 소켓이든 녹화된 pcap 파일이든, 해석기 입장에서는 "패킷이 하나씩 온다"는 사실만 같다.
//! 출처마다 다른 사용법(소켓 옵션, 파일 형식)을 `PacketSource` 하나의 모양으로 맞춰 주는 것이 Adapter다.
//!
//! ```text
//!  UdpSource  ──┐
//!               ├──► PacketSource::recv() ──► Received::{Packet, Timeout, End}
//!  PcapSource ──┘
//! ```

use crate::HostTime;

/// 받은 UDP 페이로드 하나.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet {
    /// UDP 페이로드 (VLP-16이면 1206 B). Ethernet·IP·UDP 헤더는 이미 벗겨진 상태.
    pub payload: Vec<u8>,
    /// 호스트가 이 패킷을 받은 시각. 소켓이면 커널 수신 시각, pcap이면 캡처 시각. 모르면 `None`.
    pub received_at: Option<HostTime>,
}

/// `recv`의 결과. "패킷 없음"을 둘로 구분: `Timeout`(조용함), `End`(입력 끝).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Received {
    Packet(Packet),
    /// 정해진 시간 동안 아무것도 안 옴 (센서가 조용함). 출처는 살아 있다 → 계속 기다릴 수 있음.
    Timeout,
    /// 더 올 것이 없음 (파일 끝). 실시간 소켓에서는 나오지 않는다.
    End,
}

pub trait PacketSource {
    /// 다음 패킷을 기다려 받는다. I/O 오류는 `Err`.
    fn recv(&mut self) -> std::io::Result<Received>;

    /// 이 출처에서 커널이 버린 패킷 수 (누적). 소켓이면 수신 버퍼가 넘쳐 버린 수, 파일이면 `None`.
    fn kernel_drops(&self) -> Option<u64> {
        None
    }

    /// 다 쓴 패킷을 돌려준다. 출처는 그 배열에 다음 패킷을 받을 수 있다 (패킷마다 새로 할당하지 않게).
    /// 기본은 버림.
    fn recycle(&mut self, _packet: Packet) {}
}
