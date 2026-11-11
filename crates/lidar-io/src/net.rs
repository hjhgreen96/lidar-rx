//! Ethernet 프레임에서 UDP 페이로드 꺼내기.
//!
//! ```text
//!  Ethernet 14 B          IPv4 (IHL×4 B, 보통 20)                      UDP 8 B        페이로드
//!  ┌──────┬──────┬────┐  ┌───┬───┬─────┬──┬─────┬───┬────┬───┬───┬───┐ ┌───┬───┬───┬───┐ ┌──────┐
//!  │dst 6 │src 6 │type│  │ver│IHL│ …   │…│flags│…  │prot│ … │src│dst│ │src│dst│len│sum│ │ …    │
//!  │      │      │ 2  │  │ 4b│ 4b│     │  │+frag│   │ 17 │   │ 4 │ 4 │ │ 2 │ 2 │ 2 │ 2 │ │      │
//!  └──────┴──────┴────┘  └───┴───┴─────┴──┴─────┴───┴────┴───┴───┴───┘ └───┴───┴───┴───┘ └──────┘
//!   type 0x0800 = IPv4    IHL: 헤더 길이(4 B 단위). 옵션이 있으면 20보다 큼
//!   (0x8100 = VLAN 태그    flags+frag: 조각난 패킷이면 UDP 헤더가 첫 조각에만 있음 → 조각은 건너뜀
//!    4 B가 끼어 있음)      prot 17 = UDP
//! ```
//!
//! 네트워크 바이트 순서는 big-endian (높은 자리 바이트가 앞). VLP-16 페이로드 안쪽은 little-endian.

use std::net::Ipv4Addr;

const ETHERTYPE_IPV4: u16 = 0x0800;
const ETHERTYPE_VLAN: u16 = 0x8100;
const IP_PROTO_UDP: u8 = 17;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UdpDatagram<'a> {
    pub src: Ipv4Addr,
    pub dst: Ipv4Addr,
    pub src_port: u16,
    pub dst_port: u16,
    pub payload: &'a [u8],
}

/// Ethernet 프레임 → UDP 데이터그램. IPv4·UDP가 아니거나, 조각났거나, 길이가 안 맞으면 `None`.
pub fn parse_udp(frame: &[u8]) -> Option<UdpDatagram<'_>> {
    let mut at = 12;
    let mut ethertype = be16(frame, at)?;
    if ethertype == ETHERTYPE_VLAN {
        at += 4;
        ethertype = be16(frame, at)?;
    }
    if ethertype != ETHERTYPE_IPV4 {
        return None;
    }
    let ip = frame.get(at + 2..)?;

    let version = ip.first()? >> 4;
    let header_len = ((ip.first()? & 0x0F) as usize) * 4;
    if version != 4 || header_len < 20 || ip.len() < header_len {
        return None;
    }
    let total_len = be16(ip, 2)? as usize;
    let flags_fragment = be16(ip, 6)?;
    let more_fragments = flags_fragment & 0x2000 != 0;
    let fragment_offset = flags_fragment & 0x1FFF;
    if more_fragments || fragment_offset != 0 || ip[9] != IP_PROTO_UDP {
        return None;
    }
    if total_len < header_len {
        return None; // 깨진 패킷: 전체 길이가 헤더보다 짧음
    }
    // Ethernet은 최소 길이를 맞추려고 뒤에 패딩을 붙일 수 있음 → IP가 말하는 길이까지만 봄
    let ip = ip.get(..total_len)?;
    let src = Ipv4Addr::new(ip[12], ip[13], ip[14], ip[15]);
    let dst = Ipv4Addr::new(ip[16], ip[17], ip[18], ip[19]);

    let udp = ip.get(header_len..)?;
    let udp_len = be16(udp, 4)? as usize;
    if udp_len < 8 || udp.len() < udp_len {
        return None;
    }
    Some(UdpDatagram {
        src,
        dst,
        src_port: be16(udp, 0)?,
        dst_port: be16(udp, 2)?,
        payload: &udp[8..udp_len],
    })
}

fn be16(buf: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_be_bytes([*buf.get(at)?, *buf.get(at + 1)?]))
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// 테스트용 Ethernet + IPv4(20 B) + UDP 프레임
    pub fn frame(dst_port: u16, payload: &[u8]) -> Vec<u8> {
        let mut f = vec![0u8; 12];
        f.extend(ETHERTYPE_IPV4.to_be_bytes());
        let total = (20 + 8 + payload.len()) as u16;
        f.extend([0x45, 0]);
        f.extend(total.to_be_bytes());
        f.extend([0, 0, 0x40, 0]); // id, flags = DF
        f.extend([64, IP_PROTO_UDP, 0, 0]);
        f.extend([192, 168, 1, 201, 255, 255, 255, 255]);
        f.extend(2368u16.to_be_bytes());
        f.extend(dst_port.to_be_bytes());
        f.extend(((8 + payload.len()) as u16).to_be_bytes());
        f.extend([0, 0]);
        f.extend(payload);
        f
    }

    #[test]
    fn extracts_udp_payload() {
        let f = frame(2368, b"hello");
        let d = parse_udp(&f).unwrap();
        assert_eq!(d.dst_port, 2368);
        assert_eq!(d.src, Ipv4Addr::new(192, 168, 1, 201));
        assert_eq!(d.payload, b"hello");
    }

    #[test]
    fn ignores_ethernet_padding() {
        let mut f = frame(2368, b"hi");
        f.extend([0u8; 10]);
        assert_eq!(parse_udp(&f).unwrap().payload, b"hi");
    }

    #[test]
    fn corrupt_lengths_are_skipped_not_panicking() {
        let mut f = frame(2368, b"hello");
        f[14 + 2..14 + 4].copy_from_slice(&10u16.to_be_bytes()); // IP 전체 길이 10 < 헤더 20
        assert!(parse_udp(&f).is_none());

        let mut f = frame(2368, b"hello");
        f[14 + 20 + 4..14 + 20 + 6].copy_from_slice(&500u16.to_be_bytes()); // UDP 길이가 실제보다 김
        assert!(parse_udp(&f).is_none());
    }

    #[test]
    fn skips_fragments_and_non_udp() {
        let mut f = frame(2368, b"x");
        f[14 + 6] = 0x20; // MF 플래그
        assert!(parse_udp(&f).is_none());

        let mut f = frame(2368, b"x");
        f[14 + 9] = 6; // TCP
        assert!(parse_udp(&f).is_none());
    }
}
