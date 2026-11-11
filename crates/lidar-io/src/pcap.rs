//! pcap 파일 형식 (libpcap의 기본 형식) 읽기.
//!
//! ```text
//!  [전역 헤더 24 B][레코드 헤더 16 B][캡처한 바이트 …][레코드 헤더 16 B][캡처한 바이트 …] …
//!
//!  전역 헤더                              레코드 헤더
//!  0  magic       4                       0   ts_sec    4   캡처 시각 (초)
//!  4  version     2+2                     4   ts_frac   4   초 아래 (µs 또는 ns, magic이 정함)
//!  8  thiszone    4   (안 씀)             8   incl_len  4   파일에 저장된 길이
//!  12 sigfigs     4   (안 씀)             12  orig_len  4   원래 패킷 길이 (snaplen에 잘렸으면 더 큼)
//!  16 snaplen     4   레코드 최대 길이
//!  20 linktype    4   1 = Ethernet
//!
//!  magic: A1 B2 C3 D4 → 시각이 µs, A1 B2 3C 4D → ns.
//!         파일에 D4 C3 B2 A1 순서로 보이면 little-endian 파일, 그대로 보이면 big-endian 파일.
//! ```

use lidar_core::HostTime;
use std::io::{self, Read};

/// linktype 1 = Ethernet. 이 프로젝트가 다루는 캡처는 모두 이 종류.
pub const LINKTYPE_ETHERNET: u32 = 1;

const MAGIC_MICROS: u32 = 0xA1B2_C3D4;
const MAGIC_NANOS: u32 = 0xA1B2_3C4D;
/// 레코드 길이 상한 (libpcap의 최대 snaplen). 이보다 크면 깨진 파일로 본다 - 4 GB를 할당하려다 죽지 않도록.
const MAX_RECORD_LEN: usize = 262_144;

/// 레코드 하나 = 캡처한 패킷 하나.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Record {
    /// 캡처한 컴퓨터의 커널이 찍은 수신 시각
    pub captured_at: HostTime,
    /// 링크 계층부터의 바이트 (Ethernet이면 Ethernet 헤더부터)
    pub data: Vec<u8>,
    /// 원래 길이. `data.len()`보다 크면 캡처할 때 잘렸다는 뜻.
    pub orig_len: u32,
}

pub struct PcapReader<R> {
    reader: R,
    big_endian: bool,
    nanos: bool,
    linktype: u32,
}

impl<R: Read> PcapReader<R> {
    /// 전역 헤더를 읽고 바이트 순서와 시각 단위를 정한다.
    pub fn new(mut reader: R) -> io::Result<Self> {
        let mut header = [0u8; 24];
        reader.read_exact(&mut header)?;

        let magic_le = u32::from_le_bytes(header[0..4].try_into().unwrap());
        let magic_be = u32::from_be_bytes(header[0..4].try_into().unwrap());
        let (big_endian, nanos) = match (magic_le, magic_be) {
            (MAGIC_MICROS, _) => (false, false),
            (MAGIC_NANOS, _) => (false, true),
            (_, MAGIC_MICROS) => (true, false),
            (_, MAGIC_NANOS) => (true, true),
            _ => return Err(invalid(format!("not a pcap file (magic {magic_le:08x})"))),
        };

        let mut r = PcapReader {
            reader,
            big_endian,
            nanos,
            linktype: 0,
        };
        r.linktype = r.u32_at(&header, 20);
        Ok(r)
    }

    pub fn linktype(&self) -> u32 {
        self.linktype
    }

    /// 다음 레코드. 파일 끝이면 `None`. 레코드 중간에서 끝나면(잘린 파일) 오류.
    pub fn next_record(&mut self) -> io::Result<Option<Record>> {
        let mut header = [0u8; 16];
        if !read_exact_or_eof(&mut self.reader, &mut header)? {
            return Ok(None);
        }
        let secs = self.u32_at(&header, 0) as i64;
        let frac = self.u32_at(&header, 4);
        let incl_len = self.u32_at(&header, 8) as usize;
        let orig_len = self.u32_at(&header, 12);

        // 깨진 파일 방어: 길이와 초 아래 값의 범위를 확인한 뒤에 할당·계산한다
        if incl_len > MAX_RECORD_LEN {
            return Err(invalid(format!(
                "record length {incl_len} B is not plausible"
            )));
        }
        let nanos = match self.nanos {
            true if frac < 1_000_000_000 => frac,
            false if frac < 1_000_000 => frac * 1000,
            _ => return Err(invalid(format!("sub-second field {frac} out of range"))),
        };
        let mut data = vec![0u8; incl_len];
        self.reader.read_exact(&mut data)?;
        Ok(Some(Record {
            captured_at: HostTime::from_secs_nanos(secs, nanos),
            data,
            orig_len,
        }))
    }

    fn u32_at(&self, buf: &[u8], at: usize) -> u32 {
        let bytes: [u8; 4] = buf[at..at + 4].try_into().unwrap();
        if self.big_endian {
            u32::from_be_bytes(bytes)
        } else {
            u32::from_le_bytes(bytes)
        }
    }
}

/// `buf`를 채우면 `true`, 한 바이트도 못 읽고 끝나면 `false`, 중간에 끝나면 오류.
fn read_exact_or_eof(reader: &mut impl Read, buf: &mut [u8]) -> io::Result<bool> {
    let mut filled = 0;
    while filled < buf.len() {
        match reader.read(&mut buf[filled..]) {
            Ok(0) if filled == 0 => return Ok(false),
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "truncated pcap record",
                ))
            }
            Ok(n) => filled += n,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
    Ok(true)
}

fn invalid(msg: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn file(magic: u32, big_endian: bool, records: &[(u32, u32, &[u8])]) -> Vec<u8> {
        let w = |v: u32| {
            if big_endian {
                v.to_be_bytes()
            } else {
                v.to_le_bytes()
            }
        };
        let mut f = Vec::new();
        f.extend(w(magic));
        f.extend(if big_endian {
            [0, 2, 0, 4]
        } else {
            [2, 0, 4, 0]
        }); // version 2.4
        f.extend([0u8; 8]);
        f.extend(w(65535));
        f.extend(w(LINKTYPE_ETHERNET));
        for &(sec, frac, data) in records {
            f.extend(w(sec));
            f.extend(w(frac));
            f.extend(w(data.len() as u32));
            f.extend(w(data.len() as u32));
            f.extend(data);
        }
        f
    }

    #[test]
    fn reads_little_endian_micros() {
        let bytes = file(
            MAGIC_MICROS,
            false,
            &[(10, 250_000, b"abc"), (11, 0, b"de")],
        );
        let mut r = PcapReader::new(&bytes[..]).unwrap();
        assert_eq!(r.linktype(), LINKTYPE_ETHERNET);
        let a = r.next_record().unwrap().unwrap();
        assert_eq!(a.captured_at, HostTime(10_250_000_000));
        assert_eq!(a.data, b"abc");
        assert_eq!(r.next_record().unwrap().unwrap().data, b"de");
        assert!(r.next_record().unwrap().is_none());
    }

    #[test]
    fn reads_big_endian_nanos() {
        let bytes = file(MAGIC_NANOS, true, &[(10, 7, b"x")]);
        let mut r = PcapReader::new(&bytes[..]).unwrap();
        assert_eq!(
            r.next_record().unwrap().unwrap().captured_at,
            HostTime(10_000_000_007)
        );
    }

    #[test]
    fn truncated_record_is_an_error() {
        let mut bytes = file(MAGIC_MICROS, false, &[(10, 0, b"abcdef")]);
        bytes.truncate(bytes.len() - 2);
        let mut r = PcapReader::new(&bytes[..]).unwrap();
        assert!(r.next_record().is_err());
    }

    #[test]
    fn implausible_fields_are_errors_not_panics() {
        let bytes = file(MAGIC_MICROS, false, &[(10, 2_000_000, b"x")]); // µs 칸에 2초
        assert!(PcapReader::new(&bytes[..]).unwrap().next_record().is_err());

        let mut bytes = file(MAGIC_MICROS, false, &[(10, 0, b"x")]);
        bytes[24 + 8..24 + 12].copy_from_slice(&u32::MAX.to_le_bytes()); // 길이 4 GB
        assert!(PcapReader::new(&bytes[..]).unwrap().next_record().is_err());
    }
}
