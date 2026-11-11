//! pcap 파일을 `PacketSource`로 맞추는 어댑터.
//!
//! 파일을 가능한 한 빨리 읽어 넘긴다. 원래 간격대로 보내는 것(재생 속도)은 `pcap-replay`의 일이다.

use crate::net::parse_udp;
use crate::pcap::{PcapReader, LINKTYPE_ETHERNET};
use lidar_core::{Packet, PacketSource, Received};
use std::fs::File;
use std::io::{self, BufReader, Read};
use std::path::Path;

pub struct PcapSource<R> {
    reader: PcapReader<R>,
    dst_port: u16,
}

impl PcapSource<BufReader<File>> {
    /// 파일을 열어, 목적지 포트가 `dst_port`인 UDP 패킷만 내보낸다 (VLP-16 데이터는 2368).
    pub fn open(path: impl AsRef<Path>, dst_port: u16) -> io::Result<Self> {
        Self::new(BufReader::new(File::open(path)?), dst_port)
    }
}

impl<R: Read> PcapSource<R> {
    pub fn new(reader: R, dst_port: u16) -> io::Result<Self> {
        let reader = PcapReader::new(reader)?;
        if reader.linktype() != LINKTYPE_ETHERNET {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("linktype {} is not Ethernet", reader.linktype()),
            ));
        }
        Ok(PcapSource { reader, dst_port })
    }
}

impl<R: Read> PacketSource for PcapSource<R> {
    fn recv(&mut self) -> io::Result<Received> {
        while let Some(record) = self.reader.next_record()? {
            if let Some(udp) = parse_udp(&record.data).filter(|d| d.dst_port == self.dst_port) {
                return Ok(Received::Packet(Packet {
                    payload: udp.payload.to_vec(),
                    received_at: Some(record.captured_at),
                }));
            }
        }
        Ok(Received::End)
    }
}
