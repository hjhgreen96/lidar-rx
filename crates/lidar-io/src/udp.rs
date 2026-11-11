//! 실시간 UDP 수신 (리눅스 전용).
//!
//! ```text
//!  UdpSource::recv
//!    poll(소켓, 타임아웃)            아무것도 안 오면 → Received::Timeout (센서가 조용함)
//!    recvmsg(…, MSG_DONTWAIT)        기다리지 않고 읽음. 없으면(체크섬 오류로 버려짐 등) → Received::Timeout
//!      페이로드                   → Packet::payload
//!      제어 메시지 SCM_TIMESTAMPNS  → Packet::received_at   커널이 패킷을 받은 시각 (CLOCK_REALTIME)
//!      제어 메시지 SO_RXQ_OVFL      → kernel_drops()        수신 버퍼가 넘쳐 커널이 버린 누적 수
//! ```
//!
//! ## 커널 수신 시각을 쓰는 이유
//!
//! `recv`가 돌아온 순간에 시계를 읽으면, 그 사이의 스케줄링 지연(스레드가 깨어나기까지)이 섞인다.
//! `SO_TIMESTAMPNS`는 커널이 패킷을 네트워크 스택에 넘길 때 찍은 시각이라 그 지연이 없다.
//!
//! ## `SO_RXQ_OVFL`의 성질
//!
//! 값은 **그 패킷이 큐에 들어갈 때**의 누적 드롭 수이고, 0이면 아예 붙지 않는다. 그래서 버퍼가 넘친 직후
//! 큐에 남아 있던 패킷들(넘치기 전에 들어감)에는 붙지 않고, 그 뒤에 들어온 패킷부터 붙는다. 이 소스는
//! 마지막으로 본 값을 기억한다.
//!
//! `libc`를 직접 부르는 unsafe 코드는 모두 아래 `sys` 모듈 안에 있다.

use lidar_core::{Packet, PacketSource, Received};
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::os::fd::AsRawFd;
use std::time::Duration;

/// UDP 데이터그램 하나를 담을 버퍼 크기. VLP-16은 1206 B.
const MAX_DATAGRAM: usize = 2048;

#[derive(Clone, Debug, PartialEq)]
pub struct UdpOptions {
    /// 요청할 수신 버퍼 크기 (`SO_RCVBUF`). 커널은 요청값을 먼저 `net.core.rmem_max`로 자른 뒤, 회계용 여유를
    /// 위해 두 배로 잡는다 → 실제 상한은 2 × rmem_max. `None`이면 시스템 기본값(`net.core.rmem_default`).
    pub recv_buffer_bytes: Option<usize>,
    /// 이만큼 아무것도 안 오면 `Received::Timeout`
    pub timeout: Duration,
}

impl Default for UdpOptions {
    fn default() -> Self {
        UdpOptions {
            recv_buffer_bytes: None,
            timeout: Duration::from_millis(500),
        }
    }
}

pub struct UdpSource {
    socket: UdpSocket,
    timeout_ms: i32,
    buf: Vec<u8>,
    /// 돌려받은 페이로드 배열. 다음 패킷을 여기에 담는다 (패킷마다 새로 할당하지 않게)
    spare: Vec<u8>,
    socket_drops: u32,
    truncated: u64,
}

impl UdpSource {
    pub fn bind(addr: SocketAddr, options: &UdpOptions) -> io::Result<Self> {
        let socket = UdpSocket::bind(addr)?;
        let fd = socket.as_raw_fd();
        if let Some(bytes) = options.recv_buffer_bytes {
            let bytes = i32::try_from(bytes).map_err(|_| {
                io::Error::new(io::ErrorKind::InvalidInput, "receive buffer too large")
            })?;
            sys::set_int(fd, libc::SOL_SOCKET, libc::SO_RCVBUF, bytes)?;
        }
        sys::set_int(fd, libc::SOL_SOCKET, libc::SO_TIMESTAMPNS, 1)?;
        sys::set_int(fd, libc::SOL_SOCKET, libc::SO_RXQ_OVFL, 1)?;
        Ok(UdpSource {
            socket,
            timeout_ms: options.timeout.as_millis().min(i32::MAX as u128) as i32,
            buf: vec![0; MAX_DATAGRAM],
            spare: Vec::new(),
            socket_drops: 0,
            truncated: 0,
        })
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// 커널이 실제로 잡은 수신 버퍼 크기 (요청값의 두 배, 상한에서 잘림). 시작할 때 로그로 남긴다.
    pub fn recv_buffer_bytes(&self) -> io::Result<usize> {
        sys::get_int(self.socket.as_raw_fd(), libc::SOL_SOCKET, libc::SO_RCVBUF).map(|v| v as usize)
    }

    /// 버퍼보다 커서 잘린 데이터그램 수
    pub fn truncated_packets(&self) -> u64 {
        self.truncated
    }
}

impl PacketSource for UdpSource {
    fn recv(&mut self) -> io::Result<Received> {
        let fd = self.socket.as_raw_fd();
        if !sys::wait_readable(fd, self.timeout_ms)? {
            return Ok(Received::Timeout);
        }
        let Some(meta) = sys::recv_with_meta(fd, &mut self.buf)? else {
            return Ok(Received::Timeout);
        };
        if let Some(drops) = meta.drops {
            self.socket_drops = drops;
        }
        if meta.truncated {
            self.truncated += 1;
        }
        let mut payload = std::mem::take(&mut self.spare);
        payload.clear();
        payload.extend_from_slice(&self.buf[..meta.len]);
        Ok(Received::Packet(Packet {
            payload,
            received_at: meta.timestamp,
        }))
    }

    fn kernel_drops(&self) -> Option<u64> {
        Some(self.socket_drops as u64)
    }

    fn recycle(&mut self, packet: Packet) {
        self.spare = packet.payload;
    }
}

/// 운영체제 호출. unsafe는 여기에만 있다.
mod sys {
    use lidar_core::HostTime;
    use std::io;
    use std::mem;
    use std::os::fd::RawFd;
    use std::ptr::read_unaligned;

    pub fn set_int(
        fd: RawFd,
        level: libc::c_int,
        name: libc::c_int,
        value: libc::c_int,
    ) -> io::Result<()> {
        // SAFETY: value는 호출 동안 살아 있고, 정확한 크기를 넘긴다.
        let rc = unsafe {
            libc::setsockopt(
                fd,
                level,
                name,
                (&value as *const libc::c_int).cast(),
                mem::size_of::<libc::c_int>() as libc::socklen_t,
            )
        };
        if rc == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    pub fn get_int(fd: RawFd, level: libc::c_int, name: libc::c_int) -> io::Result<libc::c_int> {
        let mut value: libc::c_int = 0;
        let mut len = mem::size_of::<libc::c_int>() as libc::socklen_t;
        // SAFETY: value·len은 호출 동안 살아 있고, len이 value의 크기를 알린다.
        let rc = unsafe {
            libc::getsockopt(
                fd,
                level,
                name,
                (&mut value as *mut libc::c_int).cast(),
                &mut len,
            )
        };
        if rc == 0 {
            Ok(value)
        } else {
            Err(io::Error::last_os_error())
        }
    }

    /// 읽을 것이 생길 때까지 최대 `timeout_ms` 기다린다. 신호 때문에 일찍 깨면(EINTR) "없음"으로 본다.
    pub fn wait_readable(fd: RawFd, timeout_ms: i32) -> io::Result<bool> {
        let mut pfd = libc::pollfd {
            fd,
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: pfd 하나를 가리키고 개수 1을 넘긴다.
        let rc = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };
        if rc < 0 {
            let err = io::Error::last_os_error();
            return if err.kind() == io::ErrorKind::Interrupted {
                Ok(false)
            } else {
                Err(err)
            };
        }
        Ok(rc > 0)
    }

    pub struct Meta {
        pub len: usize,
        pub truncated: bool,
        pub timestamp: Option<HostTime>,
        pub drops: Option<u32>,
    }

    /// 기다리지 않고 하나 읽는다. 읽을 것이 없으면 `None`.
    /// poll이 "읽을 것 있음"이라 해도 커널이 읽는 순간 체크섬 오류로 그 패킷을 버리면 큐가 빌 수 있다 (select(2) BUGS).
    /// 그때 막혀서 기다리면 센서가 조용할 때 루프가 서고 watchdog이 서비스를 죽인다.
    pub fn recv_with_meta(fd: RawFd, buf: &mut [u8]) -> io::Result<Option<Meta>> {
        // 제어 메시지 공간. cmsghdr가 size_t 정렬을 요구하므로 u64 배열로 잡는다.
        let mut control = [0u64; 16];
        let mut iov = libc::iovec {
            iov_base: buf.as_mut_ptr().cast(),
            iov_len: buf.len(),
        };
        // SAFETY: msghdr는 모든 필드가 0이어도 유효한 C 구조체다.
        let mut msg: libc::msghdr = unsafe { mem::zeroed() };
        msg.msg_iov = &mut iov;
        msg.msg_iovlen = 1;
        msg.msg_control = control.as_mut_ptr().cast();
        msg.msg_controllen = mem::size_of_val(&control) as _;

        // SAFETY: msg가 가리키는 buf·control은 호출 동안 살아 있고, 길이를 정확히 알린다.
        let n = unsafe { libc::recvmsg(fd, &mut msg, libc::MSG_DONTWAIT) };
        if n < 0 {
            let err = io::Error::last_os_error();
            return match err.kind() {
                io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted => Ok(None),
                _ => Err(err),
            };
        }

        let mut meta = Meta {
            len: n as usize,
            truncated: msg.msg_flags & libc::MSG_TRUNC != 0,
            timestamp: None,
            drops: None,
        };
        // SAFETY: CMSG_* 매크로는 커널이 채운 control 안(msg_controllen까지)에서만 움직이고,
        // 데이터는 길이를 확인한 뒤 정렬과 무관하게(read_unaligned) 읽는다.
        unsafe {
            let mut c = libc::CMSG_FIRSTHDR(&msg);
            while !c.is_null() {
                let header = &*c;
                let data = libc::CMSG_DATA(c);
                let Some(data_len) =
                    (header.cmsg_len as usize).checked_sub(libc::CMSG_LEN(0) as usize)
                else {
                    break; // 길이가 헤더보다 짧은 제어 메시지: 더 읽지 않음
                };
                if header.cmsg_level == libc::SOL_SOCKET {
                    if header.cmsg_type == libc::SCM_TIMESTAMPNS {
                        meta.timestamp = parse_timespec(data, data_len);
                    } else if header.cmsg_type == libc::SO_RXQ_OVFL && data_len >= 4 {
                        meta.drops = Some(read_unaligned(data.cast::<u32>()));
                    }
                }
                c = libc::CMSG_NXTHDR(&msg, c);
            }
        }
        Ok(Some(meta))
    }

    /// 커널이 넣어 준 `struct timespec`. 64비트 시스템은 (i64 초, i64 나노초) 16 B이고,
    /// 32비트 ARM(보드)에서는 옛 형식 (i32, i32) 8 B일 수 있어 길이로 구분한다.
    unsafe fn parse_timespec(data: *const u8, len: usize) -> Option<HostTime> {
        let (secs, nanos) = match len {
            16 => (
                read_unaligned(data.cast::<i64>()),
                read_unaligned(data.add(8).cast::<i64>()),
            ),
            8 => (
                read_unaligned(data.cast::<i32>()) as i64,
                read_unaligned(data.add(4).cast::<i32>()) as i64,
            ),
            _ => return None,
        };
        Some(HostTime::from_secs_nanos(secs, nanos as u32))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn loopback(options: UdpOptions) -> (UdpSource, UdpSocket) {
        let source = UdpSource::bind("127.0.0.1:0".parse().unwrap(), &options).unwrap();
        let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
        sender.connect(source.local_addr().unwrap()).unwrap();
        (source, sender)
    }

    fn now_ns() -> i64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as i64
    }

    #[test]
    fn receives_payload_with_kernel_timestamp() {
        let (mut source, sender) = loopback(UdpOptions::default());
        let before = now_ns();
        sender.send(b"hello").unwrap();
        let Received::Packet(p) = source.recv().unwrap() else {
            panic!("no packet")
        };
        let after = now_ns();
        assert_eq!(p.payload, b"hello");
        let t = p.received_at.expect("kernel timestamp").0;
        assert!(before <= t && t <= after, "{before} <= {t} <= {after}");
    }

    #[test]
    fn receives_the_next_packet_into_a_payload_given_back() {
        let (mut source, sender) = loopback(UdpOptions::default());
        sender.send(b"first packet").unwrap();
        sender.send(b"second").unwrap();
        let Received::Packet(first) = source.recv().unwrap() else {
            panic!("no packet")
        };
        let at = first.payload.as_ptr();
        source.recycle(first);
        let Received::Packet(second) = source.recv().unwrap() else {
            panic!("no packet")
        };
        assert_eq!(second.payload, b"second");
        assert_eq!(second.payload.as_ptr(), at);
    }

    #[test]
    fn times_out_when_quiet() {
        let options = UdpOptions {
            timeout: Duration::from_millis(30),
            ..UdpOptions::default()
        };
        let (mut source, _sender) = loopback(options);
        assert_eq!(source.recv().unwrap(), Received::Timeout);
    }

    #[test]
    fn reports_socket_drops_on_the_first_packet_queued_after_overflow() {
        let options = UdpOptions {
            recv_buffer_bytes: Some(4096),
            ..UdpOptions::default()
        };
        let (mut source, sender) = loopback(options);
        let packet = [0u8; 1206];
        for _ in 0..64 {
            sender.send(&packet).unwrap(); // 읽지 않고 보냄 → 버퍼가 넘침
        }

        let mut received = 0;
        let quick = UdpOptions {
            timeout: Duration::from_millis(20),
            ..UdpOptions::default()
        };
        source.timeout_ms = quick.timeout.as_millis() as i32;
        while let Received::Packet(_) = source.recv().unwrap() {
            received += 1;
        }
        // 큐에 남아 있던 패킷들은 넘치기 전에 들어갔으므로 드롭 수가 붙지 않았다
        assert_eq!(source.kernel_drops(), Some(0));

        sender.send(&packet).unwrap(); // 넘친 뒤 처음 들어가는 패킷
        assert!(matches!(source.recv().unwrap(), Received::Packet(_)));
        assert_eq!(source.kernel_drops(), Some(64 - received));
        assert!(received < 64);
    }
}
