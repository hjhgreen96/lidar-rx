//! SIGTERM·SIGINT를 받으면 수신 루프를 끝내고, 소비자를 정리하고, 요약을 남긴 뒤 정상 종료한다 (4단계).
//! systemd는 서비스를 멈출 때 SIGTERM을 보내고, `TimeoutStopSec` 안에 안 끝나면 SIGKILL로 죽인다.
//!
//! ```text
//!  신호 ──► 처리기: 플래그만 세움 (처리기 안에서는 원자적 쓰기 말고는 안전하게 할 수 있는 일이 거의 없다)
//!  수신 루프 ── 한 바퀴마다 플래그 확인 ──► 끝냄
//!     패킷이 없어도 출처의 타임아웃(0.5 s)마다 한 바퀴 돌므로 늦어도 그 안에 알아챈다.
//!     SA_RESTART를 주지 않아, 신호가 수신 스레드에 오면 기다리던 poll이 EINTR로 바로 깬다.
//! ```

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};

static STOP: AtomicBool = AtomicBool::new(false);

pub fn stop_requested() -> bool {
    STOP.load(Ordering::Relaxed)
}

#[cfg(target_os = "linux")]
pub fn install() -> io::Result<()> {
    extern "C" fn on_signal(_: libc::c_int) {
        STOP.store(true, Ordering::Relaxed);
    }
    for signal in [libc::SIGTERM, libc::SIGINT] {
        // SAFETY: 0으로 채운 sigaction은 "플래그 없음, 마스크 비움"으로 유효하다. 처리기는 원자적 쓰기만 한다.
        let rc = unsafe {
            let mut action: libc::sigaction = std::mem::zeroed();
            action.sa_sigaction = on_signal as extern "C" fn(libc::c_int) as libc::sighandler_t;
            libc::sigemptyset(&mut action.sa_mask);
            libc::sigaction(signal, &action, std::ptr::null_mut())
        };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
    }
    Ok(())
}

#[cfg(not(target_os = "linux"))]
pub fn install() -> io::Result<()> {
    Ok(())
}

#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;

    #[test]
    fn sigterm_sets_the_stop_flag_instead_of_killing() {
        install().unwrap();
        assert!(!stop_requested());
        // SAFETY: 이 프로세스에 SIGTERM을 보냄. 위에서 처리기를 걸었으므로 죽지 않고 플래그만 선다.
        unsafe { libc::raise(libc::SIGTERM) };
        assert!(stop_requested());
    }
}
