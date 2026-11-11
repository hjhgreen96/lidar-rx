//! 수신 스레드의 실행 조건 (실험 4): CPU 고정과 실시간 정책.
//!
//! ```text
//!  pthread_setschedparam(self, SCHED_FIFO, p)    일반 정책(SCHED_OTHER)의 작업보다 항상 먼저 돈다.
//!                                                같은 CPU에서 일반 작업이 CPU를 쓰고 있어도, 패킷이 와서
//!                                                깨어나면 그 자리에서 CPU를 빼앗아 돈다
//!  sched_setaffinity(0, {cpu})                   이 스레드를 그 CPU에서만 돌게 함 (정책을 바꾼 **뒤에**)
//! ```
//!
//! `--lock-memory`: `mlockall(MCL_CURRENT | MCL_FUTURE)` - 이것만은 프로세스 전체. 실행 코드·스택·힙이 RAM에서 밀려나
//! 수신 스레드가 SD 카드 읽기(큰 페이지 폴트)를 기다리는 일을 막는다 (`sys::lock_memory`의 설명).
//!
//! 정책·CPU는 프로세스 전체가 아니라 **수신 스레드 하나에만** 적용한다. 리눅스에서 pid 0은 "부른 스레드"이고, 새 스레드는
//! 만들어질 때 만든 스레드의 정책을 물려받으므로, 소비자 스레드를 다 만든 뒤에 부른다 (`pipeline::run`).
//! SCHED_FIFO는 root, 또는 `CAP_SYS_NICE`나 `RLIMIT_RTPRIO`(systemd의 `LimitRTPRIO=`)가 있어야 한다.

use std::io;

/// 수신 스레드에 줄 실행 조건. 모두 비어 있으면 아무것도 바꾸지 않는다.
#[derive(Clone, Copy, Debug, Default)]
pub struct ThreadSetup {
    pub cpu: Option<usize>,
    /// SCHED_FIFO 우선순위 (1 ~ 99)
    pub rt_priority: Option<i32>,
    /// 프로세스 메모리를 RAM에 고정 (`mlockall`, `--lock-memory`)
    pub lock_memory: bool,
}

/// 지금 스레드에 적용하고, 커널에서 다시 읽은 실제 상태를 한 줄로 돌려준다 (적용할 것이 없으면 `None`).
pub fn apply(setup: &ThreadSetup) -> io::Result<Option<String>> {
    if setup.cpu.is_none() && setup.rt_priority.is_none() && !setup.lock_memory {
        return Ok(None);
    }
    let locked = if setup.lock_memory {
        sys::lock_memory().map_err(|e| {
            context(
                e,
                "--lock-memory (needs RLIMIT_MEMLOCK above the process size: systemd LimitMEMLOCK=)"
                    .into(),
            )
        })?;
        format!(", memory locked {} kB", sys::locked_kb()?)
    } else {
        String::new()
    };
    // 순서가 중요하다: 실시간 정책을 먼저 주고 그다음 CPU를 옮긴다.
    // 거꾸로 하면 옮기는 순간에는 아직 일반 정책이라, 그 CPU를 다른 실시간 작업이 쓰고 있으면 그 작업이 쉴 때까지
    // 옮겨 가지 못하고 기다린다 (__set_cpus_allowed_ptr가 완료를 기다림). 재검에서 591 ms 멈춘 것을 추적으로 확인.
    if let Some(priority) = setup.rt_priority {
        sys::set_fifo(priority).map_err(|e| {
            context(
                e,
                format!("--rt-prio {priority} (needs root, CAP_SYS_NICE or RLIMIT_RTPRIO)"),
            )
        })?;
    }
    if let Some(cpu) = setup.cpu {
        sys::pin_to_cpu(cpu).map_err(|e| context(e, format!("--cpu {cpu}")))?;
    }
    let (policy, priority) = sys::policy()?;
    let policy = match policy {
        "SCHED_FIFO" | "SCHED_RR" => format!("{policy} priority {priority}"),
        _ => policy.to_string(),
    };
    Ok(Some(format!(
        "{policy}, CPUs {}{locked}",
        ranges(&sys::allowed_cpus()?)
    )))
}

/// 오류에 어느 설정이 실패했는지를 붙임 ("Operation not permitted"만으로는 무엇을 할지 모름)
fn context(e: io::Error, what: String) -> io::Error {
    io::Error::new(e.kind(), format!("{what}: {e}"))
}

/// [0, 1, 2, 5] → "0-2,5"
fn ranges(cpus: &[usize]) -> String {
    let mut parts = Vec::new();
    let mut i = 0;
    while i < cpus.len() {
        let start = cpus[i];
        while i + 1 < cpus.len() && cpus[i + 1] == cpus[i] + 1 {
            i += 1;
        }
        parts.push(if cpus[i] == start {
            start.to_string()
        } else {
            format!("{start}-{}", cpus[i])
        });
        i += 1;
    }
    parts.join(",")
}

#[cfg(target_os = "linux")]
mod sys {
    use std::io;
    use std::mem::{size_of, zeroed};

    pub fn pin_to_cpu(cpu: usize) -> io::Result<()> {
        if cpu >= libc::CPU_SETSIZE as usize {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("CPU {cpu} is out of range"),
            ));
        }
        // SAFETY: cpu_set_t는 비트 배열이라 0으로 채운 값이 유효한 빈 집합이다. CPU_SET의 범위는 위에서 확인했고,
        // sched_setaffinity는 넘겨준 크기만큼 집합을 읽기만 한다.
        let rc = unsafe {
            let mut set: libc::cpu_set_t = zeroed();
            libc::CPU_SET(cpu, &mut set);
            libc::sched_setaffinity(0, size_of::<libc::cpu_set_t>(), &set)
        };
        if rc != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    pub fn set_fifo(priority: i32) -> io::Result<()> {
        // sched_param은 C 라이브러리마다 필드가 다르다 (musl에는 필드가 더 있음) → 0으로 채우고 우선순위만 씀
        // SAFETY: 정수 필드만 있는 구조체라 0으로 채운 값이 유효하다.
        let mut param: libc::sched_param = unsafe { zeroed() };
        param.sched_priority = priority;
        // SAFETY: pthread_self()는 늘 지금 스레드이고, param은 읽기만 한다.
        let rc =
            unsafe { libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param) };
        if rc != 0 {
            // pthread 함수는 errno가 아니라 오류 번호를 돌려준다 (권한이 없으면 EPERM)
            return Err(io::Error::from_raw_os_error(rc));
        }
        Ok(())
    }

    pub fn policy() -> io::Result<(&'static str, i32)> {
        let mut policy = 0;
        // SAFETY: 위와 같음. 결과를 받을 두 변수를 넘긴다.
        let mut param: libc::sched_param = unsafe { zeroed() };
        let rc =
            unsafe { libc::pthread_getschedparam(libc::pthread_self(), &mut policy, &mut param) };
        if rc != 0 {
            return Err(io::Error::from_raw_os_error(rc));
        }
        let name = match policy {
            libc::SCHED_OTHER => "SCHED_OTHER",
            libc::SCHED_FIFO => "SCHED_FIFO",
            libc::SCHED_RR => "SCHED_RR",
            _ => "other policy",
        };
        Ok((name, param.sched_priority))
    }

    /// 지금 있는 메모리(실행 코드 전체, 스레드 스택, 힙)를 채워 넣고 고정하고(MCL_CURRENT), 앞으로 받는 메모리도
    /// 받는 순간 채워 넣고 고정한다(MCL_FUTURE).
    ///
    /// MCL_FUTURE의 채우기는 받는 스레드에서 일어나므로, 수신 스레드가 프레임마다 큰 배열을 새로 받던 때는(보드의 musl은
    /// 128 KB 넘는 할당을 mmap으로 받음) 보드에서 대기 p99가 0.3 → 2.0 ms였다. 다 쓴 배열을 돌려받아 다시 쓰게 한 뒤로는
    /// 수신 스레드가 시작 직후 말고는 메모리를 받지 않아 차이가 없다 (실측, docs/runs/5-board.md 11절)
    pub fn lock_memory() -> io::Result<()> {
        // SAFETY: 플래그만 넘기는 시스템 호출
        if unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    /// /proc/self/status의 VmLck (고정된 메모리, kB)
    pub fn locked_kb() -> io::Result<u64> {
        let status = std::fs::read_to_string("/proc/self/status")?;
        Ok(status
            .lines()
            .find_map(|l| l.strip_prefix("VmLck:"))
            .and_then(|v| v.trim().trim_end_matches("kB").trim().parse().ok())
            .unwrap_or(0))
    }

    pub fn allowed_cpus() -> io::Result<Vec<usize>> {
        // SAFETY: 0으로 채운 집합을 sched_getaffinity가 채운다. CPU_ISSET은 집합 크기 안에서만 부른다.
        unsafe {
            let mut set: libc::cpu_set_t = zeroed();
            if libc::sched_getaffinity(0, size_of::<libc::cpu_set_t>(), &mut set) != 0 {
                return Err(io::Error::last_os_error());
            }
            Ok((0..libc::CPU_SETSIZE as usize)
                .filter(|&cpu| libc::CPU_ISSET(cpu, &set))
                .collect())
        }
    }
}

#[cfg(not(target_os = "linux"))]
mod sys {
    use std::io;

    fn unsupported() -> io::Error {
        io::Error::new(
            io::ErrorKind::Unsupported,
            "thread setup is implemented for Linux only",
        )
    }
    pub fn pin_to_cpu(_: usize) -> io::Result<()> {
        Err(unsupported())
    }
    pub fn set_fifo(_: i32) -> io::Result<()> {
        Err(unsupported())
    }
    pub fn policy() -> io::Result<(&'static str, i32)> {
        Err(unsupported())
    }
    pub fn lock_memory() -> io::Result<()> {
        Err(unsupported())
    }
    pub fn locked_kb() -> io::Result<u64> {
        Err(unsupported())
    }
    pub fn allowed_cpus() -> io::Result<Vec<usize>> {
        Err(unsupported())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_lists_are_shown_as_ranges() {
        assert_eq!(ranges(&[2]), "2");
        assert_eq!(ranges(&[0, 1, 2, 5, 7, 8]), "0-2,5,7-8");
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn pinning_applies_to_this_thread_only_and_reads_back() {
        let before = sys::allowed_cpus().unwrap();
        let line = std::thread::spawn(|| {
            apply(&ThreadSetup {
                cpu: Some(0),
                ..ThreadSetup::default()
            })
        })
        .join()
        .unwrap()
        .unwrap();
        assert_eq!(line.as_deref(), Some("SCHED_OTHER, CPUs 0"));
        // 고정한 것은 새 스레드뿐: 이 스레드(만든 쪽)의 CPU 집합은 그대로
        assert_eq!(sys::allowed_cpus().unwrap(), before);
    }
}
