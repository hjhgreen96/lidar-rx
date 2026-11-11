#!/bin/bash
# 보드(Arty Z7 + PYNQ 이미지)의 환경을 한 파일로 모은다. 보드에 올리기 전에 정할 것들을 PC에서 판단하려고:
# 커널 모듈 크로스 빌드에 맞출 커널 버전·설정·컴파일러, 실시간 설정, NIC·인터럽트, 추적 도구, 부팅 파일.
#
#   sudo bash board_info.sh > board_info.txt 2>&1      (root: dmesg, ethtool -S, tracefs, 부트 파티션을 읽으려고)
#
# 읽기만 한다 (예외: /proc/config.gz가 없으면 configs 모듈을 올려 봄, 부트 파티션이 안 붙어 있으면 읽기 전용으로 잠깐 붙임).
# 없는 명령·파일은 그렇다고 적고 다음으로 넘어간다 (set -e를 쓰지 않음). 출력은 영어.
#   IFACE=eth1 sudo -E bash board_info.sh ...         Ethernet 인터페이스 이름이 eth0이 아닐 때

export LC_ALL=C SYSTEMD_PAGER='' PAGER=cat

have() { command -v "$1" > /dev/null 2>&1; }

section() { printf '\n========== %s ==========\n' "$*"; }

# Print the command, run it with a time limit, keep going whatever happens.
run() {
    printf '$ %s\n' "$*"
    if ! have "$1"; then
        echo "(command not found: $1)"
        return 0
    fi
    local rc
    if have timeout; then
        timeout 60 "$@" 2>&1
    else
        "$@" 2>&1
    fi
    rc=$?
    if [ "$rc" -eq 124 ]; then
        echo "(timed out after 60 s)"
    elif [ "$rc" -ne 0 ]; then
        echo "(exit status $rc)"
    fi
    return 0
}

show_file() {
    local f
    for f in "$@"; do
        if [ -r "$f" ]; then
            echo "--- $f"
            cat "$f" 2>&1
            echo
        else
            echo "--- $f: not present"
        fi
    done
}

echo "board_info.sh: $(date -u '+%Y-%m-%d %H:%M:%S UTC' 2>/dev/null), host $(hostname 2>/dev/null), uid $(id -u)"
[ "$(id -u)" = 0 ] || echo "NOTE: not running as root; some items below will be incomplete (use sudo)"

IFACE=${IFACE:-eth0}
if [ ! -e "/sys/class/net/$IFACE" ]; then
    for n in /sys/class/net/*; do
        n=${n##*/}
        case $n in lo | docker* | veth* | br-* | virbr* | wl*) continue ;; esac
        echo "NOTE: $IFACE does not exist; using $n instead"
        IFACE=$n
        break
    done
fi
KREL=$(uname -r 2>/dev/null)

# ---------------------------------------------------------------------------------------------------------------------
section "1. image: /etc/os-release"
show_file /etc/os-release

section "2. image: PYNQ version"
show_file /home/xilinx/REVISION
if [ -r /etc/environment ]; then
    echo "--- /etc/environment (BOARD / XILINX / PYNQ lines)"
    grep -E 'BOARD|XILINX|PYNQ' /etc/environment || echo "(no matching lines)"
fi
run pip3 show pynq
for venv in /usr/local/share/pynq-venv /home/xilinx/pynq-venv; do
    if [ -x "$venv/bin/pip3" ]; then
        run "$venv/bin/pip3" show pynq
    else
        echo "--- $venv/bin/pip3: not present"
    fi
done

section "3. kernel: uname -a"
run uname -a

section "4. kernel: /proc/version (compiler that built the kernel)"
show_file /proc/version

section "5. CPU: nproc"
run nproc

section "6. CPU: lscpu"
run lscpu

section "7. memory: free -m"
run free -m

section "8. disk: df -h /"
run df -h /

section "9. kernel command line: /proc/cmdline"
show_file /proc/cmdline

# ---------------------------------------------------------------------------------------------------------------------
section "10. module build: /lib/modules/$KREL/build"
BUILD=/lib/modules/$KREL/build
if [ -e "$BUILD" ] || [ -L "$BUILD" ]; then
    run ls -ld "$BUILD"
    echo "resolves to: $(readlink -f "$BUILD" 2>/dev/null || echo '(unresolvable)')"
    for f in Makefile .config Module.symvers include/generated/utsrelease.h include/config/kernel.release \
        scripts/mod/modpost; do
        if [ -e "$BUILD/$f" ]; then echo "  present: $f"; else echo "  missing: $f"; fi
    done
    [ -r "$BUILD/include/generated/utsrelease.h" ] && cat "$BUILD/include/generated/utsrelease.h"
    [ -e "$BUILD/scripts/mod/modpost" ] && run file "$BUILD/scripts/mod/modpost"
else
    echo "$BUILD: not present (no kernel headers/build tree for the running kernel)"
fi
run ls /lib/modules

section "11. module build: vermagic of an existing module (a module we build must match this)"
if have modinfo; then
    ko=$(find "/lib/modules/$KREL" -name '*.ko*' 2>/dev/null | head -n 1)
    mod=$(awk 'NR == 1 { print $1 }' /proc/modules 2>/dev/null)
    [ -n "$ko" ] && run modinfo -F vermagic "$ko"
    [ -n "$mod" ] && run modinfo -F vermagic "$mod"
    [ -n "$ko$mod" ] || echo "(no .ko under /lib/modules/$KREL and no loaded module)"
else
    echo "(command not found: modinfo)"
fi
show_file /sys/module/module/parameters/sig_enforce
run lsmod

section "12. module build: installed kernel packages (dpkg -l | grep linux-headers|linux-image)"
if have dpkg; then
    dpkg -l 2>/dev/null | grep -i -E 'linux-headers|linux-image' || echo "(no matching packages)"
else
    echo "(command not found: dpkg)"
fi

section "13. module build: gcc --version"
run gcc --version

section "14. module build: make --version"
run make --version

# ---------------------------------------------------------------------------------------------------------------------
section "15. kernel config"
CONFIG=""
if [ ! -r /proc/config.gz ] && have modprobe; then
    echo "/proc/config.gz not present; trying: modprobe configs"
    modprobe configs 2>&1 || true
fi
if [ -r /proc/config.gz ] && have zcat; then
    echo "source: /proc/config.gz"
    CONFIG=$(zcat /proc/config.gz 2>/dev/null)
elif [ -r "/boot/config-$KREL" ]; then
    echo "source: /boot/config-$KREL"
    CONFIG=$(cat "/boot/config-$KREL")
else
    for f in /boot/config-*; do
        if [ -r "$f" ]; then
            echo "source: $f (WARNING: not named after the running kernel $KREL)"
            CONFIG=$(cat "$f")
            break
        fi
    done
fi
if [ -z "$CONFIG" ]; then
    echo "(no kernel config found: no /proc/config.gz, no /boot/config-*)"
else
    for opt in MODULES MODVERSIONS MODULE_SIG MODULE_SIG_FORCE DEBUG_INFO_BTF MODULE_ALLOW_BTF_MISMATCH NETFILTER \
        PROC_FS FTRACE FUNCTION_TRACER TRACEPOINTS KPROBES BPF_SYSCALL RT_GROUP_SCHED PREEMPT PREEMPT_NONE \
        PREEMPT_VOLUNTARY PREEMPT_DYNAMIC PREEMPT_RT HZ NO_HZ_IDLE NO_HZ_FULL HIGH_RES_TIMERS CC_VERSION_TEXT \
        LOCALVERSION LOCALVERSION_AUTO; do
        line=$(printf '%s\n' "$CONFIG" | grep -E "^(# )?CONFIG_${opt}[= ]" | head -n 1)
        if [ -z "$line" ]; then
            value="(absent: option does not exist in this kernel or its dependencies are off)"
        elif [ "${line#\# }" != "$line" ]; then
            value="not set"
        else
            value=${line#*=}
        fi
        printf '  %-32s %s\n' "CONFIG_$opt" "$value"
    done
fi

# ---------------------------------------------------------------------------------------------------------------------
section "16. systemd: version"
run systemctl --version

section "17. systemd: systemd-detect-virt (none = real hardware)"
run systemd-detect-virt

section "18. systemd: systemctl is-active systemd-modules-load"
run systemctl is-active systemd-modules-load

section "19. systemd: running services"
run systemctl list-units --type=service --state=running --no-pager --no-legend

section "20. systemd: systemctl is-active irqbalance"
run systemctl is-active irqbalance

section "21. systemd: boot time (systemd-analyze)"
run systemd-analyze

section "22. systemd: systemd-analyze critical-chain"
run systemd-analyze critical-chain --no-pager

# ---------------------------------------------------------------------------------------------------------------------
section "23. sysctl"
for key in net.core.rmem_max net.core.rmem_default net.core.netdev_max_backlog net.core.netdev_budget \
    kernel.sched_rt_runtime_us kernel.sched_rt_period_us kernel.core_pattern; do
    p=/proc/sys/${key//.//}
    if [ -r "$p" ]; then
        printf '  %-32s %s\n' "$key" "$(cat "$p")"
    else
        printf '  %-32s %s\n' "$key" "(not present)"
    fi
done

section "24. real-time limits: ulimit -r"
echo "this shell ($(id -un 2>/dev/null)): $(ulimit -r)"
if [ -n "${SUDO_USER:-}" ] && have sudo; then
    echo "user $SUDO_USER: $(sudo -u "$SUDO_USER" bash -c 'ulimit -r' 2>&1)"
fi

section "25. real-time priorities: chrt -m"
run chrt -m

# ---------------------------------------------------------------------------------------------------------------------
section "26. network: ip -br addr"
run ip -br addr

section "27. network: ip route"
run ip route

section "28. NIC ($IFACE): ethtool -i"
run ethtool -i "$IFACE"

section "29. NIC ($IFACE): ethtool -k (offloads: checksum, GRO, ...)"
if have ethtool; then
    echo "$ ethtool -k $IFACE | (selected lines)"
    ethtool -k "$IFACE" 2>&1 | grep -E '^(Features|Cannot|rx-checksumming|tx-checksumming|scatter-gather|tcp-segmentation-offload|generic-segmentation-offload|generic-receive-offload|large-receive-offload|rx-gro-hw|rx-gro-list|rx-udp-gro-forwarding|rx-vlan-offload|receive-hashing)' ||
        echo "(no matching lines)"
else
    echo "(command not found: ethtool)"
fi

section "30. NIC ($IFACE): ethtool -g (ring sizes)"
run ethtool -g "$IFACE"

section "31. NIC ($IFACE): ethtool -S (first 60 lines)"
if have ethtool; then
    echo "$ ethtool -S $IFACE | head -n 60"
    ethtool -S "$IFACE" 2>&1 | head -n 60
else
    echo "(command not found: ethtool)"
fi

section "32. NIC interrupts: /proc/interrupts (header + Ethernet lines) and smp_affinity"
IRQS=""
if [ -r /proc/interrupts ]; then
    lines=$(grep -i -E "$IFACE|macb|gem|ethernet|e000b000" /proc/interrupts)
    if [ -n "$lines" ]; then
        head -n 1 /proc/interrupts
        printf '%s\n' "$lines"
        IRQS=$(printf '%s\n' "$lines" | awk -F: '{ gsub(/ /, "", $1); print $1 }')
    else
        echo "(no Ethernet line found; full table follows)"
        cat /proc/interrupts
    fi
else
    echo "/proc/interrupts: not readable"
fi
for irq in $IRQS; do
    case $irq in *[!0-9]*) continue ;; esac
    for f in smp_affinity smp_affinity_list effective_affinity_list; do
        if [ -r "/proc/irq/$irq/$f" ]; then
            printf '  /proc/irq/%s/%-24s %s\n' "$irq" "$f" "$(cat "/proc/irq/$irq/$f")"
        fi
    done
done
if have ps; then
    echo "--- IRQ / softirq kernel threads (class, RT priority, CPU)"
    ps -eLo pid,cls,rtprio,psr,comm 2>/dev/null | awk 'NR == 1 || /irq\/|ksoftirqd/'
fi

section "33. NIC driver: /sys/bus/platform/drivers (macb / gem)"
if [ -d /sys/bus/platform/drivers ]; then
    found=0
    for d in /sys/bus/platform/drivers/*; do
        case ${d##*/} in
            *macb* | *gem* | *GEM* | *Macb*)
                found=1
                echo "--- $d"
                ls -l "$d" 2>&1
                ;;
        esac
    done
    [ "$found" = 1 ] || echo "(no driver directory matching macb/gem)"
else
    echo "/sys/bus/platform/drivers: not present"
fi

section "34. device tree: ethernet@* nodes (compatible, status, phy-mode)"
if [ -d /proc/device-tree ]; then
    nodes=$(find /proc/device-tree/ -type d -name 'ethernet@*' 2>/dev/null)
    if [ -z "$nodes" ]; then
        echo "(no ethernet@* node)"
    fi
    for n in $nodes; do
        echo "--- $n"
        for prop in compatible status phy-mode phy-connection-type; do
            if [ -r "$n/$prop" ]; then
                printf '  %-20s %s\n' "$prop" "$(tr '\0' ' ' < "$n/$prop")"
            fi
        done
    done
else
    echo "/proc/device-tree: not present"
fi

section "35. NIC driver: /sys/class/net/$IFACE/device/driver"
if [ -e "/sys/class/net/$IFACE/device/driver" ]; then
    echo "link target: $(readlink "/sys/class/net/$IFACE/device/driver")"
    echo "resolved:    $(readlink -f "/sys/class/net/$IFACE/device/driver")"
else
    echo "/sys/class/net/$IFACE/device/driver: not present"
fi

# ---------------------------------------------------------------------------------------------------------------------
section "36. tracing: tracefs / events"
if [ -r /proc/mounts ]; then
    echo "--- tracefs / debugfs mounts"
    grep -E ' (tracefs|debugfs) ' /proc/mounts || echo "(none mounted)"
fi
TRACE=""
for t in /sys/kernel/tracing /sys/kernel/debug/tracing; do
    if [ -e "$t/trace" ]; then
        echo "$t: usable"
        [ -n "$TRACE" ] || TRACE=$t
    else
        echo "$t: not usable (missing or not mounted)"
    fi
done
if [ -n "$TRACE" ]; then
    for e in skb/kfree_skb udp/udp_fail_queue_rcv_skb; do
        if [ -d "$TRACE/events/$e" ]; then echo "  event $e: present"; else echo "  event $e: missing"; fi
    done
    [ -r "$TRACE/available_tracers" ] && echo "  available_tracers: $(cat "$TRACE/available_tracers")"
fi

# ---------------------------------------------------------------------------------------------------------------------
section "37. boot: lsblk -f"
run lsblk -f

section "38. boot: findmnt"
run findmnt

section "39. boot: files on the FAT boot partition"
fat_mounts=""
have findmnt && fat_mounts=$(findmnt -n -r -t vfat -o TARGET 2>/dev/null)
if [ -n "$fat_mounts" ]; then
    for m in $fat_mounts; do
        echo "--- mounted at $m"
        ls -la "$m" 2>&1
    done
elif have lsblk && [ "$(id -u)" = 0 ]; then
    devs=$(lsblk -nrpo NAME,FSTYPE 2>/dev/null | awk '$2 == "vfat" { print $1 }')
    [ -n "$devs" ] || echo "(no vfat partition found)"
    for dev in $devs; do
        tmp=$(mktemp -d)
        if mount -o ro "$dev" "$tmp" 2>/dev/null; then
            echo "--- $dev (not mounted; mounted read-only at $tmp for this listing)"
            ls -la "$tmp" 2>&1
            umount "$tmp" 2>&1
        else
            echo "--- $dev: could not mount read-only"
        fi
        rmdir "$tmp" 2>/dev/null
    done
else
    echo "(no vfat mount found; run as root to mount it read-only for the listing)"
fi

section "40. boot: dmesg | head -n 60"
if have dmesg; then
    dmesg 2>&1 | head -n 60
else
    echo "(command not found: dmesg)"
fi

section "41. boot: dmesg lines for macb / eth0 / machine model / devicetree"
if have dmesg; then
    dmesg 2>&1 | grep -i -E 'macb|eth0|Machine model|Booting Linux|devicetree' || echo "(no matching lines)"
else
    echo "(command not found: dmesg)"
fi

echo
echo "board_info.sh: done"
