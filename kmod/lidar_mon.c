// SPDX-License-Identifier: GPL-2.0
/*
 * lidar_mon - IP 계층에서 VLP-16 데이터 패킷을 세는 관찰용 커널 모듈.
 *
 *   NIC → 드라이버 → ip_rcv ─► [PRE_ROUTING 훅: lidar_mon] ─► 라우팅 → UDP → 소켓 수신 큐 → 사용자 공간
 *                                     │
 *                                     └─ /proc/lidar_mon (seen, lost, reordered, restarts, …)
 *
 * UDP 목적지 포트가 맞는 패킷마다 페이로드의 센서 타임스탬프를 읽어 손실·순서 바뀜·흐름 재시작을 판정한다.
 * 판정 규칙은 사용자 공간 해석기(crates/vlp16/src/assemble.rs)와 같다. 패킷은 절대 바꾸지 않고
 * 항상 통과시킨다(NF_ACCEPT).
 *
 * 자세한 설명: docs/modules/kmod.md
 */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/skbuff.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/u64_stats_sync.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <net/ip.h>
#include <net/net_namespace.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
#error "lidar_mon uses struct proc_ops (Linux 5.6+)"
#endif

static ushort port = 2368;
module_param(port, ushort, 0444);
MODULE_PARM_DESC(port, "UDP destination port of the LiDAR data packets (default 2368)");

/* VLP-16 데이터 패킷 (UDP 페이로드) */
#define LM_PAYLOAD_LEN		1206
#define LM_TS_OFFSET		1200	/* 타임스탬프 4 B (little-endian, 정시 이후 µs) + 반환 모드 1 B */
#define LM_RETURN_DUAL		0x39
#define LM_INTERVAL_NS		1327104ULL	/* 단일 반환: 발사 24회 × 55.296 µs */
#define LM_INTERVAL_DUAL_NS	663552ULL	/* 이중 반환: 블록 두 개가 같은 발사 → 패킷당 시간 절반 */

/* 판정 기준 (사용자 공간 해석기와 같음) */
#define LM_HOUR_US		3600000000U
#define LM_REORDER_WINDOW_US	10000
#define LM_RESTART_GAP_US	1000000

/*
 * CPU별 카운터: 훅은 여러 CPU에서 동시에 돌 수 있다. CPU마다 따로 세면 잠금이 필요 없다.
 * u64를 32비트 CPU(보드의 Cortex-A9)에서 읽으면 두 번에 나눠 읽혀 값이 찢어질 수 있어 u64_stats_sync로 감싼다
 * (64비트에서는 아무 비용 없이 사라진다).
 */
struct lm_cpu_stats {
	u64_stats_t seen;	/* 포트가 맞는 UDP 패킷 */
	u64_stats_t malformed;	/* VLP-16 데이터 패킷 길이가 아님 */
	u64_stats_t fragments;	/* 조각난 IP 패킷 (페이로드를 볼 수 없어 건너뜀) */
	struct u64_stats_sync syncp;
};
static DEFINE_PER_CPU(struct lm_cpu_stats, lm_cpu);

/*
 * 6.1 전 커널을 32비트 단일 코어(SMP 아님)로 빌드하면 u64_stats_fetch_begin은 선점만 막아, 읽는 도중
 * softirq(훅)가 끼어들 수 있다 → 인터럽트까지 막는 _irq 판을 쓴다. 6.1부터는 둘이 같다 (소스로 확인).
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
#define lm_fetch_begin u64_stats_fetch_begin_irq
#define lm_fetch_retry u64_stats_fetch_retry_irq
#else
#define lm_fetch_begin u64_stats_fetch_begin
#define lm_fetch_retry u64_stats_fetch_retry
#endif

/*
 * 순서 상태: 패킷을 "지난 패킷"과 비교해야 하므로 하나뿐인 상태를 잠금으로 보호한다.
 * 훅(softirq)에서는 spin_lock, /proc 읽기(프로세스 문맥)에서는 spin_lock_bh를 쓴다.
 */
static struct {
	spinlock_t lock;
	bool have_last;
	u32 last_ts;		/* 지난 패킷의 센서 타임스탬프, 정시 이후 µs */
	u64 lost;		/* 타임스탬프 간격으로 판정한 손실 */
	u64 reordered;		/* 조금 뒤로 간 패킷 (순서 바뀜·중복) */
	u64 restarts;		/* 크게 뒤로 가거나 1초 넘게 끊김 (재생 반복, 센서 재시작) */
	u64 last_arrival_ns;	/* 마지막 패킷이 이 훅에 도착한 시각, CLOCK_REALTIME */
} lm = {
	.lock = __SPIN_LOCK_UNLOCKED(lm.lock),
};

static void lm_count(u64_stats_t *counter)
{
	struct lm_cpu_stats *s = this_cpu_ptr(&lm_cpu);

	u64_stats_update_begin(&s->syncp);
	u64_stats_inc(counter);
	u64_stats_update_end(&s->syncp);
}

/* 지난 패킷과 비교해 손실·순서 바뀜·재시작을 판정한다. 커널에서는 부동소수점을 쓰지 않으므로 정수로 계산. */
static void lm_track(u32 ts, bool dual, u64 now_ns)
{
	u64 interval_ns = dual ? LM_INTERVAL_DUAL_NS : LM_INTERVAL_NS;
	s64 dt_us;

	spin_lock(&lm.lock);
	lm.last_arrival_ns = now_ns;
	if (!lm.have_last)
		goto accept;

	/* 정시 넘김: 30분 넘게 뒤로 간 값은 1시간을 더해 이어 붙임.
	 * 반대로 30분 넘게 앞으로 간 값은 정시 직후에 늦게 온 지난 시간의 패킷 → 1시간을 빼서 조금 뒤로 간 것으로 봄 */
	if (ts < lm.last_ts && lm.last_ts - ts > LM_HOUR_US / 2)
		dt_us = (s64)ts + LM_HOUR_US - lm.last_ts;
	else if (ts > lm.last_ts && ts - lm.last_ts > LM_HOUR_US / 2)
		dt_us = (s64)ts - LM_HOUR_US - lm.last_ts;
	else
		dt_us = (s64)ts - lm.last_ts;

	if (dt_us <= 0 && dt_us > -LM_REORDER_WINDOW_US) {
		lm.reordered++;
		goto out;	/* 지난 패킷 기준은 그대로 둠 */
	}
	if (dt_us <= -LM_REORDER_WINDOW_US || dt_us > LM_RESTART_GAP_US) {
		lm.restarts++;
	} else {
		/* 간격이 패킷 n개분이면 n − 1개 손실. round(dt / interval)을 정수로: (dt + interval/2) / interval */
		u64 n = div64_u64((u64)dt_us * NSEC_PER_USEC + interval_ns / 2, interval_ns);

		if (n > 1)
			lm.lost += n - 1;
	}
accept:
	lm.have_last = true;
	lm.last_ts = ts;
out:
	spin_unlock(&lm.lock);
}

/*
 * 훅 함수. softirq 문맥에서 돈다 → 잠들 수 없다 (GFP_KERNEL 할당, mutex 금지).
 * PRE_ROUTING에서는 skb->data가 IP 헤더를 가리킨다.
 */
static unsigned int lm_hook(void *priv, struct sk_buff *skb, const struct nf_hook_state *state)
{
	struct iphdr _iph;
	const struct iphdr *iph;
	struct udphdr _uh;
	const struct udphdr *uh;
	u8 _tail[5];
	const u8 *tail;
	unsigned int ihl;

	/* 패킷 데이터가 한 덩어리로 붙어 있다는 보장이 없음 → skb_header_pointer로 필요한 바이트만 안전하게 읽음 */
	iph = skb_header_pointer(skb, 0, sizeof(_iph), &_iph);
	if (!iph || iph->protocol != IPPROTO_UDP)
		return NF_ACCEPT;
	ihl = iph->ihl * 4;
	if (ihl < sizeof(*iph))
		return NF_ACCEPT;
	/* 조각난 패킷의 뒷조각에는 UDP 헤더가 없다 → 포트를 알 수 없으므로 그냥 통과 */
	if (ntohs(iph->frag_off) & IP_OFFSET)
		return NF_ACCEPT;

	uh = skb_header_pointer(skb, ihl, sizeof(_uh), &_uh);
	if (!uh || ntohs(uh->dest) != port)
		return NF_ACCEPT;
	if (ip_is_fragment(iph)) {
		lm_count(&this_cpu_ptr(&lm_cpu)->fragments);
		return NF_ACCEPT;
	}

	lm_count(&this_cpu_ptr(&lm_cpu)->seen);
	if (ntohs(uh->len) != sizeof(*uh) + LM_PAYLOAD_LEN) {
		lm_count(&this_cpu_ptr(&lm_cpu)->malformed);
		return NF_ACCEPT;
	}

	/* 페이로드 1200..1204: 타임스탬프 4 B (little-endian) + 반환 모드 1 B. 바이트로 읽어 정렬 문제를 피함 */
	tail = skb_header_pointer(skb, ihl + sizeof(*uh) + LM_TS_OFFSET, sizeof(_tail), _tail);
	if (!tail) {
		lm_count(&this_cpu_ptr(&lm_cpu)->malformed);
		return NF_ACCEPT;
	}
	/* 시각은 포트가 맞는 패킷에서만 읽는다 (다른 모든 IPv4 패킷마다 읽을 필요 없음) */
	lm_track(tail[0] | tail[1] << 8 | tail[2] << 16 | (u32)tail[3] << 24,
		 tail[4] == LM_RETURN_DUAL, ktime_get_real_ns());
	return NF_ACCEPT;	/* 관찰만 한다 */
}

static int lm_show(struct seq_file *m, void *v)
{
	u64 seen = 0, malformed = 0, fragments = 0;
	u64 lost, reordered, restarts, last_arrival_ns;
	u32 last_ts;
	int cpu;

	for_each_possible_cpu(cpu) {
		const struct lm_cpu_stats *s = per_cpu_ptr(&lm_cpu, cpu);
		u64 a, b, c;
		unsigned int start;

		do {	/* 쓰는 쪽(훅)이 중간에 끼어들었으면 다시 읽음 */
			start = lm_fetch_begin(&s->syncp);
			a = u64_stats_read(&s->seen);
			b = u64_stats_read(&s->malformed);
			c = u64_stats_read(&s->fragments);
		} while (lm_fetch_retry(&s->syncp, start));
		seen += a;
		malformed += b;
		fragments += c;
	}

	/* 프로세스 문맥: 같은 CPU에서 softirq(훅)가 끼어들어 같은 잠금을 잡으면 교착 → _bh로 softirq를 막고 잠금 */
	spin_lock_bh(&lm.lock);
	lost = lm.lost;
	reordered = lm.reordered;
	restarts = lm.restarts;
	last_ts = lm.last_ts;
	last_arrival_ns = lm.last_arrival_ns;
	spin_unlock_bh(&lm.lock);

	seq_printf(m, "port %u\n", port);
	seq_printf(m, "seen %llu\n", seen);
	seq_printf(m, "lost %llu\n", lost);
	seq_printf(m, "reordered %llu\n", reordered);
	seq_printf(m, "restarts %llu\n", restarts);
	seq_printf(m, "malformed %llu\n", malformed);
	seq_printf(m, "fragments %llu\n", fragments);
	seq_printf(m, "last_timestamp_us %u\n", last_ts);
	seq_printf(m, "last_arrival_ns %llu\n", last_arrival_ns);
	return 0;
}

static int lm_open(struct inode *inode, struct file *file)
{
	return single_open(file, lm_show, NULL);
}

static const struct proc_ops lm_proc_ops = {
	.proc_open = lm_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static struct nf_hook_ops lm_ops = {
	.hook = lm_hook,
	.pf = NFPROTO_IPV4,
	.hooknum = NF_INET_PRE_ROUTING,
	/* conntrack·방화벽보다 앞 → "여기서 봤다" = "IP 계층에 도착했다" */
	.priority = NF_IP_PRI_FIRST,
};

static struct proc_dir_entry *lm_proc;

static int __init lm_init(void)
{
	int cpu, err;

	for_each_possible_cpu(cpu)
		u64_stats_init(&per_cpu_ptr(&lm_cpu, cpu)->syncp);

	lm_proc = proc_create("lidar_mon", 0444, NULL, &lm_proc_ops);
	if (!lm_proc)
		return -ENOMEM;

	/* 훅은 네트워크 네임스페이스마다 따로 등록한다. 여기서는 기본 네임스페이스만 본다. */
	err = nf_register_net_hook(&init_net, &lm_ops);
	if (err) {
		proc_remove(lm_proc);	/* 실패하면 앞에서 만든 것을 되돌림 */
		return err;
	}
	pr_info("lidar_mon: watching UDP port %u at IPv4 PRE_ROUTING\n", port);
	return 0;
}

static void __exit lm_exit(void)
{
	/*
	 * 훅을 먼저 뗀다. 이 함수는 훅 목록에서 빼기만 하고 기다리지 않는다(옛 목록은 call_rcu로 나중에 해제,
	 * net/netfilter/core.c). 그래서 이미 훅에 들어간 다른 CPU가 잠깐 더 실행할 수 있다.
	 * 그래도 안전한 이유: 훅은 RCU 읽기 구간 안에서 돌고, 훅이 쓰는 데이터는 모두 이 모듈의 정적 변수다.
	 * 모듈 로더가 모듈 메모리를 풀기 전에 synchronize_rcu()로 진행 중인 훅이 끝나길 기다린다(kernel/module/main.c).
	 * (훅이 kmalloc한 메모리를 쓴다면 여기서 synchronize_net() 뒤에 풀어야 한다.)
	 */
	nf_unregister_net_hook(&init_net, &lm_ops);
	proc_remove(lm_proc);
	pr_info("lidar_mon: unloaded\n");
}

module_init(lm_init);
module_exit(lm_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Count VLP-16 LiDAR packets at the IPv4 layer and detect loss from sensor timestamps");
