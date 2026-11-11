// 직접 빌드한 커널 트리로 만든 모듈이 실행 중인 WSL 커널에 올라가는지만 확인하는 최소 모듈.
// 확인하는 것: vermagic(버전 문자열)과 심볼 체크섬(_printk 등)이 맞는가.
#include <linux/module.h>
#include <linux/init.h>

static int __init smoke_init(void)
{
	pr_info("smoke: loaded\n");
	return 0;
}

static void __exit smoke_exit(void)
{
	pr_info("smoke: unloaded\n");
}

module_init(smoke_init);
module_exit(smoke_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("toolchain smoke test");
