
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/pm_wakeup.h>


static struct notifier_block pm_nb;
struct wakeup_source *delay_wlock;


static int wk_delay_resume(struct notifier_block *nb,
				unsigned long val, void *ign)
{
	switch (val) {
	case PM_POST_SUSPEND:
		__pm_wakeup_event(delay_wlock, 3000);
	default:
		return NOTIFY_DONE;
	}
}

static int __init wk_delay_init(void)
{
	delay_wlock = wakeup_source_register("delay_wakelock");

	if (!delay_wlock) {
		printk ("ERROR: failed to register delay_wakelock\n");
		return -1;
	}

	pm_nb.priority = 0;
	pm_nb.notifier_call = wk_delay_resume;

	return register_pm_notifier(&pm_nb);
}

subsys_initcall(wk_delay_init);

