/*
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/cpu.h>
#include <linux/efi.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/arm-smccc.h>

/* function ids for SMC calls to oem service*/
#define OEM_CPU_FREQ_INFO	0xC300FF03

struct smc_data {
	struct mutex lock;
	/*
	 * fid will be used for passing function id to
	 * smc internal function and the result of smc call will
	 * be stored in smc_retval after execution.
	 */
	u64 fid;
	struct arm_smccc_res res;
};

static struct smc_data smc_fn_data;

/*
 * smc calls from secodary cpu cores will produce undefined exception, Do not
 * call this function directly on smp systems. Use smp_call_function_single()
 * or other similar method to execute it on cpu core 0 only.
 */
static void iproc_smc_internal(void *info)
{
	struct smc_data *data = (struct smc_data *)info;

	arm_smccc_smc(data->fid, 0, 0, 0, 0, 0, 0, 0,
		      &data->res);
}

static int cpufreq_fetch(void)
{
	int ret = 0;

	mutex_lock_interruptible(&smc_fn_data.lock);

	smc_fn_data.fid = OEM_CPU_FREQ_INFO; /* SMC function id */
	ret = smp_call_function_single(0, iproc_smc_internal,
						&smc_fn_data, true);
	if (ret < 0) {
		pr_err("smc_call_function_single() failed, ret=%d\n", ret);
		goto out;
	}
	ret = smc_fn_data.res.a0; /* value returned from smc call */
out:
	mutex_unlock(&smc_fn_data.lock);
	return ret;
}

static ssize_t cpuinfo_freq(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%d MHz\n", cpufreq_fetch());
}

static DEVICE_ATTR(frequency, S_IRUSR, cpuinfo_freq, NULL);

static int __init cpuinfo_init(void)
{
	int cpu;

	mutex_init(&smc_fn_data.lock);
	for_each_possible_cpu(cpu)
		device_create_file(get_cpu_device(cpu), &dev_attr_frequency);

	return 0;
}

static void __exit cpuinfo_exit(void)
{
	int cpu;

	for_each_possible_cpu(cpu)
		device_remove_file(get_cpu_device(cpu), &dev_attr_frequency);
}

module_init(cpuinfo_init);
module_exit(cpuinfo_exit);

MODULE_DESCRIPTION("Displays Broadcom CPU Frequency");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
