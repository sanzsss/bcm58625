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
#include <linux/clk.h>
#include <linux/of.h>

/* Void parameter because all cores are on the same clock */
static unsigned long cortex_a9_freq(void)
{
	struct clk *c;
	struct device_node *np;
	unsigned long rate = 0;

	/* Use periph_clk to get value of its parent, a9pll clock */
	np = of_find_node_by_name(NULL, "periph_clk");
	if (!np)
		goto err;

	c = of_clk_get_by_name(np, NULL);
	if (IS_ERR(c))
		goto err;

	rate = clk_get_rate(c);
	if (rate)
		rate /= 1000000; /* Change from Hz into MHz */

	clk_put(c);
err:
	return rate;
}

static ssize_t cpuinfo_freq(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%ld MHz\n", cortex_a9_freq());
}

static DEVICE_ATTR(frequency, S_IRUSR, cpuinfo_freq, NULL);

static int __init cpuinfo_init(void)
{
	int cpu;

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

MODULE_DESCRIPTION("Displays Broadcom Northstar Plus CPU Frequency");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
