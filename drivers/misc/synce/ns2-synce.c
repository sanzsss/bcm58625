/*
 * Copyright (C) 2017 Broadcom
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>

#define SYNCE_BIT_MASK 0x00000007

struct synce_dev {
	struct miscdevice syncedev;
	void __iomem *synce_addr;
	int synce_clk_sel;
};

static int ns2_synce_get(struct device *dev)
{
	struct synce_dev *priv = dev_get_drvdata(dev);
	u32 val;

	if (!priv->synce_addr)
		return -ENOMEM;

	val = readl(priv->synce_addr);
	priv->synce_clk_sel = val & SYNCE_BIT_MASK;
	dev_dbg(dev, "ns2_synce_get: reg_val %x\n", priv->synce_clk_sel);

	return 0;
}

static int ns2_synce_set(struct device *dev, unsigned long val)
{
	struct synce_dev *priv = dev_get_drvdata(dev);
	u32 reg;

	if (!priv->synce_addr)
		return -ENOMEM;

	if (val < 0 || val > 7)
		return -EINVAL;

	reg = readl(priv->synce_addr);
	reg &= ~SYNCE_BIT_MASK;
	reg |= (val & SYNCE_BIT_MASK);
	dev_dbg(dev, "ns2_synce_set: reg_val %x\n", reg);
	writel(reg, priv->synce_addr);

	return 0;
}

static ssize_t synce_src_clk_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct synce_dev *priv = dev_get_drvdata(dev);
	int err;

	err = ns2_synce_get(dev);
	if (err)
		return err;

	return sprintf(buf, "%d\n", priv->synce_clk_sel);
}

static ssize_t synce_src_clk_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	err = ns2_synce_get(dev);
	if (err)
		return err;

	err = ns2_synce_set(dev, val);
	if (err)
		return err;

	return count;
}

static DEVICE_ATTR(synce_src_clks, S_IRUGO | S_IWUSR,
		   synce_src_clk_show,
		   synce_src_clk_store);

static int ns2_synce_probe(struct platform_device *pdev)
{
	struct synce_dev *priv;
	struct resource *res;
	int rv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->synce_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->synce_addr)) {
		dev_err(&pdev->dev, "Error mapping synce_addr\n");
		return PTR_ERR(priv->synce_addr);
	}

	priv->syncedev.name = "synce";
	priv->syncedev.minor = MISC_DYNAMIC_MINOR;
	priv->syncedev.parent = &pdev->dev;

	rv = misc_register(&priv->syncedev);
	if (rv) {
		dev_err(&pdev->dev, "failed to register syncedev\n");
		return rv;
	}

	dev_set_drvdata(&pdev->dev, priv);

	rv = device_create_file(&pdev->dev,
				&dev_attr_synce_src_clks);
	if (rv)
		return rv;

	dev_info(&pdev->dev, "Northstar2 Synce registered\n");

	return 0;
}

static int ns2_synce_remove(struct platform_device *pdev)
{
	struct synce_dev *priv = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_synce_src_clks);
	misc_deregister(&priv->syncedev);

	return 0;
}

static const struct of_device_id ns2_synce_of_match[] = {
	{ .compatible = "brcm,ns2-synce" },
	{}
};

static struct platform_driver ns2_synce_driver = {
	.probe	= ns2_synce_probe,
	.remove	= ns2_synce_remove,
	.driver = {
		.name = "ns2-synce",
		.of_match_table = ns2_synce_of_match,
	},
};
module_platform_driver(ns2_synce_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Northstar2 SyncE Driver");
MODULE_LICENSE("GPL v2");
