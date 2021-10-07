/*
 * Copyright (C) 2017 Broadcom Corporation
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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>

/* dmu_spare_ctrl_reg_t */
union dmu_spare_ctrl_reg {
	uint32_t word;

#ifdef BIG_ENDIAN
	struct {
		uint32_t reserved0:8;			/* 31:24 */
		uint32_t synce_use_link_status:1;	/* 23:23 */
		uint32_t synce_use_rxseqdone:1;		/* 22:22 */
		uint32_t synce_clk_musx_sel:3;		/* 21:19 */
		uint32_t synce_val_musx_sel:3;		/* 18:16 */
		uint32_t invert_qspi_wp_polarity:1;	/* 15:15 */
		uint32_t reserved1:15;			/* 14:00 */
	};
#else
	struct {
		uint32_t reserved1:15;			/* 14:00 */
		uint32_t invert_qspi_wp_polarity:1;	/* 15:15 */
		uint32_t synce_val_musx_sel:3;		/* 18:16 */
		uint32_t synce_clk_musx_sel:3;		/* 21:19 */
		uint32_t synce_use_rxseqdone:1;		/* 22:22 */
		uint32_t synce_use_link_status:1;	/* 23:23 */
		uint32_t reserved0:8;			/* 31:24 */
	};
#endif
};

struct synce_cfg {
	int src_clk;
	int use_link;
	int use_rxseq;
};

struct synce_dev {
	struct miscdevice syncedev;
	void __iomem *dmu_synce_addr;
};

static int nsp_synce_get(struct device *dev, struct synce_cfg *synce_cfg)
{
	union dmu_spare_ctrl_reg ctrl_info;
	struct synce_dev *priv = dev_get_drvdata(dev);

	if (!priv->dmu_synce_addr)
		return -ENOMEM;

	ctrl_info.word = readl(priv->dmu_synce_addr);
	synce_cfg->use_link = ctrl_info.synce_use_link_status;
	synce_cfg->use_rxseq = ctrl_info.synce_use_rxseqdone;
	synce_cfg->src_clk = ctrl_info.synce_clk_musx_sel;

	return 0;
}

static int nsp_synce_set(struct device *dev, struct synce_cfg synce_cfg)
{
	union dmu_spare_ctrl_reg ctrl_info;
	struct synce_dev *priv = dev_get_drvdata(dev);

	if (!priv->dmu_synce_addr)
		return -ENOMEM;

	if (synce_cfg.src_clk < 0 || synce_cfg.src_clk > 7)
		return -EINVAL;
	if (synce_cfg.use_link != 0 && synce_cfg.use_link != 1)
		return -EINVAL;
	if (synce_cfg.use_rxseq != 0 && synce_cfg.use_rxseq != 1)
		return -EINVAL;

	ctrl_info.word = readl(priv->dmu_synce_addr);
	ctrl_info.synce_use_link_status = synce_cfg.use_link;
	ctrl_info.synce_use_rxseqdone = synce_cfg.use_rxseq;
	ctrl_info.synce_clk_musx_sel = synce_cfg.src_clk;
	ctrl_info.synce_val_musx_sel = ctrl_info.synce_clk_musx_sel;
	writel(ctrl_info.word, priv->dmu_synce_addr);

	return 0;
}

static ssize_t synce_src_clk_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct synce_cfg tmp_cfg;
	int err;

	memset(&tmp_cfg, 0, sizeof(struct synce_cfg));
	err = nsp_synce_get(dev, &tmp_cfg);
	if (err)
		return err;

	return sprintf(buf, "%d\n", tmp_cfg.src_clk);
}

static ssize_t synce_src_clk_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct synce_cfg tmp_cfg;
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;
	memset(&tmp_cfg, 0, sizeof(struct synce_cfg));
	err = nsp_synce_get(dev, &tmp_cfg);
	if (err)
		return err;
	tmp_cfg.src_clk = val;
	err = nsp_synce_set(dev, tmp_cfg);
	if (err)
		return err;

	return count;
}

static ssize_t synce_use_link_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct synce_cfg tmp_cfg;
	int err;

	memset(&tmp_cfg, 0, sizeof(struct synce_cfg));
	err = nsp_synce_get(dev, &tmp_cfg);
	if (err)
		return err;

	return sprintf(buf, "%d\n", tmp_cfg.use_link);
}

static ssize_t synce_use_link_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct synce_cfg tmp_cfg;
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;
	memset(&tmp_cfg, 0, sizeof(struct synce_cfg));
	err = nsp_synce_get(dev, &tmp_cfg);
	if (err)
		return err;
	tmp_cfg.use_link = val;
	err = nsp_synce_set(dev, tmp_cfg);
	if (err)
		return err;

	return count;
}

static ssize_t synce_use_rxseq_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct synce_cfg tmp_cfg;
	int err;

	memset(&tmp_cfg, 0, sizeof(struct synce_cfg));
	err = nsp_synce_get(dev, &tmp_cfg);
	if (err)
		return err;

	return sprintf(buf, "%d\n", tmp_cfg.use_rxseq);
}

static ssize_t synce_use_rxseq_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct synce_cfg tmp_cfg;
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;
	memset(&tmp_cfg, 0, sizeof(struct synce_cfg));
	err = nsp_synce_get(dev, &tmp_cfg);
	if (err)
		return err;
	tmp_cfg.use_rxseq = val;
	err = nsp_synce_set(dev, tmp_cfg);
	if (err)
		return err;

	return count;
}

static DEVICE_ATTR(synce_src_clks, S_IRUGO | S_IWUSR,
		   synce_src_clk_show,
		   synce_src_clk_store);
static DEVICE_ATTR(synce_use_link, S_IRUGO | S_IWUSR,
		   synce_use_link_show,
		   synce_use_link_store);
static DEVICE_ATTR(synce_use_rxseq, S_IRUGO | S_IWUSR,
		   synce_use_rxseq_show,
		   synce_use_rxseq_store);

static int nsp_synce_probe(struct platform_device *pdev)
{
	struct synce_dev *priv;
	struct resource *res;
	int rv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return rv;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->dmu_synce_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->dmu_synce_addr)) {
		dev_err(&pdev->dev, "Error mapping dmu_synce_addr\n");
		return PTR_ERR(priv->dmu_synce_addr);
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

	rv = device_create_file(&pdev->dev, &dev_attr_synce_src_clks);
	if (rv)
		return rv;

	rv = device_create_file(&pdev->dev, &dev_attr_synce_use_link);
	if (rv)
		return rv;

	rv = device_create_file(&pdev->dev, &dev_attr_synce_use_rxseq);
	if (rv)
		return rv;

	dev_info(&pdev->dev, "NSP Synce registered\n");

	return 0;
}

static int nsp_synce_remove(struct platform_device *pdev)
{
	struct synce_dev *priv = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_synce_src_clks);
	device_remove_file(&pdev->dev, &dev_attr_synce_use_link);
	device_remove_file(&pdev->dev, &dev_attr_synce_use_rxseq);
	misc_deregister(&priv->syncedev);

	return 0;
}

static const struct of_device_id nsp_synce_of_match[] = {
	{ .compatible = "brcm,nsp-synce" },
	{}
};

static struct platform_driver nsp_synce_driver = {
	.probe	= nsp_synce_probe,
	.remove	= nsp_synce_remove,
	.driver = {
		.name = "nsp-synce",
		.of_match_table = nsp_synce_of_match,
	},
};
module_platform_driver(nsp_synce_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM953025 SyncE Driver");
MODULE_LICENSE("GPL v2");
