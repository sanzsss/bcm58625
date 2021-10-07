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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/mdio.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>

struct peg_combo_phy {
	struct mdio_device *mdiodev;
	struct phy *phy;
};

static int peg_combo_phy_init(struct phy *p)
{
	struct peg_combo_phy *phy = phy_get_drvdata(p);
	struct device *dev = &phy->mdiodev->dev;

	if (of_property_match_string(dev->of_node,
				     "device-type", "pcie") >= 0) {
		dev_info(&phy->mdiodev->dev, "combo-phy: device type PCIE\n");
		return 0;
	}
	if (of_property_match_string(dev->of_node,
				     "device-type", "sata") >= 0) {
		dev_info(&phy->mdiodev->dev, "combo-phy: device type SATA\n");
		/* TBD */
		return -EINVAL;
	}
	dev_err(&phy->mdiodev->dev, "Invalid device-type passed\n");
	return -EIO;
}

static struct phy_ops peg_combo_phy_ops = {
	.init = peg_combo_phy_init,
};

static int peg_combo_phy_probe(struct mdio_device *mdiodev)
{
	struct device *dev = &mdiodev->dev;
	struct phy_provider *provider;
	struct peg_combo_phy *p;
	struct phy *phy;

	phy = devm_phy_create(dev, dev->of_node, &peg_combo_phy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "failed to create Phy\n");
		return PTR_ERR(phy);
	}

	p = devm_kmalloc(dev, sizeof(struct peg_combo_phy),
			 GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->mdiodev = mdiodev;
	dev_set_drvdata(dev, p);

	p->phy = phy;
	phy_set_drvdata(phy, p);

	provider = devm_of_phy_provider_register(&phy->dev,
						 of_phy_simple_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "failed to register Phy provider\n");
		return PTR_ERR(provider);
	}

	dev_info(dev, "%s PHY registered\n", dev_name(dev));

	return 0;
}

static const struct of_device_id peg_combo_phy_of_match[] = {
	{ .compatible = "brcm,peg-combo-phy", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, peg_combo_phy_of_match);

static struct mdio_driver peg_combo_phy_driver = {
	.mdiodrv = {
		.driver = {
			.name = "phy-bcm-peg-combo",
			.of_match_table = peg_combo_phy_of_match,
		},
	},
	.probe = peg_combo_phy_probe,
};
mdio_module_driver(peg_combo_phy_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Pegasus Combo Phy driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:phy-bcm-peg-combo");
