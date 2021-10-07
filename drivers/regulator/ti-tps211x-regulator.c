/*
 * Support for TI TPS211X voltage regulator
 *
 * Copyright (C) 2015 Broadcom Corporation
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#define TPS211X_VOLTAGE_MIN	1800000
#define TPS211X_VOLTAGE_MAX	3600000
#define TPS211X_VOLTAGE_STEP	1600000

#define SDIO0_1V3_MASK		(1 << 6)
#define SDIO1_1V3_MASK		(1 << 7)
#define SDIO0_REGLT_ID		0
#define SDIO1_REGLT_ID		1

#define BOARD_REV_20		"rev_20"
#define BOARD_REV_30		"rev_30"

/* num of regulators configurations */
#define MAX_REGULATOR_CFGS	2
/* total selectors available per regualtor */
#define MAX_VOL_SELECTORS	2

/* PMIC details */
struct ti_tps211x {
	struct i2c_client	*client;
	struct regulator_dev	*rdev[MAX_REGULATOR_CFGS];
	struct mutex		mtx;
	struct regulator_consumer_supply supply;
	struct regulator_init_data init_data;
	bool enable_reglt;
	bool invert_vals;
};

static int ti_tps_rglt_get_voltage_sel(struct regulator_dev *dev)
{
	struct ti_tps211x *tps211x = rdev_get_drvdata(dev);
	int idx, sel;

	mutex_lock(&tps211x->mtx);

	idx = i2c_smbus_read_byte(tps211x->client);
	if (idx < 0) {
		dev_err(&tps211x->client->dev, "Error getting voltage\n");
		sel = -EIO;
	} else {
		if ((idx & SDIO0_1V3_MASK) || (idx & SDIO1_1V3_MASK))
			sel = tps211x->invert_vals ? 0 : 1;
		else
			sel = tps211x->invert_vals ? 1 : 0;
	}

	mutex_unlock(&tps211x->mtx);

	return sel;
}

static int ti_tps_rglt_set_voltage_sel(struct regulator_dev *dev,
				    unsigned selector)
{
	struct ti_tps211x *tps211x = rdev_get_drvdata(dev);
	int err, idx;
	u8 mask;
	u8 byte;

	mutex_lock(&tps211x->mtx);

	idx = i2c_smbus_read_byte(tps211x->client);
	if (idx < 0) {
		dev_err(&tps211x->client->dev, "Error getting voltage\n");
		err = idx;
		goto end;
	}

	/* check the slot */
	mask = dev->desc->id == SDIO0_REGLT_ID ? SDIO0_1V3_MASK :
							SDIO1_1V3_MASK;

	if (selector == 0) /* switch to 1.8V */
		byte = tps211x->invert_vals ? (idx | mask) :
					(idx & ~(mask));
	else /* switch to 3.3V */
		byte = tps211x->invert_vals ? (idx & ~(mask)) :
					(idx | mask);

	err = i2c_smbus_write_byte(tps211x->client, byte);
	if (err < 0) {
		dev_err(&tps211x->client->dev, "Error setting voltageV\n");
		goto end;
	}

	err = i2c_smbus_read_byte(tps211x->client);
	if (err < 0) {
		dev_err(&tps211x->client->dev, "Error getting voltage\n");
	} else if (err != byte) {
		dev_err(&tps211x->client->dev, "Could not set voltage level.\n");
		err = -EIO;
	} else {
		err = 0;
	}

end:
	mutex_unlock(&tps211x->mtx);
	return err;
}

static int ti_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct ti_tps211x *tps211x = rdev_get_drvdata(rdev);
	int idx;

	if (!tps211x->enable_reglt)
		return 0;

	mutex_lock(&tps211x->mtx);
	/*for debugging purpose, print the current value in dmesg*/
	idx = i2c_smbus_read_byte(tps211x->client);
	if (idx < 0)
		dev_err(&tps211x->client->dev, "Error getting voltage\n");
	else
		dev_info(&tps211x->client->dev, "Current regulator value:%x\n",
									idx);

	mutex_unlock(&tps211x->mtx);

	return 1;
}

static int ti_regulator_enable(struct regulator_dev *rdev)
{
	struct ti_tps211x *tps211x = rdev_get_drvdata(rdev);

	tps211x->enable_reglt = true;
	return 0;
}

static int ti_regulator_disable(struct regulator_dev *rdev)
{
	struct ti_tps211x *tps211x = rdev_get_drvdata(rdev);

	tps211x->enable_reglt = false;
	return 0;
}

static struct regulator_ops tps_core_ops = {
	.get_voltage_sel	= ti_tps_rglt_get_voltage_sel,
	.set_voltage_sel	= ti_tps_rglt_set_voltage_sel,
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= ti_regulator_is_enabled,
	.enable			= ti_regulator_enable,
	.disable		= ti_regulator_disable
};

static const struct regulator_desc tps_rd[] = {
	{
		.name		= "ti_tps_rglt0",
		.id		= SDIO0_REGLT_ID,
		.n_voltages	= MAX_VOL_SELECTORS,
		.ops		= &tps_core_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
		.min_uV		= TPS211X_VOLTAGE_MIN,
		.uV_step	= TPS211X_VOLTAGE_STEP,
	},
	{
		.name		= "ti_tps_rglt1",
		.id		= SDIO1_REGLT_ID,
		.n_voltages	= MAX_VOL_SELECTORS,
		.ops		= &tps_core_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
		.min_uV		= TPS211X_VOLTAGE_MIN,
		.uV_step	= TPS211X_VOLTAGE_STEP,
	},
};

static int ti_tps_rglt_probe(struct i2c_client *i2c,
				     const struct i2c_device_id *id)
{
	struct device_node *nproot1, *nproot2, *np;
	struct ti_tps211x *tps211x;
	int ret, i, j;
	int initial_val;
	const char *board_rev;

	ret = i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA);
	if (!ret)
		return -EIO;

	tps211x = devm_kzalloc(&i2c->dev,
				sizeof(struct ti_tps211x), GFP_KERNEL);
	if (!tps211x)
		return -ENOMEM;

	tps211x->client = i2c;

	mutex_init(&tps211x->mtx);

	tps211x->init_data.constraints.valid_ops_mask =
		REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS;
	tps211x->init_data.constraints.min_uV		= TPS211X_VOLTAGE_MIN;
	tps211x->init_data.constraints.max_uV		= TPS211X_VOLTAGE_MAX;
	tps211x->init_data.consumer_supplies		= &tps211x->supply;
	tps211x->init_data.consumer_supplies->supply	= "vqmmc";

	for (i = 0; i < MAX_REGULATOR_CFGS; i++) {
		struct regulator_config config = { };

		dev_info(&i2c->dev, "Configure data...\n");
		config.dev = &i2c->dev;
		config.driver_data = tps211x;
		config.init_data = &tps211x->init_data;

		nproot1 = of_node_get(config.dev->of_node);
		if (!nproot1) {
			dev_err(&i2c->dev, "of_node_get(config.dev->of_node); failed");
			ret = -ENODEV;
			goto error;
		}

		nproot2 = of_find_node_by_name(nproot1, "regulators");
		if (!nproot2) {
			dev_err(&i2c->dev, "of_find_node_by_name(nproot1, regulators); failed");
			ret = -ENODEV;
			goto error;
		}

		for_each_child_of_node(nproot2, np) {
			if (!of_node_cmp(np->name,
						tps_rd[i].name)) {
				config.of_node = np;
				break;
			}
		}

		tps211x->rdev[i] = regulator_register(&tps_rd[i], &config);
		if (IS_ERR(tps211x->rdev[i])) {
			dev_err(&i2c->dev, "failed to register %s\n", id->name);
			ret = PTR_ERR(tps211x->rdev[i]);
			goto error;
		}

		tps211x->rdev[i]->constraints->max_uV = TPS211X_VOLTAGE_MAX;
		tps211x->rdev[i]->constraints->min_uV = TPS211X_VOLTAGE_MIN;
		of_node_put(nproot1);
		of_node_put(nproot2);
	}

	i2c_set_clientdata(i2c, tps211x);

	/*
	 * I2C values to control voltage regulator are inverted for rev20
	 * So, check if board is rev20 from device tree property.
	 */
	tps211x->invert_vals = false;
	ret = of_property_read_string(i2c->dev.of_node, "board-rev",
								&board_rev);
	if (ret == 0) {
		if (strncmp(board_rev, BOARD_REV_20,
						strlen(BOARD_REV_20)) == 0) {
			dev_info(&i2c->dev, "inverted vals for rev20 compatibility\n");
			tps211x->invert_vals = true;
		}
	}

	initial_val = tps211x->invert_vals ? 0xFF : 0x3F;

	/* Setting default 3.3V for both slots */
	ret = i2c_smbus_write_byte(i2c, initial_val);
	if (ret < 0) {
		dev_err(&i2c->dev, "Error setting voltage\n");
		ret = -EIO;
		goto error;
	}

	return 0;
error:

	if (!nproot1)
		of_node_put(nproot1);
	if (!nproot2)
		of_node_put(nproot2);

	for (j = 0; j < i; j++)
		regulator_unregister(tps211x->rdev[j]);
	return ret;
}

static const struct i2c_device_id ti_tps_rglt_id = {
	.name = "ns2_regulator", 0
};

static const struct of_device_id ti_iproc_i2c_of_match = {
	.compatible = "ti,iproc-tps211x"
};

MODULE_DEVICE_TABLE(i2c, ti_tps_rglt_id)

static struct i2c_driver ti_tps_rglt_i2c_driver = {
	.driver = {
		.name = "ti-tps-rglt-i2c",
		.of_match_table = &ti_iproc_i2c_of_match,
		.owner = THIS_MODULE,
	},
	.probe = ti_tps_rglt_probe,
	.id_table = &ti_tps_rglt_id,
};

module_i2c_driver(ti_tps_rglt_i2c_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("TI TPS211X voltage regulator driver");
MODULE_LICENSE("GPL");
