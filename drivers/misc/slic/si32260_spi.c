/*
 * Copyright (C) 2016 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>

#include "si32260_spi.h"

static DEFINE_MUTEX(spi_add_lock);

static const struct file_operations si32260_fops;

struct si32260_drv_data {
	dev_t dev;
	struct cdev c_dev;
	struct class *cl;
	struct spi_device *spi;
};

static struct si32260_drv_data *ddata;

static int si32260_reg_read(struct spi_device *spi, unsigned char channel,
			unsigned char reg, unsigned char *buf)
{
	unsigned char dout[3];
	unsigned char *din = buf;

	struct spi_transfer     t[3];
	struct spi_message      m;

	int status;

	/* Delay 1 ms */
	udelay(1000);

	spi_message_init(&m);
	memset(t, 0, sizeof(t));

	/* Add Control Byte */
	if (channel == 0) {
		/*Control byte for Slic Reg Read on Channel 0*/
		dout[0] = 0x60;
	} else {
		/*Control byte for Slic Reg Read on Channel 1*/
		dout[0] = 0x70;
	}

	t[0].tx_buf = &dout[0];
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);

	/* Add Address Byte */
	dout[1] = reg;
	t[1].tx_buf = &dout[1];
	t[1].len = 1;
	spi_message_add_tail(&t[1], &m);

	/* Read Data Byte */
	t[2].rx_buf = buf;
	t[2].len = 1;
	spi_message_add_tail(&t[2], &m);

	mutex_lock(&spi_add_lock);

	status = spi_sync(spi, &m);

	if (status) {
		dev_err(&spi->dev, "si32260: spi_sync cmd status %d\n", status);
		mutex_unlock(&spi_add_lock);
		return status;
	}

	mutex_unlock(&spi_add_lock);

	dev_dbg(&spi->dev, "si32260: register read - Channel %d: Reg %d: value: 0x%x\n",
							channel, reg, *din);

	return 0;
}

static int si32260_reg_write(struct spi_device *spi, unsigned char channel,
				unsigned char reg, unsigned char data)
{
	unsigned char dout;
	int status;

	/* Delay 1 ms */
	udelay(1000);

	/* Send Control Byte */
	if (channel == 0) {
		/*Control byte for Slic Reg write on Channel 0*/
		dout = 0x20;
	} else {
		/*Control byte for Slic Reg write on Channel 1*/
		dout = 0x30;
	}

	mutex_lock(&spi_add_lock);

	status = spi_write(spi, &dout, 1);
	if (status) {
		dev_err(&spi->dev, "si32260: spi write cmd status %d\n",
								status);
		goto end;
	}

	/* Send Address Byte */
	dout = reg;
	status = spi_write(spi, &dout, 1);
	if (status) {
		dev_err(&spi->dev, "si32260: spi write addr status %d\n",
								status);
		goto end;
	}

	/* Write Data Byte */
	dout = data;
	status = spi_write(spi, &dout, 1);
	if (status) {
		dev_err(&spi->dev, "si32260: spi write data status %d\n",
								status);
		goto end;
	}


	dev_dbg(&spi->dev, "si32260: reg write - Channel %d Reg %d: value: 0x%x\n",
						channel, reg, dout);
end:
	mutex_unlock(&spi_add_lock);
	return status;
}

static int check_ram_intf_busy_status(struct spi_device *spi,
				unsigned char channel, unsigned int address)
{
	int i;
	unsigned char reg_val;
	int status;

	status = si32260_reg_read(spi, channel, SI32260_RAMSTAT_REG, &reg_val);
	if (status)
		return status;

	/*SI32260_RAMSTAT_REG indicating busy*/
	for (i = 0; (reg_val & 0x1) && (i < 5); i++) {
		mdelay(100);
		dev_dbg(&spi->dev, "si32260: RAMSTAT_REG busy bit is set. Retry %d\n",
									i + 1);
		si32260_reg_read(spi, channel, SI32260_RAMSTAT_REG, &reg_val);
	}
	if (i == 5) {
		dev_err(&spi->dev, "si32260: RAM_WRITE to addr:%d failed\n",
			((address & 0x700) | (address & 0xff)));
		return -EBUSY;
	}

	return 0;
}

/* The SLIC scratch RAM has 1639 address locations, each 29-bit wide.
 * To access full 29-bits during read/write to RAM space, indirect RAM
 * access method should be used.
 * To access just 16-bits, direct RAM access method should be used.
 * To access location 1024 or above, protected mode should be set.
 * The present code uses indirect RAM access methods.
 */

/* Indirect RAM access - read */
static int si32260_ram_read(struct spi_device *spi, unsigned char channel,
					unsigned int address, unsigned int *buf)
{
	unsigned int data;
	unsigned char reg_val;
	int status;

	/*Get ram_stat prior to reading*/
	status = si32260_reg_read(spi, channel, SI32260_RAMSTAT_REG, &reg_val);
	if (status)
		return status;

	/* Write Reg 5(SI32260_RAM_ADDR_HI) to set up bits [10:8]
	 * of the RAM address
	 */
	status = si32260_reg_write(spi, channel, SI32260_RAM_ADDR_HI,
			(unsigned char)((address >> 3) & 0xE0));
	if (status)
		return status;

	/* Write Reg 10(SI32260_RAM_ADDR_LO) to set up bits [7:0] */
	status = si32260_reg_write(spi, channel, SI32260_RAM_ADDR_LO,
					(unsigned char)(address & 0xFF));
	if (status)
		return status;

	/* Wait for RAM_STAT bit of Reg 4 to clear for Write complete. */
	status = check_ram_intf_busy_status(spi, channel, address);
	if (status)
		return status;

	/* Read the 29-bit data from Reg 6 thru Reg 9
	 * (SI32260_RAM_DATA_B0 thru SI32260_RAM_DATA_B3)
	 */

	/*Bits 0-4*/
	status = si32260_reg_read(spi, channel, SI32260_RAM_DATA_B0, &reg_val);
	if (status)
		return status;

	/*Bits 7:3 correspond to Ram Data 4:0*/
	*buf = (unsigned int)((reg_val & 0xF8) >> 3);

	/*Bits 5-12*/
	status = si32260_reg_read(spi, channel, SI32260_RAM_DATA_B1, &reg_val);
	if (status)
		return status;

	data = reg_val;
	*buf |= data << 5;

	/*Bits 13-20*/
	status = si32260_reg_read(spi, channel, SI32260_RAM_DATA_B2, &reg_val);
	if (status)
		return status;

	data = reg_val;
	*buf |= data << 13;

	/*Bits 21-28*/
	status = si32260_reg_read(spi, channel, SI32260_RAM_DATA_B3, &reg_val);
	if (status)
		return status;

	data = reg_val;
	*buf |= data << 21;

	dev_dbg(&spi->dev, "si32260: scratch RAM read - Channel %d: Address %d: value: 0x%x\n",
							channel, address, *buf);

	return 0;
}

static int si32260_ram_write(struct spi_device *spi, unsigned char channel,
					unsigned int address, unsigned int data)
{
	unsigned int addr;
	unsigned char value;
	int status;

	/* Write Reg 5(SI32260_RAM_ADDR_HI) to set
	 * up bits [10:8] of the RAM address
	 */

	addr = (address & 0x700) >> 3;
	status = si32260_reg_write(spi, channel, SI32260_RAM_ADDR_HI,
						(unsigned char)addr);
	if (status)
		return status;

	/* Write the 29-bit data to Reg 6 thru Reg 9
	 * (SI32260_RAM_DATA_B0 thru SI32260_RAM_DATA_B3)
	 */

	/*Write Bits 4:0 in data to bits 7:3*/
	value = (unsigned char)((data & 0x1F) << 3);
	status = si32260_reg_write(spi, channel, SI32260_RAM_DATA_B0, value);
	if (status)
		return status;

	/*Bits 12:5*/
	value = (unsigned char)((data >> 5) & 0xFF);
	status = si32260_reg_write(spi, channel, SI32260_RAM_DATA_B1, value);
	if (status)
		return status;

	/*Bits 20:13*/
	value = (unsigned char)((data >> 13) & 0xFF);
	status = si32260_reg_write(spi, channel, SI32260_RAM_DATA_B2, value);
	if (status)
		return status;

	/*Bits 28:21*/
	value = (unsigned char)((data >> 21) & 0xFF);
	status = si32260_reg_write(spi, channel, SI32260_RAM_DATA_B3, value);
	if (status)
		return status;

	/* Write Reg 10(SI32260_RAM_ADDR_LO) to set up bits [7:0] */
	addr = (address & 0xFF);
	status = si32260_reg_write(spi, channel, SI32260_RAM_ADDR_LO,
						(unsigned char)addr);
	if (status)
		return status;

	/* Wait for RAM_STAT bit of Reg 4 to clear for Write complete. */
	status = check_ram_intf_busy_status(spi, channel, address);
	if (status)
		return status;

	dev_dbg(&spi->dev, "si32260: scratch RAM Write - Channel %d Address %d: value: 0x%x\n",
							channel, address, data);

	return 0;
}

static int si32260_probe(struct spi_device *spi)
{
	int retval;
	struct device *dev;

	ddata = kmalloc(sizeof(*ddata), GFP_KERNEL);
	if (ddata == NULL) {
		retval = -ENOMEM;
		goto end;
	}

	retval = alloc_chrdev_region(&ddata->dev, 0, 1, "si32260-spi-ctrl");
	if (retval < 0)
		goto release_mem;

	ddata->cl = class_create(THIS_MODULE, "spidrv");
	if (ddata->cl == NULL) {
		retval = -ENODEV;
		goto unregister_device;
	}

	dev = device_create(ddata->cl, NULL, ddata->dev, NULL,
						"si32260-spi-ctrl");
	if (dev == NULL) {
		retval = -ENODEV;
		goto destroy_class;
	}

	cdev_init(&ddata->c_dev, &si32260_fops);

	retval = cdev_add(&ddata->c_dev, ddata->dev, 1);
	if (retval < 0)
		goto destroy_device;

	dev_set_drvdata(&spi->dev, ddata);

	ddata->spi = spi;

	return 0;

destroy_device:
	device_destroy(ddata->cl, ddata->dev);
destroy_class:
	class_destroy(ddata->cl);
unregister_device:
	unregister_chrdev_region(ddata->dev, 1);
release_mem:
	kfree(ddata);
end:
	return retval;
}

static int si32260_remove(struct spi_device *spi)
{
	cdev_del(&ddata->c_dev);
	device_destroy(ddata->cl, ddata->dev);
	class_destroy(ddata->cl);
	unregister_chrdev_region(ddata->dev, 1);
	kfree(ddata);
	return 0;
}

static const struct of_device_id ns2_mach_of_match[] = {
	{.compatible = "bcm,si32260-spi-ctrl", },
	{ },
};

MODULE_DEVICE_TABLE(of, ns2_mach_of_match);

static struct spi_driver si32260_spi_driver = {
	.driver = {
		.name	= "si32260-ctrl-interface",
		.owner	= THIS_MODULE,
		.of_match_table = ns2_mach_of_match,
	},
	.probe	= si32260_probe,
	.remove	= si32260_remove,
};

static int si32260_open(struct inode *i, struct file *f)
{
	return 0;
}

static int si32260_close(struct inode *i, struct file *f)
{
	return 0;
}

static ssize_t si32260_read(struct file *f, char __user *buf,
	size_t len, loff_t *off)
{
	return 0;
}

static ssize_t si32260_write(struct file *f, const char __user *buf,
	size_t len, loff_t *off)
{
	return len;
}

static long int si32260_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	struct si32260_ioctl_data *p = (struct si32260_ioctl_data *) arg;
	unsigned char channel;
	unsigned char reg;
	int val;
	unsigned int data;
	unsigned int address;
	int retval;

	if (copy_from_user(&channel, &p->channel, sizeof(channel)) != 0)
		return -EFAULT;

	if (copy_from_user(&reg, &p->reg, sizeof(reg)) != 0)
		return -EFAULT;

	if (copy_from_user(&data, &p->data, sizeof(data)) != 0)
		return -EFAULT;

	if (copy_from_user(&address, &p->address, sizeof(address)) != 0)
		return -EFAULT;

	switch (cmd) {
	case REG_READ:
		retval = si32260_reg_read(ddata->spi, channel, reg,
						(unsigned char *)&val);

		if (copy_to_user(&p->data, &val, sizeof(char)) != 0)
			return -EFAULT;

		break;
	case REG_WRITE:
		retval = si32260_reg_write(ddata->spi, channel, reg, data);
		break;

	case RAM_READ:
		retval = si32260_ram_read(ddata->spi, channel,
				address, (unsigned int *)&val);
		if (copy_to_user(&p->data, &val, sizeof(int)) != 0)
			return -EFAULT;
		break;

	case RAM_WRITE:
		retval = si32260_ram_write(ddata->spi, channel, address, data);
		break;

	default:
		retval = -EINVAL;
		break;
	}

	return retval;
}

static const struct file_operations si32260_fops = {
	.owner = THIS_MODULE,
	.open = si32260_open,
	.release = si32260_close,
	.read = si32260_read,
	.write = si32260_write,
	.unlocked_ioctl = si32260_ioctl,
};

static int __init si32260_init(void)
{
	int status;

	status = spi_register_driver(&si32260_spi_driver);
	if (status < 0) {
		pr_err("si32260: init failed\n");
		return status;
	}

	return 0;
}

static void __exit si32260_exit(void)
{
	spi_unregister_driver(&si32260_spi_driver);
	pr_info("si32260: unregistered");
}

module_init(si32260_init);
module_exit(si32260_exit);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("Proslic SPI Control Interface Driver");
MODULE_LICENSE("GPL v2");
