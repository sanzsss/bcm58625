/*
 * Copyright (c) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/slab.h>
#include <linux/io.h>

/*
 *  Driver for iProc IDM bus slave wrapper timeout/error handling
 *
 *  This driver is intended to treat all of the IDM bus slave wrappers
 *  together, so they are all monitored by a single driver.  It does not,
 *  however, probe multiple IDM slave wrappers on interrupt -- it only services
 *  the wrapper assigned to the particular interrupt (and the assumption is
 *  that they all have unique interrupts).  Maybe we could add polling
 *  functionality in the future (this code should be reasonably easily
 *  extensible for that) but it is not supported at this time.
 *
 *  If, for some reason, it is preferable to have a separate instance of the
 *  driver for some wrapper(s), this should be possible by adding an additional
 *  instance of the device in the device tree.  However, this currently seems
 *  wasteful on several fronts.
 */

/*
 *  This describes a single IDM slave wrapper, including 'fixed' attributes
 *  (such as name and base address) and configurable features (such as how
 *  particular faults are handled).
 */
enum idm_slave_flags {
	IDM_ERROR_INTERRUPT   = 0x000000001, /* interrupt on other error */
	IDM_ERROR_RESET       = 0x000000002, /* reset slave on other error */
	IDM_TIMEOUT_INTERRUPT = 0x000000004, /* interrupt on timeout */
	IDM_TIMEOUT_RESET     = 0x000000008, /* reset slave on timeout */
};

struct idm_info;
struct idm_slave_info {
	struct idm_info *info;	/* pointer to idm_info instance */
	char *name;		/* slave port name */
	u8 *base;		/* IDM error handling ctrl base (mapped) */
	phys_addr_t physBase;	/* IDM error handling ctrl base (physical) */
	phys_addr_t physEnd;	/* IDM error handling ctrl limit (physical) */
	int interrupt;		/* IDM error interrupt */
	u32 flags;		/* flags (see idm_slave_flags above) */
	u32 timeout_exp;	/* timeout exponent */
	unsigned int errors;	/* number of detected errors */
	unsigned int timeouts;	/* number of detected timeouts */
};

struct idm_info {
	int slaveCount;			/* number of slaves */
	struct device *dev;		/* device information */
	struct idm_slave_info slave[];	/* data for the slaves */
};

#ifdef CONFIG_PHYS_ADDR_T_64BIT
#define PA_FORM "%016llX"
#define PA_CAST(_x) ((uint64_t)(_x))
#else /* def CONFIG_PHYS_ADDR_T_64BIT */
#define PA_FORM "%08X"
#define PA_CAST(_x) ((uint32_t)(_x))
#endif /* def CONFIG_PHYS_ADDR_T_64BIT */

#define IDM_ERR_CONTROL		0x00000000
#define IDM_ERR_CONTROL__TO_ENAB	9
#define IDM_ERR_CONTROL__TO_EXP__LEN	5
#define IDM_ERR_CONTROL__TO_EXP		4
#define IDM_ERR_CONTROL__TO_IRQ		3
#define IDM_ERR_CONTROL__TO_RESET	2
#define IDM_ERR_CONTROL__ERR_IRQ	1
#define IDM_ERR_CONTROL__ERR_RESET	0
#define IDM_ERR_COMPLETE	0x00000004
#define IDM_ERR_COMPLETE__OVFL		1
#define IDM_ERR_COMPLETE__ERR		0
#define IDM_ERR_STATUS		0x00000008
#define IDM_ERR_STATUS__OVFL		2
#define IDM_ERR_STATUS__TYPE__DEC	3
#define IDM_ERR_STATUS__TYPE__TO	2
#define IDM_ERR_STATUS__TYPE__SLV	1
#define IDM_ERR_STATUS__TYPE__NONE	0
#define IDM_ERR_STATUS__TYPE__LEN	2
#define IDM_ERR_STATUS__TYPE		0
#define IDM_ERR_ADDR_LSB	0x0000000C
#define IDM_ERR_ADDR_MSB	0x00000010
#define IDM_ERR_ID		0x00000014
#define IDM_ERR_ID__VMASTER_LEN		16
#define IDM_ERR_ID__VMASTER		16
#define IDM_ERR_ID__MASTER__LEN		16
#define IDM_ERR_ID__MASTER		0
/* ????                         0x00000018 */
#define IDM_ERR_FLAGS		0x0000001C
#define IDM_ERR_FLAGS__WRITE		24
#define IDM_ERR_FLAGS__PROT__LEN	3
#define IDM_ERR_FLAGS__PROT		20
#define IDM_ERR_FLAGS__CACHE__LEN	4
#define IDM_ERR_FLAGS__CACHE		16
#define IDM_ERR_FLAGS__LOCK__LEN	2
#define IDM_ERR_FLAGS__LOCK		14
#define IDM_ERR_FLAGS__BURST__LEN	2
#define IDM_ERR_FLAGS__BURST		12
#define IDM_ERR_FLAGS__DATA_IN__LEN	3
#define IDM_ERR_FLAGS__DATA_IN		8
#define IDM_ERR_FLAGS__LEN__LEN		4
#define IDM_ERR_FLAGS__LEN		0
#define IDM_ERR_IRQ_STATUS	0x00000100
#define IDM_ERR_IRQ_STATUS__TIMEOUT	1
#define IDM_ERR_IRQ_STATUS__ERROR	0

/*
 *  Find the index of a given slave fault monitor
 */
static int idm_iproc_find_child(struct idm_slave_info *sp)
{
	struct idm_info *info = sp->info;
	int index;

	for (index = 0; index < info->slaveCount; index++) {
		if (sp == (&(info->slave[index]))) {
			/* same address, so it's the same object */
			break;
		}
	}
	if (index >= info->slaveCount) {
		/* did not find it */
		index = -1;
	}
	return index;
}

/*
 *  Check for (and report if applicable) a fault detected by an IDM slave
 *  wrapper   Also report any related anomalies (bogus interrupt or similar).
 */
#if defined(CONFIG_PHYS_ADDR_T_64BIT)
#define IICRUCIS0 "Unable to clear %s (%2d %08llX %3d) int status (%08X)\n"
#define IICRUCIS1 "Unable to clear %s (%2d %08llX ---) int status (%08X)\n"
#else /* defined(CONFIG_PHYS_ADDR_T_64BIT) */
#define IICRUCIS0 "Unable to clear %s (%2d %08X %3d) int status (%08X)\n"
#define IICRUCIS1 "Unable to clear %s (%2d %08X ---) int status (%08X)\n"
#endif /* defined(CONFIG_PHYS_ADDR_T_64BIT) */
static int idm_iproc_check_and_report(int irq,
				      struct idm_slave_info *sp,
				      int index)
{
	struct idm_info *info = sp->info;
	int faults = 0;
	int overflow;
	unsigned int type;
	u32 regval;
#if defined(CONFIG_PHYS_ADDR_T_64BIT)
	u32 regval1;
#endif /* defined(CONFIG_PHYS_ADDR_T_64BIT) */

	if (index < 0) {
		/* caller didn't provide the index; find it */
		index = idm_iproc_find_child(sp);
	}
	/* make sure it's the expected interrupt */
	if (irq != sp->interrupt) {
		dev_err(info->dev,
			"Unexpected interrupt %d for index %d (%s);"
			" expected interrupt %d\n",
			irq,
			index,
			sp->name,
			sp->interrupt);
		/* something really wrong; it's not even the right interrupt */
		goto skip;
	}
	/* make sure an interrupt actually occurred */
	regval = readl_relaxed(&(sp->base[IDM_ERR_STATUS]));
	type = ((regval >> IDM_ERR_STATUS__TYPE) &
		((1 << IDM_ERR_STATUS__TYPE__LEN) - 1));
	overflow = (0 != (regval & (1 << IDM_ERR_STATUS__OVFL)));
	if ((!overflow) && (type == IDM_ERR_STATUS__TYPE__NONE)) {
		/* this wrapper did not detect a fault */
		goto skip;
	}
	/* at this point, we have a fault we wanted to check */
	faults++;
	if (sp->interrupt >= 0) {
#if defined(CONFIG_PHYS_ADDR_T_64BIT)
		dev_crit(info->dev,
			 "%s (%2u %016llX %3d) fault\n",
			 sp->name,
			 index,
			 sp->physBase,
			 sp->interrupt);
#else /* defined(CONFIG_PHYS_ADDR_T_64BIT) */
		dev_crit(info->dev,
			 "%s (%2u %08X %3d) fault\n",
			 sp->name,
			 index,
			 sp->physBase,
			 sp->interrupt);
#endif /* defined(CONFIG_PHYS_ADDR_T_64BIT) */
	} else {
#if defined(CONFIG_PHYS_ADDR_T_64BIT)
		dev_crit(info->dev,
			 "%s (%2u %016llX ---) fault\n",
			 sp->name,
			 index,
			 sp->physBase);
#else /* defined(CONFIG_PHYS_ADDR_T_64BIT) */
		dev_crit(info->dev,
			 "%s (%2u %08X ---) fault\n",
			 sp->name,
			 index,
			 sp->physBase);
#endif /* defined(CONFIG_PHYS_ADDR_T_64BIT) */
	}
	dev_crit(info->dev,
		 "%s ERR_STAT %08X Ov%s C=%-5s\n",
		 sp->name,
		 regval,
		 overflow?"+":"-",
		 (type == IDM_ERR_STATUS__TYPE__SLV)?"Slave" :
		 (type == IDM_ERR_STATUS__TYPE__TO)?"TimeOut" :
		 (type == IDM_ERR_STATUS__TYPE__DEC)?"Decode" :
		 "None");
	regval = readl_relaxed(&(sp->base[IDM_ERR_IRQ_STATUS]));
	dev_crit(info->dev,
		 "%s INT_STAT %08X Err%s TmOut%s\n",
		 sp->name,
		 regval,
		 (regval & (1 << IDM_ERR_IRQ_STATUS__ERROR))?"+":"-",
		 (regval & (1 << IDM_ERR_IRQ_STATUS__TIMEOUT))?"+":"-");
	regval = readl_relaxed(&(sp->base[IDM_ERR_ID]));
	dev_crit(info->dev,
		 "%s MASTERID %08X M=%04X V=%04X\n",
		 sp->name,
		 regval,
		 (regval >> IDM_ERR_ID__MASTER) &
		 ((1 << IDM_ERR_ID__MASTER__LEN) - 1),
		 (regval >> IDM_ERR_ID__VMASTER) &
		 ((1 << IDM_ERR_ID__VMASTER_LEN) - 1));
#if defined(CONFIG_PHYS_ADDR_T_64BIT)
	regval = readl_relaxed(&(sp->base[IDM_ERR_ADDR_LSB]));
	regval1 = readl_relaxed(&(sp->base[IDM_ERR_ADDR_MSB]));
	dev_crit(info->dev,
		 "%s ADDRESS  %08X%08X\n",
		 sp->name,
		 regval1,
		 regval);
#else /* defined(CONFIG_PHYS_ADDR_T_64BIT) */
	regval = readl_relaxed(&(sp->base[IDM_ERR_ADDR_LSB]));
	dev_crit(info->dev,
		 "%s ADDRESS  %08X\n",
		 sp->name,
		 regval);
#endif /* defined(CONFIG_PHYS_ADDR_T_64BIT) */
	regval = readl_relaxed(&(sp->base[IDM_ERR_FLAGS]));
	dev_crit(info->dev,
		 "%s ERR_FLGS %08X %s\n",
		 sp->name,
		 regval,
		 (regval & (1 << IDM_ERR_FLAGS__WRITE))?"Write":"Read");
	dev_crit(info->dev,
		 "%s ERR_FLGS %08X P=%X C=%X Lk=%X\n",
		 sp->name,
		 regval,
		 (regval >> IDM_ERR_FLAGS__PROT) &
		 ((1 << IDM_ERR_FLAGS__PROT__LEN) - 1),
		 (regval >> IDM_ERR_FLAGS__CACHE) &
		 ((1 << IDM_ERR_FLAGS__CACHE__LEN) - 1),
		 (regval >> IDM_ERR_FLAGS__LOCK) &
		 ((1 << IDM_ERR_FLAGS__LOCK__LEN) - 1));
	dev_crit(info->dev,
		 "%s ERR_FLGS %08X B=%X D=%X Ln=%X\n",
		 sp->name,
		 regval,
		 (regval >> IDM_ERR_FLAGS__BURST) &
		 ((1 << IDM_ERR_FLAGS__BURST__LEN) - 1),
		 (regval >> IDM_ERR_FLAGS__DATA_IN) &
		 ((1 << IDM_ERR_FLAGS__DATA_IN__LEN) - 1),
		 (regval >> IDM_ERR_FLAGS__LEN) &
		 ((1 << IDM_ERR_FLAGS__LEN__LEN) - 1));
	regval = readl_relaxed(&(sp->base[IDM_ERR_CONTROL]));
	dev_crit(info->dev,
		 "%s CONTROL  %08X To%s ToExp=%02X\n",
		 sp->name,
		 regval,
		 (regval & (1 << IDM_ERR_CONTROL__TO_ENAB))?"+":"-",
		 (regval >> IDM_ERR_CONTROL__TO_EXP) &
		 ((1 << IDM_ERR_CONTROL__TO_EXP__LEN) - 1));
	dev_crit(info->dev,
		 "%s CONTROL  %08X ToI%s ToR%s\n",
		 sp->name,
		 regval,
		 (regval & (1 << IDM_ERR_CONTROL__TO_IRQ))?"+":"-",
		 (regval & (1 << IDM_ERR_CONTROL__TO_RESET))?"+":"-");
	dev_crit(info->dev,
		 "%s CONTROL  %08X ErI%s ErR%s\n",
		 sp->name,
		 regval,
		 (regval & (1 << IDM_ERR_CONTROL__ERR_IRQ))?"+":"-",
		 (regval & (1 << IDM_ERR_CONTROL__ERR_RESET))?"+":"-");
	/* ack the interrupt */
	regval = 0;
	if (type)
		regval |= (1 << IDM_ERR_COMPLETE__ERR);
	if (overflow)
		regval |= (1 << IDM_ERR_COMPLETE__OVFL);
	writel_relaxed(regval, &(sp->base[IDM_ERR_COMPLETE]));
	/* make sure the ack was written to hardware */
	regval = readl_relaxed(&(sp->base[IDM_ERR_COMPLETE]));
	/* make sure the hardware accepted it */
	regval = readl_relaxed(&(sp->base[IDM_ERR_IRQ_STATUS]));
	if (regval) {
		if (sp->interrupt >= 0) {
			dev_err(info->dev,
				IICRUCIS0,
				sp->name,
				index,
				sp->physBase,
				sp->interrupt,
				regval);
		} else {
			dev_err(info->dev,
				IICRUCIS1,
				sp->name,
				index,
				sp->physBase,
				regval);
		}
	}
skip:
	return faults;
}

static irqreturn_t idm_iproc_irq_handler(int irq, void *ptr)
{
	struct idm_slave_info *sp = (struct idm_slave_info *)ptr;
	int faults;

	faults = idm_iproc_check_and_report(irq, sp, -1);
	if (faults)
		return IRQ_HANDLED;
	else
		return IRQ_NONE;
}

/*
 *  Clean up all of the state and put the hardware in a state that allows any
 *  resets to continue normally but does not trigger interrupts.  This also
 *  releases all resources, so it should not be used carelessly.
 */
static void idm_iproc_cleanup(struct idm_info *info)
{
	struct idm_slave_info *sp;
	u32 regval;
	int index;

	if (!info)
		return;

	for (index = 0; index < info->slaveCount; index++) {
		sp = &(info->slave[index]);
		if (sp->name) {
			dev_info(info->dev,
				 "%s: disconnect driver\n",
				 sp->name);
		}
		if (sp->base) {
			/* disable interrupts */
			regval = readl_relaxed(&(sp->base[IDM_ERR_CONTROL]));
			regval &= (~(IDM_ERR_CONTROL__ERR_IRQ |
				     IDM_ERR_CONTROL__TO_IRQ));
			if (sp->interrupt >= 0) {
				dev_info(info->dev,
					 "IDM %2u CTRL %08X IRQ %3d: %s\n",
					 index,
					 regval,
					 sp->interrupt,
					 sp->name);
			} else {
				dev_info(info->dev,
					 "IDM %2u CTRL %08X: %s\n",
					 index,
					 regval,
					 sp->name);
			}
			writel_relaxed(regval,
				       &(sp->base[IDM_ERR_CONTROL]));
			/* reread just to force write commit */
			regval = readl_relaxed(&(sp->base[IDM_ERR_CONTROL]));
			/* unmap the registers */
			dev_dbg(info->dev, "  release memory mapped I/O\n");
			iounmap(sp->base);
		}
		if (sp->interrupt >= 0) {
			dev_dbg(info->dev, "  release IRQ\n");
			free_irq(sp->interrupt, sp);
		}
		dev_dbg(info->dev, "  cleanup\n");
		kfree(sp->name);
		memset(sp, 0x00, sizeof(*sp));
	}
	dev_set_drvdata(info->dev, NULL);
	kfree(info);
}

static const struct of_device_id idm_slave_iproc_of_match[] = {
	{ .compatible = "brcm,iproc-idm-monitor" },
	{ }
};
MODULE_DEVICE_TABLE(of, idm_slave_iproc_of_match);

static int idm_iproc_probe(struct platform_device *pdev)
{
	struct device_node *np;
	struct device_node *child_np;
	const struct of_device_id *match;
	struct device *dev;
	struct idm_info *info;
	struct idm_slave_info *sp;
	struct resource res;
	u32 temp;
	int slaveCount;
	int index;
	int faults;
	int ret = 0;

	/* Make sure device matches */
	dev = &(pdev->dev);
	match = of_match_device(idm_slave_iproc_of_match, &(pdev->dev));
	if (!match) {
		dev_err(dev, "no match\n");
		return -EINVAL;
	}

	/* Iterate through the child nodes for the parameters */
	np = dev->of_node;
	slaveCount = of_get_available_child_count(np);
	dev_dbg(dev,
		"has %d child node%s\n",
		slaveCount,
		(slaveCount == 1)?"":"s");
	dev_dbg(dev,
		"allocate %u bytes for descriptor\n",
		(unsigned int)(sizeof(*info) +
			       (sizeof(info->slave[0]) *
				slaveCount)));
	info = kzalloc(sizeof(*info) + (sizeof(info->slave[0]) * slaveCount),
		       GFP_KERNEL);
	if (IS_ERR_OR_NULL(info)) {
		dev_err(dev,
			"unable to allocate %u bytes for descriptor\n",
			(unsigned int)(sizeof(*info) +
				       (sizeof(info->slave[0]) *
					slaveCount)));
		info = NULL;
		ret = -ENOMEM;
		goto abort;
	}
	info->slaveCount = slaveCount;
	info->dev = dev;
	for (index = 0; index < slaveCount; index++)
		info->slave[index].interrupt = -1;
	index = 0;
	for_each_available_child_of_node(np, child_np) {
		sp = &(info->slave[index]);
		sp->info = info;
		temp = 6 + strlen(child_np->name);
		sp->name = kzalloc(temp, GFP_KERNEL);
		if (IS_ERR_OR_NULL(sp->name)) {
			dev_err(dev,
				"%s: Unable to alloc space for name\n",
				child_np->name);
			ret = -ENOMEM;
			sp->name = NULL;
			goto abort;
		} else {
			snprintf(sp->name, temp - 1, "idm_%s", child_np->name);
			dev_dbg(dev,
				"IDM slave wrapper %d: %s\n",
				index,
				sp->name);
		}
		if (of_property_read_bool(child_np, "timeout_irq"))
			sp->flags |= IDM_TIMEOUT_INTERRUPT;
		if (of_property_read_bool(child_np, "timeout_reset"))
			sp->flags |= IDM_TIMEOUT_RESET;
		if (of_property_read_bool(child_np, "error_irq"))
			sp->flags |= IDM_ERROR_INTERRUPT;
		if (of_property_read_bool(child_np, "error_reset"))
			sp->flags |= IDM_ERROR_RESET;
		dev_dbg(dev, "  on error%s%s\n",
			(sp->flags & IDM_ERROR_INTERRUPT)?", IRQ":"",
			(sp->flags & IDM_ERROR_RESET)?", reset slave":"");
		dev_dbg(dev, "  on timeout%s%s\n",
			(sp->flags & IDM_TIMEOUT_INTERRUPT)?", IRQ":"",
			(sp->flags & IDM_TIMEOUT_RESET)?", reset slave":"");
		if (sp->flags & (IDM_TIMEOUT_INTERRUPT | IDM_TIMEOUT_RESET)) {
			if (!of_property_read_u32(child_np,
						  "timeout_exp",
						  &temp)) {
				if (temp < 4)
					temp = 4;
				if (temp > 31)
					temp = 31;
				sp->timeout_exp = temp;
			} else {
				sp->timeout_exp = 31;
			}
			dev_dbg(dev,
				"  timeout %u AXI cycles\n",
				1 << sp->timeout_exp);
		}
		sp->base = of_iomap(child_np, 0);
		if (IS_ERR_OR_NULL(sp->base)) {
			dev_err(dev,
				"%s: unable to map registers: %ld\n",
				sp->name,
				PTR_ERR(sp->base));
			sp->base = NULL;
			ret = -ENOMEM;
			goto abort;
		}
		dev_dbg(dev, "  virtAddr = %p\n", sp->base);
		ret = of_address_to_resource(child_np, 0, &res);
		if (!ret) {
			sp->physBase = res.start;
			sp->physEnd = res.end;
			dev_dbg(dev,
				"  physAddr = " PA_FORM ".." PA_FORM "\n",
				PA_CAST(sp->physBase),
				PA_CAST(sp->physEnd));
		}
		dev_dbg(dev,
			"  current control = %08X\n",
			readl_relaxed(&(sp->base[IDM_ERR_CONTROL])));
		dev_dbg(dev,
			"  current status = %08X\n",
			readl_relaxed(&(sp->base[IDM_ERR_STATUS])));
		/* but that was only informational */
		ret = 0;
		sp->interrupt = irq_of_parse_and_map(child_np, 0);
		if (sp->interrupt <= 0) {
			/* don't use interrupt */
			dev_dbg(dev, "  no interrupt\n");
			sp->interrupt = -1;
		} else {
			dev_dbg(dev, "  interrupt %d\n", sp->interrupt);
			ret = request_irq(sp->interrupt,
					  &idm_iproc_irq_handler,
					  IRQF_SHARED,
					  sp->name,
					  sp);
			if (ret) {
				dev_err(dev,
					"%s: unable to acquire interrupt %d: %d\n",
					sp->name,
					sp->interrupt,
					ret);
				sp->interrupt = -1;
			}
			/* but let it slide for now */
			ret = 0;
		}
		if (sp->interrupt < 0) {
			sp->flags &= (~(IDM_ERROR_INTERRUPT |
					IDM_TIMEOUT_INTERRUPT));
			dev_err(dev,
				"%s: polled operation not supported\n",
				sp->name);
			if (0 == (sp->flags & IDM_TIMEOUT_RESET)) {
				dev_err(dev,
					"%s: no action will be taken on timeout\n",
					sp->name);
			}
			if (0 == (sp->flags & IDM_ERROR_RESET)) {
				dev_err(dev,
					"%s: no action will be taken on other errors\n",
					sp->name);
			}
		}
		index++;
	}
	dev_set_drvdata(dev, info);
	dev_info(dev, "Check pending faults\n");
	for (index = 0, faults = 0; index < slaveCount; index++) {
		sp = &(info->slave[index]);
		faults += idm_iproc_check_and_report(sp->interrupt,
						     sp,
						     index);
	}
	dev_info(dev, "Enable actions to be taken on faults\n");
	for (index = 0, faults = 0; index < slaveCount; index++) {
		sp = &(info->slave[index]);
		/* encode the configuration */
		temp = 0;
		if (sp->flags & IDM_ERROR_RESET)
			temp |= (1 << IDM_ERR_CONTROL__ERR_RESET);
		if (sp->flags & IDM_ERROR_INTERRUPT)
			temp |= (1 << IDM_ERR_CONTROL__ERR_IRQ);
		if (sp->flags & IDM_TIMEOUT_RESET)
			temp |= (1 << IDM_ERR_CONTROL__TO_RESET);
		if (sp->flags & IDM_TIMEOUT_INTERRUPT)
			temp |= (1 << IDM_ERR_CONTROL__TO_IRQ);
		if (sp->flags & ((1 << IDM_ERR_CONTROL__TO_RESET) |
				 (1 << IDM_ERR_CONTROL__TO_IRQ))) {
			temp |= ((1 << IDM_ERR_CONTROL__TO_ENAB) |
				 ((sp->timeout_exp &
				   ((1 << IDM_ERR_CONTROL__TO_EXP__LEN) - 1))
				  << IDM_ERR_CONTROL__TO_EXP));
		}
		if (sp->interrupt >= 0) {
			dev_info(info->dev,
				 "IDM %2u CTRL %08X IRQ %3d: %s\n",
				 index,
				 temp,
				 sp->interrupt,
				 sp->name);
		} else {
			dev_info(info->dev,
				 "IDM %2u CTRL %08X: %s\n",
				 index,
				 temp,
				 sp->name);
		}
		/* write the configuration */
		writel_relaxed(temp, &(sp->base[IDM_ERR_CONTROL]));
		/* reread just to force write commit */
		temp = readl_relaxed(&(sp->base[IDM_ERR_CONTROL]));
	}

abort:
	if (ret) {
		dev_err(dev, "FATAL: init aborted, %d\n", ret);
		/* clean up everything */
		idm_iproc_cleanup(info);
	}
	return ret;
}

static int idm_iproc_remove(struct platform_device *pdev)
{
	struct idm_info *info;

	info = dev_get_drvdata(&(pdev->dev));
	if (!IS_ERR_OR_NULL(info)) {
		idm_iproc_cleanup(info);
		return 0;
	} else {
		return -EFAULT;
	}
}

static struct platform_driver iproc_idm_driver = {
	.driver = {
		.name = "iproc-idm",
		.of_match_table = idm_slave_iproc_of_match,
	},
	.probe = idm_iproc_probe,
	.remove = idm_iproc_remove,
};
module_platform_driver(iproc_idm_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("IPROC IDM wrapper fault handler");
MODULE_LICENSE("GPL v2");
