/*
 * Copyright (C) 2014 Broadcom Corporation
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

/*
 * iProc SDHCI platform driver
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/acpi.h>
#include "sdhci-pltfm.h"

struct sdhci_iproc_data {
	const struct sdhci_pltfm_data *pdata;
	u32 caps;
	u32 caps1;
	u32 mmc_caps;
};

struct sdhci_iproc_host {
	const struct sdhci_iproc_data *data;
	u32 shadow_cmd;
	bool is_cmd_shadowed;
	u32 shadow_blk;
	void __iomem *idm_io_control;
};

#define REG_OFFSET_IN_BITS(reg) ((reg) << 3 & 0x18)
/* NS2 specific #defines */

#define SDIO_IDM_IO_CONTROL_DIRECT_OFFSET 0x0
#define SDIO_IDM0_IO_CONTROL_DIRECT__ARCACHE_R 27
#define SDIO_IDM0_IO_CONTROL_DIRECT__AWCACHE_R 23
#define SDIO_IDM0_IO_CONTROL_DIRECT__CMD_COMFLICT_DISABLE_R 22
#define SDIO_IDM0_IO_CONTROL_DIRECT__FEEDBACK_CLK_EN_R 21
#define SDIO_IDM0_IO_CONTROL_DIRECT__TUNING_CMD_SUCCESS_CNT_R 17
#define SDIO_IDM0_IO_CONTROL_DIRECT__OP_DELAY_CTRL_R 15
#define SDIO_IDM0_IO_CONTROL_DIRECT__OP_TAP_DELAY_R 11
#define SDIO_IDM0_IO_CONTROL_DIRECT__OP_TAP_EN_R 10
#define SDIO_IDM0_IO_CONTROL_DIRECT__IP_DELAY_CTRL_R 8
#define SDIO_IDM0_IO_CONTROL_DIRECT__IP_TAP_EN_R 7
#define SDIO_IDM0_IO_CONTROL_DIRECT__IP_TAP_DELAY_R 1
#define SDIO_IDM0_IO_CONTROL_DIRECT__clk_enable_R 0
#define SDIO_IDM_RESET_CONTROL_OFFSET 0x3F8
/* NS2 specific #defines */

static inline u32 sdhci_iproc_readl(struct sdhci_host *host, int reg)
{
	u32 val = readl(host->ioaddr + reg);

	pr_debug("%s: readl [0x%02x] 0x%08x\n",
		 mmc_hostname(host->mmc), reg, val);
	return val;
}

static u16 sdhci_iproc_readw(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_iproc_host *iproc_host = sdhci_pltfm_priv(pltfm_host);
	u32 val, word;

	val = iproc_host->is_cmd_shadowed ? iproc_host->shadow_cmd :
			sdhci_iproc_readl(host, (reg & ~3));

	word = val >> REG_OFFSET_IN_BITS(reg) & 0xffff;

	return word;
}

static u8 sdhci_iproc_readb(struct sdhci_host *host, int reg)
{
	u32 val = sdhci_iproc_readl(host, (reg & ~3));
	u8 byte = val >> REG_OFFSET_IN_BITS(reg) & 0xff;
	return byte;
}

static inline void sdhci_iproc_writel(struct sdhci_host *host, u32 val, int reg)
{
	pr_debug("%s: writel [0x%02x] 0x%08x\n",
		 mmc_hostname(host->mmc), reg, val);

	writel(val, host->ioaddr + reg);

	if (host->clock <= 400000) {
		/* Round up to micro-second four SD clock delay */
		if (host->clock)
			udelay((4 * 1000000 + host->clock - 1) / host->clock);
		else
			udelay(10);
	}
}

/*
 * The Arasan has a bugette whereby it may lose the content of successive
 * writes to the same register that are within two SD-card clock cycles of
 * each other (a clock domain crossing problem). The data
 * register does not have this problem, which is just as well - otherwise we'd
 * have to nobble the DMA engine too.
 *
 * This wouldn't be a problem with the code except that we can only write the
 * controller with 32-bit writes.  So two different 16-bit registers are
 * written back to back creates the problem.
 *
 * In reality, this only happens when SDHCI_BLOCK_SIZE and SDHCI_BLOCK_COUNT
 * are written followed by SDHCI_TRANSFER_MODE and SDHCI_COMMAND.
 * The BLOCK_SIZE and BLOCK_COUNT are meaningless until a command issued so
 * the work around can be further optimized. We can keep shadow values of
 * BLOCK_SIZE, BLOCK_COUNT, and TRANSFER_MODE until a COMMAND is issued.
 * Then, write the BLOCK_SIZE+BLOCK_COUNT in a single 32-bit write followed
 * by the TRANSFER+COMMAND in another 32-bit write.
 */
static void sdhci_iproc_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_iproc_host *iproc_host = sdhci_pltfm_priv(pltfm_host);
	u32 word_shift = REG_OFFSET_IN_BITS(reg);
	u32 mask = 0xffff << word_shift;
	u32 oldval, newval;

	if (reg == SDHCI_COMMAND) {
		/* Write the block now as we are issuing a command */
		if (iproc_host->shadow_blk != 0) {
			sdhci_iproc_writel(host, iproc_host->shadow_blk,
				SDHCI_BLOCK_SIZE);
			iproc_host->shadow_blk = 0;
		}
		oldval = iproc_host->shadow_cmd;
	} else if (reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) {
		/* Block size and count are stored in shadow reg */
		oldval = iproc_host->shadow_blk;
	} else {
		/* Read reg, all other registers are not shadowed */
		oldval = sdhci_iproc_readl(host, (reg & ~3));
	}
	newval = (oldval & ~mask) | (val << word_shift);

	if (reg == SDHCI_TRANSFER_MODE) {
		/* Save the transfer mode until the command is issued */
		iproc_host->shadow_cmd = newval;
		iproc_host->is_cmd_shadowed = true;
	} else if (reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) {
		/* Save the block info until the command is issued */
		iproc_host->shadow_blk = newval;
	} else {
		iproc_host->is_cmd_shadowed = false;
		/* Command or other regular 32-bit write */
		sdhci_iproc_writel(host, newval, reg & ~3);
	}
}

static void sdhci_iproc_writeb(struct sdhci_host *host, u8 val, int reg)
{
	u32 oldval = sdhci_iproc_readl(host, (reg & ~3));
	u32 byte_shift = REG_OFFSET_IN_BITS(reg);
	u32 mask = 0xff << byte_shift;
	u32 newval = (oldval & ~mask) | (val << byte_shift);

	sdhci_iproc_writel(host, newval, reg & ~3);
}

static const struct sdhci_ops sdhci_iproc_cygnus_ops = {
	.read_l = sdhci_iproc_readl,
	.read_w = sdhci_iproc_readw,
	.read_b = sdhci_iproc_readb,
	.write_l = sdhci_iproc_writel,
	.write_w = sdhci_iproc_writew,
	.write_b = sdhci_iproc_writeb,
	.set_clock = sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static const struct sdhci_ops sdhci_iproc_ns2_ops = {
	.read_l = sdhci_iproc_readl,
	.read_w = sdhci_iproc_readw,
	.read_b = sdhci_iproc_readb,
	.write_l = sdhci_iproc_writel,
	.write_w = sdhci_iproc_writew,
	.write_b = sdhci_iproc_writeb,
	.set_clock = sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data sdhci_iproc_cygnus_pltfm_data = {
	.quirks = SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK,
	.quirks2 = SDHCI_QUIRK2_ACMD23_BROKEN,
	.ops = &sdhci_iproc_cygnus_ops,
};

static const struct sdhci_pltfm_data sdhci_iproc_ns2_pltfm_data = {
	.quirks = SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12 |
		SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK,
	.quirks2 = SDHCI_QUIRK2_ACMD23_BROKEN |
		SDHCI_QUIRK2_NEED_DELAY_AFTER_INT_CLK_RST,
	.ops = &sdhci_iproc_ns2_ops,
};

static const struct sdhci_iproc_data iproc_cygnus_data = {
	.pdata = &sdhci_iproc_cygnus_pltfm_data,
	.caps = 0x05E90000,
	.caps1 = 0x00000064,
};

static const struct sdhci_iproc_data iproc_ns2_data = {
	.pdata = &sdhci_iproc_ns2_pltfm_data,
	.caps = 0x05E90000,
	.caps1 = 0x00000064,
};

static const struct sdhci_ops sdhci_iproc_ops = {
	.read_l = sdhci_iproc_readl,
	.read_w = sdhci_iproc_readw,
	.read_b = sdhci_iproc_readb,
	.write_l = sdhci_iproc_writel,
	.write_w = sdhci_iproc_writew,
	.write_b = sdhci_iproc_writeb,
	.set_clock = sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data sdhci_iproc_pltfm_data = {
	.quirks = SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12 |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK,
	.quirks2 = SDHCI_QUIRK2_ACMD23_BROKEN |
		SDHCI_QUIRK2_NEED_DELAY_AFTER_INT_CLK_RST,
	.ops = &sdhci_iproc_ops,
};

static const struct sdhci_iproc_data iproc_data = {
	.pdata = &sdhci_iproc_pltfm_data,
	.caps = 0x05E90000,
	.caps1 = 0x00000064,
	.mmc_caps = MMC_CAP_1_8V_DDR,
};

static const struct sdhci_pltfm_data sdhci_bcm2835_pltfm_data = {
	.quirks = SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_MISSING_CAPS,
	.ops = &sdhci_iproc_ops,
};

static const struct sdhci_iproc_data bcm2835_data = {
	.pdata = &sdhci_bcm2835_pltfm_data,
	.caps = SDHCI_CAN_VDD_330,
	.caps1 = 0x00000000,
	.mmc_caps = 0x00000000,
};

static const struct of_device_id sdhci_iproc_of_match[] = {
	{ .compatible = "brcm,bcm2835-sdhci", .data = &bcm2835_data },
	{ .compatible = "brcm,sdhci-iproc-cygnus", .data = &iproc_cygnus_data },
	{ .compatible = "brcm,sdhci-iproc-ns2", .data = &iproc_ns2_data },
	{ .compatible = "brcm,sdhci-iproc", .data = &iproc_data },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_iproc_of_match);

#ifdef CONFIG_ACPI
enum iproc_sdhci_type {
	SDHCI_BCM2835,
	SDHCI_CYGNUS,
	SDHCI_NS2,
	SDHCI_IPROC,
	LAST_SDHCI_TYPE /* always last */
};

static const struct sdhci_iproc_data *sdhci_iproc_acpi_data[LAST_SDHCI_TYPE] = {
	&bcm2835_data,
	&iproc_cygnus_data,
	&iproc_ns2_data,
	&iproc_data,
};

static const struct acpi_device_id sdhci_iproc_acpi_ids[] = {
	{ .id = "BRCM2835", .driver_data =  SDHCI_BCM2835},
	{ .id = "BRCM5830", .driver_data =  SDHCI_CYGNUS},
	{ .id = "BRCM5871", .driver_data =  SDHCI_NS2},
	{ .id = "BRCM0000", .driver_data =  SDHCI_IPROC},
	{ },
};
MODULE_DEVICE_TABLE(acpi, sdhci_iproc_acpi_ids);
#endif /* CONFIG_ACPI */

/* Initial value for the MMC/SD IDM_IO_CONTROL_DIRECT registers */
/* We use the existing setting for AxCACHE, so not specified here */
#define SDIO_IDM_IO_CONTROL_DIRECT_SETTING \
	((0 << SDIO_IDM0_IO_CONTROL_DIRECT__CMD_COMFLICT_DISABLE_R) | \
	 (1 << SDIO_IDM0_IO_CONTROL_DIRECT__FEEDBACK_CLK_EN_R) |  \
	 (0 << SDIO_IDM0_IO_CONTROL_DIRECT__TUNING_CMD_SUCCESS_CNT_R) | \
	 (0 << SDIO_IDM0_IO_CONTROL_DIRECT__OP_DELAY_CTRL_R) | \
	 (0xB << SDIO_IDM0_IO_CONTROL_DIRECT__OP_TAP_DELAY_R) | \
	 (1 << SDIO_IDM0_IO_CONTROL_DIRECT__OP_TAP_EN_R) | \
	 (0 << SDIO_IDM0_IO_CONTROL_DIRECT__IP_DELAY_CTRL_R) | \
	 (1 << SDIO_IDM0_IO_CONTROL_DIRECT__IP_TAP_EN_R) | \
	 (0x02 << SDIO_IDM0_IO_CONTROL_DIRECT__IP_TAP_DELAY_R) | \
	 (1 << SDIO_IDM0_IO_CONTROL_DIRECT__clk_enable_R))

static int sdhci_iproc_of_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct sdhci_iproc_data *iproc_data;
	struct sdhci_host *host;
	struct sdhci_iproc_host *iproc_host;
	struct sdhci_pltfm_host *pltfm_host;
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 reg;

	match = of_match_device(sdhci_iproc_of_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	iproc_data = match->data;

	host = sdhci_pltfm_init(pdev, iproc_data->pdata, sizeof(*iproc_host));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	iproc_host = sdhci_pltfm_priv(pltfm_host);

	iproc_host->data = iproc_data;
	iproc_host->is_cmd_shadowed = false;

	mmc_of_parse(host->mmc);
	sdhci_get_of_property(pdev);

	host->mmc->caps |= iproc_host->data->mmc_caps;

	pltfm_host->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pltfm_host->clk)) {
		ret = PTR_ERR(pltfm_host->clk);
		goto err;
	}
	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable host clk\n");
		goto err;
	}

	if (iproc_host->data->pdata->quirks & SDHCI_QUIRK_MISSING_CAPS) {
		host->caps = iproc_host->data->caps;
		host->caps1 = iproc_host->data->caps1;
	}

	if (of_device_is_compatible(np, "brcm,sdhci-iproc-ns2")) {
		/* IDM reset (for NS2) */
		iproc_host->idm_io_control = of_iomap(np, 1);
		if (IS_ERR_OR_NULL(iproc_host->idm_io_control)) {
			dev_err(dev,
				"of_iomap failed for idm_io_control, %d\n",
				ret);
			ret = -ENOMEM;
			goto err;
		}

		/* preserve AxCACHE while enabling */
		reg = readl(iproc_host->idm_io_control +
				   SDIO_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg &= ((0xF << SDIO_IDM0_IO_CONTROL_DIRECT__ARCACHE_R) |
			(0xF << SDIO_IDM0_IO_CONTROL_DIRECT__AWCACHE_R));
		writel(SDIO_IDM_IO_CONTROL_DIRECT_SETTING | reg,
		       iproc_host->idm_io_control +
		       SDIO_IDM_IO_CONTROL_DIRECT_OFFSET);

		/* Reset SDIO device */
		writel(0x1, iproc_host->idm_io_control +
			    SDIO_IDM_RESET_CONTROL_OFFSET);
		mdelay(100);

		/* Release reset */
		writel(0x0, iproc_host->idm_io_control +
			    SDIO_IDM_RESET_CONTROL_OFFSET);
		mdelay(100);
	}

	ret = sdhci_add_host(host);
	if (ret)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
err:
	sdhci_pltfm_free(pdev);
	return ret;
}

#ifdef CONFIG_ACPI
static int sdhci_iproc_acpi_probe(struct platform_device *pdev)
{
	const struct sdhci_iproc_data *iproc_data;
	const struct acpi_device_id *acpi_id;
	struct sdhci_host *host;
	struct sdhci_iproc_host *iproc_host;
	struct sdhci_pltfm_host *pltfm_host;
	struct device *dev = &pdev->dev;
	acpi_handle handle = ACPI_HANDLE(dev);
	struct acpi_device *device;
	struct resource *iomem;
	const char *hid;
	const char *uid;
	int ret;
	u32 reg;
	u32 bus_width;

	if (acpi_bus_get_device(handle, &device))
		return -ENODEV;

	if (acpi_bus_get_status(device) || !device->status.present)
		return -ENODEV;

	hid = acpi_device_hid(device);
	uid = device->pnp.unique_id;

	acpi_id = acpi_match_device(ACPI_PTR(sdhci_iproc_acpi_ids), &pdev->dev);
	if (!acpi_id)
		return -ENODEV;
	iproc_data = sdhci_iproc_acpi_data[acpi_id->driver_data];

	host = sdhci_pltfm_init(pdev, iproc_data->pdata, sizeof(*iproc_host));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	iproc_host = sdhci_pltfm_priv(pltfm_host);

	iproc_host->data = iproc_data;
	iproc_host->is_cmd_shadowed = false;

	/* Parse ACPI device property for host->mmc->caps and host->caps
	 * Given lack of ACPI function similar to mmc_of_parse and
	 * sdhci_get_of_property, we support bus-width only for now
	 */
	if (device_property_read_u32(&pdev->dev, "bus-width", &bus_width) < 0) {
		dev_dbg(&pdev->dev,
			"\"bus-width\" property is missing, assuming 1 bit.\n");
		bus_width = 1;
	}
	switch (bus_width) {
	case 8:
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;
		/* Hosts capable of 8-bit transfers can also do 4 bits */
	case 4:
		host->mmc->caps |= MMC_CAP_4_BIT_DATA;
		break;
	case 1:
		break;
	default:
		dev_err(&pdev->dev,
			"Invalid \"bus-width\" value %u!\n", bus_width);
		return -EINVAL;
	}

	host->mmc->caps |= iproc_host->data->mmc_caps;

	if (iproc_host->data->pdata->quirks & SDHCI_QUIRK_MISSING_CAPS) {
		host->caps = iproc_host->data->caps;
		host->caps1 = iproc_host->data->caps1;
	}

	/* Platform specific code during probe goes here */
	if (hid && !strcmp(hid, "BRCMNS2")) {
		/* IDM reset (for NS2) */
		iomem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!iomem) {
			dev_err(&pdev->dev, "no idm registers defined\n");
			return -EINVAL;
		}

		iproc_host->idm_io_control =
			devm_ioremap(&pdev->dev, iomem->start,
				     resource_size(iomem));
		if (IS_ERR_OR_NULL(iproc_host->idm_io_control)) {
			dev_err(&pdev->dev,
				"iomap failed for idm_io_control\n");
			return -ENOMEM;
		}

		/* preserve AxCACHE while enabling */
		reg = readl(iproc_host->idm_io_control +
			    SDIO_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg &= ((0xF << SDIO_IDM0_IO_CONTROL_DIRECT__ARCACHE_R) |
			(0xF << SDIO_IDM0_IO_CONTROL_DIRECT__AWCACHE_R));
		writel(SDIO_IDM_IO_CONTROL_DIRECT_SETTING | reg,
		       iproc_host->idm_io_control +
		       SDIO_IDM_IO_CONTROL_DIRECT_OFFSET);

		/* Reset SDIO device */
		writel(0x1, iproc_host->idm_io_control +
			    SDIO_IDM_RESET_CONTROL_OFFSET);
		mdelay(100);

		/* Release reset */
		writel(0x0, iproc_host->idm_io_control +
			    SDIO_IDM_RESET_CONTROL_OFFSET);
		mdelay(100);
	}

	ret = sdhci_add_host(host);
	if (!ret)
		return 0;

	sdhci_pltfm_free(pdev);
	return ret;
}
#else /* CONFIG_ACPI */
static int sdhci_iproc_acpi_probe(struct platform_device *pdev) {}
#endif  /* CONFIG_ACPI */

static int sdhci_iproc_probe(struct platform_device *pdev)
{
	if (pdev->dev.of_node)
		return sdhci_iproc_of_probe(pdev);

	return sdhci_iproc_acpi_probe(pdev);
}

static struct platform_driver sdhci_iproc_driver = {
	.driver = {
		.name = "sdhci-iproc",
		.of_match_table = sdhci_iproc_of_match,
		.acpi_match_table	= ACPI_PTR(sdhci_iproc_acpi_ids),
		.pm = SDHCI_PLTFM_PMOPS,
	},
	.probe = sdhci_iproc_probe,
	.remove = sdhci_pltfm_unregister,
};
module_platform_driver(sdhci_iproc_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("IPROC SDHCI driver");
MODULE_LICENSE("GPL v2");
