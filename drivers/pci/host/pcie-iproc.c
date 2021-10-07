/*
 * Copyright (C) 2014 Hauke Mehrtens <hauke@hauke-m.de>
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
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mbus.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/log2.h>
#include <linux/timer.h>

#include "pcie-iproc.h"

#define INVALID_CFG_RD 0xFFFFFFFF
#define INTR_EN_OFFSET 0xF30
#define INTR_CLEAR_OFFSET 0xF34
#define INTR_STATUS_OFFSET 0xF38
#define APB_TIMEOUT_INTR_SHIFT 11
#define APB_TIMEOUT_INTR BIT(APB_TIMEOUT_INTR_SHIFT)
#define CMPL_TIMEOUT_INTR_SHIFT 6
#define CMPL_TIMEOUT_INTR BIT(CMPL_TIMEOUT_INTR_SHIFT)

#define IDM_RESET_CONTROL_OFFSET 0x0
#define IDM_ERROR_LOG_CONTROL_OFFSET 0x100
#define IDM_IDM_ERROR_LOG_STATUS 0x108
#define TIMEOUT_ENABLE_SHIFT 9
#define TIMEOUT_ENABLE BIT(TIMEOUT_ENABLE_SHIFT)
#define TIMEOUT_EXPONENT_SHIFT 4
#define TIMEOUT_EXPONENT_MASK 0x1F0
#define TIMEOUT_EXPONENT(exp_value) (((exp_value) << TIMEOUT_EXPONENT_SHIFT) & \
				      TIMEOUT_EXPONENT_MASK)
#define TIMEOUT_INTERRUPT_SHIFT 3
#define TIMEOUT_INTERRUPT BIT(TIMEOUT_INTERRUPT_SHIFT)
#define BUS_ERROR_INTERRUPT_SHIFT 1
#define BUS_ERROR_INTERRUPT BIT(BUS_ERROR_INTERRUPT_SHIFT)

#define IDM_ERROR_LOG_COMPLETE_OFFSET 0x104
#define OVERFLOW_COMPLETE_SHIFT 1
#define OVERFLOW_COMPLETE BIT(OVERFLOW_COMPLETE_SHIFT)
#define ERROR_COMPLETE_SHIFT 0
#define ERROR_COMPLETE BIT(ERROR_COMPLETE_SHIFT)

#define PCI_STATUS_COMMAND 0x4
#define SERR_ENA_SHIFT 8
#define SERR_ENA BIT(PERR_ENA_SHIFT)
#define PERR_ENA_SHIFT 6
#define PERR_ENA BIT(PERR_ENA_SHIFT)
#define BUS_MASTER_SHIFT 2
#define BUS_MASTER BIT(BUS_MASTER_SHIFT)
#define MEM_SPACE_SHIFT 1
#define MEM_SPACE BIT(MEM_SPACE_SHIFT)

#define EP_PERST_SOURCE_SELECT_SHIFT 2
#define EP_PERST_SOURCE_SELECT       BIT(EP_PERST_SOURCE_SELECT_SHIFT)
#define EP_MODE_SURVIVE_PERST_SHIFT  1
#define EP_MODE_SURVIVE_PERST        BIT(EP_MODE_SURVIVE_PERST_SHIFT)
#define RC_PCIE_RST_OUTPUT_SHIFT     0
#define RC_PCIE_RST_OUTPUT           BIT(RC_PCIE_RST_OUTPUT_SHIFT)
#define PAXC_RESET_MASK              0x7f

#define GIC_V3_CFG_SHIFT             0
#define GIC_V3_CFG                   BIT(GIC_V3_CFG_SHIFT)

#define MSI_ENABLE_CFG_SHIFT         0
#define MSI_ENABLE_CFG               BIT(MSI_ENABLE_CFG_SHIFT)

#define CFG_IND_ADDR_MASK            0x00001ffc

#define CFG_DATA_OFFSET              0x1FC
#define CFG_ADDR_OFFSET              0x1F8
#define CFG_ADDR_BUS_NUM_SHIFT       20
#define CFG_ADDR_BUS_NUM_MASK        0x0ff00000
#define CFG_ADDR_DEV_NUM_SHIFT       15
#define CFG_ADDR_DEV_NUM_MASK        0x000f8000
#define CFG_ADDR_FUNC_NUM_SHIFT      12
#define CFG_ADDR_FUNC_NUM_MASK       0x00007000
#define CFG_ADDR_REG_NUM_SHIFT       2
#define CFG_ADDR_REG_NUM_MASK        0x00000ffc
#define CFG_ADDR_CFG_TYPE_SHIFT      0
#define CFG_ADDR_CFG_TYPE_MASK       0x00000003

#define SYS_RC_INTX_MASK             0xf

#define PCIE_PHYLINKUP_SHIFT         3
#define PCIE_PHYLINKUP               BIT(PCIE_PHYLINKUP_SHIFT)
#define PCIE_DL_ACTIVE_SHIFT         2
#define PCIE_DL_ACTIVE               BIT(PCIE_DL_ACTIVE_SHIFT)

#define APB_ERR_EN_SHIFT             0
#define APB_ERR_EN                   BIT(APB_ERR_EN_SHIFT)

#define OARR_VALID_SHIFT             0
#define OARR_VALID                   BIT(OARR_VALID_SHIFT)
#define OARR_SIZE_CFG_SHIFT          1
#define OARR_SIZE_CFG                BIT(OARR_SIZE_CFG_SHIFT)

#define MAX_NUM_OB_WINDOWS           2
#define MAX_NUM_PAXC_PF              4

#define IB_IMAP_VALID                0x1
#define IB_IMAP_MAX                  8

#define IARR_0_WINDOW_MASK           0xfffff000
#define IARR_SIZE_CFG_SHIFT          0
#define IARR_SIZE_CFG                BIT(IARR_SIZE_CFG_SHIFT)

#define IPROC_PCIE_REG_INVALID       0xffff

#define PAXB_OARR_OFFSET             0x8
#define PAXB_OARR_V2_OFFSET          0x90
#define PAXB_IARR_V2_OFFSET          0x68
#define PAXB_IARR3_OFFSET            0xf0
#define PAXB_IMAP3_OFFSET            0x148
#define PAXB_IARR_V2_OFFSET          0x68
/*
 * get the offset to IMAP_LO and IMAP_HI
 * applicable to only IMAP_3 and IMAP_4.
 */
#define PAXB_IMAP_LO_OFFSET(window, which_imap) \
	(PAXB_IMAP3_OFFSET + ((window - 1) * \
	PAXB_IARR_V2_OFFSET) + (which_imap * IB_IMAP_MAX))

#define PAXB_IMAP_HI_OFFSET(window, which_imap) \
	(PAXB_IMAP3_OFFSET + ((window - 1) * \
	 PAXB_IARR_V2_OFFSET) + (which_imap * IB_IMAP_MAX) + 4)

#define REG_LINK_CAPABILITY	0x4dc
#define MAX_LINK_WIDTH_SHIFT	4
#define MAX_LINK_WIDTH_MASK	0x1f0
#define MAX_LINK_WIDTH(lanes)	(((lanes) << MAX_LINK_WIDTH_SHIFT) & \
				 MAX_LINK_WIDTH_MASK)

/*
 * iProc PCIe host registers
 */
enum iproc_pcie_reg {
	/* clock/reset signal control */
	IPROC_PCIE_CLK_CTRL = 0,

	/*
	 * To allow MSI to be steered to an external MSI controller (e.g., ARM
	 * GICv3 ITS). Only available in PAXC v2
	 */
	IPROC_PCIE_MSI_GIC_MODE,

	/*
	 * IPROC_PCIE_MSI_BASE_ADDR and IPROC_PCIE_MSI_WINDOW_SIZE define the
	 * window where the MSI posted writes are written, for the writes to be
	 * interpreted as MSI writes. Only available in PAXC v2
	 */
	IPROC_PCIE_MSI_BASE_ADDR,
	IPROC_PCIE_MSI_WINDOW_SIZE,

	/*
	 * To hold the address of the register where the MSI writes are
	 * programed. When ARM GICv3 ITS is used, this should be programmed
	 * with the address of the GITS_TRANSLATER register. Only available in
	 * PAXC v2
	 */
	IPROC_PCIE_MSI_ADDR_LO,
	IPROC_PCIE_MSI_ADDR_HI,

	/* enable MSI. Only available in PAXC v2 */
	IPROC_PCIE_MSI_EN_CFG,

	/* allow access to root complex configuration space */
	IPROC_PCIE_CFG_IND_ADDR,
	IPROC_PCIE_CFG_IND_DATA,

	/* allow access to device configuration space */
	IPROC_PCIE_CFG_ADDR,
	IPROC_PCIE_CFG_DATA,

	/* enable INTx. Only available in PAXB */
	IPROC_PCIE_INTX_EN,

	/* outbound address mapping */
	IPROC_PCIE_OARR_LO,
	IPROC_PCIE_OARR_HI,
	IPROC_PCIE_OMAP_LO,
	IPROC_PCIE_OMAP_HI,

	/* gic_its mapping. */
	IPROC_PCIE_IARR0_LO,
	IPROC_PCIE_IARR0_HI,
	IPROC_PCIE_IMAP0_LO,
	IPROC_PCIE_IMAP0_HI,

	/* inbound address mapping */
	IPROC_PCIE_IARR_LO,
	IPROC_PCIE_IARR_HI,
	IPROC_PCIE_IMAP_LO,
	IPROC_PCIE_IMAP_HI,

	/* link status. Only available in PAXB */
	IPROC_PCIE_LINK_STATUS,

	/* enable APB error for unsupported requests */
	IPROC_PCIE_APB_ERR_EN,
};

/* iProc PCIe PAXB registers */
static const u16 iproc_pcie_reg_paxb[] = {
	[IPROC_PCIE_CLK_CTRL]         = 0x000,
	[IPROC_PCIE_MSI_GIC_MODE]     = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_BASE_ADDR]    = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_WINDOW_SIZE]  = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_ADDR_LO]      = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_ADDR_HI]      = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_EN_CFG]       = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_CFG_IND_ADDR]     = 0x120,
	[IPROC_PCIE_CFG_IND_DATA]     = 0x124,
	[IPROC_PCIE_CFG_ADDR]         = 0x1f8,
	[IPROC_PCIE_CFG_DATA]         = 0x1fc,
	[IPROC_PCIE_INTX_EN]          = 0x330,
	[IPROC_PCIE_OARR_LO]          = 0xd20,
	[IPROC_PCIE_OARR_HI]          = 0xd24,
	[IPROC_PCIE_OMAP_LO]          = 0xd40,
	[IPROC_PCIE_OMAP_HI]          = 0xd44,
	[IPROC_PCIE_IARR0_LO]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR0_HI]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP0_LO]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP0_HI]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_LINK_STATUS]      = 0xf0c,
	[IPROC_PCIE_APB_ERR_EN]       = 0xf40,
};

/* iProc PCIe PAXB_V2 registers */
static const u16 iproc_pcie_reg_paxb_v2[] = {
	[IPROC_PCIE_CLK_CTRL]         = 0x000,
	[IPROC_PCIE_MSI_GIC_MODE]     = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_BASE_ADDR]    = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_WINDOW_SIZE]  = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_ADDR_LO]      = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_ADDR_HI]      = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_EN_CFG]       = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_CFG_IND_ADDR]     = 0x120,
	[IPROC_PCIE_CFG_IND_DATA]     = 0x124,
	[IPROC_PCIE_CFG_ADDR]         = 0x1f8,
	[IPROC_PCIE_CFG_DATA]         = 0x1fc,
	[IPROC_PCIE_INTX_EN]          = 0x330,
	[IPROC_PCIE_OARR_LO]          = 0xd60,
	[IPROC_PCIE_OARR_HI]          = 0xd64,
	[IPROC_PCIE_OMAP_LO]          = 0xd68,
	[IPROC_PCIE_OMAP_HI]          = 0xd6c,
	[IPROC_PCIE_IARR0_LO]         = 0xd00,
	[IPROC_PCIE_IARR0_HI]         = 0xd04,
	[IPROC_PCIE_IMAP0_LO]         = 0xc00,
	[IPROC_PCIE_IMAP0_HI]         = 0xc04,
	[IPROC_PCIE_IARR_LO]          = 0xd10,
	[IPROC_PCIE_IARR_HI]          = 0xd14,
	[IPROC_PCIE_IMAP_LO]          = 0xcc0,
	[IPROC_PCIE_IMAP_HI]          = 0xcc4,
	[IPROC_PCIE_LINK_STATUS]      = 0xf0c,
	[IPROC_PCIE_APB_ERR_EN]       = 0xf40,
};

/* iProc PCIe PAXC v1 registers */
static const u16 iproc_pcie_reg_paxc[] = {
	[IPROC_PCIE_CLK_CTRL]         = 0x000,
	[IPROC_PCIE_MSI_GIC_MODE]     = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_BASE_ADDR]    = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_WINDOW_SIZE]  = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_ADDR_LO]      = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_ADDR_HI]      = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_EN_CFG]       = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_CFG_IND_ADDR]     = 0x1f0,
	[IPROC_PCIE_CFG_IND_DATA]     = 0x1f4,
	[IPROC_PCIE_CFG_ADDR]         = 0x1f8,
	[IPROC_PCIE_CFG_DATA]         = 0x1fc,
	[IPROC_PCIE_INTX_EN]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_OARR_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_OARR_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_OMAP_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_OMAP_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR0_LO]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR0_HI]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP0_LO]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP0_HI]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_LINK_STATUS]      = IPROC_PCIE_REG_INVALID,
};

/* iProc PCIe PAXC v2 registers */
static const u16 iproc_pcie_reg_paxc_v2[] = {
	[IPROC_PCIE_CLK_CTRL]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_MSI_GIC_MODE]     = 0x050,
	[IPROC_PCIE_MSI_BASE_ADDR]    = 0x074,
	[IPROC_PCIE_MSI_WINDOW_SIZE]  = 0x078,
	[IPROC_PCIE_MSI_ADDR_LO]      = 0x07c,
	[IPROC_PCIE_MSI_ADDR_HI]      = 0x080,
	[IPROC_PCIE_MSI_EN_CFG]       = 0x09c,
	[IPROC_PCIE_CFG_IND_ADDR]     = 0x1f0,
	[IPROC_PCIE_CFG_IND_DATA]     = 0x1f4,
	[IPROC_PCIE_CFG_ADDR]         = 0x1f8,
	[IPROC_PCIE_CFG_DATA]         = 0x1fc,
	[IPROC_PCIE_INTX_EN]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_OARR_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_OARR_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_OMAP_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_OMAP_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR0_LO]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR0_HI]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP0_LO]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP0_HI]         = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IARR_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP_LO]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_IMAP_HI]          = IPROC_PCIE_REG_INVALID,
	[IPROC_PCIE_LINK_STATUS]      = IPROC_PCIE_REG_INVALID,
};

static inline struct iproc_pcie *iproc_data(struct pci_bus *bus)
{
	struct iproc_pcie *pcie;
#ifdef CONFIG_ARM
	struct pci_sys_data *sys = bus->sysdata;

	pcie = sys->private_data;
#else
	pcie = bus->sysdata;
#endif
	return pcie;
}

static inline bool iproc_pcie_reg_is_invalid(u16 reg_offset)
{
	return !!(reg_offset == IPROC_PCIE_REG_INVALID);
}

static inline u16 iproc_pcie_reg_offset(struct iproc_pcie *pcie,
					enum iproc_pcie_reg reg)
{
	return pcie->reg_offsets[reg];
}

static inline u32 iproc_pcie_read_reg(struct iproc_pcie *pcie,
				      enum iproc_pcie_reg reg)
{
	u16 offset = iproc_pcie_reg_offset(pcie, reg);

	if (iproc_pcie_reg_is_invalid(offset))
		return 0;

	return readl(pcie->base + offset);
}

static inline void iproc_pcie_write_reg(struct iproc_pcie *pcie,
					enum iproc_pcie_reg reg, u32 val)
{
	u16 offset = iproc_pcie_reg_offset(pcie, reg);

	if (iproc_pcie_reg_is_invalid(offset))
		return;

	writel(val, pcie->base + offset);
}

/**
 * APB error forwarding can be disabled during access of configuration
 * registers from the endpoint device, to prevent unsupported requests
 * (typically seen during enumeration with multi-function devices) from
 * triggering a system exception
 */
static inline void iproc_pcie_apb_err_disable(struct pci_bus *bus,
					      bool disable)
{
	struct iproc_pcie *pcie = iproc_data(bus);
	u32 val;

	if (bus->number && pcie->has_apb_err_disable) {
		val = iproc_pcie_read_reg(pcie, IPROC_PCIE_APB_ERR_EN);
		if (disable)
			val &= ~APB_ERR_EN;
		else
			val |= APB_ERR_EN;
		iproc_pcie_write_reg(pcie, IPROC_PCIE_APB_ERR_EN, val);
	}
}

static inline void iproc_pcie_ob_write(struct iproc_pcie *pcie,
				       enum iproc_pcie_reg reg,
				       unsigned window, u32 val)
{
	u16 offset = iproc_pcie_reg_offset(pcie, reg);
	u32 oarr_offset = (pcie->type == IPROC_PCIE_PAXB_V2) ?
			PAXB_OARR_V2_OFFSET : PAXB_OARR_OFFSET;

	if (iproc_pcie_reg_is_invalid(offset))
		return;

	writel(val, pcie->base + offset + (window * oarr_offset));
}

static inline void iproc_pcie_idm_reset(struct iproc_pcie *pcie)
{
	if (pcie->idm) {
		writel(1, pcie->idm);
		udelay(100);
		writel(0, pcie->idm);
		udelay(100);
	}
}

/**
 * Note access to the configuration registers are protected at the higher layer
 * by 'pci_lock' in drivers/pci/access.c
 */
static void __iomem *iproc_pcie_map_cfg_bus(struct pci_bus *bus,
					    unsigned int devfn,
					    int where)
{
	struct iproc_pcie *pcie = iproc_data(bus);
	unsigned slot = PCI_SLOT(devfn);
	unsigned fn = PCI_FUNC(devfn);
	unsigned busno = bus->number;
	u32 val;
	u16 offset;

	/* root complex access */
	if (busno == 0) {
		if (slot > 0 || fn > 0)
			return NULL;

		iproc_pcie_write_reg(pcie, IPROC_PCIE_CFG_IND_ADDR,
				     where & CFG_IND_ADDR_MASK);
		offset = iproc_pcie_reg_offset(pcie, IPROC_PCIE_CFG_IND_DATA);
		if (iproc_pcie_reg_is_invalid(offset))
			return NULL;
		else
			return (pcie->base + offset);
	}

	/*
	 * PAXC is connected to an internally emulated EP within the SoC.  It
	 * allows only one device.
	 */

	if (pcie->ep_is_internal) {
		if (slot > 0 && pcie->type != IPROC_PCIE_PAXC)
			return NULL;

		/* only enumerate up to supported number of PFs */
		if (fn >= pcie->nr_pf)
			return NULL;
	} else {
		val = iproc_pcie_read_reg(pcie, IPROC_PCIE_LINK_STATUS);
		if (!(val & PCIE_PHYLINKUP) || !(val & PCIE_DL_ACTIVE)) {
			iproc_pcie_idm_reset(pcie);
			pcie->link_status = 0;
			return NULL;
		}
	}

	/* EP device access */
	val = (busno << CFG_ADDR_BUS_NUM_SHIFT) |
		(slot << CFG_ADDR_DEV_NUM_SHIFT) |
		(fn << CFG_ADDR_FUNC_NUM_SHIFT) |
		(where & CFG_ADDR_REG_NUM_MASK) |
		(1 & CFG_ADDR_CFG_TYPE_MASK);
	iproc_pcie_write_reg(pcie, IPROC_PCIE_CFG_ADDR, val);
	offset = iproc_pcie_reg_offset(pcie, IPROC_PCIE_CFG_DATA);
	if (iproc_pcie_reg_is_invalid(offset))
		return NULL;
	else
		return (pcie->base + offset);
}

static int iproc_pcie_config_read32(struct pci_bus *bus, unsigned int devfn,
				    int where, int size, u32 *val)
{
	int ret;
	struct iproc_pcie *pcie = iproc_data(bus);
	u32 intr_status;
	u32 idm_value;


	iproc_pcie_apb_err_disable(bus, true);
	ret = pci_generic_config_read32(bus, devfn, where, size, val);
	if (pcie->ep_is_internal == false) {
		intr_status = readl(pcie->base + INTR_STATUS_OFFSET);
		intr_status &= (APB_TIMEOUT_INTR | CMPL_TIMEOUT_INTR);
		if (intr_status) {
			dev_err(pcie->dev, "Timeout Happened\n");
			*val = INVALID_CFG_RD;
			writel(intr_status, pcie->base + INTR_CLEAR_OFFSET);
			if ((!pcie->link_status) && (pcie->idm)) {
				idm_value = readl(pcie->idm +
						  IDM_IDM_ERROR_LOG_STATUS);
				dev_err(pcie->dev,
					"TimeOut:Performing IDM reset:%x:\n",
					idm_value);
				if (idm_value != 0) {
					writel(OVERFLOW_COMPLETE |
					       ERROR_COMPLETE, pcie->idm +
					       IDM_ERROR_LOG_COMPLETE_OFFSET);
					udelay(100);
				}
				iproc_pcie_idm_reset(pcie);
				idm_value = readl(pcie->idm +
						  IDM_IDM_ERROR_LOG_STATUS);
				dev_err(pcie->dev,
					"TimeOut:Performed IDM reset:%x:\n",
					idm_value);
			}
		}
	}
	iproc_pcie_apb_err_disable(bus, false);

	return ret;
}

static int iproc_pcie_config_write32(struct pci_bus *bus, unsigned int devfn,
				     int where, int size, u32 val)
{
	int ret;

	iproc_pcie_apb_err_disable(bus, true);
	ret = pci_generic_config_write32(bus, devfn, where, size, val);
	iproc_pcie_apb_err_disable(bus, false);

	return ret;
}

static struct pci_ops iproc_pcie_ops = {
	.map_bus = iproc_pcie_map_cfg_bus,
	.read = iproc_pcie_config_read32,
	.write = iproc_pcie_config_write32,
};

static int iproc_pcie_set_lanes(struct iproc_pcie *pcie)
{
	u32 val;
	u32 pcie_num_lanes;

	if (of_property_read_u32(pcie->dev->of_node,
	    "brcm,pcie-num-lanes", &pcie_num_lanes)) {
		dev_dbg(pcie->dev,
			"Optional brcm,pcie-num-lanes property not found\n");
		return 0;
	}
	dev_info(pcie->dev, "Number of PCIE Lanes:%d:\n", pcie_num_lanes);

	if (!pcie_num_lanes) {
		dev_err(pcie->dev, "brcm,pcie-num-lanes cannot be zero\n");
		return -EINVAL;
	}
	if (!is_power_of_2(pcie_num_lanes)) {
		dev_err(pcie->dev,
			"brcm,pcie-num-lanes:%d: is not power of two\n",
			pcie_num_lanes);
		return -EINVAL;
	}

	iproc_pcie_write_reg(pcie, IPROC_PCIE_CFG_IND_ADDR,
			     REG_LINK_CAPABILITY & CFG_IND_ADDR_MASK);
	val = iproc_pcie_read_reg(pcie, IPROC_PCIE_CFG_IND_DATA);
	val |= MAX_LINK_WIDTH(pcie_num_lanes);
	iproc_pcie_write_reg(pcie, IPROC_PCIE_CFG_IND_ADDR,
			     REG_LINK_CAPABILITY & CFG_IND_ADDR_MASK);
	iproc_pcie_write_reg(pcie, IPROC_PCIE_CFG_IND_DATA, val);
	return 0;
}

static void iproc_pcie_reset(struct iproc_pcie *pcie)
{
	u32 val;

	/*
	 * PAXC and the internal emulated endpoint device downstream should not
	 * be reset. If firmware has been loaded on the endpoint device at an
	 * earlier boot stage, reset here causes issues
	 */
	if (pcie->ep_is_internal)
		return;

	/*
	 * Select perst_b signal as reset source. Put the device into reset,
	 * and then bring it out of reset
	 */
	val = iproc_pcie_read_reg(pcie, IPROC_PCIE_CLK_CTRL);
	val &= ~EP_PERST_SOURCE_SELECT & ~EP_MODE_SURVIVE_PERST &
		~RC_PCIE_RST_OUTPUT;
	iproc_pcie_write_reg(pcie, IPROC_PCIE_CLK_CTRL, val);
	udelay(250);

	if (iproc_pcie_set_lanes(pcie))
		dev_err(pcie->dev, "Couldn't set pcie lanes\n");

	val |= RC_PCIE_RST_OUTPUT;
	iproc_pcie_write_reg(pcie, IPROC_PCIE_CLK_CTRL, val);
	if (in_interrupt()) {
		dev_dbg(pcie->dev, "In Interrupt Context");
		mdelay(100);
	} else {
		dev_dbg(pcie->dev, "Not In Interrupt Context");
		msleep(100);
	}
}

static int iproc_pcie_check_link(struct iproc_pcie *pcie, struct pci_bus *bus)
{
	u8 hdr_type;
	u32 link_ctrl, class, val;
	u16 pos, link_status;
	bool link_is_active = false;

	/*
	 * PAXC connects to emulated endpoint devices directly and does not
	 * have a Serdes.  Therefore skip the link detection logic here.
	 */
	if (pcie->ep_is_internal)
		return 0;

	/* make sure we are not in EP mode */
	pci_bus_read_config_byte(bus, 0, PCI_HEADER_TYPE, &hdr_type);
	if ((hdr_type & 0x7f) != PCI_HEADER_TYPE_BRIDGE) {
		dev_err(pcie->dev, "in EP mode, hdr=%#02x\n", hdr_type);
		return -EFAULT;
	}

	/* force class to PCI_CLASS_BRIDGE_PCI (0x0604) */
#define PCI_BRIDGE_CTRL_REG_OFFSET 0x43c
#define PCI_CLASS_BRIDGE_MASK      0xffff00
#define PCI_CLASS_BRIDGE_SHIFT     8
	pci_bus_read_config_dword(bus, 0, PCI_BRIDGE_CTRL_REG_OFFSET, &class);
	class &= ~PCI_CLASS_BRIDGE_MASK;
	class |= (PCI_CLASS_BRIDGE_PCI << PCI_CLASS_BRIDGE_SHIFT);
	pci_bus_write_config_dword(bus, 0, PCI_BRIDGE_CTRL_REG_OFFSET, class);

	val = iproc_pcie_read_reg(pcie, IPROC_PCIE_LINK_STATUS);
	if (!(val & PCIE_PHYLINKUP) || !(val & PCIE_DL_ACTIVE)) {
		dev_err(pcie->dev, "PHY or data link is INACTIVE!\n");
		pcie->link_status = false;
		return -ENODEV;
	}
	pcie->link_status = true;

	/* check link status to see if link is active */
	pos = pci_bus_find_capability(bus, 0, PCI_CAP_ID_EXP);
	pci_bus_read_config_word(bus, 0, pos + PCI_EXP_LNKSTA, &link_status);
	if (link_status & PCI_EXP_LNKSTA_NLW)
		link_is_active = true;

	if (!link_is_active) {
		/* try GEN 1 link speed */
#define PCI_LINK_STATUS_CTRL_2_OFFSET 0x0dc
#define PCI_TARGET_LINK_SPEED_MASK    0xf
#define PCI_TARGET_LINK_SPEED_GEN2    0x2
#define PCI_TARGET_LINK_SPEED_GEN1    0x1
		pci_bus_read_config_dword(bus, 0,
					  PCI_LINK_STATUS_CTRL_2_OFFSET,
					  &link_ctrl);
		if ((link_ctrl & PCI_TARGET_LINK_SPEED_MASK) ==
		    PCI_TARGET_LINK_SPEED_GEN2) {
			link_ctrl &= ~PCI_TARGET_LINK_SPEED_MASK;
			link_ctrl |= PCI_TARGET_LINK_SPEED_GEN1;
			pci_bus_write_config_dword(bus, 0,
					   PCI_LINK_STATUS_CTRL_2_OFFSET,
					   link_ctrl);
			msleep(100);

			pos = pci_bus_find_capability(bus, 0, PCI_CAP_ID_EXP);
			pci_bus_read_config_word(bus, 0, pos + PCI_EXP_LNKSTA,
						 &link_status);
			if (link_status & PCI_EXP_LNKSTA_NLW)
				link_is_active = true;
		}
	}

	dev_info(pcie->dev, "link: %s\n", link_is_active ? "UP" : "DOWN");

	return link_is_active ? 0 : -ENODEV;
}

static void iproc_pcie_enable(struct iproc_pcie *pcie)
{
	iproc_pcie_write_reg(pcie, IPROC_PCIE_INTX_EN, SYS_RC_INTX_MASK);
}

/**
 * Some iProc SoCs require the SW to configure the outbound address mapping
 *
 * Outbound address translation:
 *
 * iproc_pcie_address = axi_address - axi_offset
 * OARR = iproc_pcie_address
 * OMAP = pci_addr
 *
 * axi_addr -> iproc_pcie_address -> OARR -> OMAP -> pci_address
 */
static int iproc_pcie_setup_ob(struct iproc_pcie *pcie, u64 axi_addr,
			       u64 pci_addr, resource_size_t size)
{
	struct iproc_pcie_ob *ob = &pcie->ob;
	unsigned i;
	u64 max_size = (u64)ob->window_size * MAX_NUM_OB_WINDOWS;
	u64 remainder;

	if (size > max_size) {
		dev_err(pcie->dev,
			"res size %pap exceeds max supported size 0x%llx\n",
			&size, max_size);
		return -EINVAL;
	}

	div64_u64_rem(size, ob->window_size, &remainder);
	if (remainder) {
		dev_err(pcie->dev,
			"res size %pap needs to be multiple of window size %pap\n",
			&size, &ob->window_size);
		return -EINVAL;
	}

	if (axi_addr < ob->axi_offset) {
		dev_err(pcie->dev,
			"axi address %pap less than offset %pap\n",
			&axi_addr, &ob->axi_offset);
		return -EINVAL;
	}

	/*
	 * Translate the AXI address to the internal address used by the iProc
	 * PCIe core before programming the OARR
	 */
	axi_addr -= ob->axi_offset;

	for (i = 0; i < MAX_NUM_OB_WINDOWS; i++) {
		iproc_pcie_ob_write(pcie, IPROC_PCIE_OARR_LO, i,
				    lower_32_bits(axi_addr) | OARR_VALID |
			    (ob->oarr_size_bits << OARR_SIZE_CFG_SHIFT));
		iproc_pcie_ob_write(pcie, IPROC_PCIE_OARR_HI, i,
				    upper_32_bits(axi_addr));
		iproc_pcie_ob_write(pcie, IPROC_PCIE_OMAP_LO, i,
				    lower_32_bits(pci_addr));
		iproc_pcie_ob_write(pcie, IPROC_PCIE_OMAP_HI, i,
				    upper_32_bits(pci_addr));

		size -= ob->window_size;
		if (size == 0)
			break;

		axi_addr += ob->window_size;
		pci_addr += ob->window_size;
	}

	return 0;
}

/**
 * iproc_pcie_ib_write_imapx - map imap to inbound memory
 * @pcie: iproc pcie
 * @reg: imap reg
 * @window: which window ? e.g. iarr_2 iarr_3 iarr_4
 * @size: size of the window
 * @axi_addr: address to map
 * @wmask: window mask for particular iarr
 */
static inline void iproc_pcie_ib_write_imapx(struct iproc_pcie *pcie,
				enum iproc_pcie_reg reg,
				unsigned int window,
				unsigned long size,
				resource_size_t axi_addr,
				unsigned int wmask)
{
	int imap;
	unsigned int val;
	u16 offset = iproc_pcie_reg_offset(pcie, reg);

	if (iproc_pcie_reg_is_invalid(offset))
		return;

	if (window == 0) {
		/* IARR_2 does not have windows. */
		val = (lower_32_bits(axi_addr) & wmask) | IB_IMAP_VALID;
		writel(val, pcie->base + offset);
		val = upper_32_bits(axi_addr);
		writel(val, pcie->base + offset + 4);
		return;
	}
	size = size / IB_IMAP_MAX;
	for (imap = 0; imap < IB_IMAP_MAX; imap++) {
		val = (lower_32_bits(axi_addr) & wmask) | IB_IMAP_VALID;
		writel(val, pcie->base + offset +
			PAXB_IMAP_LO_OFFSET(window, imap));
		val = upper_32_bits(axi_addr);
		writel(val, pcie->base + offset +
			PAXB_IMAP_HI_OFFSET(window, imap));
		axi_addr += size;
	}
}

static inline void iproc_pcie_ib_write(struct iproc_pcie *pcie,
				       enum iproc_pcie_reg reg,
				       unsigned int window,
				       u32 val)
{
	u16 offset = iproc_pcie_reg_offset(pcie, reg);

	if (iproc_pcie_reg_is_invalid(offset))
		return;

	if (window == 0)
		writel(val, pcie->base + offset);
	else
		writel(val, pcie->base + offset + PAXB_IARR3_OFFSET +
			((window - 1) * PAXB_IARR_V2_OFFSET));
}

static int iproc_pcie_map_ib_ranges(struct iproc_pcie *pcie)
{
	int iarr;

	for (iarr = 0; iarr < pcie->num_of_ib; iarr++) {
		iproc_pcie_ib_write(pcie, IPROC_PCIE_IARR_LO, iarr,
				(lower_32_bits(pcie->ib[iarr].pci_addr) &
				pcie->ib[iarr].wmask) |
				(pcie->ib[iarr].iarr_size_bits <<
				IARR_SIZE_CFG_SHIFT));
		iproc_pcie_ib_write(pcie, IPROC_PCIE_IARR_HI, iarr,
					upper_32_bits(pcie->ib[iarr].pci_addr));
		iproc_pcie_ib_write_imapx(pcie, IPROC_PCIE_IMAP_LO, iarr,
					pcie->ib[iarr].window_size,
					pcie->ib[iarr].axi_addr,
					pcie->ib[iarr].wmask);
	}
	return 0;
}

static int iproc_pcie_map_ranges(struct iproc_pcie *pcie,
				 struct list_head *resources)
{
	struct resource_entry *window;
	int ret;

	resource_list_for_each_entry(window, resources) {
		struct resource *res = window->res;
		u64 res_type = resource_type(res);

		switch (res_type) {
		case IORESOURCE_IO:
		case IORESOURCE_BUS:
			break;
		case IORESOURCE_MEM:
			ret = iproc_pcie_setup_ob(pcie, res->start,
						  res->start - window->offset,
						  resource_size(res));
			if (ret)
				return ret;
			break;
		default:
			dev_err(pcie->dev, "invalid resource %pR\n", res);
			return -EINVAL;
		}
	}

	return 0;
}

static int iproce_pcie_get_msi(struct iproc_pcie *pcie,
				struct device_node *msi_node,
				u64 *msi_addr)
{
	int ret;
	struct resource res;

	/*
	 * Check if 'msi-map' points to ARM GICv3 ITS,
	 * which is the only MSI-controller hooked up with both
	 * PAXC v2 and PAXB v2
	 */
	if (!of_device_is_compatible(msi_node, "arm,gic-v3-its")) {
		dev_err(pcie->dev,
			"unable to find compatible MSI controller\n");
		return -ENODEV;
	}

	/* derive GITS_TRANSLATER address from GICv3 */
	ret = of_address_to_resource(msi_node, 0, &res);
	if (ret < 0) {
		dev_err(pcie->dev,
			"unable to obtain MSI controller resources\n");
		return ret;
	}

	*msi_addr = res.start + GITS_TRANSLATER;
	return 0;
}

static int iproc_pcie_paxb_msi_steer(struct iproc_pcie *pcie,
				     struct device_node *msi_node)
{
	int ret;
	u64 msi_addr;
	u32 val;

	ret = iproce_pcie_get_msi(pcie, msi_node, &msi_addr);
	if (ret < 0) {
		dev_err(pcie->dev,
			"paxb msi steering failed\n");
		return ret;
	}

	/* program incoming pci address to iarr0. */
	val = (lower_32_bits(msi_addr) & IARR_0_WINDOW_MASK) | IB_IMAP_VALID;
	iproc_pcie_write_reg(pcie, IPROC_PCIE_IARR0_LO, val);
	iproc_pcie_write_reg(pcie, IPROC_PCIE_IARR0_HI,
		upper_32_bits(msi_addr));
	/* program translation register to point it to GICv3 ITS. */
	iproc_pcie_write_reg(pcie, IPROC_PCIE_IMAP0_LO, val);
	iproc_pcie_write_reg(pcie, IPROC_PCIE_IMAP0_HI,
		upper_32_bits(msi_addr));
	return 0;
}

static int iproc_pcie_paxc_msi_steer(struct iproc_pcie *pcie,
				     struct device_node *msi_node)
{
	int ret;
	u64 msi_addr;
	u32 val;

	/* if PAXC v2 event queue based MSI controller is detected, use it */
	if (of_device_is_compatible(msi_node, "brcm,iproc-msi-paxc-v2"))
		return iproc_msi_paxc_v2_init(pcie, msi_node);

	ret = iproce_pcie_get_msi(pcie, msi_node, &msi_addr);
	if (ret < 0) {
		dev_err(pcie->dev,
			"paxc msi steering failed\n");
		return ret;
	}

	/*
	 * Program bits [43:13] of address of GITS_TRANSLATER register into
	 * bits [30:0] of the MSI base address register. In fact, in all iProc
	 * based SoCs, all I/O register bases are well below the 32-bit
	 * boundary, so we can safely assume bits [43:32] are always zeros
	 */
	iproc_pcie_write_reg(pcie, IPROC_PCIE_MSI_BASE_ADDR,
			     (u32)(msi_addr >> 13));

	/* use a default 8K window size */
	iproc_pcie_write_reg(pcie, IPROC_PCIE_MSI_WINDOW_SIZE, 0);

	/* steering MSI to GICv3 ITS */
	val = iproc_pcie_read_reg(pcie, IPROC_PCIE_MSI_GIC_MODE);
	val |= GIC_V3_CFG;
	iproc_pcie_write_reg(pcie, IPROC_PCIE_MSI_GIC_MODE, val);

	/*
	 * Program bits [43:2] of address of GITS_TRANSLATER register into the
	 * iProc MSI address registers
	 */
	msi_addr >>= 2;
	iproc_pcie_write_reg(pcie, IPROC_PCIE_MSI_ADDR_HI,
			     upper_32_bits(msi_addr));
	iproc_pcie_write_reg(pcie, IPROC_PCIE_MSI_ADDR_LO,
			     lower_32_bits(msi_addr));

	/* enable MSI */
	val = iproc_pcie_read_reg(pcie, IPROC_PCIE_MSI_EN_CFG);
	val |= MSI_ENABLE_CFG;
	iproc_pcie_write_reg(pcie, IPROC_PCIE_MSI_EN_CFG, val);

	return 0;
}

static int iproc_pcie_msi_enable(struct iproc_pcie *pcie)
{
	struct device_node *msi_node;
	int ret;

	/*
	 * Either the "msi-parent" or the "msi-map" phandle needs to exist
	 * for us to obtain the MSI node
	 */
	msi_node = of_parse_phandle(pcie->dev->of_node, "msi-parent", 0);
	if (!msi_node) {
		const __be32 *msi_map = NULL;
		int len;
		u32 phandle;

		msi_map = of_get_property(pcie->dev->of_node, "msi-map", &len);
		if (!msi_map)
			return -ENODEV;

		phandle = be32_to_cpup(msi_map + 1);
		msi_node = of_find_node_by_phandle(phandle);
		if (!msi_node)
			return -ENODEV;
	}

	/*
	 * PAXB v2 requires additional configurations to steer MSI to another
	 * MSI controller
	 */
	if (pcie->type == IPROC_PCIE_PAXB_V2) {
		ret = iproc_pcie_paxb_msi_steer(pcie, msi_node);
		if (ret)
			return ret;
	}

	/*
	 * PAXC v2 requires additional configurations to steer MSI to another
	 * MSI controller
	 */
	if (pcie->type == IPROC_PCIE_PAXC_V2) {
		ret = iproc_pcie_paxc_msi_steer(pcie, msi_node);
		if (ret)
			return ret;
	}

	/*
	 * If another MSI controller is being used, the call below should fail
	 * but that is okay
	 */
	return iproc_msi_init(pcie, msi_node);
}

static void iproc_pcie_msi_disable(struct iproc_pcie *pcie)
{
	iproc_msi_exit(pcie);
}

void iproc_timer_hdlr(unsigned long data)
{
	struct pci_bus *bus = (struct pci_bus *)data;
	struct iproc_pcie *pcie = iproc_data(bus);
	u32 val;
	u32 idm_value;

	mod_timer(&pcie->timer_hdlr, jiffies +
		  msecs_to_jiffies(pcie->link_poll_interval));
	val = iproc_pcie_read_reg(pcie, IPROC_PCIE_LINK_STATUS);
	if (!(val & PCIE_PHYLINKUP) || !(val & PCIE_DL_ACTIVE)) {
		if (pcie->link_status) {
			dev_err(pcie->dev,
				"%s PHY or data link is DOWN!\n", bus->name);
			dev_err(pcie->dev, "LinkDown:Performing IDM reset\n");
			pcie->link_status = false;
			iproc_pcie_idm_reset(pcie);
		}
		if (pcie->idm) {
			idm_value = readl(pcie->idm +
					  IDM_IDM_ERROR_LOG_STATUS);
			if (idm_value != 0) {
				dev_err(pcie->dev,
					"LinkDown:Performing IDM Clear:%x:\n",
					idm_value);
				writel(OVERFLOW_COMPLETE | ERROR_COMPLETE,
				       pcie->idm +
				       IDM_ERROR_LOG_COMPLETE_OFFSET);
			}
		}
		return;
	}
	if (!pcie->link_status) {
		dev_err(pcie->dev, "LinkUp Detected for bus:%s:\n", bus->name);
		iproc_pcie_idm_reset(pcie);
		iproc_pcie_reset(pcie);
		pci_bus_read_config_dword(pcie->root_bus, 0,
					  PCI_STATUS_COMMAND, &val);
		val |= (SERR_ENA | PERR_ENA | BUS_MASTER | MEM_SPACE);
		pci_bus_write_config_dword(pcie->root_bus, 0,
					   PCI_STATUS_COMMAND, val);

		/* Re-confirm link status */
		val = 0;
		pci_bus_read_config_dword(pcie->root_bus, 0,
					  PCI_STATUS_COMMAND, &val);
		if ((val & BUS_MASTER) && (val & MEM_SPACE)) {
			dev_err(pcie->dev, "%s:%x PHY or data link is UP!\n",
				bus->name, val);
			pcie->link_status = 1;
		}
	}
}
int iproc_pcie_setup(struct iproc_pcie *pcie, struct list_head *res)
{
	int ret;
	void *sysdata;
	struct pci_bus *bus, *child;

	if (!pcie || !pcie->dev || !pcie->base)
		return -EINVAL;

	ret = phy_init(pcie->phy);
	if (ret) {
		dev_err(pcie->dev, "unable to initialize PCIe PHY\n");
		return ret;
	}

	ret = phy_power_on(pcie->phy);
	if (ret) {
		dev_err(pcie->dev, "unable to power on PCIe PHY\n");
		goto err_exit_phy;
	}

	switch (pcie->type) {
	case IPROC_PCIE_PAXB:
		pcie->reg_offsets = iproc_pcie_reg_paxb;
		pcie->ep_is_internal = false;
		pcie->has_apb_err_disable = true;
		break;
	case IPROC_PCIE_PAXB_V2:
		pcie->reg_offsets = iproc_pcie_reg_paxb_v2;
		pcie->ep_is_internal = false;
		pcie->has_apb_err_disable = true;
		break;
	case IPROC_PCIE_PAXC:
		pcie->reg_offsets = iproc_pcie_reg_paxc;
		pcie->ep_is_internal = true;
		pcie->nr_pf = 4;
		break;
	case IPROC_PCIE_PAXC_V2:
		pcie->reg_offsets = iproc_pcie_reg_paxc_v2;
		pcie->ep_is_internal = true;
		pcie->nr_pf = 1;
		break;
	default:
		dev_err(pcie->dev, "incompatible iProc PCIe interface\n");
		ret = -EINVAL;
		goto err_power_off_phy;
	}

	iproc_pcie_reset(pcie);

	if (pcie->need_ob_cfg) {
		ret = iproc_pcie_map_ranges(pcie, res);
		if (ret) {
			dev_err(pcie->dev, "map failed\n");
			goto err_power_off_phy;
		}
	}

	if (pcie->need_ib_cfg) {
		ret = iproc_pcie_map_ib_ranges(pcie);
		if (ret) {
			dev_err(pcie->dev, "inbound mapping failed\n");
			goto err_power_off_phy;
		}
	}

#ifdef CONFIG_ARM
	pcie->sysdata.private_data = pcie;
	sysdata = &pcie->sysdata;
#else
	sysdata = pcie;
#endif

	bus = pci_create_root_bus(pcie->dev, 0, &iproc_pcie_ops, sysdata, res);
	if (!bus) {
		dev_err(pcie->dev, "unable to create PCI root bus\n");
		ret = -ENOMEM;
		goto err_power_off_phy;
	}
	pcie->root_bus = bus;

	sprintf(bus->name, "IPROC_ROOT_BUS_:%x:\n",
		(unsigned int) pcie->base_addr);
	dev_dbg(pcie->dev, "Created root bus:Num %d:name=%s: base:%x\n",
		bus->number, bus->name, (unsigned int) pcie->base_addr);

	ret = iproc_pcie_check_link(pcie, bus);
	if (ret)
		dev_err(pcie->dev, "no PCIe EP device detected\n");
	else
		dev_dbg(pcie->dev, "PCIe EP device UP....");

	iproc_pcie_enable(pcie);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		if (iproc_pcie_msi_enable(pcie))
			dev_info(pcie->dev, "not using iProc MSI\n");

	pci_scan_child_bus(bus);
	pci_assign_unassigned_bus_resources(bus);

	if (pcie->map_irq)
		pci_bus_fixup_irqs(bus, pci_common_swizzle, pcie->map_irq);

	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);

	pci_bus_add_devices(bus);

	if (pcie->idm)
		writel(TIMEOUT_ENABLE | TIMEOUT_EXPONENT(0x1f) |
		       TIMEOUT_INTERRUPT | BUS_ERROR_INTERRUPT,
		       pcie->idm + IDM_ERROR_LOG_CONTROL_OFFSET);
	if (pcie->link_poll_interval) {
		setup_timer(&pcie->timer_hdlr, iproc_timer_hdlr,
			    (unsigned long)bus);
		mod_timer(&pcie->timer_hdlr, jiffies +
			  msecs_to_jiffies(pcie->link_poll_interval));
	}

	return 0;

err_power_off_phy:
	phy_power_off(pcie->phy);
err_exit_phy:
	phy_exit(pcie->phy);
	return ret;
}
EXPORT_SYMBOL(iproc_pcie_setup);

int iproc_pcie_remove(struct iproc_pcie *pcie)
{
	pci_stop_root_bus(pcie->root_bus);
	pci_remove_root_bus(pcie->root_bus);

	iproc_pcie_msi_disable(pcie);

	phy_power_off(pcie->phy);
	phy_exit(pcie->phy);
	del_timer(&pcie->timer_hdlr);

	return 0;
}
EXPORT_SYMBOL(iproc_pcie_remove);

int iproc_pcie_resume(struct iproc_pcie *pcie, struct list_head *res)
{
	int ret;

	if (!pcie || !pcie->dev || !pcie->base)
		return -EINVAL;

	if (pcie->phy) {
		ret = phy_init(pcie->phy);
		if (ret) {
			dev_err(pcie->dev, "unable to initialize PCIe PHY\n");
			return ret;
		}

		ret = phy_power_on(pcie->phy);
		if (ret) {
			dev_err(pcie->dev, "unable to power on PCIe PHY\n");
			goto err_exit_phy;
		}
	}

	iproc_pcie_reset(pcie);

	if (pcie->need_ob_cfg) {
		ret = iproc_pcie_map_ranges(pcie, res);
		if (ret) {
			dev_err(pcie->dev, "map failed\n");
			goto err_power_off_phy;
		}
	}

	if (pcie->need_ib_cfg) {
		ret = iproc_pcie_map_ib_ranges(pcie);
		if (ret) {
			dev_err(pcie->dev, "inbound mapping failed\n");
			goto err_power_off_phy;
		}
	}

	/*
	 * Based on testing, give more time for PHY link to become active.
	 */
	msleep(20);

	ret = iproc_pcie_check_link(pcie, pcie->root_bus);
	if (ret) {
		dev_err(pcie->dev, "no PCIe EP device detected\n");
		goto err_power_off_phy;
	}

	iproc_pcie_enable(pcie);

	if (pcie->msi) {
		if (pcie->type == IPROC_PCIE_PAXC_V2)
			iproc_msi_paxc_v2_enable(pcie->msi);
		else
			iproc_msi_enable(pcie->msi);
	}

	return 0;

err_power_off_phy:
	phy_power_off(pcie->phy);
err_exit_phy:
	phy_exit(pcie->phy);
	return ret;
}
EXPORT_SYMBOL(iproc_pcie_resume);

int iproc_pcie_suspend(struct iproc_pcie *pcie)
{
	if (pcie->msi) {
		if (pcie->type == IPROC_PCIE_PAXC_V2)
			iproc_msi_paxc_v2_disable(pcie->msi);
		else
			iproc_msi_disable(pcie->msi);
	}

	if (pcie->phy) {
		phy_power_off(pcie->phy);
		phy_exit(pcie->phy);
	}

	return 0;
}
EXPORT_SYMBOL(iproc_pcie_suspend);

/**
 * FIXME
 * Hacky code to work around the ASIC issue with PAXC and Nitro
 *
 * 1. The bridge header fix should eventually be moved to pci/quirks.c
 * 2. The Nitro fix should be moved to either Chimp firmware or the Nitro
 * kernel driver, which we have no control at this point. Or, hopefully this
 * may be fixed in NS2 B0
 */
static void quirk_paxc_bridge(struct pci_dev *pdev)
{
	struct iproc_pcie *pcie = iproc_data(pdev->bus);
	int pf;

	/* MPSS is not being set properly (as it is currently 0).  Add this as
	 * a workaround until a fix can be added to chimp fw.  The MPS is being
	 * set to 512 bytes.  So, I'll assume that the MPSS can be at least 512
	 * (or a MPSS value of 2).
	 */
	pdev->pcie_mpss = 2;

	if (pdev->hdr_type == PCI_HEADER_TYPE_BRIDGE) {
		pdev->class = PCI_CLASS_BRIDGE_PCI << 8;

		/* Ensure the bridge on PAXC is identified as a PCIe root port
		 * even when Chimp is not installed or an older version is
		 * running.  This prevents issues during device enumeration.
		 */
		pdev->pcie_flags_reg |= 0x2;
		pdev->pcie_flags_reg |= (PCI_EXP_TYPE_ROOT_PORT << 4);

#define PAXC_RX_DEBUG_CONTROL	0x28
#define PAXC_RX_WR_INRC_AWID	(1 << 13)
#define PAXC_RX_IGNORE_RDCMD_ORDERING	(1 << 12)
#define PAXC_RX_WR_BURST	(4 << 9)
#define PAXC_RX_RD_BURST	(4 << 6)
#define PAXC_RX_IGNORE_BRESP	(1 << 5)
#define PAXC_RX_FREE_CMPL_BUF	(7 << 2)
#define PAXC_RX_FREE_ARID_CNT	(3)

		/* Tune the PAXC RX for increased performance */
		writel(PAXC_RX_WR_INRC_AWID | PAXC_RX_WR_BURST |
		       PAXC_RX_RD_BURST | PAXC_RX_IGNORE_BRESP |
		       PAXC_RX_FREE_CMPL_BUF | PAXC_RX_FREE_ARID_CNT |
		       PAXC_RX_IGNORE_RDCMD_ORDERING,
		       pcie->base + PAXC_RX_DEBUG_CONTROL);

#define PAXC_TRANSACTION_SIZE_128B_ADDR 0x1000b4
#define PAXC_TRANSACTION_SIZE_128B_DATA 0x102c50

		writel(PAXC_TRANSACTION_SIZE_128B_ADDR,
		       pcie->base + CFG_ADDR_OFFSET);
		writel(PAXC_TRANSACTION_SIZE_128B_DATA,
		       pcie->base + CFG_DATA_OFFSET);
		return;
	}
#define PAXC_CFG_ECM_ADDR_OFFSET     0x1e0
#define PAXC_CFG_ECM_DBG_EN_SHIFT    31
#define PAXC_CFG_ECM_DBG_EN          BIT(PAXC_CFG_ECM_DBG_EN_SHIFT)
#define PAXC_CFG_FUNC_SHIFT          12
#define PAXC_CFG_FUNC_MASK           0x7000
#define PAXC_CFG_FUNC(pf)            (((pf) << PAXC_CFG_FUNC_SHIFT) & \
				      PAXC_CFG_FUNC_MASK)
#define PAXC_CFG_ECM_DATA_OFFSET     0x1e4

#define NITRO_MSI_CFG_OFFSET         0x4c4
#define NITRO_QSIZE_OFFSET           0x4c0
	for (pf = 0; pf < MAX_NUM_PAXC_PF; pf++) {
		u32 val;

		/*
		 * TODO:
		 * Need to figure out what these hardcoded values mean.
		 * It's unbelievable that after weeks of poking around and
		 * digging, there's still no one who can point me to a proper
		 * Nitro documentation
		 */
		val = PAXC_CFG_ECM_DBG_EN | PAXC_CFG_FUNC(pf) |
			NITRO_MSI_CFG_OFFSET;
		writel(val, pcie->base + PAXC_CFG_ECM_ADDR_OFFSET);
		writel(0x4, pcie->base + PAXC_CFG_ECM_DATA_OFFSET);

		val = PAXC_CFG_ECM_DBG_EN | PAXC_CFG_FUNC(pf) |
			NITRO_QSIZE_OFFSET;
		writel(val, pcie->base + PAXC_CFG_ECM_ADDR_OFFSET);
		writel(0xba80b, pcie->base + PAXC_CFG_ECM_DATA_OFFSET);
	}
	writel(0, pcie->base + PAXC_CFG_ECM_ADDR_OFFSET);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_BROADCOM, PCI_DEVICE_ID_NX2_57810,
			quirk_paxc_bridge);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_BROADCOM, 0x16cd, quirk_paxc_bridge);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_BROADCOM, 0x16f0, quirk_paxc_bridge);

static void northstar_ns2_perftune(struct pci_dev *pdev)
{
#define NS2_CREDIT_REPORTING		0xa14
#define NS2_CREDIT_REPORTING_INTERVAL	0x03402020

	pci_bus_write_config_dword(pdev->bus, 0, NS2_CREDIT_REPORTING,
				   NS2_CREDIT_REPORTING_INTERVAL);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_BROADCOM, 0xd300, northstar_ns2_perftune);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_BROADCOM, 0xe711, northstar_ns2_perftune);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_BROADCOM, 0xd712, northstar_ns2_perftune);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_BROADCOM, 0xd713, northstar_ns2_perftune);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_BROADCOM, 0xd715, northstar_ns2_perftune);

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom iPROC PCIe common driver");
MODULE_LICENSE("GPL v2");
