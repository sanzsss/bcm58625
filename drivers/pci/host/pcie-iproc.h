/*
 * Copyright (C) 2014-2015 Broadcom Corporation
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

#ifndef _PCIE_IPROC_H
#define _PCIE_IPROC_H

/**
 * iProc PCIe interface type
 *
 * PAXB is the wrapper used in root complex that can be connected to an
 * external endpoint device.
 *
 * PAXC is the wrapper used in root complex dedicated for internal emulated
 * endpoint devices.
 *
 * PAXC v2 is the second generation of root complex wrapper dedicated for
 * internal emulated endpoint devices.
 */
enum iproc_pcie_type {
	IPROC_PCIE_PAXB = 0,
	IPROC_PCIE_PAXB_V2,
	IPROC_PCIE_PAXC,
	IPROC_PCIE_PAXC_V2,
};

/**
 * iProc PCIe outbound mapping
 * @oarr_size_bits: indicates the OARR size bits
 * @axi_offset: offset from the AXI address to the internal address used by
 * the iProc PCIe core
 * @window_size: outbound window size
 */
struct iproc_pcie_ob {
	unsigned int oarr_size_bits;
	resource_size_t axi_offset;
	resource_size_t window_size;
};

/**
 * iProc PCIe inbound mapping
 * @axi_addr: the AXI address to the internal address used by RC.
 * @pci_addr: the PCI address which are seen by RC.
 * @window_size: inbound window size
 * @iarr_size_bits: iarr size bitmap
 * @wmask: window mask
 */
struct iproc_pcie_ib {
	resource_size_t axi_addr;
	resource_size_t pci_addr;
	resource_size_t window_size;
	unsigned int iarr_size_bits;
	unsigned int wmask;
};


struct iproc_msi;

/**
 * iProc PCIe device
 *
 * @dev: pointer to device data structure
 * @type: iProc PCIe interface type
 * @reg_offsets: register offsets
 * @base: PCIe host controller I/O register base
 * @base_addr: PCIe host controller register base physical address
 * @sysdata: Per PCI controller data (ARM-specific)
 * @root_bus: pointer to root bus
 * @phy: optional PHY device that controls the Serdes
 * @map_irq: function callback to map interrupts
 * @need_ob_cfg: indicates SW needs to configure the outbound mapping window
 * @need_ib_cfg: indicates SW needs to configure the inbound mapping window
 * @ep_is_internal: indicates an internal emulated endpoint device is connected
 * @nr_pf: number of physical functions (only valid for internally emulated
 * EPs)
 * @ob: outbound mapping parameters
 * @ib: inbound mapping parameters
 * @num_of_ib: number of inbound windows
 * @msi: MSI data
 */
struct iproc_pcie {
	struct device *dev;
	enum iproc_pcie_type type;
	const u16 *reg_offsets;
	void __iomem *base;
	phys_addr_t base_addr;
#ifdef CONFIG_ARM
	struct pci_sys_data sysdata;
#endif
	struct pci_bus *root_bus;
	struct phy *phy;
	int (*map_irq)(const struct pci_dev *, u8, u8);
	bool need_ob_cfg;
	bool need_ib_cfg;
	bool ep_is_internal;
	unsigned int nr_pf;
	bool has_apb_err_disable;
	struct iproc_pcie_ob ob;
	struct iproc_pcie_ib *ib;
	unsigned int num_of_ib;
	struct iproc_msi *msi;

	struct timer_list timer_hdlr;
	bool link_status;
	u32 link_poll_interval;
	void __iomem *idm;
	phys_addr_t idm_addr;
};

int iproc_pcie_setup(struct iproc_pcie *pcie, struct list_head *res);
int iproc_pcie_remove(struct iproc_pcie *pcie);
int iproc_pcie_resume(struct iproc_pcie *pcie, struct list_head *res);
int iproc_pcie_suspend(struct iproc_pcie *pcie);

#ifdef CONFIG_PCIE_IPROC_MSI
int iproc_msi_init(struct iproc_pcie *pcie, struct device_node *node);
void iproc_msi_exit(struct iproc_pcie *pcie);
void iproc_msi_enable(struct iproc_msi *msi);
void iproc_msi_disable(struct iproc_msi *msi);
int iproc_msi_paxc_v2_init(struct iproc_pcie *pcie, struct device_node *node);
void iproc_msi_paxc_v2_exit(struct iproc_pcie *pcie);
void iproc_msi_paxc_v2_enable(struct iproc_msi *msi);
void iproc_msi_paxc_v2_disable(struct iproc_msi *msi);
#else
static inline int iproc_msi_init(struct iproc_pcie *pcie,
				 struct device_node *node)
{
	return -ENODEV;
}
static inline void iproc_msi_exit(struct iproc_pcie *pcie)
{
}
static inline void iproc_msi_enable(struct iproc_msi *msi)
{
}
static inline void iproc_msi_disable(struct iproc_msi *msi)
{
}
static inline int iproc_msi_paxc_v2_init(struct iproc_pcie *pcie,
					 struct device_node *node)
{
	return -ENODEV;
}
static inline void iproc_msi_paxc_v2_exit(struct iproc_pcie *pcie)
{
}
static inline void iproc_msi_paxc_v2_enable(struct iproc_msi *msi)
{
}
static inline void iproc_msi_paxc_v2_disable(struct iproc_msi *msi)
{
}
#endif

#endif /* _PCIE_IPROC_H */
