/*
 * Copyright (C) 2015 Broadcom Corporation
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

#ifndef __BCM_AMAC_ENET_H__
#define __BCM_AMAC_ENET_H__

#include <linux/kfifo.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

#define AMAC_DEF_MSG_ENABLE	  \
	(NETIF_MSG_DRV		| \
	 NETIF_MSG_RX_ERR	| \
	 NETIF_MSG_TX_ERR)

/* Wake on LAN */
#define AMAC_WOL_ENABLE		1
#define AMAC_WOL_DISABLE	0

#define AMAC_WARM_RESET_SEQ 0x1111

/* Ethernet Statistics
 * Internally maintained data structure with detailed stats
 */
struct sysctl_ethstats {
	u64 rx_bytes;
	u64 rx_dropped_pkts;
	u64 rx_resyncs;
	u64 rx_wraparounds;
	u64 rx_syncchecked;
	u64 rx_syncdroppedpkts;
	u64 rx_noskb;
	u64 rx_broadcast;
	u64 rx_multicast;
	u64 rx_unicast;
	u64 tx_broadcast;
	u64 tx_multicast;
	u64 tx_unicast;
	u64 tx_dropped_pkts;
	u64 tx_errors;
	u64 rx_errors;
};

struct port_status {
	u32 link; /* link status */
	u32 speed; /* port speed */
	u32 duplex; /* port duplex */
	u32 pause; /* pause frames */
	u32 aneg; /* auto negotiation */
};

struct port_info {
	/* Current port status for link updates */
	struct port_status stat;

	/* In case of switch-by-pass mode */
	struct device_node *phy_node;
	struct phy_device *phydev; /* Connected PHY dev */
	int phy_mode; /* phy mode */
	bool lswap; /* lane swapping */
	bool phy54810_rgmii_sync; /* PHY54810 fix up */
	bool phy54810_rgmii_lswap; /* PHY54810 XMC fix*/
	bool pause_disable;
};

/* Ethernet Port data structure */
struct port_data {
	u32 imp_port_speed; /* IMP Port (Port8) max speed */
	/* external port (with internal or ext PHY) */
	struct port_info ext_port;
};

/* AMAC registers */
struct bcm_amac_reg_base {
	void __iomem *amac_core;
	void __iomem *amac_idm_base;
	void __iomem *rgmii_regs;
	void __iomem *ctf_regs;
	void __iomem *switch_global_cfg;
	void __iomem *crmu_io_pad_ctrl;
	void __iomem *amac3_io_ctrl;
	void __iomem *apb2pbus_base;
	void __iomem *eth_config_status;
};

/* Structure contains all the hardware info */
struct bcm_amac_hw {
	struct bcm_amac_reg_base reg; /* iomapped register addresses */
	u32 intr_num; /* Interrupt number */
};

enum amac_state {
	AMAC_INIT,
	AMAC_OPENED,
	AMAC_STARTED,
	AMAC_RESUMED,
	AMAC_SUSPENDED,
	AMAC_STOPPED,
	AMAC_CLOSED,
	AMAC_SHUTDOWN,
};

bool prbs_test_per_lane(void *pdev, int lane);
void bcm_amac_set_ethtool_ops(struct net_device *netdev);
void dump_pm_regs(void *pdev);

#endif /*__BCM_AMAC_ENET_H__*/

