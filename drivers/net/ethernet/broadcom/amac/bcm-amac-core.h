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
#ifndef __BCM_AMAC_CORE_H__
#define __BCM_AMAC_CORE_H__

#include "bcm-amac-enet.h"
#include "bcm-amac-regs.h"

#define BCM_AMAC_CORE_INTR_UNKNOWN 0
#define BCM_AMAC_CORE_INTR_TX      1
#define BCM_AMAC_CORE_INTR_RX      2

#define BCM_AMAC_DMA_BUSY 1
#define BCM_AMAC_DMA_FREE 0

#define BRCM_HEADER_LEN	4

#define BCM_AMAC_DIR_TX false
#define BCM_AMAC_DIR_RX true

#define GFP_FLAGS (GFP_KERNEL)

#define SHIM_HEADER_SIZE_BYTES  8
#define PROC_OP_SHIFT           29
#define PROC_OP_MASK            0xE0000000
#define PROC_OP(prot_type)      (((prot_type) << PROC_OP_SHIFT) & PROC_OP_MASK)

#define TC_SHIFT                26
#define TC_MASK                 0x1c000000
#define TC(tc_value)            (((tc_value) << TC_SHIFT) & TC_MASK)

#define TE_SHIFT                24
#define TE_MASK                 0x3000000
#define TE(te_value)            (((te_value) << TE_SHIFT) & TE_MASK)

#define TS_SHIFT                23
#define TS_MASK                 0x800000
#define TS(ts_value)            (((ts_value) << TS_SHIFT) & TS_MASK)

#define IPV4_OFFSET_SHIFT        17
#define IPV4_OFFSET_MASK         0x17E0000
#define IPV4_OFFSET(offset)      (((offset) << IPV4_OFFSET_SHIFT) & \
				    IPV4_OFFSET_MASK)

#define L4OFFSET_SHIFT          10
#define L4OFFSET_MASK           0x1FC00
#define L4OFFSET(offset)        (((offset) << L4OFFSET_SHIFT) & L4OFFSET_MASK)

#define DST_MAP_SHIFT           0
#define DST_MAP_MASK            0x3FC
#define DST_MAP(value)          (((value) << DST_MAP_SHIFT) & DST_MAP_MASK)

/* Shim Header Word1 */

#define OFFSET1_SHIFT           28
#define OFFSET1_MASK            0xF0000000
#define OFFSET1(value)          (((value) << OFFSET1_SHIFT) & OFFSET1_MASK)

#define OFFSET0_SHIFT           22
#define OFFSET0_MASK            0xFC00000
#define OFFSET0(value)          (((value) << OFFSET0_SHIFT) & OFFSET0_MASK)

#define REG_CHECKSUM_SHIFT      21
#define REG_CHECKSUM_MASK       0x200000
#define REG_CHECKSUM(value)     (((value) << REG_CHECKSUM_SHIFT) & \
				   REG_CHECKSUM_MASK)

#define OSTS_SHIFT              20
#define OSTS_MASK               0x100000
#define OSTS(value)             (((value) << OSTS_SHIFT) & OSTS_MASK)
#define DO_TSTAMP_SHIFT         19
#define DO_TSTAMP_MASK          0x80000
#define DO_TSTAMP(value)        (((value) << DO_TSTAMP_SHIFT) & DO_TSTAMP_MASK)

#define L4_FRAG_OFFSET_SHIFT    15
#define L4_FRAG_OFFSET_MASK     0x78000
#define L4_FRAG_OFFSET(value)   (((value) << L4_FRAG_OFFSET_SHIFT) & \
				   L4_FRAG_OFFSET_MASK)

#define MAX_SEG_SIZE_SHIFT      0
#define MAX_SEG_SIZE_MASK       0x7FFF
#define MAX_SEG_SIZE(value)     (((value) << MAX_SEG_SIZE_SHIFT) & \
				   MAX_SEG_SIZE_MASK)

/* DMA Descriptor */
struct amac_dma64_desc {
	u32 ctrl1;    /* misc control bits */
	u32 ctrl2;    /* buffer count and address extension */
	u32 addrlow;  /* mem addr of data buffer, bits 31:0 */
	u32 addrhigh; /* mem addr of data buffer, bits 63:32 */
};

/* DMA configuration data structure
 * These are used to configure the DMA block
 */
struct amac_dma_cfg {
	void *raw_descp;
	dma_addr_t raw_addr;
	void *descp;    /* Aligned Descriptor pointer */
	dma_addr_t base_addr;/* Aligned bus base address */
	u32 ring_len;   /* Total number of descriptors */
	u32 alloc_size; /* Total memory alloc in bytes */
	u32 index;      /* Current descriptor index */
};

/* SKB node data structure */
struct skb_list_node {
	struct sk_buff *skb;
	dma_addr_t dma_addr;/* bus base address of region */
	int len;
};

/* DMA private data for both RX and TX */
struct amac_dma_priv {
	struct amac_dma_cfg rx;
	struct amac_dma_cfg tx;
	struct kfifo txfifo;
	u32 tx_max_pkts; /* max number of packets */
	u32 tx_curr;     /* current packet index */
	struct skb_list_node *tx_skb_list; /* list of skb given to hw for tx */
	struct skb_list_node *rx_skb_list; /* list of skb given to hw for rx */
	atomic_t tx_dma_busy; /* keep track of DMA status */
};

/* AMAC Driver's private data structure.
 * Contains data for the driver instance. This structure is used
 * through-out the driver to derrive status of various blocks,
 * settings, stats, hw registers etc.
 */
struct bcm_amac_priv {
	u8   amac_id;
	s32  b_unimac;
	enum amac_state state;
	u64  register_base;
	u32  rings_offloaded;
	struct napi_struct napi ____cacheline_aligned;
	struct tasklet_struct tx_tasklet;
	u32 dmactrlflags; /* DMA Flags */

	struct sysctl_ethstats eth_stats;
	struct tasklet_struct rx_tasklet_errors;
	struct tasklet_struct tx_tasklet_errors;
	struct work_struct    tx_work;
	struct net_device *ndev; /* net device reference */
	struct platform_device *pdev; /* platform device reference */
	struct platform_device *brm;  /* brm platform device reference */

	struct port_data port; /* Port and PHY Info */

	struct amac_dma_priv dma; /* DMA info */

	/* netlink socket for link change notifications */
	struct sock *nl_sk;
	/* tx timeout handler */
	struct work_struct tx_timeout_task;

	u32 nl_seq; /* link notification sequence number */

	struct bcm_amac_hw hw; /* Hardware info */

	struct sockaddr cur_etheraddr; /* local ethernet address */

	bool switch_mode; /* internal switch availability */

	spinlock_t lock; /* used by netdev api's */
	u32 msg_enable; /* message filter bit mask */
};

int bcm_amac_gphy_init(struct bcm_amac_priv *privp);
void bcm_amac_gphy_exit(struct bcm_amac_priv *privp);
int bcm_amac_gphy_powerup(struct bcm_amac_priv *privp, bool powerup);
void bcm_amac_gphy_shutdown(struct bcm_amac_priv *privp);
void bcm_amac_gphy_start(struct bcm_amac_priv *privp, bool start);
void bcm_amac_gphy_rgmii_init(struct bcm_amac_priv *privp, bool enable);

int bcm_amac_core_init(struct bcm_amac_priv *privp);
void bcm_amac_core_term(struct bcm_amac_priv *privp);
int bcm_amac_dma_start(struct bcm_amac_priv *privp);
void bcm_amac_dma_stop(struct bcm_amac_priv *privp);
void bcm_amac_unimac_enable(struct bcm_amac_priv *privp, bool enable);
void bcm_amac_tx_send_packet(struct bcm_amac_priv *privp);
#ifdef RMO_SUPPORT
void bcm_amac_tx_clean(void *privp, u8 cos,
		       u32 num_pkts, struct sk_buff **skbp);
#else
void bcm_amac_tx_clean(struct bcm_amac_priv *privp);
#endif

int amac_dma_tx_alloc(struct bcm_amac_priv *privp);
int amac_dma_rx_alloc(struct bcm_amac_priv *privp);
int bcm_amac_enable_tx_dma(struct bcm_amac_priv *privp, bool enable);
int bcm_amac_enable_rx_dma(struct bcm_amac_priv *privp, bool enable);
void bcm_amac_enable_rx_intr(struct bcm_amac_priv *privp, bool enable);

void bcm_amac_enable_intr(struct bcm_amac_priv *privp, bool is_rx, bool enable);
void bcm_amac_disable_intr(struct bcm_amac_priv *privp, int intr_bit);
int bcm_amac_get_tx_flag(void);
int bcm_amac_dma_get_rx_data(struct bcm_amac_priv *privp,
			     struct sk_buff **skbp);
void bcm_amac_set_rx_mode(struct net_device *ndev);

void bcm_amac_clear_intr(struct bcm_amac_priv *privp, bool dir);
void bcm_amac_enable_intr(struct bcm_amac_priv *privp, bool dir, bool enable);
int bcm_amac_set_mac(struct bcm_amac_priv *privp, char *macp);
void bcm_amac_setup_unimac_port(struct bcm_amac_priv *privp);
void amac_tx_error_task(unsigned long data);
void amac_rx_error_task(unsigned long data);

irqreturn_t bcm_amac_isr(int irq, void *userdata);
int bcm_amac_print_mib_counters(struct bcm_amac_priv *privp);

extern u8 amac_version;
#ifdef PORTMACRO_ENABLE
typedef void *pm_device_t;
extern void *apb2pbus_base;

void lm_hw_pm_apb2pbus_brdg_init(pm_device_t pdev);
void lm_hw_pm_minimal_default_xlmac_cfg(pm_device_t pdev, u8 ten_gig_en);
int lm_hw_pm_bringup_phy(pm_device_t pdev);
int lm_hw_pm_reset(pm_device_t pdev);
int lm_hw_pm_get_link_status(pm_device_t pdev, int *link_status, u32 *speed);
int lm_hw_pm_cfg_mac_core(pm_device_t pdev, u32 speed);
int lm_hw_pm_cfg_mtu(pm_device_t pdev);
int lm_hw_pm_rd_reg(pm_device_t pdev,
		    u32 addr, u32 *data_hi, u32 *data_low, u8 reg_type,
		    u8 port);
int lm_hw_pm_wr_reg(pm_device_t pdev,
		    u32 addr, u32 data_hi, u32 data_low,
		    u8 reg_type, u8 port);
int get_speed(struct bcm_amac_priv *privp);

#endif

#endif /*__BCM_AMAC_CORE_H__*/
