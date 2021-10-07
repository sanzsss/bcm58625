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

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/of_net.h>
#include <linux/rtnetlink.h>
#include <linux/ctype.h>

#include "bcm-amac-core.h"
#include "bcm-amac-enet.h"

#include "bcm-amac-ethtool.h"
#include "bcm-amac-regs.h"

#include "bcm-amac-ring-offload.h"

#ifdef RMO_SUPPORT
#include <brm.h>
#endif

#define TX_TIMEOUT (5 * HZ)

/* Netlink */
#define NETLINK_MAX_PAYLOAD 256
#define KERNEL_PID 0
#define DST_GROUP 1 /* to mcast group 1<<0 */

struct bcm_amac_cmd_line_param {
	char mac_addr[20];
	unsigned int lswap;
	bool bparsed_mac;
	struct sockaddr parsed_mac;
};

static struct bcm_amac_cmd_line_param cmdline_params = {
	"00:10:19:D0:B2:AC",
	0
};

static int bcm_amac_enet_remove(struct platform_device *pdev);
static int bcm_amac_enet_probe(struct platform_device *pdev);
static int bcm_amac_enet_init(struct bcm_amac_priv *privp);
static void bcm_amac_enet_term(struct bcm_amac_priv *privp);
static int bcm_amac_enet_start(struct net_device *ndev);
static int bcm_amac_enet_stop(struct net_device *ndev);
static int bcm_amac_enet_do_ioctl(struct net_device *dev,
				  struct ifreq *ifr, int cmd);
static int bcm_amac_get_ring_offload_ioctl(struct net_device *dev,
					   void __user *uaddr);
static int bcm_amac_set_ring_offload_ioctl(struct net_device *dev,
					   void __user *uaddr);
static int bcm_amac_get_dt_data(struct bcm_amac_priv *privp);
static int bcm_amac_enet_open(struct net_device *dev);
static int bcm_amac_enet_set_mac(struct net_device *dev, void *addr);
#ifdef RMO_SUPPORT
static void bcm_amac_rx_task(void *data, u8 cos, u32 num_pkts,
			     struct sk_buff **skbp);
#else
static void bcm_amac_tx_task(struct work_struct *w);
#endif
static int bcm_amac_enet_close(struct net_device *dev);
static int bcm_amac_enet_hard_xmit(struct sk_buff *skb, struct net_device *dev);
static void bcm_amac_enet_tx_timeout(struct net_device *dev);
static int bcm_amac_enet_rx_poll(struct napi_struct *napi, int quota);
static struct net_device_stats *bcm_amac_enet_get_stats(struct net_device *dev);

static const struct net_device_ops bcm_amac_enet_ops = {
	.ndo_open = bcm_amac_enet_open,
	.ndo_stop = bcm_amac_enet_close,
	.ndo_start_xmit = bcm_amac_enet_hard_xmit,
	.ndo_tx_timeout = bcm_amac_enet_tx_timeout,
	.ndo_get_stats = bcm_amac_enet_get_stats,
	.ndo_set_rx_mode = bcm_amac_set_rx_mode,
	.ndo_do_ioctl = bcm_amac_enet_do_ioctl,
	.ndo_set_mac_address = bcm_amac_enet_set_mac,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_change_mtu = eth_change_mtu
};

/* bcm_amac_enet_get_stats() - Get device status
 * @dev: net device pointer
 *
 * Returns: network statistics
 */
static struct net_device_stats *bcm_amac_enet_get_stats(struct net_device *ndev)
{
	struct bcm_amac_priv *privp;
#ifdef RMO_SUPPORT
	struct brm_if_stats if_stats;
#endif

	privp = netdev_priv(ndev);
#ifdef AMAC_DEBUG
	bcm_amac_print_mib_counters(privp);
	dump_pm_regs(privp);
#endif

#ifdef RMO_SUPPORT
	/*FIXME:*/
	brm_if_statistic_get(privp->pdev, BRM_ETH, privp->amac_id, &if_stats);
#endif

#ifdef PRBS_LOOPBACK
	if (IS_PORTMACRO_ENABLED)
		prbs_test_per_lane(privp, privp->amac_id);
#endif

#ifdef AMAC_DEBUG
	pr_emerg("PortMacro Status1:%#X, EEE Cfg:%#X\n",
		 readl(privp->hw.reg.eth_config_status),
		 readl(privp->hw.reg.eth_config_status + 0x4));

	if (privp->amac_id == 3)
		pr_emerg("EGPHY Status1:%#X",
			 readl(privp->hw.reg.eth_config_status + 0x8));
#endif

	return &ndev->stats;
}

#ifdef EXTRA_CODE
/**
 * bcm_amac_enet_parse_mac_addr() - parse the mac address
 * @macstr - string to be parsed
 * @parsedmac - parsed mac address in hex
 */
static void bcm_amac_enet_parse_mac_addr(char *macstr, char *parsedmac)
{
	int i, j;
	unsigned char result, value;

	for (i = 0; i < ETH_ALEN; i++) {
		result = 0;

		if (i != (ETH_ALEN - 1) && *(macstr + 2) != ':')
			return;

		for (j = 0; j < 2; j++) {
			if (!isxdigit(*macstr))
				return;
			value = isdigit(*macstr) ?
			    *macstr - '0' : toupper(*macstr) - 'A' + 10;
			if (value > 16)
				return;
			result = result * 16 + value;
			macstr++;
		}
		macstr++;
		parsedmac[i] = result;
	}
}
#endif

/* bcm_amac_enet_start() - Initialize core, phy,mac address etc.
 * @privp: device info pointer
 *
 * @Returns: 0 or error
 */
static int bcm_amac_enet_init(struct bcm_amac_priv *privp)
{
	int rc;
	struct sockaddr parsed_mac;
	bool ret;

	/* parse cmd line and set mac address */
	if (!cmdline_params.bparsed_mac) {
		ret = mac_pton(cmdline_params.mac_addr, parsed_mac.sa_data);
		if (ret) {
			cmdline_params.bparsed_mac = 1;
			rc = bcm_amac_enet_set_mac(privp->ndev,
						   (void *)&parsed_mac);
			if (rc)
				return rc;
		} else {
			netdev_err(privp->ndev, "Error parsing MAC address\n");
			rc = -EFAULT;
		}
	}

	if (IS_UNIMAC_PATH) {
		/* Initialize the PHY's */
		rc = bcm_amac_gphy_init(privp);
		if (rc != 0) {
			dev_err(&privp->pdev->dev, "%s: PHY Init failed\n",
				__func__);
			return rc;
		}

		/* Disable EGPHY & Unimac to prevent CTF data by reset time
		 * as Unimac Tx & Rx are On by default
		 */
		bcm_amac_gphy_shutdown(privp);
	}
	/* This can't be done before bcm_amac_core_init():
	 * bcm_amac_unimac_enable(privp, 0);
	 */
	rc = bcm_amac_core_init(privp);
	if (rc != 0) {
		dev_err(&privp->pdev->dev, "core init failed!\n");
		goto err_amac_enet_init;
	}

#ifdef RMO_SUPPORT
	/* Hack: Disable before brm_if_disable is ready
	 * brm_if_disable(privp->pdev, BRM_ETH, privp->amac_id,
	 *	       BRM_COS_ALL, BRM_BIDIR);
	 */
#else
	/* Disable RX and TX DMA just in case it was enabled earlier */
	rc = bcm_amac_enable_rx_dma(privp, false);
	if (rc) {
		dev_err(&privp->pdev->dev, "Rx DMA config failed!\n");
		goto err_amac_enet_init;
	}

	rc = bcm_amac_enable_tx_dma(privp, false);
	if (rc) {
		dev_err(&privp->pdev->dev, "Tx DMA config failed!\n");
		goto err_amac_enet_init;
	}
#endif

	return rc;

err_amac_enet_init:
	if (IS_UNIMAC_PATH)
		bcm_amac_gphy_exit(privp);

	return rc;
}

static void bcm_amac_enet_term(struct bcm_amac_priv *privp)
{
	if (IS_UNIMAC_PATH)
		bcm_amac_gphy_exit(privp);
}

static int bcm_amac_enet_stop(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (privp->state < AMAC_STARTED)
		return -1;

	if (!privp->rings_offloaded)
		netif_stop_queue(ndev);

	/* Shutdown PHY(s) */
	if (IS_UNIMAC_PATH)
		bcm_amac_gphy_shutdown(privp);

	/* Disable RX NAPI */
	napi_disable(&privp->napi);
#ifdef RMO_SUPPORT
	brm_if_disable(privp->pdev, BRM_ETH, privp->amac_id,
		       BRM_COS_ALL, BRM_BIDIR);
#else
	/* Stop DMA */
	bcm_amac_dma_stop(privp);
#endif

	kfree(privp->dma.rx_skb_list);
	privp->dma.rx_skb_list = NULL;

	if (!privp->rings_offloaded)
		netif_tx_disable(ndev);

	if (!privp->rings_offloaded)
		netif_carrier_off(ndev);

	privp->state = AMAC_STOPPED;
	return 0;
}

/* bcm_amac_tx_task() - Packet transmission routing.
 * @data: device info pointer
 *
 * This api is registered with the tx tasklet. The task performs the
 * transmission of packets if the tx is free
 *
 * Returns: none
 */
#ifdef RMO_SUPPORT
static void bcm_amac_tx_task(unsigned long data)
#else
static void bcm_amac_tx_task(struct work_struct *w)
#endif
{
	struct bcm_amac_priv *privp;

#ifdef RMO_SUPPORT
	privp = (struct bcm_amac_priv *)data;
#else
	privp = container_of(w, struct bcm_amac_priv, tx_work);
#endif

#ifndef RMO_SUPPORT
	/* If TX DMA is busy return */
	if (atomic_read(&privp->dma.tx_dma_busy) != BCM_AMAC_DMA_FREE)
		return;

	bcm_amac_tx_clean(privp);
#endif
	bcm_amac_tx_send_packet(privp);
	if (unlikely(netif_queue_stopped(privp->ndev)) &&
	    (kfifo_avail(&privp->dma.txfifo) >= sizeof(struct sk_buff *)))
		netif_wake_queue(privp->ndev);
}

/* bcm_amac_enet_rx_poll() - Packet reception routine
 * @napi: napi structure pointer
 * @quota: quota info
 *
 * This NAPI RX routine gets the received packets and sends it to the upper
 * layers. IT strips off any Broadcom Tags found in the packet. It also
 * updates the packet stats.
 * It enables the RX interrupt based on the quota.
 *
 * Returns: quota used
 */
static int bcm_amac_enet_rx_poll(struct napi_struct *napi, int quota)
{
	struct bcm_amac_priv *privp =
		container_of(napi, struct bcm_amac_priv, napi);
	int used = 0;
	struct sk_buff *skb;
#ifndef RMO_SUPPORT
	int len;
	void *bufp;
	struct amac_dma_priv *dmap = &privp->dma;
	u32 end_slot;
#endif
	static int last_used;
	static int prev_used[8] = {0};
	static int prev_idx;

	if (!privp)
		return -EINVAL;

#ifdef RMO_SUPPORT
	used = brm_rx_available_pkt_slots(privp->pdev, BRM_ETH,
					  privp->amac_id,
					  BRM_COS_ALL);
	if (used > quota)
		used = quota;
	for (prev_idx = 0; prev_idx < used; prev_idx++) {
		u32 num_pkts = 1;

		brm_rx_pkt(privp->pdev, BRM_ETH, privp->amac_id,
			   BRM_COS_ALL, &skb, &num_pkts);
	   netif_receive_skb(skb);
	   /* TODO: Update Rx stats & Is it possible to convey multiple packets
	    * to kernel N/W stack at once?
	    */
	}

#else
	end_slot = readl(privp->hw.reg.amac_core + GMAC_DMA_RX_STATUS0_REG);
	end_slot &= D64_PTR_MASK;
	end_slot -= dmap->rx.base_addr & D64_PTR_MASK;
	end_slot /= sizeof(struct amac_dma64_desc);

	while (used < quota && dmap->rx.index != end_slot) {
		/* Check and retrieve rx packets */
		len = bcm_amac_dma_get_rx_data(privp, &skb);
		if (len > 0) {
			/* Check frame length and discard if invalid*/
			if (unlikely(len < ETH_ZLEN)) {
				netdev_warn(privp->ndev,
					    "bad frame: len=%i\n", len);

				dev_kfree_skb_any(skb);

				/* Update error stats */
				privp->ndev->stats.rx_dropped++;
				continue;
			}
			/* Process the packet */
			bufp = skb->data;

			/* Update remainder of socket buffer information */
			skb_put(skb, len);

			skb->dev = privp->ndev;
			skb->protocol = eth_type_trans(skb, privp->ndev);
			skb->ip_summed = CHECKSUM_NONE;

			/* Update Stats */
			privp->ndev->stats.rx_bytes += len;
			privp->ndev->stats.rx_packets++;
			if (is_multicast_ether_addr((char *)bufp))
				privp->ndev->stats.multicast++;

			/* Pass the packet up for processing */
			netif_receive_skb(skb);

			used++;
		} else if (len == 0) {
			break; /* no frames to process */
		} else if (len == -EBADMSG) {
			/* Update error stats */
			privp->ndev->stats.rx_dropped++;
			privp->ndev->stats.rx_length_errors++;

			if (netif_msg_rx_err(privp) && net_ratelimit())
				netdev_err(privp->ndev,
					   "rx frame length err, used=%d\n",
					   used);

			continue;
		} else {
			/* Error retriving frame */

			/* Update error stats */
			privp->ndev->stats.rx_dropped++;
			privp->ndev->stats.rx_fifo_errors++;

			if (netif_msg_rx_err(privp) && net_ratelimit())
				netdev_err(privp->ndev,
					   "rx skb alloc err, used=%d\n", used);

			/* Don't try to read any more frames
			 * just drop out of the loop
			 */
			break;
		}
	}
#endif

	/* If quota not fully consumed, exit polling mode */
	if (likely(used < quota)) {
		napi_complete(napi);

#ifdef RMO_SUPPORT
		brm_rx_poll_mode_set(privp->pdev, BRM_ETH,
				     privp->amac_id,
				     false);
#else
		/* Enable RX Interrupt */
		bcm_amac_enable_intr(privp, BCM_AMAC_DIR_RX, true);
#endif
	}

	last_used = used;
	prev_used[prev_idx++ % 8] = used;

	return used;
}

#ifdef RMO_SUPPORT
static void bcm_amac_rx_task(void *data, u8 cos, u32 num_pkts,
			     struct sk_buff **skbp)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)data;
	int j, i;

	/* 1st convey the existing packets to N/W stack */
	for (j = 0; j < num_pkts; j++) {
		/* set skb->dev */
		skbp[j]->dev = privp->ndev;

		/* process metadata and set skb->len */
		skbp[j]->len = *((u32 *)(skbp[j]->data)) & 0xFFFF;

		pr_debug("%s: ndev %p received skb 0x%p len 0x%x\n",
			 __func__, privp->ndev, skbp[j], skbp[j]->len);

		for (i = 0; i < skbp[j]->len; i++) {
			if (!(i % 16))
				pr_debug("\n%s: 0x", __func__);

			pr_debug("%02x", (unsigned char)(skbp[j]->data[i]));
		}

		netif_rx(skbp[j]);
	}

	/* set NAPI polling mode & tell BRM driver to disable interrupts */
	brm_rx_poll_mode_set(privp->pdev, BRM_ETH,
			     privp->amac_id, true);
}
#endif

/* Joey hack: setup DMA engines */
#define SPINWAIT(exp, us) { \
	uint countdown = (us) + 9; \
	while ((exp) && (countdown >= 10)) {\
		udelay(10); \
		countdown -= 10; \
	} \
}

#ifdef EXTRA_CODE
static int bcm_amac_dma_setup(struct bcm_amac_priv *privp)
{
	u32 control;
	u32 intr_status, status;
	u32 intr_mask;
	struct amac_dma_priv *dma_p = &privp->dma;

	/* Rx DMAs
	 *  initailize the DMA channel
	 */
	writel((u32)dma_p->rx.base_addr,
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_ADDR_LO_REG));

	writel((u32)(dma_p->rx.base_addr >> 32), (privp->hw.reg.amac_core
					     + GMAC_DMA_RX_ADDR_HI_REG));

	/* now update the dma last descriptor, config as empty */
	writel(dma_p->rx.base_addr,
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_REG));

	bcm_amac_enable_rx_intr(privp, false);

	control = (readl(privp->hw.reg.amac_core +
			 GMAC_DMA_RX_CTRL_REG) &
		   D64_RC_AE) | D64_RC_RE;

	if ((privp->dmactrlflags & DMA_CTRL_RX_PEN) == 0)
		control |= D64_RC_PD;

	if (privp->dmactrlflags & DMA_CTRL_RX_ROC)
		control |= D64_RC_OC;

	/* These bits 20:18 (burstLen) of control register can be
	 * written but will take effect only if these bits are
	 * valid. So this will not affect previous versions
	 * of the DMA. They will continue to have those bits
	 */

	/* set to 0. */
	control &= ~D64_RC_BL_MASK;
	/* Keep default Rx burstlen */
	control |= readl(privp->hw.reg.amac_core +
			 GMAC_DMA_RX_CTRL_REG)
		& D64_RC_BL_MASK;
	control |= HWRXOFF << D64_RC_RO_SHIFT;

	/* Joey Hack: put in prefetch, prefetch_threshold, burst_en */
	control |= (0x3) << D64_RC_PT_SHIFT;
	control |= (0x3) << D64_RC_PC_SHIFT;
	control &= ~D64_RC_BL_MASK;
	control |= (0x3) << D64_RC_BL_SHIFT;

	writel(control,
	       privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_REG);

	/* Tx DMAs
	 *   initailize the DMA channel
	 */
	writel((u32)(unsigned long)(&((struct amac_dma64_desc *)
				      (dma_p->tx.base_addr))[0]),
	       (privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_LO_REG));
	writel((u32)((unsigned long)(&((struct amac_dma64_desc *)
				       (dma_p->tx.base_addr))[0]) >> 32),
	       (privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_HI_REG));

	/* Clear Rx/TX interrupt */
	intr_status = readl(privp->hw.reg.amac_core + GMAC_INT_STATUS_REG);
	intr_status &= (I_XI_ALL | I_RI);	/* Clear only TX interrupt(s) */
	writel(intr_status, (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));

	/* disable Rx/TX DMA Interrupts */
	intr_mask = readl(privp->hw.reg.amac_core + GMAC_INT_MASK_REG);
	intr_mask |= (I_XI_ALL | I_RI);
	writel(intr_mask,
	       (privp->hw.reg.amac_core + GMAC_INT_MASK_REG));

	/* These bits 20:18 (burstLen) of control register can be
	 * written but will take effect only if these bits are
	 * valid. So this will not affect previous versions
	 * of the DMA. They will continue to have those bits set to 0.
	 */
	control = readl(privp->hw.reg.amac_core +
			GMAC_DMA_TX_CTRL_REG);

	control |= D64_XC_XE;
#ifdef ENABLE_DMA_LOOPBACK
	/*enable loopback */
	control |= D64_XC_LE;
#else
	control &= ~D64_XC_LE;
#endif
	if ((privp->dmactrlflags & DMA_CTRL_TX_PEN) == 0)
		control |= D64_XC_PD;

	control |= 0x3 << D64_XC_BL_SHIFT;	/* Burst length of 3 */

	/* Joey Hack: put in prefetch, prefetch_threshold */
	control |= (0x3) << D64_XC_PT_SHIFT;
	control |= (0x3) << D64_XC_PC_SHIFT;

	writel(control,
	       (privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_REG));

	SPINWAIT(((status =
		   (readl(privp->hw.reg.amac_core +
			  GMAC_DMA_TX_STATUS0_REG)
		    & D64_XS0_XS_MASK)) != D64_XS0_XS_IDLE), 10000);
	return 0;
}
#endif

static int bcm_amac_enet_start(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int rc = 0;

#ifdef RMO_SUPPORT
	rc = brm_if_enable(privp->pdev, BRM_ETH, privp->amac_id,
			   BRM_COS_ALL, BRM_BIDIR);
	if (rc) {
		netdev_err(ndev, "Failed to start RMO Eth Interface:%d\n",
			   privp->amac_id);
		return rc;
	}
#else
	/* Register GMAC Interrupt */
	rc = devm_request_irq(&privp->pdev->dev, privp->hw.intr_num,
			      bcm_amac_isr, IRQF_SHARED, "amac_enet", privp);
	if (rc) {
		netdev_err(privp->ndev,
			   "IRQ request failed, irq=%i, err=%i\n",
			   privp->hw.intr_num, rc);
		return rc;
	}
	/* Start DMA */
	rc = bcm_amac_dma_start(privp);
	if (rc) {
		netdev_err(ndev, "Failed to start DMA\n");
		return rc;
	}
#endif

	/* if (IS_UNIMAC_PATH)
	 *	bcm_amac_unimac_enable(privp, 1);
	 */

	/* Clear Interrupts */
	writel(I_INTMASK, (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));
	/* Power up the PHY */
	if (IS_UNIMAC_PATH)
		bcm_amac_gphy_powerup(privp, true);
	napi_enable(&privp->napi);

	netif_start_queue(ndev);

	netif_carrier_on(ndev);

	privp->state = AMAC_STARTED;

	return rc;
}

/* bcm_amac_enet_open() - Ethernet interface open routine
 * @ndev: network device pointer
 *
 * The routine is called when the Ethernet interface is opened.
 * This stats the DMA's, enables the RX (NAPI), powers up the PHY,
 * starts up the TX queue etc.
 *
 * Returns: '0' for success or the error number
 */
static int bcm_amac_enet_open(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int rc;
#ifdef RMO_SUPPORT
	int j;
	struct brm_if_config if_params;
	struct amac_dma_priv *dma_p;
	struct platform_device *pdev = privp->pdev;
#endif

	/* Allocate a TX fifo to stash to hold skb pointers */
	rc = kfifo_alloc(&privp->dma.txfifo,
			 AMAC_DMA_TX_MAX_QUEUE_LEN * sizeof(void *),
			 GFP_FLAGS);
	if (rc) {
		netdev_err(ndev,
			   "cannot alloc tx fifo, err=%i\n", rc);
		return rc;
	}
#ifdef RMO_SUPPORT
	pr_info("%s: ndev->ifindex %d\n", __func__, privp->amac_id);
	ndev->ifindex = 3;
	pr_info("%s: forcing ndev->ifindex to %d\n", __func__, privp->amac_id);

	/* JoeyHack: the RMO_EN bit has to be set before brm_if_create */

	/* JoeyHack: Allocate rx/tx descriptor rings */
	rc = amac_dma_tx_alloc(privp);
	if (rc != 0) {
		dev_err(&pdev->dev,
			"%s: Failed to alloc amac tx ring\n", __func__);
		goto err_free_kfifo;
	}

	rc = amac_dma_rx_alloc(privp);
	if (rc != 0) {
		dev_err(&pdev->dev,
			"%s: Failed to alloc amac rx ring\n", __func__);
		goto err_free_kfifo;
	}
	dma_p = &privp->dma;

	rc = bcm_amac_dma_setup(privp);
	if (rc) {
		netdev_err(ndev, "Failed to setup dma engine for Eth If:%d\n",
			   privp->amac_id);
		goto err_free_kfifo;
	}

	/*TODO: some parameters are not known as of now
	 *   Need to have multi-cos support
	 */
	memset(&if_params, 0, sizeof(struct brm_if_config));
	if_params.tx_cb                = bcm_amac_tx_clean;
	if_params.rx_cb                = bcm_amac_rx_task;
	if_params.buff_pool_id         = 0; /* FIXME */
	if_params.lro_buff_pool_id     = 1; /* FIXME */
	if_params.num_cos              = 1;/*BRM_COS_MAX*/
	if_params.num_lro_cos          = 0;/*BRM_COS_MAX*/
	if_params.buffs_per_packet     = 1;
	if_params.lro_buffs_per_packet = 1;
	if_params.amac_egress_offload = BRM_OFFLOAD_OFF;
	if_params.egress_offload = BRM_OFFLOAD_OFF;
	if_params.ingress_offload = BRM_OFFLOAD_OFF;
	if_params.owner = (void *)privp;

	for (j = 0; j < BRM_COS_MAX; j++) {
		if_params.amac_ring_tx_base[j]     = dma_p->tx.base_addr;
		if_params.amac_ring_rx_base[j]     = dma_p->rx.base_addr;
		if_params.lro_amac_ring_rx_base[j] = 0; /*FIXME*/
		if_params.amac_ring_tx_size[j]     = DMA_TX_DESC_NUM;
		if_params.amac_ring_rx_size[j]     = DMA_RX_DESC_NUM;
		if_params.lro_amac_ring_rx_size[j] = DMA_RX_DESC_NUM;
	}

	rc = brm_if_create(pdev, BRM_ETH, privp->amac_id, &if_params);
	if (rc) {
		netdev_err(ndev,
			   "%s: Failed to allocate BRM Eth interface:%d\n",
			   __func__, privp->amac_id);
		goto err_free_kfifo;
	}

	rc = brm_if_enable(privp->pdev, BRM_ETH, ndev->ifindex,
			   BRM_COS_ALL, BRM_BIDIR);
	if (rc) {
		netdev_err(ndev, "Failed to start RMO Eth Interface:%d\n",
			   privp->amac_id);
		goto err_free_kfifo;
	}
#else

	privp->state = AMAC_OPENED;
	/* Start DMA */
	rc = bcm_amac_enet_start(ndev);
	if (rc) {
		netdev_err(ndev, "bcm_amac_enet_start Failed amac_id:%d\n",
			   privp->amac_id);
		goto err_enet_start;
	}
#endif

	/* Power-up the PHY(s) */

	return rc;

err_enet_start:
	bcm_amac_dma_stop(privp);
	goto err_free_kfifo;

err_free_kfifo:
	if (IS_UNIMAC_PATH)
		bcm_amac_unimac_enable(privp, false);
	napi_disable(&privp->napi);
	kfifo_free(&privp->dma.txfifo);
	netdev_err(ndev, "%s, open failed!\n", __func__);

	return rc;
}

/* bcm_amac_enet_close() - Ethernet interface close routine
 * @ndev: network device pointer
 *
 * The routine is called when the Ethernet interface is closed or disabled.
 * This stops the DMA, disables interrupts, switches off the PHY, disables
 * NAPI routine etc.
 *
 * Returns: '0'
 */
static int bcm_amac_enet_close(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!privp)
		return -1;

	if (privp->state < AMAC_OPENED)
		return -1;

	bcm_amac_enet_stop(ndev);

#ifdef RMO_SUPPORT
	brm_if_delete(privp->pdev, BRM_ETH, privp->amac_id);
#else
	devm_free_irq(&privp->pdev->dev, privp->hw.intr_num, privp);

	if (privp->dma.rx.raw_descp) {
		dma_free_coherent(&privp->pdev->dev,
				  privp->dma.rx.alloc_size,
				  privp->dma.rx.raw_descp,
				  privp->dma.rx.raw_addr);
		privp->dma.rx.descp = NULL;
		privp->dma.rx.raw_descp = NULL;
		privp->dma.rx.base_addr = 0;
		privp->dma.rx.raw_addr = 0;
	}
	if (privp->dma.tx.raw_descp) {
		dma_free_coherent(&privp->pdev->dev,
				  privp->dma.tx.alloc_size,
				  privp->dma.tx.raw_descp,
				  privp->dma.tx.raw_addr);

		privp->dma.tx.raw_descp = NULL;
		privp->dma.tx.descp = NULL;
		privp->dma.tx.raw_addr = 0;
		privp->dma.tx.base_addr = 0;
	}

	kfree(privp->dma.rx_skb_list);
	privp->dma.rx_skb_list = NULL;

	kfree(privp->dma.tx_skb_list);
	privp->dma.tx_skb_list = NULL;
#endif

	if (IS_UNIMAC_PATH)
		bcm_amac_unimac_enable(privp, false);

	kfifo_free(&privp->dma.txfifo);
	privp->state = AMAC_CLOSED;
	return 0;
}

/* bcm_amac_enet_set_mac() - Assigns the mac address to the interface.
 * @dev: network device pointer
 * @addr: mac address
 *
 * Returns: '0' or error
 */
static int bcm_amac_enet_set_mac(struct net_device *ndev, void *addr)
{
	int rc;
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	rc = eth_mac_addr(ndev, addr);
	if (rc) {
		netdev_err(ndev, "cannot setup MAC, err=%i\n", rc);
		return rc;
	}

	memcpy(privp->cur_etheraddr.sa_data, ndev->dev_addr, 6);

	return 0;
}

/* bcm_amac_enet_hard_xmit() - hard transmit routine
 * @skb: skb buffer pointer with data to be transmitted
 * @ndev: network device pointer.
 *
 * The hard transmit routine is called by the upper layers to transmit data.
 * The data is part of the skb pointer. The interface adds broadcom tags if
 * enabled, inserts the skb into the internal transmit queue and schedules
 * the transmit task to run.
 *
 * Returns: NETDEV_TX_OK
 */
static int bcm_amac_enet_hard_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct bcm_amac_priv *privp;
	int len;
	int rc;

	rc = NETDEV_TX_OK;
	privp = netdev_priv(ndev);

	if (privp->rings_offloaded)
		return NETDEV_TX_OK;

#ifdef AMAC_DEBUG
	pr_debug("%s: start to transmit pkt skb %p length %d\n",
		 __func__, (void *)skb, skb->len);
#endif
	if (unlikely(skb->len < ETH_ZLEN)) {
		/* Clear the padded memory to avoid 'etherleak'
		 * vulnerability
		 */
		memset(skb->data + skb->len, 0, (ETH_ZLEN - skb->len));
		skb->len = ETH_ZLEN;
	}

	/* Insert skb pointer into fifo */
	len = kfifo_in_locked(&privp->dma.txfifo, (unsigned char *)&skb,
			      sizeof(skb), &privp->lock);
	if (unlikely(len != sizeof(skb))) {
		/* Not enough space, which shouldn't happen since the queue
		 * should have been stopped already.
		 */
		netif_stop_queue(ndev);
		netdev_info(privp->ndev,
			    "xmit called with no tx desc avail!");

		ndev->stats.tx_fifo_errors++;

		rc = NETDEV_TX_BUSY;
		goto err_enet_hard_xmit;
	}

	if (unlikely(kfifo_avail(&privp->dma.txfifo) <= sizeof(skb)))
		netif_stop_queue(ndev);

#ifndef RMO_SUPPORT
	schedule_work(&privp->tx_work);
#else
	tasklet_schedule(&privp->tx_tasklet);
#endif

	/* Update stats */
	if (is_multicast_ether_addr((char *)skb->data))
		ndev->stats.multicast++;

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	return rc;

err_enet_hard_xmit:
	/* Update stats */
	ndev->stats.tx_dropped++;
	ndev->stats.tx_fifo_errors++;

	return rc;
}

/* bcm_amac_enet_tx_timeout() - Transmit timeout routine
 * @ndev - network device pointer
 */
static void bcm_amac_enet_tx_timeout(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	/* Reset outside of interrupt context, to avoid request_irq
	*  in interrupt context
	*/
	schedule_work(&privp->tx_timeout_task);
}

static void bcm_amac_tx_timeout_task(struct work_struct *work)
{
	struct bcm_amac_priv *privp = container_of(work, struct bcm_amac_priv,
							tx_timeout_task);
	struct net_device *ndev = privp->ndev;

	rtnl_lock();
	if (netif_running(ndev)) {
		ndev->stats.tx_errors++;
		bcm_amac_enet_close(ndev);
		bcm_amac_enet_open(ndev);
	}
	rtnl_unlock();
}

static int bcm_amac_get_ring_offload_ioctl(struct net_device *dev,
					   void __user *uaddr)
{
	struct bcm_amac_priv *priv = netdev_priv(dev);
	struct bcm_amac_ring_offload_get bro;
	struct bcm_amac_ring_offload_get *pbro;
	struct amac_dma_priv *dma_p = &priv->dma;
	struct ring_info *ring;

	if (copy_from_user(&bro, uaddr,
			   sizeof(struct bcm_amac_ring_offload_get))) {
		netdev_err(dev, "Failed to copy BRO ioctl data from user\n");
		return -EFAULT;
	}

	if (bro.version != BCM_AMAC_RING_OFFLOAD_VERSION) {
		netdev_err(dev, "Invalid BRO ioctl data version: expected 0x%x got %x\n",
			   BCM_AMAC_RING_OFFLOAD_VERSION, bro.version);
		return -EINVAL;
	}

	/* One ring pair, tx/rx */
	bro.ring_count = 2;
	bro.register_base = priv->register_base;

	if (bro.get_op == BCM_AMAC_RING_OFFLOAD_GET_OP_RING_COUNT) {
		if (copy_to_user(uaddr, &bro,
				 sizeof(struct bcm_amac_ring_offload_get))) {
			netdev_err(dev, "Failed to copy BRO ioctl data to user\n");
			return -EFAULT;
		}

		return 0;
	}

	if (bro.get_op != BCM_AMAC_RING_OFFLOAD_GET_OP_RING_INFO) {
		netdev_err(dev, "Invalid BRO ioctl command: 0x%x\n",
			   bro.get_op);
		return -EINVAL;
	}

	pbro = kmalloc((sizeof(struct bcm_amac_ring_offload_get) +
			(sizeof(struct ring_info) * bro.ring_count)),
		       GFP_KERNEL);
	if (!pbro) {
		netdev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	memcpy(pbro, &bro, sizeof(struct bcm_amac_ring_offload_get));

	/* Initialize the RX ring */
	ring = &pbro->rings[0];
	ring->size        = DMA_RX_DESC_NUM;
	ring->nr_pages    = DMA_RX_DESC_NUM * sizeof(struct amac_dma64_desc) /
								PAGE_SIZE;
	if (ring->nr_pages == 0) {
		netdev_err(dev,
			   "Error, RX Descriptor count too small causing page count to be 0\n");
		return -EINVAL;
	}
	ring->type        = RING_TYPE_RX;
	ring->mapping     = (u64)dma_p->rx.base_addr;
	ring->prod_offset = GMAC_DMA_RX_PTR_REG;
	ring->cons_offset = GMAC_DMA_RX_STATUS0_REG;

	/* Initialize the TX ring */
	ring = &pbro->rings[1];
	ring->size        = DMA_TX_DESC_NUM;
	ring->nr_pages    = DMA_TX_DESC_NUM * sizeof(struct amac_dma64_desc) /
								PAGE_SIZE;
	if (ring->nr_pages == 0) {
		netdev_err(dev,
			   "Error, TX Descriptor count too small causing page count to be 0\n");
		return -EINVAL;
	}
	ring->type        = RING_TYPE_TX;
	ring->mapping     = (u64)dma_p->tx.base_addr;
	ring->prod_offset = GMAC_DMA_TX_PTR_REG;
	ring->cons_offset = GMAC_DMA_TX_STATUS0_REG;

	if (copy_to_user(uaddr, pbro,
			 (sizeof(struct bcm_amac_ring_offload_get) +
			  (sizeof(struct ring_info) * bro.ring_count)))) {
		netdev_err(dev, "Failed to copy BRO ioctl data to user\n");
		kfree(pbro);
		return -EFAULT;
	}

	kfree(pbro);
	return 0;
}

static int bcm_amac_set_ring_offload_ioctl(struct net_device *dev,
					   void __user *uaddr)
{
	struct bcm_amac_ring_offload_set bro;
	struct bcm_amac_priv *privp = netdev_priv(dev);

	if (copy_from_user(&bro, uaddr,
			   sizeof(struct bcm_amac_ring_offload_set))) {
		netdev_err(dev, "Failed to copy BRO ioctl data from user\n");
		return -EFAULT;
	}

	if (bro.version != BCM_AMAC_RING_OFFLOAD_VERSION) {
		netdev_err(dev, "Invalid BRO ioctl data version: expected 0x%x got %x\n",
			   BCM_AMAC_RING_OFFLOAD_VERSION, bro.version);
		return -EINVAL;
	}

	switch (bro.set_op) {
	case BCM_AMAC_RING_OFFLOAD_SET_OP_STOP:
		netdev_dbg(privp->ndev,
			   "%s() bro.set_op %s\n",
			   __func__, "BCM_AMAC_RING_OFFLOAD_SET_OP_STOP");
		if ((privp->state == AMAC_STARTED ||
		     privp->state == AMAC_RESUMED) &&
		     !privp->rings_offloaded) {
			privp->rings_offloaded = true;
			bcm_amac_enet_stop(dev);
		}
		break;

	case BCM_AMAC_RING_OFFLOAD_SET_OP_START:
		netdev_dbg(privp->ndev,
			   "%s() bro.set_op %s\n",
			   __func__, "BCM_AMAC_RING_OFFLOAD_SET_OP_START");
		if ((privp->state == AMAC_STOPPED ||
		     privp->state == AMAC_SUSPENDED) &&
		     privp->rings_offloaded) {
			bcm_amac_enet_start(dev);
			privp->rings_offloaded = false;
		}
		break;

	case BCM_AMAC_RING_OFFLOAD_SET_OP_RECV_MASK_ON:
		if ((privp->state == AMAC_STOPPED ||
		     privp->state == AMAC_SUSPENDED) &&
		     privp->rings_offloaded) {
			netdev_dbg(privp->ndev,
				   "%s() bro.set_op %s\n", __func__,
				   "BCM_AMAC_RING_OFFLOAD_SET_OP_RECV_MASK_ON");
			bcm_amac_enable_rx_dma(privp, 1);
			bcm_amac_enable_tx_dma(privp, 1);
			if (IS_UNIMAC_PATH)
				bcm_amac_gphy_powerup(privp, true);
		}
		break;

	case BCM_AMAC_RING_OFFLOAD_SET_OP_RECV_MASK_OFF:
		if ((privp->state == AMAC_STOPPED ||
		     privp->state == AMAC_SUSPENDED) &&
		     privp->rings_offloaded) {
			netdev_dbg(privp->ndev,
				   "%s() bro.set_op %s\n",
				   __func__,
				   "BCM_AMAC_RING_OFFLOAD_SET_OP_RECV_MASK_OFF");
			bcm_amac_enable_rx_dma(privp, 0);
		}
		break;

	default:
		netdev_err(dev, "Invalid BRO ioctl command: 0x%x\n",
			   bro.set_op);
		return -EINVAL;
	}

	return 0;
}

/* bcm_amac_enet_do_ioctl() - ioctl support in the driver
 * @ndev: network device
 * @ifr: ioctl data pointer
 * @cmd: ioctl cmd
 *
 * Returns: '0' or error
 */
static int bcm_amac_enet_do_ioctl(struct net_device *ndev,
				  struct ifreq *ifr, int cmd)
{
	int rc;
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!netif_running(ndev))
		return -EINVAL;

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		if (privp->port.ext_port.phydev)
			rc = phy_mii_ioctl(privp->port.ext_port.phydev,
					   ifr, cmd);
		else
			rc = -ENODEV;
		break;
	break;
	case GIORINGOFFLOAD:
		if (!netif_running(ndev))
			return -EAGAIN;
		return bcm_amac_get_ring_offload_ioctl(ndev,
						ifr->ifr_ifru.ifru_data);
	case SIORINGOFFLOAD:
		if (!netif_running(ndev)) {
			return -EAGAIN;
		} else {
			return bcm_amac_set_ring_offload_ioctl(ndev,
						ifr->ifr_ifru.ifru_data);
		}

	default:
		rc = -EOPNOTSUPP;
	}

	return rc;
}

/* bcm_amac_get_dt_data() - Retrieve data from the device tree
 * @pdev: platform device data structure
 * @privp: driver privagte data structure
 *
 * Returns: '0' or error
 */
static int bcm_amac_get_dt_data(struct bcm_amac_priv *privp)
{
	struct platform_device *pdev = privp->pdev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *iomem;
	struct device_node *phy_node = NULL;
	static void __iomem *ctf_regs, *amac3_io_ctrl;
	static void __iomem *eth_config_status;
	const char *mac_addr;
	const unsigned char *local_mac_addr;
	int rc, len;

	if (!of_device_is_compatible(np, "brcm,amac-enet-v3")) {
		amac_version = 2;
		unimac_port_num = 0;
		privp->amac_id  = 0;
		privp->b_unimac = 1;
		portmacro_enable = false;
	} else {
		amac_version = 3;
		unimac_port_num = UNIMAC_PORT_NUM_PEG;
#ifdef PORTMACRO_ENABLE
		portmacro_enable = true;
#endif
	}

	/* GMAC Core register */
	iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "core_base");
	privp->hw.reg.amac_core = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.amac_core)) {
		dev_err(&privp->pdev->dev,
			"%s: ioremap of amac_core failed\n",
			__func__);
		return PTR_ERR(privp->hw.reg.amac_core);
	}
	privp->register_base = (u64)iomem->start;

	if (amac_version == 3) {
		iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "eth_config_status");
		privp->hw.reg.eth_config_status = devm_ioremap_resource(
						&pdev->dev, iomem);
		if (IS_ERR(privp->hw.reg.eth_config_status)) {
			dev_dbg(&privp->pdev->dev,
				"%s: ioremap of eth_config_status failed\n",
				__func__);
			privp->hw.reg.eth_config_status = eth_config_status;
		} else {
			eth_config_status = privp->hw.reg.eth_config_status;
		}

		rc = of_property_read_u8(pdev->dev.of_node, "amac_id",
					 &privp->amac_id);
		if (rc != 0) {
			dev_err(&privp->pdev->dev,
				"No amac_id in DT node: exiting...\n");
			return -EINVAL;
		}
	}
	privp->b_unimac = -1; /* Default value */
	/* Don't move this to change order of parsing parameters */
	rc = of_property_read_s32(pdev->dev.of_node, "b_unimac",
				  &privp->b_unimac);
	if ((rc != 0) || (privp->b_unimac > 1))
		privp->b_unimac = -1;

	if (!IS_UNIMAC_PORT)
		privp->b_unimac = -1;
	/* AMAC IO Ctrl register */
	iomem = platform_get_resource_byname(pdev,
					     IORESOURCE_MEM, "amac_idm_base");
	privp->hw.reg.amac_idm_base = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.amac_idm_base)) {
		dev_err(&privp->pdev->dev,
			"AMAC%d: ioremap of amac_idm_base failed\n",
			privp->amac_id);
		if (amac3_io_ctrl && (IS_UNIMAC_PORT))
			privp->hw.reg.amac_idm_base = amac3_io_ctrl;
		else
			return PTR_ERR(privp->hw.reg.amac_idm_base);
	}

	/* optional RGMII base register */
	iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					     "rgmii_base");
	if (iomem) {
		privp->hw.reg.rgmii_regs =
				devm_ioremap_resource(&pdev->dev, iomem);
		if (IS_ERR(privp->hw.reg.rgmii_regs)) {
			dev_err(&privp->pdev->dev,
				"%s: ioremap of rgmii failed\n",
				__func__);
			return PTR_ERR(privp->hw.reg.rgmii_regs);
		}
	}

	if (amac_version != 3) {
		privp->switch_mode = of_property_read_bool(np,
				"brcm,enet-switch-mode");

		if (!privp->switch_mode) {
			/* optional SWITCH GLOBAL CONFIG register
			 * This is only required for switch-by-pass
			 * mode to connect IMP port to the PHY
			 */
			iomem = platform_get_resource_byname(pdev,
							     IORESOURCE_MEM,
							"switch_global_base");
			if (iomem) {
				privp->hw.reg.switch_global_cfg =
					devm_ioremap_resource(&pdev->dev,
							      iomem);
				if (IS_ERR(privp->hw.reg.switch_global_cfg)) {
					dev_err(&privp->pdev->dev,
						"%s: ioremap of switch_global_cfg failed\n",
						__func__);
					return PTR_ERR(privp->hw.reg.
					      switch_global_cfg);
				}
			}
		}

		/* optional CRMU IO PAD CTRL register */
		iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "crmu_io_pad_ctrl");
		if (iomem) {
			privp->hw.reg.crmu_io_pad_ctrl =
				devm_ioremap_resource(&pdev->dev, iomem);
			if (IS_ERR(privp->hw.reg.crmu_io_pad_ctrl)) {
				dev_err(&privp->pdev->dev,
					"%s: ioremap of crmu_io_pad_ctrl failed\n",
					__func__);
				return PTR_ERR(privp->hw.reg.crmu_io_pad_ctrl);
			}
		}
		privp->b_unimac = 1;
	} else {  /* Pegasus code */
		iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "ctf_base");
		privp->hw.reg.ctf_regs = devm_ioremap_resource(&pdev->dev,
							       iomem);
		if (IS_ERR(privp->hw.reg.ctf_regs)) {
			dev_dbg(&privp->pdev->dev,
				"AMAC%d: ioremap of ctf_regs failed\n",
				privp->amac_id);
			privp->hw.reg.ctf_regs = ctf_regs;
		} else {
			ctf_regs = privp->hw.reg.ctf_regs;
		}

		iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "amac3_io_ctrl");
		privp->hw.reg.amac3_io_ctrl =
			devm_ioremap_resource(&pdev->dev, iomem);
		if (IS_ERR(privp->hw.reg.amac3_io_ctrl)) {
			dev_dbg(&privp->pdev->dev,
				"AMAC%d: ioremap of amac3_io_ctrl failed\n",
				privp->amac_id);
			privp->hw.reg.amac3_io_ctrl = amac3_io_ctrl;
		} else {
			amac3_io_ctrl = privp->hw.reg.amac3_io_ctrl;
		}

		/* AMAC3 port DT node */
		if (IS_UNIMAC_PORT)
			privp->hw.reg.amac_idm_base = amac3_io_ctrl;

		iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "apb2pbus_base");
		privp->hw.reg.apb2pbus_base =
			devm_ioremap_resource(&pdev->dev, iomem);
		if (IS_ERR(privp->hw.reg.apb2pbus_base)) {
			dev_dbg(&privp->pdev->dev,
				"AMAC%d: ioremap of apb2pbus_base failed\n",
				privp->amac_id);
			privp->hw.reg.apb2pbus_base = apb2pbus_base;
		} else {
			apb2pbus_base = privp->hw.reg.apb2pbus_base;
		}
	}

	/* Read Interrupt */
	privp->hw.intr_num = platform_get_irq(pdev, 0);
	if (privp->hw.intr_num == 0) {
		dev_err(&privp->pdev->dev,
			"%s: gmac0 interrupt not specified\n",
			__func__);
		return -EINVAL;
	}
	local_mac_addr = of_get_property(pdev->dev.of_node, "local-mac-address",
					 &len);
	if (local_mac_addr && len == 6) {
		memcpy(cmdline_params.parsed_mac.sa_data, local_mac_addr, 6);
		cmdline_params.bparsed_mac = 1;
	}
	rc = of_property_read_string(pdev->dev.of_node,
				     "mac-address", &mac_addr);
	/* Override local-mac-address by mac-address if it exists */
	if (rc == 0) {
		strcpy(cmdline_params.mac_addr, mac_addr);
		cmdline_params.bparsed_mac = 0;
	}

	/* AMAC handles the PHY for SoC's without an
	 * internal switch or 'switch-by-pass' mode in
	 * case of SoC's with switch.
	 *
	 * In both cases above, the PHY connects directly
	 * to the IMP port.
	 *
	 * With an internal switch involved all PHY
	 * handling will be done by the switch.
	 */
	if (IS_UNIMAC_PATH) {
		phy_node = of_parse_phandle(np, "phy-handle", 0);
		if (!phy_node) {
			dev_err(&privp->pdev->dev,
				"%s: phy-handle not specified\n",
				__func__);
			return -EINVAL;
		}
	}

	/* max-speed setting for the IMP port */
	rc = of_property_read_u32(np, "max-speed",
				  &privp->port.imp_port_speed);
	if (rc)
		privp->port.imp_port_speed = AMAC_PORT_DEFAULT_SPEED;

	/* Get internal / external PHY info */

	/* NOTE: 'External' only refers to the port type
	 * The PHY that is connected can be an internal PHY
	 * or an external PHY.
	 * IMP is considered 'internal' port.
	 */
	if (IS_UNIMAC_PATH) {
		privp->port.ext_port.phy_node = phy_node;

		privp->port.ext_port.phy_mode = of_get_phy_mode(np);
		if (privp->port.ext_port.phy_mode < 0) {
			dev_err(&privp->pdev->dev,
				"Invalid phy interface specified\n");
			return -EINVAL;
		}
	}

	privp->port.ext_port.lswap =
		of_property_read_bool(np, "brcm,enet-phy-lswap");

	privp->port.ext_port.pause_disable =
		of_property_read_bool(np, "brcm,enet-pause-disable");

	privp->port.ext_port.phy54810_rgmii_sync =
		of_property_read_bool(np, "brcm,enet-phy54810-rgmii-sync");

	privp->port.ext_port.phy54810_rgmii_lswap =
		of_property_read_bool(np, "brcm,enet-phy54810-rgmii-lswap");

	return 0;
}

/* bcm_amac_enet_probe() - driver probe function
 * @pdev: platform device pointer
 *
 * Returns: '0' or error
 */
static int bcm_amac_enet_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct bcm_amac_priv *privp;
	int rc;

	if (amac_version == 3) {
#ifdef RMO_SUPPORT
		if (!find_module("brm")) {
			pr_err("%s: BRM module not available exiting...\n",
			       __func__);
			return -EINVAL;
		}
#endif
	}
	/* Initialize driver resource */
	ndev = alloc_etherdev(sizeof(struct bcm_amac_priv));
	if (!ndev) {
		dev_err(&pdev->dev,
			"%s: Failed to allocate device\n", __func__);
		return -ENOMEM;
	}

#ifdef RMO_SUPPORT
	/* Hack: need extra headroom, put 64 byts for now */
	ndev->needed_headroom += (64);
#endif
	/* We actually need SHIM_HEADER_SIZE_BYTES=8 only but
	 * for 64bit alignment, better we reserve 64
	 */
	ndev->needed_headroom += (64);

	privp = netdev_priv(ndev);
	memset(privp, 0, sizeof(struct bcm_amac_priv));
	privp->pdev = pdev;
	privp->ndev = ndev;

	/* Read DT data */
	rc = bcm_amac_get_dt_data(privp);
	if (rc != 0) {
		dev_err(&pdev->dev,
			"%s: Failed to get platform data\n", __func__);
		goto amac_err_plat_data;
	}

	if ((!IS_PORTMACRO_ENABLED) && (!IS_UNIMAC_PATH)) {
		pr_err("Probe failed:Port:%d not supported as PM disabled!\n",
		       privp->amac_id);
		return -ESOCKTNOSUPPORT;
	}

	spin_lock_init(&privp->lock);

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	bcm_amac_set_ethtool_ops(ndev);

	ndev->netdev_ops = &bcm_amac_enet_ops;
	ndev->watchdog_timeo = TX_TIMEOUT;
	INIT_WORK(&privp->tx_timeout_task, bcm_amac_tx_timeout_task);

	netif_napi_add(ndev, &privp->napi, bcm_amac_enet_rx_poll,
		       NAPI_POLL_WEIGHT);

	ndev->features &= ~(NETIF_F_SG | NETIF_F_FRAGLIST);
	ndev->tx_queue_len = AMAC_DMA_TX_MAX_QUEUE_LEN;

	if (amac_version == 3) {
		/* Enable Hardware TSO and Checksum features */
		ndev->hw_features |= NETIF_F_ALL_TSO | NETIF_F_ALL_CSUM |
				     NETIF_F_SG;
		ndev->vlan_features |= NETIF_F_ALL_TSO | NETIF_F_ALL_CSUM |
				       NETIF_F_SG;
		ndev->features |= NETIF_F_ALL_TSO | NETIF_F_ALL_CSUM |
				  NETIF_F_SG;
	}

	/* Clear stats */
	memset(&ndev->stats, 0, sizeof(ndev->stats));

	/* Start ethernet block */
	rc = bcm_amac_enet_init(privp);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: Failed to start ethernet block\n", __func__);
		goto amac_err_plat_data;
	}

#ifndef RMO_SUPPORT
	INIT_WORK(&privp->tx_work, bcm_amac_tx_task);
#else
	tasklet_init(&privp->tx_tasklet, bcm_amac_tx_task,
		     (unsigned long)privp);
#endif
	tasklet_init(&privp->rx_tasklet_errors, amac_rx_error_task,
		     (unsigned long)privp);
	tasklet_init(&privp->tx_tasklet_errors, amac_tx_error_task,
		     (unsigned long)privp);

	rc = register_netdev(ndev);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: netdev register failed\n", __func__);
		goto amac_err_stop_eth;
	}

	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	ndev->dev.dma_mask = &ndev->dev.coherent_dma_mask;

	if (amac_version == 3)
		pdev->dev.coherent_dma_mask = PEG_DMA_MASK;
	else
		pdev->dev.coherent_dma_mask = DEFAULT_DMA_MASK;

	ndev->dev.coherent_dma_mask = pdev->dev.coherent_dma_mask;

	/* FIX ME */
	/* Keep dma ops of both ndev & pdev in sync */
	ndev->dev.archdata.dma_ops = pdev->dev.archdata.dma_ops;
	privp->state = AMAC_INIT;

	/* create netlink socket to send link notifications */
	privp->nl_sk = netlink_kernel_create(&init_net,
		NETLINK_USERSOCK, (struct netlink_kernel_cfg *)NULL);

	netdev_info(ndev, "NETLINK_USERSOCK create: %s!\n",
		    privp->nl_sk ? "ok" : "failed");

	return 0;

amac_err_stop_eth:
	tasklet_kill(&privp->tx_tasklet);
	bcm_amac_enet_stop(ndev);
	netif_napi_del(&privp->napi);

amac_err_plat_data:
	/* unregister_netdevice(ndev); */
	free_netdev(ndev);

	return rc;
}

/* bcm_amac_enet_remove() - interface remove callback
 * @pdev: platform data structure pointer
 *
 * Returns: 0
 */
static int bcm_amac_enet_remove(struct platform_device *pdev)
{
	struct bcm_amac_priv *privp;
	struct net_device *ndev;

	ndev = platform_get_drvdata(pdev);
	privp = netdev_priv(ndev);
	privp->state = AMAC_SHUTDOWN;
	unregister_netdev(privp->ndev);

	if (privp->nl_sk) {
		netlink_kernel_release(privp->nl_sk);
		privp->nl_sk = NULL;
		netdev_dbg(privp->ndev, "netlink released\n");
	}

	netif_napi_del(&privp->napi);

	tasklet_kill(&privp->tx_tasklet);

	bcm_amac_enet_term(privp);
	free_netdev(privp->ndev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
/* bcm_amac_enet_suspend() - interface suspend callback
 * @dev: device data structure pointer
 *
 * Suspends the Ethernet interface by disabling dma, interrupt etc.
 *
 * Returns: 0
 */
static int bcm_amac_enet_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	netdev_info(ndev, "Suspending AMAC driver\n");

	privp->state = AMAC_SUSPENDED;
	if (netif_running(ndev)) {
		/* Stop TX queues */
		netif_stop_queue(ndev);

		/* Wait for TX FIFO to drain */
		while (kfifo_len(&privp->dma.txfifo) != 0)
			;

#ifdef RMO_SUPPORT
		brm_if_disable(privp->pdev, BRM_ETH, privp->amac_id,
			       BRM_COS_ALL, BRM_BIDIR);
#else
		/* Stop the DMA */
		bcm_amac_dma_stop(privp);
#endif

		/* Disable RX NAPI */
		napi_disable(&privp->napi);

		netif_tx_lock(ndev);
		netif_device_detach(ndev);
		netif_tx_unlock(ndev);
	}

	/* Stop PHY's */
	if (IS_UNIMAC_PATH)
		bcm_amac_gphy_start(privp, false);

	return 0;
}

/* bcm_amac_enet_resume() - interface resume callback
 * @dev: device data structure pointer
 *
 * Resumes the Ethernet interface from sleep or deepsleep. Restores device
 * settings, dma, interrupts.
 *
 * Returns: 0 or error number
 */
static int bcm_amac_enet_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int rc;

	netdev_info(ndev, "Resuming AMAC driver\n");

	/* Since we dont have h/w support for register retention,
	 * initialize everything.
	 */
	rc = bcm_amac_core_init(privp);
	if (rc != 0) {
		netdev_err(ndev, "core init failed!\n");
		return rc;
	}

	if (IS_UNIMAC_PATH)
		bcm_amac_gphy_start(privp, true);

	if (netif_running(ndev)) {
#ifdef RMO_SUPPORT
		pr_info("%s: ndev->ifindex %d\n", __func__, ndev->ifindex);
		ndev->ifindex = 3;
		pr_info("%s: forcing ndev->ifindex to %d\n", __func__,
			ndev->ifindex);

		rc = brm_if_enable(privp->pdev, BRM_ETH, privp->amac_id,
				   BRM_COS_ALL, BRM_BIDIR);
#else
		/* Start DMA */
		rc = bcm_amac_dma_start(privp);
#endif
		if (rc) {
			netdev_err(ndev, "Failed to start DMA Ring interface\n");
			goto err_res_dma_start;
		}

		if (IS_UNIMAC_PATH)
			bcm_amac_unimac_enable(privp, 1);

	/* Power up the PHY */
	if (IS_UNIMAC_PATH) {
		rc = bcm_amac_gphy_powerup(privp, true);
		if (rc) {
			netdev_err(ndev, "PHY powerup failed!\n");
			goto err_res_phy_powerup;
		}
	}
}

		napi_enable(&privp->napi);

		netif_tx_lock(ndev);
		netif_device_attach(ndev);
		netif_tx_unlock(ndev);

		/*netif_start_queue(ndev);*/
		netif_wake_queue(ndev);

	return 0;

err_res_phy_powerup:
	if (IS_UNIMAC_PATH)
		bcm_amac_unimac_enable(privp, false);
	bcm_amac_dma_stop(privp);

err_res_dma_start:
	/* Stop PHY's */
	if (IS_UNIMAC_PATH)
		bcm_amac_gphy_start(privp, false);

	privp->state = AMAC_RESUMED;
	return rc;
}

static const struct dev_pm_ops bcm_amac_enet_pm_ops = {
	.suspend = bcm_amac_enet_suspend,
	.resume = bcm_amac_enet_resume
};
#endif /* CONFIG_PM_SLEEP */

static int __init bcm_amac_setup_ethaddr(char *s)
{
	bool rc;

	if ((!s) || (!strlen(s))) {
		pr_err("bcm-amac: No ethaddr specified\n");
		return 0;
	}

	rc = is_valid_ether_addr(s);
	if (rc) {
		pr_info("bcm-amac: setting ethaddr: %s\n", s);
		strcpy(cmdline_params.mac_addr, s);
	} else {
		pr_err("bcm-amac: Invalid ethaddr - %s\n", s);
		return 0;
	}

	return 1;
}
__setup("ethaddr=", bcm_amac_setup_ethaddr);

static const struct of_device_id bcm_amac_of_enet_match[] = {
	{.compatible = "brcm,amac-enet",},
	{.compatible = "brcm,amac-enet-v2",},
	{.compatible = "brcm,amac-enet-v3",},
	{},
};

MODULE_DEVICE_TABLE(of, bcm_amac_of_enet_match);

static struct platform_driver bcm_amac_enet_driver = {
	.driver = {
		.name  = "amac-enet",
		.of_match_table = bcm_amac_of_enet_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &bcm_amac_enet_pm_ops,
#endif
	},
	.probe    = bcm_amac_enet_probe,
	.remove   = bcm_amac_enet_remove,
};

module_platform_driver(bcm_amac_enet_driver)

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("Broadcom AMAC Ethernet Driver");
MODULE_LICENSE("GPL v2");
