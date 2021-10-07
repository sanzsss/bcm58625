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

#include <linux/kfifo.h>
#include <linux/types.h>
#include <linux/netdevice.h>
#include <linux/of_address.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <asm-generic/delay.h>

#include "bcm-amac-core.h"

#ifdef RMO_SUPPORT
#include <brm.h>
#endif

#define GMAC_RESET_DELAY 2

#define DMA_DESC_ALIGN  (MAX_NUM_DESCS * sizeof(struct amac_dma64_desc))

#define DMA_LOW_ADDR(addr) cpu_to_le32(lower_32_bits(addr))
#define DMA_HIGH_ADDR(addr) cpu_to_le32(upper_32_bits(addr))

#define SPINWAIT(exp, us, err) { \
	uint countdown = (us) * 100; \
	while ((exp) && (countdown >= 1)) {\
		ndelay(10); \
		countdown--; \
	} \
	if (!countdown) \
		err = -EBUSY; \
}

#ifdef PORTMACRO_ENABLE
bool pm_inited;
#endif

u8 amac_version = 1;
bool ctf_inited;
int unimac_port_num;
bool portmacro_enable;

#ifdef EXTRA_CODE
static void amac_dma_ctrlflags(struct bcm_amac_priv *privp, u32 mask, u32 flags)
{
	privp->dmactrlflags &= ~mask;
	privp->dmactrlflags |= flags;

	/*If trying to enable parity, check if parity is actually supported */
	if (privp->dmactrlflags & DMA_CTRL_TX_PEN) {
		u32 control;

		control = readl(privp->hw.reg.amac_core +
				GMAC_DMA_TX_CTRL_REG);
#ifdef ENABLE_DMA_LOOPBACK
		/*enable loopback */
		control |= D64_XC_LE;
#else
		control &= ~D64_XC_LE;
#endif
		writel(control | D64_XC_PD,
		       (privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_REG));
		if (readl((privp->hw.reg.amac_core +
			   GMAC_DMA_TX_CTRL_REG)) & D64_XC_PD) {
			/* We *can* disable it so it is supported,
			 * restore control register
			 */
			writel(control,
			       (privp->hw.reg.amac_core +
				GMAC_DMA_TX_CTRL_REG));
		} else {
			/* Not supported, don't allow it to be enabled */
			privp->dmactrlflags &= ~DMA_CTRL_TX_PEN;
		}
	}
}
#endif

/* amac_alloc_rx_skb() - Allocate RX SKB
 * @privp: driver info pointer
 * @len: length of skb
 * @node: skb node pointer
 *
 * Returns: error or 0
 */
static int amac_alloc_rx_skb(struct bcm_amac_priv *privp,
			     int len,
			     struct skb_list_node *node)
{
	int offset;
	struct sk_buff *skb;
	struct net_device *ndev = privp->ndev;
	dma_addr_t dma_addr;

	skb = __netdev_alloc_skb(ndev, (len + (2 * AMAC_DMA_RXALIGN)),
				 GFP_ATOMIC | GFP_DMA);
	if (!skb)
		return -ENOMEM;

	/* Align buffer for DMA requirements */
	/* Desc has to be 16-bit aligned */
	offset = PTR_ALIGN(skb->data, RX_ALIGN_BOUNDARY) - skb->data;
	skb_reserve(skb, offset);

	/* Set buffer ownership of the new skb pointer */
	dma_addr = dma_map_single(&privp->pdev->dev, skb->data,
				  len,
				  DMA_FROM_DEVICE);
	if (dma_mapping_error(&privp->pdev->dev, dma_addr)) {
		netdev_err(privp->ndev, "RX: SKB DMA mapping error\n");

		dev_kfree_skb_any(skb);
		return -EFAULT;
	}

	node->skb = skb;
	node->len = len;
	node->dma_addr = dma_addr;

	return 0;
}

/* amac_free_rx_skb() - Allocate RX SKB
 * @privp: driver info pointer
 * @len: length of skb
 * @node: skb node pointer
 */
static void amac_free_rx_skb(struct bcm_amac_priv *privp,
			     int len,
			     struct skb_list_node *node)
{
	dma_unmap_single(&privp->pdev->dev,
			 node->dma_addr,
			 len,
			 DMA_FROM_DEVICE);

	dev_kfree_skb_any(node->skb);

	node->skb = NULL;
	node->dma_addr = 0;
	node->len = 0;
}

/* bcm_amac_clear_intr() - Clear Interrupt status
 * @privp: driver info pointer
 * @is_rx: TX or RX direction to clear
 *
 * Returns: none
 */
void bcm_amac_clear_intr(struct bcm_amac_priv *privp, bool is_rx)
{
	u32 intr_status;

	intr_status = readl(privp->hw.reg.amac_core +
			    GMAC_INT_STATUS_REG);

	if (is_rx)
		intr_status &= I_RI; /* Clear only RX interrupt */
	else
		intr_status &= I_XI_ALL; /* Clear only TX interrupt(s) */

	writel(intr_status,
	       (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));
}

 /* bcm_amac_enable_intr_bit() - Enable associated interrupt
 * @privp: driver info pointer
 * @intr_bit: Interrupt status register bit to enable
 *
 * Returns: none
 */
void bcm_amac_enable_intr_bit(struct bcm_amac_priv *privp, int intr_bit)
{
	u32 intr_mask, intr_status;

	/* Clear interrupt status */
	intr_status = readl(privp->hw.reg.amac_core + GMAC_INT_STATUS_REG);
	intr_status &= intr_bit;
	writel(intr_status, (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));

	/* Enable interrupt */
	intr_mask = readl(privp->hw.reg.amac_core + GMAC_INT_MASK_REG);
	intr_mask &= ~intr_bit;
	intr_mask |= intr_bit;
	writel(intr_mask, (privp->hw.reg.amac_core + GMAC_INT_MASK_REG));
}

/**
 * bcm_amac_disable_intr_bit() - Disable associated interrupt
 * @privp: driver info pointer
 * @intr_bit: Interrupt status register bit to disable
 *
 * Returns: none
 */
void bcm_amac_disable_intr_bit(struct bcm_amac_priv *privp, int intr_bit)
{
	u32 intr_mask, intr_status;

	/* Disable interrupt */
	intr_mask = readl(privp->hw.reg.amac_core + GMAC_INT_MASK_REG);
	intr_mask &= ~intr_bit;
	writel(intr_mask, (privp->hw.reg.amac_core + GMAC_INT_MASK_REG));

	/* Clear interrupt status */
	intr_status = readl(privp->hw.reg.amac_core + GMAC_INT_STATUS_REG);
	intr_status &= intr_bit;
	writel(intr_status, (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));
}

/* amac_enable_tx_intr() - Enable TX interrupt
 * @privp: driver info pointer
 * @is_rx: rx or tx
 * @enable: enable/disable the interrupt
 *
 * Returns: none
 */
void bcm_amac_enable_intr(struct bcm_amac_priv *privp, bool is_rx, bool enable)
{
	u32 intr_mask;

	intr_mask = readl(privp->hw.reg.amac_core +
			  GMAC_INT_MASK_REG);

	if (enable) {
		if (is_rx)
			intr_mask |= I_RI;
		else
			intr_mask |= I_XI_ALL;

	} else {
		if (is_rx)
			intr_mask &= ~I_RI;
		else
			intr_mask &= ~I_XI_ALL;
	}

	writel(intr_mask,
	       (privp->hw.reg.amac_core + GMAC_INT_MASK_REG));

	/* Clear the interrupt status if disabling */
	if (!enable)
		bcm_amac_clear_intr(privp, is_rx);
}

/* amac_rx_error_task() - Handle receive errors
 * @data: device info pointer
 *
 * This api is registered with the rx_tasklet_errors tasklet. This
 * gets scheduled when there are any receive errors mentioned in
 * the interrupt status register.
 * The task stop rx dma, start dma and enables RX interrupt and
 * RX error interrupts.
 *
 * Returns: none
 */
void amac_rx_error_task(unsigned long data)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)data;

	/* If offloaded then ODP does all the work */
	if (privp->rings_offloaded)
		return;

	netdev_info(privp->ndev, "Resetting AMAC driver RX channel\n");

#ifndef RMO_SUPPORT /*FIXME*/
	/* Stop RX DMA */
	bcm_amac_enable_rx_dma(privp, false);

	/* Start RX DMA */
	bcm_amac_enable_rx_dma(privp, true);

	/* the rx descriptor ring should have
	 * the addresses set properly
	 * set the lastdscr for the rx ring
	 */
	writel((unsigned long)((privp->dma.rx.descp) +
	       (AMAC_DMA_RX_DESC_CNT - 1) *
	       sizeof(struct amac_dma64_desc)) &
	       D64_XP_LD_MASK,
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_REG));
#endif
	bcm_amac_enable_intr_bit(privp, (I_PDEE | I_PDE | I_DE | I_RFO));
}

/**
 * amac_tx_error_task() - Handle transmit errors
 * @data: device info pointer
 *
 * This api is registered with the tx_tasklet_errors tasklet. This
 * gets scheduled when there are any transmit errors mentioned in
 * the interrupt status register.
 * The task stop tx queue, disable dma to reset the channel, enable
 * dma, enables Tx interrupt and TX error interrupts and then wakes
 * the tx queue.
 *
 * Returns: none
 */
void amac_tx_error_task(unsigned long data)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)data;

	/* If offloaded then ODP does all the work */
	if (privp->rings_offloaded)
		return;

	netdev_info(privp->ndev, "Resetting AMAC driver TX channel\n");

	netif_stop_queue(privp->ndev);

#ifndef RMO_SUPPORT /*FIXME*/
	/* Stop TX DMA */
	bcm_amac_enable_tx_dma(privp, false);

	/* Start TX DMA */
	bcm_amac_enable_tx_dma(privp, true);
#endif
	bcm_amac_enable_intr_bit(privp, (I_PDEE | I_PDE | I_DE | I_XFU));

	netif_wake_queue(privp->ndev);
}

/* bcm_amac_isr() - GMAC ISR Routine. Handles both RX and TX interrupts.
 * @irq: intr number
 * @userdata: driver info data pointer
 *
 * RX interrupt: the interrupt is disabled, cleared and the
 *   NAPI routine is invoked.
 *
 * TX interrupt: interrupt and DMA are both disabled and the tasklet is
 *   scheduled, in case there is more data in the queue.
 *
 * Returns: interrupt handler status
 */
irqreturn_t bcm_amac_isr(int irq, void *userdata)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)userdata;
	u32 intr_status;
#ifdef AMAC_DEBUG
	u32 stat0, stat1;
#endif

	intr_status = readl(privp->hw.reg.amac_core + GMAC_INT_STATUS_REG);
#ifdef AMAC_DEBUG
	stat0 = readl(
		privp->hw.reg.amac_core + GMAC_DMA_RX_STATUS0_REG)
		& D64_RS0_CD_MASK;

	stat1 = readl(
		privp->hw.reg.amac_core + GMAC_DMA_RX_STATUS1_REG)
		& D64_RS0_CD_MASK;
	pr_info("bcm_amac_isr BGN: intr_status = %#X, rxstat0=%#X, rxstat1=%#X\n",
		intr_status, stat0, stat1);
 #endif

#ifndef RMO_SUPPORT
	if (intr_status & I_RI) {
		pr_debug("RX_DMA_COMPLETE INTERRUPT\n");
		/* RX DMA complete interrupt */
		if (likely(netif_running(privp->ndev))) {
			/* Disable RX DMA Interrupt*/
			bcm_amac_enable_intr(privp, BCM_AMAC_DIR_RX, false);

			napi_schedule(&privp->napi);
		}
		return IRQ_HANDLED;
	}

	if (intr_status & I_XI_ALL) {
#ifdef AMAC_DEBUG
		pr_info("TX_DMA_COMPLETE INTERRUPT\n");
#endif
		/* TX DMA complete interrupt */
		if (likely(netif_running(privp->ndev))) {
			/* Disable TX DMA */
			bcm_amac_enable_tx_dma(privp, false);

			bcm_amac_clear_intr(privp, BCM_AMAC_DIR_TX);

			atomic_set(&privp->dma.tx_dma_busy, BCM_AMAC_DMA_FREE);

			/* trigger tx processing in case packets are waiting */
			schedule_work(&privp->tx_work);
		}
		return IRQ_HANDLED;
	}
#endif

	if (intr_status & I_RDU) {
		/* This is not a error condition
		 * Associated interrupt is cleared by supplying
		 * descriptors to the rx channel
		 */
		netdev_dbg(privp->ndev,
			   "receive descriptor underflow error\n");
		return IRQ_HANDLED;
	}

	if (intr_status & I_PDEE) {
		/* Clear associated Interrupt status and disable associated
		 * interrupt.
		 */
		bcm_amac_disable_intr_bit(privp, I_PDEE);
		/* Transmit Descriptor read error */
		if ((readl(privp->hw.reg.amac_core +
			GMAC_DMA_TX_STATUS1_REG)
				>> D64_XS0_XS_SHIFT) == D64_XS1_XE_PDEE) {
			netdev_dbg(privp->ndev,
				   "Transmit Descriptor read error\n");
			privp->eth_stats.tx_errors++;
			tasklet_schedule(&privp->tx_tasklet_errors);
		}
		/* Receive Descriptor read error */
		if ((readl(privp->hw.reg.amac_core +
			GMAC_DMA_RX_STATUS1_REG)
				>> D64_RS0_RS_SHIFT) == D64_RS1_RE_PDEE) {
			netdev_dbg(privp->ndev,
				   "Receive Descriptor read error\n");
			privp->eth_stats.rx_errors++;
			tasklet_schedule(&privp->rx_tasklet_errors);
		}
		return IRQ_HANDLED;
	}

	if (intr_status & I_PDE) {
		/* Clear associated Interrupt status and disable associated
		 * interrupt.
		 */
		bcm_amac_disable_intr_bit(privp, I_PDE);
		/* Transmit Data transfer error */
		if ((readl(privp->hw.reg.amac_core +
			GMAC_DMA_TX_STATUS1_REG)
				>> D64_XS0_XS_SHIFT) == D64_XS1_XE_PDE) {
			netdev_dbg(privp->ndev,
				   "Transmit data transfer error\n");
			privp->eth_stats.tx_errors++;
			tasklet_schedule(&privp->tx_tasklet_errors);
		}
		/* Receive Data transfer error */
		if ((readl(privp->hw.reg.amac_core +
			GMAC_DMA_RX_STATUS1_REG)
				>> D64_RS0_RS_SHIFT) == D64_RS1_RE_PDE) {
			netdev_dbg(privp->ndev,
				   "Receive data transfer error\n");
			privp->eth_stats.rx_errors++;
			tasklet_schedule(&privp->rx_tasklet_errors);
		}
		return IRQ_HANDLED;
	}

	if (intr_status & I_DE) {
		/* Clear associated Interrupt status and disable associated
		 * interrupt.
		 */
		bcm_amac_disable_intr_bit(privp, I_DE);
		/* Transmit Data transfer error */
		if ((readl(privp->hw.reg.amac_core +
			GMAC_DMA_TX_STATUS1_REG)
				>> D64_XS0_XS_SHIFT) == D64_XS1_XE_DE) {
			netdev_dbg(privp->ndev,
				   "Transmit descriptor protocol error\n");
			privp->eth_stats.tx_errors++;
			tasklet_schedule(&privp->tx_tasklet_errors);
		}
		/* Receive Data transfer error */
		if ((readl(privp->hw.reg.amac_core +
			GMAC_DMA_RX_STATUS1_REG)
				>> D64_RS0_RS_SHIFT) == D64_RS1_RE_DE) {
			netdev_dbg(privp->ndev,
				   "Receive descriptor protocol error\n");
			privp->eth_stats.rx_errors++;
			tasklet_schedule(&privp->rx_tasklet_errors);
		}
		return IRQ_HANDLED;
	}

	if (intr_status & I_RFO) {
		/* Clear associated Interrupt status and disable associated
		 * interrupt.
		 */
		bcm_amac_disable_intr_bit(privp, I_RFO);
		if ((readl(privp->hw.reg.amac_core +
			GMAC_DMA_RX_STATUS1_REG)
				>> D64_RS0_RS_SHIFT) == D64_RS1_RE_RFO) {
			netdev_dbg(privp->ndev,
				   "Receive FIFO overflow error\n");
			privp->eth_stats.rx_errors++;
			tasklet_schedule(&privp->rx_tasklet_errors);
		}
		return IRQ_HANDLED;
	}

	if (intr_status & I_XFU) {
		/* Clear associated Interrupt status and disable associated
		 * interrupt.
		 */
		bcm_amac_disable_intr_bit(privp, I_XFU);
		if ((readl(privp->hw.reg.amac_core +
			GMAC_DMA_TX_STATUS1_REG)
				>> D64_XS0_XS_SHIFT) == D64_XS1_XE_XFU) {
			netdev_dbg(privp->ndev,
				   "Transmit data FIFO underrun error\n");
			privp->eth_stats.tx_errors++;
			tasklet_schedule(&privp->tx_tasklet_errors);
		}
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void amac_dma_rx_init_chnl(struct bcm_amac_priv *privp)
{
	struct amac_dma_priv *dma_p = &privp->dma;

	/* initailize the DMA channel */
	writel(lower_32_bits(dma_p->rx.base_addr),
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_ADDR_LO_REG));

	writel(upper_32_bits(dma_p->rx.base_addr),
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_ADDR_HI_REG));

	/* now update the dma last descriptor */
	writel(dma_p->rx.base_addr,
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_REG));
}

/* amac_dma_rx_init() - RX DMA initialization routine
 * @privp: driver info pointer
 *
 * Descriptors and skb are allocated and initlialized, the various rx registers
 * are updated with the descriptor information
 *
 * Returns: '0' or error
 */
static int amac_dma_rx_init(struct bcm_amac_priv *privp)
{
	struct net_device *ndev = privp->ndev;
	struct amac_dma_priv *dma_p = &privp->dma;
	struct amac_dma64_desc *descp;
	u32 offset;
#ifndef RMO_SUPPORT
	unsigned int size;
	u32 ctrl, i;
	int rc = 0;
#endif

	if (dma_p->rx.ring_len)
		return -EFAULT;

	dma_p->rx.ring_len = AMAC_DMA_RX_DESC_CNT;

	/* Allocate rx descriptors */
	dma_p->rx.index = 0;
	dma_p->rx.alloc_size = (AMAC_DMA_RX_DESC_CNT *
			       sizeof(struct amac_dma64_desc)) + DMA_DESC_ALIGN;
	dma_p->rx.raw_descp = dma_alloc_coherent(&privp->pdev->dev,
					     dma_p->rx.alloc_size,
					     &dma_p->rx.raw_addr,
					     GFP_FLAGS);
	if (!dma_p->rx.raw_descp) {
		netdev_err(ndev, "Failed to alloc rx dma desc\n");
		return -ENOMEM;
	}

	dma_p->rx.base_addr = ALIGN(dma_p->rx.raw_addr, DMA_DESC_ALIGN);
	offset = dma_p->rx.base_addr - dma_p->rx.raw_addr;
	dma_p->rx.descp = dma_p->rx.raw_descp + offset;

#ifndef RMO_SUPPORT
	/* Allocate rx skb list */
	size = dma_p->rx.ring_len * sizeof(*dma_p->rx_skb_list);
	dma_p->rx_skb_list = kzalloc(size, GFP_FLAGS);
	if (!dma_p->rx_skb_list) {
		netdev_err(ndev, "Failed to alloc rx skb list size=%u\n", size);
		rc = -ENOMEM;
		goto rx_init_err;
	}

	/* Setup rx descriptor ring */
	for (i = 0; i < AMAC_DMA_RX_DESC_CNT; i++) {
		descp = (struct amac_dma64_desc *)(dma_p->rx.descp) + i;

		rc = amac_alloc_rx_skb(privp, AMAC_DMA_RX_BUF_LEN,
				       &dma_p->rx_skb_list[i]);
		if (rc)
			goto rx_init_skb_err;

		ctrl = 0;

		/* if last descr set endOfTable */
		if (i == (AMAC_DMA_RX_DESC_CNT - 1))
			ctrl = D64_CTRL1_EOT;

		descp->ctrl1 = cpu_to_le32(ctrl);
		descp->ctrl2 = cpu_to_le32(AMAC_DMA_RX_BUF_LEN);
		descp->addrlow =
			DMA_LOW_ADDR(dma_p->rx_skb_list[i].dma_addr);
		descp->addrhigh =
			DMA_HIGH_ADDR(dma_p->rx_skb_list[i].dma_addr);
	}
#endif

	amac_dma_rx_init_chnl(privp);

	return 0;

rx_init_skb_err:

#ifndef RMO_SUPPORT
	for (i = 0; i < AMAC_DMA_RX_DESC_CNT; i++) {
		if (dma_p->rx_skb_list[i].skb)
			amac_free_rx_skb(privp, AMAC_DMA_RX_BUF_LEN,
					 &dma_p->rx_skb_list[i]);
		else
			break;
	}

	kfree(privp->dma.rx_skb_list);
#endif
	privp->dma.rx_skb_list = NULL;

rx_init_err:
	if (privp->dma.rx.descp) {
		dma_free_coherent(&privp->pdev->dev,
				  privp->dma.rx.alloc_size,
				  privp->dma.rx.descp,
				  privp->dma.rx.base_addr);
		privp->dma.rx.descp = NULL;
	}

	return rc;
}

/* amac_dma_tx_init() - TX DMA initialization routine
 * @privp: driver info pointer
 *
 * Descriptors and skb list are allocated.
 *
 * Returns: 0 or error
 */
static int amac_dma_tx_init(struct bcm_amac_priv *privp)
{
	struct net_device *ndev = privp->ndev;
	struct amac_dma_priv *dma_p = &privp->dma;
	u32 size;
	u32 offset;

	if (dma_p->tx.ring_len)
		return -EFAULT;

	dma_p->tx.ring_len = AMAC_DMA_TX_DESC_CNT;
	dma_p->tx_max_pkts = AMAC_DMA_TX_MAX_QUEUE_LEN;

	/* Allocate tx descriptors */
	dma_p->tx.index = 0;
	dma_p->tx.alloc_size = (dma_p->tx.ring_len *
				sizeof(struct amac_dma64_desc)) +
				DMA_DESC_ALIGN;
	dma_p->tx.raw_descp = dma_alloc_coherent(&privp->pdev->dev,
					     dma_p->tx.alloc_size,
					     &dma_p->tx.raw_addr,
					     GFP_FLAGS);
	if (!dma_p->tx.raw_descp) {
		netdev_err(ndev, "Cannot allocate tx dma descriptors.\n");
		return -ENOMEM;
	}

	dma_p->tx.base_addr = ALIGN(dma_p->tx.raw_addr, DMA_DESC_ALIGN);
	offset = dma_p->tx.base_addr - dma_p->tx.raw_addr;
	dma_p->tx.descp = dma_p->tx.raw_descp + offset;

	/* Allocate tx skb list */
	size = dma_p->tx_max_pkts * sizeof(*dma_p->tx_skb_list);
	dma_p->tx_skb_list = kzalloc(size, GFP_FLAGS);
	if (!dma_p->tx_skb_list) {
		netdev_err(ndev, "Failed to alloc tx skb list size=%u\n", size);
		goto tx_init_err;
	}

	bcm_amac_enable_tx_dma(privp, false);

	/* We may not have DMA rings allocated when RMO is not used */
	if (dma_p->tx.raw_descp && dma_p->tx.ring_len)
		memset(dma_p->tx.raw_descp, 0, dma_p->tx.alloc_size);

	/* initailize the DMA channel */
	writel((u32)(unsigned long)(&((struct amac_dma64_desc *)
				       (dma_p->tx.base_addr))[0]),
	       (privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_LO_REG));
	writel((u32)((unsigned long)(&((struct amac_dma64_desc *)
					(dma_p->tx.base_addr))[0]) >> 32),
	       (privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_HI_REG));

	return 0;

tx_init_err:
	dma_free_coherent(&privp->pdev->dev,
			  dma_p->tx.alloc_size,
			  privp->dma.tx.descp,
			  privp->dma.tx.base_addr);
	dma_p->tx.alloc_size = 0;

	return -ENOMEM;
}

static inline void amac_core_init_reset(struct bcm_amac_priv *privp)
{
	u32 tmp;

	tmp = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);
	tmp |= CC_SR;
	writel(tmp, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));

	udelay(GMAC_RESET_DELAY);
}

static inline void amac_core_clear_reset(struct bcm_amac_priv *privp)
{
	u32 tmp;

	tmp = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);
	tmp &= ~(CC_SR);
	writel(tmp, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));

	udelay(GMAC_RESET_DELAY);
}

/* amac_set_prom() - Enable / Disable promiscuous mode
 * @privp: device data pointer
 * @enable: '0' disables promiscuous mode, >0 enables promiscuous mode
 */
static void amac_set_prom(struct bcm_amac_priv *privp, bool enable)
{
	u32 reg;

	reg = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);

	if ((enable == true) && (!(reg & CC_PROM)))
		reg |= CC_PROM;
	else if ((enable == false) && (reg & CC_PROM))
		reg &= ~CC_PROM;
	else
		return;

	amac_core_init_reset(privp);
	writel(reg, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));
	amac_core_clear_reset(privp);
}

/* bcm_amac_set_ctf_pass_thru_mode() - Set ctf pass through mode
 * @privp: device data pointer
 */
static void bcm_amac_set_ctf_pass_thru_mode(struct bcm_amac_priv *privp)
{
	int val;

	val = CTF_PASS_TRU_PORT_VALUE;
	writel(val, privp->hw.reg.ctf_regs + CTF_PASS_TRU_PORT0_REG);
	writel(val, privp->hw.reg.ctf_regs + CTF_PASS_TRU_PORT1_REG);
	writel(val, privp->hw.reg.ctf_regs + CTF_PASS_TRU_PORT2_REG);
	writel(val, privp->hw.reg.ctf_regs + CTF_PASS_TRU_PORT3_REG);

	/* We may need following setting for FA4.
	 * Pegasus SV code does not contain this CTF_CONTROL setting for basic
	 * etehrnet with out FA4. Commenting this as this is not needed
	 * for basic Ethernet.
	 * But code works with this setting as well
	 */

	/*writel(0x0400000F, privp->hw.reg.amac_core + CTF_CONTROL_REG);*/
}

/* bcm_amac_setup_unimac_port() - Set port 3 as Unimac port
 * @privp: device data pointer
 */
void bcm_amac_setup_unimac_port(struct bcm_amac_priv *privp)
{
	u32 tmp;

	tmp = readl(privp->hw.reg.ctf_regs + CTF_PORTS_CONFIG_REG);
	tmp |= CTF_PORT3_UNIMAC_ENABLE;
#ifdef PORTMACRO_ENABLE
	if (privp->b_unimac != 1)
		tmp &= ~CTF_PORT3_UNIMAC_ENABLE;
#endif
	writel(tmp, (privp->hw.reg.ctf_regs + CTF_PORTS_CONFIG_REG));
}

static void bcm_amac_unimac_init(struct bcm_amac_priv *privp)
{
	u32 cmd;
	/* reset GMAC core */
	cmd = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);
	cmd &= ~(CC_TE | CC_RE | CC_RPI | CC_TAI | CC_HD | CC_ML |
		 CC_CFE | CC_RL | CC_RED | CC_PE | CC_TPI | CC_PAD_EN | CC_PF);
	cmd |= (CC_NLC | CC_CFE);       /* keep promiscuous mode disabled */

	/* enable 802.3x tx flow control (honor received PAUSE frames) */
	cmd &= ~CC_RPI;

	/* keep promiscuous mode disable */

#ifdef MAC_LOOPBACK
	/* Enable Unimac loopback mode */
	cmd |= (CC_ML);         /* Unimac Loopback mode */
#else
	cmd &= ~CC_ML;
#endif
	/* set the speed */
	/*cmd &= ~(CC_ES_MASK | CC_HD | (1<<4));*/
	/* Set to 1Gbps and full duplex by default */
	cmd |= (2 << CC_ES_SHIFT);

	/* Enable TX/RX Unimac path */
	cmd |= CC_TE | CC_RE;
	/*cmd = UNIMAC_CMD_CONFIG_PEG;*/

	amac_core_init_reset(privp);    /* Put GMAC in reset */
	writel(cmd, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));
	amac_core_clear_reset(privp);
#ifdef AMAC_DEBUG
	cmd = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);
	pr_emerg("Unimac CmdCfg reg.=%#X\n", cmd);
#endif
	writel(0xFFFFFEFF, privp->hw.reg.amac_core + GMAC_INT_MASK_REG);
}

/* bcm_amac_core_init() - Initialize the gmac core
 * @privp: driver info pointer
 *
 * Initialize the gmac core, phy, setup clock, initialize swith mode or
 * switch bypass mode.
 *
 * Returns: '0' for success or '-1'
 */
int bcm_amac_core_init(struct bcm_amac_priv *privp)
{
	u32 tmp;
	u32 cmd;
	struct device_node *np = privp->pdev->dev.of_node;
#ifdef PORTMACRO_ENABLE
	int link_status, link_speed;
#endif

	/* Reset AMAC core */
	tmp = readl(privp->hw.reg.amac_idm_base + IDM_RST_CTRL_REG);
	tmp |= AMAC_RESET_ENABLE;
	writel(tmp, privp->hw.reg.amac_idm_base + IDM_RST_CTRL_REG);
	tmp &= ~AMAC_RESET_ENABLE;
	writel(tmp, privp->hw.reg.amac_idm_base + IDM_RST_CTRL_REG);

	/* Set clock */
	tmp = readl(privp->hw.reg.amac_idm_base + AMAC_IDM_IO_CTRL_DIRECT_REG);
	tmp &= ~BIT(AMAC_IDM_IO_CTRL_CLK_250_SEL_BIT);
	tmp &= ~BIT(AMAC_IDM_IO_CTRL_DEST_SYNC_MODE_EN_BIT);
	if (amac_version != 3)
		tmp |= BIT(AMAC_IDM_IO_CTRL_GMII_MODE_BIT);
	if (of_dma_is_coherent(np))
		tmp |= (0xF << AMAC_IDM_IO_CTRL_AWCACHE_OFFSET
			| 0xF << AMAC_IDM_IO_CTRL_ARCACHE_OFFSET);
	writel(tmp, privp->hw.reg.amac_idm_base + AMAC_IDM_IO_CTRL_DIRECT_REG);

	if (amac_version == 3)
	if (privp->hw.reg.ctf_regs && (!ctf_inited)) {
		bcm_amac_set_ctf_pass_thru_mode(privp);
		if (IS_UNIMAC_PORT)
			bcm_amac_setup_unimac_port(privp);
		tmp =
		readl(privp->hw.reg.amac_core + GMAC_CHECKSUM_CTRL_REG);

		/* Default reset value:Read is buggy in Pegasus A0 */
		tmp = 0x00801c00;
		tmp &= ~(CKSUM_CONTROL_CTF_BYPASS);

		/* Enable TSO and Shim Header */
		tmp |= CKSUM_CONTROL_SHIM_HDR_EN;
		tmp |= CKSUM_CONTROL_TSO_ENABLE;
		writel(tmp,
		       (privp->hw.reg.amac_core + GMAC_CHECKSUM_CTRL_REG));
		ctf_inited = 1;
	}

	/* reset GMAC core */
	cmd = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);
	cmd &= ~(CC_TE | CC_RE | CC_RPI | CC_TAI | CC_HD | CC_ML |
		CC_CFE | CC_RL | CC_RED | CC_PE | CC_TPI |
		CC_PAD_EN | CC_PF);
	cmd |= (CC_NLC | CC_CFE); /* keep promiscuous mode disabled */

	amac_core_init_reset(privp);
	if (amac_version == 3)
		cmd = UNIMAC_CMD_CONFIG_PEG;
	writel(cmd, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));
	amac_core_clear_reset(privp);

	if (!IS_UNIMAC_PORT) {
		/* Reset AMAC3 core to allow CTF access
		 * if this is not Unimac port (AMAC0-2)
		 */
		tmp = readl(privp->hw.reg.amac3_io_ctrl + IDM_RST_CTRL_REG);
		tmp |= 0x1;
		writel(tmp, privp->hw.reg.amac3_io_ctrl + IDM_RST_CTRL_REG);
		tmp &= 0xFFFFFFFE;
		writel(tmp, privp->hw.reg.amac3_io_ctrl + IDM_RST_CTRL_REG);

		/* Set clock for AMAC3 core to allow CTF access */
		tmp = readl(privp->hw.reg.amac3_io_ctrl +
			    AMAC_IDM_IO_CTRL_DIRECT_REG);
		tmp &= ~(1 << AMAC_IDM_IO_CTRL_CLK_250_SEL_BIT);
		tmp |= (1 << AMAC_IDM_IO_CTRL_DEST_SYNC_MODE_EN_BIT);
		writel(tmp, privp->hw.reg.amac_idm_base);
		writel(tmp, privp->hw.reg.amac3_io_ctrl +
		       AMAC_IDM_IO_CTRL_DIRECT_REG);
	}

	/* Enable clear MIB on read */
	tmp = readl(privp->hw.reg.amac_core);
	tmp |= (DC_MROR | DC_CFCO);
	writel(tmp, privp->hw.reg.amac_core);

	/* These flow control settings are approximately
	 * same as defaults 65% & 75% adjusted for IMIX 300 pkt boundary
	 */
	tmp = AMAC_FLOW_CTRL_ON_THRESH << AMAC_FLOW_CTRL_ON_THRESH_OFF |
		AMAC_FLOW_CTRL_OFF_THRESH << AMAC_FLOW_CTRL_OFF_THRESH_OFF;
	writel(tmp, privp->hw.reg.amac_core + AMAC_FLOW_CTRL_THRESH_REG);

	/* PHY set smi_master to driver mdc_clk */
	tmp = readl(privp->hw.reg.amac_core + GMAC_PHY_CTRL_REG);
	tmp |= PC_MTE;
	writel(tmp, (privp->hw.reg.amac_core + GMAC_PHY_CTRL_REG));

	tmp = readl(privp->hw.reg.amac_core + GMAC_CHECKSUM_CTRL_REG);

	tmp = 0x00801c00; /*Default reset value:Read is buggy in Pegasus A0*/

#ifdef ENABLE_DMA_LOOPBACK
	tmp |= CKSUM_CONTROL_CTF_BYPASS;
#else
	tmp &= ~(CKSUM_CONTROL_CTF_BYPASS);
#endif

#ifdef EXTRA_CODE
	tmp &= ~(CKSUM_CONTROL_BRCM_HDR_EN | CKSUM_CONTROL_SHIM_HDR_EN |
		 CKSUM_CONTROL_TSO_ENABLE | CKSUM_CONTROL_LRO_ENABLED |
		 CKSUM_CONTROL_RMO_ENABLED | CKSUM_CONTROL_TXHDRCKSUMEN |
		 CKSUM_CONTROL_RXCKSUMEN | CKSUM_CONTROL_TXCKSUMEN);
#endif

#ifdef RMO_SUPPORT
	tmp |= CKSUM_CONTROL_RMO_ENABLED;
#endif

	/* Enable TSO and Shim Header */
	tmp |= CKSUM_CONTROL_SHIM_HDR_EN;
	tmp |= CKSUM_CONTROL_TSO_ENABLE;

	writel(tmp, (privp->hw.reg.amac_core + GMAC_CHECKSUM_CTRL_REG));
	/* Clear persistent sw intstatus */
	writel(0, (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));

	/* Configure 1G port */
	/* enable one rx interrupt per received frame */
	if (amac_version == 3) {
		tmp = readl(privp->hw.reg.ctf_regs + CTF_PROPERTY_PORT_BASE +
			    privp->amac_id * 4);
		tmp |= (1 << CTF_PROPERTY_PAUSE_TO_PORT);
		writel(tmp, privp->hw.reg.ctf_regs +
		       CTF_PROPERTY_PORT_BASE + privp->amac_id * 4);
		writel((DEFAULT_RX_INT_CNT << GMAC0_IRL_FRAMECOUNT_SHIFT)
		       | RX_LAZY_TIMEO,
		       (privp->hw.reg.amac_core +
			GMAC_INTR_RX_LAZY_REG_PEG));
	} else {
		if (privp->hw.reg.switch_global_cfg) {
			/* Required to access the PHY's in
			 * switch-by-pass mode in some SoC's
			 * when the PHY needs to be connected
			 * directly to the mac.
			 */
			dev_info(&privp->pdev->dev,
				 "%s: Switch bypass mode\n", __func__);
			/* Configure Switch */
			tmp = readl(privp->hw.reg.switch_global_cfg);
			tmp |= BIT(CDRU_SWITCH_CFG_BYPASS_SWITCH);
			writel(tmp, (privp->hw.reg.switch_global_cfg));
		}

		if (privp->hw.reg.crmu_io_pad_ctrl) {
			/* Setup IO PAD CTRL */
			tmp = readl(privp->hw.reg.crmu_io_pad_ctrl);
			tmp &= ~BIT(
			   CRMU_CHIP_IO_PAD_CONTROL__CDRU_IOMUX_FORCE_PAD_IN);
			writel(tmp, (privp->hw.reg.crmu_io_pad_ctrl));
		}

		writel(((NAPI_POLL_WEIGHT / 2) << GMAC0_IRL_FRAMECOUNT_SHIFT)
		       | RX_LAZY_TIMEO / 10,
		       (privp->hw.reg.amac_core +
			GMAC_INTR_RX_LAZY_REG));
	}

	if ((!IS_PORTMACRO_ENABLED) && IS_UNIMAC_PORT)
		privp->b_unimac = 1;

	if ((IS_PORTMACRO_ENABLED) && (!IS_UNIMAC_PATH)) {
		if (!pm_inited) {
			tmp = readl(privp->hw.reg.eth_config_status + 0x4);
			tmp |= 0x2;
			writel(tmp, privp->hw.reg.eth_config_status + 0x4);
			lm_hw_pm_apb2pbus_brdg_init(privp);
		}
		mdelay(100);
		lm_hw_pm_minimal_default_xlmac_cfg(privp, 1);
		lm_hw_pm_cfg_mtu(privp);
		if (!pm_inited) {
			lm_hw_pm_reset(privp);
			/*lm_hw_pm_probe(privp);*/
			mdelay(100);
			lm_hw_pm_bringup_phy(privp);
			lm_hw_pm_get_link_status(privp, &link_status,
						 &link_speed);
			pm_inited = 1;
		}
		lm_hw_pm_cfg_mac_core(privp, 10000);
		mdelay(100);
	} else {
		bcm_amac_unimac_init(privp);
	}

	writel(0xFFFFFEFF, privp->hw.reg.amac_core + GMAC_INT_MASK_REG);
	return 0;
}

void bcm_amac_core_term(struct bcm_amac_priv *privp)
{
	u32 tmp;

	/* Reset AMAC core */
	tmp = readl(privp->hw.reg.amac_idm_base + IDM_RST_CTRL_REG);
	tmp |= 0x1;
	writel(tmp, privp->hw.reg.amac_idm_base + IDM_RST_CTRL_REG);
	/*TODO: Also set IDM M3 for CTF*/
}

/* bcm_amac_unimac_enable() - Enable GMAC core
 * @privp: driver info pointer
 * @enable: Enable or disable
 *
 * Returns: none
 */
void bcm_amac_unimac_enable(struct bcm_amac_priv *privp, bool enable)
{
	u32 cmdcfg, val;

	amac_core_init_reset(privp);

	cmdcfg = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);

	/* if not enable exit now */
	if (!enable)
		return;

	/* enable the mac transmit and receive paths now */
	udelay(2);
	cmdcfg &= ~CC_SR;
	if (enable == 0)
		cmdcfg &= ~(CC_RE | CC_TE);
	else
		cmdcfg |= (CC_RE | CC_TE);

	/* assert rx_ena and tx_ena when out of reset to enable the mac */
	writel(cmdcfg, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));
	amac_core_clear_reset(privp);

	val  = 0;
#ifdef MAC_LOOPBACK
	/* Unimac command config reg. values */
	val = CMD_CFG_DEFAULT | CC_CF | CC_PF | CC_PROM | CC_ML | CC_RPI;
	val &= ~(CC_PAD_EN | CC_NLC | CC_RFD | CC_AE | CC_LC |
		 CC_CFE | CC_OE | CC_TPI | CC_TAI);
	writel(val, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));

	udelay(10);

	val = CMD_CFG_ML_PEG;
	writel(val, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));

	val = UNIMAC_TAG1;
	writel(val, (privp->hw.reg.amac_core + UNIMAC_TAG1_REG));

	val = UNIMAC_FRMAE_LEN_VAL;
	writel(val, (privp->hw.reg.amac_core + 0x814));

	val = UNIMAC_IPG_HD_BKP_CTRL_VAL;
	writel(val, (privp->hw.reg.amac_core + UNIMAC_IPG_HD_BKP_CTRL_REG));
 #endif
	/* Clear Interrupts */
	writel(I_INTMASK, (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));
}

/* bcm_amac_tx_clean() - Prepare for transmission
 * @privp: driver info pointer
 *
 * Returns: none
 */
#ifdef RMO_SUPPORT
void bcm_amac_tx_clean(void *privp, u8 cos, u32 num_pkts,
		       struct sk_buff **skbp)
#else
void bcm_amac_tx_clean(struct bcm_amac_priv *privp)
#endif
{
#ifdef RMO_SUPPORT
	int j;

	pr_info("%s: free skbs for %d pkts from cos %d\n",
		__func__, num_pkts, cos);

	for (j = 0; j < num_pkts; j++) {
		/*TODO: update stats */
		dev_kfree_skb_any(skbp[j]);
	}
#else
	const struct net_device *ndev = privp->ndev;
	struct amac_dma_priv *dmap = &privp->dma;
	int i;

	for (i = 0; i < dmap->tx_curr; i++) {
		if (dmap->tx_skb_list[i].skb) {
			dma_unmap_single(&privp->pdev->dev,
					 dmap->tx_skb_list[i].dma_addr,
					 dmap->tx_skb_list[i].len,
					 DMA_TO_DEVICE);

			dev_kfree_skb_any(dmap->tx_skb_list[i].skb);
		} else {
			netdev_err(ndev, "invalid skb:%d:?\n", i);
		}

		dmap->tx_skb_list[i].skb = NULL;
		dmap->tx_skb_list[i].len = 0;
		dmap->tx_skb_list[i].dma_addr = 0;
	}

	dmap->tx.index = 0;
	dmap->tx_curr = 0;
#endif
}

/* bcm_amac_set_rx_mode() - Set the rx mode callback
 * @ndev: net device pointer
 *
 * The API enables multicast or promiscuous mode as required. Otherwise
 * it disables multicast and promiscuous mode and adds ARL entries.
 */
void bcm_amac_set_rx_mode(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!netif_running(ndev))
		return;

	if (ndev->flags & (IFF_PROMISC | IFF_ALLMULTI)) {
		/* Enable promiscuous mode in switch bypass mode */
		amac_set_prom(privp, true);
		return;
	}

		/* Disable promiscuous in switch bypass mode */
		amac_set_prom(privp, false);

	/* In switch by pass mode we can only enable promiscuous mode
	 * to allow all packets.
	 */
	if ((ndev->flags & IFF_MULTICAST) && netdev_mc_count(ndev))
		/* In switch bypass mode there is no mac filtering
		 * so enable promiscuous mode to pass all packets up.
		 */
		amac_set_prom(privp, true);
}

#define UNIMAC_MIB_COUNTER(privp, reg)  (privp->\
		hw.reg.amac_core + unimac_mib_base + (reg))
#define UNIMAC_MIB_COUNTER_VAL(privp, reg) readl(UNIMAC_MIB_COUNTER(privp, reg))
#define PM_MIB_COUNTER_VAL(privp, reg) \
	lm_hw_pm_rd_reg(privp, reg, &cntr_msb, &cntr_lsb, 0, privp->amac_id)

static u32 unimac_mib_base;

static void print_mib_counter(struct bcm_amac_priv *privp, char *mib_reg_name,
			      u32 reg)
{
	u64 cntr = 0;
#ifdef PORTMACRO_ENABLE
	u32 cntr_msb, cntr_lsb;
#endif

	if (amac_version == 3)
		unimac_mib_base = MIB_REG_BASE_PEG;
	else
		unimac_mib_base = MIB_REG_BASE;

	if (IS_UNIMAC_PATH)
		cntr = (u32)UNIMAC_MIB_COUNTER_VAL(privp, reg);
#ifdef PORTMACRO_ENABLE
	else {
		PM_MIB_COUNTER_VAL(privp, reg);
		cntr = (cntr_msb & 0xFFUL) << 32 | cntr_lsb;
	}
#endif
	/* print only non-0 counters */
	if (cntr)
		pr_emerg("%s:%llu\n", mib_reg_name, cntr);
}

#define PRN_MIB_COUNTER(privp, reg) {\
mib_reg_name = #reg;\
print_mib_counter(privp, mib_reg_name, reg);\
}

#ifdef PORTMACRO_ENABLE
int bcm_amac_print_pm_mib_counters(struct bcm_amac_priv *privp)
{
	char *mib_reg_name;

	if (!privp) {
		netdev_err(privp->ndev,
			   "bcm_amac_print_mib_counters : pointer passed is NULL!\n");
		return -1;
	}

	PRN_MIB_COUNTER(privp, MAC_GRX64);
	PRN_MIB_COUNTER(privp, MAC_GRX127);
	PRN_MIB_COUNTER(privp, MAC_GRX255);
	PRN_MIB_COUNTER(privp, MAC_GRX511);
	PRN_MIB_COUNTER(privp, MAC_GRX1023);
	PRN_MIB_COUNTER(privp, MAC_GRX1518);
	PRN_MIB_COUNTER(privp, MAC_GRX1522);
	PRN_MIB_COUNTER(privp, MAC_GRX2047);
	PRN_MIB_COUNTER(privp, MAC_GRX4095);
	PRN_MIB_COUNTER(privp, MAC_GRX9216);
	PRN_MIB_COUNTER(privp, MAC_GRX16383);
	PRN_MIB_COUNTER(privp, MAC_GRXPKT);
	PRN_MIB_COUNTER(privp, MAC_GRXUCA);
	PRN_MIB_COUNTER(privp, MAC_GRXMCA);
	PRN_MIB_COUNTER(privp, MAC_GRXBCA);
	PRN_MIB_COUNTER(privp, MAC_GRXFCS);
	PRN_MIB_COUNTER(privp, MAC_GRXCF);
	PRN_MIB_COUNTER(privp, MAC_GRXPF);
	PRN_MIB_COUNTER(privp, MAC_GRXPP);
	PRN_MIB_COUNTER(privp, MAC_GRXUO);
	PRN_MIB_COUNTER(privp, MAC_GRXUDA);
	PRN_MIB_COUNTER(privp, MAC_GRXWSA);
	PRN_MIB_COUNTER(privp, MAC_GRXALN);
	PRN_MIB_COUNTER(privp, MAC_GRXFLR);
	PRN_MIB_COUNTER(privp, MAC_GRXFRERR);
	PRN_MIB_COUNTER(privp, MAC_GRXFCR);
	PRN_MIB_COUNTER(privp, MAC_GRXOVR);
	PRN_MIB_COUNTER(privp, MAC_GRXJBR);
	PRN_MIB_COUNTER(privp, MAC_GRXMTUE);
	PRN_MIB_COUNTER(privp, MAC_GRXMCRC);
	PRN_MIB_COUNTER(privp, MAC_GRXPRM);
	PRN_MIB_COUNTER(privp, MAC_GRXVLN);
	PRN_MIB_COUNTER(privp, MAC_GRXDVLN);
	PRN_MIB_COUNTER(privp, MAC_GRXTRFU);
	PRN_MIB_COUNTER(privp, MAC_GRXPOK);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCOFF0);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCOFF1);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCOFF2);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCOFF3);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCOFF4);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCOFF5);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCOFF6);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCOFF7);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCP0);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCP1);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCP2);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCP3);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCP4);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCP5);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCP6);
	PRN_MIB_COUNTER(privp, MAC_GRXPFCP7);
	PRN_MIB_COUNTER(privp, MAC_GRXSCHCRC);
	PRN_MIB_COUNTER(privp, MAC_GRXUND);
	PRN_MIB_COUNTER(privp, MAC_GRXFRG);
	PRN_MIB_COUNTER(privp, RX_EEE_LPI_EVENT_COUNTER);
	PRN_MIB_COUNTER(privp, RX_EEE_LPI_DURATION_COUNTER);
	PRN_MIB_COUNTER(privp, RX_LLFC_PHY_COUNTER);
	PRN_MIB_COUNTER(privp, RX_LLFC_LOG_COUNTER);
	PRN_MIB_COUNTER(privp, RX_LLFC_CRC_COUNTER);
	PRN_MIB_COUNTER(privp, RX_HCFC_COUNTER);
	PRN_MIB_COUNTER(privp, RX_HCFC_CRC_COUNTER);
	PRN_MIB_COUNTER(privp, MAC_GRXBYT);
	PRN_MIB_COUNTER(privp, MAC_GRXRBYT);
	PRN_MIB_COUNTER(privp, MAC_GRXRPKT);
	PRN_MIB_COUNTER(privp, MAC_GTX64);
	PRN_MIB_COUNTER(privp, MAC_GTX127);
	PRN_MIB_COUNTER(privp, MAC_GTX255);
	PRN_MIB_COUNTER(privp, MAC_GTX511);
	PRN_MIB_COUNTER(privp, MAC_GTX1023);
	PRN_MIB_COUNTER(privp, MAC_GTX1518);
	PRN_MIB_COUNTER(privp, MAC_GTX1522);
	PRN_MIB_COUNTER(privp, MAC_GTX2047);
	PRN_MIB_COUNTER(privp, MAC_GTX4095);
	PRN_MIB_COUNTER(privp, MAC_GTX9216);
	PRN_MIB_COUNTER(privp, MAC_GTX16383);
	PRN_MIB_COUNTER(privp, MAC_GTXPOK);
	PRN_MIB_COUNTER(privp, MAC_GTXPKT);
	PRN_MIB_COUNTER(privp, MAC_GTXUCA);
	PRN_MIB_COUNTER(privp, MAC_GTXMCA);
	PRN_MIB_COUNTER(privp, MAC_GTXBCA);
	PRN_MIB_COUNTER(privp, MAC_GTXPF);
	PRN_MIB_COUNTER(privp, MAC_GTXPP);
	PRN_MIB_COUNTER(privp, MAC_GTXJBR);
	PRN_MIB_COUNTER(privp, MAC_GTXFCS);
	PRN_MIB_COUNTER(privp, MAC_GTXCF);
	PRN_MIB_COUNTER(privp, MAC_GTXOVR);
	PRN_MIB_COUNTER(privp, MAC_GTXDFR);
	PRN_MIB_COUNTER(privp, MAC_GTXEDF);
	PRN_MIB_COUNTER(privp, MAC_GTXSCL);
	PRN_MIB_COUNTER(privp, MAC_GTXMCL);
	PRN_MIB_COUNTER(privp, MAC_GTXLCL);
	PRN_MIB_COUNTER(privp, MAC_GTXXCL);
	PRN_MIB_COUNTER(privp, MAC_GTXFRG);
	PRN_MIB_COUNTER(privp, MAC_GTXERR);
	PRN_MIB_COUNTER(privp, MAC_GTXVLN);
	PRN_MIB_COUNTER(privp, MAC_GTXDVLN);
	PRN_MIB_COUNTER(privp, MAC_GTXRPKT);
	PRN_MIB_COUNTER(privp, MAC_GTXUFL);
	PRN_MIB_COUNTER(privp, MAC_GTXPFCP0);
	PRN_MIB_COUNTER(privp, MAC_GTXPFCP1);
	PRN_MIB_COUNTER(privp, MAC_GTXPFCP2);
	PRN_MIB_COUNTER(privp, MAC_GTXPFCP3);
	PRN_MIB_COUNTER(privp, MAC_GTXPFCP4);
	PRN_MIB_COUNTER(privp, MAC_GTXPFCP5);
	PRN_MIB_COUNTER(privp, MAC_GTXPFCP6);
	PRN_MIB_COUNTER(privp, MAC_GTXPFCP7);
	PRN_MIB_COUNTER(privp, TX_EEE_LPI_EVENT_COUNTER);
	PRN_MIB_COUNTER(privp, TX_EEE_LPI_DURATION_COUNTER);
	PRN_MIB_COUNTER(privp, TX_LLFC_LOG_COUNTER);
	PRN_MIB_COUNTER(privp, TX_HCFC_COUNTER);
	PRN_MIB_COUNTER(privp, MAC_GTXNCL);
	PRN_MIB_COUNTER(privp, MAC_GTXBYT);
	PRN_MIB_COUNTER(privp, XTHOL);

	return 0;
}
#endif

int bcm_amac_print_unimac_mib_counters(struct bcm_amac_priv *privp)
{
	char *mib_reg_name;

	if (!privp) {
		netdev_err(privp->ndev,
			   "bcm_amac_print_mib_counters : pointer passed is NULL!\n");
		return -1;
	}

	PRN_MIB_COUNTER(privp, MIB_TX_GD_OCTETS_LO);
	PRN_MIB_COUNTER(privp, MIB_TX_GD_OCTETS_HI);
	PRN_MIB_COUNTER(privp, MIB_TX_GD_PKTS);
	PRN_MIB_COUNTER(privp, MIB_TX_ALL_OCTETS_LO);
	PRN_MIB_COUNTER(privp, MIB_TX_ALL_OCTETS_HI);
	PRN_MIB_COUNTER(privp, MIB_TX_ALL_PKTS);
	PRN_MIB_COUNTER(privp, MIB_TX_BRDCAST);
	PRN_MIB_COUNTER(privp, MIB_TX_MULT);
	PRN_MIB_COUNTER(privp, MIB_TX_64);
	PRN_MIB_COUNTER(privp, MIB_TX_65_127);
	PRN_MIB_COUNTER(privp, MIB_TX_128_255);
	PRN_MIB_COUNTER(privp, MIB_TX_256_511);
	PRN_MIB_COUNTER(privp, MIB_TX_512_1023);
	PRN_MIB_COUNTER(privp, MIB_TX_1024_1522);
	PRN_MIB_COUNTER(privp, MIB_TX_1523_2047);
	PRN_MIB_COUNTER(privp, MIB_TX_2048_4095);
	PRN_MIB_COUNTER(privp, MIB_TX_4096_8191);
	PRN_MIB_COUNTER(privp, MIB_TX_8192_MAX);
	PRN_MIB_COUNTER(privp, MIB_TX_JAB);
	PRN_MIB_COUNTER(privp, MIB_TX_OVER);
	PRN_MIB_COUNTER(privp, MIB_TX_FRAG);
	PRN_MIB_COUNTER(privp, MIB_TX_UNDERRUN);
	PRN_MIB_COUNTER(privp, MIB_TX_COL);
	PRN_MIB_COUNTER(privp, MIB_TX_1_COL);
	PRN_MIB_COUNTER(privp, MIB_TX_M_COL);
	PRN_MIB_COUNTER(privp, MIB_TX_EX_COL);
	PRN_MIB_COUNTER(privp, MIB_TX_LATE);
	PRN_MIB_COUNTER(privp, MIB_TX_DEF);
	PRN_MIB_COUNTER(privp, MIB_TX_CRS);
	PRN_MIB_COUNTER(privp, MIB_TX_PAUS);
	PRN_MIB_COUNTER(privp, MIB_TXUNICASTPKT);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ0PKT);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ0OCTET_LO);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ0OCTET_HI);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ1PKT);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ1OCTET_LO);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ1OCTET_HI);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ2PKT);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ2OCTET_LO);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ2OCTET_HI);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ3PKT);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ3OCTET_LO);
	PRN_MIB_COUNTER(privp, MIB_TXQOSQ3OCTET_HI);
	PRN_MIB_COUNTER(privp, MIB_RX_GD_OCTETS_LO);
	PRN_MIB_COUNTER(privp, MIB_RX_GD_OCTETS_HI);
	PRN_MIB_COUNTER(privp, MIB_RX_GD_PKTS);
	PRN_MIB_COUNTER(privp, MIB_RX_ALL_OCTETS_LO);
	PRN_MIB_COUNTER(privp, MIB_RX_ALL_OCTETS_HI);
	PRN_MIB_COUNTER(privp, MIB_RX_ALL_PKTS);
	PRN_MIB_COUNTER(privp, MIB_RX_BRDCAST);
	PRN_MIB_COUNTER(privp, MIB_RX_MULT);
	PRN_MIB_COUNTER(privp, MIB_RX_64);
	PRN_MIB_COUNTER(privp, MIB_RX_65_127);
	PRN_MIB_COUNTER(privp, MIB_RX_128_255);
	PRN_MIB_COUNTER(privp, MIB_RX_256_511);
	PRN_MIB_COUNTER(privp, MIB_RX_512_1023);
	PRN_MIB_COUNTER(privp, MIB_RX_1024_1522);
	PRN_MIB_COUNTER(privp, MIB_RX_1523_2047);
	PRN_MIB_COUNTER(privp, MIB_RX_2048_4095);
	PRN_MIB_COUNTER(privp, MIB_RX_4096_8191);
	PRN_MIB_COUNTER(privp, MIB_RX_8192_MAX);
	PRN_MIB_COUNTER(privp, MIB_RX_JAB);
	PRN_MIB_COUNTER(privp, MIB_RX_OVR);
	PRN_MIB_COUNTER(privp, MIB_RX_FRAG);
	PRN_MIB_COUNTER(privp, MIB_RX_DROP);
	PRN_MIB_COUNTER(privp, MIB_RX_CRC_ALIGN);
	PRN_MIB_COUNTER(privp, MIB_RX_UND);
	PRN_MIB_COUNTER(privp, MIB_RX_CRC);
	PRN_MIB_COUNTER(privp, MIB_RX_ALIGN);
	PRN_MIB_COUNTER(privp, MIB_RX_SYM);
	PRN_MIB_COUNTER(privp, MIB_RX_PAUS);
	PRN_MIB_COUNTER(privp, MIB_RX_CNTRL);
	PRN_MIB_COUNTER(privp, MIB_RXSACHANGES);
	PRN_MIB_COUNTER(privp, MIB_RXUNICASTPKTS);

	return 0;
}

int bcm_amac_print_mib_counters(struct bcm_amac_priv *privp)
{
	if (IS_UNIMAC_PATH)
		bcm_amac_print_unimac_mib_counters(privp);
#ifdef PORTMACRO_ENABLE
	else if (IS_PORTMACRO_ENABLED)
		bcm_amac_print_pm_mib_counters(privp);
#endif

	return 0;
}

/* bcm_amac_enable_tx_dma() - Enable the DMA
 * @privp: driver info pointer
 * @dir: TX or RX direction to enable
 *
 * Returns: 0 or error
 */
int bcm_amac_enable_tx_dma(struct bcm_amac_priv *privp, bool enable)
{
	u32 control;
	int status = 0;

	if (enable) {
		/* tx dma flags:
		 *  burst len (2**(N+4)): N=3
		 *  parity check: disabled
		 *  tx enable: enabled
		 */
		control = (3 << D64_XC_BL_SHIFT) | D64_XC_PD  | D64_XC_XE;

		writel(control,
		       (privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_REG));

		SPINWAIT(((status =
			  (readl(privp->hw.reg.amac_core +
				 GMAC_DMA_TX_STATUS0_REG) &
			   D64_XS0_XS_MASK)) != D64_XS0_XS_IDLE),
			  100, status);
	} else {
		/* Disable TX DMA */

		/* suspend tx DMA first, if active */
		status = readl(privp->hw.reg.amac_core +
			       GMAC_DMA_TX_STATUS0_REG)
				& D64_XS0_XS_MASK;

		if (status == D64_XS0_XS_ACTIVE) {
			status = readl(privp->hw.reg.amac_core +
				       GMAC_DMA_TX_CTRL_REG);
			status |= D64_XC_SE;

			writel(status, (privp->hw.reg.amac_core +
			       GMAC_DMA_TX_CTRL_REG));

			/* DMA engines are not disabled */
			/* until transfer finishes */
			SPINWAIT(
				 ((readl(privp->hw.reg.amac_core +
					GMAC_DMA_TX_STATUS0_REG)
				  & D64_XS0_XS_MASK) == D64_XS0_XS_ACTIVE),
				 100, status);
		}

		writel(0, (privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_REG));
		SPINWAIT(((status =
			  (readl(privp->hw.reg.amac_core +
				 GMAC_DMA_TX_STATUS0_REG)
			   & D64_XS0_XS_MASK)) != D64_XS0_XS_DISABLED),
			 100, status);
	}

	return status;
}

/* bcm_amac_enable_rx_dma() - Enable/Disable the RX DMA
 * @privp: driver data structure pointer
 * @enable: enable/disable rx dma
 *
 * Returns: 0 or error
 */
int bcm_amac_enable_rx_dma(struct bcm_amac_priv *privp, bool enable)
{
	u32 control;
	u32 status = 0;

	if (enable) {
		/* rx dma flags:
		 *  dma prefetch control: upto 4
		 *  burst len(2**(N+4)): default (N=1)
		 *  parity check: disabled
		 *  overflow continue: enabled
		 *  hw status size: 30
		 *  rx enable: enabled
		 */
		control = (D64_RC_PC_4_DESC << D64_RC_PC_SHIFT) |
			BIT(D64_XC_BL_SHIFT) |
			D64_XC_PD |
			D64_RC_OC |
			(HWRXOFF << D64_RC_RO_SHIFT) |
			D64_RC_RE;
		control |= (D64_RC_PC_4_DESC) << D64_RC_PT_SHIFT;
		control &= ~D64_RC_BL_MASK;
		control |= (0x3) << D64_RC_BL_SHIFT;

		writel(control,
		       privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_REG);
	} else {
		/* Disable RX DMA */

		/* PR2414 WAR: DMA engines are not disabled until
		 * transfer completes
		 */
		writel(0, (privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_REG));
		SPINWAIT(((status =
			(readl(privp->hw.reg.amac_core +
				GMAC_DMA_RX_STATUS0_REG)
			& D64_RS0_RS_MASK)) != D64_RS0_RS_DISABLED),
			 100, status);
	}

	return status;
}

void bcm_amac_dma_stop(struct bcm_amac_priv *privp)
{
	struct amac_dma_priv *dma_p = &privp->dma;
	u32 i;

	/* Stop the RX DMA */
	bcm_amac_enable_rx_dma(privp, false);

	/* Disable RX Interrupt */
	bcm_amac_enable_intr(privp, BCM_AMAC_DIR_RX, false);

#ifndef RMO_SUPPORT
	/* Free Rx buffers */
	for (i = 0; i < AMAC_DMA_RX_DESC_CNT; i++)
		if (privp->dma.rx_skb_list[i].skb)
			amac_free_rx_skb(privp, AMAC_DMA_RX_BUF_LEN,
					 &privp->dma.rx_skb_list[i]);
#endif

#ifndef RMO_SUPPORT
	/* Stop the TX DMA */
	bcm_amac_enable_tx_dma(privp, false);

	/* Disable TX Interrupt */
	bcm_amac_enable_intr(privp, BCM_AMAC_DIR_TX, false);

	/* Free Tx buffers */
	bcm_amac_tx_clean(privp);
#endif

	dma_p->tx.ring_len = 0;
	dma_p->rx.ring_len = 0;
}

/* bcm_amac_dma_start() - Initialize and start the DMA.
 * @privp: driver info pointer
 *
 * Both RX and TX are initialized.
 * Only RX is enabled, TX is enabled as required.
 *
 * Returns: '0' or error
 */
int bcm_amac_dma_start(struct bcm_amac_priv *privp)
{
	int rc;

	if (!privp)
		return -EINVAL;
	/* amac_dma_ctrlflags(privp, (DMA_CTRL_ROC | DMA_CTRL_PEN), 0); */

	rc = amac_dma_rx_init(privp); /* Initialize RX DMA */
	if (rc) {
		netdev_err(privp->ndev, "Failed!! DMA RX Init\n");
		goto dma_start_err;
	}

	rc = amac_dma_tx_init(privp); /* Initialize TX DMA */
	if (rc != 0) {
		netdev_err(privp->ndev, "Failed!! DMA TX Init\n");
		goto dma_start_err;
	}

	bcm_amac_enable_intr(privp, BCM_AMAC_DIR_TX, true);
	bcm_amac_enable_intr(privp, BCM_AMAC_DIR_RX, true);
	/* enable the overflow continue feature and disable parity
	 * amac_dma_ctrlflags(privp, DMA_CTRL_ROC |
	 *		   DMA_CTRL_PEN, DMA_CTRL_ROC);
	 */

	/* Enable RX DMA */
	rc = bcm_amac_enable_rx_dma(privp, true);
	if (rc) {
		netdev_err(privp->ndev, "Rx DMA enable failed!\n");
		goto dma_start_err;
	}

	/* the rx descriptor ring should have
	 * the addresses set properly
	 * set the lastdscr for the rx ring
	 */
	writel((unsigned long)((privp->dma.rx.descp) +
	       (AMAC_DMA_RX_DESC_CNT - 1) *
	       sizeof(struct amac_dma64_desc)) &
	       D64_XP_LD_MASK,
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_REG));

	return 0;

dma_start_err:
	/* Stop DMA */
	bcm_amac_dma_stop(privp);

	return rc;
}

struct amac_header {
	u16 pkt_len;
	u16 pkt_info;
};

/* bcm_amac_dma_get_rx_data() - Retrieves RX data if available.
 * @privp: driver info pointer
 * @skb: skb pointer
 *
 * If data is available, the function updates the rx pointer register.
 * It also strips out the hw specific status from the data.
 *
 * Returns: '0' if no frames, 'length' of packet or error
 */
int bcm_amac_dma_get_rx_data(struct bcm_amac_priv *privp,
			     struct sk_buff **skbp)
{
	u32 rd_offset = HWRXOFF;
	int len, pkt_info;
	struct amac_header *ptr;
	dma_addr_t  old_dma_buff;
	u32 rx_ptr;
	char *bufp;
	struct amac_dma64_desc *descp;
	struct amac_dma64_desc *rx_ptr_desc;
	struct amac_dma_priv *dmap = &privp->dma;
	struct skb_list_node  read_skb_node;
	struct skb_list_node *const node =
		&dmap->rx_skb_list[dmap->rx.index];
#ifdef AMAC_DEBUG
	int j;

	pr_debug("bcm_amac_dma_get_rx_data called\n");
#endif

	/* Get frame descriptor */
	descp = (&((struct amac_dma64_desc *)(dmap->rx.descp))[dmap->rx.index]);
	descp->ctrl2 = cpu_to_le32(AMAC_DMA_RX_BUF_LEN);

	rx_ptr_desc = (struct amac_dma64_desc *)dmap->rx.base_addr;
	rx_ptr = ((unsigned long)(&rx_ptr_desc[dmap->rx.index]) & 0xFFFFFFFF);

	/* Save current skb, we will allocate a new one in place */
	memcpy(&read_skb_node, node, sizeof(struct skb_list_node));

	old_dma_buff = read_skb_node.dma_addr;
	node->dma_addr = 0;
	/* Allocate and re-arm the descriptor with new skb */
	len = amac_alloc_rx_skb(privp,
				AMAC_DMA_RX_BUF_LEN,
				node);
	if (len) {
		netdev_err(privp->ndev, "SKB alloc error\n");
		/* skb allocation error, no new skb available.
		 * So leave the existing skb in place for re-use.
		 * Will not process the frame further, dropping it.
		 * That's the best we can do at this point.
		 */
		goto rx_dma_data_done;
	}

	/* Re-arm descriptor with new skb */
	descp->addrlow = DMA_LOW_ADDR(node->dma_addr);
	descp->addrhigh = DMA_HIGH_ADDR(node->dma_addr);

	dma_sync_single_for_cpu(&privp->pdev->dev,
				read_skb_node.dma_addr,
				AMAC_DMA_RX_BUF_LEN,
				DMA_FROM_DEVICE);

	*skbp = read_skb_node.skb;

	/* Process the SKB with data */
	bufp = (*skbp)->data;
	ptr = (struct amac_header *)bufp;
	len = cpu_to_le16(ptr->pkt_len);
	pkt_info = cpu_to_le16(ptr->pkt_info);
#ifdef AMAC_DEBUG
	netdev_dbg(privp->ndev, "Packet_info = %#X:len = %d\n", pkt_info, len);
#endif

	if (unlikely((pkt_info & RCV_PKT_INFO_RX_OVERFLOW) ||
		     (pkt_info & RCV_PKT_INFO_CRC_ERR_OFFSET) ||
		     (len > DMA_RX_BUF_LEN))) {
		netdev_err(privp->ndev,
			   "Rx Packet Rrror, Packet_info = %#X\n", pkt_info);
		len = -EBADMSG;
		amac_free_rx_skb(privp, AMAC_DMA_RX_BUF_LEN, &read_skb_node);
		*skbp = NULL;
		goto rx_dma_data_done;
	}

	len = cpu_to_le16(*((u16 *)bufp));

	/* Received an invalid frame length */
	if (len > AMAC_DMA_RX_BUF_LEN) {
		netdev_err(privp->ndev, "Invalid frame length, len=%d\n", len);

		/* Dropping frame, free the saved skb node */
		amac_free_rx_skb(privp, AMAC_DMA_RX_BUF_LEN, &read_skb_node);
		*skbp = NULL;

		len = -EBADMSG;
		goto rx_dma_data_done;
	}

	/* Point to real data */
	bufp += rd_offset;
	if (amac_version == 3)
		bufp += BRCM_HEADER_LEN;

#ifdef AMAC_DEBUG
	for (j = 0; j < 64; j++)
		pr_debug("Rx skb[%d]=%#X ", j, bufp[j]);
	pr_emerg("\n");
#endif
	/* Realign the data in SKB */
	(*skbp)->data = bufp;

rx_dma_data_done:
	/* Update RX pointer */
	writel(rx_ptr, (privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_REG));

	/* Increment descp index */
	dmap->rx.index = (dmap->rx.index + 1) % AMAC_DMA_RX_DESC_CNT;

	return len;
}

static int bcm_amac_tx_tso_csum_offload(struct bcm_amac_priv *privp,
					struct sk_buff *skb,
					u32 *shim_hdr)
{
	struct iphdr *iph = ip_hdr(skb);
	__be16 protocol;
	__sum16 csum;
	u32 mss = 0;

	if (amac_version != 3)
		return 0;

	if (skb_headroom(skb) <= SHIM_HEADER_SIZE_BYTES) {
		netdev_err(privp->ndev, "NotEnough HeadRoom\n");
		dev_kfree_skb_any(skb);

		privp->ndev->stats.tx_bytes -= skb->len;
		privp->ndev->stats.tx_packets--;
		privp->ndev->stats.tx_fifo_errors++;
		privp->ndev->stats.tx_dropped++;
		return -EPERM;
	}
	shim_hdr[0] = 0;
	shim_hdr[1] = 0;
	if ((privp->ndev->features & NETIF_F_ALL_TSO) ||
	    (privp->ndev->features & NETIF_F_ALL_CSUM)) {
		protocol = vlan_get_protocol(skb);

		if ((protocol == htons(ETH_P_IPV6)) ||
		    (protocol == htons(ETH_P_IP))) {
			if (skb->ip_summed == CHECKSUM_PARTIAL) {
				shim_hdr[0] |= IPV4_OFFSET(
						skb_network_offset(skb));
				switch (iph->protocol) {
				case IPPROTO_TCP:
					pr_debug("TCP:%d:\n", privp->ndev->mtu);
					shim_hdr[0] |= PROC_OP(3);
					shim_hdr[0] |= L4OFFSET(
							skb_transport_offset(
								skb));
					iph->check = 0;
					csum = ~csum_tcpudp_magic(
							iph->saddr, iph->daddr,
							0, IPPROTO_TCP, 0);
					tcp_hdr(skb)->check = csum;
					break;
				case IPPROTO_UDP:
					pr_debug("UDP:%d\n", privp->ndev->mtu);
					shim_hdr[0] |= PROC_OP(4);
					shim_hdr[0] |= L4OFFSET(
							skb_transport_offset(
								skb));
					iph->check = 0;
					csum = ~csum_tcpudp_magic(
							iph->saddr, iph->daddr,
							0, IPPROTO_UDP, 0);
					udp_hdr(skb)->check = csum;
					break;
				default:
					pr_info("Not Supported protocol=%d\n",
						iph->protocol);
					break;
				}
				if (skb_shinfo(skb)->gso_size) {
					pr_debug("BSP Size=%d Seg=%d len=%d\n",
						 skb_shinfo(skb)->gso_size,
						 skb_shinfo(skb)->gso_segs,
						 skb->len);
					pr_debug("BSP nr_frag=%d type=%d\n",
						 skb_shinfo(skb)->nr_frags,
						 skb_shinfo(skb)->gso_type);
					mss = privp->ndev->mtu;
					if (iph->protocol == IPPROTO_TCP)
						mss -= (skb_transport_offset(
							skb) +
							tcp_hdrlen(skb));
					if (iph->protocol == IPPROTO_UDP)
						mss -= (skb_transport_offset(
							skb) + 8);
					shim_hdr[1] |= MAX_SEG_SIZE(mss);
				} else {
					shim_hdr[1] |= MAX_SEG_SIZE(0);
					pr_debug("SSP len=%d datalen=%d\n",
						 skb->len, skb->data_len);
				}
			}
		}
	}

	skb_push(skb, SHIM_HEADER_SIZE_BYTES);
	pr_debug("shim_hdr[0]=%d:%x shim_hdr[1]=%d:%x\n",
		 shim_hdr[0], shim_hdr[0], shim_hdr[1], shim_hdr[1]);
	pr_debug("tcp_offset:%d tcp_len:%d\n",
		 skb_transport_offset(skb), tcp_hdrlen(skb));

	skb->data[0] = shim_hdr[0] & 0xFF;
	skb->data[1] = (shim_hdr[0] & 0xFF00) >> 8;
	skb->data[2] = (shim_hdr[0] & 0xFF0000) >> 16;
	skb->data[3] = (shim_hdr[0] & 0xFF000000) >> 24;

	skb->data[4] = shim_hdr[1] & 0xFF;
	skb->data[5] = (shim_hdr[1] & 0xFF00) >> 8;
	skb->data[6] = (shim_hdr[1] & 0xFF0000) >> 16;
	skb->data[7] = (shim_hdr[1] & 0xFF000000) >> 24;
	pr_debug("nr_frags:%d len:%d data_len=%d\n",
		 skb_shinfo(skb)->nr_frags, skb->len, skb->data_len);
	return 0;
}

/* bcm_amac_tx_send_packet() - Ethernet TX routine.
 * @privp: driver info pointer
 *
 * Called within the TX tasklet with packets in fifo.
 * Gets data from the queue, formats the descriptor ,
 * setup and starts the TX DMA.
 *
 * Returns: none
 */
void bcm_amac_tx_send_packet(struct bcm_amac_priv *privp)
{
	struct amac_dma_priv *dmap = &privp->dma;
	struct sk_buff *skb;
	u32    len;
#ifdef AMAC_DEBUG
	u32	intr_status, intr_mask, xstatus0, xstatus1, xctrl;
	u32	ctf_miss_cnt, ctf_hit_cnt, ctf_fifo_high_wmark;
	u32	i;
#endif

#ifndef RMO_SUPPORT
	int curr = 0;
	int desc_idx = 0;
	struct amac_dma64_desc *descp = NULL;
	u32 last_desc;
	dma_addr_t buf_dma;
	dma_addr_t buf_dma1;
	u32 dma_flags;
	u32 ctrl;
#else
	int cos = 0;
	u32 num_pkts;

	/*FIXME: Which CoS value to consider here? */
	num_pkts = brm_tx_available_pkt_slots(privp->pdev, BRM_ETH,
					      privp->amac_id, cos);
#endif
	u32 shim_hdr[2];
	struct skb_frag_struct *frag;
	int frag_no;

	/* Build descriptor chain */
#ifdef AMAC_DEBUG
	intr_mask =  readl(privp->hw.reg.amac_core + GMAC_INT_MASK_REG);

	ctf_miss_cnt = readl(privp->hw.reg.ctf_regs + 0xac);
	ctf_hit_cnt = readl(privp->hw.reg.ctf_regs + 0xa8);
	ctf_fifo_high_wmark = readl(privp->hw.reg.ctf_regs + 0x248);

	pr_info("| ctf_hit_cnt=%#X, ctf_miss_cnt=%#X, ctf_fifo_high_wmark= %#X\n",
		ctf_hit_cnt, ctf_miss_cnt, ctf_fifo_high_wmark);
	xstatus0 = readl(privp->hw.reg.amac_core +
					GMAC_DMA_TX_STATUS0_REG);
	xstatus1 = readl(privp->hw.reg.amac_core +
					GMAC_DMA_TX_STATUS1_REG);

	intr_status =  readl(privp->hw.reg.amac_core + GMAC_INT_STATUS_REG);
	xctrl = readl(privp->hw.reg.amac_core +
					GMAC_DMA_TX_CTRL_REG);
	pr_info("|intr_status=%#X, intr_mask=%#X, xstatus0:%#X: xstatus1:%#x:xctrl:%#X|\n",
		intr_status, intr_mask, xstatus0, xstatus1, xctrl);
#endif

	/* Build descriptor chain */
	while (((len = kfifo_out_spinlocked(&dmap->txfifo,
					    (unsigned char *)&skb,
					    sizeof(struct sk_buff *),
					    &privp->lock)) ==
		   sizeof(struct sk_buff *)) &&
		   (curr < (dmap->tx_max_pkts - 1))) {
#ifdef AMAC_DEBUG
		for (i = 0; i < skb->len; i++) {
			if (!(i % 16))
				pr_debug("\n%s: 0x", __func__);

			pr_debug("%02x", (unsigned char)(skb->data[i]));
		}
#endif

#ifdef RMO_SUPPORT
		/* Joey Hack: Why we are not checking num_pkts here to make
		 *  sure there is room for in RMO host tx ring
		 */
		num_pkts = 1;
		pr_debug("%s: ndev %p start to transmit skb 0x%p len 0x%x\n",
			 __func__, privp->ndev, skb, skb->len);
#ifdef AMAC_DEBUG
		for (i = 0; i < skb->len; i++) {
			if (!(i % 16))
				pr_debug("\n%s: 0x", __func__);

			pr_debug("%02x", (unsigned char)(skb->data[i]));
		}
#endif

		/* cos = find_cos_of_skb(skb);*/  /* TODO: implementg this */
		brm_tx_pkts(privp->pdev, BRM_ETH, privp->amac_id,
			    cos, &skb, &num_pkts);
		/*TODO:Optimization: convert list of skb into an array of skb
		 * and call tx API at once
		 */
#else
		/* Indicate we are busy sending a packet */
		atomic_set(&privp->dma.tx_dma_busy, BCM_AMAC_DMA_BUSY);

		len = skb->len;

		if (unlikely(len < MIN_FRAME_LEN)) {
			/* Clear the padded memory to avoid 'etherleak'
			 * vulnerability
			 */
#ifdef AMAC_DEBUG
			pr_debug("[%s():%d]len:%d, skb->data + len:%p\n",
				 __func__, __LINE__, len, skb->data + len);
#endif
			memset(skb->data + len, 0, (MIN_FRAME_LEN - len));
			len = MIN_FRAME_LEN;
		}
		/* Timestamp the packet */
		skb_tx_timestamp(skb);

		if (bcm_amac_tx_tso_csum_offload(privp, skb, &shim_hdr[0]) < 0)
			continue;

		if (skb_shinfo(skb)->nr_frags) {
			pr_debug("fragemented packet\n");

			len = skb_headlen(skb);

			buf_dma = dma_map_single(&privp->pdev->dev, skb->data,
						 len, DMA_TO_DEVICE);
			if (dma_mapping_error(&privp->pdev->dev, buf_dma)) {
				netdev_err(privp->ndev,
					   "TX: DMA mapping Failed !!\n");
				dev_kfree_skb_any(skb);
				privp->ndev->stats.tx_bytes -= len;
				privp->ndev->stats.tx_packets--;
				privp->ndev->stats.tx_fifo_errors++;
				privp->ndev->stats.tx_dropped++;
				continue;
			}

			descp = (&((struct amac_dma64_desc *)
					(dmap->tx.descp))[(desc_idx)]);

			descp = &((struct amac_dma64_desc *)
				  (dmap->tx.descp))[(desc_idx)];
			dma_flags = D64_CTRL1_SOF;

			ctrl      = (len & D64_CTRL2_BC_MASK);
			descp->addrhigh = cpu_to_le32((u32)(buf_dma >> 32));
			descp->addrlow  = cpu_to_le32((u32)buf_dma);
			descp->ctrl1    = cpu_to_le32((u32)dma_flags);
			descp->ctrl2    = cpu_to_le32((u32)ctrl);

			/* Add skb to list */
			dmap->tx_skb_list[curr].skb = skb;
			dmap->tx_skb_list[curr].len = len;
			dmap->tx_skb_list[curr].dma_addr = buf_dma;

			desc_idx++;
			curr++;
			for (frag_no = 0; frag_no < skb_shinfo(skb)->nr_frags;
			     frag_no++) {
				frag = &skb_shinfo(skb)->frags[frag_no];

				buf_dma1 = skb_frag_dma_map(
						&privp->pdev->dev,
						frag, 0, skb_frag_size(frag),
						DMA_TO_DEVICE);
				if (dma_mapping_error(&privp->pdev->dev,
						      buf_dma1))
					pr_warn("#ATTN: Mapping error\n");

				descp = &((struct amac_dma64_desc *)
					  (dmap->tx.descp))[(desc_idx)];
				pr_debug("EOF %d\n", skb_frag_size(frag));
				if (frag_no == (skb_shinfo(skb)->nr_frags - 1))
					dma_flags = D64_CTRL1_EOF;
				else
					dma_flags = 0;
				ctrl = skb_frag_size(frag) & D64_CTRL2_BC_MASK;
				descp->addrhigh = cpu_to_le32(
							(u32)(buf_dma1 >> 32));
				descp->addrlow  = cpu_to_le32((u32)buf_dma1);
				descp->ctrl1    = cpu_to_le32((u32)dma_flags);
				descp->ctrl2    = cpu_to_le32((u32)ctrl);

				desc_idx++;
				curr++;
			}
		} else {
			len = skb->len;

			buf_dma = dma_map_single(&privp->pdev->dev, skb->data,
						 len, DMA_TO_DEVICE);
			if (dma_mapping_error(&privp->pdev->dev, buf_dma)) {
				netdev_err(privp->ndev,
					   "TX: DMA mapping Failed !!\n");
				dev_kfree_skb_any(skb);
				privp->ndev->stats.tx_bytes -= len;
				privp->ndev->stats.tx_packets--;
				privp->ndev->stats.tx_fifo_errors++;
				privp->ndev->stats.tx_dropped++;
				continue;
			}

			descp = (&((struct amac_dma64_desc *)
					(dmap->tx.descp))[(desc_idx)]);

			descp = &((struct amac_dma64_desc *)
				  (dmap->tx.descp))[(desc_idx)];
			dma_flags = D64_CTRL1_SOF | D64_CTRL1_EOF;
			ctrl      = (len & D64_CTRL2_BC_MASK);
			descp->addrhigh = cpu_to_le32((u32)(buf_dma >> 32));
			descp->addrlow  = cpu_to_le32((u32)buf_dma);
			descp->ctrl1    = cpu_to_le32((u32)dma_flags);
			descp->ctrl2    = cpu_to_le32((u32)ctrl);

			/* Add skb to list */
			dmap->tx_skb_list[curr].skb = skb;
			dmap->tx_skb_list[curr].len = len;
			dmap->tx_skb_list[curr].dma_addr = buf_dma;

			desc_idx++;
			curr++;
		}
#endif
	}

#ifndef RMO_SUPPORT
	if (descp) {
		dmap->tx_curr = curr;
		dmap->tx.index = desc_idx;

		/* Interrupt after the last one*/
		descp->ctrl1 |= cpu_to_le32(D64_CTRL1_IOC);

		descp = (&((struct amac_dma64_desc *)
					(dmap->tx.descp))[(desc_idx)]);

		/* Mark last descriptor as EOT */
		descp->ctrl1 = cpu_to_le32(D64_CTRL1_EOT);

		last_desc = ((unsigned long)(&((struct amac_dma64_desc *)
			(dmap->tx.base_addr))[desc_idx]));
		last_desc &= D64_XP_LD_MASK;
		/* Disable TX DMA */
		bcm_amac_enable_tx_dma(privp, false);

		/* initailize the DMA channel */
		writel(lower_32_bits((dma_addr_t)
			&((struct amac_dma64_desc *)(dmap->tx.base_addr))[0]),
			(privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_LO_REG));
		writel(upper_32_bits((dma_addr_t)&
			((struct amac_dma64_desc *)(dmap->tx.base_addr))[0]),
			(privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_HI_REG));

		/* Ensure the DMA descriptors are setup before progressing */

		/* Enable TX DMA Interrupts */
		bcm_amac_enable_intr(privp, BCM_AMAC_DIR_TX, true);

		/* Enable TX DMA */
		bcm_amac_enable_tx_dma(privp, true);

		/* update the dma last descriptor */
		writel(last_desc,
		       (privp->hw.reg.amac_core + GMAC_DMA_TX_PTR_REG));
#endif
	}
}

