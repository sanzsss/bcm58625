/*
 * Driver for (BCM4706)? GBit MAC core on BCMA bus.
 *
 * Copyright (C) 2012 Rafał Miłecki <zajec5@gmail.com>
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */


#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/bcma/bcma.h>
#include <linux/etherdevice.h>
#include <linux/bcm47xx_nvram.h>
#include <linux/rtnetlink.h>
#include "bgmac.h"

#ifdef CONFIG_BGMAC_OFFLOAD
#include "bcm_ring_offload.h"
#endif /* CONFIG_BGMAC_OFFLOAD */

#define TX_TIMEOUT (5 * HZ)

static bool bgmac_wait_value(struct bgmac *bgmac, u16 reg, u32 mask,
			     u32 value, int timeout)
{
	u32 val;
	int i;

	for (i = 0; i < timeout / 10; i++) {
		val = bgmac_read(bgmac, reg);
		if ((val & mask) == value)
			return true;
		udelay(10);
	}
	dev_err(bgmac->dev, "Timeout waiting for reg 0x%X\n", reg);
	return false;
}

/**************************************************
 * DMA
 **************************************************/

static void bgmac_dma_tx_reset(struct bgmac *bgmac, struct bgmac_dma_ring *ring)
{
	u32 val;
	int i;

	if (!ring->mmio_base)
		return;

	/* Suspend DMA TX ring first.
	 * bgmac_wait_value doesn't support waiting for any of few values, so
	 * implement whole loop here.
	 */
	bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_TX_CTL,
		    BGMAC_DMA_TX_SUSPEND);
	for (i = 0; i < 10000 / 10; i++) {
		val = bgmac_read(bgmac, ring->mmio_base + BGMAC_DMA_TX_STATUS);
		val &= BGMAC_DMA_TX_STAT;
		if (val == BGMAC_DMA_TX_STAT_DISABLED ||
		    val == BGMAC_DMA_TX_STAT_IDLEWAIT ||
		    val == BGMAC_DMA_TX_STAT_STOPPED) {
			i = 0;
			break;
		}
		udelay(10);
	}
	if (i)
		dev_err(bgmac->dev, "Timeout suspending DMA TX ring 0x%X (BGMAC_DMA_TX_STAT: 0x%08X)\n",
			ring->mmio_base, val);

	/* Remove SUSPEND bit */
	bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_TX_CTL, 0);
	if (!bgmac_wait_value(bgmac,
			      ring->mmio_base + BGMAC_DMA_TX_STATUS,
			      BGMAC_DMA_TX_STAT, BGMAC_DMA_TX_STAT_DISABLED,
			      10000)) {
		dev_warn(bgmac->dev, "DMA TX ring 0x%X wasn't disabled on time, waiting additional 300us\n",
			 ring->mmio_base);
		udelay(300);
		val = bgmac_read(bgmac, ring->mmio_base + BGMAC_DMA_TX_STATUS);
		if ((val & BGMAC_DMA_TX_STAT) != BGMAC_DMA_TX_STAT_DISABLED)
			dev_err(bgmac->dev, "Reset of DMA TX ring 0x%X failed\n",
				ring->mmio_base);
	}
}

static void bgmac_dma_tx_enable(struct bgmac *bgmac,
				struct bgmac_dma_ring *ring)
{
	u32 ctl;

	ctl = bgmac_read(bgmac, ring->mmio_base + BGMAC_DMA_TX_CTL);
	if (bgmac->feature_flags & BGMAC_FEAT_TX_MASK_SETUP) {
		ctl &= ~BGMAC_DMA_TX_BL_MASK;
		ctl |= BGMAC_DMA_TX_BL_128 << BGMAC_DMA_TX_BL_SHIFT;

		ctl &= ~BGMAC_DMA_TX_MR_MASK;
		ctl |= BGMAC_DMA_TX_MR_2 << BGMAC_DMA_TX_MR_SHIFT;

		ctl &= ~BGMAC_DMA_TX_PC_MASK;
		ctl |= BGMAC_DMA_TX_PC_16 << BGMAC_DMA_TX_PC_SHIFT;

		ctl &= ~BGMAC_DMA_TX_PT_MASK;
		ctl |= BGMAC_DMA_TX_PT_8 << BGMAC_DMA_TX_PT_SHIFT;
	}
	ctl |= BGMAC_DMA_TX_ENABLE;
	ctl |= BGMAC_DMA_TX_PARITY_DISABLE;
	bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_TX_CTL, ctl);
}

static void
bgmac_dma_tx_add_buf(struct bgmac *bgmac, struct bgmac_dma_ring *ring,
		     int i, int len, u32 ctl0)
{
	struct bgmac_slot_info *slot;
	struct bgmac_dma_desc *dma_desc;
	u32 ctl1;

	if (i == BGMAC_TX_RING_SLOTS - 1)
		ctl0 |= BGMAC_DESC_CTL0_EOT;

	ctl1 = len & BGMAC_DESC_CTL1_LEN;

	slot = &ring->slots[i];
	dma_desc = &ring->cpu_base[i];
	dma_desc->addr_low = cpu_to_le32(lower_32_bits(slot->dma_addr));
	dma_desc->addr_high = cpu_to_le32(upper_32_bits(slot->dma_addr));
	dma_desc->ctl0 = cpu_to_le32(ctl0);
	dma_desc->ctl1 = cpu_to_le32(ctl1);
}

static netdev_tx_t bgmac_dma_tx_add(struct bgmac *bgmac,
				    struct bgmac_dma_ring *ring,
				    struct sk_buff *skb)
{
	struct device *dma_dev = bgmac->dma_dev;
	struct net_device *net_dev = bgmac->net_dev;
	int index = ring->end % BGMAC_TX_RING_SLOTS;
	struct bgmac_slot_info *slot = &ring->slots[index];
	int nr_frags;
	u32 flags;
	int i;

	if (skb->len > BGMAC_DESC_CTL1_LEN) {
		netdev_err(bgmac->net_dev, "Too long skb (%d)\n", skb->len);
		goto err_drop;
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL)
		skb_checksum_help(skb);

	nr_frags = skb_shinfo(skb)->nr_frags;

	/* ring->end - ring->start will return the number of valid slots,
	 * even when ring->end overflows
	 */
	if (ring->end - ring->start + nr_frags + 1 >= BGMAC_TX_RING_SLOTS) {
		netdev_err(bgmac->net_dev, "TX ring is full, queue should be stopped!\n");
		netif_stop_queue(net_dev);
		return NETDEV_TX_BUSY;
	}

	slot->dma_addr = dma_map_single(dma_dev, skb->data, skb_headlen(skb),
					DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dma_dev, slot->dma_addr)))
		goto err_dma_head;

	flags = BGMAC_DESC_CTL0_SOF;
	if (!nr_frags)
		flags |= BGMAC_DESC_CTL0_EOF | BGMAC_DESC_CTL0_IOC;

	bgmac_dma_tx_add_buf(bgmac, ring, index, skb_headlen(skb), flags);
	flags = 0;

	for (i = 0; i < nr_frags; i++) {
		struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);

		index = (index + 1) % BGMAC_TX_RING_SLOTS;
		slot = &ring->slots[index];
		slot->dma_addr = skb_frag_dma_map(dma_dev, frag, 0,
						  len, DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(dma_dev, slot->dma_addr)))
			goto err_dma;

		if (i == nr_frags - 1)
			flags |= BGMAC_DESC_CTL0_EOF | BGMAC_DESC_CTL0_IOC;

		bgmac_dma_tx_add_buf(bgmac, ring, index, len, flags);
	}

	slot->skb = skb;
	ring->end += nr_frags + 1;
	netdev_sent_queue(net_dev, skb->len);

	wmb();

	/* Increase ring->end to point empty slot. We tell hardware the first
	 * slot it should *not* read.
	 */
	bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_TX_INDEX,
		    ring->index_base +
		    (ring->end % BGMAC_TX_RING_SLOTS) *
		    sizeof(struct bgmac_dma_desc));

	if (ring->end - ring->start >= BGMAC_TX_RING_SLOTS - 8)
		netif_stop_queue(net_dev);

	return NETDEV_TX_OK;

err_dma:
	dma_unmap_single(dma_dev, slot->dma_addr, skb_headlen(skb),
			 DMA_TO_DEVICE);

	while (i-- > 0) {
		int index = (ring->end + i) % BGMAC_TX_RING_SLOTS;
		struct bgmac_slot_info *slot = &ring->slots[index];
		u32 ctl1 = le32_to_cpu(ring->cpu_base[index].ctl1);
		int len = ctl1 & BGMAC_DESC_CTL1_LEN;

		dma_unmap_page(dma_dev, slot->dma_addr, len, DMA_TO_DEVICE);
	}

err_dma_head:
	netdev_err(bgmac->net_dev, "Mapping error of skb on ring 0x%X\n",
		   ring->mmio_base);

err_drop:
	dev_kfree_skb(skb);
	net_dev->stats.tx_dropped++;
	net_dev->stats.tx_errors++;
	return NETDEV_TX_OK;
}

/* Free transmitted packets */
static void bgmac_dma_tx_free(struct bgmac *bgmac, struct bgmac_dma_ring *ring)
{
	struct device *dma_dev = bgmac->dma_dev;
	int empty_slot;
	bool freed = false;
	unsigned bytes_compl = 0, pkts_compl = 0;

	/* The last slot that hardware didn't consume yet */
	empty_slot = bgmac_read(bgmac, ring->mmio_base + BGMAC_DMA_TX_STATUS);
	empty_slot &= BGMAC_DMA_TX_STATDPTR;
	empty_slot -= ring->index_base;
	empty_slot &= BGMAC_DMA_TX_STATDPTR;
	empty_slot /= sizeof(struct bgmac_dma_desc);

	while (ring->start != ring->end) {
		int slot_idx = ring->start % BGMAC_TX_RING_SLOTS;
		struct bgmac_slot_info *slot = &ring->slots[slot_idx];
		u32 ctl0, ctl1;
		int len;

		if (slot_idx == empty_slot)
			break;

		ctl0 = le32_to_cpu(ring->cpu_base[slot_idx].ctl0);
		ctl1 = le32_to_cpu(ring->cpu_base[slot_idx].ctl1);
		len = ctl1 & BGMAC_DESC_CTL1_LEN;
		if (ctl0 & BGMAC_DESC_CTL0_SOF)
			/* Unmap no longer used buffer */
			dma_unmap_single(dma_dev, slot->dma_addr, len,
					 DMA_TO_DEVICE);
		else
			dma_unmap_page(dma_dev, slot->dma_addr, len,
				       DMA_TO_DEVICE);

		if (slot->skb) {
			u64_stats_update_begin(&bgmac->tx_stats.syncp);
			bgmac->tx_stats.bytes += slot->skb->len;
			bgmac->tx_stats.packets++;
			u64_stats_update_end(&bgmac->tx_stats.syncp);
			bytes_compl += slot->skb->len;
			pkts_compl++;

			/* Free memory! :) */
			dev_kfree_skb(slot->skb);
			slot->skb = NULL;
		}

		slot->dma_addr = 0;
		ring->start++;
		freed = true;
	}

	if (!pkts_compl)
		return;

	netdev_completed_queue(bgmac->net_dev, pkts_compl, bytes_compl);

	if (netif_queue_stopped(bgmac->net_dev))
		netif_wake_queue(bgmac->net_dev);
}

static void bgmac_dma_rx_reset(struct bgmac *bgmac, struct bgmac_dma_ring *ring)
{
	if (!ring->mmio_base)
		return;

	bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_RX_CTL, 0);
	if (!bgmac_wait_value(bgmac,
			      ring->mmio_base + BGMAC_DMA_RX_STATUS,
			      BGMAC_DMA_RX_STAT, BGMAC_DMA_RX_STAT_DISABLED,
			      10000))
		dev_err(bgmac->dev, "Reset of ring 0x%X RX failed\n",
			ring->mmio_base);
}

static void bgmac_dma_rx_enable(struct bgmac *bgmac,
				struct bgmac_dma_ring *ring)
{
	u32 ctl;

	ctl = bgmac_read(bgmac, ring->mmio_base + BGMAC_DMA_RX_CTL);

	/* preserve ONLY bits 16-17 from current hardware value */
	ctl &= BGMAC_DMA_RX_ADDREXT_MASK;

	if (bgmac->feature_flags & BGMAC_FEAT_RX_MASK_SETUP) {
		ctl &= ~BGMAC_DMA_RX_BL_MASK;
		ctl |= BGMAC_DMA_RX_BL_128 << BGMAC_DMA_RX_BL_SHIFT;

		ctl &= ~BGMAC_DMA_RX_PC_MASK;
		ctl |= BGMAC_DMA_RX_PC_8 << BGMAC_DMA_RX_PC_SHIFT;

		ctl &= ~BGMAC_DMA_RX_PT_MASK;
		ctl |= BGMAC_DMA_RX_PT_1 << BGMAC_DMA_RX_PT_SHIFT;
	}
	ctl |= BGMAC_DMA_RX_ENABLE;
	ctl |= BGMAC_DMA_RX_PARITY_DISABLE;
	ctl |= BGMAC_DMA_RX_OVERFLOW_CONT;
	ctl |= BGMAC_RX_FRAME_OFFSET << BGMAC_DMA_RX_FRAME_OFFSET_SHIFT;
	bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_RX_CTL, ctl);
}

static int bgmac_dma_rx_skb_for_slot(struct bgmac *bgmac,
				     struct bgmac_slot_info *slot)
{
	struct device *dma_dev = bgmac->dma_dev;
	dma_addr_t dma_addr;
	struct bgmac_rx_header *rx;
	void *buf;

	/* Alloc skb */
	buf = netdev_alloc_frag(bgmac->rx_alloc_size);
	if (!buf)
		return -ENOMEM;

	/* Poison - if everything goes fine, hardware will overwrite it */
	rx = buf + BGMAC_RX_BUF_OFFSET;
	rx->len = cpu_to_le16(0xdead);
	rx->flags = cpu_to_le16(0xbeef);

	/* Map skb for the DMA */
	dma_addr = dma_map_single(dma_dev, buf + BGMAC_RX_BUF_OFFSET,
				  bgmac->rx_buf_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(dma_dev, dma_addr)) {
		netdev_err(bgmac->net_dev, "DMA mapping error\n");
		put_page(virt_to_head_page(buf));
		return -ENOMEM;
	}

	/* Update the slot */
	slot->buf = buf;
	slot->dma_addr = dma_addr;

	return 0;
}

static void bgmac_dma_rx_update_index(struct bgmac *bgmac,
				      struct bgmac_dma_ring *ring)
{
	dma_wmb();

	bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_RX_INDEX,
		    ring->index_base +
		    ring->end * sizeof(struct bgmac_dma_desc));
}

static void bgmac_dma_rx_setup_desc(struct bgmac *bgmac,
				    struct bgmac_dma_ring *ring, int desc_idx)
{
	struct bgmac_dma_desc *dma_desc = ring->cpu_base + desc_idx;
	u32 ctl0 = 0, ctl1 = 0;

	if (desc_idx == BGMAC_RX_RING_SLOTS - 1)
		ctl0 |= BGMAC_DESC_CTL0_EOT;
	ctl1 |= bgmac->rx_buf_size & BGMAC_DESC_CTL1_LEN;
	/* Is there any BGMAC device that requires extension? */
	/* ctl1 |= (addrext << B43_DMA64_DCTL1_ADDREXT_SHIFT) &
	 * B43_DMA64_DCTL1_ADDREXT_MASK;
	 */

	dma_desc->addr_low = cpu_to_le32(lower_32_bits(ring->slots[desc_idx].dma_addr));
	dma_desc->addr_high = cpu_to_le32(upper_32_bits(ring->slots[desc_idx].dma_addr));
	dma_desc->ctl0 = cpu_to_le32(ctl0);
	dma_desc->ctl1 = cpu_to_le32(ctl1);

	ring->end = desc_idx;
}

static void bgmac_dma_rx_poison_buf(struct bgmac *bgmac, struct device *dma_dev,
				    struct bgmac_slot_info *slot)
{
	struct bgmac_rx_header *rx = slot->buf + BGMAC_RX_BUF_OFFSET;

	dma_sync_single_for_cpu(dma_dev, slot->dma_addr, bgmac->rx_buf_size,
				DMA_FROM_DEVICE);
	rx->len = cpu_to_le16(0xdead);
	rx->flags = cpu_to_le16(0xbeef);
	dma_sync_single_for_device(dma_dev, slot->dma_addr, bgmac->rx_buf_size,
				   DMA_FROM_DEVICE);
}

static int bgmac_dma_rx_read(struct bgmac *bgmac, struct bgmac_dma_ring *ring,
			     int weight)
{
	u32 end_slot;
	int handled = 0;

	end_slot = bgmac_read(bgmac, ring->mmio_base + BGMAC_DMA_RX_STATUS);
	end_slot &= BGMAC_DMA_RX_STATDPTR;
	end_slot -= ring->index_base;
	end_slot &= BGMAC_DMA_RX_STATDPTR;
	end_slot /= sizeof(struct bgmac_dma_desc);

	while (ring->start != end_slot) {
		struct device *dma_dev = bgmac->dma_dev;
		struct bgmac_slot_info *slot = &ring->slots[ring->start];
		struct bgmac_rx_header *rx = slot->buf + BGMAC_RX_BUF_OFFSET;
		struct sk_buff *skb;
		void *buf = slot->buf;
		dma_addr_t dma_addr = slot->dma_addr;
		u16 len, flags;

		do {
			/* Prepare new skb as replacement */
			if (bgmac_dma_rx_skb_for_slot(bgmac, slot)) {
				bgmac_dma_rx_poison_buf(bgmac, dma_dev, slot);
				break;
			}

			/* Unmap buffer to make it accessible to the CPU */
			dma_unmap_single(dma_dev, dma_addr,
					 bgmac->rx_buf_size, DMA_FROM_DEVICE);

			/* Get info from the header */
			len = le16_to_cpu(rx->len);
			flags = le16_to_cpu(rx->flags);

			/* Check for poison and drop or pass the packet */
			if (len == 0xdead && flags == 0xbeef) {
				netdev_err(bgmac->net_dev, "Found poisoned packet at slot %d, DMA issue!\n",
					   ring->start);
				put_page(virt_to_head_page(buf));
				u64_stats_update_begin(&bgmac->rx_stats.syncp);
				bgmac->rx_stats.errors++;
				u64_stats_update_end(&bgmac->rx_stats.syncp);
				break;
			}

			if (len > bgmac->rx_alloc_size) {
				netdev_err(bgmac->net_dev, "Found oversized packet at slot %d, DMA issue!\n",
					   ring->start);
				put_page(virt_to_head_page(buf));
				u64_stats_update_begin(&bgmac->rx_stats.syncp);
				bgmac->rx_stats.length_errors++;
				bgmac->rx_stats.errors++;
				u64_stats_update_end(&bgmac->rx_stats.syncp);
				break;
			}

			if (!(bgmac->feature_flags & BGMAC_FEAT_CRC_STRIP))
				len -= ETH_FCS_LEN;

			skb = build_skb(buf, bgmac->rx_alloc_size);
			if (unlikely(!skb)) {
				netdev_err(bgmac->net_dev, "build_skb failed\n");
				put_page(virt_to_head_page(buf));
				u64_stats_update_begin(&bgmac->rx_stats.syncp);
				bgmac->rx_stats.errors++;
				u64_stats_update_end(&bgmac->rx_stats.syncp);
				break;
			}
			skb_put(skb, BGMAC_RX_FRAME_OFFSET +
				BGMAC_RX_BUF_OFFSET + len);
			skb_pull(skb, BGMAC_RX_FRAME_OFFSET +
				 BGMAC_RX_BUF_OFFSET);

			skb_checksum_none_assert(skb);
			skb->protocol = eth_type_trans(skb, bgmac->net_dev);
			u64_stats_update_begin(&bgmac->rx_stats.syncp);
			bgmac->rx_stats.bytes += len;
			bgmac->rx_stats.packets++;
			u64_stats_update_end(&bgmac->rx_stats.syncp);
			napi_gro_receive(&bgmac->napi, skb);
			handled++;
		} while (0);

		bgmac_dma_rx_setup_desc(bgmac, ring, ring->start);

		if (++ring->start >= BGMAC_RX_RING_SLOTS)
			ring->start = 0;

		if (handled >= weight) /* Should never be greater */
			break;
	}

	bgmac_dma_rx_update_index(bgmac, ring);

	return handled;
}

/* Does ring support unaligned addressing? */
static bool bgmac_dma_unaligned(struct bgmac *bgmac,
				struct bgmac_dma_ring *ring,
				enum bgmac_dma_ring_type ring_type)
{
	switch (ring_type) {
	case BGMAC_DMA_RING_TX:
		bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_TX_RINGLO,
			    0xff0);
		if (bgmac_read(bgmac, ring->mmio_base + BGMAC_DMA_TX_RINGLO))
			return true;
		break;
	case BGMAC_DMA_RING_RX:
		bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_RX_RINGLO,
			    0xff0);
		if (bgmac_read(bgmac, ring->mmio_base + BGMAC_DMA_RX_RINGLO))
			return true;
		break;
	}
	return false;
}

static void bgmac_dma_tx_ring_free(struct bgmac *bgmac,
				   struct bgmac_dma_ring *ring)
{
	struct device *dma_dev = bgmac->dma_dev;
	struct bgmac_dma_desc *dma_desc = ring->cpu_base;
	struct bgmac_slot_info *slot;
	int i;

	for (i = 0; i < BGMAC_TX_RING_SLOTS; i++) {
		int len = dma_desc[i].ctl1 & BGMAC_DESC_CTL1_LEN;

		slot = &ring->slots[i];
		dev_kfree_skb(slot->skb);

		if (!slot->dma_addr)
			continue;

		if (slot->skb)
			dma_unmap_single(dma_dev, slot->dma_addr,
					 len, DMA_TO_DEVICE);
		else
			dma_unmap_page(dma_dev, slot->dma_addr,
				       len, DMA_TO_DEVICE);
	}
}

static void bgmac_dma_rx_ring_free(struct bgmac *bgmac,
				   struct bgmac_dma_ring *ring)
{
	struct device *dma_dev = bgmac->dma_dev;
	struct bgmac_slot_info *slot;
	int i;

	for (i = 0; i < BGMAC_RX_RING_SLOTS; i++) {
		slot = &ring->slots[i];
		if (!slot->dma_addr)
			continue;

		dma_unmap_single(dma_dev, slot->dma_addr,
				 bgmac->rx_buf_size,
				 DMA_FROM_DEVICE);
		put_page(virt_to_head_page(slot->buf));
		slot->dma_addr = 0;
	}
}

static void bgmac_dma_ring_desc_free(struct bgmac *bgmac,
				     struct bgmac_dma_ring *ring,
				     int num_slots)
{
	struct device *dma_dev = bgmac->dma_dev;
	int size;

	if (!ring->cpu_base)
	    return;

	/* Free ring of descriptors */
	size = num_slots * sizeof(struct bgmac_dma_desc);
	dma_free_coherent(dma_dev, size, ring->cpu_base,
			  ring->dma_base);
}

static void bgmac_dma_cleanup(struct bgmac *bgmac)
{
	int i;

	for (i = 0; i < BGMAC_MAX_TX_RINGS; i++)
		bgmac_dma_tx_ring_free(bgmac, &bgmac->tx_ring[i]);

	for (i = 0; i < BGMAC_MAX_RX_RINGS; i++)
		bgmac_dma_rx_ring_free(bgmac, &bgmac->rx_ring[i]);
}

static void bgmac_dma_free(struct bgmac *bgmac)
{
	int i;

	for (i = 0; i < BGMAC_MAX_TX_RINGS; i++)
		bgmac_dma_ring_desc_free(bgmac, &bgmac->tx_ring[i],
					 BGMAC_TX_RING_SLOTS);

	for (i = 0; i < BGMAC_MAX_RX_RINGS; i++)
		bgmac_dma_ring_desc_free(bgmac, &bgmac->rx_ring[i],
					 BGMAC_RX_RING_SLOTS);
}

static int bgmac_dma_alloc(struct bgmac *bgmac)
{
	struct device *dma_dev = bgmac->dma_dev;
	struct bgmac_dma_ring *ring;
	static const u16 ring_base[] = { BGMAC_DMA_BASE0, BGMAC_DMA_BASE1,
					 BGMAC_DMA_BASE2, BGMAC_DMA_BASE3, };
	int size; /* ring size: different for Tx and Rx */
	int err;
	int i;

	BUILD_BUG_ON(BGMAC_MAX_TX_RINGS > ARRAY_SIZE(ring_base));
	BUILD_BUG_ON(BGMAC_MAX_RX_RINGS > ARRAY_SIZE(ring_base));

	if (!(bgmac_idm_read(bgmac, BCMA_IOST) & BCMA_IOST_DMA64)) {
		dev_err(bgmac->dev, "Core does not report 64-bit DMA\n");
		return -ENOTSUPP;
	}

	for (i = 0; i < BGMAC_MAX_TX_RINGS; i++) {
		ring = &bgmac->tx_ring[i];
		ring->mmio_base = ring_base[i];

		/* Alloc ring of descriptors */
		size = BGMAC_TX_RING_SLOTS * sizeof(struct bgmac_dma_desc);
		ring->cpu_base = dma_zalloc_coherent(dma_dev, size,
						     &ring->dma_base,
						     GFP_KERNEL);
		if (!ring->cpu_base) {
			dev_err(bgmac->dev, "Allocation of TX ring 0x%X failed\n",
				ring->mmio_base);
			goto err_dma_free;
		}

		ring->unaligned = bgmac_dma_unaligned(bgmac, ring,
						      BGMAC_DMA_RING_TX);
		if (ring->unaligned)
			ring->index_base = lower_32_bits(ring->dma_base);
		else
			ring->index_base = 0;

		/* No need to alloc TX slots yet */
	}

	for (i = 0; i < BGMAC_MAX_RX_RINGS; i++) {
		ring = &bgmac->rx_ring[i];
		ring->mmio_base = ring_base[i];

		/* Alloc ring of descriptors */
		size = BGMAC_RX_RING_SLOTS * sizeof(struct bgmac_dma_desc);
		ring->cpu_base = dma_zalloc_coherent(dma_dev, size,
						     &ring->dma_base,
						     GFP_KERNEL);
		if (!ring->cpu_base) {
			dev_err(bgmac->dev, "Allocation of RX ring 0x%X failed\n",
				ring->mmio_base);
			err = -ENOMEM;
			goto err_dma_free;
		}

		ring->unaligned = bgmac_dma_unaligned(bgmac, ring,
						      BGMAC_DMA_RING_RX);
		if (ring->unaligned)
			ring->index_base = lower_32_bits(ring->dma_base);
		else
			ring->index_base = 0;
	}

	return 0;

err_dma_free:
	bgmac_dma_free(bgmac);
	return -ENOMEM;
}

static int bgmac_dma_init(struct bgmac *bgmac)
{
	struct bgmac_dma_ring *ring;
	int i, err;

	for (i = 0; i < BGMAC_MAX_TX_RINGS; i++) {
		ring = &bgmac->tx_ring[i];

		if (!ring->unaligned)
			bgmac_dma_tx_enable(bgmac, ring);
		bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_TX_RINGLO,
			    lower_32_bits(ring->dma_base));
		bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_TX_RINGHI,
			    upper_32_bits(ring->dma_base));
		if (ring->unaligned)
			bgmac_dma_tx_enable(bgmac, ring);

		ring->start = 0;
		ring->end = 0;	/* Points the slot that should *not* be read */
	}

	for (i = 0; i < BGMAC_MAX_RX_RINGS; i++) {
		int j;

		ring = &bgmac->rx_ring[i];

		if (!ring->unaligned)
			bgmac_dma_rx_enable(bgmac, ring);
		bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_RX_RINGLO,
			    lower_32_bits(ring->dma_base));
		bgmac_write(bgmac, ring->mmio_base + BGMAC_DMA_RX_RINGHI,
			    upper_32_bits(ring->dma_base));
		if (ring->unaligned)
			bgmac_dma_rx_enable(bgmac, ring);

		ring->start = 0;
		ring->end = 0;
		for (j = 0; j < BGMAC_RX_RING_SLOTS; j++) {
			err = bgmac_dma_rx_skb_for_slot(bgmac, &ring->slots[j]);
			if (err)
				goto error;

			bgmac_dma_rx_setup_desc(bgmac, ring, j);
		}

		bgmac_dma_rx_update_index(bgmac, ring);
	}

	return 0;

error:
	bgmac_dma_cleanup(bgmac);
	return err;
}


/**************************************************
 * Chip ops
 **************************************************/

/* TODO: can we just drop @force? Can we don't reset MAC at all if there is
 * nothing to change? Try if after stabilizng driver.
 */
static void bgmac_cmdcfg_maskset(struct bgmac *bgmac, u32 mask, u32 set,
				 bool force)
{
	u32 cmdcfg = bgmac_read(bgmac, BGMAC_CMDCFG);
	u32 new_val = (cmdcfg & mask) | set;
	u32 cmdcfg_sr;

	if (bgmac->feature_flags & BGMAC_FEAT_CMDCFG_SR_REV4)
		cmdcfg_sr = BGMAC_CMDCFG_SR_REV4;
	else
		cmdcfg_sr = BGMAC_CMDCFG_SR_REV0;

	bgmac_set(bgmac, BGMAC_CMDCFG, cmdcfg_sr);
	udelay(2);

	if (new_val != cmdcfg || force)
		bgmac_write(bgmac, BGMAC_CMDCFG, new_val);

	bgmac_mask(bgmac, BGMAC_CMDCFG, ~cmdcfg_sr);
	udelay(2);
}

static void bgmac_write_mac_address(struct bgmac *bgmac, u8 *addr)
{
	u32 tmp;

	tmp = (addr[0] << 24) | (addr[1] << 16) | (addr[2] << 8) | addr[3];
	bgmac_write(bgmac, BGMAC_MACADDR_HIGH, tmp);
	tmp = (addr[4] << 8) | addr[5];
	bgmac_write(bgmac, BGMAC_MACADDR_LOW, tmp);
}

static void bgmac_set_rx_mode(struct net_device *net_dev)
{
	struct bgmac *bgmac = netdev_priv(net_dev);

	if (net_dev->flags & IFF_PROMISC)
		bgmac_cmdcfg_maskset(bgmac, ~0, BGMAC_CMDCFG_PROM, true);
	else
		bgmac_cmdcfg_maskset(bgmac, ~BGMAC_CMDCFG_PROM, 0, true);
}

#if 0 /* We don't use that regs yet */
static void bgmac_chip_stats_update(struct bgmac *bgmac)
{
	int i;

	if (!(bgmac->feature_flags & BGMAC_FEAT_NO_CLR_MIB)) {
		for (i = 0; i < BGMAC_NUM_MIB_TX_REGS; i++)
			bgmac->mib_tx_regs[i] =
				bgmac_read(bgmac,
					   BGMAC_TX_GOOD_OCTETS + (i * 4));
		for (i = 0; i < BGMAC_NUM_MIB_RX_REGS; i++)
			bgmac->mib_rx_regs[i] =
				bgmac_read(bgmac,
					   BGMAC_RX_GOOD_OCTETS + (i * 4));
	}

	/* TODO: what else? how to handle BCM4706? Specs are needed */
}
#endif

static void bgmac_clear_mib(struct bgmac *bgmac)
{
	int i;

	if (bgmac->feature_flags & BGMAC_FEAT_NO_CLR_MIB)
		return;

	bgmac_set(bgmac, BGMAC_DEV_CTL, BGMAC_DC_MROR);
	for (i = 0; i < BGMAC_NUM_MIB_TX_REGS; i++)
		bgmac_read(bgmac, BGMAC_TX_GOOD_OCTETS + (i * 4));
	for (i = 0; i < BGMAC_NUM_MIB_RX_REGS; i++)
		bgmac_read(bgmac, BGMAC_RX_GOOD_OCTETS + (i * 4));
}

/* http://bcm-v4.sipsolutions.net/mac-gbit/gmac/gmac_speed */
static void bgmac_mac_speed(struct bgmac *bgmac)
{
	u32 mask = ~(BGMAC_CMDCFG_ES_MASK | BGMAC_CMDCFG_HD);
	u32 set = 0;

	switch (bgmac->mac_speed) {
	case SPEED_10:
		set |= BGMAC_CMDCFG_ES_10;
		break;
	case SPEED_100:
		set |= BGMAC_CMDCFG_ES_100;
		break;
	case SPEED_1000:
		set |= BGMAC_CMDCFG_ES_1000;
		break;
	case SPEED_2500:
		set |= BGMAC_CMDCFG_ES_2500;
		break;
	default:
		dev_err(bgmac->dev, "Unsupported speed: %d\n",
			bgmac->mac_speed);
	}

	if (bgmac->mac_duplex == DUPLEX_HALF)
		set |= BGMAC_CMDCFG_HD;

	bgmac_cmdcfg_maskset(bgmac, mask, set, true);
}

static void bgmac_miiconfig(struct bgmac *bgmac)
{
	if (bgmac->feature_flags & BGMAC_FEAT_FORCE_SPEED_2500) {
		bgmac_idm_write(bgmac, BCMA_IOCTL,
				bgmac_idm_read(bgmac, BCMA_IOCTL) | 0x40 |
				BGMAC_BCMA_IOCTL_SW_CLKEN);
		bgmac->mac_speed = SPEED_2500;
		bgmac->mac_duplex = DUPLEX_FULL;
		bgmac_mac_speed(bgmac);
	} else {
		u8 imode;

		imode = (bgmac_read(bgmac, BGMAC_DEV_STATUS) &
			BGMAC_DS_MM_MASK) >> BGMAC_DS_MM_SHIFT;
		if (imode == 0 || imode == 1) {
			bgmac->mac_speed = SPEED_100;
			bgmac->mac_duplex = DUPLEX_FULL;
			bgmac_mac_speed(bgmac);
		}
	}
}

/* http://bcm-v4.sipsolutions.net/mac-gbit/gmac/chipreset */
static void bgmac_chip_reset(struct bgmac *bgmac)
{
	u32 cmdcfg_sr;
	u32 iost;
	int i;

	if (bgmac_clk_enabled(bgmac)) {
		if (!bgmac->stats_grabbed) {
			/* bgmac_chip_stats_update(bgmac); */
			bgmac->stats_grabbed = true;
		}

		for (i = 0; i < BGMAC_MAX_TX_RINGS; i++)
			bgmac_dma_tx_reset(bgmac, &bgmac->tx_ring[i]);

		bgmac_cmdcfg_maskset(bgmac, ~0, BGMAC_CMDCFG_ML, false);
		udelay(1);

		for (i = 0; i < BGMAC_MAX_RX_RINGS; i++)
			bgmac_dma_rx_reset(bgmac, &bgmac->rx_ring[i]);

		/* TODO: Clear software multicast filter list */
	}

	iost = bgmac_idm_read(bgmac, BCMA_IOST);
	if (bgmac->feature_flags & BGMAC_FEAT_IOST_ATTACHED)
		iost &= ~BGMAC_BCMA_IOST_ATTACHED;

	/* 3GMAC: for BCM4707 & BCM47094, only do core reset at bgmac_probe() */
	if (!(bgmac->feature_flags & BGMAC_FEAT_NO_RESET)) {
		u32 flags = 0;
		if (iost & BGMAC_BCMA_IOST_ATTACHED) {
			flags = BGMAC_BCMA_IOCTL_SW_CLKEN;
			if (!bgmac->has_robosw)
				flags |= BGMAC_BCMA_IOCTL_SW_RESET;
		}
		bgmac_clk_enable(bgmac, flags);
	}

	/* Request Misc PLL for corerev > 2 */
	if (bgmac->feature_flags & BGMAC_FEAT_MISC_PLL_REQ) {
		bgmac_set(bgmac, BCMA_CLKCTLST,
			  BGMAC_BCMA_CLKCTLST_MISC_PLL_REQ);
		bgmac_wait_value(bgmac, BCMA_CLKCTLST,
				 BGMAC_BCMA_CLKCTLST_MISC_PLL_ST,
				 BGMAC_BCMA_CLKCTLST_MISC_PLL_ST,
				 1000);
	}

	if (bgmac->feature_flags & BGMAC_FEAT_SW_TYPE_PHY) {
		u8 et_swtype = 0;
		u8 sw_type = BGMAC_CHIPCTL_1_SW_TYPE_EPHY |
			     BGMAC_CHIPCTL_1_IF_TYPE_MII;
		char buf[4];

		if (bcm47xx_nvram_getenv("et_swtype", buf, sizeof(buf)) > 0) {
			if (kstrtou8(buf, 0, &et_swtype))
				dev_err(bgmac->dev, "Failed to parse et_swtype (%s)\n",
					buf);
			et_swtype &= 0x0f;
			et_swtype <<= 4;
			sw_type = et_swtype;
		} else if (bgmac->feature_flags & BGMAC_FEAT_SW_TYPE_EPHYRMII) {
			sw_type = BGMAC_CHIPCTL_1_SW_TYPE_EPHYRMII;
		} else if (bgmac->feature_flags & BGMAC_FEAT_SW_TYPE_RGMII) {
			sw_type = BGMAC_CHIPCTL_1_IF_TYPE_RGMII |
				  BGMAC_CHIPCTL_1_SW_TYPE_RGMII;
		}
		bgmac_cco_ctl_maskset(bgmac, 1, ~(BGMAC_CHIPCTL_1_IF_TYPE_MASK |
						  BGMAC_CHIPCTL_1_SW_TYPE_MASK),
				      sw_type);
	}

	if (iost & BGMAC_BCMA_IOST_ATTACHED && !bgmac->has_robosw)
		bgmac_idm_write(bgmac, BCMA_IOCTL,
				bgmac_idm_read(bgmac, BCMA_IOCTL) &
				~BGMAC_BCMA_IOCTL_SW_RESET);

	/* http://bcm-v4.sipsolutions.net/mac-gbit/gmac/gmac_reset
	 * Specs don't say about using BGMAC_CMDCFG_SR, but in this routine
	 * BGMAC_CMDCFG is read _after_ putting chip in a reset. So it has to
	 * be keps until taking MAC out of the reset.
	 */
	if (bgmac->feature_flags & BGMAC_FEAT_CMDCFG_SR_REV4)
		cmdcfg_sr = BGMAC_CMDCFG_SR_REV4;
	else
		cmdcfg_sr = BGMAC_CMDCFG_SR_REV0;

	bgmac_cmdcfg_maskset(bgmac,
			     ~(BGMAC_CMDCFG_TE |
			       BGMAC_CMDCFG_RE |
			       BGMAC_CMDCFG_RPI |
			       BGMAC_CMDCFG_TAI |
			       BGMAC_CMDCFG_HD |
			       BGMAC_CMDCFG_ML |
			       BGMAC_CMDCFG_CFE |
			       BGMAC_CMDCFG_RL |
			       BGMAC_CMDCFG_RED |
			       BGMAC_CMDCFG_PE |
			       BGMAC_CMDCFG_TPI |
			       BGMAC_CMDCFG_PAD_EN |
			       BGMAC_CMDCFG_PF),
			     BGMAC_CMDCFG_PROM |
			     BGMAC_CMDCFG_NLC |
			     BGMAC_CMDCFG_CFE |
			     cmdcfg_sr,
			     false);
	bgmac->mac_speed = SPEED_UNKNOWN;
	bgmac->mac_duplex = DUPLEX_UNKNOWN;

	bgmac_clear_mib(bgmac);
	if (bgmac->feature_flags & BGMAC_FEAT_CMN_PHY_CTL)
		bgmac_cmn_maskset32(bgmac, BCMA_GMAC_CMN_PHY_CTL, ~0,
				    BCMA_GMAC_CMN_PC_MTE);
	else
		bgmac_set(bgmac, BGMAC_PHY_CNTL, BGMAC_PC_MTE);
	bgmac_miiconfig(bgmac);
	if (bgmac->mii_bus)
		bgmac->mii_bus->reset(bgmac->mii_bus);

	netdev_reset_queue(bgmac->net_dev);
}

static void bgmac_chip_intrs_on(struct bgmac *bgmac)
{
	bgmac_write(bgmac, BGMAC_INT_MASK, bgmac->int_mask);
}

static void bgmac_chip_intrs_off(struct bgmac *bgmac)
{
	bgmac_write(bgmac, BGMAC_INT_MASK, 0);
	bgmac_read(bgmac, BGMAC_INT_MASK);
}

/* http://bcm-v4.sipsolutions.net/mac-gbit/gmac/gmac_enable */
static void bgmac_enable(struct bgmac *bgmac)
{
	u32 cmdcfg_sr;
	u32 cmdcfg;
	u32 mode;

	if (bgmac->feature_flags & BGMAC_FEAT_CMDCFG_SR_REV4)
		cmdcfg_sr = BGMAC_CMDCFG_SR_REV4;
	else
		cmdcfg_sr = BGMAC_CMDCFG_SR_REV0;

	cmdcfg = bgmac_read(bgmac, BGMAC_CMDCFG);
	bgmac_cmdcfg_maskset(bgmac, ~(BGMAC_CMDCFG_TE | BGMAC_CMDCFG_RE),
			     cmdcfg_sr, true);
	udelay(2);
	cmdcfg |= BGMAC_CMDCFG_TE | BGMAC_CMDCFG_RE;
	bgmac_write(bgmac, BGMAC_CMDCFG, cmdcfg);

	mode = (bgmac_read(bgmac, BGMAC_DEV_STATUS) & BGMAC_DS_MM_MASK) >>
		BGMAC_DS_MM_SHIFT;
	if (bgmac->feature_flags & BGMAC_FEAT_CLKCTLST || mode != 0)
		bgmac_set(bgmac, BCMA_CLKCTLST, BCMA_CLKCTLST_FORCEHT);
	if (bgmac->feature_flags & BGMAC_FEAT_CLKCTLST && mode == 2)
		bgmac_cco_ctl_maskset(bgmac, 1, ~0,
				      BGMAC_CHIPCTL_1_RXC_DLL_BYPASS);

	if (bgmac->feature_flags & (BGMAC_FEAT_FLW_CTRL1 |
				    BGMAC_FEAT_FLW_CTRL2)) {
		u32 fl_ctl;

		if (bgmac->feature_flags & BGMAC_FEAT_FLW_CTRL1)
			fl_ctl = 0x2300e1;
		else
			fl_ctl = 0x03cb04cb;

		bgmac_write(bgmac, BGMAC_FLOW_CTL_THRESH, fl_ctl);
		bgmac_write(bgmac, BGMAC_PAUSE_CTL, 0x27fff);
	}

	if (bgmac->feature_flags & BGMAC_FEAT_SET_RXQ_CLK) {
		u32 rxq_ctl;
		u16 bp_clk;
		u8 mdp;

		rxq_ctl = bgmac_read(bgmac, BGMAC_RXQ_CTL);
		rxq_ctl &= ~BGMAC_RXQ_CTL_MDP_MASK;
		bp_clk = bgmac_get_bus_clock(bgmac) / 1000000;
		mdp = (bp_clk * 128 / 1000) - 3;
		rxq_ctl |= (mdp << BGMAC_RXQ_CTL_MDP_SHIFT);
		bgmac_write(bgmac, BGMAC_RXQ_CTL, rxq_ctl);
	}
}

/* http://bcm-v4.sipsolutions.net/mac-gbit/gmac/chipinit */
static void bgmac_chip_init(struct bgmac *bgmac)
{
	/* Clear any erroneously pending interrupts */
	bgmac_write(bgmac, BGMAC_INT_STATUS, ~0);

	/* 1 interrupt per received frame */
	bgmac_write(bgmac, BGMAC_INT_RECV_LAZY, 1 << BGMAC_IRL_FC_SHIFT);

	/* Enable 802.3x tx flow control (honor received PAUSE frames) */
	bgmac_cmdcfg_maskset(bgmac, ~BGMAC_CMDCFG_RPI, 0, true);

	bgmac_set_rx_mode(bgmac->net_dev);

	bgmac_write_mac_address(bgmac, bgmac->net_dev->dev_addr);

	if (bgmac->loopback)
		bgmac_cmdcfg_maskset(bgmac, ~0, BGMAC_CMDCFG_ML, false);
	else
		bgmac_cmdcfg_maskset(bgmac, ~BGMAC_CMDCFG_ML, 0, false);

	if (bgmac->feature_flags & BGMAC_FEAT_CRC_STRIP)
		bgmac_cmdcfg_maskset(bgmac, ~BGMAC_CMDCFG_CF, 0, true);

	if (bgmac->feature_flags & BGMAC_FEAT_OOB_FC) {
		bgmac_cmdcfg_maskset(bgmac, ~0, BGMAC_CMDCFG_AT, true);
		bgmac_cmdcfg_maskset(bgmac, ~0, BGMAC_CMDCFG_TPI, true);
		bgmac_cmdcfg_maskset(bgmac, ~0, BGMAC_CMDCFG_RPI, true);
		bgmac_write(bgmac, BGMAC_TXIPG, 8);
	}

	bgmac_write(bgmac, BGMAC_RXMAX_LENGTH, 32 + bgmac->net_dev->mtu);

	bgmac_chip_intrs_on(bgmac);

	bgmac_enable(bgmac);
}

static irqreturn_t bgmac_interrupt(int irq, void *dev_id)
{
	struct bgmac *bgmac = netdev_priv(dev_id);

	u32 int_status = bgmac_read(bgmac, BGMAC_INT_STATUS);
	int_status &= bgmac->int_mask;

	if (!int_status)
		return IRQ_NONE;

	int_status &= ~(BGMAC_IS_TX0 | BGMAC_IS_RX);
	if (int_status)
		dev_err(bgmac->dev, "Unknown IRQs: 0x%08X\n", int_status);

	/* Disable new interrupts until handling existing ones */
	bgmac_chip_intrs_off(bgmac);

	napi_schedule(&bgmac->napi);

	return IRQ_HANDLED;
}

static int bgmac_poll(struct napi_struct *napi, int weight)
{
	struct bgmac *bgmac = container_of(napi, struct bgmac, napi);
	int handled = 0;

	/* Ack */
	bgmac_write(bgmac, BGMAC_INT_STATUS, ~0);

	bgmac_dma_tx_free(bgmac, &bgmac->tx_ring[0]);
	handled += bgmac_dma_rx_read(bgmac, &bgmac->rx_ring[0], weight);

	/* Poll again if more events arrived in the meantime */
	if (bgmac_read(bgmac, BGMAC_INT_STATUS) & (BGMAC_IS_TX0 | BGMAC_IS_RX))
		return weight;

	if (handled < weight) {
		napi_complete(napi);
		bgmac_chip_intrs_on(bgmac);
	}

	return handled;
}

/**************************************************
 * net_device_ops
 **************************************************/

static int bgmac_open(struct net_device *net_dev)
{
	struct bgmac *bgmac = netdev_priv(net_dev);
	int err = 0;

	bgmac_chip_reset(bgmac);

	err = bgmac_dma_init(bgmac);
	if (err)
		return err;

	/* Specs say about reclaiming rings here, but we do that in DMA init */
	bgmac_chip_init(bgmac);

	err = request_irq(bgmac->irq, bgmac_interrupt, IRQF_SHARED,
			  KBUILD_MODNAME, net_dev);
	if (err < 0) {
		dev_err(bgmac->dev, "IRQ request error: %d!\n", err);
		bgmac_dma_cleanup(bgmac);
		return err;
	}
	napi_enable(&bgmac->napi);

	phy_start(net_dev->phydev);

	netif_start_queue(net_dev);

	return 0;
}

static int bgmac_stop(struct net_device *net_dev)
{
	struct bgmac *bgmac = netdev_priv(net_dev);

	netif_carrier_off(net_dev);

#ifdef CONFIG_BGMAC_OFFLOAD
	if (!bgmac->rings_offloaded)
		phy_stop(net_dev->phydev);
#else
	phy_stop(net_dev->phydev);
#endif /* CONFIG_BGMAC_OFFLOAD */

	napi_disable(&bgmac->napi);
	bgmac_chip_intrs_off(bgmac);
	free_irq(bgmac->irq, net_dev);

#ifdef CONFIG_BGMAC_OFFLOAD
	if (!bgmac->rings_offloaded)
		bgmac_chip_reset(bgmac);
#else
	bgmac_chip_reset(bgmac);
#endif /* CONFIG_BGMAC_OFFLOAD */
	bgmac_dma_cleanup(bgmac);

	return 0;
}

static netdev_tx_t bgmac_start_xmit(struct sk_buff *skb,
				    struct net_device *net_dev)
{
	struct bgmac *bgmac = netdev_priv(net_dev);
	struct bgmac_dma_ring *ring;

#ifdef CONFIG_BGMAC_OFFLOAD
	if (bgmac->rings_offloaded)
		return NETDEV_TX_OK;
#endif /* CONFIG_BGMAC_OFFLOAD */

	/* No QOS support yet */
	ring = &bgmac->tx_ring[0];
	return bgmac_dma_tx_add(bgmac, ring, skb);
}

/* bgmac_enet_tx_timeout() - Transmit timeout routine
 * @net_dev - network device pointer
 */

static void bgmac_enet_tx_timeout(struct net_device *net_dev)
{
	struct bgmac *bgmac = netdev_priv(net_dev);

	/* Reset outside of interrupt context, to avoid request_irq
	*  in interrupt context
	*/
	schedule_work(&bgmac->tx_timeout_task);
}

static void bgmac_tx_timeout_task(struct work_struct *work)
{
	struct bgmac *bgmac = container_of(work, struct bgmac, tx_timeout_task);
	struct net_device *net_dev = bgmac->net_dev;

#ifdef CONFIG_BGMAC_OFFLOAD
	if (bgmac->rings_offloaded)
		return;
#endif /* CONFIG_BGMAC_OFFLOAD */

	rtnl_lock();
	if (netif_running(net_dev)) {
		net_dev->stats.tx_errors++;
		bgmac_stop(net_dev);
		bgmac_open(net_dev);
	}
	rtnl_unlock();
}

static int bgmac_set_mac_address(struct net_device *net_dev, void *addr)
{
	struct bgmac *bgmac = netdev_priv(net_dev);
	int ret;
	struct sockaddr *sa = addr;

	ret = eth_prepare_mac_addr_change(net_dev, addr);
	if (ret < 0)
		return ret;

	ether_addr_copy(bgmac->mac_addr, sa->sa_data);
	bgmac_write_mac_address(bgmac, bgmac->mac_addr);

	eth_commit_mac_addr_change(net_dev, addr);
	return 0;
}

#ifdef CONFIG_BGMAC_OFFLOAD
static int bgmac_get_ring_offload_ioctl(struct net_device *dev,
					void __user *uaddr)
{
	/* To be enhanced when ODP driver supports requesting ring
	 * information.
	 */
	return 0;
}

static int bgmac_set_ring_offload_ioctl(struct net_device *dev,
					void __user *uaddr)
{
	struct bcm_ring_offload_set bro;
	struct bgmac *bgmac = netdev_priv(dev);

	if (copy_from_user(&bro, uaddr,
			   sizeof(struct bcm_ring_offload_set))) {
		netdev_err(bgmac->net_dev,
			   "Failed to copy BRO ioctl data from user\n");
		return -EFAULT;
	}

	if (bro.version != BCM_RING_OFFLOAD_VERSION) {
		netdev_err(bgmac->net_dev,
			   "Invalid BRO ioctl data version: expected 0x%x got %x\n",
			   BCM_RING_OFFLOAD_VERSION, bro.version);
		return -EINVAL;
	}

	switch (bro.set_op) {
	case BCM_RING_OFFLOAD_SET_OP_STOP:
		netdev_dbg(bgmac->net_dev,
			   "%s() bro.set_op %s\n",
			   __func__, "BCM_RING_OFFLOAD_SET_OP_STOP");
		if (!bgmac->rings_offloaded) {
			bgmac->rings_offloaded = true;
			bgmac_stop(dev);
		}
		break;

	case BCM_RING_OFFLOAD_SET_OP_START:
		netdev_dbg(bgmac->net_dev,
			   "%s() bro.set_op %s\n",
			   __func__, "BCM_RING_OFFLOAD_SET_OP_START");
		if (bgmac->rings_offloaded) {
			bgmac_open(dev);
			bgmac->rings_offloaded = false;
		}
		break;

	default:
		netdev_err(bgmac->net_dev, "Invalid BRO ioctl command: 0x%x\n",
			   bro.set_op);
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_BGMAC_OFFLOAD */

static int bgmac_ioctl(struct net_device *net_dev, struct ifreq *ifr, int cmd)
{
#ifdef CONFIG_BGMAC_OFFLOAD
	int rc;
#endif /* CONFIG_BGMAC_OFFLOAD */

	if (!netif_running(net_dev))
		return -EINVAL;

#ifdef CONFIG_BGMAC_OFFLOAD
	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		rc = phy_mii_ioctl(net_dev->phydev, ifr, cmd);
		break;
	case GIORINGOFFLOAD:
		if (!netif_running(net_dev))
			return -EAGAIN;
		rc = bgmac_get_ring_offload_ioctl(net_dev,
						  ifr->ifr_ifru.ifru_data);
		break;
	case SIORINGOFFLOAD:
		if (!netif_running(net_dev))
			return -EAGAIN;
		rc = bgmac_set_ring_offload_ioctl(net_dev,
						  ifr->ifr_ifru.ifru_data);
		break;
	default:
		rc = -EOPNOTSUPP;
	}

	return rc;
#else
	return phy_mii_ioctl(net_dev->phydev, ifr, cmd);
#endif /* CONFIG_BGMAC_OFFLOAD */
}

/* bgmac_enet_get_stats() - Get device status
 * @net_dev: net device pointer
 * @stats: rtnl_link_stats64 pointer
 *
 * Returns: network statistics
 */

static struct
rtnl_link_stats64 *bgmac_enet_get_stats(struct net_device *net_dev,
					struct rtnl_link_stats64 *stats)
{
	struct bgmac *bgmac = netdev_priv(net_dev);
	unsigned int start;

	spin_lock_bh(&bgmac->stats_lock);
	netdev_stats_to_stats64(stats, &bgmac->net_dev->stats);
	spin_unlock_bh(&bgmac->stats_lock);

	rcu_read_lock();
	do {
		start = u64_stats_fetch_begin_irq(&bgmac->rx_stats.syncp);
		stats->rx_packets = bgmac->rx_stats.packets;
		stats->rx_bytes = bgmac->rx_stats.bytes;
		stats->rx_errors =  bgmac->rx_stats.errors;
		stats->rx_length_errors = bgmac->rx_stats.length_errors;
	} while (u64_stats_fetch_retry_irq(&bgmac->rx_stats.syncp, start));

	do {
		start = u64_stats_fetch_begin_irq(&bgmac->tx_stats.syncp);
		stats->tx_packets = bgmac->tx_stats.packets;
		stats->tx_bytes = bgmac->tx_stats.bytes;
	} while (u64_stats_fetch_retry_irq(&bgmac->tx_stats.syncp, start));
	rcu_read_unlock();

	return stats;
}

static int bgmac_change_mtu(struct net_device *net_dev, int new_mtu)
{
	struct bgmac *bgmac = netdev_priv(net_dev);
	u32 max_mtu;

	if (bgmac->feature_flags & BGMAC_JUMBO_FRAME_SUPPORT)
		max_mtu = ETHER_MAX_LEN_JUMBO;
	else
		max_mtu = ETHER_MAX_LEN;

	if (new_mtu > max_mtu) {
		pr_err("MTU cannot be greater than : %d\n", max_mtu);
		return -1;
	}

	bgmac_write(bgmac, BGMAC_RXMAX_LENGTH, new_mtu + 32);

	net_dev->mtu = new_mtu;

	return 0;
}

static const struct net_device_ops bgmac_netdev_ops = {
	.ndo_open		= bgmac_open,
	.ndo_stop		= bgmac_stop,
	.ndo_start_xmit		= bgmac_start_xmit,
	.ndo_set_rx_mode	= bgmac_set_rx_mode,
	.ndo_set_mac_address	= bgmac_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl           = bgmac_ioctl,
	.ndo_tx_timeout         = bgmac_enet_tx_timeout,
	.ndo_get_stats64	= bgmac_enet_get_stats,
	.ndo_change_mtu		= bgmac_change_mtu,
};

/**************************************************
 * ethtool_ops
 **************************************************/

struct bgmac_stat {
	u8 size;
	u32 offset;
	const char *name;
};

static struct bgmac_stat bgmac_get_strings_stats[] = {
	{ 8, BGMAC_TX_GOOD_OCTETS, "tx_good_octets" },
	{ 4, BGMAC_TX_GOOD_PKTS, "tx_good" },
	{ 8, BGMAC_TX_OCTETS, "tx_octets" },
	{ 4, BGMAC_TX_PKTS, "tx_pkts" },
	{ 4, BGMAC_TX_BROADCAST_PKTS, "tx_broadcast" },
	{ 4, BGMAC_TX_MULTICAST_PKTS, "tx_multicast" },
	{ 4, BGMAC_TX_LEN_64, "tx_64" },
	{ 4, BGMAC_TX_LEN_65_TO_127, "tx_65_127" },
	{ 4, BGMAC_TX_LEN_128_TO_255, "tx_128_255" },
	{ 4, BGMAC_TX_LEN_256_TO_511, "tx_256_511" },
	{ 4, BGMAC_TX_LEN_512_TO_1023, "tx_512_1023" },
	{ 4, BGMAC_TX_LEN_1024_TO_1522, "tx_1024_1522" },
	{ 4, BGMAC_TX_LEN_1523_TO_2047, "tx_1523_2047" },
	{ 4, BGMAC_TX_LEN_2048_TO_4095, "tx_2048_4095" },
	{ 4, BGMAC_TX_LEN_4096_TO_8191, "tx_4096_8191" },
	{ 4, BGMAC_TX_LEN_8192_TO_MAX, "tx_8192_max" },
	{ 4, BGMAC_TX_JABBER_PKTS, "tx_jabber" },
	{ 4, BGMAC_TX_OVERSIZE_PKTS, "tx_oversize" },
	{ 4, BGMAC_TX_FRAGMENT_PKTS, "tx_fragment" },
	{ 4, BGMAC_TX_UNDERRUNS, "tx_underruns" },
	{ 4, BGMAC_TX_TOTAL_COLS, "tx_total_cols" },
	{ 4, BGMAC_TX_SINGLE_COLS, "tx_single_cols" },
	{ 4, BGMAC_TX_MULTIPLE_COLS, "tx_multiple_cols" },
	{ 4, BGMAC_TX_EXCESSIVE_COLS, "tx_excessive_cols" },
	{ 4, BGMAC_TX_LATE_COLS, "tx_late_cols" },
	{ 4, BGMAC_TX_DEFERED, "tx_defered" },
	{ 4, BGMAC_TX_CARRIER_LOST, "tx_carrier_lost" },
	{ 4, BGMAC_TX_PAUSE_PKTS, "tx_pause" },
	{ 4, BGMAC_TX_UNI_PKTS, "tx_unicast" },
	{ 4, BGMAC_TX_Q0_PKTS, "tx_q0" },
	{ 8, BGMAC_TX_Q0_OCTETS, "tx_q0_octets" },
	{ 4, BGMAC_TX_Q1_PKTS, "tx_q1" },
	{ 8, BGMAC_TX_Q1_OCTETS, "tx_q1_octets" },
	{ 4, BGMAC_TX_Q2_PKTS, "tx_q2" },
	{ 8, BGMAC_TX_Q2_OCTETS, "tx_q2_octets" },
	{ 4, BGMAC_TX_Q3_PKTS, "tx_q3" },
	{ 8, BGMAC_TX_Q3_OCTETS, "tx_q3_octets" },
	{ 8, BGMAC_RX_GOOD_OCTETS, "rx_good_octets" },
	{ 4, BGMAC_RX_GOOD_PKTS, "rx_good" },
	{ 8, BGMAC_RX_OCTETS, "rx_octets" },
	{ 4, BGMAC_RX_PKTS, "rx_pkts" },
	{ 4, BGMAC_RX_BROADCAST_PKTS, "rx_broadcast" },
	{ 4, BGMAC_RX_MULTICAST_PKTS, "rx_multicast" },
	{ 4, BGMAC_RX_LEN_64, "rx_64" },
	{ 4, BGMAC_RX_LEN_65_TO_127, "rx_65_127" },
	{ 4, BGMAC_RX_LEN_128_TO_255, "rx_128_255" },
	{ 4, BGMAC_RX_LEN_256_TO_511, "rx_256_511" },
	{ 4, BGMAC_RX_LEN_512_TO_1023, "rx_512_1023" },
	{ 4, BGMAC_RX_LEN_1024_TO_1522, "rx_1024_1522" },
	{ 4, BGMAC_RX_LEN_1523_TO_2047, "rx_1523_2047" },
	{ 4, BGMAC_RX_LEN_2048_TO_4095, "rx_2048_4095" },
	{ 4, BGMAC_RX_LEN_4096_TO_8191, "rx_4096_8191" },
	{ 4, BGMAC_RX_LEN_8192_TO_MAX, "rx_8192_max" },
	{ 4, BGMAC_RX_JABBER_PKTS, "rx_jabber" },
	{ 4, BGMAC_RX_OVERSIZE_PKTS, "rx_oversize" },
	{ 4, BGMAC_RX_FRAGMENT_PKTS, "rx_fragment" },
	{ 4, BGMAC_RX_MISSED_PKTS, "rx_missed" },
	{ 4, BGMAC_RX_CRC_ALIGN_ERRS, "rx_crc_align" },
	{ 4, BGMAC_RX_UNDERSIZE, "rx_undersize" },
	{ 4, BGMAC_RX_CRC_ERRS, "rx_crc" },
	{ 4, BGMAC_RX_ALIGN_ERRS, "rx_align" },
	{ 4, BGMAC_RX_SYMBOL_ERRS, "rx_symbol" },
	{ 4, BGMAC_RX_PAUSE_PKTS, "rx_pause" },
	{ 4, BGMAC_RX_NONPAUSE_PKTS, "rx_nonpause" },
	{ 4, BGMAC_RX_SACHANGES, "rx_sa_changes" },
	{ 4, BGMAC_RX_UNI_PKTS, "rx_unicast" },
};

#define BGMAC_STATS_LEN	ARRAY_SIZE(bgmac_get_strings_stats)

static int bgmac_get_sset_count(struct net_device *dev, int string_set)
{
	switch (string_set) {
	case ETH_SS_STATS:
		return BGMAC_STATS_LEN;
	}

	return -EOPNOTSUPP;
}

static void bgmac_get_strings(struct net_device *dev, u32 stringset,
			      u8 *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < BGMAC_STATS_LEN; i++)
		strlcpy(data + i * ETH_GSTRING_LEN,
			bgmac_get_strings_stats[i].name, ETH_GSTRING_LEN);
}

static void bgmac_get_ethtool_stats(struct net_device *dev,
				    struct ethtool_stats *ss, uint64_t *data)
{
	struct bgmac *bgmac = netdev_priv(dev);
	const struct bgmac_stat *s;
	unsigned int i;
	u64 val;

	if (!netif_running(dev))
		return;

	for (i = 0; i < BGMAC_STATS_LEN; i++) {
		s = &bgmac_get_strings_stats[i];
		val = 0;
		if (s->size == 8)
			val = (u64)bgmac_read(bgmac, s->offset + 4) << 32;
		val |= bgmac_read(bgmac, s->offset);
		data[i] = val;
	}
}

static int bgmac_get_settings(struct net_device *net_dev,
			      struct ethtool_cmd *cmd)
{
	return phy_ethtool_gset(net_dev->phydev, cmd);
}

static int bgmac_set_settings(struct net_device *net_dev,
			      struct ethtool_cmd *cmd)
{
	return phy_ethtool_sset(net_dev->phydev, cmd);
}

static void bgmac_get_drvinfo(struct net_device *net_dev,
			      struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, KBUILD_MODNAME, sizeof(info->driver));
	strlcpy(info->bus_info, "AXI", sizeof(info->bus_info));
}

static int bgmac_ethtool_nway_reset(struct net_device *net_dev)
{
	if (!net_dev->phydev)
		return -ENODEV;

	if (!netif_running(net_dev))
		return -EAGAIN;

	return phy_start_aneg(net_dev->phydev);
}

static void bgmac_ethtool_get_pauseparam(struct net_device *net_dev,
					 struct ethtool_pauseparam *epause)
{
	epause->autoneg = 0;
	epause->rx_pause = 0;
	epause->tx_pause = 0;

	if (net_dev->phydev) {
		struct bgmac *bgmac = netdev_priv(net_dev);

		epause->autoneg = net_dev->phydev->autoneg;

		if (bgmac->pause & FLOW_CTRL_RX)
			epause->rx_pause = 1;
		if (bgmac->pause & FLOW_CTRL_TX)
			epause->tx_pause = 1;
	}
}

static int bgmac_ethtool_set_pauseparam(struct net_device *net_dev,
					struct ethtool_pauseparam *epause)
{
	if (netif_running(net_dev)) {
		struct bgmac *bgmac = netdev_priv(net_dev);
		u32 mac_mode = bgmac_read(bgmac, BGMAC_MAC_MODE);
		u32 cmdcfg = bgmac_read(bgmac, BGMAC_CMDCFG);
		u32 speed[4] = {10, 100, 1000, 2500};

		if (epause->tx_pause) {
			cmdcfg &= ~BGMAC_CMDCFG_TPI;
			bgmac->pause |= FLOW_CTRL_TX;
		} else {
			cmdcfg |= BGMAC_CMDCFG_TPI;
			bgmac->pause &= ~FLOW_CTRL_TX;
		}

		if (epause->rx_pause) {
			cmdcfg &= ~BGMAC_CMDCFG_RPI;
			bgmac->pause |= FLOW_CTRL_RX;
		} else {
			cmdcfg |= BGMAC_CMDCFG_RPI;
			bgmac->pause &= ~FLOW_CTRL_RX;
		}
		if (epause->autoneg != net_dev->phydev->autoneg) {
			net_dev->phydev->autoneg = epause->autoneg;
			phy_start_aneg(net_dev->phydev);
		}
		bgmac_write(bgmac, BGMAC_CMDCFG, cmdcfg);

		mac_mode = bgmac_read(bgmac, BGMAC_MAC_MODE);
		netdev_dbg(bgmac->net_dev, "MAC Level Additional Info\n");
		netdev_dbg(bgmac->net_dev,
			   "RxPause:%s TxPause:%s Duplex:%s Speed:%dMbps Link:%s:Mode:%x:\n",
			   mac_mode & MAC_RX_PAUSE ? "enabled" : "disabled",
			   mac_mode & MAC_TX_PAUSE ? "enabled" : "disabled",
			   mac_mode & MAC_DUPLEX ? "half" : "full",
			   speed[MAC_SPEED(mac_mode)],
			   mac_mode & MAC_LINK_STAT ? "down" : "up",
			   mac_mode);
		if (epause->tx_pause != ((mac_mode & MAC_TX_PAUSE) ? 1 : 0)) {
			netdev_err(bgmac->net_dev, "Tx Pause not matching\n");
			return -EIO;
		}
		if (epause->rx_pause != ((mac_mode & MAC_RX_PAUSE) ? 1 : 0)) {
			netdev_err(bgmac->net_dev, "Rx Pause not matching\n");
			return -EIO;
		}
	}
	return 0;
}

static const struct ethtool_ops bgmac_ethtool_ops = {
	.get_strings		= bgmac_get_strings,
	.get_sset_count		= bgmac_get_sset_count,
	.get_ethtool_stats	= bgmac_get_ethtool_stats,
	.get_settings		= bgmac_get_settings,
	.set_settings		= bgmac_set_settings,
	.get_drvinfo		= bgmac_get_drvinfo,
	.nway_reset             = bgmac_ethtool_nway_reset,
	.get_link               = ethtool_op_get_link,
	.get_pauseparam         = bgmac_ethtool_get_pauseparam,
	.set_pauseparam         = bgmac_ethtool_set_pauseparam,
};

/**************************************************
 * MII
 **************************************************/

void bgmac_adjust_link(struct net_device *net_dev)
{
	struct bgmac *bgmac = netdev_priv(net_dev);
	struct phy_device *phy_dev = net_dev->phydev;
	bool update = false;
	u32 mac_mode;

	if (phy_dev->link) {
		if (phy_dev->speed != bgmac->mac_speed) {
			bgmac->mac_speed = phy_dev->speed;
			update = true;
		}

		if (phy_dev->duplex != bgmac->mac_duplex) {
			bgmac->mac_duplex = phy_dev->duplex;
			update = true;
		}
	}

	if (update) {
		bgmac_mac_speed(bgmac);
		phy_print_status(phy_dev);
	}
	mac_mode = bgmac_read(bgmac, BGMAC_MAC_MODE);
	bgmac->pause |= (mac_mode & MAC_RX_PAUSE) ? FLOW_CTRL_RX : 0;
	bgmac->pause |= (mac_mode & MAC_TX_PAUSE) ? FLOW_CTRL_TX : 0;
}
EXPORT_SYMBOL_GPL(bgmac_adjust_link);

int bgmac_phy_connect_direct(struct bgmac *bgmac)
{
	struct fixed_phy_status fphy_status = {
		.link = 1,
		.speed = SPEED_1000,
		.duplex = DUPLEX_FULL,
	};
	struct phy_device *phy_dev;
	int err;

	phy_dev = fixed_phy_register(PHY_POLL, &fphy_status, -1, NULL);
	if (!phy_dev || IS_ERR(phy_dev)) {
		dev_err(bgmac->dev, "Failed to register fixed PHY device\n");
		return -ENODEV;
	}

	err = phy_connect_direct(bgmac->net_dev, phy_dev, bgmac_adjust_link,
				 PHY_INTERFACE_MODE_MII);
	if (err) {
		dev_err(bgmac->dev, "Connecting PHY failed\n");
		return err;
	}

	return err;
}
EXPORT_SYMBOL_GPL(bgmac_phy_connect_direct);

int bgmac_enet_probe(struct bgmac *info)
{
	struct net_device *net_dev;
	struct bgmac *bgmac;
	int err;

	/* Allocation and references */
	net_dev = alloc_etherdev(sizeof(*bgmac));
	if (!net_dev)
		return -ENOMEM;

	net_dev->netdev_ops = &bgmac_netdev_ops;
	net_dev->ethtool_ops = &bgmac_ethtool_ops;
	bgmac = netdev_priv(net_dev);
	memcpy(bgmac, info, sizeof(*bgmac));

	if (bgmac->feature_flags & BGMAC_JUMBO_FRAME_SUPPORT) {
		bgmac->rx_buf_size = BGMAC_RX_JUMBO_BUF_SIZE;
		bgmac->rx_alloc_size = BGMAC_RX_ALLOC_JUMBO_SIZE;
	} else {
		bgmac->rx_buf_size = BGMAC_RX_BUF_SIZE;
		bgmac->rx_alloc_size = BGMAC_RX_ALLOC_SIZE;
	}

	err = dma_set_mask_and_coherent(bgmac->dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(bgmac->dev, "No usable DMA configuration, aborting\n");
		goto err_netdev_free;
	}

	bgmac->net_dev = net_dev;
	net_dev->irq = bgmac->irq;
	net_dev->mtu = ETHER_MAX_LEN;
	SET_NETDEV_DEV(net_dev, bgmac->dev);
	dev_set_drvdata(bgmac->dev, bgmac);

	if (!is_valid_ether_addr(bgmac->mac_addr)) {
		dev_err(bgmac->dev, "Invalid MAC addr: %pM\n",
			bgmac->mac_addr);
		eth_random_addr(bgmac->mac_addr);
		dev_warn(bgmac->dev, "Using random MAC: %pM\n",
			 bgmac->mac_addr);
	}
	ether_addr_copy(net_dev->dev_addr, bgmac->mac_addr);

	/* This (reset &) enable is not preset in specs or reference driver but
	 * Broadcom does it in arch PCI code when enabling fake PCI device.
	 */
	bgmac_clk_enable(bgmac, 0);

	bgmac_chip_reset(bgmac);

	err = bgmac_dma_alloc(bgmac);
	if (err) {
		dev_err(bgmac->dev, "Unable to alloc memory for DMA\n");
		goto err_netdev_free;
	}

	bgmac->int_mask = BGMAC_IS_ERRMASK | BGMAC_IS_RX | BGMAC_IS_TX_MASK;
	if (bcm47xx_nvram_getenv("et0_no_txint", NULL, 0) == 0)
		bgmac->int_mask &= ~BGMAC_IS_TX_MASK;
	net_dev->watchdog_timeo = TX_TIMEOUT;
	INIT_WORK(&bgmac->tx_timeout_task, bgmac_tx_timeout_task);
	spin_lock_init(&bgmac->stats_lock);
	u64_stats_init(&bgmac->tx_stats.syncp);
	u64_stats_init(&bgmac->rx_stats.syncp);

	netif_napi_add(net_dev, &bgmac->napi, bgmac_poll, BGMAC_WEIGHT);

	err = bgmac_phy_connect(bgmac);
	if (err) {
		dev_err(bgmac->dev, "Cannot connect to phy\n");
		goto err_dma_free;
	}

	net_dev->features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
	net_dev->hw_features = net_dev->features;
	net_dev->vlan_features = net_dev->features;

	err = register_netdev(bgmac->net_dev);
	if (err) {
		dev_err(bgmac->dev, "Cannot register net device\n");
		goto err_phy_disconnect;
	}

	netif_carrier_off(net_dev);

	return 0;

err_phy_disconnect:
	phy_disconnect(net_dev->phydev);
err_dma_free:
	bgmac_dma_free(bgmac);
err_netdev_free:
	free_netdev(net_dev);

	return err;
}
EXPORT_SYMBOL_GPL(bgmac_enet_probe);

void bgmac_enet_remove(struct bgmac *bgmac)
{
	unregister_netdev(bgmac->net_dev);
	phy_disconnect(bgmac->net_dev->phydev);
	netif_napi_del(&bgmac->napi);
	bgmac_dma_free(bgmac);
	free_netdev(bgmac->net_dev);
}
EXPORT_SYMBOL_GPL(bgmac_enet_remove);

int bgmac_enet_suspend(struct bgmac *bgmac)
{
	int rc = 0;

	netdev_info(bgmac->net_dev, "Suspending BGMAC driver\n");

	if (netif_running(bgmac->net_dev)) {
		netif_stop_queue(bgmac->net_dev);

		napi_disable(&bgmac->napi);

		netif_tx_lock(bgmac->net_dev);
		netif_device_detach(bgmac->net_dev);
		netif_tx_unlock(bgmac->net_dev);

		bgmac_chip_intrs_off(bgmac);
		bgmac_chip_reset(bgmac);
		bgmac_dma_cleanup(bgmac);
	}

	phy_stop(bgmac->net_dev->phydev);

	return rc;
}
EXPORT_SYMBOL_GPL(bgmac_enet_suspend);

int bgmac_enet_resume(struct bgmac *bgmac)
{
	int rc = 0;

	netdev_info(bgmac->net_dev, "Resuming BGMAC driver\n");

	phy_start(bgmac->net_dev->phydev);

	if (netif_running(bgmac->net_dev)) {
		rc = bgmac_dma_init(bgmac);
		if (rc)
			return rc;

		bgmac_chip_init(bgmac);

		napi_enable(&bgmac->napi);

		netif_tx_lock(bgmac->net_dev);
		netif_device_attach(bgmac->net_dev);
		netif_tx_unlock(bgmac->net_dev);

		netif_start_queue(bgmac->net_dev);
	}

	return rc;
}
EXPORT_SYMBOL_GPL(bgmac_enet_resume);

MODULE_AUTHOR("Rafał Miłecki");
MODULE_LICENSE("GPL");