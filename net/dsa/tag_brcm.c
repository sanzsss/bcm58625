/*
 * Broadcom tag support
 *
 * Copyright (C) 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/etherdevice.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/if_vlan.h>
#include <net/dsa.h>
#include "dsa_priv.h"

/* Tag is constructed and desconstructed using byte by byte access
 * because the tag is placed after the MAC Source Address, which does
 * not make it 4-bytes aligned, so this might cause unaligned accesses
 * on most systems where this is used.
 */

/* Ingress and egress opcodes */
#define BRCM_OPCODE_SHIFT	5
#define BRCM_OPCODE_MASK	0x7

/* Ingress fields */
/* 1st byte in the tag */
#define BRCM_IG_TC_SHIFT	2
#define BRCM_IG_TC_MASK		0x7
/* 2nd byte in the tag */
#define BRCM_IG_TE_MASK		0x3
#define BRCM_IG_TS_SHIFT	7
/* 3rd byte in the tag */
#define BRCM_IG_DSTMAP2_MASK	1
#define BRCM_IG_DSTMAP1_MASK	0xff

/* Egress fields for opcode 0 */
/* 2nd byte in the tag */
#define BRCM_EG_CID_MASK	0xff

/* 3rd byte in the tag */
#define BRCM_EG_RC_MASK		0xff
#define BRCM_EG_RC_RSVD		(3 << 6)
#define BRCM_EG_RC_EXCEPTION	BIT(5)
#define BRCM_EG_RC_PROT_SNOOP	BIT(4)
#define BRCM_EG_RC_PROT_TERM	BIT(3)
#define BRCM_EG_RC_SWITCH	BIT(2)
#define BRCM_EG_RC_MAC_LEARN	BIT(1)
#define BRCM_EG_RC_MIRROR	BIT(0)
#define BRCM_EG_TC_SHIFT	5
#define BRCM_EG_TC_MASK		0x7
#define BRCM_EG_PID_MASK	0x1f

#ifndef CONFIG_NET_DSA_TAG_BRCM_PREPEND

static struct sk_buff *brcm_tag_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dsa_slave_priv *p = netdev_priv(dev);
	u8 *brcm_tag;
	u16 etype = 0;
	u8 taglen = 0;

	if (skb_cow_head(skb, DSA_TAG_BRCM_LEN) < 0)
		goto out_free;

	/* Grab ethertype before inserting tag */
	if (skb->len >= sizeof(struct ethhdr))
		etype = ntohs(((struct ethhdr *)(skb->data))->h_proto);

	skb_push(skb, DSA_TAG_BRCM_LEN);

	memmove(skb->data, skb->data + DSA_TAG_BRCM_LEN, 2 * ETH_ALEN);

	/* Build the tag after the MAC Source Address */
	brcm_tag = skb->data + 2 * ETH_ALEN;

	/* Set the ingress opcode, traffic class, tag enforcment is
	 * deprecated
	 */
	brcm_tag[0] = (1 << BRCM_OPCODE_SHIFT) |
			((skb->priority << BRCM_IG_TC_SHIFT) & BRCM_IG_TC_MASK);
	brcm_tag[1] = 0;
	brcm_tag[2] = 0;
	if (p->port == 8)
		brcm_tag[2] = BRCM_IG_DSTMAP2_MASK;
	brcm_tag[3] = (1 << p->port) & BRCM_IG_DSTMAP1_MASK;

	/* Pad packet to min length + DSA_TAG_BRCM_LEN, so that when brcm tag is
	 * stripped later, pkt will still be >= min length.
	 */
	if (etype == ETH_P_8021Q)
		taglen = VLAN_HLEN;	/* Account for vlan tag */

	if (skb_put_padto(skb, ETH_ZLEN + taglen + DSA_TAG_BRCM_LEN))
		goto out;

	return skb;

out_free:
	kfree_skb(skb);
out:
	return NULL;
}

static int brcm_tag_rcv(struct sk_buff *skb, struct net_device *dev,
			struct packet_type *pt, struct net_device *orig_dev)
{
	struct dsa_switch_tree *dst = dev->dsa_ptr;
	struct dsa_switch *ds;
	int source_port;
	u8 *brcm_tag;

	if (unlikely(!dst))
		goto out_drop;

	ds = dst->ds[0];

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

	if (unlikely(!pskb_may_pull(skb, DSA_TAG_BRCM_LEN)))
		goto out_drop;

	/* skb->data points to the EtherType, the tag is right before it */
	brcm_tag = skb->data - 2;

	/* The opcode should never be different than 0b000 */
	if (unlikely((brcm_tag[0] >> BRCM_OPCODE_SHIFT) & BRCM_OPCODE_MASK))
		goto out_drop;

	/* We should never see a reserved reason code without knowing how to
	 * handle it
	 */
	WARN_ON(brcm_tag[2] & BRCM_EG_RC_RSVD);

	/* Locate which port this is coming from */
	source_port = brcm_tag[3] & BRCM_EG_PID_MASK;

	/* Validate port against switch setup, either the port is totally */
	if (source_port >= DSA_MAX_PORTS || !ds->ports[source_port].netdev)
		goto out_drop;

	/* Remove Broadcom tag and update checksum */
	skb_pull_rcsum(skb, DSA_TAG_BRCM_LEN);

	/* Move the Ethernet DA and SA */
	memmove(skb->data - ETH_HLEN,
		skb->data - ETH_HLEN - DSA_TAG_BRCM_LEN,
		2 * ETH_ALEN);

	skb_push(skb, ETH_HLEN);
	skb->pkt_type = PACKET_HOST;
	skb->dev = ds->ports[source_port].netdev;
	skb->protocol = eth_type_trans(skb, skb->dev);

	skb->dev->stats.rx_packets++;
	skb->dev->stats.rx_bytes += skb->len;

	netif_receive_skb(skb);

	return 0;

out_drop:
	kfree_skb(skb);
out:
	return 0;
}

const struct dsa_device_ops brcm_netdev_ops = {
	.xmit	= brcm_tag_xmit,
	.rcv	= brcm_tag_rcv,
};

#else /* CONFIG_NET_DSA_TAG_BRCM_PREPEND */

#define BRCM_EG_TAG_OPCODE_TYPE_0	0
#define BRCM_EG_TAG_OPCODE_TYPE_2	2

struct in_brcmtag1 {
	u8 opcode_tc_te;	/* opcode[31:29]
				 * tc[28:26]
				 * te[25:24]
				 */
	u8 ts;			/* ts[23]
				 */
	u8 rsv;
	u8 dst_map;		/* dstmap[9:0]
				 */
};

struct eg_brcmtag0 {
	u8 opcode;		/* opcode[31:29]
				 * rsv[28:24]
				 */
	u8 class_id;		/* class[23:16]
				 */
	u8 reason_code;		/* reason code[15:8]
				 */
	u8 tc_srcport;		/* tc[7:5] srcport[4:0]
				 */
};

struct eg_brcmtag2 {
	u8 opcode_abf_flowhi;	/* opcode[31:29]
				 * all buckets full[28]
				 * napt flow hi[27:24]
				 */
	u8 flowlo;		/* napt flow lo[23:16]
				 */
	u8 hdr_chk_result;	/* check results[15:8]
				 */
	u8 tc_srcport;		/* tc[7:5] srcport[4:0]
				 */
};

/* Egress fields for opcode 2 */
/* 1st byte in the tag */
#define BRCM_EG_TAG_FLOWHI_MASK		0x0f
#define BRCM_EG_TAG_FLOWHI_SHIFT	8
#define BRCM_EG_TAG_ABF_MASK		0x10

/* 3rd byte in the tag */
#define BRCM_EG_TAG_HDRCHK_INFLOW_MISS	0x1C
#define BRCM_EG_TAG_HDRCHK_OUTFLOW_MISS	0x1D

/* Separate function for prepended broadcom headers are here because
 * in the future additional broadcom header decoding will be required
 * and the code will diverge further from the normal case
 */
static struct sk_buff *brcm_tag_xmit_prepend(struct sk_buff *skb,
					     struct net_device *dev)
{
	struct dsa_slave_priv *p = netdev_priv(dev);
	struct in_brcmtag1 *tag1;

	if (skb_cow_head(skb, DSA_TAG_BRCM_LEN) < 0)
		goto out_free;

	skb_push(skb, DSA_TAG_BRCM_LEN);

	/* Build the tag before the MAC DA */
	tag1 = (struct in_brcmtag1 *)skb->data;

	/* Set the ingress opcode, traffic class, tag enforcement is
	 * deprecated
	 */
	tag1->opcode_tc_te = (1 << BRCM_OPCODE_SHIFT) |
			((skb->priority << BRCM_IG_TC_SHIFT) & BRCM_IG_TC_MASK);
	tag1->ts = 0;
	tag1->rsv = 0;
	tag1->dst_map = (1 << p->port) & BRCM_IG_DSTMAP1_MASK;

	return skb;

out_free:
	kfree_skb(skb);
	return NULL;
}

static int brcm_tag_rcv_prepend(struct sk_buff *skb, struct net_device *dev,
				struct packet_type *pt,
				struct net_device *orig_dev)
{
	struct dsa_switch_tree *dst = dev->dsa_ptr;
	struct dsa_switch *ds;
	int source_port;
	struct eg_brcmtag0 *tag0;
	u8 opcode;

	if (unlikely(!dst))
		goto out_drop;

	ds = dst->ds[0];

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (!skb)
		goto out;

	if (unlikely(!pskb_may_pull(skb, DSA_TAG_BRCM_LEN)))
		goto out_drop;

	/* skb->data points to payload, subtract SA(6)/DA(6)/Et(2) */
	tag0 = (struct eg_brcmtag0 *)(skb->data - ETH_HLEN);

	opcode = ((tag0->opcode >> BRCM_OPCODE_SHIFT) & BRCM_OPCODE_MASK);
	if (opcode != 0 && opcode != 2)
		goto out_drop;

	/* Locate which port this is coming from */
	source_port = tag0->tc_srcport & BRCM_EG_PID_MASK;

	/* Validate port against switch setup */
	if (source_port >= DSA_MAX_PORTS || !ds->ports[source_port].netdev)
		goto out_drop;

#ifdef CONFIG_IPROC_ACCELERATOR
	if (likely(opcode == 2)) {
		struct eg_brcmtag2 *tag2 = (struct eg_brcmtag2 *)tag0;
		u16 flow_id = 0;
		/* If this is a flow miss */
		if (tag2->hdr_chk_result == BRCM_EG_TAG_HDRCHK_INFLOW_MISS ||
		    tag2->hdr_chk_result == BRCM_EG_TAG_HDRCHK_OUTFLOW_MISS) {
			flow_id |= (tag2->opcode_abf_flowhi &
				    BRCM_EG_TAG_FLOWHI_MASK);
			flow_id = flow_id << BRCM_EG_TAG_FLOWHI_SHIFT;
			flow_id |= (tag2->flowlo);

			skb->flow_info = 0;
			skb->flow_info |= flow_id;
			skb->flow_info |= source_port << DSA_TAG_BRCM_SRC_SHIFT;
			skb->flow_info |= flow_id;
		} else {
			skb->flow_info = DSA_TAG_BRCM_FLOW_INVALID;
		}
	}
#endif

	/* Remove Broadcom tag and update checksum
	 * (combine these 2 lines into skb_push_rcsum()
	 *  in the future as later kernels have this function)
	 */
	skb_pull_rcsum(skb, DSA_TAG_BRCM_LEN);
	skb_push(skb, ETH_HLEN);

	skb->pkt_type = PACKET_HOST;
	skb->dev = ds->ports[source_port].netdev;
	skb->protocol = eth_type_trans(skb, skb->dev);

	skb->dev->stats.rx_packets++;
	skb->dev->stats.rx_bytes += skb->len;

	netif_receive_skb(skb);

	return 0;

out_drop:
	kfree_skb(skb);
out:
	return 0;
}

const struct dsa_device_ops brcm_netdev_ops = {
	.xmit	= brcm_tag_xmit_prepend,
	.rcv	= brcm_tag_rcv_prepend,
};

#endif
