/*
 * Copyright (C) 2016 Broadcom
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
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/debugfs.h>
#include "fa.h"
#include "fa2.h"
#include "fa2_tables.h"
#include "fa2_debug.h"

static struct dentry *fa2_dbg_root;

static char fa2_dbg_table_ops_buf[256] = "";

/* common ENTRY_HEADER_STR and TABLE_FOOTER_STR
 */
#define ENTRY_HEADER_STR "-------------------------------------------"
#define TABLE_FOOTER_STR "-------------------------------------------\n"

/* used by fa2_decode table functions to get hw_entry
 */
static inline u32 fa2_dbg_field_get(u8 *data, u32 msb, u32 width)
{
	u32 value;

	for (value = 0; width; --width, --msb) {
		value <<= 1;
		value |= ((data[msb / 8] & (0x01 << (msb % 8))) ? 1 : 0);
	}
	return value;
}

static void fa2_dbg_dcd_port(struct fa2_port_entry *sw_entry,
			     u32 *hw_entry)
{
	sw_entry->external = fa2_dbg_field_get((u8 *)hw_entry,
					  FA2_PORT_EXTERNAL_MSB,
					  FA2_PORT_EXTERNAL_WIDTH);
}

static void fa2_dbg_dcd_mac(struct fa2_mac_entry *sw_entry,
			    u32 *hw_entry)
{
	sw_entry->host =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_MAC_HOST_MSB,
				  FA2_MAC_HOST_WIDTH);

	sw_entry->l4_checksum_check =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_MAC_L4_CSUM_CHECK_MSB,
				  FA2_MAC_L4_CSUM_CHECK_WIDTH);

	sw_entry->external =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_MAC_EXTERNAL_MSB,
				  FA2_MAC_EXTERNAL_WIDTH);

	sw_entry->rmac_0_15 =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_MAC_RMAC_0_15_MSB,
				  FA2_MAC_RMAC_0_15_WIDTH);

	sw_entry->rmac_16_47 =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_MAC_RMAC_16_47_MSB,
				  FA2_MAC_RMAC_16_47_WIDTH);
}

static void fa2_dbg_dcd_flow(struct fa2_flow_entry *sw_entry,
			     u32 *hw_entry)
{
	sw_entry->ipv4_key_type =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_IPV4_KEY_TYPE_MSB,
				  FA2_FLOW_IPV4_KEY_TYPE_WIDTH);
	sw_entry->valid =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_VALID_MSB,
				  FA2_FLOW_VALID_WIDTH);
	sw_entry->sip =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_SIP_MSB,
				  FA2_FLOW_SIP_WIDTH);
	sw_entry->dip =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_DIP_MSB,
				  FA2_FLOW_DIP_WIDTH);
	sw_entry->protocol =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_PROTOCOL_MSB,
				  FA2_FLOW_PROTOCOL_WIDTH);
	sw_entry->sport =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_SPORT_MSB,
				  FA2_FLOW_SPORT_WIDTH);
	sw_entry->dport =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_DPORT_MSB,
				  FA2_FLOW_DPORT_WIDTH);
	sw_entry->rev_flow_ptr =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_REV_FLOW_PTR_MSB,
				  FA2_FLOW_REV_FLOW_PTR_WIDTH);
	sw_entry->brcm_tag_opcode =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_BRCM_TAG_OPCODE_MSB,
				  FA2_FLOW_BRCM_TAG_OPCODE_WIDTH);
	sw_entry->brcm_tag_tc =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_BRCM_TAG_TC_MSB,
				  FA2_FLOW_BRCM_TAG_TC_WIDTH);
	sw_entry->brcm_tag_te =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_BRCM_TAG_TE_MSB,
				  FA2_FLOW_BRCM_TAG_TE_WIDTH);
	sw_entry->brcm_tag_ts =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_BRCM_TAG_TS_MSB,
				  FA2_FLOW_BRCM_TAG_TS_WIDTH);
	sw_entry->brcm_tag_destmap =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_BRCM_TAG_DESTMAP_MSB,
				  FA2_FLOW_BRCM_TAG_DESTMAP_WIDTH);
	sw_entry->direction =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_DIRECTION_MSB,
				  FA2_FLOW_DIRECTION_WIDTH);
	sw_entry->l4_chksum_chk =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_L4_CSUM_CHECK_MSB,
				  FA2_FLOW_L4_CSUM_CHECK_WIDTH);
	sw_entry->ppp_tunnel_en =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_PPP_TUNNEL_EN_MSB,
				  FA2_FLOW_PPP_TUNNEL_EN_WIDTH);
	sw_entry->ppp_tunnel_idx =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_PPP_TUNNEL_IDX_MSB,
				  FA2_FLOW_PPP_TUNNEL_IDX_WIDTH);
	sw_entry->mtu_idx =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_MTU_IDX_MSB,
				  FA2_FLOW_MTU_IDX_WIDTH);
	sw_entry->next_hop_idx =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_NEXT_HOP_IDX_MSB,
				  FA2_FLOW_NEXT_HOP_IDX_WIDTH);
	sw_entry->remap_sa_idx =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_REMAP_SA_IDX_MSB,
				  FA2_FLOW_REMAP_SA_IDX_WIDTH);
	sw_entry->dest_dma_chan =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_DEST_DMA_CHAN_MSB,
				  FA2_FLOW_DEST_DMA_CHAN_WIDTH);
	sw_entry->action =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_ACTION_MSB,
				  FA2_FLOW_ACTION_WIDTH);
	sw_entry->hits =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_HITS_MSB,
				  FA2_FLOW_HITS_WIDTH);
	sw_entry->tcp_fin =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_TCP_FIN_MSB,
				  FA2_FLOW_TCP_FIN_WIDTH);
	sw_entry->tcp_rst =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_TCP_RST_MSB,
				  FA2_FLOW_TCP_RST_WIDTH);
	sw_entry->tcp_ack_after_close =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_TCP_ACK_AFTER_CLOSE_MSB,
				  FA2_FLOW_TCP_ACK_AFTER_CLOSE_WIDTH);
	sw_entry->hit_after_close =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_HIT_AFTER_CLOSE_MSB,
				  FA2_FLOW_HIT_AFTER_CLOSE_WIDTH);
	sw_entry->flow_state =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_STATE_MSB,
				  FA2_FLOW_STATE_WIDTH);
	sw_entry->flow_timer =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_FLOW_TIMER_MSB,
				  FA2_FLOW_TIMER_WIDTH);
}

static void fa2_dbg_dcd_nhop(struct fa2_nhop_entry *sw_entry,
			     u32 *hw_entry)
{
	sw_entry->vlan =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_NEXT_HOP_VLAN_MSB,
				  FA2_NEXT_HOP_VLAN_WIDTH);
	sw_entry->op =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_NEXT_HOP_OP_MSB,
				  FA2_NEXT_HOP_OP_WIDTH);
	sw_entry->l2_frame_type =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_NEXT_HOP_L2_FRAME_TYPE_MSB,
				  FA2_NEXT_HOP_L2_FRAME_TYPE_WIDTH);
	sw_entry->da_0_15 =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_NEXT_HOP_DA_0_15_MSB,
				  FA2_NEXT_HOP_DA_0_15_WIDTH);
	sw_entry->da_16_47 =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_NEXT_HOP_DA_16_47_MSB,
				  FA2_NEXT_HOP_DA_16_47_WIDTH);
}

static void fa2_dbg_dcd_mtu(struct fa2_mtu_entry *sw_entry,
			    u32 *hw_entry)
{
	sw_entry->mtu =
		fa2_dbg_field_get((u8 *)hw_entry,
				  FA2_MTU_MSB,
				  FA2_MTU_WIDTH);
}

static void fa2_dbg_dump_port(struct fa *fa,
			      struct fa2_port_entry *sw_entry)
{
	dev_info(fa->dev, "external [%3d..%3d] = %u.\n",
		 FA2_PORT_EXTERNAL_MSB,
		 FA2_PORT_EXTERNAL_MSB - FA2_PORT_EXTERNAL_WIDTH + 1,
		 sw_entry->external);
}

static void fa2_dbg_dump_mac(struct fa *fa,
			     struct fa2_mac_entry *sw_entry)
{
	dev_info(fa->dev, "host [%3d..%3d] = %u.\n",
		 FA2_MAC_HOST_MSB,
		 FA2_MAC_HOST_MSB - FA2_MAC_HOST_WIDTH + 1,
		 sw_entry->host);

	dev_info(fa->dev, "l4_csum_check [%3d..%3d] = %u.\n",
		 FA2_MAC_L4_CSUM_CHECK_MSB,
		 FA2_MAC_L4_CSUM_CHECK_MSB - FA2_MAC_L4_CSUM_CHECK_WIDTH + 1,
		 sw_entry->l4_checksum_check);

	dev_info(fa->dev, "external [%3d..%3d] = %u.\n",
		 FA2_MAC_EXTERNAL_MSB,
		 FA2_MAC_EXTERNAL_MSB - FA2_MAC_EXTERNAL_WIDTH + 1,
		 sw_entry->external);

	dev_info(fa->dev, "rmac_0_15 [%3d..%3d] = 0x%04x.\n",
		 FA2_MAC_RMAC_0_15_MSB,
		 FA2_MAC_RMAC_0_15_MSB - FA2_MAC_RMAC_0_15_WIDTH + 1,
		 sw_entry->rmac_0_15);

	dev_info(fa->dev, "rmac_16_47 [%3d..%3d] = 0x%08x.\n",
		 FA2_MAC_RMAC_16_47_MSB,
		 FA2_MAC_RMAC_16_47_MSB - FA2_MAC_RMAC_16_47_WIDTH + 1,
		 sw_entry->rmac_16_47);
}

static void fa2_dbg_dump_flow(struct fa *fa,
			      struct fa2_flow_entry *sw_entry)
{
	dev_info(fa->dev, "ipv4_key_type [%3d..%3d] = %u.\n",
		 FA2_FLOW_IPV4_KEY_TYPE_MSB,
		 FA2_FLOW_IPV4_KEY_TYPE_MSB - FA2_FLOW_IPV4_KEY_TYPE_WIDTH + 1,
		 sw_entry->ipv4_key_type);

	dev_info(fa->dev, "valid [%3d..%3d] = %u.\n",
		 FA2_FLOW_VALID_MSB,
		 FA2_FLOW_VALID_MSB - FA2_FLOW_VALID_WIDTH + 1,
		 sw_entry->valid);

	dev_info(fa->dev, "sip [%3d..%3d] = 0x%08x.\n",
		 FA2_FLOW_SIP_MSB,
		 FA2_FLOW_SIP_MSB - FA2_FLOW_SIP_WIDTH + 1,
		 sw_entry->sip);

	dev_info(fa->dev, "dip[%3d..%3d] = 0x%08x.\n",
		 FA2_FLOW_DIP_MSB,
		 FA2_FLOW_DIP_MSB - FA2_FLOW_DIP_WIDTH + 1,
		 sw_entry->dip);

	dev_info(fa->dev, "protocol[%3d..%3d] = %u.\n",
		 FA2_FLOW_PROTOCOL_MSB,
		 FA2_FLOW_PROTOCOL_MSB - FA2_FLOW_PROTOCOL_WIDTH + 1,
		 sw_entry->protocol);

	dev_info(fa->dev, "sport [%3d..%3d] = %u.\n",
		 FA2_FLOW_SPORT_MSB,
		 FA2_FLOW_SPORT_MSB - FA2_FLOW_SPORT_WIDTH + 1,
		 sw_entry->sport);

	dev_info(fa->dev, "dport [%3d..%3d] = %u.\n",
		 FA2_FLOW_DPORT_MSB,
		 FA2_FLOW_DPORT_MSB - FA2_FLOW_DPORT_WIDTH + 1,
		 sw_entry->dport);

	dev_info(fa->dev, "rev_flow_ptr [%3d..%3d] = %u.\n",
		 FA2_FLOW_REV_FLOW_PTR_MSB,
		 FA2_FLOW_REV_FLOW_PTR_MSB - FA2_FLOW_REV_FLOW_PTR_WIDTH + 1,
		 sw_entry->rev_flow_ptr);

	dev_info(fa->dev, "brcm_tag_opcode [%3d..%3d] = %u.\n",
		 FA2_FLOW_BRCM_TAG_OPCODE_MSB,
		 FA2_FLOW_BRCM_TAG_OPCODE_MSB -
		 FA2_FLOW_BRCM_TAG_OPCODE_WIDTH + 1,
		 sw_entry->brcm_tag_opcode);

	dev_info(fa->dev, "brcm_tag_tc [%3d..%3d] = %u.\n",
		 FA2_FLOW_BRCM_TAG_TC_MSB,
		 FA2_FLOW_BRCM_TAG_TC_MSB - FA2_FLOW_BRCM_TAG_TC_WIDTH + 1,
		 sw_entry->brcm_tag_tc);

	dev_info(fa->dev, "brcm_tag_te [%3d..%3d] = %u.\n",
		 FA2_FLOW_BRCM_TAG_TE_MSB,
		 FA2_FLOW_BRCM_TAG_TE_MSB - FA2_FLOW_BRCM_TAG_TE_WIDTH + 1,
		 sw_entry->brcm_tag_te);

	dev_info(fa->dev, "brcm_tag_ts [%3d..%3d] = %u.\n",
		 FA2_FLOW_BRCM_TAG_TS_MSB,
		 FA2_FLOW_BRCM_TAG_TS_MSB - FA2_FLOW_BRCM_TAG_TS_WIDTH + 1,
		 sw_entry->brcm_tag_ts);

	dev_info(fa->dev, "brcm_tag_destmap [%3d..%3d] = %u.\n",
		 FA2_FLOW_BRCM_TAG_DESTMAP_MSB,
		 FA2_FLOW_BRCM_TAG_DESTMAP_MSB -
		 FA2_FLOW_BRCM_TAG_DESTMAP_WIDTH + 1,
		 sw_entry->brcm_tag_destmap);

	dev_info(fa->dev, "direction [%3d..%3d] = %u.\n",
		 FA2_FLOW_DIRECTION_MSB,
		 FA2_FLOW_DIRECTION_MSB - FA2_FLOW_DIRECTION_WIDTH + 1,
		 sw_entry->direction);

	dev_info(fa->dev, "l4_chksum_chk [%3d..%3d] = %u.\n",
		 FA2_FLOW_L4_CSUM_CHECK_MSB,
		 FA2_FLOW_L4_CSUM_CHECK_MSB -
		 FA2_FLOW_L4_CSUM_CHECK_WIDTH + 1,
		 sw_entry->l4_chksum_chk);

	dev_info(fa->dev, "ppp_tunnel_en [%3d..%3d] = %u.\n",
		 FA2_FLOW_PPP_TUNNEL_EN_MSB,
		 FA2_FLOW_PPP_TUNNEL_EN_MSB - FA2_FLOW_PPP_TUNNEL_EN_WIDTH + 1,
		 sw_entry->ppp_tunnel_en);

	dev_info(fa->dev, "ppp_tunnel_idx [%3d..%3d] = %u.\n",
		 FA2_FLOW_PPP_TUNNEL_IDX_MSB,
		 FA2_FLOW_PPP_TUNNEL_IDX_MSB -
		 FA2_FLOW_PPP_TUNNEL_IDX_WIDTH + 1,
		 sw_entry->ppp_tunnel_idx);

	dev_info(fa->dev, "mtu_idx [%3d..%3d] = %u.\n",
		 FA2_FLOW_MTU_IDX_MSB,
		 FA2_FLOW_MTU_IDX_MSB - FA2_FLOW_MTU_IDX_WIDTH + 1,
		 sw_entry->mtu_idx);

	dev_info(fa->dev, "next_hop_idx [%3d..%3d] = %u.\n",
		 FA2_FLOW_NEXT_HOP_IDX_MSB,
		 FA2_FLOW_NEXT_HOP_IDX_MSB - FA2_FLOW_NEXT_HOP_IDX_WIDTH + 1,
		 sw_entry->next_hop_idx);

	dev_info(fa->dev, "remap_sa_idx [%3d..%3d] = %u.\n",
		 FA2_FLOW_REMAP_SA_IDX_MSB,
		 FA2_FLOW_REMAP_SA_IDX_MSB - FA2_FLOW_REMAP_SA_IDX_WIDTH + 1,
		 sw_entry->remap_sa_idx);

	dev_info(fa->dev, "dest_dma_chan [%3d..%3d] = %u.\n",
		 FA2_FLOW_DEST_DMA_CHAN_MSB,
		 FA2_FLOW_DEST_DMA_CHAN_MSB - FA2_FLOW_DEST_DMA_CHAN_WIDTH + 1,
		 sw_entry->dest_dma_chan);

	dev_info(fa->dev, "action [%3d..%3d] = %u.\n",
		 FA2_FLOW_ACTION_MSB,
		 FA2_FLOW_ACTION_MSB - FA2_FLOW_ACTION_WIDTH + 1,
		 sw_entry->action);

	dev_info(fa->dev, "hits [%3d..%3d] = %u.\n",
		 FA2_FLOW_HITS_MSB,
		 FA2_FLOW_HITS_MSB - FA2_FLOW_HITS_WIDTH + 1,
		 sw_entry->hits);

	dev_info(fa->dev, "tcp_fin [%3d..%3d] = %u.\n",
		 FA2_FLOW_TCP_FIN_MSB,
		 FA2_FLOW_TCP_FIN_MSB - FA2_FLOW_TCP_FIN_WIDTH + 1,
		 sw_entry->tcp_fin);

	dev_info(fa->dev, "tcp_rst [%3d..%3d] = %u.\n",
		 FA2_FLOW_TCP_RST_MSB,
		 FA2_FLOW_TCP_RST_MSB - FA2_FLOW_TCP_RST_WIDTH + 1,
		 sw_entry->tcp_rst);

	dev_info(fa->dev, "tcp_ack_after_close [%3d..%3d] = %u.\n",
		 FA2_FLOW_TCP_ACK_AFTER_CLOSE_MSB,
		 FA2_FLOW_TCP_ACK_AFTER_CLOSE_MSB -
		 FA2_FLOW_TCP_ACK_AFTER_CLOSE_WIDTH + 1,
		 sw_entry->tcp_ack_after_close);

	dev_info(fa->dev, "hit_after_close [%3d..%3d] = %u.\n",
		 FA2_FLOW_HIT_AFTER_CLOSE_MSB,
		 FA2_FLOW_HIT_AFTER_CLOSE_MSB -
		 FA2_FLOW_HIT_AFTER_CLOSE_WIDTH + 1,
		 sw_entry->hit_after_close);

	dev_info(fa->dev, "flow_state [%3d..%3d] = %u.\n",
		 FA2_FLOW_STATE_MSB,
		 FA2_FLOW_STATE_MSB - FA2_FLOW_STATE_WIDTH + 1,
		 sw_entry->flow_state);

	dev_info(fa->dev, "flow_timer [%3d..%3d] = %u.\n",
		 FA2_FLOW_TIMER_MSB,
		 FA2_FLOW_TIMER_MSB - FA2_FLOW_TIMER_WIDTH + 1,
		 sw_entry->flow_timer);
}

static void fa2_dbg_dump_nhop(struct fa *fa, struct fa2_nhop_entry *sw_entry)
{
	dev_info(fa->dev, "vlan [%3d..%3d] = %u.\n",
		 FA2_NEXT_HOP_VLAN_MSB,
		 FA2_NEXT_HOP_VLAN_MSB - FA2_NEXT_HOP_VLAN_WIDTH + 1,
		 sw_entry->vlan);

	dev_info(fa->dev, "op [%3d..%3d] = %u.\n",
		 FA2_NEXT_HOP_OP_MSB,
		 FA2_NEXT_HOP_OP_MSB - FA2_NEXT_HOP_OP_WIDTH + 1,
		 sw_entry->op);

	dev_info(fa->dev, "l2_frame_type [%3d..%3d] = %u.\n",
		 FA2_NEXT_HOP_L2_FRAME_TYPE_MSB,
		 FA2_NEXT_HOP_L2_FRAME_TYPE_MSB -
		 FA2_NEXT_HOP_L2_FRAME_TYPE_WIDTH + 1,
		 sw_entry->l2_frame_type);

	dev_info(fa->dev, "da_0_15 [%3d..%3d] = 0x%04x.\n",
		 FA2_NEXT_HOP_DA_0_15_MSB,
		 FA2_NEXT_HOP_DA_0_15_MSB - FA2_NEXT_HOP_DA_0_15_WIDTH + 1,
		 sw_entry->da_0_15);

	dev_info(fa->dev, "da_16_47 [%3d..%3d] = 0x%08x.\n",
		 FA2_NEXT_HOP_DA_16_47_MSB,
		 FA2_NEXT_HOP_DA_16_47_MSB - FA2_NEXT_HOP_DA_16_47_WIDTH + 1,
		 sw_entry->da_16_47);
}

static void fa2_dbg_dump_mtu(struct fa *fa,
			     struct fa2_mtu_entry *sw_entry)
{
	dev_info(fa->dev, "mtu [%3d..%3d] = %u.\n",
		 FA2_MTU_MSB,
		 FA2_MTU_MSB - FA2_MTU_WIDTH + 1,
		 sw_entry->mtu);
}

static void fa2_dbg_dump_stats(struct fa *fa)
{
	u32 val;

	val = fa->read(fa, FA2_HIT_COUNT_OFF);
	if (val)
		dev_info(fa->dev, "   HitCnt(%d)\n", val);

	val = fa->read(fa, FA2_MISS_COUNT_OFF);
	if (val)
		dev_info(fa->dev, "   MissCnt(%d)\n", val);

	val = fa->read(fa, FA2_NAPT_FLOW_MISS_COUNT_OFF);
	if (val)
		dev_info(fa->dev, "   NaptMissCnt(%d)\n", val);

	val = fa->read(fa, FA2_TUNNEL_DROP_COUNT_OFF);
	if (val)
		dev_info(fa->dev, "   TunnelDropCnt(%d)\n", val);

	val = fa->read(fa, FA2_FRAG_PKTS_COUNT_OFF);
	if (val)
		dev_info(fa->dev, "   FragPktCnt(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF);
	if (val)
		dev_info(fa->dev, "0x1 HdrChkRxOversizeErr(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 4);
	if (val)
		dev_info(fa->dev, "0x2 HdrChkUnimacCrcErr(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 8);
	if (val)
		dev_info(fa->dev, "0x3 HdrChkUnknownOpcode(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 12);
	if (val)
		dev_info(fa->dev, "0x4 HdrChkRcErr(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 16);
	if (val)
		dev_info(fa->dev, "0x5 HdrChkProcOpErr(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 20);
	if (val)
		dev_info(fa->dev, "0x6 HdrChkL2SnapFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 24);
	if (val)
		dev_info(fa->dev, "0x7 HdrChkL2BypassEt(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 28);
	if (val)
		dev_info(fa->dev, "0x8 HdrChkL2EtCheckFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 32);
	if (val)
		dev_info(fa->dev, "0x9 HdrChkL2PppoeLenFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 36);
	if (val)
		dev_info(fa->dev, "0xa HdrChkL2PppoeDisc(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 40);
	if (val)
		dev_info(fa->dev, "0xb HdrChkL2PppoeProtFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 44);
	if (val)
		dev_info(fa->dev, "0xc HdrChkL3VersionFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 48);
	if (val)
		dev_info(fa->dev, "0xd HdrChkL3TtlFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 52);
	if (val)
		dev_info(fa->dev, "0xe HdrChkL3OptionsFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 56);
	if (val)
		dev_info(fa->dev, "0xf HdrChkL3HdrLenFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 60);
	if (val)
		dev_info(fa->dev, "0x10 HdrChkL3Ipv4CsumFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 64);
	if (val)
		dev_info(fa->dev, "0x11 HdrChkL3Ipv4FragFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 68);
	if (val)
		dev_info(fa->dev, "0x12 HdrChkFlowMissL2DmacHost(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 72);
	if (val)
		dev_info(fa->dev, "0x13 HdrChkFlowMissL3Proto(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 76);
	if (val)
		dev_info(fa->dev, "0x14 HdrChkL2EtSnapCvtFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 80);
	if (val)
		dev_info(fa->dev, "0x15 HdrChkL4TcpFrameSyn(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 84);
	if (val)
		dev_info(fa->dev, "0x16 HdrChkRouterMacMiss(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 88);
	if (val)
		dev_info(fa->dev, "0x17 HdrChkFlowMacDirHdrPortMiss(%d)\n",
			 val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 92);
	if (val)
		dev_info(fa->dev, "0x18 HdrChkFlowMacDirMiss(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 96);
	if (val)
		dev_info(fa->dev, "0x19 HdrChkFlowPppoeSessionMiss(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 100);
	if (val)
		dev_info(fa->dev, "0x1a HdrChkFlowPPPoeHit(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 104);
	if (val)
		dev_info(fa->dev, "0x1b HdrChkReversLookupMiss(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 108);
	if (val)
		dev_info(fa->dev, "0x1c HdrChkInFlowMiss(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 112);
	if (val)
		dev_info(fa->dev, "0x1d HdrChkOutFlowMiss(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 116);
	if (val)
		dev_info(fa->dev, "0x1e HdrChkPppoeIntern(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 120);
	if (val)
		dev_info(fa->dev, "0x1f HdrChkPppoeBindFail(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 124);
	if (val)
		dev_info(fa->dev, "0x20 HdrChkFragIpv4MtuVio(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 128);
	if (val)
		dev_info(fa->dev, "0x21 HdrChkIpv4FragErr(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 132);
	if (val)
		dev_info(fa->dev, "0x22 HdrChkFragDisMtuErr(%d)\n", val);

	val = fa->read(fa, FA2_HDR_CHECK_COUNT_OFF + 136);
	if (val)
		dev_info(fa->dev, "0x23 HdrChkPkt2xMtuErr(%d)\n", val);
}

/* dump table[type].entry[index] with option to dump decoded fields
 */
static u32 fa2_dbg_dump_table(struct fa *fa, int type, u32 index, bool dcd)
{
	u32 hw_entry[FA2_MAX_TABLE_ENTRY_SIZE_WORDS];
	int err;

	/* check type */
	if (type >= FA2_NUM_TABLES) {
		dev_err(fa->dev, "type = %d is out of range, [0..%d)", type,
			FA2_NUM_TABLES);
		return -EINVAL;
	}

	/* clear hw_entry since we are printing all 7 words */
	memset(hw_entry, 0, sizeof(hw_entry));

	/* read from hw */
	err = fa->read_table(fa, type, index, &hw_entry[0]);
	if (err)
		return err;

	/* if decoding fields */
	if (dcd) {
		u8 sw_entry[512];
		dev_info(fa->dev, "***index(%d)***\n", index);
		switch (type) {
		case FA2_FLOW_TABLE:
			fa2_dbg_dcd_flow((struct fa2_flow_entry *)sw_entry,
					 hw_entry);
			fa2_dbg_dump_flow(fa,
					  (struct fa2_flow_entry *)sw_entry);
			break;

		case FA2_MAC_TABLE:
			fa2_dbg_dcd_mac((struct fa2_mac_entry *)sw_entry,
					hw_entry);
			fa2_dbg_dump_mac(fa, (struct fa2_mac_entry *)sw_entry);
			break;

		case FA2_NEXT_HOP_TABLE:
			fa2_dbg_dcd_nhop((struct fa2_nhop_entry *)sw_entry,
					 hw_entry);
			fa2_dbg_dump_nhop(fa,
					  (struct fa2_nhop_entry *)sw_entry);
			break;

		case FA2_MTU_TABLE:
			fa2_dbg_dcd_mtu((struct fa2_mtu_entry *)sw_entry,
					hw_entry);
			fa2_dbg_dump_mtu(fa, (struct fa2_mtu_entry *)sw_entry);
			break;

		case FA2_PORT_TABLE:
			fa2_dbg_dcd_port((struct fa2_port_entry *)sw_entry,
					 hw_entry);
			fa2_dbg_dump_port(fa,
					  (struct fa2_port_entry *)sw_entry);
			break;
		default:
			dev_err(fa->dev, "invalid table(%d)\n", type);
			break;
		}
	}
	return 0;
}

static u32 fa2_dbg_get_flow_table(struct fa *fa)
{
	u32 i;
	struct fa2_flow_entry *flow;
	struct fa2_flow_entry *peer;

	dev_info(fa->dev, "HW flow table");
	for (i = 0; i < FA2_FLOW_TABLE_SIZE; i++) {
		flow = fa2_tbl_get_flow(i);

		if (!FLOW_ENTRY_IS_HW_OWNED(flow) ||
		    (flow->rev_flow_ptr < i))
			continue;

		dev_info(fa->dev, ENTRY_HEADER_STR);
		dev_info(fa->dev, "\n");
		if (flow->aged)
			dev_info(fa->dev, "flow entry %d : <aged>", i);
		else if ((flow->tcp_fin) || (flow->tcp_rst))
			dev_info(fa->dev, "flow entry %d : <historical>", i);
		else
			dev_info(fa->dev, "flow entry %d :", i);

		if (FLOW_ENTRY_IS_V4(flow))
			fa2_dbg_dump_table(fa, FA2_FLOW_TABLE, i, true);

		peer = fa2_tbl_get_flow(flow->rev_flow_ptr);
		dev_info(fa->dev, "\n");
		if (peer->aged)
			dev_info(fa->dev, "flow peer %d : <aged>",
				 flow->rev_flow_ptr);
		else
			dev_info(fa->dev, "flow peer %d :", flow->rev_flow_ptr);

		if (FLOW_ENTRY_IS_V4(peer))
			fa2_dbg_dump_table(fa, FA2_FLOW_TABLE,
					   flow->rev_flow_ptr,
					   true);
	}
	dev_info(fa->dev, TABLE_FOOTER_STR);
	return 0;
}

/* dump sw flow table
 */
static void fa2_dbg_dump_flow_cache(struct fa *fa)
{
	u32 i;
	struct fa2_flow_entry *flow;

	dev_info(fa->dev, "SW flow table cache");

	for (i = 0; i < FA2_FLOW_TABLE_SIZE; i++) {
		flow = fa2_tbl_get_flow(i);
		if (!FLOW_ENTRY_IS_TX_OWNED(flow))
			continue;
		dev_info(fa->dev, ENTRY_HEADER_STR);
		dev_info(fa->dev, "flow entry %d : <unidirectional>", i);
		if (FLOW_ENTRY_IS_V4(flow))
			fa2_dbg_dump_flow(fa, flow);
	}
	dev_info(fa->dev, TABLE_FOOTER_STR);
}

/* dump hw next hop table
 */
static int fa2_dbg_get_nhop_table(struct fa *fa)
{
	u32 i;
	struct fa2_nhop_entry *nhop;

	dev_info(fa->dev, "next hop table");
	for (i = 0; i < FA2_NEXT_HOP_TABLE_SIZE; i++) {
		nhop = fa2_tbl_get_nhop(i);
		if (!nhop->op)
			continue;

		dev_info(fa->dev, ENTRY_HEADER_STR);
		fa2_dbg_dump_table(fa, FA2_NEXT_HOP_TABLE, i, true);
	}
	dev_info(fa->dev, TABLE_FOOTER_STR);
	return 0;
}

/* dump hw mac table
 */
static int fa2_dbg_get_mac_table(struct fa *fa)
{
	u32 i;
	struct fa2_mac_entry *mac;

	dev_info(fa->dev, "mac table");
	for (i = 0; i < FA2_MAC_TABLE_SIZE; i++) {
		mac = fa2_tbl_get_mac(i);
		if (!mac->host)
			continue;

		dev_info(fa->dev, ENTRY_HEADER_STR);
		fa2_dbg_dump_table(fa, FA2_MAC_TABLE, i, true);
	}
	dev_info(fa->dev, TABLE_FOOTER_STR);
	return 0;
}

/* fa2_dbg_table_ops_read - read for table_ops data
 */
static ssize_t fa2_dbg_table_ops_read(struct file *filp, char __user *buffer,
				      size_t count, loff_t *ppos)
{
	char *buf;
	int len;

	/* don't allow partial reads */
	if (*ppos != 0)
		return 0;

	buf = kasprintf(GFP_KERNEL, "%s: %s\n",
			fa_driver_name,
			fa2_dbg_table_ops_buf);
	if (!buf)
		return -ENOMEM;

	if (count < strlen(buf)) {
		kfree(buf);
		return -ENOSPC;
	}

	len = simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));

	kfree(buf);
	return len;
}

/* fa2_dbg_table_ops_write - write into reg_ops datum
 */
static ssize_t fa2_dbg_table_ops_write(struct file *filp,
				       const char __user *buffer,
				       size_t count, loff_t *ppos)
{
	struct fa *fa = filp->private_data;
	int len;
	unsigned long int index;
	int rc;

	/* don't allow partial writes */
	if (*ppos != 0)
		return 0;
	if (count >= sizeof(fa2_dbg_table_ops_buf))
		return -ENOSPC;

	len = simple_write_to_buffer(fa2_dbg_table_ops_buf,
				     sizeof(fa2_dbg_table_ops_buf) - 1,
				     ppos,
				     buffer,
				     count);
	if (len < 0)
		return len;

	fa2_dbg_table_ops_buf[len] = '\0';

	if (strncmp(fa2_dbg_table_ops_buf, "port", 4) == 0) {
		rc = kstrtoul(&fa2_dbg_table_ops_buf[5], 16, &index);
		if (!rc && (index < FA2_PORT_TABLE_SIZE))
			fa2_dbg_dump_table(fa, FA2_PORT_TABLE, index, true);
		else
			dev_info(fa->dev, "port <src_pid 0-31>\n");

	} else if (strncmp(fa2_dbg_table_ops_buf, "mac_all", 7) == 0) {
		fa2_dbg_get_mac_table(fa);

	} else if (strncmp(fa2_dbg_table_ops_buf, "mac", 3) == 0) {
		rc = kstrtoul(&fa2_dbg_table_ops_buf[4], 16, &index);

		if (!rc && (index < FA2_MAC_TABLE_SIZE))
			fa2_dbg_dump_table(fa, FA2_MAC_TABLE, index, true);
		else
			dev_info(fa->dev, "mac <remap_sa_idx 0-7>\n");

	} else if (strncmp(fa2_dbg_table_ops_buf, "flow_all", 8) == 0) {
		fa2_dbg_get_flow_table(fa);

	} else if (strncmp(fa2_dbg_table_ops_buf, "flow", 4) == 0) {
		rc = kstrtoul(&fa2_dbg_table_ops_buf[5], 16, &index);

		if (!rc && (index < FA2_FLOW_TABLE_SIZE))
			fa2_dbg_dump_table(fa, FA2_FLOW_TABLE, index, true);
		else
			dev_info(fa->dev, "flow <hash_bucket 0-4095>\n");

	}  else if (strncmp(fa2_dbg_table_ops_buf, "nexthop_all", 11) == 0) {
		fa2_dbg_get_nhop_table(fa);

	} else if (strncmp(fa2_dbg_table_ops_buf, "nexthop", 7) == 0) {
		rc = kstrtoul(&fa2_dbg_table_ops_buf[8], 16, &index);

		if (!rc && (index < FA2_NEXT_HOP_TABLE_SIZE))
			fa2_dbg_dump_table(fa, FA2_NEXT_HOP_TABLE, index, true);
		else
			dev_info(fa->dev, "nexthop <idx 0-128>\n");

	} else if (strncmp(fa2_dbg_table_ops_buf, "swflowcache", 11) == 0) {
		fa2_dbg_dump_flow_cache(fa);

	} else if (strncmp(fa2_dbg_table_ops_buf, "mtu", 3) == 0) {
		rc = kstrtoul(&fa2_dbg_table_ops_buf[4], 16, &index);

		if (!rc && (index < FA2_MTU_TABLE_SIZE))
			fa2_dbg_dump_table(fa, FA2_MTU_TABLE, index, true);
		else
			dev_info(fa->dev, "mtu <idx 0-7>\n");
	} else if (strncmp(fa2_dbg_table_ops_buf, "stats", 4) == 0) {
		fa2_dbg_dump_stats(fa);
	} else if (strncmp(fa2_dbg_table_ops_buf, "hook_on", 7) == 0) {
		fa->hook_is_enabled = true;
		dev_info(fa->dev, "netfilter hook enabled\n");
	} else if (strncmp(fa2_dbg_table_ops_buf, "hook_off", 8) == 0) {
		fa->hook_is_enabled = false;
		dev_info(fa->dev, "netfilter hook disabled\n");
	} else {
		dev_info(fa->dev, "Unknown cmd %s\n", fa2_dbg_table_ops_buf);
		dev_info(fa->dev, "Available Flow Accelerator debug commands:\n");
		dev_info(fa->dev, "  port <src_pid>       -dump port tbl entry\n");
		dev_info(fa->dev, "  mac <remap_sa_idx>   -dump mac tbl entry\n");
		dev_info(fa->dev, "  mac_all              -dump all mac tbl\n");
		dev_info(fa->dev, "  flow <hash_bucket>   -dump flow tbl entry\n");
		dev_info(fa->dev, "  flow_all             -dump all flow tbl\n");
		dev_info(fa->dev, "  nexthop <nh_idx>     -dump nhop tbl entry\n");
		dev_info(fa->dev, "  nexthop_all\n        -dump all nhop tbl");
		dev_info(fa->dev, "  mtu <mtu_idx>        -dump mtu tbl entry\n");
		dev_info(fa->dev, "  swflowcache          -dump sw flow cache\n");
		dev_info(fa->dev, "  stats                -dump dbg stats\n");
		dev_info(fa->dev, "  hook_on              -enable flow setup\n");
		dev_info(fa->dev, "  hook_off             -disable flow setup\n");
	}
	return count;
}

static const struct file_operations fa2_dbg_table_ops_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read =  fa2_dbg_table_ops_read,
	.write = fa2_dbg_table_ops_write,
};

void fa2_dbg_exit(struct fa *fa)
{
	debugfs_remove_recursive(fa2_dbg_root);
	fa2_dbg_root = NULL;
}

/* fa2_dbg_init - start up debugfs for the driver
 */
void fa2_dbg_init(struct fa *fa)
{
	struct dentry *pfile;

	fa2_dbg_root = debugfs_create_dir(fa_driver_name, NULL);
	if (!fa2_dbg_root) {
		dev_err(fa->dev, "init of debugfs failed\n");
	} else {
		pfile = debugfs_create_file("table_ops", 0600,
					    fa2_dbg_root, fa,
					    &fa2_dbg_table_ops_fops);
		if (!pfile)
			dev_err(fa->dev, "table_ops %s failed\n",
				fa_driver_name);
		else
			dev_info(fa->dev, "%s: created table_ops file\n",
				 fa_driver_name);
	}
}
