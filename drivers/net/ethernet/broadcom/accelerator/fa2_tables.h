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

#ifndef __FA2_TABLES_H__
#define __FA2_TABLES_H__

#define FA2_FLOW_TABLE		0
#define FA2_MAC_TABLE		1
#define FA2_NEXT_HOP_TABLE	2
#define FA2_MTU_TABLE		3
#define FA2_TUNNEL_TABLE	4
#define FA2_PORT_TABLE		5
#define FA2_NUM_TABLES		6

#define FA2_FLOW_TABLE_SIZE	4096
#define FA2_MAC_TABLE_SIZE	8
#define FA2_NEXT_HOP_TABLE_SIZE	128
#define FA2_MTU_TABLE_SIZE	8
#define FA2_TUNNEL_TABLE_SIZE	16
#define FA2_PORT_TABLE_SIZE	32

/* Size in 32 bit words
 */
#define FA2_HW_TABLE_ENTRY_MAX	7
#define FA2_NAT_KEY_SIZE	5

/* Default mtu size index
 */
#define FA2_MTU_DEFAULT_IDX	1

/* Port table - used to designate whether received
 * packet is from an internal or an external port as
 * indexed by the source port.
 *
 * source robo port->port table-->int (LAN)/ ext (WAN)
 */
struct __packed fa2_port_entry {
	u8 external;
};

#define FA2_PORT_EXTERNAL_MSB		0
#define FA2_PORT_EXTERNAL_WIDTH		1

/* Router MAC Table - there are only 8 entries here.
 * All entries are searched for a DA match by the hardware.
 */
struct __packed fa2_mac_entry {
	u8 host;
	u8 l4_checksum_check;
	u8 external;
	u16 rmac_0_15;
	u32 rmac_16_47;
};

#define FA2_MAC_HOST_MSB			50
#define FA2_MAC_HOST_WIDTH			1
#define FA2_MAC_L4_CSUM_CHECK_MSB		49
#define FA2_MAC_L4_CSUM_CHECK_WIDTH		1
#define FA2_MAC_EXTERNAL_MSB			48
#define FA2_MAC_EXTERNAL_WIDTH			1
#define FA2_MAC_RMAC_0_15_MSB			47
#define FA2_MAC_RMAC_0_15_WIDTH			16
#define FA2_MAC_RMAC_16_47_MSB			31
#define FA2_MAC_RMAC_16_47_WIDTH		32

/* rules to keep flow table lock free
 */
#define FLOW_ENTRY_IS_V4(flow) (((flow)->valid) && \
				((flow)->ipv4_key_type))

#define FLOW_ENTRY_IS_RX_OWNED(flow) ((flow)->valid)

#define FLOW_ENTRY_IS_TX_OWNED(flow) \
			(((flow)->valid) && \
			((flow)->rev_flow_ptr \
			== UNKNOWN_REV_PTR)) /* uni-directional */

#define FLOW_ENTRY_IS_HW_OWNED(flow) \
			(((flow)->valid) && \
			((flow)->rev_flow_ptr \
			!= UNKNOWN_REV_PTR))	/* with a peer */

struct __packed fa2_flow_entry {
	bool aged;
	struct nf_conntrack *nfct;
	struct {
		u8 ipv4_key_type;
		u8 valid;
		u32 sip;
		u32 dip;
		u8 protocol;
		u16 sport;
		u16 dport;
		u16 rev_flow_ptr;
		u8 brcm_tag_opcode;
		u8 brcm_tag_tc;
		u8 brcm_tag_te;
		u8 brcm_tag_ts;
		u16 brcm_tag_destmap;
		u8 direction;
		u8 l4_chksum_chk;
		u8 ppp_tunnel_en;
		u8 ppp_tunnel_idx;
		u8 mtu_idx;
		u8 next_hop_idx;
		u8 remap_sa_idx;
		u8 dest_dma_chan;
		u8 action;
		u32 hits;
		u8 tcp_fin;
		u8 tcp_rst;
		u8 tcp_ack_after_close;
		u8 hit_after_close;
		u8 flow_state;
		u8 flow_timer;
	} __packed;
};

struct fa2_flow_entry *fa2_tbl_get_flow(u32 index);
struct fa2_mac_entry *fa2_tbl_get_mac(u32 index);
struct fa2_nhop_entry *fa2_tbl_get_nhop(u32 index);

#define FA2_FLOW_IPV4_KEY_TYPE_MSB		208
#define FA2_FLOW_IPV4_KEY_TYPE_WIDTH		1
#define FA2_FLOW_VALID_MSB			207
#define FA2_FLOW_VALID_WIDTH			1
#define FA2_FLOW_SIP_MSB			206
#define FA2_FLOW_SIP_WIDTH			32
#define FA2_FLOW_DIP_MSB			174
#define FA2_FLOW_DIP_WIDTH			32
#define FA2_FLOW_PROTOCOL_MSB			142
#define FA2_FLOW_PROTOCOL_WIDTH			8
#define FA2_FLOW_SPORT_MSB			134
#define FA2_FLOW_SPORT_WIDTH			16
#define FA2_FLOW_DPORT_MSB			118
#define FA2_FLOW_DPORT_WIDTH			16
#define FA2_FLOW_REV_FLOW_PTR_MSB		102
#define FA2_FLOW_REV_FLOW_PTR_WIDTH		12
#define FA2_FLOW_BRCM_TAG_OPCODE_MSB		90
#define FA2_FLOW_BRCM_TAG_OPCODE_WIDTH		3
#define FA2_FLOW_BRCM_TAG_TC_MSB		87
#define FA2_FLOW_BRCM_TAG_TC_WIDTH		3
#define FA2_FLOW_BRCM_TAG_TE_MSB		84
#define FA2_FLOW_BRCM_TAG_TE_WIDTH		2
#define FA2_FLOW_BRCM_TAG_TS_MSB		82
#define FA2_FLOW_BRCM_TAG_TS_WIDTH		1
#define FA2_FLOW_BRCM_TAG_DESTMAP_MSB		81
#define FA2_FLOW_BRCM_TAG_DESTMAP_WIDTH		10
#define FA2_FLOW_DIRECTION_MSB			71
#define FA2_FLOW_DIRECTION_WIDTH		1
#define FA2_FLOW_L4_CSUM_CHECK_MSB		70
#define FA2_FLOW_L4_CSUM_CHECK_WIDTH		1
#define FA2_FLOW_PPP_TUNNEL_EN_MSB		69
#define FA2_FLOW_PPP_TUNNEL_EN_WIDTH		1
#define FA2_FLOW_PPP_TUNNEL_IDX_MSB		68
#define FA2_FLOW_PPP_TUNNEL_IDX_WIDTH		4
#define FA2_FLOW_MTU_IDX_MSB			64
#define FA2_FLOW_MTU_IDX_WIDTH			3
#define FA2_FLOW_NEXT_HOP_IDX_MSB		61
#define FA2_FLOW_NEXT_HOP_IDX_WIDTH		7
#define FA2_FLOW_REMAP_SA_IDX_MSB		54
#define FA2_FLOW_REMAP_SA_IDX_WIDTH		3
#define FA2_FLOW_DEST_DMA_CHAN_MSB		51
#define FA2_FLOW_DEST_DMA_CHAN_WIDTH		2
#define FA2_FLOW_ACTION_MSB			49
#define FA2_FLOW_ACTION_WIDTH			3
#define FA2_FLOW_HITS_MSB			46
#define FA2_FLOW_HITS_WIDTH			32
#define FA2_FLOW_TCP_FIN_MSB			14
#define FA2_FLOW_TCP_FIN_WIDTH			1
#define FA2_FLOW_TCP_RST_MSB			13
#define FA2_FLOW_TCP_RST_WIDTH			1
#define FA2_FLOW_TCP_ACK_AFTER_CLOSE_MSB	12
#define FA2_FLOW_TCP_ACK_AFTER_CLOSE_WIDTH	1
#define FA2_FLOW_HIT_AFTER_CLOSE_MSB		11
#define FA2_FLOW_HIT_AFTER_CLOSE_WIDTH		1
#define FA2_FLOW_STATE_MSB			10
#define FA2_FLOW_STATE_WIDTH			3
#define FA2_FLOW_TIMER_MSB			7
#define FA2_FLOW_TIMER_WIDTH			8

struct __packed fa2_nhop_entry {
	u16 vlan;
	u8 op;
	u8 l2_frame_type;
	u16 da_0_15;
	u32 da_16_47;
};

#define FA2_NEXT_HOP_VLAN_MSB			67
#define FA2_NEXT_HOP_VLAN_WIDTH			16
#define FA2_NEXT_HOP_OP_MSB			51
#define FA2_NEXT_HOP_OP_WIDTH			3
#define FA2_NEXT_HOP_L2_FRAME_TYPE_MSB		48
#define FA2_NEXT_HOP_L2_FRAME_TYPE_WIDTH	1
#define FA2_NEXT_HOP_DA_0_15_MSB		47
#define FA2_NEXT_HOP_DA_0_15_WIDTH		16
#define FA2_NEXT_HOP_DA_16_47_MSB		31
#define FA2_NEXT_HOP_DA_16_47_WIDTH		32

/* Next hop operations */
#define FA2_NHOP_OP_REPLACE_HDR_S_TAG		0
#define FA2_NHOP_OP_REPLACE_HDR_C_TAG		1
#define FA2_NHOP_OP_REPLACE_HDR_NO_TAG		2
#define FA2_NHOP_OP_UPDATE_SA_DA_ONLY		3
#define FA2_NHOP_OP_UPDATE_SA_ONLY		4
#define FA2_NHOP_OP_UPDATE_NONE			5

/* Broadcom tags can be 4 or 8 bytes */
#define BCM_TAG_LEN_MAX 8
#define FA2_MTU_DEFAULT_SIZE (ETH_FRAME_LEN + ETH_FCS_LEN + BCM_TAG_LEN_MAX)

struct __packed fa2_mtu_entry {
	u16 mtu;
};

#define FA2_MTU_MSB				13
#define FA2_MTU_WIDTH				14

/* use table size as peer flow id until determined
 */
#define UNKNOWN_REV_PTR FA2_FLOW_TABLE_SIZE

/* Size of the maximum size table entry */
#define FA2_MAX_TABLE_ENTRY_SIZE_WORDS 7

int fa2_tbl_update_mtu_table(struct fa *fa, u16 mtu, u32 index);
int fa2_tbl_update_port_table(struct fa *fa, u8 external, u32 index);
int fa2_tbl_update_sw_flow_table(struct fa *fa, struct fa_frame_info *info);
int fa2_tbl_update_hw_flow_table(struct fa *fa, struct fa_frame_info *info);
int fa2_tbl_update_table(struct fa *fa, int type, u32 index, void *sw_entry);
#endif /* __FA2_TABLES_H__ */
