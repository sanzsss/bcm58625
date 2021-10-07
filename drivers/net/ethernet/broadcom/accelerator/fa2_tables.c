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
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <net/ip.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/string.h>

#include "fa.h"
#include "fa2.h"
#include "fa2_tables.h"

/* NAPT functionality
 *        WAN      BCM SoC    LAN
 *       SMAC=Y      box     SMAC=L
 *       DMAC=W       /      DMAC=X
 * MAC Y -----> MAC W / MAC L -----> MAC=X
 *                    /             (192.168...)
 *
 * The above shows a possible NAPT forwarding setup which can
 * support NAPT hardware flow acceleration.
 */

/* sw flow table -
 * in order to be lockless, following rules must be observed by rx, tx and
 * aging code
 * [valid, rev_flow_ptr]     entry ownership
 * [0, DON'T CARE]        => rx
 * [1, UNKNOWN_REV_PTR]   => tx
 * [1, !UNKNOWN_REV_PTR]  => aging
 */
struct fa2_flow_entry fa2_flow[FA2_FLOW_TABLE_SIZE];

/* sw mac table
 */
struct fa2_mac_entry fa2_mac[FA2_MAC_TABLE_SIZE];

/* sw next hop table
 */
struct fa2_nhop_entry fa2_hop[FA2_NEXT_HOP_TABLE_SIZE];

/* Get a sw flow table entry
 */
struct fa2_flow_entry *fa2_tbl_get_flow(u32 index)
{
	if (index < FA2_FLOW_TABLE_SIZE)
		return &fa2_flow[index];
	else
		return NULL;
}

/* Get a sw mac entry
 */
struct fa2_mac_entry *fa2_tbl_get_mac(u32 index)
{
	if (index < FA2_MAC_TABLE_SIZE)
		return &fa2_mac[index];
	else
		return NULL;
}

/* Get a sw next hop entry
 */
struct fa2_nhop_entry *fa2_tbl_get_nhop(u32 index)
{
	if (index < FA2_NEXT_HOP_TABLE_SIZE)
		return &fa2_hop[index];
	else
		return NULL;
}

/* Given the msb, the width and a pointer to the data
 * write the value to the table.
 */
static inline void
fa2_tbl_field_set(u8 *data, u32 msb, u32 width, u32 value)
{
	for (msb -= width - 1; value && width; --width, ++msb) {
		if (value & 0x01)
			data[msb / 8] |= (0x01 << (msb % 8));
		value >>= 1;
	}
}

static void fa2_tbl_encode_port_entry(struct fa2_port_entry *sw_entry,
				      u32 *hw_entry)
{
	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_PORT_EXTERNAL_MSB,
			  FA2_PORT_EXTERNAL_WIDTH,
			  sw_entry->external);
}

static void fa2_tbl_encode_mac_entry(struct fa2_mac_entry *sw_entry,
				     u32 *hw_entry)
{
	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_MAC_HOST_MSB,
			  FA2_MAC_HOST_WIDTH,
			  sw_entry->host);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_MAC_L4_CSUM_CHECK_MSB,
			  FA2_MAC_L4_CSUM_CHECK_WIDTH,
			  sw_entry->l4_checksum_check);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_MAC_EXTERNAL_MSB,
			  FA2_MAC_EXTERNAL_WIDTH,
			  sw_entry->external);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_MAC_RMAC_0_15_MSB,
			  FA2_MAC_RMAC_0_15_WIDTH,
			  sw_entry->rmac_0_15);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_MAC_RMAC_16_47_MSB,
			  FA2_MAC_RMAC_16_47_WIDTH,
			  sw_entry->rmac_16_47);
}

static void fa2_tbl_encode_flow_entry(struct fa2_flow_entry *sw_entry,
				      u32 *hw_entry)
{
	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_IPV4_KEY_TYPE_MSB,
			  FA2_FLOW_IPV4_KEY_TYPE_WIDTH,
			  sw_entry->ipv4_key_type);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_VALID_MSB,
			  FA2_FLOW_VALID_WIDTH,
			  sw_entry->valid);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_SIP_MSB,
			  FA2_FLOW_SIP_WIDTH,
			  sw_entry->sip);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_DIP_MSB,
			  FA2_FLOW_DIP_WIDTH,
			  sw_entry->dip);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_PROTOCOL_MSB,
			  FA2_FLOW_PROTOCOL_WIDTH,
			  sw_entry->protocol);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_SPORT_MSB,
			  FA2_FLOW_SPORT_WIDTH,
			  sw_entry->sport);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_DPORT_MSB,
			  FA2_FLOW_DPORT_WIDTH,
			  sw_entry->dport);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_REV_FLOW_PTR_MSB,
			  FA2_FLOW_REV_FLOW_PTR_WIDTH,
			  sw_entry->rev_flow_ptr);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_BRCM_TAG_OPCODE_MSB,
			  FA2_FLOW_BRCM_TAG_OPCODE_WIDTH,
			  sw_entry->brcm_tag_opcode);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_BRCM_TAG_TC_MSB,
			  FA2_FLOW_BRCM_TAG_TC_WIDTH,
			  sw_entry->brcm_tag_tc);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_BRCM_TAG_TE_MSB,
			  FA2_FLOW_BRCM_TAG_TE_WIDTH,
			  sw_entry->brcm_tag_te);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_BRCM_TAG_TS_MSB,
			  FA2_FLOW_BRCM_TAG_TS_WIDTH,
			  sw_entry->brcm_tag_ts);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_BRCM_TAG_DESTMAP_MSB,
			  FA2_FLOW_BRCM_TAG_DESTMAP_WIDTH,
			  sw_entry->brcm_tag_destmap);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_DIRECTION_MSB,
			  FA2_FLOW_DIRECTION_WIDTH,
			  sw_entry->direction);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_L4_CSUM_CHECK_MSB,
			  FA2_FLOW_L4_CSUM_CHECK_WIDTH,
			  sw_entry->l4_chksum_chk);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_PPP_TUNNEL_EN_MSB,
			  FA2_FLOW_PPP_TUNNEL_EN_WIDTH,
			  sw_entry->ppp_tunnel_en);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_PPP_TUNNEL_IDX_MSB,
			  FA2_FLOW_PPP_TUNNEL_IDX_WIDTH,
			  sw_entry->ppp_tunnel_idx);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_MTU_IDX_MSB,
			  FA2_FLOW_MTU_IDX_WIDTH,
			  sw_entry->mtu_idx);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_NEXT_HOP_IDX_MSB,
			  FA2_FLOW_NEXT_HOP_IDX_WIDTH,
			  sw_entry->next_hop_idx);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_REMAP_SA_IDX_MSB,
			  FA2_FLOW_REMAP_SA_IDX_WIDTH,
			  sw_entry->remap_sa_idx);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_DEST_DMA_CHAN_MSB,
			  FA2_FLOW_DEST_DMA_CHAN_WIDTH,
			  sw_entry->dest_dma_chan);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_ACTION_MSB,
			  FA2_FLOW_ACTION_WIDTH,
			  sw_entry->action);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_HITS_MSB,
			  FA2_FLOW_HITS_WIDTH,
			  sw_entry->hits);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_TCP_FIN_MSB,
			  FA2_FLOW_TCP_FIN_WIDTH,
			  sw_entry->tcp_fin);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_TCP_RST_MSB,
			  FA2_FLOW_TCP_RST_WIDTH,
			  sw_entry->tcp_rst);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_TCP_ACK_AFTER_CLOSE_MSB,
			  FA2_FLOW_TCP_ACK_AFTER_CLOSE_WIDTH,
			  sw_entry->tcp_ack_after_close);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_HIT_AFTER_CLOSE_MSB,
			  FA2_FLOW_HIT_AFTER_CLOSE_WIDTH,
			  sw_entry->hit_after_close);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_STATE_MSB,
			  FA2_FLOW_STATE_WIDTH,
			  sw_entry->flow_state);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_FLOW_TIMER_MSB,
			  FA2_FLOW_TIMER_WIDTH,
			  sw_entry->flow_timer);
}

static void fa2_tbl_encode_nhop_entry(struct fa2_nhop_entry *sw_entry,
				      u32 *hw_entry)
{
	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_NEXT_HOP_VLAN_MSB,
			  FA2_NEXT_HOP_VLAN_WIDTH,
			  sw_entry->vlan);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_NEXT_HOP_OP_MSB,
			  FA2_NEXT_HOP_OP_WIDTH,
			  sw_entry->op);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_NEXT_HOP_L2_FRAME_TYPE_MSB,
			  FA2_NEXT_HOP_L2_FRAME_TYPE_WIDTH,
			  sw_entry->l2_frame_type);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_NEXT_HOP_DA_0_15_MSB,
			  FA2_NEXT_HOP_DA_0_15_WIDTH,
			  sw_entry->da_0_15);

	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_NEXT_HOP_DA_16_47_MSB,
			  FA2_NEXT_HOP_DA_16_47_WIDTH,
			  sw_entry->da_16_47);
}

static void fa2_tbl_encode_mtu_entry(struct fa2_mtu_entry *sw_entry,
				     u32 *hw_entry)
{
	fa2_tbl_field_set((u8 *)hw_entry,
			  FA2_MTU_MSB,
			  FA2_MTU_WIDTH,
			  sw_entry->mtu);
}

/* encode sw_entries and write to table[type].entry[index]
 */
int fa2_tbl_update_table(struct fa *fa, int type,
			 u32 index, void *sw_entry)
{
	u32 hw_entry[FA2_HW_TABLE_ENTRY_MAX];
	int rc;

	dev_dbg(fa->dev, "type = %d, index = %d", type, index);

	/* check type */
	if (type >= FA2_NUM_TABLES) {
		dev_err(fa->dev, "type = %d is out of range, [0..%d)",
			type, FA2_NUM_TABLES);
		return -EINVAL;
	}

	/* clear hw_entry */
	memset(hw_entry, 0, sizeof(hw_entry));

	switch (type) {
	case FA2_FLOW_TABLE:
		fa2_tbl_encode_flow_entry((struct fa2_flow_entry *)sw_entry,
					  hw_entry);
		break;

	case FA2_MAC_TABLE:
		fa2_tbl_encode_mac_entry((struct fa2_mac_entry *)sw_entry,
					 hw_entry);
		break;

	case FA2_NEXT_HOP_TABLE:
		fa2_tbl_encode_nhop_entry((struct fa2_nhop_entry *)sw_entry,
					  hw_entry);
		break;

	case FA2_MTU_TABLE:
		fa2_tbl_encode_mtu_entry((struct fa2_mtu_entry *)sw_entry,
					 hw_entry);
		break;

	case FA2_PORT_TABLE:
		fa2_tbl_encode_port_entry((struct fa2_port_entry *)sw_entry,
					  hw_entry);
		break;
	default:
		dev_err(fa->dev, "invalid table(%d)\n", type);
		break;
	}

	/* write hw_entry to hw */
	rc = fa->write_table(fa, type, index, hw_entry);

	return rc;
}

/* load hw port table base on system config
 */
int fa2_tbl_update_port_table(struct fa *fa, u8 external, u32 index)
{
	struct fa2_port_entry sw_entry;
	int err;

	dev_info(fa->dev, "Updating Port table external(%d), port(%d)",
		 external, index);

	sw_entry.external = external;
	err = fa2_tbl_update_table(fa, FA2_PORT_TABLE, index, &sw_entry);
	if (err) {
		dev_err(fa->dev, "Port Table write failed %d\n", err);
		return err;
	}
	return 0;
}

/* Update MAC table, if successful, 0 is returned and the remap_sa_idx is
 * returned for the given da and src_pid.
 */
int fa2_tbl_update_mac_table(struct fa *fa, u8 *da, u8 external,
			     u32 *remap_sa_idx)
{
	u32 i;
	struct fa2_mac_entry *mac;
	struct fa2_mac_entry sw_entry;
	int err;

	sw_entry.host = 1;
	sw_entry.l4_checksum_check = 0;
	sw_entry.external =  external;
	sw_entry.rmac_0_15 = ntohs(*((u16 *)&da[0]));
	sw_entry.rmac_16_47 = ntohl(*((u32 *)&da[2]));

	/* try finding a match */
	for (i = 0; i < FA2_MAC_TABLE_SIZE; i++) {
		mac = fa2_tbl_get_mac(i);
		if (!mac || !mac->host)
			continue;

		if ((mac->external == sw_entry.external) &&
		    (mac->rmac_0_15 == sw_entry.rmac_0_15) &&
		    (mac->rmac_16_47 == sw_entry.rmac_16_47)) {
			*remap_sa_idx = i;
			return 0;
		}
	}

	/* try adding an entry */
	for (i = 0; i < FA2_MAC_TABLE_SIZE; i++) {
		mac = fa2_tbl_get_mac(i);
		if (!mac || mac->host)
			continue;

		memcpy(mac, &sw_entry, sizeof(sw_entry));

		err = fa2_tbl_update_table(fa, FA2_MAC_TABLE, i, &sw_entry);
		if (err) {
			dev_err(fa->dev, "Port Table write failed %d\n", err);
			return err;
		}
		*remap_sa_idx = i;

		dev_dbg(fa->dev, "mac entry %d added", i);

		return 0;
	}

	dev_err(fa->dev, "failed to add MAC entry");
	return -ENOMEM;
}

/* find/add next hop table entry needed for creating a flow entry
 */
int fa2_tbl_update_nhop_table(struct fa *fa, u8 *sa,
			      u16 *vlan, u32 *index)
{
	u32 i;
	struct fa2_nhop_entry sw_entry;
	struct fa2_nhop_entry *nhop;
	int err;

	if (*vlan) {
		sw_entry.vlan = ntohs(*vlan);
		sw_entry.op = FA2_NHOP_OP_REPLACE_HDR_C_TAG;
	} else {
		sw_entry.vlan = 0;
		sw_entry.op = FA2_NHOP_OP_UPDATE_SA_DA_ONLY;
	}

	sw_entry.l2_frame_type = 0;
	sw_entry.da_0_15 = ntohs(*((u16 *)&sa[0]));
	sw_entry.da_16_47 = ntohl(*((u32 *)&sa[2]));

	/* try finding a match */
	for (i = 0; i < FA2_NEXT_HOP_TABLE_SIZE; i++) {
		nhop = fa2_tbl_get_nhop(i);
		if (!nhop || !nhop->op)
			continue;

		if ((nhop->vlan == sw_entry.vlan) &&
		    (nhop->da_0_15 == sw_entry.da_0_15) &&
		    (nhop->da_16_47 == sw_entry.da_16_47)) {
			*index = i;
			return 0;
		}
	}

	/* try adding an entry */
	for (i = 0; i < FA2_NEXT_HOP_TABLE_SIZE; i++) {
		nhop = fa2_tbl_get_nhop(i);
		if (!nhop || nhop->op)
			continue;

		memcpy(nhop, &sw_entry, sizeof(sw_entry));
		*index = i;

		err = fa2_tbl_update_table(fa, FA2_NEXT_HOP_TABLE,
					   i, &sw_entry);
		if (err) {
			dev_err(fa->dev, "Next hop table write error(%d)\n",
				err);
			return err;
		}
		dev_warn(fa->dev, "entry %d added", i);
		return 0;
	}
	dev_err(fa->dev, "failed to add entry");
	return -ENOMEM;
}

/* load hw mtu table base on system config
 */
int fa2_tbl_update_mtu_table(struct fa *fa, u16 mtu, u32 index)
{
	struct fa2_mtu_entry entry;
	int err;

	dev_dbg(fa->dev, "Updating MTU table idx(%d) mtu(%d)", index, mtu);

	entry.mtu = mtu;
	err = fa2_tbl_update_table(fa, FA2_MTU_TABLE, index, &entry);
	if (err) {
		dev_err(fa->dev, "Next hop table write error(%d)\n",
			err);
		return err;
	}

	return 0;
}

/* insert half flow entries into flow table as part of rx monitoring
 * Note that the flow table isn't updated in the hardware here.
 * But, the next hop table and the mac table is populated
 */
int fa2_tbl_update_sw_flow_table(struct fa *fa, struct fa_frame_info *info)
{
	u32 next_hop_idx, remap_sa_idx;
	struct fa2_flow_entry sw_entry;
	struct fa2_flow_entry *flow;
	int err;
	u8 external;

	flow = fa2_tbl_get_flow(info->flow_id);
	if (!flow)
		return -EINVAL;

	if (flow->valid) {
		dev_dbg(fa->dev, "flow %d is no longer owned by rx",
			info->flow_id);
	} else {
		err = fa2_tbl_update_nhop_table(fa, info->sa,
						&info->vlan,
						&next_hop_idx);
		if (err)
			return err;

		external = info->src_pid == FA2_WAN_PORT;

		err = fa2_tbl_update_mac_table(fa, info->da,
					       external,
					       &remap_sa_idx);
		if (err)
			return err;

		if (info->l3_proto == ETH_P_IP) {
			sw_entry.aged = false,
			sw_entry.nfct = info->nfct,
			sw_entry.ipv4_key_type = true,
			sw_entry.valid = true,
			sw_entry.sip = info->sip,
			sw_entry.dip = info->dip,
			sw_entry.protocol = info->l4_proto,
			sw_entry.sport = info->sport,
			sw_entry.dport = info->dport,
			sw_entry.rev_flow_ptr = UNKNOWN_REV_PTR,
			sw_entry.brcm_tag_opcode = 0,
			sw_entry.brcm_tag_tc = 0,
			sw_entry.brcm_tag_te = 0,
			sw_entry.brcm_tag_ts = 0,
			sw_entry.brcm_tag_destmap = 0,
			sw_entry.direction = external,
			sw_entry.l4_chksum_chk = 0,
			sw_entry.mtu_idx = FA2_MTU_DEFAULT_IDX,
			sw_entry.next_hop_idx = next_hop_idx,
			sw_entry.remap_sa_idx = remap_sa_idx,
			sw_entry.dest_dma_chan = 3,
			sw_entry.action = 1,
			sw_entry.hits = 0,
			sw_entry.tcp_fin = 0,
			sw_entry.tcp_rst = 0,
			sw_entry.tcp_ack_after_close = 0,
			sw_entry.hit_after_close = 0,
			sw_entry.flow_state = (fa->aging_is_enabled)
				    ? ((info->l4_proto == IPPROTO_TCP) ? 1 : 0)
				    : 2,
			sw_entry.flow_timer = (fa->aging_is_enabled)
				    ? ((info->l4_proto ==
					IPPROTO_TCP) ? 28 : 21)
				    : 0,

			dev_dbg(fa->dev, "entry %d is now ipv4", info->flow_id);

			memcpy(flow, &sw_entry, sizeof(sw_entry));
		} else {
			dev_dbg(fa->dev, "IPv6 unsupported\n");
		}
	}
	return 0;
}

/* Update both half flow entries to hardware as part of tx monitoring
 */
int fa2_tbl_update_hw_flow_table(struct fa *fa, struct fa_frame_info *info)
{
	struct fa2_flow_entry *flow;
	struct fa2_flow_entry *peer;
	int rc;
	u32 i;
	u32 key[FA2_NAT_KEY_SIZE];

	flow = fa2_tbl_get_flow(info->flow_id);
	if (!flow)
		return -EINVAL;

	if (!flow->valid) {
		dev_dbg(fa->dev, "flow %d is still owned by rx", info->flow_id);
		goto done;
	}
	if (flow->rev_flow_ptr != UNKNOWN_REV_PTR) {
		dev_warn(fa->dev, "flow %d is no longer owned by tx dir(%d)",
			 info->flow_id, flow->direction);
		goto done;
	}
	if (!flow->ipv4_key_type) {
		dev_warn(fa->dev, "flow(%d) not ipv4\n", info->flow_id);
		goto done;
	}

	key[0] = flow->direction ? info->dip : flow->dip,
	key[1] = flow->direction ? flow->sip : info->sip,
	key[2] = flow->direction ? info->dport : flow->dport,
	key[3] = flow->direction ? flow->sport : info->sport,
	key[4] = flow->protocol;

	dev_dbg(fa->dev, "looking for peer for v4 flow %d", info->flow_id);
	for (i = 0; i < FA2_FLOW_TABLE_SIZE; i++) {
		peer = fa2_tbl_get_flow(i);
		if (!peer)
			return -EINVAL;

		if ((!FLOW_ENTRY_IS_V4(peer)) || (i == info->flow_id))
			continue;

		if (FLOW_ENTRY_IS_TX_OWNED(peer)) {
			if ((peer->sip == key[0]) &&
			    (peer->dip == key[1]) &&
			    (peer->sport == key[2]) &&
			    (peer->dport == key[3]) &&
			    (peer->protocol == key[4])) {
				u32 tmp;

				dev_warn(fa->dev, "flow %d and %d are flow peers",
					 info->flow_id, i);

				flow->rev_flow_ptr = i;
				peer->rev_flow_ptr = info->flow_id;
				tmp = peer->next_hop_idx;
				peer->next_hop_idx = flow->next_hop_idx;
				flow->next_hop_idx = tmp;
				tmp = peer->remap_sa_idx;
				peer->remap_sa_idx = flow->remap_sa_idx;
				flow->remap_sa_idx = tmp;

				rc = fa2_tbl_update_table(fa, FA2_FLOW_TABLE,
							  info->flow_id, flow);
				if (rc)
					return rc;

				rc = fa2_tbl_update_table(fa, FA2_FLOW_TABLE,
							  i, peer);
				if (rc)
					return rc;
				goto done;
			}
		}
	}
done:
	return 0;
}
