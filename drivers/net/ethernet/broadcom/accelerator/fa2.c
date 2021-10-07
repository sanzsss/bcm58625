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
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/dsa.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>

#include "fa.h"
#include "fa2.h"
#include "fa2_tables.h"
#include "fa2_debug.h"

/* Number of entries per table type */
static u32 fa2_table_size[] = {
	[FA2_FLOW_TABLE] = FA2_FLOW_TABLE_SIZE,
	[FA2_MAC_TABLE] = FA2_MAC_TABLE_SIZE,
	[FA2_NEXT_HOP_TABLE] = FA2_NEXT_HOP_TABLE_SIZE,
	[FA2_MTU_TABLE] = FA2_MTU_TABLE_SIZE,
	[FA2_TUNNEL_TABLE] = FA2_TUNNEL_TABLE_SIZE,
	[FA2_PORT_TABLE] = FA2_PORT_TABLE_SIZE,
};

#define FA2_FLOW_ENTRY_SIZE 7

/* Size of entry in bytes per table type */
static u32 fa2_entry_size[] = {
	[FA2_FLOW_TABLE] = FA2_FLOW_ENTRY_SIZE,
	[FA2_MAC_TABLE] = 2,
	[FA2_NEXT_HOP_TABLE] = 3,
	[FA2_MTU_TABLE] = 1,
	[FA2_TUNNEL_TABLE] = 3,
	[FA2_PORT_TABLE] = 1,
};

uint fa2_hook_func(void *priv, struct sk_buff *skb,
		   const struct nf_hook_state *state)
{
	struct fa_frame_info info;
	struct fa *fa = (struct fa *)priv;

	if (!fa->hook_is_enabled)
		goto accept;

	/* Check if TCP/UDP nfct connection established
	 * and flow hash is valid.
	 */
	if ((skb->flow_info != DSA_TAG_BRCM_FLOW_INVALID) && (skb->nfct)) {
		struct nf_conn *ct;
		enum ip_conntrack_info ctinfo;
		int dir;
		u32 old_dip;

		ct = nf_ct_get(skb, &ctinfo);
		if (!ct)
			goto accept;

		fa->parse_skb(skb, &info);

		if ((info.l4_proto != IPPROTO_TCP) &&
		    (info.l4_proto != IPPROTO_UDP))
			goto accept;

		/* Don't set up if IP addresses are in payload
		 * (Protocol issue with NAT translation)
		 */
		if ((test_bit(info.sport, fa->port_bypass)) ||
		    (test_bit(info.dport, fa->port_bypass)))
			goto accept;

		if (!(test_bit(IPS_CONFIRMED_BIT, &ct->status)) ||
		    !(test_bit(IPS_ASSURED_BIT, &ct->status)))
			goto accept;

		dir = CTINFO2DIR(ctinfo);

		if (dir != IP_CT_DIR_REPLY) {
			fa2_tbl_update_sw_flow_table(fa, &info);
			goto accept;
		}

		/* External/reply direction - only set up once
		 * connection is established.  Take dip from ct
		 * instead of packet since packet is mangled in reply
		 * direction, save off old info.  Not sure why
		 * packet is already mangled when we have pre routing
		 * hook though...
		 */
		old_dip = info.dip;
		info.dip = ntohl(ct->tuplehash[dir].tuple.dst.u3.ip);

		/* Update sw cache for external side using dip from ct
		 */
		fa2_tbl_update_sw_flow_table(fa, &info);

		info.dip = old_dip;

		/* Push table updates to hardware if ct established
		 * and we are in the reply direction.
		 */
		 fa2_tbl_update_hw_flow_table(fa, &info);
	}
accept:
	return NF_ACCEPT;
}

int fa2_write_table(struct fa *fa, u32 type, u32 index, u32 *hw_entry)
{
	u32 i;
	u32 table_id = type;
	u32 acc_ctrl = FA2_MEM_SELECT_MODE_4K | FA2_DO_ACCESS |
			(table_id << 12) | index;

	/* check index */
	if (index >= fa2_table_size[type]) {
		dev_err(fa->dev, "index = %d is out of range, [0..%d)",
			index, fa2_table_size[type]);
		return -EINVAL;
	}

	if (hw_entry) {
		for (i = 0; i < fa2_entry_size[type]; ++i)
			fa->write(fa,
				  FA2_MEM_ACC_DATA0_OFF + (4 * i),
				  hw_entry[i]);
	}

	fa->write(fa, FA2_MEM_ACC_CONTROL_OFF, acc_ctrl);
	if (!fa->wait_value(fa, FA2_MEM_ACC_CONTROL_OFF,
			    FA2_MEM_ACC_BUSY,
			    0, 100, 10000)) {
		dev_err(fa->dev, "Timeout waiting for FA2 table write\n");
		return -ETIMEDOUT;
	}

	return 0;
}

int fa2_read_table(struct fa *fa, u32 type, u32 index, u32 *hw_entry)
{
	u32 i;
	u32 table_id = type;
	u32 acc_ctrl = FA2_MEM_SELECT_MODE_4K | FA2_DO_ACCESS |
			(1 << 15) | (table_id << 12) | index;

	dev_dbg(fa->dev, "type = %d, index = %d", type, index);

	/* check index */
	if (index >= fa2_table_size[type]) {
		dev_err(fa->dev, "index = %d is out of range, [0..%d)",
			index, fa2_table_size[type]);
		return -EINVAL;
	}

	fa->write(fa, FA2_MEM_ACC_CONTROL_OFF, acc_ctrl);

	if (!fa->wait_value(fa, FA2_MEM_ACC_CONTROL_OFF,
			    FA2_MEM_ACC_BUSY,
			    0, 100, 10000)) {
		dev_err(fa->dev, "Timeout waiting for FA2 table read\n");
		return -ETIMEDOUT;
	}

	for (i = 0; i < fa2_entry_size[type]; ++i)
		hw_entry[i] = fa->read(fa, FA2_MEM_ACC_DATA0_OFF + (4 * i));

	return 0;
}

static u32 null_flow[FA2_FLOW_ENTRY_SIZE] = {};

static void fa2_age_task(struct work_struct *work)
{
	u32 reg, flow_id;
	struct fa *fa = container_of(work, struct fa, age_task.work);
	struct fa2_flow_entry *flow, *peer;

	if (!fa->aging_is_enabled)
		goto done;

	reg = fa->read(fa, FA2_STATUS_OFF);

	if (!(reg & FA2_STATUS_FLOW_TIMEOUT_FIFO_NONEMPTY))
		goto done;

	fa->write(fa, FA2_STATUS_OFF,
		  FA2_STATUS_FLOW_TIMEOUT_FIFO_NONEMPTY);

	while (1) {
		reg = fa->read(fa, FA2_FLOW_TIMEOUT_CONTROL_OFF);
		if (!(reg & FA2_FLOW_TIMEOUT_CTRL_FIFO_DEPTH))
			break;

		flow_id = (reg & FA2_FLOW_TIMEOUT_CTRL_FLOW_ENTRY_POINTER) >> 5;

		flow = fa2_tbl_get_flow(flow_id);

		if (!flow || (!flow->valid) ||
		    (flow->rev_flow_ptr == UNKNOWN_REV_PTR)) {
			dev_err(fa->dev, "Invalid state for flow age flow(%d)\n",
				flow_id);
			continue;
		}

		dev_dbg(fa->dev, "marking sw flow(%d) as aged", flow_id);

		flow->aged = true;
		peer = fa2_tbl_get_flow(flow->rev_flow_ptr);

		if (!peer) {
			dev_err(fa->dev, "peer flow(%d) out of range\n",
				flow->rev_flow_ptr);
			goto done;
		}

		if (peer->aged) {
			dev_dbg(fa->dev, "deleting aged peer flow(%d)",
				flow->rev_flow_ptr);

			fa2_tbl_update_table(fa, FA2_FLOW_TABLE,
					     flow->rev_flow_ptr, null_flow);

			memset(peer, 0, sizeof(struct fa2_flow_entry));

			dev_dbg(fa->dev, "deleting aged flow(%d)", flow_id);

			fa2_tbl_update_table(fa, FA2_FLOW_TABLE,
					     flow_id, null_flow);

			memset(flow, 0, sizeof(struct fa2_flow_entry));
		}
	}
done:
	schedule_delayed_work(&fa->age_task, msecs_to_jiffies(500));
}

int fa2_init(struct fa *fa)
{
	u32 reg;
	int err;
	u32 i;

	/* clear spurious errors */
	reg = fa->read(fa, FA2_SER_EVENT_OFF);
	if (reg)
		fa->write(fa, FA2_SER_EVENT_OFF, reg);

	/* Update ctf_control register */
	reg = fa->read(fa, FA2_CTF_CONTROL_OFF);

	reg &= ~FA2_UNIMAC_MIN_PKT_MASK;
	reg |= FA_PAD_PACKET_TO_MIN_SIZE << FA2_UNIMAC_MIN_PKT_SHIFT;

	reg |= FA2_DISABLE_MAC_DA_CHK | FA2_CTF_MODE | FA2_MEM_INIT
			| FA2_SPU_ENABLE;

	fa->write(fa, FA2_CTF_CONTROL_OFF, reg);

	if (!fa->wait_value(fa, FA2_STATUS_OFF,
			    FA2_STATUS_INIT_DONE,
			    FA2_STATUS_INIT_DONE,
			    100,
			    10000)) {
		dev_err(fa->dev, "Timeout waiting for FA2 memory init done\n");
		return -EIO;
	}
	fa->write(fa, FA2_STATUS_OFF, reg);

	fa->write(fa, FA2_L2_SKIP_CONTROL_OFF, FA2_ETH2_TO_SNAP_CONVERSION);
	fa->write(fa, FA2_HASH_SEED_OFF, FA2_HASH_SEED);
	fa->write(fa, FA2_DRR_CONFIG_OFF,
		  (0x20 << FA2_DRR_CONFIG_MAC_WEIGHT_SHIFT) | 0x30);

	/* Reason code decode is not required.
	 */
	fa->write(fa, FA2_BRCM_HDR_CONTROL_OFF, 0);

	/* All mtu entries set to a default value
	 */
	for (i = 0; i < FA2_MTU_TABLE_SIZE; i++) {
		err = fa2_tbl_update_mtu_table(fa, FA2_MTU_DEFAULT_SIZE, i);
		if (err)
			return err;
	}
	/* Set WAN port as external in port table
	 */
	for (i = 0; i < FA2_PORT_TABLE_SIZE; i++) {
		u8 external = 0;

		if (i == FA2_WAN_PORT)
			external = 1;

		err = fa2_tbl_update_port_table(fa, external, i);
		if (err)
			return err;
	}

	fa->nfho.hook = fa2_hook_func;
	fa->nfho.hooknum = NF_INET_PRE_ROUTING;
	fa->nfho.pf = PF_INET;
	fa->nfho.priority = NF_IP_PRI_FIRST;
	fa->nfho.priv = fa;
	fa->hook_is_enabled = false;

	err = nf_register_hook(&fa->nfho);
	if (err) {
		dev_err(fa->dev, "hook preroute register err(%d)\n",
			err);
		return err;
	}

	reg = fa->read(fa, FA2_FLOW_TIMEOUT_CONTROL_OFF);
	fa->write(fa, FA2_FLOW_TIMEOUT_CONTROL_OFF, reg |
		  FA2_FLOW_TIMEOUT_CTRL_HW_TIMEOUT_MSG_ENABLE);

	/* Set timeout to 2^31 8.59s to 36.65m
	 */
	reg = ((1 << FA2_FLOW_TIMER_CONFIG0_UDP_EST_SHIFT) |
	       (1 << FA2_FLOW_TIMER_CONFIG0_TCP_EST_SHIFT) |
		1);
	fa->write(fa, FA2_FLOW_TIMER_CONFIG0_OFF, reg);

	/* Set initial timeout values
	 */
	reg = ((21 << FA2_FLOW_TIMER_CONFIG1_UDP_EST_SHIFT) |
	       (28 << FA2_FLOW_TIMER_CONFIG1_TCP_EST_SHIFT) |
		4);
	fa->write(fa, FA2_FLOW_TIMER_CONFIG1_OFF, reg);

	fa->aging_is_enabled = true;
	INIT_DELAYED_WORK(&fa->age_task, fa2_age_task);
	schedule_delayed_work(&fa->age_task, 10 * HZ);

	return 0;
}

void fa2_exit(struct fa *fa)
{
	nf_unregister_hook(&fa->nfho);
	cancel_delayed_work_sync(&fa->age_task);
}
