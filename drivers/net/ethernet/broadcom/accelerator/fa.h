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
#ifndef _FA_H
#define _FA_H

#include <linux/if_ether.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/workqueue.h>

enum fa_type {
	FA_TYPE_1 = 0,	/* Northstar */
	FA_TYPE_2,	/* Northstar+ */
};

struct fa_frame_info {
	struct nf_conntrack *nfct;
	u32 l4_offset;
	u16 hdr_type;
	u16 hdr_chk;
	u16 flow_id;
	u8 all_buckets_full;
	u16 src_pid;
	u16 proc_op;
	u8 *da;
	u8 *sa;
	u16 ethertype;
	u16 vlan;
	u16 l3_proto;
	u32 sip;
	u32 dip;
	u16 l4_proto;
	u16 sport;
	u16 dport;
	u8 *data;
};

#define FA_PORT_TABLE_SIZE 2048

struct fa {
	struct device *dev;
	enum fa_type type;
	void *base;
	int irq;
	u32 int_mask;
	struct nf_hook_ops nfho;
	bool hook_is_enabled;
	bool aging_is_enabled;
	struct delayed_work age_task;

	unsigned int long port_bypass[FA_PORT_TABLE_SIZE];
	bool (*wait_value)(struct fa *fa, u16 offset, u32 mask,
			   u32 value, int mintime, int timeout);
	u32 (*read)(struct fa *fa, u16 offset);
	void (*write)(struct fa *fa, u16 offset, u32 value);
	int (*write_table)(struct fa *fa, u32 type, u32 index, u32 *hw_entry);
	int (*read_table)(struct fa *fa, u32 type, u32 index, u32 *hw_entry);
	void (*parse_skb)(struct sk_buff *skb, struct fa_frame_info *info);
};

#define FA_PAD_PACKET_TO_MIN_SIZE (ETH_ZLEN + ETH_FCS_LEN + DSA_TAG_BRCM_LEN)

extern char fa_driver_name[];

#endif /* _FA_H */
