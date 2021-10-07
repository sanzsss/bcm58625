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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/if_vlan.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/bitops.h>

#include "fa.h"
#include "fa2.h"
#include "fa2_debug.h"

char fa_driver_name[] = "bcm-fa";
char fa_driver_string[] =
		"Broadcom Flow Accelerator Driver";

#define DRV_VERSION "0.1"

const char fa_driver_version[] = DRV_VERSION;

static const struct of_device_id fa_of_match_table[] = {
	{
		.compatible = "brcm,fa2",
		.data = (int *)FA_TYPE_2,
	},
	{/* sentinel */},
};

#ifdef FA_DEBUG
/* dump frame info structure
 */
static void fa_dump_frame_info(struct fa_frame_info *info)
{
	u8 ip_str[256];

	if (info->sip) {
		u8 *sip = (u8 *)&info->sip;
		u8 *dip = (u8 *)&info->dip;

		sprintf(ip_str, "sip = %d.%d.%d.%d, dip = %d.%d.%d.%d", sip[3],
			sip[2], sip[1], sip[0], dip[3], dip[2], dip[1], dip[0]);
	} else {
		ip_str[0] = 0;
	}
	pr_info("  flow_id: 0x%03x src_pid: 0x%01x\n"
		"  da: %02x:%02x:%02x:%02x:%02x:%02x\n"
		"  sa: %02x:%02x:%02x:%02x:%02x:%02x\n"
		"  vlan: %d\n"
		"  et: 0x%04x\n"
		"  l3_proto: 0x%04x, %s\n"
		"  l4_proto: %d, offset: %d, sport: %d, dport: %d\n",
		info->flow_id, info->src_pid,
		info->da[0], info->da[1], info->da[2], info->da[3],
		info->da[4], info->da[5], info->sa[0], info->sa[1],
		info->sa[2], info->sa[3], info->sa[4], info->sa[5],
		info->vlan ? ntohs(info->vlan) : 0,
		info->ethertype,
		info->l3_proto, ip_str, info->l4_proto, info->l4_offset,
		info->sport, info->dport);
}
#endif

static void fa_parse_skb(struct sk_buff *skb, struct fa_frame_info *info)
{
	struct ethhdr *eth;
	struct vlan_hdr *vhdr;

	memset(info, 0, sizeof(struct fa_frame_info));

	info->nfct = skb->nfct;
	info->flow_id = (skb->flow_info & DSA_TAG_BRCM_FLOW_MASK);
	info->src_pid = (skb->flow_info >> DSA_TAG_BRCM_SRC_SHIFT)
			& DSA_TAG_BRCM_SRC_MASK;

	/* save MAC pointers */
	eth = (struct ethhdr *)skb_mac_header(skb);
	info->data = (u8 *)eth;
	info->da = eth->h_dest;
	info->data += sizeof(eth->h_dest);
	info->sa = eth->h_source;
	info->data += sizeof(eth->h_source);
	info->ethertype = ntohs(*(u16 *)info->data);
	info->data += sizeof(eth->h_proto);

	/* Save VLAN info */
	if (skb_vlan_tag_present(skb)) {
		vhdr = (struct vlan_hdr *)info->data;
		info->vlan = ntohs((u16)skb_vlan_tag_get_id(skb));
		info->ethertype = ntohs((u16)vlan_get_protocol(skb));
		info->data += sizeof(struct vlan_hdr);
	}

	info->l3_proto = info->ethertype;

	info->l4_offset = (u32)skb_transport_header(skb) -
			  (u32)skb_network_header(skb);

	if (info->l3_proto == ETH_P_IP) {
		struct iphdr *iph = ip_hdr(skb);

		info->sip = ntohl(iph->saddr);
		info->dip = ntohl(iph->daddr);
		info->l4_proto = iph->protocol;

	} else {
		/* since l4_proto is not set, no action below
		 */
	}

	if (info->l4_proto == IPPROTO_TCP) {
		struct tcphdr *th = (struct tcphdr *)skb_transport_header(skb);
		info->sport = ntohs(th->source);
		info->dport = ntohs(th->dest);
	} else if (info->l4_proto == IPPROTO_UDP) {
		struct udphdr *uh = (struct udphdr *)skb_transport_header(skb);
		info->sport = ntohs(uh->source);
		info->dport = ntohs(uh->dest);
	}

#ifdef FA_DEBUG
	fa_dump_frame_info(info);
#endif
}

static u32 fa_read(struct fa *fa, u16 offset)
{
	u32 value;

	value = readl(fa->base + offset);

	dev_dbg(fa->dev, "addr 0x%08x", (u32)(fa->base + offset));
	dev_dbg(fa->dev, "rd   0x%08x\n", value);

	return value;
}

static void fa_write(struct fa *fa, u16 offset, u32 value)
{
	writel(value, fa->base + offset);

	dev_dbg(fa->dev, "addr 0x%08x\n", (u32)(fa->base + offset));
	dev_dbg(fa->dev, "wr   0x%08x\n", value);
}

static bool fa_wait_value(struct fa *fa, u16 offset, u32 mask,
			  u32 value, int mintime, int timeout)
{
	u32 val;
	int i;

	for (i = 0; i < timeout / 10; i++) {
		val = fa->read(fa, offset);
		if ((val & mask) == value)
			return true;
		usleep_range(mintime, timeout / 10);
	}
	dev_err(fa->dev, "Timeout waiting for reg 0x%X\n", offset);
	return false;
}

static int fa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fa *fa;
	struct resource regs;
	struct device_node *np;
	const struct of_device_id *of_id;
	int rc;

	of_id = of_match_device(fa_of_match_table, dev);
	if (!of_id)
		return -EINVAL;

	dev_info(&pdev->dev, "%s - version %s\n", fa_driver_string,
		 fa_driver_version);

	fa = devm_kzalloc(&pdev->dev, sizeof(*fa), GFP_KERNEL);
	if (!fa)
		return -ENOMEM;

	platform_set_drvdata(pdev, fa);

	fa->dev = dev;
	fa->type = (enum fa_type)of_id->data;

	np = fa->dev->of_node;

	rc = of_address_to_resource(np, 0, &regs);
	if (rc < 0) {
		dev_err(dev, "Unable to obtain fa resources\n");
		return rc;
	}

	fa->base = devm_ioremap(dev, regs.start, resource_size(&regs));
	if (IS_ERR(fa->base)) {
		dev_err(dev, "Unable to map fa registers\n");
		return PTR_ERR(fa->base);
	}

	/* Set up common functions
	 */
	fa->read = fa_read;
	fa->write = fa_write;
	fa->wait_value = fa_wait_value;
	fa->parse_skb = fa_parse_skb;

	/* Set up tcp/udp port bypass list for common
	 * FTP/DNS/BOOTP/TFTP ports which have IP
	 * addresses in their payload so do not work
	 * well with NAT.
	 */
	set_bit(21, fa->port_bypass);
	set_bit(53, fa->port_bypass);
	set_bit(67, fa->port_bypass);
	set_bit(68, fa->port_bypass);
	set_bit(69, fa->port_bypass);

	if (fa->type == FA_TYPE_2) {
		fa->type = FA_TYPE_2;
		fa->write_table = fa2_write_table;
		fa->read_table = fa2_read_table;
		fa2_dbg_init(fa);
		rc = fa2_init(fa);
	} else {
		dev_err(&pdev->dev, "Currently only fa2 supported\n");
		return -EINVAL;
	}

	return rc;
}

static int fa_remove(struct platform_device *pdev)
{
	struct fa *fa = platform_get_drvdata(pdev);
	struct device_node *np;

	dev_info(fa->dev, "Remove fa driver");

	np = fa->dev->of_node;

	if (of_device_is_compatible(np, "brcm,fa2")) {
		fa2_dbg_exit(fa);

		fa2_exit(fa);
	} else {
		dev_err(&pdev->dev, "Currently only fa2 supported\n");
		return -EINVAL;
	}
	return 0;
}

MODULE_DEVICE_TABLE(of, fa_of_match_table);

static struct platform_driver fa_driver = {
	.probe = fa_probe,
	.remove = fa_remove,
	.driver = {
		   .name = fa_driver_name,
		   .of_match_table = of_match_ptr(fa_of_match_table),
		   },
};

module_platform_driver(fa_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION(fa_driver_string);
MODULE_LICENSE("GPL");
