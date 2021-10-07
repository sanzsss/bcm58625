/*
 * Copyright (C) 2016 Broadcom Corporation
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

#ifndef __BCM_AMAC_RING_OFFLOAD_H__
#define __BCM_AMAC_RING_OFFLOAD_H__

#define GIORINGOFFLOAD	(SIOCDEVPRIVATE + 1)
#define SIORINGOFFLOAD	(SIOCDEVPRIVATE + 2)

#define BCM_AMAC_RING_OFFLOAD_VERSION 1

#define BCM_AMAC_RING_OFFLOAD_GET_OP_RING_COUNT 1
#define BCM_AMAC_RING_OFFLOAD_GET_OP_RING_INFO  2

#define BCM_AMAC_RING_OFFLOAD_SET_OP_STOP          1
#define BCM_AMAC_RING_OFFLOAD_SET_OP_START         2
#define BCM_AMAC_RING_OFFLOAD_SET_OP_RECV_MASK_ON  3
#define BCM_AMAC_RING_OFFLOAD_SET_OP_RECV_MASK_OFF 4

enum {
	RING_TYPE_RX	= 0,
	RING_TYPE_TX	= 1,
};

struct ring_info {
	u32 size;		/* number of BDs in the ring */
	u16 nr_pages;		/* number of kernel pages */
	u16 type;		/* ring type: rx, tx */
	u32 prod_offset;	/* Producer register offset */
	u32 cons_offset;	/* Consumer register offset */
	u64 mapping;		/* Physical start addr of the
				 * descriptor ring
				 */
};

struct bcm_amac_ring_offload_get {
	u32			version;
	u32			get_op;
	u32			port;

	u64			register_base;  /* register base for the port */
	u32			ring_count;
	struct ring_info	rings[0];
};

struct bcm_amac_ring_offload_set {
	u32			version;
	u32			set_op;
	u32			port;
};

#endif /* __BCM_AMAC_RING_OFFLOAD_H__ */
