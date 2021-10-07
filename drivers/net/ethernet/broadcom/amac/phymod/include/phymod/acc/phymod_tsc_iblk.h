/*
 * Copyright © 2016 Broadcom.
 * The term “Broadcom” refers to Broadcom Limited and/or its
 * subsidiaries.
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


#ifndef __PHYMOD_TSC_IBLK_H__
#define __PHYMOD_TSC_IBLK_H__

#include <phymod/phymod_acc.h>
#include <phymod/phymod_reg.h>

/* Special lane values for broadcast and dual-lane multicast */
#define PHYMOD_TSC_IBLK_MCAST01    4
#define PHYMOD_TSC_IBLK_MCAST23    5
#define PHYMOD_TSC_IBLK_BCAST      6

extern int
phymod_tsc_iblk_read(const phymod_access_t *pa, uint32_t addr, uint32_t *data);

extern int
phymod_tsc_iblk_write(const phymod_access_t *pa, uint32_t addr, uint32_t data);

#endif /* __PHYMOD_TSC_IBLK_H__ */
