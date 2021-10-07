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

#ifndef __LM_HW_PM_H__
#define __LM_HW_PM_H__

#include <linux/types.h>
#include <phymod/phymod.h>
#include <phymod/phymod_symbols.h>
#include "lm_defs.h"
#include "pm_intf.h"
#include "nvm_cfg.h"

#define NUM_LANES   4		/* num of lanes per core */
#define MAX_PHYS    4		/* Eqvivalent to max ports */

#define TSC_CORE_ADDR (5)

#define DbgMessage(a, b, c)
#define DbgMessage1(a, b, c, d)
#define DbgMessage2(a, b, c, d, e)
#define DbgMessage3(a, b, c, d, e, f)
#define DbgMessage4(a, b, c, d, e, f, g)
#define DbgMessage5(a, b, c, d, e, f, g, h)
#define DbgMessage6(a, b, c, d, e, f, g, h, i)
#define DbgMessage7(a, b, c, d, e, f, g, h, i, j)

void phymode_usleep(u32 usecs);
void phymod_sleep(int secs);
void *phymod_malloc(size_t size, char *descr);
void phymod_free(void *buf);
/* Function prototypes */
lm_status_t lm_hw_pm_get_eye_scan(pm_device_t pdev, u8 *outstr_p);

typedef struct pmm_core {
  int init;
  phymod_core_access_t pm_core;
  phymod_core_init_config_t init_config;
} pmm_core_t;

typedef struct pmm_phymod_phy {
	u32 lane_map;		/* Which lanes produce the port */
	u32 num_lanes;
	u32 max_speed;
   phymod_interface_t interface_type;
   pmm_core_t *core;
   phymod_phy_access_t pm_phy;
   phymod_phy_init_config_t init_config;
} pmm_phymod_phy_t;

typedef struct pmm_phy_ctrl {
  pmm_phymod_phy_t int_phy;
} pmm_phy_ctrl_t;

typedef struct pmm_config {
	u8 num_ports;
	u32 core_config;
   pmm_core_t pmm_core;
   pmm_phy_ctrl_t phy[MAX_PHYS];
} pmm_config_t;

#define GET_PMM_CFG(pdev) (&device_pmm_cfg)
#define GET_PMM_BUS_BLK(pdev) (&device_pmm_bus)

#endif /*  __LM_HW_PM_H__ */
