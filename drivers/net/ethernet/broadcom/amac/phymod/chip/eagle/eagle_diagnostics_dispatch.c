/*
 *         
 * $Id: phymod.xml,v 1.1.2.5 2013/09/12 10:43:06 nirf Exp $
 * 
 *
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


#include <phymod/phymod.h>
#include <phymod/phymod_diagnostics.h>
#include <phymod/phymod_diagnostics_dispatch.h>

#ifdef PHYMOD_EAGLE_SUPPORT

#include <phymod/chip/eagle_diagnostics.h>


__phymod_diagnostics__dispatch__t__ phymod_diagnostics_eagle_diagnostics_driver = {

    eagle_phy_rx_slicer_position_set,
    eagle_phy_rx_slicer_position_get,
    eagle_phy_rx_slicer_position_max_get,
    eagle_phy_prbs_config_set,
    eagle_phy_prbs_config_get,
    eagle_phy_prbs_enable_set,
    eagle_phy_prbs_enable_get,
    eagle_phy_prbs_status_get,
    eagle_phy_pattern_config_set,
    eagle_phy_pattern_config_get,
    eagle_phy_pattern_enable_set,
    eagle_phy_pattern_enable_get,
    eagle_core_diagnostics_get,
    eagle_phy_diagnostics_get,
    eagle_phy_pmd_info_dump,
    NULL, /* phymod_phy_pcs_info_dump */
    eagle_phy_eyescan_run,
    NULL, /* phymod_phy_link_mon_enable_set */
    NULL, /* phymod_phy_link_mon_enable_get */
    NULL, /* phymod_phy_link_mon_status_get */
};

#endif /* PHYMOD_EAGLE_SUPPORT */
