/*
 *         
 * $Id: phymod.xml,v 1.1.2.5 2013/09/12 10:43:06 nirf Exp $
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


#ifndef _PHY8806X_DIAGNOSTICS_H__H_
#define _PHY8806X_DIAGNOSTICS_H__H_

#include <phymod/phymod_definitions.h>
/*Set\get slicer position*/
int phy8806x_phy_rx_slicer_position_set(const phymod_phy_access_t* phy, uint32_t flags, const phymod_slicer_position_t* position);
int phy8806x_phy_rx_slicer_position_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_slicer_position_t* position);

/*Get slicer range limitation*/
int phy8806x_phy_rx_slicer_position_max_get(const phymod_phy_access_t* phy, uint32_t flags, const phymod_slicer_position_t* position_min, const phymod_slicer_position_t* position_max);

/*set\get PRBS configuration*/
int phy8806x_phy_prbs_config_set(const phymod_phy_access_t* phy, uint32_t flags , const phymod_prbs_t* prbs);
int phy8806x_phy_prbs_config_get(const phymod_phy_access_t* phy, uint32_t flags , phymod_prbs_t* prbs);

/*Set\get PRBS enable state*/
int phy8806x_phy_prbs_enable_set(const phymod_phy_access_t* phy, uint32_t flags , uint32_t enable);
int phy8806x_phy_prbs_enable_get(const phymod_phy_access_t* phy, uint32_t flags , uint32_t* enable);

/*Get PRBS Status*/
int phy8806x_phy_prbs_status_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_prbs_status_t* prbs_status);

/*Set\get pattern state*/
int phy8806x_phy_pattern_config_set(const phymod_phy_access_t* phy, const phymod_pattern_t* pattern);
int phy8806x_phy_pattern_config_get(const phymod_phy_access_t* phy, phymod_pattern_t* pattern);

/*Set\get pattern state*/
int phy8806x_phy_pattern_enable_set(const phymod_phy_access_t* phy, uint32_t enable, const phymod_pattern_t* pattern);
int phy8806x_phy_pattern_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*Get core diagnostics information*/
int phy8806x_core_diagnostics_get(const phymod_core_access_t* core, phymod_core_diagnostics_t* diag);

/*Get phy diagnostics information*/
int phy8806x_phy_diagnostics_get(const phymod_phy_access_t* phy, phymod_phy_diagnostics_t* diag);

/*Get phy diagnostics information*/
int phy8806x_phy_pmd_info_dump(const phymod_phy_access_t* phy, char* type);

/*Display eyescan information*/
int phy8806x_phy_eyescan_run(const phymod_phy_access_t* phy, uint32_t flags, phymod_eyescan_mode_t mode, const phymod_phy_eyescan_options_t* eyescan_options);

#endif /*_PHY8806X_DIAGNOSTICS_H_*/
