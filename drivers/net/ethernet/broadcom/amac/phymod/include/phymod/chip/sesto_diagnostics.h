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


#ifndef _SESTO_DIAGNOSTICS_H__H_
#define _SESTO_DIAGNOSTICS_H__H_

#include <phymod/phymod_definitions.h>
/*set\get PRBS configuration*/
int sesto_phy_prbs_config_set(const phymod_phy_access_t* phy, uint32_t flags , const phymod_prbs_t* prbs);
int sesto_phy_prbs_config_get(const phymod_phy_access_t* phy, uint32_t flags , phymod_prbs_t* prbs);

/*Set\get PRBS enable state*/
int sesto_phy_prbs_enable_set(const phymod_phy_access_t* phy, uint32_t flags , uint32_t enable);
int sesto_phy_prbs_enable_get(const phymod_phy_access_t* phy, uint32_t flags , uint32_t* enable);

/*Get PRBS Status*/
int sesto_phy_prbs_status_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_prbs_status_t* prbs_status);

/*Get core diagnostics information*/
int sesto_core_diagnostics_get(const phymod_core_access_t* core, phymod_core_diagnostics_t* diag);

/*Get phy diagnostics information*/
int sesto_phy_diagnostics_get(const phymod_phy_access_t* phy, phymod_phy_diagnostics_t* diag);

/*Get phy diagnostics information*/
int sesto_phy_pmd_info_dump(const phymod_phy_access_t* phy, char* type);

/*Display eyescan information*/
int sesto_phy_eyescan_run(const phymod_phy_access_t* phy, uint32_t flags, phymod_eyescan_mode_t mode, const phymod_phy_eyescan_options_t* eyescan_options);

/*Get/Set link monitor enable/disable*/
int sesto_phy_link_mon_enable_set(const phymod_phy_access_t* phy, phymod_link_monitor_mode_t link_mon_mode, uint32_t enable);
int sesto_phy_link_mon_enable_get(const phymod_phy_access_t* phy, phymod_link_monitor_mode_t link_mon_mode, uint32_t* enable);

/*Get link monitor Status*/
int sesto_phy_link_mon_status_get(const phymod_phy_access_t* phy, uint32_t* lock_status, uint32_t* lock_lost_lh, uint32_t* error_count);

#endif /*_SESTO_DIAGNOSTICS_H_*/