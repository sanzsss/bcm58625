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


#ifndef _DINO_DIAGNOSTICS_H__H_
#define _DINO_DIAGNOSTICS_H__H_

#include <phymod/phymod_definitions.h>
/*set\get PRBS configuration*/
int dino_phy_prbs_config_set(const phymod_phy_access_t* phy, uint32_t flags , const phymod_prbs_t* prbs);
int dino_phy_prbs_config_get(const phymod_phy_access_t* phy, uint32_t flags , phymod_prbs_t* prbs);

/*Set\get PRBS enable state*/
int dino_phy_prbs_enable_set(const phymod_phy_access_t* phy, uint32_t flags , uint32_t enable);
int dino_phy_prbs_enable_get(const phymod_phy_access_t* phy, uint32_t flags , uint32_t* enable);

/*Get PRBS Status*/
int dino_phy_prbs_status_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_prbs_status_t* prbs_status);

/*Get phy diagnostics information*/
int dino_phy_diagnostics_get(const phymod_phy_access_t* phy, phymod_phy_diagnostics_t* diag);

/*Display eyescan information*/
int dino_phy_eyescan_run(const phymod_phy_access_t* phy, uint32_t flags, phymod_eyescan_mode_t mode, const phymod_phy_eyescan_options_t* eyescan_options);

#endif /*_DINO_DIAGNOSTICS_H_*/
