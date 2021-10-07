/*
 *         
 * $Id: phymod_definitions.h,v 1.2.2.12 2013/09/03 06:54:51 dayad Exp $
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


#ifndef _PHYMOD_UTIL_H_
#define _PHYMOD_UTIL_H_

#include <phymod/phymod.h>


/******************************************************************************
Functions
******************************************************************************/


int phymod_util_lane_config_get(const phymod_access_t *phys, int *start_lane, int *num_of_lane);

#endif /*_PHYMOD_UTIL_H_*/
