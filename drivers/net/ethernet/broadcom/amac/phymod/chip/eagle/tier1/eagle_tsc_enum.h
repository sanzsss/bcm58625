/******************************************************************************
******************************************************************************
*  Revision      :  $Id: eagle_tsc_enum.h 1140 2015-09-18 21:05:29Z kirand $ *
*                                                                            *
*  Description   :  Enum types used by Serdes API functions                  *
*                                                                            *
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
 *
******************************************************************************/

/** @file eagle_tsc_enum.h
 * Enum types used by Serdes API functions
 */

#ifndef EAGLE_TSC_API_ENUM_H
#define EAGLE_TSC_API_ENUM_H

#include "common/srds_api_enum.h"



/** Eagle PLL Config Enum */
enum eagle_tsc_pll_enum {
	EAGLE_TSC_pll_div_40x,
	EAGLE_TSC_pll_div_42x,
	EAGLE_TSC_pll_div_46x,
	EAGLE_TSC_pll_div_50x,
	EAGLE_TSC_pll_div_52x,
	EAGLE_TSC_pll_div_60x,
	EAGLE_TSC_pll_div_64x_refc161,
	EAGLE_TSC_pll_div_64x_refc156,
	EAGLE_TSC_pll_div_64x,
	EAGLE_TSC_pll_div_66x,
	EAGLE_TSC_pll_div_68x,
	EAGLE_TSC_pll_div_70x,
	EAGLE_TSC_pll_div_72x,
	EAGLE_TSC_pll_div_73p6x,
	EAGLE_TSC_pll_div_80x_refc125,
	EAGLE_TSC_pll_div_80x_refc106,
	EAGLE_TSC_pll_div_80x,
	EAGLE_TSC_pll_div_80x_refc156,
	EAGLE_TSC_pll_div_82p5x,
	EAGLE_TSC_pll_div_87p5x,
	EAGLE_TSC_pll_div_92x,
	EAGLE_TSC_pll_div_100x,
	EAGLE_TSC_pll_div_199p04x,
	EAGLE_TSC_pll_div_36p8x
};














#endif
