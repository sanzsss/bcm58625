/********************************************************************************
********************************************************************************
*                                                                              *
*  Revision      :  $Id: eagle_tsc_common.h 1140 2015-09-18 21:05:29Z kirand $ *
*                                                                              *
*  Description   :  Defines and Enumerations required by Serdes APIs           *
*                                                                              *
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


/** @file eagle_tsc_common.h
 * Defines and Enumerations shared across Eagle/Merlin/Falcon APIs BUT NOT MICROCODE
 */

#ifndef EAGLE_TSC_API_COMMON_H
#define EAGLE_TSC_API_COMMON_H
#include "common/srds_api_uc_common.h"

/** Macro to determine sign of a value */
#define sign(x) ((x >= 0) ? 1 : -1)

#define UCODE_MAX_SIZE  32768

/*
 * Register Address Defines used by the API that are different between IPs
 */
#define DSC_A_DSC_UC_CTRL 0xD00D
#define TLB_RX_PRBS_CHK_ERR_CNT_MSB_STATUS 0xD0DA

/* PLL Lock and change Status Register define */
#define PLL_STATUS_ADDR 0xD128

/* PMD Lock and change Status Register define */
#define PMD_LOCK_STATUS_ADDR 0xD0DC

/* Sigdet and change Status Register define */
#define SIGDET_STATUS_ADDR 0xD0C8

#define MDIO_MMDSEL_AER_COM_MDIO_MASKDATA  0xFFDB

/*
 * Register Address Defines used by the API that are COMMON across IPs
 */


/*
 * IP-Specific Iteration Bounds
 */

#endif
