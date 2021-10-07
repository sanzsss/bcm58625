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


#ifndef _EAGLE_COMMON_H__H_
#define _EAGLE_COMMON_H__H_

#include <phymod/phymod.h>

/* Translate eagle osr_mode register value to phymod enum*/
int eagle_osr_mode_to_enum(int osr_mode, phymod_osr_mode_t* osr_mode_en);

#endif /*_EAGLE_COMMON_H__H_*/
