/*
 * $Id: phymod_acc.c,v 1.1.2.6 2013/07/10 07:18:49 mlarsen Exp $
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


#include <phymod/phymod_acc.h>

int
phymod_acc_check(const phymod_access_t *pa)
{
    if (pa == 0 ||
        PHYMOD_ACC_BUS(pa) == 0 ||
        PHYMOD_ACC_BUS(pa)->read == 0 ||
        PHYMOD_ACC_BUS(pa)->write == 0) {
        return -1;
    }
    return 0;
}

int
phymod_bus_read(const phymod_access_t *pa, uint32_t reg, uint32_t *data)
{
    /* Read raw PHY data */
    return PHYMOD_ACC_BUS(pa)->read(PHYMOD_ACC_USER_ACC(pa),
                                    PHYMOD_ACC_BUS_ADDR(pa),
                                    reg, data);
}

int
phymod_bus_write(const phymod_access_t *pa, uint32_t reg, uint32_t data)
{
    /* Write raw PHY data */
    return PHYMOD_ACC_BUS(pa)->write(PHYMOD_ACC_USER_ACC(pa),
                                     PHYMOD_ACC_BUS_ADDR(pa),
                                     reg, data);
}

int
phymod_is_write_disabled(const phymod_access_t *pa, uint32_t *data)
{
    if((PHYMOD_ACC_BUS(pa)->is_write_disabled) == NULL) { *data = 0; } else {
        return PHYMOD_ACC_BUS(pa)->is_write_disabled(PHYMOD_ACC_USER_ACC(pa),
                                     data);
    }
    return 0;
}
