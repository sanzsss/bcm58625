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
 
#include <phymod/phymod_util.h>

int phymod_util_lane_config_get(const phymod_access_t *phys, int *start_lane, int *num_of_lane)
{
    switch (phys->lane_mask) {
        case 0x1:
            *start_lane = 0;
            *num_of_lane = 1;
            break;
        case 0x2:
            *start_lane = 1;
            *num_of_lane = 1;
            break;
        case 0x4:
            *start_lane = 2;
            *num_of_lane = 1;
            break;
        case 0x8:
            *start_lane = 3;
            *num_of_lane = 1;
            break;
        case 0x3:
            *start_lane = 0;
            *num_of_lane = 2;
            break;
        case 0xc:
            *start_lane = 2;
            *num_of_lane = 2;
            break;
        case 0x7:
            *start_lane = 0;
            *num_of_lane = 3;
            break;
        case 0xf:
            *start_lane = 0;
            *num_of_lane = 4;
            break;
        case 0x10:
            *start_lane = 4;
            *num_of_lane = 1;
            break;
        case 0x20:
            *start_lane = 5;
            *num_of_lane = 1;
            break;
        case 0x40:
            *start_lane = 6;
            *num_of_lane = 1;
            break;
        case 0x80:
            *start_lane = 7;
            *num_of_lane = 1;
            break;
        case 0x100:
            *start_lane = 8;
            *num_of_lane = 1;
            break;
        case 0x200:
            *start_lane = 9;
            *num_of_lane = 1;
            break;
        case 0x400:
            *start_lane = 10;
            *num_of_lane = 1;
            break;
        case 0x800:
            *start_lane = 11;
            *num_of_lane = 1;
            break;
        case 0x1000:
            *start_lane = 12;
            *num_of_lane = 1;
            break;
        case 0x2000:
            *start_lane = 13;
            *num_of_lane = 1;
            break;
        case 0x4000:
            *start_lane = 14;
            *num_of_lane = 1;
            break;
        case 0x8000:
            *start_lane = 15;
            *num_of_lane = 1;
            break;
        default:
            return PHYMOD_E_INTERNAL;

    }
    return PHYMOD_E_NONE;
}

