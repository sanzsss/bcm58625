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
 *
 *
 * This function accepts a 32-bit PHY register address and will
 * properly configure clause 45 DEVAD and XAUI lane access.
 * Please see phymod_reg.h for additional information.
 *
 */

#include <phymod/acc/phymod_tsc_iblk.h>
#include <phymod/phymod_debug.h>

/*
 * Some bus drivers can handle both clause 22 and clause 45 access, so
 * we use this bit to request clause 45 access.
 */
#define FORCE_CL45      0x20

int
phymod_tsc_iblk_read(const phymod_access_t *pa, uint32_t addr, uint32_t *data)
{
    int ioerr = 0;
    uint32_t devad = (addr >> 16) & 0xf;
    uint32_t blkaddr, regaddr;
    uint32_t lane_map, lane;
    uint32_t aer, add;
    phymod_bus_t* bus;

    add = addr ;

    if (pa == NULL) {
        PHYMOD_VDBG(DBG_ACC,pa,("iblk_rd add=%x pa=null\n", add));
        return -1;
    }

    bus = PHYMOD_ACC_BUS(pa);
    
    /* Do not attempt to read write-only registers */
    if (addr & PHYMOD_REG_ACC_TSC_IBLK_WR_ONLY) {
        *data = 0;
        PHYMOD_VDBG(DBG_ACC,pa,("iblk_rd add=%x WO=1\n", add));
        return 0;
    }

    /* Determine which lane to read from */
    lane = 0;
    if (addr & PHYMOD_REG_ACC_AER_IBLK_FORCE_LANE) {
        /* Forcing lane overrides default behavior */
        lane = (addr >> PHYMOD_REG_ACCESS_FLAGS_SHIFT) & 0x7;
    } else {
        /* Use first lane in lane map by default */
        lane_map = PHYMOD_ACC_LANE_MASK(pa);
        if (lane_map & 0x1) {
            lane = 0;
        } else if (lane_map & 0x2) {
            lane = 1;
        } else if (lane_map & 0x4) {
            lane = 2;
        } else if (lane_map & 0x8) {
            lane = 3;
        } else if (lane_map & 0xfff0) {
            lane = -1;
            while (lane_map) {
                lane++;
                lane_map >>= 1;
            }
        }
    }    

    /* Use default DEVAD if none specified */
    if(((pa->devad & PHYMOD_ACC_DEVAD_0_OVERRIDE_MASK) != 0) && (devad == 0)) {
        devad = (pa->devad & PHYMOD_ACC_DEVAD_MASK);
    /* Force DEVAD if want special address */
    } else if(pa->devad & PHYMOD_ACC_DEVAD_FORCE_MASK) {
        devad = (pa->devad & PHYMOD_ACC_DEVAD_MASK);
    }

    /* Encode address extension */
    aer = lane | (devad << 11);

    /* Mask raw register value */
    addr &= 0xffff;

    /* If bus driver supports lane control, then we are done */
    if (PHYMOD_BUS_CAP_LANE_CTRL_GET(bus)) {
        ioerr += PHYMOD_BUS_READ(pa, addr | (aer << 16), data);
        PHYMOD_VDBG(DBG_ACC,pa,("iblk_rd sbus add=%x aer=%x adr=%x lm=%0x rtn=%0d d=%x\n", 
                                add, aer, addr, pa->lane_mask, ioerr, *data));
        return ioerr;
    }

    /* Use clause 45 access if supported */
    if (PHYMOD_ACC_F_CLAUSE45_GET(pa)) {
        devad |= FORCE_CL45;
        ioerr += PHYMOD_BUS_WRITE(pa, 0xffde | (devad << 16), aer);
        ioerr += PHYMOD_BUS_READ(pa, addr | (devad << 16), data);
        PHYMOD_VDBG(DBG_ACC,pa,("iblk_rd cl45 add=%x dev=%x aer=%x adr=%x lm=%0x rtn=%0d d=%x\n", 
                                add, devad, aer, addr, pa->lane_mask, ioerr, *data));
        return ioerr;
    }

    /* Write address extension register */
    ioerr += PHYMOD_BUS_WRITE(pa, 0x1f, 0xffd0);
    ioerr += PHYMOD_BUS_WRITE(pa, 0x1e, aer);

    /* Select block */
    blkaddr = addr & 0xfff0;
    ioerr += PHYMOD_BUS_WRITE(pa, 0x1f, blkaddr);

    /* Read register value */
    regaddr = addr & 0xf;
    if (addr & 0x8000) {
        regaddr |= 0x10;
    }
    ioerr += PHYMOD_BUS_READ(pa, regaddr, data);
    PHYMOD_VDBG(DBG_ACC,pa,("iblk_rd cl22 add=%x aer=%x blk=%x adr=%x reg=%x lm=%0d rtn=%0d d=%x\n", 
                            add, aer, blkaddr, addr, regaddr, pa->lane_mask, ioerr, *data));
    return ioerr;
}

int
phymod_tsc_iblk_write(const phymod_access_t *pa, uint32_t addr, uint32_t data)
{
    int ioerr = 0;
    uint32_t devad = (addr >> 16) & 0xf;
    uint32_t blkaddr, regaddr;
    uint32_t lane_map, lane;
    uint32_t aer, add;
    uint32_t wr_mask, rdata;
    phymod_bus_t* bus;
    uint32_t is_write_disabled;
    
    add = addr ;

    if (pa == NULL) {
        PHYMOD_VDBG(DBG_ACC,pa,("iblk_wr add=%x pa=null\n", addr));
        return -1;
    }

    bus = PHYMOD_ACC_BUS(pa);

    if (PHYMOD_IS_WRITE_DISABLED(pa, &is_write_disabled) ==  0) {
        if (is_write_disabled) {
            return ioerr;
        }
    }

    lane = 0;
    if (addr & PHYMOD_REG_ACC_AER_IBLK_FORCE_LANE) {
        /* Forcing lane overrides default behavior */
        lane = (addr >> PHYMOD_REG_ACCESS_FLAGS_SHIFT) & 0x7;
    } else {
        /* Write to all lanes by default */
        lane_map = PHYMOD_ACC_LANE_MASK(pa);
        if (lane_map == 0xf) {
            lane = PHYMOD_TSC_IBLK_BCAST;
        } else if (lane_map == 0x3) {
            lane = PHYMOD_TSC_IBLK_MCAST01;
        } else if (lane_map == 0xc) {
            lane = PHYMOD_TSC_IBLK_MCAST23;
        } else if (lane_map & 0xffff) {
            lane = -1;
            while (lane_map) {
                lane++;
                lane_map >>= 1;
            }
        }
    }

    /* Use default DEVAD if none specified */
    if(((pa->devad & PHYMOD_ACC_DEVAD_0_OVERRIDE_MASK) != 0) && (devad == 0)) {
        devad = (pa->devad & PHYMOD_ACC_DEVAD_MASK);
    /* Force DEVAD if want special address */
    } else if(pa->devad & PHYMOD_ACC_DEVAD_FORCE_MASK) {
        devad = (pa->devad & PHYMOD_ACC_DEVAD_MASK);
    }

    /* Check if write mask is specified */
    wr_mask = (data >> 16);
    if (wr_mask) {
        /* Read register if bus driver does not support write mask */
        if (PHYMOD_BUS_CAP_WR_MODIFY_GET(bus) == 0) {
            ioerr += phymod_tsc_iblk_read(pa, addr, &rdata);
            data = (rdata & ~wr_mask) | (data & wr_mask);
            data &= 0xffff;
        }
    }

    /* Encode address extension */
    aer = lane | (devad << 11);

    /* Mask raw register value */
    addr &= 0xffff;

    /* If bus driver supports lane control, then we are done */
    if (PHYMOD_BUS_CAP_LANE_CTRL_GET(bus)) {
        ioerr += PHYMOD_BUS_WRITE(pa, addr | (aer << 16), data);
        PHYMOD_VDBG(DBG_ACC,pa,("iblk_wr sbus add=%x aer=%x adr=%x lm=%0x rtn=%0d d=%x\n", 
                                add, aer, addr, pa->lane_mask, ioerr, data));
        return ioerr;
    }

    /* Use clause 45 access if supported */
    if (PHYMOD_ACC_F_CLAUSE45_GET(pa)) {
        addr &= 0xffff;
        devad |= FORCE_CL45;
        ioerr += PHYMOD_BUS_WRITE(pa, 0xffde | (devad << 16), aer);
        ioerr += PHYMOD_BUS_WRITE(pa, addr | (devad << 16), data);
        PHYMOD_VDBG(DBG_ACC,pa,("iblk_wr cl45 add=%x dev=%x aer=%x adr=%x lm=%0x rtn=%0d d=%x\n", 
                                add, devad, aer, addr, pa->lane_mask, ioerr, data));
        return ioerr;
    }

    /* Write address extension register */
    ioerr += PHYMOD_BUS_WRITE(pa, 0x1f, 0xffd0);
    ioerr += PHYMOD_BUS_WRITE(pa, 0x1e, aer);

    /* Select block */
    blkaddr = addr & 0xfff0;
    ioerr += PHYMOD_BUS_WRITE(pa, 0x1f, blkaddr);

    /* Write register value */
    regaddr = addr & 0xf;
    if (addr & 0x8000) {
        regaddr |= 0x10;
    }
    ioerr += PHYMOD_BUS_WRITE(pa, regaddr, data);
    PHYMOD_VDBG(DBG_ACC,pa,("iblk_wr cl22 add=%x aer=%x blk=%x reg=%x adr=%x lm=%0x rtn=%0d d=%x\n", 
                            addr, aer, blkaddr, regaddr, addr, pa->lane_mask, ioerr, data));
    return ioerr;
}
