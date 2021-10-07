#ifndef __BCMI_VIPER_XGXS_DEFS_H__
#define __BCMI_VIPER_XGXS_DEFS_H__

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


#ifndef _DV_TB_
#include <phymod/acc/phymod_tsc_iblk.h>
#endif /* _DV_TB_ */

/*******************************************************************************
 *
 *                    CHIP DEFINITIONS BEGIN HERE
 *
 ******************************************************************************/



/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  MIICTL
 * BLOCKS:   CL22_B0
 * REGADDR:  0x0000
 * DESC:     IEEE MII control register
 * RESETVAL: 0x140 (320)
 * ACCESS:   R/W
 * FIELDS:
 *     MANUAL_SPEED1    Bits6 13----1 1 = Reserved1 0 = SGMII 1000 Mb/s0 1 = SGMII 100 Mb/s0 0 = SGMII 10 Mb/sNote: speed1 & speed0 bits for SGMII mode only
 *     COLLISION_TEST_EN 1 = collision test mode enabled0 = collision test mode disabled
 *     FULL_DUPLEX      1 = full duplex0 = half duplex
 *     RESTART_AUTONEG  1 = restart auto-negotiation process0 = normal operation
 *     PWRDWN_SW        1 = low power mode0 = normal operation
 *     AUTONEG_ENABLE   1 = auto-negotiation enabled0 = auto-negotiation disabled
 *     MANUAL_SPEED0    Bits6 13----1 1 = Reserved1 0 = SGMII 1000 Mb/s0 1 = SGMII 100 Mb/s0 0 = SGMII 10 Mb/sNote: speed1 & speed0 bits for SGMII mode only
 *     GLOOPBACK        1 = Global loopback mode is enabled (i.e. TX->RX)0 = normal operation
 *     RST_HW           1 = PHY Reset0 = normal operation
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_MIICTLr (0x00000000 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_MIICTLr_SIZE 4

/*
 * This structure should be used to declare and program MIICTL.
 *
 */
typedef union BCMI_VIPER_XGXS_MIICTLr_s {
	uint32_t v[1];
	uint32_t miictl[1];
	uint32_t _miictl;
} BCMI_VIPER_XGXS_MIICTLr_t;

#define BCMI_VIPER_XGXS_MIICTLr_CLR(r) (r).miictl[0] = 0
#define BCMI_VIPER_XGXS_MIICTLr_SET(r,d) (r).miictl[0] = d
#define BCMI_VIPER_XGXS_MIICTLr_GET(r) (r).miictl[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_MIICTLr_RST_HWf_GET(r) ((((r).miictl[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_MIICTLr_RST_HWf_SET(r,f) (r).miictl[0]=(((r).miictl[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_MIICTLr_GLOOPBACKf_GET(r) ((((r).miictl[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_MIICTLr_GLOOPBACKf_SET(r,f) (r).miictl[0]=(((r).miictl[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_MIICTLr_MANUAL_SPEED0f_GET(r) ((((r).miictl[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_MIICTLr_MANUAL_SPEED0f_SET(r,f) (r).miictl[0]=(((r).miictl[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_MIICTLr_AUTONEG_ENABLEf_GET(r) ((((r).miictl[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_MIICTLr_AUTONEG_ENABLEf_SET(r,f) (r).miictl[0]=(((r).miictl[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_MIICTLr_PWRDWN_SWf_GET(r) ((((r).miictl[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_MIICTLr_PWRDWN_SWf_SET(r,f) (r).miictl[0]=(((r).miictl[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_MIICTLr_RESTART_AUTONEGf_GET(r) ((((r).miictl[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_MIICTLr_RESTART_AUTONEGf_SET(r,f) (r).miictl[0]=(((r).miictl[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_MIICTLr_FULL_DUPLEXf_GET(r) ((((r).miictl[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_MIICTLr_FULL_DUPLEXf_SET(r,f) (r).miictl[0]=(((r).miictl[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_MIICTLr_COLLISION_TEST_ENf_GET(r) ((((r).miictl[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_MIICTLr_COLLISION_TEST_ENf_SET(r,f) (r).miictl[0]=(((r).miictl[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_MIICTLr_MANUAL_SPEED1f_GET(r) ((((r).miictl[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_MIICTLr_MANUAL_SPEED1f_SET(r,f) (r).miictl[0]=(((r).miictl[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))

/*
 * These macros can be used to access MIICTL.
 *
 */
#define BCMI_VIPER_XGXS_READ_MIICTLr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MIICTLr,(_r._miictl))
#define BCMI_VIPER_XGXS_WRITE_MIICTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIICTLr,(_r._miictl)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_MIICTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIICTLr,(_r._miictl))
#define BCMI_VIPER_XGXS_READLN_MIICTLr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MIICTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._miictl))
#define BCMI_VIPER_XGXS_WRITELN_MIICTLr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIICTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._miictl))
#define BCMI_VIPER_XGXS_WRITEALL_MIICTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIICTLr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._miictl))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define MIICTLr BCMI_VIPER_XGXS_MIICTLr
#define MIICTLr_SIZE BCMI_VIPER_XGXS_MIICTLr_SIZE
typedef BCMI_VIPER_XGXS_MIICTLr_t MIICTLr_t;
#define MIICTLr_CLR BCMI_VIPER_XGXS_MIICTLr_CLR
#define MIICTLr_SET BCMI_VIPER_XGXS_MIICTLr_SET
#define MIICTLr_GET BCMI_VIPER_XGXS_MIICTLr_GET
#define MIICTLr_RST_HWf_GET BCMI_VIPER_XGXS_MIICTLr_RST_HWf_GET
#define MIICTLr_RST_HWf_SET BCMI_VIPER_XGXS_MIICTLr_RST_HWf_SET
#define MIICTLr_GLOOPBACKf_GET BCMI_VIPER_XGXS_MIICTLr_GLOOPBACKf_GET
#define MIICTLr_GLOOPBACKf_SET BCMI_VIPER_XGXS_MIICTLr_GLOOPBACKf_SET
#define MIICTLr_MANUAL_SPEED0f_GET BCMI_VIPER_XGXS_MIICTLr_MANUAL_SPEED0f_GET
#define MIICTLr_MANUAL_SPEED0f_SET BCMI_VIPER_XGXS_MIICTLr_MANUAL_SPEED0f_SET
#define MIICTLr_AUTONEG_ENABLEf_GET BCMI_VIPER_XGXS_MIICTLr_AUTONEG_ENABLEf_GET
#define MIICTLr_AUTONEG_ENABLEf_SET BCMI_VIPER_XGXS_MIICTLr_AUTONEG_ENABLEf_SET
#define MIICTLr_PWRDWN_SWf_GET BCMI_VIPER_XGXS_MIICTLr_PWRDWN_SWf_GET
#define MIICTLr_PWRDWN_SWf_SET BCMI_VIPER_XGXS_MIICTLr_PWRDWN_SWf_SET
#define MIICTLr_RESTART_AUTONEGf_GET BCMI_VIPER_XGXS_MIICTLr_RESTART_AUTONEGf_GET
#define MIICTLr_RESTART_AUTONEGf_SET BCMI_VIPER_XGXS_MIICTLr_RESTART_AUTONEGf_SET
#define MIICTLr_FULL_DUPLEXf_GET BCMI_VIPER_XGXS_MIICTLr_FULL_DUPLEXf_GET
#define MIICTLr_FULL_DUPLEXf_SET BCMI_VIPER_XGXS_MIICTLr_FULL_DUPLEXf_SET
#define MIICTLr_COLLISION_TEST_ENf_GET BCMI_VIPER_XGXS_MIICTLr_COLLISION_TEST_ENf_GET
#define MIICTLr_COLLISION_TEST_ENf_SET BCMI_VIPER_XGXS_MIICTLr_COLLISION_TEST_ENf_SET
#define MIICTLr_MANUAL_SPEED1f_GET BCMI_VIPER_XGXS_MIICTLr_MANUAL_SPEED1f_GET
#define MIICTLr_MANUAL_SPEED1f_SET BCMI_VIPER_XGXS_MIICTLr_MANUAL_SPEED1f_SET
#define READ_MIICTLr BCMI_VIPER_XGXS_READ_MIICTLr
#define WRITE_MIICTLr BCMI_VIPER_XGXS_WRITE_MIICTLr
#define MODIFY_MIICTLr BCMI_VIPER_XGXS_MODIFY_MIICTLr
#define READLN_MIICTLr BCMI_VIPER_XGXS_READLN_MIICTLr
#define WRITELN_MIICTLr BCMI_VIPER_XGXS_WRITELN_MIICTLr
#define WRITEALL_MIICTLr BCMI_VIPER_XGXS_WRITEALL_MIICTLr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_MIICTLr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  MIISTAT
 * BLOCKS:   CL22_B0
 * REGADDR:  0x0001
 * DESC:     IEEE MII status register
 * RESETVAL: 0x109 (265)
 * ACCESS:   R/O
 * FIELDS:
 *     EXTENDED_CAPABILITY 1 = extended register capabilities supported0 = basic register set capabilities only
 *     JABBER_DETECT    1 = jabber condition detected0 = no jabber condition detected
 *     LINK_STATUS      1 = link pass0 = link faillatching-low
 *     AUTONEG_ABILITY  1 = auto-negotiation capable0 = not auto-negotiation capable
 *     REMOTE_FAULT     1 = remote fault detected0 = no remote fault detectedlatching-high
 *     AUTONEG_COMPLETE 1 = auto-negotiation complete0 = auto-negotiation in progress
 *     MF_PREAMBLE_SUPRESSION 1 = PHY will accept management frames with preamble suppressed0 = PHY will not accept management frames with
 *     EXTENDED_STATUS  1 = extended status information in register 0Fh0 = no extended status info in register 0Fh
 *     S100BASE_T2_HALF_DUPLEX_CAPABLE 1 = 100Base-T2 half duplex capable0 = not 100Base-T2 half duplex capable
 *     S100BASE_T2_FULL_DUPLEX_CAPABLE 1 = 100Base-T2 full duplex capable0 = not 100Base-T2 full duplex capable
 *     S10BASE_T_HALF_DUPLEX_CAPABLE 1 = 10Base-T half duplex capable0 = not 10Base-T half duplex capable
 *     S10BASE_T_FULL_DUPLEX_CAPABLE 1 = 10Base-T full duplex capable0 = not 10Base-T full duplex capable
 *     S100BASE_X_HALF_DUPLEX_CAPABLE 1 = 100Base-X half duplex capable0 = not 100Base-X half duplex capable
 *     S100BASE_X_FULL_DUPLEX_CAPABLE 1 = 100Base-X full duplex capable0 = not 100Base-X full duplex capable
 *     S100BASE_T4_CAPABLE 1 = 100Base-T4 capable0 = not 100Base-T4 capable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_MIISTATr (0x00000001 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_MIISTATr_SIZE 4

/*
 * This structure should be used to declare and program MIISTAT.
 *
 */
typedef union BCMI_VIPER_XGXS_MIISTATr_s {
	uint32_t v[1];
	uint32_t miistat[1];
	uint32_t _miistat;
} BCMI_VIPER_XGXS_MIISTATr_t;

#define BCMI_VIPER_XGXS_MIISTATr_CLR(r) (r).miistat[0] = 0
#define BCMI_VIPER_XGXS_MIISTATr_SET(r,d) (r).miistat[0] = d
#define BCMI_VIPER_XGXS_MIISTATr_GET(r) (r).miistat[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_T4_CAPABLEf_GET(r) ((((r).miistat[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_T4_CAPABLEf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).miistat[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).miistat[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).miistat[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).miistat[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).miistat[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).miistat[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_MIISTATr_EXTENDED_STATUSf_GET(r) ((((r).miistat[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_EXTENDED_STATUSf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_MIISTATr_MF_PREAMBLE_SUPRESSIONf_GET(r) ((((r).miistat[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_MF_PREAMBLE_SUPRESSIONf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_MIISTATr_AUTONEG_COMPLETEf_GET(r) ((((r).miistat[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_AUTONEG_COMPLETEf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_MIISTATr_REMOTE_FAULTf_GET(r) ((((r).miistat[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_REMOTE_FAULTf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_MIISTATr_AUTONEG_ABILITYf_GET(r) ((((r).miistat[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_AUTONEG_ABILITYf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_MIISTATr_LINK_STATUSf_GET(r) ((((r).miistat[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_LINK_STATUSf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_MIISTATr_JABBER_DETECTf_GET(r) ((((r).miistat[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_JABBER_DETECTf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_MIISTATr_EXTENDED_CAPABILITYf_GET(r) (((r).miistat[0]) & 0x1)
#define BCMI_VIPER_XGXS_MIISTATr_EXTENDED_CAPABILITYf_SET(r,f) (r).miistat[0]=(((r).miistat[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access MIISTAT.
 *
 */
#define BCMI_VIPER_XGXS_READ_MIISTATr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MIISTATr,(_r._miistat))
#define BCMI_VIPER_XGXS_WRITE_MIISTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIISTATr,(_r._miistat)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_MIISTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIISTATr,(_r._miistat))
#define BCMI_VIPER_XGXS_READLN_MIISTATr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MIISTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._miistat))
#define BCMI_VIPER_XGXS_WRITELN_MIISTATr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIISTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._miistat))
#define BCMI_VIPER_XGXS_WRITEALL_MIISTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIISTATr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._miistat))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define MIISTATr BCMI_VIPER_XGXS_MIISTATr
#define MIISTATr_SIZE BCMI_VIPER_XGXS_MIISTATr_SIZE
typedef BCMI_VIPER_XGXS_MIISTATr_t MIISTATr_t;
#define MIISTATr_CLR BCMI_VIPER_XGXS_MIISTATr_CLR
#define MIISTATr_SET BCMI_VIPER_XGXS_MIISTATr_SET
#define MIISTATr_GET BCMI_VIPER_XGXS_MIISTATr_GET
#define MIISTATr_S100BASE_T4_CAPABLEf_GET BCMI_VIPER_XGXS_MIISTATr_S100BASE_T4_CAPABLEf_GET
#define MIISTATr_S100BASE_T4_CAPABLEf_SET BCMI_VIPER_XGXS_MIISTATr_S100BASE_T4_CAPABLEf_SET
#define MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_GET
#define MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_SET
#define MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_GET
#define MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_SET
#define MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_GET
#define MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_SET
#define MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_GET
#define MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_SET
#define MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_GET
#define MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_SET
#define MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_GET
#define MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_SET
#define MIISTATr_EXTENDED_STATUSf_GET BCMI_VIPER_XGXS_MIISTATr_EXTENDED_STATUSf_GET
#define MIISTATr_EXTENDED_STATUSf_SET BCMI_VIPER_XGXS_MIISTATr_EXTENDED_STATUSf_SET
#define MIISTATr_MF_PREAMBLE_SUPRESSIONf_GET BCMI_VIPER_XGXS_MIISTATr_MF_PREAMBLE_SUPRESSIONf_GET
#define MIISTATr_MF_PREAMBLE_SUPRESSIONf_SET BCMI_VIPER_XGXS_MIISTATr_MF_PREAMBLE_SUPRESSIONf_SET
#define MIISTATr_AUTONEG_COMPLETEf_GET BCMI_VIPER_XGXS_MIISTATr_AUTONEG_COMPLETEf_GET
#define MIISTATr_AUTONEG_COMPLETEf_SET BCMI_VIPER_XGXS_MIISTATr_AUTONEG_COMPLETEf_SET
#define MIISTATr_REMOTE_FAULTf_GET BCMI_VIPER_XGXS_MIISTATr_REMOTE_FAULTf_GET
#define MIISTATr_REMOTE_FAULTf_SET BCMI_VIPER_XGXS_MIISTATr_REMOTE_FAULTf_SET
#define MIISTATr_AUTONEG_ABILITYf_GET BCMI_VIPER_XGXS_MIISTATr_AUTONEG_ABILITYf_GET
#define MIISTATr_AUTONEG_ABILITYf_SET BCMI_VIPER_XGXS_MIISTATr_AUTONEG_ABILITYf_SET
#define MIISTATr_LINK_STATUSf_GET BCMI_VIPER_XGXS_MIISTATr_LINK_STATUSf_GET
#define MIISTATr_LINK_STATUSf_SET BCMI_VIPER_XGXS_MIISTATr_LINK_STATUSf_SET
#define MIISTATr_JABBER_DETECTf_GET BCMI_VIPER_XGXS_MIISTATr_JABBER_DETECTf_GET
#define MIISTATr_JABBER_DETECTf_SET BCMI_VIPER_XGXS_MIISTATr_JABBER_DETECTf_SET
#define MIISTATr_EXTENDED_CAPABILITYf_GET BCMI_VIPER_XGXS_MIISTATr_EXTENDED_CAPABILITYf_GET
#define MIISTATr_EXTENDED_CAPABILITYf_SET BCMI_VIPER_XGXS_MIISTATr_EXTENDED_CAPABILITYf_SET
#define READ_MIISTATr BCMI_VIPER_XGXS_READ_MIISTATr
#define WRITE_MIISTATr BCMI_VIPER_XGXS_WRITE_MIISTATr
#define MODIFY_MIISTATr BCMI_VIPER_XGXS_MODIFY_MIISTATr
#define READLN_MIISTATr BCMI_VIPER_XGXS_READLN_MIISTATr
#define WRITELN_MIISTATr BCMI_VIPER_XGXS_WRITELN_MIISTATr
#define WRITEALL_MIISTATr BCMI_VIPER_XGXS_WRITEALL_MIISTATr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_MIISTATr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  ID1
 * BLOCKS:   CL22_B0
 * REGADDR:  0x0002
 * DESC:     IEEE phy ID LSByte register
 * RESETVAL: 0x143 (323)
 * ACCESS:   R/O
 * FIELDS:
 *     REGID            PHYID register, MSB
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_ID1r (0x00000002 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_ID1r_SIZE 4

/*
 * This structure should be used to declare and program ID1.
 *
 */
typedef union BCMI_VIPER_XGXS_ID1r_s {
	uint32_t v[1];
	uint32_t id1[1];
	uint32_t _id1;
} BCMI_VIPER_XGXS_ID1r_t;

#define BCMI_VIPER_XGXS_ID1r_CLR(r) (r).id1[0] = 0
#define BCMI_VIPER_XGXS_ID1r_SET(r,d) (r).id1[0] = d
#define BCMI_VIPER_XGXS_ID1r_GET(r) (r).id1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_ID1r_REGIDf_GET(r) (((r).id1[0]) & 0xffff)
#define BCMI_VIPER_XGXS_ID1r_REGIDf_SET(r,f) (r).id1[0]=(((r).id1[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access ID1.
 *
 */
#define BCMI_VIPER_XGXS_READ_ID1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_ID1r,(_r._id1))
#define BCMI_VIPER_XGXS_WRITE_ID1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_ID1r,(_r._id1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_ID1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_ID1r,(_r._id1))
#define BCMI_VIPER_XGXS_READLN_ID1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_ID1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._id1))
#define BCMI_VIPER_XGXS_WRITELN_ID1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_ID1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._id1))
#define BCMI_VIPER_XGXS_WRITEALL_ID1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_ID1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._id1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define ID1r BCMI_VIPER_XGXS_ID1r
#define ID1r_SIZE BCMI_VIPER_XGXS_ID1r_SIZE
typedef BCMI_VIPER_XGXS_ID1r_t ID1r_t;
#define ID1r_CLR BCMI_VIPER_XGXS_ID1r_CLR
#define ID1r_SET BCMI_VIPER_XGXS_ID1r_SET
#define ID1r_GET BCMI_VIPER_XGXS_ID1r_GET
#define ID1r_REGIDf_GET BCMI_VIPER_XGXS_ID1r_REGIDf_GET
#define ID1r_REGIDf_SET BCMI_VIPER_XGXS_ID1r_REGIDf_SET
#define READ_ID1r BCMI_VIPER_XGXS_READ_ID1r
#define WRITE_ID1r BCMI_VIPER_XGXS_WRITE_ID1r
#define MODIFY_ID1r BCMI_VIPER_XGXS_MODIFY_ID1r
#define READLN_ID1r BCMI_VIPER_XGXS_READLN_ID1r
#define WRITELN_ID1r BCMI_VIPER_XGXS_WRITELN_ID1r
#define WRITEALL_ID1r BCMI_VIPER_XGXS_WRITEALL_ID1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_ID1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  ID2
 * BLOCKS:   CL22_B0
 * REGADDR:  0x0003
 * DESC:     IEEE phy ID MSByte register
 * RESETVAL: 0xbff0 (49136)
 * ACCESS:   R/O
 * FIELDS:
 *     REGID            PHYID register, LSB
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_ID2r (0x00000003 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_ID2r_SIZE 4

/*
 * This structure should be used to declare and program ID2.
 *
 */
typedef union BCMI_VIPER_XGXS_ID2r_s {
	uint32_t v[1];
	uint32_t id2[1];
	uint32_t _id2;
} BCMI_VIPER_XGXS_ID2r_t;

#define BCMI_VIPER_XGXS_ID2r_CLR(r) (r).id2[0] = 0
#define BCMI_VIPER_XGXS_ID2r_SET(r,d) (r).id2[0] = d
#define BCMI_VIPER_XGXS_ID2r_GET(r) (r).id2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_ID2r_REGIDf_GET(r) (((r).id2[0]) & 0xffff)
#define BCMI_VIPER_XGXS_ID2r_REGIDf_SET(r,f) (r).id2[0]=(((r).id2[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access ID2.
 *
 */
#define BCMI_VIPER_XGXS_READ_ID2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_ID2r,(_r._id2))
#define BCMI_VIPER_XGXS_WRITE_ID2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_ID2r,(_r._id2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_ID2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_ID2r,(_r._id2))
#define BCMI_VIPER_XGXS_READLN_ID2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_ID2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._id2))
#define BCMI_VIPER_XGXS_WRITELN_ID2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_ID2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._id2))
#define BCMI_VIPER_XGXS_WRITEALL_ID2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_ID2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._id2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define ID2r BCMI_VIPER_XGXS_ID2r
#define ID2r_SIZE BCMI_VIPER_XGXS_ID2r_SIZE
typedef BCMI_VIPER_XGXS_ID2r_t ID2r_t;
#define ID2r_CLR BCMI_VIPER_XGXS_ID2r_CLR
#define ID2r_SET BCMI_VIPER_XGXS_ID2r_SET
#define ID2r_GET BCMI_VIPER_XGXS_ID2r_GET
#define ID2r_REGIDf_GET BCMI_VIPER_XGXS_ID2r_REGIDf_GET
#define ID2r_REGIDf_SET BCMI_VIPER_XGXS_ID2r_REGIDf_SET
#define READ_ID2r BCMI_VIPER_XGXS_READ_ID2r
#define WRITE_ID2r BCMI_VIPER_XGXS_WRITE_ID2r
#define MODIFY_ID2r BCMI_VIPER_XGXS_MODIFY_ID2r
#define READLN_ID2r BCMI_VIPER_XGXS_READLN_ID2r
#define WRITELN_ID2r BCMI_VIPER_XGXS_WRITELN_ID2r
#define WRITEALL_ID2r BCMI_VIPER_XGXS_WRITEALL_ID2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_ID2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  AUTONEGADV
 * BLOCKS:   CL22_B0
 * REGADDR:  0x0004
 * DESC:     IEEE auto-negotiation advertised abilities register
 * RESETVAL: 0x20 (32)
 * ACCESS:   R/W
 * FIELDS:
 *     FULL_DUPLEX      1 = advertise full-duplex0 = do not advertise full-duplex
 *     HALF_DUPLEX      1 = advertise half-duplex0 = do not advertise half-duplex
 *     PAUSE            Bits8 7---0 0 = no pause0 1 = symmetric pause1 0 = asymmetric pause toward link partner1 1 = both symmetric pause and asymmetricpause toward local device
 *     REMOTE_FAULT     Bits13 12-----0 0 = no_remote_fault0 1 = link_failure1 0 = offline1 1 = autoneg_error
 *     NEXT_PAGE        1 = supports additional pages using NP function0 = does not support additional pages using NP function
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_AUTONEGADVr (0x00000004 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_AUTONEGADVr_SIZE 4

/*
 * This structure should be used to declare and program AUTONEGADV.
 *
 */
typedef union BCMI_VIPER_XGXS_AUTONEGADVr_s {
	uint32_t v[1];
	uint32_t autonegadv[1];
	uint32_t _autonegadv;
} BCMI_VIPER_XGXS_AUTONEGADVr_t;

#define BCMI_VIPER_XGXS_AUTONEGADVr_CLR(r) (r).autonegadv[0] = 0
#define BCMI_VIPER_XGXS_AUTONEGADVr_SET(r,d) (r).autonegadv[0] = d
#define BCMI_VIPER_XGXS_AUTONEGADVr_GET(r) (r).autonegadv[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_AUTONEGADVr_NEXT_PAGEf_GET(r) ((((r).autonegadv[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGADVr_NEXT_PAGEf_SET(r,f) (r).autonegadv[0]=(((r).autonegadv[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_AUTONEGADVr_REMOTE_FAULTf_GET(r) ((((r).autonegadv[0]) >> 12) & 0x3)
#define BCMI_VIPER_XGXS_AUTONEGADVr_REMOTE_FAULTf_SET(r,f) (r).autonegadv[0]=(((r).autonegadv[0] & ~((uint32_t)0x3 << 12)) | ((((uint32_t)f) & 0x3) << 12)) | (3 << (16 + 12))
#define BCMI_VIPER_XGXS_AUTONEGADVr_PAUSEf_GET(r) ((((r).autonegadv[0]) >> 7) & 0x3)
#define BCMI_VIPER_XGXS_AUTONEGADVr_PAUSEf_SET(r,f) (r).autonegadv[0]=(((r).autonegadv[0] & ~((uint32_t)0x3 << 7)) | ((((uint32_t)f) & 0x3) << 7)) | (3 << (16 + 7))
#define BCMI_VIPER_XGXS_AUTONEGADVr_HALF_DUPLEXf_GET(r) ((((r).autonegadv[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGADVr_HALF_DUPLEXf_SET(r,f) (r).autonegadv[0]=(((r).autonegadv[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_AUTONEGADVr_FULL_DUPLEXf_GET(r) ((((r).autonegadv[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGADVr_FULL_DUPLEXf_SET(r,f) (r).autonegadv[0]=(((r).autonegadv[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))

/*
 * These macros can be used to access AUTONEGADV.
 *
 */
#define BCMI_VIPER_XGXS_READ_AUTONEGADVr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGADVr,(_r._autonegadv))
#define BCMI_VIPER_XGXS_WRITE_AUTONEGADVr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGADVr,(_r._autonegadv)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_AUTONEGADVr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGADVr,(_r._autonegadv))
#define BCMI_VIPER_XGXS_READLN_AUTONEGADVr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGADVr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autonegadv))
#define BCMI_VIPER_XGXS_WRITELN_AUTONEGADVr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGADVr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autonegadv))
#define BCMI_VIPER_XGXS_WRITEALL_AUTONEGADVr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGADVr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._autonegadv))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define AUTONEGADVr BCMI_VIPER_XGXS_AUTONEGADVr
#define AUTONEGADVr_SIZE BCMI_VIPER_XGXS_AUTONEGADVr_SIZE
typedef BCMI_VIPER_XGXS_AUTONEGADVr_t AUTONEGADVr_t;
#define AUTONEGADVr_CLR BCMI_VIPER_XGXS_AUTONEGADVr_CLR
#define AUTONEGADVr_SET BCMI_VIPER_XGXS_AUTONEGADVr_SET
#define AUTONEGADVr_GET BCMI_VIPER_XGXS_AUTONEGADVr_GET
#define AUTONEGADVr_NEXT_PAGEf_GET BCMI_VIPER_XGXS_AUTONEGADVr_NEXT_PAGEf_GET
#define AUTONEGADVr_NEXT_PAGEf_SET BCMI_VIPER_XGXS_AUTONEGADVr_NEXT_PAGEf_SET
#define AUTONEGADVr_REMOTE_FAULTf_GET BCMI_VIPER_XGXS_AUTONEGADVr_REMOTE_FAULTf_GET
#define AUTONEGADVr_REMOTE_FAULTf_SET BCMI_VIPER_XGXS_AUTONEGADVr_REMOTE_FAULTf_SET
#define AUTONEGADVr_PAUSEf_GET BCMI_VIPER_XGXS_AUTONEGADVr_PAUSEf_GET
#define AUTONEGADVr_PAUSEf_SET BCMI_VIPER_XGXS_AUTONEGADVr_PAUSEf_SET
#define AUTONEGADVr_HALF_DUPLEXf_GET BCMI_VIPER_XGXS_AUTONEGADVr_HALF_DUPLEXf_GET
#define AUTONEGADVr_HALF_DUPLEXf_SET BCMI_VIPER_XGXS_AUTONEGADVr_HALF_DUPLEXf_SET
#define AUTONEGADVr_FULL_DUPLEXf_GET BCMI_VIPER_XGXS_AUTONEGADVr_FULL_DUPLEXf_GET
#define AUTONEGADVr_FULL_DUPLEXf_SET BCMI_VIPER_XGXS_AUTONEGADVr_FULL_DUPLEXf_SET
#define READ_AUTONEGADVr BCMI_VIPER_XGXS_READ_AUTONEGADVr
#define WRITE_AUTONEGADVr BCMI_VIPER_XGXS_WRITE_AUTONEGADVr
#define MODIFY_AUTONEGADVr BCMI_VIPER_XGXS_MODIFY_AUTONEGADVr
#define READLN_AUTONEGADVr BCMI_VIPER_XGXS_READLN_AUTONEGADVr
#define WRITELN_AUTONEGADVr BCMI_VIPER_XGXS_WRITELN_AUTONEGADVr
#define WRITEALL_AUTONEGADVr BCMI_VIPER_XGXS_WRITEALL_AUTONEGADVr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_AUTONEGADVr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  AUTONEGLPABIL
 * BLOCKS:   CL22_B0
 * REGADDR:  0x0005
 * DESC:     IEEE auto-negotiation link partner abilities register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     SGMII_MODE       1 = SGMII mode0 = fiber modeNote: When the link partner is in SGMII mode (bit 0 = 1), then bit 15 = link, bit 12 = duplex, bits 11:10 = speed, bit 14 = acknowledge. The other bits are reserved and should be zero.
 *     FULL_DUPLEX      1 = link partner is full duplex capable0 = link partner is not full duplex capable
 *     HALF_DUPLEX      1 = link partner is half-duplex capable0 = link partner is not half-duplex capable
 *     PAUSE            Bits8 7---0 0 = no pause0 1 = symmetric pause1 0 = asymmetric pause toward link partner1 1 = both symmetric pause and asymmetricpause toward local device
 *     REMOTE_FAULT     Bits13 12-----0 0 = no_remote_fault0 1 = link_failure1 0 = offline1 1 = autoneg_error
 *     ACKNOWLEDGE      1 = link partner has received link code word0 = link partner has not received link code word
 *     NEXT_PAGE        1 = link partner is next page able0 = link partner is not next page able
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_AUTONEGLPABILr (0x00000005 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_AUTONEGLPABILr_SIZE 4

/*
 * This structure should be used to declare and program AUTONEGLPABIL.
 *
 */
typedef union BCMI_VIPER_XGXS_AUTONEGLPABILr_s {
	uint32_t v[1];
	uint32_t autoneglpabil[1];
	uint32_t _autoneglpabil;
} BCMI_VIPER_XGXS_AUTONEGLPABILr_t;

#define BCMI_VIPER_XGXS_AUTONEGLPABILr_CLR(r) (r).autoneglpabil[0] = 0
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_SET(r,d) (r).autoneglpabil[0] = d
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_GET(r) (r).autoneglpabil[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_NEXT_PAGEf_GET(r) ((((r).autoneglpabil[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_NEXT_PAGEf_SET(r,f) (r).autoneglpabil[0]=(((r).autoneglpabil[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_ACKNOWLEDGEf_GET(r) ((((r).autoneglpabil[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_ACKNOWLEDGEf_SET(r,f) (r).autoneglpabil[0]=(((r).autoneglpabil[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_REMOTE_FAULTf_GET(r) ((((r).autoneglpabil[0]) >> 12) & 0x3)
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_REMOTE_FAULTf_SET(r,f) (r).autoneglpabil[0]=(((r).autoneglpabil[0] & ~((uint32_t)0x3 << 12)) | ((((uint32_t)f) & 0x3) << 12)) | (3 << (16 + 12))
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_PAUSEf_GET(r) ((((r).autoneglpabil[0]) >> 7) & 0x3)
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_PAUSEf_SET(r,f) (r).autoneglpabil[0]=(((r).autoneglpabil[0] & ~((uint32_t)0x3 << 7)) | ((((uint32_t)f) & 0x3) << 7)) | (3 << (16 + 7))
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_HALF_DUPLEXf_GET(r) ((((r).autoneglpabil[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_HALF_DUPLEXf_SET(r,f) (r).autoneglpabil[0]=(((r).autoneglpabil[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_FULL_DUPLEXf_GET(r) ((((r).autoneglpabil[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_FULL_DUPLEXf_SET(r,f) (r).autoneglpabil[0]=(((r).autoneglpabil[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_SGMII_MODEf_GET(r) (((r).autoneglpabil[0]) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABILr_SGMII_MODEf_SET(r,f) (r).autoneglpabil[0]=(((r).autoneglpabil[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access AUTONEGLPABIL.
 *
 */
#define BCMI_VIPER_XGXS_READ_AUTONEGLPABILr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGLPABILr,(_r._autoneglpabil))
#define BCMI_VIPER_XGXS_WRITE_AUTONEGLPABILr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGLPABILr,(_r._autoneglpabil)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_AUTONEGLPABILr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGLPABILr,(_r._autoneglpabil))
#define BCMI_VIPER_XGXS_READLN_AUTONEGLPABILr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGLPABILr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autoneglpabil))
#define BCMI_VIPER_XGXS_WRITELN_AUTONEGLPABILr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGLPABILr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autoneglpabil))
#define BCMI_VIPER_XGXS_WRITEALL_AUTONEGLPABILr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGLPABILr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._autoneglpabil))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define AUTONEGLPABILr BCMI_VIPER_XGXS_AUTONEGLPABILr
#define AUTONEGLPABILr_SIZE BCMI_VIPER_XGXS_AUTONEGLPABILr_SIZE
typedef BCMI_VIPER_XGXS_AUTONEGLPABILr_t AUTONEGLPABILr_t;
#define AUTONEGLPABILr_CLR BCMI_VIPER_XGXS_AUTONEGLPABILr_CLR
#define AUTONEGLPABILr_SET BCMI_VIPER_XGXS_AUTONEGLPABILr_SET
#define AUTONEGLPABILr_GET BCMI_VIPER_XGXS_AUTONEGLPABILr_GET
#define AUTONEGLPABILr_NEXT_PAGEf_GET BCMI_VIPER_XGXS_AUTONEGLPABILr_NEXT_PAGEf_GET
#define AUTONEGLPABILr_NEXT_PAGEf_SET BCMI_VIPER_XGXS_AUTONEGLPABILr_NEXT_PAGEf_SET
#define AUTONEGLPABILr_ACKNOWLEDGEf_GET BCMI_VIPER_XGXS_AUTONEGLPABILr_ACKNOWLEDGEf_GET
#define AUTONEGLPABILr_ACKNOWLEDGEf_SET BCMI_VIPER_XGXS_AUTONEGLPABILr_ACKNOWLEDGEf_SET
#define AUTONEGLPABILr_REMOTE_FAULTf_GET BCMI_VIPER_XGXS_AUTONEGLPABILr_REMOTE_FAULTf_GET
#define AUTONEGLPABILr_REMOTE_FAULTf_SET BCMI_VIPER_XGXS_AUTONEGLPABILr_REMOTE_FAULTf_SET
#define AUTONEGLPABILr_PAUSEf_GET BCMI_VIPER_XGXS_AUTONEGLPABILr_PAUSEf_GET
#define AUTONEGLPABILr_PAUSEf_SET BCMI_VIPER_XGXS_AUTONEGLPABILr_PAUSEf_SET
#define AUTONEGLPABILr_HALF_DUPLEXf_GET BCMI_VIPER_XGXS_AUTONEGLPABILr_HALF_DUPLEXf_GET
#define AUTONEGLPABILr_HALF_DUPLEXf_SET BCMI_VIPER_XGXS_AUTONEGLPABILr_HALF_DUPLEXf_SET
#define AUTONEGLPABILr_FULL_DUPLEXf_GET BCMI_VIPER_XGXS_AUTONEGLPABILr_FULL_DUPLEXf_GET
#define AUTONEGLPABILr_FULL_DUPLEXf_SET BCMI_VIPER_XGXS_AUTONEGLPABILr_FULL_DUPLEXf_SET
#define AUTONEGLPABILr_SGMII_MODEf_GET BCMI_VIPER_XGXS_AUTONEGLPABILr_SGMII_MODEf_GET
#define AUTONEGLPABILr_SGMII_MODEf_SET BCMI_VIPER_XGXS_AUTONEGLPABILr_SGMII_MODEf_SET
#define READ_AUTONEGLPABILr BCMI_VIPER_XGXS_READ_AUTONEGLPABILr
#define WRITE_AUTONEGLPABILr BCMI_VIPER_XGXS_WRITE_AUTONEGLPABILr
#define MODIFY_AUTONEGLPABILr BCMI_VIPER_XGXS_MODIFY_AUTONEGLPABILr
#define READLN_AUTONEGLPABILr BCMI_VIPER_XGXS_READLN_AUTONEGLPABILr
#define WRITELN_AUTONEGLPABILr BCMI_VIPER_XGXS_WRITELN_AUTONEGLPABILr
#define WRITEALL_AUTONEGLPABILr BCMI_VIPER_XGXS_WRITEALL_AUTONEGLPABILr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_AUTONEGLPABILr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  AUTONEGEXP
 * BLOCKS:   CL22_B0
 * REGADDR:  0x0006
 * DESC:     IEEE auto-negotiation expansion register
 * RESETVAL: 0x4 (4)
 * ACCESS:   R/O
 * FIELDS:
 *     PAGE_RECEIVED    1 = new link code word has been received0 = new link code word has not been received
 *     NEXT_PAGE_ABILITY 1 = local device is next page able0 = local device is not next page able
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_AUTONEGEXPr (0x00000006 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_AUTONEGEXPr_SIZE 4

/*
 * This structure should be used to declare and program AUTONEGEXP.
 *
 */
typedef union BCMI_VIPER_XGXS_AUTONEGEXPr_s {
	uint32_t v[1];
	uint32_t autonegexp[1];
	uint32_t _autonegexp;
} BCMI_VIPER_XGXS_AUTONEGEXPr_t;

#define BCMI_VIPER_XGXS_AUTONEGEXPr_CLR(r) (r).autonegexp[0] = 0
#define BCMI_VIPER_XGXS_AUTONEGEXPr_SET(r,d) (r).autonegexp[0] = d
#define BCMI_VIPER_XGXS_AUTONEGEXPr_GET(r) (r).autonegexp[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_AUTONEGEXPr_NEXT_PAGE_ABILITYf_GET(r) ((((r).autonegexp[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGEXPr_NEXT_PAGE_ABILITYf_SET(r,f) (r).autonegexp[0]=(((r).autonegexp[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_AUTONEGEXPr_PAGE_RECEIVEDf_GET(r) ((((r).autonegexp[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGEXPr_PAGE_RECEIVEDf_SET(r,f) (r).autonegexp[0]=(((r).autonegexp[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))

/*
 * These macros can be used to access AUTONEGEXP.
 *
 */
#define BCMI_VIPER_XGXS_READ_AUTONEGEXPr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGEXPr,(_r._autonegexp))
#define BCMI_VIPER_XGXS_WRITE_AUTONEGEXPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGEXPr,(_r._autonegexp)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_AUTONEGEXPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGEXPr,(_r._autonegexp))
#define BCMI_VIPER_XGXS_READLN_AUTONEGEXPr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGEXPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autonegexp))
#define BCMI_VIPER_XGXS_WRITELN_AUTONEGEXPr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGEXPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autonegexp))
#define BCMI_VIPER_XGXS_WRITEALL_AUTONEGEXPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGEXPr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._autonegexp))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define AUTONEGEXPr BCMI_VIPER_XGXS_AUTONEGEXPr
#define AUTONEGEXPr_SIZE BCMI_VIPER_XGXS_AUTONEGEXPr_SIZE
typedef BCMI_VIPER_XGXS_AUTONEGEXPr_t AUTONEGEXPr_t;
#define AUTONEGEXPr_CLR BCMI_VIPER_XGXS_AUTONEGEXPr_CLR
#define AUTONEGEXPr_SET BCMI_VIPER_XGXS_AUTONEGEXPr_SET
#define AUTONEGEXPr_GET BCMI_VIPER_XGXS_AUTONEGEXPr_GET
#define AUTONEGEXPr_NEXT_PAGE_ABILITYf_GET BCMI_VIPER_XGXS_AUTONEGEXPr_NEXT_PAGE_ABILITYf_GET
#define AUTONEGEXPr_NEXT_PAGE_ABILITYf_SET BCMI_VIPER_XGXS_AUTONEGEXPr_NEXT_PAGE_ABILITYf_SET
#define AUTONEGEXPr_PAGE_RECEIVEDf_GET BCMI_VIPER_XGXS_AUTONEGEXPr_PAGE_RECEIVEDf_GET
#define AUTONEGEXPr_PAGE_RECEIVEDf_SET BCMI_VIPER_XGXS_AUTONEGEXPr_PAGE_RECEIVEDf_SET
#define READ_AUTONEGEXPr BCMI_VIPER_XGXS_READ_AUTONEGEXPr
#define WRITE_AUTONEGEXPr BCMI_VIPER_XGXS_WRITE_AUTONEGEXPr
#define MODIFY_AUTONEGEXPr BCMI_VIPER_XGXS_MODIFY_AUTONEGEXPr
#define READLN_AUTONEGEXPr BCMI_VIPER_XGXS_READLN_AUTONEGEXPr
#define WRITELN_AUTONEGEXPr BCMI_VIPER_XGXS_WRITELN_AUTONEGEXPr
#define WRITEALL_AUTONEGEXPr BCMI_VIPER_XGXS_WRITEALL_AUTONEGEXPr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_AUTONEGEXPr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  AUTONEGNP
 * BLOCKS:   CL22_B0
 * REGADDR:  0x0007
 * DESC:     IEEE auto-negotiation next page register
 * RESETVAL: 0x2001 (8193)
 * ACCESS:   R/W
 * FIELDS:
 *     MESSAGE          Message or Unformatted Code Field11'h400 = Over 1G Message Page11'h410 = Remote CuPHY Message Page11'h411 = MDIO Register Write Message Page11'h412 = MDIO Register Read Request Message Page11'h413 = MDIO Register Response Message PageSee IEEE 802.3 Annex 28C for more standard next page detailsSee BRCM-Serdes-AN for BAM specific details
 *     TOGGLE           Opposite value of bit in previous page
 *     ACK2             Acknowledge 2 bit
 *     MESSAGE_PAGE     0 = Unformatted Page1 = Message Page
 *     ACK              Acknowledge bit
 *     NEXT_PAGE        0 = last page1 = additional next page(s) follow
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_AUTONEGNPr (0x00000007 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_AUTONEGNPr_SIZE 4

/*
 * This structure should be used to declare and program AUTONEGNP.
 *
 */
typedef union BCMI_VIPER_XGXS_AUTONEGNPr_s {
	uint32_t v[1];
	uint32_t autonegnp[1];
	uint32_t _autonegnp;
} BCMI_VIPER_XGXS_AUTONEGNPr_t;

#define BCMI_VIPER_XGXS_AUTONEGNPr_CLR(r) (r).autonegnp[0] = 0
#define BCMI_VIPER_XGXS_AUTONEGNPr_SET(r,d) (r).autonegnp[0] = d
#define BCMI_VIPER_XGXS_AUTONEGNPr_GET(r) (r).autonegnp[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_AUTONEGNPr_NEXT_PAGEf_GET(r) ((((r).autonegnp[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGNPr_NEXT_PAGEf_SET(r,f) (r).autonegnp[0]=(((r).autonegnp[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_AUTONEGNPr_ACKf_GET(r) ((((r).autonegnp[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGNPr_ACKf_SET(r,f) (r).autonegnp[0]=(((r).autonegnp[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_AUTONEGNPr_MESSAGE_PAGEf_GET(r) ((((r).autonegnp[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGNPr_MESSAGE_PAGEf_SET(r,f) (r).autonegnp[0]=(((r).autonegnp[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_AUTONEGNPr_ACK2f_GET(r) ((((r).autonegnp[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGNPr_ACK2f_SET(r,f) (r).autonegnp[0]=(((r).autonegnp[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_AUTONEGNPr_TOGGLEf_GET(r) ((((r).autonegnp[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGNPr_TOGGLEf_SET(r,f) (r).autonegnp[0]=(((r).autonegnp[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_AUTONEGNPr_MESSAGEf_GET(r) (((r).autonegnp[0]) & 0x7ff)
#define BCMI_VIPER_XGXS_AUTONEGNPr_MESSAGEf_SET(r,f) (r).autonegnp[0]=(((r).autonegnp[0] & ~((uint32_t)0x7ff)) | (((uint32_t)f) & 0x7ff)) | (0x7ff << 16)

/*
 * These macros can be used to access AUTONEGNP.
 *
 */
#define BCMI_VIPER_XGXS_READ_AUTONEGNPr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGNPr,(_r._autonegnp))
#define BCMI_VIPER_XGXS_WRITE_AUTONEGNPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGNPr,(_r._autonegnp)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_AUTONEGNPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGNPr,(_r._autonegnp))
#define BCMI_VIPER_XGXS_READLN_AUTONEGNPr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGNPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autonegnp))
#define BCMI_VIPER_XGXS_WRITELN_AUTONEGNPr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGNPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autonegnp))
#define BCMI_VIPER_XGXS_WRITEALL_AUTONEGNPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGNPr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._autonegnp))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define AUTONEGNPr BCMI_VIPER_XGXS_AUTONEGNPr
#define AUTONEGNPr_SIZE BCMI_VIPER_XGXS_AUTONEGNPr_SIZE
typedef BCMI_VIPER_XGXS_AUTONEGNPr_t AUTONEGNPr_t;
#define AUTONEGNPr_CLR BCMI_VIPER_XGXS_AUTONEGNPr_CLR
#define AUTONEGNPr_SET BCMI_VIPER_XGXS_AUTONEGNPr_SET
#define AUTONEGNPr_GET BCMI_VIPER_XGXS_AUTONEGNPr_GET
#define AUTONEGNPr_NEXT_PAGEf_GET BCMI_VIPER_XGXS_AUTONEGNPr_NEXT_PAGEf_GET
#define AUTONEGNPr_NEXT_PAGEf_SET BCMI_VIPER_XGXS_AUTONEGNPr_NEXT_PAGEf_SET
#define AUTONEGNPr_ACKf_GET BCMI_VIPER_XGXS_AUTONEGNPr_ACKf_GET
#define AUTONEGNPr_ACKf_SET BCMI_VIPER_XGXS_AUTONEGNPr_ACKf_SET
#define AUTONEGNPr_MESSAGE_PAGEf_GET BCMI_VIPER_XGXS_AUTONEGNPr_MESSAGE_PAGEf_GET
#define AUTONEGNPr_MESSAGE_PAGEf_SET BCMI_VIPER_XGXS_AUTONEGNPr_MESSAGE_PAGEf_SET
#define AUTONEGNPr_ACK2f_GET BCMI_VIPER_XGXS_AUTONEGNPr_ACK2f_GET
#define AUTONEGNPr_ACK2f_SET BCMI_VIPER_XGXS_AUTONEGNPr_ACK2f_SET
#define AUTONEGNPr_TOGGLEf_GET BCMI_VIPER_XGXS_AUTONEGNPr_TOGGLEf_GET
#define AUTONEGNPr_TOGGLEf_SET BCMI_VIPER_XGXS_AUTONEGNPr_TOGGLEf_SET
#define AUTONEGNPr_MESSAGEf_GET BCMI_VIPER_XGXS_AUTONEGNPr_MESSAGEf_GET
#define AUTONEGNPr_MESSAGEf_SET BCMI_VIPER_XGXS_AUTONEGNPr_MESSAGEf_SET
#define READ_AUTONEGNPr BCMI_VIPER_XGXS_READ_AUTONEGNPr
#define WRITE_AUTONEGNPr BCMI_VIPER_XGXS_WRITE_AUTONEGNPr
#define MODIFY_AUTONEGNPr BCMI_VIPER_XGXS_MODIFY_AUTONEGNPr
#define READLN_AUTONEGNPr BCMI_VIPER_XGXS_READLN_AUTONEGNPr
#define WRITELN_AUTONEGNPr BCMI_VIPER_XGXS_WRITELN_AUTONEGNPr
#define WRITEALL_AUTONEGNPr BCMI_VIPER_XGXS_WRITEALL_AUTONEGNPr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_AUTONEGNPr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  AUTONEGLPABIL2
 * BLOCKS:   CL22_B0
 * REGADDR:  0x0008
 * DESC:     IEEE auto-negotiation link partner next page register
 * RESETVAL: 0x2001 (8193)
 * ACCESS:   R/O
 * FIELDS:
 *     MESSAGE          Message or Unformatted Code Field11'h400 = Over 1G Message Page11'h410 = Remote CuPHY Message Page11'h411 = MDIO Register Write Message Page11'h412 = MDIO Register Read Request Message Page11'h413 = MDIO Register Response Message PageSee IEEE 802.3 Annex 28C for more standard next page detailsSee BRCM-Serdes-AN for BAM specific details
 *     TOGGLE           Opposite value of bit in previous page
 *     ACK2             Acknowledge 2 bit
 *     MESSAGE_PAGE     0 = Unformatted Page1 = Message Page
 *     ACK              Acknowledge bit
 *     NEXT_PAGE        0 = last page1 = additional next page(s) follow
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r (0x00000008 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_SIZE 4

/*
 * This structure should be used to declare and program AUTONEGLPABIL2.
 *
 */
typedef union BCMI_VIPER_XGXS_AUTONEGLPABIL2r_s {
	uint32_t v[1];
	uint32_t autoneglpabil2[1];
	uint32_t _autoneglpabil2;
} BCMI_VIPER_XGXS_AUTONEGLPABIL2r_t;

#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_CLR(r) (r).autoneglpabil2[0] = 0
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_SET(r,d) (r).autoneglpabil2[0] = d
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_GET(r) (r).autoneglpabil2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_NEXT_PAGEf_GET(r) ((((r).autoneglpabil2[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_NEXT_PAGEf_SET(r,f) (r).autoneglpabil2[0]=(((r).autoneglpabil2[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_ACKf_GET(r) ((((r).autoneglpabil2[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_ACKf_SET(r,f) (r).autoneglpabil2[0]=(((r).autoneglpabil2[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_MESSAGE_PAGEf_GET(r) ((((r).autoneglpabil2[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_MESSAGE_PAGEf_SET(r,f) (r).autoneglpabil2[0]=(((r).autoneglpabil2[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_ACK2f_GET(r) ((((r).autoneglpabil2[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_ACK2f_SET(r,f) (r).autoneglpabil2[0]=(((r).autoneglpabil2[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_TOGGLEf_GET(r) ((((r).autoneglpabil2[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_TOGGLEf_SET(r,f) (r).autoneglpabil2[0]=(((r).autoneglpabil2[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_MESSAGEf_GET(r) (((r).autoneglpabil2[0]) & 0x7ff)
#define BCMI_VIPER_XGXS_AUTONEGLPABIL2r_MESSAGEf_SET(r,f) (r).autoneglpabil2[0]=(((r).autoneglpabil2[0] & ~((uint32_t)0x7ff)) | (((uint32_t)f) & 0x7ff)) | (0x7ff << 16)

/*
 * These macros can be used to access AUTONEGLPABIL2.
 *
 */
#define BCMI_VIPER_XGXS_READ_AUTONEGLPABIL2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGLPABIL2r,(_r._autoneglpabil2))
#define BCMI_VIPER_XGXS_WRITE_AUTONEGLPABIL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGLPABIL2r,(_r._autoneglpabil2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_AUTONEGLPABIL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGLPABIL2r,(_r._autoneglpabil2))
#define BCMI_VIPER_XGXS_READLN_AUTONEGLPABIL2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AUTONEGLPABIL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autoneglpabil2))
#define BCMI_VIPER_XGXS_WRITELN_AUTONEGLPABIL2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGLPABIL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._autoneglpabil2))
#define BCMI_VIPER_XGXS_WRITEALL_AUTONEGLPABIL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AUTONEGLPABIL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._autoneglpabil2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define AUTONEGLPABIL2r BCMI_VIPER_XGXS_AUTONEGLPABIL2r
#define AUTONEGLPABIL2r_SIZE BCMI_VIPER_XGXS_AUTONEGLPABIL2r_SIZE
typedef BCMI_VIPER_XGXS_AUTONEGLPABIL2r_t AUTONEGLPABIL2r_t;
#define AUTONEGLPABIL2r_CLR BCMI_VIPER_XGXS_AUTONEGLPABIL2r_CLR
#define AUTONEGLPABIL2r_SET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_SET
#define AUTONEGLPABIL2r_GET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_GET
#define AUTONEGLPABIL2r_NEXT_PAGEf_GET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_NEXT_PAGEf_GET
#define AUTONEGLPABIL2r_NEXT_PAGEf_SET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_NEXT_PAGEf_SET
#define AUTONEGLPABIL2r_ACKf_GET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_ACKf_GET
#define AUTONEGLPABIL2r_ACKf_SET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_ACKf_SET
#define AUTONEGLPABIL2r_MESSAGE_PAGEf_GET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_MESSAGE_PAGEf_GET
#define AUTONEGLPABIL2r_MESSAGE_PAGEf_SET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_MESSAGE_PAGEf_SET
#define AUTONEGLPABIL2r_ACK2f_GET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_ACK2f_GET
#define AUTONEGLPABIL2r_ACK2f_SET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_ACK2f_SET
#define AUTONEGLPABIL2r_TOGGLEf_GET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_TOGGLEf_GET
#define AUTONEGLPABIL2r_TOGGLEf_SET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_TOGGLEf_SET
#define AUTONEGLPABIL2r_MESSAGEf_GET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_MESSAGEf_GET
#define AUTONEGLPABIL2r_MESSAGEf_SET BCMI_VIPER_XGXS_AUTONEGLPABIL2r_MESSAGEf_SET
#define READ_AUTONEGLPABIL2r BCMI_VIPER_XGXS_READ_AUTONEGLPABIL2r
#define WRITE_AUTONEGLPABIL2r BCMI_VIPER_XGXS_WRITE_AUTONEGLPABIL2r
#define MODIFY_AUTONEGLPABIL2r BCMI_VIPER_XGXS_MODIFY_AUTONEGLPABIL2r
#define READLN_AUTONEGLPABIL2r BCMI_VIPER_XGXS_READLN_AUTONEGLPABIL2r
#define WRITELN_AUTONEGLPABIL2r BCMI_VIPER_XGXS_WRITELN_AUTONEGLPABIL2r
#define WRITEALL_AUTONEGLPABIL2r BCMI_VIPER_XGXS_WRITEALL_AUTONEGLPABIL2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_AUTONEGLPABIL2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  MIIEXTSTAT
 * BLOCKS:   CL22_B0
 * REGADDR:  0x000f
 * DESC:     IEEE MII extended status register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     S1000BASE_T_HALF_DUPLEX_CAPABLE 1 = 1000Base-T half duplex capable0 = not 1000Base-T half duplex capable
 *     S1000BASE_T_FULL_DUPLEX_CAPABLE 1 = 1000Base-T full duplex capable0 = not 1000Base-T full duplex capable
 *     S1000BASE_X_HALF_DUPLEX_CAPABLE 1 = 1000Base-X half duplex capable0 = not 1000Base-X half duplex capable
 *     S1000BASE_X_FULL_DUPLEX_CAPABLE 1 = 1000Base-X full duplex capable0 = not 1000Base-X full duplex capable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_MIIEXTSTATr (0x0000000f | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_MIIEXTSTATr_SIZE 4

/*
 * This structure should be used to declare and program MIIEXTSTAT.
 *
 */
typedef union BCMI_VIPER_XGXS_MIIEXTSTATr_s {
	uint32_t v[1];
	uint32_t miiextstat[1];
	uint32_t _miiextstat;
} BCMI_VIPER_XGXS_MIIEXTSTATr_t;

#define BCMI_VIPER_XGXS_MIIEXTSTATr_CLR(r) (r).miiextstat[0] = 0
#define BCMI_VIPER_XGXS_MIIEXTSTATr_SET(r,d) (r).miiextstat[0] = d
#define BCMI_VIPER_XGXS_MIIEXTSTATr_GET(r) (r).miiextstat[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).miiextstat[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).miiextstat[0]=(((r).miiextstat[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).miiextstat[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).miiextstat[0]=(((r).miiextstat[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).miiextstat[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).miiextstat[0]=(((r).miiextstat[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).miiextstat[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).miiextstat[0]=(((r).miiextstat[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))

/*
 * These macros can be used to access MIIEXTSTAT.
 *
 */
#define BCMI_VIPER_XGXS_READ_MIIEXTSTATr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MIIEXTSTATr,(_r._miiextstat))
#define BCMI_VIPER_XGXS_WRITE_MIIEXTSTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIIEXTSTATr,(_r._miiextstat)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_MIIEXTSTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIIEXTSTATr,(_r._miiextstat))
#define BCMI_VIPER_XGXS_READLN_MIIEXTSTATr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MIIEXTSTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._miiextstat))
#define BCMI_VIPER_XGXS_WRITELN_MIIEXTSTATr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIIEXTSTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._miiextstat))
#define BCMI_VIPER_XGXS_WRITEALL_MIIEXTSTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MIIEXTSTATr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._miiextstat))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define MIIEXTSTATr BCMI_VIPER_XGXS_MIIEXTSTATr
#define MIIEXTSTATr_SIZE BCMI_VIPER_XGXS_MIIEXTSTATr_SIZE
typedef BCMI_VIPER_XGXS_MIIEXTSTATr_t MIIEXTSTATr_t;
#define MIIEXTSTATr_CLR BCMI_VIPER_XGXS_MIIEXTSTATr_CLR
#define MIIEXTSTATr_SET BCMI_VIPER_XGXS_MIIEXTSTATr_SET
#define MIIEXTSTATr_GET BCMI_VIPER_XGXS_MIIEXTSTATr_GET
#define MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_GET
#define MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_SET
#define MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_GET
#define MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_SET
#define MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_GET
#define MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_SET
#define MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_GET
#define MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_SET
#define READ_MIIEXTSTATr BCMI_VIPER_XGXS_READ_MIIEXTSTATr
#define WRITE_MIIEXTSTATr BCMI_VIPER_XGXS_WRITE_MIIEXTSTATr
#define MODIFY_MIIEXTSTATr BCMI_VIPER_XGXS_MODIFY_MIIEXTSTATr
#define READLN_MIIEXTSTATr BCMI_VIPER_XGXS_READLN_MIIEXTSTATr
#define WRITELN_MIIEXTSTATr BCMI_VIPER_XGXS_WRITELN_MIIEXTSTATr
#define WRITEALL_MIIEXTSTATr BCMI_VIPER_XGXS_WRITEALL_MIIEXTSTATr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_MIIEXTSTATr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  XGXSCTL
 * BLOCKS:   BLK0
 * REGADDR:  0x8000
 * DESC:     XGXS control register
 * RESETVAL: 0x2c2f (11311)
 * ACCESS:   R/W
 * FIELDS:
 *     TXCKO_DIV        Selects txcko divide by 21 = txcko/20 = txcko.
 *     AFRST_EN         Automatic 10+G Tx input fifo reset enable
 *     EDEN             8B/10B encoder/decoder enable for 4-lane XGXS/XAUI modes
 *     CDET_EN          Comma detect enable
 *     MDIO_CONT_EN     Enable 1G MDIO controls
 *     RESERVED_5       
 *     RLOOP            Remote loop enable; i.e. wrap Rx output (rxdt[79:0]) into Tx input (tx_rxdt[79:0])
 *     PLL_BYPASS       1 = selects reference clock0 = selects clock used for clock compensation.
 *     MODE             NOTES:- noCC = Clock compensation turned off- noDsk = Deskew function turned off- noLss = LSS function turned off- OS5 = Over Sample by 5 to get 1G & 2.5G- ComboCoreMode = Serdes/UniCore mode; i.e. 10M, 100M, 1G, 2.5G, and autoneg to XGXS speeds (5G, 6G, 10G, 10G HiGig, 12G HiGig, 13G, etc.)Clocks are turned off for all unspecified modesOperational modes:Default is set by strap pins, mode_strap
 *     RESET_ANLG       Reset Analog
 *     START_SEQUENCER  1 = Enable Pll sequencer0 = reset Pll sequencer
 *     PCMP_EN          Pattern comparator enable
 *     PGEN_EN          Pattern generator enable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_XGXSCTLr (0x00008000 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_XGXSCTLr_SIZE 4

/*
 * This structure should be used to declare and program XGXSCTL.
 *
 */
typedef union BCMI_VIPER_XGXS_XGXSCTLr_s {
	uint32_t v[1];
	uint32_t xgxsctl[1];
	uint32_t _xgxsctl;
} BCMI_VIPER_XGXS_XGXSCTLr_t;

#define BCMI_VIPER_XGXS_XGXSCTLr_CLR(r) (r).xgxsctl[0] = 0
#define BCMI_VIPER_XGXS_XGXSCTLr_SET(r,d) (r).xgxsctl[0] = d
#define BCMI_VIPER_XGXS_XGXSCTLr_GET(r) (r).xgxsctl[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_XGXSCTLr_PGEN_ENf_GET(r) ((((r).xgxsctl[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_PGEN_ENf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_XGXSCTLr_PCMP_ENf_GET(r) ((((r).xgxsctl[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_PCMP_ENf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_XGXSCTLr_START_SEQUENCERf_GET(r) ((((r).xgxsctl[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_START_SEQUENCERf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_XGXSCTLr_RESET_ANLGf_GET(r) ((((r).xgxsctl[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_RESET_ANLGf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_XGXSCTLr_MODEf_GET(r) ((((r).xgxsctl[0]) >> 8) & 0xf)
#define BCMI_VIPER_XGXS_XGXSCTLr_MODEf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0xf << 8)) | ((((uint32_t)f) & 0xf) << 8)) | (15 << (16 + 8))
#define BCMI_VIPER_XGXS_XGXSCTLr_PLL_BYPASSf_GET(r) ((((r).xgxsctl[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_PLL_BYPASSf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_XGXSCTLr_RLOOPf_GET(r) ((((r).xgxsctl[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_RLOOPf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_XGXSCTLr_RESERVED_5f_GET(r) ((((r).xgxsctl[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_RESERVED_5f_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_XGXSCTLr_MDIO_CONT_ENf_GET(r) ((((r).xgxsctl[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_MDIO_CONT_ENf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_XGXSCTLr_CDET_ENf_GET(r) ((((r).xgxsctl[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_CDET_ENf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_XGXSCTLr_EDENf_GET(r) ((((r).xgxsctl[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_EDENf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_XGXSCTLr_AFRST_ENf_GET(r) ((((r).xgxsctl[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_AFRST_ENf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_XGXSCTLr_TXCKO_DIVf_GET(r) (((r).xgxsctl[0]) & 0x1)
#define BCMI_VIPER_XGXS_XGXSCTLr_TXCKO_DIVf_SET(r,f) (r).xgxsctl[0]=(((r).xgxsctl[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access XGXSCTL.
 *
 */
#define BCMI_VIPER_XGXS_READ_XGXSCTLr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_XGXSCTLr,(_r._xgxsctl))
#define BCMI_VIPER_XGXS_WRITE_XGXSCTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSCTLr,(_r._xgxsctl)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_XGXSCTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSCTLr,(_r._xgxsctl))
#define BCMI_VIPER_XGXS_READLN_XGXSCTLr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_XGXSCTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._xgxsctl))
#define BCMI_VIPER_XGXS_WRITELN_XGXSCTLr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSCTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._xgxsctl))
#define BCMI_VIPER_XGXS_WRITEALL_XGXSCTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSCTLr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._xgxsctl))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define XGXSCTLr BCMI_VIPER_XGXS_XGXSCTLr
#define XGXSCTLr_SIZE BCMI_VIPER_XGXS_XGXSCTLr_SIZE
typedef BCMI_VIPER_XGXS_XGXSCTLr_t XGXSCTLr_t;
#define XGXSCTLr_CLR BCMI_VIPER_XGXS_XGXSCTLr_CLR
#define XGXSCTLr_SET BCMI_VIPER_XGXS_XGXSCTLr_SET
#define XGXSCTLr_GET BCMI_VIPER_XGXS_XGXSCTLr_GET
#define XGXSCTLr_PGEN_ENf_GET BCMI_VIPER_XGXS_XGXSCTLr_PGEN_ENf_GET
#define XGXSCTLr_PGEN_ENf_SET BCMI_VIPER_XGXS_XGXSCTLr_PGEN_ENf_SET
#define XGXSCTLr_PCMP_ENf_GET BCMI_VIPER_XGXS_XGXSCTLr_PCMP_ENf_GET
#define XGXSCTLr_PCMP_ENf_SET BCMI_VIPER_XGXS_XGXSCTLr_PCMP_ENf_SET
#define XGXSCTLr_START_SEQUENCERf_GET BCMI_VIPER_XGXS_XGXSCTLr_START_SEQUENCERf_GET
#define XGXSCTLr_START_SEQUENCERf_SET BCMI_VIPER_XGXS_XGXSCTLr_START_SEQUENCERf_SET
#define XGXSCTLr_RESET_ANLGf_GET BCMI_VIPER_XGXS_XGXSCTLr_RESET_ANLGf_GET
#define XGXSCTLr_RESET_ANLGf_SET BCMI_VIPER_XGXS_XGXSCTLr_RESET_ANLGf_SET
#define XGXSCTLr_MODEf_GET BCMI_VIPER_XGXS_XGXSCTLr_MODEf_GET
#define XGXSCTLr_MODEf_SET BCMI_VIPER_XGXS_XGXSCTLr_MODEf_SET
#define XGXSCTLr_PLL_BYPASSf_GET BCMI_VIPER_XGXS_XGXSCTLr_PLL_BYPASSf_GET
#define XGXSCTLr_PLL_BYPASSf_SET BCMI_VIPER_XGXS_XGXSCTLr_PLL_BYPASSf_SET
#define XGXSCTLr_RLOOPf_GET BCMI_VIPER_XGXS_XGXSCTLr_RLOOPf_GET
#define XGXSCTLr_RLOOPf_SET BCMI_VIPER_XGXS_XGXSCTLr_RLOOPf_SET
#define XGXSCTLr_RESERVED_5f_GET BCMI_VIPER_XGXS_XGXSCTLr_RESERVED_5f_GET
#define XGXSCTLr_RESERVED_5f_SET BCMI_VIPER_XGXS_XGXSCTLr_RESERVED_5f_SET
#define XGXSCTLr_MDIO_CONT_ENf_GET BCMI_VIPER_XGXS_XGXSCTLr_MDIO_CONT_ENf_GET
#define XGXSCTLr_MDIO_CONT_ENf_SET BCMI_VIPER_XGXS_XGXSCTLr_MDIO_CONT_ENf_SET
#define XGXSCTLr_CDET_ENf_GET BCMI_VIPER_XGXS_XGXSCTLr_CDET_ENf_GET
#define XGXSCTLr_CDET_ENf_SET BCMI_VIPER_XGXS_XGXSCTLr_CDET_ENf_SET
#define XGXSCTLr_EDENf_GET BCMI_VIPER_XGXS_XGXSCTLr_EDENf_GET
#define XGXSCTLr_EDENf_SET BCMI_VIPER_XGXS_XGXSCTLr_EDENf_SET
#define XGXSCTLr_AFRST_ENf_GET BCMI_VIPER_XGXS_XGXSCTLr_AFRST_ENf_GET
#define XGXSCTLr_AFRST_ENf_SET BCMI_VIPER_XGXS_XGXSCTLr_AFRST_ENf_SET
#define XGXSCTLr_TXCKO_DIVf_GET BCMI_VIPER_XGXS_XGXSCTLr_TXCKO_DIVf_GET
#define XGXSCTLr_TXCKO_DIVf_SET BCMI_VIPER_XGXS_XGXSCTLr_TXCKO_DIVf_SET
#define READ_XGXSCTLr BCMI_VIPER_XGXS_READ_XGXSCTLr
#define WRITE_XGXSCTLr BCMI_VIPER_XGXS_WRITE_XGXSCTLr
#define MODIFY_XGXSCTLr BCMI_VIPER_XGXS_MODIFY_XGXSCTLr
#define READLN_XGXSCTLr BCMI_VIPER_XGXS_READLN_XGXSCTLr
#define WRITELN_XGXSCTLr BCMI_VIPER_XGXS_WRITELN_XGXSCTLr
#define WRITEALL_XGXSCTLr BCMI_VIPER_XGXS_WRITEALL_XGXSCTLr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_XGXSCTLr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  MMDSEL
 * BLOCKS:   BLK0
 * REGADDR:  0x800d
 * DESC:     MMD select register
 * RESETVAL: 0xf (15)
 * ACCESS:   R/W
 * FIELDS:
 *     DEVCL22_EN       When set and multiMMDs_en=1 then the CL22 combo core registers can be directly accessed through the MDIO serial data stream.
 *     DEVDEVAD_EN      When set and multiMMDs_en=1 then the top level XGXS registers as configured with DEVAD_STRAP can be directly accessed through the MDIO serial data stream.
 *     DEVPMD_EN        When set and multiMMDs_en=1 then the PMA/PMD device=1 registers can be directly accessed through the MDIO serial data stream.
 *     DEVAN_EN         When set and multiMMDs_en=1 then the CL73 AN device=7 registers can be directly accessed through the MDIO serial data stream.
 *     MULTIMMDS_EN     When set enables the multiple MMD functionality.  MD_ST is ignored and each device can be accessed directly with the appropriate CL22 or CL45 protocol.  The CL73 AN device would be accessed via a CL45 MDIO stream with devad=7 and the appropriate prtad.
 *     MULTIPRTS_EN     When set enables multiple prtad functionality.  Each of the lanes' MMDs can be accessed with consecutive PRTADs.  Lane 0 is accessed with PRTAD_STRAP, lane 1 with PRTAD_STRAP+1, lane 2 with PRTAD_STRAP+2 and lane 3 with PRTAD_STRAP+3.
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_MMDSELr (0x0000800d | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_MMDSELr_SIZE 4

/*
 * This structure should be used to declare and program MMDSEL.
 *
 */
typedef union BCMI_VIPER_XGXS_MMDSELr_s {
	uint32_t v[1];
	uint32_t mmdsel[1];
	uint32_t _mmdsel;
} BCMI_VIPER_XGXS_MMDSELr_t;

#define BCMI_VIPER_XGXS_MMDSELr_CLR(r) (r).mmdsel[0] = 0
#define BCMI_VIPER_XGXS_MMDSELr_SET(r,d) (r).mmdsel[0] = d
#define BCMI_VIPER_XGXS_MMDSELr_GET(r) (r).mmdsel[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_MMDSELr_MULTIPRTS_ENf_GET(r) ((((r).mmdsel[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_MMDSELr_MULTIPRTS_ENf_SET(r,f) (r).mmdsel[0]=(((r).mmdsel[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_MMDSELr_MULTIMMDS_ENf_GET(r) ((((r).mmdsel[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_MMDSELr_MULTIMMDS_ENf_SET(r,f) (r).mmdsel[0]=(((r).mmdsel[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_MMDSELr_DEVAN_ENf_GET(r) ((((r).mmdsel[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_MMDSELr_DEVAN_ENf_SET(r,f) (r).mmdsel[0]=(((r).mmdsel[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_MMDSELr_DEVPMD_ENf_GET(r) ((((r).mmdsel[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_MMDSELr_DEVPMD_ENf_SET(r,f) (r).mmdsel[0]=(((r).mmdsel[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_MMDSELr_DEVDEVAD_ENf_GET(r) ((((r).mmdsel[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_MMDSELr_DEVDEVAD_ENf_SET(r,f) (r).mmdsel[0]=(((r).mmdsel[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_MMDSELr_DEVCL22_ENf_GET(r) (((r).mmdsel[0]) & 0x1)
#define BCMI_VIPER_XGXS_MMDSELr_DEVCL22_ENf_SET(r,f) (r).mmdsel[0]=(((r).mmdsel[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access MMDSEL.
 *
 */
#define BCMI_VIPER_XGXS_READ_MMDSELr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MMDSELr,(_r._mmdsel))
#define BCMI_VIPER_XGXS_WRITE_MMDSELr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MMDSELr,(_r._mmdsel)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_MMDSELr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MMDSELr,(_r._mmdsel))
#define BCMI_VIPER_XGXS_READLN_MMDSELr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MMDSELr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._mmdsel))
#define BCMI_VIPER_XGXS_WRITELN_MMDSELr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MMDSELr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._mmdsel))
#define BCMI_VIPER_XGXS_WRITEALL_MMDSELr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MMDSELr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._mmdsel))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define MMDSELr BCMI_VIPER_XGXS_MMDSELr
#define MMDSELr_SIZE BCMI_VIPER_XGXS_MMDSELr_SIZE
typedef BCMI_VIPER_XGXS_MMDSELr_t MMDSELr_t;
#define MMDSELr_CLR BCMI_VIPER_XGXS_MMDSELr_CLR
#define MMDSELr_SET BCMI_VIPER_XGXS_MMDSELr_SET
#define MMDSELr_GET BCMI_VIPER_XGXS_MMDSELr_GET
#define MMDSELr_MULTIPRTS_ENf_GET BCMI_VIPER_XGXS_MMDSELr_MULTIPRTS_ENf_GET
#define MMDSELr_MULTIPRTS_ENf_SET BCMI_VIPER_XGXS_MMDSELr_MULTIPRTS_ENf_SET
#define MMDSELr_MULTIMMDS_ENf_GET BCMI_VIPER_XGXS_MMDSELr_MULTIMMDS_ENf_GET
#define MMDSELr_MULTIMMDS_ENf_SET BCMI_VIPER_XGXS_MMDSELr_MULTIMMDS_ENf_SET
#define MMDSELr_DEVAN_ENf_GET BCMI_VIPER_XGXS_MMDSELr_DEVAN_ENf_GET
#define MMDSELr_DEVAN_ENf_SET BCMI_VIPER_XGXS_MMDSELr_DEVAN_ENf_SET
#define MMDSELr_DEVPMD_ENf_GET BCMI_VIPER_XGXS_MMDSELr_DEVPMD_ENf_GET
#define MMDSELr_DEVPMD_ENf_SET BCMI_VIPER_XGXS_MMDSELr_DEVPMD_ENf_SET
#define MMDSELr_DEVDEVAD_ENf_GET BCMI_VIPER_XGXS_MMDSELr_DEVDEVAD_ENf_GET
#define MMDSELr_DEVDEVAD_ENf_SET BCMI_VIPER_XGXS_MMDSELr_DEVDEVAD_ENf_SET
#define MMDSELr_DEVCL22_ENf_GET BCMI_VIPER_XGXS_MMDSELr_DEVCL22_ENf_GET
#define MMDSELr_DEVCL22_ENf_SET BCMI_VIPER_XGXS_MMDSELr_DEVCL22_ENf_SET
#define READ_MMDSELr BCMI_VIPER_XGXS_READ_MMDSELr
#define WRITE_MMDSELr BCMI_VIPER_XGXS_WRITE_MMDSELr
#define MODIFY_MMDSELr BCMI_VIPER_XGXS_MODIFY_MMDSELr
#define READLN_MMDSELr BCMI_VIPER_XGXS_READLN_MMDSELr
#define WRITELN_MMDSELr BCMI_VIPER_XGXS_WRITELN_MMDSELr
#define WRITEALL_MMDSELr BCMI_VIPER_XGXS_WRITEALL_MMDSELr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_MMDSELr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  MISCCTL1
 * BLOCKS:   BLK0
 * REGADDR:  0x800e
 * DESC:     Miscellaneous control 1 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     IEEE_BLKSEL_VAL  1 = Enable 802.3ae (XAUI) IEEE address space0 = Enable 1000-BaseX (1.25G Serdes) Combo core IEEE address space (mode_10g_strap != 4'hC)
 *     IEEE_BLKSEL_AUTODET 1 = Enable IEEE address space to be dynamically enabled as a function of the resolved rate.0 = Enable manual control of IEEE address space mapping (via ieee_blksel_val).
 *     INVERT_RX_SIGDET 1 = Invert Rx signal detect0 = Do not invert Rx Signal detect
 *     PARDET10G_PWRDNLINK_EN 1 = Causes powerdown on unused Rx lanes when link_status is acquired0 = Does not powerdown unused Rx lanes when link_status is acquired
 *     FORCE_DIV5_FOR_LXCK25 1 = Enable force refclk div/5 for clk25 derivation0 = Disable force refclk div/5 for clk25 derivation
 *     LATCH_LINKDOWN_ENABLE 1 = Enable latch linkdown function.  When enabled this causes the Unicore datapath's to be kept in a reset state upon the link-down transition until the clear_linkdown control is toggled.0 = Disable latch linkdown function.
 *     CLEAR_LINKDOWN   1 = Clear the latch_linkdown latch0 = Do not clear the latch_linkdown latch
 *     PMD_DEV_EN_OVERRIDE 1 = Enables PMA/PMD registers in cl22 mode
 *     PCS_DEV_EN_OVERRIDE 1 = Enables PCS XS registers in cl22 mode
 *     GLOBAL_PMD_TX_DISABLE 1 = Disable all transmitter outputs0 = Enable all transmitter outputs
 *     PMD_LANE0_TX_DISABLE 1 = Disable output on transmit lane 00 = Enable output on transmit lane 0
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_MISCCTL1r (0x0000800e | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_MISCCTL1r_SIZE 4

/*
 * This structure should be used to declare and program MISCCTL1.
 *
 */
typedef union BCMI_VIPER_XGXS_MISCCTL1r_s {
	uint32_t v[1];
	uint32_t miscctl1[1];
	uint32_t _miscctl1;
} BCMI_VIPER_XGXS_MISCCTL1r_t;

#define BCMI_VIPER_XGXS_MISCCTL1r_CLR(r) (r).miscctl1[0] = 0
#define BCMI_VIPER_XGXS_MISCCTL1r_SET(r,d) (r).miscctl1[0] = d
#define BCMI_VIPER_XGXS_MISCCTL1r_GET(r) (r).miscctl1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_MISCCTL1r_PMD_LANE0_TX_DISABLEf_GET(r) ((((r).miscctl1[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_PMD_LANE0_TX_DISABLEf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_MISCCTL1r_GLOBAL_PMD_TX_DISABLEf_GET(r) ((((r).miscctl1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_GLOBAL_PMD_TX_DISABLEf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_MISCCTL1r_PCS_DEV_EN_OVERRIDEf_GET(r) ((((r).miscctl1[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_PCS_DEV_EN_OVERRIDEf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_MISCCTL1r_PMD_DEV_EN_OVERRIDEf_GET(r) ((((r).miscctl1[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_PMD_DEV_EN_OVERRIDEf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_MISCCTL1r_CLEAR_LINKDOWNf_GET(r) ((((r).miscctl1[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_CLEAR_LINKDOWNf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_MISCCTL1r_LATCH_LINKDOWN_ENABLEf_GET(r) ((((r).miscctl1[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_LATCH_LINKDOWN_ENABLEf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_MISCCTL1r_FORCE_DIV5_FOR_LXCK25f_GET(r) ((((r).miscctl1[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_FORCE_DIV5_FOR_LXCK25f_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_MISCCTL1r_PARDET10G_PWRDNLINK_ENf_GET(r) ((((r).miscctl1[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_PARDET10G_PWRDNLINK_ENf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_MISCCTL1r_INVERT_RX_SIGDETf_GET(r) ((((r).miscctl1[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_INVERT_RX_SIGDETf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_MISCCTL1r_IEEE_BLKSEL_AUTODETf_GET(r) ((((r).miscctl1[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_IEEE_BLKSEL_AUTODETf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_MISCCTL1r_IEEE_BLKSEL_VALf_GET(r) (((r).miscctl1[0]) & 0x1)
#define BCMI_VIPER_XGXS_MISCCTL1r_IEEE_BLKSEL_VALf_SET(r,f) (r).miscctl1[0]=(((r).miscctl1[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access MISCCTL1.
 *
 */
#define BCMI_VIPER_XGXS_READ_MISCCTL1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MISCCTL1r,(_r._miscctl1))
#define BCMI_VIPER_XGXS_WRITE_MISCCTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MISCCTL1r,(_r._miscctl1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_MISCCTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MISCCTL1r,(_r._miscctl1))
#define BCMI_VIPER_XGXS_READLN_MISCCTL1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_MISCCTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._miscctl1))
#define BCMI_VIPER_XGXS_WRITELN_MISCCTL1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MISCCTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._miscctl1))
#define BCMI_VIPER_XGXS_WRITEALL_MISCCTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_MISCCTL1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._miscctl1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define MISCCTL1r BCMI_VIPER_XGXS_MISCCTL1r
#define MISCCTL1r_SIZE BCMI_VIPER_XGXS_MISCCTL1r_SIZE
typedef BCMI_VIPER_XGXS_MISCCTL1r_t MISCCTL1r_t;
#define MISCCTL1r_CLR BCMI_VIPER_XGXS_MISCCTL1r_CLR
#define MISCCTL1r_SET BCMI_VIPER_XGXS_MISCCTL1r_SET
#define MISCCTL1r_GET BCMI_VIPER_XGXS_MISCCTL1r_GET
#define MISCCTL1r_PMD_LANE0_TX_DISABLEf_GET BCMI_VIPER_XGXS_MISCCTL1r_PMD_LANE0_TX_DISABLEf_GET
#define MISCCTL1r_PMD_LANE0_TX_DISABLEf_SET BCMI_VIPER_XGXS_MISCCTL1r_PMD_LANE0_TX_DISABLEf_SET
#define MISCCTL1r_GLOBAL_PMD_TX_DISABLEf_GET BCMI_VIPER_XGXS_MISCCTL1r_GLOBAL_PMD_TX_DISABLEf_GET
#define MISCCTL1r_GLOBAL_PMD_TX_DISABLEf_SET BCMI_VIPER_XGXS_MISCCTL1r_GLOBAL_PMD_TX_DISABLEf_SET
#define MISCCTL1r_PCS_DEV_EN_OVERRIDEf_GET BCMI_VIPER_XGXS_MISCCTL1r_PCS_DEV_EN_OVERRIDEf_GET
#define MISCCTL1r_PCS_DEV_EN_OVERRIDEf_SET BCMI_VIPER_XGXS_MISCCTL1r_PCS_DEV_EN_OVERRIDEf_SET
#define MISCCTL1r_PMD_DEV_EN_OVERRIDEf_GET BCMI_VIPER_XGXS_MISCCTL1r_PMD_DEV_EN_OVERRIDEf_GET
#define MISCCTL1r_PMD_DEV_EN_OVERRIDEf_SET BCMI_VIPER_XGXS_MISCCTL1r_PMD_DEV_EN_OVERRIDEf_SET
#define MISCCTL1r_CLEAR_LINKDOWNf_GET BCMI_VIPER_XGXS_MISCCTL1r_CLEAR_LINKDOWNf_GET
#define MISCCTL1r_CLEAR_LINKDOWNf_SET BCMI_VIPER_XGXS_MISCCTL1r_CLEAR_LINKDOWNf_SET
#define MISCCTL1r_LATCH_LINKDOWN_ENABLEf_GET BCMI_VIPER_XGXS_MISCCTL1r_LATCH_LINKDOWN_ENABLEf_GET
#define MISCCTL1r_LATCH_LINKDOWN_ENABLEf_SET BCMI_VIPER_XGXS_MISCCTL1r_LATCH_LINKDOWN_ENABLEf_SET
#define MISCCTL1r_FORCE_DIV5_FOR_LXCK25f_GET BCMI_VIPER_XGXS_MISCCTL1r_FORCE_DIV5_FOR_LXCK25f_GET
#define MISCCTL1r_FORCE_DIV5_FOR_LXCK25f_SET BCMI_VIPER_XGXS_MISCCTL1r_FORCE_DIV5_FOR_LXCK25f_SET
#define MISCCTL1r_PARDET10G_PWRDNLINK_ENf_GET BCMI_VIPER_XGXS_MISCCTL1r_PARDET10G_PWRDNLINK_ENf_GET
#define MISCCTL1r_PARDET10G_PWRDNLINK_ENf_SET BCMI_VIPER_XGXS_MISCCTL1r_PARDET10G_PWRDNLINK_ENf_SET
#define MISCCTL1r_INVERT_RX_SIGDETf_GET BCMI_VIPER_XGXS_MISCCTL1r_INVERT_RX_SIGDETf_GET
#define MISCCTL1r_INVERT_RX_SIGDETf_SET BCMI_VIPER_XGXS_MISCCTL1r_INVERT_RX_SIGDETf_SET
#define MISCCTL1r_IEEE_BLKSEL_AUTODETf_GET BCMI_VIPER_XGXS_MISCCTL1r_IEEE_BLKSEL_AUTODETf_GET
#define MISCCTL1r_IEEE_BLKSEL_AUTODETf_SET BCMI_VIPER_XGXS_MISCCTL1r_IEEE_BLKSEL_AUTODETf_SET
#define MISCCTL1r_IEEE_BLKSEL_VALf_GET BCMI_VIPER_XGXS_MISCCTL1r_IEEE_BLKSEL_VALf_GET
#define MISCCTL1r_IEEE_BLKSEL_VALf_SET BCMI_VIPER_XGXS_MISCCTL1r_IEEE_BLKSEL_VALf_SET
#define READ_MISCCTL1r BCMI_VIPER_XGXS_READ_MISCCTL1r
#define WRITE_MISCCTL1r BCMI_VIPER_XGXS_WRITE_MISCCTL1r
#define MODIFY_MISCCTL1r BCMI_VIPER_XGXS_MODIFY_MISCCTL1r
#define READLN_MISCCTL1r BCMI_VIPER_XGXS_READLN_MISCCTL1r
#define WRITELN_MISCCTL1r BCMI_VIPER_XGXS_WRITELN_MISCCTL1r
#define WRITEALL_MISCCTL1r BCMI_VIPER_XGXS_WRITEALL_MISCCTL1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_MISCCTL1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  BLKADDR0
 * BLOCKS:   BLK0
 * REGADDR:  0x800f
 * DESC:     Block Address register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     BLOCKADDRESS     Bits 14:4 of the address
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_BLKADDR0r (0x0000800f | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_BLKADDR0r_SIZE 4

/*
 * This structure should be used to declare and program BLKADDR0.
 *
 */
typedef union BCMI_VIPER_XGXS_BLKADDR0r_s {
	uint32_t v[1];
	uint32_t blkaddr0[1];
	uint32_t _blkaddr0;
} BCMI_VIPER_XGXS_BLKADDR0r_t;

#define BCMI_VIPER_XGXS_BLKADDR0r_CLR(r) (r).blkaddr0[0] = 0
#define BCMI_VIPER_XGXS_BLKADDR0r_SET(r,d) (r).blkaddr0[0] = d
#define BCMI_VIPER_XGXS_BLKADDR0r_GET(r) (r).blkaddr0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_BLKADDR0r_BLOCKADDRESSf_GET(r) ((((r).blkaddr0[0]) >> 4) & 0x7ff)
#define BCMI_VIPER_XGXS_BLKADDR0r_BLOCKADDRESSf_SET(r,f) (r).blkaddr0[0]=(((r).blkaddr0[0] & ~((uint32_t)0x7ff << 4)) | ((((uint32_t)f) & 0x7ff) << 4)) | (2047 << (16 + 4))

/*
 * These macros can be used to access BLKADDR0.
 *
 */
#define BCMI_VIPER_XGXS_READ_BLKADDR0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_BLKADDR0r,(_r._blkaddr0))
#define BCMI_VIPER_XGXS_WRITE_BLKADDR0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_BLKADDR0r,(_r._blkaddr0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_BLKADDR0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_BLKADDR0r,(_r._blkaddr0))
#define BCMI_VIPER_XGXS_READLN_BLKADDR0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_BLKADDR0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._blkaddr0))
#define BCMI_VIPER_XGXS_WRITELN_BLKADDR0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_BLKADDR0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._blkaddr0))
#define BCMI_VIPER_XGXS_WRITEALL_BLKADDR0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_BLKADDR0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._blkaddr0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define BLKADDR0r BCMI_VIPER_XGXS_BLKADDR0r
#define BLKADDR0r_SIZE BCMI_VIPER_XGXS_BLKADDR0r_SIZE
typedef BCMI_VIPER_XGXS_BLKADDR0r_t BLKADDR0r_t;
#define BLKADDR0r_CLR BCMI_VIPER_XGXS_BLKADDR0r_CLR
#define BLKADDR0r_SET BCMI_VIPER_XGXS_BLKADDR0r_SET
#define BLKADDR0r_GET BCMI_VIPER_XGXS_BLKADDR0r_GET
#define BLKADDR0r_BLOCKADDRESSf_GET BCMI_VIPER_XGXS_BLKADDR0r_BLOCKADDRESSf_GET
#define BLKADDR0r_BLOCKADDRESSf_SET BCMI_VIPER_XGXS_BLKADDR0r_BLOCKADDRESSf_SET
#define READ_BLKADDR0r BCMI_VIPER_XGXS_READ_BLKADDR0r
#define WRITE_BLKADDR0r BCMI_VIPER_XGXS_WRITE_BLKADDR0r
#define MODIFY_BLKADDR0r BCMI_VIPER_XGXS_MODIFY_BLKADDR0r
#define READLN_BLKADDR0r BCMI_VIPER_XGXS_READLN_BLKADDR0r
#define WRITELN_BLKADDR0r BCMI_VIPER_XGXS_WRITELN_BLKADDR0r
#define WRITEALL_BLKADDR0r BCMI_VIPER_XGXS_WRITEALL_BLKADDR0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_BLKADDR0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  LANECTL0
 * BLOCKS:   BLK1
 * REGADDR:  0x8015
 * DESC:     Lane control 0 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     CL36_PCS_EN_TX        Enable IEEE802.3 Clause36 pcs for TX lane 3:0  
 *     CL36_PCS_EN_RX        Enable IEEE802.3 Clause36 pcs for RX lane 3:0  
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_LANECTL0r (0x00008015 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_LANECTL0r_SIZE 4

/*
 * This structure should be used to declare and program LANECTL0.
 *
 */
typedef union BCMI_VIPER_XGXS_LANECTL0r_s {
	uint32_t v[1];
	uint32_t lanectl0[1];
	uint32_t _lanectl0;
} BCMI_VIPER_XGXS_LANECTL0r_t;

#define BCMI_VIPER_XGXS_LANECTL0r_CLR(r) (r).lanectl0[0] = 0
#define BCMI_VIPER_XGXS_LANECTL0r_SET(r,d) (r).lanectl0[0] = d
#define BCMI_VIPER_XGXS_LANECTL0r_GET(r) (r).lanectl0[0]

/*
 * These macros can be used to access individual fields.
 *
 */

#define BCMI_VIPER_XGXS_LANECTL0r_RESERVED_15_08f_GET(r) ((((r).lanectl0[0]) >> 8) & 0xff)
#define BCMI_VIPER_XGXS_LANECTL0r_RESERVED_15_08f_SET(r,f) (r).lanectl0[0]=(((r).lanectl0[0] & ~((uint32_t)0xff << 8)) | ((((uint32_t)f) & 0xff) << 8)) | (0xff << (16 + 8))
#define BCMI_VIPER_XGXS_LANECTL0r_CL36_PCS_EN_RXf_GET(r) ((((r).lanectl0[0]) >> 4) & 0xf)
#define BCMI_VIPER_XGXS_LANECTL0r_CL36_PCS_EN_RXf_SET(r,f) (r).lanectl0[0]=(((r).lanectl0[0] & ~((uint32_t)0xf << 4)) | ((((uint32_t)f) & 0xf) << 4)) | (0xf << (16 + 4))
#define BCMI_VIPER_XGXS_LANECTL0r_CL36_PCS_EN_TXf_GET(r) (((r).lanectl0[0]) & 0xf)
#define BCMI_VIPER_XGXS_LANECTL0r_CL36_PCS_EN_TXf_SET(r,f) (r).lanectl0[0]=(((r).lanectl0[0] & ~((uint32_t)0xf)) | (((uint32_t)f) & 0xf)) | (0xf << 16)


/*
 * These macros can be used to access LANECTL0.
 *
 */
#define BCMI_VIPER_XGXS_READ_LANECTL0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANECTL0r,(_r._lanectl0))
#define BCMI_VIPER_XGXS_WRITE_LANECTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL0r,(_r._lanectl0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_LANECTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL0r,(_r._lanectl0))
#define BCMI_VIPER_XGXS_READLN_LANECTL0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANECTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanectl0))
#define BCMI_VIPER_XGXS_WRITELN_LANECTL0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanectl0))
#define BCMI_VIPER_XGXS_WRITEALL_LANECTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._lanectl0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define LANECTL0r BCMI_VIPER_XGXS_LANECTL0r
#define LANECTL0r_SIZE BCMI_VIPER_XGXS_LANECTL0r_SIZE
typedef BCMI_VIPER_XGXS_LANECTL0r_t LANECTL0r_t;
#define LANECTL0r_CLR BCMI_VIPER_XGXS_LANECTL0r_CLR
#define LANECTL0r_SET BCMI_VIPER_XGXS_LANECTL0r_SET
#define LANECTL0r_GET BCMI_VIPER_XGXS_LANECTL0r_GET
#define LANECTL0r_CL36_PCS_EN_RXf_GET BCMI_VIPER_XGXS_LANECTL0r_CL36_PCS_EN_RXf_GET
#define LANECTL0r_CL36_PCS_EN_RXf_SET BCMI_VIPER_XGXS_LANECTL0r_CL36_PCS_EN_RXf_SET
#define LANECTL0r_CL36_PCS_EN_TXf_GET BCMI_VIPER_XGXS_LANECTL0r_CL36_PCS_EN_TXf_GET
#define LANECTL0r_CL36_PCS_EN_TXf_SET BCMI_VIPER_XGXS_LANECTL0r_CL36_PCS_EN_TXf_SET
#define READ_LANECTL0r BCMI_VIPER_XGXS_READ_LANECTL0r
#define WRITE_LANECTL0r BCMI_VIPER_XGXS_WRITE_LANECTL0r
#define MODIFY_LANECTL0r BCMI_VIPER_XGXS_MODIFY_LANECTL0r
#define READLN_LANECTL0r BCMI_VIPER_XGXS_READLN_LANECTL0r
#define WRITELN_LANECTL0r BCMI_VIPER_XGXS_WRITELN_LANECTL0r
#define WRITEALL_LANECTL0r BCMI_VIPER_XGXS_WRITEALL_LANECTL0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_LANECTL0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  LANECTL1
 * BLOCKS:   BLK1
 * REGADDR:  0x8016
 * DESC:     Lane control 1 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TX1G_MODE_LN0          Independent Lane 0 TX mode 
 *     TX1G_MODE_LN1          Independent Lane 1 TX mode 
 *     TX1G_MODE_LN2          Independent Lane 2 TX mode 
 *     TX1G_MODE_LN3          Independent Lane 3 TX mode 
 *     RX1G_MODE_LN0          Independent Lane 0 RX mode 
 *     RX1G_MODE_LN1          Independent Lane 1 RX mode 
 *     RX1G_MODE_LN2          Independent Lane 2 RX mode 
 *     RX1G_MODE_LN3          Independent Lane 3 RX mode 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_LANECTL1r (0x00008016 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_LANECTL1r_SIZE 4

/*
 * This structure should be used to declare and program LANECTL1.
 *
 */
typedef union BCMI_VIPER_XGXS_LANECTL1r_s {
	uint32_t v[1];
	uint32_t lanectl1[1];
	uint32_t _lanectl1;
} BCMI_VIPER_XGXS_LANECTL1r_t;

#define BCMI_VIPER_XGXS_LANECTL1r_CLR(r) (r).lanectl1[0] = 0
#define BCMI_VIPER_XGXS_LANECTL1r_SET(r,d) (r).lanectl1[0] = d
#define BCMI_VIPER_XGXS_LANECTL1r_GET(r) (r).lanectl1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN3f_GET(r) ((((r).lanectl1[0]) >> 14) & 0x3)
#define BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN3f_SET(r,f) (r).lanectl1[0]=(((r).lanectl1[0] & ~((uint32_t)0x3 << 14)) | ((((uint32_t)f) & 0x3) << 14)) | (3 << (16 + 14) )
#define BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN2f_GET(r) ((((r).lanectl1[0]) >> 12) & 0x3)
#define BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN2f_SET(r,f) (r).lanectl1[0]=(((r).lanectl1[0] & ~((uint32_t)0x3 << 12)) | ((((uint32_t)f) & 0x3) << 12)) | (3 << (16 + 12) )
#define BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN1f_GET(r) ((((r).lanectl1[0]) >> 10) & 0x3)
#define BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN1f_SET(r,f) (r).lanectl1[0]=(((r).lanectl1[0] & ~((uint32_t)0x3 << 10)) | ((((uint32_t)f) & 0x3) << 10)) | (3 << (16 + 10) )
#define BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN0f_GET(r) ((((r).lanectl1[0]) >> 8) & 0x3)
#define BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN0f_SET(r,f) (r).lanectl1[0]=(((r).lanectl1[0] & ~((uint32_t)0x3 << 8)) | ((((uint32_t)f) & 0x3) << 8)) | (3 << (16 + 8) )
#define BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN3f_GET(r) ((((r).lanectl1[0]) >> 6) & 0x3)
#define BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN3f_SET(r,f) (r).lanectl1[0]=(((r).lanectl1[0] & ~((uint32_t)0x3 << 6)) | ((((uint32_t)f) & 0x3) << 6)) | (3 << (16 + 6) )
#define BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN2f_GET(r) ((((r).lanectl1[0]) >> 4) & 0x3)
#define BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN2f_SET(r,f) (r).lanectl1[0]=(((r).lanectl1[0] & ~((uint32_t)0x3 << 4)) | ((((uint32_t)f) & 0x3) << 4)) | (3 << (16 + 4) )
#define BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN1f_GET(r) ((((r).lanectl1[0]) >> 2) & 0x3)
#define BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN1f_SET(r,f) (r).lanectl1[0]=(((r).lanectl1[0] & ~((uint32_t)0x3 << 2)) | ((((uint32_t)f) & 0x3) << 2)) | (3 << (16 + 2) )
#define BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN0f_GET(r) (((r).lanectl1[0]) & 0x3)
#define BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN0f_SET(r,f) (r).lanectl1[0]=(((r).lanectl1[0] & ~((uint32_t)0x3)) | (((uint32_t)f) & 0x3)) | (0x3 << 16)

/*
 * These macros can be used to access LANECTL1.
 *
 */
#define BCMI_VIPER_XGXS_READ_LANECTL1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANECTL1r,(_r._lanectl1))
#define BCMI_VIPER_XGXS_WRITE_LANECTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL1r,(_r._lanectl1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_LANECTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL1r,(_r._lanectl1))
#define BCMI_VIPER_XGXS_READLN_LANECTL1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANECTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanectl1))
#define BCMI_VIPER_XGXS_WRITELN_LANECTL1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanectl1))
#define BCMI_VIPER_XGXS_WRITEALL_LANECTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._lanectl1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define LANECTL1r BCMI_VIPER_XGXS_LANECTL1r
#define LANECTL1r_SIZE BCMI_VIPER_XGXS_LANECTL1r_SIZE
typedef BCMI_VIPER_XGXS_LANECTL1r_t LANECTL1r_t;
#define LANECTL1r_CLR BCMI_VIPER_XGXS_LANECTL1r_CLR
#define LANECTL1r_SET BCMI_VIPER_XGXS_LANECTL1r_SET
#define LANECTL1r_GET BCMI_VIPER_XGXS_LANECTL1r_GET
#define LANECTL1r_RX1G_MODE_LN3f_GET BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN3f_GET
#define LANECTL1r_RX1G_MODE_LN3f_SET BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN3f_SET
#define LANECTL1r_RX1G_MODE_LN2f_GET BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN3f_GET
#define LANECTL1r_RX1G_MODE_LN2f_SET BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN3f_SET
#define LANECTL1r_RX1G_MODE_LN1f_GET BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN3f_GET
#define LANECTL1r_RX1G_MODE_LN1f_SET BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN3f_SET
#define LANECTL1r_RX1G_MODE_LN0f_GET BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN0f_GET
#define LANECTL1r_RX1G_MODE_LN0f_SET BCMI_VIPER_XGXS_LANECTL1r_RX1G_MODE_LN0f_SET
#define LANECTL1r_TX1G_MODE_LN3f_GET BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN3f_GET
#define LANECTL1r_TX1G_MODE_LN3f_SET BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN3f_SET
#define LANECTL1r_TX1G_MODE_LN2f_GET BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN3f_GET
#define LANECTL1r_TX1G_MODE_LN2f_SET BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN3f_SET
#define LANECTL1r_TX1G_MODE_LN1f_GET BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN3f_GET
#define LANECTL1r_TX1G_MODE_LN1f_SET BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN3f_SET
#define LANECTL1r_TX1G_MODE_LN0f_GET BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN0f_GET
#define LANECTL1r_TX1G_MODE_LN0f_SET BCMI_VIPER_XGXS_LANECTL1r_TX1G_MODE_LN0f_SET
#define READ_LANECTL1r BCMI_VIPER_XGXS_READ_LANECTL1r
#define WRITE_LANECTL1r BCMI_VIPER_XGXS_WRITE_LANECTL1r
#define MODIFY_LANECTL1r BCMI_VIPER_XGXS_MODIFY_LANECTL1r
#define READLN_LANECTL1r BCMI_VIPER_XGXS_READLN_LANECTL1r
#define WRITELN_LANECTL1r BCMI_VIPER_XGXS_WRITELN_LANECTL1r
#define WRITEALL_LANECTL1r BCMI_VIPER_XGXS_WRITEALL_LANECTL1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_LANECTL1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  LANECTL2
 * BLOCKS:   BLK1
 * REGADDR:  0x8017
 * DESC:     Lane control 2 register
 * RESETVAL: 0xff00 (65280)
 * ACCESS:   R/W
 * FIELDS:
 *     GLOOP1G          1G global loopback (Tx-> Rx), lane 3:0 
 *     RLOOP1G          1G global loopback (Tx-> Rx), lane 3:0 
 *     EDEN1G           1G 8B/ 10B enable, lane 3:0 
 *     CDET_EN1G        1G comma detect enable, lane 3:0 
 *     RESERVED_15_13   
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_LANECTL2r (0x00008017 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_LANECTL2r_SIZE 4

/*
 * This structure should be used to declare and program LANECTL2.
 *
 */
typedef union BCMI_VIPER_XGXS_LANECTL2r_s {
	uint32_t v[1];
	uint32_t lanectl2[1];
	uint32_t _lanectl2;
} BCMI_VIPER_XGXS_LANECTL2r_t;

#define BCMI_VIPER_XGXS_LANECTL2r_CLR(r) (r).lanectl2[0] = 0
#define BCMI_VIPER_XGXS_LANECTL2r_SET(r,d) (r).lanectl2[0] = d
#define BCMI_VIPER_XGXS_LANECTL2r_GET(r) (r).lanectl2[0]

/*
 * These macros can be used to access individual fields.
 *
 */

#define BCMI_VIPER_XGXS_LANECTL2r_CDET_EN1Gf_GET(r) ((((r).lanectl2[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_LANECTL2r_CDET_EN1Gf_SET(r,f) (r).lanectl2[0]=(((r).lanectl2[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (0xf << (16 + 12))
#define BCMI_VIPER_XGXS_LANECTL2r_EDEN1Gf_GET(r) ((((r).lanectl2[0]) >> 8) & 0xf)
#define BCMI_VIPER_XGXS_LANECTL2r_EDEN1Gf_SET(r,f) (r).lanectl2[0]=(((r).lanectl2[0] & ~((uint32_t)0xf << 8)) | ((((uint32_t)f) & 0xf) << 8)) | (0xf << (16 + 8))
#define BCMI_VIPER_XGXS_LANECTL2r_RLOOP1Gf_GET(r) ((((r).lanectl2[0]) >> 4 )& 0xf)
#define BCMI_VIPER_XGXS_LANECTL2r_RLOOP1Gf_SET(r,f) (r).lanectl2[0]=(((r).lanectl2[0] & ~((uint32_t)0xf << 4)) | ((((uint32_t)f) & 0xf) << 4)) | (0xf << (16 + 4))
#define BCMI_VIPER_XGXS_LANECTL2r_GLOOP1Gf_GET(r) (((r).lanectl2[0]) & 0xf)
#define BCMI_VIPER_XGXS_LANECTL2r_GLOOP1Gf_SET(r,f) (r).lanectl2[0]=(((r).lanectl2[0] & ~((uint32_t)0xf)) | (((uint32_t)f) & 0xf)) | (0xf << 16)

/*
 * These macros can be used to access LANECTL2.
 *
 */
#define BCMI_VIPER_XGXS_READ_LANECTL2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANECTL2r,(_r._lanectl2))
#define BCMI_VIPER_XGXS_WRITE_LANECTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL2r,(_r._lanectl2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_LANECTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL2r,(_r._lanectl2))
#define BCMI_VIPER_XGXS_READLN_LANECTL2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANECTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanectl2))
#define BCMI_VIPER_XGXS_WRITELN_LANECTL2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanectl2))
#define BCMI_VIPER_XGXS_WRITEALL_LANECTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._lanectl2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define LANECTL2r BCMI_VIPER_XGXS_LANECTL2r
#define LANECTL2r_SIZE BCMI_VIPER_XGXS_LANECTL2r_SIZE
typedef BCMI_VIPER_XGXS_LANECTL2r_t LANECTL2r_t;
#define LANECTL2r_CLR BCMI_VIPER_XGXS_LANECTL2r_CLR
#define LANECTL2r_SET BCMI_VIPER_XGXS_LANECTL2r_SET
#define LANECTL2r_GET BCMI_VIPER_XGXS_LANECTL2r_GET
#define LANECTL2r_CDET_EN1Gf_GET BCMI_VIPER_XGXS_LANECTL2r_CDET_EN1Gf_GET
#define LANECTL2r_CDET_EN1Gf_SET BCMI_VIPER_XGXS_LANECTL2r_CDET_EN1Gf_SET
#define LANECTL2r_EDEN1Gf_GET BCMI_VIPER_XGXS_LANECTL2r_EDEN1Gf_GET
#define LANECTL2r_EDEN1Gf_SET BCMI_VIPER_XGXS_LANECTL2r_EDEN1Gf_SET
#define LANECTL2r_RLOOP1Gf_GET BCMI_VIPER_XGXS_LANECTL2r_RLOOP1Gf_GET
#define LANECTL2r_RLOOP1Gf_SET BCMI_VIPER_XGXS_LANECTL2r_RLOOP1Gf_SET
#define LANECTL2r_GLOOP1Gf_GET BCMI_VIPER_XGXS_LANECTL2r_GLOOP1Gf_GET
#define LANECTL2r_GLOOP1Gf_SET BCMI_VIPER_XGXS_LANECTL2r_GLOOP1Gf_SET
#define READ_LANECTL2r BCMI_VIPER_XGXS_READ_LANECTL2r
#define WRITE_LANECTL2r BCMI_VIPER_XGXS_WRITE_LANECTL2r
#define MODIFY_LANECTL2r BCMI_VIPER_XGXS_MODIFY_LANECTL2r
#define READLN_LANECTL2r BCMI_VIPER_XGXS_READLN_LANECTL2r
#define WRITELN_LANECTL2r BCMI_VIPER_XGXS_WRITELN_LANECTL2r
#define WRITEALL_LANECTL2r BCMI_VIPER_XGXS_WRITEALL_LANECTL2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_LANECTL2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  LANECTL3
 * BLOCKS:   BLK1
 * REGADDR:  0x8018
 * DESC:     Lane control 3 register
 * RESETVAL: 0x200 (512)
 * ACCESS:   R/W
 * FIELDS:
 *     PWRDN_RX         Powerdown Rx lane 0
 *     PWRDN_TX         Powerdown Tx lane 0
 *     PWRDWN_PLL       Powerdown PLL
 *     RESERVED_9       
 *     LOCK_REF_EN      Enables lock_ref lane 3:0
 *     PWRDWN_FORCE     
 *     LOCK_REF         Command timing recovery to lock to reference clock, lane 0
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_LANECTL3r (0x00008018 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_LANECTL3r_SIZE 4

/*
 * This structure should be used to declare and program LANECTL3.
 *
 */
typedef union BCMI_VIPER_XGXS_LANECTL3r_s {
	uint32_t v[1];
	uint32_t lanectl3[1];
	uint32_t _lanectl3;
} BCMI_VIPER_XGXS_LANECTL3r_t;

#define BCMI_VIPER_XGXS_LANECTL3r_CLR(r) (r).lanectl3[0] = 0
#define BCMI_VIPER_XGXS_LANECTL3r_SET(r,d) (r).lanectl3[0] = d
#define BCMI_VIPER_XGXS_LANECTL3r_GET(r) (r).lanectl3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_LANECTL3r_LOCK_REFf_GET(r) ((((r).lanectl3[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_LANECTL3r_LOCK_REFf_SET(r,f) (r).lanectl3[0]=(((r).lanectl3[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_LANECTL3r_PWRDWN_FORCEf_GET(r) ((((r).lanectl3[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_LANECTL3r_PWRDWN_FORCEf_SET(r,f) (r).lanectl3[0]=(((r).lanectl3[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_LANECTL3r_LOCK_REF_ENf_GET(r) ((((r).lanectl3[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_LANECTL3r_LOCK_REF_ENf_SET(r,f) (r).lanectl3[0]=(((r).lanectl3[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_LANECTL3r_RESERVED_9f_GET(r) ((((r).lanectl3[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_LANECTL3r_RESERVED_9f_SET(r,f) (r).lanectl3[0]=(((r).lanectl3[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_LANECTL3r_PWRDWN_PLLf_GET(r) ((((r).lanectl3[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_LANECTL3r_PWRDWN_PLLf_SET(r,f) (r).lanectl3[0]=(((r).lanectl3[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_LANECTL3r_PWRDN_TXf_GET(r) ((((r).lanectl3[0]) >> 4) & 0xf)
#define BCMI_VIPER_XGXS_LANECTL3r_PWRDN_TXf_SET(r,f) (r).lanectl3[0]=(((r).lanectl3[0] & ~((uint32_t)0xf << 4)) | ((((uint32_t)f) & 0xf) << 4)) | (0xf << (16 + 4))
#define BCMI_VIPER_XGXS_LANECTL3r_PWRDN_RXf_GET(r) (((r).lanectl3[0]) & 0xf)
#define BCMI_VIPER_XGXS_LANECTL3r_PWRDN_RXf_SET(r,f) (r).lanectl3[0]=(((r).lanectl3[0] & ~((uint32_t)0xf)) | (((uint32_t)f) & 0xf)) | (0xf << 16)

/*
 * These macros can be used to access LANECTL3.
 *
 */
#define BCMI_VIPER_XGXS_READ_LANECTL3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANECTL3r,(_r._lanectl3))
#define BCMI_VIPER_XGXS_WRITE_LANECTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL3r,(_r._lanectl3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_LANECTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL3r,(_r._lanectl3))
#define BCMI_VIPER_XGXS_READLN_LANECTL3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANECTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanectl3))
#define BCMI_VIPER_XGXS_WRITELN_LANECTL3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanectl3))
#define BCMI_VIPER_XGXS_WRITEALL_LANECTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANECTL3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._lanectl3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define LANECTL3r BCMI_VIPER_XGXS_LANECTL3r
#define LANECTL3r_SIZE BCMI_VIPER_XGXS_LANECTL3r_SIZE
typedef BCMI_VIPER_XGXS_LANECTL3r_t LANECTL3r_t;
#define LANECTL3r_CLR BCMI_VIPER_XGXS_LANECTL3r_CLR
#define LANECTL3r_SET BCMI_VIPER_XGXS_LANECTL3r_SET
#define LANECTL3r_GET BCMI_VIPER_XGXS_LANECTL3r_GET
#define LANECTL3r_LOCK_REFf_GET BCMI_VIPER_XGXS_LANECTL3r_LOCK_REFf_GET
#define LANECTL3r_LOCK_REFf_SET BCMI_VIPER_XGXS_LANECTL3r_LOCK_REFf_SET
#define LANECTL3r_PWRDWN_FORCEf_GET BCMI_VIPER_XGXS_LANECTL3r_PWRDWN_FORCEf_GET
#define LANECTL3r_PWRDWN_FORCEf_SET BCMI_VIPER_XGXS_LANECTL3r_PWRDWN_FORCEf_SET
#define LANECTL3r_LOCK_REF_ENf_GET BCMI_VIPER_XGXS_LANECTL3r_LOCK_REF_ENf_GET
#define LANECTL3r_LOCK_REF_ENf_SET BCMI_VIPER_XGXS_LANECTL3r_LOCK_REF_ENf_SET
#define LANECTL3r_RESERVED_9f_GET BCMI_VIPER_XGXS_LANECTL3r_RESERVED_9f_GET
#define LANECTL3r_RESERVED_9f_SET BCMI_VIPER_XGXS_LANECTL3r_RESERVED_9f_SET
#define LANECTL3r_PWRDWN_PLLf_GET BCMI_VIPER_XGXS_LANECTL3r_PWRDWN_PLLf_GET
#define LANECTL3r_PWRDWN_PLLf_SET BCMI_VIPER_XGXS_LANECTL3r_PWRDWN_PLLf_SET
#define LANECTL3r_PWRDN_TXf_GET BCMI_VIPER_XGXS_LANECTL3r_PWRDN_TXf_GET
#define LANECTL3r_PWRDN_TXf_SET BCMI_VIPER_XGXS_LANECTL3r_PWRDN_TXf_SET
#define LANECTL3r_PWRDN_RXf_GET BCMI_VIPER_XGXS_LANECTL3r_PWRDN_RXf_GET
#define LANECTL3r_PWRDN_RXf_SET BCMI_VIPER_XGXS_LANECTL3r_PWRDN_RXf_SET
#define READ_LANECTL3r BCMI_VIPER_XGXS_READ_LANECTL3r
#define WRITE_LANECTL3r BCMI_VIPER_XGXS_WRITE_LANECTL3r
#define MODIFY_LANECTL3r BCMI_VIPER_XGXS_MODIFY_LANECTL3r
#define READLN_LANECTL3r BCMI_VIPER_XGXS_READLN_LANECTL3r
#define WRITELN_LANECTL3r BCMI_VIPER_XGXS_WRITELN_LANECTL3r
#define WRITEALL_LANECTL3r BCMI_VIPER_XGXS_WRITEALL_LANECTL3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_LANECTL3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  LANEPRBS
 * BLOCKS:   BLK1
 * REGADDR:  0x8019
 * DESC:     Lane PRBS control register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     PRBS_ORDER0      Lane 0 PRBS order
 *     PRBS_INV0        Invert Lane 0 PRBS polynomial
 *     PRBS_EN0         Lane 0 PRBS generator/ monitor enable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_LANEPRBSr (0x00008019 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_LANEPRBSr_SIZE 4

/*
 * This structure should be used to declare and program LANEPRBS.
 *
 */
typedef union BCMI_VIPER_XGXS_LANEPRBSr_s {
	uint32_t v[1];
	uint32_t laneprbs[1];
	uint32_t _laneprbs;
} BCMI_VIPER_XGXS_LANEPRBSr_t;

#define BCMI_VIPER_XGXS_LANEPRBSr_CLR(r) (r).laneprbs[0] = 0
#define BCMI_VIPER_XGXS_LANEPRBSr_SET(r,d) (r).laneprbs[0] = d
#define BCMI_VIPER_XGXS_LANEPRBSr_GET(r) (r).laneprbs[0]

/*
 * These macros can be used to access individual fields.
 *
 */

#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN3f_GET(r) ((((r).laneprbs[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN3f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV3f_GET(r) ((((r).laneprbs[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV3f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER3f_GET(r) ((((r).laneprbs[0]) >> 12) & 0x3)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER3f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x3 << 12)) | ((((uint32_t)f) & 0x3) << 12)) | (0x3 << (16 + 12) )
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN2f_GET(r) ((((r).laneprbs[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN2f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV2f_GET(r) ((((r).laneprbs[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV2f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER2f_GET(r) ((((r).laneprbs[0]) >> 8) & 0x3)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER2f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x3 << 8)) | ((((uint32_t)f) & 0x3) << 8)) | (0x3 << (16 + 8) )
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN1f_GET(r) ((((r).laneprbs[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN1f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV1f_GET(r) ((((r).laneprbs[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV1f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER1f_GET(r) ((((r).laneprbs[0]) >> 4) & 0x3)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER1f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x3 << 4)) | ((((uint32_t)f) & 0x3) << 4)) | (0x3 << (16 + 4) )
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN0f_GET(r) ((((r).laneprbs[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN0f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV0f_GET(r) ((((r).laneprbs[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV0f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER0f_GET(r) (((r).laneprbs[0]) & 0x3)
#define BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER0f_SET(r,f) (r).laneprbs[0]=(((r).laneprbs[0] & ~((uint32_t)0x3)) | (((uint32_t)f) & 0x3)) | (0x3 << 16)

/*
 * These macros can be used to access LANEPRBS.
 *
 */
#define BCMI_VIPER_XGXS_READ_LANEPRBSr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANEPRBSr,(_r._laneprbs))
#define BCMI_VIPER_XGXS_WRITE_LANEPRBSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANEPRBSr,(_r._laneprbs)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_LANEPRBSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANEPRBSr,(_r._laneprbs))
#define BCMI_VIPER_XGXS_READLN_LANEPRBSr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANEPRBSr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._laneprbs))
#define BCMI_VIPER_XGXS_WRITELN_LANEPRBSr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANEPRBSr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._laneprbs))
#define BCMI_VIPER_XGXS_WRITEALL_LANEPRBSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANEPRBSr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._laneprbs))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define LANEPRBSr BCMI_VIPER_XGXS_LANEPRBSr
#define LANEPRBSr_SIZE BCMI_VIPER_XGXS_LANEPRBSr_SIZE
typedef BCMI_VIPER_XGXS_LANEPRBSr_t LANEPRBSr_t;
#define LANEPRBSr_CLR BCMI_VIPER_XGXS_LANEPRBSr_CLR
#define LANEPRBSr_SET BCMI_VIPER_XGXS_LANEPRBSr_SET
#define LANEPRBSr_GET BCMI_VIPER_XGXS_LANEPRBSr_GET
#define LANEPRBSr_PRBS_EN3f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN3f_GET
#define LANEPRBSr_PRBS_EN3f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN3f_SET
#define LANEPRBSr_PRBS_INV3f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV3f_GET
#define LANEPRBSr_PRBS_INV3f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV3f_SET
#define LANEPRBSr_PRBS_ORDER3f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER3f_GET
#define LANEPRBSr_PRBS_ORDER3f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER3f_SET
#define LANEPRBSr_PRBS_EN2f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN2f_GET
#define LANEPRBSr_PRBS_EN2f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN2f_SET
#define LANEPRBSr_PRBS_INV2f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV2f_GET
#define LANEPRBSr_PRBS_INV2f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV2f_SET
#define LANEPRBSr_PRBS_ORDER2f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER2f_GET
#define LANEPRBSr_PRBS_ORDER2f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER2f_SET
#define LANEPRBSr_PRBS_EN1f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN1f_GET
#define LANEPRBSr_PRBS_EN1f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN1f_SET
#define LANEPRBSr_PRBS_INV1f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV1f_GET
#define LANEPRBSr_PRBS_INV1f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV1f_SET
#define LANEPRBSr_PRBS_ORDER1f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER1f_GET
#define LANEPRBSr_PRBS_ORDER1f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER1f_SET
#define LANEPRBSr_PRBS_EN0f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN0f_GET
#define LANEPRBSr_PRBS_EN0f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_EN0f_SET
#define LANEPRBSr_PRBS_INV0f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV0f_GET
#define LANEPRBSr_PRBS_INV0f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_INV0f_SET
#define LANEPRBSr_PRBS_ORDER0f_GET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER0f_GET
#define LANEPRBSr_PRBS_ORDER0f_SET BCMI_VIPER_XGXS_LANEPRBSr_PRBS_ORDER0f_SET
#define READ_LANEPRBSr BCMI_VIPER_XGXS_READ_LANEPRBSr
#define WRITE_LANEPRBSr BCMI_VIPER_XGXS_WRITE_LANEPRBSr
#define MODIFY_LANEPRBSr BCMI_VIPER_XGXS_MODIFY_LANEPRBSr
#define READLN_LANEPRBSr BCMI_VIPER_XGXS_READLN_LANEPRBSr
#define WRITELN_LANEPRBSr BCMI_VIPER_XGXS_WRITELN_LANEPRBSr
#define WRITEALL_LANEPRBSr BCMI_VIPER_XGXS_WRITEALL_LANEPRBSr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_LANEPRBSr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  LANETEST
 * BLOCKS:   BLK1
 * REGADDR:  0x801a
 * DESC:     Lane test control register
 * RESETVAL: 0x100 (256)
 * ACCESS:   R/W
 * FIELDS:
 *     LFCK_BYPASS      
 *     PLL_LOCK_RSTB_R  When set disables reset from being asserted while the PLL is not locked
 *     RXSEQSTART_EXT_DIS Isolates rxSeqStar primary input pins
 *     PWRDWN_CLKS_EN   When set enables output clocks to be present when the core is commanded into powerdown state.
 *     PWRDN_SAFE_DIS   When set disables break-before make logic which allows a graceful transition on the clock outputs when powerdown is asserted.  Otherwise powerdown assertion is treated as an asynchronous event and glitches on output clocks can occur during its assertion.
 *     PWRDN_EXT_DIS    Isolates pwrdn primary input pins
 *     TMUX_SEL         
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_LANETESTr (0x0000801a | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_LANETESTr_SIZE 4

/*
 * This structure should be used to declare and program LANETEST.
 *
 */
typedef union BCMI_VIPER_XGXS_LANETESTr_s {
	uint32_t v[1];
	uint32_t lanetest[1];
	uint32_t _lanetest;
} BCMI_VIPER_XGXS_LANETESTr_t;

#define BCMI_VIPER_XGXS_LANETESTr_CLR(r) (r).lanetest[0] = 0
#define BCMI_VIPER_XGXS_LANETESTr_SET(r,d) (r).lanetest[0] = d
#define BCMI_VIPER_XGXS_LANETESTr_GET(r) (r).lanetest[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_LANETESTr_TMUX_SELf_GET(r) ((((r).lanetest[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_LANETESTr_TMUX_SELf_SET(r,f) (r).lanetest[0]=(((r).lanetest[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_LANETESTr_PWRDN_EXT_DISf_GET(r) ((((r).lanetest[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_LANETESTr_PWRDN_EXT_DISf_SET(r,f) (r).lanetest[0]=(((r).lanetest[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_LANETESTr_PWRDN_SAFE_DISf_GET(r) ((((r).lanetest[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_LANETESTr_PWRDN_SAFE_DISf_SET(r,f) (r).lanetest[0]=(((r).lanetest[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_LANETESTr_PWRDWN_CLKS_ENf_GET(r) ((((r).lanetest[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_LANETESTr_PWRDWN_CLKS_ENf_SET(r,f) (r).lanetest[0]=(((r).lanetest[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_LANETESTr_RXSEQSTART_EXT_DISf_GET(r) ((((r).lanetest[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_LANETESTr_RXSEQSTART_EXT_DISf_SET(r,f) (r).lanetest[0]=(((r).lanetest[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_LANETESTr_PLL_LOCK_RSTB_Rf_GET(r) ((((r).lanetest[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_LANETESTr_PLL_LOCK_RSTB_Rf_SET(r,f) (r).lanetest[0]=(((r).lanetest[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_LANETESTr_LFCK_BYPASSf_GET(r) ((((r).lanetest[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_LANETESTr_LFCK_BYPASSf_SET(r,f) (r).lanetest[0]=(((r).lanetest[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))

/*
 * These macros can be used to access LANETEST.
 *
 */
#define BCMI_VIPER_XGXS_READ_LANETESTr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANETESTr,(_r._lanetest))
#define BCMI_VIPER_XGXS_WRITE_LANETESTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANETESTr,(_r._lanetest)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_LANETESTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANETESTr,(_r._lanetest))
#define BCMI_VIPER_XGXS_READLN_LANETESTr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANETESTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanetest))
#define BCMI_VIPER_XGXS_WRITELN_LANETESTr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANETESTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanetest))
#define BCMI_VIPER_XGXS_WRITEALL_LANETESTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANETESTr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._lanetest))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define LANETESTr BCMI_VIPER_XGXS_LANETESTr
#define LANETESTr_SIZE BCMI_VIPER_XGXS_LANETESTr_SIZE
typedef BCMI_VIPER_XGXS_LANETESTr_t LANETESTr_t;
#define LANETESTr_CLR BCMI_VIPER_XGXS_LANETESTr_CLR
#define LANETESTr_SET BCMI_VIPER_XGXS_LANETESTr_SET
#define LANETESTr_GET BCMI_VIPER_XGXS_LANETESTr_GET
#define LANETESTr_TMUX_SELf_GET BCMI_VIPER_XGXS_LANETESTr_TMUX_SELf_GET
#define LANETESTr_TMUX_SELf_SET BCMI_VIPER_XGXS_LANETESTr_TMUX_SELf_SET
#define LANETESTr_PWRDN_EXT_DISf_GET BCMI_VIPER_XGXS_LANETESTr_PWRDN_EXT_DISf_GET
#define LANETESTr_PWRDN_EXT_DISf_SET BCMI_VIPER_XGXS_LANETESTr_PWRDN_EXT_DISf_SET
#define LANETESTr_PWRDN_SAFE_DISf_GET BCMI_VIPER_XGXS_LANETESTr_PWRDN_SAFE_DISf_GET
#define LANETESTr_PWRDN_SAFE_DISf_SET BCMI_VIPER_XGXS_LANETESTr_PWRDN_SAFE_DISf_SET
#define LANETESTr_PWRDWN_CLKS_ENf_GET BCMI_VIPER_XGXS_LANETESTr_PWRDWN_CLKS_ENf_GET
#define LANETESTr_PWRDWN_CLKS_ENf_SET BCMI_VIPER_XGXS_LANETESTr_PWRDWN_CLKS_ENf_SET
#define LANETESTr_RXSEQSTART_EXT_DISf_GET BCMI_VIPER_XGXS_LANETESTr_RXSEQSTART_EXT_DISf_GET
#define LANETESTr_RXSEQSTART_EXT_DISf_SET BCMI_VIPER_XGXS_LANETESTr_RXSEQSTART_EXT_DISf_SET
#define LANETESTr_PLL_LOCK_RSTB_Rf_GET BCMI_VIPER_XGXS_LANETESTr_PLL_LOCK_RSTB_Rf_GET
#define LANETESTr_PLL_LOCK_RSTB_Rf_SET BCMI_VIPER_XGXS_LANETESTr_PLL_LOCK_RSTB_Rf_SET
#define LANETESTr_LFCK_BYPASSf_GET BCMI_VIPER_XGXS_LANETESTr_LFCK_BYPASSf_GET
#define LANETESTr_LFCK_BYPASSf_SET BCMI_VIPER_XGXS_LANETESTr_LFCK_BYPASSf_SET
#define READ_LANETESTr BCMI_VIPER_XGXS_READ_LANETESTr
#define WRITE_LANETESTr BCMI_VIPER_XGXS_WRITE_LANETESTr
#define MODIFY_LANETESTr BCMI_VIPER_XGXS_MODIFY_LANETESTr
#define READLN_LANETESTr BCMI_VIPER_XGXS_READLN_LANETESTr
#define WRITELN_LANETESTr BCMI_VIPER_XGXS_WRITELN_LANETESTr
#define WRITEALL_LANETESTr BCMI_VIPER_XGXS_WRITEALL_LANETESTr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_LANETESTr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  BLKADDR1
 * BLOCKS:   BLK1
 * REGADDR:  0x801f
 * DESC:     Block Address register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     BLOCKADDRESS     Bits 14:4 of the address
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_BLKADDR1r (0x0000801f | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_BLKADDR1r_SIZE 4

/*
 * This structure should be used to declare and program BLKADDR1.
 *
 */
typedef union BCMI_VIPER_XGXS_BLKADDR1r_s {
	uint32_t v[1];
	uint32_t blkaddr1[1];
	uint32_t _blkaddr1;
} BCMI_VIPER_XGXS_BLKADDR1r_t;

#define BCMI_VIPER_XGXS_BLKADDR1r_CLR(r) (r).blkaddr1[0] = 0
#define BCMI_VIPER_XGXS_BLKADDR1r_SET(r,d) (r).blkaddr1[0] = d
#define BCMI_VIPER_XGXS_BLKADDR1r_GET(r) (r).blkaddr1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_BLKADDR1r_BLOCKADDRESSf_GET(r) ((((r).blkaddr1[0]) >> 4) & 0x7ff)
#define BCMI_VIPER_XGXS_BLKADDR1r_BLOCKADDRESSf_SET(r,f) (r).blkaddr1[0]=(((r).blkaddr1[0] & ~((uint32_t)0x7ff << 4)) | ((((uint32_t)f) & 0x7ff) << 4)) | (2047 << (16 + 4))

/*
 * These macros can be used to access BLKADDR1.
 *
 */
#define BCMI_VIPER_XGXS_READ_BLKADDR1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_BLKADDR1r,(_r._blkaddr1))
#define BCMI_VIPER_XGXS_WRITE_BLKADDR1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_BLKADDR1r,(_r._blkaddr1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_BLKADDR1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_BLKADDR1r,(_r._blkaddr1))
#define BCMI_VIPER_XGXS_READLN_BLKADDR1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_BLKADDR1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._blkaddr1))
#define BCMI_VIPER_XGXS_WRITELN_BLKADDR1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_BLKADDR1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._blkaddr1))
#define BCMI_VIPER_XGXS_WRITEALL_BLKADDR1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_BLKADDR1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._blkaddr1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define BLKADDR1r BCMI_VIPER_XGXS_BLKADDR1r
#define BLKADDR1r_SIZE BCMI_VIPER_XGXS_BLKADDR1r_SIZE
typedef BCMI_VIPER_XGXS_BLKADDR1r_t BLKADDR1r_t;
#define BLKADDR1r_CLR BCMI_VIPER_XGXS_BLKADDR1r_CLR
#define BLKADDR1r_SET BCMI_VIPER_XGXS_BLKADDR1r_SET
#define BLKADDR1r_GET BCMI_VIPER_XGXS_BLKADDR1r_GET
#define BLKADDR1r_BLOCKADDRESSf_GET BCMI_VIPER_XGXS_BLKADDR1r_BLOCKADDRESSf_GET
#define BLKADDR1r_BLOCKADDRESSf_SET BCMI_VIPER_XGXS_BLKADDR1r_BLOCKADDRESSf_SET
#define READ_BLKADDR1r BCMI_VIPER_XGXS_READ_BLKADDR1r
#define WRITE_BLKADDR1r BCMI_VIPER_XGXS_WRITE_BLKADDR1r
#define MODIFY_BLKADDR1r BCMI_VIPER_XGXS_MODIFY_BLKADDR1r
#define READLN_BLKADDR1r BCMI_VIPER_XGXS_READLN_BLKADDR1r
#define WRITELN_BLKADDR1r BCMI_VIPER_XGXS_WRITELN_BLKADDR1r
#define WRITEALL_BLKADDR1r BCMI_VIPER_XGXS_WRITEALL_BLKADDR1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_BLKADDR1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_CTL0
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x8050
 * DESC:     Analog PLL Control0 Register
 * RESETVAL: 0x5740 (22336)
 * ACCESS:   R/W
 * FIELDS:
 *     CM_SEL           Select the common mode level in XTAL
 *     HIPASS           Select the high-pass pole in XTAL
 *     CURR_SEL         Charge Ipump (differential) Current Settings (step size 100uA)0000:  100uA0001:  200uA...0011   400uA...1111:  1.6mA
 *     RPAR             Reduce filter resistor to lower PLL bandwidth0000: 1.22K0001: 1.33K0010: 1.48K0011: 1.65K0100: 1.82K0101: 2.09K0110: 2.55K0111: 3.15K1000: 3.48K1001: 3.75K1010: 4.24K1011: 4.65K1100: 5.45K1101: 6.15K1110: 7.65K1111: 9.15K
 *     CPAR             Increase Filter Shunt Capacitor00:3pF01:6pF10:9pF11:18pF
 *     XTAL_BIAS        REFCLK  buffer XTAL bias controls0	0	0	33%0	0	1	83%0	1	0	67%0	1	1	117%1	0	0	50%1	0	1	100%1	1	0	83%1	1	1	133%
 *     EN_TEST_INTEGER_CLK 1:Enable integer-N divider output as CMOS test clock
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r (0x00008050 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_CTL0.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_CTL0r_s {
	uint32_t v[1];
	uint32_t pll_afe_ctl0[1];
	uint32_t _pll_afe_ctl0;
} BCMI_VIPER_XGXS_PLL_AFE_CTL0r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CLR(r) (r).pll_afe_ctl0[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_SET(r,d) (r).pll_afe_ctl0[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_GET(r) (r).pll_afe_ctl0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_EN_TEST_INTEGER_CLKf_GET(r) ((((r).pll_afe_ctl0[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_EN_TEST_INTEGER_CLKf_SET(r,f) (r).pll_afe_ctl0[0]=(((r).pll_afe_ctl0[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_XTAL_BIASf_GET(r) ((((r).pll_afe_ctl0[0]) >> 12) & 0x7)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_XTAL_BIASf_SET(r,f) (r).pll_afe_ctl0[0]=(((r).pll_afe_ctl0[0] & ~((uint32_t)0x7 << 12)) | ((((uint32_t)f) & 0x7) << 12)) | (7 << (16 + 12))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CPARf_GET(r) ((((r).pll_afe_ctl0[0]) >> 10) & 0x3)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CPARf_SET(r,f) (r).pll_afe_ctl0[0]=(((r).pll_afe_ctl0[0] & ~((uint32_t)0x3 << 10)) | ((((uint32_t)f) & 0x3) << 10)) | (3 << (16 + 10))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_RPARf_GET(r) ((((r).pll_afe_ctl0[0]) >> 6) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_RPARf_SET(r,f) (r).pll_afe_ctl0[0]=(((r).pll_afe_ctl0[0] & ~((uint32_t)0xf << 6)) | ((((uint32_t)f) & 0xf) << 6)) | (15 << (16 + 6))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CURR_SELf_GET(r) ((((r).pll_afe_ctl0[0]) >> 2) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CURR_SELf_SET(r,f) (r).pll_afe_ctl0[0]=(((r).pll_afe_ctl0[0] & ~((uint32_t)0xf << 2)) | ((((uint32_t)f) & 0xf) << 2)) | (15 << (16 + 2))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_HIPASSf_GET(r) ((((r).pll_afe_ctl0[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_HIPASSf_SET(r,f) (r).pll_afe_ctl0[0]=(((r).pll_afe_ctl0[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CM_SELf_GET(r) (((r).pll_afe_ctl0[0]) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CM_SELf_SET(r,f) (r).pll_afe_ctl0[0]=(((r).pll_afe_ctl0[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access PLL_AFE_CTL0.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL0r,(_r._pll_afe_ctl0))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL0r,(_r._pll_afe_ctl0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL0r,(_r._pll_afe_ctl0))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl0))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl0))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_ctl0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_CTL0r BCMI_VIPER_XGXS_PLL_AFE_CTL0r
#define PLL_AFE_CTL0r_SIZE BCMI_VIPER_XGXS_PLL_AFE_CTL0r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_CTL0r_t PLL_AFE_CTL0r_t;
#define PLL_AFE_CTL0r_CLR BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CLR
#define PLL_AFE_CTL0r_SET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_SET
#define PLL_AFE_CTL0r_GET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_GET
#define PLL_AFE_CTL0r_EN_TEST_INTEGER_CLKf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_EN_TEST_INTEGER_CLKf_GET
#define PLL_AFE_CTL0r_EN_TEST_INTEGER_CLKf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_EN_TEST_INTEGER_CLKf_SET
#define PLL_AFE_CTL0r_XTAL_BIASf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_XTAL_BIASf_GET
#define PLL_AFE_CTL0r_XTAL_BIASf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_XTAL_BIASf_SET
#define PLL_AFE_CTL0r_CPARf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CPARf_GET
#define PLL_AFE_CTL0r_CPARf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CPARf_SET
#define PLL_AFE_CTL0r_RPARf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_RPARf_GET
#define PLL_AFE_CTL0r_RPARf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_RPARf_SET
#define PLL_AFE_CTL0r_CURR_SELf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CURR_SELf_GET
#define PLL_AFE_CTL0r_CURR_SELf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CURR_SELf_SET
#define PLL_AFE_CTL0r_HIPASSf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_HIPASSf_GET
#define PLL_AFE_CTL0r_HIPASSf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_HIPASSf_SET
#define PLL_AFE_CTL0r_CM_SELf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CM_SELf_GET
#define PLL_AFE_CTL0r_CM_SELf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL0r_CM_SELf_SET
#define READ_PLL_AFE_CTL0r BCMI_VIPER_XGXS_READ_PLL_AFE_CTL0r
#define WRITE_PLL_AFE_CTL0r BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL0r
#define MODIFY_PLL_AFE_CTL0r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL0r
#define READLN_PLL_AFE_CTL0r BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL0r
#define WRITELN_PLL_AFE_CTL0r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL0r
#define WRITEALL_PLL_AFE_CTL0r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_CTL0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_CTL1
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x8051
 * DESC:     Analog PLL Control1 Register
 * RESETVAL: 0x1d0 (464)
 * ACCESS:   R/W
 * FIELDS:
 *     EN_TEST_FRAC_CLK 1:Enable frac-N divider (MMD)r output as CMOS test clock
 *     LV_EN            Adjusts refrence current of reference clk buffer for low VDD operation, active 0
 *     XTAL_CORE_BIAS   ref_clk buffer XTAL core bias controls0000 200uA0010 300uA0100 400uA0110 500uA1000 600uA1010 700uA1100 800uA1110 900uA
 *     PLL_PON          Resistor Calibration Control Setting0000: min resistance process corner, (increases on-chip R by +25%)1111: max resistance process corner, (decreases on-chip R by -25%)Error between two consecutive codes is maintained less than 3%.By default, pon<3:0> match with resistor calibration output; they can also be independently adjusted through RTL.
 *     LEAKAGE_TEST     Test pins for leakage power  internal analog test only
 *     RESERVED_31_28   
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r (0x00008051 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_CTL1.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_CTL1r_s {
	uint32_t v[1];
	uint32_t pll_afe_ctl1[1];
	uint32_t _pll_afe_ctl1;
} BCMI_VIPER_XGXS_PLL_AFE_CTL1r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_CLR(r) (r).pll_afe_ctl1[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_SET(r,d) (r).pll_afe_ctl1[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_GET(r) (r).pll_afe_ctl1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_RESERVED_31_28f_GET(r) ((((r).pll_afe_ctl1[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_RESERVED_31_28f_SET(r,f) (r).pll_afe_ctl1[0]=(((r).pll_afe_ctl1[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_LEAKAGE_TESTf_GET(r) ((((r).pll_afe_ctl1[0]) >> 10) & 0x3)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_LEAKAGE_TESTf_SET(r,f) (r).pll_afe_ctl1[0]=(((r).pll_afe_ctl1[0] & ~((uint32_t)0x3 << 10)) | ((((uint32_t)f) & 0x3) << 10)) | (3 << (16 + 10))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_PLL_PONf_GET(r) ((((r).pll_afe_ctl1[0]) >> 6) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_PLL_PONf_SET(r,f) (r).pll_afe_ctl1[0]=(((r).pll_afe_ctl1[0] & ~((uint32_t)0xf << 6)) | ((((uint32_t)f) & 0xf) << 6)) | (15 << (16 + 6))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_XTAL_CORE_BIASf_GET(r) ((((r).pll_afe_ctl1[0]) >> 2) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_XTAL_CORE_BIASf_SET(r,f) (r).pll_afe_ctl1[0]=(((r).pll_afe_ctl1[0] & ~((uint32_t)0xf << 2)) | ((((uint32_t)f) & 0xf) << 2)) | (15 << (16 + 2))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_LV_ENf_GET(r) ((((r).pll_afe_ctl1[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_LV_ENf_SET(r,f) (r).pll_afe_ctl1[0]=(((r).pll_afe_ctl1[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_EN_TEST_FRAC_CLKf_GET(r) (((r).pll_afe_ctl1[0]) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL1r_EN_TEST_FRAC_CLKf_SET(r,f) (r).pll_afe_ctl1[0]=(((r).pll_afe_ctl1[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access PLL_AFE_CTL1.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL1r,(_r._pll_afe_ctl1))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL1r,(_r._pll_afe_ctl1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL1r,(_r._pll_afe_ctl1))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl1))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl1))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_ctl1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_CTL1r BCMI_VIPER_XGXS_PLL_AFE_CTL1r
#define PLL_AFE_CTL1r_SIZE BCMI_VIPER_XGXS_PLL_AFE_CTL1r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_CTL1r_t PLL_AFE_CTL1r_t;
#define PLL_AFE_CTL1r_CLR BCMI_VIPER_XGXS_PLL_AFE_CTL1r_CLR
#define PLL_AFE_CTL1r_SET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_SET
#define PLL_AFE_CTL1r_GET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_GET
#define PLL_AFE_CTL1r_RESERVED_31_28f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_RESERVED_31_28f_GET
#define PLL_AFE_CTL1r_RESERVED_31_28f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_RESERVED_31_28f_SET
#define PLL_AFE_CTL1r_LEAKAGE_TESTf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_LEAKAGE_TESTf_GET
#define PLL_AFE_CTL1r_LEAKAGE_TESTf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_LEAKAGE_TESTf_SET
#define PLL_AFE_CTL1r_PLL_PONf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_PLL_PONf_GET
#define PLL_AFE_CTL1r_PLL_PONf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_PLL_PONf_SET
#define PLL_AFE_CTL1r_XTAL_CORE_BIASf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_XTAL_CORE_BIASf_GET
#define PLL_AFE_CTL1r_XTAL_CORE_BIASf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_XTAL_CORE_BIASf_SET
#define PLL_AFE_CTL1r_LV_ENf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_LV_ENf_GET
#define PLL_AFE_CTL1r_LV_ENf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_LV_ENf_SET
#define PLL_AFE_CTL1r_EN_TEST_FRAC_CLKf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_EN_TEST_FRAC_CLKf_GET
#define PLL_AFE_CTL1r_EN_TEST_FRAC_CLKf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL1r_EN_TEST_FRAC_CLKf_SET
#define READ_PLL_AFE_CTL1r BCMI_VIPER_XGXS_READ_PLL_AFE_CTL1r
#define WRITE_PLL_AFE_CTL1r BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL1r
#define MODIFY_PLL_AFE_CTL1r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL1r
#define READLN_PLL_AFE_CTL1r BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL1r
#define WRITELN_PLL_AFE_CTL1r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL1r
#define WRITEALL_PLL_AFE_CTL1r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_CTL1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_CTL2
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x8052
 * DESC:     Analog PLL Control2 Register
 * RESETVAL: 0x19f0 (6640)
 * ACCESS:   R/W
 * FIELDS:
 *     RESERVED_33_32   
 *     TEST_VC          Enable VCO control voltage on PTESTP/N; active high
 *     EN10T            enables 10T clock
 *     INTN_FB_EN       enables integer divider output
 *     TEST_AMP         Set Testport Output Impedance0000 8000001 4000010 2670011 2000100 1600101 1330110 1140111 1001000 891001 801010 731011 67
 *     RESERVED_41      
 *     TEST_PLL_MODE    selects fb clock to be tested. 0:int-N, 1:frac-N
 *     VCO_ICTR         none
 *     I_PLL_FRAC_MODE  00,11: direct integer division mapping for MMD: M = i_ndiv_int<9:5> && A = i_ndiv_int<4:0>01: div8/9 output mapping i for MMD: M1 && A110: div4/5 output mapping is used for MMD: M2 && A2
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r (0x00008052 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_CTL2.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_CTL2r_s {
	uint32_t v[1];
	uint32_t pll_afe_ctl2[1];
	uint32_t _pll_afe_ctl2;
} BCMI_VIPER_XGXS_PLL_AFE_CTL2r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_CLR(r) (r).pll_afe_ctl2[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_SET(r,d) (r).pll_afe_ctl2[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_GET(r) (r).pll_afe_ctl2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_I_PLL_FRAC_MODEf_GET(r) ((((r).pll_afe_ctl2[0]) >> 14) & 0x3)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_I_PLL_FRAC_MODEf_SET(r,f) (r).pll_afe_ctl2[0]=(((r).pll_afe_ctl2[0] & ~((uint32_t)0x3 << 14)) | ((((uint32_t)f) & 0x3) << 14)) | (3 << (16 + 14))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_VCO_ICTRf_GET(r) ((((r).pll_afe_ctl2[0]) >> 11) & 0x7)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_VCO_ICTRf_SET(r,f) (r).pll_afe_ctl2[0]=(((r).pll_afe_ctl2[0] & ~((uint32_t)0x7 << 11)) | ((((uint32_t)f) & 0x7) << 11)) | (7 << (16 + 11))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_PLL_MODEf_GET(r) ((((r).pll_afe_ctl2[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_PLL_MODEf_SET(r,f) (r).pll_afe_ctl2[0]=(((r).pll_afe_ctl2[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_RESERVED_41f_GET(r) ((((r).pll_afe_ctl2[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_RESERVED_41f_SET(r,f) (r).pll_afe_ctl2[0]=(((r).pll_afe_ctl2[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_AMPf_GET(r) ((((r).pll_afe_ctl2[0]) >> 5) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_AMPf_SET(r,f) (r).pll_afe_ctl2[0]=(((r).pll_afe_ctl2[0] & ~((uint32_t)0xf << 5)) | ((((uint32_t)f) & 0xf) << 5)) | (15 << (16 + 5))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_INTN_FB_ENf_GET(r) ((((r).pll_afe_ctl2[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_INTN_FB_ENf_SET(r,f) (r).pll_afe_ctl2[0]=(((r).pll_afe_ctl2[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_EN10Tf_GET(r) ((((r).pll_afe_ctl2[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_EN10Tf_SET(r,f) (r).pll_afe_ctl2[0]=(((r).pll_afe_ctl2[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_VCf_GET(r) ((((r).pll_afe_ctl2[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_VCf_SET(r,f) (r).pll_afe_ctl2[0]=(((r).pll_afe_ctl2[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_RESERVED_33_32f_GET(r) (((r).pll_afe_ctl2[0]) & 0x3)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL2r_RESERVED_33_32f_SET(r,f) (r).pll_afe_ctl2[0]=(((r).pll_afe_ctl2[0] & ~((uint32_t)0x3)) | (((uint32_t)f) & 0x3)) | (0x3 << 16)

/*
 * These macros can be used to access PLL_AFE_CTL2.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL2r,(_r._pll_afe_ctl2))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL2r,(_r._pll_afe_ctl2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL2r,(_r._pll_afe_ctl2))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl2))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl2))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_ctl2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_CTL2r BCMI_VIPER_XGXS_PLL_AFE_CTL2r
#define PLL_AFE_CTL2r_SIZE BCMI_VIPER_XGXS_PLL_AFE_CTL2r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_CTL2r_t PLL_AFE_CTL2r_t;
#define PLL_AFE_CTL2r_CLR BCMI_VIPER_XGXS_PLL_AFE_CTL2r_CLR
#define PLL_AFE_CTL2r_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_SET
#define PLL_AFE_CTL2r_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_GET
#define PLL_AFE_CTL2r_I_PLL_FRAC_MODEf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_I_PLL_FRAC_MODEf_GET
#define PLL_AFE_CTL2r_I_PLL_FRAC_MODEf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_I_PLL_FRAC_MODEf_SET
#define PLL_AFE_CTL2r_VCO_ICTRf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_VCO_ICTRf_GET
#define PLL_AFE_CTL2r_VCO_ICTRf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_VCO_ICTRf_SET
#define PLL_AFE_CTL2r_TEST_PLL_MODEf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_PLL_MODEf_GET
#define PLL_AFE_CTL2r_TEST_PLL_MODEf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_PLL_MODEf_SET
#define PLL_AFE_CTL2r_RESERVED_41f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_RESERVED_41f_GET
#define PLL_AFE_CTL2r_RESERVED_41f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_RESERVED_41f_SET
#define PLL_AFE_CTL2r_TEST_AMPf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_AMPf_GET
#define PLL_AFE_CTL2r_TEST_AMPf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_AMPf_SET
#define PLL_AFE_CTL2r_INTN_FB_ENf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_INTN_FB_ENf_GET
#define PLL_AFE_CTL2r_INTN_FB_ENf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_INTN_FB_ENf_SET
#define PLL_AFE_CTL2r_EN10Tf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_EN10Tf_GET
#define PLL_AFE_CTL2r_EN10Tf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_EN10Tf_SET
#define PLL_AFE_CTL2r_TEST_VCf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_VCf_GET
#define PLL_AFE_CTL2r_TEST_VCf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_TEST_VCf_SET
#define PLL_AFE_CTL2r_RESERVED_33_32f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_RESERVED_33_32f_GET
#define PLL_AFE_CTL2r_RESERVED_33_32f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL2r_RESERVED_33_32f_SET
#define READ_PLL_AFE_CTL2r BCMI_VIPER_XGXS_READ_PLL_AFE_CTL2r
#define WRITE_PLL_AFE_CTL2r BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL2r
#define MODIFY_PLL_AFE_CTL2r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL2r
#define READLN_PLL_AFE_CTL2r BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL2r
#define WRITELN_PLL_AFE_CTL2r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL2r
#define WRITEALL_PLL_AFE_CTL2r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_CTL2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_CTL3
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x8053
 * DESC:     Analog PLL Control3 Register
 * RESETVAL: 0x2b00 (11008)
 * ACCESS:   R/W
 * FIELDS:
 *     TCEST_NEG        Sets the amplitude without changing the Output ImpedanceWhen test_amp<3:0>=1111 and Rload = infinity (open, no far-end RX connected)000 100%001 87.5%010 75%011 62.5%100 50%101 37.5%110 25%111 12.5%
 *     TESTCLK_EN       Enables ref clock for test
 *     TEST_SEL         Selects the signal to be tested000: off (high-z)001: refclk010: fbclk011: rxclk100: txclk101: reserved110: vcp,vcn111: reserved
 *     MMD_EN           enables MMD divider
 *     INT_DIV_EN       enables integer divider
 *     REFCLK_IN_BIAS   Set refclk input bias<5:4> Buffer #1 Bias<3:2> Buffer #2 Bias<1:0> D2C BiasCurrent Adjustment:00 75%01/10 100%11 125%
 *     PLL_MODE         selects fbk clock 0:int-N 1:Frac-N
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r (0x00008053 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_CTL3.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_CTL3r_s {
	uint32_t v[1];
	uint32_t pll_afe_ctl3[1];
	uint32_t _pll_afe_ctl3;
} BCMI_VIPER_XGXS_PLL_AFE_CTL3r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_CLR(r) (r).pll_afe_ctl3[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_SET(r,d) (r).pll_afe_ctl3[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_GET(r) (r).pll_afe_ctl3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_PLL_MODEf_GET(r) ((((r).pll_afe_ctl3[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_PLL_MODEf_SET(r,f) (r).pll_afe_ctl3[0]=(((r).pll_afe_ctl3[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_REFCLK_IN_BIASf_GET(r) ((((r).pll_afe_ctl3[0]) >> 9) & 0x3f)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_REFCLK_IN_BIASf_SET(r,f) (r).pll_afe_ctl3[0]=(((r).pll_afe_ctl3[0] & ~((uint32_t)0x3f << 9)) | ((((uint32_t)f) & 0x3f) << 9)) | (63 << (16 + 9))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_INT_DIV_ENf_GET(r) ((((r).pll_afe_ctl3[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_INT_DIV_ENf_SET(r,f) (r).pll_afe_ctl3[0]=(((r).pll_afe_ctl3[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_MMD_ENf_GET(r) ((((r).pll_afe_ctl3[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_MMD_ENf_SET(r,f) (r).pll_afe_ctl3[0]=(((r).pll_afe_ctl3[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TEST_SELf_GET(r) ((((r).pll_afe_ctl3[0]) >> 4) & 0x7)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TEST_SELf_SET(r,f) (r).pll_afe_ctl3[0]=(((r).pll_afe_ctl3[0] & ~((uint32_t)0x7 << 4)) | ((((uint32_t)f) & 0x7) << 4)) | (7 << (16 + 4))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TESTCLK_ENf_GET(r) ((((r).pll_afe_ctl3[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TESTCLK_ENf_SET(r,f) (r).pll_afe_ctl3[0]=(((r).pll_afe_ctl3[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TCEST_NEGf_GET(r) (((r).pll_afe_ctl3[0]) & 0x7)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TCEST_NEGf_SET(r,f) (r).pll_afe_ctl3[0]=(((r).pll_afe_ctl3[0] & ~((uint32_t)0x7)) | (((uint32_t)f) & 0x7)) | (0x7 << 16)

/*
 * These macros can be used to access PLL_AFE_CTL3.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL3r,(_r._pll_afe_ctl3))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL3r,(_r._pll_afe_ctl3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL3r,(_r._pll_afe_ctl3))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl3))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl3))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_ctl3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_CTL3r BCMI_VIPER_XGXS_PLL_AFE_CTL3r
#define PLL_AFE_CTL3r_SIZE BCMI_VIPER_XGXS_PLL_AFE_CTL3r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_CTL3r_t PLL_AFE_CTL3r_t;
#define PLL_AFE_CTL3r_CLR BCMI_VIPER_XGXS_PLL_AFE_CTL3r_CLR
#define PLL_AFE_CTL3r_SET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_SET
#define PLL_AFE_CTL3r_GET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_GET
#define PLL_AFE_CTL3r_PLL_MODEf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_PLL_MODEf_GET
#define PLL_AFE_CTL3r_PLL_MODEf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_PLL_MODEf_SET
#define PLL_AFE_CTL3r_REFCLK_IN_BIASf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_REFCLK_IN_BIASf_GET
#define PLL_AFE_CTL3r_REFCLK_IN_BIASf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_REFCLK_IN_BIASf_SET
#define PLL_AFE_CTL3r_INT_DIV_ENf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_INT_DIV_ENf_GET
#define PLL_AFE_CTL3r_INT_DIV_ENf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_INT_DIV_ENf_SET
#define PLL_AFE_CTL3r_MMD_ENf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_MMD_ENf_GET
#define PLL_AFE_CTL3r_MMD_ENf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_MMD_ENf_SET
#define PLL_AFE_CTL3r_TEST_SELf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TEST_SELf_GET
#define PLL_AFE_CTL3r_TEST_SELf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TEST_SELf_SET
#define PLL_AFE_CTL3r_TESTCLK_ENf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TESTCLK_ENf_GET
#define PLL_AFE_CTL3r_TESTCLK_ENf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TESTCLK_ENf_SET
#define PLL_AFE_CTL3r_TCEST_NEGf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TCEST_NEGf_GET
#define PLL_AFE_CTL3r_TCEST_NEGf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL3r_TCEST_NEGf_SET
#define READ_PLL_AFE_CTL3r BCMI_VIPER_XGXS_READ_PLL_AFE_CTL3r
#define WRITE_PLL_AFE_CTL3r BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL3r
#define MODIFY_PLL_AFE_CTL3r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL3r
#define READLN_PLL_AFE_CTL3r BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL3r
#define WRITELN_PLL_AFE_CTL3r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL3r
#define WRITEALL_PLL_AFE_CTL3r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_CTL3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_CTL4
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x8054
 * DESC:     Analog PLL Control4 Register
 * RESETVAL: 0x26 (38)
 * ACCESS:   R/W
 * FIELDS:
 *     DIV              Integer divider ratio.  Not used for fractional mode.In integer mode: pll_ctrl[63]=0, pll_ctrl[56]=1For PCIE with 54MHz internal refclk: pll_ctrl[63]=0, pll_ctrl[56:55]=11For SGMII with 156.25MHz refclk: 0 = x32For SGMII with 50MHz refclk: 6 = x100
 *     EN_CAP           Adjusts the C in the RC delay in the frequency doubler000: not used.001: 50fF010: 100fF011: 150fF100: 200fF101: 250fF110: 300fF111:  350fF
 *     RESERVED_74_71   
 *     MMD_RESETB       Reset the deltasigma Modulator. After reset the modulator outputs, M<4:0> and A<4:0> (See Fig.1) of deltasigma should be set to 0110 and 0000 respectively:This is active LOW for resetting the internal states. It also resets the internal states of the MMD PSC and newly added feature for resetting the mmd_resetb needs to see an active-low pulse (1 first, 0 for couple of microcontroller clock cycles and then goes to high, 1)The calibration cycle needs to start after the aforementioned signal be applied to mmd_resetb
 *     ADJ              Increases the reference current for the CP000: 50uA001: 50uA + 8%010: 50uA+ 16%...111: 50uA+ 56%
 *     MMD_PRSC8OR9PWDB Enables 8/9 prescaler. 0:disable, 1:enable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r (0x00008054 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_CTL4.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_CTL4r_s {
	uint32_t v[1];
	uint32_t pll_afe_ctl4[1];
	uint32_t _pll_afe_ctl4;
} BCMI_VIPER_XGXS_PLL_AFE_CTL4r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_CLR(r) (r).pll_afe_ctl4[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_SET(r,d) (r).pll_afe_ctl4[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_GET(r) (r).pll_afe_ctl4[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_MMD_PRSC8OR9PWDBf_GET(r) ((((r).pll_afe_ctl4[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_MMD_PRSC8OR9PWDBf_SET(r,f) (r).pll_afe_ctl4[0]=(((r).pll_afe_ctl4[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_ADJf_GET(r) ((((r).pll_afe_ctl4[0]) >> 12) & 0x7)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_ADJf_SET(r,f) (r).pll_afe_ctl4[0]=(((r).pll_afe_ctl4[0] & ~((uint32_t)0x7 << 12)) | ((((uint32_t)f) & 0x7) << 12)) | (7 << (16 + 12))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_MMD_RESETBf_GET(r) ((((r).pll_afe_ctl4[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_MMD_RESETBf_SET(r,f) (r).pll_afe_ctl4[0]=(((r).pll_afe_ctl4[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_RESERVED_74_71f_GET(r) ((((r).pll_afe_ctl4[0]) >> 7) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_RESERVED_74_71f_SET(r,f) (r).pll_afe_ctl4[0]=(((r).pll_afe_ctl4[0] & ~((uint32_t)0xf << 7)) | ((((uint32_t)f) & 0xf) << 7)) | (15 << (16 + 7))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_EN_CAPf_GET(r) ((((r).pll_afe_ctl4[0]) >> 4) & 0x7)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_EN_CAPf_SET(r,f) (r).pll_afe_ctl4[0]=(((r).pll_afe_ctl4[0] & ~((uint32_t)0x7 << 4)) | ((((uint32_t)f) & 0x7) << 4)) | (7 << (16 + 4))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_DIVf_GET(r) (((r).pll_afe_ctl4[0]) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL4r_DIVf_SET(r,f) (r).pll_afe_ctl4[0]=(((r).pll_afe_ctl4[0] & ~((uint32_t)0xf)) | (((uint32_t)f) & 0xf)) | (0xf << 16)

/*
 * These macros can be used to access PLL_AFE_CTL4.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_CTL4r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL4r,(_r._pll_afe_ctl4))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL4r,(_r._pll_afe_ctl4)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL4r,(_r._pll_afe_ctl4))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL4r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl4))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL4r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl4))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_ctl4))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_CTL4r BCMI_VIPER_XGXS_PLL_AFE_CTL4r
#define PLL_AFE_CTL4r_SIZE BCMI_VIPER_XGXS_PLL_AFE_CTL4r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_CTL4r_t PLL_AFE_CTL4r_t;
#define PLL_AFE_CTL4r_CLR BCMI_VIPER_XGXS_PLL_AFE_CTL4r_CLR
#define PLL_AFE_CTL4r_SET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_SET
#define PLL_AFE_CTL4r_GET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_GET
#define PLL_AFE_CTL4r_MMD_PRSC8OR9PWDBf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_MMD_PRSC8OR9PWDBf_GET
#define PLL_AFE_CTL4r_MMD_PRSC8OR9PWDBf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_MMD_PRSC8OR9PWDBf_SET
#define PLL_AFE_CTL4r_ADJf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_ADJf_GET
#define PLL_AFE_CTL4r_ADJf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_ADJf_SET
#define PLL_AFE_CTL4r_MMD_RESETBf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_MMD_RESETBf_GET
#define PLL_AFE_CTL4r_MMD_RESETBf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_MMD_RESETBf_SET
#define PLL_AFE_CTL4r_RESERVED_74_71f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_RESERVED_74_71f_GET
#define PLL_AFE_CTL4r_RESERVED_74_71f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_RESERVED_74_71f_SET
#define PLL_AFE_CTL4r_EN_CAPf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_EN_CAPf_GET
#define PLL_AFE_CTL4r_EN_CAPf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_EN_CAPf_SET
#define PLL_AFE_CTL4r_DIVf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_DIVf_GET
#define PLL_AFE_CTL4r_DIVf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL4r_DIVf_SET
#define READ_PLL_AFE_CTL4r BCMI_VIPER_XGXS_READ_PLL_AFE_CTL4r
#define WRITE_PLL_AFE_CTL4r BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL4r
#define MODIFY_PLL_AFE_CTL4r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL4r
#define READLN_PLL_AFE_CTL4r BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL4r
#define WRITELN_PLL_AFE_CTL4r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL4r
#define WRITEALL_PLL_AFE_CTL4r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL4r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_CTL4r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_CTL5
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x8055
 * DESC:     Analog PLL Control5 Register
 * RESETVAL: 0x44 (68)
 * ACCESS:   R/W
 * FIELDS:
 *     MMD_PRSC4OR5PWDB Enables 4/5 prescaler. 0:disable, 1:enable
 *     CALIB_ADJ        Sets the control voltage for the calibration000:75mV001:90mV010:105mV011:120mV100:135mV101:150mV110:165mV111:180mV
 *     RESERVED_84      
 *     EN_CUR           Adjusts the R in the RC delay in the  frequency doubler000: not used001:32k ohms010:16k ohms011: 10.7k ohms100: 8k ohms101: 6.4k ohms110: 5.3k ohms111: 4.6k ohms
 *     I_PFD_OFFSET_ENLARGE 1: In offset_PFD mode, offset is extended by ~300ps
 *     REF_DOUBLER_EN   1: Input reference clock is doubled in frequency to alleviate sigma delta noise for lower reference frequencies. The feedback divider ratio is divided by '2' to maintain PLL frequency
 *     I_PFD_OFFSET     In fractional PLL mode, reset pulse can be offset by 664ps(typical)/418ps(min), where the minimum is greater than 4 Tvco.00: regular pfd operation for integer PLL01: reference is offset.10: feedback is offset.11: not used.
 *     I_NDIV_FRAC_3_0  
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r (0x00008055 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_CTL5.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_CTL5r_s {
	uint32_t v[1];
	uint32_t pll_afe_ctl5[1];
	uint32_t _pll_afe_ctl5;
} BCMI_VIPER_XGXS_PLL_AFE_CTL5r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_CLR(r) (r).pll_afe_ctl5[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_SET(r,d) (r).pll_afe_ctl5[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_GET(r) (r).pll_afe_ctl5[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_NDIV_FRAC_3_0f_GET(r) ((((r).pll_afe_ctl5[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_NDIV_FRAC_3_0f_SET(r,f) (r).pll_afe_ctl5[0]=(((r).pll_afe_ctl5[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_PFD_OFFSETf_GET(r) ((((r).pll_afe_ctl5[0]) >> 10) & 0x3)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_PFD_OFFSETf_SET(r,f) (r).pll_afe_ctl5[0]=(((r).pll_afe_ctl5[0] & ~((uint32_t)0x3 << 10)) | ((((uint32_t)f) & 0x3) << 10)) | (3 << (16 + 10))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_REF_DOUBLER_ENf_GET(r) ((((r).pll_afe_ctl5[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_REF_DOUBLER_ENf_SET(r,f) (r).pll_afe_ctl5[0]=(((r).pll_afe_ctl5[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_PFD_OFFSET_ENLARGEf_GET(r) ((((r).pll_afe_ctl5[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_PFD_OFFSET_ENLARGEf_SET(r,f) (r).pll_afe_ctl5[0]=(((r).pll_afe_ctl5[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_EN_CURf_GET(r) ((((r).pll_afe_ctl5[0]) >> 5) & 0x7)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_EN_CURf_SET(r,f) (r).pll_afe_ctl5[0]=(((r).pll_afe_ctl5[0] & ~((uint32_t)0x7 << 5)) | ((((uint32_t)f) & 0x7) << 5)) | (7 << (16 + 5))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_RESERVED_84f_GET(r) ((((r).pll_afe_ctl5[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_RESERVED_84f_SET(r,f) (r).pll_afe_ctl5[0]=(((r).pll_afe_ctl5[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_CALIB_ADJf_GET(r) ((((r).pll_afe_ctl5[0]) >> 1) & 0x7)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_CALIB_ADJf_SET(r,f) (r).pll_afe_ctl5[0]=(((r).pll_afe_ctl5[0] & ~((uint32_t)0x7 << 1)) | ((((uint32_t)f) & 0x7) << 1)) | (7 << (16 + 1))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_MMD_PRSC4OR5PWDBf_GET(r) (((r).pll_afe_ctl5[0]) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL5r_MMD_PRSC4OR5PWDBf_SET(r,f) (r).pll_afe_ctl5[0]=(((r).pll_afe_ctl5[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access PLL_AFE_CTL5.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_CTL5r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL5r,(_r._pll_afe_ctl5))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL5r,(_r._pll_afe_ctl5)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL5r,(_r._pll_afe_ctl5))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL5r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl5))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL5r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl5))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_ctl5))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_CTL5r BCMI_VIPER_XGXS_PLL_AFE_CTL5r
#define PLL_AFE_CTL5r_SIZE BCMI_VIPER_XGXS_PLL_AFE_CTL5r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_CTL5r_t PLL_AFE_CTL5r_t;
#define PLL_AFE_CTL5r_CLR BCMI_VIPER_XGXS_PLL_AFE_CTL5r_CLR
#define PLL_AFE_CTL5r_SET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_SET
#define PLL_AFE_CTL5r_GET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_GET
#define PLL_AFE_CTL5r_I_NDIV_FRAC_3_0f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_NDIV_FRAC_3_0f_GET
#define PLL_AFE_CTL5r_I_NDIV_FRAC_3_0f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_NDIV_FRAC_3_0f_SET
#define PLL_AFE_CTL5r_I_PFD_OFFSETf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_PFD_OFFSETf_GET
#define PLL_AFE_CTL5r_I_PFD_OFFSETf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_PFD_OFFSETf_SET
#define PLL_AFE_CTL5r_REF_DOUBLER_ENf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_REF_DOUBLER_ENf_GET
#define PLL_AFE_CTL5r_REF_DOUBLER_ENf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_REF_DOUBLER_ENf_SET
#define PLL_AFE_CTL5r_I_PFD_OFFSET_ENLARGEf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_PFD_OFFSET_ENLARGEf_GET
#define PLL_AFE_CTL5r_I_PFD_OFFSET_ENLARGEf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_I_PFD_OFFSET_ENLARGEf_SET
#define PLL_AFE_CTL5r_EN_CURf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_EN_CURf_GET
#define PLL_AFE_CTL5r_EN_CURf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_EN_CURf_SET
#define PLL_AFE_CTL5r_RESERVED_84f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_RESERVED_84f_GET
#define PLL_AFE_CTL5r_RESERVED_84f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_RESERVED_84f_SET
#define PLL_AFE_CTL5r_CALIB_ADJf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_CALIB_ADJf_GET
#define PLL_AFE_CTL5r_CALIB_ADJf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_CALIB_ADJf_SET
#define PLL_AFE_CTL5r_MMD_PRSC4OR5PWDBf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_MMD_PRSC4OR5PWDBf_GET
#define PLL_AFE_CTL5r_MMD_PRSC4OR5PWDBf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL5r_MMD_PRSC4OR5PWDBf_SET
#define READ_PLL_AFE_CTL5r BCMI_VIPER_XGXS_READ_PLL_AFE_CTL5r
#define WRITE_PLL_AFE_CTL5r BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL5r
#define MODIFY_PLL_AFE_CTL5r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL5r
#define READLN_PLL_AFE_CTL5r BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL5r
#define WRITELN_PLL_AFE_CTL5r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL5r
#define WRITEALL_PLL_AFE_CTL5r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL5r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_CTL5r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_CTL6
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x8056
 * DESC:     Analog PLL Control6 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     I_NDIV_FRAC_17_4 Fractional control of feedback dividerDivider fraction=i_ndiv_frac[109:92]/2^18For OC192, this value is set to binary( 0.70099*2^18)=10 1100 1101 1101 0000
 *     I_NDIV_INT_1_0   
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_CTL6r (0x00008056 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_CTL6r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_CTL6.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_CTL6r_s {
	uint32_t v[1];
	uint32_t pll_afe_ctl6[1];
	uint32_t _pll_afe_ctl6;
} BCMI_VIPER_XGXS_PLL_AFE_CTL6r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_CTL6r_CLR(r) (r).pll_afe_ctl6[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_CTL6r_SET(r,d) (r).pll_afe_ctl6[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_CTL6r_GET(r) (r).pll_afe_ctl6[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_CTL6r_I_NDIV_INT_1_0f_GET(r) ((((r).pll_afe_ctl6[0]) >> 14) & 0x3)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL6r_I_NDIV_INT_1_0f_SET(r,f) (r).pll_afe_ctl6[0]=(((r).pll_afe_ctl6[0] & ~((uint32_t)0x3 << 14)) | ((((uint32_t)f) & 0x3) << 14)) | (3 << (16 + 14))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL6r_I_NDIV_FRAC_17_4f_GET(r) (((r).pll_afe_ctl6[0]) & 0x3fff)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL6r_I_NDIV_FRAC_17_4f_SET(r,f) (r).pll_afe_ctl6[0]=(((r).pll_afe_ctl6[0] & ~((uint32_t)0x3fff)) | (((uint32_t)f) & 0x3fff)) | (0x3fff << 16)

/*
 * These macros can be used to access PLL_AFE_CTL6.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_CTL6r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL6r,(_r._pll_afe_ctl6))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL6r,(_r._pll_afe_ctl6)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL6r,(_r._pll_afe_ctl6))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL6r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl6))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL6r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl6))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_ctl6))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_CTL6r BCMI_VIPER_XGXS_PLL_AFE_CTL6r
#define PLL_AFE_CTL6r_SIZE BCMI_VIPER_XGXS_PLL_AFE_CTL6r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_CTL6r_t PLL_AFE_CTL6r_t;
#define PLL_AFE_CTL6r_CLR BCMI_VIPER_XGXS_PLL_AFE_CTL6r_CLR
#define PLL_AFE_CTL6r_SET BCMI_VIPER_XGXS_PLL_AFE_CTL6r_SET
#define PLL_AFE_CTL6r_GET BCMI_VIPER_XGXS_PLL_AFE_CTL6r_GET
#define PLL_AFE_CTL6r_I_NDIV_INT_1_0f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL6r_I_NDIV_INT_1_0f_GET
#define PLL_AFE_CTL6r_I_NDIV_INT_1_0f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL6r_I_NDIV_INT_1_0f_SET
#define PLL_AFE_CTL6r_I_NDIV_FRAC_17_4f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL6r_I_NDIV_FRAC_17_4f_GET
#define PLL_AFE_CTL6r_I_NDIV_FRAC_17_4f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL6r_I_NDIV_FRAC_17_4f_SET
#define READ_PLL_AFE_CTL6r BCMI_VIPER_XGXS_READ_PLL_AFE_CTL6r
#define WRITE_PLL_AFE_CTL6r BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL6r
#define MODIFY_PLL_AFE_CTL6r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL6r
#define READLN_PLL_AFE_CTL6r BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL6r
#define WRITELN_PLL_AFE_CTL6r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL6r
#define WRITEALL_PLL_AFE_CTL6r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL6r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_CTL6r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_CTL7
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x8057
 * DESC:     Analog PLL Control7 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     I_NDIV_INT_9_2   Feedback divider integer control (divider ratio=code)As shown in Fig.3 (page5) this value is added to fractional
 *     MMD_DIV_RANGE    Select mmd division range: (depends on division ratio)0= mmd uses div8/9 pre-scalar1= mmd uses div4/5 pre-scalar
 *     I_NDIV_DITHER_EN Enable 1 bit dithering of the fractional input i_pll_ctrl<83:66>0= dithering OFF1= dithering ON
 *     INV_VCO_CAL      Inverts the logic of vco calibration clock going to RTL from rising edge to falling edge
 *     I_PLL_SDM_PWRDNB Power down sdm rtl only when LOW(1: for fractional mode)
 *     SEL_FP3CAP       Controls the filter cap for the third loop filter pole in WIS modeSel_fp3cap<4:0>	Cap value0001	1p0010	1p0100	3p1000	3p
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r (0x00008057 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_CTL7.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_CTL7r_s {
	uint32_t v[1];
	uint32_t pll_afe_ctl7[1];
	uint32_t _pll_afe_ctl7;
} BCMI_VIPER_XGXS_PLL_AFE_CTL7r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_CLR(r) (r).pll_afe_ctl7[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_SET(r,d) (r).pll_afe_ctl7[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_GET(r) (r).pll_afe_ctl7[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_SEL_FP3CAPf_GET(r) ((((r).pll_afe_ctl7[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_SEL_FP3CAPf_SET(r,f) (r).pll_afe_ctl7[0]=(((r).pll_afe_ctl7[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_PLL_SDM_PWRDNBf_GET(r) ((((r).pll_afe_ctl7[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_PLL_SDM_PWRDNBf_SET(r,f) (r).pll_afe_ctl7[0]=(((r).pll_afe_ctl7[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_INV_VCO_CALf_GET(r) ((((r).pll_afe_ctl7[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_INV_VCO_CALf_SET(r,f) (r).pll_afe_ctl7[0]=(((r).pll_afe_ctl7[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_NDIV_DITHER_ENf_GET(r) ((((r).pll_afe_ctl7[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_NDIV_DITHER_ENf_SET(r,f) (r).pll_afe_ctl7[0]=(((r).pll_afe_ctl7[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_MMD_DIV_RANGEf_GET(r) ((((r).pll_afe_ctl7[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_MMD_DIV_RANGEf_SET(r,f) (r).pll_afe_ctl7[0]=(((r).pll_afe_ctl7[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_NDIV_INT_9_2f_GET(r) (((r).pll_afe_ctl7[0]) & 0xff)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_NDIV_INT_9_2f_SET(r,f) (r).pll_afe_ctl7[0]=(((r).pll_afe_ctl7[0] & ~((uint32_t)0xff)) | (((uint32_t)f) & 0xff)) | (0xff << 16)

/*
 * These macros can be used to access PLL_AFE_CTL7.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_CTL7r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL7r,(_r._pll_afe_ctl7))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL7r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL7r,(_r._pll_afe_ctl7)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL7r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL7r,(_r._pll_afe_ctl7))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL7r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL7r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl7))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL7r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL7r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl7))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL7r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL7r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_ctl7))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_CTL7r BCMI_VIPER_XGXS_PLL_AFE_CTL7r
#define PLL_AFE_CTL7r_SIZE BCMI_VIPER_XGXS_PLL_AFE_CTL7r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_CTL7r_t PLL_AFE_CTL7r_t;
#define PLL_AFE_CTL7r_CLR BCMI_VIPER_XGXS_PLL_AFE_CTL7r_CLR
#define PLL_AFE_CTL7r_SET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_SET
#define PLL_AFE_CTL7r_GET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_GET
#define PLL_AFE_CTL7r_SEL_FP3CAPf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_SEL_FP3CAPf_GET
#define PLL_AFE_CTL7r_SEL_FP3CAPf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_SEL_FP3CAPf_SET
#define PLL_AFE_CTL7r_I_PLL_SDM_PWRDNBf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_PLL_SDM_PWRDNBf_GET
#define PLL_AFE_CTL7r_I_PLL_SDM_PWRDNBf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_PLL_SDM_PWRDNBf_SET
#define PLL_AFE_CTL7r_INV_VCO_CALf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_INV_VCO_CALf_GET
#define PLL_AFE_CTL7r_INV_VCO_CALf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_INV_VCO_CALf_SET
#define PLL_AFE_CTL7r_I_NDIV_DITHER_ENf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_NDIV_DITHER_ENf_GET
#define PLL_AFE_CTL7r_I_NDIV_DITHER_ENf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_NDIV_DITHER_ENf_SET
#define PLL_AFE_CTL7r_MMD_DIV_RANGEf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_MMD_DIV_RANGEf_GET
#define PLL_AFE_CTL7r_MMD_DIV_RANGEf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_MMD_DIV_RANGEf_SET
#define PLL_AFE_CTL7r_I_NDIV_INT_9_2f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_NDIV_INT_9_2f_GET
#define PLL_AFE_CTL7r_I_NDIV_INT_9_2f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL7r_I_NDIV_INT_9_2f_SET
#define READ_PLL_AFE_CTL7r BCMI_VIPER_XGXS_READ_PLL_AFE_CTL7r
#define WRITE_PLL_AFE_CTL7r BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL7r
#define MODIFY_PLL_AFE_CTL7r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL7r
#define READLN_PLL_AFE_CTL7r BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL7r
#define WRITELN_PLL_AFE_CTL7r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL7r
#define WRITEALL_PLL_AFE_CTL7r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL7r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_CTL7r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_CTL8
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x8058
 * DESC:     Analog PLL Control8 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     VDD1P0_PLL_EN    0: lower vdd supply operation range    0.81  0.95 V1: higher vdd supply operation range   0.90  1.05 VWhen disabled the bias current for the pll is increased by ~ 10%This will affect the RX operation
 *     RESERVED_143_129 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_CTL8r (0x00008058 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_CTL8r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_CTL8.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_CTL8r_s {
	uint32_t v[1];
	uint32_t pll_afe_ctl8[1];
	uint32_t _pll_afe_ctl8;
} BCMI_VIPER_XGXS_PLL_AFE_CTL8r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_CTL8r_CLR(r) (r).pll_afe_ctl8[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_CTL8r_SET(r,d) (r).pll_afe_ctl8[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_CTL8r_GET(r) (r).pll_afe_ctl8[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_CTL8r_RESERVED_143_129f_GET(r) ((((r).pll_afe_ctl8[0]) >> 1) & 0x7fff)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL8r_RESERVED_143_129f_SET(r,f) (r).pll_afe_ctl8[0]=(((r).pll_afe_ctl8[0] & ~((uint32_t)0x7fff << 1)) | ((((uint32_t)f) & 0x7fff) << 1)) | (32767 << (16 + 1))
#define BCMI_VIPER_XGXS_PLL_AFE_CTL8r_VDD1P0_PLL_ENf_GET(r) (((r).pll_afe_ctl8[0]) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_CTL8r_VDD1P0_PLL_ENf_SET(r,f) (r).pll_afe_ctl8[0]=(((r).pll_afe_ctl8[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access PLL_AFE_CTL8.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_CTL8r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL8r,(_r._pll_afe_ctl8))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL8r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL8r,(_r._pll_afe_ctl8)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL8r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL8r,(_r._pll_afe_ctl8))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL8r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL8r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl8))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL8r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL8r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_ctl8))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL8r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_CTL8r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_ctl8))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_CTL8r BCMI_VIPER_XGXS_PLL_AFE_CTL8r
#define PLL_AFE_CTL8r_SIZE BCMI_VIPER_XGXS_PLL_AFE_CTL8r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_CTL8r_t PLL_AFE_CTL8r_t;
#define PLL_AFE_CTL8r_CLR BCMI_VIPER_XGXS_PLL_AFE_CTL8r_CLR
#define PLL_AFE_CTL8r_SET BCMI_VIPER_XGXS_PLL_AFE_CTL8r_SET
#define PLL_AFE_CTL8r_GET BCMI_VIPER_XGXS_PLL_AFE_CTL8r_GET
#define PLL_AFE_CTL8r_RESERVED_143_129f_GET BCMI_VIPER_XGXS_PLL_AFE_CTL8r_RESERVED_143_129f_GET
#define PLL_AFE_CTL8r_RESERVED_143_129f_SET BCMI_VIPER_XGXS_PLL_AFE_CTL8r_RESERVED_143_129f_SET
#define PLL_AFE_CTL8r_VDD1P0_PLL_ENf_GET BCMI_VIPER_XGXS_PLL_AFE_CTL8r_VDD1P0_PLL_ENf_GET
#define PLL_AFE_CTL8r_VDD1P0_PLL_ENf_SET BCMI_VIPER_XGXS_PLL_AFE_CTL8r_VDD1P0_PLL_ENf_SET
#define READ_PLL_AFE_CTL8r BCMI_VIPER_XGXS_READ_PLL_AFE_CTL8r
#define WRITE_PLL_AFE_CTL8r BCMI_VIPER_XGXS_WRITE_PLL_AFE_CTL8r
#define MODIFY_PLL_AFE_CTL8r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_CTL8r
#define READLN_PLL_AFE_CTL8r BCMI_VIPER_XGXS_READLN_PLL_AFE_CTL8r
#define WRITELN_PLL_AFE_CTL8r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_CTL8r
#define WRITEALL_PLL_AFE_CTL8r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_CTL8r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_CTL8r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL_AFE_TCATIMER1
 * BLOCKS:   PLL_AFE
 * REGADDR:  0x805a
 * DESC:     Analog PLL Control8 Register
 * RESETVAL: 0x400 (1024)
 * ACCESS:   R/W
 * FIELDS:
 *      XGXS_SEL        Boolean
 *      INTERP_OVERRIDE Boolean             
 *      TCA_FINE_TIMER  Reset value is 0x400 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r (0x0000805A | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_SIZE 4

/*
 * This structure should be used to declare and program PLL_AFE_TCATIMER1.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_s {
	uint32_t v[1];
	uint32_t pll_afe_tcatimer1[1];
	uint32_t _pll_afe_tcatimer1;
} BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_t;

#define BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_CLR(r) (r).pll_afe_tcatimer1[0] = 0
#define BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_SET(r,d) (r).pll_afe_tcatimer1[0] = d
#define BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_GET(r) (r).pll_afe_tcatimer1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_XGXS_SELf_GET(r) ((((r).pll_afe_tcatimer1[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_XGXS_SELf_SET(r,f) (r).pll_afe_tcatimer1[0]=(((r).pll_afe_tcatimer1[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_INTERP_OVERRIDEf_GET(r) ((((r).pll_afe_tcatimer[0]) >> 14 )& 0x1)
#define BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_INTERP_OVERRIDEf_SET(r,f) (r).pll_afe_tcatimer[0]=(((r).pll_afe_tcatimer[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (1 << (16 + 14))

/*
 * These macros can be used to access PLL_AFE_TCATIMER1.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL_AFE_TCATIMER1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r,(_r._pll_afe_tcatimer1))
#define BCMI_VIPER_XGXS_WRITE_PLL_AFE_TCATIMER1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r,(_r._pll_afe_tcatimer1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL_AFE_TCATIMER1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r,(_r._pll_afe_tcatimer1))
#define BCMI_VIPER_XGXS_READLN_PLL_AFE_TCATIMER1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_tcatimer1))
#define BCMI_VIPER_XGXS_WRITELN_PLL_AFE_TCATIMER1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll_afe_tcatimer1))
#define BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_TCATIMER1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll_afe_tcatimer1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL_AFE_TCATIMER1r BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r
#define PLL_AFE_TCATIMER1r_SIZE BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_SIZE
typedef BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_t PLL_AFE_TCATIMER1r_t;
#define PLL_AFE_TCATIMER1r_CLR BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_CLR
#define PLL_AFE_TCATIMER1r_SET BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_SET
#define PLL_AFE_TCATIMER1r_GET BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_GET
#define PLL_AFE_TCATIMER1r_XGXS_SELf_GET BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_XGXS_SELf_GET
#define PLL_AFE_TCATIMER1r_XGXS_SELf_SET BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_XGXS_SELf_SET
#define PLL_AFE_TCATIMER1r_INTERP_OVERRIDEf_GET BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_INTERP_OVERRIDEf_GET
#define PLL_AFE_TCATIMER1r_INTERP_OVERRIDEf_SET BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r_INTERP_OVERRIDEf_SET
#define READ_PLL_AFE_TCATIMER1r BCMI_VIPER_XGXS_READ_PLL_AFE_TCATIMER1r
#define WRITE_PLL_AFE_TCATIMER1r BCMI_VIPER_XGXS_WRITE_PLL_AFE_TCATIMER1r
#define MODIFY_PLL_AFE_TCATIMER1r BCMI_VIPER_XGXS_MODIFY_PLL_AFE_TCATIMER1r
#define READLN_PLL_AFE_TCATIMER1r BCMI_VIPER_XGXS_READLN_PLL_AFE_TCATIMER1r
#define WRITELN_PLL_AFE_TCATIMER1r BCMI_VIPER_XGXS_WRITELN_PLL_AFE_TCATIMER1r
#define WRITEALL_PLL_AFE_TCATIMER1r BCMI_VIPER_XGXS_WRITEALL_PLL_AFE_TCATIMER1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL_AFE_TCATIMER1r'
 ******************************************************************************/



/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TX_AFE_ANATXASTS0
 * BLOCKS:   TX_AFE
 * REGADDR:  0x8060
 * DESC:     Tx analog status 0 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     TXPLL_LOCK       
 *     RLTXFERR_STKY    R-Loop FIFO error, sticky
 *     TX_PWRDN         
 *     TX_RESET         
 *     TBI_MODE         
 *     TXFERR_STKY      
 *     TXDISABLE_LN     
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r (0x00008060 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_SIZE 4

/*
 * This structure should be used to declare and program TX_AFE_ANATXASTS0.
 *
 */
typedef union BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_s {
	uint32_t v[1];
	uint32_t tx_afe_anatxasts0[1];
	uint32_t _tx_afe_anatxasts0;
} BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_t;

#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_CLR(r) (r).tx_afe_anatxasts0[0] = 0
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_SET(r,d) (r).tx_afe_anatxasts0[0] = d
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_GET(r) (r).tx_afe_anatxasts0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXDISABLE_LNf_GET(r) ((((r).tx_afe_anatxasts0[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXDISABLE_LNf_SET(r,f) (r).tx_afe_anatxasts0[0]=(((r).tx_afe_anatxasts0[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXFERR_STKYf_GET(r) ((((r).tx_afe_anatxasts0[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXFERR_STKYf_SET(r,f) (r).tx_afe_anatxasts0[0]=(((r).tx_afe_anatxasts0[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TBI_MODEf_GET(r) ((((r).tx_afe_anatxasts0[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TBI_MODEf_SET(r,f) (r).tx_afe_anatxasts0[0]=(((r).tx_afe_anatxasts0[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TX_RESETf_GET(r) ((((r).tx_afe_anatxasts0[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TX_RESETf_SET(r,f) (r).tx_afe_anatxasts0[0]=(((r).tx_afe_anatxasts0[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TX_PWRDNf_GET(r) ((((r).tx_afe_anatxasts0[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TX_PWRDNf_SET(r,f) (r).tx_afe_anatxasts0[0]=(((r).tx_afe_anatxasts0[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_RLTXFERR_STKYf_GET(r) ((((r).tx_afe_anatxasts0[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_RLTXFERR_STKYf_SET(r,f) (r).tx_afe_anatxasts0[0]=(((r).tx_afe_anatxasts0[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXPLL_LOCKf_GET(r) (((r).tx_afe_anatxasts0[0]) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXPLL_LOCKf_SET(r,f) (r).tx_afe_anatxasts0[0]=(((r).tx_afe_anatxasts0[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access TX_AFE_ANATXASTS0.
 *
 */
#define BCMI_VIPER_XGXS_READ_TX_AFE_ANATXASTS0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r,(_r._tx_afe_anatxasts0))
#define BCMI_VIPER_XGXS_WRITE_TX_AFE_ANATXASTS0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r,(_r._tx_afe_anatxasts0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TX_AFE_ANATXASTS0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r,(_r._tx_afe_anatxasts0))
#define BCMI_VIPER_XGXS_READLN_TX_AFE_ANATXASTS0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_anatxasts0))
#define BCMI_VIPER_XGXS_WRITELN_TX_AFE_ANATXASTS0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_anatxasts0))
#define BCMI_VIPER_XGXS_WRITEALL_TX_AFE_ANATXASTS0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._tx_afe_anatxasts0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TX_AFE_ANATXASTS0r BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r
#define TX_AFE_ANATXASTS0r_SIZE BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_SIZE
typedef BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_t TX_AFE_ANATXASTS0r_t;
#define TX_AFE_ANATXASTS0r_CLR BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_CLR
#define TX_AFE_ANATXASTS0r_SET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_SET
#define TX_AFE_ANATXASTS0r_GET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_GET
#define TX_AFE_ANATXASTS0r_TXDISABLE_LNf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXDISABLE_LNf_GET
#define TX_AFE_ANATXASTS0r_TXDISABLE_LNf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXDISABLE_LNf_SET
#define TX_AFE_ANATXASTS0r_TXFERR_STKYf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXFERR_STKYf_GET
#define TX_AFE_ANATXASTS0r_TXFERR_STKYf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXFERR_STKYf_SET
#define TX_AFE_ANATXASTS0r_TBI_MODEf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TBI_MODEf_GET
#define TX_AFE_ANATXASTS0r_TBI_MODEf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TBI_MODEf_SET
#define TX_AFE_ANATXASTS0r_TX_RESETf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TX_RESETf_GET
#define TX_AFE_ANATXASTS0r_TX_RESETf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TX_RESETf_SET
#define TX_AFE_ANATXASTS0r_TX_PWRDNf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TX_PWRDNf_GET
#define TX_AFE_ANATXASTS0r_TX_PWRDNf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TX_PWRDNf_SET
#define TX_AFE_ANATXASTS0r_RLTXFERR_STKYf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_RLTXFERR_STKYf_GET
#define TX_AFE_ANATXASTS0r_RLTXFERR_STKYf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_RLTXFERR_STKYf_SET
#define TX_AFE_ANATXASTS0r_TXPLL_LOCKf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXPLL_LOCKf_GET
#define TX_AFE_ANATXASTS0r_TXPLL_LOCKf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r_TXPLL_LOCKf_SET
#define READ_TX_AFE_ANATXASTS0r BCMI_VIPER_XGXS_READ_TX_AFE_ANATXASTS0r
#define WRITE_TX_AFE_ANATXASTS0r BCMI_VIPER_XGXS_WRITE_TX_AFE_ANATXASTS0r
#define MODIFY_TX_AFE_ANATXASTS0r BCMI_VIPER_XGXS_MODIFY_TX_AFE_ANATXASTS0r
#define READLN_TX_AFE_ANATXASTS0r BCMI_VIPER_XGXS_READLN_TX_AFE_ANATXASTS0r
#define WRITELN_TX_AFE_ANATXASTS0r BCMI_VIPER_XGXS_WRITELN_TX_AFE_ANATXASTS0r
#define WRITEALL_TX_AFE_ANATXASTS0r BCMI_VIPER_XGXS_WRITEALL_TX_AFE_ANATXASTS0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TX_AFE_ANATXASTS0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TX_AFE_ANATXACTL0
 * BLOCKS:   TX_AFE
 * REGADDR:  0x8061
 * DESC:     Tx analog control 0 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TX_MDATA_EN      Enables tx mdio data
 *     TXPAT_EN         Enables tx test pattern
 *     EDEN_FORCE_R     Enables eden_r control bit
 *     EDEN_R           8B/10B enable
 *     RTBI_FLIP        Flip RTBI nibble sequence
 *     TXPOL_FLIP       Flip the analog transmit output
 *     PCKT_STRT        Tx BERT packet start
 *     PCKT_EN          Tx BERT Packet mode enable
 *     PRBS_EN          Tx BERT enable
 *     GLOOPOUTDIS      Disable serializer output data when G-Loop is enabled.
 *     TXCK_DME_EN_SM   
 *     CATCH_ALL_8B10B_DIS Disables the catch-all case for invalid code-groups.
 *     FORCE_EXT_FRST_SM 
 *     TX1G_FIFO_RST    Tx 1G FIFO reset
 *     FORCE_TXCLK      Force txclk (bypass clock switch)
 *     MDIO_FORCE       
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r (0x00008061 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_SIZE 4

/*
 * This structure should be used to declare and program TX_AFE_ANATXACTL0.
 *
 */
typedef union BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_s {
	uint32_t v[1];
	uint32_t tx_afe_anatxactl0[1];
	uint32_t _tx_afe_anatxactl0;
} BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_t;

#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_CLR(r) (r).tx_afe_anatxactl0[0] = 0
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_SET(r,d) (r).tx_afe_anatxactl0[0] = d
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_GET(r) (r).tx_afe_anatxactl0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_MDIO_FORCEf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_MDIO_FORCEf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_FORCE_TXCLKf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_FORCE_TXCLKf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TX1G_FIFO_RSTf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TX1G_FIFO_RSTf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_FORCE_EXT_FRST_SMf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_FORCE_EXT_FRST_SMf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_CATCH_ALL_8B10B_DISf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_CATCH_ALL_8B10B_DISf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXCK_DME_EN_SMf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXCK_DME_EN_SMf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_GLOOPOUTDISf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_GLOOPOUTDISf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PRBS_ENf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PRBS_ENf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PCKT_ENf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PCKT_ENf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PCKT_STRTf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PCKT_STRTf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXPOL_FLIPf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXPOL_FLIPf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_RTBI_FLIPf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_RTBI_FLIPf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_EDEN_Rf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_EDEN_Rf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_EDEN_FORCE_Rf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_EDEN_FORCE_Rf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXPAT_ENf_GET(r) ((((r).tx_afe_anatxactl0[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXPAT_ENf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TX_MDATA_ENf_GET(r) (((r).tx_afe_anatxactl0[0]) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TX_MDATA_ENf_SET(r,f) (r).tx_afe_anatxactl0[0]=(((r).tx_afe_anatxactl0[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access TX_AFE_ANATXACTL0.
 *
 */
#define BCMI_VIPER_XGXS_READ_TX_AFE_ANATXACTL0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r,(_r._tx_afe_anatxactl0))
#define BCMI_VIPER_XGXS_WRITE_TX_AFE_ANATXACTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r,(_r._tx_afe_anatxactl0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TX_AFE_ANATXACTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r,(_r._tx_afe_anatxactl0))
#define BCMI_VIPER_XGXS_READLN_TX_AFE_ANATXACTL0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_anatxactl0))
#define BCMI_VIPER_XGXS_WRITELN_TX_AFE_ANATXACTL0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_anatxactl0))
#define BCMI_VIPER_XGXS_WRITEALL_TX_AFE_ANATXACTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._tx_afe_anatxactl0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TX_AFE_ANATXACTL0r BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r
#define TX_AFE_ANATXACTL0r_SIZE BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_SIZE
typedef BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_t TX_AFE_ANATXACTL0r_t;
#define TX_AFE_ANATXACTL0r_CLR BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_CLR
#define TX_AFE_ANATXACTL0r_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_SET
#define TX_AFE_ANATXACTL0r_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_GET
#define TX_AFE_ANATXACTL0r_MDIO_FORCEf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_MDIO_FORCEf_GET
#define TX_AFE_ANATXACTL0r_MDIO_FORCEf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_MDIO_FORCEf_SET
#define TX_AFE_ANATXACTL0r_FORCE_TXCLKf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_FORCE_TXCLKf_GET
#define TX_AFE_ANATXACTL0r_FORCE_TXCLKf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_FORCE_TXCLKf_SET
#define TX_AFE_ANATXACTL0r_TX1G_FIFO_RSTf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TX1G_FIFO_RSTf_GET
#define TX_AFE_ANATXACTL0r_TX1G_FIFO_RSTf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TX1G_FIFO_RSTf_SET
#define TX_AFE_ANATXACTL0r_FORCE_EXT_FRST_SMf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_FORCE_EXT_FRST_SMf_GET
#define TX_AFE_ANATXACTL0r_FORCE_EXT_FRST_SMf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_FORCE_EXT_FRST_SMf_SET
#define TX_AFE_ANATXACTL0r_CATCH_ALL_8B10B_DISf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_CATCH_ALL_8B10B_DISf_GET
#define TX_AFE_ANATXACTL0r_CATCH_ALL_8B10B_DISf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_CATCH_ALL_8B10B_DISf_SET
#define TX_AFE_ANATXACTL0r_TXCK_DME_EN_SMf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXCK_DME_EN_SMf_GET
#define TX_AFE_ANATXACTL0r_TXCK_DME_EN_SMf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXCK_DME_EN_SMf_SET
#define TX_AFE_ANATXACTL0r_GLOOPOUTDISf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_GLOOPOUTDISf_GET
#define TX_AFE_ANATXACTL0r_GLOOPOUTDISf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_GLOOPOUTDISf_SET
#define TX_AFE_ANATXACTL0r_PRBS_ENf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PRBS_ENf_GET
#define TX_AFE_ANATXACTL0r_PRBS_ENf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PRBS_ENf_SET
#define TX_AFE_ANATXACTL0r_PCKT_ENf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PCKT_ENf_GET
#define TX_AFE_ANATXACTL0r_PCKT_ENf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PCKT_ENf_SET
#define TX_AFE_ANATXACTL0r_PCKT_STRTf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PCKT_STRTf_GET
#define TX_AFE_ANATXACTL0r_PCKT_STRTf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_PCKT_STRTf_SET
#define TX_AFE_ANATXACTL0r_TXPOL_FLIPf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXPOL_FLIPf_GET
#define TX_AFE_ANATXACTL0r_TXPOL_FLIPf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXPOL_FLIPf_SET
#define TX_AFE_ANATXACTL0r_RTBI_FLIPf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_RTBI_FLIPf_GET
#define TX_AFE_ANATXACTL0r_RTBI_FLIPf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_RTBI_FLIPf_SET
#define TX_AFE_ANATXACTL0r_EDEN_Rf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_EDEN_Rf_GET
#define TX_AFE_ANATXACTL0r_EDEN_Rf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_EDEN_Rf_SET
#define TX_AFE_ANATXACTL0r_EDEN_FORCE_Rf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_EDEN_FORCE_Rf_GET
#define TX_AFE_ANATXACTL0r_EDEN_FORCE_Rf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_EDEN_FORCE_Rf_SET
#define TX_AFE_ANATXACTL0r_TXPAT_ENf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXPAT_ENf_GET
#define TX_AFE_ANATXACTL0r_TXPAT_ENf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TXPAT_ENf_SET
#define TX_AFE_ANATXACTL0r_TX_MDATA_ENf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TX_MDATA_ENf_GET
#define TX_AFE_ANATXACTL0r_TX_MDATA_ENf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r_TX_MDATA_ENf_SET
#define READ_TX_AFE_ANATXACTL0r BCMI_VIPER_XGXS_READ_TX_AFE_ANATXACTL0r
#define WRITE_TX_AFE_ANATXACTL0r BCMI_VIPER_XGXS_WRITE_TX_AFE_ANATXACTL0r
#define MODIFY_TX_AFE_ANATXACTL0r BCMI_VIPER_XGXS_MODIFY_TX_AFE_ANATXACTL0r
#define READLN_TX_AFE_ANATXACTL0r BCMI_VIPER_XGXS_READLN_TX_AFE_ANATXACTL0r
#define WRITELN_TX_AFE_ANATXACTL0r BCMI_VIPER_XGXS_WRITELN_TX_AFE_ANATXACTL0r
#define WRITEALL_TX_AFE_ANATXACTL0r BCMI_VIPER_XGXS_WRITEALL_TX_AFE_ANATXACTL0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TX_AFE_ANATXACTL0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TX_AFE_ANATXMDATA0
 * BLOCKS:   TX_AFE
 * REGADDR:  0x8062
 * DESC:     Tx test mux data 0 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TXMDIOTSTDATAL   lower 10 bits of pattern
 *     RLFIFO_TSTSEL    
 *     TXTESTMUXSEL     
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r (0x00008062 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_SIZE 4

/*
 * This structure should be used to declare and program TX_AFE_ANATXMDATA0.
 *
 */
typedef union BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_s {
	uint32_t v[1];
	uint32_t tx_afe_anatxmdata0[1];
	uint32_t _tx_afe_anatxmdata0;
} BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_t;

#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_CLR(r) (r).tx_afe_anatxmdata0[0] = 0
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_SET(r,d) (r).tx_afe_anatxmdata0[0] = d
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_GET(r) (r).tx_afe_anatxmdata0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_TXTESTMUXSELf_GET(r) ((((r).tx_afe_anatxmdata0[0]) >> 13) & 0x7)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_TXTESTMUXSELf_SET(r,f) (r).tx_afe_anatxmdata0[0]=(((r).tx_afe_anatxmdata0[0] & ~((uint32_t)0x7 << 13)) | ((((uint32_t)f) & 0x7) << 13)) | (7 << (16 + 13))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_RLFIFO_TSTSELf_GET(r) ((((r).tx_afe_anatxmdata0[0]) >> 10) & 0x7)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_RLFIFO_TSTSELf_SET(r,f) (r).tx_afe_anatxmdata0[0]=(((r).tx_afe_anatxmdata0[0] & ~((uint32_t)0x7 << 10)) | ((((uint32_t)f) & 0x7) << 10)) | (7 << (16 + 10))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_TXMDIOTSTDATALf_GET(r) (((r).tx_afe_anatxmdata0[0]) & 0x3ff)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_TXMDIOTSTDATALf_SET(r,f) (r).tx_afe_anatxmdata0[0]=(((r).tx_afe_anatxmdata0[0] & ~((uint32_t)0x3ff)) | (((uint32_t)f) & 0x3ff)) | (0x3ff << 16)

/*
 * These macros can be used to access TX_AFE_ANATXMDATA0.
 *
 */
#define BCMI_VIPER_XGXS_READ_TX_AFE_ANATXMDATA0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r,(_r._tx_afe_anatxmdata0))
#define BCMI_VIPER_XGXS_WRITE_TX_AFE_ANATXMDATA0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r,(_r._tx_afe_anatxmdata0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TX_AFE_ANATXMDATA0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r,(_r._tx_afe_anatxmdata0))
#define BCMI_VIPER_XGXS_READLN_TX_AFE_ANATXMDATA0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_anatxmdata0))
#define BCMI_VIPER_XGXS_WRITELN_TX_AFE_ANATXMDATA0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_anatxmdata0))
#define BCMI_VIPER_XGXS_WRITEALL_TX_AFE_ANATXMDATA0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._tx_afe_anatxmdata0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TX_AFE_ANATXMDATA0r BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r
#define TX_AFE_ANATXMDATA0r_SIZE BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_SIZE
typedef BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_t TX_AFE_ANATXMDATA0r_t;
#define TX_AFE_ANATXMDATA0r_CLR BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_CLR
#define TX_AFE_ANATXMDATA0r_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_SET
#define TX_AFE_ANATXMDATA0r_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_GET
#define TX_AFE_ANATXMDATA0r_TXTESTMUXSELf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_TXTESTMUXSELf_GET
#define TX_AFE_ANATXMDATA0r_TXTESTMUXSELf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_TXTESTMUXSELf_SET
#define TX_AFE_ANATXMDATA0r_RLFIFO_TSTSELf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_RLFIFO_TSTSELf_GET
#define TX_AFE_ANATXMDATA0r_RLFIFO_TSTSELf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_RLFIFO_TSTSELf_SET
#define TX_AFE_ANATXMDATA0r_TXMDIOTSTDATALf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_TXMDIOTSTDATALf_GET
#define TX_AFE_ANATXMDATA0r_TXMDIOTSTDATALf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r_TXMDIOTSTDATALf_SET
#define READ_TX_AFE_ANATXMDATA0r BCMI_VIPER_XGXS_READ_TX_AFE_ANATXMDATA0r
#define WRITE_TX_AFE_ANATXMDATA0r BCMI_VIPER_XGXS_WRITE_TX_AFE_ANATXMDATA0r
#define MODIFY_TX_AFE_ANATXMDATA0r BCMI_VIPER_XGXS_MODIFY_TX_AFE_ANATXMDATA0r
#define READLN_TX_AFE_ANATXMDATA0r BCMI_VIPER_XGXS_READLN_TX_AFE_ANATXMDATA0r
#define WRITELN_TX_AFE_ANATXMDATA0r BCMI_VIPER_XGXS_WRITELN_TX_AFE_ANATXMDATA0r
#define WRITEALL_TX_AFE_ANATXMDATA0r BCMI_VIPER_XGXS_WRITEALL_TX_AFE_ANATXMDATA0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TX_AFE_ANATXMDATA1
 * BLOCKS:   TX_AFE
 * REGADDR:  0x8063
 * DESC:     Tx test mux data 1 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TXMDIOTSTDATAH   upper 10 bits of pattern
 *     PRE_EMPH_STAIR_EN Tx Electric Idle
 *     PRE_EMPH_STAIR_REV_EN Reverses order for staircase counter
 *     GLPBK_CLK_EN     Enable G-Loopback clock
 *     TX_ELECIDLE      
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r (0x00008063 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_SIZE 4

/*
 * This structure should be used to declare and program TX_AFE_ANATXMDATA1.
 *
 */
typedef union BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_s {
	uint32_t v[1];
	uint32_t tx_afe_anatxmdata1[1];
	uint32_t _tx_afe_anatxmdata1;
} BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_t;

#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_CLR(r) (r).tx_afe_anatxmdata1[0] = 0
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_SET(r,d) (r).tx_afe_anatxmdata1[0] = d
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_GET(r) (r).tx_afe_anatxmdata1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_TX_ELECIDLEf_GET(r) ((((r).tx_afe_anatxmdata1[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_TX_ELECIDLEf_SET(r,f) (r).tx_afe_anatxmdata1[0]=(((r).tx_afe_anatxmdata1[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_GLPBK_CLK_ENf_GET(r) ((((r).tx_afe_anatxmdata1[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_GLPBK_CLK_ENf_SET(r,f) (r).tx_afe_anatxmdata1[0]=(((r).tx_afe_anatxmdata1[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_REV_ENf_GET(r) ((((r).tx_afe_anatxmdata1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_REV_ENf_SET(r,f) (r).tx_afe_anatxmdata1[0]=(((r).tx_afe_anatxmdata1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_ENf_GET(r) ((((r).tx_afe_anatxmdata1[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_ENf_SET(r,f) (r).tx_afe_anatxmdata1[0]=(((r).tx_afe_anatxmdata1[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_TXMDIOTSTDATAHf_GET(r) (((r).tx_afe_anatxmdata1[0]) & 0x3ff)
#define BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_TXMDIOTSTDATAHf_SET(r,f) (r).tx_afe_anatxmdata1[0]=(((r).tx_afe_anatxmdata1[0] & ~((uint32_t)0x3ff)) | (((uint32_t)f) & 0x3ff)) | (0x3ff << 16)

/*
 * These macros can be used to access TX_AFE_ANATXMDATA1.
 *
 */
#define BCMI_VIPER_XGXS_READ_TX_AFE_ANATXMDATA1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r,(_r._tx_afe_anatxmdata1))
#define BCMI_VIPER_XGXS_WRITE_TX_AFE_ANATXMDATA1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r,(_r._tx_afe_anatxmdata1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TX_AFE_ANATXMDATA1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r,(_r._tx_afe_anatxmdata1))
#define BCMI_VIPER_XGXS_READLN_TX_AFE_ANATXMDATA1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_anatxmdata1))
#define BCMI_VIPER_XGXS_WRITELN_TX_AFE_ANATXMDATA1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_anatxmdata1))
#define BCMI_VIPER_XGXS_WRITEALL_TX_AFE_ANATXMDATA1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._tx_afe_anatxmdata1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TX_AFE_ANATXMDATA1r BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r
#define TX_AFE_ANATXMDATA1r_SIZE BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_SIZE
typedef BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_t TX_AFE_ANATXMDATA1r_t;
#define TX_AFE_ANATXMDATA1r_CLR BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_CLR
#define TX_AFE_ANATXMDATA1r_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_SET
#define TX_AFE_ANATXMDATA1r_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_GET
#define TX_AFE_ANATXMDATA1r_TX_ELECIDLEf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_TX_ELECIDLEf_GET
#define TX_AFE_ANATXMDATA1r_TX_ELECIDLEf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_TX_ELECIDLEf_SET
#define TX_AFE_ANATXMDATA1r_GLPBK_CLK_ENf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_GLPBK_CLK_ENf_GET
#define TX_AFE_ANATXMDATA1r_GLPBK_CLK_ENf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_GLPBK_CLK_ENf_SET
#define TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_REV_ENf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_REV_ENf_GET
#define TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_REV_ENf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_REV_ENf_SET
#define TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_ENf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_ENf_GET
#define TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_ENf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_PRE_EMPH_STAIR_ENf_SET
#define TX_AFE_ANATXMDATA1r_TXMDIOTSTDATAHf_GET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_TXMDIOTSTDATAHf_GET
#define TX_AFE_ANATXMDATA1r_TXMDIOTSTDATAHf_SET BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r_TXMDIOTSTDATAHf_SET
#define READ_TX_AFE_ANATXMDATA1r BCMI_VIPER_XGXS_READ_TX_AFE_ANATXMDATA1r
#define WRITE_TX_AFE_ANATXMDATA1r BCMI_VIPER_XGXS_WRITE_TX_AFE_ANATXMDATA1r
#define MODIFY_TX_AFE_ANATXMDATA1r BCMI_VIPER_XGXS_MODIFY_TX_AFE_ANATXMDATA1r
#define READLN_TX_AFE_ANATXMDATA1r BCMI_VIPER_XGXS_READLN_TX_AFE_ANATXMDATA1r
#define WRITELN_TX_AFE_ANATXMDATA1r BCMI_VIPER_XGXS_WRITELN_TX_AFE_ANATXMDATA1r
#define WRITEALL_TX_AFE_ANATXMDATA1r BCMI_VIPER_XGXS_WRITEALL_TX_AFE_ANATXMDATA1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TX_AFE_ANATXMDATA1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TX_AFE_CTL0
 * BLOCKS:   TX_AFE
 * REGADDR:  0x8065
 * DESC:     Analog TX Control 0 Register
 * RESETVAL: 0x3220 (12832)
 * ACCESS:   R/W
 * FIELDS:
 *     TX_PWRDN         0       (OR with direct pin i_tx_pd)0: Normal Operation1: TX Power Down (This should not affect AC-JTAG power-down)i_test ( AC-JTAG enable ) over writes this bit
 *     TESTSEL          Select  0T,4T0 : divide by 101:  divide by 4
 *     RESERVED_3_2     
 *     IDLE_ENA         0       (OR with direct pin i_TxElecIdle)0: Normal Operation1: TX Idle => Pull outp/n to VDD/2 through 5k resistors when TX is not power down
 *     RXDETECT_TH      RX Termination Detection Control Settings, adjusting trip point of internal comp.00: 0.756xVDD01: 0.813xVDD10: 0.813xVDD11: 0.876xVDD
 *     RESERVED_7       
 *     MAIN_CONTROL     1.      1V mode:I_tx_ctrl<17> = 0;  I_tx_ctrl<15> = 0;mV (diff)       I_tx_ctrl<13:8>          mV(diff)       I_tx_ctrl<13:8>650	001100		981	100110667	001101		991	100111683	001110		1000	101000699	001111		1009	101001714	010000		1018	101010729	010001		1027	101011744	010010		1036	101100759	010011		1044	101101773	010100		1053	101110787	010101		1061	101111800	010110		1069	110000813	010111		1077	110001826	011000		1085	110010839	011001		1092	110011851	011010		1100	110100863	011011		1107	110101875	011100		1115	110110887	011101		1122	110111898	011110		1129	111000909	011111		1136	111001920	100000		1143	111010931	100001		1150	111011941	100010		1156	111100951	100011		1163	111101962	100100		1169	111110971	100101		1176	1111112.	0.5V mode:I_tx_ctrl<17> = 1;  I_tx_ctrl<15> = 0;mV (diff)	I_tx_ctrl<13:8>		 mV(diff)	I_tx_ctrl<13:8>220	000011		579	011010239	000100		591	011011258	000101		603	011100277	000110		615	011101295	000111		627	011110313	001000		639	011111330	001001		650	100000347	001010		661	100001364	001011		672	100010380	001100		683	100011396	001101		694	100100412	001110		704	100101427	001111		714	100110442	010000		724	100111457	010001		734	101000472	010010		744	101001486	010011		754	101010500	010100		763	101011514	010101		773	101100527	010110		782	101101541	010111		791	101110554	011000		800	101111566	011001
 *     QUARTER_UNIT_EN  0: quarter unit disabled;1: quarter unit enabled: finer pre-emphasis steps (in constant impedance mode)
 *     FIX_10UNITS_EN   0: the fixed extra units are disabled1: the fixed extra units are enabled (further amplitude increase: >1.2Vpp)
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r (0x00008065 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_SIZE 4

/*
 * This structure should be used to declare and program TX_AFE_CTL0.
 *
 */
typedef union BCMI_VIPER_XGXS_TX_AFE_CTL0r_s {
	uint32_t v[1];
	uint32_t tx_afe_ctl0[1];
	uint32_t _tx_afe_ctl0;
} BCMI_VIPER_XGXS_TX_AFE_CTL0r_t;

#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_CLR(r) (r).tx_afe_ctl0[0] = 0
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_SET(r,d) (r).tx_afe_ctl0[0] = d
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_GET(r) (r).tx_afe_ctl0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_FIX_10UNITS_ENf_GET(r) ((((r).tx_afe_ctl0[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_FIX_10UNITS_ENf_SET(r,f) (r).tx_afe_ctl0[0]=(((r).tx_afe_ctl0[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_QUARTER_UNIT_ENf_GET(r) ((((r).tx_afe_ctl0[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_QUARTER_UNIT_ENf_SET(r,f) (r).tx_afe_ctl0[0]=(((r).tx_afe_ctl0[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_MAIN_CONTROLf_GET(r) ((((r).tx_afe_ctl0[0]) >> 8) & 0x3f)
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_MAIN_CONTROLf_SET(r,f) (r).tx_afe_ctl0[0]=(((r).tx_afe_ctl0[0] & ~((uint32_t)0x3f << 8)) | ((((uint32_t)f) & 0x3f) << 8)) | (63 << (16 + 8))
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_RESERVED_7f_GET(r) ((((r).tx_afe_ctl0[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_RESERVED_7f_SET(r,f) (r).tx_afe_ctl0[0]=(((r).tx_afe_ctl0[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_RXDETECT_THf_GET(r) ((((r).tx_afe_ctl0[0]) >> 5) & 0x3)
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_RXDETECT_THf_SET(r,f) (r).tx_afe_ctl0[0]=(((r).tx_afe_ctl0[0] & ~((uint32_t)0x3 << 5)) | ((((uint32_t)f) & 0x3) << 5)) | (3 << (16 + 5))
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_IDLE_ENAf_GET(r) ((((r).tx_afe_ctl0[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_IDLE_ENAf_SET(r,f) (r).tx_afe_ctl0[0]=(((r).tx_afe_ctl0[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_RESERVED_3_2f_GET(r) ((((r).tx_afe_ctl0[0]) >> 2) & 0x3)
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_RESERVED_3_2f_SET(r,f) (r).tx_afe_ctl0[0]=(((r).tx_afe_ctl0[0] & ~((uint32_t)0x3 << 2)) | ((((uint32_t)f) & 0x3) << 2)) | (3 << (16 + 2))
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_TESTSELf_GET(r) ((((r).tx_afe_ctl0[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_TESTSELf_SET(r,f) (r).tx_afe_ctl0[0]=(((r).tx_afe_ctl0[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_TX_PWRDNf_GET(r) (((r).tx_afe_ctl0[0]) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL0r_TX_PWRDNf_SET(r,f) (r).tx_afe_ctl0[0]=(((r).tx_afe_ctl0[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access TX_AFE_CTL0.
 *
 */
#define BCMI_VIPER_XGXS_READ_TX_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL0r,(_r._tx_afe_ctl0))
#define BCMI_VIPER_XGXS_WRITE_TX_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL0r,(_r._tx_afe_ctl0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TX_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL0r,(_r._tx_afe_ctl0))
#define BCMI_VIPER_XGXS_READLN_TX_AFE_CTL0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_ctl0))
#define BCMI_VIPER_XGXS_WRITELN_TX_AFE_CTL0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_ctl0))
#define BCMI_VIPER_XGXS_WRITEALL_TX_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._tx_afe_ctl0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TX_AFE_CTL0r BCMI_VIPER_XGXS_TX_AFE_CTL0r
#define TX_AFE_CTL0r_SIZE BCMI_VIPER_XGXS_TX_AFE_CTL0r_SIZE
typedef BCMI_VIPER_XGXS_TX_AFE_CTL0r_t TX_AFE_CTL0r_t;
#define TX_AFE_CTL0r_CLR BCMI_VIPER_XGXS_TX_AFE_CTL0r_CLR
#define TX_AFE_CTL0r_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_SET
#define TX_AFE_CTL0r_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_GET
#define TX_AFE_CTL0r_FIX_10UNITS_ENf_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_FIX_10UNITS_ENf_GET
#define TX_AFE_CTL0r_FIX_10UNITS_ENf_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_FIX_10UNITS_ENf_SET
#define TX_AFE_CTL0r_QUARTER_UNIT_ENf_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_QUARTER_UNIT_ENf_GET
#define TX_AFE_CTL0r_QUARTER_UNIT_ENf_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_QUARTER_UNIT_ENf_SET
#define TX_AFE_CTL0r_MAIN_CONTROLf_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_MAIN_CONTROLf_GET
#define TX_AFE_CTL0r_MAIN_CONTROLf_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_MAIN_CONTROLf_SET
#define TX_AFE_CTL0r_RESERVED_7f_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_RESERVED_7f_GET
#define TX_AFE_CTL0r_RESERVED_7f_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_RESERVED_7f_SET
#define TX_AFE_CTL0r_RXDETECT_THf_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_RXDETECT_THf_GET
#define TX_AFE_CTL0r_RXDETECT_THf_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_RXDETECT_THf_SET
#define TX_AFE_CTL0r_IDLE_ENAf_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_IDLE_ENAf_GET
#define TX_AFE_CTL0r_IDLE_ENAf_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_IDLE_ENAf_SET
#define TX_AFE_CTL0r_RESERVED_3_2f_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_RESERVED_3_2f_GET
#define TX_AFE_CTL0r_RESERVED_3_2f_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_RESERVED_3_2f_SET
#define TX_AFE_CTL0r_TESTSELf_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_TESTSELf_GET
#define TX_AFE_CTL0r_TESTSELf_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_TESTSELf_SET
#define TX_AFE_CTL0r_TX_PWRDNf_GET BCMI_VIPER_XGXS_TX_AFE_CTL0r_TX_PWRDNf_GET
#define TX_AFE_CTL0r_TX_PWRDNf_SET BCMI_VIPER_XGXS_TX_AFE_CTL0r_TX_PWRDNf_SET
#define READ_TX_AFE_CTL0r BCMI_VIPER_XGXS_READ_TX_AFE_CTL0r
#define WRITE_TX_AFE_CTL0r BCMI_VIPER_XGXS_WRITE_TX_AFE_CTL0r
#define MODIFY_TX_AFE_CTL0r BCMI_VIPER_XGXS_MODIFY_TX_AFE_CTL0r
#define READLN_TX_AFE_CTL0r BCMI_VIPER_XGXS_READLN_TX_AFE_CTL0r
#define WRITELN_TX_AFE_CTL0r BCMI_VIPER_XGXS_WRITELN_TX_AFE_CTL0r
#define WRITEALL_TX_AFE_CTL0r BCMI_VIPER_XGXS_WRITEALL_TX_AFE_CTL0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TX_AFE_CTL0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TX_AFE_CTL1
 * BLOCKS:   TX_AFE
 * REGADDR:  0x8066
 * DESC:     Analog TX Control 1 Register
 * RESETVAL: 0x8009 (32777)
 * ACCESS:   R/W
 * FIELDS:
 *     VDD1P0_ENB       1 for lower range (0.81 to 0.95) and 0 for upper range ( 0.9 to 1.05)
 *     AMP_MODE         0: 1V mode;  1: 0.5V modeThis bit also changes the resistor calibration accordingly
 *     CONST_IMPEDANCE  0: Impedance varying mode; i.e. the output impedance increases when pre-emphasis increases1: constant impedance mode; i.e. the output impedance is constant at various pre-emphasis settingWhen 1 ensure i_tx_ctrl<28> = 1
 *     PI_BW_SEL        (updated from Tamer)
 *     PWD_LVL2PI       (updated from Tamer)
 *     RESERVED_21      
 *     POST_CONTROL     For all modes: i_tx_ctrl<28> = 1 enables the post path1V mode: I_tx_ctrl<17> = 0;a)	Impedance-varying mode: (the output impedance increases when pre-emphasis increases; assuming the main tap is set at default)I_tx_ctrl<18> = 0;  I_tx_ctrl<13:8> = 101001dB	I_tx_ctrl<27:22>		 dB	I_tx_ctrl<27:22>0	000000		-2.57	010110-0.08	000001		-2.74	010111-0.17	000010		-2.92	011000-0.25	000011		-3.11	011001-0.34	000100		-3.31	011010-0.43	000101		-3.52	011011-0.53	000110		-3.74	011100-0.62	000111		-3.97	011101-0.72	001000		-4.22	011110-0.83	001001		-4.47	011111-0.94	001010		-4.75	100000-1.05	001011		-5.04	100001-1.16	001100		-5.34	100010-1.28	001101		-5.67	100011-1.4	001110		-6.02	100100-1.53	001111		-6.4	100101-1.66	010000		-6.8	100110-1.8	010001		-7.23	100111-1.94	010010		-7.71	101000-2.1	010011		-8.22	101001-2.24	010100		-8.79	101010-2.4	010101b)	Constant Impedance mode: (the output impedance is constant at various pre-emphasis setting)I_tx_ctrl<18> = 1;Program I_tx_ctrl<13:8> and I_tx_ctrl<27:22> as in the following table:dB	I_tx_ctrl<13:8>	I_tx_ctrl<27:22>	I_tx_ctrl<14>0	101000	000000	0-0.17	100110	000010	1-0.32	100101	000011	0-0.5	100101	000011	1-0.66	100100	000100	0-0.84	100100	000100	1-1	100011	000101	0-1.19	100011	000101	1-1.37	100010	000110	0-1.56	100010	000110	1-1.76	100001	000111	0-1.95	100001	000111	1-2.16	100000	001000	0-2.36	100000	001000	1-2.58	011111	001001	0-2.78	011111	001001	1-3.02	011110	001010	0-3.23	011110	001010	1-3.49	011101	001011	0-3.71	011101	001011	1-3.98	011100	001100	0-4.21	011100	001100	1-4.5	011011	001101	0-4.74	011011	001101	1-5.1	011010	001110	0-5.31	011010	001110	1-5.66	011001	001111	0-5.92	011001	001111	1-6.3	011000	010000	0-6.57	011000	010000	1-6.99	010111	010001	0-7.28	010111	010001	1-7.75	010110	010010	0-8.05	010110	010010	1-8.57	010101	010011	0-8.9	010101	010011	1
 *     POST_ENABLE      0: post path is disable, 1: post path enable and pre-emphasis can be used
 *     RESERVED_29      
 *     SLEW_RATE_CONTROL 00: load cap 0pF(SATA3); 01: load cap 1pF(SATA2); 10: load cap 2pF(SATA1); 11: load cap 3pF
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r (0x00008066 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_SIZE 4

/*
 * This structure should be used to declare and program TX_AFE_CTL1.
 *
 */
typedef union BCMI_VIPER_XGXS_TX_AFE_CTL1r_s {
	uint32_t v[1];
	uint32_t tx_afe_ctl1[1];
	uint32_t _tx_afe_ctl1;
} BCMI_VIPER_XGXS_TX_AFE_CTL1r_t;

#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_CLR(r) (r).tx_afe_ctl1[0] = 0
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_SET(r,d) (r).tx_afe_ctl1[0] = d
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_GET(r) (r).tx_afe_ctl1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_SLEW_RATE_CONTROLf_GET(r) ((((r).tx_afe_ctl1[0]) >> 14) & 0x3)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_SLEW_RATE_CONTROLf_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x3 << 14)) | ((((uint32_t)f) & 0x3) << 14)) | (3 << (16 + 14))
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_RESERVED_29f_GET(r) ((((r).tx_afe_ctl1[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_RESERVED_29f_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_POST_ENABLEf_GET(r) ((((r).tx_afe_ctl1[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_POST_ENABLEf_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_POST_CONTROLf_GET(r) ((((r).tx_afe_ctl1[0]) >> 6) & 0x3f)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_POST_CONTROLf_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x3f << 6)) | ((((uint32_t)f) & 0x3f) << 6)) | (63 << (16 + 6))
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_RESERVED_21f_GET(r) ((((r).tx_afe_ctl1[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_RESERVED_21f_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_PWD_LVL2PIf_GET(r) ((((r).tx_afe_ctl1[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_PWD_LVL2PIf_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_PI_BW_SELf_GET(r) ((((r).tx_afe_ctl1[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_PI_BW_SELf_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_CONST_IMPEDANCEf_GET(r) ((((r).tx_afe_ctl1[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_CONST_IMPEDANCEf_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_AMP_MODEf_GET(r) ((((r).tx_afe_ctl1[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_AMP_MODEf_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_VDD1P0_ENBf_GET(r) (((r).tx_afe_ctl1[0]) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL1r_VDD1P0_ENBf_SET(r,f) (r).tx_afe_ctl1[0]=(((r).tx_afe_ctl1[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access TX_AFE_CTL1.
 *
 */
#define BCMI_VIPER_XGXS_READ_TX_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL1r,(_r._tx_afe_ctl1))
#define BCMI_VIPER_XGXS_WRITE_TX_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL1r,(_r._tx_afe_ctl1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TX_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL1r,(_r._tx_afe_ctl1))
#define BCMI_VIPER_XGXS_READLN_TX_AFE_CTL1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_ctl1))
#define BCMI_VIPER_XGXS_WRITELN_TX_AFE_CTL1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_ctl1))
#define BCMI_VIPER_XGXS_WRITEALL_TX_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._tx_afe_ctl1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TX_AFE_CTL1r BCMI_VIPER_XGXS_TX_AFE_CTL1r
#define TX_AFE_CTL1r_SIZE BCMI_VIPER_XGXS_TX_AFE_CTL1r_SIZE
typedef BCMI_VIPER_XGXS_TX_AFE_CTL1r_t TX_AFE_CTL1r_t;
#define TX_AFE_CTL1r_CLR BCMI_VIPER_XGXS_TX_AFE_CTL1r_CLR
#define TX_AFE_CTL1r_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_SET
#define TX_AFE_CTL1r_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_GET
#define TX_AFE_CTL1r_SLEW_RATE_CONTROLf_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_SLEW_RATE_CONTROLf_GET
#define TX_AFE_CTL1r_SLEW_RATE_CONTROLf_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_SLEW_RATE_CONTROLf_SET
#define TX_AFE_CTL1r_RESERVED_29f_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_RESERVED_29f_GET
#define TX_AFE_CTL1r_RESERVED_29f_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_RESERVED_29f_SET
#define TX_AFE_CTL1r_POST_ENABLEf_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_POST_ENABLEf_GET
#define TX_AFE_CTL1r_POST_ENABLEf_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_POST_ENABLEf_SET
#define TX_AFE_CTL1r_POST_CONTROLf_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_POST_CONTROLf_GET
#define TX_AFE_CTL1r_POST_CONTROLf_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_POST_CONTROLf_SET
#define TX_AFE_CTL1r_RESERVED_21f_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_RESERVED_21f_GET
#define TX_AFE_CTL1r_RESERVED_21f_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_RESERVED_21f_SET
#define TX_AFE_CTL1r_PWD_LVL2PIf_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_PWD_LVL2PIf_GET
#define TX_AFE_CTL1r_PWD_LVL2PIf_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_PWD_LVL2PIf_SET
#define TX_AFE_CTL1r_PI_BW_SELf_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_PI_BW_SELf_GET
#define TX_AFE_CTL1r_PI_BW_SELf_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_PI_BW_SELf_SET
#define TX_AFE_CTL1r_CONST_IMPEDANCEf_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_CONST_IMPEDANCEf_GET
#define TX_AFE_CTL1r_CONST_IMPEDANCEf_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_CONST_IMPEDANCEf_SET
#define TX_AFE_CTL1r_AMP_MODEf_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_AMP_MODEf_GET
#define TX_AFE_CTL1r_AMP_MODEf_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_AMP_MODEf_SET
#define TX_AFE_CTL1r_VDD1P0_ENBf_GET BCMI_VIPER_XGXS_TX_AFE_CTL1r_VDD1P0_ENBf_GET
#define TX_AFE_CTL1r_VDD1P0_ENBf_SET BCMI_VIPER_XGXS_TX_AFE_CTL1r_VDD1P0_ENBf_SET
#define READ_TX_AFE_CTL1r BCMI_VIPER_XGXS_READ_TX_AFE_CTL1r
#define WRITE_TX_AFE_CTL1r BCMI_VIPER_XGXS_WRITE_TX_AFE_CTL1r
#define MODIFY_TX_AFE_CTL1r BCMI_VIPER_XGXS_MODIFY_TX_AFE_CTL1r
#define READLN_TX_AFE_CTL1r BCMI_VIPER_XGXS_READLN_TX_AFE_CTL1r
#define WRITELN_TX_AFE_CTL1r BCMI_VIPER_XGXS_WRITELN_TX_AFE_CTL1r
#define WRITEALL_TX_AFE_CTL1r BCMI_VIPER_XGXS_WRITEALL_TX_AFE_CTL1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TX_AFE_CTL1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TX_AFE_CTL2
 * BLOCKS:   TX_AFE
 * REGADDR:  0x8067
 * DESC:     Analog TX Control 2 Register
 * RESETVAL: 0x38 (56)
 * ACCESS:   R/W
 * FIELDS:
 *     TESTCK_EN        This bit enables testport mux, for testing 10T and 4T clocks
 *     TICKSEL          ticksel[1:0] control load timing of the data respect to 10T clock00 allows parallel data to be delayed by 8T01 allows parallel data to be delayed by 10T10 allows parallel data to be delayed by 12T11 allows parallel data to be delayed by 14T
 *     TX_PON           Resistor Calibration Control Setting (Update table)0000: min resistance process corner, (increases on-chip R by +25%)1111: max resistance process corner, (decreases on-chip R by -25%)Error between two consecutive codes is maintained less than 3%.By default, pon<3:0> match with resistor calibration output; they can also be independently adjusted through RTL.
 *     VDD_NOISE_SHAPE  Controls shape of injected current of the noise cancelation circuit to vdd
 *     NOISE_CNCL_BIAS  Controls bias current of the noise cancelation circuit
 *     VDD_NOISE_CNCL_EN Enables noise cancelation circuit
 *     LEAKAGE_TEST     Test pin for leakage power internal analog test only
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r (0x00008067 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_SIZE 4

/*
 * This structure should be used to declare and program TX_AFE_CTL2.
 *
 */
typedef union BCMI_VIPER_XGXS_TX_AFE_CTL2r_s {
	uint32_t v[1];
	uint32_t tx_afe_ctl2[1];
	uint32_t _tx_afe_ctl2;
} BCMI_VIPER_XGXS_TX_AFE_CTL2r_t;

#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_CLR(r) (r).tx_afe_ctl2[0] = 0
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_SET(r,d) (r).tx_afe_ctl2[0] = d
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_GET(r) (r).tx_afe_ctl2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_LEAKAGE_TESTf_GET(r) ((((r).tx_afe_ctl2[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_LEAKAGE_TESTf_SET(r,f) (r).tx_afe_ctl2[0]=(((r).tx_afe_ctl2[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_VDD_NOISE_CNCL_ENf_GET(r) ((((r).tx_afe_ctl2[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_VDD_NOISE_CNCL_ENf_SET(r,f) (r).tx_afe_ctl2[0]=(((r).tx_afe_ctl2[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_NOISE_CNCL_BIASf_GET(r) ((((r).tx_afe_ctl2[0]) >> 10) & 0xf)
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_NOISE_CNCL_BIASf_SET(r,f) (r).tx_afe_ctl2[0]=(((r).tx_afe_ctl2[0] & ~((uint32_t)0xf << 10)) | ((((uint32_t)f) & 0xf) << 10)) | (15 << (16 + 10))
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_VDD_NOISE_SHAPEf_GET(r) ((((r).tx_afe_ctl2[0]) >> 7) & 0x7)
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_VDD_NOISE_SHAPEf_SET(r,f) (r).tx_afe_ctl2[0]=(((r).tx_afe_ctl2[0] & ~((uint32_t)0x7 << 7)) | ((((uint32_t)f) & 0x7) << 7)) | (7 << (16 + 7))
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_TX_PONf_GET(r) ((((r).tx_afe_ctl2[0]) >> 3) & 0xf)
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_TX_PONf_SET(r,f) (r).tx_afe_ctl2[0]=(((r).tx_afe_ctl2[0] & ~((uint32_t)0xf << 3)) | ((((uint32_t)f) & 0xf) << 3)) | (15 << (16 + 3))
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_TICKSELf_GET(r) ((((r).tx_afe_ctl2[0]) >> 1) & 0x3)
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_TICKSELf_SET(r,f) (r).tx_afe_ctl2[0]=(((r).tx_afe_ctl2[0] & ~((uint32_t)0x3 << 1)) | ((((uint32_t)f) & 0x3) << 1)) | (3 << (16 + 1))
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_TESTCK_ENf_GET(r) (((r).tx_afe_ctl2[0]) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_CTL2r_TESTCK_ENf_SET(r,f) (r).tx_afe_ctl2[0]=(((r).tx_afe_ctl2[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access TX_AFE_CTL2.
 *
 */
#define BCMI_VIPER_XGXS_READ_TX_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL2r,(_r._tx_afe_ctl2))
#define BCMI_VIPER_XGXS_WRITE_TX_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL2r,(_r._tx_afe_ctl2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TX_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL2r,(_r._tx_afe_ctl2))
#define BCMI_VIPER_XGXS_READLN_TX_AFE_CTL2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_ctl2))
#define BCMI_VIPER_XGXS_WRITELN_TX_AFE_CTL2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_ctl2))
#define BCMI_VIPER_XGXS_WRITEALL_TX_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._tx_afe_ctl2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TX_AFE_CTL2r BCMI_VIPER_XGXS_TX_AFE_CTL2r
#define TX_AFE_CTL2r_SIZE BCMI_VIPER_XGXS_TX_AFE_CTL2r_SIZE
typedef BCMI_VIPER_XGXS_TX_AFE_CTL2r_t TX_AFE_CTL2r_t;
#define TX_AFE_CTL2r_CLR BCMI_VIPER_XGXS_TX_AFE_CTL2r_CLR
#define TX_AFE_CTL2r_SET BCMI_VIPER_XGXS_TX_AFE_CTL2r_SET
#define TX_AFE_CTL2r_GET BCMI_VIPER_XGXS_TX_AFE_CTL2r_GET
#define TX_AFE_CTL2r_LEAKAGE_TESTf_GET BCMI_VIPER_XGXS_TX_AFE_CTL2r_LEAKAGE_TESTf_GET
#define TX_AFE_CTL2r_LEAKAGE_TESTf_SET BCMI_VIPER_XGXS_TX_AFE_CTL2r_LEAKAGE_TESTf_SET
#define TX_AFE_CTL2r_VDD_NOISE_CNCL_ENf_GET BCMI_VIPER_XGXS_TX_AFE_CTL2r_VDD_NOISE_CNCL_ENf_GET
#define TX_AFE_CTL2r_VDD_NOISE_CNCL_ENf_SET BCMI_VIPER_XGXS_TX_AFE_CTL2r_VDD_NOISE_CNCL_ENf_SET
#define TX_AFE_CTL2r_NOISE_CNCL_BIASf_GET BCMI_VIPER_XGXS_TX_AFE_CTL2r_NOISE_CNCL_BIASf_GET
#define TX_AFE_CTL2r_NOISE_CNCL_BIASf_SET BCMI_VIPER_XGXS_TX_AFE_CTL2r_NOISE_CNCL_BIASf_SET
#define TX_AFE_CTL2r_VDD_NOISE_SHAPEf_GET BCMI_VIPER_XGXS_TX_AFE_CTL2r_VDD_NOISE_SHAPEf_GET
#define TX_AFE_CTL2r_VDD_NOISE_SHAPEf_SET BCMI_VIPER_XGXS_TX_AFE_CTL2r_VDD_NOISE_SHAPEf_SET
#define TX_AFE_CTL2r_TX_PONf_GET BCMI_VIPER_XGXS_TX_AFE_CTL2r_TX_PONf_GET
#define TX_AFE_CTL2r_TX_PONf_SET BCMI_VIPER_XGXS_TX_AFE_CTL2r_TX_PONf_SET
#define TX_AFE_CTL2r_TICKSELf_GET BCMI_VIPER_XGXS_TX_AFE_CTL2r_TICKSELf_GET
#define TX_AFE_CTL2r_TICKSELf_SET BCMI_VIPER_XGXS_TX_AFE_CTL2r_TICKSELf_SET
#define TX_AFE_CTL2r_TESTCK_ENf_GET BCMI_VIPER_XGXS_TX_AFE_CTL2r_TESTCK_ENf_GET
#define TX_AFE_CTL2r_TESTCK_ENf_SET BCMI_VIPER_XGXS_TX_AFE_CTL2r_TESTCK_ENf_SET
#define READ_TX_AFE_CTL2r BCMI_VIPER_XGXS_READ_TX_AFE_CTL2r
#define WRITE_TX_AFE_CTL2r BCMI_VIPER_XGXS_WRITE_TX_AFE_CTL2r
#define MODIFY_TX_AFE_CTL2r BCMI_VIPER_XGXS_MODIFY_TX_AFE_CTL2r
#define READLN_TX_AFE_CTL2r BCMI_VIPER_XGXS_READLN_TX_AFE_CTL2r
#define WRITELN_TX_AFE_CTL2r BCMI_VIPER_XGXS_WRITELN_TX_AFE_CTL2r
#define WRITEALL_TX_AFE_CTL2r BCMI_VIPER_XGXS_WRITEALL_TX_AFE_CTL2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TX_AFE_CTL2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TX_AFE_CTL3
 * BLOCKS:   TX_AFE
 * REGADDR:  0x8068
 * DESC:     Analog TX Control 3 Register
 * RESETVAL: 0x16 (22)
 * ACCESS:   R/W
 * FIELDS:
 *     RESERVED_63_48   
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TX_AFE_CTL3r (0x00008068 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TX_AFE_CTL3r_SIZE 4

/*
 * This structure should be used to declare and program TX_AFE_CTL3.
 *
 */
typedef union BCMI_VIPER_XGXS_TX_AFE_CTL3r_s {
	uint32_t v[1];
	uint32_t tx_afe_ctl3[1];
	uint32_t _tx_afe_ctl3;
} BCMI_VIPER_XGXS_TX_AFE_CTL3r_t;

#define BCMI_VIPER_XGXS_TX_AFE_CTL3r_CLR(r) (r).tx_afe_ctl3[0] = 0
#define BCMI_VIPER_XGXS_TX_AFE_CTL3r_SET(r,d) (r).tx_afe_ctl3[0] = d
#define BCMI_VIPER_XGXS_TX_AFE_CTL3r_GET(r) (r).tx_afe_ctl3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TX_AFE_CTL3r_RESERVED_63_48f_GET(r) (((r).tx_afe_ctl3[0]) & 0xffff)
#define BCMI_VIPER_XGXS_TX_AFE_CTL3r_RESERVED_63_48f_SET(r,f) (r).tx_afe_ctl3[0]=(((r).tx_afe_ctl3[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access TX_AFE_CTL3.
 *
 */
#define BCMI_VIPER_XGXS_READ_TX_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL3r,(_r._tx_afe_ctl3))
#define BCMI_VIPER_XGXS_WRITE_TX_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL3r,(_r._tx_afe_ctl3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TX_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL3r,(_r._tx_afe_ctl3))
#define BCMI_VIPER_XGXS_READLN_TX_AFE_CTL3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_ctl3))
#define BCMI_VIPER_XGXS_WRITELN_TX_AFE_CTL3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_ctl3))
#define BCMI_VIPER_XGXS_WRITEALL_TX_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._tx_afe_ctl3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TX_AFE_CTL3r BCMI_VIPER_XGXS_TX_AFE_CTL3r
#define TX_AFE_CTL3r_SIZE BCMI_VIPER_XGXS_TX_AFE_CTL3r_SIZE
typedef BCMI_VIPER_XGXS_TX_AFE_CTL3r_t TX_AFE_CTL3r_t;
#define TX_AFE_CTL3r_CLR BCMI_VIPER_XGXS_TX_AFE_CTL3r_CLR
#define TX_AFE_CTL3r_SET BCMI_VIPER_XGXS_TX_AFE_CTL3r_SET
#define TX_AFE_CTL3r_GET BCMI_VIPER_XGXS_TX_AFE_CTL3r_GET
#define TX_AFE_CTL3r_RESERVED_63_48f_GET BCMI_VIPER_XGXS_TX_AFE_CTL3r_RESERVED_63_48f_GET
#define TX_AFE_CTL3r_RESERVED_63_48f_SET BCMI_VIPER_XGXS_TX_AFE_CTL3r_RESERVED_63_48f_SET
#define READ_TX_AFE_CTL3r BCMI_VIPER_XGXS_READ_TX_AFE_CTL3r
#define WRITE_TX_AFE_CTL3r BCMI_VIPER_XGXS_WRITE_TX_AFE_CTL3r
#define MODIFY_TX_AFE_CTL3r BCMI_VIPER_XGXS_MODIFY_TX_AFE_CTL3r
#define READLN_TX_AFE_CTL3r BCMI_VIPER_XGXS_READLN_TX_AFE_CTL3r
#define WRITELN_TX_AFE_CTL3r BCMI_VIPER_XGXS_WRITELN_TX_AFE_CTL3r
#define WRITEALL_TX_AFE_CTL3r BCMI_VIPER_XGXS_WRITEALL_TX_AFE_CTL3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TX_AFE_CTL3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TX_AFE_INTERP
 * BLOCKS:   TX_AFE
 * REGADDR:  0x8069
 * DESC:     txinterp Control Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TX_RATESELECT   1: when OS4 or OS5 0: when OS2, to use halfrate preempasis
 *     INTERP_CTRL_PHS  
 *     INTERP_CTRL_QUADRANT 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TX_AFE_INTERPr (0x00008069 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_SIZE 4

/*
 * This structure should be used to declare and program TX_AFE_INTERP.
 *
 */
typedef union BCMI_VIPER_XGXS_TX_AFE_INTERPr_s {
	uint32_t v[1];
	uint32_t tx_afe_interp[1];
	uint32_t _tx_afe_interp;
} BCMI_VIPER_XGXS_TX_AFE_INTERPr_t;

#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_CLR(r) (r).tx_afe_interp[0] = 0
#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_SET(r,d) (r).tx_afe_interp[0] = d
#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_GET(r) (r).tx_afe_interp[0]

/*
 * These macros can be used to access individual fields.
 *
 */


#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_TX_RATESELECTf_GET(r) ((((r).tx_afe_interp[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_TX_RATESELECTf_SET(r,f) (r).tx_afe_interp[0]=(((r).tx_afe_interp[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_INTERP_CTRL_QUADRANTf_GET(r) ((((r).tx_afe_interp[0]) >> 5) & 0x3)
#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_INTERP_CTRL_QUADRANTf_SET(r,f) (r).tx_afe_interp[0]=(((r).tx_afe_interp[0] & ~((uint32_t)0x3 << 5)) | ((((uint32_t)f) & 0x3) << 5)) | (3 << (16 + 5))
#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_INTERP_CTRL_PHSf_GET(r) (((r).tx_afe_interp[0]) & 0x1f)
#define BCMI_VIPER_XGXS_TX_AFE_INTERPr_INTERP_CTRL_PHSf_SET(r,f) (r).tx_afe_interp[0]=(((r).tx_afe_interp[0] & ~((uint32_t)0x1f)) | (((uint32_t)f) & 0x1f)) | (0x1f << 16)

/*
 * These macros can be used to access TX_AFE_INTERP.
 *
 */
#define BCMI_VIPER_XGXS_READ_TX_AFE_INTERPr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_INTERPr,(_r._tx_afe_interp))
#define BCMI_VIPER_XGXS_WRITE_TX_AFE_INTERPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_INTERPr,(_r._tx_afe_interp)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TX_AFE_INTERPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_INTERPr,(_r._tx_afe_interp))
#define BCMI_VIPER_XGXS_READLN_TX_AFE_INTERPr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TX_AFE_INTERPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_interp))
#define BCMI_VIPER_XGXS_WRITELN_TX_AFE_INTERPr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_INTERPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._tx_afe_interp))
#define BCMI_VIPER_XGXS_WRITEALL_TX_AFE_INTERPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TX_AFE_INTERPr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._tx_afe_interp))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TX_AFE_INTERPr BCMI_VIPER_XGXS_TX_AFE_INTERPr
#define TX_AFE_INTERPr_SIZE BCMI_VIPER_XGXS_TX_AFE_INTERPr_SIZE
typedef BCMI_VIPER_XGXS_TX_AFE_INTERPr_t TX_AFE_INTERPr_t;
#define TX_AFE_INTERPr_CLR BCMI_VIPER_XGXS_TX_AFE_INTERPr_CLR
#define TX_AFE_INTERPr_SET BCMI_VIPER_XGXS_TX_AFE_INTERPr_SET
#define TX_AFE_INTERPr_GET BCMI_VIPER_XGXS_TX_AFE_INTERPr_GET
#define TX_AFE_INTERPr_TX_RATESELECTf_GET BCMI_VIPER_XGXS_TX_AFE_INTERPr_TX_RATESELECTf_GET
#define TX_AFE_INTERPr_TX_RATESELECTf_SET BCMI_VIPER_XGXS_TX_AFE_INTERPr_TX_RATESELECTf_SET
#define TX_AFE_INTERPr_INTERP_CTRL_QUADRANTf_GET BCMI_VIPER_XGXS_TX_AFE_INTERPr_INTERP_CTRL_QUADRANTf_GET
#define TX_AFE_INTERPr_INTERP_CTRL_QUADRANTf_SET BCMI_VIPER_XGXS_TX_AFE_INTERPr_INTERP_CTRL_QUADRANTf_SET
#define TX_AFE_INTERPr_INTERP_CTRL_PHSf_GET BCMI_VIPER_XGXS_TX_AFE_INTERPr_INTERP_CTRL_PHSf_GET
#define TX_AFE_INTERPr_INTERP_CTRL_PHSf_SET BCMI_VIPER_XGXS_TX_AFE_INTERPr_INTERP_CTRL_PHSf_SET
#define READ_TX_AFE_INTERPr BCMI_VIPER_XGXS_READ_TX_AFE_INTERPr
#define WRITE_TX_AFE_INTERPr BCMI_VIPER_XGXS_WRITE_TX_AFE_INTERPr
#define MODIFY_TX_AFE_INTERPr BCMI_VIPER_XGXS_MODIFY_TX_AFE_INTERPr
#define READLN_TX_AFE_INTERPr BCMI_VIPER_XGXS_READLN_TX_AFE_INTERPr
#define WRITELN_TX_AFE_INTERPr BCMI_VIPER_XGXS_WRITELN_TX_AFE_INTERPr
#define WRITEALL_TX_AFE_INTERPr BCMI_VIPER_XGXS_WRITEALL_TX_AFE_INTERPr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TX_AFE_INTERPr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_ANARXSTS
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b0
 * DESC:     Rx lane status register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     RX_AFE_ANARXSTS  RX_AFE_ANARXSTS
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr (0x000080b0 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_ANARXSTS.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_s {
	uint32_t v[1];
	uint32_t rx_afe_anarxsts[1];
	uint32_t _rx_afe_anarxsts;
} BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_t;

#define BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_CLR(r) (r).rx_afe_anarxsts[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_SET(r,d) (r).rx_afe_anarxsts[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_GET(r) (r).rx_afe_anarxsts[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_RX_AFE_ANARXSTSf_GET(r) (((r).rx_afe_anarxsts[0]) & 0xffff)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_RX_AFE_ANARXSTSf_SET(r,f) (r).rx_afe_anarxsts[0]=(((r).rx_afe_anarxsts[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access RX_AFE_ANARXSTS.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_ANARXSTSr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr,(_r._rx_afe_anarxsts))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXSTSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr,(_r._rx_afe_anarxsts)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXSTSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr,(_r._rx_afe_anarxsts))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXSTSr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxsts))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXSTSr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxsts))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXSTSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_anarxsts))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_ANARXSTSr BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr
#define RX_AFE_ANARXSTSr_SIZE BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_t RX_AFE_ANARXSTSr_t;
#define RX_AFE_ANARXSTSr_CLR BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_CLR
#define RX_AFE_ANARXSTSr_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_SET
#define RX_AFE_ANARXSTSr_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_GET
#define RX_AFE_ANARXSTSr_RX_AFE_ANARXSTSf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_RX_AFE_ANARXSTSf_GET
#define RX_AFE_ANARXSTSr_RX_AFE_ANARXSTSf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr_RX_AFE_ANARXSTSf_SET
#define READ_RX_AFE_ANARXSTSr BCMI_VIPER_XGXS_READ_RX_AFE_ANARXSTSr
#define WRITE_RX_AFE_ANARXSTSr BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXSTSr
#define MODIFY_RX_AFE_ANARXSTSr BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXSTSr
#define READLN_RX_AFE_ANARXSTSr BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXSTSr
#define WRITELN_RX_AFE_ANARXSTSr BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXSTSr
#define WRITEALL_RX_AFE_ANARXSTSr BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXSTSr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_ANARXSTSr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_ANARXCTL
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b1
 * DESC:     Rx lane control register
 * RESETVAL: 0x1c40 (7232)
 * ACCESS:   R/W
 * FIELDS:
 *     STATUS_SEL       
 *     FLIP_EYEMON_POLARITY 
 *     FORCERXSEQDONE_SM 
 *     PHFREQ_RST_DIS_NRML_SM 
 *     PHFREQ_RST_DIS_FST_SM 
 *     OVERRIDE_SIGDET_VAL Override signal detect enable value.
 *     OVERRIDE_SIGDET_EN Override signal detect enable.
 *     SIGDETMONITOR_EN_SM 
 *     SIGDETRESTART_EN_SM 
 *     SIGDETECTED_EN_SM 
 *     FAST_ACQ_EN_FORCE_R 
 *     FAST_ACQ_EN_R    
 *     RXSEQRESTART_SM  
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr (0x000080b1 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_ANARXCTL.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_s {
	uint32_t v[1];
	uint32_t rx_afe_anarxctl[1];
	uint32_t _rx_afe_anarxctl;
} BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_t;

#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_CLR(r) (r).rx_afe_anarxctl[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SET(r,d) (r).rx_afe_anarxctl[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_GET(r) (r).rx_afe_anarxctl[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_RXSEQRESTART_SMf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_RXSEQRESTART_SMf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FAST_ACQ_EN_Rf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FAST_ACQ_EN_Rf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FAST_ACQ_EN_FORCE_Rf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FAST_ACQ_EN_FORCE_Rf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETECTED_EN_SMf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETECTED_EN_SMf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETRESTART_EN_SMf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETRESTART_EN_SMf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETMONITOR_EN_SMf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETMONITOR_EN_SMf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_ENf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_ENf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_VALf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_VALf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_FST_SMf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_FST_SMf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_NRML_SMf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_NRML_SMf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FORCERXSEQDONE_SMf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FORCERXSEQDONE_SMf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FLIP_EYEMON_POLARITYf_GET(r) ((((r).rx_afe_anarxctl[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FLIP_EYEMON_POLARITYf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_STATUS_SELf_GET(r) (((r).rx_afe_anarxctl[0]) & 0x7)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_STATUS_SELf_SET(r,f) (r).rx_afe_anarxctl[0]=(((r).rx_afe_anarxctl[0] & ~((uint32_t)0x7)) | (((uint32_t)f) & 0x7)) | (0x7 << 16)

/*
 * These macros can be used to access RX_AFE_ANARXCTL.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_ANARXCTLr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr,(_r._rx_afe_anarxctl))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXCTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr,(_r._rx_afe_anarxctl)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXCTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr,(_r._rx_afe_anarxctl))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXCTLr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxctl))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXCTLr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxctl))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXCTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_anarxctl))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_ANARXCTLr BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr
#define RX_AFE_ANARXCTLr_SIZE BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_t RX_AFE_ANARXCTLr_t;
#define RX_AFE_ANARXCTLr_CLR BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_CLR
#define RX_AFE_ANARXCTLr_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SET
#define RX_AFE_ANARXCTLr_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_GET
#define RX_AFE_ANARXCTLr_RXSEQRESTART_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_RXSEQRESTART_SMf_GET
#define RX_AFE_ANARXCTLr_RXSEQRESTART_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_RXSEQRESTART_SMf_SET
#define RX_AFE_ANARXCTLr_FAST_ACQ_EN_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FAST_ACQ_EN_Rf_GET
#define RX_AFE_ANARXCTLr_FAST_ACQ_EN_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FAST_ACQ_EN_Rf_SET
#define RX_AFE_ANARXCTLr_FAST_ACQ_EN_FORCE_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FAST_ACQ_EN_FORCE_Rf_GET
#define RX_AFE_ANARXCTLr_FAST_ACQ_EN_FORCE_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FAST_ACQ_EN_FORCE_Rf_SET
#define RX_AFE_ANARXCTLr_SIGDETECTED_EN_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETECTED_EN_SMf_GET
#define RX_AFE_ANARXCTLr_SIGDETECTED_EN_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETECTED_EN_SMf_SET
#define RX_AFE_ANARXCTLr_SIGDETRESTART_EN_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETRESTART_EN_SMf_GET
#define RX_AFE_ANARXCTLr_SIGDETRESTART_EN_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETRESTART_EN_SMf_SET
#define RX_AFE_ANARXCTLr_SIGDETMONITOR_EN_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETMONITOR_EN_SMf_GET
#define RX_AFE_ANARXCTLr_SIGDETMONITOR_EN_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_SIGDETMONITOR_EN_SMf_SET
#define RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_ENf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_ENf_GET
#define RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_ENf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_ENf_SET
#define RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_VALf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_VALf_GET
#define RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_VALf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_OVERRIDE_SIGDET_VALf_SET
#define RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_FST_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_FST_SMf_GET
#define RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_FST_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_FST_SMf_SET
#define RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_NRML_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_NRML_SMf_GET
#define RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_NRML_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_PHFREQ_RST_DIS_NRML_SMf_SET
#define RX_AFE_ANARXCTLr_FORCERXSEQDONE_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FORCERXSEQDONE_SMf_GET
#define RX_AFE_ANARXCTLr_FORCERXSEQDONE_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FORCERXSEQDONE_SMf_SET
#define RX_AFE_ANARXCTLr_FLIP_EYEMON_POLARITYf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FLIP_EYEMON_POLARITYf_GET
#define RX_AFE_ANARXCTLr_FLIP_EYEMON_POLARITYf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_FLIP_EYEMON_POLARITYf_SET
#define RX_AFE_ANARXCTLr_STATUS_SELf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_STATUS_SELf_GET
#define RX_AFE_ANARXCTLr_STATUS_SELf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr_STATUS_SELf_SET
#define READ_RX_AFE_ANARXCTLr BCMI_VIPER_XGXS_READ_RX_AFE_ANARXCTLr
#define WRITE_RX_AFE_ANARXCTLr BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXCTLr
#define MODIFY_RX_AFE_ANARXCTLr BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXCTLr
#define READLN_RX_AFE_ANARXCTLr BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXCTLr
#define WRITELN_RX_AFE_ANARXCTLr BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXCTLr
#define WRITEALL_RX_AFE_ANARXCTLr BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXCTLr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_ANARXCTLr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_CTL0
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b2
 * DESC:     AFE Control Register 15:0
 * RESETVAL: 0x780 (1920)
 * ACCESS:   R/W
 * FIELDS:
 *     RESERVED_0       
 *     PD_EQZ           Equalizer power down
 *     PD_LMTNG         Limiting amplifier power down
 *     RESERVED_5_3     
 *     FILTER_BAND      Controls degeneration cap in Equalizer00 2.3pF01 4.1pF11 3.7pF10 100fF (Default)
 *     RX_PON           Resistor Calibration Control Setting0000: min resistance process corner, (increases on-chip R by +25%)1111: max resistance process corner, (decreases on-chip R by -25%)Error between two consecutive codes is maintained less than 3%.By default, pon<3:0> match with resistor calibration output; they can also be independently adjusted through RTL.
 *     RESERVED_14_12   
 *     SLICER_PD        Power down for the  full slicer, and clock distribution.
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r (0x000080b2 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_CTL0.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_CTL0r_s {
	uint32_t v[1];
	uint32_t rx_afe_ctl0[1];
	uint32_t _rx_afe_ctl0;
} BCMI_VIPER_XGXS_RX_AFE_CTL0r_t;

#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_CLR(r) (r).rx_afe_ctl0[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_SET(r,d) (r).rx_afe_ctl0[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_GET(r) (r).rx_afe_ctl0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_SLICER_PDf_GET(r) ((((r).rx_afe_ctl0[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_SLICER_PDf_SET(r,f) (r).rx_afe_ctl0[0]=(((r).rx_afe_ctl0[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_14_12f_GET(r) ((((r).rx_afe_ctl0[0]) >> 12) & 0x7)
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_14_12f_SET(r,f) (r).rx_afe_ctl0[0]=(((r).rx_afe_ctl0[0] & ~((uint32_t)0x7 << 12)) | ((((uint32_t)f) & 0x7) << 12)) | (7 << (16 + 12))
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_RX_PONf_GET(r) ((((r).rx_afe_ctl0[0]) >> 8) & 0xf)
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_RX_PONf_SET(r,f) (r).rx_afe_ctl0[0]=(((r).rx_afe_ctl0[0] & ~((uint32_t)0xf << 8)) | ((((uint32_t)f) & 0xf) << 8)) | (15 << (16 + 8))
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_FILTER_BANDf_GET(r) ((((r).rx_afe_ctl0[0]) >> 6) & 0x3)
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_FILTER_BANDf_SET(r,f) (r).rx_afe_ctl0[0]=(((r).rx_afe_ctl0[0] & ~((uint32_t)0x3 << 6)) | ((((uint32_t)f) & 0x3) << 6)) | (3 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_5_3f_GET(r) ((((r).rx_afe_ctl0[0]) >> 3) & 0x7)
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_5_3f_SET(r,f) (r).rx_afe_ctl0[0]=(((r).rx_afe_ctl0[0] & ~((uint32_t)0x7 << 3)) | ((((uint32_t)f) & 0x7) << 3)) | (7 << (16 + 3))
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_PD_LMTNGf_GET(r) ((((r).rx_afe_ctl0[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_PD_LMTNGf_SET(r,f) (r).rx_afe_ctl0[0]=(((r).rx_afe_ctl0[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_PD_EQZf_GET(r) ((((r).rx_afe_ctl0[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_PD_EQZf_SET(r,f) (r).rx_afe_ctl0[0]=(((r).rx_afe_ctl0[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_0f_GET(r) (((r).rx_afe_ctl0[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_0f_SET(r,f) (r).rx_afe_ctl0[0]=(((r).rx_afe_ctl0[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX_AFE_CTL0.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL0r,(_r._rx_afe_ctl0))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL0r,(_r._rx_afe_ctl0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL0r,(_r._rx_afe_ctl0))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_CTL0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl0))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl0))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_ctl0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_CTL0r BCMI_VIPER_XGXS_RX_AFE_CTL0r
#define RX_AFE_CTL0r_SIZE BCMI_VIPER_XGXS_RX_AFE_CTL0r_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_CTL0r_t RX_AFE_CTL0r_t;
#define RX_AFE_CTL0r_CLR BCMI_VIPER_XGXS_RX_AFE_CTL0r_CLR
#define RX_AFE_CTL0r_SET BCMI_VIPER_XGXS_RX_AFE_CTL0r_SET
#define RX_AFE_CTL0r_GET BCMI_VIPER_XGXS_RX_AFE_CTL0r_GET
#define RX_AFE_CTL0r_SLICER_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL0r_SLICER_PDf_GET
#define RX_AFE_CTL0r_SLICER_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL0r_SLICER_PDf_SET
#define RX_AFE_CTL0r_RESERVED_14_12f_GET BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_14_12f_GET
#define RX_AFE_CTL0r_RESERVED_14_12f_SET BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_14_12f_SET
#define RX_AFE_CTL0r_RX_PONf_GET BCMI_VIPER_XGXS_RX_AFE_CTL0r_RX_PONf_GET
#define RX_AFE_CTL0r_RX_PONf_SET BCMI_VIPER_XGXS_RX_AFE_CTL0r_RX_PONf_SET
#define RX_AFE_CTL0r_FILTER_BANDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL0r_FILTER_BANDf_GET
#define RX_AFE_CTL0r_FILTER_BANDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL0r_FILTER_BANDf_SET
#define RX_AFE_CTL0r_RESERVED_5_3f_GET BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_5_3f_GET
#define RX_AFE_CTL0r_RESERVED_5_3f_SET BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_5_3f_SET
#define RX_AFE_CTL0r_PD_LMTNGf_GET BCMI_VIPER_XGXS_RX_AFE_CTL0r_PD_LMTNGf_GET
#define RX_AFE_CTL0r_PD_LMTNGf_SET BCMI_VIPER_XGXS_RX_AFE_CTL0r_PD_LMTNGf_SET
#define RX_AFE_CTL0r_PD_EQZf_GET BCMI_VIPER_XGXS_RX_AFE_CTL0r_PD_EQZf_GET
#define RX_AFE_CTL0r_PD_EQZf_SET BCMI_VIPER_XGXS_RX_AFE_CTL0r_PD_EQZf_SET
#define RX_AFE_CTL0r_RESERVED_0f_GET BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_0f_GET
#define RX_AFE_CTL0r_RESERVED_0f_SET BCMI_VIPER_XGXS_RX_AFE_CTL0r_RESERVED_0f_SET
#define READ_RX_AFE_CTL0r BCMI_VIPER_XGXS_READ_RX_AFE_CTL0r
#define WRITE_RX_AFE_CTL0r BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL0r
#define MODIFY_RX_AFE_CTL0r BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL0r
#define READLN_RX_AFE_CTL0r BCMI_VIPER_XGXS_READLN_RX_AFE_CTL0r
#define WRITELN_RX_AFE_CTL0r BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL0r
#define WRITEALL_RX_AFE_CTL0r BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_CTL0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_CTL1
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b3
 * DESC:     AFE Control Register 31:16
 * RESETVAL: 0x8001 (32769)
 * ACCESS:   R/W
 * FIELDS:
 *     EYEM_PD          Power down for the  eye monitor slicer, and clock distribution
 *     RESERVED_26_17   
 *     DIV_4_DEMUX_ENABLE 0: DEMUX 10:1 mode output1: DEMUX 4:1  mode output
 *     DEMUX_ZERO_PD    Power down of the zero path DEMUX0 Enable (Default)1 Disable
 *     DEMUX_PEAK_PD    Power down of the peak path DEMUX0 Enable (Default)1 Disable
 *     DEMUX_PD         Power down of the full DEMUX0 Enable (Default)1 Disable
 *     DEMUX_EYEM_PD    Power down of the eye monitor path DEMUX0 Enable1 Disable (Default)
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r (0x000080b3 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_CTL1.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_CTL1r_s {
	uint32_t v[1];
	uint32_t rx_afe_ctl1[1];
	uint32_t _rx_afe_ctl1;
} BCMI_VIPER_XGXS_RX_AFE_CTL1r_t;

#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_CLR(r) (r).rx_afe_ctl1[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_SET(r,d) (r).rx_afe_ctl1[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_GET(r) (r).rx_afe_ctl1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_EYEM_PDf_GET(r) ((((r).rx_afe_ctl1[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_EYEM_PDf_SET(r,f) (r).rx_afe_ctl1[0]=(((r).rx_afe_ctl1[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_PDf_GET(r) ((((r).rx_afe_ctl1[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_PDf_SET(r,f) (r).rx_afe_ctl1[0]=(((r).rx_afe_ctl1[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_PEAK_PDf_GET(r) ((((r).rx_afe_ctl1[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_PEAK_PDf_SET(r,f) (r).rx_afe_ctl1[0]=(((r).rx_afe_ctl1[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_ZERO_PDf_GET(r) ((((r).rx_afe_ctl1[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_ZERO_PDf_SET(r,f) (r).rx_afe_ctl1[0]=(((r).rx_afe_ctl1[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DIV_4_DEMUX_ENABLEf_GET(r) ((((r).rx_afe_ctl1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_DIV_4_DEMUX_ENABLEf_SET(r,f) (r).rx_afe_ctl1[0]=(((r).rx_afe_ctl1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_RESERVED_26_17f_GET(r) ((((r).rx_afe_ctl1[0]) >> 1) & 0x3ff)
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_RESERVED_26_17f_SET(r,f) (r).rx_afe_ctl1[0]=(((r).rx_afe_ctl1[0] & ~((uint32_t)0x3ff << 1)) | ((((uint32_t)f) & 0x3ff) << 1)) | (1023 << (16 + 1))
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_EYEM_PDf_GET(r) (((r).rx_afe_ctl1[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL1r_EYEM_PDf_SET(r,f) (r).rx_afe_ctl1[0]=(((r).rx_afe_ctl1[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX_AFE_CTL1.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL1r,(_r._rx_afe_ctl1))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL1r,(_r._rx_afe_ctl1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL1r,(_r._rx_afe_ctl1))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_CTL1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl1))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl1))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_ctl1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_CTL1r BCMI_VIPER_XGXS_RX_AFE_CTL1r
#define RX_AFE_CTL1r_SIZE BCMI_VIPER_XGXS_RX_AFE_CTL1r_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_CTL1r_t RX_AFE_CTL1r_t;
#define RX_AFE_CTL1r_CLR BCMI_VIPER_XGXS_RX_AFE_CTL1r_CLR
#define RX_AFE_CTL1r_SET BCMI_VIPER_XGXS_RX_AFE_CTL1r_SET
#define RX_AFE_CTL1r_GET BCMI_VIPER_XGXS_RX_AFE_CTL1r_GET
#define RX_AFE_CTL1r_DEMUX_EYEM_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_EYEM_PDf_GET
#define RX_AFE_CTL1r_DEMUX_EYEM_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_EYEM_PDf_SET
#define RX_AFE_CTL1r_DEMUX_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_PDf_GET
#define RX_AFE_CTL1r_DEMUX_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_PDf_SET
#define RX_AFE_CTL1r_DEMUX_PEAK_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_PEAK_PDf_GET
#define RX_AFE_CTL1r_DEMUX_PEAK_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_PEAK_PDf_SET
#define RX_AFE_CTL1r_DEMUX_ZERO_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_ZERO_PDf_GET
#define RX_AFE_CTL1r_DEMUX_ZERO_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DEMUX_ZERO_PDf_SET
#define RX_AFE_CTL1r_DIV_4_DEMUX_ENABLEf_GET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DIV_4_DEMUX_ENABLEf_GET
#define RX_AFE_CTL1r_DIV_4_DEMUX_ENABLEf_SET BCMI_VIPER_XGXS_RX_AFE_CTL1r_DIV_4_DEMUX_ENABLEf_SET
#define RX_AFE_CTL1r_RESERVED_26_17f_GET BCMI_VIPER_XGXS_RX_AFE_CTL1r_RESERVED_26_17f_GET
#define RX_AFE_CTL1r_RESERVED_26_17f_SET BCMI_VIPER_XGXS_RX_AFE_CTL1r_RESERVED_26_17f_SET
#define RX_AFE_CTL1r_EYEM_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL1r_EYEM_PDf_GET
#define RX_AFE_CTL1r_EYEM_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL1r_EYEM_PDf_SET
#define READ_RX_AFE_CTL1r BCMI_VIPER_XGXS_READ_RX_AFE_CTL1r
#define WRITE_RX_AFE_CTL1r BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL1r
#define MODIFY_RX_AFE_CTL1r BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL1r
#define READLN_RX_AFE_CTL1r BCMI_VIPER_XGXS_READLN_RX_AFE_CTL1r
#define WRITELN_RX_AFE_CTL1r BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL1r
#define WRITEALL_RX_AFE_CTL1r BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_CTL1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_ANARXSIGDET
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b4
 * DESC:     Rx Sigdet Control
 * RESETVAL: 0x20 (32)
 * ACCESS:   R/W
 * FIELDS:
 *     RX_PF_CTRL    Receiver Peaking Equalizer 0x0 = dB_0p0 0x1 = dB_0p7 0x3 = dB_1p4 0x2 = dB_2p1 0x6 = dB_2p8 0x7 = dB_3p5 0x5 = dB_4p2 0x4 = dB_4p9
 *                                            0xc = dB_5p6 0xd = dB_6p3 0xf = dB_7p0 0xe = dB_7p7 0xa = dB_8p4 0xb = dB_9p1 0x9 = dB_9p8 0x8 = dB_10p5
 *     INVERT_RX_SIGDET 1 = invert rx_sigdet0 = normal
 *     RX_SIGDET_FORCE_R 1 = force rx_sigdet via bit40 = normal
 *     RX_SIGDET_R      1 = forced sigdet value of 10 = forced sigdet value of 0
 *     CX4_SIGDET_EN_SM 
 *     EXT_SIGDET_EN_SM 1 = enable ext sigdet0 = disable ext sigdet
 *     CX4_SIGDET_CNT_LD_SM 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr (0x000080b4 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_ANARXSIGDET.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_s {
	uint32_t v[1];
	uint32_t rx_afe_anarxsigdet[1];
	uint32_t _rx_afe_anarxsigdet;
} BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_t;

#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CLR(r) (r).rx_afe_anarxsigdet[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_SET(r,d) (r).rx_afe_anarxsigdet[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_GET(r) (r).rx_afe_anarxsigdet[0]

/*
 * These macros can be used to access individual fields.
 *
 */

#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_PF_CTRLf_GET(r) ((((r).rx_afe_anarxsigdet[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_PF_CTRLf_SET(r,f) (r).rx_afe_anarxsigdet[0]=(((r).rx_afe_anarxsigdet[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CX4_SIGDET_CNT_LD_SMf_GET(r) ((((r).rx_afe_anarxsigdet[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CX4_SIGDET_CNT_LD_SMf_SET(r,f) (r).rx_afe_anarxsigdet[0]=(((r).rx_afe_anarxsigdet[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_EXT_SIGDET_EN_SMf_GET(r) ((((r).rx_afe_anarxsigdet[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_EXT_SIGDET_EN_SMf_SET(r,f) (r).rx_afe_anarxsigdet[0]=(((r).rx_afe_anarxsigdet[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CX4_SIGDET_EN_SMf_GET(r) ((((r).rx_afe_anarxsigdet[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CX4_SIGDET_EN_SMf_SET(r,f) (r).rx_afe_anarxsigdet[0]=(((r).rx_afe_anarxsigdet[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_SIGDET_Rf_GET(r) ((((r).rx_afe_anarxsigdet[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_SIGDET_Rf_SET(r,f) (r).rx_afe_anarxsigdet[0]=(((r).rx_afe_anarxsigdet[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_SIGDET_FORCE_Rf_GET(r) ((((r).rx_afe_anarxsigdet[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_SIGDET_FORCE_Rf_SET(r,f) (r).rx_afe_anarxsigdet[0]=(((r).rx_afe_anarxsigdet[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_INVERT_RX_SIGDETf_GET(r) ((((r).rx_afe_anarxsigdet[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_INVERT_RX_SIGDETf_SET(r,f) (r).rx_afe_anarxsigdet[0]=(((r).rx_afe_anarxsigdet[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))

/*
 * These macros can be used to access RX_AFE_ANARXSIGDET.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_ANARXSIGDETr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr,(_r._rx_afe_anarxsigdet))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXSIGDETr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr,(_r._rx_afe_anarxsigdet)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXSIGDETr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr,(_r._rx_afe_anarxsigdet))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXSIGDETr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxsigdet))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXSIGDETr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxsigdet))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXSIGDETr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_anarxsigdet))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_ANARXSIGDETr BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr
#define RX_AFE_ANARXSIGDETr_SIZE BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_t RX_AFE_ANARXSIGDETr_t;
#define RX_AFE_ANARXSIGDETr_CLR BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CLR
#define RX_AFE_ANARXSIGDETr_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_SET
#define RX_AFE_ANARXSIGDETr_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_GET
#define RX_AFE_ANARXSIGDETr_RX_PF_CTRLf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_PF_CTRLf_GET
#define RX_AFE_ANARXSIGDETr_RX_PF_CTRLf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_PF_CTRLf_SET
#define RX_AFE_ANARXSIGDETr_CX4_SIGDET_CNT_LD_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CX4_SIGDET_CNT_LD_SMf_GET
#define RX_AFE_ANARXSIGDETr_CX4_SIGDET_CNT_LD_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CX4_SIGDET_CNT_LD_SMf_SET
#define RX_AFE_ANARXSIGDETr_EXT_SIGDET_EN_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_EXT_SIGDET_EN_SMf_GET
#define RX_AFE_ANARXSIGDETr_EXT_SIGDET_EN_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_EXT_SIGDET_EN_SMf_SET
#define RX_AFE_ANARXSIGDETr_CX4_SIGDET_EN_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CX4_SIGDET_EN_SMf_GET
#define RX_AFE_ANARXSIGDETr_CX4_SIGDET_EN_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_CX4_SIGDET_EN_SMf_SET
#define RX_AFE_ANARXSIGDETr_RX_SIGDET_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_SIGDET_Rf_GET
#define RX_AFE_ANARXSIGDETr_RX_SIGDET_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_SIGDET_Rf_SET
#define RX_AFE_ANARXSIGDETr_RX_SIGDET_FORCE_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_SIGDET_FORCE_Rf_GET
#define RX_AFE_ANARXSIGDETr_RX_SIGDET_FORCE_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_RX_SIGDET_FORCE_Rf_SET
#define RX_AFE_ANARXSIGDETr_INVERT_RX_SIGDETf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_INVERT_RX_SIGDETf_GET
#define RX_AFE_ANARXSIGDETr_INVERT_RX_SIGDETf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr_INVERT_RX_SIGDETf_SET
#define READ_RX_AFE_ANARXSIGDETr BCMI_VIPER_XGXS_READ_RX_AFE_ANARXSIGDETr
#define WRITE_RX_AFE_ANARXSIGDETr BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXSIGDETr
#define MODIFY_RX_AFE_ANARXSIGDETr BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXSIGDETr
#define READLN_RX_AFE_ANARXSIGDETr BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXSIGDETr
#define WRITELN_RX_AFE_ANARXSIGDETr BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXSIGDETr
#define WRITEALL_RX_AFE_ANARXSIGDETr BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXSIGDETr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_ANARXSIGDETr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_CTL2
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b5
 * DESC:     AFE Control Register 47:32
 * RESETVAL: 0xa (10)
 * ACCESS:   R/W
 * FIELDS:
 *     RESERVED_32      
 *     DIV4_PD          Power down divide by 4 to the RTL0 div4 out is active1 div4 out is idle  (default)
 *     DIV10_PD         Power down divide by 10 to the RTL0 div10 out is active (default)1 div10 out is idle
 *     INPUTERM_CM_EN   0: common mode DC floating1: Common mode connected to DC voltageDefault for CM should on DC set at 0.8V
 *     INPUTERM_CMULT_EN 0: common mode capacitor  multiplier active OFF1: common mode capacitor  multiplier active ON
 *     INPUTERM_LOWZGND_EN 1: Input common mode DC value 0
 *     INPUTERM_LOWZVDD_EN 1: Input common mode DC value VDD
 *     RESERVED_47_39   
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r (0x000080b5 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_CTL2.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_CTL2r_s {
	uint32_t v[1];
	uint32_t rx_afe_ctl2[1];
	uint32_t _rx_afe_ctl2;
} BCMI_VIPER_XGXS_RX_AFE_CTL2r_t;

#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_CLR(r) (r).rx_afe_ctl2[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_SET(r,d) (r).rx_afe_ctl2[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_GET(r) (r).rx_afe_ctl2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_RESERVED_47_39f_GET(r) ((((r).rx_afe_ctl2[0]) >> 7) & 0x1ff)
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_RESERVED_47_39f_SET(r,f) (r).rx_afe_ctl2[0]=(((r).rx_afe_ctl2[0] & ~((uint32_t)0x1ff << 7)) | ((((uint32_t)f) & 0x1ff) << 7)) | (511 << (16 + 7))
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_LOWZVDD_ENf_GET(r) ((((r).rx_afe_ctl2[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_LOWZVDD_ENf_SET(r,f) (r).rx_afe_ctl2[0]=(((r).rx_afe_ctl2[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_LOWZGND_ENf_GET(r) ((((r).rx_afe_ctl2[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_LOWZGND_ENf_SET(r,f) (r).rx_afe_ctl2[0]=(((r).rx_afe_ctl2[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_CMULT_ENf_GET(r) ((((r).rx_afe_ctl2[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_CMULT_ENf_SET(r,f) (r).rx_afe_ctl2[0]=(((r).rx_afe_ctl2[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_CM_ENf_GET(r) ((((r).rx_afe_ctl2[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_CM_ENf_SET(r,f) (r).rx_afe_ctl2[0]=(((r).rx_afe_ctl2[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_DIV10_PDf_GET(r) ((((r).rx_afe_ctl2[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_DIV10_PDf_SET(r,f) (r).rx_afe_ctl2[0]=(((r).rx_afe_ctl2[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_DIV4_PDf_GET(r) ((((r).rx_afe_ctl2[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_DIV4_PDf_SET(r,f) (r).rx_afe_ctl2[0]=(((r).rx_afe_ctl2[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_RESERVED_32f_GET(r) (((r).rx_afe_ctl2[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL2r_RESERVED_32f_SET(r,f) (r).rx_afe_ctl2[0]=(((r).rx_afe_ctl2[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX_AFE_CTL2.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL2r,(_r._rx_afe_ctl2))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL2r,(_r._rx_afe_ctl2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL2r,(_r._rx_afe_ctl2))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_CTL2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl2))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl2))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_ctl2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_CTL2r BCMI_VIPER_XGXS_RX_AFE_CTL2r
#define RX_AFE_CTL2r_SIZE BCMI_VIPER_XGXS_RX_AFE_CTL2r_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_CTL2r_t RX_AFE_CTL2r_t;
#define RX_AFE_CTL2r_CLR BCMI_VIPER_XGXS_RX_AFE_CTL2r_CLR
#define RX_AFE_CTL2r_SET BCMI_VIPER_XGXS_RX_AFE_CTL2r_SET
#define RX_AFE_CTL2r_GET BCMI_VIPER_XGXS_RX_AFE_CTL2r_GET
#define RX_AFE_CTL2r_RESERVED_47_39f_GET BCMI_VIPER_XGXS_RX_AFE_CTL2r_RESERVED_47_39f_GET
#define RX_AFE_CTL2r_RESERVED_47_39f_SET BCMI_VIPER_XGXS_RX_AFE_CTL2r_RESERVED_47_39f_SET
#define RX_AFE_CTL2r_INPUTERM_LOWZVDD_ENf_GET BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_LOWZVDD_ENf_GET
#define RX_AFE_CTL2r_INPUTERM_LOWZVDD_ENf_SET BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_LOWZVDD_ENf_SET
#define RX_AFE_CTL2r_INPUTERM_LOWZGND_ENf_GET BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_LOWZGND_ENf_GET
#define RX_AFE_CTL2r_INPUTERM_LOWZGND_ENf_SET BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_LOWZGND_ENf_SET
#define RX_AFE_CTL2r_INPUTERM_CMULT_ENf_GET BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_CMULT_ENf_GET
#define RX_AFE_CTL2r_INPUTERM_CMULT_ENf_SET BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_CMULT_ENf_SET
#define RX_AFE_CTL2r_INPUTERM_CM_ENf_GET BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_CM_ENf_GET
#define RX_AFE_CTL2r_INPUTERM_CM_ENf_SET BCMI_VIPER_XGXS_RX_AFE_CTL2r_INPUTERM_CM_ENf_SET
#define RX_AFE_CTL2r_DIV10_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL2r_DIV10_PDf_GET
#define RX_AFE_CTL2r_DIV10_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL2r_DIV10_PDf_SET
#define RX_AFE_CTL2r_DIV4_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL2r_DIV4_PDf_GET
#define RX_AFE_CTL2r_DIV4_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL2r_DIV4_PDf_SET
#define RX_AFE_CTL2r_RESERVED_32f_GET BCMI_VIPER_XGXS_RX_AFE_CTL2r_RESERVED_32f_GET
#define RX_AFE_CTL2r_RESERVED_32f_SET BCMI_VIPER_XGXS_RX_AFE_CTL2r_RESERVED_32f_SET
#define READ_RX_AFE_CTL2r BCMI_VIPER_XGXS_READ_RX_AFE_CTL2r
#define WRITE_RX_AFE_CTL2r BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL2r
#define MODIFY_RX_AFE_CTL2r BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL2r
#define READLN_RX_AFE_CTL2r BCMI_VIPER_XGXS_READLN_RX_AFE_CTL2r
#define WRITELN_RX_AFE_CTL2r BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL2r
#define WRITEALL_RX_AFE_CTL2r BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_CTL2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_CTL3
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b6
 * DESC:     AFE Control Register 63:48
 * RESETVAL: 0xc0 (192)
 * ACCESS:   R/W
 * FIELDS:
 *     RESERVED_52_48   
 *     DCC_EN           Enable automatic duty cycle correction circuitDefault is disabled
 *     DUTY_CYCLE       Duty cycle correction for internal 1T clock from PLL:100 - 46.5%101 - 48.5%110 - 50.5%111 - 52.5%0xx  open loop duty cycle correction disabled
 *     PD_BIAS          Power down of the bias circuit0: Enabled (Default)1: Disabled
 *     RESERVED_63_58   
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r (0x000080b6 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_CTL3.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_CTL3r_s {
	uint32_t v[1];
	uint32_t rx_afe_ctl3[1];
	uint32_t _rx_afe_ctl3;
} BCMI_VIPER_XGXS_RX_AFE_CTL3r_t;

#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_CLR(r) (r).rx_afe_ctl3[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_SET(r,d) (r).rx_afe_ctl3[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_GET(r) (r).rx_afe_ctl3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_RESERVED_63_58f_GET(r) ((((r).rx_afe_ctl3[0]) >> 10) & 0x3f)
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_RESERVED_63_58f_SET(r,f) (r).rx_afe_ctl3[0]=(((r).rx_afe_ctl3[0] & ~((uint32_t)0x3f << 10)) | ((((uint32_t)f) & 0x3f) << 10)) | (63 << (16 + 10))
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_PD_BIASf_GET(r) ((((r).rx_afe_ctl3[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_PD_BIASf_SET(r,f) (r).rx_afe_ctl3[0]=(((r).rx_afe_ctl3[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_DUTY_CYCLEf_GET(r) ((((r).rx_afe_ctl3[0]) >> 6) & 0x7)
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_DUTY_CYCLEf_SET(r,f) (r).rx_afe_ctl3[0]=(((r).rx_afe_ctl3[0] & ~((uint32_t)0x7 << 6)) | ((((uint32_t)f) & 0x7) << 6)) | (7 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_DCC_ENf_GET(r) ((((r).rx_afe_ctl3[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_DCC_ENf_SET(r,f) (r).rx_afe_ctl3[0]=(((r).rx_afe_ctl3[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_RESERVED_52_48f_GET(r) (((r).rx_afe_ctl3[0]) & 0x1f)
#define BCMI_VIPER_XGXS_RX_AFE_CTL3r_RESERVED_52_48f_SET(r,f) (r).rx_afe_ctl3[0]=(((r).rx_afe_ctl3[0] & ~((uint32_t)0x1f)) | (((uint32_t)f) & 0x1f)) | (0x1f << 16)

/*
 * These macros can be used to access RX_AFE_CTL3.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL3r,(_r._rx_afe_ctl3))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL3r,(_r._rx_afe_ctl3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL3r,(_r._rx_afe_ctl3))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_CTL3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl3))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl3))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_ctl3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_CTL3r BCMI_VIPER_XGXS_RX_AFE_CTL3r
#define RX_AFE_CTL3r_SIZE BCMI_VIPER_XGXS_RX_AFE_CTL3r_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_CTL3r_t RX_AFE_CTL3r_t;
#define RX_AFE_CTL3r_CLR BCMI_VIPER_XGXS_RX_AFE_CTL3r_CLR
#define RX_AFE_CTL3r_SET BCMI_VIPER_XGXS_RX_AFE_CTL3r_SET
#define RX_AFE_CTL3r_GET BCMI_VIPER_XGXS_RX_AFE_CTL3r_GET
#define RX_AFE_CTL3r_RESERVED_63_58f_GET BCMI_VIPER_XGXS_RX_AFE_CTL3r_RESERVED_63_58f_GET
#define RX_AFE_CTL3r_RESERVED_63_58f_SET BCMI_VIPER_XGXS_RX_AFE_CTL3r_RESERVED_63_58f_SET
#define RX_AFE_CTL3r_PD_BIASf_GET BCMI_VIPER_XGXS_RX_AFE_CTL3r_PD_BIASf_GET
#define RX_AFE_CTL3r_PD_BIASf_SET BCMI_VIPER_XGXS_RX_AFE_CTL3r_PD_BIASf_SET
#define RX_AFE_CTL3r_DUTY_CYCLEf_GET BCMI_VIPER_XGXS_RX_AFE_CTL3r_DUTY_CYCLEf_GET
#define RX_AFE_CTL3r_DUTY_CYCLEf_SET BCMI_VIPER_XGXS_RX_AFE_CTL3r_DUTY_CYCLEf_SET
#define RX_AFE_CTL3r_DCC_ENf_GET BCMI_VIPER_XGXS_RX_AFE_CTL3r_DCC_ENf_GET
#define RX_AFE_CTL3r_DCC_ENf_SET BCMI_VIPER_XGXS_RX_AFE_CTL3r_DCC_ENf_SET
#define RX_AFE_CTL3r_RESERVED_52_48f_GET BCMI_VIPER_XGXS_RX_AFE_CTL3r_RESERVED_52_48f_GET
#define RX_AFE_CTL3r_RESERVED_52_48f_SET BCMI_VIPER_XGXS_RX_AFE_CTL3r_RESERVED_52_48f_SET
#define READ_RX_AFE_CTL3r BCMI_VIPER_XGXS_READ_RX_AFE_CTL3r
#define WRITE_RX_AFE_CTL3r BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL3r
#define MODIFY_RX_AFE_CTL3r BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL3r
#define READLN_RX_AFE_CTL3r BCMI_VIPER_XGXS_READLN_RX_AFE_CTL3r
#define WRITELN_RX_AFE_CTL3r BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL3r
#define WRITEALL_RX_AFE_CTL3r BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_CTL3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_CTL4
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b7
 * DESC:     AFE Control Register 79:64
 * RESETVAL: 0x492 (1170)
 * ACCESS:   R/W
 * FIELDS:
 *     BIAS_LA_CTRL     Controls the current biasing for the limiting amplifierIout1	1	1	175%1	1	0	160%1	0	1	145%1	0	0	130%0	1	1	115%0	1	0	100%0	0	1	85%0	0	0	70%
 *     BIAS_LA_DAC_CTRL Controls the current biasing for the limiting amplifier offset trimming DACIout1	1	1	175%1	1	0	160%1	0	1	145%1	0	0	130%0	1	1	115%0	1	0	100%0	0	1	85%0	0	0	70%
 *     BIAS_EYEM_CTRL   Controls the current biasing for the eye monitor referenceIout1	1	1	175%1	1	0	160%1	0	1	145%1	0	0	130%0	1	1	115%0	1	0	100%0	0	1	85%0	0	0	70%
 *     BIAS_SIGDET_CTRL Controls the current biasing for the signal detector biasingIout1	1	1	175%1	1	0	160%1	0	1	145%1	0	0	130%0	1	1	115%0	1	0	100%0	0	1	85%0	0	0	70%
 *     SIGDET_BYPASS    Active High:1:  Bypass Sig Detect output and send 1 at sigdetect output0:  Normal sigdetect operation (Default)
 *     SIGDET_PD        Powerdown for the sigdet0: Enabled (Default)1: Disabled
 *     SIGDET_MODESELECT Selects the mode of operation of the sigdet: (not used)0:   PCIE mode and USB3.0 mode
 *     EN_TESTMUX       Enable test mux0: Disabled (Default)1: Enabled
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r (0x000080b7 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_CTL4.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_CTL4r_s {
	uint32_t v[1];
	uint32_t rx_afe_ctl4[1];
	uint32_t _rx_afe_ctl4;
} BCMI_VIPER_XGXS_RX_AFE_CTL4r_t;

#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_CLR(r) (r).rx_afe_ctl4[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_SET(r,d) (r).rx_afe_ctl4[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_GET(r) (r).rx_afe_ctl4[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_EN_TESTMUXf_GET(r) ((((r).rx_afe_ctl4[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_EN_TESTMUXf_SET(r,f) (r).rx_afe_ctl4[0]=(((r).rx_afe_ctl4[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_MODESELECTf_GET(r) ((((r).rx_afe_ctl4[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_MODESELECTf_SET(r,f) (r).rx_afe_ctl4[0]=(((r).rx_afe_ctl4[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_PDf_GET(r) ((((r).rx_afe_ctl4[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_PDf_SET(r,f) (r).rx_afe_ctl4[0]=(((r).rx_afe_ctl4[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_BYPASSf_GET(r) ((((r).rx_afe_ctl4[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_BYPASSf_SET(r,f) (r).rx_afe_ctl4[0]=(((r).rx_afe_ctl4[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_SIGDET_CTRLf_GET(r) ((((r).rx_afe_ctl4[0]) >> 9) & 0x7)
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_SIGDET_CTRLf_SET(r,f) (r).rx_afe_ctl4[0]=(((r).rx_afe_ctl4[0] & ~((uint32_t)0x7 << 9)) | ((((uint32_t)f) & 0x7) << 9)) | (7 << (16 + 9))
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_EYEM_CTRLf_GET(r) ((((r).rx_afe_ctl4[0]) >> 6) & 0x7)
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_EYEM_CTRLf_SET(r,f) (r).rx_afe_ctl4[0]=(((r).rx_afe_ctl4[0] & ~((uint32_t)0x7 << 6)) | ((((uint32_t)f) & 0x7) << 6)) | (7 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_LA_DAC_CTRLf_GET(r) ((((r).rx_afe_ctl4[0]) >> 3) & 0x7)
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_LA_DAC_CTRLf_SET(r,f) (r).rx_afe_ctl4[0]=(((r).rx_afe_ctl4[0] & ~((uint32_t)0x7 << 3)) | ((((uint32_t)f) & 0x7) << 3)) | (7 << (16 + 3))
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_LA_CTRLf_GET(r) (((r).rx_afe_ctl4[0]) & 0x7)
#define BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_LA_CTRLf_SET(r,f) (r).rx_afe_ctl4[0]=(((r).rx_afe_ctl4[0] & ~((uint32_t)0x7)) | (((uint32_t)f) & 0x7)) | (0x7 << 16)

/*
 * These macros can be used to access RX_AFE_CTL4.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_CTL4r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL4r,(_r._rx_afe_ctl4))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL4r,(_r._rx_afe_ctl4)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL4r,(_r._rx_afe_ctl4))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_CTL4r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl4))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL4r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl4))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_ctl4))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_CTL4r BCMI_VIPER_XGXS_RX_AFE_CTL4r
#define RX_AFE_CTL4r_SIZE BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_CTL4r_t RX_AFE_CTL4r_t;
#define RX_AFE_CTL4r_CLR BCMI_VIPER_XGXS_RX_AFE_CTL4r_CLR
#define RX_AFE_CTL4r_SET BCMI_VIPER_XGXS_RX_AFE_CTL4r_SET
#define RX_AFE_CTL4r_GET BCMI_VIPER_XGXS_RX_AFE_CTL4r_GET
#define RX_AFE_CTL4r_EN_TESTMUXf_GET BCMI_VIPER_XGXS_RX_AFE_CTL4r_EN_TESTMUXf_GET
#define RX_AFE_CTL4r_EN_TESTMUXf_SET BCMI_VIPER_XGXS_RX_AFE_CTL4r_EN_TESTMUXf_SET
#define RX_AFE_CTL4r_SIGDET_MODESELECTf_GET BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_MODESELECTf_GET
#define RX_AFE_CTL4r_SIGDET_MODESELECTf_SET BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_MODESELECTf_SET
#define RX_AFE_CTL4r_SIGDET_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_PDf_GET
#define RX_AFE_CTL4r_SIGDET_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_PDf_SET
#define RX_AFE_CTL4r_SIGDET_BYPASSf_GET BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_BYPASSf_GET
#define RX_AFE_CTL4r_SIGDET_BYPASSf_SET BCMI_VIPER_XGXS_RX_AFE_CTL4r_SIGDET_BYPASSf_SET
#define RX_AFE_CTL4r_BIAS_SIGDET_CTRLf_GET BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_SIGDET_CTRLf_GET
#define RX_AFE_CTL4r_BIAS_SIGDET_CTRLf_SET BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_SIGDET_CTRLf_SET
#define RX_AFE_CTL4r_BIAS_EYEM_CTRLf_GET BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_EYEM_CTRLf_GET
#define RX_AFE_CTL4r_BIAS_EYEM_CTRLf_SET BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_EYEM_CTRLf_SET
#define RX_AFE_CTL4r_BIAS_LA_DAC_CTRLf_GET BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_LA_DAC_CTRLf_GET
#define RX_AFE_CTL4r_BIAS_LA_DAC_CTRLf_SET BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_LA_DAC_CTRLf_SET
#define RX_AFE_CTL4r_BIAS_LA_CTRLf_GET BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_LA_CTRLf_GET
#define RX_AFE_CTL4r_BIAS_LA_CTRLf_SET BCMI_VIPER_XGXS_RX_AFE_CTL4r_BIAS_LA_CTRLf_SET
#define READ_RX_AFE_CTL4r BCMI_VIPER_XGXS_READ_RX_AFE_CTL4r
#define WRITE_RX_AFE_CTL4r BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL4r
#define MODIFY_RX_AFE_CTL4r BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL4r
#define READLN_RX_AFE_CTL4r BCMI_VIPER_XGXS_READLN_RX_AFE_CTL4r
#define WRITELN_RX_AFE_CTL4r BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL4r
#define WRITEALL_RX_AFE_CTL4r BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL4r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_CTL4r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_ANARXTEST
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b8
 * DESC:     Rx lane control register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TESTMUXSELECT_SM 
 *     TPCTRL_SM        
 *     SIGDET_MUX_SM    
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr (0x000080b8 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_ANARXTEST.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_s {
	uint32_t v[1];
	uint32_t rx_afe_anarxtest[1];
	uint32_t _rx_afe_anarxtest;
} BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_t;

#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_CLR(r) (r).rx_afe_anarxtest[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_SET(r,d) (r).rx_afe_anarxtest[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_GET(r) (r).rx_afe_anarxtest[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_SIGDET_MUX_SMf_GET(r) ((((r).rx_afe_anarxtest[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_SIGDET_MUX_SMf_SET(r,f) (r).rx_afe_anarxtest[0]=(((r).rx_afe_anarxtest[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_TPCTRL_SMf_GET(r) ((((r).rx_afe_anarxtest[0]) >> 4) & 0x1f)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_TPCTRL_SMf_SET(r,f) (r).rx_afe_anarxtest[0]=(((r).rx_afe_anarxtest[0] & ~((uint32_t)0x1f << 4)) | ((((uint32_t)f) & 0x1f) << 4)) | (31 << (16 + 4))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_TESTMUXSELECT_SMf_GET(r) (((r).rx_afe_anarxtest[0]) & 0xf)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_TESTMUXSELECT_SMf_SET(r,f) (r).rx_afe_anarxtest[0]=(((r).rx_afe_anarxtest[0] & ~((uint32_t)0xf)) | (((uint32_t)f) & 0xf)) | (0xf << 16)

/*
 * These macros can be used to access RX_AFE_ANARXTEST.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_ANARXTESTr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr,(_r._rx_afe_anarxtest))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXTESTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr,(_r._rx_afe_anarxtest)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXTESTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr,(_r._rx_afe_anarxtest))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXTESTr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxtest))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXTESTr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxtest))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXTESTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_anarxtest))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_ANARXTESTr BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr
#define RX_AFE_ANARXTESTr_SIZE BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_t RX_AFE_ANARXTESTr_t;
#define RX_AFE_ANARXTESTr_CLR BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_CLR
#define RX_AFE_ANARXTESTr_SET BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_SET
#define RX_AFE_ANARXTESTr_GET BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_GET
#define RX_AFE_ANARXTESTr_SIGDET_MUX_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_SIGDET_MUX_SMf_GET
#define RX_AFE_ANARXTESTr_SIGDET_MUX_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_SIGDET_MUX_SMf_SET
#define RX_AFE_ANARXTESTr_TPCTRL_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_TPCTRL_SMf_GET
#define RX_AFE_ANARXTESTr_TPCTRL_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_TPCTRL_SMf_SET
#define RX_AFE_ANARXTESTr_TESTMUXSELECT_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_TESTMUXSELECT_SMf_GET
#define RX_AFE_ANARXTESTr_TESTMUXSELECT_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr_TESTMUXSELECT_SMf_SET
#define READ_RX_AFE_ANARXTESTr BCMI_VIPER_XGXS_READ_RX_AFE_ANARXTESTr
#define WRITE_RX_AFE_ANARXTESTr BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXTESTr
#define MODIFY_RX_AFE_ANARXTESTr BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXTESTr
#define READLN_RX_AFE_ANARXTESTr BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXTESTr
#define WRITELN_RX_AFE_ANARXTESTr BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXTESTr
#define WRITEALL_RX_AFE_ANARXTESTr BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXTESTr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_ANARXTESTr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_ANARXCTL1G
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80b9
 * DESC:     Rx 1G Control register
 * RESETVAL: 0x80 (128)
 * ACCESS:   R/W
 * FIELDS:
 *     FREQ_SEL         0 = div/11 = div/2
 *     FREQ_SEL_FORCE   Force freq_sel
 *     SPD_RSTB_DIS_SM  
 *     PHASE_SEL_SM     
 *     RTBI_FLIP        
 *     RTBI_CKFLIP      
 *     CSTRETCH         Enable rxck0_1g clock/data phase alignment
 *     CGBAD_EN         Set bit 9 of symbol when bad symbol is detected
 *     PRBS_EN          Enable prbs monitor
 *     EMON_EN          Enable |E| monitor
 *     CGBAD_TST        Define error code as 10'h3FE for prbs monitor
 *     RXD_DEC_SEL      Select 8B/10B output for prbs monitor
 *     PRBS_CLR_DIS     Disable prbs clear-on-read
 *     STAMUXREGDIS     Disable registering of status mux
 *     PKT_COUNT_EN     Enable Packet Counter
 *     FPAT_MD          Fixed Pattern Mode enable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr (0x000080b9 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_ANARXCTL1G.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_s {
	uint32_t v[1];
	uint32_t rx_afe_anarxctl1g[1];
	uint32_t _rx_afe_anarxctl1g;
} BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_t;

#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CLR(r) (r).rx_afe_anarxctl1g[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_SET(r,d) (r).rx_afe_anarxctl1g[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_GET(r) (r).rx_afe_anarxctl1g[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FPAT_MDf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FPAT_MDf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PKT_COUNT_ENf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PKT_COUNT_ENf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_STAMUXREGDISf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_STAMUXREGDISf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PRBS_CLR_DISf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PRBS_CLR_DISf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RXD_DEC_SELf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RXD_DEC_SELf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CGBAD_TSTf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CGBAD_TSTf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_EMON_ENf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_EMON_ENf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PRBS_ENf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PRBS_ENf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CGBAD_ENf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CGBAD_ENf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CSTRETCHf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CSTRETCHf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RTBI_CKFLIPf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RTBI_CKFLIPf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RTBI_FLIPf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RTBI_FLIPf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PHASE_SEL_SMf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PHASE_SEL_SMf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_SPD_RSTB_DIS_SMf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_SPD_RSTB_DIS_SMf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FREQ_SEL_FORCEf_GET(r) ((((r).rx_afe_anarxctl1g[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FREQ_SEL_FORCEf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FREQ_SELf_GET(r) (((r).rx_afe_anarxctl1g[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FREQ_SELf_SET(r,f) (r).rx_afe_anarxctl1g[0]=(((r).rx_afe_anarxctl1g[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX_AFE_ANARXCTL1G.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_ANARXCTL1Gr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr,(_r._rx_afe_anarxctl1g))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXCTL1Gr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr,(_r._rx_afe_anarxctl1g)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXCTL1Gr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr,(_r._rx_afe_anarxctl1g))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXCTL1Gr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxctl1g))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXCTL1Gr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxctl1g))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXCTL1Gr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_anarxctl1g))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_ANARXCTL1Gr BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr
#define RX_AFE_ANARXCTL1Gr_SIZE BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_t RX_AFE_ANARXCTL1Gr_t;
#define RX_AFE_ANARXCTL1Gr_CLR BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CLR
#define RX_AFE_ANARXCTL1Gr_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_SET
#define RX_AFE_ANARXCTL1Gr_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_GET
#define RX_AFE_ANARXCTL1Gr_FPAT_MDf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FPAT_MDf_GET
#define RX_AFE_ANARXCTL1Gr_FPAT_MDf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FPAT_MDf_SET
#define RX_AFE_ANARXCTL1Gr_PKT_COUNT_ENf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PKT_COUNT_ENf_GET
#define RX_AFE_ANARXCTL1Gr_PKT_COUNT_ENf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PKT_COUNT_ENf_SET
#define RX_AFE_ANARXCTL1Gr_STAMUXREGDISf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_STAMUXREGDISf_GET
#define RX_AFE_ANARXCTL1Gr_STAMUXREGDISf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_STAMUXREGDISf_SET
#define RX_AFE_ANARXCTL1Gr_PRBS_CLR_DISf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PRBS_CLR_DISf_GET
#define RX_AFE_ANARXCTL1Gr_PRBS_CLR_DISf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PRBS_CLR_DISf_SET
#define RX_AFE_ANARXCTL1Gr_RXD_DEC_SELf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RXD_DEC_SELf_GET
#define RX_AFE_ANARXCTL1Gr_RXD_DEC_SELf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RXD_DEC_SELf_SET
#define RX_AFE_ANARXCTL1Gr_CGBAD_TSTf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CGBAD_TSTf_GET
#define RX_AFE_ANARXCTL1Gr_CGBAD_TSTf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CGBAD_TSTf_SET
#define RX_AFE_ANARXCTL1Gr_EMON_ENf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_EMON_ENf_GET
#define RX_AFE_ANARXCTL1Gr_EMON_ENf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_EMON_ENf_SET
#define RX_AFE_ANARXCTL1Gr_PRBS_ENf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PRBS_ENf_GET
#define RX_AFE_ANARXCTL1Gr_PRBS_ENf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PRBS_ENf_SET
#define RX_AFE_ANARXCTL1Gr_CGBAD_ENf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CGBAD_ENf_GET
#define RX_AFE_ANARXCTL1Gr_CGBAD_ENf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CGBAD_ENf_SET
#define RX_AFE_ANARXCTL1Gr_CSTRETCHf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CSTRETCHf_GET
#define RX_AFE_ANARXCTL1Gr_CSTRETCHf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_CSTRETCHf_SET
#define RX_AFE_ANARXCTL1Gr_RTBI_CKFLIPf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RTBI_CKFLIPf_GET
#define RX_AFE_ANARXCTL1Gr_RTBI_CKFLIPf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RTBI_CKFLIPf_SET
#define RX_AFE_ANARXCTL1Gr_RTBI_FLIPf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RTBI_FLIPf_GET
#define RX_AFE_ANARXCTL1Gr_RTBI_FLIPf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_RTBI_FLIPf_SET
#define RX_AFE_ANARXCTL1Gr_PHASE_SEL_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PHASE_SEL_SMf_GET
#define RX_AFE_ANARXCTL1Gr_PHASE_SEL_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_PHASE_SEL_SMf_SET
#define RX_AFE_ANARXCTL1Gr_SPD_RSTB_DIS_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_SPD_RSTB_DIS_SMf_GET
#define RX_AFE_ANARXCTL1Gr_SPD_RSTB_DIS_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_SPD_RSTB_DIS_SMf_SET
#define RX_AFE_ANARXCTL1Gr_FREQ_SEL_FORCEf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FREQ_SEL_FORCEf_GET
#define RX_AFE_ANARXCTL1Gr_FREQ_SEL_FORCEf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FREQ_SEL_FORCEf_SET
#define RX_AFE_ANARXCTL1Gr_FREQ_SELf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FREQ_SELf_GET
#define RX_AFE_ANARXCTL1Gr_FREQ_SELf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr_FREQ_SELf_SET
#define READ_RX_AFE_ANARXCTL1Gr BCMI_VIPER_XGXS_READ_RX_AFE_ANARXCTL1Gr
#define WRITE_RX_AFE_ANARXCTL1Gr BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXCTL1Gr
#define MODIFY_RX_AFE_ANARXCTL1Gr BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXCTL1Gr
#define READLN_RX_AFE_ANARXCTL1Gr BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXCTL1Gr
#define WRITELN_RX_AFE_ANARXCTL1Gr BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXCTL1Gr
#define WRITEALL_RX_AFE_ANARXCTL1Gr BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXCTL1Gr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_ANARXCTL1Gr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_ANARXCTLPCI
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80ba
 * DESC:     Rx PCI Control register
 * RESETVAL: 0x10 (16)
 * ACCESS:   R/W
 * FIELDS:
 *     INTEG_MODE_SM    Timing recovery integration mode
 *     RX_POLARITY_R    Rx Polarity
 *     RX_POLARITY_FORCE_SM Forces rx_polarity_r
 *     LINK_EN_R        Turns off byte sync state machine
 *     LINK_EN_FORCE_SM Forces link_en_SM
 *     COMMA_ADJ_EN_R   Enables external comma adjust enable
 *     COMMA_ADJ_EN_FORCE_R_SM Forces comma_adj_en_ext_SM
 *     COMMA_ADJ_EN_FORCE_SYNC_SM Forces comma_adj_en to be derived from comma_adj_en_sync
 *     COMMA_ADJ_EN_FORCE_EXT_SM Forces comma_adj_en to be derived from pins
 *     SYNC_STATUS_FORCE_R Forces on sync_status
 *     SYNC_STATUS_FORCE_R_SM Forces sync_status_r
 *     SYNC_STATUS_FORCE_SYNC_SM Forces sync_status_sync
 *     COMMA_MASK_R     Masks /K/ (for 802.3ae)
 *     COMMA_MASK_FORCE_R Forces comma_mask_r
 *     COMMA_ADJ_SYNC_SEL Selects between comma_adj_en and sync_status for comma_adj_en_SM
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr (0x000080ba | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_ANARXCTLPCI.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_s {
	uint32_t v[1];
	uint32_t rx_afe_anarxctlpci[1];
	uint32_t _rx_afe_anarxctlpci;
} BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_t;

#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_CLR(r) (r).rx_afe_anarxctlpci[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SET(r,d) (r).rx_afe_anarxctlpci[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_GET(r) (r).rx_afe_anarxctlpci[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_SYNC_SELf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_SYNC_SELf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_MASK_FORCE_Rf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_MASK_FORCE_Rf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_MASK_Rf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_MASK_Rf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_SYNC_SMf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_SYNC_SMf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_R_SMf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_R_SMf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_Rf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_Rf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_EXT_SMf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_EXT_SMf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_SYNC_SMf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_SYNC_SMf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_R_SMf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_R_SMf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_Rf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_Rf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_LINK_EN_FORCE_SMf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_LINK_EN_FORCE_SMf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_LINK_EN_Rf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_LINK_EN_Rf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_RX_POLARITY_FORCE_SMf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_RX_POLARITY_FORCE_SMf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_RX_POLARITY_Rf_GET(r) ((((r).rx_afe_anarxctlpci[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_RX_POLARITY_Rf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_INTEG_MODE_SMf_GET(r) (((r).rx_afe_anarxctlpci[0]) & 0x3)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_INTEG_MODE_SMf_SET(r,f) (r).rx_afe_anarxctlpci[0]=(((r).rx_afe_anarxctlpci[0] & ~((uint32_t)0x3)) | (((uint32_t)f) & 0x3)) | (0x3 << 16)

/*
 * These macros can be used to access RX_AFE_ANARXCTLPCI.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_ANARXCTLPCIr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr,(_r._rx_afe_anarxctlpci))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXCTLPCIr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr,(_r._rx_afe_anarxctlpci)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXCTLPCIr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr,(_r._rx_afe_anarxctlpci))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXCTLPCIr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxctlpci))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXCTLPCIr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxctlpci))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXCTLPCIr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_anarxctlpci))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_ANARXCTLPCIr BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr
#define RX_AFE_ANARXCTLPCIr_SIZE BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_t RX_AFE_ANARXCTLPCIr_t;
#define RX_AFE_ANARXCTLPCIr_CLR BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_CLR
#define RX_AFE_ANARXCTLPCIr_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SET
#define RX_AFE_ANARXCTLPCIr_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_GET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_SYNC_SELf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_SYNC_SELf_GET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_SYNC_SELf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_SYNC_SELf_SET
#define RX_AFE_ANARXCTLPCIr_COMMA_MASK_FORCE_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_MASK_FORCE_Rf_GET
#define RX_AFE_ANARXCTLPCIr_COMMA_MASK_FORCE_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_MASK_FORCE_Rf_SET
#define RX_AFE_ANARXCTLPCIr_COMMA_MASK_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_MASK_Rf_GET
#define RX_AFE_ANARXCTLPCIr_COMMA_MASK_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_MASK_Rf_SET
#define RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_SYNC_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_SYNC_SMf_GET
#define RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_SYNC_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_SYNC_SMf_SET
#define RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_R_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_R_SMf_GET
#define RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_R_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_R_SMf_SET
#define RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_Rf_GET
#define RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_SYNC_STATUS_FORCE_Rf_SET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_EXT_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_EXT_SMf_GET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_EXT_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_EXT_SMf_SET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_SYNC_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_SYNC_SMf_GET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_SYNC_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_SYNC_SMf_SET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_R_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_R_SMf_GET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_R_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_FORCE_R_SMf_SET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_Rf_GET
#define RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_COMMA_ADJ_EN_Rf_SET
#define RX_AFE_ANARXCTLPCIr_LINK_EN_FORCE_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_LINK_EN_FORCE_SMf_GET
#define RX_AFE_ANARXCTLPCIr_LINK_EN_FORCE_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_LINK_EN_FORCE_SMf_SET
#define RX_AFE_ANARXCTLPCIr_LINK_EN_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_LINK_EN_Rf_GET
#define RX_AFE_ANARXCTLPCIr_LINK_EN_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_LINK_EN_Rf_SET
#define RX_AFE_ANARXCTLPCIr_RX_POLARITY_FORCE_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_RX_POLARITY_FORCE_SMf_GET
#define RX_AFE_ANARXCTLPCIr_RX_POLARITY_FORCE_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_RX_POLARITY_FORCE_SMf_SET
#define RX_AFE_ANARXCTLPCIr_RX_POLARITY_Rf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_RX_POLARITY_Rf_GET
#define RX_AFE_ANARXCTLPCIr_RX_POLARITY_Rf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_RX_POLARITY_Rf_SET
#define RX_AFE_ANARXCTLPCIr_INTEG_MODE_SMf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_INTEG_MODE_SMf_GET
#define RX_AFE_ANARXCTLPCIr_INTEG_MODE_SMf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr_INTEG_MODE_SMf_SET
#define READ_RX_AFE_ANARXCTLPCIr BCMI_VIPER_XGXS_READ_RX_AFE_ANARXCTLPCIr
#define WRITE_RX_AFE_ANARXCTLPCIr BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXCTLPCIr
#define MODIFY_RX_AFE_ANARXCTLPCIr BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXCTLPCIr
#define READLN_RX_AFE_ANARXCTLPCIr BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXCTLPCIr
#define WRITELN_RX_AFE_ANARXCTLPCIr BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXCTLPCIr
#define WRITEALL_RX_AFE_ANARXCTLPCIr BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXCTLPCIr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_ANARXCTLPCIr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_ANARXASTS
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80bb
 * DESC:     Rx analog status register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     GENSTAT          
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr (0x000080bb | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_ANARXASTS.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_s {
	uint32_t v[1];
	uint32_t rx_afe_anarxasts[1];
	uint32_t _rx_afe_anarxasts;
} BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_t;

#define BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_CLR(r) (r).rx_afe_anarxasts[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_SET(r,d) (r).rx_afe_anarxasts[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_GET(r) (r).rx_afe_anarxasts[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_GENSTATf_GET(r) (((r).rx_afe_anarxasts[0]) & 0xffff)
#define BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_GENSTATf_SET(r,f) (r).rx_afe_anarxasts[0]=(((r).rx_afe_anarxasts[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access RX_AFE_ANARXASTS.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_ANARXASTSr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr,(_r._rx_afe_anarxasts))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXASTSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr,(_r._rx_afe_anarxasts)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXASTSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr,(_r._rx_afe_anarxasts))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXASTSr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxasts))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXASTSr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_anarxasts))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXASTSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_anarxasts))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_ANARXASTSr BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr
#define RX_AFE_ANARXASTSr_SIZE BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_t RX_AFE_ANARXASTSr_t;
#define RX_AFE_ANARXASTSr_CLR BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_CLR
#define RX_AFE_ANARXASTSr_SET BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_SET
#define RX_AFE_ANARXASTSr_GET BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_GET
#define RX_AFE_ANARXASTSr_GENSTATf_GET BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_GENSTATf_GET
#define RX_AFE_ANARXASTSr_GENSTATf_SET BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr_GENSTATf_SET
#define READ_RX_AFE_ANARXASTSr BCMI_VIPER_XGXS_READ_RX_AFE_ANARXASTSr
#define WRITE_RX_AFE_ANARXASTSr BCMI_VIPER_XGXS_WRITE_RX_AFE_ANARXASTSr
#define MODIFY_RX_AFE_ANARXASTSr BCMI_VIPER_XGXS_MODIFY_RX_AFE_ANARXASTSr
#define READLN_RX_AFE_ANARXASTSr BCMI_VIPER_XGXS_READLN_RX_AFE_ANARXASTSr
#define WRITELN_RX_AFE_ANARXASTSr BCMI_VIPER_XGXS_WRITELN_RX_AFE_ANARXASTSr
#define WRITEALL_RX_AFE_ANARXASTSr BCMI_VIPER_XGXS_WRITEALL_RX_AFE_ANARXASTSr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_ANARXASTSr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_CTL5
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80bc
 * DESC:     AFE Control Register 95:80
 * RESETVAL: 0x2c0e (11278)
 * ACCESS:   R/W
 * FIELDS:
 *     SIGDET_THRESHOLD Use 1001 for PCIE/SATA/RXAUI mode and 0100 for USB3 mode
 *     SEL_CLK          Selects the test clock to the output:00  4T Clock01  8T Clock10  10T Clock11  20T Clock
 *     EYEM_REFADJUST   Reference for the eye monitor+/-260mV 32 steps @16mV (Refer to Eye Monitoring Section)
 *     EYEMONITORREF_PD Eye monitor power down0: Enabled1: Disabled (Default)
 *     EYEMONITOR_REF_ZERO Eye monitor reference forced to 0V differentialCommon mode at vdd/2
 *     PI_MAIN_ENABLE   Enable for the phase interpolator used in the main path clocks.0: Disabled1: Enabled (Default)
 *     PI_EYEM_ENABLE   Enable for the phase interpolator used in the eye monitoring clock only0: Disabled (Default)1: Enabled
 *     SIGDET_USB_EN    Signal detect in the USB mode (output filter)1: USB3.0 and XFI Mode0: PCI-GEN mode (Default)
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r (0x000080bc | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_CTL5.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_CTL5r_s {
	uint32_t v[1];
	uint32_t rx_afe_ctl5[1];
	uint32_t _rx_afe_ctl5;
} BCMI_VIPER_XGXS_RX_AFE_CTL5r_t;

#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_CLR(r) (r).rx_afe_ctl5[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_SET(r,d) (r).rx_afe_ctl5[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_GET(r) (r).rx_afe_ctl5[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIGDET_USB_ENf_GET(r) ((((r).rx_afe_ctl5[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIGDET_USB_ENf_SET(r,f) (r).rx_afe_ctl5[0]=(((r).rx_afe_ctl5[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_PI_EYEM_ENABLEf_GET(r) ((((r).rx_afe_ctl5[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_PI_EYEM_ENABLEf_SET(r,f) (r).rx_afe_ctl5[0]=(((r).rx_afe_ctl5[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_PI_MAIN_ENABLEf_GET(r) ((((r).rx_afe_ctl5[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_PI_MAIN_ENABLEf_SET(r,f) (r).rx_afe_ctl5[0]=(((r).rx_afe_ctl5[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEMONITOR_REF_ZEROf_GET(r) ((((r).rx_afe_ctl5[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEMONITOR_REF_ZEROf_SET(r,f) (r).rx_afe_ctl5[0]=(((r).rx_afe_ctl5[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEMONITORREF_PDf_GET(r) ((((r).rx_afe_ctl5[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEMONITORREF_PDf_SET(r,f) (r).rx_afe_ctl5[0]=(((r).rx_afe_ctl5[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEM_REFADJUSTf_GET(r) ((((r).rx_afe_ctl5[0]) >> 6) & 0x1f)
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEM_REFADJUSTf_SET(r,f) (r).rx_afe_ctl5[0]=(((r).rx_afe_ctl5[0] & ~((uint32_t)0x1f << 6)) | ((((uint32_t)f) & 0x1f) << 6)) | (31 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_SEL_CLKf_GET(r) ((((r).rx_afe_ctl5[0]) >> 4) & 0x3)
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_SEL_CLKf_SET(r,f) (r).rx_afe_ctl5[0]=(((r).rx_afe_ctl5[0] & ~((uint32_t)0x3 << 4)) | ((((uint32_t)f) & 0x3) << 4)) | (3 << (16 + 4))
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIGDET_THRESHOLDf_GET(r) (((r).rx_afe_ctl5[0]) & 0xf)
#define BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIGDET_THRESHOLDf_SET(r,f) (r).rx_afe_ctl5[0]=(((r).rx_afe_ctl5[0] & ~((uint32_t)0xf)) | (((uint32_t)f) & 0xf)) | (0xf << 16)

/*
 * These macros can be used to access RX_AFE_CTL5.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_CTL5r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL5r,(_r._rx_afe_ctl5))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL5r,(_r._rx_afe_ctl5)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL5r,(_r._rx_afe_ctl5))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_CTL5r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl5))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL5r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl5))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_ctl5))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_CTL5r BCMI_VIPER_XGXS_RX_AFE_CTL5r
#define RX_AFE_CTL5r_SIZE BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_CTL5r_t RX_AFE_CTL5r_t;
#define RX_AFE_CTL5r_CLR BCMI_VIPER_XGXS_RX_AFE_CTL5r_CLR
#define RX_AFE_CTL5r_SET BCMI_VIPER_XGXS_RX_AFE_CTL5r_SET
#define RX_AFE_CTL5r_GET BCMI_VIPER_XGXS_RX_AFE_CTL5r_GET
#define RX_AFE_CTL5r_SIGDET_USB_ENf_GET BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIGDET_USB_ENf_GET
#define RX_AFE_CTL5r_SIGDET_USB_ENf_SET BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIGDET_USB_ENf_SET
#define RX_AFE_CTL5r_PI_EYEM_ENABLEf_GET BCMI_VIPER_XGXS_RX_AFE_CTL5r_PI_EYEM_ENABLEf_GET
#define RX_AFE_CTL5r_PI_EYEM_ENABLEf_SET BCMI_VIPER_XGXS_RX_AFE_CTL5r_PI_EYEM_ENABLEf_SET
#define RX_AFE_CTL5r_PI_MAIN_ENABLEf_GET BCMI_VIPER_XGXS_RX_AFE_CTL5r_PI_MAIN_ENABLEf_GET
#define RX_AFE_CTL5r_PI_MAIN_ENABLEf_SET BCMI_VIPER_XGXS_RX_AFE_CTL5r_PI_MAIN_ENABLEf_SET
#define RX_AFE_CTL5r_EYEMONITOR_REF_ZEROf_GET BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEMONITOR_REF_ZEROf_GET
#define RX_AFE_CTL5r_EYEMONITOR_REF_ZEROf_SET BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEMONITOR_REF_ZEROf_SET
#define RX_AFE_CTL5r_EYEMONITORREF_PDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEMONITORREF_PDf_GET
#define RX_AFE_CTL5r_EYEMONITORREF_PDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEMONITORREF_PDf_SET
#define RX_AFE_CTL5r_EYEM_REFADJUSTf_GET BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEM_REFADJUSTf_GET
#define RX_AFE_CTL5r_EYEM_REFADJUSTf_SET BCMI_VIPER_XGXS_RX_AFE_CTL5r_EYEM_REFADJUSTf_SET
#define RX_AFE_CTL5r_SEL_CLKf_GET BCMI_VIPER_XGXS_RX_AFE_CTL5r_SEL_CLKf_GET
#define RX_AFE_CTL5r_SEL_CLKf_SET BCMI_VIPER_XGXS_RX_AFE_CTL5r_SEL_CLKf_SET
#define RX_AFE_CTL5r_SIGDET_THRESHOLDf_GET BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIGDET_THRESHOLDf_GET
#define RX_AFE_CTL5r_SIGDET_THRESHOLDf_SET BCMI_VIPER_XGXS_RX_AFE_CTL5r_SIGDET_THRESHOLDf_SET
#define READ_RX_AFE_CTL5r BCMI_VIPER_XGXS_READ_RX_AFE_CTL5r
#define WRITE_RX_AFE_CTL5r BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL5r
#define MODIFY_RX_AFE_CTL5r BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL5r
#define READLN_RX_AFE_CTL5r BCMI_VIPER_XGXS_READLN_RX_AFE_CTL5r
#define WRITELN_RX_AFE_CTL5r BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL5r
#define WRITEALL_RX_AFE_CTL5r BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL5r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_CTL5r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX_AFE_CTL6
 * BLOCKS:   RX_AFE
 * REGADDR:  0x80bd
 * DESC:     AFE Control Register 111:96
 * RESETVAL: 0x44 (68)
 * ACCESS:   R/W
 * FIELDS:
 *     RESERVED_96      
 *     MAIN_PI_PWD_LVL2PI Power Down the Cross Coupled PI0: Cross coupled working (Default)1: Cross coupled power down
 *     MAIN_PI_BW_SEL   Control for the PI BW1: High BW (Default)0: Low BW
 *     PI_LOWVDD_ENB    Disables low vdd operation on the Rx (Affecting PI, SigDetect)0: low vdd configuration is enabled (Default)1: high vdd configuration is enabled
 *     RESERVED_100     
 *     EYE_MONITOR_PI_PWD_LVL2PI Power Down the Cross Coupled PI0: Cross coupled working (Default)1: Cross coupled power down
 *     EYE_MONITOR_PI_BW_SEL Control for the PI BW1: High BW (Default)0: Low BW
 *     RESERVED_105_103 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r (0x000080bd | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_SIZE 4

/*
 * This structure should be used to declare and program RX_AFE_CTL6.
 *
 */
typedef union BCMI_VIPER_XGXS_RX_AFE_CTL6r_s {
	uint32_t v[1];
	uint32_t rx_afe_ctl6[1];
	uint32_t _rx_afe_ctl6;
} BCMI_VIPER_XGXS_RX_AFE_CTL6r_t;

#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_CLR(r) (r).rx_afe_ctl6[0] = 0
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_SET(r,d) (r).rx_afe_ctl6[0] = d
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_GET(r) (r).rx_afe_ctl6[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_105_103f_GET(r) ((((r).rx_afe_ctl6[0]) >> 7) & 0x7)
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_105_103f_SET(r,f) (r).rx_afe_ctl6[0]=(((r).rx_afe_ctl6[0] & ~((uint32_t)0x7 << 7)) | ((((uint32_t)f) & 0x7) << 7)) | (7 << (16 + 7))
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_EYE_MONITOR_PI_BW_SELf_GET(r) ((((r).rx_afe_ctl6[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_EYE_MONITOR_PI_BW_SELf_SET(r,f) (r).rx_afe_ctl6[0]=(((r).rx_afe_ctl6[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_EYE_MONITOR_PI_PWD_LVL2PIf_GET(r) ((((r).rx_afe_ctl6[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_EYE_MONITOR_PI_PWD_LVL2PIf_SET(r,f) (r).rx_afe_ctl6[0]=(((r).rx_afe_ctl6[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_100f_GET(r) ((((r).rx_afe_ctl6[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_100f_SET(r,f) (r).rx_afe_ctl6[0]=(((r).rx_afe_ctl6[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_PI_LOWVDD_ENBf_GET(r) ((((r).rx_afe_ctl6[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_PI_LOWVDD_ENBf_SET(r,f) (r).rx_afe_ctl6[0]=(((r).rx_afe_ctl6[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_MAIN_PI_BW_SELf_GET(r) ((((r).rx_afe_ctl6[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_MAIN_PI_BW_SELf_SET(r,f) (r).rx_afe_ctl6[0]=(((r).rx_afe_ctl6[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_MAIN_PI_PWD_LVL2PIf_GET(r) ((((r).rx_afe_ctl6[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_MAIN_PI_PWD_LVL2PIf_SET(r,f) (r).rx_afe_ctl6[0]=(((r).rx_afe_ctl6[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_96f_GET(r) (((r).rx_afe_ctl6[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_96f_SET(r,f) (r).rx_afe_ctl6[0]=(((r).rx_afe_ctl6[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX_AFE_CTL6.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX_AFE_CTL6r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL6r,(_r._rx_afe_ctl6))
#define BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL6r,(_r._rx_afe_ctl6)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL6r,(_r._rx_afe_ctl6))
#define BCMI_VIPER_XGXS_READLN_RX_AFE_CTL6r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl6))
#define BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL6r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx_afe_ctl6))
#define BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX_AFE_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx_afe_ctl6))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX_AFE_CTL6r BCMI_VIPER_XGXS_RX_AFE_CTL6r
#define RX_AFE_CTL6r_SIZE BCMI_VIPER_XGXS_RX_AFE_CTL6r_SIZE
typedef BCMI_VIPER_XGXS_RX_AFE_CTL6r_t RX_AFE_CTL6r_t;
#define RX_AFE_CTL6r_CLR BCMI_VIPER_XGXS_RX_AFE_CTL6r_CLR
#define RX_AFE_CTL6r_SET BCMI_VIPER_XGXS_RX_AFE_CTL6r_SET
#define RX_AFE_CTL6r_GET BCMI_VIPER_XGXS_RX_AFE_CTL6r_GET
#define RX_AFE_CTL6r_RESERVED_105_103f_GET BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_105_103f_GET
#define RX_AFE_CTL6r_RESERVED_105_103f_SET BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_105_103f_SET
#define RX_AFE_CTL6r_EYE_MONITOR_PI_BW_SELf_GET BCMI_VIPER_XGXS_RX_AFE_CTL6r_EYE_MONITOR_PI_BW_SELf_GET
#define RX_AFE_CTL6r_EYE_MONITOR_PI_BW_SELf_SET BCMI_VIPER_XGXS_RX_AFE_CTL6r_EYE_MONITOR_PI_BW_SELf_SET
#define RX_AFE_CTL6r_EYE_MONITOR_PI_PWD_LVL2PIf_GET BCMI_VIPER_XGXS_RX_AFE_CTL6r_EYE_MONITOR_PI_PWD_LVL2PIf_GET
#define RX_AFE_CTL6r_EYE_MONITOR_PI_PWD_LVL2PIf_SET BCMI_VIPER_XGXS_RX_AFE_CTL6r_EYE_MONITOR_PI_PWD_LVL2PIf_SET
#define RX_AFE_CTL6r_RESERVED_100f_GET BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_100f_GET
#define RX_AFE_CTL6r_RESERVED_100f_SET BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_100f_SET
#define RX_AFE_CTL6r_PI_LOWVDD_ENBf_GET BCMI_VIPER_XGXS_RX_AFE_CTL6r_PI_LOWVDD_ENBf_GET
#define RX_AFE_CTL6r_PI_LOWVDD_ENBf_SET BCMI_VIPER_XGXS_RX_AFE_CTL6r_PI_LOWVDD_ENBf_SET
#define RX_AFE_CTL6r_MAIN_PI_BW_SELf_GET BCMI_VIPER_XGXS_RX_AFE_CTL6r_MAIN_PI_BW_SELf_GET
#define RX_AFE_CTL6r_MAIN_PI_BW_SELf_SET BCMI_VIPER_XGXS_RX_AFE_CTL6r_MAIN_PI_BW_SELf_SET
#define RX_AFE_CTL6r_MAIN_PI_PWD_LVL2PIf_GET BCMI_VIPER_XGXS_RX_AFE_CTL6r_MAIN_PI_PWD_LVL2PIf_GET
#define RX_AFE_CTL6r_MAIN_PI_PWD_LVL2PIf_SET BCMI_VIPER_XGXS_RX_AFE_CTL6r_MAIN_PI_PWD_LVL2PIf_SET
#define RX_AFE_CTL6r_RESERVED_96f_GET BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_96f_GET
#define RX_AFE_CTL6r_RESERVED_96f_SET BCMI_VIPER_XGXS_RX_AFE_CTL6r_RESERVED_96f_SET
#define READ_RX_AFE_CTL6r BCMI_VIPER_XGXS_READ_RX_AFE_CTL6r
#define WRITE_RX_AFE_CTL6r BCMI_VIPER_XGXS_WRITE_RX_AFE_CTL6r
#define MODIFY_RX_AFE_CTL6r BCMI_VIPER_XGXS_MODIFY_RX_AFE_CTL6r
#define READLN_RX_AFE_CTL6r BCMI_VIPER_XGXS_READLN_RX_AFE_CTL6r
#define WRITELN_RX_AFE_CTL6r BCMI_VIPER_XGXS_WRITELN_RX_AFE_CTL6r
#define WRITEALL_RX_AFE_CTL6r BCMI_VIPER_XGXS_WRITEALL_RX_AFE_CTL6r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX_AFE_CTL6r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER: UNICOREMODE10G 
 * BLOCKS:   BLK2
 * REGADDR:  0x8104
 * DESC:     Test mode lane select register
 * RESETVAL: 0x91 (145)
 * ACCESS:   R/W
 * FIELDS:
 *     UNICOREMODE10GCX4    Configures the XGXS mode type/enabled features. It is active when the mode10G[3:0] = 4'hC(i.e. 0x8000[11:8]) AND acutal_speed is set to 10GBASE-CX4 or 10GBASE-KX4 CASE(unicoreMode10gCX4)0x0 = XGXS 0x1 = XGXG_nCC 0x4 = IndLane_OS8 0x5 = IndLane_OS5 0x6 = IndLane_OS4 0x7 = PCI 0x8 = XGXS_nLQ 0x9 = XGXS_nLQnCC 0xa = PBypass 0xb = PBypass_nDSK 0xc = ComboCoreMode 0xf = Clocks_off reset value is XGXS
 *     UNICOREMODE10GHIG    Configures the XGXS mode type/enabled features.  *     UNICOREMODE10GCX4    Configures the XGXS mode type/enabled features. It is active when the mode10G[3:0] = 4'hC(i.e. 0x8000[11:8]) AND acutal_speed is set to an aggreagate X4 mode other than 10GBASE-CX4/KX4 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_UNICOREMODE10Gr (0x00008104 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_UNICOREMODE10Gr_SIZE 4


/*
 * This structure should be used to declare and program UNICOREMODE10G.
 *
 */
typedef union BCMI_VIPER_XGXS_UNICOREMODE10Gr_s {
	uint32_t v[1];
	uint32_t unicoremode10g[1];
	uint32_t _unicoremode10g;
} BCMI_VIPER_XGXS_UNICOREMODE10Gr_t;

#define BCMI_VIPER_XGXS_UNICOREMODE10G(r) (r).unicoremode10g[0] = 0
#define BCMI_VIPER_XGXS_UNICOREMODE10Gr_SET(r,d) (r).unicoremode10g[0] = d
#define BCMI_VIPER_XGXS_UNICOREMODE10Gr_GET(r) (r).unicoremode10g[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_UNICOREMODE10Gr_UNICOREMODE10GHIGf_GET(r) ((((r).unicoremode10g[0]) >> 4) & 0xf)
#define BCMI_VIPER_XGXS_UNICOREMODE10Gr_UNICOREMODE10GHIGf_SET(r,f) (r).unicoremode10g[0]=(((r).unicoremode10g[0] & ~((uint32_t)0xf << 4)) | (((uint32_t)f) & 0xf) << 4) | (0xf << (16 + 4))
#define BCMI_VIPER_XGXS_UNICOREMODE10Gr_UNICOREMODE10GCX4f_GET(r) ((((r).unicoremode10g[0]) & 0xf)
#define BCMI_VIPER_XGXS_UNICOREMODE10Gr_UNICOREMODE10GCX4f_SET(r,f) (r).unicoremode10g[0]=(((r).unicoremode10g[0] & ~((uint32_t)0xf )) | (((uint32_t)f) & 0xf)) | (0xf << 16)


/*
 * These macros can be used to access UNICOREMODE10G.
 *
 */
#define BCMI_VIPER_XGXS_READ_UNICOREMODE10Gr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_UNICOREMODE10Gr,(_r._unicoremode10g))
#define BCMI_VIPER_XGXS_WRITE_UNICOREMODE10Gr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_UNICOREMODE10Gr,(_r._unicoremode10g)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_UNICOREMODE10Gr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_UNICOREMODE10Gr,(_r._unicoremode10g))
#define BCMI_VIPER_XGXS_READLN_UNICOREMODE10Gr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_UNICOREMODE10Gr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._unicoremode10g))
#define BCMI_VIPER_XGXS_WRITELN_UNICOREMODE10Gr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_UNICOREMODE10Gr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._unicoremode10g))
#define BCMI_VIPER_XGXS_WRITEALL_UNICOREMODE10Gr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_UNICOREMODE10Gr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._unicoremode10g))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define UNICOREMODE10Gr BCMI_VIPER_XGXS_UNICOREMODE10Gr
#define UNICOREMODE10Gr_SIZE BCMI_VIPER_XGXS_UNICOREMODE10Gr_SIZE
typedef BCMI_VIPER_XGXS_UNICOREMODE10Gr_t UNICOREMODE10Gr_t;
#define UNICOREMODE10Gr_CLR BCMI_VIPER_XGXS_UNICOREMODE10Gr_CLR
#define UNICOREMODE10Gr_SET BCMI_VIPER_XGXS_UNICOREMODE10Gr_SET
#define UNICOREMODE10Gr_GET BCMI_VIPER_XGXS_UNICOREMODE10Gr_GET
#define UNICOREMODE10Gr_UNICOREMODE10GHIGf_GET BCMI_VIPER_XGXS_UNICOREMODE10Gr_UNICOREMODE10GHIGf_GET
#define UNICOREMODE10Gr_UNICOREMODE10GHIGf_SET BCMI_VIPER_XGXS_UNICOREMODE10Gr_UNICOREMODE10GHIGf_SET
#define UNICOREMODE10Gr_UNICOREMODE10GCX4f_GET BCMI_VIPER_XGXS_UNICOREMODE10Gr_UNICOREMODE10GCX4f_GET
#define UNICOREMODE10Gr_UNICOREMODE10GCX4f_SET BCMI_VIPER_XGXS_UNICOREMODE10Gr_UNICOREMODE10GCX4f_SET
#define READ_UNICOREMODE10Gr BCMI_VIPER_XGXS_READ_UNICOREMODE10Gr
#define WRITE_UNICOREMODE10Gr BCMI_VIPER_XGXS_WRITE_UNICOREMODE10Gr
#define MODIFY_UNICOREMODE10Gr BCMI_VIPER_XGXS_MODIFY_UNICOREMODE10Gr
#define READLN_UNICOREMODE10Gr BCMI_VIPER_XGXS_READLN_UNICOREMODE10Gr
#define WRITELN_UNICOREMODE10Gr BCMI_VIPER_XGXS_WRITELN_UNICOREMODE10Gr
#define WRITEALL_UNICOREMODE10Gr BCMI_VIPER_XGXS_WRITEALL_UNICOREMODE10Gr


#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_UNICOREMODE10Gr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TESTMODELANE
 * BLOCKS:   BLK2
 * REGADDR:  0x8106
 * DESC:     Test mode lane select register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TXFIFO_BYPASS    to enable txfifo_bypass, destination synchronous for txckstrap set to 1 in PentaCore
 *     INDCK_MODE_EN_RX_VAL override value indck_mode_en_rx
 *     INDCK_MODE_EN_RX_FORCE override indck_mode_en_rx
 *     EEE_GATEOUTCLK_EN Enable EEE clock gating of interface clocks
 *     EEE_GATECLK_EN   Enable EEE clock gating of clocks for internal logic
 *     EEE_FAST_TIMER_EN EEE state machines run at refclk
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TESTMODELANEr (0x00008106 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TESTMODELANEr_SIZE 4

/*
 * This structure should be used to declare and program TESTMODELANE.
 *
 */
typedef union BCMI_VIPER_XGXS_TESTMODELANEr_s {
	uint32_t v[1];
	uint32_t testmodelane[1];
	uint32_t _testmodelane;
} BCMI_VIPER_XGXS_TESTMODELANEr_t;

#define BCMI_VIPER_XGXS_TESTMODELANEr_CLR(r) (r).testmodelane[0] = 0
#define BCMI_VIPER_XGXS_TESTMODELANEr_SET(r,d) (r).testmodelane[0] = d
#define BCMI_VIPER_XGXS_TESTMODELANEr_GET(r) (r).testmodelane[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TESTMODELANEr_EEE_FAST_TIMER_ENf_GET(r) ((((r).testmodelane[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_TESTMODELANEr_EEE_FAST_TIMER_ENf_SET(r,f) (r).testmodelane[0]=(((r).testmodelane[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_TESTMODELANEr_EEE_GATECLK_ENf_GET(r) ((((r).testmodelane[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_TESTMODELANEr_EEE_GATECLK_ENf_SET(r,f) (r).testmodelane[0]=(((r).testmodelane[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_TESTMODELANEr_EEE_GATEOUTCLK_ENf_GET(r) ((((r).testmodelane[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_TESTMODELANEr_EEE_GATEOUTCLK_ENf_SET(r,f) (r).testmodelane[0]=(((r).testmodelane[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_TESTMODELANEr_INDCK_MODE_EN_RX_FORCEf_GET(r) ((((r).testmodelane[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_TESTMODELANEr_INDCK_MODE_EN_RX_FORCEf_SET(r,f) (r).testmodelane[0]=(((r).testmodelane[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_TESTMODELANEr_INDCK_MODE_EN_RX_VALf_GET(r) ((((r).testmodelane[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_TESTMODELANEr_INDCK_MODE_EN_RX_VALf_SET(r,f) (r).testmodelane[0]=(((r).testmodelane[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_TESTMODELANEr_TXFIFO_BYPASSf_GET(r) ((((r).testmodelane[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_TESTMODELANEr_TXFIFO_BYPASSf_SET(r,f) (r).testmodelane[0]=(((r).testmodelane[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))

/*
 * These macros can be used to access TESTMODELANE.
 *
 */
#define BCMI_VIPER_XGXS_READ_TESTMODELANEr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TESTMODELANEr,(_r._testmodelane))
#define BCMI_VIPER_XGXS_WRITE_TESTMODELANEr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODELANEr,(_r._testmodelane)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TESTMODELANEr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODELANEr,(_r._testmodelane))
#define BCMI_VIPER_XGXS_READLN_TESTMODELANEr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TESTMODELANEr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._testmodelane))
#define BCMI_VIPER_XGXS_WRITELN_TESTMODELANEr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODELANEr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._testmodelane))
#define BCMI_VIPER_XGXS_WRITEALL_TESTMODELANEr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODELANEr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._testmodelane))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TESTMODELANEr BCMI_VIPER_XGXS_TESTMODELANEr
#define TESTMODELANEr_SIZE BCMI_VIPER_XGXS_TESTMODELANEr_SIZE
typedef BCMI_VIPER_XGXS_TESTMODELANEr_t TESTMODELANEr_t;
#define TESTMODELANEr_CLR BCMI_VIPER_XGXS_TESTMODELANEr_CLR
#define TESTMODELANEr_SET BCMI_VIPER_XGXS_TESTMODELANEr_SET
#define TESTMODELANEr_GET BCMI_VIPER_XGXS_TESTMODELANEr_GET
#define TESTMODELANEr_EEE_FAST_TIMER_ENf_GET BCMI_VIPER_XGXS_TESTMODELANEr_EEE_FAST_TIMER_ENf_GET
#define TESTMODELANEr_EEE_FAST_TIMER_ENf_SET BCMI_VIPER_XGXS_TESTMODELANEr_EEE_FAST_TIMER_ENf_SET
#define TESTMODELANEr_EEE_GATECLK_ENf_GET BCMI_VIPER_XGXS_TESTMODELANEr_EEE_GATECLK_ENf_GET
#define TESTMODELANEr_EEE_GATECLK_ENf_SET BCMI_VIPER_XGXS_TESTMODELANEr_EEE_GATECLK_ENf_SET
#define TESTMODELANEr_EEE_GATEOUTCLK_ENf_GET BCMI_VIPER_XGXS_TESTMODELANEr_EEE_GATEOUTCLK_ENf_GET
#define TESTMODELANEr_EEE_GATEOUTCLK_ENf_SET BCMI_VIPER_XGXS_TESTMODELANEr_EEE_GATEOUTCLK_ENf_SET
#define TESTMODELANEr_INDCK_MODE_EN_RX_FORCEf_GET BCMI_VIPER_XGXS_TESTMODELANEr_INDCK_MODE_EN_RX_FORCEf_GET
#define TESTMODELANEr_INDCK_MODE_EN_RX_FORCEf_SET BCMI_VIPER_XGXS_TESTMODELANEr_INDCK_MODE_EN_RX_FORCEf_SET
#define TESTMODELANEr_INDCK_MODE_EN_RX_VALf_GET BCMI_VIPER_XGXS_TESTMODELANEr_INDCK_MODE_EN_RX_VALf_GET
#define TESTMODELANEr_INDCK_MODE_EN_RX_VALf_SET BCMI_VIPER_XGXS_TESTMODELANEr_INDCK_MODE_EN_RX_VALf_SET
#define TESTMODELANEr_TXFIFO_BYPASSf_GET BCMI_VIPER_XGXS_TESTMODELANEr_TXFIFO_BYPASSf_GET
#define TESTMODELANEr_TXFIFO_BYPASSf_SET BCMI_VIPER_XGXS_TESTMODELANEr_TXFIFO_BYPASSf_SET
#define READ_TESTMODELANEr BCMI_VIPER_XGXS_READ_TESTMODELANEr
#define WRITE_TESTMODELANEr BCMI_VIPER_XGXS_WRITE_TESTMODELANEr
#define MODIFY_TESTMODELANEr BCMI_VIPER_XGXS_MODIFY_TESTMODELANEr
#define READLN_TESTMODELANEr BCMI_VIPER_XGXS_READLN_TESTMODELANEr
#define WRITELN_TESTMODELANEr BCMI_VIPER_XGXS_WRITELN_TESTMODELANEr
#define WRITEALL_TESTMODELANEr BCMI_VIPER_XGXS_WRITEALL_TESTMODELANEr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TESTMODELANEr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TESTMODECOMBO
 * BLOCKS:   BLK2
 * REGADDR:  0x8107
 * DESC:     Test mode monitor control register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TEST_MONITOR_MODE1 Select test mode for tpout[11:0]see testability section in specification for details
 *     TEST_MONITOR_MODE2 Select test mode for tpout[23:12]see testability section in specification for details
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TESTMODECOMBOr (0x00008107 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TESTMODECOMBOr_SIZE 4

/*
 * This structure should be used to declare and program TESTMODECOMBO.
 *
 */
typedef union BCMI_VIPER_XGXS_TESTMODECOMBOr_s {
	uint32_t v[1];
	uint32_t testmodecombo[1];
	uint32_t _testmodecombo;
} BCMI_VIPER_XGXS_TESTMODECOMBOr_t;

#define BCMI_VIPER_XGXS_TESTMODECOMBOr_CLR(r) (r).testmodecombo[0] = 0
#define BCMI_VIPER_XGXS_TESTMODECOMBOr_SET(r,d) (r).testmodecombo[0] = d
#define BCMI_VIPER_XGXS_TESTMODECOMBOr_GET(r) (r).testmodecombo[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TESTMODECOMBOr_TEST_MONITOR_MODE2f_GET(r) ((((r).testmodecombo[0]) >> 6) & 0x3f)
#define BCMI_VIPER_XGXS_TESTMODECOMBOr_TEST_MONITOR_MODE2f_SET(r,f) (r).testmodecombo[0]=(((r).testmodecombo[0] & ~((uint32_t)0x3f << 6)) | ((((uint32_t)f) & 0x3f) << 6)) | (63 << (16 + 6))
#define BCMI_VIPER_XGXS_TESTMODECOMBOr_TEST_MONITOR_MODE1f_GET(r) (((r).testmodecombo[0]) & 0x3f)
#define BCMI_VIPER_XGXS_TESTMODECOMBOr_TEST_MONITOR_MODE1f_SET(r,f) (r).testmodecombo[0]=(((r).testmodecombo[0] & ~((uint32_t)0x3f)) | (((uint32_t)f) & 0x3f)) | (0x3f << 16)

/*
 * These macros can be used to access TESTMODECOMBO.
 *
 */
#define BCMI_VIPER_XGXS_READ_TESTMODECOMBOr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TESTMODECOMBOr,(_r._testmodecombo))
#define BCMI_VIPER_XGXS_WRITE_TESTMODECOMBOr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODECOMBOr,(_r._testmodecombo)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TESTMODECOMBOr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODECOMBOr,(_r._testmodecombo))
#define BCMI_VIPER_XGXS_READLN_TESTMODECOMBOr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TESTMODECOMBOr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._testmodecombo))
#define BCMI_VIPER_XGXS_WRITELN_TESTMODECOMBOr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODECOMBOr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._testmodecombo))
#define BCMI_VIPER_XGXS_WRITEALL_TESTMODECOMBOr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODECOMBOr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._testmodecombo))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TESTMODECOMBOr BCMI_VIPER_XGXS_TESTMODECOMBOr
#define TESTMODECOMBOr_SIZE BCMI_VIPER_XGXS_TESTMODECOMBOr_SIZE
typedef BCMI_VIPER_XGXS_TESTMODECOMBOr_t TESTMODECOMBOr_t;
#define TESTMODECOMBOr_CLR BCMI_VIPER_XGXS_TESTMODECOMBOr_CLR
#define TESTMODECOMBOr_SET BCMI_VIPER_XGXS_TESTMODECOMBOr_SET
#define TESTMODECOMBOr_GET BCMI_VIPER_XGXS_TESTMODECOMBOr_GET
#define TESTMODECOMBOr_TEST_MONITOR_MODE2f_GET BCMI_VIPER_XGXS_TESTMODECOMBOr_TEST_MONITOR_MODE2f_GET
#define TESTMODECOMBOr_TEST_MONITOR_MODE2f_SET BCMI_VIPER_XGXS_TESTMODECOMBOr_TEST_MONITOR_MODE2f_SET
#define TESTMODECOMBOr_TEST_MONITOR_MODE1f_GET BCMI_VIPER_XGXS_TESTMODECOMBOr_TEST_MONITOR_MODE1f_GET
#define TESTMODECOMBOr_TEST_MONITOR_MODE1f_SET BCMI_VIPER_XGXS_TESTMODECOMBOr_TEST_MONITOR_MODE1f_SET
#define READ_TESTMODECOMBOr BCMI_VIPER_XGXS_READ_TESTMODECOMBOr
#define WRITE_TESTMODECOMBOr BCMI_VIPER_XGXS_WRITE_TESTMODECOMBOr
#define MODIFY_TESTMODECOMBOr BCMI_VIPER_XGXS_MODIFY_TESTMODECOMBOr
#define READLN_TESTMODECOMBOr BCMI_VIPER_XGXS_READLN_TESTMODECOMBOr
#define WRITELN_TESTMODECOMBOr BCMI_VIPER_XGXS_WRITELN_TESTMODECOMBOr
#define WRITEALL_TESTMODECOMBOr BCMI_VIPER_XGXS_WRITEALL_TESTMODECOMBOr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TESTMODECOMBOr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TESTMODEMUX
 * BLOCKS:   BLK2
 * REGADDR:  0x8108
 * DESC:     Test mode mux control register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TMUX_EN          Enable internal test mux
 *     TMUX_SEL         control for top level testmux3'b000 = rxSeqDone,rx,tx active3'b001 = rx,tx active, lock,xg_rx_tmux3'b010 = xg_tx_tmux3'b011 = XGXS fe mux3'b100 = combo tpout13'b101 = combo tpout2
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TESTMODEMUXr (0x00008108 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TESTMODEMUXr_SIZE 4

/*
 * This structure should be used to declare and program TESTMODEMUX.
 *
 */
typedef union BCMI_VIPER_XGXS_TESTMODEMUXr_s {
	uint32_t v[1];
	uint32_t testmodemux[1];
	uint32_t _testmodemux;
} BCMI_VIPER_XGXS_TESTMODEMUXr_t;

#define BCMI_VIPER_XGXS_TESTMODEMUXr_CLR(r) (r).testmodemux[0] = 0
#define BCMI_VIPER_XGXS_TESTMODEMUXr_SET(r,d) (r).testmodemux[0] = d
#define BCMI_VIPER_XGXS_TESTMODEMUXr_GET(r) (r).testmodemux[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TESTMODEMUXr_TMUX_SELf_GET(r) ((((r).testmodemux[0]) >> 1) & 0x7)
#define BCMI_VIPER_XGXS_TESTMODEMUXr_TMUX_SELf_SET(r,f) (r).testmodemux[0]=(((r).testmodemux[0] & ~((uint32_t)0x7 << 1)) | ((((uint32_t)f) & 0x7) << 1)) | (7 << (16 + 1))
#define BCMI_VIPER_XGXS_TESTMODEMUXr_TMUX_ENf_GET(r) (((r).testmodemux[0]) & 0x1)
#define BCMI_VIPER_XGXS_TESTMODEMUXr_TMUX_ENf_SET(r,f) (r).testmodemux[0]=(((r).testmodemux[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access TESTMODEMUX.
 *
 */
#define BCMI_VIPER_XGXS_READ_TESTMODEMUXr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TESTMODEMUXr,(_r._testmodemux))
#define BCMI_VIPER_XGXS_WRITE_TESTMODEMUXr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODEMUXr,(_r._testmodemux)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TESTMODEMUXr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODEMUXr,(_r._testmodemux))
#define BCMI_VIPER_XGXS_READLN_TESTMODEMUXr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TESTMODEMUXr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._testmodemux))
#define BCMI_VIPER_XGXS_WRITELN_TESTMODEMUXr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODEMUXr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._testmodemux))
#define BCMI_VIPER_XGXS_WRITEALL_TESTMODEMUXr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TESTMODEMUXr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._testmodemux))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TESTMODEMUXr BCMI_VIPER_XGXS_TESTMODEMUXr
#define TESTMODEMUXr_SIZE BCMI_VIPER_XGXS_TESTMODEMUXr_SIZE
typedef BCMI_VIPER_XGXS_TESTMODEMUXr_t TESTMODEMUXr_t;
#define TESTMODEMUXr_CLR BCMI_VIPER_XGXS_TESTMODEMUXr_CLR
#define TESTMODEMUXr_SET BCMI_VIPER_XGXS_TESTMODEMUXr_SET
#define TESTMODEMUXr_GET BCMI_VIPER_XGXS_TESTMODEMUXr_GET
#define TESTMODEMUXr_TMUX_SELf_GET BCMI_VIPER_XGXS_TESTMODEMUXr_TMUX_SELf_GET
#define TESTMODEMUXr_TMUX_SELf_SET BCMI_VIPER_XGXS_TESTMODEMUXr_TMUX_SELf_SET
#define TESTMODEMUXr_TMUX_ENf_GET BCMI_VIPER_XGXS_TESTMODEMUXr_TMUX_ENf_GET
#define TESTMODEMUXr_TMUX_ENf_SET BCMI_VIPER_XGXS_TESTMODEMUXr_TMUX_ENf_SET
#define READ_TESTMODEMUXr BCMI_VIPER_XGXS_READ_TESTMODEMUXr
#define WRITE_TESTMODEMUXr BCMI_VIPER_XGXS_WRITE_TESTMODEMUXr
#define MODIFY_TESTMODEMUXr BCMI_VIPER_XGXS_MODIFY_TESTMODEMUXr
#define READLN_TESTMODEMUXr BCMI_VIPER_XGXS_READLN_TESTMODEMUXr
#define WRITELN_TESTMODEMUXr BCMI_VIPER_XGXS_WRITELN_TESTMODEMUXr
#define WRITEALL_TESTMODEMUXr BCMI_VIPER_XGXS_WRITEALL_TESTMODEMUXr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TESTMODEMUXr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  CX4SIGDETCNT
 * BLOCKS:   BLK2
 * REGADDR:  0x8109
 * DESC:     10GBASE-CX4 signal detect timeout value
 * RESETVAL: 0x1970 (6512)
 * ACCESS:   R/W
 * FIELDS:
 *     CX4SIGDETCNT     Signal detect down timeout counter value. This counter begins counting down whenever the analog, per Rx XAUI lane signal detect de-asserts to 1'b0. The counter resets whenever the signal detect asserts to 1'b1. Therefore, this counter expires only if the signal detect value is remains 1'b0 continuously for the duration of the counter timeout period. The IEEE802.3ae-2002, CX4 requirement is for a default value of 250us; note the counter is clocked @38.4ns lfck clock rate.
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_CX4SIGDETCNTr (0x00008109 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_CX4SIGDETCNTr_SIZE 4

/*
 * This structure should be used to declare and program CX4SIGDETCNT.
 *
 */
typedef union BCMI_VIPER_XGXS_CX4SIGDETCNTr_s {
	uint32_t v[1];
	uint32_t cx4sigdetcnt[1];
	uint32_t _cx4sigdetcnt;
} BCMI_VIPER_XGXS_CX4SIGDETCNTr_t;

#define BCMI_VIPER_XGXS_CX4SIGDETCNTr_CLR(r) (r).cx4sigdetcnt[0] = 0
#define BCMI_VIPER_XGXS_CX4SIGDETCNTr_SET(r,d) (r).cx4sigdetcnt[0] = d
#define BCMI_VIPER_XGXS_CX4SIGDETCNTr_GET(r) (r).cx4sigdetcnt[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_CX4SIGDETCNTr_CX4SIGDETCNTf_GET(r) (((r).cx4sigdetcnt[0]) & 0xffff)
#define BCMI_VIPER_XGXS_CX4SIGDETCNTr_CX4SIGDETCNTf_SET(r,f) (r).cx4sigdetcnt[0]=(((r).cx4sigdetcnt[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access CX4SIGDETCNT.
 *
 */
#define BCMI_VIPER_XGXS_READ_CX4SIGDETCNTr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_CX4SIGDETCNTr,(_r._cx4sigdetcnt))
#define BCMI_VIPER_XGXS_WRITE_CX4SIGDETCNTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_CX4SIGDETCNTr,(_r._cx4sigdetcnt)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_CX4SIGDETCNTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_CX4SIGDETCNTr,(_r._cx4sigdetcnt))
#define BCMI_VIPER_XGXS_READLN_CX4SIGDETCNTr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_CX4SIGDETCNTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._cx4sigdetcnt))
#define BCMI_VIPER_XGXS_WRITELN_CX4SIGDETCNTr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_CX4SIGDETCNTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._cx4sigdetcnt))
#define BCMI_VIPER_XGXS_WRITEALL_CX4SIGDETCNTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_CX4SIGDETCNTr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._cx4sigdetcnt))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define CX4SIGDETCNTr BCMI_VIPER_XGXS_CX4SIGDETCNTr
#define CX4SIGDETCNTr_SIZE BCMI_VIPER_XGXS_CX4SIGDETCNTr_SIZE
typedef BCMI_VIPER_XGXS_CX4SIGDETCNTr_t CX4SIGDETCNTr_t;
#define CX4SIGDETCNTr_CLR BCMI_VIPER_XGXS_CX4SIGDETCNTr_CLR
#define CX4SIGDETCNTr_SET BCMI_VIPER_XGXS_CX4SIGDETCNTr_SET
#define CX4SIGDETCNTr_GET BCMI_VIPER_XGXS_CX4SIGDETCNTr_GET
#define CX4SIGDETCNTr_CX4SIGDETCNTf_GET BCMI_VIPER_XGXS_CX4SIGDETCNTr_CX4SIGDETCNTf_GET
#define CX4SIGDETCNTr_CX4SIGDETCNTf_SET BCMI_VIPER_XGXS_CX4SIGDETCNTr_CX4SIGDETCNTf_SET
#define READ_CX4SIGDETCNTr BCMI_VIPER_XGXS_READ_CX4SIGDETCNTr
#define WRITE_CX4SIGDETCNTr BCMI_VIPER_XGXS_WRITE_CX4SIGDETCNTr
#define MODIFY_CX4SIGDETCNTr BCMI_VIPER_XGXS_MODIFY_CX4SIGDETCNTr
#define READLN_CX4SIGDETCNTr BCMI_VIPER_XGXS_READLN_CX4SIGDETCNTr
#define WRITELN_CX4SIGDETCNTr BCMI_VIPER_XGXS_WRITELN_CX4SIGDETCNTr
#define WRITEALL_CX4SIGDETCNTr BCMI_VIPER_XGXS_WRITEALL_CX4SIGDETCNTr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_CX4SIGDETCNTr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  LANERESET
 * BLOCKS:   BLK2
 * REGADDR:  0x810a
 * DESC:     Lane reset register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     RESET_RX         Reset rx lane 0
 *     RESET_TX         Reset Tx lane 0
 *     RESET_PLL        Reset the pll
 *     RESET_MDIO       Reset all mdio registers and statemachines
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_LANERESETr (0x0000810a | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_LANERESETr_SIZE 4

/*
 * This structure should be used to declare and program LANERESET.
 *
 */
typedef union BCMI_VIPER_XGXS_LANERESETr_s {
	uint32_t v[1];
	uint32_t lanereset[1];
	uint32_t _lanereset;
} BCMI_VIPER_XGXS_LANERESETr_t;

#define BCMI_VIPER_XGXS_LANERESETr_CLR(r) (r).lanereset[0] = 0
#define BCMI_VIPER_XGXS_LANERESETr_SET(r,d) (r).lanereset[0] = d
#define BCMI_VIPER_XGXS_LANERESETr_GET(r) (r).lanereset[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_LANERESETr_RESET_MDIOf_GET(r) ((((r).lanereset[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_LANERESETr_RESET_MDIOf_SET(r,f) (r).lanereset[0]=(((r).lanereset[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_LANERESETr_RESET_PLLf_GET(r) ((((r).lanereset[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_LANERESETr_RESET_PLLf_SET(r,f) (r).lanereset[0]=(((r).lanereset[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_LANERESETr_RESET_TXf_GET(r) ((((r).lanereset[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_LANERESETr_RESET_TXf_SET(r,f) (r).lanereset[0]=(((r).lanereset[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_LANERESETr_RESET_RXf_GET(r) (((r).lanereset[0]) & 0x1)
#define BCMI_VIPER_XGXS_LANERESETr_RESET_RXf_SET(r,f) (r).lanereset[0]=(((r).lanereset[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access LANERESET.
 *
 */
#define BCMI_VIPER_XGXS_READ_LANERESETr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANERESETr,(_r._lanereset))
#define BCMI_VIPER_XGXS_WRITE_LANERESETr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANERESETr,(_r._lanereset)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_LANERESETr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANERESETr,(_r._lanereset))
#define BCMI_VIPER_XGXS_READLN_LANERESETr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_LANERESETr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanereset))
#define BCMI_VIPER_XGXS_WRITELN_LANERESETr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANERESETr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._lanereset))
#define BCMI_VIPER_XGXS_WRITEALL_LANERESETr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_LANERESETr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._lanereset))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define LANERESETr BCMI_VIPER_XGXS_LANERESETr
#define LANERESETr_SIZE BCMI_VIPER_XGXS_LANERESETr_SIZE
typedef BCMI_VIPER_XGXS_LANERESETr_t LANERESETr_t;
#define LANERESETr_CLR BCMI_VIPER_XGXS_LANERESETr_CLR
#define LANERESETr_SET BCMI_VIPER_XGXS_LANERESETr_SET
#define LANERESETr_GET BCMI_VIPER_XGXS_LANERESETr_GET
#define LANERESETr_RESET_MDIOf_GET BCMI_VIPER_XGXS_LANERESETr_RESET_MDIOf_GET
#define LANERESETr_RESET_MDIOf_SET BCMI_VIPER_XGXS_LANERESETr_RESET_MDIOf_SET
#define LANERESETr_RESET_PLLf_GET BCMI_VIPER_XGXS_LANERESETr_RESET_PLLf_GET
#define LANERESETr_RESET_PLLf_SET BCMI_VIPER_XGXS_LANERESETr_RESET_PLLf_SET
#define LANERESETr_RESET_TXf_GET BCMI_VIPER_XGXS_LANERESETr_RESET_TXf_GET
#define LANERESETr_RESET_TXf_SET BCMI_VIPER_XGXS_LANERESETr_RESET_TXf_SET
#define LANERESETr_RESET_RXf_GET BCMI_VIPER_XGXS_LANERESETr_RESET_RXf_GET
#define LANERESETr_RESET_RXf_SET BCMI_VIPER_XGXS_LANERESETr_RESET_RXf_SET
#define READ_LANERESETr BCMI_VIPER_XGXS_READ_LANERESETr
#define WRITE_LANERESETr BCMI_VIPER_XGXS_WRITE_LANERESETr
#define MODIFY_LANERESETr BCMI_VIPER_XGXS_MODIFY_LANERESETr
#define READLN_LANERESETr BCMI_VIPER_XGXS_READLN_LANERESETr
#define WRITELN_LANERESETr BCMI_VIPER_XGXS_WRITELN_LANERESETr
#define WRITEALL_LANERESETr BCMI_VIPER_XGXS_WRITEALL_LANERESETr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_LANERESETr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  XGXSSTS1
 * BLOCKS:   BLK4
 * REGADDR:  0x8122
 * DESC:     status 1 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     ACTUAL_SPEED_LN0 
 *     AUTONEG_COMPLETE Indicates autonegotiation is complete
 *     LINKSTAT         Indicates Serdes link status
 *     SGMII_MODE       Indicates current mode is Serdes, SGMII mode; I.e. 10M, 100M, or 1G
 *     SERDESMODE_EN_TX Indicates current mode is Serdes mode; I.e. 10M, 100M, 1G, or 2.5G
 *     MODE_TX          NOTES: ComboCoreMode = Serdes/UniCore mode; i.e. 10M, 100M, 1G, 2.5G, and autoneg to XGXS speeds (5G, 6G, 10G, 10G HiGig, 12G HiGig, 13G, etc.)nCC = no clock compensationnLQ = no LssQnDSK = no deskewClksOFF: mode_10g[3:0] = 4'hD, 4'hE, or 4'hFDefault is set by strap pins mode_strapOperation modes
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_XGXSSTS1r (0x00008122 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_XGXSSTS1r_SIZE 4

/*
 * This structure should be used to declare and program XGXSSTS1.
 *
 */
typedef union BCMI_VIPER_XGXS_XGXSSTS1r_s {
	uint32_t v[1];
	uint32_t xgxssts1[1];
	uint32_t _xgxssts1;
} BCMI_VIPER_XGXS_XGXSSTS1r_t;

#define BCMI_VIPER_XGXS_XGXSSTS1r_CLR(r) (r).xgxssts1[0] = 0
#define BCMI_VIPER_XGXS_XGXSSTS1r_SET(r,d) (r).xgxssts1[0] = d
#define BCMI_VIPER_XGXS_XGXSSTS1r_GET(r) (r).xgxssts1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_XGXSSTS1r_MODE_TXf_GET(r) ((((r).xgxssts1[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_XGXSSTS1r_MODE_TXf_SET(r,f) (r).xgxssts1[0]=(((r).xgxssts1[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_XGXSSTS1r_SERDESMODE_EN_TXf_GET(r) ((((r).xgxssts1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_XGXSSTS1r_SERDESMODE_EN_TXf_SET(r,f) (r).xgxssts1[0]=(((r).xgxssts1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_XGXSSTS1r_SGMII_MODEf_GET(r) ((((r).xgxssts1[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_XGXSSTS1r_SGMII_MODEf_SET(r,f) (r).xgxssts1[0]=(((r).xgxssts1[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#if 0
#define BCMI_VIPER_XGXS_XGXSSTS1r_LINKSTATf_GET(r) ((((r).xgxssts1[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_XGXSSTS1r_LINKSTATf_SET(r,f) (r).xgxssts1[0]=(((r).xgxssts1[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#endif
#define BCMI_VIPER_XGXS_XGXSSTS1r_LINKSTATf_GET(r) ((((r).xgxssts1[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_XGXSSTS1r_LINKSTATf_SET(r,f) (r).xgxssts1[0]=(((r).xgxssts1[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_XGXSSTS1r_AUTONEG_COMPLETEf_GET(r) ((((r).xgxssts1[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_XGXSSTS1r_AUTONEG_COMPLETEf_SET(r,f) (r).xgxssts1[0]=(((r).xgxssts1[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_XGXSSTS1r_ACTUAL_SPEED_LN0f_GET(r) (((r).xgxssts1[0]) & 0xf)
#define BCMI_VIPER_XGXS_XGXSSTS1r_ACTUAL_SPEED_LN0f_SET(r,f) (r).xgxssts1[0]=(((r).xgxssts1[0] & ~((uint32_t)0xf)) | (((uint32_t)f) & 0xf)) | (0xf << 16)

/*
 * These macros can be used to access XGXSSTS1.
 *
 */
#define BCMI_VIPER_XGXS_READ_XGXSSTS1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_XGXSSTS1r,(_r._xgxssts1))
#define BCMI_VIPER_XGXS_WRITE_XGXSSTS1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSSTS1r,(_r._xgxssts1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_XGXSSTS1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSSTS1r,(_r._xgxssts1))
#define BCMI_VIPER_XGXS_READLN_XGXSSTS1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_XGXSSTS1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._xgxssts1))
#define BCMI_VIPER_XGXS_WRITELN_XGXSSTS1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSSTS1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._xgxssts1))
#define BCMI_VIPER_XGXS_WRITEALL_XGXSSTS1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSSTS1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._xgxssts1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define XGXSSTS1r BCMI_VIPER_XGXS_XGXSSTS1r
#define XGXSSTS1r_SIZE BCMI_VIPER_XGXS_XGXSSTS1r_SIZE
typedef BCMI_VIPER_XGXS_XGXSSTS1r_t XGXSSTS1r_t;
#define XGXSSTS1r_CLR BCMI_VIPER_XGXS_XGXSSTS1r_CLR
#define XGXSSTS1r_SET BCMI_VIPER_XGXS_XGXSSTS1r_SET
#define XGXSSTS1r_GET BCMI_VIPER_XGXS_XGXSSTS1r_GET
#define XGXSSTS1r_MODE_TXf_GET BCMI_VIPER_XGXS_XGXSSTS1r_MODE_TXf_GET
#define XGXSSTS1r_MODE_TXf_SET BCMI_VIPER_XGXS_XGXSSTS1r_MODE_TXf_SET
#define XGXSSTS1r_SERDESMODE_EN_TXf_GET BCMI_VIPER_XGXS_XGXSSTS1r_SERDESMODE_EN_TXf_GET
#define XGXSSTS1r_SERDESMODE_EN_TXf_SET BCMI_VIPER_XGXS_XGXSSTS1r_SERDESMODE_EN_TXf_SET
#define XGXSSTS1r_SGMII_MODEf_GET BCMI_VIPER_XGXS_XGXSSTS1r_SGMII_MODEf_GET
#define XGXSSTS1r_SGMII_MODEf_SET BCMI_VIPER_XGXS_XGXSSTS1r_SGMII_MODEf_SET
#define XGXSSTS1r_LINKSTATf_GET BCMI_VIPER_XGXS_XGXSSTS1r_LINKSTATf_GET
#define XGXSSTS1r_LINKSTATf_SET BCMI_VIPER_XGXS_XGXSSTS1r_LINKSTATf_SET
#define XGXSSTS1r_AUTONEG_COMPLETEf_GET BCMI_VIPER_XGXS_XGXSSTS1r_AUTONEG_COMPLETEf_GET
#define XGXSSTS1r_AUTONEG_COMPLETEf_SET BCMI_VIPER_XGXS_XGXSSTS1r_AUTONEG_COMPLETEf_SET
#define XGXSSTS1r_ACTUAL_SPEED_LN0f_GET BCMI_VIPER_XGXS_XGXSSTS1r_ACTUAL_SPEED_LN0f_GET
#define XGXSSTS1r_ACTUAL_SPEED_LN0f_SET BCMI_VIPER_XGXS_XGXSSTS1r_ACTUAL_SPEED_LN0f_SET
#define READ_XGXSSTS1r BCMI_VIPER_XGXS_READ_XGXSSTS1r
#define WRITE_XGXSSTS1r BCMI_VIPER_XGXS_WRITE_XGXSSTS1r
#define MODIFY_XGXSSTS1r BCMI_VIPER_XGXS_MODIFY_XGXSSTS1r
#define READLN_XGXSSTS1r BCMI_VIPER_XGXS_READLN_XGXSSTS1r
#define WRITELN_XGXSSTS1r BCMI_VIPER_XGXS_WRITELN_XGXSSTS1r
#define WRITEALL_XGXSSTS1r BCMI_VIPER_XGXS_WRITEALL_XGXSSTS1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_XGXSSTS1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  XGXSSTS2
 * BLOCKS:   BLK4
 * REGADDR:  0x8123
 * DESC:     status 2 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     FREQ_SEL_TX      
 *     FREQ_SEL_RX      
 *     GPWRDWN_TX       
 *     GPWRDWN_RX       
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_XGXSSTS2r (0x00008123 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_XGXSSTS2r_SIZE 4

/*
 * This structure should be used to declare and program XGXSSTS2.
 *
 */
typedef union BCMI_VIPER_XGXS_XGXSSTS2r_s {
	uint32_t v[1];
	uint32_t xgxssts2[1];
	uint32_t _xgxssts2;
} BCMI_VIPER_XGXS_XGXSSTS2r_t;

#define BCMI_VIPER_XGXS_XGXSSTS2r_CLR(r) (r).xgxssts2[0] = 0
#define BCMI_VIPER_XGXS_XGXSSTS2r_SET(r,d) (r).xgxssts2[0] = d
#define BCMI_VIPER_XGXS_XGXSSTS2r_GET(r) (r).xgxssts2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_XGXSSTS2r_GPWRDWN_RXf_GET(r) ((((r).xgxssts2[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_XGXSSTS2r_GPWRDWN_RXf_SET(r,f) (r).xgxssts2[0]=(((r).xgxssts2[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_XGXSSTS2r_GPWRDWN_TXf_GET(r) ((((r).xgxssts2[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_XGXSSTS2r_GPWRDWN_TXf_SET(r,f) (r).xgxssts2[0]=(((r).xgxssts2[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_XGXSSTS2r_FREQ_SEL_RXf_GET(r) ((((r).xgxssts2[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_XGXSSTS2r_FREQ_SEL_RXf_SET(r,f) (r).xgxssts2[0]=(((r).xgxssts2[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_XGXSSTS2r_FREQ_SEL_TXf_GET(r) (((r).xgxssts2[0]) & 0x1)
#define BCMI_VIPER_XGXS_XGXSSTS2r_FREQ_SEL_TXf_SET(r,f) (r).xgxssts2[0]=(((r).xgxssts2[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access XGXSSTS2.
 *
 */
#define BCMI_VIPER_XGXS_READ_XGXSSTS2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_XGXSSTS2r,(_r._xgxssts2))
#define BCMI_VIPER_XGXS_WRITE_XGXSSTS2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSSTS2r,(_r._xgxssts2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_XGXSSTS2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSSTS2r,(_r._xgxssts2))
#define BCMI_VIPER_XGXS_READLN_XGXSSTS2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_XGXSSTS2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._xgxssts2))
#define BCMI_VIPER_XGXS_WRITELN_XGXSSTS2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSSTS2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._xgxssts2))
#define BCMI_VIPER_XGXS_WRITEALL_XGXSSTS2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_XGXSSTS2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._xgxssts2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define XGXSSTS2r BCMI_VIPER_XGXS_XGXSSTS2r
#define XGXSSTS2r_SIZE BCMI_VIPER_XGXS_XGXSSTS2r_SIZE
typedef BCMI_VIPER_XGXS_XGXSSTS2r_t XGXSSTS2r_t;
#define XGXSSTS2r_CLR BCMI_VIPER_XGXS_XGXSSTS2r_CLR
#define XGXSSTS2r_SET BCMI_VIPER_XGXS_XGXSSTS2r_SET
#define XGXSSTS2r_GET BCMI_VIPER_XGXS_XGXSSTS2r_GET
#define XGXSSTS2r_GPWRDWN_RXf_GET BCMI_VIPER_XGXS_XGXSSTS2r_GPWRDWN_RXf_GET
#define XGXSSTS2r_GPWRDWN_RXf_SET BCMI_VIPER_XGXS_XGXSSTS2r_GPWRDWN_RXf_SET
#define XGXSSTS2r_GPWRDWN_TXf_GET BCMI_VIPER_XGXS_XGXSSTS2r_GPWRDWN_TXf_GET
#define XGXSSTS2r_GPWRDWN_TXf_SET BCMI_VIPER_XGXS_XGXSSTS2r_GPWRDWN_TXf_SET
#define XGXSSTS2r_FREQ_SEL_RXf_GET BCMI_VIPER_XGXS_XGXSSTS2r_FREQ_SEL_RXf_GET
#define XGXSSTS2r_FREQ_SEL_RXf_SET BCMI_VIPER_XGXS_XGXSSTS2r_FREQ_SEL_RXf_SET
#define XGXSSTS2r_FREQ_SEL_TXf_GET BCMI_VIPER_XGXS_XGXSSTS2r_FREQ_SEL_TXf_GET
#define XGXSSTS2r_FREQ_SEL_TXf_SET BCMI_VIPER_XGXS_XGXSSTS2r_FREQ_SEL_TXf_SET
#define READ_XGXSSTS2r BCMI_VIPER_XGXS_READ_XGXSSTS2r
#define WRITE_XGXSSTS2r BCMI_VIPER_XGXS_WRITE_XGXSSTS2r
#define MODIFY_XGXSSTS2r BCMI_VIPER_XGXS_MODIFY_XGXSSTS2r
#define READLN_XGXSSTS2r BCMI_VIPER_XGXS_READLN_XGXSSTS2r
#define WRITELN_XGXSSTS2r BCMI_VIPER_XGXS_WRITELN_XGXSSTS2r
#define WRITEALL_XGXSSTS2r BCMI_VIPER_XGXS_WRITEALL_XGXSSTS2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_XGXSSTS2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  STS1000X1
 * BLOCKS:   BLK4
 * REGADDR:  0x8124
 * DESC:     1000X status 1 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     SGMII_MODE       1 = sgmii mode0 = fiber mode (1000-X)
 *     LINK_STATUS      1 = link is up0 = link is down
 *     DUPLEX_STATUS    1 = full-duplex0 = half-duplex
 *     SPEED_STATUS     11 = 2.5G10 = gigabit01 = 100 mbps00 = 10 mbps
 *     PAUSE_RESOLUTION_TXSIDE 1 = enable pause transmit0 = disable pause transmit
 *     PAUSE_RESOLUTION_RXSIDE 1 = enable pause receive0 = disable pause receive
 *     LINK_STATUS_CHANGE 1 = link status has changed since last read0 = link status has not changed since last readLH, Latching High
 *     EARLY_END_EXTENSION_DETECTED 1 = early end extension since last read (early_end_ext in pcs receive fsm)0 = no early end extension since last readLH, Latching High
 *     CARRIER_EXTEND_ERR_DETECTED 1 = carrier extend error since last read (extend_err in pcs receive fsm)0 = no carrier extend error since last readLH, Latching High
 *     RX_ERR_DETECTED  1 = receive error since last read (early_end state in pcs receive fsm)0 = no receive error since last readLH, Latching High
 *     TX_ERR_DETECTED  1 = transmit error code detected since last read (rx_data_error state in pcs receive fsm)0 = no transmit error code detected since last readLH, Latching High
 *     CRC_ERR_DETECTED 1 = crc error detected since last read0 = no crc error detected since last read or detection is disabled via register Control1000X1, bit crc_checker_disableLH, Latching High
 *     FALSE_CARRIER_DETECTED 1 = false carrier detected since last read0 = no false carrier detected since last readLH, Latching High
 *     RXFIFO_ERR_DETECTED 1 = receive fifo error detected since last read0 = no receive fifo error detected since last readLH, Latching High
 *     TXFIFO_ERR_DETECTED 1 = transmit fifo error detected since last read0 = no transmit fifo error detected since last readLH, Latching High
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_STS1000X1r (0x00008124 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_STS1000X1r_SIZE 4

/*
 * This structure should be used to declare and program STS1000X1.
 *
 */
typedef union BCMI_VIPER_XGXS_STS1000X1r_s {
	uint32_t v[1];
	uint32_t sts1000x1[1];
	uint32_t _sts1000x1;
} BCMI_VIPER_XGXS_STS1000X1r_t;

#define BCMI_VIPER_XGXS_STS1000X1r_CLR(r) (r).sts1000x1[0] = 0
#define BCMI_VIPER_XGXS_STS1000X1r_SET(r,d) (r).sts1000x1[0] = d
#define BCMI_VIPER_XGXS_STS1000X1r_GET(r) (r).sts1000x1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_STS1000X1r_TXFIFO_ERR_DETECTEDf_GET(r) ((((r).sts1000x1[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_TXFIFO_ERR_DETECTEDf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_STS1000X1r_RXFIFO_ERR_DETECTEDf_GET(r) ((((r).sts1000x1[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_RXFIFO_ERR_DETECTEDf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_STS1000X1r_FALSE_CARRIER_DETECTEDf_GET(r) ((((r).sts1000x1[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_FALSE_CARRIER_DETECTEDf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_STS1000X1r_CRC_ERR_DETECTEDf_GET(r) ((((r).sts1000x1[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_CRC_ERR_DETECTEDf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_STS1000X1r_TX_ERR_DETECTEDf_GET(r) ((((r).sts1000x1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_TX_ERR_DETECTEDf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_STS1000X1r_RX_ERR_DETECTEDf_GET(r) ((((r).sts1000x1[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_RX_ERR_DETECTEDf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_GET(r) ((((r).sts1000x1[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_GET(r) ((((r).sts1000x1[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_STS1000X1r_LINK_STATUS_CHANGEf_GET(r) ((((r).sts1000x1[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_LINK_STATUS_CHANGEf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_GET(r) ((((r).sts1000x1[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_GET(r) ((((r).sts1000x1[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_STS1000X1r_SPEED_STATUSf_GET(r) ((((r).sts1000x1[0]) >> 3) & 0x3)
#define BCMI_VIPER_XGXS_STS1000X1r_SPEED_STATUSf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x3 << 3)) | ((((uint32_t)f) & 0x3) << 3)) | (3 << (16 + 3))
#define BCMI_VIPER_XGXS_STS1000X1r_DUPLEX_STATUSf_GET(r) ((((r).sts1000x1[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_DUPLEX_STATUSf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_STS1000X1r_LINK_STATUSf_GET(r) ((((r).sts1000x1[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_LINK_STATUSf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_STS1000X1r_SGMII_MODEf_GET(r) (((r).sts1000x1[0]) & 0x1)
#define BCMI_VIPER_XGXS_STS1000X1r_SGMII_MODEf_SET(r,f) (r).sts1000x1[0]=(((r).sts1000x1[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access STS1000X1.
 *
 */
#define BCMI_VIPER_XGXS_READ_STS1000X1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_STS1000X1r,(_r._sts1000x1))
#define BCMI_VIPER_XGXS_WRITE_STS1000X1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_STS1000X1r,(_r._sts1000x1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_STS1000X1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_STS1000X1r,(_r._sts1000x1))
#define BCMI_VIPER_XGXS_READLN_STS1000X1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_STS1000X1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._sts1000x1))
#define BCMI_VIPER_XGXS_WRITELN_STS1000X1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_STS1000X1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._sts1000x1))
#define BCMI_VIPER_XGXS_WRITEALL_STS1000X1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_STS1000X1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._sts1000x1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define STS1000X1r BCMI_VIPER_XGXS_STS1000X1r
#define STS1000X1r_SIZE BCMI_VIPER_XGXS_STS1000X1r_SIZE
typedef BCMI_VIPER_XGXS_STS1000X1r_t STS1000X1r_t;
#define STS1000X1r_CLR BCMI_VIPER_XGXS_STS1000X1r_CLR
#define STS1000X1r_SET BCMI_VIPER_XGXS_STS1000X1r_SET
#define STS1000X1r_GET BCMI_VIPER_XGXS_STS1000X1r_GET
#define STS1000X1r_TXFIFO_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_STS1000X1r_TXFIFO_ERR_DETECTEDf_GET
#define STS1000X1r_TXFIFO_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_STS1000X1r_TXFIFO_ERR_DETECTEDf_SET
#define STS1000X1r_RXFIFO_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_STS1000X1r_RXFIFO_ERR_DETECTEDf_GET
#define STS1000X1r_RXFIFO_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_STS1000X1r_RXFIFO_ERR_DETECTEDf_SET
#define STS1000X1r_FALSE_CARRIER_DETECTEDf_GET BCMI_VIPER_XGXS_STS1000X1r_FALSE_CARRIER_DETECTEDf_GET
#define STS1000X1r_FALSE_CARRIER_DETECTEDf_SET BCMI_VIPER_XGXS_STS1000X1r_FALSE_CARRIER_DETECTEDf_SET
#define STS1000X1r_CRC_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_STS1000X1r_CRC_ERR_DETECTEDf_GET
#define STS1000X1r_CRC_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_STS1000X1r_CRC_ERR_DETECTEDf_SET
#define STS1000X1r_TX_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_STS1000X1r_TX_ERR_DETECTEDf_GET
#define STS1000X1r_TX_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_STS1000X1r_TX_ERR_DETECTEDf_SET
#define STS1000X1r_RX_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_STS1000X1r_RX_ERR_DETECTEDf_GET
#define STS1000X1r_RX_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_STS1000X1r_RX_ERR_DETECTEDf_SET
#define STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_GET
#define STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_SET
#define STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_GET BCMI_VIPER_XGXS_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_GET
#define STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_SET BCMI_VIPER_XGXS_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_SET
#define STS1000X1r_LINK_STATUS_CHANGEf_GET BCMI_VIPER_XGXS_STS1000X1r_LINK_STATUS_CHANGEf_GET
#define STS1000X1r_LINK_STATUS_CHANGEf_SET BCMI_VIPER_XGXS_STS1000X1r_LINK_STATUS_CHANGEf_SET
#define STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_GET BCMI_VIPER_XGXS_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_GET
#define STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_SET BCMI_VIPER_XGXS_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_SET
#define STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_GET BCMI_VIPER_XGXS_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_GET
#define STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_SET BCMI_VIPER_XGXS_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_SET
#define STS1000X1r_SPEED_STATUSf_GET BCMI_VIPER_XGXS_STS1000X1r_SPEED_STATUSf_GET
#define STS1000X1r_SPEED_STATUSf_SET BCMI_VIPER_XGXS_STS1000X1r_SPEED_STATUSf_SET
#define STS1000X1r_DUPLEX_STATUSf_GET BCMI_VIPER_XGXS_STS1000X1r_DUPLEX_STATUSf_GET
#define STS1000X1r_DUPLEX_STATUSf_SET BCMI_VIPER_XGXS_STS1000X1r_DUPLEX_STATUSf_SET
#define STS1000X1r_LINK_STATUSf_GET BCMI_VIPER_XGXS_STS1000X1r_LINK_STATUSf_GET
#define STS1000X1r_LINK_STATUSf_SET BCMI_VIPER_XGXS_STS1000X1r_LINK_STATUSf_SET
#define STS1000X1r_SGMII_MODEf_GET BCMI_VIPER_XGXS_STS1000X1r_SGMII_MODEf_GET
#define STS1000X1r_SGMII_MODEf_SET BCMI_VIPER_XGXS_STS1000X1r_SGMII_MODEf_SET
#define READ_STS1000X1r BCMI_VIPER_XGXS_READ_STS1000X1r
#define WRITE_STS1000X1r BCMI_VIPER_XGXS_WRITE_STS1000X1r
#define MODIFY_STS1000X1r BCMI_VIPER_XGXS_MODIFY_STS1000X1r
#define READLN_STS1000X1r BCMI_VIPER_XGXS_READLN_STS1000X1r
#define WRITELN_STS1000X1r BCMI_VIPER_XGXS_WRITELN_STS1000X1r
#define WRITEALL_STS1000X1r BCMI_VIPER_XGXS_WRITEALL_STS1000X1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_STS1000X1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  EEECTL
 * BLOCKS:   BLK7
 * REGADDR:  0x8150
 * DESC:     Cl48 PCS EEE Control register
 * RESETVAL: 0x3 (3)
 * ACCESS:   R/W
 * FIELDS:
 *     LPI_EN_TX        Enable low power idles pass-through mode for transmitterDefault is set by LPI_enable_10g_strap
 *     LPI_EN_RX        Enable low power idles pass-through mode for receiverDefault is set by LPI_enable_10g_strap
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_EEECTLr (0x00008150 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_EEECTLr_SIZE 4

/*
 * This structure should be used to declare and program EEECTL.
 *
 */
typedef union BCMI_VIPER_XGXS_EEECTLr_s {
	uint32_t v[1];
	uint32_t eeectl[1];
	uint32_t _eeectl;
} BCMI_VIPER_XGXS_EEECTLr_t;

#define BCMI_VIPER_XGXS_EEECTLr_CLR(r) (r).eeectl[0] = 0
#define BCMI_VIPER_XGXS_EEECTLr_SET(r,d) (r).eeectl[0] = d
#define BCMI_VIPER_XGXS_EEECTLr_GET(r) (r).eeectl[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_EEECTLr_LPI_EN_RXf_GET(r) ((((r).eeectl[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_EEECTLr_LPI_EN_RXf_SET(r,f) (r).eeectl[0]=(((r).eeectl[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_EEECTLr_LPI_EN_TXf_GET(r) (((r).eeectl[0]) & 0x1)
#define BCMI_VIPER_XGXS_EEECTLr_LPI_EN_TXf_SET(r,f) (r).eeectl[0]=(((r).eeectl[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access EEECTL.
 *
 */
#define BCMI_VIPER_XGXS_READ_EEECTLr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_EEECTLr,(_r._eeectl))
#define BCMI_VIPER_XGXS_WRITE_EEECTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_EEECTLr,(_r._eeectl)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_EEECTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_EEECTLr,(_r._eeectl))
#define BCMI_VIPER_XGXS_READLN_EEECTLr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_EEECTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._eeectl))
#define BCMI_VIPER_XGXS_WRITELN_EEECTLr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_EEECTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._eeectl))
#define BCMI_VIPER_XGXS_WRITEALL_EEECTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_EEECTLr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._eeectl))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define EEECTLr BCMI_VIPER_XGXS_EEECTLr
#define EEECTLr_SIZE BCMI_VIPER_XGXS_EEECTLr_SIZE
typedef BCMI_VIPER_XGXS_EEECTLr_t EEECTLr_t;
#define EEECTLr_CLR BCMI_VIPER_XGXS_EEECTLr_CLR
#define EEECTLr_SET BCMI_VIPER_XGXS_EEECTLr_SET
#define EEECTLr_GET BCMI_VIPER_XGXS_EEECTLr_GET
#define EEECTLr_LPI_EN_RXf_GET BCMI_VIPER_XGXS_EEECTLr_LPI_EN_RXf_GET
#define EEECTLr_LPI_EN_RXf_SET BCMI_VIPER_XGXS_EEECTLr_LPI_EN_RXf_SET
#define EEECTLr_LPI_EN_TXf_GET BCMI_VIPER_XGXS_EEECTLr_LPI_EN_TXf_GET
#define EEECTLr_LPI_EN_TXf_SET BCMI_VIPER_XGXS_EEECTLr_LPI_EN_TXf_SET
#define READ_EEECTLr BCMI_VIPER_XGXS_READ_EEECTLr
#define WRITE_EEECTLr BCMI_VIPER_XGXS_WRITE_EEECTLr
#define MODIFY_EEECTLr BCMI_VIPER_XGXS_MODIFY_EEECTLr
#define READLN_EEECTLr BCMI_VIPER_XGXS_READLN_EEECTLr
#define WRITELN_EEECTLr BCMI_VIPER_XGXS_WRITELN_EEECTLr
#define WRITEALL_EEECTLr BCMI_VIPER_XGXS_WRITEALL_EEECTLr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_EEECTLr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER: PRBS_DECOUPLE
 * BLOCKS:   BLK7
 * REGADDR:  0x815a
 * DESC:     PRBS MONITOR DECOUPLE Control register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     TX_DATAI_PRBS_SEL_0      
 *     TX_DATAI_PRBS_SEL_1      
 *     TX_DATAI_PRBS_SEL_2      
 *     TX_DATAI_PRBS_SEL_3      
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr (0x0000815a | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_SIZE 4

/*
 * This structure should be used to declare and program PRBS_DECOUPLE.
 *
 */
typedef union BCMI_VIPER_XGXS_PRBS_DECOUPLEr_s {
	uint32_t v[1];
	uint32_t prbs_decouple[1];
	uint32_t _prbs_decouple;
} BCMI_VIPER_XGXS_PRBS_DECOUPLEr_t;

#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_CLR(r) (r).prbs_decouple[0] = 0
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_SET(r,d) (r).prbs_decouple[0] = d
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_GET(r) (r).prbs_decouple[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_3f_GET(r) ((((r).prbs_decouple[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_3f_SET(r,f) (r).prbs_decouple[0]=(((r).prbs_decouple[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (0x1 << (16 + 7))
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_2f_GET(r) ((((r).prbs_decouple[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_2f_SET(r,f) (r).prbs_decouple[0]=(((r).prbs_decouple[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (0x1 << (16 + 6))
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_1f_GET(r) ((((r).prbs_decouple[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_1f_SET(r,f) (r).prbs_decouple[0]=(((r).prbs_decouple[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (0x1 << (16 + 5))
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_0f_GET(r) ((((r).prbs_decouple[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_0f_SET(r,f) (r).prbs_decouple[0]=(((r).prbs_decouple[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (0x1 << (16 + 4))

/*
 * These macros can be used to access PRBS_DECOUPLE.
 *
 */
#define BCMI_VIPER_XGXS_READ_PRBS_DECOUPLEr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PRBS_DECOUPLEr,(_r._prbs_decouple))
#define BCMI_VIPER_XGXS_WRITE_PRBS_DECOUPLEr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PRBS_DECOUPLEr,(_r._prbs_decouple)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PRBS_DECOUPLEr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PRBS_DECOUPLEr,(_r._prbs_decouple))
#define BCMI_VIPER_XGXS_READLN_PRBS_DECOUPLEr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PRBS_DECOUPLEr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._prbs_decouple))
#define BCMI_VIPER_XGXS_WRITELN_PRBS_DECOUPLEr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PRBS_DECOUPLEr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._prbs_decouple))
#define BCMI_VIPER_XGXS_WRITEALL_PRBS_DECOUPLEr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PRBS_DECOUPLEr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._prbs_decouple))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PRBS_DECOUPLEr BCMI_VIPER_XGXS_PRBS_DECOUPLEr
#define PRBS_DECOUPLEr_SIZE BCMI_VIPER_XGXS_PRBS_DECOUPLEr_SIZE
typedef BCMI_VIPER_XGXS_PRBS_DECOUPLEr_t PRBS_DECOUPLE_t;
#define PRBS_DECOUPLEr_CLR BCMI_VIPER_XGXS_PRBS_DECOUPLEr_CLR
#define PRBS_DECOUPLEr_SET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_SET
#define PRBS_DECOUPLEr_GET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_GET
#define PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_3f_GET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_3f_GET
#define PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_3f_SET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_3f_SET
#define PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_2f_GET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_2f_GET
#define PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_2f_SET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_2f_SET
#define PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_1f_GET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_1f_GET
#define PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_1f_SET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_1f_SET
#define PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_0f_GET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_0f_GET
#define PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_0f_SET BCMI_VIPER_XGXS_PRBS_DECOUPLEr_TX_DATAI_PRBS_SEL_0f_SET
#define READ_PRBS_DECOUPLEr BCMI_VIPER_XGXS_READ_PRBS_DECOUPLEr
#define WRITE_PRBS_DECOUPLEr BCMI_VIPER_XGXS_WRITE_PRBS_DECOUPLEr
#define MODIFY_PRBS_DECOUPLEr BCMI_VIPER_XGXS_MODIFY_PRBS_DECOUPLEr
#define READLN_PRBS_DECOUPLEr BCMI_VIPER_XGXS_READLN_PRBS_DECOUPLEr
#define WRITELN_PRBS_DECOUPLEr BCMI_VIPER_XGXS_WRITELN_PRBS_DECOUPLEr
#define WRITEALL_PRBS_DECOUPLEr BCMI_VIPER_XGXS_WRITEALL_PRBS_DECOUPLEr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PRBS_DECOUPLEr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TXLNSWP1
 * BLOCKS:   BLK8
 * REGADDR:  0x8169
 * DESC:     TX Lane Swap Control 1 Register
 * RESETVAL: 0xe4 (228)
 * ACCESS:   R/W
 * FIELDS:
 *     TX0_LNSWAP_SEL   TX DIGITAL to ANALOG Lane0 swap select
 *     TX1_LNSWAP_SEL   TX DIGITAL to ANALOG Lane1 swap select
 *     TX2_LNSWAP_SEL   TX DIGITAL to ANALOG Lane2 swap select
 *     TX3_LNSWAP_SEL   TX DIGITAL to ANALOG Lane3 swap select
 *     FORCE_GB_BYPASS_EN set GearBox bypass control in force mode on 4 lanes individualy
 *     FORCE_GB_BYPASS_VAL for each individual lane, when force_GB_bypass_en is set,force GearBox to be bypassed when set to 1'b1; not bypassed when set to 1'b0
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TXLNSWP1r (0x00008169 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TXLNSWP1r_SIZE 4

/*
 * This structure should be used to declare and program TXLNSWP1.
 *
 */
typedef union BCMI_VIPER_XGXS_TXLNSWP1r_s {
	uint32_t v[1];
	uint32_t txlnswp1[1];
	uint32_t _txlnswp1;
} BCMI_VIPER_XGXS_TXLNSWP1r_t;

#define BCMI_VIPER_XGXS_TXLNSWP1r_CLR(r) (r).txlnswp1[0] = 0
#define BCMI_VIPER_XGXS_TXLNSWP1r_SET(r,d) (r).txlnswp1[0] = d
#define BCMI_VIPER_XGXS_TXLNSWP1r_GET(r) (r).txlnswp1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TXLNSWP1r_FORCE_GB_BYPASS_VALf_GET(r) ((((r).txlnswp1[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_TXLNSWP1r_FORCE_GB_BYPASS_VALf_SET(r,f) (r).txlnswp1[0]=(((r).txlnswp1[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_TXLNSWP1r_FORCE_GB_BYPASS_ENf_GET(r) ((((r).txlnswp1[0]) >> 8) & 0xf)
#define BCMI_VIPER_XGXS_TXLNSWP1r_FORCE_GB_BYPASS_ENf_SET(r,f) (r).txlnswp1[0]=(((r).txlnswp1[0] & ~((uint32_t)0xf << 8)) | ((((uint32_t)f) & 0xf) << 8)) | (15 << (16 + 8))
#define BCMI_VIPER_XGXS_TXLNSWP1r_TX3_LNSWAP_SELf_GET(r) ((((r).txlnswp1[0]) >> 6) & 0x3)
#define BCMI_VIPER_XGXS_TXLNSWP1r_TX3_LNSWAP_SELf_SET(r,f) (r).txlnswp1[0]=(((r).txlnswp1[0] & ~((uint32_t)0x3 << 6)) | ((((uint32_t)f) & 0x3) << 6)) | (3 << (16 + 6))
#define BCMI_VIPER_XGXS_TXLNSWP1r_TX2_LNSWAP_SELf_GET(r) ((((r).txlnswp1[0]) >> 4) & 0x3)
#define BCMI_VIPER_XGXS_TXLNSWP1r_TX2_LNSWAP_SELf_SET(r,f) (r).txlnswp1[0]=(((r).txlnswp1[0] & ~((uint32_t)0x3 << 4)) | ((((uint32_t)f) & 0x3) << 4)) | (3 << (16 + 4))
#define BCMI_VIPER_XGXS_TXLNSWP1r_TX1_LNSWAP_SELf_GET(r) ((((r).txlnswp1[0]) >> 2) & 0x3)
#define BCMI_VIPER_XGXS_TXLNSWP1r_TX1_LNSWAP_SELf_SET(r,f) (r).txlnswp1[0]=(((r).txlnswp1[0] & ~((uint32_t)0x3 << 2)) | ((((uint32_t)f) & 0x3) << 2)) | (3 << (16 + 2))
#define BCMI_VIPER_XGXS_TXLNSWP1r_TX0_LNSWAP_SELf_GET(r) (((r).txlnswp1[0]) & 0x3)
#define BCMI_VIPER_XGXS_TXLNSWP1r_TX0_LNSWAP_SELf_SET(r,f) (r).txlnswp1[0]=(((r).txlnswp1[0] & ~((uint32_t)0x3)) | (((uint32_t)f) & 0x3)) | (0x3 << 16)

/*
 * These macros can be used to access TXLNSWP1.
 *
 */
#define BCMI_VIPER_XGXS_READ_TXLNSWP1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TXLNSWP1r,(_r._txlnswp1))
#define BCMI_VIPER_XGXS_WRITE_TXLNSWP1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TXLNSWP1r,(_r._txlnswp1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TXLNSWP1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TXLNSWP1r,(_r._txlnswp1))
#define BCMI_VIPER_XGXS_READLN_TXLNSWP1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TXLNSWP1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._txlnswp1))
#define BCMI_VIPER_XGXS_WRITELN_TXLNSWP1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TXLNSWP1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._txlnswp1))
#define BCMI_VIPER_XGXS_WRITEALL_TXLNSWP1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TXLNSWP1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._txlnswp1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TXLNSWP1r BCMI_VIPER_XGXS_TXLNSWP1r
#define TXLNSWP1r_SIZE BCMI_VIPER_XGXS_TXLNSWP1r_SIZE
typedef BCMI_VIPER_XGXS_TXLNSWP1r_t TXLNSWP1r_t;
#define TXLNSWP1r_CLR BCMI_VIPER_XGXS_TXLNSWP1r_CLR
#define TXLNSWP1r_SET BCMI_VIPER_XGXS_TXLNSWP1r_SET
#define TXLNSWP1r_GET BCMI_VIPER_XGXS_TXLNSWP1r_GET
#define TXLNSWP1r_FORCE_GB_BYPASS_VALf_GET BCMI_VIPER_XGXS_TXLNSWP1r_FORCE_GB_BYPASS_VALf_GET
#define TXLNSWP1r_FORCE_GB_BYPASS_VALf_SET BCMI_VIPER_XGXS_TXLNSWP1r_FORCE_GB_BYPASS_VALf_SET
#define TXLNSWP1r_FORCE_GB_BYPASS_ENf_GET BCMI_VIPER_XGXS_TXLNSWP1r_FORCE_GB_BYPASS_ENf_GET
#define TXLNSWP1r_FORCE_GB_BYPASS_ENf_SET BCMI_VIPER_XGXS_TXLNSWP1r_FORCE_GB_BYPASS_ENf_SET
#define TXLNSWP1r_TX3_LNSWAP_SELf_GET BCMI_VIPER_XGXS_TXLNSWP1r_TX3_LNSWAP_SELf_GET
#define TXLNSWP1r_TX3_LNSWAP_SELf_SET BCMI_VIPER_XGXS_TXLNSWP1r_TX3_LNSWAP_SELf_SET
#define TXLNSWP1r_TX2_LNSWAP_SELf_GET BCMI_VIPER_XGXS_TXLNSWP1r_TX2_LNSWAP_SELf_GET
#define TXLNSWP1r_TX2_LNSWAP_SELf_SET BCMI_VIPER_XGXS_TXLNSWP1r_TX2_LNSWAP_SELf_SET
#define TXLNSWP1r_TX1_LNSWAP_SELf_GET BCMI_VIPER_XGXS_TXLNSWP1r_TX1_LNSWAP_SELf_GET
#define TXLNSWP1r_TX1_LNSWAP_SELf_SET BCMI_VIPER_XGXS_TXLNSWP1r_TX1_LNSWAP_SELf_SET
#define TXLNSWP1r_TX0_LNSWAP_SELf_GET BCMI_VIPER_XGXS_TXLNSWP1r_TX0_LNSWAP_SELf_GET
#define TXLNSWP1r_TX0_LNSWAP_SELf_SET BCMI_VIPER_XGXS_TXLNSWP1r_TX0_LNSWAP_SELf_SET
#define READ_TXLNSWP1r BCMI_VIPER_XGXS_READ_TXLNSWP1r
#define WRITE_TXLNSWP1r BCMI_VIPER_XGXS_WRITE_TXLNSWP1r
#define MODIFY_TXLNSWP1r BCMI_VIPER_XGXS_MODIFY_TXLNSWP1r
#define READLN_TXLNSWP1r BCMI_VIPER_XGXS_READLN_TXLNSWP1r
#define WRITELN_TXLNSWP1r BCMI_VIPER_XGXS_WRITELN_TXLNSWP1r
#define WRITEALL_TXLNSWP1r BCMI_VIPER_XGXS_WRITEALL_TXLNSWP1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TXLNSWP1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  TXLNSWP2
 * BLOCKS:   BLK8
 * REGADDR:  0x816a
 * DESC:     TX Lane Swap Control 2 Register
 * RESETVAL: 0xe4 (228)
 * ACCESS:   R/W
 * FIELDS:
 *     TX0_ANA_LNSWAP_SEL TX ANALOG to DIGITAL Lane0 swap select
 *     TX1_ANA_LNSWAP_SEL TX ANALOG to DIGITAL Lane1 swap select
 *     TX2_ANA_LNSWAP_SEL TX ANALOG to DIGITAL Lane2 swap select
 *     TX3_ANA_LNSWAP_SEL TX ANALOG to DIGITAL Lane3 swap select
 *     FORCE_TX_ANA_LNSWAP_SEL Force ANALOG to DIGITAL mux-selects
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_TXLNSWP2r (0x0000816a | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_TXLNSWP2r_SIZE 4

/*
 * This structure should be used to declare and program TXLNSWP2.
 *
 */
typedef union BCMI_VIPER_XGXS_TXLNSWP2r_s {
	uint32_t v[1];
	uint32_t txlnswp2[1];
	uint32_t _txlnswp2;
} BCMI_VIPER_XGXS_TXLNSWP2r_t;

#define BCMI_VIPER_XGXS_TXLNSWP2r_CLR(r) (r).txlnswp2[0] = 0
#define BCMI_VIPER_XGXS_TXLNSWP2r_SET(r,d) (r).txlnswp2[0] = d
#define BCMI_VIPER_XGXS_TXLNSWP2r_GET(r) (r).txlnswp2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_TXLNSWP2r_FORCE_TX_ANA_LNSWAP_SELf_GET(r) ((((r).txlnswp2[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_TXLNSWP2r_FORCE_TX_ANA_LNSWAP_SELf_SET(r,f) (r).txlnswp2[0]=(((r).txlnswp2[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_TXLNSWP2r_TX3_ANA_LNSWAP_SELf_GET(r) ((((r).txlnswp2[0]) >> 6) & 0x3)
#define BCMI_VIPER_XGXS_TXLNSWP2r_TX3_ANA_LNSWAP_SELf_SET(r,f) (r).txlnswp2[0]=(((r).txlnswp2[0] & ~((uint32_t)0x3 << 6)) | ((((uint32_t)f) & 0x3) << 6)) | (3 << (16 + 6))
#define BCMI_VIPER_XGXS_TXLNSWP2r_TX2_ANA_LNSWAP_SELf_GET(r) ((((r).txlnswp2[0]) >> 4) & 0x3)
#define BCMI_VIPER_XGXS_TXLNSWP2r_TX2_ANA_LNSWAP_SELf_SET(r,f) (r).txlnswp2[0]=(((r).txlnswp2[0] & ~((uint32_t)0x3 << 4)) | ((((uint32_t)f) & 0x3) << 4)) | (3 << (16 + 4))
#define BCMI_VIPER_XGXS_TXLNSWP2r_TX1_ANA_LNSWAP_SELf_GET(r) ((((r).txlnswp2[0]) >> 2) & 0x3)
#define BCMI_VIPER_XGXS_TXLNSWP2r_TX1_ANA_LNSWAP_SELf_SET(r,f) (r).txlnswp2[0]=(((r).txlnswp2[0] & ~((uint32_t)0x3 << 2)) | ((((uint32_t)f) & 0x3) << 2)) | (3 << (16 + 2))
#define BCMI_VIPER_XGXS_TXLNSWP2r_TX0_ANA_LNSWAP_SELf_GET(r) (((r).txlnswp2[0]) & 0x3)
#define BCMI_VIPER_XGXS_TXLNSWP2r_TX0_ANA_LNSWAP_SELf_SET(r,f) (r).txlnswp2[0]=(((r).txlnswp2[0] & ~((uint32_t)0x3)) | (((uint32_t)f) & 0x3)) | (0x3 << 16)

/*
 * These macros can be used to access TXLNSWP2.
 *
 */
#define BCMI_VIPER_XGXS_READ_TXLNSWP2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TXLNSWP2r,(_r._txlnswp2))
#define BCMI_VIPER_XGXS_WRITE_TXLNSWP2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TXLNSWP2r,(_r._txlnswp2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_TXLNSWP2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TXLNSWP2r,(_r._txlnswp2))
#define BCMI_VIPER_XGXS_READLN_TXLNSWP2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_TXLNSWP2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._txlnswp2))
#define BCMI_VIPER_XGXS_WRITELN_TXLNSWP2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TXLNSWP2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._txlnswp2))
#define BCMI_VIPER_XGXS_WRITEALL_TXLNSWP2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_TXLNSWP2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._txlnswp2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define TXLNSWP2r BCMI_VIPER_XGXS_TXLNSWP2r
#define TXLNSWP2r_SIZE BCMI_VIPER_XGXS_TXLNSWP2r_SIZE
typedef BCMI_VIPER_XGXS_TXLNSWP2r_t TXLNSWP2r_t;
#define TXLNSWP2r_CLR BCMI_VIPER_XGXS_TXLNSWP2r_CLR
#define TXLNSWP2r_SET BCMI_VIPER_XGXS_TXLNSWP2r_SET
#define TXLNSWP2r_GET BCMI_VIPER_XGXS_TXLNSWP2r_GET
#define TXLNSWP2r_FORCE_TX_ANA_LNSWAP_SELf_GET BCMI_VIPER_XGXS_TXLNSWP2r_FORCE_TX_ANA_LNSWAP_SELf_GET
#define TXLNSWP2r_FORCE_TX_ANA_LNSWAP_SELf_SET BCMI_VIPER_XGXS_TXLNSWP2r_FORCE_TX_ANA_LNSWAP_SELf_SET
#define TXLNSWP2r_TX3_ANA_LNSWAP_SELf_GET BCMI_VIPER_XGXS_TXLNSWP2r_TX3_ANA_LNSWAP_SELf_GET
#define TXLNSWP2r_TX3_ANA_LNSWAP_SELf_SET BCMI_VIPER_XGXS_TXLNSWP2r_TX3_ANA_LNSWAP_SELf_SET
#define TXLNSWP2r_TX2_ANA_LNSWAP_SELf_GET BCMI_VIPER_XGXS_TXLNSWP2r_TX2_ANA_LNSWAP_SELf_GET
#define TXLNSWP2r_TX2_ANA_LNSWAP_SELf_SET BCMI_VIPER_XGXS_TXLNSWP2r_TX2_ANA_LNSWAP_SELf_SET
#define TXLNSWP2r_TX1_ANA_LNSWAP_SELf_GET BCMI_VIPER_XGXS_TXLNSWP2r_TX1_ANA_LNSWAP_SELf_GET
#define TXLNSWP2r_TX1_ANA_LNSWAP_SELf_SET BCMI_VIPER_XGXS_TXLNSWP2r_TX1_ANA_LNSWAP_SELf_SET
#define TXLNSWP2r_TX0_ANA_LNSWAP_SELf_GET BCMI_VIPER_XGXS_TXLNSWP2r_TX0_ANA_LNSWAP_SELf_GET
#define TXLNSWP2r_TX0_ANA_LNSWAP_SELf_SET BCMI_VIPER_XGXS_TXLNSWP2r_TX0_ANA_LNSWAP_SELf_SET
#define READ_TXLNSWP2r BCMI_VIPER_XGXS_READ_TXLNSWP2r
#define WRITE_TXLNSWP2r BCMI_VIPER_XGXS_WRITE_TXLNSWP2r
#define MODIFY_TXLNSWP2r BCMI_VIPER_XGXS_MODIFY_TXLNSWP2r
#define READLN_TXLNSWP2r BCMI_VIPER_XGXS_READLN_TXLNSWP2r
#define WRITELN_TXLNSWP2r BCMI_VIPER_XGXS_WRITELN_TXLNSWP2r
#define WRITEALL_TXLNSWP2r BCMI_VIPER_XGXS_WRITEALL_TXLNSWP2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_TXLNSWP2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RXLNSWP1
 * BLOCKS:   BLK8
 * REGADDR:  0x816b
 * DESC:     RX Lane Swap Control 1 Register
 * RESETVAL: 0xe4 (228)
 * ACCESS:   R/W
 * FIELDS:
 *     RX0_LNSWAP_SEL   RX ANALOG to DIGITAL Lane0 swap select
 *     RX1_LNSWAP_SEL   RX ANALOG to DIGITAL Lane1 swap select
 *     RX2_LNSWAP_SEL   RX ANALOG to DIGITAL Lane2 swap select
 *     RX3_LNSWAP_SEL   RX ANALOG to DIGITAL Lane3 swap select
 *     RFIFO_20TO20_PTR_SW_RST for 4 individual lane, software ptr_rst for rxfifo_20to20, when set to 1'b1, fifo ptr is reset.
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RXLNSWP1r (0x0000816b | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RXLNSWP1r_SIZE 4

/*
 * This structure should be used to declare and program RXLNSWP1.
 *
 */
typedef union BCMI_VIPER_XGXS_RXLNSWP1r_s {
	uint32_t v[1];
	uint32_t rxlnswp1[1];
	uint32_t _rxlnswp1;
} BCMI_VIPER_XGXS_RXLNSWP1r_t;

#define BCMI_VIPER_XGXS_RXLNSWP1r_CLR(r) (r).rxlnswp1[0] = 0
#define BCMI_VIPER_XGXS_RXLNSWP1r_SET(r,d) (r).rxlnswp1[0] = d
#define BCMI_VIPER_XGXS_RXLNSWP1r_GET(r) (r).rxlnswp1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RXLNSWP1r_RFIFO_20TO20_PTR_SW_RSTf_GET(r) ((((r).rxlnswp1[0]) >> 8) & 0xf)
#define BCMI_VIPER_XGXS_RXLNSWP1r_RFIFO_20TO20_PTR_SW_RSTf_SET(r,f) (r).rxlnswp1[0]=(((r).rxlnswp1[0] & ~((uint32_t)0xf << 8)) | ((((uint32_t)f) & 0xf) << 8)) | (15 << (16 + 8))
#define BCMI_VIPER_XGXS_RXLNSWP1r_RX3_LNSWAP_SELf_GET(r) ((((r).rxlnswp1[0]) >> 6) & 0x3)
#define BCMI_VIPER_XGXS_RXLNSWP1r_RX3_LNSWAP_SELf_SET(r,f) (r).rxlnswp1[0]=(((r).rxlnswp1[0] & ~((uint32_t)0x3 << 6)) | ((((uint32_t)f) & 0x3) << 6)) | (3 << (16 + 6))
#define BCMI_VIPER_XGXS_RXLNSWP1r_RX2_LNSWAP_SELf_GET(r) ((((r).rxlnswp1[0]) >> 4) & 0x3)
#define BCMI_VIPER_XGXS_RXLNSWP1r_RX2_LNSWAP_SELf_SET(r,f) (r).rxlnswp1[0]=(((r).rxlnswp1[0] & ~((uint32_t)0x3 << 4)) | ((((uint32_t)f) & 0x3) << 4)) | (3 << (16 + 4))
#define BCMI_VIPER_XGXS_RXLNSWP1r_RX1_LNSWAP_SELf_GET(r) ((((r).rxlnswp1[0]) >> 2) & 0x3)
#define BCMI_VIPER_XGXS_RXLNSWP1r_RX1_LNSWAP_SELf_SET(r,f) (r).rxlnswp1[0]=(((r).rxlnswp1[0] & ~((uint32_t)0x3 << 2)) | ((((uint32_t)f) & 0x3) << 2)) | (3 << (16 + 2))
#define BCMI_VIPER_XGXS_RXLNSWP1r_RX0_LNSWAP_SELf_GET(r) (((r).rxlnswp1[0]) & 0x3)
#define BCMI_VIPER_XGXS_RXLNSWP1r_RX0_LNSWAP_SELf_SET(r,f) (r).rxlnswp1[0]=(((r).rxlnswp1[0] & ~((uint32_t)0x3)) | (((uint32_t)f) & 0x3)) | (0x3 << 16)

/*
 * These macros can be used to access RXLNSWP1.
 *
 */
#define BCMI_VIPER_XGXS_READ_RXLNSWP1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RXLNSWP1r,(_r._rxlnswp1))
#define BCMI_VIPER_XGXS_WRITE_RXLNSWP1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RXLNSWP1r,(_r._rxlnswp1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RXLNSWP1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RXLNSWP1r,(_r._rxlnswp1))
#define BCMI_VIPER_XGXS_READLN_RXLNSWP1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RXLNSWP1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rxlnswp1))
#define BCMI_VIPER_XGXS_WRITELN_RXLNSWP1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RXLNSWP1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rxlnswp1))
#define BCMI_VIPER_XGXS_WRITEALL_RXLNSWP1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RXLNSWP1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rxlnswp1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RXLNSWP1r BCMI_VIPER_XGXS_RXLNSWP1r
#define RXLNSWP1r_SIZE BCMI_VIPER_XGXS_RXLNSWP1r_SIZE
typedef BCMI_VIPER_XGXS_RXLNSWP1r_t RXLNSWP1r_t;
#define RXLNSWP1r_CLR BCMI_VIPER_XGXS_RXLNSWP1r_CLR
#define RXLNSWP1r_SET BCMI_VIPER_XGXS_RXLNSWP1r_SET
#define RXLNSWP1r_GET BCMI_VIPER_XGXS_RXLNSWP1r_GET
#define RXLNSWP1r_RFIFO_20TO20_PTR_SW_RSTf_GET BCMI_VIPER_XGXS_RXLNSWP1r_RFIFO_20TO20_PTR_SW_RSTf_GET
#define RXLNSWP1r_RFIFO_20TO20_PTR_SW_RSTf_SET BCMI_VIPER_XGXS_RXLNSWP1r_RFIFO_20TO20_PTR_SW_RSTf_SET
#define RXLNSWP1r_RX3_LNSWAP_SELf_GET BCMI_VIPER_XGXS_RXLNSWP1r_RX3_LNSWAP_SELf_GET
#define RXLNSWP1r_RX3_LNSWAP_SELf_SET BCMI_VIPER_XGXS_RXLNSWP1r_RX3_LNSWAP_SELf_SET
#define RXLNSWP1r_RX2_LNSWAP_SELf_GET BCMI_VIPER_XGXS_RXLNSWP1r_RX2_LNSWAP_SELf_GET
#define RXLNSWP1r_RX2_LNSWAP_SELf_SET BCMI_VIPER_XGXS_RXLNSWP1r_RX2_LNSWAP_SELf_SET
#define RXLNSWP1r_RX1_LNSWAP_SELf_GET BCMI_VIPER_XGXS_RXLNSWP1r_RX1_LNSWAP_SELf_GET
#define RXLNSWP1r_RX1_LNSWAP_SELf_SET BCMI_VIPER_XGXS_RXLNSWP1r_RX1_LNSWAP_SELf_SET
#define RXLNSWP1r_RX0_LNSWAP_SELf_GET BCMI_VIPER_XGXS_RXLNSWP1r_RX0_LNSWAP_SELf_GET
#define RXLNSWP1r_RX0_LNSWAP_SELf_SET BCMI_VIPER_XGXS_RXLNSWP1r_RX0_LNSWAP_SELf_SET
#define READ_RXLNSWP1r BCMI_VIPER_XGXS_READ_RXLNSWP1r
#define WRITE_RXLNSWP1r BCMI_VIPER_XGXS_WRITE_RXLNSWP1r
#define MODIFY_RXLNSWP1r BCMI_VIPER_XGXS_MODIFY_RXLNSWP1r
#define READLN_RXLNSWP1r BCMI_VIPER_XGXS_READLN_RXLNSWP1r
#define WRITELN_RXLNSWP1r BCMI_VIPER_XGXS_WRITELN_RXLNSWP1r
#define WRITEALL_RXLNSWP1r BCMI_VIPER_XGXS_WRITEALL_RXLNSWP1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RXLNSWP1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RXLNSWP2
 * BLOCKS:   BLK8
 * REGADDR:  0x816c
 * DESC:     RX Lane Swap Control 2 Register
 * RESETVAL: 0xe4 (228)
 * ACCESS:   R/W
 * FIELDS:
 *     RX0_DIG_LNSWAP_SEL RX DIGITAL to ANALOG Lane0 swap select
 *     RX1_DIG_LNSWAP_SEL RX DIGITAL to ANALOG Lane1 swap select
 *     RX2_DIG_LNSWAP_SEL RX DIGITAL to ANALOG Lane2 swap select
 *     RX3_DIG_LNSWAP_SEL RX DIGITAL to ANALOG Lane3 swap select
 *     FORCE_RX_DIG_LNSWAP_SEL Force DIGITAL to ANALOG mux-selects
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RXLNSWP2r (0x0000816c | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RXLNSWP2r_SIZE 4

/*
 * This structure should be used to declare and program RXLNSWP2.
 *
 */
typedef union BCMI_VIPER_XGXS_RXLNSWP2r_s {
	uint32_t v[1];
	uint32_t rxlnswp2[1];
	uint32_t _rxlnswp2;
} BCMI_VIPER_XGXS_RXLNSWP2r_t;

#define BCMI_VIPER_XGXS_RXLNSWP2r_CLR(r) (r).rxlnswp2[0] = 0
#define BCMI_VIPER_XGXS_RXLNSWP2r_SET(r,d) (r).rxlnswp2[0] = d
#define BCMI_VIPER_XGXS_RXLNSWP2r_GET(r) (r).rxlnswp2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RXLNSWP2r_FORCE_RX_DIG_LNSWAP_SELf_GET(r) ((((r).rxlnswp2[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_RXLNSWP2r_FORCE_RX_DIG_LNSWAP_SELf_SET(r,f) (r).rxlnswp2[0]=(((r).rxlnswp2[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_RXLNSWP2r_RX3_DIG_LNSWAP_SELf_GET(r) ((((r).rxlnswp2[0]) >> 6) & 0x3)
#define BCMI_VIPER_XGXS_RXLNSWP2r_RX3_DIG_LNSWAP_SELf_SET(r,f) (r).rxlnswp2[0]=(((r).rxlnswp2[0] & ~((uint32_t)0x3 << 6)) | ((((uint32_t)f) & 0x3) << 6)) | (3 << (16 + 6))
#define BCMI_VIPER_XGXS_RXLNSWP2r_RX2_DIG_LNSWAP_SELf_GET(r) ((((r).rxlnswp2[0]) >> 4) & 0x3)
#define BCMI_VIPER_XGXS_RXLNSWP2r_RX2_DIG_LNSWAP_SELf_SET(r,f) (r).rxlnswp2[0]=(((r).rxlnswp2[0] & ~((uint32_t)0x3 << 4)) | ((((uint32_t)f) & 0x3) << 4)) | (3 << (16 + 4))
#define BCMI_VIPER_XGXS_RXLNSWP2r_RX1_DIG_LNSWAP_SELf_GET(r) ((((r).rxlnswp2[0]) >> 2) & 0x3)
#define BCMI_VIPER_XGXS_RXLNSWP2r_RX1_DIG_LNSWAP_SELf_SET(r,f) (r).rxlnswp2[0]=(((r).rxlnswp2[0] & ~((uint32_t)0x3 << 2)) | ((((uint32_t)f) & 0x3) << 2)) | (3 << (16 + 2))
#define BCMI_VIPER_XGXS_RXLNSWP2r_RX0_DIG_LNSWAP_SELf_GET(r) (((r).rxlnswp2[0]) & 0x3)
#define BCMI_VIPER_XGXS_RXLNSWP2r_RX0_DIG_LNSWAP_SELf_SET(r,f) (r).rxlnswp2[0]=(((r).rxlnswp2[0] & ~((uint32_t)0x3)) | (((uint32_t)f) & 0x3)) | (0x3 << 16)

/*
 * These macros can be used to access RXLNSWP2.
 *
 */
#define BCMI_VIPER_XGXS_READ_RXLNSWP2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RXLNSWP2r,(_r._rxlnswp2))
#define BCMI_VIPER_XGXS_WRITE_RXLNSWP2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RXLNSWP2r,(_r._rxlnswp2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RXLNSWP2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RXLNSWP2r,(_r._rxlnswp2))
#define BCMI_VIPER_XGXS_READLN_RXLNSWP2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RXLNSWP2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rxlnswp2))
#define BCMI_VIPER_XGXS_WRITELN_RXLNSWP2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RXLNSWP2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rxlnswp2))
#define BCMI_VIPER_XGXS_WRITEALL_RXLNSWP2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RXLNSWP2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rxlnswp2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RXLNSWP2r BCMI_VIPER_XGXS_RXLNSWP2r
#define RXLNSWP2r_SIZE BCMI_VIPER_XGXS_RXLNSWP2r_SIZE
typedef BCMI_VIPER_XGXS_RXLNSWP2r_t RXLNSWP2r_t;
#define RXLNSWP2r_CLR BCMI_VIPER_XGXS_RXLNSWP2r_CLR
#define RXLNSWP2r_SET BCMI_VIPER_XGXS_RXLNSWP2r_SET
#define RXLNSWP2r_GET BCMI_VIPER_XGXS_RXLNSWP2r_GET
#define RXLNSWP2r_FORCE_RX_DIG_LNSWAP_SELf_GET BCMI_VIPER_XGXS_RXLNSWP2r_FORCE_RX_DIG_LNSWAP_SELf_GET
#define RXLNSWP2r_FORCE_RX_DIG_LNSWAP_SELf_SET BCMI_VIPER_XGXS_RXLNSWP2r_FORCE_RX_DIG_LNSWAP_SELf_SET
#define RXLNSWP2r_RX3_DIG_LNSWAP_SELf_GET BCMI_VIPER_XGXS_RXLNSWP2r_RX3_DIG_LNSWAP_SELf_GET
#define RXLNSWP2r_RX3_DIG_LNSWAP_SELf_SET BCMI_VIPER_XGXS_RXLNSWP2r_RX3_DIG_LNSWAP_SELf_SET
#define RXLNSWP2r_RX2_DIG_LNSWAP_SELf_GET BCMI_VIPER_XGXS_RXLNSWP2r_RX2_DIG_LNSWAP_SELf_GET
#define RXLNSWP2r_RX2_DIG_LNSWAP_SELf_SET BCMI_VIPER_XGXS_RXLNSWP2r_RX2_DIG_LNSWAP_SELf_SET
#define RXLNSWP2r_RX1_DIG_LNSWAP_SELf_GET BCMI_VIPER_XGXS_RXLNSWP2r_RX1_DIG_LNSWAP_SELf_GET
#define RXLNSWP2r_RX1_DIG_LNSWAP_SELf_SET BCMI_VIPER_XGXS_RXLNSWP2r_RX1_DIG_LNSWAP_SELf_SET
#define RXLNSWP2r_RX0_DIG_LNSWAP_SELf_GET BCMI_VIPER_XGXS_RXLNSWP2r_RX0_DIG_LNSWAP_SELf_GET
#define RXLNSWP2r_RX0_DIG_LNSWAP_SELf_SET BCMI_VIPER_XGXS_RXLNSWP2r_RX0_DIG_LNSWAP_SELf_SET
#define READ_RXLNSWP2r BCMI_VIPER_XGXS_READ_RXLNSWP2r
#define WRITE_RXLNSWP2r BCMI_VIPER_XGXS_WRITE_RXLNSWP2r
#define MODIFY_RXLNSWP2r BCMI_VIPER_XGXS_MODIFY_RXLNSWP2r
#define READLN_RXLNSWP2r BCMI_VIPER_XGXS_READLN_RXLNSWP2r
#define WRITELN_RXLNSWP2r BCMI_VIPER_XGXS_WRITELN_RXLNSWP2r
#define WRITEALL_RXLNSWP2r BCMI_VIPER_XGXS_WRITEALL_RXLNSWP2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RXLNSWP2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL2_STAT0
 * BLOCKS:   PLL2
 * REGADDR:  0x8180
 * DESC:     Analog PLL Status0 Register
 * RESETVAL: 0x422 (1058)
 * ACCESS:   R/O
 * FIELDS:
 *     CAL_ERROR        
 *     CAL_VALID        
 *     CAL_STATE        
 *     VCO_RANGE        
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL2_STAT0r (0x00008180 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL2_STAT0r_SIZE 4

/*
 * This structure should be used to declare and program PLL2_STAT0.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL2_STAT0r_s {
	uint32_t v[1];
	uint32_t pll2_stat0[1];
	uint32_t _pll2_stat0;
} BCMI_VIPER_XGXS_PLL2_STAT0r_t;

#define BCMI_VIPER_XGXS_PLL2_STAT0r_CLR(r) (r).pll2_stat0[0] = 0
#define BCMI_VIPER_XGXS_PLL2_STAT0r_SET(r,d) (r).pll2_stat0[0] = d
#define BCMI_VIPER_XGXS_PLL2_STAT0r_GET(r) (r).pll2_stat0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL2_STAT0r_VCO_RANGEf_GET(r) ((((r).pll2_stat0[0]) >> 6) & 0x7f)
#define BCMI_VIPER_XGXS_PLL2_STAT0r_VCO_RANGEf_SET(r,f) (r).pll2_stat0[0]=(((r).pll2_stat0[0] & ~((uint32_t)0x7f << 6)) | ((((uint32_t)f) & 0x7f) << 6)) | (127 << (16 + 6))
#define BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_STATEf_GET(r) ((((r).pll2_stat0[0]) >> 2) & 0xf)
#define BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_STATEf_SET(r,f) (r).pll2_stat0[0]=(((r).pll2_stat0[0] & ~((uint32_t)0xf << 2)) | ((((uint32_t)f) & 0xf) << 2)) | (15 << (16 + 2))
#define BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_VALIDf_GET(r) ((((r).pll2_stat0[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_VALIDf_SET(r,f) (r).pll2_stat0[0]=(((r).pll2_stat0[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_ERRORf_GET(r) (((r).pll2_stat0[0]) & 0x1)
#define BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_ERRORf_SET(r,f) (r).pll2_stat0[0]=(((r).pll2_stat0[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access PLL2_STAT0.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL2_STAT0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_STAT0r,(_r._pll2_stat0))
#define BCMI_VIPER_XGXS_WRITE_PLL2_STAT0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_STAT0r,(_r._pll2_stat0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL2_STAT0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_STAT0r,(_r._pll2_stat0))
#define BCMI_VIPER_XGXS_READLN_PLL2_STAT0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_STAT0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_stat0))
#define BCMI_VIPER_XGXS_WRITELN_PLL2_STAT0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_STAT0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_stat0))
#define BCMI_VIPER_XGXS_WRITEALL_PLL2_STAT0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_STAT0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll2_stat0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL2_STAT0r BCMI_VIPER_XGXS_PLL2_STAT0r
#define PLL2_STAT0r_SIZE BCMI_VIPER_XGXS_PLL2_STAT0r_SIZE
typedef BCMI_VIPER_XGXS_PLL2_STAT0r_t PLL2_STAT0r_t;
#define PLL2_STAT0r_CLR BCMI_VIPER_XGXS_PLL2_STAT0r_CLR
#define PLL2_STAT0r_SET BCMI_VIPER_XGXS_PLL2_STAT0r_SET
#define PLL2_STAT0r_GET BCMI_VIPER_XGXS_PLL2_STAT0r_GET
#define PLL2_STAT0r_VCO_RANGEf_GET BCMI_VIPER_XGXS_PLL2_STAT0r_VCO_RANGEf_GET
#define PLL2_STAT0r_VCO_RANGEf_SET BCMI_VIPER_XGXS_PLL2_STAT0r_VCO_RANGEf_SET
#define PLL2_STAT0r_CAL_STATEf_GET BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_STATEf_GET
#define PLL2_STAT0r_CAL_STATEf_SET BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_STATEf_SET
#define PLL2_STAT0r_CAL_VALIDf_GET BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_VALIDf_GET
#define PLL2_STAT0r_CAL_VALIDf_SET BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_VALIDf_SET
#define PLL2_STAT0r_CAL_ERRORf_GET BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_ERRORf_GET
#define PLL2_STAT0r_CAL_ERRORf_SET BCMI_VIPER_XGXS_PLL2_STAT0r_CAL_ERRORf_SET
#define READ_PLL2_STAT0r BCMI_VIPER_XGXS_READ_PLL2_STAT0r
#define WRITE_PLL2_STAT0r BCMI_VIPER_XGXS_WRITE_PLL2_STAT0r
#define MODIFY_PLL2_STAT0r BCMI_VIPER_XGXS_MODIFY_PLL2_STAT0r
#define READLN_PLL2_STAT0r BCMI_VIPER_XGXS_READLN_PLL2_STAT0r
#define WRITELN_PLL2_STAT0r BCMI_VIPER_XGXS_WRITELN_PLL2_STAT0r
#define WRITEALL_PLL2_STAT0r BCMI_VIPER_XGXS_WRITEALL_PLL2_STAT0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL2_STAT0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL2_CTL1
 * BLOCKS:   PLL2
 * REGADDR:  0x8181
 * DESC:     Analog PLL Control1 Register
 * RESETVAL: 0x300 (768)
 * ACCESS:   R/W
 * FIELDS:
 *     EXT_RANGE        
 *     CAL_TH           
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL2_CTL1r (0x00008181 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL2_CTL1r_SIZE 4

/*
 * This structure should be used to declare and program PLL2_CTL1.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL2_CTL1r_s {
	uint32_t v[1];
	uint32_t pll2_ctl1[1];
	uint32_t _pll2_ctl1;
} BCMI_VIPER_XGXS_PLL2_CTL1r_t;

#define BCMI_VIPER_XGXS_PLL2_CTL1r_CLR(r) (r).pll2_ctl1[0] = 0
#define BCMI_VIPER_XGXS_PLL2_CTL1r_SET(r,d) (r).pll2_ctl1[0] = d
#define BCMI_VIPER_XGXS_PLL2_CTL1r_GET(r) (r).pll2_ctl1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL2_CTL1r_CAL_THf_GET(r) ((((r).pll2_ctl1[0]) >> 7) & 0xf)
#define BCMI_VIPER_XGXS_PLL2_CTL1r_CAL_THf_SET(r,f) (r).pll2_ctl1[0]=(((r).pll2_ctl1[0] & ~((uint32_t)0xf << 7)) | ((((uint32_t)f) & 0xf) << 7)) | (15 << (16 + 7))
#define BCMI_VIPER_XGXS_PLL2_CTL1r_EXT_RANGEf_GET(r) (((r).pll2_ctl1[0]) & 0x7f)
#define BCMI_VIPER_XGXS_PLL2_CTL1r_EXT_RANGEf_SET(r,f) (r).pll2_ctl1[0]=(((r).pll2_ctl1[0] & ~((uint32_t)0x7f)) | (((uint32_t)f) & 0x7f)) | (0x7f << 16)

/*
 * These macros can be used to access PLL2_CTL1.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL2_CTL1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL1r,(_r._pll2_ctl1))
#define BCMI_VIPER_XGXS_WRITE_PLL2_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL1r,(_r._pll2_ctl1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL2_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL1r,(_r._pll2_ctl1))
#define BCMI_VIPER_XGXS_READLN_PLL2_CTL1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl1))
#define BCMI_VIPER_XGXS_WRITELN_PLL2_CTL1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl1))
#define BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll2_ctl1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL2_CTL1r BCMI_VIPER_XGXS_PLL2_CTL1r
#define PLL2_CTL1r_SIZE BCMI_VIPER_XGXS_PLL2_CTL1r_SIZE
typedef BCMI_VIPER_XGXS_PLL2_CTL1r_t PLL2_CTL1r_t;
#define PLL2_CTL1r_CLR BCMI_VIPER_XGXS_PLL2_CTL1r_CLR
#define PLL2_CTL1r_SET BCMI_VIPER_XGXS_PLL2_CTL1r_SET
#define PLL2_CTL1r_GET BCMI_VIPER_XGXS_PLL2_CTL1r_GET
#define PLL2_CTL1r_CAL_THf_GET BCMI_VIPER_XGXS_PLL2_CTL1r_CAL_THf_GET
#define PLL2_CTL1r_CAL_THf_SET BCMI_VIPER_XGXS_PLL2_CTL1r_CAL_THf_SET
#define PLL2_CTL1r_EXT_RANGEf_GET BCMI_VIPER_XGXS_PLL2_CTL1r_EXT_RANGEf_GET
#define PLL2_CTL1r_EXT_RANGEf_SET BCMI_VIPER_XGXS_PLL2_CTL1r_EXT_RANGEf_SET
#define READ_PLL2_CTL1r BCMI_VIPER_XGXS_READ_PLL2_CTL1r
#define WRITE_PLL2_CTL1r BCMI_VIPER_XGXS_WRITE_PLL2_CTL1r
#define MODIFY_PLL2_CTL1r BCMI_VIPER_XGXS_MODIFY_PLL2_CTL1r
#define READLN_PLL2_CTL1r BCMI_VIPER_XGXS_READLN_PLL2_CTL1r
#define WRITELN_PLL2_CTL1r BCMI_VIPER_XGXS_WRITELN_PLL2_CTL1r
#define WRITEALL_PLL2_CTL1r BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL2_CTL1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL2_CTL2
 * BLOCKS:   PLL2
 * REGADDR:  0x8182
 * DESC:     Analog PLL Control2 Register
 * RESETVAL: 0x2000 (8192)
 * ACCESS:   R/W
 * FIELDS:
 *     RANGE_OVRD_VAL   
 *     RANGE_OVRD       
 *     RANGE_DFS        
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL2_CTL2r (0x00008182 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL2_CTL2r_SIZE 4

/*
 * This structure should be used to declare and program PLL2_CTL2.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL2_CTL2r_s {
	uint32_t v[1];
	uint32_t pll2_ctl2[1];
	uint32_t _pll2_ctl2;
} BCMI_VIPER_XGXS_PLL2_CTL2r_t;

#define BCMI_VIPER_XGXS_PLL2_CTL2r_CLR(r) (r).pll2_ctl2[0] = 0
#define BCMI_VIPER_XGXS_PLL2_CTL2r_SET(r,d) (r).pll2_ctl2[0] = d
#define BCMI_VIPER_XGXS_PLL2_CTL2r_GET(r) (r).pll2_ctl2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_DFSf_GET(r) ((((r).pll2_ctl2[0]) >> 8) & 0x7f)
#define BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_DFSf_SET(r,f) (r).pll2_ctl2[0]=(((r).pll2_ctl2[0] & ~((uint32_t)0x7f << 8)) | ((((uint32_t)f) & 0x7f) << 8)) | (127 << (16 + 8))
#define BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_OVRDf_GET(r) ((((r).pll2_ctl2[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_OVRDf_SET(r,f) (r).pll2_ctl2[0]=(((r).pll2_ctl2[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_OVRD_VALf_GET(r) (((r).pll2_ctl2[0]) & 0x7f)
#define BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_OVRD_VALf_SET(r,f) (r).pll2_ctl2[0]=(((r).pll2_ctl2[0] & ~((uint32_t)0x7f)) | (((uint32_t)f) & 0x7f)) | (0x7f << 16)

/*
 * These macros can be used to access PLL2_CTL2.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL2_CTL2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL2r,(_r._pll2_ctl2))
#define BCMI_VIPER_XGXS_WRITE_PLL2_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL2r,(_r._pll2_ctl2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL2_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL2r,(_r._pll2_ctl2))
#define BCMI_VIPER_XGXS_READLN_PLL2_CTL2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl2))
#define BCMI_VIPER_XGXS_WRITELN_PLL2_CTL2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl2))
#define BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll2_ctl2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL2_CTL2r BCMI_VIPER_XGXS_PLL2_CTL2r
#define PLL2_CTL2r_SIZE BCMI_VIPER_XGXS_PLL2_CTL2r_SIZE
typedef BCMI_VIPER_XGXS_PLL2_CTL2r_t PLL2_CTL2r_t;
#define PLL2_CTL2r_CLR BCMI_VIPER_XGXS_PLL2_CTL2r_CLR
#define PLL2_CTL2r_SET BCMI_VIPER_XGXS_PLL2_CTL2r_SET
#define PLL2_CTL2r_GET BCMI_VIPER_XGXS_PLL2_CTL2r_GET
#define PLL2_CTL2r_RANGE_DFSf_GET BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_DFSf_GET
#define PLL2_CTL2r_RANGE_DFSf_SET BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_DFSf_SET
#define PLL2_CTL2r_RANGE_OVRDf_GET BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_OVRDf_GET
#define PLL2_CTL2r_RANGE_OVRDf_SET BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_OVRDf_SET
#define PLL2_CTL2r_RANGE_OVRD_VALf_GET BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_OVRD_VALf_GET
#define PLL2_CTL2r_RANGE_OVRD_VALf_SET BCMI_VIPER_XGXS_PLL2_CTL2r_RANGE_OVRD_VALf_SET
#define READ_PLL2_CTL2r BCMI_VIPER_XGXS_READ_PLL2_CTL2r
#define WRITE_PLL2_CTL2r BCMI_VIPER_XGXS_WRITE_PLL2_CTL2r
#define MODIFY_PLL2_CTL2r BCMI_VIPER_XGXS_MODIFY_PLL2_CTL2r
#define READLN_PLL2_CTL2r BCMI_VIPER_XGXS_READLN_PLL2_CTL2r
#define WRITELN_PLL2_CTL2r BCMI_VIPER_XGXS_WRITELN_PLL2_CTL2r
#define WRITEALL_PLL2_CTL2r BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL2_CTL2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL2_CTL3
 * BLOCKS:   PLL2
 * REGADDR:  0x8183
 * DESC:     Analog PLL Control3 Register
 * RESETVAL: 0x1e84 (7812)
 * ACCESS:   R/W
 * FIELDS:
 *     CALIB_CAP_CHARGE_TIME 50us based on a 156.25MHz refclkuse 09C4h if using a 50MHz refclk
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL2_CTL3r (0x00008183 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL2_CTL3r_SIZE 4

/*
 * This structure should be used to declare and program PLL2_CTL3.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL2_CTL3r_s {
	uint32_t v[1];
	uint32_t pll2_ctl3[1];
	uint32_t _pll2_ctl3;
} BCMI_VIPER_XGXS_PLL2_CTL3r_t;

#define BCMI_VIPER_XGXS_PLL2_CTL3r_CLR(r) (r).pll2_ctl3[0] = 0
#define BCMI_VIPER_XGXS_PLL2_CTL3r_SET(r,d) (r).pll2_ctl3[0] = d
#define BCMI_VIPER_XGXS_PLL2_CTL3r_GET(r) (r).pll2_ctl3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL2_CTL3r_CALIB_CAP_CHARGE_TIMEf_GET(r) (((r).pll2_ctl3[0]) & 0xffff)
#define BCMI_VIPER_XGXS_PLL2_CTL3r_CALIB_CAP_CHARGE_TIMEf_SET(r,f) (r).pll2_ctl3[0]=(((r).pll2_ctl3[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access PLL2_CTL3.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL2_CTL3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL3r,(_r._pll2_ctl3))
#define BCMI_VIPER_XGXS_WRITE_PLL2_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL3r,(_r._pll2_ctl3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL2_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL3r,(_r._pll2_ctl3))
#define BCMI_VIPER_XGXS_READLN_PLL2_CTL3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl3))
#define BCMI_VIPER_XGXS_WRITELN_PLL2_CTL3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl3))
#define BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll2_ctl3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL2_CTL3r BCMI_VIPER_XGXS_PLL2_CTL3r
#define PLL2_CTL3r_SIZE BCMI_VIPER_XGXS_PLL2_CTL3r_SIZE
typedef BCMI_VIPER_XGXS_PLL2_CTL3r_t PLL2_CTL3r_t;
#define PLL2_CTL3r_CLR BCMI_VIPER_XGXS_PLL2_CTL3r_CLR
#define PLL2_CTL3r_SET BCMI_VIPER_XGXS_PLL2_CTL3r_SET
#define PLL2_CTL3r_GET BCMI_VIPER_XGXS_PLL2_CTL3r_GET
#define PLL2_CTL3r_CALIB_CAP_CHARGE_TIMEf_GET BCMI_VIPER_XGXS_PLL2_CTL3r_CALIB_CAP_CHARGE_TIMEf_GET
#define PLL2_CTL3r_CALIB_CAP_CHARGE_TIMEf_SET BCMI_VIPER_XGXS_PLL2_CTL3r_CALIB_CAP_CHARGE_TIMEf_SET
#define READ_PLL2_CTL3r BCMI_VIPER_XGXS_READ_PLL2_CTL3r
#define WRITE_PLL2_CTL3r BCMI_VIPER_XGXS_WRITE_PLL2_CTL3r
#define MODIFY_PLL2_CTL3r BCMI_VIPER_XGXS_MODIFY_PLL2_CTL3r
#define READLN_PLL2_CTL3r BCMI_VIPER_XGXS_READLN_PLL2_CTL3r
#define WRITELN_PLL2_CTL3r BCMI_VIPER_XGXS_WRITELN_PLL2_CTL3r
#define WRITEALL_PLL2_CTL3r BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL2_CTL3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL2_CTL4
 * BLOCKS:   PLL2
 * REGADDR:  0x8184
 * DESC:     Analog PLL Control4 Register
 * RESETVAL: 0x61a (1562)
 * ACCESS:   R/W
 * FIELDS:
 *     CALIB_DELAY_TIME 10us based on a 156.25MHz refclkuse 01F4h if using a 50MHz refclk
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL2_CTL4r (0x00008184 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL2_CTL4r_SIZE 4

/*
 * This structure should be used to declare and program PLL2_CTL4.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL2_CTL4r_s {
	uint32_t v[1];
	uint32_t pll2_ctl4[1];
	uint32_t _pll2_ctl4;
} BCMI_VIPER_XGXS_PLL2_CTL4r_t;

#define BCMI_VIPER_XGXS_PLL2_CTL4r_CLR(r) (r).pll2_ctl4[0] = 0
#define BCMI_VIPER_XGXS_PLL2_CTL4r_SET(r,d) (r).pll2_ctl4[0] = d
#define BCMI_VIPER_XGXS_PLL2_CTL4r_GET(r) (r).pll2_ctl4[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL2_CTL4r_CALIB_DELAY_TIMEf_GET(r) (((r).pll2_ctl4[0]) & 0xffff)
#define BCMI_VIPER_XGXS_PLL2_CTL4r_CALIB_DELAY_TIMEf_SET(r,f) (r).pll2_ctl4[0]=(((r).pll2_ctl4[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access PLL2_CTL4.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL2_CTL4r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL4r,(_r._pll2_ctl4))
#define BCMI_VIPER_XGXS_WRITE_PLL2_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL4r,(_r._pll2_ctl4)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL2_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL4r,(_r._pll2_ctl4))
#define BCMI_VIPER_XGXS_READLN_PLL2_CTL4r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl4))
#define BCMI_VIPER_XGXS_WRITELN_PLL2_CTL4r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl4))
#define BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll2_ctl4))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL2_CTL4r BCMI_VIPER_XGXS_PLL2_CTL4r
#define PLL2_CTL4r_SIZE BCMI_VIPER_XGXS_PLL2_CTL4r_SIZE
typedef BCMI_VIPER_XGXS_PLL2_CTL4r_t PLL2_CTL4r_t;
#define PLL2_CTL4r_CLR BCMI_VIPER_XGXS_PLL2_CTL4r_CLR
#define PLL2_CTL4r_SET BCMI_VIPER_XGXS_PLL2_CTL4r_SET
#define PLL2_CTL4r_GET BCMI_VIPER_XGXS_PLL2_CTL4r_GET
#define PLL2_CTL4r_CALIB_DELAY_TIMEf_GET BCMI_VIPER_XGXS_PLL2_CTL4r_CALIB_DELAY_TIMEf_GET
#define PLL2_CTL4r_CALIB_DELAY_TIMEf_SET BCMI_VIPER_XGXS_PLL2_CTL4r_CALIB_DELAY_TIMEf_SET
#define READ_PLL2_CTL4r BCMI_VIPER_XGXS_READ_PLL2_CTL4r
#define WRITE_PLL2_CTL4r BCMI_VIPER_XGXS_WRITE_PLL2_CTL4r
#define MODIFY_PLL2_CTL4r BCMI_VIPER_XGXS_MODIFY_PLL2_CTL4r
#define READLN_PLL2_CTL4r BCMI_VIPER_XGXS_READLN_PLL2_CTL4r
#define WRITELN_PLL2_CTL4r BCMI_VIPER_XGXS_WRITELN_PLL2_CTL4r
#define WRITEALL_PLL2_CTL4r BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL4r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL2_CTL4r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL2_CTL5
 * BLOCKS:   PLL2
 * REGADDR:  0x8185
 * DESC:     Analog PLL Control5 Register
 * RESETVAL: 0x3d09 (15625)
 * ACCESS:   R/W
 * FIELDS:
 *     CALIB_STEP_TIME  100us based on a 156.25MHz refclkuse 1388h if using a 50MHz refclk
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL2_CTL5r (0x00008185 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL2_CTL5r_SIZE 4

/*
 * This structure should be used to declare and program PLL2_CTL5.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL2_CTL5r_s {
	uint32_t v[1];
	uint32_t pll2_ctl5[1];
	uint32_t _pll2_ctl5;
} BCMI_VIPER_XGXS_PLL2_CTL5r_t;

#define BCMI_VIPER_XGXS_PLL2_CTL5r_CLR(r) (r).pll2_ctl5[0] = 0
#define BCMI_VIPER_XGXS_PLL2_CTL5r_SET(r,d) (r).pll2_ctl5[0] = d
#define BCMI_VIPER_XGXS_PLL2_CTL5r_GET(r) (r).pll2_ctl5[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL2_CTL5r_CALIB_STEP_TIMEf_GET(r) (((r).pll2_ctl5[0]) & 0xffff)
#define BCMI_VIPER_XGXS_PLL2_CTL5r_CALIB_STEP_TIMEf_SET(r,f) (r).pll2_ctl5[0]=(((r).pll2_ctl5[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access PLL2_CTL5.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL2_CTL5r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL5r,(_r._pll2_ctl5))
#define BCMI_VIPER_XGXS_WRITE_PLL2_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL5r,(_r._pll2_ctl5)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL2_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL5r,(_r._pll2_ctl5))
#define BCMI_VIPER_XGXS_READLN_PLL2_CTL5r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl5))
#define BCMI_VIPER_XGXS_WRITELN_PLL2_CTL5r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl5))
#define BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll2_ctl5))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL2_CTL5r BCMI_VIPER_XGXS_PLL2_CTL5r
#define PLL2_CTL5r_SIZE BCMI_VIPER_XGXS_PLL2_CTL5r_SIZE
typedef BCMI_VIPER_XGXS_PLL2_CTL5r_t PLL2_CTL5r_t;
#define PLL2_CTL5r_CLR BCMI_VIPER_XGXS_PLL2_CTL5r_CLR
#define PLL2_CTL5r_SET BCMI_VIPER_XGXS_PLL2_CTL5r_SET
#define PLL2_CTL5r_GET BCMI_VIPER_XGXS_PLL2_CTL5r_GET
#define PLL2_CTL5r_CALIB_STEP_TIMEf_GET BCMI_VIPER_XGXS_PLL2_CTL5r_CALIB_STEP_TIMEf_GET
#define PLL2_CTL5r_CALIB_STEP_TIMEf_SET BCMI_VIPER_XGXS_PLL2_CTL5r_CALIB_STEP_TIMEf_SET
#define READ_PLL2_CTL5r BCMI_VIPER_XGXS_READ_PLL2_CTL5r
#define WRITE_PLL2_CTL5r BCMI_VIPER_XGXS_WRITE_PLL2_CTL5r
#define MODIFY_PLL2_CTL5r BCMI_VIPER_XGXS_MODIFY_PLL2_CTL5r
#define READLN_PLL2_CTL5r BCMI_VIPER_XGXS_READLN_PLL2_CTL5r
#define WRITELN_PLL2_CTL5r BCMI_VIPER_XGXS_WRITELN_PLL2_CTL5r
#define WRITEALL_PLL2_CTL5r BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL5r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL2_CTL5r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  PLL2_CTL6
 * BLOCKS:   PLL2
 * REGADDR:  0x8186
 * DESC:     Analog PLL Control6 Register
 * RESETVAL: 0x38 (56)
 * ACCESS:   R/W
 * FIELDS:
 *     DFE0_VCOCAL_VALID_OVRD_VAL 
 *     DFE0_VCOCAL_VALID_OVRD 
 *     DFE0_CALIB_SEARCH_BIT 
 *     DFE0_HALFSTEP_EN 
 *     DFE0_EN_CALIB_N  
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_PLL2_CTL6r (0x00008186 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_PLL2_CTL6r_SIZE 4

/*
 * This structure should be used to declare and program PLL2_CTL6.
 *
 */
typedef union BCMI_VIPER_XGXS_PLL2_CTL6r_s {
	uint32_t v[1];
	uint32_t pll2_ctl6[1];
	uint32_t _pll2_ctl6;
} BCMI_VIPER_XGXS_PLL2_CTL6r_t;

#define BCMI_VIPER_XGXS_PLL2_CTL6r_CLR(r) (r).pll2_ctl6[0] = 0
#define BCMI_VIPER_XGXS_PLL2_CTL6r_SET(r,d) (r).pll2_ctl6[0] = d
#define BCMI_VIPER_XGXS_PLL2_CTL6r_GET(r) (r).pll2_ctl6[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_EN_CALIB_Nf_GET(r) ((((r).pll2_ctl6[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_EN_CALIB_Nf_SET(r,f) (r).pll2_ctl6[0]=(((r).pll2_ctl6[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_HALFSTEP_ENf_GET(r) ((((r).pll2_ctl6[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_HALFSTEP_ENf_SET(r,f) (r).pll2_ctl6[0]=(((r).pll2_ctl6[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_CALIB_SEARCH_BITf_GET(r) ((((r).pll2_ctl6[0]) >> 2) & 0x7)
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_CALIB_SEARCH_BITf_SET(r,f) (r).pll2_ctl6[0]=(((r).pll2_ctl6[0] & ~((uint32_t)0x7 << 2)) | ((((uint32_t)f) & 0x7) << 2)) | (7 << (16 + 2))
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRDf_GET(r) ((((r).pll2_ctl6[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRDf_SET(r,f) (r).pll2_ctl6[0]=(((r).pll2_ctl6[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRD_VALf_GET(r) (((r).pll2_ctl6[0]) & 0x1)
#define BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRD_VALf_SET(r,f) (r).pll2_ctl6[0]=(((r).pll2_ctl6[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access PLL2_CTL6.
 *
 */
#define BCMI_VIPER_XGXS_READ_PLL2_CTL6r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL6r,(_r._pll2_ctl6))
#define BCMI_VIPER_XGXS_WRITE_PLL2_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL6r,(_r._pll2_ctl6)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_PLL2_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL6r,(_r._pll2_ctl6))
#define BCMI_VIPER_XGXS_READLN_PLL2_CTL6r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_PLL2_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl6))
#define BCMI_VIPER_XGXS_WRITELN_PLL2_CTL6r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._pll2_ctl6))
#define BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_PLL2_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._pll2_ctl6))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define PLL2_CTL6r BCMI_VIPER_XGXS_PLL2_CTL6r
#define PLL2_CTL6r_SIZE BCMI_VIPER_XGXS_PLL2_CTL6r_SIZE
typedef BCMI_VIPER_XGXS_PLL2_CTL6r_t PLL2_CTL6r_t;
#define PLL2_CTL6r_CLR BCMI_VIPER_XGXS_PLL2_CTL6r_CLR
#define PLL2_CTL6r_SET BCMI_VIPER_XGXS_PLL2_CTL6r_SET
#define PLL2_CTL6r_GET BCMI_VIPER_XGXS_PLL2_CTL6r_GET
#define PLL2_CTL6r_DFE0_EN_CALIB_Nf_GET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_EN_CALIB_Nf_GET
#define PLL2_CTL6r_DFE0_EN_CALIB_Nf_SET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_EN_CALIB_Nf_SET
#define PLL2_CTL6r_DFE0_HALFSTEP_ENf_GET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_HALFSTEP_ENf_GET
#define PLL2_CTL6r_DFE0_HALFSTEP_ENf_SET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_HALFSTEP_ENf_SET
#define PLL2_CTL6r_DFE0_CALIB_SEARCH_BITf_GET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_CALIB_SEARCH_BITf_GET
#define PLL2_CTL6r_DFE0_CALIB_SEARCH_BITf_SET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_CALIB_SEARCH_BITf_SET
#define PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRDf_GET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRDf_GET
#define PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRDf_SET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRDf_SET
#define PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRD_VALf_GET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRD_VALf_GET
#define PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRD_VALf_SET BCMI_VIPER_XGXS_PLL2_CTL6r_DFE0_VCOCAL_VALID_OVRD_VALf_SET
#define READ_PLL2_CTL6r BCMI_VIPER_XGXS_READ_PLL2_CTL6r
#define WRITE_PLL2_CTL6r BCMI_VIPER_XGXS_WRITE_PLL2_CTL6r
#define MODIFY_PLL2_CTL6r BCMI_VIPER_XGXS_MODIFY_PLL2_CTL6r
#define READLN_PLL2_CTL6r BCMI_VIPER_XGXS_READLN_PLL2_CTL6r
#define WRITELN_PLL2_CTL6r BCMI_VIPER_XGXS_WRITELN_PLL2_CTL6r
#define WRITEALL_PLL2_CTL6r BCMI_VIPER_XGXS_WRITEALL_PLL2_CTL6r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_PLL2_CTL6r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_CTL1000X1
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8300
 * DESC:     1000X control 1 register
 * RESETVAL: 0x180 (384)
 * ACCESS:   R/W
 * FIELDS:
 *     FIBER_MODE_1000X 1 = Fiber mode (1000X)0 = SGMII mode
 *     SIGNAL_DETECT_EN 1 = signal detect from pin must be set in order to achieve synchronization. In SGMII the signal detect is always ignored regardless of the setting of this bit.0 = ignore signal detect from pin
 *     INVERT_SIGNAL_DETECT 1 = invert signal detect from pin0 = use signal detect from pin
 *     AUTODET_EN 1 = enable auto-detection
 *     SGMII_MASTER_MODE 1 = sgmii mode operates in "phy mode" sending out link, speed, and duplex settings from register 0 to link partner0 = normal operation
 *     DISABLE_PLL_PWRDWN 1 = pll will never be powered down. (use this when the mac/switch uses the tx_wclk_o output)0 = pll will be powered down when register bit ieee0Blk.MIICntl.pwrdwn_sw is set
 *     CRC_CHECKER_DISABLE 1 = disable crc checker by gating the clock to save power0 = enable crc checker
 *     COMMA_DET_EN     1 = enable comma detection0 = disable comma detection
 *     ZERO_COMMA_DETECTOR_PHASE 1 = force comma detector phase to zero0 = normal operation
 *     REMOTE_LOOPBACK  1 = enable remote loopback (only operates in gigabit speed)0 = normal operation
 *     SEL_RX_PKTS_FOR_CNTR 1 = select received packets for SerdesDigital.CrcErr_RxPkt.CrcErr_RxPkt counter0 = select crc errors for SerdesDigital.CrcErr_RxPkt.CrcErr_RxPkt counter
 *     DISABLE_SIGNAL_DETECT_FILTER 1 = disable filter for signal detect0 = filter signal detect from pin before using for synchronization
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r (0x00008300 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_SIZE 4

/*
 * This structure should be used to declare and program DIG_CTL1000X1.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_CTL1000X1r_s {
	uint32_t v[1];
	uint32_t dig_ctl1000x1[1];
	uint32_t _dig_ctl1000x1;
} BCMI_VIPER_XGXS_DIG_CTL1000X1r_t;

#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_CLR(r) (r).dig_ctl1000x1[0] = 0
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_SET(r,d) (r).dig_ctl1000x1[0] = d
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_GET(r) (r).dig_ctl1000x1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_DISABLE_SIGNAL_DETECT_FILTERf_GET(r) ((((r).dig_ctl1000x1[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_DISABLE_SIGNAL_DETECT_FILTERf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_SEL_RX_PKTS_FOR_CNTRf_GET(r) ((((r).dig_ctl1000x1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_SEL_RX_PKTS_FOR_CNTRf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_REMOTE_LOOPBACKf_GET(r) ((((r).dig_ctl1000x1[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_REMOTE_LOOPBACKf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_ZERO_COMMA_DETECTOR_PHASEf_GET(r) ((((r).dig_ctl1000x1[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_ZERO_COMMA_DETECTOR_PHASEf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_COMMA_DET_ENf_GET(r) ((((r).dig_ctl1000x1[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_COMMA_DET_ENf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_CRC_CHECKER_DISABLEf_GET(r) ((((r).dig_ctl1000x1[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_CRC_CHECKER_DISABLEf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_DISABLE_PLL_PWRDWNf_GET(r) ((((r).dig_ctl1000x1[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_DISABLE_PLL_PWRDWNf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_SGMII_MASTER_MODEf_GET(r) ((((r).dig_ctl1000x1[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_SGMII_MASTER_MODEf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_AUTODET_ENf_GET(r) ((((r).dig_ctl1000x1[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_AUTODET_ENf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_INVERT_SIGNAL_DETECTf_GET(r) ((((r).dig_ctl1000x1[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_INVERT_SIGNAL_DETECTf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_SIGNAL_DETECT_ENf_GET(r) ((((r).dig_ctl1000x1[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_SIGNAL_DETECT_ENf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_FIBER_MODE_1000Xf_GET(r) (((r).dig_ctl1000x1[0]) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X1r_FIBER_MODE_1000Xf_SET(r,f) (r).dig_ctl1000x1[0]=(((r).dig_ctl1000x1[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access DIG_CTL1000X1.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_CTL1000X1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X1r,(_r._dig_ctl1000x1))
#define BCMI_VIPER_XGXS_WRITE_DIG_CTL1000X1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X1r,(_r._dig_ctl1000x1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_CTL1000X1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X1r,(_r._dig_ctl1000x1))
#define BCMI_VIPER_XGXS_READLN_DIG_CTL1000X1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_ctl1000x1))
#define BCMI_VIPER_XGXS_WRITELN_DIG_CTL1000X1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_ctl1000x1))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_CTL1000X1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_ctl1000x1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_CTL1000X1r BCMI_VIPER_XGXS_DIG_CTL1000X1r
#define DIG_CTL1000X1r_SIZE BCMI_VIPER_XGXS_DIG_CTL1000X1r_SIZE
typedef BCMI_VIPER_XGXS_DIG_CTL1000X1r_t DIG_CTL1000X1r_t;
#define DIG_CTL1000X1r_CLR BCMI_VIPER_XGXS_DIG_CTL1000X1r_CLR
#define DIG_CTL1000X1r_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_SET
#define DIG_CTL1000X1r_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_GET
#define DIG_CTL1000X1r_DISABLE_SIGNAL_DETECT_FILTERf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_DISABLE_SIGNAL_DETECT_FILTERf_GET
#define DIG_CTL1000X1r_DISABLE_SIGNAL_DETECT_FILTERf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_DISABLE_SIGNAL_DETECT_FILTERf_SET
#define DIG_CTL1000X1r_SEL_RX_PKTS_FOR_CNTRf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_SEL_RX_PKTS_FOR_CNTRf_GET
#define DIG_CTL1000X1r_SEL_RX_PKTS_FOR_CNTRf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_SEL_RX_PKTS_FOR_CNTRf_SET
#define DIG_CTL1000X1r_REMOTE_LOOPBACKf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_REMOTE_LOOPBACKf_GET
#define DIG_CTL1000X1r_REMOTE_LOOPBACKf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_REMOTE_LOOPBACKf_SET
#define DIG_CTL1000X1r_ZERO_COMMA_DETECTOR_PHASEf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_ZERO_COMMA_DETECTOR_PHASEf_GET
#define DIG_CTL1000X1r_ZERO_COMMA_DETECTOR_PHASEf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_ZERO_COMMA_DETECTOR_PHASEf_SET
#define DIG_CTL1000X1r_COMMA_DET_ENf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_COMMA_DET_ENf_GET
#define DIG_CTL1000X1r_COMMA_DET_ENf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_COMMA_DET_ENf_SET
#define DIG_CTL1000X1r_CRC_CHECKER_DISABLEf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_CRC_CHECKER_DISABLEf_GET
#define DIG_CTL1000X1r_CRC_CHECKER_DISABLEf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_CRC_CHECKER_DISABLEf_SET
#define DIG_CTL1000X1r_DISABLE_PLL_PWRDWNf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_DISABLE_PLL_PWRDWNf_GET
#define DIG_CTL1000X1r_DISABLE_PLL_PWRDWNf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_DISABLE_PLL_PWRDWNf_SET
#define DIG_CTL1000X1r_SGMII_MASTER_MODEf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_SGMII_MASTER_MODEf_GET
#define DIG_CTL1000X1r_SGMII_MASTER_MODEf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_SGMII_MASTER_MODEf_SET
#define DIG_CTL1000X1r_AUTODET_ENf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_AUTODET_ENf_GET
#define DIG_CTL1000X1r_AUTODET_ENf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_AUTODET_ENf_SET
#define DIG_CTL1000X1r_INVERT_SIGNAL_DETECTf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_INVERT_SIGNAL_DETECTf_GET
#define DIG_CTL1000X1r_INVERT_SIGNAL_DETECTf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_INVERT_SIGNAL_DETECTf_SET
#define DIG_CTL1000X1r_SIGNAL_DETECT_ENf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_SIGNAL_DETECT_ENf_GET
#define DIG_CTL1000X1r_SIGNAL_DETECT_ENf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_SIGNAL_DETECT_ENf_SET
#define DIG_CTL1000X1r_FIBER_MODE_1000Xf_GET BCMI_VIPER_XGXS_DIG_CTL1000X1r_FIBER_MODE_1000Xf_GET
#define DIG_CTL1000X1r_FIBER_MODE_1000Xf_SET BCMI_VIPER_XGXS_DIG_CTL1000X1r_FIBER_MODE_1000Xf_SET
#define READ_DIG_CTL1000X1r BCMI_VIPER_XGXS_READ_DIG_CTL1000X1r
#define WRITE_DIG_CTL1000X1r BCMI_VIPER_XGXS_WRITE_DIG_CTL1000X1r
#define MODIFY_DIG_CTL1000X1r BCMI_VIPER_XGXS_MODIFY_DIG_CTL1000X1r
#define READLN_DIG_CTL1000X1r BCMI_VIPER_XGXS_READLN_DIG_CTL1000X1r
#define WRITELN_DIG_CTL1000X1r BCMI_VIPER_XGXS_WRITELN_DIG_CTL1000X1r
#define WRITEALL_DIG_CTL1000X1r BCMI_VIPER_XGXS_WRITEALL_DIG_CTL1000X1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_CTL1000X1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_CTL1000X2
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8301
 * DESC:     1000X control 2 register
 * RESETVAL: 0x6 (6)
 * ACCESS:   R/W
 * FIELDS:
 *     ENABLE_PARALLEL_DETECTION 1 = enable paralle detection
 *     DISABLE_FALSE_LINK 1 = do not allow link to be established when auto-negotiation is disabled and receiving auto-negotiation code words. The link will only be established in this case after idles are received. (This bit does not need to be set, if bit 0 below is set.)0 = normal operation
 *     FILTER_FORCE_LINK 1 = sync-status must be set for a solid 10ms before a valid link will be established when auto-negotiation is disabled. (This is useful in fiber applications where the user does not have the signal detect pin connected to the fiber module and auto-negotiation is turned off.)0 = normal operation
 *     ENABLE_AUTONEG_ERR_TIMER 1 = enable auto-negotiation error timer. Error occurs when timer expires in ability-detect, ack-detect, or idle-detect. When the error occurs, config words of all zeros are sent until an ability match occurs, then the autoneg-enable state is entered.0 = normal operation
 *     DISABLE_REMOTE_FAULT_SENSING 1 = disable automatic sensing of remote faults, such as auto-negotiation error0 = automatically detect remote faults and send remote fault status to link partner via auto-negotiation when fiber mode is selected. (SGMII does not support remote faults)
 *     FORCE_XMIT_DATA_ON_TXSIDE 1 = allow packets to be transmitted regardless of the condition of the link or synchronization0 = normal operation
 *     AUTONEG_FAST_TIMERS simulation only speed up for AN sims1 = speed up timers during auto-negotiation for testing0 = normal operation
 *     DISABLE_CARRIER_EXTEND 1 = disable carrier extension in pcs receive0 = normal operation
 *     DISABLE_TRRR_GENERATION 1 = disable TRRR generation in pcs transmit0 = normal operation
 *     BYPASS_PCS_RX    1 = bypass pcs receive operation0 = normal operation
 *     BYPASS_PCS_TX    1 = bypass pcs transmit operation0 = normal operation
 *     TEST_CNTR        1 = increment CrcErr_RxPkt counter each clock cycle for testing0 = normal operation
 *     TRANSMIT_PACKET_SEQ_TEST 1 = enable 16-stage 10-bit idle transmit test sequence to serdes transmitter0 = normal operation
 *     TRANSMIT_IDLEJAM_SEQ_TEST 1 = enable 16-stage 10-bit idle transmit test sequence to serdes transmitter0 = normal operationSC, Self-Clearing
 *     CLEAR_BER_COUNTER 1 = clear bit-error-rate counter (register SerdesDigital.CrcErr_RxPkt.badCodeGroups)0 = normal operation
 *     DISABLE_EXTEND_FDX_ONLY 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r (0x00008301 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_SIZE 4

/*
 * This structure should be used to declare and program DIG_CTL1000X2.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_CTL1000X2r_s {
	uint32_t v[1];
	uint32_t dig_ctl1000x2[1];
	uint32_t _dig_ctl1000x2;
} BCMI_VIPER_XGXS_DIG_CTL1000X2r_t;

#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_CLR(r) (r).dig_ctl1000x2[0] = 0
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_SET(r,d) (r).dig_ctl1000x2[0] = d
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_GET(r) (r).dig_ctl1000x2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_EXTEND_FDX_ONLYf_GET(r) ((((r).dig_ctl1000x2[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_EXTEND_FDX_ONLYf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_CLEAR_BER_COUNTERf_GET(r) ((((r).dig_ctl1000x2[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_CLEAR_BER_COUNTERf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_TRANSMIT_IDLEJAM_SEQ_TESTf_GET(r) ((((r).dig_ctl1000x2[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_TRANSMIT_IDLEJAM_SEQ_TESTf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_TRANSMIT_PACKET_SEQ_TESTf_GET(r) ((((r).dig_ctl1000x2[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_TRANSMIT_PACKET_SEQ_TESTf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_TEST_CNTRf_GET(r) ((((r).dig_ctl1000x2[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_TEST_CNTRf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_BYPASS_PCS_TXf_GET(r) ((((r).dig_ctl1000x2[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_BYPASS_PCS_TXf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_BYPASS_PCS_RXf_GET(r) ((((r).dig_ctl1000x2[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_BYPASS_PCS_RXf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_TRRR_GENERATIONf_GET(r) ((((r).dig_ctl1000x2[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_TRRR_GENERATIONf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_CARRIER_EXTENDf_GET(r) ((((r).dig_ctl1000x2[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_CARRIER_EXTENDf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_AUTONEG_FAST_TIMERSf_GET(r) ((((r).dig_ctl1000x2[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_AUTONEG_FAST_TIMERSf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_FORCE_XMIT_DATA_ON_TXSIDEf_GET(r) ((((r).dig_ctl1000x2[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_FORCE_XMIT_DATA_ON_TXSIDEf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_REMOTE_FAULT_SENSINGf_GET(r) ((((r).dig_ctl1000x2[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_REMOTE_FAULT_SENSINGf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_ENABLE_AUTONEG_ERR_TIMERf_GET(r) ((((r).dig_ctl1000x2[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_ENABLE_AUTONEG_ERR_TIMERf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_FILTER_FORCE_LINKf_GET(r) ((((r).dig_ctl1000x2[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_FILTER_FORCE_LINKf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_FALSE_LINKf_GET(r) ((((r).dig_ctl1000x2[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_FALSE_LINKf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_ENABLE_PARALLEL_DETECTIONf_GET(r) (((r).dig_ctl1000x2[0]) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X2r_ENABLE_PARALLEL_DETECTIONf_SET(r,f) (r).dig_ctl1000x2[0]=(((r).dig_ctl1000x2[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (1 << 16)

/*
 * These macros can be used to access DIG_CTL1000X2.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_CTL1000X2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X2r,(_r._dig_ctl1000x2))
#define BCMI_VIPER_XGXS_WRITE_DIG_CTL1000X2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X2r,(_r._dig_ctl1000x2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_CTL1000X2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X2r,(_r._dig_ctl1000x2))
#define BCMI_VIPER_XGXS_READLN_DIG_CTL1000X2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_ctl1000x2))
#define BCMI_VIPER_XGXS_WRITELN_DIG_CTL1000X2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_ctl1000x2))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_CTL1000X2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_ctl1000x2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_CTL1000X2r BCMI_VIPER_XGXS_DIG_CTL1000X2r
#define DIG_CTL1000X2r_SIZE BCMI_VIPER_XGXS_DIG_CTL1000X2r_SIZE
typedef BCMI_VIPER_XGXS_DIG_CTL1000X2r_t DIG_CTL1000X2r_t;
#define DIG_CTL1000X2r_CLR BCMI_VIPER_XGXS_DIG_CTL1000X2r_CLR
#define DIG_CTL1000X2r_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_SET
#define DIG_CTL1000X2r_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_GET
#define DIG_CTL1000X2r_DISABLE_EXTEND_FDX_ONLYf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_EXTEND_FDX_ONLYf_GET
#define DIG_CTL1000X2r_DISABLE_EXTEND_FDX_ONLYf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_EXTEND_FDX_ONLYf_SET
#define DIG_CTL1000X2r_CLEAR_BER_COUNTERf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_CLEAR_BER_COUNTERf_GET
#define DIG_CTL1000X2r_CLEAR_BER_COUNTERf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_CLEAR_BER_COUNTERf_SET
#define DIG_CTL1000X2r_TRANSMIT_IDLEJAM_SEQ_TESTf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_TRANSMIT_IDLEJAM_SEQ_TESTf_GET
#define DIG_CTL1000X2r_TRANSMIT_IDLEJAM_SEQ_TESTf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_TRANSMIT_IDLEJAM_SEQ_TESTf_SET
#define DIG_CTL1000X2r_TRANSMIT_PACKET_SEQ_TESTf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_TRANSMIT_PACKET_SEQ_TESTf_GET
#define DIG_CTL1000X2r_TRANSMIT_PACKET_SEQ_TESTf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_TRANSMIT_PACKET_SEQ_TESTf_SET
#define DIG_CTL1000X2r_TEST_CNTRf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_TEST_CNTRf_GET
#define DIG_CTL1000X2r_TEST_CNTRf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_TEST_CNTRf_SET
#define DIG_CTL1000X2r_BYPASS_PCS_TXf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_BYPASS_PCS_TXf_GET
#define DIG_CTL1000X2r_BYPASS_PCS_TXf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_BYPASS_PCS_TXf_SET
#define DIG_CTL1000X2r_BYPASS_PCS_RXf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_BYPASS_PCS_RXf_GET
#define DIG_CTL1000X2r_BYPASS_PCS_RXf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_BYPASS_PCS_RXf_SET
#define DIG_CTL1000X2r_DISABLE_TRRR_GENERATIONf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_TRRR_GENERATIONf_GET
#define DIG_CTL1000X2r_DISABLE_TRRR_GENERATIONf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_TRRR_GENERATIONf_SET
#define DIG_CTL1000X2r_DISABLE_CARRIER_EXTENDf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_CARRIER_EXTENDf_GET
#define DIG_CTL1000X2r_DISABLE_CARRIER_EXTENDf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_CARRIER_EXTENDf_SET
#define DIG_CTL1000X2r_AUTONEG_FAST_TIMERSf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_AUTONEG_FAST_TIMERSf_GET
#define DIG_CTL1000X2r_AUTONEG_FAST_TIMERSf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_AUTONEG_FAST_TIMERSf_SET
#define DIG_CTL1000X2r_FORCE_XMIT_DATA_ON_TXSIDEf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_FORCE_XMIT_DATA_ON_TXSIDEf_GET
#define DIG_CTL1000X2r_FORCE_XMIT_DATA_ON_TXSIDEf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_FORCE_XMIT_DATA_ON_TXSIDEf_SET
#define DIG_CTL1000X2r_DISABLE_REMOTE_FAULT_SENSINGf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_REMOTE_FAULT_SENSINGf_GET
#define DIG_CTL1000X2r_DISABLE_REMOTE_FAULT_SENSINGf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_REMOTE_FAULT_SENSINGf_SET
#define DIG_CTL1000X2r_ENABLE_AUTONEG_ERR_TIMERf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_ENABLE_AUTONEG_ERR_TIMERf_GET
#define DIG_CTL1000X2r_ENABLE_AUTONEG_ERR_TIMERf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_ENABLE_AUTONEG_ERR_TIMERf_SET
#define DIG_CTL1000X2r_FILTER_FORCE_LINKf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_FILTER_FORCE_LINKf_GET
#define DIG_CTL1000X2r_FILTER_FORCE_LINKf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_FILTER_FORCE_LINKf_SET
#define DIG_CTL1000X2r_DISABLE_FALSE_LINKf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_FALSE_LINKf_GET
#define DIG_CTL1000X2r_DISABLE_FALSE_LINKf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_DISABLE_FALSE_LINKf_SET
#define DIG_CTL1000X2r_ENABLE_PARALLEL_DETECTIONf_GET BCMI_VIPER_XGXS_DIG_CTL1000X2r_ENABLE_PARALLEL_DETECTIONf_GET
#define DIG_CTL1000X2r_ENABLE_PARALLEL_DETECTIONf_SET BCMI_VIPER_XGXS_DIG_CTL1000X2r_ENABLE_PARALLEL_DETECTIONf_SET

#define READ_DIG_CTL1000X2r BCMI_VIPER_XGXS_READ_DIG_CTL1000X2r
#define WRITE_DIG_CTL1000X2r BCMI_VIPER_XGXS_WRITE_DIG_CTL1000X2r
#define MODIFY_DIG_CTL1000X2r BCMI_VIPER_XGXS_MODIFY_DIG_CTL1000X2r
#define READLN_DIG_CTL1000X2r BCMI_VIPER_XGXS_READLN_DIG_CTL1000X2r
#define WRITELN_DIG_CTL1000X2r BCMI_VIPER_XGXS_WRITELN_DIG_CTL1000X2r
#define WRITEALL_DIG_CTL1000X2r BCMI_VIPER_XGXS_WRITEALL_DIG_CTL1000X2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_CTL1000X2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_CTL1000X3
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8302
 * DESC:     1000X control 3 register
 * RESETVAL: 0x2 (2)
 * ACCESS:   R/W
 * FIELDS:
 *     TX_FIFO_RST      1 = reset transmit fifo. Fifo will remain in reset until this bit is cleared with a software write.0 = normal operation
 *     FIFO_ELASICITY_TX 00 = supports packets up to 5k bytes01 = supports packets up to 10k bytes10 = supports packets up to 13.5k bytes11 = supports packets up to 18.5k bytes
 *     EARLY_PREAMBLE_TX 1 = send extra bytes of preamble to avoid fifo latency. (Used in half-duplex applications to reduce collision domain latency. MAC must send 5 bytes of preamble or less to avoid non-compliant behavior.)0 = normal operation
 *     EARLY_PREAMBLE_RX 1 = send extra bytes of preamble to avoid fifo latency. (Not necessary if MAC uses crs to determine collision)0 = normal operation
 *     RESERVED_5       
 *     RESERVED_6       
 *     BYPASS_TXFIFO1000 1 = bypass transmit fifo in gigabit mode. (Useful for fiber or gigabit only applications where the MAC is using the tx_wclk_o as the clk_in port. User must meet timing to the tx_wclk_o domain)0 = normal operation
 *     FORCE_TXFIFO_ON  1 = force transmit fifo to free-run in gigabit mode (Requires clk_in and tx_wclk_o to be frequency locked.)0 = normal operation
 *     BLOCK_TXEN_MODE  1 = block txen when necessary to guarantee an ipg of at least 6.5 bytes in 10/100 mode, 7 bytes in 1000 mode.0 = normal operation
 *     JAM_FALSE_CARRIER_MODE 1 = change false carriers received into packets with preamble only. (Not necessary if MAC uses crs to determine collision)0 = normal operation
 *     EXT_PHY_CRS_MODE 1 = use external pin for the phys receive only crs output. (Useful in sgmii 10/100 half-duplex applications in order to reduce the collision domain latency. Requires a phy which generates a receive only crs output to a pin.)0 = normal operation
 *     INVERT_EXT_PHY_CRS 1 = invert receive crs from phy pin0 = use receive crs from phy pin
 *     DISABLE_TX_CRS   1 = disable generating crs from transmitting in half-duplex mode. Only receiving will generate crs.0 = normal operation
 *     RXFIFO_GMII_RESET 
 *     DISABLE_PACKET_MISALIGN 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r (0x00008302 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_SIZE 4

/*
 * This structure should be used to declare and program DIG_CTL1000X3.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_CTL1000X3r_s {
	uint32_t v[1];
	uint32_t dig_ctl1000x3[1];
	uint32_t _dig_ctl1000x3;
} BCMI_VIPER_XGXS_DIG_CTL1000X3r_t;

#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_CLR(r) (r).dig_ctl1000x3[0] = 0
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_SET(r,d) (r).dig_ctl1000x3[0] = d
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_GET(r) (r).dig_ctl1000x3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_DISABLE_PACKET_MISALIGNf_GET(r) ((((r).dig_ctl1000x3[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_DISABLE_PACKET_MISALIGNf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_RXFIFO_GMII_RESETf_GET(r) ((((r).dig_ctl1000x3[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_RXFIFO_GMII_RESETf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_DISABLE_TX_CRSf_GET(r) ((((r).dig_ctl1000x3[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_DISABLE_TX_CRSf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_INVERT_EXT_PHY_CRSf_GET(r) ((((r).dig_ctl1000x3[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_INVERT_EXT_PHY_CRSf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_EXT_PHY_CRS_MODEf_GET(r) ((((r).dig_ctl1000x3[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_EXT_PHY_CRS_MODEf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_JAM_FALSE_CARRIER_MODEf_GET(r) ((((r).dig_ctl1000x3[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_JAM_FALSE_CARRIER_MODEf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_BLOCK_TXEN_MODEf_GET(r) ((((r).dig_ctl1000x3[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_BLOCK_TXEN_MODEf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_FORCE_TXFIFO_ONf_GET(r) ((((r).dig_ctl1000x3[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_FORCE_TXFIFO_ONf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_BYPASS_TXFIFO1000f_GET(r) ((((r).dig_ctl1000x3[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_BYPASS_TXFIFO1000f_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_RESERVED_6f_GET(r) ((((r).dig_ctl1000x3[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_RESERVED_6f_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_RESERVED_5f_GET(r) ((((r).dig_ctl1000x3[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_RESERVED_5f_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_EARLY_PREAMBLE_RXf_GET(r) ((((r).dig_ctl1000x3[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_EARLY_PREAMBLE_RXf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_EARLY_PREAMBLE_TXf_GET(r) ((((r).dig_ctl1000x3[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_EARLY_PREAMBLE_TXf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_FIFO_ELASICITY_TXf_GET(r) ((((r).dig_ctl1000x3[0]) >> 1) & 0x3)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_FIFO_ELASICITY_TXf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x3 << 1)) | ((((uint32_t)f) & 0x3) << 1)) | (3 << (16 + 1))
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_TX_FIFO_RSTf_GET(r) (((r).dig_ctl1000x3[0]) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X3r_TX_FIFO_RSTf_SET(r,f) (r).dig_ctl1000x3[0]=(((r).dig_ctl1000x3[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access DIG_CTL1000X3.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_CTL1000X3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X3r,(_r._dig_ctl1000x3))
#define BCMI_VIPER_XGXS_WRITE_DIG_CTL1000X3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X3r,(_r._dig_ctl1000x3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_CTL1000X3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X3r,(_r._dig_ctl1000x3))
#define BCMI_VIPER_XGXS_READLN_DIG_CTL1000X3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_ctl1000x3))
#define BCMI_VIPER_XGXS_WRITELN_DIG_CTL1000X3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_ctl1000x3))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_CTL1000X3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_ctl1000x3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_CTL1000X3r BCMI_VIPER_XGXS_DIG_CTL1000X3r
#define DIG_CTL1000X3r_SIZE BCMI_VIPER_XGXS_DIG_CTL1000X3r_SIZE
typedef BCMI_VIPER_XGXS_DIG_CTL1000X3r_t DIG_CTL1000X3r_t;
#define DIG_CTL1000X3r_CLR BCMI_VIPER_XGXS_DIG_CTL1000X3r_CLR
#define DIG_CTL1000X3r_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_SET
#define DIG_CTL1000X3r_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_GET
#define DIG_CTL1000X3r_DISABLE_PACKET_MISALIGNf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_DISABLE_PACKET_MISALIGNf_GET
#define DIG_CTL1000X3r_DISABLE_PACKET_MISALIGNf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_DISABLE_PACKET_MISALIGNf_SET
#define DIG_CTL1000X3r_RXFIFO_GMII_RESETf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_RXFIFO_GMII_RESETf_GET
#define DIG_CTL1000X3r_RXFIFO_GMII_RESETf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_RXFIFO_GMII_RESETf_SET
#define DIG_CTL1000X3r_DISABLE_TX_CRSf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_DISABLE_TX_CRSf_GET
#define DIG_CTL1000X3r_DISABLE_TX_CRSf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_DISABLE_TX_CRSf_SET
#define DIG_CTL1000X3r_INVERT_EXT_PHY_CRSf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_INVERT_EXT_PHY_CRSf_GET
#define DIG_CTL1000X3r_INVERT_EXT_PHY_CRSf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_INVERT_EXT_PHY_CRSf_SET
#define DIG_CTL1000X3r_EXT_PHY_CRS_MODEf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_EXT_PHY_CRS_MODEf_GET
#define DIG_CTL1000X3r_EXT_PHY_CRS_MODEf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_EXT_PHY_CRS_MODEf_SET
#define DIG_CTL1000X3r_JAM_FALSE_CARRIER_MODEf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_JAM_FALSE_CARRIER_MODEf_GET
#define DIG_CTL1000X3r_JAM_FALSE_CARRIER_MODEf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_JAM_FALSE_CARRIER_MODEf_SET
#define DIG_CTL1000X3r_BLOCK_TXEN_MODEf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_BLOCK_TXEN_MODEf_GET
#define DIG_CTL1000X3r_BLOCK_TXEN_MODEf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_BLOCK_TXEN_MODEf_SET
#define DIG_CTL1000X3r_FORCE_TXFIFO_ONf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_FORCE_TXFIFO_ONf_GET
#define DIG_CTL1000X3r_FORCE_TXFIFO_ONf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_FORCE_TXFIFO_ONf_SET
#define DIG_CTL1000X3r_BYPASS_TXFIFO1000f_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_BYPASS_TXFIFO1000f_GET
#define DIG_CTL1000X3r_BYPASS_TXFIFO1000f_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_BYPASS_TXFIFO1000f_SET
#define DIG_CTL1000X3r_RESERVED_6f_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_RESERVED_6f_GET
#define DIG_CTL1000X3r_RESERVED_6f_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_RESERVED_6f_SET
#define DIG_CTL1000X3r_RESERVED_5f_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_RESERVED_5f_GET
#define DIG_CTL1000X3r_RESERVED_5f_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_RESERVED_5f_SET
#define DIG_CTL1000X3r_EARLY_PREAMBLE_RXf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_EARLY_PREAMBLE_RXf_GET
#define DIG_CTL1000X3r_EARLY_PREAMBLE_RXf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_EARLY_PREAMBLE_RXf_SET
#define DIG_CTL1000X3r_EARLY_PREAMBLE_TXf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_EARLY_PREAMBLE_TXf_GET
#define DIG_CTL1000X3r_EARLY_PREAMBLE_TXf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_EARLY_PREAMBLE_TXf_SET
#define DIG_CTL1000X3r_FIFO_ELASICITY_TXf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_FIFO_ELASICITY_TXf_GET
#define DIG_CTL1000X3r_FIFO_ELASICITY_TXf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_FIFO_ELASICITY_TXf_SET
#define DIG_CTL1000X3r_TX_FIFO_RSTf_GET BCMI_VIPER_XGXS_DIG_CTL1000X3r_TX_FIFO_RSTf_GET
#define DIG_CTL1000X3r_TX_FIFO_RSTf_SET BCMI_VIPER_XGXS_DIG_CTL1000X3r_TX_FIFO_RSTf_SET
#define READ_DIG_CTL1000X3r BCMI_VIPER_XGXS_READ_DIG_CTL1000X3r
#define WRITE_DIG_CTL1000X3r BCMI_VIPER_XGXS_WRITE_DIG_CTL1000X3r
#define MODIFY_DIG_CTL1000X3r BCMI_VIPER_XGXS_MODIFY_DIG_CTL1000X3r
#define READLN_DIG_CTL1000X3r BCMI_VIPER_XGXS_READLN_DIG_CTL1000X3r
#define WRITELN_DIG_CTL1000X3r BCMI_VIPER_XGXS_WRITELN_DIG_CTL1000X3r
#define WRITEALL_DIG_CTL1000X3r BCMI_VIPER_XGXS_WRITEALL_DIG_CTL1000X3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_CTL1000X3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_CTL1000X4
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8303
 * DESC:     1000X control 4 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     MISCRXSTATUS_SEL 0h = Misc Rx status set 01h = Misc Rx status set 12h = Misc Rx status set 23h = Misc Rx status set 3
 *     LINK_FORCE       
 *     LATCH_LINKDOWN_ENABLE 
 *     CLEAR_LINKDOWN   
 *     ZERO_RXDGMII     
 *     TX_CONFIG_REG_SEL 
 *     ENABLE_LAST_RESOLUTION_ERR 
 *     DISABLE_RESOLUTION_ERR_RESTART 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r (0x00008303 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_SIZE 4

/*
 * This structure should be used to declare and program DIG_CTL1000X4.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_CTL1000X4r_s {
	uint32_t v[1];
	uint32_t dig_ctl1000x4[1];
	uint32_t _dig_ctl1000x4;
} BCMI_VIPER_XGXS_DIG_CTL1000X4r_t;

#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_CLR(r) (r).dig_ctl1000x4[0] = 0
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_SET(r,d) (r).dig_ctl1000x4[0] = d
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_GET(r) (r).dig_ctl1000x4[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_DISABLE_RESOLUTION_ERR_RESTARTf_GET(r) ((((r).dig_ctl1000x4[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_DISABLE_RESOLUTION_ERR_RESTARTf_SET(r,f) (r).dig_ctl1000x4[0]=(((r).dig_ctl1000x4[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_ENABLE_LAST_RESOLUTION_ERRf_GET(r) ((((r).dig_ctl1000x4[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_ENABLE_LAST_RESOLUTION_ERRf_SET(r,f) (r).dig_ctl1000x4[0]=(((r).dig_ctl1000x4[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_TX_CONFIG_REG_SELf_GET(r) ((((r).dig_ctl1000x4[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_TX_CONFIG_REG_SELf_SET(r,f) (r).dig_ctl1000x4[0]=(((r).dig_ctl1000x4[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_ZERO_RXDGMIIf_GET(r) ((((r).dig_ctl1000x4[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_ZERO_RXDGMIIf_SET(r,f) (r).dig_ctl1000x4[0]=(((r).dig_ctl1000x4[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_CLEAR_LINKDOWNf_GET(r) ((((r).dig_ctl1000x4[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_CLEAR_LINKDOWNf_SET(r,f) (r).dig_ctl1000x4[0]=(((r).dig_ctl1000x4[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_LATCH_LINKDOWN_ENABLEf_GET(r) ((((r).dig_ctl1000x4[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_LATCH_LINKDOWN_ENABLEf_SET(r,f) (r).dig_ctl1000x4[0]=(((r).dig_ctl1000x4[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_LINK_FORCEf_GET(r) ((((r).dig_ctl1000x4[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_LINK_FORCEf_SET(r,f) (r).dig_ctl1000x4[0]=(((r).dig_ctl1000x4[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_MISCRXSTATUS_SELf_GET(r) (((r).dig_ctl1000x4[0]) & 0x7)
#define BCMI_VIPER_XGXS_DIG_CTL1000X4r_MISCRXSTATUS_SELf_SET(r,f) (r).dig_ctl1000x4[0]=(((r).dig_ctl1000x4[0] & ~((uint32_t)0x7)) | (((uint32_t)f) & 0x7)) | (0x7 << 16)

/*
 * These macros can be used to access DIG_CTL1000X4.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_CTL1000X4r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X4r,(_r._dig_ctl1000x4))
#define BCMI_VIPER_XGXS_WRITE_DIG_CTL1000X4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X4r,(_r._dig_ctl1000x4)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_CTL1000X4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X4r,(_r._dig_ctl1000x4))
#define BCMI_VIPER_XGXS_READLN_DIG_CTL1000X4r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_ctl1000x4))
#define BCMI_VIPER_XGXS_WRITELN_DIG_CTL1000X4r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_ctl1000x4))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_CTL1000X4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_CTL1000X4r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_ctl1000x4))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_CTL1000X4r BCMI_VIPER_XGXS_DIG_CTL1000X4r
#define DIG_CTL1000X4r_SIZE BCMI_VIPER_XGXS_DIG_CTL1000X4r_SIZE
typedef BCMI_VIPER_XGXS_DIG_CTL1000X4r_t DIG_CTL1000X4r_t;
#define DIG_CTL1000X4r_CLR BCMI_VIPER_XGXS_DIG_CTL1000X4r_CLR
#define DIG_CTL1000X4r_SET BCMI_VIPER_XGXS_DIG_CTL1000X4r_SET
#define DIG_CTL1000X4r_GET BCMI_VIPER_XGXS_DIG_CTL1000X4r_GET
#define DIG_CTL1000X4r_DISABLE_RESOLUTION_ERR_RESTARTf_GET BCMI_VIPER_XGXS_DIG_CTL1000X4r_DISABLE_RESOLUTION_ERR_RESTARTf_GET
#define DIG_CTL1000X4r_DISABLE_RESOLUTION_ERR_RESTARTf_SET BCMI_VIPER_XGXS_DIG_CTL1000X4r_DISABLE_RESOLUTION_ERR_RESTARTf_SET
#define DIG_CTL1000X4r_ENABLE_LAST_RESOLUTION_ERRf_GET BCMI_VIPER_XGXS_DIG_CTL1000X4r_ENABLE_LAST_RESOLUTION_ERRf_GET
#define DIG_CTL1000X4r_ENABLE_LAST_RESOLUTION_ERRf_SET BCMI_VIPER_XGXS_DIG_CTL1000X4r_ENABLE_LAST_RESOLUTION_ERRf_SET
#define DIG_CTL1000X4r_TX_CONFIG_REG_SELf_GET BCMI_VIPER_XGXS_DIG_CTL1000X4r_TX_CONFIG_REG_SELf_GET
#define DIG_CTL1000X4r_TX_CONFIG_REG_SELf_SET BCMI_VIPER_XGXS_DIG_CTL1000X4r_TX_CONFIG_REG_SELf_SET
#define DIG_CTL1000X4r_ZERO_RXDGMIIf_GET BCMI_VIPER_XGXS_DIG_CTL1000X4r_ZERO_RXDGMIIf_GET
#define DIG_CTL1000X4r_ZERO_RXDGMIIf_SET BCMI_VIPER_XGXS_DIG_CTL1000X4r_ZERO_RXDGMIIf_SET
#define DIG_CTL1000X4r_CLEAR_LINKDOWNf_GET BCMI_VIPER_XGXS_DIG_CTL1000X4r_CLEAR_LINKDOWNf_GET
#define DIG_CTL1000X4r_CLEAR_LINKDOWNf_SET BCMI_VIPER_XGXS_DIG_CTL1000X4r_CLEAR_LINKDOWNf_SET
#define DIG_CTL1000X4r_LATCH_LINKDOWN_ENABLEf_GET BCMI_VIPER_XGXS_DIG_CTL1000X4r_LATCH_LINKDOWN_ENABLEf_GET
#define DIG_CTL1000X4r_LATCH_LINKDOWN_ENABLEf_SET BCMI_VIPER_XGXS_DIG_CTL1000X4r_LATCH_LINKDOWN_ENABLEf_SET
#define DIG_CTL1000X4r_LINK_FORCEf_GET BCMI_VIPER_XGXS_DIG_CTL1000X4r_LINK_FORCEf_GET
#define DIG_CTL1000X4r_LINK_FORCEf_SET BCMI_VIPER_XGXS_DIG_CTL1000X4r_LINK_FORCEf_SET
#define DIG_CTL1000X4r_MISCRXSTATUS_SELf_GET BCMI_VIPER_XGXS_DIG_CTL1000X4r_MISCRXSTATUS_SELf_GET
#define DIG_CTL1000X4r_MISCRXSTATUS_SELf_SET BCMI_VIPER_XGXS_DIG_CTL1000X4r_MISCRXSTATUS_SELf_SET
#define READ_DIG_CTL1000X4r BCMI_VIPER_XGXS_READ_DIG_CTL1000X4r
#define WRITE_DIG_CTL1000X4r BCMI_VIPER_XGXS_WRITE_DIG_CTL1000X4r
#define MODIFY_DIG_CTL1000X4r BCMI_VIPER_XGXS_MODIFY_DIG_CTL1000X4r
#define READLN_DIG_CTL1000X4r BCMI_VIPER_XGXS_READLN_DIG_CTL1000X4r
#define WRITELN_DIG_CTL1000X4r BCMI_VIPER_XGXS_WRITELN_DIG_CTL1000X4r
#define WRITEALL_DIG_CTL1000X4r BCMI_VIPER_XGXS_WRITEALL_DIG_CTL1000X4r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_CTL1000X4r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_STS1000X1
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8304
 * DESC:     1000X status 1 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     SGMII_MODE       1 = sgmii mode0 = fiber mode (1000-X)
 *     LINK_STATUS      1 = link is up0 = link is down
 *     DUPLEX_STATUS    1 = full-duplex0 = half-duplex
 *     SPEED_STATUS     11 = 2.5G10 = gigabit01 = 100 mbps00 = 10 mbps
 *     PAUSE_RESOLUTION_TXSIDE 1 = enable pause transmit0 = disable pause transmit
 *     PAUSE_RESOLUTION_RXSIDE 1 = enable pause receive0 = disable pause receive
 *     LINK_STATUS_CHANGE 1 = link status has changed since last read0 = link status has not changed since last readLH, Latching High
 *     EARLY_END_EXTENSION_DETECTED 1 = early end extension since last read (early_end_ext in pcs receive fsm)0 = no early end extension since last readLH, Latching High
 *     CARRIER_EXTEND_ERR_DETECTED 1 = carrier extend error since last read (extend_err in pcs receive fsm)0 = no carrier extend error since last readLH, Latching High
 *     RX_ERR_DETECTED  1 = receive error since last read (early_end state in pcs receive fsm)0 = no receive error since last readLH, Latching High
 *     TX_ERR_DETECTED  1 = transmit error code detected since last read (rx_data_error state in pcs receive fsm)0 = no transmit error code detected since last readLH, Latching High
 *     CRC_ERR_DETECTED 1 = crc error detected since last read0 = no crc error detected since last read or detection is disabled via register Control1000X1, bit crc_checker_disableLH, Latching High
 *     FALSE_CARRIER_DETECTED 1 = false carrier detected since last read0 = no false carrier detected since last readLH, Latching High
 *     RXFIFO_ERR_DETECTED 1 = receive fifo error detected since last read0 = no receive fifo error detected since last readLH, Latching High
 *     TXFIFO_ERR_DETECTED 1 = transmit fifo error detected since last read0 = no transmit fifo error detected since last readLH, Latching High
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_STS1000X1r (0x00008304 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_STS1000X1r_SIZE 4

/*
 * This structure should be used to declare and program DIG_STS1000X1.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_STS1000X1r_s {
	uint32_t v[1];
	uint32_t dig_sts1000x1[1];
	uint32_t _dig_sts1000x1;
} BCMI_VIPER_XGXS_DIG_STS1000X1r_t;

#define BCMI_VIPER_XGXS_DIG_STS1000X1r_CLR(r) (r).dig_sts1000x1[0] = 0
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_SET(r,d) (r).dig_sts1000x1[0] = d
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_GET(r) (r).dig_sts1000x1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_TXFIFO_ERR_DETECTEDf_GET(r) ((((r).dig_sts1000x1[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_TXFIFO_ERR_DETECTEDf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_RXFIFO_ERR_DETECTEDf_GET(r) ((((r).dig_sts1000x1[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_RXFIFO_ERR_DETECTEDf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_FALSE_CARRIER_DETECTEDf_GET(r) ((((r).dig_sts1000x1[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_FALSE_CARRIER_DETECTEDf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_CRC_ERR_DETECTEDf_GET(r) ((((r).dig_sts1000x1[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_CRC_ERR_DETECTEDf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_TX_ERR_DETECTEDf_GET(r) ((((r).dig_sts1000x1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_TX_ERR_DETECTEDf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_RX_ERR_DETECTEDf_GET(r) ((((r).dig_sts1000x1[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_RX_ERR_DETECTEDf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_GET(r) ((((r).dig_sts1000x1[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_GET(r) ((((r).dig_sts1000x1[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_LINK_STATUS_CHANGEf_GET(r) ((((r).dig_sts1000x1[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_LINK_STATUS_CHANGEf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_GET(r) ((((r).dig_sts1000x1[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_GET(r) ((((r).dig_sts1000x1[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_SPEED_STATUSf_GET(r) ((((r).dig_sts1000x1[0]) >> 3) & 0x3)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_SPEED_STATUSf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x3 << 3)) | ((((uint32_t)f) & 0x3) << 3)) | (3 << (16 + 3))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_DUPLEX_STATUSf_GET(r) ((((r).dig_sts1000x1[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_DUPLEX_STATUSf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_LINK_STATUSf_GET(r) ((((r).dig_sts1000x1[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_LINK_STATUSf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_SGMII_MODEf_GET(r) (((r).dig_sts1000x1[0]) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X1r_SGMII_MODEf_SET(r,f) (r).dig_sts1000x1[0]=(((r).dig_sts1000x1[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access DIG_STS1000X1.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_STS1000X1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_STS1000X1r,(_r._dig_sts1000x1))
#define BCMI_VIPER_XGXS_WRITE_DIG_STS1000X1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X1r,(_r._dig_sts1000x1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_STS1000X1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X1r,(_r._dig_sts1000x1))
#define BCMI_VIPER_XGXS_READLN_DIG_STS1000X1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_STS1000X1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_sts1000x1))
#define BCMI_VIPER_XGXS_WRITELN_DIG_STS1000X1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_sts1000x1))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_STS1000X1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_sts1000x1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_STS1000X1r BCMI_VIPER_XGXS_DIG_STS1000X1r
#define DIG_STS1000X1r_SIZE BCMI_VIPER_XGXS_DIG_STS1000X1r_SIZE
typedef BCMI_VIPER_XGXS_DIG_STS1000X1r_t DIG_STS1000X1r_t;
#define DIG_STS1000X1r_CLR BCMI_VIPER_XGXS_DIG_STS1000X1r_CLR
#define DIG_STS1000X1r_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_SET
#define DIG_STS1000X1r_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_GET
#define DIG_STS1000X1r_TXFIFO_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_TXFIFO_ERR_DETECTEDf_GET
#define DIG_STS1000X1r_TXFIFO_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_TXFIFO_ERR_DETECTEDf_SET
#define DIG_STS1000X1r_RXFIFO_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_RXFIFO_ERR_DETECTEDf_GET
#define DIG_STS1000X1r_RXFIFO_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_RXFIFO_ERR_DETECTEDf_SET
#define DIG_STS1000X1r_FALSE_CARRIER_DETECTEDf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_FALSE_CARRIER_DETECTEDf_GET
#define DIG_STS1000X1r_FALSE_CARRIER_DETECTEDf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_FALSE_CARRIER_DETECTEDf_SET
#define DIG_STS1000X1r_CRC_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_CRC_ERR_DETECTEDf_GET
#define DIG_STS1000X1r_CRC_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_CRC_ERR_DETECTEDf_SET
#define DIG_STS1000X1r_TX_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_TX_ERR_DETECTEDf_GET
#define DIG_STS1000X1r_TX_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_TX_ERR_DETECTEDf_SET
#define DIG_STS1000X1r_RX_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_RX_ERR_DETECTEDf_GET
#define DIG_STS1000X1r_RX_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_RX_ERR_DETECTEDf_SET
#define DIG_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_GET
#define DIG_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_CARRIER_EXTEND_ERR_DETECTEDf_SET
#define DIG_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_GET
#define DIG_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_EARLY_END_EXTENSION_DETECTEDf_SET
#define DIG_STS1000X1r_LINK_STATUS_CHANGEf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_LINK_STATUS_CHANGEf_GET
#define DIG_STS1000X1r_LINK_STATUS_CHANGEf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_LINK_STATUS_CHANGEf_SET
#define DIG_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_GET
#define DIG_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_PAUSE_RESOLUTION_RXSIDEf_SET
#define DIG_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_GET
#define DIG_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_PAUSE_RESOLUTION_TXSIDEf_SET
#define DIG_STS1000X1r_SPEED_STATUSf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_SPEED_STATUSf_GET
#define DIG_STS1000X1r_SPEED_STATUSf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_SPEED_STATUSf_SET
#define DIG_STS1000X1r_DUPLEX_STATUSf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_DUPLEX_STATUSf_GET
#define DIG_STS1000X1r_DUPLEX_STATUSf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_DUPLEX_STATUSf_SET
#define DIG_STS1000X1r_LINK_STATUSf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_LINK_STATUSf_GET
#define DIG_STS1000X1r_LINK_STATUSf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_LINK_STATUSf_SET
#define DIG_STS1000X1r_SGMII_MODEf_GET BCMI_VIPER_XGXS_DIG_STS1000X1r_SGMII_MODEf_GET
#define DIG_STS1000X1r_SGMII_MODEf_SET BCMI_VIPER_XGXS_DIG_STS1000X1r_SGMII_MODEf_SET
#define READ_DIG_STS1000X1r BCMI_VIPER_XGXS_READ_DIG_STS1000X1r
#define WRITE_DIG_STS1000X1r BCMI_VIPER_XGXS_WRITE_DIG_STS1000X1r
#define MODIFY_DIG_STS1000X1r BCMI_VIPER_XGXS_MODIFY_DIG_STS1000X1r
#define READLN_DIG_STS1000X1r BCMI_VIPER_XGXS_READLN_DIG_STS1000X1r
#define WRITELN_DIG_STS1000X1r BCMI_VIPER_XGXS_WRITELN_DIG_STS1000X1r
#define WRITEALL_DIG_STS1000X1r BCMI_VIPER_XGXS_WRITEALL_DIG_STS1000X1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_STS1000X1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_STS1000X2
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8305
 * DESC:     1000X status 2 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     AN_ENABLE_STATE  1 = an_enable state in auto-negotiation fsm entered since last read0 = an_enable state has not been entered since last readLH, Latching High
 *     ABILITY_DETECT_STATE 1 = ability detect state in auto-negotiation fsm entered since last read0 = ability detect state not entered since last readLH, Latching High
 *     ACKNOWLEDGE_DETECT_STATE 1 = acknowledge detect state in auto-negotiation fsm entered since last read0 = acknowledge detect state not entered since last readLH, Latching High
 *     COMPLETE_ACKNOWLEDGE_STATE 1 = complete acknowledge state in auto-negotiation fsm entered since last read0 = complete acknowledge state not entered since last readLH, Latching High
 *     IDLE_DETECT_STATE 1 = idle detect state in auto-negotiation fsm entered since last read0 = idle detect state not entered since last readLH, Latching High
 *     LINKDOWN_SYNCLOSS 1 = a valid link went down due to a loss of synchronization for over 10ms0 = failure condition has not been detected since last readLH, Latching High
 *     RUDI_INVALID     1 = rudi_invalid detected since last read0 = rudi_invalid has not been detected since last readLH, Latching High
 *     RUDI_I           1 = rudi_i detected since last read0 = rudi_i has not been detected since last readLH, Latching High
 *     RUDI_C           1 = rudi_c detected since last read0 = rudi_c has not been detected since last readLH, Latching High
 *     SYNC_STATUS_OK   1 = sync_status ok detected since last read (synchronization has been achieved)0 = sync_status ok has not been detected since last readLH, Latching High
 *     SYNC_STATUS_FAIL 1 = sync_status has failed since last read (synchronization has been lost)0 = sync_status has not failed since last readLH, Latching High
 *     SGMII_SELECTOR_MISMATCH 1 = sgmii selector mismatch detected since last read (auto-negotiation page received from link partner with bit 0 = 0 while local device is in sgmii mode)0 = sgmii selector mismatch not detected since last readLH, Latching High
 *     AUTONEG_RESOLUTION_ERR 1 = auto-negotiation hcd error detected since last read (hcd is none in fiber mode)0 = auto-negotiation hcd error has not been detected since last readLH, Latching High
 *     CONSISTENCY_MISMATCH 1 = consistency mismatch detected since last read0 = consistency mismatch has not been detected since last readLH, Latching High
 *     SGMII_MODE_CHANGE 1 = sgmii mode has changed since last read (sgmii mode enabled or disabled)0 = sgmii mode has not changed since last read (fixed in sgmii or fiber mode)NOTE: This bit is useful when the auto-detection is enabled in register Control1000X1, autodet_en bit.LH, Latching High
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_STS1000X2r (0x00008305 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SIZE 4

/*
 * This structure should be used to declare and program DIG_STS1000X2.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_STS1000X2r_s {
	uint32_t v[1];
	uint32_t dig_sts1000x2[1];
	uint32_t _dig_sts1000x2;
} BCMI_VIPER_XGXS_DIG_STS1000X2r_t;

#define BCMI_VIPER_XGXS_DIG_STS1000X2r_CLR(r) (r).dig_sts1000x2[0] = 0
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SET(r,d) (r).dig_sts1000x2[0] = d
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_GET(r) (r).dig_sts1000x2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SGMII_MODE_CHANGEf_GET(r) ((((r).dig_sts1000x2[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SGMII_MODE_CHANGEf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_CONSISTENCY_MISMATCHf_GET(r) ((((r).dig_sts1000x2[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_CONSISTENCY_MISMATCHf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_AUTONEG_RESOLUTION_ERRf_GET(r) ((((r).dig_sts1000x2[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_AUTONEG_RESOLUTION_ERRf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SGMII_SELECTOR_MISMATCHf_GET(r) ((((r).dig_sts1000x2[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SGMII_SELECTOR_MISMATCHf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SYNC_STATUS_FAILf_GET(r) ((((r).dig_sts1000x2[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SYNC_STATUS_FAILf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SYNC_STATUS_OKf_GET(r) ((((r).dig_sts1000x2[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_SYNC_STATUS_OKf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_Cf_GET(r) ((((r).dig_sts1000x2[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_Cf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_If_GET(r) ((((r).dig_sts1000x2[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_If_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_INVALIDf_GET(r) ((((r).dig_sts1000x2[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_INVALIDf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_LINKDOWN_SYNCLOSSf_GET(r) ((((r).dig_sts1000x2[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_LINKDOWN_SYNCLOSSf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_IDLE_DETECT_STATEf_GET(r) ((((r).dig_sts1000x2[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_IDLE_DETECT_STATEf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_COMPLETE_ACKNOWLEDGE_STATEf_GET(r) ((((r).dig_sts1000x2[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_COMPLETE_ACKNOWLEDGE_STATEf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_ACKNOWLEDGE_DETECT_STATEf_GET(r) ((((r).dig_sts1000x2[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_ACKNOWLEDGE_DETECT_STATEf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_ABILITY_DETECT_STATEf_GET(r) ((((r).dig_sts1000x2[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_ABILITY_DETECT_STATEf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_AN_ENABLE_STATEf_GET(r) (((r).dig_sts1000x2[0]) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X2r_AN_ENABLE_STATEf_SET(r,f) (r).dig_sts1000x2[0]=(((r).dig_sts1000x2[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access DIG_STS1000X2.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_STS1000X2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_STS1000X2r,(_r._dig_sts1000x2))
#define BCMI_VIPER_XGXS_WRITE_DIG_STS1000X2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X2r,(_r._dig_sts1000x2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_STS1000X2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X2r,(_r._dig_sts1000x2))
#define BCMI_VIPER_XGXS_READLN_DIG_STS1000X2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_STS1000X2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_sts1000x2))
#define BCMI_VIPER_XGXS_WRITELN_DIG_STS1000X2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_sts1000x2))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_STS1000X2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_sts1000x2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_STS1000X2r BCMI_VIPER_XGXS_DIG_STS1000X2r
#define DIG_STS1000X2r_SIZE BCMI_VIPER_XGXS_DIG_STS1000X2r_SIZE
typedef BCMI_VIPER_XGXS_DIG_STS1000X2r_t DIG_STS1000X2r_t;
#define DIG_STS1000X2r_CLR BCMI_VIPER_XGXS_DIG_STS1000X2r_CLR
#define DIG_STS1000X2r_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_SET
#define DIG_STS1000X2r_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_GET
#define DIG_STS1000X2r_SGMII_MODE_CHANGEf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_SGMII_MODE_CHANGEf_GET
#define DIG_STS1000X2r_SGMII_MODE_CHANGEf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_SGMII_MODE_CHANGEf_SET
#define DIG_STS1000X2r_CONSISTENCY_MISMATCHf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_CONSISTENCY_MISMATCHf_GET
#define DIG_STS1000X2r_CONSISTENCY_MISMATCHf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_CONSISTENCY_MISMATCHf_SET
#define DIG_STS1000X2r_AUTONEG_RESOLUTION_ERRf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_AUTONEG_RESOLUTION_ERRf_GET
#define DIG_STS1000X2r_AUTONEG_RESOLUTION_ERRf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_AUTONEG_RESOLUTION_ERRf_SET
#define DIG_STS1000X2r_SGMII_SELECTOR_MISMATCHf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_SGMII_SELECTOR_MISMATCHf_GET
#define DIG_STS1000X2r_SGMII_SELECTOR_MISMATCHf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_SGMII_SELECTOR_MISMATCHf_SET
#define DIG_STS1000X2r_SYNC_STATUS_FAILf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_SYNC_STATUS_FAILf_GET
#define DIG_STS1000X2r_SYNC_STATUS_FAILf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_SYNC_STATUS_FAILf_SET
#define DIG_STS1000X2r_SYNC_STATUS_OKf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_SYNC_STATUS_OKf_GET
#define DIG_STS1000X2r_SYNC_STATUS_OKf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_SYNC_STATUS_OKf_SET
#define DIG_STS1000X2r_RUDI_Cf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_Cf_GET
#define DIG_STS1000X2r_RUDI_Cf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_Cf_SET
#define DIG_STS1000X2r_RUDI_If_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_If_GET
#define DIG_STS1000X2r_RUDI_If_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_If_SET
#define DIG_STS1000X2r_RUDI_INVALIDf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_INVALIDf_GET
#define DIG_STS1000X2r_RUDI_INVALIDf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_RUDI_INVALIDf_SET
#define DIG_STS1000X2r_LINKDOWN_SYNCLOSSf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_LINKDOWN_SYNCLOSSf_GET
#define DIG_STS1000X2r_LINKDOWN_SYNCLOSSf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_LINKDOWN_SYNCLOSSf_SET
#define DIG_STS1000X2r_IDLE_DETECT_STATEf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_IDLE_DETECT_STATEf_GET
#define DIG_STS1000X2r_IDLE_DETECT_STATEf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_IDLE_DETECT_STATEf_SET
#define DIG_STS1000X2r_COMPLETE_ACKNOWLEDGE_STATEf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_COMPLETE_ACKNOWLEDGE_STATEf_GET
#define DIG_STS1000X2r_COMPLETE_ACKNOWLEDGE_STATEf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_COMPLETE_ACKNOWLEDGE_STATEf_SET
#define DIG_STS1000X2r_ACKNOWLEDGE_DETECT_STATEf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_ACKNOWLEDGE_DETECT_STATEf_GET
#define DIG_STS1000X2r_ACKNOWLEDGE_DETECT_STATEf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_ACKNOWLEDGE_DETECT_STATEf_SET
#define DIG_STS1000X2r_ABILITY_DETECT_STATEf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_ABILITY_DETECT_STATEf_GET
#define DIG_STS1000X2r_ABILITY_DETECT_STATEf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_ABILITY_DETECT_STATEf_SET
#define DIG_STS1000X2r_AN_ENABLE_STATEf_GET BCMI_VIPER_XGXS_DIG_STS1000X2r_AN_ENABLE_STATEf_GET
#define DIG_STS1000X2r_AN_ENABLE_STATEf_SET BCMI_VIPER_XGXS_DIG_STS1000X2r_AN_ENABLE_STATEf_SET
#define READ_DIG_STS1000X2r BCMI_VIPER_XGXS_READ_DIG_STS1000X2r
#define WRITE_DIG_STS1000X2r BCMI_VIPER_XGXS_WRITE_DIG_STS1000X2r
#define MODIFY_DIG_STS1000X2r BCMI_VIPER_XGXS_MODIFY_DIG_STS1000X2r
#define READLN_DIG_STS1000X2r BCMI_VIPER_XGXS_READLN_DIG_STS1000X2r
#define WRITELN_DIG_STS1000X2r BCMI_VIPER_XGXS_WRITELN_DIG_STS1000X2r
#define WRITEALL_DIG_STS1000X2r BCMI_VIPER_XGXS_WRITEALL_DIG_STS1000X2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_STS1000X2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_STS1000X3
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8306
 * DESC:     1000X status 3 register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     SD_FILTER_CHG    1 = signal detect has changed since last read0 = signal detect has not changed since last readLH, Latching High
 *     SD_MUX           1 = input of signal detect filter is set0 = input of signal detect filter is not set
 *     SD_FILTER        1 = output of signal detect is set0 = output of signal detect filter is not set
 *     LATCH_LINKDOWN   
 *     REMOTEPHY_AUTOSEL 
 *     PD_PARK_AN       
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_STS1000X3r (0x00008306 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_STS1000X3r_SIZE 4

/*
 * This structure should be used to declare and program DIG_STS1000X3.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_STS1000X3r_s {
	uint32_t v[1];
	uint32_t dig_sts1000x3[1];
	uint32_t _dig_sts1000x3;
} BCMI_VIPER_XGXS_DIG_STS1000X3r_t;

#define BCMI_VIPER_XGXS_DIG_STS1000X3r_CLR(r) (r).dig_sts1000x3[0] = 0
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_SET(r,d) (r).dig_sts1000x3[0] = d
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_GET(r) (r).dig_sts1000x3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_PD_PARK_ANf_GET(r) ((((r).dig_sts1000x3[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_PD_PARK_ANf_SET(r,f) (r).dig_sts1000x3[0]=(((r).dig_sts1000x3[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_REMOTEPHY_AUTOSELf_GET(r) ((((r).dig_sts1000x3[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_REMOTEPHY_AUTOSELf_SET(r,f) (r).dig_sts1000x3[0]=(((r).dig_sts1000x3[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_LATCH_LINKDOWNf_GET(r) ((((r).dig_sts1000x3[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_LATCH_LINKDOWNf_SET(r,f) (r).dig_sts1000x3[0]=(((r).dig_sts1000x3[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_FILTERf_GET(r) ((((r).dig_sts1000x3[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_FILTERf_SET(r,f) (r).dig_sts1000x3[0]=(((r).dig_sts1000x3[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_MUXf_GET(r) ((((r).dig_sts1000x3[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_MUXf_SET(r,f) (r).dig_sts1000x3[0]=(((r).dig_sts1000x3[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_FILTER_CHGf_GET(r) ((((r).dig_sts1000x3[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_FILTER_CHGf_SET(r,f) (r).dig_sts1000x3[0]=(((r).dig_sts1000x3[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))

/*
 * These macros can be used to access DIG_STS1000X3.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_STS1000X3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_STS1000X3r,(_r._dig_sts1000x3))
#define BCMI_VIPER_XGXS_WRITE_DIG_STS1000X3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X3r,(_r._dig_sts1000x3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_STS1000X3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X3r,(_r._dig_sts1000x3))
#define BCMI_VIPER_XGXS_READLN_DIG_STS1000X3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_STS1000X3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_sts1000x3))
#define BCMI_VIPER_XGXS_WRITELN_DIG_STS1000X3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_sts1000x3))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_STS1000X3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_STS1000X3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_sts1000x3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_STS1000X3r BCMI_VIPER_XGXS_DIG_STS1000X3r
#define DIG_STS1000X3r_SIZE BCMI_VIPER_XGXS_DIG_STS1000X3r_SIZE
typedef BCMI_VIPER_XGXS_DIG_STS1000X3r_t DIG_STS1000X3r_t;
#define DIG_STS1000X3r_CLR BCMI_VIPER_XGXS_DIG_STS1000X3r_CLR
#define DIG_STS1000X3r_SET BCMI_VIPER_XGXS_DIG_STS1000X3r_SET
#define DIG_STS1000X3r_GET BCMI_VIPER_XGXS_DIG_STS1000X3r_GET
#define DIG_STS1000X3r_PD_PARK_ANf_GET BCMI_VIPER_XGXS_DIG_STS1000X3r_PD_PARK_ANf_GET
#define DIG_STS1000X3r_PD_PARK_ANf_SET BCMI_VIPER_XGXS_DIG_STS1000X3r_PD_PARK_ANf_SET
#define DIG_STS1000X3r_REMOTEPHY_AUTOSELf_GET BCMI_VIPER_XGXS_DIG_STS1000X3r_REMOTEPHY_AUTOSELf_GET
#define DIG_STS1000X3r_REMOTEPHY_AUTOSELf_SET BCMI_VIPER_XGXS_DIG_STS1000X3r_REMOTEPHY_AUTOSELf_SET
#define DIG_STS1000X3r_LATCH_LINKDOWNf_GET BCMI_VIPER_XGXS_DIG_STS1000X3r_LATCH_LINKDOWNf_GET
#define DIG_STS1000X3r_LATCH_LINKDOWNf_SET BCMI_VIPER_XGXS_DIG_STS1000X3r_LATCH_LINKDOWNf_SET
#define DIG_STS1000X3r_SD_FILTERf_GET BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_FILTERf_GET
#define DIG_STS1000X3r_SD_FILTERf_SET BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_FILTERf_SET
#define DIG_STS1000X3r_SD_MUXf_GET BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_MUXf_GET
#define DIG_STS1000X3r_SD_MUXf_SET BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_MUXf_SET
#define DIG_STS1000X3r_SD_FILTER_CHGf_GET BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_FILTER_CHGf_GET
#define DIG_STS1000X3r_SD_FILTER_CHGf_SET BCMI_VIPER_XGXS_DIG_STS1000X3r_SD_FILTER_CHGf_SET
#define READ_DIG_STS1000X3r BCMI_VIPER_XGXS_READ_DIG_STS1000X3r
#define WRITE_DIG_STS1000X3r BCMI_VIPER_XGXS_WRITE_DIG_STS1000X3r
#define MODIFY_DIG_STS1000X3r BCMI_VIPER_XGXS_MODIFY_DIG_STS1000X3r
#define READLN_DIG_STS1000X3r BCMI_VIPER_XGXS_READLN_DIG_STS1000X3r
#define WRITELN_DIG_STS1000X3r BCMI_VIPER_XGXS_WRITELN_DIG_STS1000X3r
#define WRITEALL_DIG_STS1000X3r BCMI_VIPER_XGXS_WRITEALL_DIG_STS1000X3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_STS1000X3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_BADCODEGROUP
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8307
 * DESC:     Invalid code group count register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     BADCODEGROUPS    Number of invalid code groups detected while sync_status = 1
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_BADCODEGROUPr (0x00008307 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_SIZE 4

/*
 * This structure should be used to declare and program DIG_BADCODEGROUP.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_s {
	uint32_t v[1];
	uint32_t dig_badcodegroup[1];
	uint32_t _dig_badcodegroup;
} BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_t;

#define BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_CLR(r) (r).dig_badcodegroup[0] = 0
#define BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_SET(r,d) (r).dig_badcodegroup[0] = d
#define BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_GET(r) (r).dig_badcodegroup[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_BADCODEGROUPSf_GET(r) ((((r).dig_badcodegroup[0]) >> 8) & 0xff)
#define BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_BADCODEGROUPSf_SET(r,f) (r).dig_badcodegroup[0]=(((r).dig_badcodegroup[0] & ~((uint32_t)0xff << 8)) | ((((uint32_t)f) & 0xff) << 8)) | (255 << (16 + 8))

/*
 * These macros can be used to access DIG_BADCODEGROUP.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_BADCODEGROUPr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_BADCODEGROUPr,(_r._dig_badcodegroup))
#define BCMI_VIPER_XGXS_WRITE_DIG_BADCODEGROUPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_BADCODEGROUPr,(_r._dig_badcodegroup)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_BADCODEGROUPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_BADCODEGROUPr,(_r._dig_badcodegroup))
#define BCMI_VIPER_XGXS_READLN_DIG_BADCODEGROUPr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_BADCODEGROUPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_badcodegroup))
#define BCMI_VIPER_XGXS_WRITELN_DIG_BADCODEGROUPr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_BADCODEGROUPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_badcodegroup))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_BADCODEGROUPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_BADCODEGROUPr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_badcodegroup))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_BADCODEGROUPr BCMI_VIPER_XGXS_DIG_BADCODEGROUPr
#define DIG_BADCODEGROUPr_SIZE BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_SIZE
typedef BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_t DIG_BADCODEGROUPr_t;
#define DIG_BADCODEGROUPr_CLR BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_CLR
#define DIG_BADCODEGROUPr_SET BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_SET
#define DIG_BADCODEGROUPr_GET BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_GET
#define DIG_BADCODEGROUPr_BADCODEGROUPSf_GET BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_BADCODEGROUPSf_GET
#define DIG_BADCODEGROUPr_BADCODEGROUPSf_SET BCMI_VIPER_XGXS_DIG_BADCODEGROUPr_BADCODEGROUPSf_SET
#define READ_DIG_BADCODEGROUPr BCMI_VIPER_XGXS_READ_DIG_BADCODEGROUPr
#define WRITE_DIG_BADCODEGROUPr BCMI_VIPER_XGXS_WRITE_DIG_BADCODEGROUPr
#define MODIFY_DIG_BADCODEGROUPr BCMI_VIPER_XGXS_MODIFY_DIG_BADCODEGROUPr
#define READLN_DIG_BADCODEGROUPr BCMI_VIPER_XGXS_READLN_DIG_BADCODEGROUPr
#define WRITELN_DIG_BADCODEGROUPr BCMI_VIPER_XGXS_WRITELN_DIG_BADCODEGROUPr
#define WRITEALL_DIG_BADCODEGROUPr BCMI_VIPER_XGXS_WRITEALL_DIG_BADCODEGROUPr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_BADCODEGROUPr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_MISC1
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8308
 * DESC:     Miscellaneous 1 control register
 * RESETVAL: 0xc000 (49152)
 * ACCESS:   R/W
 * FIELDS:
 *     FORCE_SPEED      Forces the speed (rate) of the link when CL37 & CL73 auto-negotiation is off.Warning: 1. force_speed_strap is not mapped like this 4 bit field.2. To set 10/100/1000 use the MII Control register.Default is set by the force_speed_strap pins.if (force_speed_b5 = 1)0x00 = dr_10G_DXGXS0x01 = dr_10p5G_HiG_DXGXS0x02 = dr_10p5G_DXGXS0x03 = dr_12p773G_HiG_DXGXS0x04 = dr_12p773G_DXGXS0x05 = dr_10G_XFI0x06 = dr_40G_X40x07 = dr_20G_HiG_DXGXS0x08 = dr_20G_DXGXS0x09 = dr_10G_SFI0x0a = dr_31p5G0x0b = dr_32p7G (not support)0x0c = dr_20G_SCR0x0d = dr_10G_HiG_DXGXS_SCR0x0e = dr_10G_DXGXS_SCR0x0f = dr_12G_R20x10 = dr_10G_X20x11 = dr_40G_KR40x12 = dr_40G_CR4 (not support)0x13 = dr_100G_CR10 (not support)0x14 = dr_5G_HiG_DXGXS (only support in HC-D0)0x15 = dr_5G_DXGXS (only support in HC-D0ox16 = dr_15p75_HiG_DXGXSif (force_speed_b5 = 0)
 *     FORCE_LN_MODE    Forces the lane mode to be derived from independent lane control registers (over-rides auto-neg).
 *     TX_UNDERRUN_1000_DIS 
 *     REFCLK_SEL       Specifies refclk frequency
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_MISC1r (0x00008308 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_MISC1r_SIZE 4

/*
 * This structure should be used to declare and program DIG_MISC1.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_MISC1r_s {
	uint32_t v[1];
	uint32_t dig_misc1[1];
	uint32_t _dig_misc1;
} BCMI_VIPER_XGXS_DIG_MISC1r_t;

#define BCMI_VIPER_XGXS_DIG_MISC1r_CLR(r) (r).dig_misc1[0] = 0
#define BCMI_VIPER_XGXS_DIG_MISC1r_SET(r,d) (r).dig_misc1[0] = d
#define BCMI_VIPER_XGXS_DIG_MISC1r_GET(r) (r).dig_misc1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_MISC1r_REFCLK_SELf_GET(r) ((((r).dig_misc1[0]) >> 13) & 0x7)
#define BCMI_VIPER_XGXS_DIG_MISC1r_REFCLK_SELf_SET(r,f) (r).dig_misc1[0]=(((r).dig_misc1[0] & ~((uint32_t)0x7 << 13)) | ((((uint32_t)f) & 0x7) << 13)) | (7 << (16 + 13))
#define BCMI_VIPER_XGXS_DIG_MISC1r_TX_UNDERRUN_1000_DISf_GET(r) ((((r).dig_misc1[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC1r_TX_UNDERRUN_1000_DISf_SET(r,f) (r).dig_misc1[0]=(((r).dig_misc1[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_DIG_MISC1r_FORCE_LN_MODEf_GET(r) ((((r).dig_misc1[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC1r_FORCE_LN_MODEf_SET(r,f) (r).dig_misc1[0]=(((r).dig_misc1[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_DIG_MISC1r_FORCE_SPEEDf_GET(r) (((r).dig_misc1[0]) & 0x1f)
#define BCMI_VIPER_XGXS_DIG_MISC1r_FORCE_SPEEDf_SET(r,f) (r).dig_misc1[0]=(((r).dig_misc1[0] & ~((uint32_t)0x1f)) | (((uint32_t)f) & 0x1f)) | (0x1f << 16)

/*
 * These macros can be used to access DIG_MISC1.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_MISC1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC1r,(_r._dig_misc1))
#define BCMI_VIPER_XGXS_WRITE_DIG_MISC1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC1r,(_r._dig_misc1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_MISC1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC1r,(_r._dig_misc1))
#define BCMI_VIPER_XGXS_READLN_DIG_MISC1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc1))
#define BCMI_VIPER_XGXS_WRITELN_DIG_MISC1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc1))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_MISC1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_misc1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_MISC1r BCMI_VIPER_XGXS_DIG_MISC1r
#define DIG_MISC1r_SIZE BCMI_VIPER_XGXS_DIG_MISC1r_SIZE
typedef BCMI_VIPER_XGXS_DIG_MISC1r_t DIG_MISC1r_t;
#define DIG_MISC1r_CLR BCMI_VIPER_XGXS_DIG_MISC1r_CLR
#define DIG_MISC1r_SET BCMI_VIPER_XGXS_DIG_MISC1r_SET
#define DIG_MISC1r_GET BCMI_VIPER_XGXS_DIG_MISC1r_GET
#define DIG_MISC1r_REFCLK_SELf_GET BCMI_VIPER_XGXS_DIG_MISC1r_REFCLK_SELf_GET
#define DIG_MISC1r_REFCLK_SELf_SET BCMI_VIPER_XGXS_DIG_MISC1r_REFCLK_SELf_SET
#define DIG_MISC1r_TX_UNDERRUN_1000_DISf_GET BCMI_VIPER_XGXS_DIG_MISC1r_TX_UNDERRUN_1000_DISf_GET
#define DIG_MISC1r_TX_UNDERRUN_1000_DISf_SET BCMI_VIPER_XGXS_DIG_MISC1r_TX_UNDERRUN_1000_DISf_SET
#define DIG_MISC1r_FORCE_LN_MODEf_GET BCMI_VIPER_XGXS_DIG_MISC1r_FORCE_LN_MODEf_GET
#define DIG_MISC1r_FORCE_LN_MODEf_SET BCMI_VIPER_XGXS_DIG_MISC1r_FORCE_LN_MODEf_SET
#define DIG_MISC1r_FORCE_SPEEDf_GET BCMI_VIPER_XGXS_DIG_MISC1r_FORCE_SPEEDf_GET
#define DIG_MISC1r_FORCE_SPEEDf_SET BCMI_VIPER_XGXS_DIG_MISC1r_FORCE_SPEEDf_SET
#define READ_DIG_MISC1r BCMI_VIPER_XGXS_READ_DIG_MISC1r
#define WRITE_DIG_MISC1r BCMI_VIPER_XGXS_WRITE_DIG_MISC1r
#define MODIFY_DIG_MISC1r BCMI_VIPER_XGXS_MODIFY_DIG_MISC1r
#define READLN_DIG_MISC1r BCMI_VIPER_XGXS_READLN_DIG_MISC1r
#define WRITELN_DIG_MISC1r BCMI_VIPER_XGXS_WRITELN_DIG_MISC1r
#define WRITEALL_DIG_MISC1r BCMI_VIPER_XGXS_WRITEALL_DIG_MISC1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_MISC1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_MISC2
 * BLOCKS:   DIGITAL
 * REGADDR:  0x8309
 * DESC:     Miscellaneous 2 control register
 * RESETVAL: 0x2190 (8592)
 * ACCESS:   R/W
 * FIELDS:
 *     AN_TXDISABLEPHASE 1 = When an_txdisable_ln is set, Tx output is held off until the TxPLL locks.0 = When an_txdisable_ln is set, Tx output is held off until both the TxPLL
 *     FIFO_ERR_CYA     1 = Disables re-synchronization of fifo error flags (legacy)0 = Enables re-synchronization of fifo error flags
 *     PMA_PMD_FORCED_SPEED_ENC_EN Enables encoded forced speed derived from ieeeControl1 bits {1.0.13, 1.0.6, 1.0.5:2}, and ieeeControl2 (1.7.3:0)
 *     MIIGMIIMUX_EN    Enable MII data on rxdgmii outputs
 *     MIIGMIIDLY_EN    CYA to delay link_status when switching betweenMII and GMII because of clock switch latency?
 *     CLK41_BYPASS     simulation only speed up1 = Sets clk41 clock period to 40ns (25MHz)0 = Sets clk41 clock period to 41us
 *     CLKSIGDET_BYPASS simulation only speed up1 = accelerates 983us and 390kHz clocks
 *     RLPBK_RXRST_EN   
 *     RLPBK_SW_FORCE   
 *     RESERVED_14_13   
 *     RXCKPL_SEL_COMBO Value of 1 forces the use the recovered clock from lane 0 as the remote loopback in combo mode
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_MISC2r (0x00008309 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_MISC2r_SIZE 4

/*
 * This structure should be used to declare and program DIG_MISC2.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_MISC2r_s {
	uint32_t v[1];
	uint32_t dig_misc2[1];
	uint32_t _dig_misc2;
} BCMI_VIPER_XGXS_DIG_MISC2r_t;

#define BCMI_VIPER_XGXS_DIG_MISC2r_CLR(r) (r).dig_misc2[0] = 0
#define BCMI_VIPER_XGXS_DIG_MISC2r_SET(r,d) (r).dig_misc2[0] = d
#define BCMI_VIPER_XGXS_DIG_MISC2r_GET(r) (r).dig_misc2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_MISC2r_RXCKPL_SEL_COMBOf_GET(r) ((((r).dig_misc2[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_RXCKPL_SEL_COMBOf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_DIG_MISC2r_RESERVED_14_13f_GET(r) ((((r).dig_misc2[0]) >> 13) & 0x3)
#define BCMI_VIPER_XGXS_DIG_MISC2r_RESERVED_14_13f_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x3 << 13)) | ((((uint32_t)f) & 0x3) << 13)) | (3 << (16 + 13))
#define BCMI_VIPER_XGXS_DIG_MISC2r_RLPBK_SW_FORCEf_GET(r) ((((r).dig_misc2[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_RLPBK_SW_FORCEf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_DIG_MISC2r_RLPBK_RXRST_ENf_GET(r) ((((r).dig_misc2[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_RLPBK_RXRST_ENf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_DIG_MISC2r_CLKSIGDET_BYPASSf_GET(r) ((((r).dig_misc2[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_CLKSIGDET_BYPASSf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_DIG_MISC2r_CLK41_BYPASSf_GET(r) ((((r).dig_misc2[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_CLK41_BYPASSf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_MISC2r_MIIGMIIDLY_ENf_GET(r) ((((r).dig_misc2[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_MIIGMIIDLY_ENf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_DIG_MISC2r_MIIGMIIMUX_ENf_GET(r) ((((r).dig_misc2[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_MIIGMIIMUX_ENf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_DIG_MISC2r_PMA_PMD_FORCED_SPEED_ENC_ENf_GET(r) ((((r).dig_misc2[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_PMA_PMD_FORCED_SPEED_ENC_ENf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_DIG_MISC2r_FIFO_ERR_CYAf_GET(r) ((((r).dig_misc2[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_FIFO_ERR_CYAf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_DIG_MISC2r_AN_TXDISABLEPHASEf_GET(r) ((((r).dig_misc2[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC2r_AN_TXDISABLEPHASEf_SET(r,f) (r).dig_misc2[0]=(((r).dig_misc2[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))

/*
 * These macros can be used to access DIG_MISC2.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_MISC2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC2r,(_r._dig_misc2))
#define BCMI_VIPER_XGXS_WRITE_DIG_MISC2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC2r,(_r._dig_misc2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_MISC2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC2r,(_r._dig_misc2))
#define BCMI_VIPER_XGXS_READLN_DIG_MISC2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc2))
#define BCMI_VIPER_XGXS_WRITELN_DIG_MISC2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc2))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_MISC2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_misc2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_MISC2r BCMI_VIPER_XGXS_DIG_MISC2r
#define DIG_MISC2r_SIZE BCMI_VIPER_XGXS_DIG_MISC2r_SIZE
typedef BCMI_VIPER_XGXS_DIG_MISC2r_t DIG_MISC2r_t;
#define DIG_MISC2r_CLR BCMI_VIPER_XGXS_DIG_MISC2r_CLR
#define DIG_MISC2r_SET BCMI_VIPER_XGXS_DIG_MISC2r_SET
#define DIG_MISC2r_GET BCMI_VIPER_XGXS_DIG_MISC2r_GET
#define DIG_MISC2r_RXCKPL_SEL_COMBOf_GET BCMI_VIPER_XGXS_DIG_MISC2r_RXCKPL_SEL_COMBOf_GET
#define DIG_MISC2r_RXCKPL_SEL_COMBOf_SET BCMI_VIPER_XGXS_DIG_MISC2r_RXCKPL_SEL_COMBOf_SET
#define DIG_MISC2r_RESERVED_14_13f_GET BCMI_VIPER_XGXS_DIG_MISC2r_RESERVED_14_13f_GET
#define DIG_MISC2r_RESERVED_14_13f_SET BCMI_VIPER_XGXS_DIG_MISC2r_RESERVED_14_13f_SET
#define DIG_MISC2r_RLPBK_SW_FORCEf_GET BCMI_VIPER_XGXS_DIG_MISC2r_RLPBK_SW_FORCEf_GET
#define DIG_MISC2r_RLPBK_SW_FORCEf_SET BCMI_VIPER_XGXS_DIG_MISC2r_RLPBK_SW_FORCEf_SET
#define DIG_MISC2r_RLPBK_RXRST_ENf_GET BCMI_VIPER_XGXS_DIG_MISC2r_RLPBK_RXRST_ENf_GET
#define DIG_MISC2r_RLPBK_RXRST_ENf_SET BCMI_VIPER_XGXS_DIG_MISC2r_RLPBK_RXRST_ENf_SET
#define DIG_MISC2r_CLKSIGDET_BYPASSf_GET BCMI_VIPER_XGXS_DIG_MISC2r_CLKSIGDET_BYPASSf_GET
#define DIG_MISC2r_CLKSIGDET_BYPASSf_SET BCMI_VIPER_XGXS_DIG_MISC2r_CLKSIGDET_BYPASSf_SET
#define DIG_MISC2r_CLK41_BYPASSf_GET BCMI_VIPER_XGXS_DIG_MISC2r_CLK41_BYPASSf_GET
#define DIG_MISC2r_CLK41_BYPASSf_SET BCMI_VIPER_XGXS_DIG_MISC2r_CLK41_BYPASSf_SET
#define DIG_MISC2r_MIIGMIIDLY_ENf_GET BCMI_VIPER_XGXS_DIG_MISC2r_MIIGMIIDLY_ENf_GET
#define DIG_MISC2r_MIIGMIIDLY_ENf_SET BCMI_VIPER_XGXS_DIG_MISC2r_MIIGMIIDLY_ENf_SET
#define DIG_MISC2r_MIIGMIIMUX_ENf_GET BCMI_VIPER_XGXS_DIG_MISC2r_MIIGMIIMUX_ENf_GET
#define DIG_MISC2r_MIIGMIIMUX_ENf_SET BCMI_VIPER_XGXS_DIG_MISC2r_MIIGMIIMUX_ENf_SET
#define DIG_MISC2r_PMA_PMD_FORCED_SPEED_ENC_ENf_GET BCMI_VIPER_XGXS_DIG_MISC2r_PMA_PMD_FORCED_SPEED_ENC_ENf_GET
#define DIG_MISC2r_PMA_PMD_FORCED_SPEED_ENC_ENf_SET BCMI_VIPER_XGXS_DIG_MISC2r_PMA_PMD_FORCED_SPEED_ENC_ENf_SET
#define DIG_MISC2r_FIFO_ERR_CYAf_GET BCMI_VIPER_XGXS_DIG_MISC2r_FIFO_ERR_CYAf_GET
#define DIG_MISC2r_FIFO_ERR_CYAf_SET BCMI_VIPER_XGXS_DIG_MISC2r_FIFO_ERR_CYAf_SET
#define DIG_MISC2r_AN_TXDISABLEPHASEf_GET BCMI_VIPER_XGXS_DIG_MISC2r_AN_TXDISABLEPHASEf_GET
#define DIG_MISC2r_AN_TXDISABLEPHASEf_SET BCMI_VIPER_XGXS_DIG_MISC2r_AN_TXDISABLEPHASEf_SET
#define READ_DIG_MISC2r BCMI_VIPER_XGXS_READ_DIG_MISC2r
#define WRITE_DIG_MISC2r BCMI_VIPER_XGXS_WRITE_DIG_MISC2r
#define MODIFY_DIG_MISC2r BCMI_VIPER_XGXS_MODIFY_DIG_MISC2r
#define READLN_DIG_MISC2r BCMI_VIPER_XGXS_READLN_DIG_MISC2r
#define WRITELN_DIG_MISC2r BCMI_VIPER_XGXS_WRITELN_DIG_MISC2r
#define WRITEALL_DIG_MISC2r BCMI_VIPER_XGXS_WRITEALL_DIG_MISC2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_MISC2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_PATGENCTL
 * BLOCKS:   DIGITAL
 * REGADDR:  0x830a
 * DESC:     Pattern generator control register
 * RESETVAL: 0x820 (2080)
 * ACCESS:   R/W
 * FIELDS:
 *     SEL_PATTERN_GEN_DATA 1 = send idles or pattern generator data into transmit fifo (ignore MAC transmit data)0 = normal operation
 *     RUN_PATTERN_GEN  1 = A rising edge on this bit, while the pattern generator is in the idle state, will start sending packets. If the single pass mode is set, then a single packet will be sent and the idle state will be entered. If the single pass mode is not set, then packets will be sent until this bit is cleared. At this point the current packet will finish transmitting and then enter the idle state.0 = do not send packetsNOTE: A valid link must be established prior to sending packets.
 *     SINGLE_PASS_MODE 1 = only send 1 packet and stop0 = send packets while bit 1 of this register is set
 *     PKT_SIZE         000000 = invalid000001 = 256 bytes000010 = 512 bytes000011 = 768 bytes000100 = 1024 bytes...111111 = 16,128 bytes
 *     IPG_SELECT       000 = invalid001 = ipg of 6 bytes010 = ipg of 10 bytes011 = ipg of 14 bytes100 = ipg of 18 bytes101 = ipg of 22 bytes110 = ipg of 26 bytes111 = ipg of 30 bytes
 *     EN_CRC_CHECKER_FRAGMENT_ERR_DET 1 = enable crc checker to detect crc errors on packets of any size (1 byte or more)0 = normal operation (crc checker only detects crc errors on packets of at least 72 bytes)
 *     SKIP_CRC         1 = do not append 32 bit crc to end of packet0 = normal operation
 *     TX_ERR           1 = set txer=1 during crc portion of packet0 = normal operation
 *     PATGEN_LPI_EN    1 = send low power idles
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr (0x0000830a | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_SIZE 4

/*
 * This structure should be used to declare and program DIG_PATGENCTL.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_PATGENCTLr_s {
	uint32_t v[1];
	uint32_t dig_patgenctl[1];
	uint32_t _dig_patgenctl;
} BCMI_VIPER_XGXS_DIG_PATGENCTLr_t;

#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_CLR(r) (r).dig_patgenctl[0] = 0
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_SET(r,d) (r).dig_patgenctl[0] = d
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_GET(r) (r).dig_patgenctl[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_PATGEN_LPI_ENf_GET(r) ((((r).dig_patgenctl[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_PATGEN_LPI_ENf_SET(r,f) (r).dig_patgenctl[0]=(((r).dig_patgenctl[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_TX_ERRf_GET(r) ((((r).dig_patgenctl[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_TX_ERRf_SET(r,f) (r).dig_patgenctl[0]=(((r).dig_patgenctl[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_SKIP_CRCf_GET(r) ((((r).dig_patgenctl[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_SKIP_CRCf_SET(r,f) (r).dig_patgenctl[0]=(((r).dig_patgenctl[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_EN_CRC_CHECKER_FRAGMENT_ERR_DETf_GET(r) ((((r).dig_patgenctl[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_EN_CRC_CHECKER_FRAGMENT_ERR_DETf_SET(r,f) (r).dig_patgenctl[0]=(((r).dig_patgenctl[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_IPG_SELECTf_GET(r) ((((r).dig_patgenctl[0]) >> 9) & 0x7)
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_IPG_SELECTf_SET(r,f) (r).dig_patgenctl[0]=(((r).dig_patgenctl[0] & ~((uint32_t)0x7 << 9)) | ((((uint32_t)f) & 0x7) << 9)) | (7 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_PKT_SIZEf_GET(r) ((((r).dig_patgenctl[0]) >> 3) & 0x3f)
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_PKT_SIZEf_SET(r,f) (r).dig_patgenctl[0]=(((r).dig_patgenctl[0] & ~((uint32_t)0x3f << 3)) | ((((uint32_t)f) & 0x3f) << 3)) | (63 << (16 + 3))
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_SINGLE_PASS_MODEf_GET(r) ((((r).dig_patgenctl[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_SINGLE_PASS_MODEf_SET(r,f) (r).dig_patgenctl[0]=(((r).dig_patgenctl[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_RUN_PATTERN_GENf_GET(r) ((((r).dig_patgenctl[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_RUN_PATTERN_GENf_SET(r,f) (r).dig_patgenctl[0]=(((r).dig_patgenctl[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_SEL_PATTERN_GEN_DATAf_GET(r) (((r).dig_patgenctl[0]) & 0x1)
#define BCMI_VIPER_XGXS_DIG_PATGENCTLr_SEL_PATTERN_GEN_DATAf_SET(r,f) (r).dig_patgenctl[0]=(((r).dig_patgenctl[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access DIG_PATGENCTL.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_PATGENCTLr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_PATGENCTLr,(_r._dig_patgenctl))
#define BCMI_VIPER_XGXS_WRITE_DIG_PATGENCTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_PATGENCTLr,(_r._dig_patgenctl)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_PATGENCTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_PATGENCTLr,(_r._dig_patgenctl))
#define BCMI_VIPER_XGXS_READLN_DIG_PATGENCTLr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_PATGENCTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_patgenctl))
#define BCMI_VIPER_XGXS_WRITELN_DIG_PATGENCTLr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_PATGENCTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_patgenctl))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_PATGENCTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_PATGENCTLr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_patgenctl))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_PATGENCTLr BCMI_VIPER_XGXS_DIG_PATGENCTLr
#define DIG_PATGENCTLr_SIZE BCMI_VIPER_XGXS_DIG_PATGENCTLr_SIZE
typedef BCMI_VIPER_XGXS_DIG_PATGENCTLr_t DIG_PATGENCTLr_t;
#define DIG_PATGENCTLr_CLR BCMI_VIPER_XGXS_DIG_PATGENCTLr_CLR
#define DIG_PATGENCTLr_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_SET
#define DIG_PATGENCTLr_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_GET
#define DIG_PATGENCTLr_PATGEN_LPI_ENf_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_PATGEN_LPI_ENf_GET
#define DIG_PATGENCTLr_PATGEN_LPI_ENf_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_PATGEN_LPI_ENf_SET
#define DIG_PATGENCTLr_TX_ERRf_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_TX_ERRf_GET
#define DIG_PATGENCTLr_TX_ERRf_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_TX_ERRf_SET
#define DIG_PATGENCTLr_SKIP_CRCf_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_SKIP_CRCf_GET
#define DIG_PATGENCTLr_SKIP_CRCf_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_SKIP_CRCf_SET
#define DIG_PATGENCTLr_EN_CRC_CHECKER_FRAGMENT_ERR_DETf_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_EN_CRC_CHECKER_FRAGMENT_ERR_DETf_GET
#define DIG_PATGENCTLr_EN_CRC_CHECKER_FRAGMENT_ERR_DETf_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_EN_CRC_CHECKER_FRAGMENT_ERR_DETf_SET
#define DIG_PATGENCTLr_IPG_SELECTf_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_IPG_SELECTf_GET
#define DIG_PATGENCTLr_IPG_SELECTf_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_IPG_SELECTf_SET
#define DIG_PATGENCTLr_PKT_SIZEf_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_PKT_SIZEf_GET
#define DIG_PATGENCTLr_PKT_SIZEf_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_PKT_SIZEf_SET
#define DIG_PATGENCTLr_SINGLE_PASS_MODEf_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_SINGLE_PASS_MODEf_GET
#define DIG_PATGENCTLr_SINGLE_PASS_MODEf_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_SINGLE_PASS_MODEf_SET
#define DIG_PATGENCTLr_RUN_PATTERN_GENf_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_RUN_PATTERN_GENf_GET
#define DIG_PATGENCTLr_RUN_PATTERN_GENf_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_RUN_PATTERN_GENf_SET
#define DIG_PATGENCTLr_SEL_PATTERN_GEN_DATAf_GET BCMI_VIPER_XGXS_DIG_PATGENCTLr_SEL_PATTERN_GEN_DATAf_GET
#define DIG_PATGENCTLr_SEL_PATTERN_GEN_DATAf_SET BCMI_VIPER_XGXS_DIG_PATGENCTLr_SEL_PATTERN_GEN_DATAf_SET
#define READ_DIG_PATGENCTLr BCMI_VIPER_XGXS_READ_DIG_PATGENCTLr
#define WRITE_DIG_PATGENCTLr BCMI_VIPER_XGXS_WRITE_DIG_PATGENCTLr
#define MODIFY_DIG_PATGENCTLr BCMI_VIPER_XGXS_MODIFY_DIG_PATGENCTLr
#define READLN_DIG_PATGENCTLr BCMI_VIPER_XGXS_READLN_DIG_PATGENCTLr
#define WRITELN_DIG_PATGENCTLr BCMI_VIPER_XGXS_WRITELN_DIG_PATGENCTLr
#define WRITEALL_DIG_PATGENCTLr BCMI_VIPER_XGXS_WRITEALL_DIG_PATGENCTLr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_PATGENCTLr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_PATGENSTAT
 * BLOCKS:   DIGITAL
 * REGADDR:  0x830b
 * DESC:     Pattern generator status register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     PATTERN_GEN_FSM  000 = idle001 = transmit preamble011 = transmit sfd010 = transmit data110 = transmit crc100 = ipg101 = ipg 2 (allows fsm to be grey-coded)
 *     PATTERN_GEN_ACTIVE 1 = pattern generator is still sending packets0 = pattern generator is idle
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_PATGENSTATr (0x0000830b | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_PATGENSTATr_SIZE 4

/*
 * This structure should be used to declare and program DIG_PATGENSTAT.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_PATGENSTATr_s {
	uint32_t v[1];
	uint32_t dig_patgenstat[1];
	uint32_t _dig_patgenstat;
} BCMI_VIPER_XGXS_DIG_PATGENSTATr_t;

#define BCMI_VIPER_XGXS_DIG_PATGENSTATr_CLR(r) (r).dig_patgenstat[0] = 0
#define BCMI_VIPER_XGXS_DIG_PATGENSTATr_SET(r,d) (r).dig_patgenstat[0] = d
#define BCMI_VIPER_XGXS_DIG_PATGENSTATr_GET(r) (r).dig_patgenstat[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_PATGENSTATr_PATTERN_GEN_ACTIVEf_GET(r) ((((r).dig_patgenstat[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_DIG_PATGENSTATr_PATTERN_GEN_ACTIVEf_SET(r,f) (r).dig_patgenstat[0]=(((r).dig_patgenstat[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_DIG_PATGENSTATr_PATTERN_GEN_FSMf_GET(r) (((r).dig_patgenstat[0]) & 0x7)
#define BCMI_VIPER_XGXS_DIG_PATGENSTATr_PATTERN_GEN_FSMf_SET(r,f) (r).dig_patgenstat[0]=(((r).dig_patgenstat[0] & ~((uint32_t)0x7)) | (((uint32_t)f) & 0x7)) | (0x7 << 16)

/*
 * These macros can be used to access DIG_PATGENSTAT.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_PATGENSTATr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_PATGENSTATr,(_r._dig_patgenstat))
#define BCMI_VIPER_XGXS_WRITE_DIG_PATGENSTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_PATGENSTATr,(_r._dig_patgenstat)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_PATGENSTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_PATGENSTATr,(_r._dig_patgenstat))
#define BCMI_VIPER_XGXS_READLN_DIG_PATGENSTATr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_PATGENSTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_patgenstat))
#define BCMI_VIPER_XGXS_WRITELN_DIG_PATGENSTATr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_PATGENSTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_patgenstat))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_PATGENSTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_PATGENSTATr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_patgenstat))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_PATGENSTATr BCMI_VIPER_XGXS_DIG_PATGENSTATr
#define DIG_PATGENSTATr_SIZE BCMI_VIPER_XGXS_DIG_PATGENSTATr_SIZE
typedef BCMI_VIPER_XGXS_DIG_PATGENSTATr_t DIG_PATGENSTATr_t;
#define DIG_PATGENSTATr_CLR BCMI_VIPER_XGXS_DIG_PATGENSTATr_CLR
#define DIG_PATGENSTATr_SET BCMI_VIPER_XGXS_DIG_PATGENSTATr_SET
#define DIG_PATGENSTATr_GET BCMI_VIPER_XGXS_DIG_PATGENSTATr_GET
#define DIG_PATGENSTATr_PATTERN_GEN_ACTIVEf_GET BCMI_VIPER_XGXS_DIG_PATGENSTATr_PATTERN_GEN_ACTIVEf_GET
#define DIG_PATGENSTATr_PATTERN_GEN_ACTIVEf_SET BCMI_VIPER_XGXS_DIG_PATGENSTATr_PATTERN_GEN_ACTIVEf_SET
#define DIG_PATGENSTATr_PATTERN_GEN_FSMf_GET BCMI_VIPER_XGXS_DIG_PATGENSTATr_PATTERN_GEN_FSMf_GET
#define DIG_PATGENSTATr_PATTERN_GEN_FSMf_SET BCMI_VIPER_XGXS_DIG_PATGENSTATr_PATTERN_GEN_FSMf_SET
#define READ_DIG_PATGENSTATr BCMI_VIPER_XGXS_READ_DIG_PATGENSTATr
#define WRITE_DIG_PATGENSTATr BCMI_VIPER_XGXS_WRITE_DIG_PATGENSTATr
#define MODIFY_DIG_PATGENSTATr BCMI_VIPER_XGXS_MODIFY_DIG_PATGENSTATr
#define READLN_DIG_PATGENSTATr BCMI_VIPER_XGXS_READLN_DIG_PATGENSTATr
#define WRITELN_DIG_PATGENSTATr BCMI_VIPER_XGXS_WRITELN_DIG_PATGENSTATr
#define WRITEALL_DIG_PATGENSTATr BCMI_VIPER_XGXS_WRITEALL_DIG_PATGENSTATr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_PATGENSTATr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_TESTMODE
 * BLOCKS:   DIGITAL
 * REGADDR:  0x830c
 * DESC:     Test mode register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     FIFO_ELASTICITY_RX 00 = supports packets up to 5k bytes01 = supports packets up to 10k bytes10 = supports packets up to 13p5k bytes11 = supports packets up to 18p5k bytes
 *     CLEAR_PACKET_COUNTERS When set to 1'b1 clears Tx/Rx packet counters
 *     DISABLE_RESET_CNT When set to 1'b1 disables packet and error counter asynchronous reset
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_TESTMODEr (0x0000830c | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_TESTMODEr_SIZE 4

/*
 * This structure should be used to declare and program DIG_TESTMODE.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_TESTMODEr_s {
	uint32_t v[1];
	uint32_t dig_testmode[1];
	uint32_t _dig_testmode;
} BCMI_VIPER_XGXS_DIG_TESTMODEr_t;

#define BCMI_VIPER_XGXS_DIG_TESTMODEr_CLR(r) (r).dig_testmode[0] = 0
#define BCMI_VIPER_XGXS_DIG_TESTMODEr_SET(r,d) (r).dig_testmode[0] = d
#define BCMI_VIPER_XGXS_DIG_TESTMODEr_GET(r) (r).dig_testmode[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_TESTMODEr_DISABLE_RESET_CNTf_GET(r) ((((r).dig_testmode[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_DIG_TESTMODEr_DISABLE_RESET_CNTf_SET(r,f) (r).dig_testmode[0]=(((r).dig_testmode[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_DIG_TESTMODEr_CLEAR_PACKET_COUNTERSf_GET(r) ((((r).dig_testmode[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_DIG_TESTMODEr_CLEAR_PACKET_COUNTERSf_SET(r,f) (r).dig_testmode[0]=(((r).dig_testmode[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_DIG_TESTMODEr_FIFO_ELASTICITY_RXf_GET(r) ((((r).dig_testmode[0]) >> 1) & 0x3)
#define BCMI_VIPER_XGXS_DIG_TESTMODEr_FIFO_ELASTICITY_RXf_SET(r,f) (r).dig_testmode[0]=(((r).dig_testmode[0] & ~((uint32_t)0x3 << 1)) | ((((uint32_t)f) & 0x3) << 1)) | (3 << (16 + 1))

/*
 * These macros can be used to access DIG_TESTMODE.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_TESTMODEr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_TESTMODEr,(_r._dig_testmode))
#define BCMI_VIPER_XGXS_WRITE_DIG_TESTMODEr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TESTMODEr,(_r._dig_testmode)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_TESTMODEr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TESTMODEr,(_r._dig_testmode))
#define BCMI_VIPER_XGXS_READLN_DIG_TESTMODEr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_TESTMODEr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_testmode))
#define BCMI_VIPER_XGXS_WRITELN_DIG_TESTMODEr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TESTMODEr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_testmode))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_TESTMODEr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TESTMODEr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_testmode))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_TESTMODEr BCMI_VIPER_XGXS_DIG_TESTMODEr
#define DIG_TESTMODEr_SIZE BCMI_VIPER_XGXS_DIG_TESTMODEr_SIZE
typedef BCMI_VIPER_XGXS_DIG_TESTMODEr_t DIG_TESTMODEr_t;
#define DIG_TESTMODEr_CLR BCMI_VIPER_XGXS_DIG_TESTMODEr_CLR
#define DIG_TESTMODEr_SET BCMI_VIPER_XGXS_DIG_TESTMODEr_SET
#define DIG_TESTMODEr_GET BCMI_VIPER_XGXS_DIG_TESTMODEr_GET
#define DIG_TESTMODEr_DISABLE_RESET_CNTf_GET BCMI_VIPER_XGXS_DIG_TESTMODEr_DISABLE_RESET_CNTf_GET
#define DIG_TESTMODEr_DISABLE_RESET_CNTf_SET BCMI_VIPER_XGXS_DIG_TESTMODEr_DISABLE_RESET_CNTf_SET
#define DIG_TESTMODEr_CLEAR_PACKET_COUNTERSf_GET BCMI_VIPER_XGXS_DIG_TESTMODEr_CLEAR_PACKET_COUNTERSf_GET
#define DIG_TESTMODEr_CLEAR_PACKET_COUNTERSf_SET BCMI_VIPER_XGXS_DIG_TESTMODEr_CLEAR_PACKET_COUNTERSf_SET
#define DIG_TESTMODEr_FIFO_ELASTICITY_RXf_GET BCMI_VIPER_XGXS_DIG_TESTMODEr_FIFO_ELASTICITY_RXf_GET
#define DIG_TESTMODEr_FIFO_ELASTICITY_RXf_SET BCMI_VIPER_XGXS_DIG_TESTMODEr_FIFO_ELASTICITY_RXf_SET
#define READ_DIG_TESTMODEr BCMI_VIPER_XGXS_READ_DIG_TESTMODEr
#define WRITE_DIG_TESTMODEr BCMI_VIPER_XGXS_WRITE_DIG_TESTMODEr
#define MODIFY_DIG_TESTMODEr BCMI_VIPER_XGXS_MODIFY_DIG_TESTMODEr
#define READLN_DIG_TESTMODEr BCMI_VIPER_XGXS_READLN_DIG_TESTMODEr
#define WRITELN_DIG_TESTMODEr BCMI_VIPER_XGXS_WRITELN_DIG_TESTMODEr
#define WRITEALL_DIG_TESTMODEr BCMI_VIPER_XGXS_WRITEALL_DIG_TESTMODEr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_TESTMODEr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_TXPKTCNT
 * BLOCKS:   DIGITAL
 * REGADDR:  0x830d
 * DESC:     Tx packet count register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     TXPKTCNT         Tx Packet counter status register.  Rolls over (does not saturate).
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_TXPKTCNTr (0x0000830d | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_TXPKTCNTr_SIZE 4

/*
 * This structure should be used to declare and program DIG_TXPKTCNT.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_TXPKTCNTr_s {
	uint32_t v[1];
	uint32_t dig_txpktcnt[1];
	uint32_t _dig_txpktcnt;
} BCMI_VIPER_XGXS_DIG_TXPKTCNTr_t;

#define BCMI_VIPER_XGXS_DIG_TXPKTCNTr_CLR(r) (r).dig_txpktcnt[0] = 0
#define BCMI_VIPER_XGXS_DIG_TXPKTCNTr_SET(r,d) (r).dig_txpktcnt[0] = d
#define BCMI_VIPER_XGXS_DIG_TXPKTCNTr_GET(r) (r).dig_txpktcnt[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_TXPKTCNTr_TXPKTCNTf_GET(r) (((r).dig_txpktcnt[0]) & 0xffff)
#define BCMI_VIPER_XGXS_DIG_TXPKTCNTr_TXPKTCNTf_SET(r,f) (r).dig_txpktcnt[0]=(((r).dig_txpktcnt[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access DIG_TXPKTCNT.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_TXPKTCNTr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_TXPKTCNTr,(_r._dig_txpktcnt))
#define BCMI_VIPER_XGXS_WRITE_DIG_TXPKTCNTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TXPKTCNTr,(_r._dig_txpktcnt)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_TXPKTCNTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TXPKTCNTr,(_r._dig_txpktcnt))
#define BCMI_VIPER_XGXS_READLN_DIG_TXPKTCNTr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_TXPKTCNTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_txpktcnt))
#define BCMI_VIPER_XGXS_WRITELN_DIG_TXPKTCNTr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TXPKTCNTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_txpktcnt))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_TXPKTCNTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TXPKTCNTr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_txpktcnt))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_TXPKTCNTr BCMI_VIPER_XGXS_DIG_TXPKTCNTr
#define DIG_TXPKTCNTr_SIZE BCMI_VIPER_XGXS_DIG_TXPKTCNTr_SIZE
typedef BCMI_VIPER_XGXS_DIG_TXPKTCNTr_t DIG_TXPKTCNTr_t;
#define DIG_TXPKTCNTr_CLR BCMI_VIPER_XGXS_DIG_TXPKTCNTr_CLR
#define DIG_TXPKTCNTr_SET BCMI_VIPER_XGXS_DIG_TXPKTCNTr_SET
#define DIG_TXPKTCNTr_GET BCMI_VIPER_XGXS_DIG_TXPKTCNTr_GET
#define DIG_TXPKTCNTr_TXPKTCNTf_GET BCMI_VIPER_XGXS_DIG_TXPKTCNTr_TXPKTCNTf_GET
#define DIG_TXPKTCNTr_TXPKTCNTf_SET BCMI_VIPER_XGXS_DIG_TXPKTCNTr_TXPKTCNTf_SET
#define READ_DIG_TXPKTCNTr BCMI_VIPER_XGXS_READ_DIG_TXPKTCNTr
#define WRITE_DIG_TXPKTCNTr BCMI_VIPER_XGXS_WRITE_DIG_TXPKTCNTr
#define MODIFY_DIG_TXPKTCNTr BCMI_VIPER_XGXS_MODIFY_DIG_TXPKTCNTr
#define READLN_DIG_TXPKTCNTr BCMI_VIPER_XGXS_READLN_DIG_TXPKTCNTr
#define WRITELN_DIG_TXPKTCNTr BCMI_VIPER_XGXS_WRITELN_DIG_TXPKTCNTr
#define WRITEALL_DIG_TXPKTCNTr BCMI_VIPER_XGXS_WRITEALL_DIG_TXPKTCNTr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_TXPKTCNTr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_RXPKTCNT
 * BLOCKS:   DIGITAL
 * REGADDR:  0x830e
 * DESC:     Rx packet count register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     RXPKTCNT         Rx Packet counter status register.  Rolls over (does not saturate).
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_RXPKTCNTr (0x0000830e | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_RXPKTCNTr_SIZE 4

/*
 * This structure should be used to declare and program DIG_RXPKTCNT.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_RXPKTCNTr_s {
	uint32_t v[1];
	uint32_t dig_rxpktcnt[1];
	uint32_t _dig_rxpktcnt;
} BCMI_VIPER_XGXS_DIG_RXPKTCNTr_t;

#define BCMI_VIPER_XGXS_DIG_RXPKTCNTr_CLR(r) (r).dig_rxpktcnt[0] = 0
#define BCMI_VIPER_XGXS_DIG_RXPKTCNTr_SET(r,d) (r).dig_rxpktcnt[0] = d
#define BCMI_VIPER_XGXS_DIG_RXPKTCNTr_GET(r) (r).dig_rxpktcnt[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_RXPKTCNTr_RXPKTCNTf_GET(r) (((r).dig_rxpktcnt[0]) & 0xffff)
#define BCMI_VIPER_XGXS_DIG_RXPKTCNTr_RXPKTCNTf_SET(r,f) (r).dig_rxpktcnt[0]=(((r).dig_rxpktcnt[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access DIG_RXPKTCNT.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_RXPKTCNTr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_RXPKTCNTr,(_r._dig_rxpktcnt))
#define BCMI_VIPER_XGXS_WRITE_DIG_RXPKTCNTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_RXPKTCNTr,(_r._dig_rxpktcnt)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_RXPKTCNTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_RXPKTCNTr,(_r._dig_rxpktcnt))
#define BCMI_VIPER_XGXS_READLN_DIG_RXPKTCNTr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_RXPKTCNTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_rxpktcnt))
#define BCMI_VIPER_XGXS_WRITELN_DIG_RXPKTCNTr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_RXPKTCNTr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_rxpktcnt))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_RXPKTCNTr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_RXPKTCNTr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_rxpktcnt))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_RXPKTCNTr BCMI_VIPER_XGXS_DIG_RXPKTCNTr
#define DIG_RXPKTCNTr_SIZE BCMI_VIPER_XGXS_DIG_RXPKTCNTr_SIZE
typedef BCMI_VIPER_XGXS_DIG_RXPKTCNTr_t DIG_RXPKTCNTr_t;
#define DIG_RXPKTCNTr_CLR BCMI_VIPER_XGXS_DIG_RXPKTCNTr_CLR
#define DIG_RXPKTCNTr_SET BCMI_VIPER_XGXS_DIG_RXPKTCNTr_SET
#define DIG_RXPKTCNTr_GET BCMI_VIPER_XGXS_DIG_RXPKTCNTr_GET
#define DIG_RXPKTCNTr_RXPKTCNTf_GET BCMI_VIPER_XGXS_DIG_RXPKTCNTr_RXPKTCNTf_GET
#define DIG_RXPKTCNTr_RXPKTCNTf_SET BCMI_VIPER_XGXS_DIG_RXPKTCNTr_RXPKTCNTf_SET
#define READ_DIG_RXPKTCNTr BCMI_VIPER_XGXS_READ_DIG_RXPKTCNTr
#define WRITE_DIG_RXPKTCNTr BCMI_VIPER_XGXS_WRITE_DIG_RXPKTCNTr
#define MODIFY_DIG_RXPKTCNTr BCMI_VIPER_XGXS_MODIFY_DIG_RXPKTCNTr
#define READLN_DIG_RXPKTCNTr BCMI_VIPER_XGXS_READLN_DIG_RXPKTCNTr
#define WRITELN_DIG_RXPKTCNTr BCMI_VIPER_XGXS_WRITELN_DIG_RXPKTCNTr
#define WRITEALL_DIG_RXPKTCNTr BCMI_VIPER_XGXS_WRITEALL_DIG_RXPKTCNTr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_RXPKTCNTr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  SERDESID0
 * BLOCKS:   SERDESID
 * REGADDR:  0x8310
 * DESC:     Serdes ID 0 register
 * RESETVAL: 0x2cf (719)
 * ACCESS:   R/O
 * FIELDS:
 *     MODEL_NUMBER     Same as phyID model number00 = SERDES_CL73 (ComboCore, 65nm)01 = XGXS_16G02 = Hypercore03 = Hyperlite04 = PCIE_G2_PIPE05 = 1.25GBd Serdes06 = SATA207 = QSGMII08 = XGXS10G09 = WarpCore0A = XFICore0B = RXFI, Reduced XFI0C = WarpLite0D = PentaCore0E = ESM0F = SGMII10 = WarpCore 311 = TSC12 = RXAUI13 = EPON ONU14 = DECA15 = HEXATX16 = HEXARX17 = Reserved18 = Core2119 = MERLIN1A = EAGLE1B = FALCON1C = MERLIN_PHY1D = XGXS_CL73, 90nm1E = SERDES_CL73, 90nm
 *     TECH_PROC        0 = 90nm1 = 65nm2 = 40nm3 = 28nm4 = 16nm5-7 = reserved
 *     BONDING          0 = wire bond1 = flip chip2-3 = reserved
 *     REV_NUMBER       0 = rev 01 = rev 1...7 = rev 7
 *     REV_LETTER       0 = rev A1 = rev B2 = rev C3 = rev D
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_SERDESID0r (0x00008310 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_SERDESID0r_SIZE 4

/*
 * This structure should be used to declare and program SERDESID0.
 *
 */
typedef union BCMI_VIPER_XGXS_SERDESID0r_s {
	uint32_t v[1];
	uint32_t serdesid0[1];
	uint32_t _serdesid0;
} BCMI_VIPER_XGXS_SERDESID0r_t;

#define BCMI_VIPER_XGXS_SERDESID0r_CLR(r) (r).serdesid0[0] = 0
#define BCMI_VIPER_XGXS_SERDESID0r_SET(r,d) (r).serdesid0[0] = d
#define BCMI_VIPER_XGXS_SERDESID0r_GET(r) (r).serdesid0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_SERDESID0r_REV_LETTERf_GET(r) ((((r).serdesid0[0]) >> 14) & 0x3)
#define BCMI_VIPER_XGXS_SERDESID0r_REV_LETTERf_SET(r,f) (r).serdesid0[0]=(((r).serdesid0[0] & ~((uint32_t)0x3 << 14)) | ((((uint32_t)f) & 0x3) << 14)) | (3 << (16 + 14))
#define BCMI_VIPER_XGXS_SERDESID0r_REV_NUMBERf_GET(r) ((((r).serdesid0[0]) >> 11) & 0x7)
#define BCMI_VIPER_XGXS_SERDESID0r_REV_NUMBERf_SET(r,f) (r).serdesid0[0]=(((r).serdesid0[0] & ~((uint32_t)0x7 << 11)) | ((((uint32_t)f) & 0x7) << 11)) | (7 << (16 + 11))
#define BCMI_VIPER_XGXS_SERDESID0r_BONDINGf_GET(r) ((((r).serdesid0[0]) >> 9) & 0x3)
#define BCMI_VIPER_XGXS_SERDESID0r_BONDINGf_SET(r,f) (r).serdesid0[0]=(((r).serdesid0[0] & ~((uint32_t)0x3 << 9)) | ((((uint32_t)f) & 0x3) << 9)) | (3 << (16 + 9))
#define BCMI_VIPER_XGXS_SERDESID0r_TECH_PROCf_GET(r) ((((r).serdesid0[0]) >> 6) & 0x7)
#define BCMI_VIPER_XGXS_SERDESID0r_TECH_PROCf_SET(r,f) (r).serdesid0[0]=(((r).serdesid0[0] & ~((uint32_t)0x7 << 6)) | ((((uint32_t)f) & 0x7) << 6)) | (7 << (16 + 6))
#define BCMI_VIPER_XGXS_SERDESID0r_MODEL_NUMBERf_GET(r) (((r).serdesid0[0]) & 0x3f)
#define BCMI_VIPER_XGXS_SERDESID0r_MODEL_NUMBERf_SET(r,f) (r).serdesid0[0]=(((r).serdesid0[0] & ~((uint32_t)0x3f)) | (((uint32_t)f) & 0x3f)) | (0x3f << 16)

/*
 * These macros can be used to access SERDESID0.
 *
 */
#define BCMI_VIPER_XGXS_READ_SERDESID0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_SERDESID0r,(_r._serdesid0))
#define BCMI_VIPER_XGXS_WRITE_SERDESID0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID0r,(_r._serdesid0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_SERDESID0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID0r,(_r._serdesid0))
#define BCMI_VIPER_XGXS_READLN_SERDESID0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_SERDESID0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._serdesid0))
#define BCMI_VIPER_XGXS_WRITELN_SERDESID0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._serdesid0))
#define BCMI_VIPER_XGXS_WRITEALL_SERDESID0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._serdesid0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define SERDESID0r BCMI_VIPER_XGXS_SERDESID0r
#define SERDESID0r_SIZE BCMI_VIPER_XGXS_SERDESID0r_SIZE
typedef BCMI_VIPER_XGXS_SERDESID0r_t SERDESID0r_t;
#define SERDESID0r_CLR BCMI_VIPER_XGXS_SERDESID0r_CLR
#define SERDESID0r_SET BCMI_VIPER_XGXS_SERDESID0r_SET
#define SERDESID0r_GET BCMI_VIPER_XGXS_SERDESID0r_GET
#define SERDESID0r_REV_LETTERf_GET BCMI_VIPER_XGXS_SERDESID0r_REV_LETTERf_GET
#define SERDESID0r_REV_LETTERf_SET BCMI_VIPER_XGXS_SERDESID0r_REV_LETTERf_SET
#define SERDESID0r_REV_NUMBERf_GET BCMI_VIPER_XGXS_SERDESID0r_REV_NUMBERf_GET
#define SERDESID0r_REV_NUMBERf_SET BCMI_VIPER_XGXS_SERDESID0r_REV_NUMBERf_SET
#define SERDESID0r_BONDINGf_GET BCMI_VIPER_XGXS_SERDESID0r_BONDINGf_GET
#define SERDESID0r_BONDINGf_SET BCMI_VIPER_XGXS_SERDESID0r_BONDINGf_SET
#define SERDESID0r_TECH_PROCf_GET BCMI_VIPER_XGXS_SERDESID0r_TECH_PROCf_GET
#define SERDESID0r_TECH_PROCf_SET BCMI_VIPER_XGXS_SERDESID0r_TECH_PROCf_SET
#define SERDESID0r_MODEL_NUMBERf_GET BCMI_VIPER_XGXS_SERDESID0r_MODEL_NUMBERf_GET
#define SERDESID0r_MODEL_NUMBERf_SET BCMI_VIPER_XGXS_SERDESID0r_MODEL_NUMBERf_SET
#define READ_SERDESID0r BCMI_VIPER_XGXS_READ_SERDESID0r
#define WRITE_SERDESID0r BCMI_VIPER_XGXS_WRITE_SERDESID0r
#define MODIFY_SERDESID0r BCMI_VIPER_XGXS_MODIFY_SERDESID0r
#define READLN_SERDESID0r BCMI_VIPER_XGXS_READLN_SERDESID0r
#define WRITELN_SERDESID0r BCMI_VIPER_XGXS_WRITELN_SERDESID0r
#define WRITEALL_SERDESID0r BCMI_VIPER_XGXS_WRITEALL_SERDESID0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_SERDESID0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  SERDESID1
 * BLOCKS:   SERDESID
 * REGADDR:  0x8311
 * DESC:     Serdes ID 1 register
 * RESETVAL: 0x1a00 (6656)
 * ACCESS:   R/O
 * FIELDS:
 *     SCRAMBLER        1 = 8B10B scrambler supported
 *     BRCM_64B66B      1 = Broadcom 64B66B endec supported
 *     PCIE_II          1 = PCIE Gen 2 supported
 *     PCIE             1 = PCIE Gen 1 supported
 *     HIGIGII          1 = HiGig II supported
 *     HIGIG            1 = HiGig supported
 *     CL48             1 = CL48 pcs supported
 *     CL36             1 = CL36 pcs supported
 *     CL73             1 = CL73 AN supported
 *     CL37             1 = CL37 AN supported
 *     MULTIPLICITY     1  = Single2  = Dual4  = Quad5  = Penta6  = Hex8  = Octal9  = Novea12 = Dozen
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_SERDESID1r (0x00008311 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_SERDESID1r_SIZE 4

/*
 * This structure should be used to declare and program SERDESID1.
 *
 */
typedef union BCMI_VIPER_XGXS_SERDESID1r_s {
	uint32_t v[1];
	uint32_t serdesid1[1];
	uint32_t _serdesid1;
} BCMI_VIPER_XGXS_SERDESID1r_t;

#define BCMI_VIPER_XGXS_SERDESID1r_CLR(r) (r).serdesid1[0] = 0
#define BCMI_VIPER_XGXS_SERDESID1r_SET(r,d) (r).serdesid1[0] = d
#define BCMI_VIPER_XGXS_SERDESID1r_GET(r) (r).serdesid1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_SERDESID1r_MULTIPLICITYf_GET(r) ((((r).serdesid1[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_SERDESID1r_MULTIPLICITYf_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_SERDESID1r_CL37f_GET(r) ((((r).serdesid1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_CL37f_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_SERDESID1r_CL73f_GET(r) ((((r).serdesid1[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_CL73f_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_SERDESID1r_CL36f_GET(r) ((((r).serdesid1[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_CL36f_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_SERDESID1r_CL48f_GET(r) ((((r).serdesid1[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_CL48f_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_SERDESID1r_HIGIGf_GET(r) ((((r).serdesid1[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_HIGIGf_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_SERDESID1r_HIGIGIIf_GET(r) ((((r).serdesid1[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_HIGIGIIf_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_SERDESID1r_PCIEf_GET(r) ((((r).serdesid1[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_PCIEf_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_SERDESID1r_PCIE_IIf_GET(r) ((((r).serdesid1[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_PCIE_IIf_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_SERDESID1r_BRCM_64B66Bf_GET(r) ((((r).serdesid1[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_BRCM_64B66Bf_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_SERDESID1r_SCRAMBLERf_GET(r) ((((r).serdesid1[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID1r_SCRAMBLERf_SET(r,f) (r).serdesid1[0]=(((r).serdesid1[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))

/*
 * These macros can be used to access SERDESID1.
 *
 */
#define BCMI_VIPER_XGXS_READ_SERDESID1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_SERDESID1r,(_r._serdesid1))
#define BCMI_VIPER_XGXS_WRITE_SERDESID1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID1r,(_r._serdesid1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_SERDESID1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID1r,(_r._serdesid1))
#define BCMI_VIPER_XGXS_READLN_SERDESID1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_SERDESID1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._serdesid1))
#define BCMI_VIPER_XGXS_WRITELN_SERDESID1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._serdesid1))
#define BCMI_VIPER_XGXS_WRITEALL_SERDESID1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._serdesid1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define SERDESID1r BCMI_VIPER_XGXS_SERDESID1r
#define SERDESID1r_SIZE BCMI_VIPER_XGXS_SERDESID1r_SIZE
typedef BCMI_VIPER_XGXS_SERDESID1r_t SERDESID1r_t;
#define SERDESID1r_CLR BCMI_VIPER_XGXS_SERDESID1r_CLR
#define SERDESID1r_SET BCMI_VIPER_XGXS_SERDESID1r_SET
#define SERDESID1r_GET BCMI_VIPER_XGXS_SERDESID1r_GET
#define SERDESID1r_MULTIPLICITYf_GET BCMI_VIPER_XGXS_SERDESID1r_MULTIPLICITYf_GET
#define SERDESID1r_MULTIPLICITYf_SET BCMI_VIPER_XGXS_SERDESID1r_MULTIPLICITYf_SET
#define SERDESID1r_CL37f_GET BCMI_VIPER_XGXS_SERDESID1r_CL37f_GET
#define SERDESID1r_CL37f_SET BCMI_VIPER_XGXS_SERDESID1r_CL37f_SET
#define SERDESID1r_CL73f_GET BCMI_VIPER_XGXS_SERDESID1r_CL73f_GET
#define SERDESID1r_CL73f_SET BCMI_VIPER_XGXS_SERDESID1r_CL73f_SET
#define SERDESID1r_CL36f_GET BCMI_VIPER_XGXS_SERDESID1r_CL36f_GET
#define SERDESID1r_CL36f_SET BCMI_VIPER_XGXS_SERDESID1r_CL36f_SET
#define SERDESID1r_CL48f_GET BCMI_VIPER_XGXS_SERDESID1r_CL48f_GET
#define SERDESID1r_CL48f_SET BCMI_VIPER_XGXS_SERDESID1r_CL48f_SET
#define SERDESID1r_HIGIGf_GET BCMI_VIPER_XGXS_SERDESID1r_HIGIGf_GET
#define SERDESID1r_HIGIGf_SET BCMI_VIPER_XGXS_SERDESID1r_HIGIGf_SET
#define SERDESID1r_HIGIGIIf_GET BCMI_VIPER_XGXS_SERDESID1r_HIGIGIIf_GET
#define SERDESID1r_HIGIGIIf_SET BCMI_VIPER_XGXS_SERDESID1r_HIGIGIIf_SET
#define SERDESID1r_PCIEf_GET BCMI_VIPER_XGXS_SERDESID1r_PCIEf_GET
#define SERDESID1r_PCIEf_SET BCMI_VIPER_XGXS_SERDESID1r_PCIEf_SET
#define SERDESID1r_PCIE_IIf_GET BCMI_VIPER_XGXS_SERDESID1r_PCIE_IIf_GET
#define SERDESID1r_PCIE_IIf_SET BCMI_VIPER_XGXS_SERDESID1r_PCIE_IIf_SET
#define SERDESID1r_BRCM_64B66Bf_GET BCMI_VIPER_XGXS_SERDESID1r_BRCM_64B66Bf_GET
#define SERDESID1r_BRCM_64B66Bf_SET BCMI_VIPER_XGXS_SERDESID1r_BRCM_64B66Bf_SET
#define SERDESID1r_SCRAMBLERf_GET BCMI_VIPER_XGXS_SERDESID1r_SCRAMBLERf_GET
#define SERDESID1r_SCRAMBLERf_SET BCMI_VIPER_XGXS_SERDESID1r_SCRAMBLERf_SET
#define READ_SERDESID1r BCMI_VIPER_XGXS_READ_SERDESID1r
#define WRITE_SERDESID1r BCMI_VIPER_XGXS_WRITE_SERDESID1r
#define MODIFY_SERDESID1r BCMI_VIPER_XGXS_MODIFY_SERDESID1r
#define READLN_SERDESID1r BCMI_VIPER_XGXS_READLN_SERDESID1r
#define WRITELN_SERDESID1r BCMI_VIPER_XGXS_WRITELN_SERDESID1r
#define WRITEALL_SERDESID1r BCMI_VIPER_XGXS_WRITEALL_SERDESID1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_SERDESID1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  SERDESID2
 * BLOCKS:   SERDESID
 * REGADDR:  0x8312
 * DESC:     Serdes ID 2 register
 * RESETVAL: 0x8007 (32775)
 * ACCESS:   R/O
 * FIELDS:
 *     DR_10M_SL        1 = 10M single lane supported
 *     DR_100M_SL       1 = 100M single lane supported
 *     DR_1G_SL         1 = 1G single lane supported
 *     DR_2P5G_SL       1 = 2.5Gbps single lane supported
 *     DR_5G_4L         1 = 5Gbps 4-lane supported
 *     DR_6G_4L         1 = 6Gbps 4-lane supported
 *     DR_10G_4L        1 = 10Gbps 4-lane supported
 *     DR_12G_4L        1 = 12Gbps 4-lane supported
 *     DR_12_5G_4L      1 = 12.5Gbps 4-lane supported
 *     DR_13G_4L        1 = 13Gbps 4-lane supported
 *     DR_15G_4L        1 = 15Gbps 4-lane supported
 *     DR_16G_4L        1 = 16Gbps 4-lane supported
 *     DR_20G_4L        1 = 20Gbps 4-lane supported
 *     DR_21G_4L        1 = 21Gbps 4-lane supported
 *     DR_25G_4L        1 = 25Gbps 4-lane supported
 *     ID3PRESENT       1 = ID3 register is present
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_SERDESID2r (0x00008312 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_SERDESID2r_SIZE 4

/*
 * This structure should be used to declare and program SERDESID2.
 *
 */
typedef union BCMI_VIPER_XGXS_SERDESID2r_s {
	uint32_t v[1];
	uint32_t serdesid2[1];
	uint32_t _serdesid2;
} BCMI_VIPER_XGXS_SERDESID2r_t;

#define BCMI_VIPER_XGXS_SERDESID2r_CLR(r) (r).serdesid2[0] = 0
#define BCMI_VIPER_XGXS_SERDESID2r_SET(r,d) (r).serdesid2[0] = d
#define BCMI_VIPER_XGXS_SERDESID2r_GET(r) (r).serdesid2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_SERDESID2r_ID3PRESENTf_GET(r) ((((r).serdesid2[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_ID3PRESENTf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_25G_4Lf_GET(r) ((((r).serdesid2[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_25G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_21G_4Lf_GET(r) ((((r).serdesid2[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_21G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_20G_4Lf_GET(r) ((((r).serdesid2[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_20G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_16G_4Lf_GET(r) ((((r).serdesid2[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_16G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_15G_4Lf_GET(r) ((((r).serdesid2[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_15G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_13G_4Lf_GET(r) ((((r).serdesid2[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_13G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_12_5G_4Lf_GET(r) ((((r).serdesid2[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_12_5G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_12G_4Lf_GET(r) ((((r).serdesid2[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_12G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_10G_4Lf_GET(r) ((((r).serdesid2[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_10G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_6G_4Lf_GET(r) ((((r).serdesid2[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_6G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_5G_4Lf_GET(r) ((((r).serdesid2[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_5G_4Lf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_2P5G_SLf_GET(r) ((((r).serdesid2[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_2P5G_SLf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_1G_SLf_GET(r) ((((r).serdesid2[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_1G_SLf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_100M_SLf_GET(r) ((((r).serdesid2[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_100M_SLf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_SERDESID2r_DR_10M_SLf_GET(r) (((r).serdesid2[0]) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID2r_DR_10M_SLf_SET(r,f) (r).serdesid2[0]=(((r).serdesid2[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access SERDESID2.
 *
 */
#define BCMI_VIPER_XGXS_READ_SERDESID2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_SERDESID2r,(_r._serdesid2))
#define BCMI_VIPER_XGXS_WRITE_SERDESID2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID2r,(_r._serdesid2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_SERDESID2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID2r,(_r._serdesid2))
#define BCMI_VIPER_XGXS_READLN_SERDESID2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_SERDESID2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._serdesid2))
#define BCMI_VIPER_XGXS_WRITELN_SERDESID2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._serdesid2))
#define BCMI_VIPER_XGXS_WRITEALL_SERDESID2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._serdesid2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define SERDESID2r BCMI_VIPER_XGXS_SERDESID2r
#define SERDESID2r_SIZE BCMI_VIPER_XGXS_SERDESID2r_SIZE
typedef BCMI_VIPER_XGXS_SERDESID2r_t SERDESID2r_t;
#define SERDESID2r_CLR BCMI_VIPER_XGXS_SERDESID2r_CLR
#define SERDESID2r_SET BCMI_VIPER_XGXS_SERDESID2r_SET
#define SERDESID2r_GET BCMI_VIPER_XGXS_SERDESID2r_GET
#define SERDESID2r_ID3PRESENTf_GET BCMI_VIPER_XGXS_SERDESID2r_ID3PRESENTf_GET
#define SERDESID2r_ID3PRESENTf_SET BCMI_VIPER_XGXS_SERDESID2r_ID3PRESENTf_SET
#define SERDESID2r_DR_25G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_25G_4Lf_GET
#define SERDESID2r_DR_25G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_25G_4Lf_SET
#define SERDESID2r_DR_21G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_21G_4Lf_GET
#define SERDESID2r_DR_21G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_21G_4Lf_SET
#define SERDESID2r_DR_20G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_20G_4Lf_GET
#define SERDESID2r_DR_20G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_20G_4Lf_SET
#define SERDESID2r_DR_16G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_16G_4Lf_GET
#define SERDESID2r_DR_16G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_16G_4Lf_SET
#define SERDESID2r_DR_15G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_15G_4Lf_GET
#define SERDESID2r_DR_15G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_15G_4Lf_SET
#define SERDESID2r_DR_13G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_13G_4Lf_GET
#define SERDESID2r_DR_13G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_13G_4Lf_SET
#define SERDESID2r_DR_12_5G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_12_5G_4Lf_GET
#define SERDESID2r_DR_12_5G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_12_5G_4Lf_SET
#define SERDESID2r_DR_12G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_12G_4Lf_GET
#define SERDESID2r_DR_12G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_12G_4Lf_SET
#define SERDESID2r_DR_10G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_10G_4Lf_GET
#define SERDESID2r_DR_10G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_10G_4Lf_SET
#define SERDESID2r_DR_6G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_6G_4Lf_GET
#define SERDESID2r_DR_6G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_6G_4Lf_SET
#define SERDESID2r_DR_5G_4Lf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_5G_4Lf_GET
#define SERDESID2r_DR_5G_4Lf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_5G_4Lf_SET
#define SERDESID2r_DR_2P5G_SLf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_2P5G_SLf_GET
#define SERDESID2r_DR_2P5G_SLf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_2P5G_SLf_SET
#define SERDESID2r_DR_1G_SLf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_1G_SLf_GET
#define SERDESID2r_DR_1G_SLf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_1G_SLf_SET
#define SERDESID2r_DR_100M_SLf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_100M_SLf_GET
#define SERDESID2r_DR_100M_SLf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_100M_SLf_SET
#define SERDESID2r_DR_10M_SLf_GET BCMI_VIPER_XGXS_SERDESID2r_DR_10M_SLf_GET
#define SERDESID2r_DR_10M_SLf_SET BCMI_VIPER_XGXS_SERDESID2r_DR_10M_SLf_SET
#define READ_SERDESID2r BCMI_VIPER_XGXS_READ_SERDESID2r
#define WRITE_SERDESID2r BCMI_VIPER_XGXS_WRITE_SERDESID2r
#define MODIFY_SERDESID2r BCMI_VIPER_XGXS_MODIFY_SERDESID2r
#define READLN_SERDESID2r BCMI_VIPER_XGXS_READLN_SERDESID2r
#define WRITELN_SERDESID2r BCMI_VIPER_XGXS_WRITELN_SERDESID2r
#define WRITEALL_SERDESID2r BCMI_VIPER_XGXS_WRITEALL_SERDESID2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_SERDESID2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  SERDESID3
 * BLOCKS:   SERDESID
 * REGADDR:  0x8313
 * DESC:     Serdes ID 3 register
 * RESETVAL: 0x1 (1)
 * ACCESS:   R/O
 * FIELDS:
 *     DR_100FX         1 = 100FX single lane supported
 *     DR_2000_SL       1 = 2Gbps single lane supported
 *     DR_4000_SL       1 = 4Gbps single lane supported
 *     DR_5000_SL       1 = 5Gbps single lane supported
 *     DR_6400_SL       1 = 6.4Gbps single lane supported
 *     DR_1200_SL       1 = 1.2Gbps single lane supported
 *     DR_2400_SL       1 = 2.4Gbps single lane supported
 *     DR_31500_4L      1 = 31.5Gbps 4-lane supported
 *     DR_32700_4L      1 = 32.7Gbps 4-lane supported
 *     DR_40000_4L      1 = 40Gbps 4-lane supported
 *     ID4PRESENT       1 = ID4 register is present
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_SERDESID3r (0x00008313 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_SERDESID3r_SIZE 4

/*
 * This structure should be used to declare and program SERDESID3.
 *
 */
typedef union BCMI_VIPER_XGXS_SERDESID3r_s {
	uint32_t v[1];
	uint32_t serdesid3[1];
	uint32_t _serdesid3;
} BCMI_VIPER_XGXS_SERDESID3r_t;

#define BCMI_VIPER_XGXS_SERDESID3r_CLR(r) (r).serdesid3[0] = 0
#define BCMI_VIPER_XGXS_SERDESID3r_SET(r,d) (r).serdesid3[0] = d
#define BCMI_VIPER_XGXS_SERDESID3r_GET(r) (r).serdesid3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_SERDESID3r_ID4PRESENTf_GET(r) ((((r).serdesid3[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_ID4PRESENTf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_40000_4Lf_GET(r) ((((r).serdesid3[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_40000_4Lf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_32700_4Lf_GET(r) ((((r).serdesid3[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_32700_4Lf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_31500_4Lf_GET(r) ((((r).serdesid3[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_31500_4Lf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_2400_SLf_GET(r) ((((r).serdesid3[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_2400_SLf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_1200_SLf_GET(r) ((((r).serdesid3[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_1200_SLf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_6400_SLf_GET(r) ((((r).serdesid3[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_6400_SLf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_5000_SLf_GET(r) ((((r).serdesid3[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_5000_SLf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_4000_SLf_GET(r) ((((r).serdesid3[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_4000_SLf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_2000_SLf_GET(r) ((((r).serdesid3[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_2000_SLf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_SERDESID3r_DR_100FXf_GET(r) (((r).serdesid3[0]) & 0x1)
#define BCMI_VIPER_XGXS_SERDESID3r_DR_100FXf_SET(r,f) (r).serdesid3[0]=(((r).serdesid3[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access SERDESID3.
 *
 */
#define BCMI_VIPER_XGXS_READ_SERDESID3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_SERDESID3r,(_r._serdesid3))
#define BCMI_VIPER_XGXS_WRITE_SERDESID3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID3r,(_r._serdesid3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_SERDESID3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID3r,(_r._serdesid3))
#define BCMI_VIPER_XGXS_READLN_SERDESID3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_SERDESID3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._serdesid3))
#define BCMI_VIPER_XGXS_WRITELN_SERDESID3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._serdesid3))
#define BCMI_VIPER_XGXS_WRITEALL_SERDESID3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_SERDESID3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._serdesid3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define SERDESID3r BCMI_VIPER_XGXS_SERDESID3r
#define SERDESID3r_SIZE BCMI_VIPER_XGXS_SERDESID3r_SIZE
typedef BCMI_VIPER_XGXS_SERDESID3r_t SERDESID3r_t;
#define SERDESID3r_CLR BCMI_VIPER_XGXS_SERDESID3r_CLR
#define SERDESID3r_SET BCMI_VIPER_XGXS_SERDESID3r_SET
#define SERDESID3r_GET BCMI_VIPER_XGXS_SERDESID3r_GET
#define SERDESID3r_ID4PRESENTf_GET BCMI_VIPER_XGXS_SERDESID3r_ID4PRESENTf_GET
#define SERDESID3r_ID4PRESENTf_SET BCMI_VIPER_XGXS_SERDESID3r_ID4PRESENTf_SET
#define SERDESID3r_DR_40000_4Lf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_40000_4Lf_GET
#define SERDESID3r_DR_40000_4Lf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_40000_4Lf_SET
#define SERDESID3r_DR_32700_4Lf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_32700_4Lf_GET
#define SERDESID3r_DR_32700_4Lf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_32700_4Lf_SET
#define SERDESID3r_DR_31500_4Lf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_31500_4Lf_GET
#define SERDESID3r_DR_31500_4Lf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_31500_4Lf_SET
#define SERDESID3r_DR_2400_SLf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_2400_SLf_GET
#define SERDESID3r_DR_2400_SLf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_2400_SLf_SET
#define SERDESID3r_DR_1200_SLf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_1200_SLf_GET
#define SERDESID3r_DR_1200_SLf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_1200_SLf_SET
#define SERDESID3r_DR_6400_SLf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_6400_SLf_GET
#define SERDESID3r_DR_6400_SLf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_6400_SLf_SET
#define SERDESID3r_DR_5000_SLf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_5000_SLf_GET
#define SERDESID3r_DR_5000_SLf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_5000_SLf_SET
#define SERDESID3r_DR_4000_SLf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_4000_SLf_GET
#define SERDESID3r_DR_4000_SLf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_4000_SLf_SET
#define SERDESID3r_DR_2000_SLf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_2000_SLf_GET
#define SERDESID3r_DR_2000_SLf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_2000_SLf_SET
#define SERDESID3r_DR_100FXf_GET BCMI_VIPER_XGXS_SERDESID3r_DR_100FXf_GET
#define SERDESID3r_DR_100FXf_SET BCMI_VIPER_XGXS_SERDESID3r_DR_100FXf_SET
#define READ_SERDESID3r BCMI_VIPER_XGXS_READ_SERDESID3r
#define WRITE_SERDESID3r BCMI_VIPER_XGXS_WRITE_SERDESID3r
#define MODIFY_SERDESID3r BCMI_VIPER_XGXS_MODIFY_SERDESID3r
#define READLN_SERDESID3r BCMI_VIPER_XGXS_READLN_SERDESID3r
#define WRITELN_SERDESID3r BCMI_VIPER_XGXS_WRITELN_SERDESID3r
#define WRITEALL_SERDESID3r BCMI_VIPER_XGXS_WRITEALL_SERDESID3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_SERDESID3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_TPOUT1
 * BLOCKS:   DIGITAL3
 * REGADDR:  0x8327
 * DESC:     Test port out bits 15:0, tpout[15:0]
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     TPOUT1           Test port output bus bits 15:0, selected from register bits SerdesDigital.TestMode.test_monitor_mode2 & SerdesDigital.TestMode.test_monitor_mode1
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_TPOUT1r (0x00008327 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_TPOUT1r_SIZE 4

/*
 * This structure should be used to declare and program DIG_TPOUT1.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_TPOUT1r_s {
	uint32_t v[1];
	uint32_t dig_tpout1[1];
	uint32_t _dig_tpout1;
} BCMI_VIPER_XGXS_DIG_TPOUT1r_t;

#define BCMI_VIPER_XGXS_DIG_TPOUT1r_CLR(r) (r).dig_tpout1[0] = 0
#define BCMI_VIPER_XGXS_DIG_TPOUT1r_SET(r,d) (r).dig_tpout1[0] = d
#define BCMI_VIPER_XGXS_DIG_TPOUT1r_GET(r) (r).dig_tpout1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_TPOUT1r_TPOUT1f_GET(r) (((r).dig_tpout1[0]) & 0xffff)
#define BCMI_VIPER_XGXS_DIG_TPOUT1r_TPOUT1f_SET(r,f) (r).dig_tpout1[0]=(((r).dig_tpout1[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access DIG_TPOUT1.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_TPOUT1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_TPOUT1r,(_r._dig_tpout1))
#define BCMI_VIPER_XGXS_WRITE_DIG_TPOUT1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TPOUT1r,(_r._dig_tpout1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_TPOUT1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TPOUT1r,(_r._dig_tpout1))
#define BCMI_VIPER_XGXS_READLN_DIG_TPOUT1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_TPOUT1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_tpout1))
#define BCMI_VIPER_XGXS_WRITELN_DIG_TPOUT1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TPOUT1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_tpout1))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_TPOUT1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TPOUT1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_tpout1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_TPOUT1r BCMI_VIPER_XGXS_DIG_TPOUT1r
#define DIG_TPOUT1r_SIZE BCMI_VIPER_XGXS_DIG_TPOUT1r_SIZE
typedef BCMI_VIPER_XGXS_DIG_TPOUT1r_t DIG_TPOUT1r_t;
#define DIG_TPOUT1r_CLR BCMI_VIPER_XGXS_DIG_TPOUT1r_CLR
#define DIG_TPOUT1r_SET BCMI_VIPER_XGXS_DIG_TPOUT1r_SET
#define DIG_TPOUT1r_GET BCMI_VIPER_XGXS_DIG_TPOUT1r_GET
#define DIG_TPOUT1r_TPOUT1f_GET BCMI_VIPER_XGXS_DIG_TPOUT1r_TPOUT1f_GET
#define DIG_TPOUT1r_TPOUT1f_SET BCMI_VIPER_XGXS_DIG_TPOUT1r_TPOUT1f_SET
#define READ_DIG_TPOUT1r BCMI_VIPER_XGXS_READ_DIG_TPOUT1r
#define WRITE_DIG_TPOUT1r BCMI_VIPER_XGXS_WRITE_DIG_TPOUT1r
#define MODIFY_DIG_TPOUT1r BCMI_VIPER_XGXS_MODIFY_DIG_TPOUT1r
#define READLN_DIG_TPOUT1r BCMI_VIPER_XGXS_READLN_DIG_TPOUT1r
#define WRITELN_DIG_TPOUT1r BCMI_VIPER_XGXS_WRITELN_DIG_TPOUT1r
#define WRITEALL_DIG_TPOUT1r BCMI_VIPER_XGXS_WRITEALL_DIG_TPOUT1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_TPOUT1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_TPOUT2
 * BLOCKS:   DIGITAL3
 * REGADDR:  0x8328
 * DESC:     Test port out bits 23:8, tpout[23:8]
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     TPOUT2           Test port output bus bits 23:8, selected from register bits SerdesDigital.TestMode.test_monitor_mode2 & SerdesDigital.TestMode.test_monitor_mode1
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_TPOUT2r (0x00008328 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_TPOUT2r_SIZE 4

/*
 * This structure should be used to declare and program DIG_TPOUT2.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_TPOUT2r_s {
	uint32_t v[1];
	uint32_t dig_tpout2[1];
	uint32_t _dig_tpout2;
} BCMI_VIPER_XGXS_DIG_TPOUT2r_t;

#define BCMI_VIPER_XGXS_DIG_TPOUT2r_CLR(r) (r).dig_tpout2[0] = 0
#define BCMI_VIPER_XGXS_DIG_TPOUT2r_SET(r,d) (r).dig_tpout2[0] = d
#define BCMI_VIPER_XGXS_DIG_TPOUT2r_GET(r) (r).dig_tpout2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_TPOUT2r_TPOUT2f_GET(r) (((r).dig_tpout2[0]) & 0xffff)
#define BCMI_VIPER_XGXS_DIG_TPOUT2r_TPOUT2f_SET(r,f) (r).dig_tpout2[0]=(((r).dig_tpout2[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access DIG_TPOUT2.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_TPOUT2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_TPOUT2r,(_r._dig_tpout2))
#define BCMI_VIPER_XGXS_WRITE_DIG_TPOUT2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TPOUT2r,(_r._dig_tpout2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_TPOUT2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TPOUT2r,(_r._dig_tpout2))
#define BCMI_VIPER_XGXS_READLN_DIG_TPOUT2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_TPOUT2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_tpout2))
#define BCMI_VIPER_XGXS_WRITELN_DIG_TPOUT2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TPOUT2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_tpout2))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_TPOUT2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_TPOUT2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_tpout2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_TPOUT2r BCMI_VIPER_XGXS_DIG_TPOUT2r
#define DIG_TPOUT2r_SIZE BCMI_VIPER_XGXS_DIG_TPOUT2r_SIZE
typedef BCMI_VIPER_XGXS_DIG_TPOUT2r_t DIG_TPOUT2r_t;
#define DIG_TPOUT2r_CLR BCMI_VIPER_XGXS_DIG_TPOUT2r_CLR
#define DIG_TPOUT2r_SET BCMI_VIPER_XGXS_DIG_TPOUT2r_SET
#define DIG_TPOUT2r_GET BCMI_VIPER_XGXS_DIG_TPOUT2r_GET
#define DIG_TPOUT2r_TPOUT2f_GET BCMI_VIPER_XGXS_DIG_TPOUT2r_TPOUT2f_GET
#define DIG_TPOUT2r_TPOUT2f_SET BCMI_VIPER_XGXS_DIG_TPOUT2r_TPOUT2f_SET
#define READ_DIG_TPOUT2r BCMI_VIPER_XGXS_READ_DIG_TPOUT2r
#define WRITE_DIG_TPOUT2r BCMI_VIPER_XGXS_WRITE_DIG_TPOUT2r
#define MODIFY_DIG_TPOUT2r BCMI_VIPER_XGXS_MODIFY_DIG_TPOUT2r
#define READLN_DIG_TPOUT2r BCMI_VIPER_XGXS_READLN_DIG_TPOUT2r
#define WRITELN_DIG_TPOUT2r BCMI_VIPER_XGXS_WRITELN_DIG_TPOUT2r
#define WRITEALL_DIG_TPOUT2r BCMI_VIPER_XGXS_WRITEALL_DIG_TPOUT2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_TPOUT2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_MISC3
 * BLOCKS:   DIGITAL4
 * REGADDR:  0x833c
 * DESC:     Miscellaneous 3 control register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     LANEDISABLE      TX lane disable bit. When set, TX output is disabled.default value is controlled by laneDisable_strap.
 *     FIFO_IPG_CYA     
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_MISC3r (0x0000833c | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_MISC3r_SIZE 4

/*
 * This structure should be used to declare and program DIG_MISC3.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_MISC3r_s {
	uint32_t v[1];
	uint32_t dig_misc3[1];
	uint32_t _dig_misc3;
} BCMI_VIPER_XGXS_DIG_MISC3r_t;

#define BCMI_VIPER_XGXS_DIG_MISC3r_CLR(r) (r).dig_misc3[0] = 0
#define BCMI_VIPER_XGXS_DIG_MISC3r_SET(r,d) (r).dig_misc3[0] = d
#define BCMI_VIPER_XGXS_DIG_MISC3r_GET(r) (r).dig_misc3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_MISC3r_FIFO_IPG_CYAf_GET(r) ((((r).dig_misc3[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC3r_FIFO_IPG_CYAf_SET(r,f) (r).dig_misc3[0]=(((r).dig_misc3[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_DIG_MISC3r_LANEDISABLEf_GET(r) ((((r).dig_misc3[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC3r_LANEDISABLEf_SET(r,f) (r).dig_misc3[0]=(((r).dig_misc3[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))

/*
 * These macros can be used to access DIG_MISC3.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_MISC3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC3r,(_r._dig_misc3))
#define BCMI_VIPER_XGXS_WRITE_DIG_MISC3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC3r,(_r._dig_misc3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_MISC3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC3r,(_r._dig_misc3))
#define BCMI_VIPER_XGXS_READLN_DIG_MISC3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc3))
#define BCMI_VIPER_XGXS_WRITELN_DIG_MISC3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc3))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_MISC3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_misc3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_MISC3r BCMI_VIPER_XGXS_DIG_MISC3r
#define DIG_MISC3r_SIZE BCMI_VIPER_XGXS_DIG_MISC3r_SIZE
typedef BCMI_VIPER_XGXS_DIG_MISC3r_t DIG_MISC3r_t;
#define DIG_MISC3r_CLR BCMI_VIPER_XGXS_DIG_MISC3r_CLR
#define DIG_MISC3r_SET BCMI_VIPER_XGXS_DIG_MISC3r_SET
#define DIG_MISC3r_GET BCMI_VIPER_XGXS_DIG_MISC3r_GET
#define DIG_MISC3r_FIFO_IPG_CYAf_GET BCMI_VIPER_XGXS_DIG_MISC3r_FIFO_IPG_CYAf_GET
#define DIG_MISC3r_FIFO_IPG_CYAf_SET BCMI_VIPER_XGXS_DIG_MISC3r_FIFO_IPG_CYAf_SET
#define DIG_MISC3r_LANEDISABLEf_GET BCMI_VIPER_XGXS_DIG_MISC3r_LANEDISABLEf_GET
#define DIG_MISC3r_LANEDISABLEf_SET BCMI_VIPER_XGXS_DIG_MISC3r_LANEDISABLEf_SET
#define READ_DIG_MISC3r BCMI_VIPER_XGXS_READ_DIG_MISC3r
#define WRITE_DIG_MISC3r BCMI_VIPER_XGXS_WRITE_DIG_MISC3r
#define MODIFY_DIG_MISC3r BCMI_VIPER_XGXS_MODIFY_DIG_MISC3r
#define READLN_DIG_MISC3r BCMI_VIPER_XGXS_READLN_DIG_MISC3r
#define WRITELN_DIG_MISC3r BCMI_VIPER_XGXS_WRITELN_DIG_MISC3r
#define WRITEALL_DIG_MISC3r BCMI_VIPER_XGXS_WRITEALL_DIG_MISC3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_MISC3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_MISC5
 * BLOCKS:   DIGITAL4
 * REGADDR:  0x833e
 * DESC:     Miscelaneous 5 control register
 * RESETVAL: 0xc000 (49152)
 * ACCESS:   R/W
 * FIELDS:
 *     LPI_EN_TX        Tx LPI enable force value for cl49 and cl36 LPI force enable is located in 0x8390[2]
 *     LPI_EN_RX        Rx LPI enable force value for cl49 and cl36 LPI force enable is located in 0x8390[2]
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_MISC5r (0x0000833e | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_MISC5r_SIZE 4

/*
 * This structure should be used to declare and program DIG_MISC5.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_MISC5r_s {
	uint32_t v[1];
	uint32_t dig_misc5[1];
	uint32_t _dig_misc5;
} BCMI_VIPER_XGXS_DIG_MISC5r_t;

#define BCMI_VIPER_XGXS_DIG_MISC5r_CLR(r) (r).dig_misc5[0] = 0
#define BCMI_VIPER_XGXS_DIG_MISC5r_SET(r,d) (r).dig_misc5[0] = d
#define BCMI_VIPER_XGXS_DIG_MISC5r_GET(r) (r).dig_misc5[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_MISC5r_LPI_EN_RXf_GET(r) ((((r).dig_misc5[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC5r_LPI_EN_RXf_SET(r,f) (r).dig_misc5[0]=(((r).dig_misc5[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_DIG_MISC5r_LPI_EN_TXf_GET(r) ((((r).dig_misc5[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_DIG_MISC5r_LPI_EN_TXf_SET(r,f) (r).dig_misc5[0]=(((r).dig_misc5[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))

/*
 * These macros can be used to access DIG_MISC5.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_MISC5r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC5r,(_r._dig_misc5))
#define BCMI_VIPER_XGXS_WRITE_DIG_MISC5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC5r,(_r._dig_misc5)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_MISC5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC5r,(_r._dig_misc5))
#define BCMI_VIPER_XGXS_READLN_DIG_MISC5r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc5))
#define BCMI_VIPER_XGXS_WRITELN_DIG_MISC5r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc5))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_MISC5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC5r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_misc5))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_MISC5r BCMI_VIPER_XGXS_DIG_MISC5r
#define DIG_MISC5r_SIZE BCMI_VIPER_XGXS_DIG_MISC5r_SIZE
typedef BCMI_VIPER_XGXS_DIG_MISC5r_t DIG_MISC5r_t;
#define DIG_MISC5r_CLR BCMI_VIPER_XGXS_DIG_MISC5r_CLR
#define DIG_MISC5r_SET BCMI_VIPER_XGXS_DIG_MISC5r_SET
#define DIG_MISC5r_GET BCMI_VIPER_XGXS_DIG_MISC5r_GET
#define DIG_MISC5r_LPI_EN_RXf_GET BCMI_VIPER_XGXS_DIG_MISC5r_LPI_EN_RXf_GET
#define DIG_MISC5r_LPI_EN_RXf_SET BCMI_VIPER_XGXS_DIG_MISC5r_LPI_EN_RXf_SET
#define DIG_MISC5r_LPI_EN_TXf_GET BCMI_VIPER_XGXS_DIG_MISC5r_LPI_EN_TXf_GET
#define DIG_MISC5r_LPI_EN_TXf_SET BCMI_VIPER_XGXS_DIG_MISC5r_LPI_EN_TXf_SET
#define READ_DIG_MISC5r BCMI_VIPER_XGXS_READ_DIG_MISC5r
#define WRITE_DIG_MISC5r BCMI_VIPER_XGXS_WRITE_DIG_MISC5r
#define MODIFY_DIG_MISC5r BCMI_VIPER_XGXS_MODIFY_DIG_MISC5r
#define READLN_DIG_MISC5r BCMI_VIPER_XGXS_READLN_DIG_MISC5r
#define WRITELN_DIG_MISC5r BCMI_VIPER_XGXS_WRITELN_DIG_MISC5r
#define WRITEALL_DIG_MISC5r BCMI_VIPER_XGXS_WRITEALL_DIG_MISC5r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_MISC5r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DIG_MISC8
 * BLOCKS:   DIGITAL5
 * REGADDR:  0x834a
 * DESC:     Misc8
 * RESETVAL: 0x1 (1)
 * ACCESS:   R/W
 * FIELDS:
 *     FORCE_OSCDR_MODE force os cdr mode value4'h1   - osx24'h2   - osx44'h3   - osx5
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DIG_MISC8r (0x0000834a | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DIG_MISC8r_SIZE 4

/*
 * This structure should be used to declare and program DIG_MISC8.
 *
 */
typedef union BCMI_VIPER_XGXS_DIG_MISC8r_s {
	uint32_t v[1];
	uint32_t dig_misc8[1];
	uint32_t _dig_misc8;
} BCMI_VIPER_XGXS_DIG_MISC8r_t;

#define BCMI_VIPER_XGXS_DIG_MISC8r_CLR(r) (r).dig_misc8[0] = 0
#define BCMI_VIPER_XGXS_DIG_MISC8r_SET(r,d) (r).dig_misc8[0] = d
#define BCMI_VIPER_XGXS_DIG_MISC8r_GET(r) (r).dig_misc8[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DIG_MISC8r_FORCE_OSCDR_MODEf_GET(r) (((r).dig_misc8[0]) & 0xf)
#define BCMI_VIPER_XGXS_DIG_MISC8r_FORCE_OSCDR_MODEf_SET(r,f) (r).dig_misc8[0]=(((r).dig_misc8[0] & ~((uint32_t)0xf)) | (((uint32_t)f) & 0xf)) | (0xf << 16)

/*
 * These macros can be used to access DIG_MISC8.
 *
 */
#define BCMI_VIPER_XGXS_READ_DIG_MISC8r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC8r,(_r._dig_misc8))
#define BCMI_VIPER_XGXS_WRITE_DIG_MISC8r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC8r,(_r._dig_misc8)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DIG_MISC8r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC8r,(_r._dig_misc8))
#define BCMI_VIPER_XGXS_READLN_DIG_MISC8r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DIG_MISC8r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc8))
#define BCMI_VIPER_XGXS_WRITELN_DIG_MISC8r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC8r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dig_misc8))
#define BCMI_VIPER_XGXS_WRITEALL_DIG_MISC8r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DIG_MISC8r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dig_misc8))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DIG_MISC8r BCMI_VIPER_XGXS_DIG_MISC8r
#define DIG_MISC8r_SIZE BCMI_VIPER_XGXS_DIG_MISC8r_SIZE
typedef BCMI_VIPER_XGXS_DIG_MISC8r_t DIG_MISC8r_t;
#define DIG_MISC8r_CLR BCMI_VIPER_XGXS_DIG_MISC8r_CLR
#define DIG_MISC8r_SET BCMI_VIPER_XGXS_DIG_MISC8r_SET
#define DIG_MISC8r_GET BCMI_VIPER_XGXS_DIG_MISC8r_GET
#define DIG_MISC8r_FORCE_OSCDR_MODEf_GET BCMI_VIPER_XGXS_DIG_MISC8r_FORCE_OSCDR_MODEf_GET
#define DIG_MISC8r_FORCE_OSCDR_MODEf_SET BCMI_VIPER_XGXS_DIG_MISC8r_FORCE_OSCDR_MODEf_SET
#define READ_DIG_MISC8r BCMI_VIPER_XGXS_READ_DIG_MISC8r
#define WRITE_DIG_MISC8r BCMI_VIPER_XGXS_WRITE_DIG_MISC8r
#define MODIFY_DIG_MISC8r BCMI_VIPER_XGXS_MODIFY_DIG_MISC8r
#define READLN_DIG_MISC8r BCMI_VIPER_XGXS_READLN_DIG_MISC8r
#define WRITELN_DIG_MISC8r BCMI_VIPER_XGXS_WRITELN_DIG_MISC8r
#define WRITEALL_DIG_MISC8r BCMI_VIPER_XGXS_WRITEALL_DIG_MISC8r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DIG_MISC8r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_CTL1
 * BLOCKS:   FX100
 * REGADDR:  0x8400
 * DESC:     100FX control register 1
 * RESETVAL: 0x14a (330)
 * ACCESS:   R/W
 * FIELDS:
 *     ENABLE           1 = select 100-FX mode0 = disable 100-FX mode
 *     FULL_DUPLEX      1 = 100-FX serdes full-duplex0 = 100-FX serdes half-duplex
 *     AUTO_DETECT_FX_MODE 1 = enable autodetect 100fx0 = disable autodetect 100fx
 *     FAR_END_FAULT_EN 1 = enable far-end fault0 = disable far-end fault
 *     FORCE_RX_QUAL    1 = always compare 2 surrounding bits with sample to filter noise0 = normal operation
 *     DISABLE_RX_QUAL  1 = always use sample bit without filtering0 = normal operation
 *     RXDATA_SEL       selects the sample bit out of 10 bits for fx100 RX data
 *     DATA_SAMPLER_EN  1 = Enable 100fx data sampler0 = Disable 100fx data sampler
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_CTL1r (0x00008400 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_CTL1r_SIZE 4

/*
 * This structure should be used to declare and program FX100_CTL1.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_CTL1r_s {
	uint32_t v[1];
	uint32_t fx100_ctl1[1];
	uint32_t _fx100_ctl1;
} BCMI_VIPER_XGXS_FX100_CTL1r_t;

#define BCMI_VIPER_XGXS_FX100_CTL1r_CLR(r) (r).fx100_ctl1[0] = 0
#define BCMI_VIPER_XGXS_FX100_CTL1r_SET(r,d) (r).fx100_ctl1[0] = d
#define BCMI_VIPER_XGXS_FX100_CTL1r_GET(r) (r).fx100_ctl1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_CTL1r_DATA_SAMPLER_ENf_GET(r) ((((r).fx100_ctl1[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL1r_DATA_SAMPLER_ENf_SET(r,f) (r).fx100_ctl1[0]=(((r).fx100_ctl1[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_FX100_CTL1r_RXDATA_SELf_GET(r) ((((r).fx100_ctl1[0]) >> 6) & 0xf)
#define BCMI_VIPER_XGXS_FX100_CTL1r_RXDATA_SELf_SET(r,f) (r).fx100_ctl1[0]=(((r).fx100_ctl1[0] & ~((uint32_t)0xf << 6)) | ((((uint32_t)f) & 0xf) << 6)) | (15 << (16 + 6))
#define BCMI_VIPER_XGXS_FX100_CTL1r_DISABLE_RX_QUALf_GET(r) ((((r).fx100_ctl1[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL1r_DISABLE_RX_QUALf_SET(r,f) (r).fx100_ctl1[0]=(((r).fx100_ctl1[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_FX100_CTL1r_FORCE_RX_QUALf_GET(r) ((((r).fx100_ctl1[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL1r_FORCE_RX_QUALf_SET(r,f) (r).fx100_ctl1[0]=(((r).fx100_ctl1[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_FX100_CTL1r_FAR_END_FAULT_ENf_GET(r) ((((r).fx100_ctl1[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL1r_FAR_END_FAULT_ENf_SET(r,f) (r).fx100_ctl1[0]=(((r).fx100_ctl1[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_FX100_CTL1r_AUTO_DETECT_FX_MODEf_GET(r) ((((r).fx100_ctl1[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL1r_AUTO_DETECT_FX_MODEf_SET(r,f) (r).fx100_ctl1[0]=(((r).fx100_ctl1[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_FX100_CTL1r_FULL_DUPLEXf_GET(r) ((((r).fx100_ctl1[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL1r_FULL_DUPLEXf_SET(r,f) (r).fx100_ctl1[0]=(((r).fx100_ctl1[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_FX100_CTL1r_ENABLEf_GET(r) (((r).fx100_ctl1[0]) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL1r_ENABLEf_SET(r,f) (r).fx100_ctl1[0]=(((r).fx100_ctl1[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access FX100_CTL1.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_CTL1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_CTL1r,(_r._fx100_ctl1))
#define BCMI_VIPER_XGXS_WRITE_FX100_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL1r,(_r._fx100_ctl1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL1r,(_r._fx100_ctl1))
#define BCMI_VIPER_XGXS_READLN_FX100_CTL1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_ctl1))
#define BCMI_VIPER_XGXS_WRITELN_FX100_CTL1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_ctl1))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_ctl1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_CTL1r BCMI_VIPER_XGXS_FX100_CTL1r
#define FX100_CTL1r_SIZE BCMI_VIPER_XGXS_FX100_CTL1r_SIZE
typedef BCMI_VIPER_XGXS_FX100_CTL1r_t FX100_CTL1r_t;
#define FX100_CTL1r_CLR BCMI_VIPER_XGXS_FX100_CTL1r_CLR
#define FX100_CTL1r_SET BCMI_VIPER_XGXS_FX100_CTL1r_SET
#define FX100_CTL1r_GET BCMI_VIPER_XGXS_FX100_CTL1r_GET
#define FX100_CTL1r_DATA_SAMPLER_ENf_GET BCMI_VIPER_XGXS_FX100_CTL1r_DATA_SAMPLER_ENf_GET
#define FX100_CTL1r_DATA_SAMPLER_ENf_SET BCMI_VIPER_XGXS_FX100_CTL1r_DATA_SAMPLER_ENf_SET
#define FX100_CTL1r_RXDATA_SELf_GET BCMI_VIPER_XGXS_FX100_CTL1r_RXDATA_SELf_GET
#define FX100_CTL1r_RXDATA_SELf_SET BCMI_VIPER_XGXS_FX100_CTL1r_RXDATA_SELf_SET
#define FX100_CTL1r_DISABLE_RX_QUALf_GET BCMI_VIPER_XGXS_FX100_CTL1r_DISABLE_RX_QUALf_GET
#define FX100_CTL1r_DISABLE_RX_QUALf_SET BCMI_VIPER_XGXS_FX100_CTL1r_DISABLE_RX_QUALf_SET
#define FX100_CTL1r_FORCE_RX_QUALf_GET BCMI_VIPER_XGXS_FX100_CTL1r_FORCE_RX_QUALf_GET
#define FX100_CTL1r_FORCE_RX_QUALf_SET BCMI_VIPER_XGXS_FX100_CTL1r_FORCE_RX_QUALf_SET
#define FX100_CTL1r_FAR_END_FAULT_ENf_GET BCMI_VIPER_XGXS_FX100_CTL1r_FAR_END_FAULT_ENf_GET
#define FX100_CTL1r_FAR_END_FAULT_ENf_SET BCMI_VIPER_XGXS_FX100_CTL1r_FAR_END_FAULT_ENf_SET
#define FX100_CTL1r_AUTO_DETECT_FX_MODEf_GET BCMI_VIPER_XGXS_FX100_CTL1r_AUTO_DETECT_FX_MODEf_GET
#define FX100_CTL1r_AUTO_DETECT_FX_MODEf_SET BCMI_VIPER_XGXS_FX100_CTL1r_AUTO_DETECT_FX_MODEf_SET
#define FX100_CTL1r_FULL_DUPLEXf_GET BCMI_VIPER_XGXS_FX100_CTL1r_FULL_DUPLEXf_GET
#define FX100_CTL1r_FULL_DUPLEXf_SET BCMI_VIPER_XGXS_FX100_CTL1r_FULL_DUPLEXf_SET
#define FX100_CTL1r_ENABLEf_GET BCMI_VIPER_XGXS_FX100_CTL1r_ENABLEf_GET
#define FX100_CTL1r_ENABLEf_SET BCMI_VIPER_XGXS_FX100_CTL1r_ENABLEf_SET
#define READ_FX100_CTL1r BCMI_VIPER_XGXS_READ_FX100_CTL1r
#define WRITE_FX100_CTL1r BCMI_VIPER_XGXS_WRITE_FX100_CTL1r
#define MODIFY_FX100_CTL1r BCMI_VIPER_XGXS_MODIFY_FX100_CTL1r
#define READLN_FX100_CTL1r BCMI_VIPER_XGXS_READLN_FX100_CTL1r
#define WRITELN_FX100_CTL1r BCMI_VIPER_XGXS_WRITELN_FX100_CTL1r
#define WRITEALL_FX100_CTL1r BCMI_VIPER_XGXS_WRITEALL_FX100_CTL1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_CTL1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_CTL2
 * BLOCKS:   FX100
 * REGADDR:  0x8401
 * DESC:     100FX control register 2
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     EXTEND_PKT_SIZE  1 = allow reception of extended length packets0 = allow normal length Ethernet packets only
 *     BYPASS_RXFIFO    1 = bypass 100-FX receive fifo0 = normal operation
 *     RESET_RXFIFO     1 = reset 100-FX receive fifo. Fifo will remain in reset until this bit is cleared with a software write.0 = normal operation
 *     MODE_CHG_NRST    1 = does not reset Serdes when switching in/out of 100-FX mode0 = reset Serdes when switching in/out of 100-FX mode
 *     MII_RXC_OUT_SM_RST 1 = resets the mii_rxc_out clock output state machine0 = normal operation
 *     MII_RXC_OUT_SW_EN 1 = control the mii_rxc_out clock output through bit[6] of this register0 = normal operation
 *     MII_RXC_OUT_SW_REF 1 = switch the mii_rxc_out clk to clk25 when bit[5] of this register is enabled0 = switch the mii_rxc_out clk to non-clk25 when bit[5] of this registser is enabled
 *     CLK_OUT_1000_SW_EN 1 = control the clk_out_1000 clock output through bit[8] of this register0 = normal operation
 *     CLK_OUT_1000_SW_DEF 1 = switch the clk_out_1000 clk to clk25 when bit[7] of this register is enabled0 = switch the clk_out_1000 clk to non-clk25 when bit[7] of this registser is enabled
 *     PLL_CLK125_SW_EN 1 = control the pll_clk125 clock output through bit[10] of this register0 = normal operation
 *     PLL_CLK125_SW_REF 1 = switch the pll_clk125 clk to clk25 when bit[9] of this register is enabled0 = switch the pll_clk125 clk to non-clk25 when bit[9] of this registser is enabled
 *     PING_PONG_DISABLE 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_CTL2r (0x00008401 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_CTL2r_SIZE 4

/*
 * This structure should be used to declare and program FX100_CTL2.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_CTL2r_s {
	uint32_t v[1];
	uint32_t fx100_ctl2[1];
	uint32_t _fx100_ctl2;
} BCMI_VIPER_XGXS_FX100_CTL2r_t;

#define BCMI_VIPER_XGXS_FX100_CTL2r_CLR(r) (r).fx100_ctl2[0] = 0
#define BCMI_VIPER_XGXS_FX100_CTL2r_SET(r,d) (r).fx100_ctl2[0] = d
#define BCMI_VIPER_XGXS_FX100_CTL2r_GET(r) (r).fx100_ctl2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_CTL2r_PING_PONG_DISABLEf_GET(r) ((((r).fx100_ctl2[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_PING_PONG_DISABLEf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_FX100_CTL2r_PLL_CLK125_SW_REFf_GET(r) ((((r).fx100_ctl2[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_PLL_CLK125_SW_REFf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_FX100_CTL2r_PLL_CLK125_SW_ENf_GET(r) ((((r).fx100_ctl2[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_PLL_CLK125_SW_ENf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_FX100_CTL2r_CLK_OUT_1000_SW_DEFf_GET(r) ((((r).fx100_ctl2[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_CLK_OUT_1000_SW_DEFf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_FX100_CTL2r_CLK_OUT_1000_SW_ENf_GET(r) ((((r).fx100_ctl2[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_CLK_OUT_1000_SW_ENf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SW_REFf_GET(r) ((((r).fx100_ctl2[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SW_REFf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SW_ENf_GET(r) ((((r).fx100_ctl2[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SW_ENf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SM_RSTf_GET(r) ((((r).fx100_ctl2[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SM_RSTf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_FX100_CTL2r_MODE_CHG_NRSTf_GET(r) ((((r).fx100_ctl2[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_MODE_CHG_NRSTf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_FX100_CTL2r_RESET_RXFIFOf_GET(r) ((((r).fx100_ctl2[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_RESET_RXFIFOf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_FX100_CTL2r_BYPASS_RXFIFOf_GET(r) ((((r).fx100_ctl2[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_BYPASS_RXFIFOf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_FX100_CTL2r_EXTEND_PKT_SIZEf_GET(r) (((r).fx100_ctl2[0]) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL2r_EXTEND_PKT_SIZEf_SET(r,f) (r).fx100_ctl2[0]=(((r).fx100_ctl2[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access FX100_CTL2.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_CTL2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_CTL2r,(_r._fx100_ctl2))
#define BCMI_VIPER_XGXS_WRITE_FX100_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL2r,(_r._fx100_ctl2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL2r,(_r._fx100_ctl2))
#define BCMI_VIPER_XGXS_READLN_FX100_CTL2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_ctl2))
#define BCMI_VIPER_XGXS_WRITELN_FX100_CTL2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_ctl2))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_ctl2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_CTL2r BCMI_VIPER_XGXS_FX100_CTL2r
#define FX100_CTL2r_SIZE BCMI_VIPER_XGXS_FX100_CTL2r_SIZE
typedef BCMI_VIPER_XGXS_FX100_CTL2r_t FX100_CTL2r_t;
#define FX100_CTL2r_CLR BCMI_VIPER_XGXS_FX100_CTL2r_CLR
#define FX100_CTL2r_SET BCMI_VIPER_XGXS_FX100_CTL2r_SET
#define FX100_CTL2r_GET BCMI_VIPER_XGXS_FX100_CTL2r_GET
#define FX100_CTL2r_PING_PONG_DISABLEf_GET BCMI_VIPER_XGXS_FX100_CTL2r_PING_PONG_DISABLEf_GET
#define FX100_CTL2r_PING_PONG_DISABLEf_SET BCMI_VIPER_XGXS_FX100_CTL2r_PING_PONG_DISABLEf_SET
#define FX100_CTL2r_PLL_CLK125_SW_REFf_GET BCMI_VIPER_XGXS_FX100_CTL2r_PLL_CLK125_SW_REFf_GET
#define FX100_CTL2r_PLL_CLK125_SW_REFf_SET BCMI_VIPER_XGXS_FX100_CTL2r_PLL_CLK125_SW_REFf_SET
#define FX100_CTL2r_PLL_CLK125_SW_ENf_GET BCMI_VIPER_XGXS_FX100_CTL2r_PLL_CLK125_SW_ENf_GET
#define FX100_CTL2r_PLL_CLK125_SW_ENf_SET BCMI_VIPER_XGXS_FX100_CTL2r_PLL_CLK125_SW_ENf_SET
#define FX100_CTL2r_CLK_OUT_1000_SW_DEFf_GET BCMI_VIPER_XGXS_FX100_CTL2r_CLK_OUT_1000_SW_DEFf_GET
#define FX100_CTL2r_CLK_OUT_1000_SW_DEFf_SET BCMI_VIPER_XGXS_FX100_CTL2r_CLK_OUT_1000_SW_DEFf_SET
#define FX100_CTL2r_CLK_OUT_1000_SW_ENf_GET BCMI_VIPER_XGXS_FX100_CTL2r_CLK_OUT_1000_SW_ENf_GET
#define FX100_CTL2r_CLK_OUT_1000_SW_ENf_SET BCMI_VIPER_XGXS_FX100_CTL2r_CLK_OUT_1000_SW_ENf_SET
#define FX100_CTL2r_MII_RXC_OUT_SW_REFf_GET BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SW_REFf_GET
#define FX100_CTL2r_MII_RXC_OUT_SW_REFf_SET BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SW_REFf_SET
#define FX100_CTL2r_MII_RXC_OUT_SW_ENf_GET BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SW_ENf_GET
#define FX100_CTL2r_MII_RXC_OUT_SW_ENf_SET BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SW_ENf_SET
#define FX100_CTL2r_MII_RXC_OUT_SM_RSTf_GET BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SM_RSTf_GET
#define FX100_CTL2r_MII_RXC_OUT_SM_RSTf_SET BCMI_VIPER_XGXS_FX100_CTL2r_MII_RXC_OUT_SM_RSTf_SET
#define FX100_CTL2r_MODE_CHG_NRSTf_GET BCMI_VIPER_XGXS_FX100_CTL2r_MODE_CHG_NRSTf_GET
#define FX100_CTL2r_MODE_CHG_NRSTf_SET BCMI_VIPER_XGXS_FX100_CTL2r_MODE_CHG_NRSTf_SET
#define FX100_CTL2r_RESET_RXFIFOf_GET BCMI_VIPER_XGXS_FX100_CTL2r_RESET_RXFIFOf_GET
#define FX100_CTL2r_RESET_RXFIFOf_SET BCMI_VIPER_XGXS_FX100_CTL2r_RESET_RXFIFOf_SET
#define FX100_CTL2r_BYPASS_RXFIFOf_GET BCMI_VIPER_XGXS_FX100_CTL2r_BYPASS_RXFIFOf_GET
#define FX100_CTL2r_BYPASS_RXFIFOf_SET BCMI_VIPER_XGXS_FX100_CTL2r_BYPASS_RXFIFOf_SET
#define FX100_CTL2r_EXTEND_PKT_SIZEf_GET BCMI_VIPER_XGXS_FX100_CTL2r_EXTEND_PKT_SIZEf_GET
#define FX100_CTL2r_EXTEND_PKT_SIZEf_SET BCMI_VIPER_XGXS_FX100_CTL2r_EXTEND_PKT_SIZEf_SET
#define READ_FX100_CTL2r BCMI_VIPER_XGXS_READ_FX100_CTL2r
#define WRITE_FX100_CTL2r BCMI_VIPER_XGXS_WRITE_FX100_CTL2r
#define MODIFY_FX100_CTL2r BCMI_VIPER_XGXS_MODIFY_FX100_CTL2r
#define READLN_FX100_CTL2r BCMI_VIPER_XGXS_READLN_FX100_CTL2r
#define WRITELN_FX100_CTL2r BCMI_VIPER_XGXS_WRITELN_FX100_CTL2r
#define WRITEALL_FX100_CTL2r BCMI_VIPER_XGXS_WRITEALL_FX100_CTL2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_CTL2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_CTL3
 * BLOCKS:   FX100
 * REGADDR:  0x8402
 * DESC:     100FX control register 3
 * RESETVAL: 0x800 (2048)
 * ACCESS:   R/W
 * FIELDS:
 *     FAST_TIMERS      1 = speed up timers to acquire lock and link (test vectors and simulation) in 100FX mode0 = normal operation
 *     FAST_UNLOCK_TIMER 1 = speed up unlock timer in 100FX mode0 = normal operation
 *     FORCE_LOCK       1 = force lock in 100FX mode0 = normal operation
 *     FORCE_LINK       1 = force link in 100FX mode0 = normal operation
 *     BYPASS_ALIGNMENT 1 = bypass 5B code group alignment in 100FX mode0 = normal operation
 *     BYPASS_ENCODER   1 = bypass 4B5B encoder in 100FX mode0 = normal operation
 *     BYPASS_NRZ       1 =bypass NRZ encoder in 100FX mode0 = normal operation
 *     CORRELATOR_DISABLE disable idle correlator
 *     NUMBER_OF_IDLE   
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_CTL3r (0x00008402 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_CTL3r_SIZE 4

/*
 * This structure should be used to declare and program FX100_CTL3.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_CTL3r_s {
	uint32_t v[1];
	uint32_t fx100_ctl3[1];
	uint32_t _fx100_ctl3;
} BCMI_VIPER_XGXS_FX100_CTL3r_t;

#define BCMI_VIPER_XGXS_FX100_CTL3r_CLR(r) (r).fx100_ctl3[0] = 0
#define BCMI_VIPER_XGXS_FX100_CTL3r_SET(r,d) (r).fx100_ctl3[0] = d
#define BCMI_VIPER_XGXS_FX100_CTL3r_GET(r) (r).fx100_ctl3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_CTL3r_NUMBER_OF_IDLEf_GET(r) ((((r).fx100_ctl3[0]) >> 8) & 0xff)
#define BCMI_VIPER_XGXS_FX100_CTL3r_NUMBER_OF_IDLEf_SET(r,f) (r).fx100_ctl3[0]=(((r).fx100_ctl3[0] & ~((uint32_t)0xff << 8)) | ((((uint32_t)f) & 0xff) << 8)) | (255 << (16 + 8))
#define BCMI_VIPER_XGXS_FX100_CTL3r_CORRELATOR_DISABLEf_GET(r) ((((r).fx100_ctl3[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL3r_CORRELATOR_DISABLEf_SET(r,f) (r).fx100_ctl3[0]=(((r).fx100_ctl3[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_NRZf_GET(r) ((((r).fx100_ctl3[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_NRZf_SET(r,f) (r).fx100_ctl3[0]=(((r).fx100_ctl3[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_ENCODERf_GET(r) ((((r).fx100_ctl3[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_ENCODERf_SET(r,f) (r).fx100_ctl3[0]=(((r).fx100_ctl3[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_ALIGNMENTf_GET(r) ((((r).fx100_ctl3[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_ALIGNMENTf_SET(r,f) (r).fx100_ctl3[0]=(((r).fx100_ctl3[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_FX100_CTL3r_FORCE_LINKf_GET(r) ((((r).fx100_ctl3[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL3r_FORCE_LINKf_SET(r,f) (r).fx100_ctl3[0]=(((r).fx100_ctl3[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_FX100_CTL3r_FORCE_LOCKf_GET(r) ((((r).fx100_ctl3[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL3r_FORCE_LOCKf_SET(r,f) (r).fx100_ctl3[0]=(((r).fx100_ctl3[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_FX100_CTL3r_FAST_UNLOCK_TIMERf_GET(r) ((((r).fx100_ctl3[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL3r_FAST_UNLOCK_TIMERf_SET(r,f) (r).fx100_ctl3[0]=(((r).fx100_ctl3[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_FX100_CTL3r_FAST_TIMERSf_GET(r) (((r).fx100_ctl3[0]) & 0x1)
#define BCMI_VIPER_XGXS_FX100_CTL3r_FAST_TIMERSf_SET(r,f) (r).fx100_ctl3[0]=(((r).fx100_ctl3[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access FX100_CTL3.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_CTL3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_CTL3r,(_r._fx100_ctl3))
#define BCMI_VIPER_XGXS_WRITE_FX100_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL3r,(_r._fx100_ctl3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL3r,(_r._fx100_ctl3))
#define BCMI_VIPER_XGXS_READLN_FX100_CTL3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_ctl3))
#define BCMI_VIPER_XGXS_WRITELN_FX100_CTL3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_ctl3))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_ctl3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_CTL3r BCMI_VIPER_XGXS_FX100_CTL3r
#define FX100_CTL3r_SIZE BCMI_VIPER_XGXS_FX100_CTL3r_SIZE
typedef BCMI_VIPER_XGXS_FX100_CTL3r_t FX100_CTL3r_t;
#define FX100_CTL3r_CLR BCMI_VIPER_XGXS_FX100_CTL3r_CLR
#define FX100_CTL3r_SET BCMI_VIPER_XGXS_FX100_CTL3r_SET
#define FX100_CTL3r_GET BCMI_VIPER_XGXS_FX100_CTL3r_GET
#define FX100_CTL3r_NUMBER_OF_IDLEf_GET BCMI_VIPER_XGXS_FX100_CTL3r_NUMBER_OF_IDLEf_GET
#define FX100_CTL3r_NUMBER_OF_IDLEf_SET BCMI_VIPER_XGXS_FX100_CTL3r_NUMBER_OF_IDLEf_SET
#define FX100_CTL3r_CORRELATOR_DISABLEf_GET BCMI_VIPER_XGXS_FX100_CTL3r_CORRELATOR_DISABLEf_GET
#define FX100_CTL3r_CORRELATOR_DISABLEf_SET BCMI_VIPER_XGXS_FX100_CTL3r_CORRELATOR_DISABLEf_SET
#define FX100_CTL3r_BYPASS_NRZf_GET BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_NRZf_GET
#define FX100_CTL3r_BYPASS_NRZf_SET BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_NRZf_SET
#define FX100_CTL3r_BYPASS_ENCODERf_GET BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_ENCODERf_GET
#define FX100_CTL3r_BYPASS_ENCODERf_SET BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_ENCODERf_SET
#define FX100_CTL3r_BYPASS_ALIGNMENTf_GET BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_ALIGNMENTf_GET
#define FX100_CTL3r_BYPASS_ALIGNMENTf_SET BCMI_VIPER_XGXS_FX100_CTL3r_BYPASS_ALIGNMENTf_SET
#define FX100_CTL3r_FORCE_LINKf_GET BCMI_VIPER_XGXS_FX100_CTL3r_FORCE_LINKf_GET
#define FX100_CTL3r_FORCE_LINKf_SET BCMI_VIPER_XGXS_FX100_CTL3r_FORCE_LINKf_SET
#define FX100_CTL3r_FORCE_LOCKf_GET BCMI_VIPER_XGXS_FX100_CTL3r_FORCE_LOCKf_GET
#define FX100_CTL3r_FORCE_LOCKf_SET BCMI_VIPER_XGXS_FX100_CTL3r_FORCE_LOCKf_SET
#define FX100_CTL3r_FAST_UNLOCK_TIMERf_GET BCMI_VIPER_XGXS_FX100_CTL3r_FAST_UNLOCK_TIMERf_GET
#define FX100_CTL3r_FAST_UNLOCK_TIMERf_SET BCMI_VIPER_XGXS_FX100_CTL3r_FAST_UNLOCK_TIMERf_SET
#define FX100_CTL3r_FAST_TIMERSf_GET BCMI_VIPER_XGXS_FX100_CTL3r_FAST_TIMERSf_GET
#define FX100_CTL3r_FAST_TIMERSf_SET BCMI_VIPER_XGXS_FX100_CTL3r_FAST_TIMERSf_SET
#define READ_FX100_CTL3r BCMI_VIPER_XGXS_READ_FX100_CTL3r
#define WRITE_FX100_CTL3r BCMI_VIPER_XGXS_WRITE_FX100_CTL3r
#define MODIFY_FX100_CTL3r BCMI_VIPER_XGXS_MODIFY_FX100_CTL3r
#define READLN_FX100_CTL3r BCMI_VIPER_XGXS_READLN_FX100_CTL3r
#define WRITELN_FX100_CTL3r BCMI_VIPER_XGXS_WRITELN_FX100_CTL3r
#define WRITEALL_FX100_CTL3r BCMI_VIPER_XGXS_WRITEALL_FX100_CTL3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_CTL3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_STS1
 * BLOCKS:   FX100
 * REGADDR:  0x8403
 * DESC:     100FX status register 1
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     LINK             1 = 100-FX mode link is up0 = 100-FX mode link is downLH, Latching High
 *     LOCKED           1 = enough idles are properly detected to lock0 = not lockedLH, Latching High
 *     FAULTING         1 = far end fault detected since last read0 = no far end fault detected since last readLH, Latching High
 *     LOST_LOCK        1 = lost lock since last read0 = lock has not been lost since last readLH, Latching High
 *     LOCK_TIMER_EXPIRED 1 = unable to lock within 730us since last read0 = condition not detected since last readLH, Latching High
 *     RX_ERR_DETECTED  1 = 100-FX mode receive coding error detected since last read0 = no 100-FX mode receive coding error detected since last readLH, Latching High
 *     TX_ERR_DETECTED  1 = 100-FX mode received packet with txer code detected since last read0 = no 100-FX mode received packet with txer code detected since last readLH, Latching High
 *     FALSE_CARRIER_DETECTED 1 = 100-FX mode false carrier detected since last read0 = no 100-FX mode false carrier detected since last readLH, Latching High
 *     BAD_ESD_DETECTED 1 = 100-FX mode bad ESD error detected since last read0 = no 100-FX mode bad ESD error detected since last readLH, Latching High
 *     LINK_STATUS_CHG  1 = 100-FX mode link status change since last read0 = 100-FX mode link status has not changed since last readLH, Latching High
 *     FIBER_PWRDWN     1 = fiber is powered down due to fiber auto powerdown register 2*10h bit[12]0 = normal opeartionLH, Latching High
 *     FIBER_PWRDWN_STATUS_CHG 1 = fiber is powered down due to fiber auto powerdown register 2*10h bit[12] since last read0 = fiber has not been powered down since last readLH, Latching High
 *     MODE_CHANGE      1 = 100FX mode has changed since last read (100FX mode enabled or disabled) NOTE: This bit is useful when the auto-detection is enabled in register 2*10h bit [2]0 = 100FX mode has not changed since last read (fixed in sgmii or fiber mode)LH, Latching High
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_STS1r (0x00008403 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_STS1r_SIZE 4

/*
 * This structure should be used to declare and program FX100_STS1.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_STS1r_s {
	uint32_t v[1];
	uint32_t fx100_sts1[1];
	uint32_t _fx100_sts1;
} BCMI_VIPER_XGXS_FX100_STS1r_t;

#define BCMI_VIPER_XGXS_FX100_STS1r_CLR(r) (r).fx100_sts1[0] = 0
#define BCMI_VIPER_XGXS_FX100_STS1r_SET(r,d) (r).fx100_sts1[0] = d
#define BCMI_VIPER_XGXS_FX100_STS1r_GET(r) (r).fx100_sts1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_STS1r_MODE_CHANGEf_GET(r) ((((r).fx100_sts1[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_MODE_CHANGEf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_FX100_STS1r_FIBER_PWRDWN_STATUS_CHGf_GET(r) ((((r).fx100_sts1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_FIBER_PWRDWN_STATUS_CHGf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_FX100_STS1r_FIBER_PWRDWNf_GET(r) ((((r).fx100_sts1[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_FIBER_PWRDWNf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_FX100_STS1r_LINK_STATUS_CHGf_GET(r) ((((r).fx100_sts1[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_LINK_STATUS_CHGf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_FX100_STS1r_BAD_ESD_DETECTEDf_GET(r) ((((r).fx100_sts1[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_BAD_ESD_DETECTEDf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_FX100_STS1r_FALSE_CARRIER_DETECTEDf_GET(r) ((((r).fx100_sts1[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_FALSE_CARRIER_DETECTEDf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_FX100_STS1r_TX_ERR_DETECTEDf_GET(r) ((((r).fx100_sts1[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_TX_ERR_DETECTEDf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_FX100_STS1r_RX_ERR_DETECTEDf_GET(r) ((((r).fx100_sts1[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_RX_ERR_DETECTEDf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_FX100_STS1r_LOCK_TIMER_EXPIREDf_GET(r) ((((r).fx100_sts1[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_LOCK_TIMER_EXPIREDf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_FX100_STS1r_LOST_LOCKf_GET(r) ((((r).fx100_sts1[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_LOST_LOCKf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_FX100_STS1r_FAULTINGf_GET(r) ((((r).fx100_sts1[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_FAULTINGf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_FX100_STS1r_LOCKEDf_GET(r) ((((r).fx100_sts1[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_LOCKEDf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_FX100_STS1r_LINKf_GET(r) (((r).fx100_sts1[0]) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS1r_LINKf_SET(r,f) (r).fx100_sts1[0]=(((r).fx100_sts1[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access FX100_STS1.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_STS1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_STS1r,(_r._fx100_sts1))
#define BCMI_VIPER_XGXS_WRITE_FX100_STS1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS1r,(_r._fx100_sts1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_STS1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS1r,(_r._fx100_sts1))
#define BCMI_VIPER_XGXS_READLN_FX100_STS1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_STS1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_sts1))
#define BCMI_VIPER_XGXS_WRITELN_FX100_STS1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_sts1))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_STS1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_sts1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_STS1r BCMI_VIPER_XGXS_FX100_STS1r
#define FX100_STS1r_SIZE BCMI_VIPER_XGXS_FX100_STS1r_SIZE
typedef BCMI_VIPER_XGXS_FX100_STS1r_t FX100_STS1r_t;
#define FX100_STS1r_CLR BCMI_VIPER_XGXS_FX100_STS1r_CLR
#define FX100_STS1r_SET BCMI_VIPER_XGXS_FX100_STS1r_SET
#define FX100_STS1r_GET BCMI_VIPER_XGXS_FX100_STS1r_GET
#define FX100_STS1r_MODE_CHANGEf_GET BCMI_VIPER_XGXS_FX100_STS1r_MODE_CHANGEf_GET
#define FX100_STS1r_MODE_CHANGEf_SET BCMI_VIPER_XGXS_FX100_STS1r_MODE_CHANGEf_SET
#define FX100_STS1r_FIBER_PWRDWN_STATUS_CHGf_GET BCMI_VIPER_XGXS_FX100_STS1r_FIBER_PWRDWN_STATUS_CHGf_GET
#define FX100_STS1r_FIBER_PWRDWN_STATUS_CHGf_SET BCMI_VIPER_XGXS_FX100_STS1r_FIBER_PWRDWN_STATUS_CHGf_SET
#define FX100_STS1r_FIBER_PWRDWNf_GET BCMI_VIPER_XGXS_FX100_STS1r_FIBER_PWRDWNf_GET
#define FX100_STS1r_FIBER_PWRDWNf_SET BCMI_VIPER_XGXS_FX100_STS1r_FIBER_PWRDWNf_SET
#define FX100_STS1r_LINK_STATUS_CHGf_GET BCMI_VIPER_XGXS_FX100_STS1r_LINK_STATUS_CHGf_GET
#define FX100_STS1r_LINK_STATUS_CHGf_SET BCMI_VIPER_XGXS_FX100_STS1r_LINK_STATUS_CHGf_SET
#define FX100_STS1r_BAD_ESD_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS1r_BAD_ESD_DETECTEDf_GET
#define FX100_STS1r_BAD_ESD_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS1r_BAD_ESD_DETECTEDf_SET
#define FX100_STS1r_FALSE_CARRIER_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS1r_FALSE_CARRIER_DETECTEDf_GET
#define FX100_STS1r_FALSE_CARRIER_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS1r_FALSE_CARRIER_DETECTEDf_SET
#define FX100_STS1r_TX_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS1r_TX_ERR_DETECTEDf_GET
#define FX100_STS1r_TX_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS1r_TX_ERR_DETECTEDf_SET
#define FX100_STS1r_RX_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS1r_RX_ERR_DETECTEDf_GET
#define FX100_STS1r_RX_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS1r_RX_ERR_DETECTEDf_SET
#define FX100_STS1r_LOCK_TIMER_EXPIREDf_GET BCMI_VIPER_XGXS_FX100_STS1r_LOCK_TIMER_EXPIREDf_GET
#define FX100_STS1r_LOCK_TIMER_EXPIREDf_SET BCMI_VIPER_XGXS_FX100_STS1r_LOCK_TIMER_EXPIREDf_SET
#define FX100_STS1r_LOST_LOCKf_GET BCMI_VIPER_XGXS_FX100_STS1r_LOST_LOCKf_GET
#define FX100_STS1r_LOST_LOCKf_SET BCMI_VIPER_XGXS_FX100_STS1r_LOST_LOCKf_SET
#define FX100_STS1r_FAULTINGf_GET BCMI_VIPER_XGXS_FX100_STS1r_FAULTINGf_GET
#define FX100_STS1r_FAULTINGf_SET BCMI_VIPER_XGXS_FX100_STS1r_FAULTINGf_SET
#define FX100_STS1r_LOCKEDf_GET BCMI_VIPER_XGXS_FX100_STS1r_LOCKEDf_GET
#define FX100_STS1r_LOCKEDf_SET BCMI_VIPER_XGXS_FX100_STS1r_LOCKEDf_SET
#define FX100_STS1r_LINKf_GET BCMI_VIPER_XGXS_FX100_STS1r_LINKf_GET
#define FX100_STS1r_LINKf_SET BCMI_VIPER_XGXS_FX100_STS1r_LINKf_SET
#define READ_FX100_STS1r BCMI_VIPER_XGXS_READ_FX100_STS1r
#define WRITE_FX100_STS1r BCMI_VIPER_XGXS_WRITE_FX100_STS1r
#define MODIFY_FX100_STS1r BCMI_VIPER_XGXS_MODIFY_FX100_STS1r
#define READLN_FX100_STS1r BCMI_VIPER_XGXS_READLN_FX100_STS1r
#define WRITELN_FX100_STS1r BCMI_VIPER_XGXS_WRITELN_FX100_STS1r
#define WRITEALL_FX100_STS1r BCMI_VIPER_XGXS_WRITEALL_FX100_STS1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_STS1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_STS3
 * BLOCKS:   FX100
 * REGADDR:  0x8405
 * DESC:     100FX status register 3
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     IDLES_DETECTED   1 = 100-FX mode two idle code words detected since last read0 = no 100-FX mode two idle code words detected since last readLH, Latching High
 *     IJ_DETECTED      1 = 100-FX mode first start of stream code following idle detected since last read0 = no 100-FX mode first start of stream code following idle detected since last readLH, Latching High
 *     SSD_DETECTED     1 = 100-FX mode start of stream delimiter detected since last read0 = no 100-FX mode start of stream delimiter detected since last readLH, Latching High
 *     ESD_DETECTED     1 = 100-FX mode end of stream delimiter detected since last read0 = no 100-FX mode end of stream delimiter detected since last readLH, Latching High
 *     ERR_DETECTED     1 = 100-FX mode non-data code detected since last read0 = no 100-FX mode non-data code detected since last readLH, Latching High
 *     CRS_IND_DETECTED 1 = 100-FX mode two non-contiguous zeroes detected since last read0 = no 100-FX mode two non-contiguous zeroes detected since last readLH, Latching High
 *     IDLES_DETECTED_5B 1 = 100-FX mode two idle code words with a 5B boundary detected since last read0 = no 100-FX mode two idle code words with a 5B boundary detected since last readLH, Latching High
 *     LINKMON_CNTR     100FX mode link monitor counter value
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_STS3r (0x00008405 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_STS3r_SIZE 4

/*
 * This structure should be used to declare and program FX100_STS3.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_STS3r_s {
	uint32_t v[1];
	uint32_t fx100_sts3[1];
	uint32_t _fx100_sts3;
} BCMI_VIPER_XGXS_FX100_STS3r_t;

#define BCMI_VIPER_XGXS_FX100_STS3r_CLR(r) (r).fx100_sts3[0] = 0
#define BCMI_VIPER_XGXS_FX100_STS3r_SET(r,d) (r).fx100_sts3[0] = d
#define BCMI_VIPER_XGXS_FX100_STS3r_GET(r) (r).fx100_sts3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_STS3r_LINKMON_CNTRf_GET(r) ((((r).fx100_sts3[0]) >> 8) & 0xff)
#define BCMI_VIPER_XGXS_FX100_STS3r_LINKMON_CNTRf_SET(r,f) (r).fx100_sts3[0]=(((r).fx100_sts3[0] & ~((uint32_t)0xff << 8)) | ((((uint32_t)f) & 0xff) << 8)) | (255 << (16 + 8))
#define BCMI_VIPER_XGXS_FX100_STS3r_IDLES_DETECTED_5Bf_GET(r) ((((r).fx100_sts3[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS3r_IDLES_DETECTED_5Bf_SET(r,f) (r).fx100_sts3[0]=(((r).fx100_sts3[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_FX100_STS3r_CRS_IND_DETECTEDf_GET(r) ((((r).fx100_sts3[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS3r_CRS_IND_DETECTEDf_SET(r,f) (r).fx100_sts3[0]=(((r).fx100_sts3[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_FX100_STS3r_ERR_DETECTEDf_GET(r) ((((r).fx100_sts3[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS3r_ERR_DETECTEDf_SET(r,f) (r).fx100_sts3[0]=(((r).fx100_sts3[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_FX100_STS3r_ESD_DETECTEDf_GET(r) ((((r).fx100_sts3[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS3r_ESD_DETECTEDf_SET(r,f) (r).fx100_sts3[0]=(((r).fx100_sts3[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_FX100_STS3r_SSD_DETECTEDf_GET(r) ((((r).fx100_sts3[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS3r_SSD_DETECTEDf_SET(r,f) (r).fx100_sts3[0]=(((r).fx100_sts3[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_FX100_STS3r_IJ_DETECTEDf_GET(r) ((((r).fx100_sts3[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS3r_IJ_DETECTEDf_SET(r,f) (r).fx100_sts3[0]=(((r).fx100_sts3[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_FX100_STS3r_IDLES_DETECTEDf_GET(r) (((r).fx100_sts3[0]) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS3r_IDLES_DETECTEDf_SET(r,f) (r).fx100_sts3[0]=(((r).fx100_sts3[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access FX100_STS3.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_STS3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_STS3r,(_r._fx100_sts3))
#define BCMI_VIPER_XGXS_WRITE_FX100_STS3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS3r,(_r._fx100_sts3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_STS3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS3r,(_r._fx100_sts3))
#define BCMI_VIPER_XGXS_READLN_FX100_STS3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_STS3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_sts3))
#define BCMI_VIPER_XGXS_WRITELN_FX100_STS3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_sts3))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_STS3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_sts3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_STS3r BCMI_VIPER_XGXS_FX100_STS3r
#define FX100_STS3r_SIZE BCMI_VIPER_XGXS_FX100_STS3r_SIZE
typedef BCMI_VIPER_XGXS_FX100_STS3r_t FX100_STS3r_t;
#define FX100_STS3r_CLR BCMI_VIPER_XGXS_FX100_STS3r_CLR
#define FX100_STS3r_SET BCMI_VIPER_XGXS_FX100_STS3r_SET
#define FX100_STS3r_GET BCMI_VIPER_XGXS_FX100_STS3r_GET
#define FX100_STS3r_LINKMON_CNTRf_GET BCMI_VIPER_XGXS_FX100_STS3r_LINKMON_CNTRf_GET
#define FX100_STS3r_LINKMON_CNTRf_SET BCMI_VIPER_XGXS_FX100_STS3r_LINKMON_CNTRf_SET
#define FX100_STS3r_IDLES_DETECTED_5Bf_GET BCMI_VIPER_XGXS_FX100_STS3r_IDLES_DETECTED_5Bf_GET
#define FX100_STS3r_IDLES_DETECTED_5Bf_SET BCMI_VIPER_XGXS_FX100_STS3r_IDLES_DETECTED_5Bf_SET
#define FX100_STS3r_CRS_IND_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS3r_CRS_IND_DETECTEDf_GET
#define FX100_STS3r_CRS_IND_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS3r_CRS_IND_DETECTEDf_SET
#define FX100_STS3r_ERR_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS3r_ERR_DETECTEDf_GET
#define FX100_STS3r_ERR_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS3r_ERR_DETECTEDf_SET
#define FX100_STS3r_ESD_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS3r_ESD_DETECTEDf_GET
#define FX100_STS3r_ESD_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS3r_ESD_DETECTEDf_SET
#define FX100_STS3r_SSD_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS3r_SSD_DETECTEDf_GET
#define FX100_STS3r_SSD_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS3r_SSD_DETECTEDf_SET
#define FX100_STS3r_IJ_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS3r_IJ_DETECTEDf_GET
#define FX100_STS3r_IJ_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS3r_IJ_DETECTEDf_SET
#define FX100_STS3r_IDLES_DETECTEDf_GET BCMI_VIPER_XGXS_FX100_STS3r_IDLES_DETECTEDf_GET
#define FX100_STS3r_IDLES_DETECTEDf_SET BCMI_VIPER_XGXS_FX100_STS3r_IDLES_DETECTEDf_SET
#define READ_FX100_STS3r BCMI_VIPER_XGXS_READ_FX100_STS3r
#define WRITE_FX100_STS3r BCMI_VIPER_XGXS_WRITE_FX100_STS3r
#define MODIFY_FX100_STS3r BCMI_VIPER_XGXS_MODIFY_FX100_STS3r
#define READLN_FX100_STS3r BCMI_VIPER_XGXS_READLN_FX100_STS3r
#define WRITELN_FX100_STS3r BCMI_VIPER_XGXS_WRITELN_FX100_STS3r
#define WRITEALL_FX100_STS3r BCMI_VIPER_XGXS_WRITEALL_FX100_STS3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_STS3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_STS4
 * BLOCKS:   FX100
 * REGADDR:  0x8406
 * DESC:     100FX status register 4
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     TX_SEJ           1 = 100 FX mode TX state machine has entered SEJ state since last read0 = SEJ state not entered since last read
 *     TX_SSJ           1 = 100 FX mode TX state machine has entered SSJ state since last read0 = SSJ state not entered since last read
 *     TX_SEK           1 = 100 FX mode TX state machine has entered SEK state since last read0 = SEK state not entered since last readLH, Latching High
 *     TX_SSK           1 = 100 FX mode TX state machine has entered SSK state since last read0 = SSk state not entered since last readLH, Latching High
 *     TX_TDATA         1 = 100 FX mode TX state machine has entered tdata state since last read0 = tdata state not entered since last readLH, Latching High
 *     TX_TERROR        1 = 100 FX mode TX state machine has entered terror state since last read0 = terror state not entered since last readLH, Latching High
 *     TX_EST           1 = 100 FX mode TX state machine has entered EST state since last read0 = EST state not entered since last readLH, Latching High
 *     TX_ESR           1 = 100 FX mode TX state machine has entered ESR state since last read0 = ESR state not entered since last readLH, Latching High
 *     FX_LINKFAIL      1 = 100 FX mode RX state machine has entered linkfail state since last read0 = linkfail state not entered since last readLH, Latching High
 *     RX_BADSSD        1 = 100 FX mode RX state machine has entered badssd state since last read0 = badssd state not entered since last readLH, Latching High
 *     RX_CONFIRMK      1 = 100 FX mode RX state machine has entered confirmk state since last read0 = confirmk state not entered since last readLH, Latching High
 *     RX_SSJ           1 = 100 FX mode RX state machine has entered SSJ state since last read0 = SSJ state not entered since last readLH, Latching High
 *     RX_SSK           1 = 100 FX mode RX state machine has entered SSK state since last read0 = SSK state not entered since last readLH, Latching High
 *     RX_DATA          1 = 100 FX mode RX state machine has entered data state since last read0 = data state not entered since last readLH, Latching High
 *     RX_BADEND        1 = 100FX mode RX state machine has entered badend state since last read0 = badend state not entered since last readLH, Latching High
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_STS4r (0x00008406 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_STS4r_SIZE 4

/*
 * This structure should be used to declare and program FX100_STS4.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_STS4r_s {
	uint32_t v[1];
	uint32_t fx100_sts4[1];
	uint32_t _fx100_sts4;
} BCMI_VIPER_XGXS_FX100_STS4r_t;

#define BCMI_VIPER_XGXS_FX100_STS4r_CLR(r) (r).fx100_sts4[0] = 0
#define BCMI_VIPER_XGXS_FX100_STS4r_SET(r,d) (r).fx100_sts4[0] = d
#define BCMI_VIPER_XGXS_FX100_STS4r_GET(r) (r).fx100_sts4[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_BADENDf_GET(r) ((((r).fx100_sts4[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_BADENDf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_DATAf_GET(r) ((((r).fx100_sts4[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_DATAf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_SSKf_GET(r) ((((r).fx100_sts4[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_SSKf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_SSJf_GET(r) ((((r).fx100_sts4[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_SSJf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_CONFIRMKf_GET(r) ((((r).fx100_sts4[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_CONFIRMKf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_BADSSDf_GET(r) ((((r).fx100_sts4[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_RX_BADSSDf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_FX100_STS4r_FX_LINKFAILf_GET(r) ((((r).fx100_sts4[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_FX_LINKFAILf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_ESRf_GET(r) ((((r).fx100_sts4[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_ESRf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_ESTf_GET(r) ((((r).fx100_sts4[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_ESTf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_TERRORf_GET(r) ((((r).fx100_sts4[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_TERRORf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_TDATAf_GET(r) ((((r).fx100_sts4[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_TDATAf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_SSKf_GET(r) ((((r).fx100_sts4[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_SSKf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_SEKf_GET(r) ((((r).fx100_sts4[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_SEKf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_SSJf_GET(r) ((((r).fx100_sts4[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_SSJf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_SEJf_GET(r) (((r).fx100_sts4[0]) & 0x1)
#define BCMI_VIPER_XGXS_FX100_STS4r_TX_SEJf_SET(r,f) (r).fx100_sts4[0]=(((r).fx100_sts4[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access FX100_STS4.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_STS4r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_STS4r,(_r._fx100_sts4))
#define BCMI_VIPER_XGXS_WRITE_FX100_STS4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS4r,(_r._fx100_sts4)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_STS4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS4r,(_r._fx100_sts4))
#define BCMI_VIPER_XGXS_READLN_FX100_STS4r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_STS4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_sts4))
#define BCMI_VIPER_XGXS_WRITELN_FX100_STS4r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_sts4))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_STS4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_STS4r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_sts4))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_STS4r BCMI_VIPER_XGXS_FX100_STS4r
#define FX100_STS4r_SIZE BCMI_VIPER_XGXS_FX100_STS4r_SIZE
typedef BCMI_VIPER_XGXS_FX100_STS4r_t FX100_STS4r_t;
#define FX100_STS4r_CLR BCMI_VIPER_XGXS_FX100_STS4r_CLR
#define FX100_STS4r_SET BCMI_VIPER_XGXS_FX100_STS4r_SET
#define FX100_STS4r_GET BCMI_VIPER_XGXS_FX100_STS4r_GET
#define FX100_STS4r_RX_BADENDf_GET BCMI_VIPER_XGXS_FX100_STS4r_RX_BADENDf_GET
#define FX100_STS4r_RX_BADENDf_SET BCMI_VIPER_XGXS_FX100_STS4r_RX_BADENDf_SET
#define FX100_STS4r_RX_DATAf_GET BCMI_VIPER_XGXS_FX100_STS4r_RX_DATAf_GET
#define FX100_STS4r_RX_DATAf_SET BCMI_VIPER_XGXS_FX100_STS4r_RX_DATAf_SET
#define FX100_STS4r_RX_SSKf_GET BCMI_VIPER_XGXS_FX100_STS4r_RX_SSKf_GET
#define FX100_STS4r_RX_SSKf_SET BCMI_VIPER_XGXS_FX100_STS4r_RX_SSKf_SET
#define FX100_STS4r_RX_SSJf_GET BCMI_VIPER_XGXS_FX100_STS4r_RX_SSJf_GET
#define FX100_STS4r_RX_SSJf_SET BCMI_VIPER_XGXS_FX100_STS4r_RX_SSJf_SET
#define FX100_STS4r_RX_CONFIRMKf_GET BCMI_VIPER_XGXS_FX100_STS4r_RX_CONFIRMKf_GET
#define FX100_STS4r_RX_CONFIRMKf_SET BCMI_VIPER_XGXS_FX100_STS4r_RX_CONFIRMKf_SET
#define FX100_STS4r_RX_BADSSDf_GET BCMI_VIPER_XGXS_FX100_STS4r_RX_BADSSDf_GET
#define FX100_STS4r_RX_BADSSDf_SET BCMI_VIPER_XGXS_FX100_STS4r_RX_BADSSDf_SET
#define FX100_STS4r_FX_LINKFAILf_GET BCMI_VIPER_XGXS_FX100_STS4r_FX_LINKFAILf_GET
#define FX100_STS4r_FX_LINKFAILf_SET BCMI_VIPER_XGXS_FX100_STS4r_FX_LINKFAILf_SET
#define FX100_STS4r_TX_ESRf_GET BCMI_VIPER_XGXS_FX100_STS4r_TX_ESRf_GET
#define FX100_STS4r_TX_ESRf_SET BCMI_VIPER_XGXS_FX100_STS4r_TX_ESRf_SET
#define FX100_STS4r_TX_ESTf_GET BCMI_VIPER_XGXS_FX100_STS4r_TX_ESTf_GET
#define FX100_STS4r_TX_ESTf_SET BCMI_VIPER_XGXS_FX100_STS4r_TX_ESTf_SET
#define FX100_STS4r_TX_TERRORf_GET BCMI_VIPER_XGXS_FX100_STS4r_TX_TERRORf_GET
#define FX100_STS4r_TX_TERRORf_SET BCMI_VIPER_XGXS_FX100_STS4r_TX_TERRORf_SET
#define FX100_STS4r_TX_TDATAf_GET BCMI_VIPER_XGXS_FX100_STS4r_TX_TDATAf_GET
#define FX100_STS4r_TX_TDATAf_SET BCMI_VIPER_XGXS_FX100_STS4r_TX_TDATAf_SET
#define FX100_STS4r_TX_SSKf_GET BCMI_VIPER_XGXS_FX100_STS4r_TX_SSKf_GET
#define FX100_STS4r_TX_SSKf_SET BCMI_VIPER_XGXS_FX100_STS4r_TX_SSKf_SET
#define FX100_STS4r_TX_SEKf_GET BCMI_VIPER_XGXS_FX100_STS4r_TX_SEKf_GET
#define FX100_STS4r_TX_SEKf_SET BCMI_VIPER_XGXS_FX100_STS4r_TX_SEKf_SET
#define FX100_STS4r_TX_SSJf_GET BCMI_VIPER_XGXS_FX100_STS4r_TX_SSJf_GET
#define FX100_STS4r_TX_SSJf_SET BCMI_VIPER_XGXS_FX100_STS4r_TX_SSJf_SET
#define FX100_STS4r_TX_SEJf_GET BCMI_VIPER_XGXS_FX100_STS4r_TX_SEJf_GET
#define FX100_STS4r_TX_SEJf_SET BCMI_VIPER_XGXS_FX100_STS4r_TX_SEJf_SET
#define READ_FX100_STS4r BCMI_VIPER_XGXS_READ_FX100_STS4r
#define WRITE_FX100_STS4r BCMI_VIPER_XGXS_WRITE_FX100_STS4r
#define MODIFY_FX100_STS4r BCMI_VIPER_XGXS_MODIFY_FX100_STS4r
#define READLN_FX100_STS4r BCMI_VIPER_XGXS_READLN_FX100_STS4r
#define WRITELN_FX100_STS4r BCMI_VIPER_XGXS_WRITELN_FX100_STS4r
#define WRITEALL_FX100_STS4r BCMI_VIPER_XGXS_WRITEALL_FX100_STS4r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_STS4r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_IDLE1
 * BLOCKS:   FX100
 * REGADDR:  0x8407
 * DESC:     100FX idle pattern register 1
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     FX100_IDLE1      Lower 10 bits of idle pattern for the 100fx idle detect correlator
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_IDLE1r (0x00008407 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_IDLE1r_SIZE 4

/*
 * This structure should be used to declare and program FX100_IDLE1.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_IDLE1r_s {
	uint32_t v[1];
	uint32_t fx100_idle1[1];
	uint32_t _fx100_idle1;
} BCMI_VIPER_XGXS_FX100_IDLE1r_t;

#define BCMI_VIPER_XGXS_FX100_IDLE1r_CLR(r) (r).fx100_idle1[0] = 0
#define BCMI_VIPER_XGXS_FX100_IDLE1r_SET(r,d) (r).fx100_idle1[0] = d
#define BCMI_VIPER_XGXS_FX100_IDLE1r_GET(r) (r).fx100_idle1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_IDLE1r_FX100_IDLE1f_GET(r) (((r).fx100_idle1[0]) & 0x3ff)
#define BCMI_VIPER_XGXS_FX100_IDLE1r_FX100_IDLE1f_SET(r,f) (r).fx100_idle1[0]=(((r).fx100_idle1[0] & ~((uint32_t)0x3ff)) | (((uint32_t)f) & 0x3ff)) | (0x3ff << 16)

/*
 * These macros can be used to access FX100_IDLE1.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_IDLE1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_IDLE1r,(_r._fx100_idle1))
#define BCMI_VIPER_XGXS_WRITE_FX100_IDLE1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLE1r,(_r._fx100_idle1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_IDLE1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLE1r,(_r._fx100_idle1))
#define BCMI_VIPER_XGXS_READLN_FX100_IDLE1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_IDLE1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_idle1))
#define BCMI_VIPER_XGXS_WRITELN_FX100_IDLE1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLE1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_idle1))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_IDLE1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLE1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_idle1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_IDLE1r BCMI_VIPER_XGXS_FX100_IDLE1r
#define FX100_IDLE1r_SIZE BCMI_VIPER_XGXS_FX100_IDLE1r_SIZE
typedef BCMI_VIPER_XGXS_FX100_IDLE1r_t FX100_IDLE1r_t;
#define FX100_IDLE1r_CLR BCMI_VIPER_XGXS_FX100_IDLE1r_CLR
#define FX100_IDLE1r_SET BCMI_VIPER_XGXS_FX100_IDLE1r_SET
#define FX100_IDLE1r_GET BCMI_VIPER_XGXS_FX100_IDLE1r_GET
#define FX100_IDLE1r_FX100_IDLE1f_GET BCMI_VIPER_XGXS_FX100_IDLE1r_FX100_IDLE1f_GET
#define FX100_IDLE1r_FX100_IDLE1f_SET BCMI_VIPER_XGXS_FX100_IDLE1r_FX100_IDLE1f_SET
#define READ_FX100_IDLE1r BCMI_VIPER_XGXS_READ_FX100_IDLE1r
#define WRITE_FX100_IDLE1r BCMI_VIPER_XGXS_WRITE_FX100_IDLE1r
#define MODIFY_FX100_IDLE1r BCMI_VIPER_XGXS_MODIFY_FX100_IDLE1r
#define READLN_FX100_IDLE1r BCMI_VIPER_XGXS_READLN_FX100_IDLE1r
#define WRITELN_FX100_IDLE1r BCMI_VIPER_XGXS_WRITELN_FX100_IDLE1r
#define WRITEALL_FX100_IDLE1r BCMI_VIPER_XGXS_WRITEALL_FX100_IDLE1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_IDLE1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_IDLE2
 * BLOCKS:   FX100
 * REGADDR:  0x8408
 * DESC:     100FX idle pattern register 2
 * RESETVAL: 0x3ff (1023)
 * ACCESS:   R/W
 * FIELDS:
 *     FX100_IDLE2      Upper 10 bits of idle pattern for the 100fx idle detect correlator
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_IDLE2r (0x00008408 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_IDLE2r_SIZE 4

/*
 * This structure should be used to declare and program FX100_IDLE2.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_IDLE2r_s {
	uint32_t v[1];
	uint32_t fx100_idle2[1];
	uint32_t _fx100_idle2;
} BCMI_VIPER_XGXS_FX100_IDLE2r_t;

#define BCMI_VIPER_XGXS_FX100_IDLE2r_CLR(r) (r).fx100_idle2[0] = 0
#define BCMI_VIPER_XGXS_FX100_IDLE2r_SET(r,d) (r).fx100_idle2[0] = d
#define BCMI_VIPER_XGXS_FX100_IDLE2r_GET(r) (r).fx100_idle2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_IDLE2r_FX100_IDLE2f_GET(r) (((r).fx100_idle2[0]) & 0x3ff)
#define BCMI_VIPER_XGXS_FX100_IDLE2r_FX100_IDLE2f_SET(r,f) (r).fx100_idle2[0]=(((r).fx100_idle2[0] & ~((uint32_t)0x3ff)) | (((uint32_t)f) & 0x3ff)) | (0x3ff << 16)

/*
 * These macros can be used to access FX100_IDLE2.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_IDLE2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_IDLE2r,(_r._fx100_idle2))
#define BCMI_VIPER_XGXS_WRITE_FX100_IDLE2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLE2r,(_r._fx100_idle2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_IDLE2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLE2r,(_r._fx100_idle2))
#define BCMI_VIPER_XGXS_READLN_FX100_IDLE2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_IDLE2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_idle2))
#define BCMI_VIPER_XGXS_WRITELN_FX100_IDLE2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLE2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_idle2))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_IDLE2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLE2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_idle2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_IDLE2r BCMI_VIPER_XGXS_FX100_IDLE2r
#define FX100_IDLE2r_SIZE BCMI_VIPER_XGXS_FX100_IDLE2r_SIZE
typedef BCMI_VIPER_XGXS_FX100_IDLE2r_t FX100_IDLE2r_t;
#define FX100_IDLE2r_CLR BCMI_VIPER_XGXS_FX100_IDLE2r_CLR
#define FX100_IDLE2r_SET BCMI_VIPER_XGXS_FX100_IDLE2r_SET
#define FX100_IDLE2r_GET BCMI_VIPER_XGXS_FX100_IDLE2r_GET
#define FX100_IDLE2r_FX100_IDLE2f_GET BCMI_VIPER_XGXS_FX100_IDLE2r_FX100_IDLE2f_GET
#define FX100_IDLE2r_FX100_IDLE2f_SET BCMI_VIPER_XGXS_FX100_IDLE2r_FX100_IDLE2f_SET
#define READ_FX100_IDLE2r BCMI_VIPER_XGXS_READ_FX100_IDLE2r
#define WRITE_FX100_IDLE2r BCMI_VIPER_XGXS_WRITE_FX100_IDLE2r
#define MODIFY_FX100_IDLE2r BCMI_VIPER_XGXS_MODIFY_FX100_IDLE2r
#define READLN_FX100_IDLE2r BCMI_VIPER_XGXS_READLN_FX100_IDLE2r
#define WRITELN_FX100_IDLE2r BCMI_VIPER_XGXS_WRITELN_FX100_IDLE2r
#define WRITEALL_FX100_IDLE2r BCMI_VIPER_XGXS_WRITEALL_FX100_IDLE2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_IDLE2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_IDLESTS
 * BLOCKS:   FX100
 * REGADDR:  0x8409
 * DESC:     100FX idle status register
 * RESETVAL: 0x3c (60)
 * ACCESS:   R/O
 * FIELDS:
 *     FX100_IDLECORR_CNT 100fx idle detect correlator counter
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_IDLESTSr (0x00008409 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_IDLESTSr_SIZE 4

/*
 * This structure should be used to declare and program FX100_IDLESTS.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_IDLESTSr_s {
	uint32_t v[1];
	uint32_t fx100_idlests[1];
	uint32_t _fx100_idlests;
} BCMI_VIPER_XGXS_FX100_IDLESTSr_t;

#define BCMI_VIPER_XGXS_FX100_IDLESTSr_CLR(r) (r).fx100_idlests[0] = 0
#define BCMI_VIPER_XGXS_FX100_IDLESTSr_SET(r,d) (r).fx100_idlests[0] = d
#define BCMI_VIPER_XGXS_FX100_IDLESTSr_GET(r) (r).fx100_idlests[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_IDLESTSr_FX100_IDLECORR_CNTf_GET(r) (((r).fx100_idlests[0]) & 0x7f)
#define BCMI_VIPER_XGXS_FX100_IDLESTSr_FX100_IDLECORR_CNTf_SET(r,f) (r).fx100_idlests[0]=(((r).fx100_idlests[0] & ~((uint32_t)0x7f)) | (((uint32_t)f) & 0x7f)) | (0x7f << 16)

/*
 * These macros can be used to access FX100_IDLESTS.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_IDLESTSr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_IDLESTSr,(_r._fx100_idlests))
#define BCMI_VIPER_XGXS_WRITE_FX100_IDLESTSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLESTSr,(_r._fx100_idlests)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_IDLESTSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLESTSr,(_r._fx100_idlests))
#define BCMI_VIPER_XGXS_READLN_FX100_IDLESTSr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_IDLESTSr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_idlests))
#define BCMI_VIPER_XGXS_WRITELN_FX100_IDLESTSr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLESTSr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_idlests))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_IDLESTSr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLESTSr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_idlests))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_IDLESTSr BCMI_VIPER_XGXS_FX100_IDLESTSr
#define FX100_IDLESTSr_SIZE BCMI_VIPER_XGXS_FX100_IDLESTSr_SIZE
typedef BCMI_VIPER_XGXS_FX100_IDLESTSr_t FX100_IDLESTSr_t;
#define FX100_IDLESTSr_CLR BCMI_VIPER_XGXS_FX100_IDLESTSr_CLR
#define FX100_IDLESTSr_SET BCMI_VIPER_XGXS_FX100_IDLESTSr_SET
#define FX100_IDLESTSr_GET BCMI_VIPER_XGXS_FX100_IDLESTSr_GET
#define FX100_IDLESTSr_FX100_IDLECORR_CNTf_GET BCMI_VIPER_XGXS_FX100_IDLESTSr_FX100_IDLECORR_CNTf_GET
#define FX100_IDLESTSr_FX100_IDLECORR_CNTf_SET BCMI_VIPER_XGXS_FX100_IDLESTSr_FX100_IDLECORR_CNTf_SET
#define READ_FX100_IDLESTSr BCMI_VIPER_XGXS_READ_FX100_IDLESTSr
#define WRITE_FX100_IDLESTSr BCMI_VIPER_XGXS_WRITE_FX100_IDLESTSr
#define MODIFY_FX100_IDLESTSr BCMI_VIPER_XGXS_MODIFY_FX100_IDLESTSr
#define READLN_FX100_IDLESTSr BCMI_VIPER_XGXS_READLN_FX100_IDLESTSr
#define WRITELN_FX100_IDLESTSr BCMI_VIPER_XGXS_WRITELN_FX100_IDLESTSr
#define WRITEALL_FX100_IDLESTSr BCMI_VIPER_XGXS_WRITEALL_FX100_IDLESTSr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_IDLESTSr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_IDLETHRES
 * BLOCKS:   FX100
 * REGADDR:  0x840a
 * DESC:     100FX idle threshold register
 * RESETVAL: 0xd5e (3422)
 * ACCESS:   R/W
 * FIELDS:
 *     FX100_IDLE_MAX_THRES 100fx idle detect correlator upper threshold
 *     FX100_IDLE_MIN_THRES 100fx idle detect correlator lower threshold
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_IDLETHRESr (0x0000840a | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_IDLETHRESr_SIZE 4

/*
 * This structure should be used to declare and program FX100_IDLETHRES.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_IDLETHRESr_s {
	uint32_t v[1];
	uint32_t fx100_idlethres[1];
	uint32_t _fx100_idlethres;
} BCMI_VIPER_XGXS_FX100_IDLETHRESr_t;

#define BCMI_VIPER_XGXS_FX100_IDLETHRESr_CLR(r) (r).fx100_idlethres[0] = 0
#define BCMI_VIPER_XGXS_FX100_IDLETHRESr_SET(r,d) (r).fx100_idlethres[0] = d
#define BCMI_VIPER_XGXS_FX100_IDLETHRESr_GET(r) (r).fx100_idlethres[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_IDLETHRESr_FX100_IDLE_MIN_THRESf_GET(r) ((((r).fx100_idlethres[0]) >> 7) & 0x7f)
#define BCMI_VIPER_XGXS_FX100_IDLETHRESr_FX100_IDLE_MIN_THRESf_SET(r,f) (r).fx100_idlethres[0]=(((r).fx100_idlethres[0] & ~((uint32_t)0x7f << 7)) | ((((uint32_t)f) & 0x7f) << 7)) | (127 << (16 + 7))
#define BCMI_VIPER_XGXS_FX100_IDLETHRESr_FX100_IDLE_MAX_THRESf_GET(r) (((r).fx100_idlethres[0]) & 0x7f)
#define BCMI_VIPER_XGXS_FX100_IDLETHRESr_FX100_IDLE_MAX_THRESf_SET(r,f) (r).fx100_idlethres[0]=(((r).fx100_idlethres[0] & ~((uint32_t)0x7f)) | (((uint32_t)f) & 0x7f)) | (0x7f << 16)

/*
 * These macros can be used to access FX100_IDLETHRES.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_IDLETHRESr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_IDLETHRESr,(_r._fx100_idlethres))
#define BCMI_VIPER_XGXS_WRITE_FX100_IDLETHRESr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLETHRESr,(_r._fx100_idlethres)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_IDLETHRESr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLETHRESr,(_r._fx100_idlethres))
#define BCMI_VIPER_XGXS_READLN_FX100_IDLETHRESr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_IDLETHRESr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_idlethres))
#define BCMI_VIPER_XGXS_WRITELN_FX100_IDLETHRESr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLETHRESr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_idlethres))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_IDLETHRESr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_IDLETHRESr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_idlethres))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_IDLETHRESr BCMI_VIPER_XGXS_FX100_IDLETHRESr
#define FX100_IDLETHRESr_SIZE BCMI_VIPER_XGXS_FX100_IDLETHRESr_SIZE
typedef BCMI_VIPER_XGXS_FX100_IDLETHRESr_t FX100_IDLETHRESr_t;
#define FX100_IDLETHRESr_CLR BCMI_VIPER_XGXS_FX100_IDLETHRESr_CLR
#define FX100_IDLETHRESr_SET BCMI_VIPER_XGXS_FX100_IDLETHRESr_SET
#define FX100_IDLETHRESr_GET BCMI_VIPER_XGXS_FX100_IDLETHRESr_GET
#define FX100_IDLETHRESr_FX100_IDLE_MIN_THRESf_GET BCMI_VIPER_XGXS_FX100_IDLETHRESr_FX100_IDLE_MIN_THRESf_GET
#define FX100_IDLETHRESr_FX100_IDLE_MIN_THRESf_SET BCMI_VIPER_XGXS_FX100_IDLETHRESr_FX100_IDLE_MIN_THRESf_SET
#define FX100_IDLETHRESr_FX100_IDLE_MAX_THRESf_GET BCMI_VIPER_XGXS_FX100_IDLETHRESr_FX100_IDLE_MAX_THRESf_GET
#define FX100_IDLETHRESr_FX100_IDLE_MAX_THRESf_SET BCMI_VIPER_XGXS_FX100_IDLETHRESr_FX100_IDLE_MAX_THRESf_SET
#define READ_FX100_IDLETHRESr BCMI_VIPER_XGXS_READ_FX100_IDLETHRESr
#define WRITE_FX100_IDLETHRESr BCMI_VIPER_XGXS_WRITE_FX100_IDLETHRESr
#define MODIFY_FX100_IDLETHRESr BCMI_VIPER_XGXS_MODIFY_FX100_IDLETHRESr
#define READLN_FX100_IDLETHRESr BCMI_VIPER_XGXS_READLN_FX100_IDLETHRESr
#define WRITELN_FX100_IDLETHRESr BCMI_VIPER_XGXS_WRITELN_FX100_IDLETHRESr
#define WRITEALL_FX100_IDLETHRESr BCMI_VIPER_XGXS_WRITEALL_FX100_IDLETHRESr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_IDLETHRESr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_LOCKTMR
 * BLOCKS:   FX100
 * REGADDR:  0x840b
 * DESC:     100FX lock timer register
 * RESETVAL: 0x4894 (18580)
 * ACCESS:   R/W
 * FIELDS:
 *     FX100_LOCK_MAXTIME maximum lock time
 *     FX100_UNLOCK_THRES 100fx unlock threshold
 *     FX100_LOCK_THRES 100fx lock threshold
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_LOCKTMRr (0x0000840b | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_SIZE 4

/*
 * This structure should be used to declare and program FX100_LOCKTMR.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_LOCKTMRr_s {
	uint32_t v[1];
	uint32_t fx100_locktmr[1];
	uint32_t _fx100_locktmr;
} BCMI_VIPER_XGXS_FX100_LOCKTMRr_t;

#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_CLR(r) (r).fx100_locktmr[0] = 0
#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_SET(r,d) (r).fx100_locktmr[0] = d
#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_GET(r) (r).fx100_locktmr[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_LOCK_THRESf_GET(r) ((((r).fx100_locktmr[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_LOCK_THRESf_SET(r,f) (r).fx100_locktmr[0]=(((r).fx100_locktmr[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_UNLOCK_THRESf_GET(r) ((((r).fx100_locktmr[0]) >> 8) & 0xf)
#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_UNLOCK_THRESf_SET(r,f) (r).fx100_locktmr[0]=(((r).fx100_locktmr[0] & ~((uint32_t)0xf << 8)) | ((((uint32_t)f) & 0xf) << 8)) | (15 << (16 + 8))
#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_LOCK_MAXTIMEf_GET(r) (((r).fx100_locktmr[0]) & 0xff)
#define BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_LOCK_MAXTIMEf_SET(r,f) (r).fx100_locktmr[0]=(((r).fx100_locktmr[0] & ~((uint32_t)0xff)) | (((uint32_t)f) & 0xff)) | (0xff << 16)

/*
 * These macros can be used to access FX100_LOCKTMR.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_LOCKTMRr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_LOCKTMRr,(_r._fx100_locktmr))
#define BCMI_VIPER_XGXS_WRITE_FX100_LOCKTMRr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_LOCKTMRr,(_r._fx100_locktmr)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_LOCKTMRr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_LOCKTMRr,(_r._fx100_locktmr))
#define BCMI_VIPER_XGXS_READLN_FX100_LOCKTMRr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_LOCKTMRr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_locktmr))
#define BCMI_VIPER_XGXS_WRITELN_FX100_LOCKTMRr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_LOCKTMRr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_locktmr))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_LOCKTMRr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_LOCKTMRr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_locktmr))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_LOCKTMRr BCMI_VIPER_XGXS_FX100_LOCKTMRr
#define FX100_LOCKTMRr_SIZE BCMI_VIPER_XGXS_FX100_LOCKTMRr_SIZE
typedef BCMI_VIPER_XGXS_FX100_LOCKTMRr_t FX100_LOCKTMRr_t;
#define FX100_LOCKTMRr_CLR BCMI_VIPER_XGXS_FX100_LOCKTMRr_CLR
#define FX100_LOCKTMRr_SET BCMI_VIPER_XGXS_FX100_LOCKTMRr_SET
#define FX100_LOCKTMRr_GET BCMI_VIPER_XGXS_FX100_LOCKTMRr_GET
#define FX100_LOCKTMRr_FX100_LOCK_THRESf_GET BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_LOCK_THRESf_GET
#define FX100_LOCKTMRr_FX100_LOCK_THRESf_SET BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_LOCK_THRESf_SET
#define FX100_LOCKTMRr_FX100_UNLOCK_THRESf_GET BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_UNLOCK_THRESf_GET
#define FX100_LOCKTMRr_FX100_UNLOCK_THRESf_SET BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_UNLOCK_THRESf_SET
#define FX100_LOCKTMRr_FX100_LOCK_MAXTIMEf_GET BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_LOCK_MAXTIMEf_GET
#define FX100_LOCKTMRr_FX100_LOCK_MAXTIMEf_SET BCMI_VIPER_XGXS_FX100_LOCKTMRr_FX100_LOCK_MAXTIMEf_SET
#define READ_FX100_LOCKTMRr BCMI_VIPER_XGXS_READ_FX100_LOCKTMRr
#define WRITE_FX100_LOCKTMRr BCMI_VIPER_XGXS_WRITE_FX100_LOCKTMRr
#define MODIFY_FX100_LOCKTMRr BCMI_VIPER_XGXS_MODIFY_FX100_LOCKTMRr
#define READLN_FX100_LOCKTMRr BCMI_VIPER_XGXS_READLN_FX100_LOCKTMRr
#define WRITELN_FX100_LOCKTMRr BCMI_VIPER_XGXS_WRITELN_FX100_LOCKTMRr
#define WRITEALL_FX100_LOCKTMRr BCMI_VIPER_XGXS_WRITEALL_FX100_LOCKTMRr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_LOCKTMRr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  FX100_LNKTMR
 * BLOCKS:   FX100
 * REGADDR:  0x840c
 * DESC:     100FX link timer register
 * RESETVAL: 0x17f4 (6132)
 * ACCESS:   R/W
 * FIELDS:
 *     FX100_LINKDN_COUNT 100fx link down count
 *     FX100_LINKUP_COUNT 100fx link up count
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_FX100_LNKTMRr (0x0000840c | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_FX100_LNKTMRr_SIZE 4

/*
 * This structure should be used to declare and program FX100_LNKTMR.
 *
 */
typedef union BCMI_VIPER_XGXS_FX100_LNKTMRr_s {
	uint32_t v[1];
	uint32_t fx100_lnktmr[1];
	uint32_t _fx100_lnktmr;
} BCMI_VIPER_XGXS_FX100_LNKTMRr_t;

#define BCMI_VIPER_XGXS_FX100_LNKTMRr_CLR(r) (r).fx100_lnktmr[0] = 0
#define BCMI_VIPER_XGXS_FX100_LNKTMRr_SET(r,d) (r).fx100_lnktmr[0] = d
#define BCMI_VIPER_XGXS_FX100_LNKTMRr_GET(r) (r).fx100_lnktmr[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_FX100_LNKTMRr_FX100_LINKUP_COUNTf_GET(r) ((((r).fx100_lnktmr[0]) >> 8) & 0xff)
#define BCMI_VIPER_XGXS_FX100_LNKTMRr_FX100_LINKUP_COUNTf_SET(r,f) (r).fx100_lnktmr[0]=(((r).fx100_lnktmr[0] & ~((uint32_t)0xff << 8)) | ((((uint32_t)f) & 0xff) << 8)) | (255 << (16 + 8))
#define BCMI_VIPER_XGXS_FX100_LNKTMRr_FX100_LINKDN_COUNTf_GET(r) (((r).fx100_lnktmr[0]) & 0xff)
#define BCMI_VIPER_XGXS_FX100_LNKTMRr_FX100_LINKDN_COUNTf_SET(r,f) (r).fx100_lnktmr[0]=(((r).fx100_lnktmr[0] & ~((uint32_t)0xff)) | (((uint32_t)f) & 0xff)) | (0xff << 16)

/*
 * These macros can be used to access FX100_LNKTMR.
 *
 */
#define BCMI_VIPER_XGXS_READ_FX100_LNKTMRr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_LNKTMRr,(_r._fx100_lnktmr))
#define BCMI_VIPER_XGXS_WRITE_FX100_LNKTMRr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_LNKTMRr,(_r._fx100_lnktmr)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_FX100_LNKTMRr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_LNKTMRr,(_r._fx100_lnktmr))
#define BCMI_VIPER_XGXS_READLN_FX100_LNKTMRr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_FX100_LNKTMRr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_lnktmr))
#define BCMI_VIPER_XGXS_WRITELN_FX100_LNKTMRr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_LNKTMRr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._fx100_lnktmr))
#define BCMI_VIPER_XGXS_WRITEALL_FX100_LNKTMRr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_FX100_LNKTMRr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._fx100_lnktmr))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define FX100_LNKTMRr BCMI_VIPER_XGXS_FX100_LNKTMRr
#define FX100_LNKTMRr_SIZE BCMI_VIPER_XGXS_FX100_LNKTMRr_SIZE
typedef BCMI_VIPER_XGXS_FX100_LNKTMRr_t FX100_LNKTMRr_t;
#define FX100_LNKTMRr_CLR BCMI_VIPER_XGXS_FX100_LNKTMRr_CLR
#define FX100_LNKTMRr_SET BCMI_VIPER_XGXS_FX100_LNKTMRr_SET
#define FX100_LNKTMRr_GET BCMI_VIPER_XGXS_FX100_LNKTMRr_GET
#define FX100_LNKTMRr_FX100_LINKUP_COUNTf_GET BCMI_VIPER_XGXS_FX100_LNKTMRr_FX100_LINKUP_COUNTf_GET
#define FX100_LNKTMRr_FX100_LINKUP_COUNTf_SET BCMI_VIPER_XGXS_FX100_LNKTMRr_FX100_LINKUP_COUNTf_SET
#define FX100_LNKTMRr_FX100_LINKDN_COUNTf_GET BCMI_VIPER_XGXS_FX100_LNKTMRr_FX100_LINKDN_COUNTf_GET
#define FX100_LNKTMRr_FX100_LINKDN_COUNTf_SET BCMI_VIPER_XGXS_FX100_LNKTMRr_FX100_LINKDN_COUNTf_SET
#define READ_FX100_LNKTMRr BCMI_VIPER_XGXS_READ_FX100_LNKTMRr
#define WRITE_FX100_LNKTMRr BCMI_VIPER_XGXS_WRITE_FX100_LNKTMRr
#define MODIFY_FX100_LNKTMRr BCMI_VIPER_XGXS_MODIFY_FX100_LNKTMRr
#define READLN_FX100_LNKTMRr BCMI_VIPER_XGXS_READLN_FX100_LNKTMRr
#define WRITELN_FX100_LNKTMRr BCMI_VIPER_XGXS_WRITELN_FX100_LNKTMRr
#define WRITEALL_FX100_LNKTMRr BCMI_VIPER_XGXS_WRITEALL_FX100_LNKTMRr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_FX100_LNKTMRr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_SEQ0
 * BLOCKS:   RX2
 * REGADDR:  0x8470
 * DESC:     rxseq0 Register
 * RESETVAL: 0x1080 (4224)
 * ACCESS:   R/W
 * FIELDS:
 *     CDRLOCKTIMEACQ_S1_SM 
 *     CDRLOCKTIMETRCKNRML_SM 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_SEQ0r (0x00008470 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_SEQ0r_SIZE 4

/*
 * This structure should be used to declare and program RX2_SEQ0.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_SEQ0r_s {
	uint32_t v[1];
	uint32_t rx2_seq0[1];
	uint32_t _rx2_seq0;
} BCMI_VIPER_XGXS_RX2_SEQ0r_t;

#define BCMI_VIPER_XGXS_RX2_SEQ0r_CLR(r) (r).rx2_seq0[0] = 0
#define BCMI_VIPER_XGXS_RX2_SEQ0r_SET(r,d) (r).rx2_seq0[0] = d
#define BCMI_VIPER_XGXS_RX2_SEQ0r_GET(r) (r).rx2_seq0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_SEQ0r_CDRLOCKTIMETRCKNRML_SMf_GET(r) ((((r).rx2_seq0[0]) >> 8) & 0xff)
#define BCMI_VIPER_XGXS_RX2_SEQ0r_CDRLOCKTIMETRCKNRML_SMf_SET(r,f) (r).rx2_seq0[0]=(((r).rx2_seq0[0] & ~((uint32_t)0xff << 8)) | ((((uint32_t)f) & 0xff) << 8)) | (255 << (16 + 8))
#define BCMI_VIPER_XGXS_RX2_SEQ0r_CDRLOCKTIMEACQ_S1_SMf_GET(r) (((r).rx2_seq0[0]) & 0xff)
#define BCMI_VIPER_XGXS_RX2_SEQ0r_CDRLOCKTIMEACQ_S1_SMf_SET(r,f) (r).rx2_seq0[0]=(((r).rx2_seq0[0] & ~((uint32_t)0xff)) | (((uint32_t)f) & 0xff)) | (0xff << 16)

/*
 * These macros can be used to access RX2_SEQ0.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_SEQ0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_SEQ0r,(_r._rx2_seq0))
#define BCMI_VIPER_XGXS_WRITE_RX2_SEQ0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_SEQ0r,(_r._rx2_seq0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_SEQ0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_SEQ0r,(_r._rx2_seq0))
#define BCMI_VIPER_XGXS_READLN_RX2_SEQ0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_SEQ0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_seq0))
#define BCMI_VIPER_XGXS_WRITELN_RX2_SEQ0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_SEQ0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_seq0))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_SEQ0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_SEQ0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_seq0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_SEQ0r BCMI_VIPER_XGXS_RX2_SEQ0r
#define RX2_SEQ0r_SIZE BCMI_VIPER_XGXS_RX2_SEQ0r_SIZE
typedef BCMI_VIPER_XGXS_RX2_SEQ0r_t RX2_SEQ0r_t;
#define RX2_SEQ0r_CLR BCMI_VIPER_XGXS_RX2_SEQ0r_CLR
#define RX2_SEQ0r_SET BCMI_VIPER_XGXS_RX2_SEQ0r_SET
#define RX2_SEQ0r_GET BCMI_VIPER_XGXS_RX2_SEQ0r_GET
#define RX2_SEQ0r_CDRLOCKTIMETRCKNRML_SMf_GET BCMI_VIPER_XGXS_RX2_SEQ0r_CDRLOCKTIMETRCKNRML_SMf_GET
#define RX2_SEQ0r_CDRLOCKTIMETRCKNRML_SMf_SET BCMI_VIPER_XGXS_RX2_SEQ0r_CDRLOCKTIMETRCKNRML_SMf_SET
#define RX2_SEQ0r_CDRLOCKTIMEACQ_S1_SMf_GET BCMI_VIPER_XGXS_RX2_SEQ0r_CDRLOCKTIMEACQ_S1_SMf_GET
#define RX2_SEQ0r_CDRLOCKTIMEACQ_S1_SMf_SET BCMI_VIPER_XGXS_RX2_SEQ0r_CDRLOCKTIMEACQ_S1_SMf_SET
#define READ_RX2_SEQ0r BCMI_VIPER_XGXS_READ_RX2_SEQ0r
#define WRITE_RX2_SEQ0r BCMI_VIPER_XGXS_WRITE_RX2_SEQ0r
#define MODIFY_RX2_SEQ0r BCMI_VIPER_XGXS_MODIFY_RX2_SEQ0r
#define READLN_RX2_SEQ0r BCMI_VIPER_XGXS_READLN_RX2_SEQ0r
#define WRITELN_RX2_SEQ0r BCMI_VIPER_XGXS_WRITELN_RX2_SEQ0r
#define WRITEALL_RX2_SEQ0r BCMI_VIPER_XGXS_WRITEALL_RX2_SEQ0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_SEQ0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_SEQ1
 * BLOCKS:   RX2
 * REGADDR:  0x8471
 * DESC:     rxseq1 Register
 * RESETVAL: 0x8080 (32896)
 * ACCESS:   R/W
 * FIELDS:
 *     CDRLOCKTIMEACQ_S3_SM 
 *     CDRLOCKTIMEACQ_S2_SM 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_SEQ1r (0x00008471 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_SEQ1r_SIZE 4

/*
 * This structure should be used to declare and program RX2_SEQ1.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_SEQ1r_s {
	uint32_t v[1];
	uint32_t rx2_seq1[1];
	uint32_t _rx2_seq1;
} BCMI_VIPER_XGXS_RX2_SEQ1r_t;

#define BCMI_VIPER_XGXS_RX2_SEQ1r_CLR(r) (r).rx2_seq1[0] = 0
#define BCMI_VIPER_XGXS_RX2_SEQ1r_SET(r,d) (r).rx2_seq1[0] = d
#define BCMI_VIPER_XGXS_RX2_SEQ1r_GET(r) (r).rx2_seq1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_SEQ1r_CDRLOCKTIMEACQ_S2_SMf_GET(r) ((((r).rx2_seq1[0]) >> 8) & 0xff)
#define BCMI_VIPER_XGXS_RX2_SEQ1r_CDRLOCKTIMEACQ_S2_SMf_SET(r,f) (r).rx2_seq1[0]=(((r).rx2_seq1[0] & ~((uint32_t)0xff << 8)) | ((((uint32_t)f) & 0xff) << 8)) | (255 << (16 + 8))
#define BCMI_VIPER_XGXS_RX2_SEQ1r_CDRLOCKTIMEACQ_S3_SMf_GET(r) (((r).rx2_seq1[0]) & 0xff)
#define BCMI_VIPER_XGXS_RX2_SEQ1r_CDRLOCKTIMEACQ_S3_SMf_SET(r,f) (r).rx2_seq1[0]=(((r).rx2_seq1[0] & ~((uint32_t)0xff)) | (((uint32_t)f) & 0xff)) | (0xff << 16)

/*
 * These macros can be used to access RX2_SEQ1.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_SEQ1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_SEQ1r,(_r._rx2_seq1))
#define BCMI_VIPER_XGXS_WRITE_RX2_SEQ1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_SEQ1r,(_r._rx2_seq1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_SEQ1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_SEQ1r,(_r._rx2_seq1))
#define BCMI_VIPER_XGXS_READLN_RX2_SEQ1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_SEQ1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_seq1))
#define BCMI_VIPER_XGXS_WRITELN_RX2_SEQ1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_SEQ1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_seq1))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_SEQ1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_SEQ1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_seq1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_SEQ1r BCMI_VIPER_XGXS_RX2_SEQ1r
#define RX2_SEQ1r_SIZE BCMI_VIPER_XGXS_RX2_SEQ1r_SIZE
typedef BCMI_VIPER_XGXS_RX2_SEQ1r_t RX2_SEQ1r_t;
#define RX2_SEQ1r_CLR BCMI_VIPER_XGXS_RX2_SEQ1r_CLR
#define RX2_SEQ1r_SET BCMI_VIPER_XGXS_RX2_SEQ1r_SET
#define RX2_SEQ1r_GET BCMI_VIPER_XGXS_RX2_SEQ1r_GET
#define RX2_SEQ1r_CDRLOCKTIMEACQ_S2_SMf_GET BCMI_VIPER_XGXS_RX2_SEQ1r_CDRLOCKTIMEACQ_S2_SMf_GET
#define RX2_SEQ1r_CDRLOCKTIMEACQ_S2_SMf_SET BCMI_VIPER_XGXS_RX2_SEQ1r_CDRLOCKTIMEACQ_S2_SMf_SET
#define RX2_SEQ1r_CDRLOCKTIMEACQ_S3_SMf_GET BCMI_VIPER_XGXS_RX2_SEQ1r_CDRLOCKTIMEACQ_S3_SMf_GET
#define RX2_SEQ1r_CDRLOCKTIMEACQ_S3_SMf_SET BCMI_VIPER_XGXS_RX2_SEQ1r_CDRLOCKTIMEACQ_S3_SMf_SET
#define READ_RX2_SEQ1r BCMI_VIPER_XGXS_READ_RX2_SEQ1r
#define WRITE_RX2_SEQ1r BCMI_VIPER_XGXS_WRITE_RX2_SEQ1r
#define MODIFY_RX2_SEQ1r BCMI_VIPER_XGXS_MODIFY_RX2_SEQ1r
#define READLN_RX2_SEQ1r BCMI_VIPER_XGXS_READLN_RX2_SEQ1r
#define WRITELN_RX2_SEQ1r BCMI_VIPER_XGXS_WRITELN_RX2_SEQ1r
#define WRITEALL_RX2_SEQ1r BCMI_VIPER_XGXS_WRITEALL_RX2_SEQ1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_SEQ1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_CDR0
 * BLOCKS:   RX2
 * REGADDR:  0x8472
 * DESC:     rxcdr0 Register
 * RESETVAL: 0x4006 (16390)
 * ACCESS:   R/W
 * FIELDS:
 *     PI_PHASE_INVERT  
 *     MDIO_EM_PWRDN    
 *     MDIO_EM_ERR_CNT_FRZ 
 *     MDIO_EM_ERR_CNT_CLR 
 *     PI_PHASE_ROTATE_OVERRIDE 
 *     PI_CLK90_OFFSET_OVERRIDE 
 *     RX_INTERP_STATUS_SEL 
 *     RX_INTERP_CTRL_CAP 
 *     EM_PHASE_SHIFT_360_OVRD 
 *     EM_PHASE_SHIFT_360_OVRD_VAL 
 *     SIGDETTIME_SM    
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_CDR0r (0x00008472 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_CDR0r_SIZE 4

/*
 * This structure should be used to declare and program RX2_CDR0.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_CDR0r_s {
	uint32_t v[1];
	uint32_t rx2_cdr0[1];
	uint32_t _rx2_cdr0;
} BCMI_VIPER_XGXS_RX2_CDR0r_t;

#define BCMI_VIPER_XGXS_RX2_CDR0r_CLR(r) (r).rx2_cdr0[0] = 0
#define BCMI_VIPER_XGXS_RX2_CDR0r_SET(r,d) (r).rx2_cdr0[0] = d
#define BCMI_VIPER_XGXS_RX2_CDR0r_GET(r) (r).rx2_cdr0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_CDR0r_SIGDETTIME_SMf_GET(r) ((((r).rx2_cdr0[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_RX2_CDR0r_SIGDETTIME_SMf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_RX2_CDR0r_EM_PHASE_SHIFT_360_OVRD_VALf_GET(r) ((((r).rx2_cdr0[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR0r_EM_PHASE_SHIFT_360_OVRD_VALf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_RX2_CDR0r_EM_PHASE_SHIFT_360_OVRDf_GET(r) ((((r).rx2_cdr0[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR0r_EM_PHASE_SHIFT_360_OVRDf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_RX2_CDR0r_RX_INTERP_CTRL_CAPf_GET(r) ((((r).rx2_cdr0[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR0r_RX_INTERP_CTRL_CAPf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_RX2_CDR0r_RX_INTERP_STATUS_SELf_GET(r) ((((r).rx2_cdr0[0]) >> 6) & 0x7)
#define BCMI_VIPER_XGXS_RX2_CDR0r_RX_INTERP_STATUS_SELf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x7 << 6)) | ((((uint32_t)f) & 0x7) << 6)) | (7 << (16 + 6))
#define BCMI_VIPER_XGXS_RX2_CDR0r_PI_CLK90_OFFSET_OVERRIDEf_GET(r) ((((r).rx2_cdr0[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR0r_PI_CLK90_OFFSET_OVERRIDEf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX2_CDR0r_PI_PHASE_ROTATE_OVERRIDEf_GET(r) ((((r).rx2_cdr0[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR0r_PI_PHASE_ROTATE_OVERRIDEf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_ERR_CNT_CLRf_GET(r) ((((r).rx2_cdr0[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_ERR_CNT_CLRf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_ERR_CNT_FRZf_GET(r) ((((r).rx2_cdr0[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_ERR_CNT_FRZf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_PWRDNf_GET(r) ((((r).rx2_cdr0[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_PWRDNf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_RX2_CDR0r_PI_PHASE_INVERTf_GET(r) (((r).rx2_cdr0[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR0r_PI_PHASE_INVERTf_SET(r,f) (r).rx2_cdr0[0]=(((r).rx2_cdr0[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX2_CDR0.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_CDR0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR0r,(_r._rx2_cdr0))
#define BCMI_VIPER_XGXS_WRITE_RX2_CDR0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR0r,(_r._rx2_cdr0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_CDR0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR0r,(_r._rx2_cdr0))
#define BCMI_VIPER_XGXS_READLN_RX2_CDR0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr0))
#define BCMI_VIPER_XGXS_WRITELN_RX2_CDR0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr0))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_CDR0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_cdr0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_CDR0r BCMI_VIPER_XGXS_RX2_CDR0r
#define RX2_CDR0r_SIZE BCMI_VIPER_XGXS_RX2_CDR0r_SIZE
typedef BCMI_VIPER_XGXS_RX2_CDR0r_t RX2_CDR0r_t;
#define RX2_CDR0r_CLR BCMI_VIPER_XGXS_RX2_CDR0r_CLR
#define RX2_CDR0r_SET BCMI_VIPER_XGXS_RX2_CDR0r_SET
#define RX2_CDR0r_GET BCMI_VIPER_XGXS_RX2_CDR0r_GET
#define RX2_CDR0r_SIGDETTIME_SMf_GET BCMI_VIPER_XGXS_RX2_CDR0r_SIGDETTIME_SMf_GET
#define RX2_CDR0r_SIGDETTIME_SMf_SET BCMI_VIPER_XGXS_RX2_CDR0r_SIGDETTIME_SMf_SET
#define RX2_CDR0r_EM_PHASE_SHIFT_360_OVRD_VALf_GET BCMI_VIPER_XGXS_RX2_CDR0r_EM_PHASE_SHIFT_360_OVRD_VALf_GET
#define RX2_CDR0r_EM_PHASE_SHIFT_360_OVRD_VALf_SET BCMI_VIPER_XGXS_RX2_CDR0r_EM_PHASE_SHIFT_360_OVRD_VALf_SET
#define RX2_CDR0r_EM_PHASE_SHIFT_360_OVRDf_GET BCMI_VIPER_XGXS_RX2_CDR0r_EM_PHASE_SHIFT_360_OVRDf_GET
#define RX2_CDR0r_EM_PHASE_SHIFT_360_OVRDf_SET BCMI_VIPER_XGXS_RX2_CDR0r_EM_PHASE_SHIFT_360_OVRDf_SET
#define RX2_CDR0r_RX_INTERP_CTRL_CAPf_GET BCMI_VIPER_XGXS_RX2_CDR0r_RX_INTERP_CTRL_CAPf_GET
#define RX2_CDR0r_RX_INTERP_CTRL_CAPf_SET BCMI_VIPER_XGXS_RX2_CDR0r_RX_INTERP_CTRL_CAPf_SET
#define RX2_CDR0r_RX_INTERP_STATUS_SELf_GET BCMI_VIPER_XGXS_RX2_CDR0r_RX_INTERP_STATUS_SELf_GET
#define RX2_CDR0r_RX_INTERP_STATUS_SELf_SET BCMI_VIPER_XGXS_RX2_CDR0r_RX_INTERP_STATUS_SELf_SET
#define RX2_CDR0r_PI_CLK90_OFFSET_OVERRIDEf_GET BCMI_VIPER_XGXS_RX2_CDR0r_PI_CLK90_OFFSET_OVERRIDEf_GET
#define RX2_CDR0r_PI_CLK90_OFFSET_OVERRIDEf_SET BCMI_VIPER_XGXS_RX2_CDR0r_PI_CLK90_OFFSET_OVERRIDEf_SET
#define RX2_CDR0r_PI_PHASE_ROTATE_OVERRIDEf_GET BCMI_VIPER_XGXS_RX2_CDR0r_PI_PHASE_ROTATE_OVERRIDEf_GET
#define RX2_CDR0r_PI_PHASE_ROTATE_OVERRIDEf_SET BCMI_VIPER_XGXS_RX2_CDR0r_PI_PHASE_ROTATE_OVERRIDEf_SET
#define RX2_CDR0r_MDIO_EM_ERR_CNT_CLRf_GET BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_ERR_CNT_CLRf_GET
#define RX2_CDR0r_MDIO_EM_ERR_CNT_CLRf_SET BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_ERR_CNT_CLRf_SET
#define RX2_CDR0r_MDIO_EM_ERR_CNT_FRZf_GET BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_ERR_CNT_FRZf_GET
#define RX2_CDR0r_MDIO_EM_ERR_CNT_FRZf_SET BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_ERR_CNT_FRZf_SET
#define RX2_CDR0r_MDIO_EM_PWRDNf_GET BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_PWRDNf_GET
#define RX2_CDR0r_MDIO_EM_PWRDNf_SET BCMI_VIPER_XGXS_RX2_CDR0r_MDIO_EM_PWRDNf_SET
#define RX2_CDR0r_PI_PHASE_INVERTf_GET BCMI_VIPER_XGXS_RX2_CDR0r_PI_PHASE_INVERTf_GET
#define RX2_CDR0r_PI_PHASE_INVERTf_SET BCMI_VIPER_XGXS_RX2_CDR0r_PI_PHASE_INVERTf_SET
#define READ_RX2_CDR0r BCMI_VIPER_XGXS_READ_RX2_CDR0r
#define WRITE_RX2_CDR0r BCMI_VIPER_XGXS_WRITE_RX2_CDR0r
#define MODIFY_RX2_CDR0r BCMI_VIPER_XGXS_MODIFY_RX2_CDR0r
#define READLN_RX2_CDR0r BCMI_VIPER_XGXS_READLN_RX2_CDR0r
#define WRITELN_RX2_CDR0r BCMI_VIPER_XGXS_WRITELN_RX2_CDR0r
#define WRITEALL_RX2_CDR0r BCMI_VIPER_XGXS_WRITEALL_RX2_CDR0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_CDR0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_CDR1
 * BLOCKS:   RX2
 * REGADDR:  0x8473
 * DESC:     rxcdr1 Register
 * RESETVAL: 0x1451 (5201)
 * ACCESS:   R/W
 * FIELDS:
 *     PHASE_SAT_CTRL   
 *     PHS_COUNTER_CLR  
 *     PHASE_DELTA      
 *     FREQ_UPD_EN      
 *     FALLING_EDGE     
 *     RISING_EDGE      
 *     FLIP_PEAK_POLARITY 
 *     FLIP_ZERO_POLARITY 
 *     STEP_ONE         purposely set to two for sgmii
 *     STEP_TWO         
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_CDR1r (0x00008473 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_CDR1r_SIZE 4

/*
 * This structure should be used to declare and program RX2_CDR1.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_CDR1r_s {
	uint32_t v[1];
	uint32_t rx2_cdr1[1];
	uint32_t _rx2_cdr1;
} BCMI_VIPER_XGXS_RX2_CDR1r_t;

#define BCMI_VIPER_XGXS_RX2_CDR1r_CLR(r) (r).rx2_cdr1[0] = 0
#define BCMI_VIPER_XGXS_RX2_CDR1r_SET(r,d) (r).rx2_cdr1[0] = d
#define BCMI_VIPER_XGXS_RX2_CDR1r_GET(r) (r).rx2_cdr1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_CDR1r_STEP_TWOf_GET(r) ((((r).rx2_cdr1[0]) >> 11) & 0x3)
#define BCMI_VIPER_XGXS_RX2_CDR1r_STEP_TWOf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x3 << 11)) | ((((uint32_t)f) & 0x3) << 11)) | (3 << (16 + 11))
#define BCMI_VIPER_XGXS_RX2_CDR1r_STEP_ONEf_GET(r) ((((r).rx2_cdr1[0]) >> 9) & 0x3)
#define BCMI_VIPER_XGXS_RX2_CDR1r_STEP_ONEf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x3 << 9)) | ((((uint32_t)f) & 0x3) << 9)) | (3 << (16 + 9))
#define BCMI_VIPER_XGXS_RX2_CDR1r_FLIP_ZERO_POLARITYf_GET(r) ((((r).rx2_cdr1[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR1r_FLIP_ZERO_POLARITYf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_RX2_CDR1r_FLIP_PEAK_POLARITYf_GET(r) ((((r).rx2_cdr1[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR1r_FLIP_PEAK_POLARITYf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_RX2_CDR1r_RISING_EDGEf_GET(r) ((((r).rx2_cdr1[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR1r_RISING_EDGEf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX2_CDR1r_FALLING_EDGEf_GET(r) ((((r).rx2_cdr1[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR1r_FALLING_EDGEf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX2_CDR1r_FREQ_UPD_ENf_GET(r) ((((r).rx2_cdr1[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR1r_FREQ_UPD_ENf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_RX2_CDR1r_PHASE_DELTAf_GET(r) ((((r).rx2_cdr1[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR1r_PHASE_DELTAf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX2_CDR1r_PHS_COUNTER_CLRf_GET(r) ((((r).rx2_cdr1[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR1r_PHS_COUNTER_CLRf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_RX2_CDR1r_PHASE_SAT_CTRLf_GET(r) (((r).rx2_cdr1[0]) & 0x3)
#define BCMI_VIPER_XGXS_RX2_CDR1r_PHASE_SAT_CTRLf_SET(r,f) (r).rx2_cdr1[0]=(((r).rx2_cdr1[0] & ~((uint32_t)0x3)) | (((uint32_t)f) & 0x3)) | (0x3 << 16)

/*
 * These macros can be used to access RX2_CDR1.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_CDR1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR1r,(_r._rx2_cdr1))
#define BCMI_VIPER_XGXS_WRITE_RX2_CDR1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR1r,(_r._rx2_cdr1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_CDR1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR1r,(_r._rx2_cdr1))
#define BCMI_VIPER_XGXS_READLN_RX2_CDR1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr1))
#define BCMI_VIPER_XGXS_WRITELN_RX2_CDR1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr1))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_CDR1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_cdr1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_CDR1r BCMI_VIPER_XGXS_RX2_CDR1r
#define RX2_CDR1r_SIZE BCMI_VIPER_XGXS_RX2_CDR1r_SIZE
typedef BCMI_VIPER_XGXS_RX2_CDR1r_t RX2_CDR1r_t;
#define RX2_CDR1r_CLR BCMI_VIPER_XGXS_RX2_CDR1r_CLR
#define RX2_CDR1r_SET BCMI_VIPER_XGXS_RX2_CDR1r_SET
#define RX2_CDR1r_GET BCMI_VIPER_XGXS_RX2_CDR1r_GET
#define RX2_CDR1r_STEP_TWOf_GET BCMI_VIPER_XGXS_RX2_CDR1r_STEP_TWOf_GET
#define RX2_CDR1r_STEP_TWOf_SET BCMI_VIPER_XGXS_RX2_CDR1r_STEP_TWOf_SET
#define RX2_CDR1r_STEP_ONEf_GET BCMI_VIPER_XGXS_RX2_CDR1r_STEP_ONEf_GET
#define RX2_CDR1r_STEP_ONEf_SET BCMI_VIPER_XGXS_RX2_CDR1r_STEP_ONEf_SET
#define RX2_CDR1r_FLIP_ZERO_POLARITYf_GET BCMI_VIPER_XGXS_RX2_CDR1r_FLIP_ZERO_POLARITYf_GET
#define RX2_CDR1r_FLIP_ZERO_POLARITYf_SET BCMI_VIPER_XGXS_RX2_CDR1r_FLIP_ZERO_POLARITYf_SET
#define RX2_CDR1r_FLIP_PEAK_POLARITYf_GET BCMI_VIPER_XGXS_RX2_CDR1r_FLIP_PEAK_POLARITYf_GET
#define RX2_CDR1r_FLIP_PEAK_POLARITYf_SET BCMI_VIPER_XGXS_RX2_CDR1r_FLIP_PEAK_POLARITYf_SET
#define RX2_CDR1r_RISING_EDGEf_GET BCMI_VIPER_XGXS_RX2_CDR1r_RISING_EDGEf_GET
#define RX2_CDR1r_RISING_EDGEf_SET BCMI_VIPER_XGXS_RX2_CDR1r_RISING_EDGEf_SET
#define RX2_CDR1r_FALLING_EDGEf_GET BCMI_VIPER_XGXS_RX2_CDR1r_FALLING_EDGEf_GET
#define RX2_CDR1r_FALLING_EDGEf_SET BCMI_VIPER_XGXS_RX2_CDR1r_FALLING_EDGEf_SET
#define RX2_CDR1r_FREQ_UPD_ENf_GET BCMI_VIPER_XGXS_RX2_CDR1r_FREQ_UPD_ENf_GET
#define RX2_CDR1r_FREQ_UPD_ENf_SET BCMI_VIPER_XGXS_RX2_CDR1r_FREQ_UPD_ENf_SET
#define RX2_CDR1r_PHASE_DELTAf_GET BCMI_VIPER_XGXS_RX2_CDR1r_PHASE_DELTAf_GET
#define RX2_CDR1r_PHASE_DELTAf_SET BCMI_VIPER_XGXS_RX2_CDR1r_PHASE_DELTAf_SET
#define RX2_CDR1r_PHS_COUNTER_CLRf_GET BCMI_VIPER_XGXS_RX2_CDR1r_PHS_COUNTER_CLRf_GET
#define RX2_CDR1r_PHS_COUNTER_CLRf_SET BCMI_VIPER_XGXS_RX2_CDR1r_PHS_COUNTER_CLRf_SET
#define RX2_CDR1r_PHASE_SAT_CTRLf_GET BCMI_VIPER_XGXS_RX2_CDR1r_PHASE_SAT_CTRLf_GET
#define RX2_CDR1r_PHASE_SAT_CTRLf_SET BCMI_VIPER_XGXS_RX2_CDR1r_PHASE_SAT_CTRLf_SET
#define READ_RX2_CDR1r BCMI_VIPER_XGXS_READ_RX2_CDR1r
#define WRITE_RX2_CDR1r BCMI_VIPER_XGXS_WRITE_RX2_CDR1r
#define MODIFY_RX2_CDR1r BCMI_VIPER_XGXS_MODIFY_RX2_CDR1r
#define READLN_RX2_CDR1r BCMI_VIPER_XGXS_READLN_RX2_CDR1r
#define WRITELN_RX2_CDR1r BCMI_VIPER_XGXS_WRITELN_RX2_CDR1r
#define WRITEALL_RX2_CDR1r BCMI_VIPER_XGXS_WRITEALL_RX2_CDR1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_CDR1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_CDR2
 * BLOCKS:   RX2
 * REGADDR:  0x8474
 * DESC:     rxcdr2 Register
 * RESETVAL: 0x1880 (6272)
 * ACCESS:   R/W
 * FIELDS:
 *     PHSACQ_TIMEOUT   
 *     PHSACQ_STEP      
 *     PHSACQ_FREQ_SEL  
 *     PHSACQ_DIR       
 *     RATE_SELECT      
 *     PHSACQ_ENABLE    
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_CDR2r (0x00008474 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_CDR2r_SIZE 4

/*
 * This structure should be used to declare and program RX2_CDR2.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_CDR2r_s {
	uint32_t v[1];
	uint32_t rx2_cdr2[1];
	uint32_t _rx2_cdr2;
} BCMI_VIPER_XGXS_RX2_CDR2r_t;

#define BCMI_VIPER_XGXS_RX2_CDR2r_CLR(r) (r).rx2_cdr2[0] = 0
#define BCMI_VIPER_XGXS_RX2_CDR2r_SET(r,d) (r).rx2_cdr2[0] = d
#define BCMI_VIPER_XGXS_RX2_CDR2r_GET(r) (r).rx2_cdr2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_ENABLEf_GET(r) ((((r).rx2_cdr2[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_ENABLEf_SET(r,f) (r).rx2_cdr2[0]=(((r).rx2_cdr2[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_RX2_CDR2r_RATE_SELECTf_GET(r) ((((r).rx2_cdr2[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR2r_RATE_SELECTf_SET(r,f) (r).rx2_cdr2[0]=(((r).rx2_cdr2[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_DIRf_GET(r) ((((r).rx2_cdr2[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_DIRf_SET(r,f) (r).rx2_cdr2[0]=(((r).rx2_cdr2[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_FREQ_SELf_GET(r) ((((r).rx2_cdr2[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_FREQ_SELf_SET(r,f) (r).rx2_cdr2[0]=(((r).rx2_cdr2[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_STEPf_GET(r) ((((r).rx2_cdr2[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_STEPf_SET(r,f) (r).rx2_cdr2[0]=(((r).rx2_cdr2[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_TIMEOUTf_GET(r) (((r).rx2_cdr2[0]) & 0xff)
#define BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_TIMEOUTf_SET(r,f) (r).rx2_cdr2[0]=(((r).rx2_cdr2[0] & ~((uint32_t)0xff)) | (((uint32_t)f) & 0xff)) | (0xff << 16)

/*
 * These macros can be used to access RX2_CDR2.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_CDR2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR2r,(_r._rx2_cdr2))
#define BCMI_VIPER_XGXS_WRITE_RX2_CDR2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR2r,(_r._rx2_cdr2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_CDR2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR2r,(_r._rx2_cdr2))
#define BCMI_VIPER_XGXS_READLN_RX2_CDR2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr2))
#define BCMI_VIPER_XGXS_WRITELN_RX2_CDR2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr2))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_CDR2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_cdr2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_CDR2r BCMI_VIPER_XGXS_RX2_CDR2r
#define RX2_CDR2r_SIZE BCMI_VIPER_XGXS_RX2_CDR2r_SIZE
typedef BCMI_VIPER_XGXS_RX2_CDR2r_t RX2_CDR2r_t;
#define RX2_CDR2r_CLR BCMI_VIPER_XGXS_RX2_CDR2r_CLR
#define RX2_CDR2r_SET BCMI_VIPER_XGXS_RX2_CDR2r_SET
#define RX2_CDR2r_GET BCMI_VIPER_XGXS_RX2_CDR2r_GET
#define RX2_CDR2r_PHSACQ_ENABLEf_GET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_ENABLEf_GET
#define RX2_CDR2r_PHSACQ_ENABLEf_SET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_ENABLEf_SET
#define RX2_CDR2r_RATE_SELECTf_GET BCMI_VIPER_XGXS_RX2_CDR2r_RATE_SELECTf_GET
#define RX2_CDR2r_RATE_SELECTf_SET BCMI_VIPER_XGXS_RX2_CDR2r_RATE_SELECTf_SET
#define RX2_CDR2r_PHSACQ_DIRf_GET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_DIRf_GET
#define RX2_CDR2r_PHSACQ_DIRf_SET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_DIRf_SET
#define RX2_CDR2r_PHSACQ_FREQ_SELf_GET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_FREQ_SELf_GET
#define RX2_CDR2r_PHSACQ_FREQ_SELf_SET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_FREQ_SELf_SET
#define RX2_CDR2r_PHSACQ_STEPf_GET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_STEPf_GET
#define RX2_CDR2r_PHSACQ_STEPf_SET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_STEPf_SET
#define RX2_CDR2r_PHSACQ_TIMEOUTf_GET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_TIMEOUTf_GET
#define RX2_CDR2r_PHSACQ_TIMEOUTf_SET BCMI_VIPER_XGXS_RX2_CDR2r_PHSACQ_TIMEOUTf_SET
#define READ_RX2_CDR2r BCMI_VIPER_XGXS_READ_RX2_CDR2r
#define WRITE_RX2_CDR2r BCMI_VIPER_XGXS_WRITE_RX2_CDR2r
#define MODIFY_RX2_CDR2r BCMI_VIPER_XGXS_MODIFY_RX2_CDR2r
#define READLN_RX2_CDR2r BCMI_VIPER_XGXS_READLN_RX2_CDR2r
#define WRITELN_RX2_CDR2r BCMI_VIPER_XGXS_WRITELN_RX2_CDR2r
#define WRITEALL_RX2_CDR2r BCMI_VIPER_XGXS_WRITEALL_RX2_CDR2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_CDR2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_CDR3
 * BLOCKS:   RX2
 * REGADDR:  0x8475
 * DESC:     rxcdr3 Register
 * RESETVAL: 0x1004 (4100)
 * ACCESS:   R/W
 * FIELDS:
 *     PHASE_DELTA_SM   
 *     PHASE_STROBE_SM  
 *     PHASE_DEC_SM     
 *     PHASE_INC_SM     
 *     PHASE_OVERRIDE_SM 
 *     PHASE_FRZ_1_EN   
 *     PHASE_FRZ_1      
 *     PHASE_STEP       purposely set to two for sgmii
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_CDR3r (0x00008475 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_CDR3r_SIZE 4

/*
 * This structure should be used to declare and program RX2_CDR3.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_CDR3r_s {
	uint32_t v[1];
	uint32_t rx2_cdr3[1];
	uint32_t _rx2_cdr3;
} BCMI_VIPER_XGXS_RX2_CDR3r_t;

#define BCMI_VIPER_XGXS_RX2_CDR3r_CLR(r) (r).rx2_cdr3[0] = 0
#define BCMI_VIPER_XGXS_RX2_CDR3r_SET(r,d) (r).rx2_cdr3[0] = d
#define BCMI_VIPER_XGXS_RX2_CDR3r_GET(r) (r).rx2_cdr3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_STEPf_GET(r) ((((r).rx2_cdr3[0]) >> 11) & 0x3)
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_STEPf_SET(r,f) (r).rx2_cdr3[0]=(((r).rx2_cdr3[0] & ~((uint32_t)0x3 << 11)) | ((((uint32_t)f) & 0x3) << 11)) | (3 << (16 + 11))
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_FRZ_1f_GET(r) ((((r).rx2_cdr3[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_FRZ_1f_SET(r,f) (r).rx2_cdr3[0]=(((r).rx2_cdr3[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_FRZ_1_ENf_GET(r) ((((r).rx2_cdr3[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_FRZ_1_ENf_SET(r,f) (r).rx2_cdr3[0]=(((r).rx2_cdr3[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_OVERRIDE_SMf_GET(r) ((((r).rx2_cdr3[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_OVERRIDE_SMf_SET(r,f) (r).rx2_cdr3[0]=(((r).rx2_cdr3[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_INC_SMf_GET(r) ((((r).rx2_cdr3[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_INC_SMf_SET(r,f) (r).rx2_cdr3[0]=(((r).rx2_cdr3[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_DEC_SMf_GET(r) ((((r).rx2_cdr3[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_DEC_SMf_SET(r,f) (r).rx2_cdr3[0]=(((r).rx2_cdr3[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_STROBE_SMf_GET(r) ((((r).rx2_cdr3[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_STROBE_SMf_SET(r,f) (r).rx2_cdr3[0]=(((r).rx2_cdr3[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_DELTA_SMf_GET(r) (((r).rx2_cdr3[0]) & 0x1f)
#define BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_DELTA_SMf_SET(r,f) (r).rx2_cdr3[0]=(((r).rx2_cdr3[0] & ~((uint32_t)0x1f)) | (((uint32_t)f) & 0x1f)) | (0x1f << 16)

/*
 * These macros can be used to access RX2_CDR3.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_CDR3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR3r,(_r._rx2_cdr3))
#define BCMI_VIPER_XGXS_WRITE_RX2_CDR3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR3r,(_r._rx2_cdr3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_CDR3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR3r,(_r._rx2_cdr3))
#define BCMI_VIPER_XGXS_READLN_RX2_CDR3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr3))
#define BCMI_VIPER_XGXS_WRITELN_RX2_CDR3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr3))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_CDR3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_cdr3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_CDR3r BCMI_VIPER_XGXS_RX2_CDR3r
#define RX2_CDR3r_SIZE BCMI_VIPER_XGXS_RX2_CDR3r_SIZE
typedef BCMI_VIPER_XGXS_RX2_CDR3r_t RX2_CDR3r_t;
#define RX2_CDR3r_CLR BCMI_VIPER_XGXS_RX2_CDR3r_CLR
#define RX2_CDR3r_SET BCMI_VIPER_XGXS_RX2_CDR3r_SET
#define RX2_CDR3r_GET BCMI_VIPER_XGXS_RX2_CDR3r_GET
#define RX2_CDR3r_PHASE_STEPf_GET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_STEPf_GET
#define RX2_CDR3r_PHASE_STEPf_SET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_STEPf_SET
#define RX2_CDR3r_PHASE_FRZ_1f_GET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_FRZ_1f_GET
#define RX2_CDR3r_PHASE_FRZ_1f_SET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_FRZ_1f_SET
#define RX2_CDR3r_PHASE_FRZ_1_ENf_GET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_FRZ_1_ENf_GET
#define RX2_CDR3r_PHASE_FRZ_1_ENf_SET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_FRZ_1_ENf_SET
#define RX2_CDR3r_PHASE_OVERRIDE_SMf_GET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_OVERRIDE_SMf_GET
#define RX2_CDR3r_PHASE_OVERRIDE_SMf_SET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_OVERRIDE_SMf_SET
#define RX2_CDR3r_PHASE_INC_SMf_GET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_INC_SMf_GET
#define RX2_CDR3r_PHASE_INC_SMf_SET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_INC_SMf_SET
#define RX2_CDR3r_PHASE_DEC_SMf_GET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_DEC_SMf_GET
#define RX2_CDR3r_PHASE_DEC_SMf_SET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_DEC_SMf_SET
#define RX2_CDR3r_PHASE_STROBE_SMf_GET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_STROBE_SMf_GET
#define RX2_CDR3r_PHASE_STROBE_SMf_SET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_STROBE_SMf_SET
#define RX2_CDR3r_PHASE_DELTA_SMf_GET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_DELTA_SMf_GET
#define RX2_CDR3r_PHASE_DELTA_SMf_SET BCMI_VIPER_XGXS_RX2_CDR3r_PHASE_DELTA_SMf_SET
#define READ_RX2_CDR3r BCMI_VIPER_XGXS_READ_RX2_CDR3r
#define WRITE_RX2_CDR3r BCMI_VIPER_XGXS_WRITE_RX2_CDR3r
#define MODIFY_RX2_CDR3r BCMI_VIPER_XGXS_MODIFY_RX2_CDR3r
#define READLN_RX2_CDR3r BCMI_VIPER_XGXS_READLN_RX2_CDR3r
#define WRITELN_RX2_CDR3r BCMI_VIPER_XGXS_WRITELN_RX2_CDR3r
#define WRITEALL_RX2_CDR3r BCMI_VIPER_XGXS_WRITEALL_RX2_CDR3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_CDR3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_CDR4
 * BLOCKS:   RX2
 * REGADDR:  0x8476
 * DESC:     rxcdr4 Register
 * RESETVAL: 0x4840 (18496)
 * ACCESS:   R/W
 * FIELDS:
 *     FREQ_OVERRIDE_VAL 
 *     FREQ_OVERRIDE_EN 
 *     FREQ_EN          
 *     INTEG_CLR        
 *     BWSEL_PROP       
 *     BWSEL_INTEG      
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_CDR4r (0x00008476 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_CDR4r_SIZE 4

/*
 * This structure should be used to declare and program RX2_CDR4.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_CDR4r_s {
	uint32_t v[1];
	uint32_t rx2_cdr4[1];
	uint32_t _rx2_cdr4;
} BCMI_VIPER_XGXS_RX2_CDR4r_t;

#define BCMI_VIPER_XGXS_RX2_CDR4r_CLR(r) (r).rx2_cdr4[0] = 0
#define BCMI_VIPER_XGXS_RX2_CDR4r_SET(r,d) (r).rx2_cdr4[0] = d
#define BCMI_VIPER_XGXS_RX2_CDR4r_GET(r) (r).rx2_cdr4[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_CDR4r_BWSEL_INTEGf_GET(r) ((((r).rx2_cdr4[0]) >> 12) & 0xf)
#define BCMI_VIPER_XGXS_RX2_CDR4r_BWSEL_INTEGf_SET(r,f) (r).rx2_cdr4[0]=(((r).rx2_cdr4[0] & ~((uint32_t)0xf << 12)) | ((((uint32_t)f) & 0xf) << 12)) | (15 << (16 + 12))
#define BCMI_VIPER_XGXS_RX2_CDR4r_BWSEL_PROPf_GET(r) ((((r).rx2_cdr4[0]) >> 8) & 0xf)
#define BCMI_VIPER_XGXS_RX2_CDR4r_BWSEL_PROPf_SET(r,f) (r).rx2_cdr4[0]=(((r).rx2_cdr4[0] & ~((uint32_t)0xf << 8)) | ((((uint32_t)f) & 0xf) << 8)) | (15 << (16 + 8))
#define BCMI_VIPER_XGXS_RX2_CDR4r_INTEG_CLRf_GET(r) ((((r).rx2_cdr4[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR4r_INTEG_CLRf_SET(r,f) (r).rx2_cdr4[0]=(((r).rx2_cdr4[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_ENf_GET(r) ((((r).rx2_cdr4[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_ENf_SET(r,f) (r).rx2_cdr4[0]=(((r).rx2_cdr4[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_OVERRIDE_ENf_GET(r) ((((r).rx2_cdr4[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_OVERRIDE_ENf_SET(r,f) (r).rx2_cdr4[0]=(((r).rx2_cdr4[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_OVERRIDE_VALf_GET(r) (((r).rx2_cdr4[0]) & 0x1f)
#define BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_OVERRIDE_VALf_SET(r,f) (r).rx2_cdr4[0]=(((r).rx2_cdr4[0] & ~((uint32_t)0x1f)) | (((uint32_t)f) & 0x1f)) | (0x1f << 16)

/*
 * These macros can be used to access RX2_CDR4.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_CDR4r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR4r,(_r._rx2_cdr4))
#define BCMI_VIPER_XGXS_WRITE_RX2_CDR4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR4r,(_r._rx2_cdr4)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_CDR4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR4r,(_r._rx2_cdr4))
#define BCMI_VIPER_XGXS_READLN_RX2_CDR4r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_CDR4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr4))
#define BCMI_VIPER_XGXS_WRITELN_RX2_CDR4r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_cdr4))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_CDR4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_CDR4r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_cdr4))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_CDR4r BCMI_VIPER_XGXS_RX2_CDR4r
#define RX2_CDR4r_SIZE BCMI_VIPER_XGXS_RX2_CDR4r_SIZE
typedef BCMI_VIPER_XGXS_RX2_CDR4r_t RX2_CDR4r_t;
#define RX2_CDR4r_CLR BCMI_VIPER_XGXS_RX2_CDR4r_CLR
#define RX2_CDR4r_SET BCMI_VIPER_XGXS_RX2_CDR4r_SET
#define RX2_CDR4r_GET BCMI_VIPER_XGXS_RX2_CDR4r_GET
#define RX2_CDR4r_BWSEL_INTEGf_GET BCMI_VIPER_XGXS_RX2_CDR4r_BWSEL_INTEGf_GET
#define RX2_CDR4r_BWSEL_INTEGf_SET BCMI_VIPER_XGXS_RX2_CDR4r_BWSEL_INTEGf_SET
#define RX2_CDR4r_BWSEL_PROPf_GET BCMI_VIPER_XGXS_RX2_CDR4r_BWSEL_PROPf_GET
#define RX2_CDR4r_BWSEL_PROPf_SET BCMI_VIPER_XGXS_RX2_CDR4r_BWSEL_PROPf_SET
#define RX2_CDR4r_INTEG_CLRf_GET BCMI_VIPER_XGXS_RX2_CDR4r_INTEG_CLRf_GET
#define RX2_CDR4r_INTEG_CLRf_SET BCMI_VIPER_XGXS_RX2_CDR4r_INTEG_CLRf_SET
#define RX2_CDR4r_FREQ_ENf_GET BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_ENf_GET
#define RX2_CDR4r_FREQ_ENf_SET BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_ENf_SET
#define RX2_CDR4r_FREQ_OVERRIDE_ENf_GET BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_OVERRIDE_ENf_GET
#define RX2_CDR4r_FREQ_OVERRIDE_ENf_SET BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_OVERRIDE_ENf_SET
#define RX2_CDR4r_FREQ_OVERRIDE_VALf_GET BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_OVERRIDE_VALf_GET
#define RX2_CDR4r_FREQ_OVERRIDE_VALf_SET BCMI_VIPER_XGXS_RX2_CDR4r_FREQ_OVERRIDE_VALf_SET
#define READ_RX2_CDR4r BCMI_VIPER_XGXS_READ_RX2_CDR4r
#define WRITE_RX2_CDR4r BCMI_VIPER_XGXS_WRITE_RX2_CDR4r
#define MODIFY_RX2_CDR4r BCMI_VIPER_XGXS_MODIFY_RX2_CDR4r
#define READLN_RX2_CDR4r BCMI_VIPER_XGXS_READLN_RX2_CDR4r
#define WRITELN_RX2_CDR4r BCMI_VIPER_XGXS_WRITELN_RX2_CDR4r
#define WRITEALL_RX2_CDR4r BCMI_VIPER_XGXS_WRITEALL_RX2_CDR4r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_CDR4r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_STS0
 * BLOCKS:   RX2
 * REGADDR:  0x8477
 * DESC:     rxstatus 0 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     RX_SLOFF2_INVALID 
 *     RX_SLOFF1_INVALID 
 *     RX_SLOFF0_INVALID 
 *     RX_SLICER_CAL_DONE 
 *     RX_LMTOFF        
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_STS0r (0x00008477 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_STS0r_SIZE 4

/*
 * This structure should be used to declare and program RX2_STS0.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_STS0r_s {
	uint32_t v[1];
	uint32_t rx2_sts0[1];
	uint32_t _rx2_sts0;
} BCMI_VIPER_XGXS_RX2_STS0r_t;

#define BCMI_VIPER_XGXS_RX2_STS0r_CLR(r) (r).rx2_sts0[0] = 0
#define BCMI_VIPER_XGXS_RX2_STS0r_SET(r,d) (r).rx2_sts0[0] = d
#define BCMI_VIPER_XGXS_RX2_STS0r_GET(r) (r).rx2_sts0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_LMTOFFf_GET(r) ((((r).rx2_sts0[0]) >> 4) & 0x3f)
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_LMTOFFf_SET(r,f) (r).rx2_sts0[0]=(((r).rx2_sts0[0] & ~((uint32_t)0x3f << 4)) | ((((uint32_t)f) & 0x3f) << 4)) | (63 << (16 + 4))
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_SLICER_CAL_DONEf_GET(r) ((((r).rx2_sts0[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_SLICER_CAL_DONEf_SET(r,f) (r).rx2_sts0[0]=(((r).rx2_sts0[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF0_INVALIDf_GET(r) ((((r).rx2_sts0[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF0_INVALIDf_SET(r,f) (r).rx2_sts0[0]=(((r).rx2_sts0[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF1_INVALIDf_GET(r) ((((r).rx2_sts0[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF1_INVALIDf_SET(r,f) (r).rx2_sts0[0]=(((r).rx2_sts0[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF2_INVALIDf_GET(r) (((r).rx2_sts0[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF2_INVALIDf_SET(r,f) (r).rx2_sts0[0]=(((r).rx2_sts0[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX2_STS0.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_STS0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS0r,(_r._rx2_sts0))
#define BCMI_VIPER_XGXS_WRITE_RX2_STS0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS0r,(_r._rx2_sts0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_STS0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS0r,(_r._rx2_sts0))
#define BCMI_VIPER_XGXS_READLN_RX2_STS0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts0))
#define BCMI_VIPER_XGXS_WRITELN_RX2_STS0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts0))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_STS0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_sts0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_STS0r BCMI_VIPER_XGXS_RX2_STS0r
#define RX2_STS0r_SIZE BCMI_VIPER_XGXS_RX2_STS0r_SIZE
typedef BCMI_VIPER_XGXS_RX2_STS0r_t RX2_STS0r_t;
#define RX2_STS0r_CLR BCMI_VIPER_XGXS_RX2_STS0r_CLR
#define RX2_STS0r_SET BCMI_VIPER_XGXS_RX2_STS0r_SET
#define RX2_STS0r_GET BCMI_VIPER_XGXS_RX2_STS0r_GET
#define RX2_STS0r_RX_LMTOFFf_GET BCMI_VIPER_XGXS_RX2_STS0r_RX_LMTOFFf_GET
#define RX2_STS0r_RX_LMTOFFf_SET BCMI_VIPER_XGXS_RX2_STS0r_RX_LMTOFFf_SET
#define RX2_STS0r_RX_SLICER_CAL_DONEf_GET BCMI_VIPER_XGXS_RX2_STS0r_RX_SLICER_CAL_DONEf_GET
#define RX2_STS0r_RX_SLICER_CAL_DONEf_SET BCMI_VIPER_XGXS_RX2_STS0r_RX_SLICER_CAL_DONEf_SET
#define RX2_STS0r_RX_SLOFF0_INVALIDf_GET BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF0_INVALIDf_GET
#define RX2_STS0r_RX_SLOFF0_INVALIDf_SET BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF0_INVALIDf_SET
#define RX2_STS0r_RX_SLOFF1_INVALIDf_GET BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF1_INVALIDf_GET
#define RX2_STS0r_RX_SLOFF1_INVALIDf_SET BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF1_INVALIDf_SET
#define RX2_STS0r_RX_SLOFF2_INVALIDf_GET BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF2_INVALIDf_GET
#define RX2_STS0r_RX_SLOFF2_INVALIDf_SET BCMI_VIPER_XGXS_RX2_STS0r_RX_SLOFF2_INVALIDf_SET
#define READ_RX2_STS0r BCMI_VIPER_XGXS_READ_RX2_STS0r
#define WRITE_RX2_STS0r BCMI_VIPER_XGXS_WRITE_RX2_STS0r
#define MODIFY_RX2_STS0r BCMI_VIPER_XGXS_MODIFY_RX2_STS0r
#define READLN_RX2_STS0r BCMI_VIPER_XGXS_READLN_RX2_STS0r
#define WRITELN_RX2_STS0r BCMI_VIPER_XGXS_WRITELN_RX2_STS0r
#define WRITEALL_RX2_STS0r BCMI_VIPER_XGXS_WRITEALL_RX2_STS0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_STS0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_STS1
 * BLOCKS:   RX2
 * REGADDR:  0x8478
 * DESC:     rxstatus 1 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     SLCAL_STATE      
 *     SLOFFX_VAL       
 *     DN_SLOFFX_VAL    
 *     UP_SLOFFX_VAL    
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_STS1r (0x00008478 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_STS1r_SIZE 4

/*
 * This structure should be used to declare and program RX2_STS1.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_STS1r_s {
	uint32_t v[1];
	uint32_t rx2_sts1[1];
	uint32_t _rx2_sts1;
} BCMI_VIPER_XGXS_RX2_STS1r_t;

#define BCMI_VIPER_XGXS_RX2_STS1r_CLR(r) (r).rx2_sts1[0] = 0
#define BCMI_VIPER_XGXS_RX2_STS1r_SET(r,d) (r).rx2_sts1[0] = d
#define BCMI_VIPER_XGXS_RX2_STS1r_GET(r) (r).rx2_sts1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_STS1r_UP_SLOFFX_VALf_GET(r) ((((r).rx2_sts1[0]) >> 11) & 0xf)
#define BCMI_VIPER_XGXS_RX2_STS1r_UP_SLOFFX_VALf_SET(r,f) (r).rx2_sts1[0]=(((r).rx2_sts1[0] & ~((uint32_t)0xf << 11)) | ((((uint32_t)f) & 0xf) << 11)) | (15 << (16 + 11))
#define BCMI_VIPER_XGXS_RX2_STS1r_DN_SLOFFX_VALf_GET(r) ((((r).rx2_sts1[0]) >> 7) & 0xf)
#define BCMI_VIPER_XGXS_RX2_STS1r_DN_SLOFFX_VALf_SET(r,f) (r).rx2_sts1[0]=(((r).rx2_sts1[0] & ~((uint32_t)0xf << 7)) | ((((uint32_t)f) & 0xf) << 7)) | (15 << (16 + 7))
#define BCMI_VIPER_XGXS_RX2_STS1r_SLOFFX_VALf_GET(r) ((((r).rx2_sts1[0]) >> 3) & 0xf)
#define BCMI_VIPER_XGXS_RX2_STS1r_SLOFFX_VALf_SET(r,f) (r).rx2_sts1[0]=(((r).rx2_sts1[0] & ~((uint32_t)0xf << 3)) | ((((uint32_t)f) & 0xf) << 3)) | (15 << (16 + 3))
#define BCMI_VIPER_XGXS_RX2_STS1r_SLCAL_STATEf_GET(r) (((r).rx2_sts1[0]) & 0x7)
#define BCMI_VIPER_XGXS_RX2_STS1r_SLCAL_STATEf_SET(r,f) (r).rx2_sts1[0]=(((r).rx2_sts1[0] & ~((uint32_t)0x7)) | (((uint32_t)f) & 0x7)) | (0x7 << 16)

/*
 * These macros can be used to access RX2_STS1.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_STS1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS1r,(_r._rx2_sts1))
#define BCMI_VIPER_XGXS_WRITE_RX2_STS1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS1r,(_r._rx2_sts1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_STS1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS1r,(_r._rx2_sts1))
#define BCMI_VIPER_XGXS_READLN_RX2_STS1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts1))
#define BCMI_VIPER_XGXS_WRITELN_RX2_STS1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts1))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_STS1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_sts1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_STS1r BCMI_VIPER_XGXS_RX2_STS1r
#define RX2_STS1r_SIZE BCMI_VIPER_XGXS_RX2_STS1r_SIZE
typedef BCMI_VIPER_XGXS_RX2_STS1r_t RX2_STS1r_t;
#define RX2_STS1r_CLR BCMI_VIPER_XGXS_RX2_STS1r_CLR
#define RX2_STS1r_SET BCMI_VIPER_XGXS_RX2_STS1r_SET
#define RX2_STS1r_GET BCMI_VIPER_XGXS_RX2_STS1r_GET
#define RX2_STS1r_UP_SLOFFX_VALf_GET BCMI_VIPER_XGXS_RX2_STS1r_UP_SLOFFX_VALf_GET
#define RX2_STS1r_UP_SLOFFX_VALf_SET BCMI_VIPER_XGXS_RX2_STS1r_UP_SLOFFX_VALf_SET
#define RX2_STS1r_DN_SLOFFX_VALf_GET BCMI_VIPER_XGXS_RX2_STS1r_DN_SLOFFX_VALf_GET
#define RX2_STS1r_DN_SLOFFX_VALf_SET BCMI_VIPER_XGXS_RX2_STS1r_DN_SLOFFX_VALf_SET
#define RX2_STS1r_SLOFFX_VALf_GET BCMI_VIPER_XGXS_RX2_STS1r_SLOFFX_VALf_GET
#define RX2_STS1r_SLOFFX_VALf_SET BCMI_VIPER_XGXS_RX2_STS1r_SLOFFX_VALf_SET
#define RX2_STS1r_SLCAL_STATEf_GET BCMI_VIPER_XGXS_RX2_STS1r_SLCAL_STATEf_GET
#define RX2_STS1r_SLCAL_STATEf_SET BCMI_VIPER_XGXS_RX2_STS1r_SLCAL_STATEf_SET
#define READ_RX2_STS1r BCMI_VIPER_XGXS_READ_RX2_STS1r
#define WRITE_RX2_STS1r BCMI_VIPER_XGXS_WRITE_RX2_STS1r
#define MODIFY_RX2_STS1r BCMI_VIPER_XGXS_MODIFY_RX2_STS1r
#define READLN_RX2_STS1r BCMI_VIPER_XGXS_READLN_RX2_STS1r
#define WRITELN_RX2_STS1r BCMI_VIPER_XGXS_WRITELN_RX2_STS1r
#define WRITEALL_RX2_STS1r BCMI_VIPER_XGXS_WRITEALL_RX2_STS1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_STS1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_STS2
 * BLOCKS:   RX2
 * REGADDR:  0x8479
 * DESC:     rxstatus 2 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     LMTCAL_ACC       
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_STS2r (0x00008479 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_STS2r_SIZE 4

/*
 * This structure should be used to declare and program RX2_STS2.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_STS2r_s {
	uint32_t v[1];
	uint32_t rx2_sts2[1];
	uint32_t _rx2_sts2;
} BCMI_VIPER_XGXS_RX2_STS2r_t;

#define BCMI_VIPER_XGXS_RX2_STS2r_CLR(r) (r).rx2_sts2[0] = 0
#define BCMI_VIPER_XGXS_RX2_STS2r_SET(r,d) (r).rx2_sts2[0] = d
#define BCMI_VIPER_XGXS_RX2_STS2r_GET(r) (r).rx2_sts2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_STS2r_LMTCAL_ACCf_GET(r) (((r).rx2_sts2[0]) & 0xffff)
#define BCMI_VIPER_XGXS_RX2_STS2r_LMTCAL_ACCf_SET(r,f) (r).rx2_sts2[0]=(((r).rx2_sts2[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access RX2_STS2.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_STS2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS2r,(_r._rx2_sts2))
#define BCMI_VIPER_XGXS_WRITE_RX2_STS2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS2r,(_r._rx2_sts2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_STS2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS2r,(_r._rx2_sts2))
#define BCMI_VIPER_XGXS_READLN_RX2_STS2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts2))
#define BCMI_VIPER_XGXS_WRITELN_RX2_STS2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts2))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_STS2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_sts2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_STS2r BCMI_VIPER_XGXS_RX2_STS2r
#define RX2_STS2r_SIZE BCMI_VIPER_XGXS_RX2_STS2r_SIZE
typedef BCMI_VIPER_XGXS_RX2_STS2r_t RX2_STS2r_t;
#define RX2_STS2r_CLR BCMI_VIPER_XGXS_RX2_STS2r_CLR
#define RX2_STS2r_SET BCMI_VIPER_XGXS_RX2_STS2r_SET
#define RX2_STS2r_GET BCMI_VIPER_XGXS_RX2_STS2r_GET
#define RX2_STS2r_LMTCAL_ACCf_GET BCMI_VIPER_XGXS_RX2_STS2r_LMTCAL_ACCf_GET
#define RX2_STS2r_LMTCAL_ACCf_SET BCMI_VIPER_XGXS_RX2_STS2r_LMTCAL_ACCf_SET
#define READ_RX2_STS2r BCMI_VIPER_XGXS_READ_RX2_STS2r
#define WRITE_RX2_STS2r BCMI_VIPER_XGXS_WRITE_RX2_STS2r
#define MODIFY_RX2_STS2r BCMI_VIPER_XGXS_MODIFY_RX2_STS2r
#define READLN_RX2_STS2r BCMI_VIPER_XGXS_READLN_RX2_STS2r
#define WRITELN_RX2_STS2r BCMI_VIPER_XGXS_WRITELN_RX2_STS2r
#define WRITEALL_RX2_STS2r BCMI_VIPER_XGXS_WRITEALL_RX2_STS2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_STS2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_STS3
 * BLOCKS:   RX2
 * REGADDR:  0x847a
 * DESC:     rxstatus 3 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     RECAL_IND        
 *     LMTCAL_ADJ_CNT   
 *     LMTCAL_VALID     
 *     RX_LA_CAL_DONE   
 *     LMTCAL_STATE     
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_STS3r (0x0000847a | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_STS3r_SIZE 4

/*
 * This structure should be used to declare and program RX2_STS3.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_STS3r_s {
	uint32_t v[1];
	uint32_t rx2_sts3[1];
	uint32_t _rx2_sts3;
} BCMI_VIPER_XGXS_RX2_STS3r_t;

#define BCMI_VIPER_XGXS_RX2_STS3r_CLR(r) (r).rx2_sts3[0] = 0
#define BCMI_VIPER_XGXS_RX2_STS3r_SET(r,d) (r).rx2_sts3[0] = d
#define BCMI_VIPER_XGXS_RX2_STS3r_GET(r) (r).rx2_sts3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_STATEf_GET(r) ((((r).rx2_sts3[0]) >> 8) & 0x7)
#define BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_STATEf_SET(r,f) (r).rx2_sts3[0]=(((r).rx2_sts3[0] & ~((uint32_t)0x7 << 8)) | ((((uint32_t)f) & 0x7) << 8)) | (7 << (16 + 8))
#define BCMI_VIPER_XGXS_RX2_STS3r_RX_LA_CAL_DONEf_GET(r) ((((r).rx2_sts3[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_RX2_STS3r_RX_LA_CAL_DONEf_SET(r,f) (r).rx2_sts3[0]=(((r).rx2_sts3[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_VALIDf_GET(r) ((((r).rx2_sts3[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_VALIDf_SET(r,f) (r).rx2_sts3[0]=(((r).rx2_sts3[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_ADJ_CNTf_GET(r) ((((r).rx2_sts3[0]) >> 1) & 0x1f)
#define BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_ADJ_CNTf_SET(r,f) (r).rx2_sts3[0]=(((r).rx2_sts3[0] & ~((uint32_t)0x1f << 1)) | ((((uint32_t)f) & 0x1f) << 1)) | (31 << (16 + 1))
#define BCMI_VIPER_XGXS_RX2_STS3r_RECAL_INDf_GET(r) (((r).rx2_sts3[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX2_STS3r_RECAL_INDf_SET(r,f) (r).rx2_sts3[0]=(((r).rx2_sts3[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX2_STS3.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_STS3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS3r,(_r._rx2_sts3))
#define BCMI_VIPER_XGXS_WRITE_RX2_STS3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS3r,(_r._rx2_sts3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_STS3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS3r,(_r._rx2_sts3))
#define BCMI_VIPER_XGXS_READLN_RX2_STS3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts3))
#define BCMI_VIPER_XGXS_WRITELN_RX2_STS3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts3))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_STS3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_sts3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_STS3r BCMI_VIPER_XGXS_RX2_STS3r
#define RX2_STS3r_SIZE BCMI_VIPER_XGXS_RX2_STS3r_SIZE
typedef BCMI_VIPER_XGXS_RX2_STS3r_t RX2_STS3r_t;
#define RX2_STS3r_CLR BCMI_VIPER_XGXS_RX2_STS3r_CLR
#define RX2_STS3r_SET BCMI_VIPER_XGXS_RX2_STS3r_SET
#define RX2_STS3r_GET BCMI_VIPER_XGXS_RX2_STS3r_GET
#define RX2_STS3r_LMTCAL_STATEf_GET BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_STATEf_GET
#define RX2_STS3r_LMTCAL_STATEf_SET BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_STATEf_SET
#define RX2_STS3r_RX_LA_CAL_DONEf_GET BCMI_VIPER_XGXS_RX2_STS3r_RX_LA_CAL_DONEf_GET
#define RX2_STS3r_RX_LA_CAL_DONEf_SET BCMI_VIPER_XGXS_RX2_STS3r_RX_LA_CAL_DONEf_SET
#define RX2_STS3r_LMTCAL_VALIDf_GET BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_VALIDf_GET
#define RX2_STS3r_LMTCAL_VALIDf_SET BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_VALIDf_SET
#define RX2_STS3r_LMTCAL_ADJ_CNTf_GET BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_ADJ_CNTf_GET
#define RX2_STS3r_LMTCAL_ADJ_CNTf_SET BCMI_VIPER_XGXS_RX2_STS3r_LMTCAL_ADJ_CNTf_SET
#define RX2_STS3r_RECAL_INDf_GET BCMI_VIPER_XGXS_RX2_STS3r_RECAL_INDf_GET
#define RX2_STS3r_RECAL_INDf_SET BCMI_VIPER_XGXS_RX2_STS3r_RECAL_INDf_SET
#define READ_RX2_STS3r BCMI_VIPER_XGXS_READ_RX2_STS3r
#define WRITE_RX2_STS3r BCMI_VIPER_XGXS_WRITE_RX2_STS3r
#define MODIFY_RX2_STS3r BCMI_VIPER_XGXS_MODIFY_RX2_STS3r
#define READLN_RX2_STS3r BCMI_VIPER_XGXS_READLN_RX2_STS3r
#define WRITELN_RX2_STS3r BCMI_VIPER_XGXS_WRITELN_RX2_STS3r
#define WRITEALL_RX2_STS3r BCMI_VIPER_XGXS_WRITEALL_RX2_STS3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_STS3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_STS4
 * BLOCKS:   RX2
 * REGADDR:  0x847b
 * DESC:     rxstatus 4 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     EM_ERR_CNT_H     
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_STS4r (0x0000847b | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_STS4r_SIZE 4

/*
 * This structure should be used to declare and program RX2_STS4.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_STS4r_s {
	uint32_t v[1];
	uint32_t rx2_sts4[1];
	uint32_t _rx2_sts4;
} BCMI_VIPER_XGXS_RX2_STS4r_t;

#define BCMI_VIPER_XGXS_RX2_STS4r_CLR(r) (r).rx2_sts4[0] = 0
#define BCMI_VIPER_XGXS_RX2_STS4r_SET(r,d) (r).rx2_sts4[0] = d
#define BCMI_VIPER_XGXS_RX2_STS4r_GET(r) (r).rx2_sts4[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_STS4r_EM_ERR_CNT_Hf_GET(r) (((r).rx2_sts4[0]) & 0xffff)
#define BCMI_VIPER_XGXS_RX2_STS4r_EM_ERR_CNT_Hf_SET(r,f) (r).rx2_sts4[0]=(((r).rx2_sts4[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access RX2_STS4.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_STS4r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS4r,(_r._rx2_sts4))
#define BCMI_VIPER_XGXS_WRITE_RX2_STS4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS4r,(_r._rx2_sts4)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_STS4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS4r,(_r._rx2_sts4))
#define BCMI_VIPER_XGXS_READLN_RX2_STS4r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts4))
#define BCMI_VIPER_XGXS_WRITELN_RX2_STS4r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts4))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_STS4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS4r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_sts4))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_STS4r BCMI_VIPER_XGXS_RX2_STS4r
#define RX2_STS4r_SIZE BCMI_VIPER_XGXS_RX2_STS4r_SIZE
typedef BCMI_VIPER_XGXS_RX2_STS4r_t RX2_STS4r_t;
#define RX2_STS4r_CLR BCMI_VIPER_XGXS_RX2_STS4r_CLR
#define RX2_STS4r_SET BCMI_VIPER_XGXS_RX2_STS4r_SET
#define RX2_STS4r_GET BCMI_VIPER_XGXS_RX2_STS4r_GET
#define RX2_STS4r_EM_ERR_CNT_Hf_GET BCMI_VIPER_XGXS_RX2_STS4r_EM_ERR_CNT_Hf_GET
#define RX2_STS4r_EM_ERR_CNT_Hf_SET BCMI_VIPER_XGXS_RX2_STS4r_EM_ERR_CNT_Hf_SET
#define READ_RX2_STS4r BCMI_VIPER_XGXS_READ_RX2_STS4r
#define WRITE_RX2_STS4r BCMI_VIPER_XGXS_WRITE_RX2_STS4r
#define MODIFY_RX2_STS4r BCMI_VIPER_XGXS_MODIFY_RX2_STS4r
#define READLN_RX2_STS4r BCMI_VIPER_XGXS_READLN_RX2_STS4r
#define WRITELN_RX2_STS4r BCMI_VIPER_XGXS_WRITELN_RX2_STS4r
#define WRITEALL_RX2_STS4r BCMI_VIPER_XGXS_WRITEALL_RX2_STS4r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_STS4r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_STS5
 * BLOCKS:   RX2
 * REGADDR:  0x847c
 * DESC:     rxstatus 5 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     EM_ERR_CNT_L     
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_STS5r (0x0000847c | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_STS5r_SIZE 4

/*
 * This structure should be used to declare and program RX2_STS5.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_STS5r_s {
	uint32_t v[1];
	uint32_t rx2_sts5[1];
	uint32_t _rx2_sts5;
} BCMI_VIPER_XGXS_RX2_STS5r_t;

#define BCMI_VIPER_XGXS_RX2_STS5r_CLR(r) (r).rx2_sts5[0] = 0
#define BCMI_VIPER_XGXS_RX2_STS5r_SET(r,d) (r).rx2_sts5[0] = d
#define BCMI_VIPER_XGXS_RX2_STS5r_GET(r) (r).rx2_sts5[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_STS5r_EM_ERR_CNT_Lf_GET(r) (((r).rx2_sts5[0]) & 0xffff)
#define BCMI_VIPER_XGXS_RX2_STS5r_EM_ERR_CNT_Lf_SET(r,f) (r).rx2_sts5[0]=(((r).rx2_sts5[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access RX2_STS5.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_STS5r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS5r,(_r._rx2_sts5))
#define BCMI_VIPER_XGXS_WRITE_RX2_STS5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS5r,(_r._rx2_sts5)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_STS5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS5r,(_r._rx2_sts5))
#define BCMI_VIPER_XGXS_READLN_RX2_STS5r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts5))
#define BCMI_VIPER_XGXS_WRITELN_RX2_STS5r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts5))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_STS5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS5r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_sts5))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_STS5r BCMI_VIPER_XGXS_RX2_STS5r
#define RX2_STS5r_SIZE BCMI_VIPER_XGXS_RX2_STS5r_SIZE
typedef BCMI_VIPER_XGXS_RX2_STS5r_t RX2_STS5r_t;
#define RX2_STS5r_CLR BCMI_VIPER_XGXS_RX2_STS5r_CLR
#define RX2_STS5r_SET BCMI_VIPER_XGXS_RX2_STS5r_SET
#define RX2_STS5r_GET BCMI_VIPER_XGXS_RX2_STS5r_GET
#define RX2_STS5r_EM_ERR_CNT_Lf_GET BCMI_VIPER_XGXS_RX2_STS5r_EM_ERR_CNT_Lf_GET
#define RX2_STS5r_EM_ERR_CNT_Lf_SET BCMI_VIPER_XGXS_RX2_STS5r_EM_ERR_CNT_Lf_SET
#define READ_RX2_STS5r BCMI_VIPER_XGXS_READ_RX2_STS5r
#define WRITE_RX2_STS5r BCMI_VIPER_XGXS_WRITE_RX2_STS5r
#define MODIFY_RX2_STS5r BCMI_VIPER_XGXS_MODIFY_RX2_STS5r
#define READLN_RX2_STS5r BCMI_VIPER_XGXS_READLN_RX2_STS5r
#define WRITELN_RX2_STS5r BCMI_VIPER_XGXS_WRITELN_RX2_STS5r
#define WRITEALL_RX2_STS5r BCMI_VIPER_XGXS_WRITEALL_RX2_STS5r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_STS5r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX2_STS6
 * BLOCKS:   RX2
 * REGADDR:  0x847d
 * DESC:     rxstatus 6 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     RX_PHS_INTERP_STATUS 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX2_STS6r (0x0000847d | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX2_STS6r_SIZE 4

/*
 * This structure should be used to declare and program RX2_STS6.
 *
 */
typedef union BCMI_VIPER_XGXS_RX2_STS6r_s {
	uint32_t v[1];
	uint32_t rx2_sts6[1];
	uint32_t _rx2_sts6;
} BCMI_VIPER_XGXS_RX2_STS6r_t;

#define BCMI_VIPER_XGXS_RX2_STS6r_CLR(r) (r).rx2_sts6[0] = 0
#define BCMI_VIPER_XGXS_RX2_STS6r_SET(r,d) (r).rx2_sts6[0] = d
#define BCMI_VIPER_XGXS_RX2_STS6r_GET(r) (r).rx2_sts6[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX2_STS6r_RX_PHS_INTERP_STATUSf_GET(r) (((r).rx2_sts6[0]) & 0xffff)
#define BCMI_VIPER_XGXS_RX2_STS6r_RX_PHS_INTERP_STATUSf_SET(r,f) (r).rx2_sts6[0]=(((r).rx2_sts6[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access RX2_STS6.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX2_STS6r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS6r,(_r._rx2_sts6))
#define BCMI_VIPER_XGXS_WRITE_RX2_STS6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS6r,(_r._rx2_sts6)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX2_STS6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS6r,(_r._rx2_sts6))
#define BCMI_VIPER_XGXS_READLN_RX2_STS6r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX2_STS6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts6))
#define BCMI_VIPER_XGXS_WRITELN_RX2_STS6r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx2_sts6))
#define BCMI_VIPER_XGXS_WRITEALL_RX2_STS6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX2_STS6r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx2_sts6))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX2_STS6r BCMI_VIPER_XGXS_RX2_STS6r
#define RX2_STS6r_SIZE BCMI_VIPER_XGXS_RX2_STS6r_SIZE
typedef BCMI_VIPER_XGXS_RX2_STS6r_t RX2_STS6r_t;
#define RX2_STS6r_CLR BCMI_VIPER_XGXS_RX2_STS6r_CLR
#define RX2_STS6r_SET BCMI_VIPER_XGXS_RX2_STS6r_SET
#define RX2_STS6r_GET BCMI_VIPER_XGXS_RX2_STS6r_GET
#define RX2_STS6r_RX_PHS_INTERP_STATUSf_GET BCMI_VIPER_XGXS_RX2_STS6r_RX_PHS_INTERP_STATUSf_GET
#define RX2_STS6r_RX_PHS_INTERP_STATUSf_SET BCMI_VIPER_XGXS_RX2_STS6r_RX_PHS_INTERP_STATUSf_SET
#define READ_RX2_STS6r BCMI_VIPER_XGXS_READ_RX2_STS6r
#define WRITE_RX2_STS6r BCMI_VIPER_XGXS_WRITE_RX2_STS6r
#define MODIFY_RX2_STS6r BCMI_VIPER_XGXS_MODIFY_RX2_STS6r
#define READLN_RX2_STS6r BCMI_VIPER_XGXS_READLN_RX2_STS6r
#define WRITELN_RX2_STS6r BCMI_VIPER_XGXS_WRITELN_RX2_STS6r
#define WRITEALL_RX2_STS6r BCMI_VIPER_XGXS_WRITEALL_RX2_STS6r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX2_STS6r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL0
 * BLOCKS:   RX3
 * REGADDR:  0x8480
 * DESC:     rx slice 0 Register
 * RESETVAL: 0x7801 (30721)
 * ACCESS:   R/W
 * FIELDS:
 *     LMTCAL_INTV_TIME Interval time after adjustment
 *     LMTCAL_MAX_ADJ   Maximum number of adjustment
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL0r (0x00008480 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL0r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL0.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL0r_s {
	uint32_t v[1];
	uint32_t rx3_ctl0[1];
	uint32_t _rx3_ctl0;
} BCMI_VIPER_XGXS_RX3_CTL0r_t;

#define BCMI_VIPER_XGXS_RX3_CTL0r_CLR(r) (r).rx3_ctl0[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL0r_SET(r,d) (r).rx3_ctl0[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL0r_GET(r) (r).rx3_ctl0[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL0r_LMTCAL_MAX_ADJf_GET(r) ((((r).rx3_ctl0[0]) >> 11) & 0x1f)
#define BCMI_VIPER_XGXS_RX3_CTL0r_LMTCAL_MAX_ADJf_SET(r,f) (r).rx3_ctl0[0]=(((r).rx3_ctl0[0] & ~((uint32_t)0x1f << 11)) | ((((uint32_t)f) & 0x1f) << 11)) | (31 << (16 + 11))
#define BCMI_VIPER_XGXS_RX3_CTL0r_LMTCAL_INTV_TIMEf_GET(r) (((r).rx3_ctl0[0]) & 0x7ff)
#define BCMI_VIPER_XGXS_RX3_CTL0r_LMTCAL_INTV_TIMEf_SET(r,f) (r).rx3_ctl0[0]=(((r).rx3_ctl0[0] & ~((uint32_t)0x7ff)) | (((uint32_t)f) & 0x7ff)) | (0x7ff << 16)

/*
 * These macros can be used to access RX3_CTL0.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL0r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL0r,(_r._rx3_ctl0))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL0r,(_r._rx3_ctl0)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL0r,(_r._rx3_ctl0))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL0r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl0))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL0r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl0))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL0r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL0r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl0))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL0r BCMI_VIPER_XGXS_RX3_CTL0r
#define RX3_CTL0r_SIZE BCMI_VIPER_XGXS_RX3_CTL0r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL0r_t RX3_CTL0r_t;
#define RX3_CTL0r_CLR BCMI_VIPER_XGXS_RX3_CTL0r_CLR
#define RX3_CTL0r_SET BCMI_VIPER_XGXS_RX3_CTL0r_SET
#define RX3_CTL0r_GET BCMI_VIPER_XGXS_RX3_CTL0r_GET
#define RX3_CTL0r_LMTCAL_MAX_ADJf_GET BCMI_VIPER_XGXS_RX3_CTL0r_LMTCAL_MAX_ADJf_GET
#define RX3_CTL0r_LMTCAL_MAX_ADJf_SET BCMI_VIPER_XGXS_RX3_CTL0r_LMTCAL_MAX_ADJf_SET
#define RX3_CTL0r_LMTCAL_INTV_TIMEf_GET BCMI_VIPER_XGXS_RX3_CTL0r_LMTCAL_INTV_TIMEf_GET
#define RX3_CTL0r_LMTCAL_INTV_TIMEf_SET BCMI_VIPER_XGXS_RX3_CTL0r_LMTCAL_INTV_TIMEf_SET
#define READ_RX3_CTL0r BCMI_VIPER_XGXS_READ_RX3_CTL0r
#define WRITE_RX3_CTL0r BCMI_VIPER_XGXS_WRITE_RX3_CTL0r
#define MODIFY_RX3_CTL0r BCMI_VIPER_XGXS_MODIFY_RX3_CTL0r
#define READLN_RX3_CTL0r BCMI_VIPER_XGXS_READLN_RX3_CTL0r
#define WRITELN_RX3_CTL0r BCMI_VIPER_XGXS_WRITELN_RX3_CTL0r
#define WRITEALL_RX3_CTL0r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL0r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL0r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL1
 * BLOCKS:   RX3
 * REGADDR:  0x8481
 * DESC:     rx slice 1 Register
 * RESETVAL: 0xc805 (51205)
 * ACCESS:   R/W
 * FIELDS:
 *     LMTCAL_INIT_TIME Initial wait time
 *     LMTCAL_ADJ_DIR   Adjustment direction
 *     LMTCAL_KP        Accumulator Gain
 *     LMTCAL_RISING_EDGE_EN Rising edge error enable
 *     LMTCAL_FALLING_EDGE_EN Falling edge error enable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL1r (0x00008481 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL1r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL1.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL1r_s {
	uint32_t v[1];
	uint32_t rx3_ctl1[1];
	uint32_t _rx3_ctl1;
} BCMI_VIPER_XGXS_RX3_CTL1r_t;

#define BCMI_VIPER_XGXS_RX3_CTL1r_CLR(r) (r).rx3_ctl1[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL1r_SET(r,d) (r).rx3_ctl1[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL1r_GET(r) (r).rx3_ctl1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_FALLING_EDGE_ENf_GET(r) ((((r).rx3_ctl1[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_FALLING_EDGE_ENf_SET(r,f) (r).rx3_ctl1[0]=(((r).rx3_ctl1[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_RISING_EDGE_ENf_GET(r) ((((r).rx3_ctl1[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_RISING_EDGE_ENf_SET(r,f) (r).rx3_ctl1[0]=(((r).rx3_ctl1[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_KPf_GET(r) ((((r).rx3_ctl1[0]) >> 12) & 0x3)
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_KPf_SET(r,f) (r).rx3_ctl1[0]=(((r).rx3_ctl1[0] & ~((uint32_t)0x3 << 12)) | ((((uint32_t)f) & 0x3) << 12)) | (3 << (16 + 12))
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_ADJ_DIRf_GET(r) ((((r).rx3_ctl1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_ADJ_DIRf_SET(r,f) (r).rx3_ctl1[0]=(((r).rx3_ctl1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_INIT_TIMEf_GET(r) (((r).rx3_ctl1[0]) & 0x7ff)
#define BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_INIT_TIMEf_SET(r,f) (r).rx3_ctl1[0]=(((r).rx3_ctl1[0] & ~((uint32_t)0x7ff)) | (((uint32_t)f) & 0x7ff)) | (0x7ff << 16)

/*
 * These macros can be used to access RX3_CTL1.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL1r,(_r._rx3_ctl1))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL1r,(_r._rx3_ctl1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL1r,(_r._rx3_ctl1))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl1))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl1))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL1r BCMI_VIPER_XGXS_RX3_CTL1r
#define RX3_CTL1r_SIZE BCMI_VIPER_XGXS_RX3_CTL1r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL1r_t RX3_CTL1r_t;
#define RX3_CTL1r_CLR BCMI_VIPER_XGXS_RX3_CTL1r_CLR
#define RX3_CTL1r_SET BCMI_VIPER_XGXS_RX3_CTL1r_SET
#define RX3_CTL1r_GET BCMI_VIPER_XGXS_RX3_CTL1r_GET
#define RX3_CTL1r_LMTCAL_FALLING_EDGE_ENf_GET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_FALLING_EDGE_ENf_GET
#define RX3_CTL1r_LMTCAL_FALLING_EDGE_ENf_SET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_FALLING_EDGE_ENf_SET
#define RX3_CTL1r_LMTCAL_RISING_EDGE_ENf_GET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_RISING_EDGE_ENf_GET
#define RX3_CTL1r_LMTCAL_RISING_EDGE_ENf_SET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_RISING_EDGE_ENf_SET
#define RX3_CTL1r_LMTCAL_KPf_GET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_KPf_GET
#define RX3_CTL1r_LMTCAL_KPf_SET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_KPf_SET
#define RX3_CTL1r_LMTCAL_ADJ_DIRf_GET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_ADJ_DIRf_GET
#define RX3_CTL1r_LMTCAL_ADJ_DIRf_SET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_ADJ_DIRf_SET
#define RX3_CTL1r_LMTCAL_INIT_TIMEf_GET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_INIT_TIMEf_GET
#define RX3_CTL1r_LMTCAL_INIT_TIMEf_SET BCMI_VIPER_XGXS_RX3_CTL1r_LMTCAL_INIT_TIMEf_SET
#define READ_RX3_CTL1r BCMI_VIPER_XGXS_READ_RX3_CTL1r
#define WRITE_RX3_CTL1r BCMI_VIPER_XGXS_WRITE_RX3_CTL1r
#define MODIFY_RX3_CTL1r BCMI_VIPER_XGXS_MODIFY_RX3_CTL1r
#define READLN_RX3_CTL1r BCMI_VIPER_XGXS_READLN_RX3_CTL1r
#define WRITELN_RX3_CTL1r BCMI_VIPER_XGXS_WRITELN_RX3_CTL1r
#define WRITEALL_RX3_CTL1r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL2
 * BLOCKS:   RX3
 * REGADDR:  0x8482
 * DESC:     rx slice 2 Register
 * RESETVAL: 0x83f0 (33776)
 * ACCESS:   R/W
 * FIELDS:
 *     LMTCAL_DONE_OVRD_VAL lmtcal done override value
 *     LMTCAL_DONE_OVRD lmtcal done override
 *     LMTCAL_EN_OVRD_VAL lmtcal enable override value
 *     LMTCAL_EN_OVRD   lmtcal enable override
 *     LMTCAL_ACC_TIME  Accumulation time
 *     LMTCAL_PD_POLARITY Change the polarity of the sum
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL2r (0x00008482 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL2r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL2.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL2r_s {
	uint32_t v[1];
	uint32_t rx3_ctl2[1];
	uint32_t _rx3_ctl2;
} BCMI_VIPER_XGXS_RX3_CTL2r_t;

#define BCMI_VIPER_XGXS_RX3_CTL2r_CLR(r) (r).rx3_ctl2[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL2r_SET(r,d) (r).rx3_ctl2[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL2r_GET(r) (r).rx3_ctl2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_PD_POLARITYf_GET(r) ((((r).rx3_ctl2[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_PD_POLARITYf_SET(r,f) (r).rx3_ctl2[0]=(((r).rx3_ctl2[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_ACC_TIMEf_GET(r) ((((r).rx3_ctl2[0]) >> 4) & 0x7ff)
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_ACC_TIMEf_SET(r,f) (r).rx3_ctl2[0]=(((r).rx3_ctl2[0] & ~((uint32_t)0x7ff << 4)) | ((((uint32_t)f) & 0x7ff) << 4)) | (2047 << (16 + 4))
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_EN_OVRDf_GET(r) ((((r).rx3_ctl2[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_EN_OVRDf_SET(r,f) (r).rx3_ctl2[0]=(((r).rx3_ctl2[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_EN_OVRD_VALf_GET(r) ((((r).rx3_ctl2[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_EN_OVRD_VALf_SET(r,f) (r).rx3_ctl2[0]=(((r).rx3_ctl2[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_DONE_OVRDf_GET(r) ((((r).rx3_ctl2[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_DONE_OVRDf_SET(r,f) (r).rx3_ctl2[0]=(((r).rx3_ctl2[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_DONE_OVRD_VALf_GET(r) (((r).rx3_ctl2[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_DONE_OVRD_VALf_SET(r,f) (r).rx3_ctl2[0]=(((r).rx3_ctl2[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX3_CTL2.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL2r,(_r._rx3_ctl2))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL2r,(_r._rx3_ctl2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL2r,(_r._rx3_ctl2))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl2))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl2))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL2r BCMI_VIPER_XGXS_RX3_CTL2r
#define RX3_CTL2r_SIZE BCMI_VIPER_XGXS_RX3_CTL2r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL2r_t RX3_CTL2r_t;
#define RX3_CTL2r_CLR BCMI_VIPER_XGXS_RX3_CTL2r_CLR
#define RX3_CTL2r_SET BCMI_VIPER_XGXS_RX3_CTL2r_SET
#define RX3_CTL2r_GET BCMI_VIPER_XGXS_RX3_CTL2r_GET
#define RX3_CTL2r_LMTCAL_PD_POLARITYf_GET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_PD_POLARITYf_GET
#define RX3_CTL2r_LMTCAL_PD_POLARITYf_SET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_PD_POLARITYf_SET
#define RX3_CTL2r_LMTCAL_ACC_TIMEf_GET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_ACC_TIMEf_GET
#define RX3_CTL2r_LMTCAL_ACC_TIMEf_SET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_ACC_TIMEf_SET
#define RX3_CTL2r_LMTCAL_EN_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_EN_OVRDf_GET
#define RX3_CTL2r_LMTCAL_EN_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_EN_OVRDf_SET
#define RX3_CTL2r_LMTCAL_EN_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_EN_OVRD_VALf_GET
#define RX3_CTL2r_LMTCAL_EN_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_EN_OVRD_VALf_SET
#define RX3_CTL2r_LMTCAL_DONE_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_DONE_OVRDf_GET
#define RX3_CTL2r_LMTCAL_DONE_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_DONE_OVRDf_SET
#define RX3_CTL2r_LMTCAL_DONE_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_DONE_OVRD_VALf_GET
#define RX3_CTL2r_LMTCAL_DONE_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL2r_LMTCAL_DONE_OVRD_VALf_SET
#define READ_RX3_CTL2r BCMI_VIPER_XGXS_READ_RX3_CTL2r
#define WRITE_RX3_CTL2r BCMI_VIPER_XGXS_WRITE_RX3_CTL2r
#define MODIFY_RX3_CTL2r BCMI_VIPER_XGXS_MODIFY_RX3_CTL2r
#define READLN_RX3_CTL2r BCMI_VIPER_XGXS_READLN_RX3_CTL2r
#define WRITELN_RX3_CTL2r BCMI_VIPER_XGXS_WRITELN_RX3_CTL2r
#define WRITEALL_RX3_CTL2r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL3
 * BLOCKS:   RX3
 * REGADDR:  0x8483
 * DESC:     rx slice 3 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     RECAL_POS_THRES  re-calibration upper threshold
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL3r (0x00008483 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL3r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL3.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL3r_s {
	uint32_t v[1];
	uint32_t rx3_ctl3[1];
	uint32_t _rx3_ctl3;
} BCMI_VIPER_XGXS_RX3_CTL3r_t;

#define BCMI_VIPER_XGXS_RX3_CTL3r_CLR(r) (r).rx3_ctl3[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL3r_SET(r,d) (r).rx3_ctl3[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL3r_GET(r) (r).rx3_ctl3[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL3r_RECAL_POS_THRESf_GET(r) (((r).rx3_ctl3[0]) & 0xffff)
#define BCMI_VIPER_XGXS_RX3_CTL3r_RECAL_POS_THRESf_SET(r,f) (r).rx3_ctl3[0]=(((r).rx3_ctl3[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access RX3_CTL3.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL3r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL3r,(_r._rx3_ctl3))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL3r,(_r._rx3_ctl3)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL3r,(_r._rx3_ctl3))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL3r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl3))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL3r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl3))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL3r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL3r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl3))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL3r BCMI_VIPER_XGXS_RX3_CTL3r
#define RX3_CTL3r_SIZE BCMI_VIPER_XGXS_RX3_CTL3r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL3r_t RX3_CTL3r_t;
#define RX3_CTL3r_CLR BCMI_VIPER_XGXS_RX3_CTL3r_CLR
#define RX3_CTL3r_SET BCMI_VIPER_XGXS_RX3_CTL3r_SET
#define RX3_CTL3r_GET BCMI_VIPER_XGXS_RX3_CTL3r_GET
#define RX3_CTL3r_RECAL_POS_THRESf_GET BCMI_VIPER_XGXS_RX3_CTL3r_RECAL_POS_THRESf_GET
#define RX3_CTL3r_RECAL_POS_THRESf_SET BCMI_VIPER_XGXS_RX3_CTL3r_RECAL_POS_THRESf_SET
#define READ_RX3_CTL3r BCMI_VIPER_XGXS_READ_RX3_CTL3r
#define WRITE_RX3_CTL3r BCMI_VIPER_XGXS_WRITE_RX3_CTL3r
#define MODIFY_RX3_CTL3r BCMI_VIPER_XGXS_MODIFY_RX3_CTL3r
#define READLN_RX3_CTL3r BCMI_VIPER_XGXS_READLN_RX3_CTL3r
#define WRITELN_RX3_CTL3r BCMI_VIPER_XGXS_WRITELN_RX3_CTL3r
#define WRITEALL_RX3_CTL3r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL3r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL3r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL4
 * BLOCKS:   RX3
 * REGADDR:  0x8484
 * DESC:     rx slice 4 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     RECAL_NEG_THRES  re-calibration lower threshold
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL4r (0x00008484 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL4r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL4.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL4r_s {
	uint32_t v[1];
	uint32_t rx3_ctl4[1];
	uint32_t _rx3_ctl4;
} BCMI_VIPER_XGXS_RX3_CTL4r_t;

#define BCMI_VIPER_XGXS_RX3_CTL4r_CLR(r) (r).rx3_ctl4[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL4r_SET(r,d) (r).rx3_ctl4[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL4r_GET(r) (r).rx3_ctl4[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL4r_RECAL_NEG_THRESf_GET(r) (((r).rx3_ctl4[0]) & 0xffff)
#define BCMI_VIPER_XGXS_RX3_CTL4r_RECAL_NEG_THRESf_SET(r,f) (r).rx3_ctl4[0]=(((r).rx3_ctl4[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access RX3_CTL4.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL4r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL4r,(_r._rx3_ctl4))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL4r,(_r._rx3_ctl4)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL4r,(_r._rx3_ctl4))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL4r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl4))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL4r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl4))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL4r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL4r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl4))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL4r BCMI_VIPER_XGXS_RX3_CTL4r
#define RX3_CTL4r_SIZE BCMI_VIPER_XGXS_RX3_CTL4r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL4r_t RX3_CTL4r_t;
#define RX3_CTL4r_CLR BCMI_VIPER_XGXS_RX3_CTL4r_CLR
#define RX3_CTL4r_SET BCMI_VIPER_XGXS_RX3_CTL4r_SET
#define RX3_CTL4r_GET BCMI_VIPER_XGXS_RX3_CTL4r_GET
#define RX3_CTL4r_RECAL_NEG_THRESf_GET BCMI_VIPER_XGXS_RX3_CTL4r_RECAL_NEG_THRESf_GET
#define RX3_CTL4r_RECAL_NEG_THRESf_SET BCMI_VIPER_XGXS_RX3_CTL4r_RECAL_NEG_THRESf_SET
#define READ_RX3_CTL4r BCMI_VIPER_XGXS_READ_RX3_CTL4r
#define WRITE_RX3_CTL4r BCMI_VIPER_XGXS_WRITE_RX3_CTL4r
#define MODIFY_RX3_CTL4r BCMI_VIPER_XGXS_MODIFY_RX3_CTL4r
#define READLN_RX3_CTL4r BCMI_VIPER_XGXS_READLN_RX3_CTL4r
#define WRITELN_RX3_CTL4r BCMI_VIPER_XGXS_WRITELN_RX3_CTL4r
#define WRITEALL_RX3_CTL4r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL4r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL4r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL5
 * BLOCKS:   RX3
 * REGADDR:  0x8485
 * DESC:     rx slice 5 Register
 * RESETVAL: 0x7e (126)
 * ACCESS:   R/W
 * FIELDS:
 *     CONT_LMTCAL_EN   cont_lmtcal enable
 *     LMTCAL_CONT_ACC_TIME Accumulation time in the LMT_DONE state
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL5r (0x00008485 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL5r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL5.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL5r_s {
	uint32_t v[1];
	uint32_t rx3_ctl5[1];
	uint32_t _rx3_ctl5;
} BCMI_VIPER_XGXS_RX3_CTL5r_t;

#define BCMI_VIPER_XGXS_RX3_CTL5r_CLR(r) (r).rx3_ctl5[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL5r_SET(r,d) (r).rx3_ctl5[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL5r_GET(r) (r).rx3_ctl5[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL5r_LMTCAL_CONT_ACC_TIMEf_GET(r) ((((r).rx3_ctl5[0]) >> 1) & 0x7ff)
#define BCMI_VIPER_XGXS_RX3_CTL5r_LMTCAL_CONT_ACC_TIMEf_SET(r,f) (r).rx3_ctl5[0]=(((r).rx3_ctl5[0] & ~((uint32_t)0x7ff << 1)) | ((((uint32_t)f) & 0x7ff) << 1)) | (2047 << (16 + 1))
#define BCMI_VIPER_XGXS_RX3_CTL5r_CONT_LMTCAL_ENf_GET(r) (((r).rx3_ctl5[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL5r_CONT_LMTCAL_ENf_SET(r,f) (r).rx3_ctl5[0]=(((r).rx3_ctl5[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX3_CTL5.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL5r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL5r,(_r._rx3_ctl5))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL5r,(_r._rx3_ctl5)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL5r,(_r._rx3_ctl5))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL5r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl5))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL5r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl5))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL5r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL5r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl5))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL5r BCMI_VIPER_XGXS_RX3_CTL5r
#define RX3_CTL5r_SIZE BCMI_VIPER_XGXS_RX3_CTL5r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL5r_t RX3_CTL5r_t;
#define RX3_CTL5r_CLR BCMI_VIPER_XGXS_RX3_CTL5r_CLR
#define RX3_CTL5r_SET BCMI_VIPER_XGXS_RX3_CTL5r_SET
#define RX3_CTL5r_GET BCMI_VIPER_XGXS_RX3_CTL5r_GET
#define RX3_CTL5r_LMTCAL_CONT_ACC_TIMEf_GET BCMI_VIPER_XGXS_RX3_CTL5r_LMTCAL_CONT_ACC_TIMEf_GET
#define RX3_CTL5r_LMTCAL_CONT_ACC_TIMEf_SET BCMI_VIPER_XGXS_RX3_CTL5r_LMTCAL_CONT_ACC_TIMEf_SET
#define RX3_CTL5r_CONT_LMTCAL_ENf_GET BCMI_VIPER_XGXS_RX3_CTL5r_CONT_LMTCAL_ENf_GET
#define RX3_CTL5r_CONT_LMTCAL_ENf_SET BCMI_VIPER_XGXS_RX3_CTL5r_CONT_LMTCAL_ENf_SET
#define READ_RX3_CTL5r BCMI_VIPER_XGXS_READ_RX3_CTL5r
#define WRITE_RX3_CTL5r BCMI_VIPER_XGXS_WRITE_RX3_CTL5r
#define MODIFY_RX3_CTL5r BCMI_VIPER_XGXS_MODIFY_RX3_CTL5r
#define READLN_RX3_CTL5r BCMI_VIPER_XGXS_READLN_RX3_CTL5r
#define WRITELN_RX3_CTL5r BCMI_VIPER_XGXS_WRITELN_RX3_CTL5r
#define WRITEALL_RX3_CTL5r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL5r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL5r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL6
 * BLOCKS:   RX3
 * REGADDR:  0x8486
 * DESC:     rx slice 6 Register
 * RESETVAL: 0x280 (640)
 * ACCESS:   R/W
 * FIELDS:
 *     RX_LMTOFF_OVRD_VAL LA calibration control override value
 *     RX_LMTOFF_OVRD   LA calibration control override
 *     LMTCAL_INTERVAL_TIME Time between CDR lock and lmt_cal enable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL6r (0x00008486 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL6r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL6.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL6r_s {
	uint32_t v[1];
	uint32_t rx3_ctl6[1];
	uint32_t _rx3_ctl6;
} BCMI_VIPER_XGXS_RX3_CTL6r_t;

#define BCMI_VIPER_XGXS_RX3_CTL6r_CLR(r) (r).rx3_ctl6[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL6r_SET(r,d) (r).rx3_ctl6[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL6r_GET(r) (r).rx3_ctl6[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL6r_LMTCAL_INTERVAL_TIMEf_GET(r) ((((r).rx3_ctl6[0]) >> 7) & 0x1f)
#define BCMI_VIPER_XGXS_RX3_CTL6r_LMTCAL_INTERVAL_TIMEf_SET(r,f) (r).rx3_ctl6[0]=(((r).rx3_ctl6[0] & ~((uint32_t)0x1f << 7)) | ((((uint32_t)f) & 0x1f) << 7)) | (31 << (16 + 7))
#define BCMI_VIPER_XGXS_RX3_CTL6r_RX_LMTOFF_OVRDf_GET(r) ((((r).rx3_ctl6[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL6r_RX_LMTOFF_OVRDf_SET(r,f) (r).rx3_ctl6[0]=(((r).rx3_ctl6[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_RX3_CTL6r_RX_LMTOFF_OVRD_VALf_GET(r) (((r).rx3_ctl6[0]) & 0x3f)
#define BCMI_VIPER_XGXS_RX3_CTL6r_RX_LMTOFF_OVRD_VALf_SET(r,f) (r).rx3_ctl6[0]=(((r).rx3_ctl6[0] & ~((uint32_t)0x3f)) | (((uint32_t)f) & 0x3f)) | (0x3f << 16)

/*
 * These macros can be used to access RX3_CTL6.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL6r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL6r,(_r._rx3_ctl6))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL6r,(_r._rx3_ctl6)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL6r,(_r._rx3_ctl6))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL6r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl6))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL6r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl6))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL6r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL6r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl6))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL6r BCMI_VIPER_XGXS_RX3_CTL6r
#define RX3_CTL6r_SIZE BCMI_VIPER_XGXS_RX3_CTL6r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL6r_t RX3_CTL6r_t;
#define RX3_CTL6r_CLR BCMI_VIPER_XGXS_RX3_CTL6r_CLR
#define RX3_CTL6r_SET BCMI_VIPER_XGXS_RX3_CTL6r_SET
#define RX3_CTL6r_GET BCMI_VIPER_XGXS_RX3_CTL6r_GET
#define RX3_CTL6r_LMTCAL_INTERVAL_TIMEf_GET BCMI_VIPER_XGXS_RX3_CTL6r_LMTCAL_INTERVAL_TIMEf_GET
#define RX3_CTL6r_LMTCAL_INTERVAL_TIMEf_SET BCMI_VIPER_XGXS_RX3_CTL6r_LMTCAL_INTERVAL_TIMEf_SET
#define RX3_CTL6r_RX_LMTOFF_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL6r_RX_LMTOFF_OVRDf_GET
#define RX3_CTL6r_RX_LMTOFF_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL6r_RX_LMTOFF_OVRDf_SET
#define RX3_CTL6r_RX_LMTOFF_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL6r_RX_LMTOFF_OVRD_VALf_GET
#define RX3_CTL6r_RX_LMTOFF_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL6r_RX_LMTOFF_OVRD_VALf_SET
#define READ_RX3_CTL6r BCMI_VIPER_XGXS_READ_RX3_CTL6r
#define WRITE_RX3_CTL6r BCMI_VIPER_XGXS_WRITE_RX3_CTL6r
#define MODIFY_RX3_CTL6r BCMI_VIPER_XGXS_MODIFY_RX3_CTL6r
#define READLN_RX3_CTL6r BCMI_VIPER_XGXS_READLN_RX3_CTL6r
#define WRITELN_RX3_CTL6r BCMI_VIPER_XGXS_WRITELN_RX3_CTL6r
#define WRITEALL_RX3_CTL6r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL6r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL6r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL7
 * BLOCKS:   RX3
 * REGADDR:  0x8487
 * DESC:     rx slice 7 Register
 * RESETVAL: 0x1000 (4096)
 * ACCESS:   R/W
 * FIELDS:
 *     SLCAL_VALID_OVRD_VAL slcal valid override value
 *     SLCAL_VALID_OVRD slcal valid override
 *     SLCAL_POL        Data polarity
 *     SLCAL_ACC_OPT    Accumulator time
 *     SLCAL_EN_OVRD_VAL slcal_en override value
 *     SLCAL_EN_OVRD    slcal_en override
 *     CAL_STATE_OVRD   cal_ctrl state machine override
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL7r (0x00008487 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL7r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL7.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL7r_s {
	uint32_t v[1];
	uint32_t rx3_ctl7[1];
	uint32_t _rx3_ctl7;
} BCMI_VIPER_XGXS_RX3_CTL7r_t;

#define BCMI_VIPER_XGXS_RX3_CTL7r_CLR(r) (r).rx3_ctl7[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL7r_SET(r,d) (r).rx3_ctl7[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL7r_GET(r) (r).rx3_ctl7[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL7r_CAL_STATE_OVRDf_GET(r) ((((r).rx3_ctl7[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL7r_CAL_STATE_OVRDf_SET(r,f) (r).rx3_ctl7[0]=(((r).rx3_ctl7[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_EN_OVRDf_GET(r) ((((r).rx3_ctl7[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_EN_OVRDf_SET(r,f) (r).rx3_ctl7[0]=(((r).rx3_ctl7[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_EN_OVRD_VALf_GET(r) ((((r).rx3_ctl7[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_EN_OVRD_VALf_SET(r,f) (r).rx3_ctl7[0]=(((r).rx3_ctl7[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_ACC_OPTf_GET(r) ((((r).rx3_ctl7[0]) >> 11) & 0x3)
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_ACC_OPTf_SET(r,f) (r).rx3_ctl7[0]=(((r).rx3_ctl7[0] & ~((uint32_t)0x3 << 11)) | ((((uint32_t)f) & 0x3) << 11)) | (3 << (16 + 11))
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_POLf_GET(r) ((((r).rx3_ctl7[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_POLf_SET(r,f) (r).rx3_ctl7[0]=(((r).rx3_ctl7[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_VALID_OVRDf_GET(r) ((((r).rx3_ctl7[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_VALID_OVRDf_SET(r,f) (r).rx3_ctl7[0]=(((r).rx3_ctl7[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_VALID_OVRD_VALf_GET(r) (((r).rx3_ctl7[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_VALID_OVRD_VALf_SET(r,f) (r).rx3_ctl7[0]=(((r).rx3_ctl7[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX3_CTL7.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL7r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL7r,(_r._rx3_ctl7))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL7r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL7r,(_r._rx3_ctl7)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL7r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL7r,(_r._rx3_ctl7))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL7r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL7r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl7))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL7r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL7r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl7))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL7r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL7r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl7))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL7r BCMI_VIPER_XGXS_RX3_CTL7r
#define RX3_CTL7r_SIZE BCMI_VIPER_XGXS_RX3_CTL7r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL7r_t RX3_CTL7r_t;
#define RX3_CTL7r_CLR BCMI_VIPER_XGXS_RX3_CTL7r_CLR
#define RX3_CTL7r_SET BCMI_VIPER_XGXS_RX3_CTL7r_SET
#define RX3_CTL7r_GET BCMI_VIPER_XGXS_RX3_CTL7r_GET
#define RX3_CTL7r_CAL_STATE_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL7r_CAL_STATE_OVRDf_GET
#define RX3_CTL7r_CAL_STATE_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL7r_CAL_STATE_OVRDf_SET
#define RX3_CTL7r_SLCAL_EN_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_EN_OVRDf_GET
#define RX3_CTL7r_SLCAL_EN_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_EN_OVRDf_SET
#define RX3_CTL7r_SLCAL_EN_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_EN_OVRD_VALf_GET
#define RX3_CTL7r_SLCAL_EN_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_EN_OVRD_VALf_SET
#define RX3_CTL7r_SLCAL_ACC_OPTf_GET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_ACC_OPTf_GET
#define RX3_CTL7r_SLCAL_ACC_OPTf_SET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_ACC_OPTf_SET
#define RX3_CTL7r_SLCAL_POLf_GET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_POLf_GET
#define RX3_CTL7r_SLCAL_POLf_SET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_POLf_SET
#define RX3_CTL7r_SLCAL_VALID_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_VALID_OVRDf_GET
#define RX3_CTL7r_SLCAL_VALID_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_VALID_OVRDf_SET
#define RX3_CTL7r_SLCAL_VALID_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_VALID_OVRD_VALf_GET
#define RX3_CTL7r_SLCAL_VALID_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL7r_SLCAL_VALID_OVRD_VALf_SET
#define READ_RX3_CTL7r BCMI_VIPER_XGXS_READ_RX3_CTL7r
#define WRITE_RX3_CTL7r BCMI_VIPER_XGXS_WRITE_RX3_CTL7r
#define MODIFY_RX3_CTL7r BCMI_VIPER_XGXS_MODIFY_RX3_CTL7r
#define READLN_RX3_CTL7r BCMI_VIPER_XGXS_READLN_RX3_CTL7r
#define WRITELN_RX3_CTL7r BCMI_VIPER_XGXS_WRITELN_RX3_CTL7r
#define WRITEALL_RX3_CTL7r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL7r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL7r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL8
 * BLOCKS:   RX3
 * REGADDR:  0x8488
 * DESC:     rx slice 8 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     RX_SLOFF0_OVRD_VAL Peak slicer control override value
 *     RX_SLOFF0_OVRD   Peak slicer control override
 *     RX_SLOFF1_OVRD_VAL Zero slicer control override value
 *     RX_SLOFF1_OVRD   Zero slicer control override
 *     RX_SLOFF2_OVRD_VAL EM slicer control override value
 *     RX_SLOFF2_OVRD   EM slicer control override
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL8r (0x00008488 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL8r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL8.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL8r_s {
	uint32_t v[1];
	uint32_t rx3_ctl8[1];
	uint32_t _rx3_ctl8;
} BCMI_VIPER_XGXS_RX3_CTL8r_t;

#define BCMI_VIPER_XGXS_RX3_CTL8r_CLR(r) (r).rx3_ctl8[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL8r_SET(r,d) (r).rx3_ctl8[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL8r_GET(r) (r).rx3_ctl8[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF2_OVRDf_GET(r) ((((r).rx3_ctl8[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF2_OVRDf_SET(r,f) (r).rx3_ctl8[0]=(((r).rx3_ctl8[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF2_OVRD_VALf_GET(r) ((((r).rx3_ctl8[0]) >> 10) & 0xf)
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF2_OVRD_VALf_SET(r,f) (r).rx3_ctl8[0]=(((r).rx3_ctl8[0] & ~((uint32_t)0xf << 10)) | ((((uint32_t)f) & 0xf) << 10)) | (15 << (16 + 10))
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF1_OVRDf_GET(r) ((((r).rx3_ctl8[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF1_OVRDf_SET(r,f) (r).rx3_ctl8[0]=(((r).rx3_ctl8[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF1_OVRD_VALf_GET(r) ((((r).rx3_ctl8[0]) >> 5) & 0xf)
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF1_OVRD_VALf_SET(r,f) (r).rx3_ctl8[0]=(((r).rx3_ctl8[0] & ~((uint32_t)0xf << 5)) | ((((uint32_t)f) & 0xf) << 5)) | (15 << (16 + 5))
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF0_OVRDf_GET(r) ((((r).rx3_ctl8[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF0_OVRDf_SET(r,f) (r).rx3_ctl8[0]=(((r).rx3_ctl8[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF0_OVRD_VALf_GET(r) (((r).rx3_ctl8[0]) & 0xf)
#define BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF0_OVRD_VALf_SET(r,f) (r).rx3_ctl8[0]=(((r).rx3_ctl8[0] & ~((uint32_t)0xf)) | (((uint32_t)f) & 0xf)) | (0xf << 16)

/*
 * These macros can be used to access RX3_CTL8.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL8r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL8r,(_r._rx3_ctl8))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL8r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL8r,(_r._rx3_ctl8)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL8r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL8r,(_r._rx3_ctl8))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL8r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL8r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl8))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL8r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL8r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl8))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL8r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL8r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl8))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL8r BCMI_VIPER_XGXS_RX3_CTL8r
#define RX3_CTL8r_SIZE BCMI_VIPER_XGXS_RX3_CTL8r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL8r_t RX3_CTL8r_t;
#define RX3_CTL8r_CLR BCMI_VIPER_XGXS_RX3_CTL8r_CLR
#define RX3_CTL8r_SET BCMI_VIPER_XGXS_RX3_CTL8r_SET
#define RX3_CTL8r_GET BCMI_VIPER_XGXS_RX3_CTL8r_GET
#define RX3_CTL8r_RX_SLOFF2_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF2_OVRDf_GET
#define RX3_CTL8r_RX_SLOFF2_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF2_OVRDf_SET
#define RX3_CTL8r_RX_SLOFF2_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF2_OVRD_VALf_GET
#define RX3_CTL8r_RX_SLOFF2_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF2_OVRD_VALf_SET
#define RX3_CTL8r_RX_SLOFF1_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF1_OVRDf_GET
#define RX3_CTL8r_RX_SLOFF1_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF1_OVRDf_SET
#define RX3_CTL8r_RX_SLOFF1_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF1_OVRD_VALf_GET
#define RX3_CTL8r_RX_SLOFF1_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF1_OVRD_VALf_SET
#define RX3_CTL8r_RX_SLOFF0_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF0_OVRDf_GET
#define RX3_CTL8r_RX_SLOFF0_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF0_OVRDf_SET
#define RX3_CTL8r_RX_SLOFF0_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF0_OVRD_VALf_GET
#define RX3_CTL8r_RX_SLOFF0_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL8r_RX_SLOFF0_OVRD_VALf_SET
#define READ_RX3_CTL8r BCMI_VIPER_XGXS_READ_RX3_CTL8r
#define WRITE_RX3_CTL8r BCMI_VIPER_XGXS_WRITE_RX3_CTL8r
#define MODIFY_RX3_CTL8r BCMI_VIPER_XGXS_MODIFY_RX3_CTL8r
#define READLN_RX3_CTL8r BCMI_VIPER_XGXS_READLN_RX3_CTL8r
#define WRITELN_RX3_CTL8r BCMI_VIPER_XGXS_WRITELN_RX3_CTL8r
#define WRITEALL_RX3_CTL8r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL8r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL8r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL9
 * BLOCKS:   RX3
 * REGADDR:  0x8489
 * DESC:     rx slice 9 Register
 * RESETVAL: 0x280 (640)
 * ACCESS:   R/W
 * FIELDS:
 *     SLCAL_UP_THRES   slcal upper threshold
 *     RX_SLICER_CALVALID_OVRD_VAL rx_slicer_calvalid override value
 *     RX_SLICER_CALVALID_OVRD rx_slicer_calvalid override
 *     CAL_STATE_OVRD_VAL cal_ctrl state machine override value
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL9r (0x00008489 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL9r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL9.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL9r_s {
	uint32_t v[1];
	uint32_t rx3_ctl9[1];
	uint32_t _rx3_ctl9;
} BCMI_VIPER_XGXS_RX3_CTL9r_t;

#define BCMI_VIPER_XGXS_RX3_CTL9r_CLR(r) (r).rx3_ctl9[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL9r_SET(r,d) (r).rx3_ctl9[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL9r_GET(r) (r).rx3_ctl9[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL9r_CAL_STATE_OVRD_VALf_GET(r) ((((r).rx3_ctl9[0]) >> 13) & 0x7)
#define BCMI_VIPER_XGXS_RX3_CTL9r_CAL_STATE_OVRD_VALf_SET(r,f) (r).rx3_ctl9[0]=(((r).rx3_ctl9[0] & ~((uint32_t)0x7 << 13)) | ((((uint32_t)f) & 0x7) << 13)) | (7 << (16 + 13))
#define BCMI_VIPER_XGXS_RX3_CTL9r_RX_SLICER_CALVALID_OVRDf_GET(r) ((((r).rx3_ctl9[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL9r_RX_SLICER_CALVALID_OVRDf_SET(r,f) (r).rx3_ctl9[0]=(((r).rx3_ctl9[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_RX3_CTL9r_RX_SLICER_CALVALID_OVRD_VALf_GET(r) ((((r).rx3_ctl9[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL9r_RX_SLICER_CALVALID_OVRD_VALf_SET(r,f) (r).rx3_ctl9[0]=(((r).rx3_ctl9[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_RX3_CTL9r_SLCAL_UP_THRESf_GET(r) (((r).rx3_ctl9[0]) & 0x7ff)
#define BCMI_VIPER_XGXS_RX3_CTL9r_SLCAL_UP_THRESf_SET(r,f) (r).rx3_ctl9[0]=(((r).rx3_ctl9[0] & ~((uint32_t)0x7ff)) | (((uint32_t)f) & 0x7ff)) | (0x7ff << 16)

/*
 * These macros can be used to access RX3_CTL9.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL9r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL9r,(_r._rx3_ctl9))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL9r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL9r,(_r._rx3_ctl9)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL9r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL9r,(_r._rx3_ctl9))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL9r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL9r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl9))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL9r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL9r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl9))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL9r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL9r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl9))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL9r BCMI_VIPER_XGXS_RX3_CTL9r
#define RX3_CTL9r_SIZE BCMI_VIPER_XGXS_RX3_CTL9r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL9r_t RX3_CTL9r_t;
#define RX3_CTL9r_CLR BCMI_VIPER_XGXS_RX3_CTL9r_CLR
#define RX3_CTL9r_SET BCMI_VIPER_XGXS_RX3_CTL9r_SET
#define RX3_CTL9r_GET BCMI_VIPER_XGXS_RX3_CTL9r_GET
#define RX3_CTL9r_CAL_STATE_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL9r_CAL_STATE_OVRD_VALf_GET
#define RX3_CTL9r_CAL_STATE_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL9r_CAL_STATE_OVRD_VALf_SET
#define RX3_CTL9r_RX_SLICER_CALVALID_OVRDf_GET BCMI_VIPER_XGXS_RX3_CTL9r_RX_SLICER_CALVALID_OVRDf_GET
#define RX3_CTL9r_RX_SLICER_CALVALID_OVRDf_SET BCMI_VIPER_XGXS_RX3_CTL9r_RX_SLICER_CALVALID_OVRDf_SET
#define RX3_CTL9r_RX_SLICER_CALVALID_OVRD_VALf_GET BCMI_VIPER_XGXS_RX3_CTL9r_RX_SLICER_CALVALID_OVRD_VALf_GET
#define RX3_CTL9r_RX_SLICER_CALVALID_OVRD_VALf_SET BCMI_VIPER_XGXS_RX3_CTL9r_RX_SLICER_CALVALID_OVRD_VALf_SET
#define RX3_CTL9r_SLCAL_UP_THRESf_GET BCMI_VIPER_XGXS_RX3_CTL9r_SLCAL_UP_THRESf_GET
#define RX3_CTL9r_SLCAL_UP_THRESf_SET BCMI_VIPER_XGXS_RX3_CTL9r_SLCAL_UP_THRESf_SET
#define READ_RX3_CTL9r BCMI_VIPER_XGXS_READ_RX3_CTL9r
#define WRITE_RX3_CTL9r BCMI_VIPER_XGXS_WRITE_RX3_CTL9r
#define MODIFY_RX3_CTL9r BCMI_VIPER_XGXS_MODIFY_RX3_CTL9r
#define READLN_RX3_CTL9r BCMI_VIPER_XGXS_READLN_RX3_CTL9r
#define WRITELN_RX3_CTL9r BCMI_VIPER_XGXS_WRITELN_RX3_CTL9r
#define WRITEALL_RX3_CTL9r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL9r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL9r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL10
 * BLOCKS:   RX3
 * REGADDR:  0x848a
 * DESC:     rx slice 10 Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     SLCAL_DN_THRES   slcal lower threshold
 *     SLICER_INTERVAL_TIME slicer interval time
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL10r (0x0000848a | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL10r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL10.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL10r_s {
	uint32_t v[1];
	uint32_t rx3_ctl10[1];
	uint32_t _rx3_ctl10;
} BCMI_VIPER_XGXS_RX3_CTL10r_t;

#define BCMI_VIPER_XGXS_RX3_CTL10r_CLR(r) (r).rx3_ctl10[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL10r_SET(r,d) (r).rx3_ctl10[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL10r_GET(r) (r).rx3_ctl10[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL10r_SLICER_INTERVAL_TIMEf_GET(r) ((((r).rx3_ctl10[0]) >> 11) & 0x1f)
#define BCMI_VIPER_XGXS_RX3_CTL10r_SLICER_INTERVAL_TIMEf_SET(r,f) (r).rx3_ctl10[0]=(((r).rx3_ctl10[0] & ~((uint32_t)0x1f << 11)) | ((((uint32_t)f) & 0x1f) << 11)) | (31 << (16 + 11))
#define BCMI_VIPER_XGXS_RX3_CTL10r_SLCAL_DN_THRESf_GET(r) (((r).rx3_ctl10[0]) & 0x7ff)
#define BCMI_VIPER_XGXS_RX3_CTL10r_SLCAL_DN_THRESf_SET(r,f) (r).rx3_ctl10[0]=(((r).rx3_ctl10[0] & ~((uint32_t)0x7ff)) | (((uint32_t)f) & 0x7ff)) | (0x7ff << 16)

/*
 * These macros can be used to access RX3_CTL10.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL10r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL10r,(_r._rx3_ctl10))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL10r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL10r,(_r._rx3_ctl10)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL10r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL10r,(_r._rx3_ctl10))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL10r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL10r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl10))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL10r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL10r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl10))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL10r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL10r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl10))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL10r BCMI_VIPER_XGXS_RX3_CTL10r
#define RX3_CTL10r_SIZE BCMI_VIPER_XGXS_RX3_CTL10r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL10r_t RX3_CTL10r_t;
#define RX3_CTL10r_CLR BCMI_VIPER_XGXS_RX3_CTL10r_CLR
#define RX3_CTL10r_SET BCMI_VIPER_XGXS_RX3_CTL10r_SET
#define RX3_CTL10r_GET BCMI_VIPER_XGXS_RX3_CTL10r_GET
#define RX3_CTL10r_SLICER_INTERVAL_TIMEf_GET BCMI_VIPER_XGXS_RX3_CTL10r_SLICER_INTERVAL_TIMEf_GET
#define RX3_CTL10r_SLICER_INTERVAL_TIMEf_SET BCMI_VIPER_XGXS_RX3_CTL10r_SLICER_INTERVAL_TIMEf_SET
#define RX3_CTL10r_SLCAL_DN_THRESf_GET BCMI_VIPER_XGXS_RX3_CTL10r_SLCAL_DN_THRESf_GET
#define RX3_CTL10r_SLCAL_DN_THRESf_SET BCMI_VIPER_XGXS_RX3_CTL10r_SLCAL_DN_THRESf_SET
#define READ_RX3_CTL10r BCMI_VIPER_XGXS_READ_RX3_CTL10r
#define WRITE_RX3_CTL10r BCMI_VIPER_XGXS_WRITE_RX3_CTL10r
#define MODIFY_RX3_CTL10r BCMI_VIPER_XGXS_MODIFY_RX3_CTL10r
#define READLN_RX3_CTL10r BCMI_VIPER_XGXS_READLN_RX3_CTL10r
#define WRITELN_RX3_CTL10r BCMI_VIPER_XGXS_WRITELN_RX3_CTL10r
#define WRITEALL_RX3_CTL10r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL10r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL10r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  RX3_CTL11
 * BLOCKS:   RX3
 * REGADDR:  0x848b
 * DESC:     rx slice 11 Register
 * RESETVAL: 0x1d80 (7552)
 * ACCESS:   R/W
 * FIELDS:
 *     PM_RXLIMITAMPCALBYP 
 *     PM_RXSLICERCALBYP 
 *     SLCAL_INIT_TIME  
 *     RECAL_IND_CLR    
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_RX3_CTL11r (0x0000848b | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_RX3_CTL11r_SIZE 4

/*
 * This structure should be used to declare and program RX3_CTL11.
 *
 */
typedef union BCMI_VIPER_XGXS_RX3_CTL11r_s {
	uint32_t v[1];
	uint32_t rx3_ctl11[1];
	uint32_t _rx3_ctl11;
} BCMI_VIPER_XGXS_RX3_CTL11r_t;

#define BCMI_VIPER_XGXS_RX3_CTL11r_CLR(r) (r).rx3_ctl11[0] = 0
#define BCMI_VIPER_XGXS_RX3_CTL11r_SET(r,d) (r).rx3_ctl11[0] = d
#define BCMI_VIPER_XGXS_RX3_CTL11r_GET(r) (r).rx3_ctl11[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_RX3_CTL11r_RECAL_IND_CLRf_GET(r) ((((r).rx3_ctl11[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL11r_RECAL_IND_CLRf_SET(r,f) (r).rx3_ctl11[0]=(((r).rx3_ctl11[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_RX3_CTL11r_SLCAL_INIT_TIMEf_GET(r) ((((r).rx3_ctl11[0]) >> 7) & 0xff)
#define BCMI_VIPER_XGXS_RX3_CTL11r_SLCAL_INIT_TIMEf_SET(r,f) (r).rx3_ctl11[0]=(((r).rx3_ctl11[0] & ~((uint32_t)0xff << 7)) | ((((uint32_t)f) & 0xff) << 7)) | (255 << (16 + 7))
#define BCMI_VIPER_XGXS_RX3_CTL11r_PM_RXSLICERCALBYPf_GET(r) ((((r).rx3_ctl11[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL11r_PM_RXSLICERCALBYPf_SET(r,f) (r).rx3_ctl11[0]=(((r).rx3_ctl11[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_RX3_CTL11r_PM_RXLIMITAMPCALBYPf_GET(r) (((r).rx3_ctl11[0]) & 0x1)
#define BCMI_VIPER_XGXS_RX3_CTL11r_PM_RXLIMITAMPCALBYPf_SET(r,f) (r).rx3_ctl11[0]=(((r).rx3_ctl11[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access RX3_CTL11.
 *
 */
#define BCMI_VIPER_XGXS_READ_RX3_CTL11r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL11r,(_r._rx3_ctl11))
#define BCMI_VIPER_XGXS_WRITE_RX3_CTL11r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL11r,(_r._rx3_ctl11)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_RX3_CTL11r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL11r,(_r._rx3_ctl11))
#define BCMI_VIPER_XGXS_READLN_RX3_CTL11r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_RX3_CTL11r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl11))
#define BCMI_VIPER_XGXS_WRITELN_RX3_CTL11r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL11r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._rx3_ctl11))
#define BCMI_VIPER_XGXS_WRITEALL_RX3_CTL11r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_RX3_CTL11r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._rx3_ctl11))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define RX3_CTL11r BCMI_VIPER_XGXS_RX3_CTL11r
#define RX3_CTL11r_SIZE BCMI_VIPER_XGXS_RX3_CTL11r_SIZE
typedef BCMI_VIPER_XGXS_RX3_CTL11r_t RX3_CTL11r_t;
#define RX3_CTL11r_CLR BCMI_VIPER_XGXS_RX3_CTL11r_CLR
#define RX3_CTL11r_SET BCMI_VIPER_XGXS_RX3_CTL11r_SET
#define RX3_CTL11r_GET BCMI_VIPER_XGXS_RX3_CTL11r_GET
#define RX3_CTL11r_RECAL_IND_CLRf_GET BCMI_VIPER_XGXS_RX3_CTL11r_RECAL_IND_CLRf_GET
#define RX3_CTL11r_RECAL_IND_CLRf_SET BCMI_VIPER_XGXS_RX3_CTL11r_RECAL_IND_CLRf_SET
#define RX3_CTL11r_SLCAL_INIT_TIMEf_GET BCMI_VIPER_XGXS_RX3_CTL11r_SLCAL_INIT_TIMEf_GET
#define RX3_CTL11r_SLCAL_INIT_TIMEf_SET BCMI_VIPER_XGXS_RX3_CTL11r_SLCAL_INIT_TIMEf_SET
#define RX3_CTL11r_PM_RXSLICERCALBYPf_GET BCMI_VIPER_XGXS_RX3_CTL11r_PM_RXSLICERCALBYPf_GET
#define RX3_CTL11r_PM_RXSLICERCALBYPf_SET BCMI_VIPER_XGXS_RX3_CTL11r_PM_RXSLICERCALBYPf_SET
#define RX3_CTL11r_PM_RXLIMITAMPCALBYPf_GET BCMI_VIPER_XGXS_RX3_CTL11r_PM_RXLIMITAMPCALBYPf_GET
#define RX3_CTL11r_PM_RXLIMITAMPCALBYPf_SET BCMI_VIPER_XGXS_RX3_CTL11r_PM_RXLIMITAMPCALBYPf_SET
#define READ_RX3_CTL11r BCMI_VIPER_XGXS_READ_RX3_CTL11r
#define WRITE_RX3_CTL11r BCMI_VIPER_XGXS_WRITE_RX3_CTL11r
#define MODIFY_RX3_CTL11r BCMI_VIPER_XGXS_MODIFY_RX3_CTL11r
#define READLN_RX3_CTL11r BCMI_VIPER_XGXS_READLN_RX3_CTL11r
#define WRITELN_RX3_CTL11r BCMI_VIPER_XGXS_WRITELN_RX3_CTL11r
#define WRITEALL_RX3_CTL11r BCMI_VIPER_XGXS_WRITEALL_RX3_CTL11r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_RX3_CTL11r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  AER
 * BLOCKS:   AERBLK
 * REGADDR:  0xffde
 * DESC:     Address Expansion Register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/W
 * FIELDS:
 *     MMD_PORT         Each lane specific register can be accessed by setting MMD_port = AER offset strap + lane number.Also be set to AER broadcast strap (aer_bcst_ofs_strap) to broadcast to all lanes.
 *     MMD_DEVICETYPE   Selects the indicated internal MMD, for a valid DEVAD in the MDIO data stream,MMD device type
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_AERr (0x0000ffde | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_AERr_SIZE 4

/*
 * This structure should be used to declare and program AER.
 *
 */
typedef union BCMI_VIPER_XGXS_AERr_s {
	uint32_t v[1];
	uint32_t aer[1];
	uint32_t _aer;
} BCMI_VIPER_XGXS_AERr_t;

#define BCMI_VIPER_XGXS_AERr_CLR(r) (r).aer[0] = 0
#define BCMI_VIPER_XGXS_AERr_SET(r,d) (r).aer[0] = d
#define BCMI_VIPER_XGXS_AERr_GET(r) (r).aer[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_AERr_MMD_DEVICETYPEf_GET(r) ((((r).aer[0]) >> 11) & 0x1f)
#define BCMI_VIPER_XGXS_AERr_MMD_DEVICETYPEf_SET(r,f) (r).aer[0]=(((r).aer[0] & ~((uint32_t)0x1f << 11)) | ((((uint32_t)f) & 0x1f) << 11)) | (31 << (16 + 11))
#define BCMI_VIPER_XGXS_AERr_MMD_PORTf_GET(r) (((r).aer[0]) & 0x7ff)
#define BCMI_VIPER_XGXS_AERr_MMD_PORTf_SET(r,f) (r).aer[0]=(((r).aer[0] & ~((uint32_t)0x7ff)) | (((uint32_t)f) & 0x7ff)) | (0x7ff << 16)

/*
 * These macros can be used to access AER.
 *
 */
#define BCMI_VIPER_XGXS_READ_AERr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AERr,(_r._aer))
#define BCMI_VIPER_XGXS_WRITE_AERr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AERr,(_r._aer)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_AERr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AERr,(_r._aer))
#define BCMI_VIPER_XGXS_READLN_AERr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_AERr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._aer))
#define BCMI_VIPER_XGXS_WRITELN_AERr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AERr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._aer))
#define BCMI_VIPER_XGXS_WRITEALL_AERr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_AERr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._aer))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define AERr BCMI_VIPER_XGXS_AERr
#define AERr_SIZE BCMI_VIPER_XGXS_AERr_SIZE
typedef BCMI_VIPER_XGXS_AERr_t AERr_t;
#define AERr_CLR BCMI_VIPER_XGXS_AERr_CLR
#define AERr_SET BCMI_VIPER_XGXS_AERr_SET
#define AERr_GET BCMI_VIPER_XGXS_AERr_GET
#define AERr_MMD_DEVICETYPEf_GET BCMI_VIPER_XGXS_AERr_MMD_DEVICETYPEf_GET
#define AERr_MMD_DEVICETYPEf_SET BCMI_VIPER_XGXS_AERr_MMD_DEVICETYPEf_SET
#define AERr_MMD_PORTf_GET BCMI_VIPER_XGXS_AERr_MMD_PORTf_GET
#define AERr_MMD_PORTf_SET BCMI_VIPER_XGXS_AERr_MMD_PORTf_SET
#define READ_AERr BCMI_VIPER_XGXS_READ_AERr
#define WRITE_AERr BCMI_VIPER_XGXS_WRITE_AERr
#define MODIFY_AERr BCMI_VIPER_XGXS_MODIFY_AERr
#define READLN_AERr BCMI_VIPER_XGXS_READLN_AERr
#define WRITELN_AERr BCMI_VIPER_XGXS_WRITELN_AERr
#define WRITEALL_AERr BCMI_VIPER_XGXS_WRITEALL_AERr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_AERr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_MIICTL
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffe0
 * DESC:     IEEE MII control register
 * RESETVAL: 0x140 (320)
 * ACCESS:   R/W
 * FIELDS:
 *     MANUAL_SPEED1    Bits6 13----1 1 = Reserved1 0 = SGMII 1000 Mb/s0 1 = SGMII 100 Mb/s0 0 = SGMII 10 Mb/sNote: speed1 & speed0 bits for SGMII mode only
 *     COLLISION_TEST_EN 1 = collision test mode enabled0 = collision test mode disabled
 *     FULL_DUPLEX      1 = full duplex0 = half duplex
 *     RESTART_AUTONEG  1 = restart auto-negotiation process0 = normal operation
 *     PWRDWN_SW        1 = low power mode0 = normal operation
 *     AUTONEG_ENABLE   1 = auto-negotiation enabled0 = auto-negotiation disabled
 *     MANUAL_SPEED0    Bits6 13----1 1 = Reserved1 0 = SGMII 1000 Mb/s0 1 = SGMII 100 Mb/s0 0 = SGMII 10 Mb/sNote: speed1 & speed0 bits for SGMII mode only
 *     GLOOPBACK        1 = Global loopback mode is enabled (i.e. TX->RX)0 = normal operation
 *     RST_HW           1 = PHY Reset0 = normal operation
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_MIICTLr (0x0000ffe0 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_MIICTLr_SIZE 4

/*
 * This structure should be used to declare and program COMBO_MIICTL.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_MIICTLr_s {
	uint32_t v[1];
	uint32_t combo_miictl[1];
	uint32_t _combo_miictl;
} BCMI_VIPER_XGXS_COMBO_MIICTLr_t;

#define BCMI_VIPER_XGXS_COMBO_MIICTLr_CLR(r) (r).combo_miictl[0] = 0
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_SET(r,d) (r).combo_miictl[0] = d
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_GET(r) (r).combo_miictl[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_RST_HWf_GET(r) ((((r).combo_miictl[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_RST_HWf_SET(r,f) (r).combo_miictl[0]=(((r).combo_miictl[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_GLOOPBACKf_GET(r) ((((r).combo_miictl[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_GLOOPBACKf_SET(r,f) (r).combo_miictl[0]=(((r).combo_miictl[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_MANUAL_SPEED0f_GET(r) ((((r).combo_miictl[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_MANUAL_SPEED0f_SET(r,f) (r).combo_miictl[0]=(((r).combo_miictl[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_AUTONEG_ENABLEf_GET(r) ((((r).combo_miictl[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_AUTONEG_ENABLEf_SET(r,f) (r).combo_miictl[0]=(((r).combo_miictl[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_PWRDWN_SWf_GET(r) ((((r).combo_miictl[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_PWRDWN_SWf_SET(r,f) (r).combo_miictl[0]=(((r).combo_miictl[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_RESTART_AUTONEGf_GET(r) ((((r).combo_miictl[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_RESTART_AUTONEGf_SET(r,f) (r).combo_miictl[0]=(((r).combo_miictl[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_FULL_DUPLEXf_GET(r) ((((r).combo_miictl[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_FULL_DUPLEXf_SET(r,f) (r).combo_miictl[0]=(((r).combo_miictl[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_COLLISION_TEST_ENf_GET(r) ((((r).combo_miictl[0]) >> 7) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_COLLISION_TEST_ENf_SET(r,f) (r).combo_miictl[0]=(((r).combo_miictl[0] & ~((uint32_t)0x1 << 7)) | ((((uint32_t)f) & 0x1) << 7)) | (1 << (16 + 7))
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_MANUAL_SPEED1f_GET(r) ((((r).combo_miictl[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIICTLr_MANUAL_SPEED1f_SET(r,f) (r).combo_miictl[0]=(((r).combo_miictl[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))

/*
 * These macros can be used to access COMBO_MIICTL.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_MIICTLr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_MIICTLr,(_r._combo_miictl))
#define BCMI_VIPER_XGXS_WRITE_COMBO_MIICTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIICTLr,(_r._combo_miictl)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_MIICTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIICTLr,(_r._combo_miictl))
#define BCMI_VIPER_XGXS_READLN_COMBO_MIICTLr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_MIICTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_miictl))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_MIICTLr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIICTLr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_miictl))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_MIICTLr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIICTLr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_miictl))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_MIICTLr BCMI_VIPER_XGXS_COMBO_MIICTLr
#define COMBO_MIICTLr_SIZE BCMI_VIPER_XGXS_COMBO_MIICTLr_SIZE
typedef BCMI_VIPER_XGXS_COMBO_MIICTLr_t COMBO_MIICTLr_t;
#define COMBO_MIICTLr_CLR BCMI_VIPER_XGXS_COMBO_MIICTLr_CLR
#define COMBO_MIICTLr_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_SET
#define COMBO_MIICTLr_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_GET
#define COMBO_MIICTLr_RST_HWf_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_RST_HWf_GET
#define COMBO_MIICTLr_RST_HWf_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_RST_HWf_SET
#define COMBO_MIICTLr_GLOOPBACKf_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_GLOOPBACKf_GET
#define COMBO_MIICTLr_GLOOPBACKf_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_GLOOPBACKf_SET
#define COMBO_MIICTLr_MANUAL_SPEED0f_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_MANUAL_SPEED0f_GET
#define COMBO_MIICTLr_MANUAL_SPEED0f_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_MANUAL_SPEED0f_SET
#define COMBO_MIICTLr_AUTONEG_ENABLEf_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_AUTONEG_ENABLEf_GET
#define COMBO_MIICTLr_AUTONEG_ENABLEf_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_AUTONEG_ENABLEf_SET
#define COMBO_MIICTLr_PWRDWN_SWf_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_PWRDWN_SWf_GET
#define COMBO_MIICTLr_PWRDWN_SWf_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_PWRDWN_SWf_SET
#define COMBO_MIICTLr_RESTART_AUTONEGf_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_RESTART_AUTONEGf_GET
#define COMBO_MIICTLr_RESTART_AUTONEGf_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_RESTART_AUTONEGf_SET
#define COMBO_MIICTLr_FULL_DUPLEXf_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_FULL_DUPLEXf_GET
#define COMBO_MIICTLr_FULL_DUPLEXf_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_FULL_DUPLEXf_SET
#define COMBO_MIICTLr_COLLISION_TEST_ENf_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_COLLISION_TEST_ENf_GET
#define COMBO_MIICTLr_COLLISION_TEST_ENf_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_COLLISION_TEST_ENf_SET
#define COMBO_MIICTLr_MANUAL_SPEED1f_GET BCMI_VIPER_XGXS_COMBO_MIICTLr_MANUAL_SPEED1f_GET
#define COMBO_MIICTLr_MANUAL_SPEED1f_SET BCMI_VIPER_XGXS_COMBO_MIICTLr_MANUAL_SPEED1f_SET
#define READ_COMBO_MIICTLr BCMI_VIPER_XGXS_READ_COMBO_MIICTLr
#define WRITE_COMBO_MIICTLr BCMI_VIPER_XGXS_WRITE_COMBO_MIICTLr
#define MODIFY_COMBO_MIICTLr BCMI_VIPER_XGXS_MODIFY_COMBO_MIICTLr
#define READLN_COMBO_MIICTLr BCMI_VIPER_XGXS_READLN_COMBO_MIICTLr
#define WRITELN_COMBO_MIICTLr BCMI_VIPER_XGXS_WRITELN_COMBO_MIICTLr
#define WRITEALL_COMBO_MIICTLr BCMI_VIPER_XGXS_WRITEALL_COMBO_MIICTLr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_MIICTLr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_MIISTAT
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffe1
 * DESC:     IEEE MII status register
 * RESETVAL: 0x109 (265)
 * ACCESS:   R/O
 * FIELDS:
 *     EXTENDED_CAPABILITY 1 = extended register capabilities supported0 = basic register set capabilities only
 *     JABBER_DETECT    1 = jabber condition detected0 = no jabber condition detected
 *     LINK_STATUS      1 = link pass0 = link faillatching-low
 *     AUTONEG_ABILITY  1 = auto-negotiation capable0 = not auto-negotiation capable
 *     REMOTE_FAULT     1 = remote fault detected0 = no remote fault detectedlatching-high
 *     AUTONEG_COMPLETE 1 = auto-negotiation complete0 = auto-negotiation in progress
 *     MF_PREAMBLE_SUPRESSION 1 = PHY will accept management frames with preamble suppressed0 = PHY will not accept management frames with
 *     EXTENDED_STATUS  1 = extended status information in register 0Fh0 = no extended status info in register 0Fh
 *     S100BASE_T2_HALF_DUPLEX_CAPABLE 1 = 100Base-T2 half duplex capable0 = not 100Base-T2 half duplex capable
 *     S100BASE_T2_FULL_DUPLEX_CAPABLE 1 = 100Base-T2 full duplex capable0 = not 100Base-T2 full duplex capable
 *     S10BASE_T_HALF_DUPLEX_CAPABLE 1 = 10Base-T half duplex capable0 = not 10Base-T half duplex capable
 *     S10BASE_T_FULL_DUPLEX_CAPABLE 1 = 10Base-T full duplex capable0 = not 10Base-T full duplex capable
 *     S100BASE_X_HALF_DUPLEX_CAPABLE 1 = 100Base-X half duplex capable0 = not 100Base-X half duplex capable
 *     S100BASE_X_FULL_DUPLEX_CAPABLE 1 = 100Base-X full duplex capable0 = not 100Base-X full duplex capable
 *     S100BASE_T4_CAPABLE 1 = 100Base-T4 capable0 = not 100Base-T4 capable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_MIISTATr (0x0000ffe1 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_MIISTATr_SIZE 4

/*
 * This structure should be used to declare and program COMBO_MIISTAT.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_MIISTATr_s {
	uint32_t v[1];
	uint32_t combo_miistat[1];
	uint32_t _combo_miistat;
} BCMI_VIPER_XGXS_COMBO_MIISTATr_t;

#define BCMI_VIPER_XGXS_COMBO_MIISTATr_CLR(r) (r).combo_miistat[0] = 0
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_SET(r,d) (r).combo_miistat[0] = d
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_GET(r) (r).combo_miistat[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T4_CAPABLEf_GET(r) ((((r).combo_miistat[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T4_CAPABLEf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miistat[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miistat[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miistat[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miistat[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miistat[0]) >> 10) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 10)) | ((((uint32_t)f) & 0x1) << 10)) | (1 << (16 + 10))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miistat[0]) >> 9) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 9)) | ((((uint32_t)f) & 0x1) << 9)) | (1 << (16 + 9))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_EXTENDED_STATUSf_GET(r) ((((r).combo_miistat[0]) >> 8) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_EXTENDED_STATUSf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 8)) | ((((uint32_t)f) & 0x1) << 8)) | (1 << (16 + 8))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_MF_PREAMBLE_SUPRESSIONf_GET(r) ((((r).combo_miistat[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_MF_PREAMBLE_SUPRESSIONf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_AUTONEG_COMPLETEf_GET(r) ((((r).combo_miistat[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_AUTONEG_COMPLETEf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_REMOTE_FAULTf_GET(r) ((((r).combo_miistat[0]) >> 4) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_REMOTE_FAULTf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 4)) | ((((uint32_t)f) & 0x1) << 4)) | (1 << (16 + 4))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_AUTONEG_ABILITYf_GET(r) ((((r).combo_miistat[0]) >> 3) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_AUTONEG_ABILITYf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 3)) | ((((uint32_t)f) & 0x1) << 3)) | (1 << (16 + 3))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_LINK_STATUSf_GET(r) ((((r).combo_miistat[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_LINK_STATUSf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_JABBER_DETECTf_GET(r) ((((r).combo_miistat[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_JABBER_DETECTf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_EXTENDED_CAPABILITYf_GET(r) (((r).combo_miistat[0]) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIISTATr_EXTENDED_CAPABILITYf_SET(r,f) (r).combo_miistat[0]=(((r).combo_miistat[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access COMBO_MIISTAT.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_MIISTATr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_MIISTATr,(_r._combo_miistat))
#define BCMI_VIPER_XGXS_WRITE_COMBO_MIISTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIISTATr,(_r._combo_miistat)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_MIISTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIISTATr,(_r._combo_miistat))
#define BCMI_VIPER_XGXS_READLN_COMBO_MIISTATr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_MIISTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_miistat))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_MIISTATr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIISTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_miistat))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_MIISTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIISTATr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_miistat))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_MIISTATr BCMI_VIPER_XGXS_COMBO_MIISTATr
#define COMBO_MIISTATr_SIZE BCMI_VIPER_XGXS_COMBO_MIISTATr_SIZE
typedef BCMI_VIPER_XGXS_COMBO_MIISTATr_t COMBO_MIISTATr_t;
#define COMBO_MIISTATr_CLR BCMI_VIPER_XGXS_COMBO_MIISTATr_CLR
#define COMBO_MIISTATr_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_SET
#define COMBO_MIISTATr_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_GET
#define COMBO_MIISTATr_S100BASE_T4_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T4_CAPABLEf_GET
#define COMBO_MIISTATr_S100BASE_T4_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T4_CAPABLEf_SET
#define COMBO_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_GET
#define COMBO_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_X_FULL_DUPLEX_CAPABLEf_SET
#define COMBO_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_GET
#define COMBO_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_X_HALF_DUPLEX_CAPABLEf_SET
#define COMBO_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_GET
#define COMBO_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_S10BASE_T_FULL_DUPLEX_CAPABLEf_SET
#define COMBO_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_GET
#define COMBO_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_S10BASE_T_HALF_DUPLEX_CAPABLEf_SET
#define COMBO_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_GET
#define COMBO_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T2_FULL_DUPLEX_CAPABLEf_SET
#define COMBO_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_GET
#define COMBO_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_S100BASE_T2_HALF_DUPLEX_CAPABLEf_SET
#define COMBO_MIISTATr_EXTENDED_STATUSf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_EXTENDED_STATUSf_GET
#define COMBO_MIISTATr_EXTENDED_STATUSf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_EXTENDED_STATUSf_SET
#define COMBO_MIISTATr_MF_PREAMBLE_SUPRESSIONf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_MF_PREAMBLE_SUPRESSIONf_GET
#define COMBO_MIISTATr_MF_PREAMBLE_SUPRESSIONf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_MF_PREAMBLE_SUPRESSIONf_SET
#define COMBO_MIISTATr_AUTONEG_COMPLETEf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_AUTONEG_COMPLETEf_GET
#define COMBO_MIISTATr_AUTONEG_COMPLETEf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_AUTONEG_COMPLETEf_SET
#define COMBO_MIISTATr_REMOTE_FAULTf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_REMOTE_FAULTf_GET
#define COMBO_MIISTATr_REMOTE_FAULTf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_REMOTE_FAULTf_SET
#define COMBO_MIISTATr_AUTONEG_ABILITYf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_AUTONEG_ABILITYf_GET
#define COMBO_MIISTATr_AUTONEG_ABILITYf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_AUTONEG_ABILITYf_SET
#define COMBO_MIISTATr_LINK_STATUSf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_LINK_STATUSf_GET
#define COMBO_MIISTATr_LINK_STATUSf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_LINK_STATUSf_SET
#define COMBO_MIISTATr_JABBER_DETECTf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_JABBER_DETECTf_GET
#define COMBO_MIISTATr_JABBER_DETECTf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_JABBER_DETECTf_SET
#define COMBO_MIISTATr_EXTENDED_CAPABILITYf_GET BCMI_VIPER_XGXS_COMBO_MIISTATr_EXTENDED_CAPABILITYf_GET
#define COMBO_MIISTATr_EXTENDED_CAPABILITYf_SET BCMI_VIPER_XGXS_COMBO_MIISTATr_EXTENDED_CAPABILITYf_SET
#define READ_COMBO_MIISTATr BCMI_VIPER_XGXS_READ_COMBO_MIISTATr
#define WRITE_COMBO_MIISTATr BCMI_VIPER_XGXS_WRITE_COMBO_MIISTATr
#define MODIFY_COMBO_MIISTATr BCMI_VIPER_XGXS_MODIFY_COMBO_MIISTATr
#define READLN_COMBO_MIISTATr BCMI_VIPER_XGXS_READLN_COMBO_MIISTATr
#define WRITELN_COMBO_MIISTATr BCMI_VIPER_XGXS_WRITELN_COMBO_MIISTATr
#define WRITEALL_COMBO_MIISTATr BCMI_VIPER_XGXS_WRITEALL_COMBO_MIISTATr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_MIISTATr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_ID1
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffe2
 * DESC:     IEEE phy ID LSByte register
 * RESETVAL: 0x143 (323)
 * ACCESS:   R/O
 * FIELDS:
 *     REGID            PHYID register, MSB
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_ID1r (0x0000ffe2 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_ID1r_SIZE 4

/*
 * This structure should be used to declare and program COMBO_ID1.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_ID1r_s {
	uint32_t v[1];
	uint32_t combo_id1[1];
	uint32_t _combo_id1;
} BCMI_VIPER_XGXS_COMBO_ID1r_t;

#define BCMI_VIPER_XGXS_COMBO_ID1r_CLR(r) (r).combo_id1[0] = 0
#define BCMI_VIPER_XGXS_COMBO_ID1r_SET(r,d) (r).combo_id1[0] = d
#define BCMI_VIPER_XGXS_COMBO_ID1r_GET(r) (r).combo_id1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_ID1r_REGIDf_GET(r) (((r).combo_id1[0]) & 0xffff)
#define BCMI_VIPER_XGXS_COMBO_ID1r_REGIDf_SET(r,f) (r).combo_id1[0]=(((r).combo_id1[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access COMBO_ID1.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_ID1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_ID1r,(_r._combo_id1))
#define BCMI_VIPER_XGXS_WRITE_COMBO_ID1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_ID1r,(_r._combo_id1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_ID1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_ID1r,(_r._combo_id1))
#define BCMI_VIPER_XGXS_READLN_COMBO_ID1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_ID1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_id1))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_ID1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_ID1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_id1))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_ID1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_ID1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_id1))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_ID1r BCMI_VIPER_XGXS_COMBO_ID1r
#define COMBO_ID1r_SIZE BCMI_VIPER_XGXS_COMBO_ID1r_SIZE
typedef BCMI_VIPER_XGXS_COMBO_ID1r_t COMBO_ID1r_t;
#define COMBO_ID1r_CLR BCMI_VIPER_XGXS_COMBO_ID1r_CLR
#define COMBO_ID1r_SET BCMI_VIPER_XGXS_COMBO_ID1r_SET
#define COMBO_ID1r_GET BCMI_VIPER_XGXS_COMBO_ID1r_GET
#define COMBO_ID1r_REGIDf_GET BCMI_VIPER_XGXS_COMBO_ID1r_REGIDf_GET
#define COMBO_ID1r_REGIDf_SET BCMI_VIPER_XGXS_COMBO_ID1r_REGIDf_SET
#define READ_COMBO_ID1r BCMI_VIPER_XGXS_READ_COMBO_ID1r
#define WRITE_COMBO_ID1r BCMI_VIPER_XGXS_WRITE_COMBO_ID1r
#define MODIFY_COMBO_ID1r BCMI_VIPER_XGXS_MODIFY_COMBO_ID1r
#define READLN_COMBO_ID1r BCMI_VIPER_XGXS_READLN_COMBO_ID1r
#define WRITELN_COMBO_ID1r BCMI_VIPER_XGXS_WRITELN_COMBO_ID1r
#define WRITEALL_COMBO_ID1r BCMI_VIPER_XGXS_WRITEALL_COMBO_ID1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_ID1r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_ID2
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffe3
 * DESC:     IEEE phy ID MSByte register
 * RESETVAL: 0xbff0 (49136)
 * ACCESS:   R/O
 * FIELDS:
 *     REGID            PHYID register, LSB
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_ID2r (0x0000ffe3 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_ID2r_SIZE 4

/*
 * This structure should be used to declare and program COMBO_ID2.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_ID2r_s {
	uint32_t v[1];
	uint32_t combo_id2[1];
	uint32_t _combo_id2;
} BCMI_VIPER_XGXS_COMBO_ID2r_t;

#define BCMI_VIPER_XGXS_COMBO_ID2r_CLR(r) (r).combo_id2[0] = 0
#define BCMI_VIPER_XGXS_COMBO_ID2r_SET(r,d) (r).combo_id2[0] = d
#define BCMI_VIPER_XGXS_COMBO_ID2r_GET(r) (r).combo_id2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_ID2r_REGIDf_GET(r) (((r).combo_id2[0]) & 0xffff)
#define BCMI_VIPER_XGXS_COMBO_ID2r_REGIDf_SET(r,f) (r).combo_id2[0]=(((r).combo_id2[0] & ~((uint32_t)0xffff)) | (((uint32_t)f) & 0xffff)) | (0xffff << 16)

/*
 * These macros can be used to access COMBO_ID2.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_ID2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_ID2r,(_r._combo_id2))
#define BCMI_VIPER_XGXS_WRITE_COMBO_ID2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_ID2r,(_r._combo_id2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_ID2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_ID2r,(_r._combo_id2))
#define BCMI_VIPER_XGXS_READLN_COMBO_ID2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_ID2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_id2))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_ID2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_ID2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_id2))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_ID2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_ID2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_id2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_ID2r BCMI_VIPER_XGXS_COMBO_ID2r
#define COMBO_ID2r_SIZE BCMI_VIPER_XGXS_COMBO_ID2r_SIZE
typedef BCMI_VIPER_XGXS_COMBO_ID2r_t COMBO_ID2r_t;
#define COMBO_ID2r_CLR BCMI_VIPER_XGXS_COMBO_ID2r_CLR
#define COMBO_ID2r_SET BCMI_VIPER_XGXS_COMBO_ID2r_SET
#define COMBO_ID2r_GET BCMI_VIPER_XGXS_COMBO_ID2r_GET
#define COMBO_ID2r_REGIDf_GET BCMI_VIPER_XGXS_COMBO_ID2r_REGIDf_GET
#define COMBO_ID2r_REGIDf_SET BCMI_VIPER_XGXS_COMBO_ID2r_REGIDf_SET
#define READ_COMBO_ID2r BCMI_VIPER_XGXS_READ_COMBO_ID2r
#define WRITE_COMBO_ID2r BCMI_VIPER_XGXS_WRITE_COMBO_ID2r
#define MODIFY_COMBO_ID2r BCMI_VIPER_XGXS_MODIFY_COMBO_ID2r
#define READLN_COMBO_ID2r BCMI_VIPER_XGXS_READLN_COMBO_ID2r
#define WRITELN_COMBO_ID2r BCMI_VIPER_XGXS_WRITELN_COMBO_ID2r
#define WRITEALL_COMBO_ID2r BCMI_VIPER_XGXS_WRITEALL_COMBO_ID2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_ID2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_AUTONEGADV
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffe4
 * DESC:     IEEE auto-negotiation advertised abilities register
 * RESETVAL: 0x20 (32)
 * ACCESS:   R/W
 * FIELDS:
 *     FULL_DUPLEX      1 = advertise full-duplex0 = do not advertise full-duplex
 *     HALF_DUPLEX      1 = advertise half-duplex0 = do not advertise half-duplex
 *     PAUSE            Bits8 7---0 0 = no pause0 1 = symmetric pause1 0 = asymmetric pause toward link partner1 1 = both symmetric pause and asymmetricpause toward local device
 *     REMOTE_FAULT     Bits13 12-----0 0 = no_remote_fault0 1 = link_failure1 0 = offline1 1 = autoneg_error
 *     NEXT_PAGE        1 = supports additional pages using NP function0 = does not support additional pages using NP function
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr (0x0000ffe4 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_SIZE 4

/*
 * This structure should be used to declare and program COMBO_AUTONEGADV.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_s {
	uint32_t v[1];
	uint32_t combo_autonegadv[1];
	uint32_t _combo_autonegadv;
} BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_t;

#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_CLR(r) (r).combo_autonegadv[0] = 0
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_SET(r,d) (r).combo_autonegadv[0] = d
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_GET(r) (r).combo_autonegadv[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_NEXT_PAGEf_GET(r) ((((r).combo_autonegadv[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_NEXT_PAGEf_SET(r,f) (r).combo_autonegadv[0]=(((r).combo_autonegadv[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_REMOTE_FAULTf_GET(r) ((((r).combo_autonegadv[0]) >> 12) & 0x3)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_REMOTE_FAULTf_SET(r,f) (r).combo_autonegadv[0]=(((r).combo_autonegadv[0] & ~((uint32_t)0x3 << 12)) | ((((uint32_t)f) & 0x3) << 12)) | (3 << (16 + 12))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_PAUSEf_GET(r) ((((r).combo_autonegadv[0]) >> 7) & 0x3)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_PAUSEf_SET(r,f) (r).combo_autonegadv[0]=(((r).combo_autonegadv[0] & ~((uint32_t)0x3 << 7)) | ((((uint32_t)f) & 0x3) << 7)) | (3 << (16 + 7))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_HALF_DUPLEXf_GET(r) ((((r).combo_autonegadv[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_HALF_DUPLEXf_SET(r,f) (r).combo_autonegadv[0]=(((r).combo_autonegadv[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_FULL_DUPLEXf_GET(r) ((((r).combo_autonegadv[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_FULL_DUPLEXf_SET(r,f) (r).combo_autonegadv[0]=(((r).combo_autonegadv[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))

/*
 * These macros can be used to access COMBO_AUTONEGADV.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_AUTONEGADVr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGADVr,(_r._combo_autonegadv))
#define BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGADVr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGADVr,(_r._combo_autonegadv)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGADVr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGADVr,(_r._combo_autonegadv))
#define BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGADVr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGADVr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autonegadv))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGADVr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGADVr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autonegadv))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGADVr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGADVr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_autonegadv))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_AUTONEGADVr BCMI_VIPER_XGXS_COMBO_AUTONEGADVr
#define COMBO_AUTONEGADVr_SIZE BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_SIZE
typedef BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_t COMBO_AUTONEGADVr_t;
#define COMBO_AUTONEGADVr_CLR BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_CLR
#define COMBO_AUTONEGADVr_SET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_SET
#define COMBO_AUTONEGADVr_GET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_GET
#define COMBO_AUTONEGADVr_NEXT_PAGEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_NEXT_PAGEf_GET
#define COMBO_AUTONEGADVr_NEXT_PAGEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_NEXT_PAGEf_SET
#define COMBO_AUTONEGADVr_REMOTE_FAULTf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_REMOTE_FAULTf_GET
#define COMBO_AUTONEGADVr_REMOTE_FAULTf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_REMOTE_FAULTf_SET
#define COMBO_AUTONEGADVr_PAUSEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_PAUSEf_GET
#define COMBO_AUTONEGADVr_PAUSEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_PAUSEf_SET
#define COMBO_AUTONEGADVr_HALF_DUPLEXf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_HALF_DUPLEXf_GET
#define COMBO_AUTONEGADVr_HALF_DUPLEXf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_HALF_DUPLEXf_SET
#define COMBO_AUTONEGADVr_FULL_DUPLEXf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_FULL_DUPLEXf_GET
#define COMBO_AUTONEGADVr_FULL_DUPLEXf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGADVr_FULL_DUPLEXf_SET
#define READ_COMBO_AUTONEGADVr BCMI_VIPER_XGXS_READ_COMBO_AUTONEGADVr
#define WRITE_COMBO_AUTONEGADVr BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGADVr
#define MODIFY_COMBO_AUTONEGADVr BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGADVr
#define READLN_COMBO_AUTONEGADVr BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGADVr
#define WRITELN_COMBO_AUTONEGADVr BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGADVr
#define WRITEALL_COMBO_AUTONEGADVr BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGADVr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_AUTONEGADVr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_AUTONEGLPABIL
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffe5
 * DESC:     IEEE auto-negotiation link partner abilities register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     SGMII_MODE       1 = SGMII mode0 = fiber modeNote: When the link partner is in SGMII mode (bit 0 = 1), then bit 15 = link, bit 12 = duplex, bits 11:10 = speed, bit 14 = acknowledge. The other bits are reserved and should be zero.
 *     FULL_DUPLEX      1 = link partner is full duplex capable0 = link partner is not full duplex capable
 *     HALF_DUPLEX      1 = link partner is half-duplex capable0 = link partner is not half-duplex capable
 *     PAUSE            Bits8 7---0 0 = no pause0 1 = symmetric pause1 0 = asymmetric pause toward link partner1 1 = both symmetric pause and asymmetricpause toward local device
 *     REMOTE_FAULT     Bits13 12-----0 0 = no_remote_fault0 1 = link_failure1 0 = offline1 1 = autoneg_error
 *     ACKNOWLEDGE      1 = link partner has received link code word0 = link partner has not received link code word
 *     NEXT_PAGE        1 = link partner is next page able0 = link partner is not next page able
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr (0x0000ffe5 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_SIZE 4

/*
 * This structure should be used to declare and program COMBO_AUTONEGLPABIL.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_s {
	uint32_t v[1];
	uint32_t combo_autoneglpabil[1];
	uint32_t _combo_autoneglpabil;
} BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_t;

#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_CLR(r) (r).combo_autoneglpabil[0] = 0
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_SET(r,d) (r).combo_autoneglpabil[0] = d
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_GET(r) (r).combo_autoneglpabil[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_NEXT_PAGEf_GET(r) ((((r).combo_autoneglpabil[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_NEXT_PAGEf_SET(r,f) (r).combo_autoneglpabil[0]=(((r).combo_autoneglpabil[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_ACKNOWLEDGEf_GET(r) ((((r).combo_autoneglpabil[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_ACKNOWLEDGEf_SET(r,f) (r).combo_autoneglpabil[0]=(((r).combo_autoneglpabil[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_REMOTE_FAULTf_GET(r) ((((r).combo_autoneglpabil[0]) >> 12) & 0x3)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_REMOTE_FAULTf_SET(r,f) (r).combo_autoneglpabil[0]=(((r).combo_autoneglpabil[0] & ~((uint32_t)0x3 << 12)) | ((((uint32_t)f) & 0x3) << 12)) | (3 << (16 + 12))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_PAUSEf_GET(r) ((((r).combo_autoneglpabil[0]) >> 7) & 0x3)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_PAUSEf_SET(r,f) (r).combo_autoneglpabil[0]=(((r).combo_autoneglpabil[0] & ~((uint32_t)0x3 << 7)) | ((((uint32_t)f) & 0x3) << 7)) | (3 << (16 + 7))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_HALF_DUPLEXf_GET(r) ((((r).combo_autoneglpabil[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_HALF_DUPLEXf_SET(r,f) (r).combo_autoneglpabil[0]=(((r).combo_autoneglpabil[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_FULL_DUPLEXf_GET(r) ((((r).combo_autoneglpabil[0]) >> 5) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_FULL_DUPLEXf_SET(r,f) (r).combo_autoneglpabil[0]=(((r).combo_autoneglpabil[0] & ~((uint32_t)0x1 << 5)) | ((((uint32_t)f) & 0x1) << 5)) | (1 << (16 + 5))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_SGMII_MODEf_GET(r) (((r).combo_autoneglpabil[0]) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_SGMII_MODEf_SET(r,f) (r).combo_autoneglpabil[0]=(((r).combo_autoneglpabil[0] & ~((uint32_t)0x1)) | (((uint32_t)f) & 0x1)) | (0x1 << 16)

/*
 * These macros can be used to access COMBO_AUTONEGLPABIL.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_AUTONEGLPABILr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr,(_r._combo_autoneglpabil))
#define BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGLPABILr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr,(_r._combo_autoneglpabil)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGLPABILr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr,(_r._combo_autoneglpabil))
#define BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGLPABILr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autoneglpabil))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGLPABILr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autoneglpabil))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGLPABILr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_autoneglpabil))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_AUTONEGLPABILr BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr
#define COMBO_AUTONEGLPABILr_SIZE BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_SIZE
typedef BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_t COMBO_AUTONEGLPABILr_t;
#define COMBO_AUTONEGLPABILr_CLR BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_CLR
#define COMBO_AUTONEGLPABILr_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_SET
#define COMBO_AUTONEGLPABILr_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_GET
#define COMBO_AUTONEGLPABILr_NEXT_PAGEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_NEXT_PAGEf_GET
#define COMBO_AUTONEGLPABILr_NEXT_PAGEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_NEXT_PAGEf_SET
#define COMBO_AUTONEGLPABILr_ACKNOWLEDGEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_ACKNOWLEDGEf_GET
#define COMBO_AUTONEGLPABILr_ACKNOWLEDGEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_ACKNOWLEDGEf_SET
#define COMBO_AUTONEGLPABILr_REMOTE_FAULTf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_REMOTE_FAULTf_GET
#define COMBO_AUTONEGLPABILr_REMOTE_FAULTf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_REMOTE_FAULTf_SET
#define COMBO_AUTONEGLPABILr_PAUSEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_PAUSEf_GET
#define COMBO_AUTONEGLPABILr_PAUSEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_PAUSEf_SET
#define COMBO_AUTONEGLPABILr_HALF_DUPLEXf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_HALF_DUPLEXf_GET
#define COMBO_AUTONEGLPABILr_HALF_DUPLEXf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_HALF_DUPLEXf_SET
#define COMBO_AUTONEGLPABILr_FULL_DUPLEXf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_FULL_DUPLEXf_GET
#define COMBO_AUTONEGLPABILr_FULL_DUPLEXf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_FULL_DUPLEXf_SET
#define COMBO_AUTONEGLPABILr_SGMII_MODEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_SGMII_MODEf_GET
#define COMBO_AUTONEGLPABILr_SGMII_MODEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr_SGMII_MODEf_SET
#define READ_COMBO_AUTONEGLPABILr BCMI_VIPER_XGXS_READ_COMBO_AUTONEGLPABILr
#define WRITE_COMBO_AUTONEGLPABILr BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGLPABILr
#define MODIFY_COMBO_AUTONEGLPABILr BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGLPABILr
#define READLN_COMBO_AUTONEGLPABILr BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGLPABILr
#define WRITELN_COMBO_AUTONEGLPABILr BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGLPABILr
#define WRITEALL_COMBO_AUTONEGLPABILr BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGLPABILr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_AUTONEGLPABILr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_AUTONEGEXP
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffe6
 * DESC:     IEEE auto-negotiation expansion register
 * RESETVAL: 0x4 (4)
 * ACCESS:   R/O
 * FIELDS:
 *     PAGE_RECEIVED    1 = new link code word has been received0 = new link code word has not been received
 *     NEXT_PAGE_ABILITY 1 = local device is next page able0 = local device is not next page able
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr (0x0000ffe6 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_SIZE 4

/*
 * This structure should be used to declare and program COMBO_AUTONEGEXP.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_s {
	uint32_t v[1];
	uint32_t combo_autonegexp[1];
	uint32_t _combo_autonegexp;
} BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_t;

#define BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_CLR(r) (r).combo_autonegexp[0] = 0
#define BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_SET(r,d) (r).combo_autonegexp[0] = d
#define BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_GET(r) (r).combo_autonegexp[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_NEXT_PAGE_ABILITYf_GET(r) ((((r).combo_autonegexp[0]) >> 2) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_NEXT_PAGE_ABILITYf_SET(r,f) (r).combo_autonegexp[0]=(((r).combo_autonegexp[0] & ~((uint32_t)0x1 << 2)) | ((((uint32_t)f) & 0x1) << 2)) | (1 << (16 + 2))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_PAGE_RECEIVEDf_GET(r) ((((r).combo_autonegexp[0]) >> 1) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_PAGE_RECEIVEDf_SET(r,f) (r).combo_autonegexp[0]=(((r).combo_autonegexp[0] & ~((uint32_t)0x1 << 1)) | ((((uint32_t)f) & 0x1) << 1)) | (1 << (16 + 1))

/*
 * These macros can be used to access COMBO_AUTONEGEXP.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_AUTONEGEXPr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr,(_r._combo_autonegexp))
#define BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGEXPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr,(_r._combo_autonegexp)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGEXPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr,(_r._combo_autonegexp))
#define BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGEXPr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autonegexp))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGEXPr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autonegexp))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGEXPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_autonegexp))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_AUTONEGEXPr BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr
#define COMBO_AUTONEGEXPr_SIZE BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_SIZE
typedef BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_t COMBO_AUTONEGEXPr_t;
#define COMBO_AUTONEGEXPr_CLR BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_CLR
#define COMBO_AUTONEGEXPr_SET BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_SET
#define COMBO_AUTONEGEXPr_GET BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_GET
#define COMBO_AUTONEGEXPr_NEXT_PAGE_ABILITYf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_NEXT_PAGE_ABILITYf_GET
#define COMBO_AUTONEGEXPr_NEXT_PAGE_ABILITYf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_NEXT_PAGE_ABILITYf_SET
#define COMBO_AUTONEGEXPr_PAGE_RECEIVEDf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_PAGE_RECEIVEDf_GET
#define COMBO_AUTONEGEXPr_PAGE_RECEIVEDf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr_PAGE_RECEIVEDf_SET
#define READ_COMBO_AUTONEGEXPr BCMI_VIPER_XGXS_READ_COMBO_AUTONEGEXPr
#define WRITE_COMBO_AUTONEGEXPr BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGEXPr
#define MODIFY_COMBO_AUTONEGEXPr BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGEXPr
#define READLN_COMBO_AUTONEGEXPr BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGEXPr
#define WRITELN_COMBO_AUTONEGEXPr BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGEXPr
#define WRITEALL_COMBO_AUTONEGEXPr BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGEXPr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_AUTONEGEXPr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_AUTONEGNP
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffe7
 * DESC:     IEEE auto-negotiation next page register
 * RESETVAL: 0x2001 (8193)
 * ACCESS:   R/W
 * FIELDS:
 *     MESSAGE          Message or Unformatted Code Field11'h400 = Over 1G Message Page11'h410 = Remote CuPHY Message Page11'h411 = MDIO Register Write Message Page11'h412 = MDIO Register Read Request Message Page11'h413 = MDIO Register Response Message PageSee IEEE 802.3 Annex 28C for more standard next page detailsSee BRCM-Serdes-AN for BAM specific details
 *     TOGGLE           Opposite value of bit in previous page
 *     ACK2             Acknowledge 2 bit
 *     MESSAGE_PAGE     0 = Unformatted Page1 = Message Page
 *     ACK              Acknowledge bit
 *     NEXT_PAGE        0 = last page1 = additional next page(s) follow
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr (0x0000ffe7 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_SIZE 4

/*
 * This structure should be used to declare and program COMBO_AUTONEGNP.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_s {
	uint32_t v[1];
	uint32_t combo_autonegnp[1];
	uint32_t _combo_autonegnp;
} BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_t;

#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_CLR(r) (r).combo_autonegnp[0] = 0
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_SET(r,d) (r).combo_autonegnp[0] = d
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_GET(r) (r).combo_autonegnp[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_NEXT_PAGEf_GET(r) ((((r).combo_autonegnp[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_NEXT_PAGEf_SET(r,f) (r).combo_autonegnp[0]=(((r).combo_autonegnp[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_ACKf_GET(r) ((((r).combo_autonegnp[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_ACKf_SET(r,f) (r).combo_autonegnp[0]=(((r).combo_autonegnp[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_MESSAGE_PAGEf_GET(r) ((((r).combo_autonegnp[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_MESSAGE_PAGEf_SET(r,f) (r).combo_autonegnp[0]=(((r).combo_autonegnp[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_ACK2f_GET(r) ((((r).combo_autonegnp[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_ACK2f_SET(r,f) (r).combo_autonegnp[0]=(((r).combo_autonegnp[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_TOGGLEf_GET(r) ((((r).combo_autonegnp[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_TOGGLEf_SET(r,f) (r).combo_autonegnp[0]=(((r).combo_autonegnp[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_MESSAGEf_GET(r) (((r).combo_autonegnp[0]) & 0x7ff)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_MESSAGEf_SET(r,f) (r).combo_autonegnp[0]=(((r).combo_autonegnp[0] & ~((uint32_t)0x7ff)) | (((uint32_t)f) & 0x7ff)) | (0x7ff << 16)

/*
 * These macros can be used to access COMBO_AUTONEGNP.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_AUTONEGNPr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGNPr,(_r._combo_autonegnp))
#define BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGNPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGNPr,(_r._combo_autonegnp)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGNPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGNPr,(_r._combo_autonegnp))
#define BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGNPr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGNPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autonegnp))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGNPr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGNPr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autonegnp))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGNPr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGNPr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_autonegnp))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_AUTONEGNPr BCMI_VIPER_XGXS_COMBO_AUTONEGNPr
#define COMBO_AUTONEGNPr_SIZE BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_SIZE
typedef BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_t COMBO_AUTONEGNPr_t;
#define COMBO_AUTONEGNPr_CLR BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_CLR
#define COMBO_AUTONEGNPr_SET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_SET
#define COMBO_AUTONEGNPr_GET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_GET
#define COMBO_AUTONEGNPr_NEXT_PAGEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_NEXT_PAGEf_GET
#define COMBO_AUTONEGNPr_NEXT_PAGEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_NEXT_PAGEf_SET
#define COMBO_AUTONEGNPr_ACKf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_ACKf_GET
#define COMBO_AUTONEGNPr_ACKf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_ACKf_SET
#define COMBO_AUTONEGNPr_MESSAGE_PAGEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_MESSAGE_PAGEf_GET
#define COMBO_AUTONEGNPr_MESSAGE_PAGEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_MESSAGE_PAGEf_SET
#define COMBO_AUTONEGNPr_ACK2f_GET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_ACK2f_GET
#define COMBO_AUTONEGNPr_ACK2f_SET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_ACK2f_SET
#define COMBO_AUTONEGNPr_TOGGLEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_TOGGLEf_GET
#define COMBO_AUTONEGNPr_TOGGLEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_TOGGLEf_SET
#define COMBO_AUTONEGNPr_MESSAGEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_MESSAGEf_GET
#define COMBO_AUTONEGNPr_MESSAGEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGNPr_MESSAGEf_SET
#define READ_COMBO_AUTONEGNPr BCMI_VIPER_XGXS_READ_COMBO_AUTONEGNPr
#define WRITE_COMBO_AUTONEGNPr BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGNPr
#define MODIFY_COMBO_AUTONEGNPr BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGNPr
#define READLN_COMBO_AUTONEGNPr BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGNPr
#define WRITELN_COMBO_AUTONEGNPr BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGNPr
#define WRITEALL_COMBO_AUTONEGNPr BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGNPr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_AUTONEGNPr'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_AUTONEGLPABIL2
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffe8
 * DESC:     IEEE auto-negotiation link partner next page register
 * RESETVAL: 0x2001 (8193)
 * ACCESS:   R/O
 * FIELDS:
 *     MESSAGE          Message or Unformatted Code Field11'h400 = Over 1G Message Page11'h410 = Remote CuPHY Message Page11'h411 = MDIO Register Write Message Page11'h412 = MDIO Register Read Request Message Page11'h413 = MDIO Register Response Message PageSee IEEE 802.3 Annex 28C for more standard next page detailsSee BRCM-Serdes-AN for BAM specific details
 *     TOGGLE           Opposite value of bit in previous page
 *     ACK2             Acknowledge 2 bit
 *     MESSAGE_PAGE     0 = Unformatted Page1 = Message Page
 *     ACK              Acknowledge bit
 *     NEXT_PAGE        0 = last page1 = additional next page(s) follow
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r (0x0000ffe8 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_SIZE 4

/*
 * This structure should be used to declare and program COMBO_AUTONEGLPABIL2.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_s {
	uint32_t v[1];
	uint32_t combo_autoneglpabil2[1];
	uint32_t _combo_autoneglpabil2;
} BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_t;

#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_CLR(r) (r).combo_autoneglpabil2[0] = 0
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_SET(r,d) (r).combo_autoneglpabil2[0] = d
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_GET(r) (r).combo_autoneglpabil2[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_NEXT_PAGEf_GET(r) ((((r).combo_autoneglpabil2[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_NEXT_PAGEf_SET(r,f) (r).combo_autoneglpabil2[0]=(((r).combo_autoneglpabil2[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_ACKf_GET(r) ((((r).combo_autoneglpabil2[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_ACKf_SET(r,f) (r).combo_autoneglpabil2[0]=(((r).combo_autoneglpabil2[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_MESSAGE_PAGEf_GET(r) ((((r).combo_autoneglpabil2[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_MESSAGE_PAGEf_SET(r,f) (r).combo_autoneglpabil2[0]=(((r).combo_autoneglpabil2[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_ACK2f_GET(r) ((((r).combo_autoneglpabil2[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_ACK2f_SET(r,f) (r).combo_autoneglpabil2[0]=(((r).combo_autoneglpabil2[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_TOGGLEf_GET(r) ((((r).combo_autoneglpabil2[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_TOGGLEf_SET(r,f) (r).combo_autoneglpabil2[0]=(((r).combo_autoneglpabil2[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_MESSAGEf_GET(r) (((r).combo_autoneglpabil2[0]) & 0x7ff)
#define BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_MESSAGEf_SET(r,f) (r).combo_autoneglpabil2[0]=(((r).combo_autoneglpabil2[0] & ~((uint32_t)0x7ff)) | (((uint32_t)f) & 0x7ff)) | (0x7ff << 16)

/*
 * These macros can be used to access COMBO_AUTONEGLPABIL2.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_AUTONEGLPABIL2r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r,(_r._combo_autoneglpabil2))
#define BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGLPABIL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r,(_r._combo_autoneglpabil2)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGLPABIL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r,(_r._combo_autoneglpabil2))
#define BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGLPABIL2r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autoneglpabil2))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGLPABIL2r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_autoneglpabil2))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGLPABIL2r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_autoneglpabil2))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_AUTONEGLPABIL2r BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r
#define COMBO_AUTONEGLPABIL2r_SIZE BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_SIZE
typedef BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_t COMBO_AUTONEGLPABIL2r_t;
#define COMBO_AUTONEGLPABIL2r_CLR BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_CLR
#define COMBO_AUTONEGLPABIL2r_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_SET
#define COMBO_AUTONEGLPABIL2r_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_GET
#define COMBO_AUTONEGLPABIL2r_NEXT_PAGEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_NEXT_PAGEf_GET
#define COMBO_AUTONEGLPABIL2r_NEXT_PAGEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_NEXT_PAGEf_SET
#define COMBO_AUTONEGLPABIL2r_ACKf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_ACKf_GET
#define COMBO_AUTONEGLPABIL2r_ACKf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_ACKf_SET
#define COMBO_AUTONEGLPABIL2r_MESSAGE_PAGEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_MESSAGE_PAGEf_GET
#define COMBO_AUTONEGLPABIL2r_MESSAGE_PAGEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_MESSAGE_PAGEf_SET
#define COMBO_AUTONEGLPABIL2r_ACK2f_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_ACK2f_GET
#define COMBO_AUTONEGLPABIL2r_ACK2f_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_ACK2f_SET
#define COMBO_AUTONEGLPABIL2r_TOGGLEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_TOGGLEf_GET
#define COMBO_AUTONEGLPABIL2r_TOGGLEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_TOGGLEf_SET
#define COMBO_AUTONEGLPABIL2r_MESSAGEf_GET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_MESSAGEf_GET
#define COMBO_AUTONEGLPABIL2r_MESSAGEf_SET BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r_MESSAGEf_SET
#define READ_COMBO_AUTONEGLPABIL2r BCMI_VIPER_XGXS_READ_COMBO_AUTONEGLPABIL2r
#define WRITE_COMBO_AUTONEGLPABIL2r BCMI_VIPER_XGXS_WRITE_COMBO_AUTONEGLPABIL2r
#define MODIFY_COMBO_AUTONEGLPABIL2r BCMI_VIPER_XGXS_MODIFY_COMBO_AUTONEGLPABIL2r
#define READLN_COMBO_AUTONEGLPABIL2r BCMI_VIPER_XGXS_READLN_COMBO_AUTONEGLPABIL2r
#define WRITELN_COMBO_AUTONEGLPABIL2r BCMI_VIPER_XGXS_WRITELN_COMBO_AUTONEGLPABIL2r
#define WRITEALL_COMBO_AUTONEGLPABIL2r BCMI_VIPER_XGXS_WRITEALL_COMBO_AUTONEGLPABIL2r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_AUTONEGLPABIL2r'
 ******************************************************************************/




/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  COMBO_MIIEXTSTAT
 * BLOCKS:   COMBO_IEEE0
 * REGADDR:  0xffef
 * DESC:     IEEE MII extended status register
 * RESETVAL: 0x0 (0)
 * ACCESS:   R/O
 * FIELDS:
 *     S1000BASE_T_HALF_DUPLEX_CAPABLE 1 = 1000Base-T half duplex capable0 = not 1000Base-T half duplex capable
 *     S1000BASE_T_FULL_DUPLEX_CAPABLE 1 = 1000Base-T full duplex capable0 = not 1000Base-T full duplex capable
 *     S1000BASE_X_HALF_DUPLEX_CAPABLE 1 = 1000Base-X half duplex capable0 = not 1000Base-X half duplex capable
 *     S1000BASE_X_FULL_DUPLEX_CAPABLE 1 = 1000Base-X full duplex capable0 = not 1000Base-X full duplex capable
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr (0x0000ffef | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_SIZE 4

/*
 * This structure should be used to declare and program COMBO_MIIEXTSTAT.
 *
 */
typedef union BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_s {
	uint32_t v[1];
	uint32_t combo_miiextstat[1];
	uint32_t _combo_miiextstat;
} BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_t;

#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_CLR(r) (r).combo_miiextstat[0] = 0
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_SET(r,d) (r).combo_miiextstat[0] = d
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_GET(r) (r).combo_miiextstat[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miiextstat[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miiextstat[0]=(((r).combo_miiextstat[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miiextstat[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miiextstat[0]=(((r).combo_miiextstat[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miiextstat[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miiextstat[0]=(((r).combo_miiextstat[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_GET(r) ((((r).combo_miiextstat[0]) >> 12) & 0x1)
#define BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_SET(r,f) (r).combo_miiextstat[0]=(((r).combo_miiextstat[0] & ~((uint32_t)0x1 << 12)) | ((((uint32_t)f) & 0x1) << 12)) | (1 << (16 + 12))

/*
 * These macros can be used to access COMBO_MIIEXTSTAT.
 *
 */
#define BCMI_VIPER_XGXS_READ_COMBO_MIIEXTSTATr(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr,(_r._combo_miiextstat))
#define BCMI_VIPER_XGXS_WRITE_COMBO_MIIEXTSTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr,(_r._combo_miiextstat)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_COMBO_MIIEXTSTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr,(_r._combo_miiextstat))
#define BCMI_VIPER_XGXS_READLN_COMBO_MIIEXTSTATr(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_miiextstat))
#define BCMI_VIPER_XGXS_WRITELN_COMBO_MIIEXTSTATr(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._combo_miiextstat))
#define BCMI_VIPER_XGXS_WRITEALL_COMBO_MIIEXTSTATr(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._combo_miiextstat))

/*
 * Unless PHYMOD_EXCLUDE_CHIPLESS_TYPES is defined, all of the above types
 * will be redefined without the chip prefix for easier programming.
 * If multiple chips will be programmed in the same source file, then you should
 * define PHYMOD_EXCLUDE_CHIPLESS_TYPES before including all chip header files
 * and refer to the fully qualified versions.
 *
 */
#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define COMBO_MIIEXTSTATr BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr
#define COMBO_MIIEXTSTATr_SIZE BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_SIZE
typedef BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_t COMBO_MIIEXTSTATr_t;
#define COMBO_MIIEXTSTATr_CLR BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_CLR
#define COMBO_MIIEXTSTATr_SET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_SET
#define COMBO_MIIEXTSTATr_GET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_GET
#define COMBO_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_GET
#define COMBO_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_X_FULL_DUPLEX_CAPABLEf_SET
#define COMBO_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_GET
#define COMBO_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_X_HALF_DUPLEX_CAPABLEf_SET
#define COMBO_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_GET
#define COMBO_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_T_FULL_DUPLEX_CAPABLEf_SET
#define COMBO_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_GET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_GET
#define COMBO_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_SET BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr_S1000BASE_T_HALF_DUPLEX_CAPABLEf_SET
#define READ_COMBO_MIIEXTSTATr BCMI_VIPER_XGXS_READ_COMBO_MIIEXTSTATr
#define WRITE_COMBO_MIIEXTSTATr BCMI_VIPER_XGXS_WRITE_COMBO_MIIEXTSTATr
#define MODIFY_COMBO_MIIEXTSTATr BCMI_VIPER_XGXS_MODIFY_COMBO_MIIEXTSTATr
#define READLN_COMBO_MIIEXTSTATr BCMI_VIPER_XGXS_READLN_COMBO_MIIEXTSTATr
#define WRITELN_COMBO_MIIEXTSTATr BCMI_VIPER_XGXS_WRITELN_COMBO_MIIEXTSTATr
#define WRITEALL_COMBO_MIIEXTSTATr BCMI_VIPER_XGXS_WRITEALL_COMBO_MIIEXTSTATr

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_COMBO_MIIEXTSTATr'
 ******************************************************************************/

/* IEEE_PCS0 registers */

/*******************************************************************************
 * CHIP:  BCMI_VIPER_XGXS
 * REGISTER:  DEV3_PCS_CTL1
 * BLOCKS:   DEV3_IEEE0
 * REGADDR:  0x1800_0000
 * DESC:     DEV3 IEEE0 pcs control register
 * RESETVAL: 0x2040 (8256)
 * ACCESS:   R/W
 * FIELDS:
 *     SPEEDSELECTION2          Bits 5 4 3 2 ---- 1 x x x = Reserved0 1 1 x = Reserved0 1 0 1 = Reserved0 1 0 0 = 100Gb/s0 0 1 1 =40Gb/s0 0 1 1 = Reserved for 802.av0 0 0 1 = 10PASS-TS/2BASE-TL0 0 0 0 = 10 Gb/s 
 *     SPEEDSELECTION1          Bits6 13----1 1 = Operation @ 10Gbps and above1 0 = Unspecified0 1 = Unspecified0 0 = Unspecified 
 *     PWRDWN_SW_10G            1 = low-power mode0 = Normal operation
 *     SPEEDSELECTION0          Bits6 13----1 1 = Operation @ 10Gbps and above1 0 = Unspecified0 1 = Unspecified0 0 = Unspecified 
 *     GLOOP10G                 1 = Enable loopback mode0 = Disable loopback mode      
 *     RST_SW                   1 = PCS reset0 = Normal opration 
 *
 ******************************************************************************/
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r (0x18000000 | PHYMOD_REG_ACC_TSC_IBLK)

#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SIZE 4

/*
 * This structure should be used to declare and program DEV3_PCS_CTL1.
 *
 */
typedef union BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_s {
        uint32_t v[1];
        uint32_t dev3_pcs_ctl1[1];
        uint32_t _dev3_pcs_ctl1;
} BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_t;

#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_CLR(r) (r).dev3_pcs_ctl1[0] = 0
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SET(r,d) (r).dev3_pcs_ctl1[0] = d
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_GET(r) (r).dev3_pcs_ctl1[0]

/*
 * These macros can be used to access individual fields.
 *
 */
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_RST_SWf_GET(r) ((((r).dev3_pcs_ctl1[0]) >> 15) & 0x1)
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_RST_SWf_SET(r,f) (r).dev3_pcs_ctl1[0]=(((r).dev3_pcs_ctl1[0] & ~((uint32_t)0x1 << 15)) | ((((uint32_t)f) & 0x1) << 15)) | (1 << (16 + 15))
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_GLOOP10Gf_GET(r) ((((r).dev3_pcs_ctl1[0]) >> 14) & 0x1)
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_GLOOP10Gf_SET(r,f) (r).dev3_pcs_ctl1[0]=(((r).dev3_pcs_ctl1[0] & ~((uint32_t)0x1 << 14)) | ((((uint32_t)f) & 0x1) << 14)) | (1 << (16 + 14))
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION0f_GET(r) ((((r).dev3_pcs_ctl1[0]) >> 13) & 0x1)
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION0f_SET(r,f) (r).dev3_pcs_ctl1[0]=(((r).dev3_pcs_ctl1[0] & ~((uint32_t)0x1 << 13)) | ((((uint32_t)f) & 0x1) << 13)) | (1 << (16 + 13))
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_PWRDWN_SW_10Gf_GET(r) ((((r).dev3_pcs_ctl1[0]) >> 11) & 0x1)
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_PWRDWN_SW_10Gf_SET(r,f) (r).dev3_pcs_ctl1[0]=(((r).dev3_pcs_ctl1[0] & ~((uint32_t)0x1 << 11)) | ((((uint32_t)f) & 0x1) << 11)) | (1 << (16 + 11))
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION1f_GET(r) ((((r).dev3_pcs_ctl1[0]) >> 6) & 0x1)
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION1f_SET(r,f) (r).dev3_pcs_ctl1[0]=(((r).dev3_pcs_ctl1[0] & ~((uint32_t)0x1 << 6)) | ((((uint32_t)f) & 0x1) << 6)) | (1 << (16 + 6))
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION2f_GET(r) ((((r).dev3_pcs_ctl1[0]) >> 2) & 0xf)
#define BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION2f_SET(r,f) (r).dev3_pcs_ctl1[0]=(((r).dev3_pcs_ctl1[0] & ~((uint32_t)0xf << 2)) | ((((uint32_t)f) & 0xf) << 2)) | (1 << (16 + 2))


/*
 * These macros can be used to access DEV3_PCS_CTL1.
 *
 */
#define BCMI_VIPER_XGXS_READ_DEV3_PCS_CTL1r(_pc,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DEV3_PCS_CTL1r,(_r._dev3_pcs_ctl1))
#define BCMI_VIPER_XGXS_WRITE_DEV3_PCS_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DEV3_PCS_CTL1r,(_r._dev3_pcs_ctl1)&0xffff)
#define BCMI_VIPER_XGXS_MODIFY_DEV3_PCS_CTL1r(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DEV3_PCS_CTL1r,(_r._dev3_pcs_ctl1))
#define BCMI_VIPER_XGXS_READLN_DEV3_PCS_CTL1r(_pc,_l,_r) phymod_tsc_iblk_read(_pc,BCMI_VIPER_XGXS_DEV3_PCS_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._dev3_pcs_ctl1))
#define BCMI_VIPER_XGXS_WRITELN_DEV3_PCS_CTL1r(_pc,_l,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DEV3_PCS_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_FORCE_LANE|LSHIFT32(((_l)&0x3),PHYMOD_REG_ACCESS_FLAGS_SHIFT),(_r._DEV3_PCS_CTL1))
#define BCMI_VIPER_XGXS_WRITEALL_DEV3_PCS_CTL1(_pc,_r) phymod_tsc_iblk_write(_pc,BCMI_VIPER_XGXS_DEV3_PCS_CTL1r|PHYMOD_REG_ACC_TSC_IBLK_BCAST,(_r._dev3_pcs_ctl1))


#ifndef PHYMOD_EXCLUDE_CHIPLESS_TYPES

#define DEV3_PCS_CTL1r BCMI_VIPER_XGXS_DEV3_PCS_CTL1r
#define DEV3_PCS_CTL1r_SIZE BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SIZE
typedef BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_t DEV3_PCS_CTL1r_t;
#define DEV3_PCS_CTL1r_CLR BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_CLR
#define DEV3_PCS_CTL1r_SET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SET
#define DEV3_PCS_CTL1r_GET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_GET
#define DEV3_PCS_CTL1r_RST_SWf_GET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_RST_HWf_GET
#define DEV3_PCS_CTL1r_RST_SWf_SET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_RST_SWf_SET
#define DEV3_PCS_CTL1r_GLOOP10Gf_GET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_GLOOP10Gf_GET
#define DEV3_PCS_CTL1r_GLOOP10Gf_SET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_GLOOP10Gf_SET
#define DEV3_PCS_CTL1r_SPEEDSELECTION0f_GET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION0f_GET
#define DEV3_PCS_CTL1r_SPEEDSELECTION0f_SET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION0f_SET
#define DEV3_PCS_CTL1r_PWRDWN_SW_10Gf_GET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_PWRDWN_SW_10Gf_GET
#define DEV3_PCS_CTL1r_PWRDWN_SW_10Gf_SET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_PWRDWN_SW_10Gf_SET
#define DEV3_PCS_CTL1r_SPEEDSELECTION1f_GET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION1f_GET
#define DEV3_PCS_CTL1r_SPEEDSELECTION1f_SET BCMI_VIPER_XGXS_DEV3_PCS_CTL1r_SPEEDSELECTION1f_SET
#define READ_DEV3_PCS_CTL1r BCMI_VIPER_XGXS_READ_DEV3_PCS_CTL1r
#define WRITE_DEV3_PCS_CTL1r BCMI_VIPER_XGXS_WRITE_DEV3_PCS_CTL1r
#define MODIFY_DEV3_PCS_CTL1r BCMI_VIPER_XGXS_MODIFY_DEV3_PCS_CTL1r
#define READLN_DEV3_PCS_CTL1r BCMI_VIPER_XGXS_READLN_DEV3_PCS_CTL1r
#define WRITELN_DEV3_PCS_CTL1r BCMI_VIPER_XGXS_WRITELN_DEV3_PCS_CTL1r
#define WRITEALL_DEV3_PCS_CTL1r BCMI_VIPER_XGXS_WRITEALL_DEV3_PCS_CTL1r

#endif /* PHYMOD_EXCLUDE_CHIPLESS_TYPES */
/*******************************************************************************
 * End of 'BCMI_VIPER_XGXS_DEV3_PCS_CTL1r'
 ******************************************************************************/




#endif /* __BCMI_VIPER_XGXS_DEFS_H__ */
