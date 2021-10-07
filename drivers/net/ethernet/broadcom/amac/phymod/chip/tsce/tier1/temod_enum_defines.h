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


#ifndef _TSCMOD_ENUM_DEFINES_H
#define _TSCMOD_ENUM_DEFINES_H

/*! \enum temod_core_select 

temod_core_select selects for programming, any combination of 3 core of the PHY.
This is only applicable to the 12x10 version of the TSCE core.
A '1' in the latter part of the enum selects the core.


*/

typedef enum {
  TEMOD_CORE_0_0_1           = 0   ,  /*!< Core number  0       selected   */
  TEMOD_CORE_0_1_0                 ,  /*!< Core number  1       selected   */
  TEMOD_CORE_1_0_0                 ,  /*!< Core number  2       selected   */
  TEMOD_CORE_0_1_1                 ,  /*!< Core number  0 & 1   selected   */
  TEMOD_CORE_1_0_1                 ,  /*!< Core number  0 & 2   selected   */
  TEMOD_CORE_1_1_1                 ,  /*!< Core number  0,1,& 2 selected   */
  TEMOD_CORE_ILLEGAL                 /*!< Illegal (programmatic boundary) */
} temod_core_select;

/*! \def CNT_temod_core_select Types of enum temod_core_select */
#define CNT_temod_core_select 7

/*!
\brief
This array returns the string version of the enum #temod_core_select when
indexed by the enum var.

*/
extern char* e2s_temod_core_select [CNT_temod_core_select];
/*!
\brief
This array associates the enum #temod_core_select enum with a bit mask.
The index is the #temod_core_select enum value.  The value for each entry is
interpreted as follows.  If bit [n] is 1, then core [n] is addressed; if bit [n]
is 0, then core [n] is addressed.

*/
extern int e2n_temod_core_select [CNT_temod_core_select];
/*! \enum temod_lane_select 

temod_lane_select selects for programming, any combination of 4 lanes of PHY.  A
'1' in the latter part of the enum name selects the lane.  Other parameters will
decide what to do with the selected lane(s). In most cases we enable/disable a
feature on the selected lanes.

If lane_select is set to TEMOD_LANE_BCST for writes we broadcast to all lanes.
You cannot read in broadcast mode.


*/

typedef enum {
  TEMOD_LANE_0_0_0_1         = 0   ,  /*!< lane number  0       selected   */
  TEMOD_LANE_0_0_1_0               ,  /*!< lane number  1       selected   */
  TEMOD_LANE_0_0_1_1               ,  /*!< lane numbers 0,1     selected   */
  TEMOD_LANE_0_1_0_0               ,  /*!< lane number  2       selected   */
  TEMOD_LANE_0_1_0_1               ,  /*!< lane numbers 2,0     selected   */
  TEMOD_LANE_0_1_1_0               ,  /*!< lane numbers 2,1     selected   */
  TEMOD_LANE_0_1_1_1               ,  /*!< lane numbers 2,1,0   selected   */
  TEMOD_LANE_1_0_0_0               ,  /*!< lane number  3       selected   */
  TEMOD_LANE_1_0_0_1               ,  /*!< lane numbers 3,0     selected   */
  TEMOD_LANE_1_0_1_0               ,  /*!< lane numbers 3,1     selected   */
  TEMOD_LANE_1_0_1_1               ,  /*!< lane numbers 3,1,0   selected   */
  TEMOD_LANE_1_1_0_0               ,  /*!< lane numbers 3,2     selected   */
  TEMOD_LANE_1_1_0_1               ,  /*!< lane numbers 3,2,0   selected   */
  TEMOD_LANE_1_1_1_0               ,  /*!< lane numbers 3,2,1   selected   */
  TEMOD_LANE_1_1_1_1               ,  /*!< lane numbers 3,2,1,0 selected   */
  TEMOD_LANE_BCST                  ,  /*!< lane numbers 3,2,1,0 BCST       */
  TEMOD_LANE_ILLEGAL                 /*!< Illegal (programmatic boundary) */
} temod_lane_select;

/*! \def CNT_temod_lane_select Types of enum temod_lane_select */
#define CNT_temod_lane_select 17

/*!
\brief
This array returns the string version of the enum #temod_lane_select when
indexed by the enum var.

*/
extern char* e2s_temod_lane_select [CNT_temod_lane_select];
/*!
\brief
This array associates the enum #temod_lane_select enum with a bit mask.
The index is the #temod_lane_select enum value.  The value for each entry is
interpreted as follows.  If bit [n] is 1, then lane [n] is enabled; if bit [n]
is 0, then lane [n] is disabled.  By enabled, we mean that such-and-such
function is to be called for a lane; by disabled, we mean that such-and-such
function is not to be called for a lane.  So a value of 0xF indicates that all
lanes or enabled, while 0x5 indicates that only lanes 0 and 2 are enabled.

*/
extern int e2n_temod_lane_select [CNT_temod_lane_select];
/*! \enum temod_spd_intfc_type 

All legal speed-interface combination are encapsulated in this enum

There are several speed and interface combinations allowed for a logical PHY
port. Names and speeds are self explanatory.

Speed and interface selection is combined because we don't want the speeds
to be incompatible with interface.

*/

typedef enum {
  TEMOD_SPD_ZERO             = 0   ,  /*!< Illegal value (enum boundary)   */
  TEMOD_SPD_10_X1_SGMII            ,  /*!< 10Mb SGMII (serial)             */
  TEMOD_SPD_100_X1_SGMII           ,  /*!< 100Mb SGMII (serial)            */
  TEMOD_SPD_1000_X1_SGMII          ,  /*!< 1Gb SGMII (serial)              */
  TEMOD_SPD_2500_X1                ,  /*!< 2.5Gb  based on 1000BASE-X      */
  TEMOD_SPD_10_SGMII               ,  /*!< 10Mb SGMII (serial)             */
  TEMOD_SPD_100_SGMII              ,  /*!< 100Mb SGMII (serial)            */
  TEMOD_SPD_1000_SGMII             ,  /*!< 1Gb SGMII (serial)              */
  TEMOD_SPD_2500                   ,  /*!< 2.5Gb  based on 1000BASE-X      */
  TEMOD_SPD_5000                   ,  /*!< 5Gb  CL36                       */
  TEMOD_SPD_1000_XFI               ,  /*!< 1Gb                             */
  TEMOD_SPD_5000_XFI               ,  /*!< 5Gb  CL49                       */
  TEMOD_SPD_10000_XFI              ,  /*!< 10Gb serial XFI                 */
  TEMOD_SPD_10600_XFI_HG           ,  /*!< 10.5Gb serial XFI (HgSOLO)      */
  TEMOD_SPD_10000_HI               ,  /*!< 10Gb XAUI HiG                   */
  TEMOD_SPD_10000                  ,  /*!< 10Gb XAUI                       */
  TEMOD_SPD_12000_HI               ,  /*!< 12Gb XAUI HiG                   */
  TEMOD_SPD_13000                  ,  /*!< 13Gb XAUI                       */
  TEMOD_SPD_15000                  ,  /*!< 15Gb XAUI                       */
  TEMOD_SPD_16000                  ,  /*!< 16Gb XAUI                       */
  TEMOD_SPD_20000                  ,  /*!< 20Gb XAUI                       */
  TEMOD_SPD_20000_SCR              ,  /*!< 20Gb XAUI scrambled             */
  TEMOD_SPD_21000                  ,  /*!< 21Gb XAUI                       */
  TEMOD_SPD_25455                  ,  /*!< 25Gb XAUI  64/66 codec          */
  TEMOD_SPD_31500                  ,  /*!< 31.5Gb quad lane XAUI           */
  TEMOD_SPD_31500_MLD              ,  /*!< 31.5Gb quad lane MLD            */
  TEMOD_SPD_40G_X4                 ,  /*!< 40Gb quad lane XAUI             */
  TEMOD_SPD_42G_X4                 ,  /*!< 40Gb quad lane XAUI  HiG        */
  TEMOD_SPD_40G_XLAUI              ,  /*!< 40Gb quad lane  MLD             */
  TEMOD_SPD_42G_XLAUI              ,  /*!< 42Gb quad lane  MLD             */
  TEMOD_SPD_10000_X2               ,  /*!< 10Gb dual lane                  */
  TEMOD_SPD_10000_HI_DXGXS         ,  /*!< 10Gb dual lane XGXS HiG         */
  TEMOD_SPD_10000_DXGXS            ,  /*!< 10Gb dual lane XGXS             */
  TEMOD_SPD_10000_HI_DXGXS_SCR       ,  /*!< 10Gb dual lane,scrambled,HiG    */
  TEMOD_SPD_10000_DXGXS_SCR        ,  /*!< 10Gb dual lane scrambled        */
  TEMOD_SPD_10500_HI               ,  /*!< 10.5Gb XAUI  lane XGXS HiG      */
  TEMOD_SPD_10500_HI_DXGXS         ,  /*!< 10.5Gb  dual lane XGXS HiG      */
  TEMOD_SPD_12773_HI_DXGXS         ,  /*!< 12.73Gb dual lane XGXS HiG      */
  TEMOD_SPD_12773_DXGXS            ,  /*!< 12.73Gb dual lane XGXS          */
  TEMOD_SPD_15750_HI_DXGXS         ,  /*!< 15.75Gb scrambled dual lane HiG */
  TEMOD_SPD_20G_MLD_DXGXS          ,  /*!< 20Gb dual lane MLD              */
  TEMOD_SPD_21G_HI_MLD_DXGXS       ,  /*!< 20Gb dual lane HiG MLD          */
  TEMOD_SPD_20G_DXGXS              ,  /*!< 20Gb dual lane BRCM             */
  TEMOD_SPD_21G_HI_DXGXS           ,  /*!< 21.2Gb dual HiG(20+plldiv=70)   */
  TEMOD_SPD_100G_CR10              ,  /*!< 100G                            */
  TEMOD_SPD_107G_HG_CR10           ,  /*!< 107G                            */
  TEMOD_SPD_120G_CR12              ,  /*!< 120G                            */
  TEMOD_SPD_127G_HG_CR12           ,  /*!< 127G                            */
  TEMOD_SPD_4000                   ,  /*!< 4G QSGMII                       */
  TEMOD_SPD_ILLEGAL                  /*!< Illegal value (enum boundary)   */
} temod_spd_intfc_type;

/*! \def CNT_temod_spd_intfc_type Types of enum temod_spd_intfc_type */
#define CNT_temod_spd_intfc_type 50

/*!
\brief
This array returns the string version of the enum #temod_lane_select when
indexed by the enum var.

*/
extern char* e2s_temod_spd_intfc_type [CNT_temod_spd_intfc_type];
/*!
\brief
This array associates the enum #temod_lane_select enum with a bit mask.
The index is the #temod_lane_select enum value.  The value for each entry is
interpreted as follows.  If bit [n] is 1, then lane [n] is enabled; if bit [n]
is 0, then lane [n] is disabled.  By enabled, we mean that such-and-such
function is to be called for a lane; by disabled, we mean that such-and-such
function is not to be called for a lane.  So a value of 0xF indicates that all
lanes or enabled, while 0x5 indicates that only lanes 0 and 2 are enabled.

*/
extern int e2n_temod_spd_intfc_type [CNT_temod_spd_intfc_type];
/*! \enum temod_regacc_type 

Types of MDIO to access PHY registers. IEEE clause 22 and clause 45 are
supported. The 8 bit parallel TOTSC bus can be used. We may use 

*/

typedef enum {
  TEMOD_REGACC_CL22          = 0   ,  /*!< IEEE clause 22 based MDIO (for PMD only) */
  TEMOD_REGACC_CL45                ,  /*!< IEEE clause 45 based MDIO (for PMD only) */
  TEMOD_REGACC_TOTSC               ,  /*!< Mission Mode. */
  TEMOD_REGACC_SBUS_FD             ,  /*!< Probably used in PM testbenches. */
  TEMOD_REGACC_SBUS_BD             ,  /*!< Probably used in PM testbenches. */
  TEMOD_REGACC_ILLEGAL               /*!< Illegal value (enum boundary) */
} temod_regacc_type;

/*! \def CNT_temod_regacc_type Types of enum temod_regacc_type */
#define CNT_temod_regacc_type 6

/*!
\brief
This array returns the string version of the enum #temod_regacc_type when indexed
by the enum var.

*/
extern char* e2s_temod_regacc_type [CNT_temod_regacc_type];
/*! \enum temod_port_type 

This is the port mode type enumeration.

WC can be configured in combo mode (i.e. entire WC is a single port) or
independent mode (i.e. WC has more than one port (2, 3, or 4) that are
controlled individually.

*/

typedef enum {
  TEMOD_SINGLE_PORT          = 0   ,  /*!< single port mode: 4 channels as one logical port */
  TEMOD_MULTI_PORT                 ,  /*!< Each channel is one logical port */
  TEMOD_DXGXS                      ,  /*!< Each paired channel(0-1, 2-3) is one logical port */
  TEMOD_TRI1_PORT                  ,  /*!< 3 ports, one of them paird as follows (0,1,2-3) */
  TEMOD_TRI2_PORT                  ,  /*!< 3 ports, one of them paird as follows (0-1,2,3) */
  TEMOD_PORT_MODE_ILLEGAL            /*!< Illegal value (enum boundary) */
} temod_port_type;

/*! \def CNT_temod_port_type Types of enum temod_port_type */
#define CNT_temod_port_type 6

/*!
\brief
This array returns the string version of the enum #temod_port_type when indexed
by the enum var.

*/
extern char* e2s_temod_port_type [CNT_temod_port_type];
/*! \enum temod_diag_type 

temod_diag_type enumerates categories of diagnostic data.
has many intermediate stages between down and up. This enum is work in progress


<table cellspacing=0>
<tr><td colspan=3><B>'per_lane_control' bit-mappings</B></td></tr>

<tr><td><B>Type</B></td><td><B>Description</B></td><td><B>Scope</B></td></tr>

<tr><td>General</td>
<td> Combo/independent, Device and Revision Id, VCO settings, Firmware state
and version, active/passive lanes, MDIO type. PLL info, Oversampling Info</td>
<td>Device</td></tr>

<tr><td>Link</td>
<td> Speeds, oversampling, interface, forced/Autoneg, link status, sync
status, RX sequencer on/off </td>
<td>Lane</td></tr>

<tr><td>Autoneg</td>
<td> Local and remote advertisement, link status, cl73/37/BAM info </td>
<td>Lane</td></tr>

<tr><td>Internal Traffic</td>
<td> PRBS type, CJPat Type, Prog_data value, Any associated recorded errors,
and misc. info (IPG etc.)</td>
<td>Lane </td></tr>

<tr>
<td>DFE</td>
<td>Equalization info, Tap settings. (pre/post/overrides), peaking filter values </td>
<td>Lane</td></tr>

<tr><td>IEEE info</td>
<td>Clause 72, FEC</td>
<td>Lane</td></tr>

<tr><td>Topology</td>
<td>Looping info (Gloop/rloop), lane swapping, polarity swap info</td>
<td>Device</td></tr>

<tr><td>EEE</td>
<td>EEE full, passthru modes, some window values?</td>
<td>Lane</td></tr>

<tr><td>Eye Margin</td>
<td>Eye margin measurement (readout only)</td>
<td>Lane</td></tr>

<tr><td>All</td>
<td>All of the above. Except eye margin.</td>
<td>Device</td></tr>

</table>


*/

typedef enum {
  TEMOD_DIAG_GENERAL         = 0x00000001 ,  /*!< General device wide information.         */
  TEMOD_DIAG_TOPOLOGY        = 0x00000002 ,  /*!< Loopbacks etc.                           */
  TEMOD_DIAG_LINK            = 0x00000004 ,  /*!< Link specific info.                      */
  TEMOD_DIAG_SPEED           = 0x00000008 ,  /*!< sub-category of TEMOD_DIAG_LINK(for SDK) */
  TEMOD_DIAG_ANEG            = 0x00000010 ,  /*!< Autoneg specific info.                   */
  TEMOD_DIAG_TFC             = 0x00000020 ,  /*!< State of tx/rx internal tfc              */
  TEMOD_DIAG_AN_TIMERS       = 0x00000040 ,  /*!< AN timers                                */
  TEMOD_DIAG_STATE           = 0x00000080 ,  /*!< Debug state registers                    */
  TEMOD_DIAG_DEBUG           = 0x00000100 ,  /*!< Debug                                    */
  TEMOD_DIAG_IEEE            = 0x00000200 ,  /*!< IEEE related info                        */
  TEMOD_DIAG_EEE             = 0x00000400 ,  /*!< EEE                                      */
  TEMOD_DIAG_BER             = 0x00000800 ,  /*!< BER - PMD bit error rate                 */
  TEMOD_DIAG_CFG             = 0x00001000 ,  /*!< CFG - PMD Lane/Core Config               */
  TEMOD_DIAG_CL72            = 0x00002000 ,  /*!< CL72 - Status                            */
  TEMOD_DIAG_DSC             = 0x00004000 ,  /*!< DSC dump                                 */
  #if 0 /* FIXME: temporarily commented out due PEDANTIC compilation issue,
           instead use a small for compilation issue */
  TEMOD_SERDES_DIAG          = 0x80000000UL ,  /*!< PMD Triage */
  TEMOD_DIAG_ALL             = 0xffffffffUL ,  /*!< Everything but eye margin                */
  #else
  TEMOD_SERDES_DIAG          = 0x00008000 ,  /*!< PMD Triage */
  TEMOD_DIAG_ALL             = 0x0000ffff ,  /*!< Everything but eye margin                */
  #endif
  TEMOD_DIAG_ILLEGAL         = 0x00000000   /*!< Illegal value. programmatic boundary.    */
} temod_diag_type;

/*! \def CNT_temod_diag_type Types of enum temod_diag_type */
#define CNT_temod_diag_type 18

/*!
\brief
This array returns the string version of the enum #temod_port_type when indexed
by the enum var.

*/
extern char* e2s_temod_diag_type [CNT_temod_diag_type];
/*! \enum temod_pmd_diag_type 

temod_pmd_diag_type enumerates categories of pmd diagnostic data.
has many intermediate stages between down and up. This enum is work in progress

*/

typedef enum {
  TEMOD_SERDES_DIAG_CORE_CONFIG = 0x00000001 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_CORE_STATE = 0x00000002 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_LANE_CONFIG = 0x00000004 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_LANE_DEBUG_STATUS = 0x00000008 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_LANE_STATE = 0x00000010 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_LANE_STATE_HDR = 0x00000020 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_LANE_STATE_LEGEND = 0x00000040 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_CL72_STATUS = 0x00000080 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_EYE_DENSITY = 0x00000100 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_EYE_SCAN = 0x00000200 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_MEAS_EYE_DENSITY_DATA = 0x00000400 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_MEAS_EYE_SCAN_DONE = 0x00000800 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_MEAS_LOWBER_EYE = 0x00001000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_READ_EYE_SCAN_STRIPE = 0x00002000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_EYE_DENSITY_DATA = 0x00004000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_LOWBER_EYE = 0x00008000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_EYE_SCAN_VERTICAL = 0x00010000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_EYE_SCAN_STRIPE = 0x00020000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_EYE_SCAN_HEADER = 0x00040000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_EYE_SCAN_FOOTER = 0x00080000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_BER_SCAN_TEST = 0x00100000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_READ_BER_SCAN_DATA = 0x00200000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_EYE_MARGIN_PROJ = 0x00400000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_DIG_LPBK = 0x00800000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_MEAS_EYE_SCAN_START = 0x01000000 ,  /*!< SERDES DIAG */
  TEMOD_SERDES_DIAG_ILLEGAL  = 0x00000000   /*!< Illegal value (enum boundary) */
} temod_pmd_diag_type;

/*! \def CNT_temod_pmd_diag_type Types of enum temod_pmd_diag_type */
#define CNT_temod_pmd_diag_type 26

/*!
\brief
This array returns the string version of the enum #temod_pmd_diag_type when indexed
by the enum var.

*/
extern char* e2s_temod_pmd_diag_type [CNT_temod_pmd_diag_type];
/*! \enum temod_eye_direction 

The direction of slicer changing always moves from the middle of the eye.
Currently all speeds have the following bit positions in temod_st

\li TEMOD_EYE_VU: Vertical,   Upward direction
\li TEMOD_EYE_VD: Vertical,   Downward direction
\li TEMOD_EYE_HL: Horizontal, Left  direction
\li TEMOD_EYE_HR: Horizontal, Right direction


*/

typedef enum {
  TEMOD_EYE_VU               = 0   ,  /*!< Vertical,   Upward direction */
  TEMOD_EYE_VD                     ,  /*!< Vertical,   Downward direction */
  TEMOD_EYE_HL                     ,  /*!< Horizontal, Left  direction */
  TEMOD_EYE_HR                     ,  /*!< Horizontal, Right direction */
  TEMOD_EYE_ILLEGAL                  /*!< Programmatic illegal boundary. */
} temod_eye_direction;

/*! \def CNT_temod_eye_direction Types of enum temod_eye_direction */
#define CNT_temod_eye_direction 5

/*!
\brief
This array returns the string version of the enum #temod_pmd_diag_type when indexed
by the enum var.

*/
extern char* e2s_temod_eye_direction [CNT_temod_eye_direction];
/*! \enum temod_tech_ability 

temod_tech_ability enumerates different types of speed advertisements in the
basic autoneg page. Currently for CL73 only.

Currently all speeds have the following bit positions in temod_st

\li TEMOD_ABILITY_1G_KX            Bit Pos: 0
\li TEMOD_ABILITY_10G_KX4          Bit Pos: 1
\li TEMOD_ABILITY_10G_KR           Bit Pos: 2
\li TEMOD_ABILITY_40G_KR4          Bit Pos: 3
\li TEMOD_ABILITY_40G_CR4          Bit Pos: 4
\li TEMOD_ABILITY_100G_CR10        Bit Pos: 5
\li TEMOD_ABILITY_20G_KR2          Bit Pos: 27
\li TEMOD_ABILITY_20G_CR2          Bit Pos: FIXME


*/

typedef enum {
  TEMOD_ABILITY_1G_KX        = 0   ,  /*!< CL73 speed 1G_KX ease        */
  TEMOD_ABILITY_10G_KX4            ,  /*!< please write comments        */
  TEMOD_ABILITY_10G_KR             ,  /*!< please write comments        */
  TEMOD_ABILITY_40G_KR4            ,  /*!< please write comments        */
  TEMOD_ABILITY_40G_CR4            ,  /*!< please write comments        */
  TEMOD_ABILITY_100G_CR10          ,  /*!< please write comments        */
  TEMOD_ABILITY_20G_KR2            ,  /*!< please write comments        */
  TEMOD_ABILITY_20G_CR2            ,  /*!< please write comments        */
  TEMOD_ABILITY_ILLEGAL              /*!< please write comments        */
} temod_tech_ability;

/*! \def CNT_temod_tech_ability Types of enum temod_tech_ability */
#define CNT_temod_tech_ability 9

/*!
\brief
This array returns the string version of the enum #temod_pmd_diag_type when indexed
by the enum var.

*/
extern char* e2s_temod_tech_ability [CNT_temod_tech_ability];
/*!
\brief
This array associates the enum #temod_lane_select enum with a bit mask.
The index is the #temod_lane_select enum value.  The value for each entry is
interpreted as follows.  If bit [n] is 1, then lane [n] is enabled; if bit [n]
is 0, then lane [n] is disabled.  By enabled, we mean that such-and-such
function is to be called for a lane; by disabled, we mean that such-and-such
function is not to be called for a lane.  So a value of 0xF indicates that all
lanes or enabled, while 0x5 indicates that only lanes 0 and 2 are enabled.

*/
extern int e2n_temod_tech_ability [CNT_temod_tech_ability];
/*! \enum temod_cl37bam_ability 

This array associates the enum #temod_lane_select enum with a bit mask.
The index is the #temod_lane_select enum value.  The value for each entry is
interpreted as follows.  If bit [n] is 1, then lane [n] is enabled; if bit [n]
is 0, then lane [n] is disabled.  Enabled implies the function under
consideration be enebled for the lane. Disabled means the opposite.  So a value
of 0xF indicates that all lanes or enabled, while 0x5 indicates that only lanes
0 and 2 are enabled.

temod_cl37bam_ability enumerates different types of speed advertisements in the
basic autoneg page for CL37BAM only.

Currently all speeds have the following bit positions in temod_st

\li TEMOD_BAM37ABL_2P5G             Bit Pos: 0
\li TEMOD_BAM37ABL_5G_X4            Bit Pos: 1
\li TEMOD_BAM37ABL_6G_X4            Bit Pos: 2
\li TEMOD_BAM37ABL_10G_HIGIG        Bit Pos: 3
\li TEMOD_BAM37ABL_10G_CX4          Bit Pos: 4
\li TEMOD_BAM37ABL_12G_X4           Bit Pos: 5
\li TEMOD_BAM37ABL_12P5_X4          Bit Pos: 6
\li TEMOD_BAM37ABL_13G_X4           Bit Pos: 7
\li TEMOD_BAM37ABL_15G_X4           Bit Pos: 8
\li TEMOD_BAM37ABL_16G_X4           Bit Pos: 9
\li TEMOD_BAM37ABL_20G_X4_CX4       Bit Pos: 10
\li TEMOD_BAM37ABL_20G_X4           Bit Pos: 11
\li TEMOD_BAM37ABL_21G_X4           Bit Pos: 12
\li TEMOD_BAM37ABL_25P455G          Bit Pos: 13
\li TEMOD_BAM37ABL_31P5G            Bit Pos: 14
\li TEMOD_BAM37ABL_32P7G            Bit Pos: 15
\li TEMOD_BAM37ABL_40G              Bit Pos: 16
\li TEMOD_BAM37ABL_10G_X2_CX4       Bit Pos: 17
\li TEMOD_BAM37ABL_10G_DXGXS        Bit Pos: 18
\li TEMOD_BAM37ABL_10P5G_DXGXS      Bit Pos: 19
\li TEMOD_BAM37ABL_12P7_DXGXS       Bit Pos: 20
\li TEMOD_BAM37ABL_15P75G_R2        Bit Pos: 21
\li TEMOD_BAM37ABL_20G_X2_CX4       Bit Pos: 22
\li TEMOD_BAM37ABL_20G_X2           Bit Pos: 23


*/

typedef enum {
  TEMOD_BAM37ABL_2P5G        = 0   ,  /*!< X1 BRCM */
  TEMOD_BAM37ABL_5G_X4             ,  /*!< BRCM */
  TEMOD_BAM37ABL_6G_X4             ,  /*!< BRCM */
  TEMOD_BAM37ABL_10G_HIGIG         ,  /*!< HG (10G_X4) */
  TEMOD_BAM37ABL_10G_CX4           ,  /*!< (10G_X4_CX4) */
  TEMOD_BAM37ABL_12G_X4            ,  /*!< HG */
  TEMOD_BAM37ABL_12P5_X4           ,  /*!< HG */
  TEMOD_BAM37ABL_13G_X4            ,  /*!< HG */
  TEMOD_BAM37ABL_15G_X4            ,  /*!< HG */
  TEMOD_BAM37ABL_16G_X4            ,  /*!< HG */
  TEMOD_BAM37ABL_20G_X4_CX4        ,  /*!< XAUI 8b10b scram(20G_X4S) */
  TEMOD_BAM37ABL_20G_X4            ,  /*!< HG */
  TEMOD_BAM37ABL_21G_X4            ,  /*!< HG */
  TEMOD_BAM37ABL_25P455G           ,  /*!< X4 HG */
  TEMOD_BAM37ABL_31P5G             ,  /*!< X4 HG */
  TEMOD_BAM37ABL_32P7G             ,  /*!< X4 HG */
  TEMOD_BAM37ABL_40G               ,  /*!< X4 BRCM HG (40G_X4) */
  TEMOD_BAM37ABL_10G_X2_CX4        ,  /*!< 10G_X2_CX4 */
  TEMOD_BAM37ABL_10G_DXGXS         ,  /*!< 10G_X2 */
  TEMOD_BAM37ABL_10P5G_DXGXS       ,  /*!< 10P5_X2 */
  TEMOD_BAM37ABL_12P7_DXGXS        ,  /*!< 12P7_X2 */
  TEMOD_BAM37ABL_15P75G_R2         ,  /*!< 15P75_X2 */
  TEMOD_BAM37ABL_20G_X2_CX4        ,  /*!< 20G_X2_CX4 */
  TEMOD_BAM37ABL_20G_X2            ,  /*!< 20G_X2 */
  TEMOD_BAM37ABL_ILLEGAL             /*!< Illegal. Programmatic boundary */
} temod_cl37bam_ability;

/*! \def CNT_temod_cl37bam_ability Types of enum temod_cl37bam_ability */
#define CNT_temod_cl37bam_ability 25

/*!
\brief
This array returns the string version of the enum #temod_pmd_diag_type when indexed
by the enum var.

*/
extern char* e2s_temod_cl37bam_ability [CNT_temod_cl37bam_ability];
/*! \enum temod_diag_an_type 

This array associates the enum #temod_lane_select enum with a bit mask.
The index is the #temod_lane_select enum value.  The value for each entry is
interpreted as follows.  If bit [n] is 1, then lane [n] is enabled; if bit [n]
is 0, then lane [n] is disabled.  By enabled, we mean that such-and-such
function is to be called for a lane; by disabled, we mean that such-and-such
function is not to be called for a lane.  So a value of 0xF indicates that all
lanes or enabled, while 0x5 indicates that only lanes 0 and 2 are enabled.

*/

typedef enum {
  TEMOD_DIAG_AN_DONE         = 0   ,  /*!< AN completion check */
  TEMOD_DIAG_AN_HCD                ,  /*!< AN HCD speed check */
  TEMOD_DIAG_AN_TYPE_ILLEGAL         /*!< Illegal value. programmatic boundary */
} temod_diag_an_type;

/*! \def CNT_temod_diag_an_type Types of enum temod_diag_an_type */
#define CNT_temod_diag_an_type 3

/*!
\brief
This array returns the string version of the enum #temod_pmd_diag_type when indexed
by the enum var.

*/
extern char* e2s_temod_diag_an_type [CNT_temod_diag_an_type];
/*! \enum temod_tier1_function_type 

This array associates the enum #temod_lane_select enum with a bit mask.
The index is the #temod_lane_select enum value.  The value for each entry is
interpreted as follows.  If bit [n] is 1, then lane [n] is enabled; if bit [n]
is 0, then lane [n] is disabled.  By enabled, we mean that such-and-such
function is to be called for a lane; by disabled, we mean that such-and-such
function is not to be called for a lane.  So a value of 0xF indicates that all
lanes or enabled, while 0x5 indicates that only lanes 0 and 2 are enabled.

*/

typedef enum {
  PCS_BYPASS_CTL             = 0   ,  /*!< temod_pcs_bypass_ctl  */
  INIT_PCS                         ,  /*!< temod_init_pcs */
  INIT_PMD_SW                      ,  /*!< temod_init_pmd_sw */
  INIT_PMD                         ,  /*!< temod_init_pmd */
  PMD_RESET_SEQ                    ,  /*!< temod_pmd_reset_seq */
  PMD_X4_RESET                     ,  /*!< temod_pmd_x4_reset */
  WAIT_SC_DONE                     ,  /*!< temod_wait_sc_done */
  ENABLE_SET                       ,  /*!< temod_enable_set */
  FECMODE_SET                      ,  /*!< temod_fecmode_set */
  GET_PCS_LINK_STATUS              ,  /*!< temod_get_pcs_link_status */
  OVERRIDE_SET                     ,  /*!< temod_override_set   */
  CREDIT_OVERRIDE_SET              ,  /*!< temod_credit_override_set   */
  ENCODE_SET                       ,  /*!< temod_encode_set   */
  DECODE_SET                       ,  /*!< temod_decode_set */
  CREDIT_CONTROL                   ,  /*!< temod_credit_control */
  TX_LANE_CONTROL                  ,  /*!< temod_tx_lane_control */
  RX_LANE_CONTROL                  ,  /*!< temod_rx_lane_control */
  TX_LANE_DISABLE                  ,  /*!< temod_tx_lane_disable */
  POWER_CONTROL                    ,  /*!< temod_power_control */
  AUTONEG_SET                      ,  /*!< temod_autoneg_set */
  AUTONEG_GET                      ,  /*!< temod_autoneg_get */
  AUTONEG_CONTROL                  ,  /*!< temod_autoneg_control */
  TSC12_CONTROL                    ,  /*!< temod_tsc12_control */
  CHECK_SC_STATS                   ,  /*!< temod_check_sc_stats */
  REG_READ                         ,  /*!< temod_reg_read */
  REG_WRITE                        ,  /*!< temod_reg_write */
  PRBS_CHECK                       ,  /*!< temod_prbs_check */
  CJPAT_CRPAT_MODE_SET             ,  /*!< temod_cjpat_crpat_mode_set */
  CJPAT_CRPAT_CHECK                ,  /*!< temod_cjpat_crpat_check */
  TEMOD_DIAG                       ,  /*!< temod_diag */
  PCS_LANE_SWAP                    ,  /*!< temod_pcs_lane_swap */
  PMD_LANE_SWAP                    ,  /*!< temod_pmd_lane_swap */
  PMD_LANE_SWAP_TX                 ,  /*!< temod_pmd_lane_swap_tx */
  PARALLEL_DETECT_CONTROL          ,  /*!< temod_parallel_detect_control */
  REVID_READ                       ,  /*!< temod_revid_read */
  SOFT_RESET                       ,  /*!< temod_soft_reset */
  SET_SPD_INTF                     ,  /*!< temod_set_spd_intf */
  SET_PORT_MODE                    ,  /*!< temod_set_port_mode */
  EEE_PASSTHRU_SET                 ,  /*!< temod_EEE_pass_thru_set */
  EEE_PASSTHRU_GET                 ,  /*!< temod_EEE_pass_thru_get */
  FEC_CONTROL                      ,  /*!< temod_FEC_control */
  HIGIG2_CONTROL                   ,  /*!< temod_HIGIG2_control */
  LF_RF_CONTROL                    ,  /*!< temod_LF_RF_control */
  RX_FAST_LOCK_CONTROL             ,  /*!< temod_rx_fast_lock_control */
  PMD_CORE_RESET                   ,  /*!< temod_pmd_core_reset */
  PMD_LANE_RESET                   ,  /*!< temod_pmd_lane_reset */
  AFE_SPEED_UP_DSC_VGA             ,  /*!< temod_afe_speed_up_dsc_vga */
  CLAUSE_72_CONTROL                ,  /*!< temod_clause72_control */
  PLL_SEQUENCER_CONTROL            ,  /*!< temod_pll_sequencer_control */
  PLL_LOCK_WAIT                    ,  /*!< temod_pll_lock_wait */
  PROG_DATA                        ,  /*!< temod_prog_data */
  RX_SEQ_CONTROL                   ,  /*!< temod_rx_seq_control */
  SET_POLARITY                     ,  /*!< temod_tx_rx_polarity */
  PRBS_CONTROL                     ,  /*!< temod_prbs_control */
  PRBS_MODE                        ,  /*!< temod_prbs_mode */
  RX_PF_CONTROL                    ,  /*!< temod_rx_PF_control */
  RX_VGA_CONTROL                   ,  /*!< temod_rx_VGA_control */
  CJPAT_CRPAT_CONTROL              ,  /*!< temod_cjpat_crpat_control */
  TX_LOOPBACK_CONTROL              ,  /*!< temod_tx_loopback_control */
  TX_PMD_LOOPBACK_CONTROL          ,  /*!< temod_tx_pmd_loopback_control */
  RX_LOOPBACK_CONTROL              ,  /*!< temod_rx_loopback_control */
  RX_P1_SLICER_CONTROL             ,  /*!< temod_rx_p1_slicer_control */
  RX_M1_SLICER_CONTROL             ,  /*!< temod_rx_m1_slicer_control */
  RX_D_SLICER_CONTROL              ,  /*!< temod_rx_d_slicer_control */
  WAIT_PMD_LOCK                    ,  /*!< temod_wait_pmd_lock */
  FIRMWARE_SET                     ,  /*!< temod_firmware_set */
  INIT_PMD_QSGMII                  ,  /*!< temod_init_pmd_qsgmii */
  INIT_PCS_ILKN                    ,  /*!< temod_init_pcs_ilkn */
  PMD_RESET_BYPASS                 ,  /*!< temod_pmd_reset_bypass */
  PMD_OVERRIDE_CONTROL             ,  /*!< temod_pmd_override_control */
  TIER1_FUNCTION_ILLEGAL             /*!< always last  */
} temod_tier1_function_type;

/*! \def CNT_temod_tier1_function_type Types of enum temod_tier1_function_type */
#define CNT_temod_tier1_function_type 71

/*!
\brief
This array returns the string version of the enum #temod_pmd_diag_type when indexed
by the enum var.

*/
extern char* e2s_temod_tier1_function_type [CNT_temod_tier1_function_type];
/*! \enum sc_mode_type 

temod_core_select selects for programming, any combination of 3
core of the PHY.
This is only applicable to the 12x10 version of the TSCE core.
A '1' in the latter part of the enum selects the core.


*/

typedef enum {
  TEMOD_SC_MODE_HT_WITH_BASIC_OVERRIDE = 0   ,  /*!< TEMOD_SC_MODE_HT_WITH_BASIC_OVERRIDE    */
  TEMOD_SC_MODE_HT_OVERRIDE        ,  /*!< TEMOD_SC_MODE_HT_OVERRIDE               */
  TEMOD_SC_MODE_ST                 ,  /*!< TEMOD_SC_MODE_ST                        */
  TEMOD_SC_MODE_ST_OVERRIDE        ,  /*!< TEMOD_SC_MODE_ST_OVERRIDE               */
  TEMOD_SC_MODE_AN_CL37            ,  /*!< TEMOD_SC_MODE_AN_CL37                   */
  TEMOD_SC_MODE_AN_CL73            ,  /*!< TEMOD_SC_MODE_AN_CL73                   */
  TEMOD_SC_MODE_BYPASS             ,  /*!< TEMOD_SC_MODE_BYPASS                    */
  TEMOD_SC_MODE_ILLEGAL              /*!< TEMOD_SC_MODE_ILLEGAL                   */
} sc_mode_type;

/*! \def CNT_sc_mode_type Types of enum sc_mode_type */
#define CNT_sc_mode_type 8

/*!
\brief
This array returns the string version of the enum
#temod_core_select when
indexed by the enum var.

*/
extern char* e2s_sc_mode_type [CNT_sc_mode_type];
/*!
\brief
This array associates the enum #temod_core_select enum with a
bit mask.
The index is the #temod_core_select enum value.  The value for
each entry is
interpreted as follows.  If bit [n] is 1, then core [n] is
addressed; if bit [n]
is 0, then core [n] is addressed.

*/
extern int e2n_sc_mode_type [CNT_sc_mode_type];
/*! \enum temod_pll_mode_type 

temod_pll_mode_type divides the PLL to get a clock which is a multiple of the
refernce clock. This clock runs all the digital logic in the MAC and PHY layers
and is referred to as TSC_CLK. To get the TSC_CLK, you simply multiply the
reference clock by the pll divider value (not the reg. bit location)

*/

typedef enum {
  TEMOD_PLL_MODE_DIV_ZERO    = 0   ,  /*!< Illegal PLL Divider Number      */
  TEMOD_PLL_MODE_DIV_40      = 2   ,  /*!< Multiply ref. clk by 40         */
  TEMOD_PLL_MODE_DIV_42      = 3   ,  /*!< Multiply ref. clk by 42         */
  TEMOD_PLL_MODE_DIV_48      = 4   ,  /*!< Multiply ref. clk by 48         */
  TEMOD_PLL_MODE_DIV_52      = 6   ,  /*!< Multiply ref. clk by 52         */
  TEMOD_PLL_MODE_DIV_54      = 7   ,  /*!< Multiply ref. clk by 54         */
  TEMOD_PLL_MODE_DIV_60      = 8   ,  /*!< Multiply ref. clk by 60         */
  TEMOD_PLL_MODE_DIV_64      = 9   ,  /*!< Multiply ref. clk by 64         */
  TEMOD_PLL_MODE_DIV_66      = 10  ,  /*!< Multiply ref. clk by 66         */
  TEMOD_PLL_MODE_DIV_70      = 12  ,  /*!< Multiply ref. clk by 70         */
  TEMOD_PLL_MODE_DIV_80      = 13  ,  /*!< Multiply ref. clk by 80         */
  TEMOD_PLL_MODE_DIV_92      = 14  ,  /*!< Multiply ref. clk by 92         */
  TEMOD_PLL_MODE_DIV_ILLEGAL         /*!< Illegal (programmatic boundary) */
} temod_pll_mode_type;

/*! \def CNT_temod_pll_mode_type Types of enum temod_pll_mode_type */
#define CNT_temod_pll_mode_type 13

/*!
\brief
This array returns the string of #temod_pll_mode_type when indexed by the enum.

*/
extern char* e2s_temod_pll_mode_type [CNT_temod_pll_mode_type];
/*!
\brief
This array returns the clock multiplier of #temod_pll_mode_type when indexed by the enum.

*/
extern int e2n_temod_pll_mode_type [CNT_temod_pll_mode_type];
/*! \enum temod_os_mode_type 


The oversampling mode means that bits are sent out more than once to reduce
the effective frequency of data transfer. For example you can send 5G bits
over a 10G bit lane by sending every bit twice, or OS=2. When the OS is 3.3
it's a bit tricky, we send bits 3 or 4 times in a pattern like so
3,3,3,4,3,3,3,4,... Similarly for OS = 8, we send 8,8,8,9,8,8,8,9,...


*/

typedef enum {
  TEMOD_PMA_OS_MODE_1        = 0   ,  /*!< Over sampling Mode 1         */
  TEMOD_PMA_OS_MODE_2        = 1   ,  /*!< Over sampling Mode 2         */
  TEMOD_PMA_OS_MODE_3        = 2   ,  /*!< Over sampling Mode 3         */
  TEMOD_PMA_OS_MODE_3_3      = 3   ,  /*!< Over sampling Mode 3.3       */
  TEMOD_PMA_OS_MODE_4        = 4   ,  /*!< Over sampling Mode 4         */
  TEMOD_PMA_OS_MODE_5        = 5   ,  /*!< Over sampling Mode 5         */
  TEMOD_PMA_OS_MODE_7_25     = 6   ,  /*!< Over sampling Mode 7.25      */
  TEMOD_PMA_OS_MODE_8        = 7   ,  /*!< Over sampling Mode 8         */
  TEMOD_PMA_OS_MODE_8_25     = 8   ,  /*!< Over sampling Mode 8.25      */
  TEMOD_PMA_OS_MODE_10       = 9   ,  /*!< Over sampling Mode 10        */
  TEMOD_PMA_OS_MODE_ILLEGAL  = 15    /*!< Over sampling Mode Illegal   */
} temod_os_mode_type;

/*! \def CNT_temod_os_mode_type Types of enum temod_os_mode_type */
#define CNT_temod_os_mode_type 11

/*!
\brief
This array returns the string of #temod_os_mode_type when indexed by the enum.

*/
extern char* e2s_temod_os_mode_type [CNT_temod_os_mode_type];
/*!
\brief
This array returns the over sampling value of #temod_os_mode_type when indexed by
the enum. For floating values 3.3 and 8.25 it returns 33 and 825.

*/
extern int e2n_temod_os_mode_type [CNT_temod_os_mode_type];
/*! \enum temod_scr_mode 

This mode indicates how many bits of a 'frame' will be scrambled. We have either
66 or 80 bits per frame (depending on 8b10b or 64b66b encoding). Normally
scrambling is 

\li bypassed for baud rates under 6.25G (0). 
\li we scramble all 66 bits (1)
\li all 80 bits(2)
\li only 64 of the 66b (3). i.e we don't scramble the 'sync' bits.

*/

typedef enum {
  TEMOD_SCR_MODE_BYPASS      = 0   ,  /*!< Scrambling Mode bypassed      */
  TEMOD_SCR_MODE_66B         = 1   ,  /*!< Scrambling Mode 66B           */
  TEMOD_SCR_MODE_80B         = 2   ,  /*!< Scrambling Mode 80B           */
  TEMOD_SCR_MODE_64B         = 3     /*!< Scrambling Mode 64B           */
} temod_scr_mode;

/*! \def CNT_temod_scr_mode Types of enum temod_scr_mode */
#define CNT_temod_scr_mode 4

/*!
\brief
This array returns the string of #temod_scr_mode when indexed by the enum.

*/
extern char* e2s_temod_scr_mode [CNT_temod_scr_mode];
/*!
\brief
This array returns the scrambling mode of #temod_scr_mode when indexed by
the enum.

*/
extern int e2n_temod_scr_mode [CNT_temod_scr_mode];
/*! \enum temod_encode_mode 

Serial bits are encoded when transmitted. The encoding depends on the baud rate
and the link type
\li 000 All encoding functions disabled for lane
\li 001 8b10b  (cl48 )
\li 010 8b10b  (cl48 rxaui)
\li 011 8b10b  (cl36 )
\li 100 64b66b (cl82 )
\li 101 64b66b (cl49 )
\li 110 64b66b (brcm )

*/

typedef enum {
  TEMOD_ENCODE_MODE_NONE     = 0   ,  /*!< Encoding Mode NONE     */
  TEMOD_ENCODE_MODE_CL48     = 1   ,  /*!< Encoding Mode CL48     */
  TEMOD_ENCODE_MODE_CL48_2_LANE = 2   ,  /*!< Encoding Mode CL48     */
  TEMOD_ENCODE_MODE_CL36     = 3   ,  /*!< Encoding Mode CL36     */
  TEMOD_ENCODE_MODE_CL82     = 4   ,  /*!< Encoding Mode CL82     */
  TEMOD_ENCODE_MODE_CL49     = 5   ,  /*!< Encoding Mode CL49     */
  TEMOD_ENCODE_MODE_BRCM     = 6   ,  /*!< Encoding Mode BRCM     */
  TEMOD_ENCODE_MODE_ILLEGAL  = 7     /*!< Encoding Mode Illegal  */
} temod_encode_mode;

/*! \def CNT_temod_encode_mode Types of enum temod_encode_mode */
#define CNT_temod_encode_mode 8

/*!
\brief
This array returns the string of #temod_encode_mode when indexed by the enum.

*/
extern char* e2s_temod_encode_mode [CNT_temod_encode_mode];
/*!
\brief
This array returns the encoding mode of #temod_encode_mode when indexed by
the enum.

*/
extern int e2n_temod_encode_mode [CNT_temod_encode_mode];
/*! \enum temod_check_end_mode 

Check end is a disparity based error bit set by the PCS for CL48. The generation
of this error bit depends on the speed.

*/

typedef enum {
  TEMOD_CL48_CHECK_END_OFF   = 0   ,  /*!< Don't generate checkend error bit  */
  TEMOD_CL48_CHECK_END_ON    = 1     /*!< generate checkend error bit */
} temod_check_end_mode;

/*! \def CNT_temod_check_end_mode Types of enum temod_check_end_mode */
#define CNT_temod_check_end_mode 2

/*!
\brief
This array returns the string of #temod_check_end_mode when indexed by the enum.

*/
extern char* e2s_temod_check_end_mode [CNT_temod_check_end_mode];
/*! \enum temod_sigdet_filter 

No documentation available

*/

typedef enum {
  TEMOD_SIGDET_FILTER_NONCX4 = 0   ,  /*!< No description available */
  TEMOD_SIGDET_FILTER_CX4    = 1     /*!< No description available */
} temod_sigdet_filter;

/*! \def CNT_temod_sigdet_filter Types of enum temod_sigdet_filter */
#define CNT_temod_sigdet_filter 2

/*!
\brief
This array returns the string of #temod_sigdet_filter when indexed by the enum.

*/
extern char* e2s_temod_sigdet_filter [CNT_temod_sigdet_filter];
/*! \enum temod_blocksync_mode 

A block is detected in ports with multiple lanes. PCS has to recognize blocks
since the bits may come in skewed in time over lanes. This feature identifies
the block type, which is a feature of the speed.

000 none
001 cl49 mode - enables func_cl49cl82_sync block
010 cl82 mode - enables func_cl49cl82_sync block
011 8b10b mode - enables the func_8b10b_sync block
100 fec mode - enables the func_fec_sync block
101 brcm 64b66b mode

*/

typedef enum {
  TEMOD_BLOCKSYNC_MODE_NONE  = 0   ,  /*!< Block sync mode none    */
  TEMOD_BLOCKSYNC_MODE_CL49  = 1   ,  /*!< Block sync mode CL49 type    */
  TEMOD_BLOCKSYNC_MODE_CL82  = 2   ,  /*!< Block sync mode CL82  type */
  TEMOD_BLOCKSYNC_MODE_8B10B = 3   ,  /*!< Block sync mode 8B10B type */
  TEMOD_BLOCKSYNC_MODE_FEC   = 4   ,  /*!< Block sync mode FEC   type */
  TEMOD_BLOCKSYNC_MODE_BRCM  = 5   ,  /*!< Block sync mode BRCM  type */
  TEMOD_BLOCKSYNC_MODE_ILLEGAL = 7     /*!< Block sync mode illegal */
} temod_blocksync_mode;

/*! \def CNT_temod_blocksync_mode Types of enum temod_blocksync_mode */
#define CNT_temod_blocksync_mode 7

/*!
\brief
This array returns the string of #temod_blocksync_mode when indexed by the enum.

*/
extern char* e2s_temod_blocksync_mode [CNT_temod_blocksync_mode];
/*! \enum temod_reorder_mode 

Depending on aggregation mode we have various types of reordering of serial
bits. 

*/

typedef enum {
  TEMOD_R_REORDER_MODE_NONE  = 0   ,  /*!< Reorder mode NONE type */
  TEMOD_R_REORDER_MODE_CL48  = 1   ,  /*!< Reorder mode CL48 type */
  TEMOD_R_REORDER_MODE_CL36  = 2   ,  /*!< Reorder mode CL36 type */
  TEMOD_R_REORDER_MODE_CL36_CL48 = 3     /*!< Reorder mode CL36_CL48 type */
} temod_reorder_mode;

/*! \def CNT_temod_reorder_mode Types of enum temod_reorder_mode */
#define CNT_temod_reorder_mode 4

/*!
\brief
This array returns the string of #temod_reorder_mode when indexed by the enum.

*/
extern char* e2s_temod_reorder_mode [CNT_temod_reorder_mode];
/*! \enum temod_cl36_mode 

Cl36 mode control

*/

typedef enum {
  TEMOD_CL36_DISABLE         = 0   ,  /*!<  */
  TEMOD_CL36_ENABLE          = 1     /*!<  */
} temod_cl36_mode;

/*! \def CNT_temod_cl36_mode Types of enum temod_cl36_mode */
#define CNT_temod_cl36_mode 2

/*!
\brief
This array returns the string of #temod_cl36_mode when indexed by the enum.

*/
extern char* e2s_temod_cl36_mode [CNT_temod_cl36_mode];
/*! \enum temod_descrambler_mode 

The descrambling must match the scrambling done at the transmitting port.

*/

typedef enum {
  TEMOD_R_DESCR1_MODE_BYPASS = 0   ,  /*!< No descrambling */
  TEMOD_R_DESCR1_MODE_66B    = 1   ,  /*!< 66b descrambling */
  TEMOD_R_DESCR1_MODE_10B    = 2   ,  /*!< 10b descrambling */
  TEMOD_R_DESCR1_MODE_ILLEGAL = 3     /*!<  */
} temod_descrambler_mode;

/*! \def CNT_temod_descrambler_mode Types of enum temod_descrambler_mode */
#define CNT_temod_descrambler_mode 4

/*!
\brief
This array returns the string of #temod_descrambler_mode when indexed by the enum.

*/
extern char* e2s_temod_descrambler_mode [CNT_temod_descrambler_mode];
/*! \enum temod_decoder_mode 

The descrambling must match the scrambling done at the transmitting port.

*/

typedef enum {
  TEMOD_DECODER_MODE_NONE    = 0   ,  /*!< decoder mode NONE    */
  TEMOD_DECODER_MODE_CL49    = 1   ,  /*!< decoder mode CL49    */
  TEMOD_DECODER_MODE_BRCM    = 2   ,  /*!< decoder mode BRCM        */
  TEMOD_DECODER_MODE_ALU     = 3   ,  /*!< decoder mode ALU       */
  TEMOD_DECODER_MODE_CL48    = 4   ,  /*!< decoder mode CL48      */
  TEMOD_DECODER_MODE_CL36    = 5   ,  /*!< decoder mode CL36    */
  TEMOD_DECODER_MODE_CL82    = 6   ,  /*!< decoder mode CL82    */
  TEMOD_DECODER_MODE_ILLEGAL = 7     /*!< decoder mode ILLEGAL */
} temod_decoder_mode;

/*! \def CNT_temod_decoder_mode Types of enum temod_decoder_mode */
#define CNT_temod_decoder_mode 8

/*!
\brief
This array returns the string of #temod_decoder_mode when indexed by the enum.

*/
extern char* e2s_temod_decoder_mode [CNT_temod_decoder_mode];
/*! \enum temod_deskew_mode 

Deskew Mode is a function of speed. 

*/

typedef enum {
  TEMOD_R_DESKEW_MODE_BYPASS = 0   ,  /*!< deskew mode BYPASS   */
  TEMOD_R_DESKEW_MODE_8B_10B = 1   ,  /*!< deskew mode 8B_10B   */
  TEMOD_R_DESKEW_MODE_BRCM_66B = 2   ,  /*!< deskew mode BRCM_66B */
  TEMOD_R_DESKEW_MODE_CL82_66B = 3   ,  /*!< deskew mode CL82_66B */
  TEMOD_R_DESKEW_MODE_CL36_10B = 4   ,  /*!< deskew mode CL36_10B */
  TEMOD_R_DESKEW_MODE_ILLEGAL = 7     /*!< deskew mode ILLEGAL  */
} temod_deskew_mode;

/*! \def CNT_temod_deskew_mode Types of enum temod_deskew_mode */
#define CNT_temod_deskew_mode 6

/*!
\brief
This array returns the string of #temod_deskew_mode when indexed by the enum.

*/
extern char* e2s_temod_deskew_mode [CNT_temod_deskew_mode];
/*! \enum temod_descrambler2_mode 

No description 

*/

typedef enum {
  TEMOD_DESC2_MODE_NONE      = 0   ,  /*!< Descrambler2 mode NONE    */
  TEMOD_DESC2_MODE_CL49      = 1   ,  /*!< Descrambler2 mode CL49    */
  TEMOD_DESC2_MODE_BRCM      = 2   ,  /*!< Descrambler2 mode BRCM    */
  TEMOD_DESC2_MODE_ALU       = 3   ,  /*!< Descrambler2 mode ALU     */
  TEMOD_DESC2_MODE_CL48      = 4   ,  /*!< Descrambler2 mode CL48    */
  TEMOD_DESC2_MODE_CL36      = 5   ,  /*!< Descrambler2 mode CL36    */
  TEMOD_DESC2_MODE_CL82      = 6   ,  /*!< Descrambler2 mode CL82    */
  TEMOD_DESC2_MODE_ILLEGAL   = 7     /*!< Descrambler2 mode ILLEGAL */
} temod_descrambler2_mode;

/*! \def CNT_temod_descrambler2_mode Types of enum temod_descrambler2_mode */
#define CNT_temod_descrambler2_mode 8

/*!
\brief
This array returns the string of #temod_descrambler2_mode when indexed by the enum.

*/
extern char* e2s_temod_descrambler2_mode [CNT_temod_descrambler2_mode];
/*! \enum temod_byte_del_mode 

For low speeds, data is duplicted 10 or 100 times to run at higher bauds. For
example in a 10G lane, we duplicate 100M data 100 times. On the receive side
these duplicate bits must be deleted.
\li 0 - 100M mode (Delete 9 out of every 10 bytes)
\li 1 - 10M mode (Delete 99 out of every 100 bytes)
\li 2 - Passthrough (No deletion)


*/

typedef enum {
  TEMOD_R_DESC2_BYTE_DELETION_100M = 0   ,  /*!< 100M type Byte deletion */
  TEMOD_R_DESC2_BYTE_DELETION_10M = 1   ,  /*!< 10M  type Byte deletion */
  TEMOD_R_DESC2_BYTE_DELETION_NONE = 2     /*!< No   Byte deletion */
} temod_byte_del_mode;

/*! \def CNT_temod_byte_del_mode Types of enum temod_byte_del_mode */
#define CNT_temod_byte_del_mode 3

/*!
\brief
This array returns the string of #temod_blocksync_mode when indexed by the enum.

*/
extern char* e2s_temod_byte_del_mode [CNT_temod_byte_del_mode];
/*! \enum temod_decode_descr_mode 

No description

*/

typedef enum {
  TEMOD_R_DEC1_DESCR_MODE_NONE = 0   ,  /*!<  */
  TEMOD_R_DEC1_DESCR_MODE_BRCM64B66B = 1     /*!<  */
} temod_decode_descr_mode;

/*! \def CNT_temod_decode_descr_mode Types of enum temod_decode_descr_mode */
#define CNT_temod_decode_descr_mode 2

/*!
\brief
This array returns the string of #temod_decode_descr_mode when indexed by the enum.

*/
extern char* e2s_temod_decode_descr_mode [CNT_temod_decode_descr_mode];
/*! \enum an_property_enable 

No description

*/

typedef enum {
  TEMOD_AN_PROPERTY_ENABLE_NONE = 0x00000000 ,  /*!<  */
  TEMOD_AN_PROPERTY_ENABLE_SGMII_MASTER_MODE = 0x00000001 ,  /*!<  */
  TEMOD_AN_PROPERTY_ENABLE_AN_PD_TO_CL37 = 0x00000002 ,  /*!<  */
  TEMOD_AN_PROPERTY_ENABLE_SGMII_TO_CL37_AUTO = 0x00000004 ,  /*!<  */
  TEMOD_AN_PROPERTY_ENABLE_CL37_BAM_to_SGMII_AUTO = 0x00000008 ,  /*!<  */
  TEMOD_AN_PROPERTY_ENABLE_HPAM_TO_CL73_AUTO = 0x00000010 ,  /*!<  */
  TEMOD_AN_PROPERTY_ENABLE_CL73_BAM_TO_HPAM_AUTO = 0x00000020 ,  /*!<  */
  TEMOD_AN_PROPERTY_ENABLE_ILLEGAL = 0x00000040   /*!<  */
} an_property_enable;

/*! \def CNT_an_property_enable Types of enum an_property_enable */
#define CNT_an_property_enable 8

/*!
\brief
This array returns the string of #an_property_enable when indexed by the enum.

*/
extern char* e2s_an_property_enable [CNT_an_property_enable];
/*! \enum credit_type_t 

No description

*/

typedef enum {
  TEMOD_CREDIT_RESET         = 0x00000000 ,  /*!<  */
  TEMOD_CREDIT_TABLE         = 0x00000001 ,  /*!<  */
  TEMOD_CREDIT_CLOCK_COUNT_0 = 0x00000002 ,  /*!<  */
  TEMOD_CREDIT_CLOCK_COUNT_1 = 0x00000004 ,  /*!<  */
  TEMOD_CREDIT_LOOP_COUNT_0  = 0x00000008 ,  /*!<  */
  TEMOD_CREDIT_LOOP_COUNT_1  = 0x00000010 ,  /*!<  */
  TEMOD_CREDIT_MAC           = 0x00000020 ,  /*!<  */
  TEMOD_CREDIT_PCS_CLOCK_COUNT_0 = 0x00000040 ,  /*!<  */
  TEMOD_CREDIT_PCS_GEN_COUNT = 0x00000080 ,  /*!<  */
  TEMOD_CREDIT_EN            = 0x00000100 ,  /*!<  */
  TEMOD_CREDIT_PCS_REPCNT    = 0x00000200 ,  /*!<  */
  TEMOD_CREDIT_SGMII_SPD     = 0x00000400 ,  /*!<  */
  TEMOD_CREDIT_ILLEGAL       = 0x00000800   /*!<  */
} credit_type_t;

/*! \def CNT_credit_type_t Types of enum credit_type_t */
#define CNT_credit_type_t 13

/*!
\brief
This array returns the string of #an_property_enable when indexed by the enum.

*/
extern char* e2s_credit_type_t [CNT_credit_type_t];
/*! \enum override_type_t 

No description

*/

typedef enum {
  TEMOD_OVERRIDE_RESET       = 0x00000000 ,  /*!<  */
  TEMOD_OVERRIDE_CL72        = 0x00000003 ,  /*!<  */
  TEMOD_OVERRIDE_SPDID       = 0x00000007 ,  /*!<  */
  TEMOD_OVERRIDE_NUM_LANES   = 0x00000001 ,  /*!<  */
  TEMOD_OVERRIDE_OS_MODE     = 0x00000002 ,  /*!<  */
  TEMOD_OVERRIDE_FEC_EN      = 0x00000004 ,  /*!<  */
  TEMOD_OVERRIDE_DESKEW_MODE = 0x00000008 ,  /*!<  */
  TEMOD_OVERRIDE_DESC2_MODE  = 0x00000010 ,  /*!<  */
  TEMOD_OVERRIDE_CL36BYTEDEL_MODE = 0x00000020 ,  /*!<  */
  TEMOD_OVERRIDE_BRCM64B66_DESCR_MODE = 0x00000040 ,  /*!<  */
  TEMOD_OVERRIDE_CHKEND_EN   = 0x00000080 ,  /*!<  */
  TEMOD_OVERRIDE_BLKSYNC_MODE = 0x00000100 ,  /*!<  */
  TEMOD_OVERRIDE_DECODE_MODE = 0x00000200 ,  /*!<  */
  TEMOD_OVERRIDE_REORDER_EN  = 0x00000400 ,  /*!<  */
  TEMOD_OVERRIDE_CL36_EN     = 0x00000800 ,  /*!<  */
  TEMOD_OVERRIDE_SCR_MODE    = 0x00001000 ,  /*!<  */
  TEMOD_OVERRIDE_DESCR_MODE  = 0x00002000 ,  /*!<  */
  TEMOD_OVERRIDE_ENCODE_MODE = 0x00004000 ,  /*!<  */
  TEMOD_OVERRIDE_ALL         = 0x00008000   /*!<  */
} override_type_t;

/*! \def CNT_override_type_t Types of enum override_type_t */
#define CNT_override_type_t 19

/*!
\brief
This array returns the string of #an_property_enable when indexed by the enum.

*/
extern char* e2s_override_type_t [CNT_override_type_t];
/*! \enum temod_ref_clk_t 

No description

*/

typedef enum {
  TEMODREFCLK156MHZ          = 0x00000000 ,  /*!< 156p25MHz */
  TEMODREFCLK125MHZ          = 0x00000001 ,  /*!< 125MHz */
  TEMODREFCLKCOUNT           = 0x00000002   /*!<  */
} temod_ref_clk_t;

/*! \def CNT_temod_ref_clk_t Types of enum temod_ref_clk_t */
#define CNT_temod_ref_clk_t 3

/*!
\brief
This array returns the string of #an_property_enable when indexed by the enum.

*/
extern char* e2s_temod_ref_clk_t [CNT_temod_ref_clk_t];
#endif /* _TSCMOD_ENUM_DEFINES_H */
