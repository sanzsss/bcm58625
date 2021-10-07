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

#ifndef __PM_INTF_H__
#define __PM_INTF_H__

#include <linux/types.h>
#include <linux/io.h>
#include <asm/delay.h>
#include "pm_cfg_data.h"

#define PM_WAIT(pdev, uS) udelay(uS)

#ifdef CHIMP_VIEW
typedef pm_phy_cfg_t *pm_device_t;
#else
typedef void *pm_device_t;
#endif

#define PBUS_TIMEOUT  1000	/* 1 ms */

#define PM_ERROR -1
#define PM_OK 0

static inline u32 tsc_addr(u8 port_addr, bool tsc_eagle, u16 reg_addr, u8 lane, bool is_bcast)
{
	u32 addr;
	u8 port_mode = lane & 0x7;

	if (is_bcast)
		port_mode = 0x6;

	addr = ((port_addr & 0x1F) << 19) | tsc_eagle << 27 | port_mode << 16 | reg_addr;

	return addr;
}

#define XLPORT_WC_UCMEM_CTRL    0x219
#define XLPORT_WC_UCMEM_DATA    0x00000000
#if 1 /*def UNCOMMENT  Legacy code*/
#define UCMEM_ADDR(_core_addr, _devad, _reg_addr, _is_bcast) \
		(((_core_addr & 0x1f) << 19) | (_devad << 27) | (_is_bcast << 17) | (_is_bcast << 18) | _reg_addr)
#define UCMEM_ADDR_LANE(port_addr, tsc_eagle, _reg_addr, lane, _is_bcast)  tsc_addr(port_addr, tsc_eagle, _reg_addr, lane, _is_bcast)
#else
#define UCMEM_ADDR(port_addr, tsc_eagle, _reg_addr, lane, _is_bcast)  tsc_addr(port_addr, tsc_eagle, _reg_addr, lane, _is_bcast)
#endif

#define UCMEM_DATA(_data) \
		((_data & 0x0000ffff) << 16)

/* Receive statistics counters */
#define PM_XLPORT_R64       0x00000000 /*  Receive 64 Byte Frame Counter */
#define PM_XLPORT_R127      0x00000001 /* Receive 65 to 127 Byte Frame Counter */
#define PM_XLPORT_R255      0x00000002 /* Receive 128 to 255 Byte Frame Counter */
#define PM_XLPORT_R511      0x00000003 /* Receive 256 to 511 Byte Frame Counter */
#define PM_XLPORT_R1023     0x00000004 /* Receive 512 to 1023 Byte Frame Counter */
#define PM_XLPORT_R1518     0x00000005 /* Receive 1024 to 1518 Byte Frame Counter */
#define PM_XLPORT_RMGV      0x00000006 /* Receive 1519 to 1522 Byte Good VLAN Frame Counter */
#define PM_XLPORT_R2047     0x00000007 /* Receive 1519 to 2047 Byte Frame Counter */
#define PM_XLPORT_R4095     0x00000008 /* Receive 2048 to 4095 Byte Frame Counter */
#define PM_XLPORT_R9216     0x00000009 /* Receive 4096 to 9216 Byte Frame Counter */
#define PM_XLPORT_R16383    0x0000000A /* Receive 9217 to 16383 Byte Frame Counter */
#define PM_XLPORT_RPKT      0x0000000B /* Receive frame/packet Counter */
#define PM_XLPORT_RUCA      0x0000000C /* Receive Unicast Frame Counter */
#define PM_XLPORT_RMCA      0x0000000D /* Receive Multicast Frame Counter */
#define PM_XLPORT_RBCA      0x0000000E /* Receive Broadcast Frame Counter */
#define PM_XLPORT_RFCS      0x0000000F /* Receive FCS Error Frame Counter */
#define PM_XLPORT_RXCF      0x00000010 /* Receive Control Frame Counter */
#define PM_XLPORT_RXPF      0x00000011 /* Receive PAUSE Frame Counter */
#define PM_XLPORT_RXPP      0x00000012  /* Receive PFC (Per-Priority Pause) Frame Counter  */
#define PM_XLPORT_RXUO      0x00000013 /* Receive Unsupported Opcode Frame Counter */
#define PM_XLPORT_RXUDA     0x00000014 /* Receive Unsupported DA for PAUSE/PFC Frame Counter */
#define PM_XLPORT_RXWSA     0x00000015 /* Receive Wrong SA Frame Counter */
#define PM_XLPORT_RALN      0x00000016 /* Receive Alignment Error Frame Counter */
#define PM_XLPORT_RFLR      0x00000017 /* Receive Length Out of Range Frame Counter */
#define PM_XLPORT_RERPKT    0x00000018 /* Receive Code Error Frame Counter */
#define PM_XLPORT_RFCR      0x00000019 /* Receive False Carrier Counter */
#define PM_XLPORT_ROVR      0x0000001A /* Receive Oversized Frame Counter */
#define PM_XLPORT_RJBR      0x0000001B /* Receive Jabber Frame Counter */
#define PM_XLPORT_RMTUE     0x0000001C /* Receive MTU Check Error Frame Counter */
#define PM_XLPORT_RMCRC     0x0000001D /* Matched CRC Frame Counter */
#define PM_XLPORT_RPRM      0x0000001E /* Receive Promiscuous Frame Counter */
#define PM_XLPORT_RVLN      0x0000001F /* Receive VLAN Tag Frame Counter */
#define PM_XLPORT_RDVLN     0x00000020 /* Receive Double VLAN Tag Frame Counter */
#define PM_XLPORT_RTRFU     0x00000021 /* Receive Truncated Frame Counter (due to RX FIFO full) */
#define PM_XLPORT_RPOK      0x00000022 /* Receive Good Packet Counter */
#define PM_XLPORT_RPFCOFF0  0x00000023 /* Receive PFC Frame Priority 0 XON to XOFF */
#define PM_XLPORT_RPFCOFF1  0x00000024 /* Receive PFC Frame Priority 1 */
#define PM_XLPORT_RPFCOFF2  0x00000025 /* Receive PFC Frame Priority 2 */
#define PM_XLPORT_RPFCOFF3  0x00000026 /* Receive PFC Frame Priority 3 */
#define PM_XLPORT_RPFCOFF4  0x00000027 /* Receive PFC Frame Priority 4 */
#define PM_XLPORT_RPFCOFF5  0x00000028 /* Receive PFC Frame Priority 5 */
#define PM_XLPORT_RPFCOFF6  0x00000029 /* Receive PFC Frame Priority 6 */
#define PM_XLPORT_RPFCOFF7  0x0000002A /* Receive PFC Frame Priority 7 */
#define PM_XLPORT_RPFC0     0x0000002B /* Receive PFC Frame Priority 0 */
#define PM_XLPORT_RPFC1     0x0000002C /* Receive PFC Frame Priority 1 */
#define PM_XLPORT_RPFC2     0x0000002D /* Receive PFC Frame Priority 2 */
#define PM_XLPORT_RPFC3     0x0000002E /* Receive PFC Frame Priority 3 */
#define PM_XLPORT_RPFC4     0x0000002F /* Receive PFC Frame Priority 4 */
#define PM_XLPORT_RPFC5     0x00000030 /* Receive PFC Frame Priority 5 */
#define PM_XLPORT_RPFC6     0x00000031 /* Receive PFC Frame Priority 6 */
#define PM_XLPORT_RPFC7     0x00000032 /* Receive PFC Frame Priority 7 */
#define PM_XLPORT_RSCHCRC   0x00000033 /* Receive SCH CRC Error */
#define PM_XLPORT_RUND      0x00000034 /* Receive Undersize Frame Counter */
#define PM_XLPORT_RFRG      0x00000035 /* Receive Fragment Counter */
#define PM_XLPORT_RX_EEE_LPI_EVENT_COUNTER    0x00000036 /* RX EEE LPI Event Counter */
#define PM_XLPORT_RX_EEE_LPI_DURATION_COUNTER 0x00000037 /*  RX EEE LPI Duration Counter */
#define PM_XLPORT_RX_LLFC_PHY_COUNTER         0x00000038 /* Receive Physical Type LLFC message counter */
#define PM_XLPORT_RX_LLFC_LOG_COUNTER         0x00000039 /* Receive Logical Type LLFC message Counter */
#define PM_XLPORT_RX_LLFC_CRC_COUNTER         0x0000003A /* Receive Type LLFC message with CRC error Counter */
#define PM_XLPORT_RX_HCFC_COUNTER             0x0000003B /* Receive HCFC message counter */
#define PM_XLPORT_RX_HCFC_CRC_COUNTER         0x0000003C /* Receive HCFC message with CRC Error counter */
#define PM_XLPORT_RBYT                        0x0000003D /* Receive Byte Counter */
#define PM_XLPORT_RRBYT                       0x0000003E /* Receive Runt Byte Counter */
#define PM_XLPORT_RRPKT                       0x0000003F /* Receive RUNT Frame Counter */

/* Transmit statistics counters */
#define PM_XLPORT_T64       0x00000040 /* Transmit 64 Byte Frame Counter */
#define PM_XLPORT_T127      0x00000041 /* Transmit 65 to 127 Byte Frame Counter */
#define PM_XLPORT_T255      0x00000042 /* Transmit 128 to 255 Byte Frame Counter */
#define PM_XLPORT_T511      0x00000043 /* Transmit 256 to 511 Byte Frame Counter */
#define PM_XLPORT_T1023     0x00000044 /* Transmit 512 to 1023 Byte Frame Counter */
#define PM_XLPORT_T1518     0x00000045 /* Transmit 1024 to 1518 Byte Frame Counter */
#define PM_XLPORT_TMGV      0x00000046 /* Transmit 1519 to 1522 Byte Good VLAN Frame Counter */
#define PM_XLPORT_T2047     0x00000047 /* Transmit 1519 to 2047 Byte Frame Counter */
#define PM_XLPORT_T4095     0x00000048 /* Transmit 2048 to 4095 Byte Frame Counter */
#define PM_XLPORT_T9216     0x00000049 /* Transmit 4096 to 9216 Byte Frame Counter */
#define PM_XLPORT_T16383    0x0000004A /* Transmit 9217 to 16383 Byte Frame Counter */
#define PM_XLPORT_TPOK      0x0000004B /* Transmit Good Packet Counter */
#define PM_XLPORT_TPKT      0x0000004C /* Transmit Packet/Frame Counter */
#define PM_XLPORT_TUCA      0x0000004D /* Transmit Unicast Frame Counter */
#define PM_XLPORT_TMCA      0x0000004E /* Transmit Multicast Frame Counter */
#define PM_XLPORT_TBCA      0x0000004F /* Transmit Broadcast Frame Counter */
#define PM_XLPORT_TXPF      0x00000050 /* Transmit Pause Control Frame Counter */
#define PM_XLPORT_TXPP      0x00000051 /* Transmit PFC/Per-Priority Pause Control Frame Counter */
#define PM_XLPORT_TJBR      0x00000052 /* Transmit Jabber Counter  */
#define PM_XLPORT_TFCS      0x00000053 /* Transmit FCS Error Counter */
#define PM_XLPORT_TXCF      0x00000054 /* Transmit Control Frame Counter */
#define PM_XLPORT_TOVR      0x00000055 /* Transmit Oversize Packet Counter */
#define PM_XLPORT_TDFR      0x00000056 /* Transmit Single Deferral Frame Counter */
#define PM_XLPORT_TEDF      0x00000057 /* Transmit Multiple Deferral Frame Counter */
#define PM_XLPORT_TSCL      0x00000058 /* Transmit Single Collision Frame Counter */
#define PM_XLPORT_TMCL      0x00000059 /* Transmit Multiple Collision Frame Counter */
#define PM_XLPORT_TLCL      0x0000005A /* Transmit Late Collision Frame Counter */
#define PM_XLPORT_TXCL      0x0000005B /* Transmit Excessive Collision Frame Counter */
#define PM_XLPORT_TFRG      0x0000005C /* Transmit Fragment Counter */
#define PM_XLPORT_TERR      0x0000005D /* Transmit Error (set by system) Counter */
#define PM_XLPORT_TVLN      0x0000005E /* Transmit VLAN Tag Frame Counter */
#define PM_XLPORT_TDVLN     0x0000005F /* Transmit Double VLAN Tag Frame Counter */
#define PM_XLPORT_TRPKT     0x00000060 /* Transmit RUNT Frame Counter */
#define PM_XLPORT_TUFL      0x00000061 /* Transmit FIFO Underrun Counter. */
#define PM_XLPORT_TPFC0     0x00000062 /* Transmit PFC Frame Priority 0 */
#define PM_XLPORT_TPFC1     0x00000063 /* Transmit PFC Frame Priority 1 */
#define PM_XLPORT_TPFC2     0x00000064 /* Transmit PFC Frame Priority 2 */
#define PM_XLPORT_TPFC3     0x00000065 /* Transmit PFC Frame Priority 3 */
#define PM_XLPORT_TPFC4     0x00000066 /* Transmit PFC Frame Priority 4 */
#define PM_XLPORT_TPFC5     0x00000067 /* Transmit PFC Frame Priority 5 */
#define PM_XLPORT_TPFC6     0x00000068 /* Transmit PFC Frame Priority 6 */
#define PM_XLPORT_TPFC7     0x00000069 /* Transmit PFC Frame Priority 7 */
#define PM_XLPORT_TX_EEE_LPI_EVENT_COUNTER    0x0000006A /* TX EEE LPI Event Counter */
#define PM_XLPORT_TX_EEE_LPI_DURATION_COUNTER 0x0000006B /* TX EEE LPI Duration Counter */
#define PM_XLPORT_TX_LLFC_LOG_COUNTER         0x0000006C /* Transmit Logical Type LLFC message counter */
#define PM_XLPORT_TX_HCFC_COUNTER             0x0000006D /* Transmit HCFC message counter */
#define PM_XLPORT_TNCL                        0x0000006E /* Transmit Total Collision Counter */
#define PM_XLPORT_TBYT                        0x0000006F /* Transmit Byte Counter */
#define PM_XLPORT_XTHOL                       0x00000070 /* Transmit End-to-End HOL packet counter (E2ECC/VOQFC). */

#define PM_XLPORT_LAG_FAILOVER_CONFIG_LINK_UP   (1L << 0)

#define PM_XLPORT_FLOW_CONTROL_CONFIG           0x00207

/* Generic registers */
#define PM_XLPORT_MODE_REG                      0x0020A
#define PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SFT 3
#define PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_MASK    (0x7 << PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_QUAD    (0 << PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_TRI_012 (1 << PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_TRI_023 (2 << PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_DUAL    (3 << PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SINGLE  (4 << PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_DISABLE (5 << PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SFT 0
#define PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_MASK (0x7 << PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_QUAD (0 << PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_TRI_012 (1 << PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_TRI_023 (2 << PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_DUAL    (3 << PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SINGLE  (4 << PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SFT)
#define PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_DISABLE (5 << PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SFT)

#define PM_XLPORT_ENABLE_REG                    0x0020B
#define PM_XLPORT_SOFT_RESET_REG                0x0020C
#define PM_XLPORT_MAC_CONTROL                   0x00210

#define PM_XLPORT_XGXS0_CTRL_REG                0x00214
#define PM_XLPORT_XGXS0_HARD_RESET            (1 << 0)
#define PM_XLPORT_XGXS0_LCREFOUT_ENABLE       (1 << 1)
#define PM_XLPORT_XGXS0_LCREFINT_ENABLE       (1 << 2)
#define PM_XLPORT_XGXS0_TSC_POWER_DOWN        (1 << 3)
#define PM_XLPORT_XGXS0_IDDQ                  (1 << 4)

#define PM_XLPORT_MIB_RESET                     0x00224

#define PM_XLMAC_BASE_ADDR                      0x00600

#define PM_XLMAC_CTRL                           0x00600
#define CLMAC_CTRL_ALLOW_40B_AND_GREATER_PKT (1 << 14)

#define PM_XLMAC_MODE                           0x00601
#define PM_XLMAC_RX_MAX_SIZE                    0x00608
#define PM_XLMAC_TX_CTRL                        0x00604
#define PM_XLMAC_TX_MAC_ADDR                    0x00605
#define PM_XLMAC_PAUSE_CTRL                     0x0060D
#define PM_XLMAC_PFC_CTRL                       0x0060E
#define PM_XLMAC_LLFC_CONTROL                   0x00612

typedef struct _pm_counter {
  /* Each counter is 40-bit */
	u32 low;
	u32 high;
} pm_counter_t;

typedef struct _pm_tx_statistics_t {
  pm_counter_t tx64;
  pm_counter_t tx127;
  pm_counter_t tx255;
  pm_counter_t tx511;
  pm_counter_t tx1023;
  pm_counter_t tx1518;
  pm_counter_t tx_good_vlan;
  pm_counter_t tx2047;
  pm_counter_t tx4095;
  pm_counter_t tx9216;
  pm_counter_t tx16383;
  pm_counter_t tx_good_packet;
  pm_counter_t tx_packet;
  pm_counter_t tx_unicast;
  pm_counter_t tx_multicast;
  pm_counter_t tx_broadcast;
  pm_counter_t tx_pause_frame;
  pm_counter_t tx_pfc_pause;
  pm_counter_t tx_jabber_frames;
  pm_counter_t tx_fcs_errors;
  pm_counter_t tx_ctrl_frames;
  pm_counter_t tx_oversized_frames;
  pm_counter_t tx_single_deferral_frames;
  pm_counter_t tx_multiple_deferral_frames;
  pm_counter_t tx_single_collision;
  pm_counter_t tx_multiple_collision;
  pm_counter_t tx_late_collision;
  pm_counter_t tx_excessive_collision;
  pm_counter_t tx_fragment;
  pm_counter_t tx_errors;
  pm_counter_t tx_vlan_frames;
  pm_counter_t tx_double_vlan_frames;
  pm_counter_t tx_runt_frames;
  pm_counter_t tx_fifo_underrun;
  pm_counter_t tx_pfc_pri0;
  pm_counter_t tx_pfc_pri1;
  pm_counter_t tx_pfc_pri2;
  pm_counter_t tx_pfc_pri3;
  pm_counter_t tx_pfc_pri4;
  pm_counter_t tx_pfc_pri5;
  pm_counter_t tx_pfc_pri6;
  pm_counter_t tx_pfc_pri7;
  pm_counter_t tx_eee_lpi_events;
  pm_counter_t tx_eee_lpi_durations;
  pm_counter_t tx_llfc_log;
  pm_counter_t tx_hcfc_messages;
  pm_counter_t tx_total_collision;
  pm_counter_t tx_byte_count;
  pm_counter_t tx_e2e_hol_packet;
} pm_tx_statistics_t;

typedef struct _pm_rx_statistics_t {
  pm_counter_t rx64;
  pm_counter_t rx127;
  pm_counter_t rx255;
  pm_counter_t rx511;
  pm_counter_t rx1023;
  pm_counter_t rx1518;
  pm_counter_t rxMGV;
  pm_counter_t rx2047;
  pm_counter_t rx4095;
  pm_counter_t rx9216;
  pm_counter_t rx16383;
  pm_counter_t rx_pkt;
  pm_counter_t rx_unicast;
  pm_counter_t rx_multicast;
  pm_counter_t rx_broadcast;
  pm_counter_t rx_fcs_errors;
  pm_counter_t rx_ctrl_frame;
  pm_counter_t rx_pause_frame;
  pm_counter_t rx_pfc_frames;
  pm_counter_t rx_unsupported_opcode_frame;
  pm_counter_t rx_unsupported_da;
  pm_counter_t rx_wrong_sa_frame;
  pm_counter_t rx_alignment_errors;
  pm_counter_t rx_out_of_range_length;
  pm_counter_t rx_error_frames;
  pm_counter_t rx_false_carrier;
  pm_counter_t rx_oversized_frames;
  pm_counter_t rx_jabber_frames;
  pm_counter_t rx_mtuch_check_errors;
  pm_counter_t rx_matched_crc_frames;
  pm_counter_t rx_promiscuous_frames;
  pm_counter_t rx_vlan_frames;
  pm_counter_t rx_double_vlan_frames;
  pm_counter_t rx_truncated_frames;
  pm_counter_t rx_good_packets;
  pm_counter_t rx_pfc_pause_pri0; /* XON -> XOFF */
  pm_counter_t rx_pfc_pause_pri1;
  pm_counter_t rx_pfc_pause_pri2;
  pm_counter_t rx_pfc_pause_pri3;
  pm_counter_t rx_pfc_pause_pri4;
  pm_counter_t rx_pfc_pause_pri5;
  pm_counter_t rx_pfc_pause_pri6;
  pm_counter_t rx_pfc_pause_pri7;
  pm_counter_t rx_pfc_ctrl_pri0;
  pm_counter_t rx_pfc_ctrl_pri1;
  pm_counter_t rx_pfc_ctrl_pri2;
  pm_counter_t rx_pfc_ctrl_pri3;
  pm_counter_t rx_pfc_ctrl_pri4;
  pm_counter_t rx_pfc_ctrl_pri5;
  pm_counter_t rx_pfc_ctrl_pri6;
  pm_counter_t rx_pfc_ctrl_pri7;
  pm_counter_t rx_sch_crc;
  pm_counter_t rx_undersize_frames;
  pm_counter_t rx_fragment_frames;
  pm_counter_t rx_eee_lpi_events;
  pm_counter_t rx_eee_lpi_duration;
  pm_counter_t rx_phy_type_llfc_messages;
  pm_counter_t rx_logical_type_llfc_messages;
  pm_counter_t rx_llfc_crc_errors;
  pm_counter_t rx_hcfc_messages;
  pm_counter_t rx_hcfc_messages_with_crc;
  pm_counter_t rx_bytes;
  pm_counter_t rx_runt_bytes;
  pm_counter_t rx_runt_packets;
} pm_rx_statistics_t;

#define PM_XLPORT_LAG_FAILOVER_CONFIG     0x00202

/* BCM84856 - KOI specific */
#define BCM8485x_NUM_OF_PORTS  2
#define PHY_ADDR_BCAST  0

/* Maxium MDIO clock is 25MHz. The core clock on Cumulus is 375MHz.
 *  MDIO clock = 375MHz/2*(7 + 1) = 23.44MHz.
 */
#define MDIO_CLK_DIVISOR 7

/* 10ms */
#define MDIO_TIMEOUT 10000

#define BCM8485x_PMD_DL_PROC_CTRL     0xa817
  #define BCM8485x_PMD_DL_PROC_CTRL_DL_MODE    0x38
  #define BCM8485x_PMD_DL_PROC_CTRL_WRITE_MODE 0x9


#define PHY_DEVAD_PMD    1
#define PHY_DEVAD_PCS    3
#define PHY_DEVAD_AN     7
#define PHY_DEVAD_USER   30


#define BCM8485x_PMD_CTRL_1            0x0
#define BCM8485x_PMD_CTRL_1_RESET      (1 << 15)
#define BCM8485x_PMD_CTRL_1_LOW_PWR    (1 << 11)
#define BCM8485x_PMD_CTRL_1_LPBK       (1 << 0)

#define BCM8485x_PMD_STATUS_1          0x1

#define BCM8485x_PMD_DL_PROC_STATUS   0xa818
#define BCM8485x_PMD_DL_ADDR_LOW      0xa819
#define BCM8485x_PMD_DL_ADDR_HIGH     0xa81a
#define BCM8485x_PMD_DL_DATA_LOW      0xa81b
#define BCM8485x_PMD_DL_DATA_HIGH     0xa81c

/* PCS registers */
#define BCM8485x_PCS_CTRL_1            0x0
#define BCM8485x_PCS_CTRL_1_LOOPBACK   (1 << 14)

#define BCM8485x_AN_10GBASET_AN_CTRL  0x0020
#define BCM8485x_AN_10GBASET_AN_CTRL_10G_ABILITY (1 << 12)

#define BCM8485x_AN_1000BASET_MII_CTRL  0xffe0
#define BCM8485x_AN_1000BASET_MII_CTRL_AUTONEG_ENABLE  (1 << 12)
#define BCM8485x_AN_1000BASET_MII_CTRL_RESTART_AUTONEG (1 << 9)

#define BCM8485x_AN_COPPER_AUTONEG_ADV 0xffe4
#define BCM8485x_AN_COPPER_AUTONEG_ADV_ASYM_PAUSE   (1 << 11)
#define BCM8485x_AN_COPPER_AUTONEG_ADV_PAUSE_CAP    (1 << 10)
#define BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_FULL (1 << 9)
#define BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_HALF (1 << 8)


#define BCM8485x_AN_1000BASET_CTRL 0xffe9
#define BCM8485x_AN_1000BASET_CTRL_1000MBPS_FULL (1 << 9)
#define BCM8485x_AN_1000BASET_CTRL_1000MBPS_HALF (1 << 8)

#define BCM8485x_USER_STATUS          0x400d
#define BCM8485x_USER_STATUS_ENERGY_DET     (1 << 1)
#define BCM8485x_USER_STATUS_SPEED_MASK     (0x3 << 3)
#define BCM8485x_USER_STATUS_SPEED_100MBPS  (0x1 << 3)
#define BCM8485x_USER_STATUS_SPEED_1GBPS    (0x2 << 3)
#define BCM8485x_USER_STATUS_SPEED_10GBPS   (0x3 << 3)
#define BCM8485x_USER_STATUS_COPPER_LINK_UP (5 << 1)
#define BCM8485x_USER_STATUS_MAC_LINK      (1 << 13)

#define PBUS_IF_ARB     PA(pm_sw_arb)
#define PBUS_IF_WCTRL PA(pm_if_wctrl)
#define PBUS_IF_RCTRL  PA(pm_if_rctrl)
#define PBUS_IF_ADDR    PA(pm_if_addr)
#define PBUS_IF_WADDR    PA(pm_if_waddr)
#define PBUS_IF_RADDR    PA(pm_if_raddr)
#define PBUS_IF_WR_DATA PA(pm_if_wrdata)
#define PBUS_IF_RD_DATA PA(pm_if_rddata)
#define PBUS_IF_STATUS  PA(pm_if_status)
#define PBUS_IF_WR_GO PA(pm_if_wr_go)
#define PBUS_IF_RD_GO PA(pm_if_rd_go)
#define PBUS_IF_STATUS_MASK PA(pm_if_status_mask)
#define PBUS_IF_DEBUG_CNTL PA(pm_if_dbg_ctrl)
#define IPC_PMFC_PHY_ADDR IPC(pmfc_phy_addr)

#define REG_WRITE(pdev, offset, data)  writel(data, offset)
#define REG_READ(pdev, offset)        readl(offset)

#if 0
#define PORT_IDX(pdev)               (((lm_device_t *)pdev)->port_idx)
#define GPIO_WRITE(pdev, pin, mode)    lm_gpio_write(((lm_device_t *)pdev), pin, mode, 0)
#endif
#define GPIO_WRITE(a, b, c)

#ifdef CHIMP_VIEW
#define PA_SW_ARB_NUM   2
#else
#define PA_SW_ARB_NUM   1
#endif

#define PM_SW_ARB_REQ   (1L << (PA_PM_ARB_REQ_SET_SFT + PA_SW_ARB_NUM))
#define PM_SW_ARB_GRANT (1L << (PA_PM_ARB_GRANT_BITS_SFT + PA_SW_ARB_NUM))
#define PM_SW_ARB_CLR   (1L << (PA_PM_ARB_REQ_CLR_SFT + PA_SW_ARB_NUM))

#define ACCESS_PER_PORT   0
#define ACCESS_GENERIC    1

int lm_hw_pm_reset(pm_device_t pdev);
int lm_hw_pm_probe(pm_device_t pdev);
int lm_hw_pm_bringup_phy(pm_device_t pdev);
int lm_hw_pm_get_link_status(pm_device_t pdev, int *link_status, u32 *speed);
int lm_hw_pm_bringdown_phy(pm_device_t pdev);
int lm_hw_pm_cfg_mac_core(pm_device_t pdev, u32 speed);
int lm_hw_pm_cfg_mtu(pm_device_t pdev);
int lm_hw_pm_cfg_pause(pm_device_t pdev);

void lm_hw_pm_minimal_default_xlmac_cfg(pm_device_t pdev, u8 ten_gig_en);
void dump_pm_regs(pm_device_t pdev);

/* Give rising edge to lag_failover_cfg register as a link LED workaround */
int lm_hw_pm_toggle_lagf(pm_device_t pdev);

void lm_hw_pm_apb2pbus_brdg_init(pm_device_t pdev);
int lm_hw_pm_wr_reg(pm_device_t pdev,
			u32 addr, u32 data_hi, u32 data_low, u8 reg_type, u8 port);
int lm_hw_pm_rd_reg(pm_device_t pdev,
			u32 addr, u32 *data_hi, u32 *data_low, u8 reg_type,
			u8 port);
int lm_hw_pm_tsc_wr(void *user_acc, u32 core_addr, u32 reg_addr, u32 val);
/*int lm_hw_pm_tsc_wr(void *pdev, u32 core_addr, u32 reg_addr, u8 lane, bool tsc_or_eagle, u32 val);*/
int lm_hw_pm_tsc_rd(void *pdev, u32 core_addr, u32 reg_addr, u8 lane, bool tsc_or_eagle, u32 *val);
int lm_hw_pm_tsc_rd_ln(void *pdev, u32 core_addr, u32 reg_addr, u8 lane, bool tsc_or_eagle, u32 *val);
int lm_hw_pm_tsc_wr_ln(void *user_acc, u32 core_addr, u32 reg_addr, u8 lane, bool tsc_or_eagle, u32 val);
int lm_hw_pm_tsc_wr_mem(pm_device_t pdev, u32 addr, bool reg_or_mem, u32 *data);
int lm_hw_pm_tsc_rd_mem(pm_device_t pdev, u32 addr, bool reg_or_mem, u32 *i_data, u32 *o_data);

int pm_user_get_config(pm_device_t pdev, pm_phy_cfg_t *phy_cfg);
void pm_wait(pm_device_t pd, u32 uS);

/* Interfaces to query port macro statistics */
int lm_pm_get_counter(pm_device_t pdev, u32 counter_idx,
			  pm_counter_t *counter);
int lm_pm_get_rx_counters(pm_device_t pdev, pm_rx_statistics_t *rx_counters);
int lm_pm_get_tx_counters(pm_device_t pdev, pm_tx_statistics_t *tx_counters);

/* MDIO access routines for both CL22 and CL45 */
void lm_hw_pm_mdio_write45(pm_device_t pdev, u8 phy_addr, u8 devad, u16 address,
			   u16 val);
u16 lm_hw_pm_mdio_read45(pm_device_t pdev, u8 phy_addr, u8 devad, u16 address);
u16 lm_hw_pm_mdio_read22(pm_device_t pdev, u8 phy_addr, u16 address);
void lm_hw_pm_mdio_write22(pm_device_t pdev, u8 phy_addr, u16 address, u16 val);

/* Specific to external PHY */
int lm_pm_get_extphy_fw_info(pm_device_t pd, u32 *image_offset, u32 *length);
u32 lm_pm_get_extphy_fw_data(pm_device_t pdev, u32 offset);

extern pa_reg_t pa;
#endif /* __PM_INTF_H__ */
