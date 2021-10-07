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

#ifndef _PM_CFG_DATA_H_
#define _PM_CFG_DATA_H_

#if defined(CHIMP_VIEW)
	#define PTR(x) (x *)
	#define CONST_PTR(x) (const x *)
#else
#define PTR(x) u32
#define CONST_PTR(x) u32
#endif

struct pm_phy_cfg;
/* Port Macro configuration */
typedef struct pm_phy_cfg {
	/* link speed in multiple of kMbps */
	u32 speed;
	/* mtu size in bytes */
	u16 mtu;
	/* physical port idx from 0 */
	u8 port_idx;
	u8 an_enabled;
	/* Autoneg Capability Mask */
	u16 autoneg_cap;
	#define PM_PHY_CFG_AUTONEG_CAP_100MBPS                  0x1
	#define PM_PHY_CFG_AUTONEG_CAP_1GBPS                    0x2
	#define PM_PHY_CFG_AUTONEG_CAP_2_5GBPS                  0x4
	#define PM_PHY_CFG_AUTONEG_CAP_10GBPS                   0x8
	#define PM_PHY_CFG_AUTONEG_CAP_20GBPS                  0x10
	#define PM_PHY_CFG_AUTONEG_CAP_25GBPS                  0x20
	#define PM_PHY_CFG_AUTONEG_CAP_40GBPS                  0x40
	#define PM_PHY_CFG_AUTONEG_CAP_50GBPS                  0x80
	/* bitfield indicates if tx/rx pause */
	u8 pause;
	/* rx pause enabled */
	#define PM_PHY_CFG_PAUSE_RX                             0x1
	/* tx pause enabled */
	#define PM_PHY_CFG_PAUSE_TX                             0x2
	/* PFC is enabled */
	#define PM_PHY_CFG_PAUSE_PFC_ENABLE                     0x4
	/* loopback mode */
	u8 lpbk;
	#define PM_PHY_CFG_LPBK_NONE                          (0x0)
	#define PM_PHY_CFG_LPBK_INT_PHY                       (0x1)
	#define PM_PHY_CFG_LPBK_EXT_PHY                       (0x2)
	#define PM_PHY_CFG_LPBK_EXT                           (0x3)
	#define PM_PHY_CFG_LPBK_MAC                           (0x4)
	/* phy core type */
	u8 core_type;
	/* For Cumulus */
	#define PM_PHY_CFG_CORE_TYPE_FALCON                   (0x0)
	/* For NS2 */
	#define PM_PHY_CFG_CORE_TYPE_EAGLE                    (0x1)
	/* same format as nvm cfg */
	u32 core_config;
	/* port mode */
	#define PM_PHY_CFG_CORE_CONFIG_PORT_MODE_MASK          0xff
	#define PM_PHY_CFG_CORE_CONFIG_PORT_MODE_SFT              0
		#define PM_PHY_CFG_CORE_CONFIG_PORT_MODE_1X10G   (0x0 << 0)
		#define PM_PHY_CFG_CORE_CONFIG_PORT_MODE_2X10G   (0x1 << 0)
		#define PM_PHY_CFG_CORE_CONFIG_PORT_MODE_4X10G   (0x2 << 0)
		#define PM_PHY_CFG_CORE_CONFIG_PORT_MODE_1X25G   (0x3 << 0)
		#define PM_PHY_CFG_CORE_CONFIG_PORT_MODE_2X25G   (0x4 << 0)
		#define PM_PHY_CFG_CORE_CONFIG_PORT_MODE_1X40G   (0x5 << 0)
		#define PM_PHY_CFG_CORE_CONFIG_PORT_MODE_1X50G   (0x6 << 0)
	/* preemphasis cfg setting */
	#define PM_PHY_CFG_CORE_CONFIG_PREEMPHASIS_CFG        0x100
		#define PM_PHY_CFG_CORE_CONFIG_PREEMPHASIS_CFG_DISABLED (0x0 << 8)
		#define PM_PHY_CFG_CORE_CONFIG_PREEMPHASIS_CFG_ENABLED (0x1 << 8)
	/* same format as nvm cfg */
	u32 lane_swap;
	#define PM_PHY_CFG_LANE_SWAP_RX_LANE0_MASK              0xf
	#define PM_PHY_CFG_LANE_SWAP_RX_LANE0_SFT                 0
	#define PM_PHY_CFG_LANE_SWAP_RX_LANE1_MASK             0xf0
	#define PM_PHY_CFG_LANE_SWAP_RX_LANE1_SFT                 4
	#define PM_PHY_CFG_LANE_SWAP_RX_LANE2_MASK            0xf00
	#define PM_PHY_CFG_LANE_SWAP_RX_LANE2_SFT                 8
	#define PM_PHY_CFG_LANE_SWAP_RX_LANE3_MASK           0xf000
	#define PM_PHY_CFG_LANE_SWAP_RX_LANE3_SFT                12
	#define PM_PHY_CFG_LANE_SWAP_TX_LANE0_MASK          0xf0000
	#define PM_PHY_CFG_LANE_SWAP_TX_LANE0_SFT                16
	#define PM_PHY_CFG_LANE_SWAP_TX_LANE1_MASK         0xf00000
	#define PM_PHY_CFG_LANE_SWAP_TX_LANE1_SFT                20
	#define PM_PHY_CFG_LANE_SWAP_TX_LANE2_MASK        0xf000000
	#define PM_PHY_CFG_LANE_SWAP_TX_LANE2_SFT                24
	#define PM_PHY_CFG_LANE_SWAP_TX_LANE3_MASK       0xf0000000
	#define PM_PHY_CFG_LANE_SWAP_TX_LANE3_SFT                28
	/* same format as nvm cfg */
	u32 lane_polarity;
	#define PM_PHY_CFG_LANE_POLARITY_RX_LANE0               0x1
	#define PM_PHY_CFG_LANE_POLARITY_RX_LANE1               0x2
	#define PM_PHY_CFG_LANE_POLARITY_RX_LANE2               0x4
	#define PM_PHY_CFG_LANE_POLARITY_RX_LANE3               0x8
	#define PM_PHY_CFG_LANE_POLARITY_TX_LANE0              0x10
	#define PM_PHY_CFG_LANE_POLARITY_TX_LANE1              0x20
	#define PM_PHY_CFG_LANE_POLARITY_TX_LANE2              0x40
	#define PM_PHY_CFG_LANE_POLARITY_TX_LANE3              0x80
	/* same format as nvm cfg */
	u32 preemphasis;
	#define PM_PHY_CFG_PREEMPHASIS_LANE0_MASK              0xff
	#define PM_PHY_CFG_PREEMPHASIS_LANE0_SFT                  0
	#define PM_PHY_CFG_PREEMPHASIS_LANE1_MASK            0xff00
	#define PM_PHY_CFG_PREEMPHASIS_LANE1_SFT                  8
	#define PM_PHY_CFG_PREEMPHASIS_LANE2_MASK          0xff0000
	#define PM_PHY_CFG_PREEMPHASIS_LANE2_SFT                 16
	#define PM_PHY_CFG_PREEMPHASIS_LANE3_MASK        0xff000000
	#define PM_PHY_CFG_PREEMPHASIS_LANE3_SFT                 24
	/* same format as nvm cfg */
	u32 driver_current;
	#define PM_PHY_CFG_DRIVER_CURRENT_LANE0_MASK           0xff
	#define PM_PHY_CFG_DRIVER_CURRENT_LANE0_SFT               0
	#define PM_PHY_CFG_DRIVER_CURRENT_LANE1_MASK         0xff00
	#define PM_PHY_CFG_DRIVER_CURRENT_LANE1_SFT               8
	#define PM_PHY_CFG_DRIVER_CURRENT_LANE2_MASK       0xff0000
	#define PM_PHY_CFG_DRIVER_CURRENT_LANE2_SFT              16
	#define PM_PHY_CFG_DRIVER_CURRENT_LANE3_MASK     0xff000000
	#define PM_PHY_CFG_DRIVER_CURRENT_LANE3_SFT              24
	/* same format as nvm cfg */
	u32 speed_cap_mask;
	#define PM_PHY_CFG_SPEED_CAP_MASK_DRV_1G                0x1
	#define PM_PHY_CFG_SPEED_CAP_MASK_DRV_10G               0x2
	#define PM_PHY_CFG_SPEED_CAP_MASK_DRV_25G               0x4
	#define PM_PHY_CFG_SPEED_CAP_MASK_DRV_40G               0x8
	#define PM_PHY_CFG_SPEED_CAP_MASK_DRV_50G              0x10
	#define PM_PHY_CFG_SPEED_CAP_MASK_FW_1G             0x10000
	#define PM_PHY_CFG_SPEED_CAP_MASK_FW_10G            0x20000
	#define PM_PHY_CFG_SPEED_CAP_MASK_FW_25G            0x40000
	#define PM_PHY_CFG_SPEED_CAP_MASK_FW_40G            0x80000
	#define PM_PHY_CFG_SPEED_CAP_MASK_FW_50G           0x100000
	/* same format as nvm cfg */
	u32 phy_cfg_mode;
	/* serdes network interface */
	#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_MASK       0xff
	#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_SFT           0
		#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_BYPASS (0x0 << 0)
		#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_KR   (0x1 << 0)
		#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_KR2  (0x2 << 0)
		#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_KR4  (0x3 << 0)
		#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_XFI  (0x4 << 0)
		#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_SFI  (0x5 << 0)
		#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_1000X (0x6 << 0)
		#define PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_SGMII (0x7 << 0)
	#define PM_PHY_CFG_PHY_CFG_MODE_AN_MODE_MASK         0xff00
	#define PM_PHY_CFG_PHY_CFG_MODE_AN_MODE_SFT               8
		#define PM_PHY_CFG_PHY_CFG_MODE_AN_MODE_NONE     (0x0 << 8)
		#define PM_PHY_CFG_PHY_CFG_MODE_AN_MODE_CL73     (0x1 << 8)
		#define PM_PHY_CFG_PHY_CFG_MODE_AN_MODE_CL37     (0x2 << 8)
		#define PM_PHY_CFG_PHY_CFG_MODE_AN_MODE_CL73_BAM (0x3 << 8)
		#define PM_PHY_CFG_PHY_CFG_MODE_AN_MODE_CL37_BAM (0x4 << 8)
		#define PM_PHY_CFG_PHY_CFG_MODE_AN_MODE_SGMII    (0x5 << 8)
	/* optical module */
	#define PM_PHY_CFG_PHY_CFG_MODE_OPTIC_MODULE_MASK   0xff0000
	#define PM_PHY_CFG_PHY_CFG_MODE_OPTIC_MODULE_SFT         16
		#define PM_PHY_CFG_PHY_CFG_MODE_OPTIC_MODULE_I2C_SEL_0 (0x0 << 16)
		#define PM_PHY_CFG_PHY_CFG_MODE_OPTIC_MODULE_I2C_SEL_1 (0x1 << 16)
		#define PM_PHY_CFG_PHY_CFG_MODE_OPTIC_MODULE_I2C_SEL_2 (0x2 << 16)
		#define PM_PHY_CFG_PHY_CFG_MODE_OPTIC_MODULE_I2C_SEL_NA (0xff << 16)
	u32 core_addr;
	/* same format as nvm cfg */
	u32 sfp_ctrl;
	#define PM_PHY_CFG_SFP_CTRL_TX_LASER_MASK              0xff
	#define PM_PHY_CFG_SFP_CTRL_TX_LASER_SFT                  0
	#define PM_PHY_CFG_SFP_CTRL_MOD_ABS_MASK             0xff00
	#define PM_PHY_CFG_SFP_CTRL_MOD_ABS_SFT                   8
	#define PM_PHY_CFG_SFP_CTRL_PWR_DIS_MASK           0xff0000
	#define PM_PHY_CFG_SFP_CTRL_PWR_DIS_SFT                  16
	/* same format as nvm cfg */
	u32 qsfp_ctrl;
	#define PM_PHY_CFG_QSFP_CTRL_WRONG_MOD_TYPE_MASK       0xff
	#define PM_PHY_CFG_QSFP_CTRL_WRONG_MOD_TYPE_SFT           0
	#define PM_PHY_CFG_QSFP_CTRL_CUR_FAULT_MASK          0xff00
	#define PM_PHY_CFG_QSFP_CTRL_CUR_FAULT_SFT                8
	#define PM_PHY_CFG_QSFP_CTRL_LP_MODE_MASK          0xff0000
	#define PM_PHY_CFG_QSFP_CTRL_LP_MODE_SFT                 16
	#define PM_PHY_CFG_QSFP_CTRL_RESET_MASK          0xff000000
	#define PM_PHY_CFG_QSFP_CTRL_RESET_SFT                   24
	/* Board Config */
	u32 board_cfg;
	/* Board config#1 */
	u32 board_cfg1;
	/* PHY config */
	u32 ext_phy;
	/* MAC address */
	u8 mac_addr[6];
} pm_phy_cfg_t, *ppm_phy_cfg_t;

typedef struct pa_reg {
	void *pa_pm_sw_arb;	/* 0x250000 */
	/* Multi Field Register. */
#define PA_PM_ARB_REQ_SET_MASK                   0xfUL
#define PA_PM_ARB_REQ_SET_SFT                    0
	/*
	 * SW Arbitration Request Set Bits. To access the PM Registers,
	 * each agent must set it's Request bit and wait for the
	 * corresponding Grant bit to be set. Once Grant is set, that
	 * agent is given exclusive access to the PM Registers. When
	 * finished, the Request bit should be Cleared to allow other
	 * agents to access the PM. Agent[0] is the TimeStamp logic
	 * internal to the PA. Agent[3:1] are for different CPUs through
	 * the chip. These bits are self-clearing.
	 */
#define PA_PM_ARB_REQ_CLR_MASK                   0xf00UL
#define PA_PM_ARB_REQ_CLR_SFT                    8
	/*
	 * SW Arbitration Request Clear Bits. Once an Agent is finished
	 * accessing the PM Registers, it's Request must be cleared.
	 * These bits are self-clearing.
	 */
#define PA_PM_ARB_GRANT_BITS_MASK                0xf0000UL
#define PA_PM_ARB_GRANT_BITS_SFT                 16
	/*
	 * SW Arbitration Grant Bits. Only one of these bits will be set
	 * at any time. When Grant is set for a certain Agent, that
	 * Agent has exclusive access to the PM Registers. Once Granted,
	 * the Agent should assume all access control registers have
	 * been modified by other Agents and should be reconfigured.
	 */
#define PA_PM_ARB_REQUEST_BITS_MASK              0xf000000UL
#define PA_PM_ARB_REQUEST_BITS_SFT               24
	/*
	 * SW Arbitration Request Bits. These bits show which Agents are
	 * currently requesting access to the PM Registers.
	 */
	void *pa_pm_if_cmd;	/* 0x250004 */
	void *pa_pm_if_wctrl;	/* 0x250004 */
	void *pa_pm_if_rctrl;	/* 0x250004 */
	/* Multi Field Register. */
#define PA_PM_IF_TYPE                            0x1UL
#define PA_PM_IF_TYPE_MEMORY_ACCESS              (0x1UL << 0)
#define PA_PM_IF_TYPE_REGISTER_ACCESS            (0x0UL << 0)
#define PA_PM_IF_TYPE_ACCESS_SFT             (23)
#define PA_PM_IF_ADDR_AUTO_INC                   0x2UL
	/*
	 * Setting this bit to 1 tells the interface logic auto
	 * increment the address based on the programmed byte count. For
	 * Register operations, address bits [24:8] are incremented. For
	 * Memory operations, address bits [25:0] are incremented.
	 */
#define PA_PM_IF_ADDR_AUTO_INC_SIZE_MASK         0xf0UL
#define PA_PM_IF_ADDR_AUTO_INC_SIZE_SFT          4
	/*
	 * In Port Macro the register addresses are index addresses, a
	 * 64bit register is considered a single register, the next 64
	 * bit register will be at addr+1. This register allows HW to
	 * automatically increment to a programmable index. Normally the
	 * value will be just 1.
	 */
#define PA_PM_IF_BYTE_COUNT_MASK                 0xff00UL
#define PA_PM_IF_BYTE_COUNT_SFT                  24
	/* Byte Count of the transaction. Limit to 32bytes. */
#define PA_PM_IF_RESET_FSM                       0x10000UL
	/* Reset the Register interface state machine */
	void *pa_pm_if_wr_go;	/* 0x250008 */
	void *pa_pm_if_rd_go;	/* 0x250008 */
	void *pa_pm_if_status_mask;	/* 0x250008 */
	void *pa_pm_if_dbg_ctrl;	/* 0x250008 */
#define  PA_PM_IF_GO (0x1UL << 31)
	void *pa_pm_if_status;	/* 0x250008 */
	/* Multi Field Register. */
#define PA_PM_IF_BUSY                            0x1UL
#define PA_PM_IF_BUSY_BUSY                       (0x1UL << 0)
	/* State Machine is busy */
#define PA_PM_IF_RD_DONE                            (0x1UL << 31)
#define PA_PM_IF_DONE_COMPLETED                  (0x1UL << 1)
	/* State Machine has completed operation. */
#define PA_PM_IF_RD_ERROR                           (0x1UL << 30)
#define PA_PM_IF_ERROR_ERROR                     (0x1UL << 2)
	/* Last transaction resulted in an error */
#define PA_PM_IF_WR_DONE                            (0x1UL << 29)
#define PA_PM_IF_WR_ERROR                           (0x1UL << 28)
	void *pa_pm_if_waddr;	/* 0x25000c RW */
	void *pa_pm_if_raddr;	/* 0x25000c RW */
	/* Address of PM IF transaction. Fields: For Register Access */
#define PM_IF_ADDR_REG_STAGE_ID_MASK             0xfc000000UL
#define PM_IF_ADDR_REG_STAGE_ID_SFT              26
	/* reg_stage_id[31:26] :Stage ID */
#define PM_IF_ADDR_REG_TYPE                      0x2000000UL
	/* reg_type[25:25] :Register Type */
#define PM_IF_ADDR_REG_TYPE_PER_PORT             (0x0UL << 25)
	/* Per Port */
#define PM_IF_ADDR_REG_TYPE_GEN_REG              (0x1UL << 25)
	/* Generic Register */
#define PM_IF_ADDR_REG_OFFSET_MASK               0x1ffff00UL
#define PM_IF_ADDR_REG_OFFSET_SFT                8
	/* reg_offset[24:8] :Register Offset */
#define PM_IF_ADDR_PORT_NUM_MASK                 0xffUL
#define PM_IF_ADDR_PORT_NUM_SFT                  0
	/* port_num[7:0] :Port Number For Memory Access */
#define PM_IF_ADDR_MEM_STAGE_ID_MASK             0xfc000000UL
#define PM_IF_ADDR_MEM_STAGE_ID_SFT              26
	/* mem_stage_id[31:26] :Stage ID[5:0] */
#define PM_IF_ADDR_MEM_IDX_MASK                  0x3ffffffUL
#define PM_IF_ADDR_MEM_IDX_SFT                   0
	/* mem_idx[25:0] :Memory Index */
	void *pa_pm_if_wrdata;	/* 0x250010 RW */
	/*
	 * Write Data. This should be the last item written for a
	 * transaction. Writing to this register will kick off a
	 * transaction
	 */
	void *pa_pm_if_rddata;	/* 0x250014 RO */
	/* Read Data */
	void *pa_debug_cntl;	/* 0x250018 */
	/* Multi Field Register. */
#define PA_DEBUG_SEL_MASK                        0xffffUL
#define PA_DEBUG_SEL_SFT                         0
	/* Select Debug Status. */
#define PA_PM_MAX_CREDIT_MASK                    0xff0000UL
#define PA_PM_MAX_CREDIT_SFT                     16
	/* Maximum TX Credit from PortMacro. */
#define PA_PM_CREDIT_OVERFLOW_CTRL_EN            0x20000000UL
	/* Enable PortMacro TX Credit Overflow Prevention Logic. */
#define PA_TIMER_TEST_MODE                       0x40000000UL
	/* Purge Timer Test Mode, 100us pulse is shortened to 160ns. */
#define PA_DEBUG_EN                              0x80000000UL
	/* Enable Debug. */
	void *pa_debug_status;	/* 0x25001c RO */
	/* Debug Status. */
	/* ** Union ** PA_REG_DEBUG_CNTL[15:0] = 16'h0 debug_sel0_data */
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_MASK     0xffffffffUL
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_SFT      0
	/*
	 * pa_rf_debug_status[31:0]:Read Packet Counts:
	 * {rx_pkt_cnt[15:0], tx_pkt_cnt[15:0]}
	 */
	/* ** Union ** PA_REG_DEBUG_CNTL[15:0] = 16'h1 debug_sel1_data */
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_1_MASK   0xffffffffUL
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_1_SFT    0
	/*
	 * pa_rf_debug_status[31:0]:Read Signals for debug:
	 * {pa_chimp_idle_idle, data_state_idle[PORTS_NUM-1:0],
	 * rx_portq_overflow, rx_portq_underflow, rx_portq_full,
	 * tx_portq_full[PORTS_NUM-1:0], rx_portq_empty}
	 */
	/* ** Union ** PA_REG_DEBUG_CNTL[15:0] = 16'h2 debug_sel2_data */
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_2_MASK   0xffffffffUL
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_2_SFT    0
	/*
	 * pa_rf_debug_status[31:0]:Read Packet Counts:
	 * {port0_rx_pkt_cnt[15:0], port0_tx_pkt_cnt[15:0]}
	 */
	/* ** Union ** PA_REG_DEBUG_CNTL[15:0] = 16'h3 debug_sel3_data */
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_3_MASK   0xffffffffUL
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_3_SFT    0
	/*
	 * pa_rf_debug_status[31:0]:Read Packet Counts:
	 * {port1_rx_pkt_cnt[15:0], port1_tx_pkt_cnt[15:0]}
	 */
	/* ** Union ** PA_REG_DEBUG_CNTL[15:0] = 16'h4 debug_sel4_data */
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_4_MASK   0xffffffffUL
#define DEBUG_STATUS_PA_RF_DEBUG_STATUS_4_SFT    0
	/*
	 * pa_rf_debug_status[31:0]:Read Packet Counts:
	 * {port2_rx_pkt_cnt[15:0], port2_tx_pkt_cnt[15:0]}
	 */
	void *pa_loopback_port_byte_cnt;	/* 0x250020 */
	/* Multi Field Register. */
#define PA_STAT_BYTE_CNT_MASK                    0xffffffffUL
#define PA_STAT_BYTE_CNT_SFT                     0
	/* R2C, byte count for packets in loopback. */
	void *pa_loopback_port_pkt_cnt;	/* 0x250024 */
	/* Multi Field Register. */
#define PA_STAT_PKT_MASK                         0xffffffffUL
#define PA_STAT_PKT_SFT                          0
	/* R2C, packet count in loopback mode. */
	void *pa_loopback_bandwidth_cntl;	/* 0x250028 */
	/* Multi Field Register. */
#define PA_LPBK_BANDWIDTH_DIV_MASK               0xffUL
#define PA_LPBK_BANDWIDTH_DIV_SFT                0
	/* Divide count used to control loopback traffic bandwidth. */
	u64 unused_0[5];	/* 0x25002c - 0x25003c */
	void *pa_tx_port_watermark[3];	/* 0x250040 - 0x250048 RW */
	/*
	 * TX Port Watermark value. To compensate for FIFO lookahead
	 * mechanism, this value should be 1 less than the intended
	 * watermark value.
	 */
#define PA_TX_PORT_WATERMARK_MASK                0xffUL	/* Reset: 6 */
#define PA_TX_PORT_WATERMARK_SFT                 0
	u64 unused_1[5];	/* 0x25004c - 0x25005c */
	void *pa_port_config;	/* 0x250060 */
	/* Multi Field Register. */
#define PA_ENABLE_LOOPBACK                       0x1UL
	/* Enable loopback port. */
#define PA_ENABLE_LOOPBACK_CRC_PAD               0x2UL
	/* Enable loopback port CRC Padding. */
#define PA_ENABLE_LOOPBACK_INT_HDR_SKIP          0x4UL
	/*
	 * Enable 4-Byte Internal Header Skip on Loopback Port Packet
	 * Type Decoding Logic. This Packet Type is Used by the Status
	 * Tracking Block.
	 */
#define PA_LOOPBACK_PFC_EN                       0x8UL
	/* Loopback PFC Enable. */
#define PA_RXP_MAC_FC_CMB_EN                     0x10UL
	/*
	 * Enable Logic that Combines RXP and MAC Flow Control To
	 * Backpressure TX.
	 */
	/* Soft_init for each TX Port. */
#define PA_TX_PORT_INIT_RSVD_MASK                0xe000UL
#define PA_TX_PORT_INIT_RSVD_SFT                 13
	/* rsvd[15:13] :Reserved. */
#define PA_TX_PORT_INIT_TX_PORT4_RST             0x1000UL
	/* tx_port4_rst[12:12] :TX Port 4 Reset. */
#define PA_TX_PORT_INIT_TX_PORT3_RST             0x800UL
	/* tx_port3_rst[11:11] :TX Port 3 Reset. */
#define PA_TX_PORT_INIT_TX_PORT2_RST             0x400UL
	/* tx_port2_rst[10:10] :TX Port 2 Reset. */
#define PA_TX_PORT_INIT_TX_PORT1_RST             0x200UL
	/* tx_port1_rst[9:9] :TX Port 1 Reset. */
#define PA_TX_PORT_INIT_TX_PORT0_RST             0x100UL
	/* tx_port0_rst[8:8] :TX Port 0 Reset. */
#define PA_RX_INIT                               0x80000000UL
	/* Soft Reset for RX Logic. */
	void *pa_pbus_tstamp_rd_addr_config;	/* 0x250064 */
	/* Multi Field Register. */
#define PA_PBUS_TSTAMP_REG_OFFSET_MASK           0x1ffffUL
#define PA_PBUS_TSTAMP_REG_OFFSET_SFT            0
	/* FIFO 1588 Register Offset on PBUS. */
#define PA_PBUS_TSTAMP_STAGE_ID_MASK             0x3f00000UL
#define PA_PBUS_TSTAMP_STAGE_ID_SFT              20
	/* FIFO 1588 PBUS Access Stage ID. */
	void *pa_int_sts_0;	/* 0x250068 */
	/* Multi Field Register. */
#define PA_PORT0_TFIFO_WR                        0x1UL
	/* Port 0 Time Stamp FIFO Write Interrupt. */
#define PA_PORT1_TFIFO_WR                        0x2UL
	/* Port 1 Time Stamp FIFO Write Interrupt. */
#define PA_PORT2_TFIFO_WR                        0x4UL
	/* Port 2 Time Stamp FIFO Write Interrupt. */
#define PA_PORT3_TFIFO_WR                        0x8UL
	/* Port 3 Time Stamp FIFO Write Interrupt. */
#define PA_PBUS_ERROR                            0x10UL
	/* PBUS Access Error Interrupt. */
#define PA_PORT0_PURGE_TRUNC                     0x20UL
	/* Port 0 Purge Packet Truncate Interrupt. */
#define PA_PORT0_TIMEOUT                         0x40UL
	/* Port 0 Timer Timeout Interrupt. */
#define PA_PORT0_PURGE_DONE                      0x80UL
	/* Port 0 Purge Done Interrupt. */
#define PA_PORT1_PURGE_TRUNC                     0x100UL
	/* Port 1 Purge Packet Truncate Interrupt. */
#define PA_PORT1_TIMEOUT                         0x200UL
	/* Port 1 Timer Timeout Interrupt. */
#define PA_PORT1_PURGE_DONE                      0x400UL
	/* Port 1 Purge Done Interrupt. */
#define PA_PORT2_PURGE_TRUNC                     0x800UL
	/* Port 2 Purge Packet Truncate Interrupt. */
#define PA_PORT2_TIMEOUT                         0x1000UL
	/* Port 2 Timer Timeout Interrupt. */
#define PA_PORT2_PURGE_DONE                      0x2000UL
	/* Port 2 Purge Done Interrupt. */
#define PA_PORT3_PURGE_TRUNC                     0x4000UL
	/* Port 3 Purge Packet Truncate Interrupt. */
#define PA_PORT3_TIMEOUT                         0x8000UL
	/* Port 3 Timer Timeout Interrupt. */
#define PA_PORT3_PURGE_DONE                      0x10000UL
	/* Port 3 Purge Done Interrupt. */
#define PA_LINK_STATUS_CHANGE                    0x80000000UL
	/* Link Status Change Interrupt. */
	void *pa_int_mask_0;	/* 0x25006c */
	/* Multi Field Register. */
#define PA_PORT0_TFIFO_WR                        0x1UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT0_TFIFO_WR .
	 */
#define PA_PORT1_TFIFO_WR                        0x2UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT1_TFIFO_WR .
	 */
#define PA_PORT2_TFIFO_WR                        0x4UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT2_TFIFO_WR .
	 */
#define PA_PORT3_TFIFO_WR                        0x8UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT3_TFIFO_WR .
	 */
#define PA_PBUS_ERROR                            0x10UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PBUS_ERROR .
	 */
#define PA_PORT0_PURGE_TRUNC                     0x20UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT0_PURGE_TRUNC .
	 */
#define PA_PORT0_TIMEOUT                         0x40UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT0_TIMEOUT .
	 */
#define PA_PORT0_PURGE_DONE                      0x80UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT0_PURGE_DONE .
	 */
#define PA_PORT1_PURGE_TRUNC                     0x100UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT1_PURGE_TRUNC .
	 */
#define PA_PORT1_TIMEOUT                         0x200UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT1_TIMEOUT .
	 */
#define PA_PORT1_PURGE_DONE                      0x400UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT1_PURGE_DONE .
	 */
#define PA_PORT2_PURGE_TRUNC                     0x800UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT2_PURGE_TRUNC .
	 */
#define PA_PORT2_TIMEOUT                         0x1000UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT2_TIMEOUT .
	 */
#define PA_PORT2_PURGE_DONE                      0x2000UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT2_PURGE_DONE .
	 */
#define PA_PORT3_PURGE_TRUNC                     0x4000UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT3_PURGE_TRUNC .
	 */
#define PA_PORT3_TIMEOUT                         0x8000UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT3_TIMEOUT .
	 */
#define PA_PORT3_PURGE_DONE                      0x10000UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.PORT3_PURGE_DONE .
	 */
#define PA_LINK_STATUS_CHANGE                    0x80000000UL
	/*
	 * This bit masks, when set, the Interrupt bit:
	 * PA_REG_INT_STS_0.LINK_STATUS_CHANGE .
	 */
	void *pa_int_sts_wr_0;	/* 0x250070 */
	/* Multi Field Register. */
#define PA_PORT0_TFIFO_WR                        0x1UL
	/* Port 0 Time Stamp FIFO Write Interrupt. */
#define PA_PORT1_TFIFO_WR                        0x2UL
	/* Port 1 Time Stamp FIFO Write Interrupt. */
#define PA_PORT2_TFIFO_WR                        0x4UL
	/* Port 2 Time Stamp FIFO Write Interrupt. */
#define PA_PORT3_TFIFO_WR                        0x8UL
	/* Port 3 Time Stamp FIFO Write Interrupt. */
#define PA_PBUS_ERROR                            0x10UL
	/* PBUS Access Error Interrupt. */
#define PA_PORT0_PURGE_TRUNC                     0x20UL
	/* Port 0 Purge Packet Truncate Interrupt. */
#define PA_PORT0_TIMEOUT                         0x40UL
	/* Port 0 Timer Timeout Interrupt. */
#define PA_PORT0_PURGE_DONE                      0x80UL
	/* Port 0 Purge Done Interrupt. */
#define PA_PORT1_PURGE_TRUNC                     0x100UL
	/* Port 1 Purge Packet Truncate Interrupt. */
#define PA_PORT1_TIMEOUT                         0x200UL
	/* Port 1 Timer Timeout Interrupt. */
#define PA_PORT1_PURGE_DONE                      0x400UL
	/* Port 1 Purge Done Interrupt. */
#define PA_PORT2_PURGE_TRUNC                     0x800UL
	/* Port 2 Purge Packet Truncate Interrupt. */
#define PA_PORT2_TIMEOUT                         0x1000UL
	/* Port 2 Timer Timeout Interrupt. */
#define PA_PORT2_PURGE_DONE                      0x2000UL
	/* Port 2 Purge Done Interrupt. */
#define PA_PORT3_PURGE_TRUNC                     0x4000UL
	/* Port 3 Purge Packet Truncate Interrupt. */
#define PA_PORT3_TIMEOUT                         0x8000UL
	/* Port 3 Timer Timeout Interrupt. */
#define PA_PORT3_PURGE_DONE                      0x10000UL
	/* Port 3 Purge Done Interrupt. */
#define PA_LINK_STATUS_CHANGE                    0x80000000UL
	/* Link Status Change Interrupt. */
	u64 unused_2[3];	/* 0x250074 - 0x25007c */
	void *pa_port_tstamph[2];	/* 0x250080 - 0x250084 RO */
	/* Upper 32 bits of Timestamp */
	u64 unused_3[2];	/* 0x250088 - 0x25008c */
	void *pa_port_tstampl[2];	/* 0x250090 - 0x250094 RO */
	/* Lower 32 bits of Timestamp */
	u64 unused_4[2];	/* 0x250098 - 0x25009c */
	void *pa_port_seqid[2];	/* 0x2500a0 - 0x2500a4 RO */
	/* Sequence ID, reading this register causes the FIFO to advance. */
#define PA_PORT_SEQID_MASK                       0xffffUL	/* Reset: 0 */
#define PA_PORT_SEQID_SFT                        0
	u64 unused_5[2];	/* 0x2500a8 - 0x2500ac */
	void *pa_port_tfifo_stat[2];	/* 0x2500b0 - 0x2500b4 RW */
	/* Fields: */
#define PORT_TFIFO_STAT_FIFO_EMPTY               0x2UL
	/* FIFO_EMPTY[1:1] :Timestamp FIFO Empty. (RO) */
#define PORT_TFIFO_STAT_FIFO_FULL                0x1UL
	/* FIFO_FULL[0:0] :Timestamp FIFO Full. (RO) */
	u64 unused_6[2];	/* 0x2500b8 - 0x2500bc */
	void *pa_purge_timer_ns2_only[4];	/* 0x2500c0 - 0x2500cc RW */
	/*
	 * Reg removed in Cumulus!!! Purge Timeout Timer, Default is
	 * Equivalent to 10s.
	 */
	void *pa_purge_cntl_ns2_only[4];	/* 0x2500d0 - 0x2500dc RW */
	/* Reg removed in Cumulus!!! Fields: */
#define PA_PURGE_CNTL_NS2_ONLY_PURGE_STAT        0x4UL
	/*
	 * PURGE_STAT[2:2] :Set by HW when Purge Starts, SW Writes 1 to
	 * Stop Purge and Clear bit. (WC)
	 */
#define PA_PURGE_CNTL_NS2_ONLY_PURGE_EN          0x2UL
	/* PURGE_EN[1:1] :Enable Packet Drop after Timeout. (RW) */
#define PA_PURGE_CNTL_NS2_ONLY_TIMER_EN          0x1UL
	/* TIMER_EN[0:0] :Enable Timer for Purge Timeout. (RW) */
	void *pa_purge_cnt_ns2_only[4];	/* 0x2500e0 - 0x2500ec RW */
	/* Reg removed in Cumulus!!! Fields: */
#define PA_PURGE_CNT_NS2_ONLY_PA_PURGE_CNT_MASK  0xffffUL
#define PA_PURGE_CNT_NS2_ONLY_PA_PURGE_CNT_SFT   0
	/* PA_PURGE_CNT[15:0] :Purge Packet Count. (W0C) */
	u64 unused_7[4];	/* 0x2500f0 - 0x2500fc */
	void *pa_purge_timer_new[3];	/* 0x250100 - 0x250108 RW */
	/* Purge Timeout Timer, Default is Equivalent to 10s. */
	u64 unused_8[61];	/* 0x25010c - 0x2501fc */
	void *pa_purge_cntl_new[3];	/* 0x250200 - 0x250208 RW */
	/* Fields: */
#define PA_PURGE_CNTL_NEW_PURGE_STAT             0x4UL
	/*
	 * PURGE_STAT[2:2] :Set by HW when Purge Starts, SW Writes 1 to
	 * Stop Purge and Clear bit. (WC)
	 */
#define PA_PURGE_CNTL_NEW_PURGE_EN               0x2UL
	/* PURGE_EN[1:1] :Enable Packet Drop after Timeout. (RW) */
#define PA_PURGE_CNTL_NEW_TIMER_EN               0x1UL
	/* TIMER_EN[0:0] :Enable Timer for Purge Timeout. (RW) */
	u64 unused_9[61];	/* 0x25020c - 0x2502fc */
	void *pa_purge_cnt_new[3];	/* 0x250300 - 0x250308 RW */
	/* Fields: */
#define PA_PURGE_CNT_NEW_PA_PURGE_CNT_MASK       0xffffUL
#define PA_PURGE_CNT_NEW_PA_PURGE_CNT_SFT        0
	/* PA_PURGE_CNT[15:0] :Purge Packet Count. (W0C) */
	u64 unused_10[829];	/* 0x25030c - 0x250ffc */
	void *pa_dbg_select;	/* 0x251000 RW */
	/* DBMUX register for selecting a line to output */
#define PA_DBG_SELECT_MASK                       0xffUL	/* Reset: 0 */
#define PA_DBG_SELECT_SFT                        0
	void *pa_dbg_dword_enable;	/* 0x251004 RW */
	/*
	 * DBMUX register. Bit mask for enabling dword (128bit line) /
	 * qword (256bit line) in the selected line (before shift).for
	 * selecting a line to output
	 */
#define PA_DBG_DWORD_ENABLE_MASK                 0xfUL	/* Reset: 0 */
#define PA_DBG_DWORD_ENABLE_SFT                  0
	void *pa_dbg_shift;	/* 0x251008 RW */
	/*
	 * DBMUX register. Circular dword (128bit line) / qword (256bit
	 * line) right shifting of the selected line (after the
	 * masking).
	 */
#define PA_DBG_SHIFT_MASK                        0x3UL	/* Reset: 0 */
#define PA_DBG_SHIFT_SFT                         0
	void *pa_dbg_force_valid;	/* 0x25100c RW */
	/*
	 * DBMUX register. Bit mask for forcing the valid signal per
	 * dword (128bit line) / qword (256bit line) (before shift).
	 */
#define PA_DBG_FORCE_VALID_MASK                  0xfUL	/* Reset: 0 */
#define PA_DBG_FORCE_VALID_SFT                   0
	void *pa_dbg_force_frame;	/* 0x251010 RW */
	/*
	 * DBMUX register. bit mask for forcing the frame signal per
	 * dword (128bit line) / qword (256bit line) (before shift).
	 */
#define PA_DBG_FORCE_FRAME_MASK                  0xfUL	/* Reset: 0 */
#define PA_DBG_FORCE_FRAME_SFT                   0
	u64 unused_11[3];	/* 0x251014 - 0x25101c */
	void *pa_dbg_out_data[8];	/* 0x251020 - 0x25103c RO */
	/* "Wide-Bus" register. Dbgmux output data */
	void *pa_dbg_out_valid;	/* 0x251040 RO */
	/* Dbgmux output valid */
#define PA_DBG_OUT_VALID_MASK                    0xfUL	/* Reset: 0 */
#define PA_DBG_OUT_VALID_SFT                     0
	void *pa_dbg_out_frame;	/* 0x251044 RO */
	/* Dbgmux output frame */
#define PA_DBG_OUT_FRAME_MASK                    0xfUL	/* Reset: 0 */
#define PA_DBG_OUT_FRAME_SFT                     0
	void *pa_eco_reserved;	/* 0x251048 RW */
	/* reserved bits for ECOs */
#define PA_ECO_RESERVED_MASK                     0xffffUL	/* Reset: 0 */
#define PA_ECO_RESERVED_SFT                      0
	void *pa_pm_if_timeout_val;	/* 0x25104c RW */
	/*
	 * This register contains a Timeout Value for PBUS Transactions.
	 * If there is no response to a PBus Read/Write after this
	 * number of cycles, the transaction is terminated and marked as
	 * an error.
	 */
#define PA_PM_IF_TIMEOUT_VAL_MASK                0x3ffUL	/* Reset: 100 */
#define PA_PM_IF_TIMEOUT_VAL_SFT                 0
	u64 unused_12[4];	/* 0x251050 - 0x25105c */
	void *pa_per_port_cntl[3];	/* 0x251060 - 0x251068 RW */
	/* Fields: */
#define PER_PORT_CNTL_RXP_CNTL_PKT_FILTER_EN     0x10UL
	/*
	 * RXP_CNTL_PKT_FILTER_EN[4:4]:Per Panel Port RXP Control Packet
	 * Filter Enable.
	 */
#define PER_PORT_CNTL_RXP_PFC_PKT_FILTER_EN      0x8UL
	/*
	 * RXP_PFC_PKT_FILTER_EN[3:3]:Per Panel Port RXP PFC Packet
	 * Filter Enable.
	 */
#define PER_PORT_CNTL_RXP_PAUSE_PKT_FILTER_EN    0x4UL
	/*
	 * RXP_PAUSE_PKT_FILTER_EN[2:2]:Per Panel Port RXP Pause Packet
	 * Filter Enable.
	 */
#define PER_PORT_CNTL_RXP_BRCM_HDR_EN            0x2UL
	/*
	 * RXP_BRCM_HDR_EN[1:1]:Per Port Broadcom Header Handling Enable
	 * for RX Path.
	 */
#define PER_PORT_CNTL_TXP_BRCM_HDR_EN            0x1UL
	/*
	 * TXP_BRCM_HDR_EN[0:0]:Per Port Broadcom Header Handling Enable
	 * for TX Path.
	 */
	u64 unused_13[5];	/* 0x25106c - 0x25107c */
	void *pa_per_panel_port_mapping[3];	/* 0x251080 - 0x251088 RW */
	/* Fields: */
#define PER_PANEL_PORT_MAPPING_PM_PORT_MAP_MASK  0xfUL
#define PER_PANEL_PORT_MAPPING_PM_PORT_MAP_SFT   0
	/*
	 * PM_PORT_MAP[3:0] :Per Port Macro TX Port or Loopback Port
	 * Mapping. Default is TXP0 to PM Port0, TXP1 to PM Port1, TXP2
	 * to Loopback Port when Loopback is enabled.
	 */
	u64 unused_14[5];	/* 0x25108c - 0x25109c */
	void *pa_ep_to_port_regen_crc_const;	/* 0x2510a0 RW */
	/* Regenerate CRC */
#define PA_EP_TO_PORT_REGEN_CRC_CONST            0x1UL	/* Reset: 0 */
	void *pa_port_enable_status;	/* 0x2510a4 RO */
	/* Port Enable Status from Port Macro. */
#define PA_PORT_ENABLE_STATUS_MASK               0x7UL	/* Reset: 0 */
#define PA_PORT_ENABLE_STATUS_SFT                0
	void *pa_port_to_ep_port_mode;	/* 0x2510a8 RO */
	/* Number of ports supported by Port Macro. */
#define PA_PORT_TO_EP_PORT_MODE_MASK             0xfUL	/* Reset: 0 */
#define PA_PORT_TO_EP_PORT_MODE_SFT              0
	void *pa_pm4x25_lane_0_status;	/* 0x2510ac RO */
	/* Status of the port. */
#define PA_PM4X25_LANE_0_STATUS_MASK             0x7UL	/* Reset: 0 */
#define PA_PM4X25_LANE_0_STATUS_SFT              0
	void *pa_pm4x25_lane_1_status;	/* 0x2510b0 RO */
	/* Status of the port. */
#define PA_PM4X25_LANE_1_STATUS_MASK             0x7UL	/* Reset: 0 */
#define PA_PM4X25_LANE_1_STATUS_SFT              0
	void *pa_pm4x25_lane_2_status;	/* 0x2510b4 RO */
	/* Status of the port. */
#define PA_PM4X25_LANE_2_STATUS_MASK             0x7UL	/* Reset: 0 */
#define PA_PM4X25_LANE_2_STATUS_SFT              0
	u64 unused_15[6];	/* 0x2510b8 - 0x2510cc */
	void *pa_port_to_ip_port_mode;	/* 0x2510d0 RO */
	/* Number of ports supported by Port Macro. */
#define PA_PORT_TO_IP_PORT_MODE_MASK             0xfUL	/* Reset: 0 */
#define PA_PORT_TO_IP_PORT_MODE_SFT              0
	void *pa_link_status;	/* 0x2510d4 */
	/* Multi Field Register. */
#define PA_LINK_STATUS_MASK                      0xfUL
#define PA_LINK_STATUS_SFT                       0
	/* Link Status from Port Macro. */
	u64 unused_16[2];	/* 0x2510d8 - 0x2510dc */
	void *pa_per_rx_port_mapping[3];	/* 0x2510e0 - 0x2510e8 RW */
	/* Fields: */
#define PER_RX_PORT_MAPPING_RX_PORT_MAP_MASK     0xfUL
#define PER_RX_PORT_MAPPING_RX_PORT_MAP_SFT      0
	/*
	 * RX_PORT_MAP[3:0] :Per RXP Port from Port Macro or Loopback
	 * Port Mapping. Default is RXP0 from PM Port0, RXP1 from PM
	 * Port1, RXP2 from Loopback Port when Loopback is enabled.
	 */
	u64 unused_17[965];	/* 0x2510ec - 0x251ffc */
} pa_reg_t;

/* #define PM_WAIT(pdev,uS) udelay(uS) */

#define CHECK_PA_REG_BLK()    (sizeof(pa_reg_t) == 0x2000 && ROFFSET(pa) == 0x250000)
#define PA(x)  (pa.pa_##x)

extern void *apb2pbus_base;

#define APB2PBUS_BASE 0x204d0000
#define APB2PBUS_CTRL_OFFSET (0x0)
#define APB2PBUS_WADDR_OFFSET (0x4)
#define APB2PBUS_WDATA0_OFFSET (0x8)
#define APB2PBUS_WDATA1_OFFSETSET 0x00c
#define APB2PBUS_WCTRL_OFFSETSET 0x028
#define APB2PBUS_WR_GO_OFFSET 0x02c
#define APB2PBUS_RADDR_OFFSET 0x030
#define APB2PBUS_RDATA0_OFFSET 0x034
#define APB2PBUS_RDATA1_OFFSET 0x038
#define APB2PBUS_RCTRL_OFFSET 0x054
#define APB2PBUS_RD_GO_OFFSET 0x058
#define APB2PBUS_RD_WR_STATUS_OFFSET 0x05c
#define APB2PBUS_RD_WR_STATUS_MASK_OFFSET 0x060

#define ETH_SS_0_PORT_ENABLE 0x204e0040

#define XLMAC_PFC_CTRL_RX_PFC_EN_HI 36
#define XLMAC_PFC_CTRL_TX_PFC_EN_HI 37
#define XLMAC_PFC_CTRL_PFC_REFRESH_EN_HI 32

#define ETH_SS_0_PORT_MACRO_EEE_CONFIG 0x204f0004

#define XLPORT_ENABLE_REG__PORT0 0
#define XLPORT_ENABLE_REG__PORT1 1
#define XLPORT_ENABLE_REG__PORT2 2
#define XLPORT_ENABLE_REG__PORT3 3

#define XLPORT_MAC_CONTROL 0x210
#define XLPORT_ENABLE_REG 0x20b
#define XLPORT_MAC_CONTROL__XMAC0_BYPASS_OSTS 1
#define XLPORT_MAC_CONTROL__XMAC0_RESET 0

#define XLPORT_MAC_RSV_MASK 0x208
#define XLMAC_CTRL 0x600
#define XLMAC_MODE 0x601
#define HDR_MODE_IEEE 0x0
#define SPEED_MODE_LINK_1G 0x2
#define XLMAC_MODE__SPEED_MODE_R 4
#define SPEED_MODE_LINK_10G_PLUS 0x4
#define SPEED_MODE_LINK_2G5 0x3

#define XLMAC_RX_CTRL 0x606
#define XLMAC_RX_CTRL__STRICT_PREAMBLE 3
#define XLMAC_CTRL__TX_EN 0
#define XLMAC_CTRL__RX_EN 1
#define XLMAC_CTRL__LOCAL_LPBK 2
#define XLMAC_CTRL__SOFT_RESET 6

#define XLMAC_TX_CRC_CORRUPT_CTRL 0x621 /* per-port */
#define XLPORT_FLOW_CONTROL_CONFIG 0x207
#define XLMAC_PAUSE_CTRL 0x60d
#define PORTMACRO_GPHY_CONFIG_STATUS_BASE 0x204f0000
#define ETH_SS_0_PORT_MACRO_EEE_CONFIG_OFFSET 0x004

#define XLMAC_REG_XLMAC_CTRL_SIZE		       0x8
/* "Wide-Bus" register. */
	/*
	 * EXTENDED_HIG2_EN[14:14]:Extended Higig 2 header is also known
	 * as sirius header. Setting this bit to 0 will disable parsing
	 * for the extended header bit(5th bit of 8th header byte) in
	 * HG2 header and hence all the Higig 2 packets will be treated
	 * as normal Higig2 packets irrespective of extended header bit
	 * value. Default value of this field is 1 which will enable
	 * parsing extended header bit in every Higig 2 header.
	 */
#define XLMAC_CTRL_EXTENDED_HIG2_EN              0x4000UL
	/*
	 * LINK_STATUS_SELECT[13:13]:This configuration chooses between
	 * link status indication from software (SW_LINK_STATUS) or the
	 * hardware link status (hw_link_status)indication from the TSC.
	 * If reset, it selects the software link status
	 */
#define XLMAC_CTRL_LINK_STATUS_SELECT            0x2000UL
	/*
	 * SW_LINK_STATUS[12:12]:Link status indication from Software.
	 * If set, indicates that link is active.
	 */
#define XLMAC_CTRL_SW_LINK_STATUS                0x1000UL
	/*
	 * XGMII_IPG_CHECK_DISABLE[11:11]:If set, this will override the
	 * one column idle/sequence ordered set check before SOP in
	 * XGMII mode - effectively supporting reception of packets with
	 * 1 byte IPG in XGMII mode
	 */
#define XLMAC_CTRL_XGMII_IPG_CHECK_DISABLE       0x800UL
	/*
	 * RS_SOFT_RESET[10:10]:Resets the RS layer functionality -
	 * Fault detection and related responses are disabled and IDLEs
	 * are sent on line
	 */
#define XLMAC_CTRL_RS_SOFT_RESET                 0x400UL
	/* RSVD_5a[9:9] :Reserved */
#define XLMAC_CTRL_RSVD_5A                       0x200UL
	/*
	 * LOCAL_LPBK_LEAK_ENB[8:8]:If set, during the local loopback
	 * mode, the transmit packets are also sent to the transmit line
	 * interface, apart from the loopback operation
	 */
#define XLMAC_CTRL_LOCAL_LPBK_LEAK_ENB           0x100UL
	/* RSVD_4b[7:7] :Reserved */
#define XLMAC_CTRL_RSVD_4B                       0x80UL
	/*
	 * SOFT_RESET[6:6] :If set, disables the corresponding port
	 * logic and status registers only. Packet data and flow control
	 * logic is disabled. Fault handling is active and the MAC will
	 * continue to respond to credits from TSC. When the soft reset
	 * is cleared MAC will issue a fresh set of credits to EP in
	 * transmit side.
	 */
#define XLMAC_CTRL_SOFT_RESET                    0x40UL
	/*
	 * LAG_FAILOVER_EN[5:5]:If set, enable LAG Failover. This bit
	 * has priority over LOCAL_LPBK. The lag failover kicks in when
	 * the link status selected by LINK_STATUS_SELECT transitions
	 * from 1 to 0. TSC clock and TSC credits must be active for lag
	 * failover.
	 */
#define XLMAC_CTRL_LAG_FAILOVER_EN               0x20UL
	/*
	 * REMOVE_FAILOVER_LPBK[4:4]:If set, XLMAC will move from lag
	 * failover state to normal operation. This bit should be set
	 * after link is up.
	 */
#define XLMAC_CTRL_REMOVE_FAILOVER_LPBK          0x10UL
	/* RSVD_1c[3:3] :Reserved */
#define XLMAC_CTRL_RSVD_1C                       0x8UL
	/*
	 * LOCAL_LPBK[2:2] :If set, enables local loopback from TX to
	 * RX. This loopback is on the line side after clock domain
	 * crossing - from the last TX pipeline stage to the first RX
	 * pipeline stage. Hence, TSC clock and TSC credits must be
	 * active for loopback. LAG_FAILOVER_EN should be disabled for
	 * this bit to work.
	 */
#define XLMAC_CTRL_LOCAL_LPBK                    0x4UL
	/*
	 * RX_EN[1:1] :If set, enables MAC receive datapath and
	 * flowcontrol logic.
	 */
#define XLMAC_CTRL_RX_EN                         0x2UL
	/*
	 * TX_EN[0:0] :If set, enables MAC transmit datapath and
	 * flowcontrol logic. When disabled, MAC will respond to TSC
	 * credits with IDLE codewords.
	 */
#define XLMAC_CTRL_TX_EN                         0x1UL
#define XLMAC_REG_XLMAC_MODE				    0x1UL
#define XLMAC_REG_XLMAC_MODE_SIZE		       0x8
/* "Wide-Bus" register. */
	/*
	 * SPEED_MODE[6:4] :Port Speed, used for LED indications and
	 * internal buffer sizing. 0 == LINK_10M - Port Speed is 10
	 * Mbps, full duplex mode 1 == LINK_100M - Port Speed is 100
	 * Mbps, full duplex mode 2 == LINK_1G - Port Speed is 1 Gbps,
	 * full duplex mode 3 == LINK_2G5 - Port Speed is 2.5 Gbps, full
	 * duplex mode 4 == LINK_10G_PLUS - Port Speed is 10Gbps or
	 * higher
	 */
#define XLMAC_MODE_SPEED_MODE_MASK               0x70UL
#define XLMAC_MODE_SPEED_MODE_SFT                4
#define XLMAC_MODE_SPEED_MODE_LINK_10M           (0x0UL << 4)
		/* Port Speed is 10 Mbps, full duplex mode */
#define XLMAC_MODE_SPEED_MODE_LINK_100M          (0x1UL << 4)
		/* Port Speed is 100 Mbps, full duplex mode */
#define XLMAC_MODE_SPEED_MODE_LINK_1G            (0x2UL << 4)
		/* Port Speed is 1 Gbps, full duplex mode */
#define XLMAC_MODE_SPEED_MODE_LINK_2G5           (0x3UL << 4)
		/* Port Speed is 2.5 Gbps, full duplex mode */
#define XLMAC_MODE_SPEED_MODE_LINK_10G_PLUS      (0x4UL << 4)
		/* Port Speed is 10Gbps or higher */
	/*
	 * NO_SOP_FOR_CRC_HG[3:3]:If set, excludes the SOP byte for CRC
	 * calculation in HIGIG modes
	 */
#define XLMAC_MODE_NO_SOP_FOR_CRC_HG             0x8UL
	/*
	 * HDR_MODE[2:0] :Packet Header mode. 0 == IEEE - thernet
	 * format. 1 == HG_PLUS - ormat with 12 Byte header. 2 == HG_2 -
	 * ormat with 16 Byte header (or sirius extension header) 5 ==
	 * SOP_ONLY_IEEE - 0xFB) character is followed by 1st byte of DA
	 * without any preamble (0x55) or SFD (0xD5).
	 */
#define XLMAC_MODE_HDR_MODE_MASK                 0x7UL
#define XLMAC_MODE_HDR_MODE_SFT                  0
#define XLMAC_MODE_HDR_MODE_IEEE                 (0x0UL << 0)
		/* thernet format. */
#define XLMAC_MODE_HDR_MODE_HG_PLUS              (0x1UL << 0)
		/* ormat with 12 Byte header. */
#define XLMAC_MODE_HDR_MODE_HG_2                 (0x2UL << 0)
		/* ormat with 16 Byte header (or sirius extension header) */
#define XLMAC_MODE_HDR_MODE_SOP_ONLY_IEEE        (0x5UL << 0)
		/*
		 * 0xFB) character is followed by 1st byte of DA
		 * without any preamble (0x55) or SFD (0xD5).
		 */
#define XLMAC_REG_XLMAC_SPARE0				    0x2UL
#define XLMAC_REG_XLMAC_SPARE0_SIZE		       0x8
/* "Wide-Bus" register. */
	/* RSVDd[31:0] :SPARE REGISTER 0 */
#define XLMAC_SPARE0_RSVDD_MASK                  0xffffffffUL
#define XLMAC_SPARE0_RSVDD_SFT                   0
#define XLMAC_REG_XLMAC_SPARE1				    0x3UL
#define XLMAC_REG_XLMAC_SPARE1_SIZE		       0x8
/* "Wide-Bus" register. */
	/* RSVDe[1:0] :SPARE REGISTER 1 */
#define XLMAC_SPARE1_RSVDE_MASK                  0x3UL
#define XLMAC_SPARE1_RSVDE_SFT                   0
#define XLMAC_REG_XLMAC_TX_CTRL				    0x4UL
#define XLMAC_REG_XLMAC_TX_CTRL_SIZE		       0x8
/* "Wide-Bus" register. */
	/*
	 * TX_THRESHOLD[41:38] :Indicates the number of 16-byte cells
	 * that are buffered per packet in the Tx CDC FIFO, before
	 * starting transmission of the packet on the line side. (for
	 * idx 1 ...)
	 */
#define XLMAC_TX_CTRL_TX_THRESHOLD_MASK          0x3c0UL
#define XLMAC_TX_CTRL_TX_THRESHOLD_SFT           6
	/*
	 * This[37:37] :If set, MAC accepts packets from the EP but does
	 * not write to the CDC FIFO and discards them on the core side
	 * without updating the statistics. (for idx 1 ...)
	 */
#define XLMAC_TX_CTRL_THIS                       0x20UL
	/*
	 * TX_PREAMBLE_LENGTH[36:33]:Number of preamble bytes for
	 * transmit IEEE packets, this value should include the K.SOP
	 * and SFD character as well (for idx 1 ...)
	 */
#define XLMAC_TX_CTRL_TX_PREAMBLE_LENGTH_MASK    0x1eUL
#define XLMAC_TX_CTRL_TX_PREAMBLE_LENGTH_SFT     1
	/*
	 * Warning: this field spans across multiple words (i.e. idx 0
	 * to 1).THROT_DENOM[32:25] :Number of bytes to transmit before
	 * adding THROT_NUM bytes to the IPG. This configuration is used
	 * for WAN IPG throttling. Refer MAC specs for more details.
	 */
#define XLMAC_TX_CTRL_THROT_DENOM_MASK           0x7fUL
#define XLMAC_TX_CTRL_THROT_DENOM_SFT            25
	/*
	 * THROT_NUM[24:19] :Number of bytes of extra IPG added whenever
	 * THROT_DENOM bytes have been transmitted. This configuration
	 * is used for WAN IPG throttling. Refer MAC specs for more
	 * details.
	 */
#define XLMAC_TX_CTRL_THROT_NUM_MASK             0x1f80000UL
#define XLMAC_TX_CTRL_THROT_NUM_SFT              19
	/*
	 * AVERAGE_IPG[18:12] :Average interpacket gap. Must be
	 * programmed greater than equal to 8. Per packet IPG will vary
	 * based on DIC for 10G+ speeds.
	 */
#define XLMAC_TX_CTRL_AVERAGE_IPG_MASK           0x7f000UL
#define XLMAC_TX_CTRL_AVERAGE_IPG_SFT            12
	/*
	 * PAD_THRESHOLD[11:5] :If padding is enabled, packets smaller
	 * than PAD_THRESHOLD are padded to this size. This must be set
	 * to a value greater than equal to 17 (decimal)
	 */
#define XLMAC_TX_CTRL_PAD_THRESHOLD_MASK         0xfe0UL
#define XLMAC_TX_CTRL_PAD_THRESHOLD_SFT          5
	/*
	 * PAD_EN[4:4] :"If set, enable XLMAC to pad packets smaller
	 * than PAD_THRESHOLD on the Tx"
	 */
#define XLMAC_TX_CTRL_PAD_EN                     0x10UL
	/*
	 * TX_ANY_START[3:3] :If reset, MAC forces the first byte of a
	 * packet to be /S/ character (0xFB) irrespective of incoming EP
	 * data at SOP location in HIGIG modes
	 */
#define XLMAC_TX_CTRL_TX_ANY_START               0x8UL
	/*
	 * DISCARD[2:2] :If set, MAC accepts packets from the EP and
	 * discards them on the line side. The statistics are updated.
	 */
#define XLMAC_TX_CTRL_DISCARD                    0x4UL
	/*
	 * CRC_MODE[1:0] :CRC mode for Transmit Side 0 == APPEND - s
	 * computed on incoming packet data and appended. 1 == KEEP -
	 * acket CRC is passed through without modifications. 2 ==
	 * REPLACE - acket CRC is replaced with the CRC value computed
	 * by the MAC. 3 == PER_PKT_MODE - The CRC mode is determined by
	 * the inputs pins on the port side (txcrcmode)
	 */
#define XLMAC_TX_CTRL_CRC_MODE_MASK              0x3UL
#define XLMAC_TX_CTRL_CRC_MODE_SFT               0
#define XLMAC_TX_CTRL_CRC_MODE_APPEND            (0x0UL << 0)
		/* s computed on incoming packet data and appended. */
#define XLMAC_TX_CTRL_CRC_MODE_KEEP              (0x1UL << 0)
		/* acket CRC is passed through without modifications. */
#define XLMAC_TX_CTRL_CRC_MODE_REPLACE           (0x2UL << 0)
		/*
		 * acket CRC is replaced with the CRC value
		 * computed by the MAC.
		 */
#define XLMAC_TX_CTRL_CRC_MODE_PER_PKT_MODE      (0x3UL << 0)
		/*
		 * The CRC mode is determined by the inputs pins
		 * on the port side (txcrcmode)
		 */
#define XLMAC_REG_XLMAC_TX_MAC_SA			    0x5UL
#define XLMAC_REG_XLMAC_TX_MAC_SA_SIZE		       0x8
/* "Wide-Bus" register. */
	/*
	 * Warning: this field spans across multiple words (i.e. idx 0
	 * to 1).CTRL_SA[47:0] :Source Address for PAUSE/PFC packets
	 * generated by the MAC
	 */
#define XLMAC_TX_MAC_SA_CTRL_SA                  0xffffffffUL
#define XLMAC_REG_XLMAC_RX_CTRL				    0x6UL
#define XLMAC_REG_XLMAC_RX_CTRL_SIZE		       0x8
/* "Wide-Bus" register. */
	/* RSVD_3f[12:12] :Reserved */
#define XLMAC_RX_CTRL_RSVD_3F                    0x1000UL
	/* RSVD_2g[11:11] :Reserved */
#define XLMAC_RX_CTRL_RSVD_2G                    0x800UL
	/*
	 * RUNT_THRESHOLD[10:4]:The runt threshold, below which the
	 * packets are dropped (CDC mode) or marked as runt (Low latency
	 * mode). Should be programmed less than 96 bytes (decimal)
	 */
#define XLMAC_RX_CTRL_RUNT_THRESHOLD_MASK        0x7f0UL
#define XLMAC_RX_CTRL_RUNT_THRESHOLD_SFT         4
	/*
	 * STRICT_PREAMBLE[3:3]:If set, MAC checks for IEEE Ethernet
	 * preamble - K.SOP + 6 "0x55" preamble bytes + "0xD5" SFD
	 * character - if this sequence is missing it is treated as an
	 * errored packet
	 */
#define XLMAC_RX_CTRL_STRICT_PREAMBLE            0x8UL
	/* STRIP_CRC[2:2] :If set, CRC is stripped from the received packet */
#define XLMAC_RX_CTRL_STRIP_CRC                  0x4UL
	/*
	 * RX_ANY_START[1:1] :If set, MAC allows any undefined control
	 * character to start a packet
	 */
#define XLMAC_RX_CTRL_RX_ANY_START               0x2UL
	/* RSVD_1h[0:0] :Reserved */
#define XLMAC_RX_CTRL_RSVD_1H                    0x1UL
#define XLMAC_REG_XLMAC_RX_MAC_SA			    0x7UL
#define XLMAC_REG_XLMAC_RX_MAC_SA_SIZE		       0x8
/* "Wide-Bus" register. */
	/*
	 * Warning: this field spans across multiple words (i.e. idx 0
	 * to 1).RX_SA[47:0] :Source Address recognized for MAC control
	 * packets in addition to the standard 0x0180C2000001
	 */
#define XLMAC_RX_MAC_SA_RX_SA                    0xffffffffUL
#define XLMAC_REG_XLMAC_RX_MAX_SIZE			    0x8UL
#define XLMAC_REG_XLMAC_RX_MAX_SIZE_SIZE	       0x8
/* "Wide-Bus" register. */
	/*
	 * RX_MAX_SIZE[13:0] :Maximum packet size in receive direction,
	 * exclusive of preamble and CRC in strip mode. Packets greater
	 * than this size are truncated to this value.
	 */
#define XLMAC_RX_MAX_SIZE_RX_MAX_SIZE_MASK       0x3fffUL
#define XLMAC_RX_MAX_SIZE_RX_MAX_SIZE_SFT        0
#define XLMAC_REG_XLMAC_RX_VLAN_TAG			    0x9UL
#define XLMAC_REG_XLMAC_RX_VLAN_TAG_SIZE	       0x8
/* "Wide-Bus" register. */
	/*
	 * OUTER_VLAN_TAG_ENABLE[33:33]:If set, MAC enables VLAN tag
	 * detection using the OUTER_VLAN_TAG (for idx 1 ...)
	 */
#define XLMAC_RX_VLAN_TAG_OUTER_VLAN_TAG_ENABLE  0x2UL
	/*
	 * INNER_VLAN_TAG_ENABLE[32:32]:If set, MAC enables VLAN tag
	 * detection using the INNER_VLAN_TAG (for idx 1 ...)
	 */
#define XLMAC_RX_VLAN_TAG_INNER_VLAN_TAG_ENABLE  0x1UL
	/* OUTER_VLAN_TAG[31:16]:TPID field for Outer VLAN tag */
#define XLMAC_RX_VLAN_TAG_OUTER_VLAN_TAG_MASK    0xffff0000UL
#define XLMAC_RX_VLAN_TAG_OUTER_VLAN_TAG_SFT     16
	/* INNER_VLAN_TAG[15:0]:TPID field for Inner VLAN tag */
#define XLMAC_RX_VLAN_TAG_INNER_VLAN_TAG_MASK    0xffffUL
#define XLMAC_RX_VLAN_TAG_INNER_VLAN_TAG_SFT     0
#define XLMAC_REG_XLMAC_RX_LSS_CTRL			    0xaUL
#define XLMAC_REG_XLMAC_RX_LSS_CTRL_SIZE	       0x8
/* "Wide-Bus" register. */
	/*
	 * RESET_FLOW_CONTROL_TIMERS_ON_LINK_DOWN[7:7]:If set, the
	 * Receive Pause, PFC and LLFC timers are reset whenever the
	 * link status is down, or local or remote faults are received.
	 * ',
	 */
#define XLMAC_RX_LSS_CTRL_RESET_FLOW_CONTROL_TIMERS_ON_LINK_DOWN 0x80UL
	/*
	 * DROP_TX_DATA_ON_LINK_INTERRUPT[6:6]:This bit determines the
	 * way MAC handles data during link interruption state, if
	 * LINK_INTERRUPTION_DISABLE is reset.If set, during link
	 * interruption state, MAC drops transmit-data (statistics are
	 * updated) and sends IDLEs on the wire.If reset, transmit data
	 * is stalled in the internal FIFO under link interruption
	 * state. },
	 */
#define XLMAC_RX_LSS_CTRL_DROP_TX_DATA_ON_LINK_INTERRUPT 0x40UL
	/*
	 * DROP_TX_DATA_ON_REMOTE_FAULT[5:5]:This bit determines the way
	 * MAC handles data during remote fault state, if
	 * REMOTE_FAULT_DISABLE is reset.If set, during remote fault
	 * state, MAC drops transmit-data (statistics are updated) and
	 * sends IDLEs on the wire.If reset, transmit data is stalled in
	 * the internal FIFO under remote fault state. },
	 */
#define XLMAC_RX_LSS_CTRL_DROP_TX_DATA_ON_REMOTE_FAULT 0x20UL
	/*
	 * DROP_TX_DATA_ON_LOCAL_FAULT[4:4]:This bit determines the way
	 * MAC handles data during local fault state, if
	 * LOCAL_FAULT_DISABLE is reset.If set, during local fault
	 * state, MAC drops transmit-data (statistics are updated) and
	 * sends remote faults on the wire.If reset, transmit data is
	 * stalled in the internal FIFO under local fault state. },
	 */
#define XLMAC_RX_LSS_CTRL_DROP_TX_DATA_ON_LOCAL_FAULT 0x10UL
	/*
	 * LINK_INTERRUPTION_DISABLE[3:3]:This bit determines the
	 * transmit response during link interruption state. The
	 * LINK_INTERRUPTION_STATUS bit is always updated irrespective
	 * of this configuration.If set, MAC will continue to transmit
	 * data irrespective of LINK_INTERRUPTION_STATUS.If reset, MAC
	 * transmit behavior is governed by
	 * DROP_TX_DATA_ON_LINK_INTERRUPT configuration. },
	 */
#define XLMAC_RX_LSS_CTRL_LINK_INTERRUPTION_DISABLE 0x8UL
	/*
	 * USE_EXTERNAL_FAULTS_FOR_TX[2:2]:If set, the transmit fault
	 * responses are determined from input pins rather than internal
	 * receive status. In this mode, input fault from pins (from a
	 * peer MAC) is directly relayed on the transmit side of this
	 * MAC.See specification document for more details. },
	 */
#define XLMAC_RX_LSS_CTRL_USE_EXTERNAL_FAULTS_FOR_TX 0x4UL
	/*
	 * REMOTE_FAULT_DISABLE[1:1]:This bit determines the transmit
	 * response during remote fault state. The REMOTE_FAULT_STATUS
	 * bit is always updated irrespective of this configuration.If
	 * set, MAC will continue to transmit data irrespective of
	 * REMOTE_FAULT_STATUS.If reset, MAC transmit behavior is
	 * governed by DROP_TX_DATA_ON_REMOTE_FAULT configuration. },
	 */
#define XLMAC_RX_LSS_CTRL_REMOTE_FAULT_DISABLE   0x2UL
	/*
	 * LOCAL_FAULT_DISABLE[0:0]:This bit determines the transmit
	 * response during local fault state. The LOCAL_FAULT_STATUS bit
	 * is always updated irrespective of this configuration.If set,
	 * MAC will continue to transmit data irrespective of
	 * LOCAL_FAULT_STATUS.If reset, MAC transmit behavior is
	 * governed by DROP_TX_DATA_ON_LOCAL_FAULT configuration. }
	 */
#define XLMAC_RX_LSS_CTRL_LOCAL_FAULT_DISABLE    0x1UL
#define XLMAC_REG_XLMAC_RX_LSS_STATUS			    0xbUL
#define XLMAC_REG_XLMAC_RX_LSS_STATUS_SIZE	       0x8
/* "Wide-Bus" register. */
	/*
	 * LINK_INTERRUPTION_STATUS[2:2]:True when link interruption
	 * state is detected as per RS layer state machine. Sticky bit
	 * is cleared by CLEAR_LINK_INTERRUPTION_STATUS.
	 */
#define XLMAC_RX_LSS_STATUS_LINK_INTERRUPTION_STATUS 0x4UL
	/*
	 * REMOTE_FAULT_STATUS[1:1]:True when remote fault state is
	 * detected as per RS layer state machine. Sticky bit is cleared
	 * by CLEAR_REMOTE_FAULT_STATUS.
	 */
#define XLMAC_RX_LSS_STATUS_REMOTE_FAULT_STATUS  0x2UL
	/*
	 * LOCAL_FAULT_STATUS[0:0]:True when local fault state is
	 * detected as per RS layer state machine. Sticky bit is cleared
	 * by CLEAR_LOCAL_FAULT_STATUS
	 */
#define XLMAC_RX_LSS_STATUS_LOCAL_FAULT_STATUS   0x1UL
#define XLMAC_REG_XLMAC_CLEAR_RX_LSS_STATUS		    0xcUL
#define XLMAC_REG_XLMAC_CLEAR_RX_LSS_STATUS_SIZE      0x8
/* "Wide-Bus" register. */
	/*
	 * CLEAR_LINK_INTERRUPTION_STATUS[2:2]:A rising edge on this
	 * register bit (0- greater than 1), clears the sticky
	 * LINK_INTERRUPTION_STATUS bit
	 */
#define XLMAC_CLEAR_RX_LSS_STATUS_CLEAR_LINK_INTERRUPTION_STATUS 0x4UL
	/*
	 * CLEAR_REMOTE_FAULT_STATUS[1:1]:A rising edge on this register
	 * bit (0- greater than 1), clears the sticky
	 * REMOTE_FAULT_STATUS bit
	 */
#define XLMAC_CLEAR_RX_LSS_STATUS_CLEAR_REMOTE_FAULT_STATUS 0x2UL
	/*
	 * CLEAR_LOCAL_FAULT_STATUS[0:0]:A rising edge on this register
	 * bit (0- greater than 1), clears the sticky LOCAL_FAULT_STATUS
	 * bit
	 */
#define XLMAC_CLEAR_RX_LSS_STATUS_CLEAR_LOCAL_FAULT_STATUS 0x1UL
#define XLMAC_REG_XLMAC_PAUSE_CTRL			    0xdUL
#define XLMAC_REG_XLMAC_PAUSE_CTRL_SIZE	       0x8
/* "Wide-Bus" register. */
	/*
	 * Warning: this field spans across multiple words (i.e. idx 0
	 * to 1).PAUSE_XOFF_TIMER[36:21]:Pause time value sent in the
	 * timer field for XOFF state (unit is 512 bit-times)
	 */
#define XLMAC_PAUSE_CTRL_PAUSE_XOFF_TIMER_MASK   0x7ffUL
#define XLMAC_PAUSE_CTRL_PAUSE_XOFF_TIMER_SFT    21
	/* RSVD_2i[20:20] :Reserved */
#define XLMAC_PAUSE_CTRL_RSVD_2I                 0x100000UL
	/* RSVD_1j[19:19] :Reserved */
#define XLMAC_PAUSE_CTRL_RSVD_1J                 0x80000UL
	/*
	 * RX_PAUSE_EN[18:18] :When set, enables detection of pause
	 * frames in the receive direction and pause/resume the transmit
	 * data path
	 */
#define XLMAC_PAUSE_CTRL_RX_PAUSE_EN             0x40000UL
	/*
	 * TX_PAUSE_EN[17:17] :When set, enables the transmission of
	 * pause frames whenever there is a transition on txbkp input to
	 * MAC from MMU
	 */
#define XLMAC_PAUSE_CTRL_TX_PAUSE_EN             0x20000UL
	/*
	 * PAUSE_REFRESH_EN[16:16]:When set, enables the periodic re-
	 * generation of XOFF pause frames based on the interval
	 * specified in PAUSE_REFRESH_TIMER
	 */
#define XLMAC_PAUSE_CTRL_PAUSE_REFRESH_EN        0x10000UL
	/*
	 * PAUSE_REFRESH_TIMER[15:0]:This field specifies the interval
	 * at which pause frames are re-generated during XOFF state,
	 * provided PAUSE_REFRESH_EN is set (unit is 512 bit-times)
	 */
#define XLMAC_PAUSE_CTRL_PAUSE_REFRESH_TIMER_MASK 0xffffUL
#define XLMAC_PAUSE_CTRL_PAUSE_REFRESH_TIMER_SFT 0
#define XLMAC_REG_XLMAC_PFC_CTRL			    0xeUL
#define XLMAC_REG_XLMAC_PFC_CTRL_SIZE		       0x8
/* "Wide-Bus" register. */
	/*
	 * TX_PFC_EN[37:37] :When set, enables the transmission of PFC
	 * frames (for idx 1 ...)
	 */
#define XLMAC_PFC_CTRL_TX_PFC_EN                 0x20UL
	/*
	 * RX_PFC_EN[36:36] :When set, enables detection of PFC frames
	 * in the receive direction and generation of COSMAPs to MMU
	 * based on incoming timer values (for idx 1 ...)
	 */
#define XLMAC_PFC_CTRL_RX_PFC_EN                 0x10UL
	/*
	 * PFC_STATS_EN[35:35] :When set, enables the generation of
	 * receive and transmit PFC events into the corresponding
	 * statistics vectors (RSV and TSV) (for idx 1 ...)
	 */
#define XLMAC_PFC_CTRL_PFC_STATS_EN              0x8UL
	/* RSVDk[34:34] :Reserved (for idx 1 ...) */
#define XLMAC_PFC_CTRL_RSVDK                     0x4UL
	/*
	 * FORCE_PFC_XON[33:33]:When set, forces the MAC to generate an
	 * XON indication to the MMU for all classes of service in the
	 * receive direction (for idx 1 ...)
	 */
#define XLMAC_PFC_CTRL_FORCE_PFC_XON             0x2UL
	/*
	 * PFC_REFRESH_EN[32:32]:When set, enables the periodic re-
	 * generation of PFC frames based on the interval specified in
	 * PFC_REFRESH_TIMER (for idx 1 ...)
	 */
#define XLMAC_PFC_CTRL_PFC_REFRESH_EN            0x1UL
	/*
	 * PFC_XOFF_TIMER[31:16]:Pause time value sent in the timer
	 * field for classes in XOFF state (unit is 512 bit-times)
	 */
#define XLMAC_PFC_CTRL_PFC_XOFF_TIMER_MASK       0xffff0000UL
#define XLMAC_PFC_CTRL_PFC_XOFF_TIMER_SFT        16
	/*
	 * PFC_REFRESH_TIMER[15:0]:This field specifies the interval at
	 * which PFC frames are re-generated for a class of service in
	 * XOFF state, provided PFC_REFRESH_EN is set (unit is 512 bit-
	 * times)
	 */
#define XLMAC_PFC_CTRL_PFC_REFRESH_TIMER_MASK    0xffffUL
#define XLMAC_PFC_CTRL_PFC_REFRESH_TIMER_SFT     0
#define XLMAC_REG_XLMAC_PFC_TYPE			    0xfUL
#define XLMAC_REG_XLMAC_PFC_TYPE_SIZE		       0x8
/* "Wide-Bus" register. */
	/*
	 * PFC_ETH_TYPE[15:0] :This field is used in the ETHERTYPE field
	 * of the PFC frame that is generated and transmitted by the MAC
	 * and also used for detection in the receive direction
	 */
#define XLMAC_PFC_TYPE_PFC_ETH_TYPE_MASK         0xffffUL
#define XLMAC_PFC_TYPE_PFC_ETH_TYPE_SFT          0
#define XLMAC_REG_XLMAC_PFC_OPCODE			   0x10UL
#define XLMAC_REG_XLMAC_PFC_OPCODE_SIZE	       0x8
/* "Wide-Bus" register. */
	/*
	 * PFC_OPCODE[15:0] :This field is used in the OPCODE field of
	 * the PFC frame that is generated and transmitted by the MAC
	 * and also used for detection in the receive direction
	 */
#define XLMAC_PFC_OPCODE_PFC_OPCODE_MASK         0xffffUL
#define XLMAC_PFC_OPCODE_PFC_OPCODE_SFT          0
#define XLMAC_REG_XLMAC_PFC_DA				   0x11UL
#define XLMAC_REG_XLMAC_PFC_DA_SIZE		       0x8
/* "Wide-Bus" register. */
	/*
	 * Warning: this field spans across multiple words (i.e. idx 0
	 * to 1).PFC_MACDA[47:0] :This field is used in the destination-
	 * address field of the PFC frame that is generated and
	 * transmitted by the MAC and also used for detection in the
	 * receive direction
	 */
#define XLMAC_PFC_DA_PFC_MACDA                   0xffffffffUL
#define XLMAC_REG_XLMAC_LLFC_CTRL			   0x12UL
#define XLMAC_REG_XLMAC_LLFC_CTRL_SIZE		       0x8
/* "Wide-Bus" register. */
	/*
	 * LLFC_IMG[13:6] :This field indicates the minimum Inter
	 * Message Gap that is enforced by the MAC between 2 LLFC
	 * messages in the transmit direction (unit is 1 credit)
	 */
#define XLMAC_LLFC_CTRL_LLFC_IMG_MASK            0x3fc0UL
#define XLMAC_LLFC_CTRL_LLFC_IMG_SFT             6
	/*
	 * NO_SOM_FOR_CRC_LLFC[5:5]:When set, LLFC CRC computation does
	 * not include the SOM character
	 */
#define XLMAC_LLFC_CTRL_NO_SOM_FOR_CRC_LLFC      0x20UL
	/*
	 * LLFC_CRC_IGNORE[4:4]:When set, disables the CRC check for
	 * LLFC messages in the receive direction
	 */
#define XLMAC_LLFC_CTRL_LLFC_CRC_IGNORE          0x10UL
	/*
	 * LLFC_CUT_THROUGH_MODE[3:3]:When LLFC_IN_IPG_ONLY is reset,
	 * the mode of transmission of LLFC messages is controlled by
	 * this bit depending upon whether the LLFC message is XON or
	 * XOFFWhen LLFC_CUT_THROUGH_MODE is reset, all LLFC messages
	 * are transmitted pre-emptively (within a packet)When
	 * LLFC_CUT_THROUGH_MODE is set, only XOFF LLFC messages are
	 * transmitted pre-emptively, XON LLFC messages are transmitted
	 * during IPG },
	 */
#define XLMAC_LLFC_CTRL_LLFC_CUT_THROUGH_MODE    0x8UL
	/*
	 * LLFC_IN_IPG_ONLY[2:2]:When set, all LLFC messages are
	 * transmitted during IPGWhen reset, the mode of insertion of
	 * LLFC messages is controlled by LLFC_CUT_THROUGH_MODE },
	 */
#define XLMAC_LLFC_CTRL_LLFC_IN_IPG_ONLY         0x4UL
	/*
	 * RX_LLFC_EN[1:1] :When set, enables processing of LLFC frames
	 * in the receive direction and generation of COSMAPs to MMU
	 */
#define XLMAC_LLFC_CTRL_RX_LLFC_EN               0x2UL
	/*
	 * TX_LLFC_EN[0:0] :When set, enables the generation and
	 * transmission of LLFC frames in the transmit direction
	 */
#define XLMAC_LLFC_CTRL_TX_LLFC_EN               0x1UL
#define XLMAC_REG_XLMAC_TX_LLFC_MSG_FIELDS		   0x13UL
#define XLMAC_REG_XLMAC_TX_LLFC_MSG_FIELDS_SIZE       0x8
/* "Wide-Bus" register. */
	/*
	 * LLFC_XOFF_TIME[27:12]:Pause time value sent in the XOFF_TIME
	 * field of the outgoing LLFC message
	 */
#define XLMAC_TX_LLFC_MSG_FIELDS_LLFC_XOFF_TIME_MASK 0xffff000UL
#define XLMAC_TX_LLFC_MSG_FIELDS_LLFC_XOFF_TIME_SFT 12
	/*
	 * TX_LLFC_FC_OBJ_LOGICAL[11:8]:This field is used in the
	 * FC_OBJ_LOGICAL field of the outgoing LLFC message
	 */
#define XLMAC_TX_LLFC_MSG_FIELDS_TX_LLFC_FC_OBJ_LOGICAL_MASK 0xf00UL
#define XLMAC_TX_LLFC_MSG_FIELDS_TX_LLFC_FC_OBJ_LOGICAL_SFT 8
	/*
	 * TX_LLFC_MSG_TYPE_LOGICAL[7:0]:This field is used in the
	 * MSG_TYPE_LOGICAL field of the outgoing LLFC message
	 */
#define XLMAC_TX_LLFC_MSG_FIELDS_TX_LLFC_MSG_TYPE_LOGICAL_MASK 0xffUL
#define XLMAC_TX_LLFC_MSG_FIELDS_TX_LLFC_MSG_TYPE_LOGICAL_SFT 0
#define XLMAC_REG_XLMAC_RX_LLFC_MSG_FIELDS		   0x14UL
#define XLMAC_REG_XLMAC_RX_LLFC_MSG_FIELDS_SIZE       0x8
/* "Wide-Bus" register. */
	/*
	 * RX_LLFC_FC_OBJ_PHYSICAL[23:20]:This value is compared against
	 * the FC_OBJ_PHYSICAL field of an incoming LLFC message in
	 * order to decode the message
	 */
#define XLMAC_RX_LLFC_MSG_FIELDS_RX_LLFC_FC_OBJ_PHYSICAL_MASK 0xf00000UL
#define XLMAC_RX_LLFC_MSG_FIELDS_RX_LLFC_FC_OBJ_PHYSICAL_SFT 20
	/*
	 * RX_LLFC_MSG_TYPE_PHYSICAL[19:12]:This value is compared
	 * against the MSG_TYPE_PHYSICAL field of an incoming LLFC
	 * message in order to decode the message
	 */
#define XLMAC_RX_LLFC_MSG_FIELDS_RX_LLFC_MSG_TYPE_PHYSICAL_MASK 0xff000UL
#define XLMAC_RX_LLFC_MSG_FIELDS_RX_LLFC_MSG_TYPE_PHYSICAL_SFT 12
	/*
	 * RX_LLFC_FC_OBJ_LOGICAL[11:8]:This value is compared against
	 * the FC_OBJ_LOGICAL field of an incoming LLFC message in order
	 * to decode the message
	 */
#define XLMAC_RX_LLFC_MSG_FIELDS_RX_LLFC_FC_OBJ_LOGICAL_MASK 0xf00UL
#define XLMAC_RX_LLFC_MSG_FIELDS_RX_LLFC_FC_OBJ_LOGICAL_SFT 8
	/*
	 * RX_LLFC_MSG_TYPE_LOGICAL[7:0]:This value is compared against
	 * the MSG_TYPE_LOGICAL field of an incoming LLFC message in
	 * order to decode the message
	 */
#define XLMAC_RX_LLFC_MSG_FIELDS_RX_LLFC_MSG_TYPE_LOGICAL_MASK 0xffUL
#define XLMAC_RX_LLFC_MSG_FIELDS_RX_LLFC_MSG_TYPE_LOGICAL_SFT 0
#define XLMAC_REG_XLMAC_TX_TIMESTAMP_FIFO_DATA		   0x15UL
#define XLMAC_REG_XLMAC_TX_TIMESTAMP_FIFO_DATA_SIZE   0x8
/* "Wide-Bus" register. */
	/*
	 * TS_ENTRY_VALID[48:48]:Active high qualifier for the TimeStamp
	 * and SEQUENCE_ID fields. (for idx 1 ...)
	 */
#define XLMAC_TX_TIMESTAMP_FIFO_DATA_TS_ENTRY_VALID 0x10000UL
	/*
	 * SEQUENCE_ID[47:32] :The Sequence Identifier extracted from
	 * the Timesync packet based on the header offset (for idx 1
	 * ...)
	 */
#define XLMAC_TX_TIMESTAMP_FIFO_DATA_SEQUENCE_ID_MASK 0xffffUL
#define XLMAC_TX_TIMESTAMP_FIFO_DATA_SEQUENCE_ID_SFT 0
	/*
	 * TIME_STAMP[31:0] :The TimeStamp value of the Tx two-step
	 * enabled packet.
	 */
#define XLMAC_TX_TIMESTAMP_FIFO_DATA_TIME_STAMP_MASK 0xffffffffUL
#define XLMAC_TX_TIMESTAMP_FIFO_DATA_TIME_STAMP_SFT 0
#define XLMAC_REG_XLMAC_TX_TIMESTAMP_FIFO_STATUS	   0x16UL
#define XLMAC_REG_XLMAC_TX_TIMESTAMP_FIFO_STATUS_SIZE  0x8
/* "Wide-Bus" register. */
	/*
	 * ENTRY_COUNT[2:0] :Number of TX time stamps currently buffered
	 * in TX Time Stamp FIFO. A valid entry is popped out whenever
	 * XLMAC_TX_TIMESTMAP_FIFO_DATA is read
	 */
#define XLMAC_TX_TIMESTAMP_FIFO_STATUS_ENTRY_COUNT_MASK 0x7UL
#define XLMAC_TX_TIMESTAMP_FIFO_STATUS_ENTRY_COUNT_SFT 0
#define XLMAC_REG_XLMAC_FIFO_STATUS			   0x17UL
#define XLMAC_REG_XLMAC_FIFO_STATUS_SIZE	       0x8
/* "Wide-Bus" register. */
	/*
	 * LINK_STATUS[8:8] :This bit indicates the link status used by
	 * XLMAC EEE and lag-failover state machines. This reflects the
	 * live status of the link as seen by the MAC. If set, indicates
	 * that link is active.
	 */
#define XLMAC_FIFO_STATUS_LINK_STATUS            0x100UL
	/* RX_PKT_OVERFLOW[7:7]:If set, indicates RX packet fifo overflow */
#define XLMAC_FIFO_STATUS_RX_PKT_OVERFLOW        0x80UL
	/*
	 * TX_TS_FIFO_OVERFLOW[6:6]:If set, indicates overflow occurred
	 * in TX two-step Time Stamp FIFO
	 */
#define XLMAC_FIFO_STATUS_TX_TS_FIFO_OVERFLOW    0x40UL
	/*
	 * TX_LLFC_MSG_OVERFLOW[5:5]:If set, indicates TX LLFC message
	 * fifo overflow
	 */
#define XLMAC_FIFO_STATUS_TX_LLFC_MSG_OVERFLOW   0x20UL
	/* RSVD_2l[4:4] :Reserved */
#define XLMAC_FIFO_STATUS_RSVD_2L                0x10UL
	/* TX_PKT_OVERFLOW[3:3]:If set, indicates tx packet fifo overflow */
#define XLMAC_FIFO_STATUS_TX_PKT_OVERFLOW        0x8UL
	/* TX_PKT_UNDERFLOW[2:2]:If set, indicates tx packet fifo underflow */
#define XLMAC_FIFO_STATUS_TX_PKT_UNDERFLOW       0x4UL
	/* RX_MSG_OVERFLOW[1:1]:If set, indicates rx message fifo overflow */
#define XLMAC_FIFO_STATUS_RX_MSG_OVERFLOW        0x2UL
	/* RSVD_1m[0:0] :Reserved */
#define XLMAC_FIFO_STATUS_RSVD_1M                0x1UL
#define XLMAC_REG_XLMAC_CLEAR_FIFO_STATUS		   0x18UL
#define XLMAC_REG_XLMAC_CLEAR_FIFO_STATUS_SIZE	       0x8
/* "Wide-Bus" register. */
	/*
	 * CLEAR_RX_PKT_OVERFLOW[7:7]:A rising edge on this register bit
	 * (0- greater than 1), clears the sticky RX_PKT_OVERFLOW status
	 * bit.
	 */
#define XLMAC_CLEAR_FIFO_STATUS_CLEAR_RX_PKT_OVERFLOW 0x80UL
	/*
	 * CLEAR_TX_TS_FIFO_OVERFLOW[6:6]:A rising edge on this register
	 * bit (0- greater than 1), clears the sticky
	 * TX_TS_FIFO_OVERFLOW status bit.
	 */
#define XLMAC_CLEAR_FIFO_STATUS_CLEAR_TX_TS_FIFO_OVERFLOW 0x40UL
	/*
	 * CLEAR_TX_LLFC_MSG_OVERFLOW[5:5]:A rising edge on this
	 * register bit (0- greater than 1), clears the sticky
	 * TX_LLFC_MSG_OVERFLOW status bit.
	 */
#define XLMAC_CLEAR_FIFO_STATUS_CLEAR_TX_LLFC_MSG_OVERFLOW 0x20UL
	/* RSVD_2n[4:4] :Reserved */
#define XLMAC_CLEAR_FIFO_STATUS_RSVD_2N          0x10UL
	/*
	 * CLEAR_TX_PKT_OVERFLOW[3:3]:A rising edge on this register bit
	 * (0- greater than 1), clears the sticky TX_PKT_OVERFLOW status
	 * bit.
	 */
#define XLMAC_CLEAR_FIFO_STATUS_CLEAR_TX_PKT_OVERFLOW 0x8UL
	/*
	 * CLEAR_TX_PKT_UNDERFLOW[2:2]:A rising edge on this register
	 * bit (0- greater than 1), clears the sticky TX_PKT_UNDERFLOW
	 * status bit
	 */
#define XLMAC_CLEAR_FIFO_STATUS_CLEAR_TX_PKT_UNDERFLOW 0x4UL
	/*
	 * CLEAR_RX_MSG_OVERFLOW[1:1]:A rising edge on this register bit
	 * (0- greater than 1), clears the sticky RX_MSG_OVERFLOW status
	 * bit
	 */
#define XLMAC_CLEAR_FIFO_STATUS_CLEAR_RX_MSG_OVERFLOW 0x2UL
	/* RSVD_1o[0:0] :Reserved */
#define XLMAC_CLEAR_FIFO_STATUS_RSVD_1O          0x1UL
#define XLMAC_REG_XLMAC_LAG_FAILOVER_STATUS		   0x19UL
#define XLMAC_REG_XLMAC_LAG_FAILOVER_STATUS_SIZE      0x8
/* "Wide-Bus" register. */
	/* RSVDp[1:1] :Reserved */
#define XLMAC_LAG_FAILOVER_STATUS_RSVDP          0x2UL
	/*
	 * LAG_FAILOVER_LOOPBACK[0:0]:Set when XLMAC is in lag failover
	 * state
	 */
#define XLMAC_LAG_FAILOVER_STATUS_LAG_FAILOVER_LOOPBACK 0x1UL
#define XLMAC_REG_XLMAC_EEE_CTRL			   0x1aUL
#define XLMAC_REG_XLMAC_EEE_CTRL_SIZE		       0x8
/* "Wide-Bus" register. */
	/* RSVDq[1:1] :Reserved */
#define XLMAC_EEE_CTRL_RSVDQ                     0x2UL
	/*
	 * EEE_EN[0:0] :When set, enables EEE state machine in the
	 * transmit direction and LPI detection/prediction in the
	 * receive direction
	 */
#define XLMAC_EEE_CTRL_EEE_EN                    0x1UL
#define XLMAC_REG_XLMAC_EEE_TIMERS			   0x1bUL
#define XLMAC_REG_XLMAC_EEE_TIMERS_SIZE	       0x8
/* "Wide-Bus" register. */
	/*
	 * EEE_REF_COUNT[63:48]:This field controls clock divider used
	 * to generate ~1us reference pulses used by EEE timers. It
	 * specifies integer number of clock cycles for 1us reference
	 * using tsc_clk (for idx 1 ...)
	 */
#define XLMAC_EEE_TIMERS_EEE_REF_COUNT_MASK      0xffff0000UL
#define XLMAC_EEE_TIMERS_EEE_REF_COUNT_SFT       16
	/*
	 * EEE_WAKE_TIMER[47:32]:This is the duration for which MAC must
	 * wait to go back to ACTIVE state from LPI state when it
	 * receives packet/flow-control frames for transmission. Unit is
	 * micro seconds (for idx 1 ...)
	 */
#define XLMAC_EEE_TIMERS_EEE_WAKE_TIMER_MASK     0xffffUL
#define XLMAC_EEE_TIMERS_EEE_WAKE_TIMER_SFT      0
	/*
	 * EEE_DELAY_ENTRY_TIMER[31:0]:This is the duration for which
	 * the MAC must wait in EMPTY state before transitioning to LPI
	 * state. Unit is micro seconds
	 */
#define XLMAC_EEE_TIMERS_EEE_DELAY_ENTRY_TIMER_MASK 0xffffffffUL
#define XLMAC_EEE_TIMERS_EEE_DELAY_ENTRY_TIMER_SFT 0
#define XLMAC_REG_XLMAC_EEE_1_SEC_LINK_STATUS_TIMER	   0x1cUL
#define XLMAC_REG_XLMAC_EEE_1_SEC_LINK_STATUS_TIMER_SIZE  0x8
/* "Wide-Bus" register. */
	/*
	 * ONE_SECOND_TIMER[23:0]:This is the duration for which EEE FSM
	 * must wait when Link status becomes active before
	 * transitioning to ACTIVE state. Unit is micro seconds
	 */
#define XLMAC_EEE_1_SEC_LINK_STATUS_TIMER_ONE_SECOND_TIMER_MASK 0xffffffUL
#define XLMAC_EEE_1_SEC_LINK_STATUS_TIMER_ONE_SECOND_TIMER_SFT 0

#define IPC_REG_NW_SERDES_MDIO_COMM		      0x4010220UL
/*
 * START_BUSY[29:29] :This bit is self clearing. When written to a '1', the
 * currently programmed MDIO transaction will activate. When the operation is
 * complete, this bit will clear and the MI_COMPLETE bit will be set in the
 * status register. Writing this bit as a '0' has no effect. This bit must be
 * read as a '0' before setting to prevent un-predictable results.
 */
#define NW_SERDES_MDIO_COMM_START_BUSY           0x20000000UL
/*
 * FAIL[28:28] :This bit is updated at the end of each MDIO transaction when the
 * START_BUSY bit is set. If an error occurred on the MDIO interface during the
 * operation, this bit will be updated to '1', otherwise, it will be updated to
 * '0'. Errors usually happen when the attached PHY fails to drive a response
 * during a read. This bit is only modified by completing a new MDIO
 * transaction.
 */
#define NW_SERDES_MDIO_COMM_FAIL                 0x10000000UL
/*
 * COMMAND[27:26] :This value is used to define command 1 == cmd_wr - Write 2 ==
 * cmd_rd - Read
 */
#define NW_SERDES_MDIO_COMM_COMMAND_MASK         0xc000000UL
#define NW_SERDES_MDIO_COMM_COMMAND_SFT          26
#define NW_SERDES_MDIO_COMM_COMMAND_CMD_WR       (0x1UL << 26)
/* Write */
#define NW_SERDES_MDIO_COMM_COMMAND_CMD_RD       (0x2UL << 26)
/* Read */
/*
 * PHY_ADDR[25:21] :This value is used to define the PHY address portion of the
 * MDIO transaction 1 == swreg_vmgmt - SWREG VMGMT 2 == swreg_vmain - SWREG
 * VMAIN 3 == swreg_vanalog - SWREG VANALOG 4 == swreg_v1p8 - SWREG V1p8
 */
#define NW_SERDES_MDIO_COMM_PHY_ADDR_MASK        0x3e00000UL
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SFT         21
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SWREG_VMGMT (0x1UL << 21)
/* SWREG VMGMT */
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SWREG_VMAIN (0x2UL << 21)
/* SWREG VMAIN */
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SWREG_VANALOG (0x3UL << 21)
/* SWREG VANALOG */
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SWREG_V1P8  (0x4UL << 21)
/* SWREG V1p8 */
/*
 * REG_ADDR[20:16] :This value is used to define the register address portion of
 * the MDIO transaction
 */
#define NW_SERDES_MDIO_COMM_REG_ADDR_MASK        0x1f0000UL
#define NW_SERDES_MDIO_COMM_REG_ADDR_SFT         16
/*
 * DATA[15:0] :When this register is read, it returns the results of the last
 * MDIO transaction that was performed. When this register value is written, it
 * updates the value that will be used on the next MDIO write transaction that
 * will be performed.
 */
#define NW_SERDES_MDIO_COMM_DATA_MASK            0xffffUL
#define NW_SERDES_MDIO_COMM_DATA_SFT             0
#define IPC_REG_NW_SERDES_MDIO_STATUS		      0x4010224UL
/*
 * FAIL[1:1] :This bit is updated at the end of each MDIO transaction when the
 * START_BUSY bit is set. If an error occurred on the MDIO interface during the
 * operation, this bit will be updated to '1', otherwise, it will be updated to
 * '0'. Errors usually happen when the attached PHY fails to drive a response
 * during a read. This bit is only modified by completing a new MDIO
 * transaction.
 */
#define NW_SERDES_MDIO_STATUS_FAIL               0x2UL
/*
 * DONE[0:0] :This bit is set each time the MDIO transaction has completed. This
 * bit is cleared when the next transaction starts.
 */
#define NW_SERDES_MDIO_STATUS_DONE               0x1UL
#define IPC_REG_NW_SERDES_MDIO_MODE		      0x4010228UL
/*
 * CLOCK_CNT[21:12] :This field controls the MDIO clock speed. The output MDIO
 * clock runs at a frequency equal to CORE_CLK/(2*(CLOCK_CNT+1)). A value of 0
 * is invalid for this register.
 */
#define NW_SERDES_MDIO_MODE_CLOCK_CNT_MASK       0x3ff000UL
#define NW_SERDES_MDIO_MODE_CLOCK_CNT_SFT        12
/*
 * MDC[11:11] :MDC pin 1 == mdc_high - Setting this bit to '1' will cause the
 * MDC pin to high if the BIT_BANG bit is set. 0 == mdc_low - Setting this pin
 * low will cause the MDC pin to drive low if the BIT_BANG bit is set.
 */
#define NW_SERDES_MDIO_MODE_MDC                  0x800UL
#define NW_SERDES_MDIO_MODE_MDC_MDC_HIGH         (0x1UL << 11)
/*
 * Setting this bit to '1' will cause the MDC pin to high if the BIT_BANG bit is
 * set.
 */
#define NW_SERDES_MDIO_MODE_MDC_MDC_LOW          (0x0UL << 11)
/*
 * Setting this pin low will cause the MDC pin to drive low if the BIT_BANG bit
 * is set.
 */
/*
 * MDIO_OE[10:10] :MDIO Output Enable 1 == mdio_drive - Setting this bit to '1'
 * will cause the MDIO pin to drive the value written to the MDIO bit if the
 * BIT_BANG bit is set. 0 == mdio_in - Setting this bit to zero will make the
 * MDIO pin an input.
 */
#define NW_SERDES_MDIO_MODE_MDIO_OE              0x400UL
#define NW_SERDES_MDIO_MODE_MDIO_OE_MDIO_DRIVE   (0x1UL << 10)
/*
 * Setting this bit to '1' will cause the MDIO pin to drive the value written to
 * the MDIO bit if the BIT_BANG bit is set.
 */
#define NW_SERDES_MDIO_MODE_MDIO_OE_MDIO_IN      (0x0UL << 10)
/* Setting this bit to zero will make the MDIO pin an input. */
/*
 * MDIO[9:9] :The write value of this bit controls the drive state of the MDIO
 * pin if the BIT_BANG bit is set. The read value of this bit always reflects
 * the state of the MDIO pin.
 */
#define NW_SERDES_MDIO_MODE_MDIO                 0x200UL
/*
 * BIT_BANG[8:8] :BIT_BANG 1 == mdio_cntrl - If this bit is '1', the MDIO
 * interface is controlled by the MDIO, MDIO_OE, and MDC bits in this register.
 * 0 == mdio_cmd_run - When this bit is '0', the commands in the mdio_cmd
 * register will be executed.
 */
#define NW_SERDES_MDIO_MODE_BIT_BANG             0x100UL
#define NW_SERDES_MDIO_MODE_BIT_BANG_MDIO_CNTRL  (0x1UL << 8)
/*
 * If this bit is '1', the MDIO interface is controlled by the MDIO, MDIO_OE,
 * and MDC bits in this register.
 */
#define NW_SERDES_MDIO_MODE_BIT_BANG_MDIO_CMD_RUN (0x0UL << 8)
/*
 * When this bit is '0', the commands in the mdio_cmd register will be executed.
 */
/* RESERVED[7:4] :Reserved bits */
#define NW_SERDES_MDIO_MODE_RESERVED_MASK        0xf0UL
#define NW_SERDES_MDIO_MODE_RESERVED_SFT         4
/*
 * CLAUSE_45[3:3] :Clause 45 1 == clause_45_mode - If this bit is '1', the MDIO
 * interface will work in Clause 45 mode. 0 == clause_22_mode - The default is
 * Clause 22.
 */
#define NW_SERDES_MDIO_MODE_CLAUSE_45            0x8UL
#define NW_SERDES_MDIO_MODE_CLAUSE_45_CLAUSE_45_MODE (0x1UL << 3)
/* If this bit is '1', the MDIO interface will work in Clause 45 mode. */
#define NW_SERDES_MDIO_MODE_CLAUSE_45_CLAUSE_22_MODE (0x0UL << 3)
/* The default is Clause 22. */
/* RESERVED[2:1] :Reserved bits */
#define NW_SERDES_MDIO_MODE_RESERVED_1_MASK      0x6UL
#define NW_SERDES_MDIO_MODE_RESERVED_1_SFT       1
/*
 * FREE_DIS[0:0] :This field disables the Free runing MDIO clock 1 ==
 * mdio_clk_free_run_dis - Disable Free running MDIO clock
 */
#define NW_SERDES_MDIO_MODE_FREE_DIS             0x1UL
#define NW_SERDES_MDIO_MODE_FREE_DIS_MDIO_CLK_FREE_RUN_DIS (0x1UL << 0)


#define IPC_REG_SWREG_SYNC_CLK_EN		      0x4010210UL
/*
 * Setting this bit to "1" will enable the phase shift logic betweent the three
 * swreg to start working. Global Register, Reset on POR
 */
#define IPC_REG_SWREG_MDIO_COMM			      0x4010214UL
/*
 * START_BUSY[29:29] :This bit is self clearing. When written to a '1', the
 * currently programmed MDIO transaction will activate. When the operation is
 * complete, this bit will clear and the MI_COMPLETE bit will be set in the
 * status register. Writing this bit as a '0' has no effect. This bit must be
 * read as a '0' before setting to prevent un-predictable results.
 */
#define SWREG_MDIO_COMM_START_BUSY               0x20000000UL
/*
 * FAIL[28:28] :This bit is updated at the end of each MDIO transaction when the
 * START_BUSY bit is set. If an error occurred on the MDIO interface during the
 * operation, this bit will be updated to '1', otherwise, it will be updated to
 * '0'. Errors usually happen when the attached PHY fails to drive a response
 * during a read. This bit is only modified by completing a new MDIO
 * transaction.
 */
#define SWREG_MDIO_COMM_FAIL                     0x10000000UL
/*
 * COMMAND[27:26] :This value is used to define command 1 == cmd_wr - Write 2 ==
 * cmd_rd - Read
 */
#define SWREG_MDIO_COMM_COMMAND_MASK             0xc000000UL
#define SWREG_MDIO_COMM_COMMAND_SFT              26
#define SWREG_MDIO_COMM_COMMAND_CMD_WR           (0x1UL << 26)
/* Write */
#define SWREG_MDIO_COMM_COMMAND_CMD_RD           (0x2UL << 26)
/* Read */
/*
 * PHY_ADDR[25:21] :This value is used to define the PHY address portion of the
 * MDIO transaction 1 == swreg_vmgmt - SWREG VMGMT 2 == swreg_vmain - SWREG
 * VMAIN 3 == swreg_vanalog - SWREG VANALOG 4 == swreg_v1p8 - SWREG V1p8
 */
#define SWREG_MDIO_COMM_PHY_ADDR_MASK            0x3e00000UL
#define SWREG_MDIO_COMM_PHY_ADDR_SFT             21
#define SWREG_MDIO_COMM_PHY_ADDR_SWREG_VMGMT     (0x1UL << 21)
/* SWREG VMGMT */
#define SWREG_MDIO_COMM_PHY_ADDR_SWREG_VMAIN     (0x2UL << 21)
/* SWREG VMAIN */
#define SWREG_MDIO_COMM_PHY_ADDR_SWREG_VANALOG   (0x3UL << 21)
/* SWREG VANALOG */
#define SWREG_MDIO_COMM_PHY_ADDR_SWREG_V1P8      (0x4UL << 21)
/* SWREG V1p8 */
/*
 * REG_ADDR[20:16] :This value is used to define the register address portion of
 * the MDIO transaction
 */
#define SWREG_MDIO_COMM_REG_ADDR_MASK            0x1f0000UL
#define SWREG_MDIO_COMM_REG_ADDR_SFT             16
/*
 * DATA[15:0] :When this register is read, it returns the results of the last
 * MDIO transaction that was performed. When this register value is written, it
 * updates the value that will be used on the next MDIO write transaction that
 * will be performed.
 */
#define SWREG_MDIO_COMM_DATA_MASK                0xffffUL
#define SWREG_MDIO_COMM_DATA_SFT                 0
#define IPC_REG_SWREG_MDIO_STATUS		      0x4010218UL
/*
 * FAIL[1:1] :This bit is updated at the end of each MDIO transaction when the
 * START_BUSY bit is set. If an error occurred on the MDIO interface during the
 * operation, this bit will be updated to '1', otherwise, it will be updated to
 * '0'. Errors usually happen when the attached PHY fails to drive a response
 * during a read. This bit is only modified by completing a new MDIO
 * transaction.
 */
#define SWREG_MDIO_STATUS_FAIL                   0x2UL
/*
 * DONE[0:0] :This bit is set each time the MDIO transaction has completed. This
 * bit is cleared when the next transaction starts.
 */
#define SWREG_MDIO_STATUS_DONE                   0x1UL
#define IPC_REG_SWREG_MDIO_MODE			      0x401021cUL
/*
 * CLOCK_CNT[21:12] :This field controls the MDIO clock speed. The output MDIO
 * clock runs at a frequency equal to CORE_CLK/(2*(CLOCK_CNT+1)). A value of 0
 * is invalid for this register.
 */
#define SWREG_MDIO_MODE_CLOCK_CNT_MASK           0x3ff000UL
#define SWREG_MDIO_MODE_CLOCK_CNT_SFT            12
/*
 * MDC[11:11] :MDC pin 1 == mdc_high - Setting this bit to '1' will cause the
 * MDC pin to high if the BIT_BANG bit is set. 0 == mdc_low - Setting this pin
 * low will cause the MDC pin to drive low if the BIT_BANG bit is set.
 */
#define SWREG_MDIO_MODE_MDC                      0x800UL
#define SWREG_MDIO_MODE_MDC_MDC_HIGH             (0x1UL << 11)
/*
 * Setting this bit to '1' will cause the MDC pin to high if the BIT_BANG bit is
 * set.
 */
#define SWREG_MDIO_MODE_MDC_MDC_LOW              (0x0UL << 11)
/*
 * Setting this pin low will cause the MDC pin to drive low if the BIT_BANG bit
 * is set.
 */
/*
 * MDIO_OE[10:10] :MDIO Output Enable 1 == mdio_drive - Setting this bit to '1'
 * will cause the MDIO pin to drive the value written to the MDIO bit if the
 * BIT_BANG bit is set. 0 == mdio_in - Setting this bit to zero will make the
 * MDIO pin an input.
 */
#define SWREG_MDIO_MODE_MDIO_OE                  0x400UL
#define SWREG_MDIO_MODE_MDIO_OE_MDIO_DRIVE       (0x1UL << 10)
/*
 * Setting this bit to '1' will cause the MDIO pin to drive the value written to
 * the MDIO bit if the BIT_BANG bit is set.
 */
#define SWREG_MDIO_MODE_MDIO_OE_MDIO_IN          (0x0UL << 10)
/* Setting this bit to zero will make the MDIO pin an input. */
/*
 * MDIO[9:9] :The write value of this bit controls the drive state of the MDIO
 * pin if the BIT_BANG bit is set. The read value of this bit always reflects
 * the state of the MDIO pin.
 */
#define SWREG_MDIO_MODE_MDIO                     0x200UL
/*
 * BIT_BANG[8:8] :BIT_BANG 1 == mdio_cntrl - If this bit is '1', the MDIO
 * interface is controlled by the MDIO, MDIO_OE, and MDC bits in this register.
 * 0 == mdio_cmd_run - When this bit is '0', the commands in the mdio_cmd
 * register will be executed.
 */
#define SWREG_MDIO_MODE_BIT_BANG                 0x100UL
#define SWREG_MDIO_MODE_BIT_BANG_MDIO_CNTRL      (0x1UL << 8)
/*
 * If this bit is '1', the MDIO interface is controlled by the MDIO, MDIO_OE,
 * and MDC bits in this register.
 */
#define SWREG_MDIO_MODE_BIT_BANG_MDIO_CMD_RUN    (0x0UL << 8)
/*
 * When this bit is '0', the commands in the mdio_cmd register will be executed.
 */
/* RESERVED[7:4] :Reserved bits */
#define SWREG_MDIO_MODE_RESERVED_MASK            0xf0UL
#define SWREG_MDIO_MODE_RESERVED_SFT             4
/*
 * CLAUSE_45[3:3] :Clause 45 1 == clause_45_mode - If this bit is '1', the MDIO
 * interface will work in Clause 45 mode. 0 == clause_22_mode - The default is
 * Clause 22.
 */
#define SWREG_MDIO_MODE_CLAUSE_45                0x8UL
#define SWREG_MDIO_MODE_CLAUSE_45_CLAUSE_45_MODE (0x1UL << 3)
/* If this bit is '1', the MDIO interface will work in Clause 45 mode. */
#define SWREG_MDIO_MODE_CLAUSE_45_CLAUSE_22_MODE (0x0UL << 3)
/* The default is Clause 22. */
/* RESERVED[2:1] :Reserved bits */
#define SWREG_MDIO_MODE_RESERVED_1_MASK          0x6UL
#define SWREG_MDIO_MODE_RESERVED_1_SFT           1
/*
 * FREE_DIS[0:0] :This field disables the Free runing MDIO clock 1 ==
 * mdio_clk_free_run_dis - Disable Free running MDIO clock
 */
#define SWREG_MDIO_MODE_FREE_DIS                 0x1UL
#define SWREG_MDIO_MODE_FREE_DIS_MDIO_CLK_FREE_RUN_DIS (0x1UL << 0)
/* Disable Free running MDIO clock */
#define IPC_REG_NW_SERDES_MDIO_COMM		      0x4010220UL
/*
 * START_BUSY[29:29] :This bit is self clearing. When written to a '1', the
 * currently programmed MDIO transaction will activate. When the operation is
 * complete, this bit will clear and the MI_COMPLETE bit will be set in the
 * status register. Writing this bit as a '0' has no effect. This bit must be
 * read as a '0' before setting to prevent un-predictable results.
 */
#define NW_SERDES_MDIO_COMM_START_BUSY           0x20000000UL
/*
 * FAIL[28:28] :This bit is updated at the end of each MDIO transaction when the
 * START_BUSY bit is set. If an error occurred on the MDIO interface during the
 * operation, this bit will be updated to '1', otherwise, it will be updated to
 * '0'. Errors usually happen when the attached PHY fails to drive a response
 * during a read. This bit is only modified by completing a new MDIO
 * transaction.
 */
#define NW_SERDES_MDIO_COMM_FAIL                 0x10000000UL
/*
 * COMMAND[27:26] :This value is used to define command 1 == cmd_wr - Write 2 ==
 * cmd_rd - Read
 */
#define NW_SERDES_MDIO_COMM_COMMAND_MASK         0xc000000UL
#define NW_SERDES_MDIO_COMM_COMMAND_SFT          26
#define NW_SERDES_MDIO_COMM_COMMAND_CMD_WR       (0x1UL << 26)
/* Write */
#define NW_SERDES_MDIO_COMM_COMMAND_CMD_RD       (0x2UL << 26)
/* Read */
/*
 * PHY_ADDR[25:21] :This value is used to define the PHY address portion of the
 * MDIO transaction 1 == swreg_vmgmt - SWREG VMGMT 2 == swreg_vmain - SWREG
 * VMAIN 3 == swreg_vanalog - SWREG VANALOG 4 == swreg_v1p8 - SWREG V1p8
 */
#define NW_SERDES_MDIO_COMM_PHY_ADDR_MASK        0x3e00000UL
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SFT         21
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SWREG_VMGMT (0x1UL << 21)
/* SWREG VMGMT */
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SWREG_VMAIN (0x2UL << 21)
/* SWREG VMAIN */
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SWREG_VANALOG (0x3UL << 21)
/* SWREG VANALOG */
#define NW_SERDES_MDIO_COMM_PHY_ADDR_SWREG_V1P8  (0x4UL << 21)
/* SWREG V1p8 */
/*
 * REG_ADDR[20:16] :This value is used to define the register address portion of
 * the MDIO transaction
 */
#define NW_SERDES_MDIO_COMM_REG_ADDR_MASK        0x1f0000UL
#define NW_SERDES_MDIO_COMM_REG_ADDR_SFT         16
/*
 * DATA[15:0] :When this register is read, it returns the results of the last
 * MDIO transaction that was performed. When this register value is written, it
 * updates the value that will be used on the next MDIO write transaction that
 * will be performed.
 */
#define NW_SERDES_MDIO_COMM_DATA_MASK            0xffffUL
#define NW_SERDES_MDIO_COMM_DATA_SFT             0
#define IPC_REG_NW_SERDES_MDIO_STATUS		      0x4010224UL
/*
 * FAIL[1:1] :This bit is updated at the end of each MDIO transaction when the
 * START_BUSY bit is set. If an error occurred on the MDIO interface during the
 * operation, this bit will be updated to '1', otherwise, it will be updated to
 * '0'. Errors usually happen when the attached PHY fails to drive a response
 * during a read. This bit is only modified by completing a new MDIO
 * transaction.
 */
#define NW_SERDES_MDIO_STATUS_FAIL               0x2UL
/*
 * DONE[0:0] :This bit is set each time the MDIO transaction has completed. This
 * bit is cleared when the next transaction starts.
 */
#define NW_SERDES_MDIO_STATUS_DONE               0x1UL
#define IPC_REG_NW_SERDES_MDIO_MODE		      0x4010228UL
/*
 * CLOCK_CNT[21:12] :This field controls the MDIO clock speed. The output MDIO
 * clock runs at a frequency equal to CORE_CLK/(2*(CLOCK_CNT+1)). A value of 0
 * is invalid for this register.
 */
#define NW_SERDES_MDIO_MODE_CLOCK_CNT_MASK       0x3ff000UL
#define NW_SERDES_MDIO_MODE_CLOCK_CNT_SFT        12
/*
 * MDC[11:11] :MDC pin 1 == mdc_high - Setting this bit to '1' will cause the
 * MDC pin to high if the BIT_BANG bit is set. 0 == mdc_low - Setting this pin
 * low will cause the MDC pin to drive low if the BIT_BANG bit is set.
 */
#define NW_SERDES_MDIO_MODE_MDC                  0x800UL
#define NW_SERDES_MDIO_MODE_MDC_MDC_HIGH         (0x1UL << 11)
/*
 * Setting this bit to '1' will cause the MDC pin to high if the BIT_BANG bit is
 * set.
 */
#define NW_SERDES_MDIO_MODE_MDC_MDC_LOW          (0x0UL << 11)
/*
 * Setting this pin low will cause the MDC pin to drive low if the BIT_BANG bit
 * is set.
 */
/*
 * MDIO_OE[10:10] :MDIO Output Enable 1 == mdio_drive - Setting this bit to '1'
 * will cause the MDIO pin to drive the value written to the MDIO bit if the
 * BIT_BANG bit is set. 0 == mdio_in - Setting this bit to zero will make the
 * MDIO pin an input.
 */
#define NW_SERDES_MDIO_MODE_MDIO_OE              0x400UL
#define NW_SERDES_MDIO_MODE_MDIO_OE_MDIO_DRIVE   (0x1UL << 10)
/*
 * Setting this bit to '1' will cause the MDIO pin to drive the value written to
 * the MDIO bit if the BIT_BANG bit is set.
 */
#define NW_SERDES_MDIO_MODE_MDIO_OE_MDIO_IN      (0x0UL << 10)
/* Setting this bit to zero will make the MDIO pin an input. */
/*
 * MDIO[9:9] :The write value of this bit controls the drive state of the MDIO
 * pin if the BIT_BANG bit is set. The read value of this bit always reflects
 * the state of the MDIO pin.
 */
#define NW_SERDES_MDIO_MODE_MDIO                 0x200UL
/*
 * BIT_BANG[8:8] :BIT_BANG 1 == mdio_cntrl - If this bit is '1', the MDIO
 * interface is controlled by the MDIO, MDIO_OE, and MDC bits in this register.
 * 0 == mdio_cmd_run - When this bit is '0', the commands in the mdio_cmd
 * register will be executed.
 */
#define NW_SERDES_MDIO_MODE_BIT_BANG             0x100UL
#define NW_SERDES_MDIO_MODE_BIT_BANG_MDIO_CNTRL  (0x1UL << 8)
/*
 * If this bit is '1', the MDIO interface is controlled by the MDIO, MDIO_OE,
 * and MDC bits in this register.
 */
#define NW_SERDES_MDIO_MODE_BIT_BANG_MDIO_CMD_RUN (0x0UL << 8)
/*
 * When this bit is '0', the commands in the mdio_cmd register will be executed.
 */
/* RESERVED[7:4] :Reserved bits */
#define NW_SERDES_MDIO_MODE_RESERVED_MASK        0xf0UL
#define NW_SERDES_MDIO_MODE_RESERVED_SFT         4
/*
 * CLAUSE_45[3:3] :Clause 45 1 == clause_45_mode - If this bit is '1', the MDIO
 * interface will work in Clause 45 mode. 0 == clause_22_mode - The default is
 * Clause 22.
 */
#define NW_SERDES_MDIO_MODE_CLAUSE_45            0x8UL
#define NW_SERDES_MDIO_MODE_CLAUSE_45_CLAUSE_45_MODE (0x1UL << 3)
/* If this bit is '1', the MDIO interface will work in Clause 45 mode. */
#define NW_SERDES_MDIO_MODE_CLAUSE_45_CLAUSE_22_MODE (0x0UL << 3)
/* The default is Clause 22. */
/* RESERVED[2:1] :Reserved bits */
#define NW_SERDES_MDIO_MODE_RESERVED_1_MASK      0x6UL
#define NW_SERDES_MDIO_MODE_RESERVED_1_SFT       1
/*
 * FREE_DIS[0:0] :This field disables the Free runing MDIO clock 1 ==
 * mdio_clk_free_run_dis - Disable Free running MDIO clock
 */
#define NW_SERDES_MDIO_MODE_FREE_DIS             0x1UL
#define NW_SERDES_MDIO_MODE_FREE_DIS_MDIO_CLK_FREE_RUN_DIS (0x1UL << 0)
/* Disable Free running MDIO clock */
#define IPC_REG_FREQ_CAPTURE			      0x401022cUL
/*
 * Setting this bit high will result in the HW to capture the frequency of Main,
 * STORM and NW clocks. This is a self clearing bit.
 */
#define IPC_REG_FREQ_CLEAR			      0x4010230UL
/* Setting this bit high will clear the frequency counters to 0. */
#define IPC_REG_FREQ_MAIN			      0x4010234UL
/* Multi Field Register. */
	/*
	 * This field shows the frequency counter for main clock over a
	 * 10uS interval. Main Clock Frequency == ~(FreqCnt / 10)Mhz
	 */
	#define IPC_REG_FREQ_MAIN_CNT_MASK		      0xffffUL
	#define IPC_REG_FREQ_MAIN_CNT_SFT			  0
	/*
	 * 0 == invld - Value in freq_cnt field is not valid 1 == vld -
	 * Value in freq_cnt field is valid
	 */
	#define IPC_REG_FREQ_MAIN_CNT_VALID		      0x10000UL
#define IPC_REG_FREQ_AHB			      0x4010238UL
/* Multi Field Register. */
	/*
	 * This field shows the frequency counter for main clock over a
	 * 10uS interval. Storm Clock Frequency == ~(FreqCnt / 10)Mhz
	 */
	#define IPC_REG_FREQ_STORM_CNT_MASK		      0xffffUL
	#define IPC_REG_FREQ_STORM_CNT_SFT			  0
	/*
	 * 0 == invld - Value in freq_cnt field is not valid 1 == vld -
	 * Value in freq_cnt field is valid
	 */
	#define IPC_REG_FREQ_STORM_CNT_VALID		      0x10000UL
#define IPC_REG_FREQ_CHIMP			      0x401023cUL
/* Multi Field Register. */
	/*
	 * This field shows the frequency counter for main clock over a
	 * 10uS interval. NW Clock Frequency == ~(FreqCnt / 10)Mhz
	 */
	#define IPC_REG_FREQ_NW_CNT_MASK		      0xffffUL
	#define IPC_REG_FREQ_NW_CNT_SFT			  0
	/*
	 * 0 == invld - Value in freq_cnt field is not valid 1 == vld -
	 * Value in freq_cnt field is valid
	 */
	#define IPC_REG_FREQ_NW_CNT_VALID		      0x10000UL
#define IPC_REG_FREE_RUNNING_CNTR_0		      0x4010240UL
/* This is a 32-bit free running counter that has 1us resolution. */
#define IPC_REG_FREE_RUNNING_CNTR_1		      0x4010244UL
/* This is a 32-bit free running counter that has 16us resolution. */
#define IPC_REG_FREE_RUNNING_CNTR_2		      0x4010248UL
/* This is a 32-bit free running counter that has 256us resolution. */
#define IPC_REG_FREE_RUNNING_CNTR_3		      0x401024cUL
/* This is a 32-bit free running counter that has 4096us resolution. */
#define IPC_REG_FREE_RUNNING_CNTR_4		      0x4010250UL
/* This is a 32-bit free running counter that has 65536us resolution. */
#define IPC_REG_LED_PORT_MODE_A0		      0x4010254UL
/*
 * Reg removed in Cumulus B0!!! This register sets the Port Mode for the Network
 * interface. 0 : 1x40G 1 : 1x50G 2 : 2x25G 3 : 2x20G 4 : 2x10G Others: Unused
 */
#define IPC_REG_LED_SWAP			      0x4010258UL
/* Multi Field Register. */
	/* Field removed */
	#define IPC_REG_P0_LANE_LED_SWAP_A0_MASK		0x3UL
	#define IPC_REG_P0_LANE_LED_SWAP_A0_SFT		  0
	/* Field removed */
	#define IPC_REG_P1_LANE_LED_SWAP_A0_MASK	       0x30UL
	#define IPC_REG_P1_LANE_LED_SWAP_A0_SFT		  4
	/* Field removed */
	#define IPC_REG_P2_LANE_LED_SWAP_A0_MASK	      0x300UL
	#define IPC_REG_P2_LANE_LED_SWAP_A0_SFT		  8
	/* Field removed */
	#define IPC_REG_P3_LANE_LED_SWAP_A0_MASK	      0x3000UL
	#define IPC_REG_P3_LANE_LED_SWAP_A0_SFT		 12
	/* Field removed */
	#define IPC_REG_P0_MAC_LED_SWAP_A0_MASK	      0x30000UL
	#define IPC_REG_P0_MAC_LED_SWAP_A0_SFT			 16
	/* Field removed */
	#define IPC_REG_P1_MAC_LED_SWAP_A0_MASK	      0x300000UL
	#define IPC_REG_P1_MAC_LED_SWAP_A0_SFT			 20
	/* Field removed */
	#define IPC_REG_P2_MAC_LED_SWAP_A0_MASK	      0x3000000UL
	#define IPC_REG_P2_MAC_LED_SWAP_A0_SFT			 24
	/* Field removed */
	#define IPC_REG_P3_MAC_LED_SWAP_A0_MASK	      0x30000000UL
	#define IPC_REG_P3_MAC_LED_SWAP_A0_SFT			 28
#define IPC_REG_LED_ACTIVITY_SWAP		      0x401025cUL
/* Multi Field Register. */
	/* Field removed */
	#define IPC_REG_P0_LED_ACTIVITY_SWAP_A0_MASK		0x3UL
	#define IPC_REG_P0_LED_ACTIVITY_SWAP_A0_SFT		  0
	/* Field removed */
	#define IPC_REG_P1_LED_ACTIVITY_SWAP_A0_MASK	       0x30UL
	#define IPC_REG_P1_LED_ACTIVITY_SWAP_A0_SFT		  4
	/* Field removed */
	#define IPC_REG_P2_LED_ACTIVITY_SWAP_A0_MASK	      0x300UL
	#define IPC_REG_P2_LED_ACTIVITY_SWAP_A0_SFT		  8
	/* Field removed */
	#define IPC_REG_P3_LED_ACTIVITY_SWAP_A0_MASK	      0x3000UL
	#define IPC_REG_P3_LED_ACTIVITY_SWAP_A0_SFT		 12
#define IPC_REG_LED_CTRL			      0x4010260UL
/* Multi Field Register. */
	/* Field removed */
	#define IPC_REG_LED_LINK_STATUS_BYPASS_A0		0x1UL
	/* Field removed */
	#define IPC_REG_LED_LINK_STATUS_SELECT_A0		0x2UL
	/* Field removed */
	#define IPC_REG_LED_SPEED_MODE_BYPASS_A0	       0x10UL
	/* Field removed */
	#define IPC_REG_CAPTURE_LED_A0			      0x100UL
#define IPC_REG_P0_LED_MODE_A0			      0x4010264UL
/* Reg removed in Cumulus B0!!! */
#define IPC_REG_P0_LED_SPD0_EN_A0		      0x4010268UL
/*
 * Reg removed in Cumulus B0!!! LED decode [0] -> 1G or lower [1] -> 10G [2] ->
 * 20G [3] -> 25G [4] -> 40G [5] -> 50G [6] -> 100G [7] -> unused A '1' to each
 * bit location will enable the corresponding speed to activate the LED.
 */
#define IPC_REG_P0_LED_SPD1_EN_A0		      0x401026cUL
/*
 * Reg removed in Cumulus B0!!! LED decode [0] -> 1G or lower [1] -> 10G [2] ->
 * 20G [3] -> 25G [4] -> 40G [5] -> 50G [6] -> 100G [7] -> unused A '1' to each
 * bit location will enable the corresponding speed to activate the LED.
 */
#define IPC_REG_P0_LED_CONTROL			      0x4010270UL
/* Multi Field Register. */
	/* Field removed */
	#define IPC_REG_P0_LED_CONTROL_OVERRIDE_TRAFFIC_A0     0x1UL
	/* Field removed */
	#define IPC_REG_P0_LED_CONTROL_TRAFFIC_A0	       0x10UL
	/* Field removed */
	#define IPC_REG_P0_LED_CONTROL_BLINK_TRAFFIC_A0      0x100UL
	/* Field removed */
	#define IPC_REG_P0_LED_CONTROL_BLINK_RATE_ENA_A0     0x1000UL
	/* Field removed */
	#define IPC_REG_P0_LED_CONTROL_BLINK_RATE_A0_MASK    0xfff00000UL
	#define IPC_REG_P0_LED_CONTROL_BLINK_RATE_A0_SFT	 20
#define IPC_REG_P0_MAC_LED_A0			      0x4010274UL
/*
 * Reg removed in Cumulus B0!!! LED decode [0] -> 1G or lower [1] -> 10G [2] ->
 * 20G [3] -> 25G [4] -> 40G [5] -> 50G [6] -> unused [7] -> unused This
 * register allows the MAC (Driver/FW) to set the link speed of the particular
 * port. This combined with the LED will activate the corresponding LED. For ex.
 * if the link speed is 10G, then SW will set bit[1] of this re
 */
#define IPC_REG_P1_LED_MODE_A0			      0x4010278UL
/* Reg removed in Cumulus B0!!! */
#define IPC_REG_P1_LED_SPD0_EN_A0		      0x401027cUL
/*
 * Reg removed in Cumulus B0!!! LED decode [0] -> 1G or lower [1] -> 10G [2] ->
 * 20G [3] -> 25G [4] -> 40G [5] -> 50G [6] -> 100G [7] -> unused A '1' to each
 * bit location will enable the corresponding speed to activate the LED.
 */
#define IPC_REG_P1_LED_SPD1_EN_A0		      0x4010280UL
/*
 * Reg removed in Cumulus B0!!! LED decode [0] -> 1G or lower [1] -> 10G [2] ->
 * 20G [3] -> 25G [4] -> 40G [5] -> 50G [6] -> 100G [7] -> unused A '1' to each
 * bit location will enable the corresponding speed to activate the LED.
 */
#define IPC_REG_P1_LED_CONTROL			      0x4010284UL
/* Multi Field Register. */
	/* Field removed */
	#define IPC_REG_P1_LED_CONTROL_OVERRIDE_TRAFFIC_A0     0x1UL
	/* Field removed */
	#define IPC_REG_P1_LED_CONTROL_TRAFFIC_A0	       0x10UL
	/* Field removed */
	#define IPC_REG_P1_LED_CONTROL_BLINK_TRAFFIC_A0      0x100UL
	/* Field removed */
	#define IPC_REG_P1_LED_CONTROL_BLINK_RATE_ENA_A0     0x1000UL
	/* Field removed */
	#define IPC_REG_P1_LED_CONTROL_BLINK_RATE_A0_MASK    0xfff00000UL
	#define IPC_REG_P1_LED_CONTROL_BLINK_RATE_A0_SFT	 20
#define IPC_REG_P1_MAC_LED_A0			      0x4010288UL
/*
 * Reg removed in Cumulus B0!!! LED decode [0] -> 1G or lower [1] -> 10G [2] ->
 * 20G [3] -> 25G [4] -> 40G [5] -> 50G [6] -> unused [7] -> unused This
 * register allows the MAC (Driver/FW) to set the link speed of the particular
 * port. This combined with the LED will activate the corresponding LED. For ex.
 * if the link speed is 10G, then SW will set bit[1] of this re
 */
#define IPC_REG_LED_RESOLVED_SPEED_A0		      0x401028cUL
/*
 * Reg removed in Cumulus B0!!! Resolved speed indication from each PM Lane.
 * [7:0] - Lane 0 - Lane 0 [15:8] - Lane 1 - Lane 1 [23:16] - Lane 2 - Lane 2
 * [31:24] - Lane 3 - Lane 3
 */
#define IPC_REG_LED_RAW_SPEED_A0		      0x4010290UL
/*
 * Reg removed in Cumulus B0!!! Raw indication from each PM Lane. [7:0] - Lane 0
 * - Lane 0 [15:8] - Lane 1 - Lane 1 [23:16] - Lane 2 - Lane 2 [31:24] - Lane 3
 * - Lane 3
 */
#define IPC_REG_LED_SPEED_MODE_A0		      0x4010294UL
/* Reg removed in Cumulus B0!!! */
#define IPC_REG_PM_LOS_CONN			      0x4010298UL
/* Multi Field Register. */
	/*
	 * This field provides the mapping of external signal detect to
	 * LOS for Lane 0 0 -> Port 0 signal detect will connect to LOS
	 * 1 -> Port 1 signal detect will connect to LOS 7 -> LOS will
	 * be tied to 0
	 */
	#define IPC_REG_PM_L0_LOS_CONN_MASK			0x7UL
	#define IPC_REG_PM_L0_LOS_CONN_SFT			  0
	/*
	 * This field provides the mapping of external signal detect to
	 * LOS for Lane 1 0 -> Port 0 signal detect will connect to LOS
	 * 1 -> Port 1 signal detect will connect to LOS 7 -> LOS will
	 * be tied to 0
	 */
	#define IPC_REG_PM_L1_LOS_CONN_MASK		       0x70UL
	#define IPC_REG_PM_L1_LOS_CONN_SFT			  4
	/*
	 * This field provides the mapping of external signal detect to
	 * LOS for Lane 2 0 -> Port 0 signal detect will connect to LOS
	 * 1 -> Port 1 signal detect will connect to LOS 7 -> LOS will
	 * be tied to 0
	 */
	#define IPC_REG_PM_L2_LOS_CONN_MASK		      0x700UL
	#define IPC_REG_PM_L2_LOS_CONN_SFT			  8
	/*
	 * This field provides the mapping of external signal detect to
	 * LOS for Lane 3 0 -> Port 0 signal detect will connect to LOS
	 * 1 -> Port 1 signal detect will connect to LOS 7 -> LOS will
	 * be tied to 0
	 */
	#define IPC_REG_PM_L3_LOS_CONN_MASK		      0x7000UL
	#define IPC_REG_PM_L3_LOS_CONN_SFT			 12
#define IPC_REG_EXTSIG_POLARITY			      0x401029cUL
/* Multi Field Register. */
	/*
	 * This field provides the polarity of signal detect for Port 0
	 * 0 -> Signal Detect is active low 1 -> Signal Detect is active
	 * high
	 */
	#define IPC_REG_SIGDET_P0_POLARITY			0x1UL
	/*
	 * This field provides the polarity of signal detect for Port 1
	 * 0 -> Signal Detect is active low 1 -> Signal Detect is active
	 * high
	 */
	#define IPC_REG_SIGDET_P1_POLARITY			0x2UL
	/*
	 * This field provides the polarity of LASI for port 0 0 -> LASI
	 * is active low 1 -> LASI is active high
	 */
	#define IPC_REG_LASI_P0_POLARITY		       0x10UL
	/*
	 * This field provides the polarity of LASI for port 1 0 -> LASI
	 * is active low 1 -> LASI is active high
	 */
	#define IPC_REG_LASI_P1_POLARITY		       0x20UL
	/* TBA */
	#define IPC_REG_LINK_STATUS_SELECT		      0x100UL
#define IPC_REG_VMAIN_POR_STATUS		      0x40102a0UL
/*
 * This register shows the current status of the VMAIN POR. 0 == vmain_down -
 * VMAIN is down 1 == vmain_up - VMAIN is up
 */
#define IPC_REG_STAT_VMAIN_POR_ASSERTION	      0x40102a4UL
/*
 * This register provides the number of times VMAIN POR was asserted. This would
 * be the count of number of times VMAIN went down.
 */
#define IPC_REG_STAT_VMAIN_POR_DEASSERTION	      0x40102a8UL
/*
 * This register provides the number of times VMAIN POR was de-asserted. This
 * would be the count of number of times VMAIN came up.
 */
#define IPC_REG_PERST_POR_STATUS		      0x40102acUL
/*
 * This register shows the current status of the PERST#. 0 == perst_assert -
 * PERST is asserted 1 == perst_deassert - PERST is de-asserted
 */
#define IPC_REG_STAT_PERST_ASSERTION		      0x40102b0UL
/* This register provides the number of times PERST# was asserted */
#define IPC_REG_STAT_PERST_DEASSERTION		      0x40102b4UL
/* This register provides the number of times PERST# was de-asserted */
#define IPC_REG_CHIP_MODE			      0x40102b8UL
/* Multi Field Register. */
/* This register shows the current mode of the chip. Fields: */
/*
 * POR_Bypass_mode[5:5]:Run all the modes in POR Bypass mode 1 == por_bypass -
 * Run all the modes in POR Bypass mode
 */
#define CHIP_MODE_FINAL_POR_BYPASS_MODE          0x20UL
#define CHIP_MODE_FINAL_POR_BYPASS_MODE_POR_BYPASS (0x1UL << 5)
/* Run all the modes in POR Bypass mode */
/*
 * Fast_Reset_mode[4:4]:Run all the modes in Fast Reset (useful in
 * Simulation/ATE) 1 == fast_reset - Run all the modes in Fast Reset (useful in
 * Simulation/ATE)
 */
#define CHIP_MODE_FINAL_FAST_RESET_MODE          0x10UL
#define CHIP_MODE_FINAL_FAST_RESET_MODE_FAST_RESET (0x1UL << 4)
/* Run all the modes in Fast Reset (useful in Simulation/ATE) */
/*
 * chip_mode[3:0] :Defines chip mode running 0x0 == Mission Mode 0x1 == Scan
 * Mode 0x2 == Debug Mode 0x3 == PCIe SERDES Standalone mode 0x4 == MAC SERDES
 * Standalone mode 0x5 == IDDQ Mode 0x6 == OVSTB Mode
 */
#define CHIP_MODE_FINAL_CHIP_MODE_MASK           0xfUL
#define CHIP_MODE_FINAL_CHIP_MODE_SFT            0
#define CHIP_MODE_FINAL_CHIP_MODE_MISSION        (0x0UL << 0)
#define CHIP_MODE_FINAL_CHIP_MODE_SCAN           (0x1UL << 0)
#define CHIP_MODE_FINAL_CHIP_MODE_DEBUG          (0x2UL << 0)
#define CHIP_MODE_FINAL_CHIP_MODE_PCIE           (0x3UL << 0)
#define CHIP_MODE_FINAL_CHIP_MODE_MAC            (0x4UL << 0)
#define CHIP_MODE_FINAL_CHIP_MODE_IDDQ           (0x5UL << 0)
#define CHIP_MODE_FINAL_CHIP_MODE_OVSTB          (0x6UL << 0)
/* This register shows the mode straps. Fields: */
/*
 * POR_Bypass_mode[13:13]:Run all the modes in POR Bypass mode 1 == por_bypass -
 * Run all the modes in POR Bypass mode
 */
#define CHIP_MODE_STRAPS_POR_BYPASS_MODE         0x2000UL
#define CHIP_MODE_STRAPS_POR_BYPASS_MODE_POR_BYPASS (0x1UL << 13)
/* Run all the modes in POR Bypass mode */
/*
 * Fast_Reset_mode[12:12]:Run all the modes in Fast Reset (useful in
 * Simulation/ATE) 1 == fast_reset - Run all the modes in Fast Reset (useful in
 * Simulation/ATE)
 */
#define CHIP_MODE_STRAPS_FAST_RESET_MODE         0x1000UL
#define CHIP_MODE_STRAPS_FAST_RESET_MODE_FAST_RESET (0x1UL << 12)
/* Run all the modes in Fast Reset (useful in Simulation/ATE) */
/*
 * chip_mode[11:8] :Defines chip mode running 0x0 == Mission Mode 0x1 == Scan
 * Mode 0x2 == Debug Mode 0x3 == PCIe SERDES Standalone mode 0x4 == MAC SERDES
 * Standalone mode 0x5 == IDDQ Mode 0x6 == OVSTB Mode
 */
#define CHIP_MODE_STRAPS_CHIP_MODE_MASK          0xf00UL
#define CHIP_MODE_STRAPS_CHIP_MODE_SFT           8
#define CHIP_MODE_STRAPS_CHIP_MODE_MISSION       (0x0UL << 8)
#define CHIP_MODE_STRAPS_CHIP_MODE_SCAN          (0x1UL << 8)
#define CHIP_MODE_STRAPS_CHIP_MODE_DEBUG         (0x2UL << 8)
#define CHIP_MODE_STRAPS_CHIP_MODE_PCIE          (0x3UL << 8)
#define CHIP_MODE_STRAPS_CHIP_MODE_MAC           (0x4UL << 8)
#define CHIP_MODE_STRAPS_CHIP_MODE_IDDQ          (0x5UL << 8)
#define CHIP_MODE_STRAPS_CHIP_MODE_OVSTB         (0x6UL << 8)
/* This register shows the shifted in mode. Fields: */
/*
 * POR_Bypass_mode[21:21]:Run all the modes in POR Bypass mode 1 == por_bypass -
 * Run all the modes in POR Bypass mode
 */
#define CHIP_MODE_SHIFTED_POR_BYPASS_MODE        0x200000UL
#define CHIP_MODE_SHIFTED_POR_BYPASS_MODE_POR_BYPASS (0x1UL << 21)
/* Run all the modes in POR Bypass mode */
/*
 * Fast_Reset_mode[20:20]:Run all the modes in Fast Reset (useful in
 * Simulation/ATE) 1 == fast_reset - Run all the modes in Fast Reset (useful in
 * Simulation/ATE)
 */
#define CHIP_MODE_SHIFTED_FAST_RESET_MODE        0x100000UL
#define CHIP_MODE_SHIFTED_FAST_RESET_MODE_FAST_RESET (0x1UL << 20)
/* Run all the modes in Fast Reset (useful in Simulation/ATE) */
/*
 * chip_mode[19:16] :Defines chip mode running 0x0 == Mission Mode 0x1 == Scan
 * Mode 0x2 == Debug Mode 0x3 == PCIe SERDES Standalone mode 0x4 == MAC SERDES
 * Standalone mode 0x5 == IDDQ Mode 0x6 == OVSTB Mode
 */
#define CHIP_MODE_SHIFTED_CHIP_MODE_MASK         0xf0000UL
#define CHIP_MODE_SHIFTED_CHIP_MODE_SFT          16
#define CHIP_MODE_SHIFTED_CHIP_MODE_MISSION      (0x0UL << 16)
#define CHIP_MODE_SHIFTED_CHIP_MODE_SCAN         (0x1UL << 16)
#define CHIP_MODE_SHIFTED_CHIP_MODE_DEBUG        (0x2UL << 16)
#define CHIP_MODE_SHIFTED_CHIP_MODE_PCIE         (0x3UL << 16)
#define CHIP_MODE_SHIFTED_CHIP_MODE_MAC          (0x4UL << 16)
#define CHIP_MODE_SHIFTED_CHIP_MODE_IDDQ         (0x5UL << 16)
#define CHIP_MODE_SHIFTED_CHIP_MODE_OVSTB        (0x6UL << 16)
	/* 1 : use shifted in mode 0 : use straps */
	#define IPC_REG_CHIP_MODE_USE_SHIFTED		      0x1000000UL
#define IPC_REG_HW_STRAPS			      0x40102bcUL
/*
 * HW Straps 5:0 - MODE Straps - mode strap 6:6 - Bono Bypass Strap - bono
 * bypass strap 8:7 - JTAG CE strap - jtag ce strap
 */
#define IPC_REG_CLK_OUT_SEL			      0x40102c0UL
/* select clocks that will come out on clk50_out. default is clk_25. */
#define IPC_REG_OTP_CONFIG_0			      0x40102c4UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_1			      0x40102c8UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_2			      0x40102ccUL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_3			      0x40102d0UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_4			      0x40102d4UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_5			      0x40102d8UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_6			      0x40102dcUL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_7			      0x40102e0UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_8			      0x40102e4UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_9			      0x40102e8UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_10			      0x40102ecUL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_11			      0x40102f0UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_12			      0x40102f4UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_13			      0x40102f8UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_14			      0x40102fcUL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_OTP_CONFIG_15			      0x4010300UL
/*
 * These bits represent the 512-bits of the configuration space in the OTP that
 * is read out at POR.
 */
#define IPC_REG_SEL_VAUX			      0x4010304UL
/*
 * SEL_VAUX_B - Control to power switching logic. [0] - output value driven by
 * chip; [1] - input pin value.
 */
#define IPC_REG_VAUX_EN_DIS			      0x4010308UL
/*
 * VAUX_ENABLE/DISABLE. [7-6] FLOAT When any of these bits is written as a '1';
 * the corresponding vaux_enable/vaux_disable bit will turn off it's drivers and
 * become an input. This is the reset state of all pins. The read value of these
 * bits will be a '1' if the last command ( SET ; CL ; or FLOAT ) for this bit
 * was a FLOAT . (reset value 0x3). [5-4] CLR When any of these bits is written
 * as a '1'; the corresponding vaux_enable/vaux_disable bit will drive low. The
 * read value of these bits will be a '1' if the last command ( SET ; CLR ; or
 * FLOAT ) for this bit was a CLR . (reset value 0). [3-2] SET When any of these
 * bits is written as a '1'; the corresponding vaux_enable/vaux_disable bit will
 * drive high. The read value of these bits will be a '1' if the last command (
 * SET ; CLR ; or FLOAT ) for this bit was a SET . (reset value 0). [1-0] VALUE
 * RO ; These bits indicate the read value of vaux_enable/vaux_disable pins.
 * This is the result value of the pin; not the drive value. Writing these bits
 * will have not effect. [0] VAUX Enable; when pulsed low; enables supply from
 * VAUX. (This is an output pin only; the FLOAT field is not applicable for this
 * pin); [1] VAUX Disable; when pulsed low; disables supply form VAUX. (This is
 * an output pin only; FLOAT field is not applicable for this pin); Global
 * register.
 */
#define IPC_REG_VAUX_EN_DIS_INT			      0x401030cUL
/*
 * VAUX_ENABLE/DISABLE INT. [7-6] OLD_CLR Writing a '1' to these bit clears the
 * corresponding bit in the OLD_VALUE register. This will acknowledge an
 * interrupt on the falling edge of corresponding vaux_enable/vaux_disable input
 * (reset value 0). [5-4] OLD_SET Writing a '1' to these bit sets the
 * corresponding bit in the OLD_VALUE register. This will acknowledge an
 * interrupt on the rising edge of corresponding vaux_enable/vaux_disable input
 * (reset value 0). [3-2] OLD_VALUE RO; These bits indicate the old value of the
 * vaux_enable/vaux_disable input value. When the INT_STATE bit is set; this bit
 * indicates the OLD value of the pin such that if INT_STATE is set and this bit
 * is '0'; then the interrupt is due to a low to high edge. If INT_STATE is set
 * and this bit is '1'; then the interrupt is due to a high to low edge (reset
 * value 0). [1-0] INT_STATE RO; These bits indicate the current
 * vaux_enable/vaux_disable interrupt state for each vaux_enable/vaux_disable
 * pin. This bit is cleared when the appropriate OLD_SET or OLD_CLR command bit
 * is written. This bit is set when the vaux_enable/vaux_disable input does not
 * match the current value in OLD_VALUE (reset value 0). Global register.
 */
#define IPC_REG_VAUX_PRESENT			      0x4010310UL
/* 0 - VAUX is not present; 1 - VAUX is present. */
#define IPC_REG_UCINT_PCIE_MODE			      0x4010314UL
/* Multi Field Register. */
	/*
	 * This field controls the swapping of the data register bytes
	 * when accessing the uC interface. 0 - Little Endian data [7:0]
	 * -> addr, data[15:8] -> addr+1, data [23:16] -> addr+2,
	 * data[31:24] -> addr+3 1 - Big Endian data [31:24] -> addr,
	 * data[23:16] -> addr+1, data [15:8] -> addr+2, data[7:0] ->
	 * addr+3 Default is 0 (Little Endian)
	 */
	#define IPC_REG_UCINT_PCIE_MODE_BYTE_SWAP	      0x100UL
	/*
	 * This field controls how many dummy ext_mem_clk cycles will be
	 * driven when a new target is enabled based on a change in the
	 * access_mode field.
	 */
	#define IPC_REG_UCINT_PCIE_MODE_DUMMY_CYCLES_MASK    0xff0000UL
	#define IPC_REG_UCINT_PCIE_MODE_DUMMY_CYCLES_SFT	 16
#define IPC_REG_UCINT_PCIE_CLK_DIV		      0x4010318UL
/*
 * This register controls the clock speed for the 2 PCIE SERDES microcontroller
 * program memory interfaces. All clocks are divided from the core_clk. 0 -
 * core_clk / 4 1 - core_clk / 8 2 - core_clk / 16
 */
#define IPC_REG_UCINT_PCIE_ADDRESS		      0x401031cUL
/*
 * This register controls the address offset for the 2 PCIE SERDES
 * microcontroller program memory interfaces. This register auto-increments
 * after each transaction.
 */
#define IPC_REG_UCINT_PCIE_DATA			      0x4010320UL
/*
 * Read/write data register for the 2 PCIE SERDES microcontroller program memory
 * interfaces. Accessing this register will start the transaction specified in
 * the mode register.
 */
#define IPC_REG_GPIO_MUX_SEL			      0x4010324UL
/*
 * This register allows SW/FW to select the right GPIO source for external GPIO
 * pins [15:0] : These bits allow SW to choose between GPIO from APE GPIO[15:0]
 * or CHiMP GPIO[15:0]. Each bit allows to select the corresponding GPIO pin. 0
 * - APE GPIO is selected 1 - CHiMP GPIO is selected [31:16] : Cumulus has 16
 * dedicated GPIOs. There are potential cases where more than 16 GPIOs will be
 * needed Other pins on the chip will be overloaded with GPIO functionality.
 * These bits allow selection of normal function or GPIO function (CHiMP
 * GPIO[31:16]). 0 - Normal Function is selected 1 - CHiMP GPIO is selected
 */
#define IPC_REG_SMB_I2C_MUX_SEL			      0x4010328UL
/*
 * Cumulus has 3 sets of I/O dedicated for I2C/SMBUS operation. By default Set 1
 * is dedicated to I2C connectivity to SFP+ cages. Set 2 and 3 default for SMBUS
 * operation To Support two ports (hence 2 SFP+ cages), two sets of I2C
 * interface is needed or an external I2C mux needs to be added on the board as
 * the SFP+ cages all respond to the same address. To avoid adding an external
 * mux, the SMBUS interfaces can be converted to I2C interfaces. These set of
 * bits allow SW to select between SMBUS or I2C operation. [0] : 0 - SMBUS
 * interface 0 is selected 1 - I2C interface 1 is selected [1] : 0 - SMBUS
 * interface 1 is selected 1 - I2C interface 2 is selected
 */
#define IPC_REG_JTAG_COMPLIANCE			      0x401032cUL
/* Multi Field Register. */
	/*
	 * These bits set the compliance enable for JTAG pins. the JTAG
	 * interface is shared by four masters and there is a dedicated
	 * 2-bit compliance enable pins on the ballout. These bits are
	 * used to override the pins if needed. 0x0 == lv_jtag - LV JTAG
	 * is selected 0x1 == avs_jtag - AVS JTAG is selected 0x2 ==
	 * proc_jtag - Processor (CHiMP, APE, TANG, KONG, BONO) JTAG is
	 * selected 0x3 == avs_ejtag - AVS EJTAG is selected
	 */
	#define IPC_REG_JTAG_COMPLIANCE_EN_MASK		0x3UL
	#define IPC_REG_JTAG_COMPLIANCE_EN_SFT			  0
	/* Set this bit to override the pins on the chip with bits[1:0] */
	#define IPC_REG_JTAG_COMPLIANCE_OVERRIDE	       0x10UL
	/*
	 * Set this bit to enable BONO in the processor JTAG chain. BONO
	 * is a embedded processor that is instantiated in the VHOST
	 * domain compared to other processors. so it is possible that
	 * BONO is powered off when JTAG of other processors need to be
	 * accessed.
	 */
	#define IPC_REG_JTAG_BONO_ENABLE		      0x100UL
#define IPC_REG_MAC_BUSY			      0x4010330UL
/*
 * MAC and PHY will be sharing the NVRAM in Whitney-BT. The baseline operation
 * is for the FW to load the FW into the Copper PHY through MDIO interface. A
 * back-up solution is to use a HW state machine to manage arbitration betweent
 * the two chip dies. the interface is a two pin interface where each side tells
 * the other if they are busy using the NVRAM. Cumulus will come up as the
 * master of interface and will be the first to access the NVRAM. In Cumulus
 * this state machine will be managed by FW using registers. This register bit
 * allows FW to tell that the MAC (Cumulus) is busy.
 */
#define IPC_REG_PHY_BUSY			      0x4010334UL
/*
 * This register bit shows the current status of the BUSY signal from the PHY.
 * in addition any transition on this signal will generate an interrupt to FW
 * through the INT registers.
 */
#define IPC_REG_NVRAM_MASTER			      0x4010338UL
/*
 * This bit selects which device is the master of the external NVRAM Interface 0
 * - MAC Master - Cumulus will be master of the NVRAM 1 - PHY Master - Copper
 * PHY will be master of the NVRAM
 */
#define IPC_REG_EXT_PHY_RESET			      0x401033cUL
/*
 * Active High. Setting this bit to 1 resets the external PHY in Whitney-BT.
 * Default is to keep the PHY in reset.
 */
#define IPC_REG_SWREG_MDIO_MASTER		      0x4010340UL

#endif /* _PM_CFG_DATA_H_ */
