/*
 * Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __BCM_AMAC_REGS_H__
#define __BCM_AMAC_REGS_H__

#include <linux/types.h>

/* End of feature selection/configuration */
/*#define RMO_SUPPORT (0)*/

/*#define EMULATION_TEST (0)*/

#define PORTMACRO_ENABLE (1)

/*#define ENABLE_DMA_LOOPBACK (0)*/

/*#define MAC_LOOPBACK (0)*/

/* End of feature selection/configuration */

#ifdef MAC_LOOPBACK
#undef ENABLE_DMA_LOOPBACK
#endif

#define AMAC_MAX_PORTS 4

#define UNIMAC_PORT_NUM_PEG (3)

#define MAX_NUM_DESCS (512)
#define AMAC_RX_BUF_SIZE      2048 /* MAX RX buffer size */
#define AMAC_DMA_RX_DESC_CNT  MAX_NUM_DESCS /* Number of rx dma descriptors */
#define DMA_RX_DESC_NUM	MAX_NUM_DESCS /* Number of rx dma descriptors */
#define DMA_RX_BUF_LEN	2016

#define RCV_PKT_INFO_CRC_ERR_OFFSET	BIT(3)
#define RCV_PKT_INFO_RX_OVERFLOW	BIT(7)
#define DMA_TX_MAX_CHAIN_LEN	128 /* Limit TX DMA chain len */
#define AMAC_DMA_RXALIGN      16 /* Alignment for SKB */
/* 802.3as defines max packet size to be 2000 bytes, size is rounded up
 * to be multiple of 32 to be cache aligned
 */
#define AMAC_DMA_RX_BUF_LEN   2016

#define AMAC_DMA_TX_CHAIN_LEN_MAX   128 /* Limit TX DMA chain len */
/* Must be power of two because of the use of kfifo */
#define AMAC_DMA_TX_MAX_QUEUE_LEN  MAX_NUM_DESCS
/* Two descriptors per packet, one each for: config data and payload */
#define AMAC_DMA_TX_DESC_CNT  (AMAC_DMA_TX_MAX_QUEUE_LEN * 2)
#define DMA_TX_DESC_NUM		AMAC_DMA_TX_MAX_QUEUE_LEN

/* PORT Settings */
#define AMAC_PORT_SPEED_1G    SPEED_1000
#define AMAC_PORT_SPEED_100M  SPEED_100
#define AMAC_PORT_SPEED_10M   SPEED_10

#define AMAC_PORT_DEFAULT_SPEED   AMAC_PORT_SPEED_1G

#define MIN_FRAME_LEN 64

#define PEG_DMA_MASK DMA_BIT_MASK(34)
#define DEFAULT_DMA_MASK DMA_BIT_MASK(32)
#define RX_ALIGN_BOUNDARY (64)

extern int unimac_port_num;
extern bool portmacro_enable;

#ifdef PORTMACRO_ENABLE
#undef PRBS_LOOPBACK
#endif

#define IS_UNIMAC_PATH (privp->b_unimac == 1 && privp->amac_id == \
			unimac_port_num)
#define IS_UNIMAC_PORT (privp->amac_id == unimac_port_num)
#define IS_PORTMACRO_ENABLED (portmacro_enable == true)

#define NICPM_ROOT			0x61030000
#define NICPM_PADRING_CFG		0x00000004
#define NICPM_IOMUX_CTRL		0x00000008

#define NICPM_PADRING_CFG_INIT_VAL	0x74000000
#define NICPM_IOMUX_CTRL_INIT_VAL	0x21880000
#define NICPM_IOMUX_CTRL_INIT_VAL_BX	0x3196e800

/* Offsets from GMAC_DEVCONTROL */
#define GMAC_INT_STATUS_REG      0x020
#define GMAC_INT_MASK_REG        0x024

#define GMAC_INTR_RX_LAZY_REG    0x100
#define GMAC_INTR_RX_LAZY_REG_PEG	0x30
#define GMAC_PHY_CTRL_REG        0x188

#define GMAC_DMA_TX_CTRL_REG     0x200
#define GMAC_DMA_TX_PTR_REG      0x204
#define GMAC_DMA_TX_ADDR_LO_REG  0x208
#define GMAC_DMA_TX_ADDR_HI_REG  0x20c
#define GMAC_DMA_TX_STATUS0_REG  0x210
#define GMAC_DMA_TX_STATUS1_REG  0x214

#define GMAC_DMA_RX_CTRL_REG     0x220
#define GMAC_DMA_RX_PTR_REG      0x224
#define GMAC_DMA_RX_ADDR_LO_REG  0x228
#define GMAC_DMA_RX_ADDR_HI_REG  0x22c
#define GMAC_DMA_RX_STATUS0_REG  0x230
#define GMAC_DMA_RX_STATUS1_REG  0x234
#define GMAC_CHECKSUM_CTRL_REG   0x400

#define UNIMAC_CMD_CFG_REG       0x808

#define GMAC0_IRL_FRAMECOUNT_SHIFT  24
#define RX_LAZY_TIMEO 192000
#define DEFAULT_RX_INT_CNT (48)

/* CTF registers */
#define AMAC3_CTF_OFFSET          0xc00
#define CTF_CONTROL_REG        0x00
#define CTF_PORTS_CONFIG_REG   0x1d8
#define CTF_PASS_TRU_PORT0_REG 0x08
#define CTF_PASS_TRU_PORT1_REG 0x0c
#define CTF_PASS_TRU_PORT2_REG 0x10
#define CTF_PASS_TRU_PORT3_REG 0x14
#define CTF_PROPERTY_PORT_BASE 0x2e8
#define CTF_PROPERTY_PAUSE_TO_PORT (8)

#define CTF_PORT3_UNIMAC_ENABLE		BIT(8)
#define CTF_PASS_TRU_PORT_VALUE		0x10000

/* PHY registers */
#define GPHY_MII_CTRL_REG           0x00
#define GPHY_MII_CTRL_REG_PWR_MASK  0x800
#define GPHY_MII_CTRL_REG_RST_MASK  0x8000

#define GPHY_EXP_DATA_REG           0x15
#define GPHY_EXP_SELECT_REG         0x17
#define GPHY_MISC_CTRL_REG          0x18  /* shadow 7 */
#define GPHY_CLK_ALIGNCTRL_REG      0x1C  /* Shadow 3 */

/* Initialization values of above PHY registers */
#define GPHY_EXP_DATA_REG_VAL                  0x11B
#define GPHY_EXP_SELECT_REG_VAL_LANE_SWAP      0x0F09
#define GPHY_EXP_SELECT_REG_VAL_BROADREACH_OFF 0x0F90
#define GPHY_MISC_CTRL_REG_SKEW_DISABLE_VAL    0xF0E7
#define GPHY_CLK_GTX_DELAY_DISALE_WR_VAL       0x8c00
#define GPHY_MISC_CTRL_REG_DELAY_DISABLE_VAL   0x7007
#define GPHY_CLK_GTX_DELAY_DISALE_RD_VAL       0x0c00

/* AMAC IDM Registers */
#define AMAC_IDM_IO_CTRL_DIRECT_REG (0x8)
#define AMAC_IDM_IO_CTRL_DEST_SYNC_MODE_EN_BIT 3
#define AMAC_IDM_IO_CTRL_GMII_MODE_BIT 5
#define AMAC_IDM_IO_CTRL_CLK_250_SEL_BIT 6
#define AMAC_IDM_IO_CTRL_ARCACHE_OFFSET (16)
#define AMAC_IDM_IO_CTRL_AWCACHE_OFFSET (7)

#define AMAC_FLOW_CTRL_THRESH_REG (0x104)
#define AMAC_FLOW_CTRL_OFF_THRESH_OFF (16)
#define AMAC_FLOW_CTRL_ON_THRESH_OFF (0)
/*Make thresholds nearly 300 multiple for IMIX*/
#define AMAC_FLOW_CTRL_ON_THRESH (450)
#define AMAC_FLOW_CTRL_OFF_THRESH (394)

#define IDM_RST_CTRL_REG	(0x400)
#define AMAC_RESET_ENABLE	BIT(0)

/* Offsets from Switch Global Config registers */
#define CDRU_SWITCH_CFG_BYPASS_SWITCH 0xD

/* Offsets from CRMU Chip IO Pad Control */
#define CRMU_CHIP_IO_PAD_CONTROL__CDRU_IOMUX_FORCE_PAD_IN 0

/* CheckSum control fields */
#define CKSUM_CONTROL_CTF_BYPASS	BIT(5)

/* register-specific flag definitions */
/* device control */
#define DC_TSM          0x00000002
#define DC_CFCO         0x00000004
#define DC_RLSS         0x00000008
#define DC_MROR         0x00000010
#define DC_FCM_MASK	    0x00000060
#define DC_FCM_SHIFT    5
#define DC_NAE          0x00000080
#define DC_TF           0x00000100
#define DC_RDS_MASK     0x00030000
#define DC_RDS_SHIFT    16
#define DC_TDS_MASK     0x000c0000
#define DC_TDS_SHIFT    18

/* device status */
#define DS_RBF       0x00000001
#define DS_RDF       0x00000002
#define DS_RIF       0x00000004
#define DS_TBF       0x00000008
#define DS_TDF       0x00000010
#define DS_TIF       0x00000020
#define DS_PO        0x00000040
#define DS_MM_MASK   0x00000300
#define DS_MM_SHIFT  8

/* bist status */
#define BS_MTF   0x00000001
#define BS_MRF   0x00000002
#define BS_TDB   0x00000004
#define BS_TIB   0x00000008
#define BS_TBF   0x00000010
#define BS_RDB   0x00000020
#define BS_RIB   0x00000040
#define BS_RBF   0x00000080
#define BS_URTF  0x00000100
#define BS_UTF   0x00000200
#define BS_URF   0x00000400

/* interrupt status and mask registers */
#define I_MRO      0x00000001
#define I_MTO      0x00000002
#define I_TFD      0x00000004
#define I_LS       0x00000008
#define I_MDIO     0x00000010
#define I_MR       0x00000020
#define I_MT       0x00000040
#define I_TO       0x00000080
#define I_PDEE     0x00000400
#define I_PDE      0x00000800
#define I_DE       0x00001000
#define I_RDU      0x00002000
#define I_RFO      0x00004000
#define I_XFU      0x00008000
#define I_RI       0x00010000
#define I_XI0      0x01000000
#define I_XI1      0x02000000
#define I_XI2      0x04000000
#define I_XI3      0x08000000
#define I_INTMASK  0x0f01fcff
#define I_ERRMASK  0x0000fc00
#define I_XI_ALL   (I_XI0 | I_XI1 | I_XI2 | I_XI3)
#define I_ERRORS_ALL  (I_PDEE | I_PDE | I_DE | I_RDU | I_RFO | I_XFU)

/* CheckSum control fields */
#define CKSUM_CONTROL_HW_PSHDR_EN BIT(28)
#define CKSUM_CONTROL_ACC_SS_MODE BIT(27)
#define CKSUM_CONTROL_BRCM_HDR_EN BIT(26)
#define CKSUM_CONTROL_SHIM_HDR_EN BIT(25)
#define CKSUM_CONTROL_TSO_ENABLE  BIT(24)

#define CKSUM_CONTROL_LRO_HDR_LEN BIT(6)
#define CKSUM_CONTROL_LRO_HDR_LEN_WIDTH BIT(8)

#define CKSUM_CONTROL_LRO_META_LEN BIT(8)
#define CKSUM_CONTROL_LRO_META_LEN_WIDTH BIT(18)

#define CKSUM_CONTROL_LRO_ENABLED BIT(7)
#define CKSUM_CONTROL_RMO_ENABLED BIT(6)
#define CKSUM_CONTROL_CTF_BYPASS BIT(5)
#define CKSUM_CONTROL_RXHDRCKSUMEN BIT(4)
#define CKSUM_CONTROL_TXHDRCKSUMEN BIT(3)
#define CKSUM_CONTROL_RXCKSUMMODE BIT(2)
#define CKSUM_CONTROL_RXCKSUMEN BIT(1)
#define CKSUM_CONTROL_TXCKSUMEN BIT(0)

/* interrupt receive lazy */
#define IRL_TO_MASK  0x00ffffff
#define IRL_FC_MASK  0xff000000
#define IRL_FC_SHIFT  24

/* flow control thresholds */
#define FCT_TT_MASK  0x00000fff
#define FCT_RT_MASK  0x0fff0000
#define FCT_RT_SHIFT 16

/* txq aribter wrr thresholds */
#define WRRT_Q0T_MASK  0x000000ff
#define WRRT_Q1T_MASK  0x0000ff00
#define WRRT_Q1T_SHIFT 8
#define WRRT_Q2T_MASK  0x00ff0000
#define WRRT_Q2T_SHIFT 16
#define WRRT_Q3T_MASK  0xff000000
#define WRRT_Q3T_SHIFT 24

/* phy access */
#define PA_DATA_MASK  0x0000ffff
#define PA_ADDR_MASK  0x001f0000
#define PA_ADDR_SHIFT 16
#define PA_REG_MASK	  0x1f000000
#define PA_REG_SHIFT  24
#define PA_WRITE      0x20000000
#define PA_START      0x40000000

/* phy control */
#define PC_EPA_MASK   0x0000001f
#define PC_MCT_MASK   0x007f0000
#define PC_MCT_SHIFT  16
#define PC_MTE        0x00800000

/* rxq control */
#define RC_DBT_MASK   0x00000fff
#define RC_DBT_SHIFT  0
#define RC_PTE        0x00001000
#define RC_MDP_MASK   0x3f000000
#define RC_MDP_SHIFT  24

#define RC_MAC_DATA_PERIOD 9

/* txq control */
#define TC_DBT_MASK  0x00000fff
#define TC_DBT_SHIFT 0

/* clk control status */
#define CS_FA  0x00000001
#define CS_FH  0x00000002
#define CS_FI  0x00000004
#define CS_AQ  0x00000008
#define CS_HQ  0x00000010
#define CS_FC  0x00000020
#define CS_ER  0x00000100
#define CS_AA  0x00010000
#define CS_HA  0x00020000
#define CS_BA  0x00040000
#define CS_BH  0x00080000
#define CS_ES  0x01000000

/* Unimac Registers */
#define CMD_CFG_ML_PEG (0x18100 | 0x8100)
#define UNIMAC_TAG1_VAL (0x18100 | 0x9100)
#define UNIMAC_FRMAE_LEN_VAL (0x05ee | 0x34BC)
#define UNIMAC_IPG_HD_BKP_CTRL_VAL (0xC)

#define UNIMAC_TAG1_REG (0x84c)
#define UNIMAC_IPG_HD_BKP_CTRL_REG (0x804)
#define UNIMAC_FRMAE_LEN_REG (0x814)

/* Unimac  command config */
#define CMD_CFG_DEFAULT (0x010000db)
#define CC_TE        0x00000001
#define CC_RE        0x00000002
#define CC_ES_MASK   0x0000000c
#define CC_ES_SHIFT  2
#define CC_PROM      0x00000010
#define CC_PAD_EN    0x00000020
#define CC_CF        0x00000040
#define CC_PF        0x00000080
#define CC_RPI       0x00000100
#define CC_TAI       0x00000200
#define CC_HD        0x00000400
#define CC_HD_SHIFT  10
#define CC_SR        0x00002000
#define CC_ML        0x00008000
#define CC_AE        0x00400000
#define CC_CFE       0x00800000
#define CC_NLC       0x01000000
#define CC_RL        0x02000000
#define CC_RED       0x04000000
#define CC_PE        0x08000000
#define CC_TPI       0x10000000

#define CC_RFD       BIT(30)
#define CC_LC        BIT(16)
#define CC_OE        BIT(12)

/* transmit channel control */
/* DMA specific bits */
#define XC_XE  ((unsigned int)1 << 0) /* transmit enable */
#define XC_SE  ((unsigned int)1 << 1) /* transmit suspend request */
#define XC_LE  ((unsigned int)1 << 2) /* loopback enable */
#define XC_FL  ((unsigned int)1 << 4) /* flush request */
#define XC_MR_MASK  0x000000C0 /* Multiple outstanding reads */
#define XC_MR_SHIFT 6
#define XC_PD       ((unsigned int)1 << 11) /* parity check disable */
#define XC_AE       ((unsigned int)3 << 16) /* address extension bits */
#define XC_AE_SHIFT 16
#define XC_BL_MASK  0x001C0000 /* BurstLen bits */
#define XC_BL_SHIFT 18
#define XC_PC_MASK  0x00E00000 /* Prefetch control */
#define XC_PC_SHIFT 21
#define XC_PT_MASK  0x03000000 /* Prefetch threshold */
#define XC_PT_SHIFT 24

/* transmit descriptor table pointer */
#define XP_LD_MASK 0xfff /* last valid descriptor */

/* transmit channel status */
#define XS_CD_MASK      0x0fff /* current descriptor pointer */
#define XS_XS_MASK      0xf000 /* transmit state */
#define XS_XS_SHIFT     12
#define XS_XS_DISABLED  0x0000 /* disabled */
#define XS_XS_ACTIVE    0x1000 /* active */
#define XS_XS_IDLE      0x2000 /* idle wait */
#define XS_XS_STOPPED   0x3000 /* stopped */
#define XS_XS_SUSP      0x4000 /* suspend pending */
#define XS_XE_MASK      0xf0000 /* transmit errors */
#define XS_XE_SHIFT     16
#define XS_XE_NOERR     0x00000 /* no error */
#define XS_XE_DPE       0x10000 /* descriptor protocol error */
#define XS_XE_DFU       0x20000 /* data fifo underrun */
#define XS_XE_BEBR      0x30000 /* bus error on buffer read */
#define XS_XE_BEDA      0x40000 /* bus error on descriptor access */
#define XS_AD_MASK      0xfff00000 /* active descriptor */
#define XS_AD_SHIFT     20

#define D64_PTR_MASK 0x1FFF
/* transmit channel control */
#define D64_XC_XE       0x00000001 /* transmit enable */
#define D64_XC_SE       0x00000002 /* transmit suspend request */
#define D64_XC_LE       0x00000004 /* loopback enable */
#define D64_XC_FL       0x00000010 /* flush request */
#define D64_XC_MR_MASK  0x000000C0 /* Multiple outstanding reads */
#define D64_XC_MR_SHIFT 6
#define D64_XC_PD       0x00000800 /* parity check disable */
#define D64_XC_AE       0x00030000 /* address extension bits */
#define D64_XC_AE_SHIFT 16
#define D64_XC_BL_MASK  0x001C0000 /* BurstLen bits */
#define D64_XC_BL_SHIFT 18
#define D64_XC_PC_MASK  0x00E00000 /* Prefetch control */
#define D64_XC_PC_SHIFT 21
#define D64_XC_PT_MASK  0x03000000 /* Prefetch threshold */
#define D64_XC_PT_SHIFT 24

/* transmit descriptor table pointer */
#define D64_XP_LD_MASK  0x00001fff /* last valid descriptor */

/* transmit channel status */
#define D64_XS0_CD_MASK     0x00001fff /* current descriptor pointer */
#define D64_XS0_XS_MASK     0xf0000000 /* transmit state */
#define D64_XS0_XS_SHIFT    28
#define D64_XS0_XS_DISABLED 0x00000000 /* disabled */
#define D64_XS0_XS_ACTIVE   0x10000000 /* active */
#define D64_XS0_XS_IDLE     0x20000000 /* idle wait */
#define D64_XS0_XS_STOPPED  0x30000000 /* stopped */
#define D64_XS0_XS_SUSP     0x40000000 /* suspend pending */

#define D64_XS1_AD_MASK     0x00001fff /* active descriptor */
#define D64_XS1_XE_MASK	    0xf0000000 /* transmit errors */
#define D64_XS1_XE_SHIFT    28
#define D64_XS1_XE_NOERR    0x00000000 /* no error */
#define D64_XS1_XE_DPE      0x10000000 /* descriptor protocol error */
#define D64_XS1_XE_DFU      0x20000000 /* data fifo underrun */
#define D64_XS1_XE_DTE      0x30000000 /* data transfer error */
#define D64_XS1_XE_DESRE    0x40000000 /* descriptor read error */
#define D64_XS1_XE_COREE    0x50000000 /* core error */
#define D64_XS1_XE_DE       0x1
#define D64_XS1_XE_XFU	    0x2
#define D64_XS1_XE_PDE	    0x3
#define D64_XS1_XE_PDEE	    0x4

/* receive channel control */
#define D64_RC_RE       0x00000001 /* receive enable */
#define D64_RC_RO_MASK  0x000000fe /* receive frame offset */
#define D64_RC_RO_SHIFT 1
#define D64_RC_FM 0x00000100 /* direct fifo receive (pio) mode */
#define D64_RC_SH 0x00000200 /* separate rx header descriptor enable */
#define D64_RC_OC 0x00000400 /* overflow continue */
#define D64_RC_PD 0x00000800 /* parity check disable */
#define D64_RC_GE 0x00004000 /* Glom enable */
#define D64_RC_AE 0x00030000 /* address extension bits */
#define D64_RC_AE_SHIFT 16
#define D64_RC_BL_MASK  0x001C0000 /* BurstLen bits */
#define D64_RC_BL_SHIFT 18
#define D64_RC_PC_16_DESC 0x3
#define D64_RC_PC_8_DESC 0x2
#define D64_RC_PC_4_DESC 0x1
#define D64_RC_PC_MASK  0x00E00000 /* Prefetch control */
#define D64_RC_PC_SHIFT 21
#define D64_RC_PT_MASK  0x03000000 /* Prefetch threshold */
#define D64_RC_PT_SHIFT 24

/* flags for dma controller */
#define DMA_CTRL_RX_PEN BIT(0) /* rx partity enable */
#define DMA_CTRL_TX_PEN BIT(1) /* tx partity enable */
#define DMA_CTRL_RX_ROC BIT(2) /* rx overflow continue */
#define DMA_CTRL_RX_PC  BIT(3) /* RX Prefetch Control */
#define DMA_CTRL_TX_PC  BIT(4) /* TX Prefetch Control */

/* receive channel status */
#define D64_RS0_CD_MASK  0x00001fff /* current descriptor pointer */
#define D64_RS0_RS_MASK  0xf0000000 /* receive state */
#define D64_RS0_RS_SHIFT    28
#define D64_RS0_RS_DISABLED 0x00000000 /* disabled */
#define D64_RS0_RS_ACTIVE   0x10000000 /* active */
#define D64_RS0_RS_IDLE     0x20000000 /* idle wait */
#define D64_RS0_RS_STOPPED  0x30000000 /* stopped */
#define D64_RS0_RS_SUSP     0x40000000 /* suspend pending */

#define D64_RS1_AD_MASK   0x0001ffff /* active descriptor */
#define D64_RS1_RE_MASK   0xf0000000 /* receive errors */
#define D64_RS1_RE_SHIFT  28
#define D64_RS1_RE_NOERR  0x00000000 /* no error */
#define D64_RS1_RE_DPO    0x10000000 /* descriptor protocol error */
#define D64_RS1_RE_DFU    0x20000000 /* data fifo overflow */
#define D64_RS1_RE_DTE    0x30000000 /* data transfer error */
#define D64_RS1_RE_DESRE  0x40000000 /* descriptor read error */
#define D64_RS1_RE_COREE  0x50000000 /* core error */
#define D64_RS1_RE_DE     0x1
#define D64_RS1_RE_RFO    0x2
#define D64_RS1_RE_PDE    0x3
#define D64_RS1_RE_PDEE   0x4

/* descriptor control flags 1 */
#define D64_CTRL_COREFLAGS 0x0ff00000 /* core specific */
#define D64_CTRL1_EOT ((unsigned int)BIT(28)) /* end of descriptor table */
#define D64_CTRL1_IOC ((unsigned int)BIT(29)) /* interrupt on completion */
#define D64_CTRL1_EOF ((unsigned int)BIT(30)) /* end of frame */
#define D64_CTRL1_SOF ((unsigned int)BIT(31)) /* start of frame */

/* descriptor control flags 2 */
#define D64_CTRL2_BC_MASK  0x00007fff /* buff byte cnt.real data len <= 16KB */
#define D64_CTRL2_AE       0x00030000 /* address extension bits */
#define D64_CTRL2_AE_SHIFT 16
#define D64_CTRL2_PARITY   0x00040000      /* parity bit */

/* control flags in the range [27:20] are core-specific and not defined here */
#define D64_CTRL_CORE_MASK  0x0ff00000

#define D64_RX_FRM_STS_LEN  0x0000ffff /* frame length mask */
#define D64_RX_FRM_STS_OVFL 0x00800000 /* RxOverFlow */

/* no. of descp used - 1, d11corerev >= 22 */
#define D64_RX_FRM_STS_DSCRCNT 0x0f000000

#define D64_RX_FRM_STS_DATATYPE 0xf0000000 /* core-dependent data type */

#define UNIMAC_CMD_CONFIG_PEG	0x6401b

#ifdef ENABLE_DMA_LOOPBACK
#define HWRXOFF    16
#else
#define HWRXOFF    64 /* Aligned val:64, metadata size:28 */
#endif

#define UNIMAC_CMD_CONFIG_PEG	0x6401b

#define MIB_REG_BASE 0x300
#define MIB_REG_BASE_PEG 0x500
#define MIB_TX_GD_OCTETS_LO 0x000
#define MIB_TX_GD_OCTETS_HI 0x004
#define MIB_TX_GD_PKTS 0x008
#define MIB_TX_ALL_OCTETS_LO 0x00c
#define MIB_TX_ALL_OCTETS_HI 0x010
#define MIB_TX_ALL_PKTS 0x014
#define MIB_TX_BRDCAST 0x018
#define MIB_TX_MULT 0x01c
#define MIB_TX_64 0x020
#define MIB_TX_65_127 0x024
#define MIB_TX_128_255 0x028
#define MIB_TX_256_511 0x02c
#define MIB_TX_512_1023 0x030
#define MIB_TX_1024_1522 0x034
#define MIB_TX_1523_2047 0x038
#define MIB_TX_2048_4095 0x03c
#define MIB_TX_4096_8191 0x040
#define MIB_TX_8192_MAX 0x044
#define MIB_TX_JAB 0x048
#define MIB_TX_OVER 0x04c
#define MIB_TX_FRAG 0x050
#define MIB_TX_UNDERRUN 0x054
#define MIB_TX_COL 0x058
#define MIB_TX_1_COL 0x05c
#define MIB_TX_M_COL 0x060
#define MIB_TX_EX_COL 0x064
#define MIB_TX_LATE 0x068
#define MIB_TX_DEF 0x06c
#define MIB_TX_CRS 0x070
#define MIB_TX_PAUS 0x074
#define MIB_TXUNICASTPKT 0x078
#define MIB_TXQOSQ0PKT 0x07c
#define MIB_TXQOSQ0OCTET_LO 0x080
#define MIB_TXQOSQ0OCTET_HI 0x084
#define MIB_TXQOSQ1PKT 0x088
#define MIB_TXQOSQ1OCTET_LO 0x08c
#define MIB_TXQOSQ1OCTET_HI 0x090
#define MIB_TXQOSQ2PKT 0x094
#define MIB_TXQOSQ2OCTET_LO 0x098
#define MIB_TXQOSQ2OCTET_HI 0x09c
#define MIB_TXQOSQ3PKT 0x0a0
#define MIB_TXQOSQ3OCTET_LO 0x0a4
#define MIB_TXQOSQ3OCTET_HI 0x0a8
#define MIB_RX_GD_OCTETS_LO 0x0b0
#define MIB_RX_GD_OCTETS_HI 0x0b4
#define MIB_RX_GD_PKTS 0x0b8
#define MIB_RX_ALL_OCTETS_LO 0x0bc
#define MIB_RX_ALL_OCTETS_HI 0x0c0
#define MIB_RX_ALL_PKTS 0x0c4
#define MIB_RX_BRDCAST 0x0c8
#define MIB_RX_MULT 0x0cc
#define MIB_RX_64 0x0d0
#define MIB_RX_65_127 0x0d4
#define MIB_RX_128_255 0x0d8
#define MIB_RX_256_511 0x0dc
#define MIB_RX_512_1023 0x0e0
#define MIB_RX_1024_1522 0x0e4
#define MIB_RX_1523_2047 0x0e8
#define MIB_RX_2048_4095 0x0ec
#define MIB_RX_4096_8191 0x0f0
#define MIB_RX_8192_MAX 0x0f4
#define MIB_RX_JAB 0x0f8
#define MIB_RX_OVR 0x0fc
#define MIB_RX_FRAG 0x100
#define MIB_RX_DROP 0x104
#define MIB_RX_CRC_ALIGN 0x108
#define MIB_RX_UND 0x10c
#define MIB_RX_CRC 0x110
#define MIB_RX_ALIGN 0x114
#define MIB_RX_SYM 0x118
#define MIB_RX_PAUS 0x11c
#define MIB_RX_CNTRL 0x120
#define MIB_RXSACHANGES 0x124
#define MIB_RXUNICASTPKTS 0x128

#define MIB_COUNTER(privp, reg)  (privp->hw.reg.amac_core + (reg))
#define TOTAL_MIB_COUNTERS (74)
#define MIB_COUNTER_VAL(privp, reg)  readl(MIB_COUNTER(privp, reg))

/* MiB page registers */
#define REG_TX_OCTETS                 0x00
#define REG_TX_DROP_PKTS              0x08
#define REG_TX_BROADCAST_PKTS         0x10
#define REG_TX_MULTICAST_PKTS         0x14
#define REG_TX_UNICAST_PKTS           0x18
#define REG_TX_COLLISIONS             0x1c
#define REG_TX_SINGLE_COLLISION       0x20
#define REG_TX_MULTIPLE_COLLISION     0x24
#define REG_TX_DEFERRED_TXMIT         0x28
#define REG_TX_LATE_COLLISION         0x2c
#define REG_TX_EXCESSIVE_COLLISION    0x30
#define REG_TX_FRAME_IN_DISC          0x34
#define REG_TX_PAUSE_PKTS             0x38
#define REG_RX_OCTETS                 0x50
#define REG_RX_UNDERSIZE_PKTS         0x58
#define REG_RX_PAUSE_PKTS             0x5c
#define REG_RX_PKTS64OCTETS           0x60
#define REG_RX_PKTS65TO127OCTETS      0x64
#define REG_RX_PKTS128TO255OCTETS     0x68
#define REG_RX_PKTS256TO511OCTETS     0x6c
#define REG_RX_PKTS512TO1023OCTETS    0x70
#define REG_RX_PKTS1024TOMAXPKTOCTETS 0x74
#define REG_RX_OVERSIZE_PKTS          0x78
#define REG_RX_JABBERS                0x7c
#define REG_RX_ALIGNMENT_ERRORS       0x80
#define REG_RX_FCS_ERRORS             0x84
#define REG_RX_GOOD_OCTETS            0x88
#define REG_RX_DROP_PKTS              0x90
#define REG_RX_UNICAST_PKTS           0x94
#define REG_RX_MULTICAST_PKTS         0x98
#define REG_RX_BROADCAST_PKTS         0x9c
#define REG_RX_SA_CHANGES             0xa0
#define REG_RX_FRAGMENTS              0xa4
#define REG_RX_JUMBO_PKTCOUNT         0xa8
#define REG_RX_SYMBOL_ERROR           0xac
#define REG_RX_DISCARD                0xc0

/* PortMacro MIB counters */
#define MAC_GRX64 0x0
#define MAC_GRX127 0x1
#define MAC_GRX255 0x2
#define MAC_GRX511 0x3
#define MAC_GRX1023 0x4
#define MAC_GRX1518 0x5
#define MAC_GRX1522 0x6
#define MAC_GRX2047 0x7
#define MAC_GRX4095 0x8
#define MAC_GRX9216 0x9
#define MAC_GRX16383 0xA
#define MAC_GRXPKT 0xB
#define MAC_GRXUCA 0x00C	/* PER PORT */
#define MAC_GRXMCA 0xD
#define MAC_GRXBCA 0xE
#define MAC_GRXFCS 0xF
#define MAC_GRXCF 0x10
#define MAC_GRXPF 0x11
#define MAC_GRXPP 0x12
#define MAC_GRXUO 0x13
#define MAC_GRXUDA 0x14
#define MAC_GRXWSA 0x15
#define MAC_GRXALN 0x16
#define MAC_GRXFLR 0x17
#define MAC_GRXFRERR 0x18
#define MAC_GRXFCR 0x19
#define MAC_GRXOVR 0x1A
#define MAC_GRXJBR 0x1B
#define MAC_GRXMTUE 0x1C
#define MAC_GRXMCRC 0x1D
#define MAC_GRXPRM 0x1E
#define MAC_GRXVLN 0x1F
#define MAC_GRXDVLN 0x20
#define MAC_GRXTRFU 0x21
#define MAC_GRXPOK 0x22
#define MAC_GRXPFCOFF0 0x23
#define MAC_GRXPFCOFF1 0x24
#define MAC_GRXPFCOFF2 0x25
#define MAC_GRXPFCOFF3 0x26
#define MAC_GRXPFCOFF4 0x27
#define MAC_GRXPFCOFF5 0x28
#define MAC_GRXPFCOFF6 0x29
#define MAC_GRXPFCOFF7 0x2A
#define MAC_GRXPFCP0 0x2B
#define MAC_GRXPFCP1 0x2C
#define MAC_GRXPFCP2 0x2D
#define MAC_GRXPFCP3 0x2E
#define MAC_GRXPFCP4 0x2F
#define MAC_GRXPFCP5 0x30
#define MAC_GRXPFCP6 0x31
#define MAC_GRXPFCP7 0x32
#define MAC_GRXSCHCRC 0x33
#define MAC_GRXUND 0x34
#define MAC_GRXFRG 0x35
#define RX_EEE_LPI_EVENT_COUNTER 0x36
#define RX_EEE_LPI_DURATION_COUNTER 0x37
#define RX_LLFC_PHY_COUNTER 0x38
#define RX_LLFC_LOG_COUNTER 0x39
#define RX_LLFC_CRC_COUNTER 0x3A
#define RX_HCFC_COUNTER 0x3B
#define RX_HCFC_CRC_COUNTER 0x3C
#define MAC_GRXBYT 0x3D
#define MAC_GRXRBYT 0x3E
#define MAC_GRXRPKT 0x3F
#define MAC_GTX64 0x40
#define MAC_GTX127 0x41
#define MAC_GTX255 0x42
#define MAC_GTX511 0x43
#define MAC_GTX1023 0x44
#define MAC_GTX1518 0x45
#define MAC_GTX1522 0x46
#define MAC_GTX2047 0x47
#define MAC_GTX4095 0x48
#define MAC_GTX9216 0x49
#define MAC_GTX16383 0x4A
#define MAC_GTXPOK 0x4B
#define MAC_GTXPKT 0x4C
#define MAC_GTXUCA 0x4D
#define MAC_GTXMCA 0x4E
#define MAC_GTXBCA 0x4F
#define MAC_GTXPF 0x50
#define MAC_GTXPP 0x51
#define MAC_GTXJBR 0x52
#define MAC_GTXFCS 0x53
#define MAC_GTXCF 0x54
#define MAC_GTXOVR 0x55
#define MAC_GTXDFR 0x56
#define MAC_GTXEDF 0x57
#define MAC_GTXSCL 0x58
#define MAC_GTXMCL 0x59
#define MAC_GTXLCL 0x5A
#define MAC_GTXXCL 0x5B
#define MAC_GTXFRG 0x5C
#define MAC_GTXERR 0x5D
#define MAC_GTXVLN 0x5E
#define MAC_GTXDVLN 0x5F
#define MAC_GTXRPKT 0x60
#define MAC_GTXUFL 0x61
#define MAC_GTXPFCP0 0x62
#define MAC_GTXPFCP1 0x63
#define MAC_GTXPFCP2 0x64
#define MAC_GTXPFCP3 0x65
#define MAC_GTXPFCP4 0x66
#define MAC_GTXPFCP5 0x67
#define MAC_GTXPFCP6 0x68
#define MAC_GTXPFCP7 0x69
#define TX_EEE_LPI_EVENT_COUNTER 0x6A
#define TX_EEE_LPI_DURATION_COUNTER 0x6B
#define TX_LLFC_LOG_COUNTER 0x6C
#define TX_HCFC_COUNTER 0x6D
#define MAC_GTXNCL 0x6E
#define MAC_GTXBYT 0x6F
#define XTHOL 0x70

#define DBG_PRN pr_emerg("[%s:%s():%d]\n", __FILE__, __func__,  __LINE__)

#endif /*__BCM_AMAC_REGS_H__ */
