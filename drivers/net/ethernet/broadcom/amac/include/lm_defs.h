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

#ifndef _LM_DEFS_H
#define _LM_DEFS_H

/* #include "bcmtype.h" */

/*****************************************************************
	* Simple constants.
	**************************************************************/

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif

#ifndef NULL
#define NULL    ((void *)0)
#endif

/* Signatures for integrity checks. */
#define LM_DEVICE_SIG           0x6d635242	/* BRcm */
#define L2PACKET_RX_SIG         0x7872324c	/* L2rx */
#define L2PACKET_TX_SIG         0x7874324c	/* L2tx */
#define L4BUFFER_RX_SIG         0x7872344c	/* L4rx */
#define L4BUFFER_TX_SIG         0x7874344c	/* L4tx */
#define L4BUFFER_SIG            0x66754254	/* TBuf */
#define L4GEN_BUFFER_SIG        0x006e6567	/* gen  */
#define L4GEN_BUFFER_SIG_END    0x0067656e	/* neg  */

#define SIZEOF_SIG              16
#define SIG(_p)                 (*((u32 *)((u8 *)(_p) - sizeof(u32))))
#define END_SIG(_p, _size)      (*((u32 *)((u8 *)(_p) + (_size))))

/* This macro rounds the given value to the next word boundary if it
	* is not already at a word boundary.
	* */
#define ALIGN_VALUE_TO_WORD_BOUNDARY(_v) \
	(((_v) + (sizeof(void *) - 1)) & ~(sizeof(void *) - 1))

/* This macro determines the delta to the next alignment which is
	* either 1, 2, 4, 8, 16, 32, etc.
	* */
#define ALIGN_DELTA_TO_BOUNDARY(_p, _a) \
	(((((u8 *)(_p) - (u8 *)0) + ((_a) - 1)) & ~((_a) - 1)) - \
	      ((u8 *)(_p) - (u8 *)0))

/* This macro returns the pointer to the next alignment if the pointer
	* is not currently on the indicated alignment boundary.
	* */
#define ALIGN_PTR_TO_BOUNDARY(_p, _a) \
	((u8 *)(_p) + ALIGN_DELTA_TO_BOUNDARY(_p, _a))

/******************************************************************
	* Status codes.
	***************************************************************/

typedef enum {
	LM_STATUS_SUCCESS = 0,
	LM_STATUS_LINK_UNKNOWN = 0,
	LM_STATUS_FAILURE = 1,
	LM_STATUS_RESOURCE = 2,
	LM_STATUS_ABORTED = 3,
	LM_STATUS_PENDING = 4,
	LM_STATUS_PAUSED = 5,
	LM_STATUS_INVALID_PARAMETER = 6,
	LM_STATUS_LINK_ACTIVE = 7,
	LM_STATUS_LINK_DOWN = 8,
	LM_STATUS_UNKNOWN_ADAPTER = 9,
	LM_STATUS_UNKNOWN_PHY = 10,
	LM_STATUS_UNKNOWN_MEDIUM = 11,
	LM_STATUS_TOO_MANY_FRAGMENTS = 12,
	LM_STATUS_BUFFER_TOO_SHORT = 16,
	LM_STATUS_UPLOAD_IN_PROGRESS = 17,
	LM_STATUS_BUSY = 18,
	LM_STATUS_INVALID_KEY = 19,
	LM_STATUS_TIMEOUT = 20,
	LM_STATUS_REQUEST_NOT_ACCEPTED = 21,
	LM_STATUS_CONNECTION_CLOSED = 22,
	LM_STATUS_BAD_SIGNATURE = 23,
	LM_STATUS_CONNECTION_RESET = 24,
	LM_STATUS_EXISTING_OBJECT = 25,
	LM_STATUS_OBJECT_NOT_FOUND = 26,
	LM_STATUS_CONNECTION_RM_DISC = 27,
	LM_STATUS_UNKNOWN_EVENT_CODE = 28
} lm_status_t;

/******************************************************************
	* Receive filter masks.
	***************************************************************/

typedef u32 lm_rx_mask_t;

#define LM_RX_MASK_ACCEPT_NONE                  0x0000
#define LM_RX_MASK_ACCEPT_UNICAST               0x0001
#define LM_RX_MASK_ACCEPT_MULTICAST             0x0002
#define LM_RX_MASK_ACCEPT_ALL_MULTICAST         0x0004
#define LM_RX_MASK_ACCEPT_BROADCAST             0x0008
#define LM_RX_MASK_ACCEPT_ERROR_PACKET          0x0010

#define LM_RX_MASK_PROMISCUOUS_MODE             0x10000

/********************************************************************
	* Flow control.
	*****************************************************************/

typedef u32 lm_flow_control_t;

#define LM_FLOW_CONTROL_NONE                    0x00
#define LM_FLOW_CONTROL_RECEIVE_PAUSE           0x01
#define LM_FLOW_CONTROL_TRANSMIT_PAUSE          0x02

/* This value can be or-ed with RECEIVE_PAUSE and TRANSMIT_PAUSE.  If the
	* auto-negotiation is disabled and the RECEIVE_PAUSE and TRANSMIT_PAUSE
	* bits are set, then flow control is enabled regardless of link partner's
	* flow control capability.  Otherwise, if this bit is set, then flow
	* is negotiated with the link partner.  Values 0x80000000 and 0x80000003 are
	* equivalent. */
#define LM_FLOW_CONTROL_AUTO_PAUSE              0x80000000

/********************************************************************
	* EEE control.
	*****************************************************************/

/* values match registry values of EeeCtrlMode.
 * Default is MED("Balanced")
 * */
typedef enum {
	LM_EEE_CONTROL_HIGH = 0,	/* MaxPowerSave */
	LM_EEE_CONTROL_MED = 1,	/* Balance */
	LM_EEE_CONTROL_LOW = 2,	/* MaxPreformance */
	LM_EEE_CONTROL_NVRAM = 3,	/* use NVRAM */
	LM_EEE_CONTROL_NA = 4	/* either N/A or disabled */
} lm_eee_policy_t;

/*********************************************************************
	* media type.
	******************************************************************/

typedef u32 lm_medium_t;

#define LM_MEDIUM_AUTO_DETECT                   0x0000

#define LM_MEDIUM_TYPE_UNKNOWN                  0x0000
#define LM_MEDIUM_TYPE_BNC                      0x0001
#define LM_MEDIUM_TYPE_UTP                      0x0002
#define LM_MEDIUM_TYPE_FIBER                    0x0003
#define LM_MEDIUM_TYPE_SERDES                   0x0004
#define LM_MEDIUM_TYPE_SERDES_SGMII             0x0005
#define LM_MEDIUM_TYPE_XGXS                     0x0006
#define LM_MEDIUM_TYPE_XGXS_SGMII               0x0007
#define LM_MEDIUM_TYPE_XMAC_LOOPBACK            0x0008
#define LM_MEDIUM_TYPE_UMAC_LOOPBACK            0x0009
#define LM_MEDIUM_TYPE_PA_LOOPBACK              0x000a
#define LM_MEDIUM_TYPE_EXT_LOOPBACK             0x00f6
#define LM_MEDIUM_TYPE_EXT_PHY_LOOPBACK         0x00f7
#define LM_MEDIUM_TYPE_SERDES_LOOPBACK          0x00f8
#define LM_MEDIUM_TYPE_XGXS_LOOPBACK            0x00f9
#define LM_MEDIUM_TYPE_XGXS_10_LOOPBACK         0x00fa
#define LM_MEDIUM_TYPE_BMAC_LOOPBACK            0x00fb
#define LM_MEDIUM_TYPE_EMAC_LOOPBACK            0x00fc
#define LM_MEDIUM_TYPE_PHY_LOOPBACK             0x00fd
#define LM_MEDIUM_TYPE_MAC_LOOPBACK             0x00fe
#define LM_MEDIUM_TYPE_NULL                     0x00ff
#define LM_MEDIUM_TYPE_MASK                     0x00ff
#define GET_MEDIUM_TYPE(m)                      ((m) & LM_MEDIUM_TYPE_MASK)
#define SET_MEDIUM_TYPE(m, t) \
	((m) = ((m) & ~(LM_MEDIUM_TYPE_MASK)) | (t))

#define LM_MEDIUM_IS_LOOPBACK(_medium) \
	(((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_BMAC_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_UMAC_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_XMAC_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_EXT_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_EXT_PHY_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_SERDES_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_XGXS_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_XGXS_10_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_PHY_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_PA_LOOPBACK) || \
	((GET_MEDIUM_TYPE(_medium)) == LM_MEDIUM_TYPE_MAC_LOOPBACK))

#define LM_MEDIUM_SPEED_AUTONEG                 0x0000

#define LM_MEDIUM_SPEED_UNKNOWN                 0x0000
#define LM_MEDIUM_SPEED_10MBPS                  0x0100
#define LM_MEDIUM_SPEED_100MBPS                 0x0200
#define LM_MEDIUM_SPEED_1000MBPS                0x0300
#define LM_MEDIUM_SPEED_2500MBPS                0x0400
#define LM_MEDIUM_SPEED_10GBPS                  0x0600
#define LM_MEDIUM_SPEED_12GBPS                  0x0700
#define LM_MEDIUM_SPEED_12_5GBPS                0x0800
#define LM_MEDIUM_SPEED_13GBPS                  0x0900
#define LM_MEDIUM_SPEED_15GBPS                  0x0a00
#define LM_MEDIUM_SPEED_16GBPS                  0x0b00
#define LM_MEDIUM_SPEED_20GBPS                  0x0c00
#define LM_MEDIUM_SPEED_25GBPS                  0x0d00
#define LM_MEDIUM_SPEED_40GBPS                  0x0e00
#define LM_MEDIUM_SPEED_50GBPS                  0x0f00
#define LM_MEDIUM_SPEED_SEQ_START               0x1d00	/* 100Mbps */
#define LM_MEDIUM_SPEED_SEQ_END                 0xE400	/* 20Gbps */
#define LM_MEDIUM_SPEED_AUTONEG_1G_FALLBACK     0xFD00	/* Serdes */
#define LM_MEDIUM_SPEED_AUTONEG_2_5G_FALLBACK   0xFE00	/* Serdes */
#define LM_MEDIUM_SPEED_HARDWARE_DEFAULT        0xff00	/* Serdes nvram def. */
#define LM_MEDIUM_SPEED_MASK                    0xff00
#define GET_MEDIUM_SPEED(m)                     ((m) & LM_MEDIUM_SPEED_MASK)
#define SET_MEDIUM_SPEED(m, s) \
	((m) = ((m) & ~LM_MEDIUM_SPEED_MASK) | (s))

#define LM_MEDIUM_FULL_DUPLEX                   0x00000
#define LM_MEDIUM_HALF_DUPLEX                   0x10000
#define GET_MEDIUM_DUPLEX(m)                    ((m) & LM_MEDIUM_HALF_DUPLEX)
#define SET_MEDIUM_DUPLEX(m, d) \
	((m) = ((m) & ~LM_MEDIUM_HALF_DUPLEX) | (d))

#define LM_MEDIUM_SELECTIVE_AUTONEG             0x01000000
#define GET_MEDIUM_AUTONEG_MODE(m)              ((m) & 0xff000000)

typedef struct _lm_link_settings_t {
	u32 flag;
#define LINK_FLAG_SELECTIVE_AUTONEG_MASK                    0x0f
#define LINK_FLAG_SELECTIVE_AUTONEG_ONE_SPEED               0x01
#define LINK_FLAG_SELECTIVE_AUTONEG_ENABLE_SLOWER_SPEEDS    0x02
#define LINK_FLAG_WIRE_SPEED                                0x10

	lm_medium_t req_medium;
	lm_flow_control_t flow_ctrl;

	u32 _reserved;
} lm_link_settings_t;

/********************************************************************
	* Power state.
	*****************************************************************/

typedef enum {
	LM_POWER_STATE_D0 = 0,
	LM_POWER_STATE_D1 = 1,
	LM_POWER_STATE_D2 = 2,
	LM_POWER_STATE_D3 = 3
} lm_power_state_t;

/********************************************************************
	* offloading.
	*****************************************************************/

typedef u32 lm_offload_t;

#define LM_OFFLOAD_NONE                         0x00000000
#define LM_OFFLOAD_TX_IP_CKSUM                  0x00000001
#define LM_OFFLOAD_RX_IP_CKSUM                  0x00000002
#define LM_OFFLOAD_TX_TCP_CKSUM                 0x00000004
#define LM_OFFLOAD_RX_TCP_CKSUM                 0x00000008
#define LM_OFFLOAD_TX_UDP_CKSUM                 0x00000010
#define LM_OFFLOAD_RX_UDP_CKSUM                 0x00000020
#define LM_OFFLOAD_IPV4_TCP_LSO                 0x00000040
#define LM_OFFLOAD_IPV6_TCP_LSO                 0x00000080
#define LM_OFFLOAD_CHIMNEY                      0x00000100
#define LM_OFFLOAD_IPV6_CHIMNEY                 0x00000200
#define LM_OFFLOAD_TX_TCP6_CKSUM                0x00001000
#define LM_OFFLOAD_RX_TCP6_CKSUM                0x00002000
#define LM_OFFLOAD_TX_UDP6_CKSUM                0x00004000
#define LM_OFFLOAD_RX_UDP6_CKSUM                0x00008000
#define LM_OFFLOAD_RSC_IPV4                     0x00010000
#define LM_OFFLOAD_RSC_IPV6                     0x00020000
#define LM_OFFLOAD_ENCAP_PACKET                 0x00040000
#define LM_OFFLOAD_FCoE_CRC                     0x00080000
#define LM_OFFLOAD_RoCE_CRC                     0x00100000

/******************************************************************
	* RSS Hash Types
	***************************************************************/

typedef u32 lm_rss_hash_t;

#define LM_RSS_HASH_IPV4                       0x00000100
#define LM_RSS_HASH_TCP_IPV4                   0x00000200
#define LM_RSS_HASH_IPV6                       0x00000400
#define LM_RSS_HASH_IPV6_EX                    0x00000800
#define LM_RSS_HASH_TCP_IPV6                   0x00001000
#define LM_RSS_HASH_TCP_IPV6_EX                0x00002000

/*******************************************************************
	* Chip reset reasons.
	****************************************************************/

typedef enum {
	LM_REASON_NONE = 0,
	LM_REASON_DRIVER_RESET = 1,
	LM_REASON_DRIVER_UNLOAD = 2,
	LM_REASON_DRIVER_SHUTDOWN = 3,
	LM_REASON_WOL_SUSPEND = 4,
	LM_REASON_NO_WOL_SUSPEND = 5,
	LM_REASON_DIAG = 6,
	LM_REASON_DRIVER_UNLOAD_POWER_DOWN = 7,	/* Power down phy/serdes */
	LM_REASON_ERROR_RECOVERY = 8
} lm_reason_t;

/********************************************************************
	* Wake up mode.
	******************************************************************/

typedef u32 lm_wake_up_mode_t;

#define LM_WAKE_UP_MODE_NONE                    0
#define LM_WAKE_UP_MODE_MAGIC_PACKET            1
#define LM_WAKE_UP_MODE_NWUF                    2
#define LM_WAKE_UP_MODE_LINK_CHANGE             4

/*********************************************************************
	* Event code.
	******************************************************************/
typedef enum {
	LM_EVENT_CODE_LINK_CHANGE = 0,
	LM_EVENT_CODE_PAUSE_OFFLOAD = 1,
	LM_EVENT_CODE_RESUME_OFFLOAD = 2,
	LM_EVENT_CODE_STOP_CHIP_ACCESS = 3,	/* For Error Recovery Flow */
	LM_EVENT_CODE_RESTART_CHIP_ACCESS = 4,/* For Error Recovery Flow */
	LM_EVENT_CODE_UPLOAD_ALL = 5,
	LM_EVENT_CODE_DCBX_OPERA_CHANGE = 6,
	LM_EVENT_CODE_DCBX_REMOTE_CHANGE = 7,
	LM_EVENT_CODE_INVALIDATE_VF_BLOCK = 8,
} lm_event_code_t;

/********************************************************************
	* Transmit control flags.
	*****************************************************************/

typedef u32 lm_tx_flag_t;

#define LM_TX_FLAG_INSERT_VLAN_TAG              0x01
#define LM_TX_FLAG_COMPUTE_IP_CKSUM             0x02
#define LM_TX_FLAG_COMPUTE_TCP_UDP_CKSUM        0x04
#define LM_TX_FLAG_TCP_LSO_FRAME                0x08
#define LM_TX_FLAG_TCP_LSO_SNAP_FRAME           0x10
#define LM_TX_FLAG_COAL_NOW                     0x20
#define LM_TX_FLAG_DONT_COMPUTE_CRC             0x40
#define LM_TX_FLAG_SKIP_MBQ_WRITE               0x80
#define LM_TX_FLAG_IPV6_PACKET                  0x100
#define LM_TX_FLAG_VLAN_TAG_EXISTS              0x200
#define LM_TX_FLAG_PUSH_MODE                    0x400

/**
	* If this flag is set, the firmware will ignore global
	* configuration (except Outer VLAN)and will handle inner Vlan
	* only according to driver instructions on the bd:
	* 1. LM_TX_FLAG_VLAN_TAG_EXISTS.
	* 2. LM_TX_FLAG_INSERT_VLAN_TAG.
	* Note that if set the firmware will not handle default vlan /
	* NIV tag / DCB.
*/
#define LM_TX_FLAG_FORCE_VLAN_MODE              0x400
/* Encapsulated packet offload flags. */
#define LM_TX_FLAG_IS_ENCAP_PACKET              0x800
#define LM_TX_FLAG_ENCAP_PACKET_IS_INNER_IPV6   0x1000
/* Tunnel flags*/
#define LM_TX_FLAG_COMPUTE_T_IP_CKSUM           0x2000
#define LM_TX_FLAG_T_IPID_LSO_FRAME             0x4000
#define LM_TX_FLAG_IPID_15BIT_FMT               0x8000
#define LM_TX_FLAG_ENCAP_TUNNEL                 0x10000
#define LM_TX_FLAG_MPLS_1_EXISTS                0x20000
#define LM_TX_FLAG_MPLS_2_EXISTS                0x40000
#define LM_TX_FLAG_TS_STAMP                     0x80000
#define LM_TX_FLAG_FCoE_CRC                     0x100000
#define LM_TX_FLAG_RoCE_CRC                     0x200000

typedef struct _lm_pkt_tx_info_t {
	lm_tx_flag_t flags;

	u16 vlan_tag;
	u16 vlan_type;
	u16 lso_mss;
	u16 lso_ip_hdr_len;
	u16 lso_tcp_hdr_len;
	u32 lso_payload_len;

	/* Everest only fields. */
	u32 lso_tcp_send_seq;
	u16 lso_ipid;
	u16 tcp_pseudo_csum;
	u8 lso_tcp_flags;
	u8 tcp_nonce_sum_bit;
	u16 fw_ip_csum_wo_len_flags_frag;

	u8 dst_mac_addr[8];
	s8 cs_any_offset;
	u8 src_mac_addr[8];
	u8 _unused1;
	u8 eth_type[4];

	/* Encapsulated packet offsets.  These fields are only valid when
	 * LM_TX_FLAG_IS_ENCAP_PACKET is set. */
	u8 encap_packet_inner_frame_offset;
	u8 encap_packet_inner_ip_relative_offset;
	u16 encap_packet_inner_tcp_relative_offset;

#ifdef WIN_CDIAG
	u8 *virt_addr;
#endif

} lm_pkt_tx_info_t;

/*********************************************************************
	* Receive control flags.
	******************************************************************/

typedef u32 lm_rx_flag_t;

#define LM_RX_FLAG_VALID_VLAN_TAG               0x01
#define LM_RX_FLAG_VALID_HASH_VALUE             0x02

#define LM_RX_FLAG_IS_FCoE_DATAGRAM             0x04
#define LM_RX_FLAG_FCoE_RoCE_CRC_IS_GOOD        0x08
#define LM_RX_FLAG_FCoE_RoCE_CRC_IS_BAD         0x10

#define LM_RX_FLAG_IS_RoCE_DATAGRAM             0x20

#define LM_RX_FLAG_IS_IPV4_DATAGRAM             0x0100
#define LM_RX_FLAG_IS_IPV6_DATAGRAM             0x0200
#define LM_RX_FLAG_IP_CKSUM_IS_GOOD             0x0400
#define LM_RX_FLAG_IP_CKSUM_IS_BAD              0x0800

#define LM_RX_FLAG_IS_UDP_DATAGRAM              0x1000
#define LM_RX_FLAG_UDP_CKSUM_IS_GOOD            0x2000
#define LM_RX_FLAG_UDP_CKSUM_IS_BAD             0x4000

#define LM_RX_FLAG_IS_TCP_SEGMENT               0x010000
#define LM_RX_FLAG_TCP_CKSUM_IS_GOOD            0x020000
#define LM_RX_FLAG_TCP_CKSUM_IS_BAD             0x040000
#define LM_RX_FLAG_START_RSC_TPA                0x080000

#define LM_RX_FLAG_TUN_IP_CKSUM_IS_GOOD         0x100000
#define LM_RX_FLAG_TUN_IP_CKSUM_IS_BAD          0x200000
#define LM_RX_FLAG_TUN_L4_CKSUM_IS_GOOD         0x400000
#define LM_RX_FLAG_TUN_L4_CKSUM_IS_BAD          0x800000

#define LM_RX_CMPL_FLAGS_PLACEMENT_JUMBO        0x1000000
#define LM_RX_CMPL_FLAGS_PLACEMENT_HDS          0x2000000
#define LM_RX_CMPL_FLAGS_PLACEMENT_GRO_JUMBO    0x4000000
#define LM_RX_CMPL_FLAGS_PLACEMENT_GRO_HDS      0x8000000

#define LM_RX_CMPL_FLAGS_VLAN_EXTRACTION        0x10000000

#define MAX_RX_AGG_BUF_NUM 33
typedef struct _lm_pkt_rx_info_t {
	lm_rx_flag_t flags;

	u32 size;

	u16 vlan_tag;
	u16 _pad;
	u32 cfa_meta;

	/* Virtual address corresponding to the first byte of the first SGL entry.
	 * This is the starting location of the packet which may begin with some
	 * control information. */
	u8 *mem_virt;
	u32 mem_size;

	u8 *agg_mem_virt[MAX_RX_AGG_BUF_NUM];
	u32 agg_chain_id;

	/* these fields only valid when LM_RX_FLAG_START_RSC_TPA is set */
	u16 coal_seg_cnt;
	u16 dup_ack_cnt;
	u32 ts_delta;		/* valid when timestamp is enabled */
	/* if the packet is RSC, this field will hold the total size of the
	 * RSC SCU */
	u32 total_packet_size;

	u32 unused;
} lm_pkt_rx_info_t;

/********************************************************************
	* various type of counters.
	******************************************************************/

typedef enum {
	LM_STATS_BASE = 0x686b3000,
	LM_STATS_FRAMES_XMITTED_OK = 0x686b3001,
	LM_STATS_FRAMES_RECEIVED_OK = 0x686b3002,
	LM_STATS_ERRORED_TRANSMIT_CNT = 0x686b3003,
	LM_STATS_ERRORED_RECEIVE_CNT = 0x686b3004,
	LM_STATS_RCV_CRC_ERROR = 0x686b3005,
	LM_STATS_ALIGNMENT_ERROR = 0x686b3006,
	LM_STATS_SINGLE_COLLISION_FRAMES = 0x686b3007,
	LM_STATS_MULTIPLE_COLLISION_FRAMES = 0x686b3008,
	LM_STATS_FRAMES_DEFERRED = 0x686b3009,
	LM_STATS_MAX_COLLISIONS = 0x686b300a,
	LM_STATS_RCV_OVERRUN = 0x686b300b,
	LM_STATS_XMIT_UNDERRUN = 0x686b300c,
	LM_STATS_UNICAST_FRAMES_XMIT = 0x686b300d,
	LM_STATS_MULTICAST_FRAMES_XMIT = 0x686b300e,
	LM_STATS_BROADCAST_FRAMES_XMIT = 0x686b300f,
	LM_STATS_UNICAST_FRAMES_RCV = 0x686b3010,
	LM_STATS_MULTICAST_FRAMES_RCV = 0x686b3011,
	LM_STATS_BROADCAST_FRAMES_RCV = 0x686b3012,
	LM_STATS_RCV_NO_BUFFER_DROP = 0x686b3013,
	LM_STATS_BYTES_RCV = 0x686b3014,
	LM_STATS_BYTES_XMIT = 0x686b3015,
	LM_STATS_IP4_OFFLOAD = 0x686b3016,
	LM_STATS_TCP_OFFLOAD = 0x686b3017,
	LM_STATS_IF_IN_DISCARDS = 0x686b3018,
	LM_STATS_IF_IN_ERRORS = 0x686b3019,
	LM_STATS_IF_OUT_ERRORS = 0x686b301a,
	LM_STATS_IP6_OFFLOAD = 0x686b301b,
	LM_STATS_TCP6_OFFLOAD = 0x686b301c,
	LM_STATS_XMIT_DISCARDS = 0x686b301d,
	LM_STATS_DIRECTED_BYTES_RCV = 0x686b301e,
	LM_STATS_MULTICAST_BYTES_RCV = 0x686b301f,
	LM_STATS_BROADCAST_BYTES_RCV = 0x686b3020,
	LM_STATS_DIRECTED_BYTES_XMIT = 0x686b3021,
	LM_STATS_MULTICAST_BYTES_XMIT = 0x686b3022,
	LM_STATS_BROADCAST_BYTES_XMIT = 0x686b3023,
} lm_stats_t;

#define NUM_OF_LM_STATS                       36

/*********************************************************************
	* 64-bit value.
	******************************************************************/

typedef union _lm_u64 {
	struct _lm_u64_as_u32 {
#ifdef BIG_ENDIAN_HOST
		u32 high;
		u32 low;
#else
		u32 low;
		u32 high;
#endif
	} as_u32;

	u64 as_u64;

	void *as_ptr;
} lm_u64;

typedef lm_u64 lm_address_t;

#ifdef WIN_CDIAG
typedef lm_u64 LM_PHYSICAL_ADDRESS, *PLM_PHYSICAL_ADDRESS;
#endif

/* 64-bit increment.  The second argument is a 32-bit value. */
#define LM_INC64(result, addend32)      \
	{                                   \
	u32 low;							\
										\
	low = (result)->as_u32.low;         \
	(result)->as_u32.low += (addend32); \
	if ((result)->as_u32.low < low) {   \
		(result)->as_u32.high++;        \
	}                                   \
	}

/* 64-bit decrement.  The second argument is a 32-bit value. */
#define LM_DEC64(result, addend32)      \
	{                                   \
	u32 low;							\
										\
	low = (result)->as_u32.low;         \
	(result)->as_u32.low -= (addend32); \
	if ((result)->as_u32.low > low) {   \
	    (result)->as_u32.high--;        \
	}                                   \
	}

/*******************************************************************
	* IP4 and TCP offload stats.
	****************************************************************/

typedef struct _lm_ip4_offload_stats_t {
	u64 in_receives;
	u64 in_delivers;
	u64 out_requests;
	u32 in_header_errors;
	u32 in_discards;
	u32 out_discards;
	u32 out_no_routes;

	u32 _pad[8];
} lm_ip4_offload_stats_t;

typedef struct _lm_tcp_offload_stats_t {
	u64 in_segments;
	u64 out_segments;
	u32 retran_segments;
	u32 in_errors;
	u32 out_resets;

	u32 _pad[8];
} lm_tcp_offload_stats_t;

/*******************************************************************
	* Host to network order conversion.
	****************************************************************/

#ifdef BIG_ENDIAN_HOST

#ifndef HTON16
#define HTON16(_val16)      (_val16)
#endif
#ifndef HTON32
#define HTON32(_val32)      (_val32)
#ifndef NTOH16
#endif
#define NTOH16(_val16)      (_val16)
#endif
#ifndef NTOH32
#define NTOH32(_val32)      (_val32)
#endif

#else

#ifndef HTON16
#define HTON16(_val16)      (((_val16 & 0xff00) >> 8) | ((_val16 & 0xff) << 8))
#endif
#ifndef HTON32
#define HTON32(_val32)      ((HTON16(_val32) << 16) | (HTON16(_val32 >> 16)))
#endif
#ifndef NTOH16
#define NTOH16(_val16)      HTON16(_val16)
#endif
#ifndef NTOH32
#define NTOH32(_val32)      HTON32(_val32)
#endif

#endif

/********************************************************************
	* Fragment structure.
	*****************************************************************/

typedef struct _lm_frag_t {
	lm_address_t addr;
	u32 size;

#if defined(_WIN64)		/* mirror the SCATTER_GATHER_ELEMENT structure. */
	u64 _reserved;
#else
	u32 _reserved;
#endif
} lm_frag_t;

typedef struct _lm_frag_list_t {
	u32 cnt;

#if defined(_WIN64)		/* mirror the SCATTER_GATHER_LIST structure. */
	u64 size;
#else
	u32 size;
#endif

	lm_frag_t frag_arr[1];
} lm_frag_list_t;

/* a macro for declaring 'lm_frag_list_t' with various array sizes. */
#define DECLARE_FRAG_LIST_BUFFER_TYPE(_FRAG_LIST_TYPE_NAME, _MAX_FRAG_CNT)   \
	typedef struct _##_FRAG_LIST_TYPE_NAME                           \
	{                                                                \
	lm_frag_list_t list;                                             \
	lm_frag_t frag_arr[_MAX_FRAG_CNT - 1];                           \
	} _FRAG_LIST_TYPE_NAME

/*********************************************************************
	* DCBX indicate event parameters.
	******************************************************************/
typedef enum _dcb_condition_selector_t {
	DCB_CONDITION_RESERVED,
	DCB_CONDITION_DEFAULT,
	DCB_CONDITION_TCP_PORT,
	DCB_CONDITION_UDP_PORT,
	DCB_CONDITION_TCP_OR_UDP_PORT,
	DCB_CONDITION_ETHERTYPE,
	DCB_CONDITION_NETDIRECT_PORT,
	DCB_CONDITION_MAX,
} dcb_condition_selector_t;

typedef enum _action_selector_t {
	DCB_ACTION_PRIORITY,
	DCB_ACTION_MAX,
} action_selector_t;

typedef struct _dcb_classif_elem_t {
	u32 flags;
#define DCB_CLASSIF_ENFORCED_BY_VBD         0x1
	dcb_condition_selector_t condition_selector;
	u16 condition_field;
	action_selector_t action_selector;
	u16 action_field;
} dcb_classif_elem_t;

typedef enum _dcb_classif_version_t {
	DCB_CLASSIFI_VER_SIMPLE_ELEM,
	DCB_CLASSIFI_VER_SIMPLE_ELEM_MAX,
} dcb_classif_version_t;

typedef struct _dcb_classif_params_t {
	u16 num_classif_elements;
	u16 _pad;
	dcb_classif_version_t classif_version;
	void *classif_table;
} dcb_classif_params_t;

typedef struct {
	u32 pfc_enable;
#define DCB_PFC_MAX_BIT_ENABLE_MASK     (0xFF)
} dcb_pfc_param_t;

typedef enum {
	TSA_ASSIGNMENT_DCB_TSA_STRICT,
	TSA_ASSIGNMENT_DCB_TSA_CBS,
	TSA_ASSIGNMENT_DCB_TSA_ETS,
} tsa_assignment;

typedef struct _dcb_ets_tsa_param_t {
	u32 num_traffic_classes;
	u8 priority_assignment_table[8];
	u8 tc_bw_assignment_table[8];
	tsa_assignment tsa_assignment_table[8];
} dcb_ets_tsa_param_t;

typedef struct {
	u32 flags;
#define DCB_PARAMS_ETS_ENABLED                      0x00000001
#define DCB_PARAMS_ETS_CHANGED                      0x00000002
#define DCB_PARAMS_PFC_ENABLED                      0x00000004
#define DCB_PARAMS_PFC_CHANGED                      0x00000008
#define DCB_PARAMS_CLASSIF_ENABLED                  0x00000020
#define DCB_PARAMS_CLASSIF_CHANGED                  0x00000040
#define DCB_PARAMS_WILLING                          0x00000080

	dcb_ets_tsa_param_t ets_params;
	dcb_pfc_param_t pfc_params;
	dcb_classif_params_t classif_params;
	u32 reserved[4];
} dcb_indicate_event_params_t;

/*********************************************************************
* Macro fore calculating the address of the base of the structure given
* its type, and an address of a field within the structure.
**********************************************************************
*/

#define GET_CONTAINING_RECORD(address, type, field) \
	((type *)((u8 *)(address) - (u8 *)(&((type *)0)->field)))

/***********************************************************
* Simple macros.
************************************************************
*/

#define IS_ETH_BROADCAST(eth_addr)                                 \
	(((unsigned char *)(eth_addr))[0] == ((unsigned char)0xff))

#define IS_ETH_MULTICAST(eth_addr)                                 \
	(((unsigned char *)(eth_addr))[0] & ((unsigned char)0x01))

#define IS_ETH_ADDRESS_EQUAL(eth_addr1, eth_addr2)                 \
	((((unsigned char *)(eth_addr1))[0] ==                         \
	((unsigned char *)(eth_addr2))[0]) &&                          \
	(((unsigned char *)(eth_addr1))[1] ==                          \
	((unsigned char *)(eth_addr2))[1]) &&                          \
	(((unsigned char *)(eth_addr1))[2] ==                          \
	((unsigned char *)(eth_addr2))[2]) &&                          \
	(((unsigned char *)(eth_addr1))[3] ==                          \
	((unsigned char *)(eth_addr2))[3]) &&                          \
	(((unsigned char *)(eth_addr1))[4] ==                          \
	((unsigned char *)(eth_addr2))[4]) &&                          \
	(((unsigned char *)(eth_addr1))[5] ==                          \
	((unsigned char *)(eth_addr2))[5]))

#define COPY_ETH_ADDRESS(src, dst)                                 \
do {		\
	((unsigned char *)(dst))[0] = ((unsigned char *)(src))[0];     \
	((unsigned char *)(dst))[1] = ((unsigned char *)(src))[1];     \
	((unsigned char *)(dst))[2] = ((unsigned char *)(src))[2];     \
	((unsigned char *)(dst))[3] = ((unsigned char *)(src))[3];     \
	((unsigned char *)(dst))[4] = ((unsigned char *)(src))[4];     \
	((unsigned char *)(dst))[5] = ((unsigned char *)(src))[5];     \
} while (0)

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

#endif /* _LM_DEFS_H */
