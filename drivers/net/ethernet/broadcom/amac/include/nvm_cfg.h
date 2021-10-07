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

#ifndef NVM_CFG_H
#define NVM_CFG_H

typedef struct nvm_cfg_mac_address {
	u32 mac_addr_hi;
#define NVM_CFG_MAC_ADDRESS_HI_MASK                             0x0000FFFF
#define NVM_CFG_MAC_ADDRESS_HI_SHIFT                            0

	u32 mac_addr_lo;
} nvm_cfg_mac_address_t;

enum nvm_cfg_sections {
	NVM_CFG_SECTION_BOARD = 0,
	NVM_CFG_SECTION_BOARD_IO,
	NVM_CFG_SECTION_PCIE,
	NVM_CFG_SECTION_FEATURES,
	NVM_CFG_SECTION_LINK_SETTINGS,
	NVM_CFG_SECTION_MGMT,
	NVM_CFG_SECTION_PHY,
	NVM_CFG_SECTION_PRE_BOOT,
	NVM_CFG_SECTION_VF,
	NVM_CFG_SECTION_DUAL_PHY,
	NVM_CFG_SECTION_NPAR,
	NVM_CFG_SECTION_TEMPERATURE_CONTROL,
	NVM_CFG_SECTION_EMBEDDED_BMC,
	NVM_CFG_SECTION_MAX = 32
};

#define NVM_CFG_SECTION_LEN_SHIFT	16
#define NVM_CFG_SECTION_LEN_MASK	0xFFFF0000
#define NVM_CFG_SECTION_OFFSET_MASK	0xFFFF

enum nvm_cfg_config {
	SHARED_CFG,
	PORT_CFG,
	FUNC_CFG,
};

/******************************************
 * shared_cfg structs
 ******************************************/

typedef struct shared_cfg {
	u32 num_sections;
	u32 sections_offset[NVM_CFG_SECTION_MAX];

#define SHARED_CFG_BOARD_SIZE	(sizeof(struct shared_cfg_board) / 4)
	struct shared_cfg_board {
		u32 board_cfg;
#define SHARED_CFG_BOARD_PORT_SWAP_MASK                         0x00000001
#define SHARED_CFG_BOARD_PORT_SWAP_SHIFT                        0
#define SHARED_CFG_BOARD_PORT_SWAP_DISABLED                     0x0
#define SHARED_CFG_BOARD_PORT_SWAP_ENABLED                      0x1
#define SHARED_CFG_BOARD_BOARD_VERSION_MASK                     0x000001FE
#define SHARED_CFG_BOARD_BOARD_VERSION_SHIFT                    1
#define SHARED_CFG_BOARD_BOARD_REVISION_MASK                    0x0001FE00
#define SHARED_CFG_BOARD_BOARD_REVISION_SHIFT                   9
#define SHARED_CFG_BOARD_HIDE_PORT1_MASK                        0x001E0000
#define SHARED_CFG_BOARD_HIDE_PORT1_SHIFT                       17
#define SHARED_CFG_BOARD_HIDE_PORT1_DISABLED                    (0x0L << 17)
#define SHARED_CFG_BOARD_HIDE_PORT1_ENABLED                     (0x1L << 17)
#define SHARED_CFG_BOARD_PORT_LAYOUT_MASK                       0x01E00000
#define SHARED_CFG_BOARD_PORT_LAYOUT_SHIFT                      21
#define SHARED_CFG_BOARD_PORT_LAYOUT_2P_01                      (0x0L << 21)
#define SHARED_CFG_BOARD_PORT_LAYOUT_2P_10                      (0x1L << 21)
#define SHARED_CFG_BOARD_PORT_LAYOUT_4P_0123                    (0x2L << 21)
#define SHARED_CFG_BOARD_PORT_LAYOUT_4P_1032                    (0x3L << 21)
#define SHARED_CFG_BOARD_PORT_LAYOUT_4P_2301                    (0x4L << 21)
#define SHARED_CFG_BOARD_PORT_LAYOUT_4P_3210                    (0x5L << 21)
#define BOARD_DEVIATION_CONFIG_MASK                             0x1E000000
#define BOARD_DEVIATION_CONFIG_SHIFT                            25

		u32 spare_number[5];

	} board;

#define SHARED_CFG_BOARD_IO_SIZE  (sizeof(struct shared_cfg_board_io)/4)
	struct shared_cfg_board_io {
		u32 board_io_cfg;
#define SHARED_CFG_BOARD_IO_FAN_FAILURE_ENFORCEMENT_MASK        0x00000001
#define SHARED_CFG_BOARD_IO_FAN_FAILURE_ENFORCEMENT_SHIFT       0
#define SHARED_CFG_BOARD_IO_FAN_FAILURE_ENFORCEMENT_DISABLED    0x0
#define SHARED_CFG_BOARD_IO_FAN_FAILURE_ENFORCEMENT_ENABLED     0x1
#define SHARED_CFG_BOARD_IO_FRU_VPD_PRESENT_MASK                0x00000002
#define SHARED_CFG_BOARD_IO_FRU_VPD_PRESENT_SHIFT               1
#define SHARED_CFG_BOARD_IO_FRU_VPD_PRESENT_DISABLED            (0x0L << 1)
#define SHARED_CFG_BOARD_IO_FRU_VPD_PRESENT_ENABLED             (0x1L << 1)
#define SHARED_CFG_BOARD_IO_FRU_VPD_SELECTION_MASK              0x0000001C
#define SHARED_CFG_BOARD_IO_FRU_VPD_SELECTION_SHIFT             2
#define SHARED_CFG_BOARD_IO_FRU_VPD_SELECTION_SMB_P0            (0x0L << 2)
#define SHARED_CFG_BOARD_IO_FRU_VPD_SELECTION_SMB_P1            (0x1L << 2)
#define SHARED_CFG_BOARD_IO_FRU_VPD_SLAVE_ADDR_MASK             0x00001FE0
#define SHARED_CFG_BOARD_IO_FRU_VPD_SLAVE_ADDR_SHIFT            5
#define EXT_THERM_SEN_PRESENT_MASK                              0x00002000
#define EXT_THERM_SEN_PRESENT_SHIFT                             13
#define EXT_THERM_SEN_PRESENT_DISABLED                          (0x0L << 13)
#define EXT_THERM_SEN_PRESENT_ENABLED                           (0x1L << 13)
#define EXT_THERM_SEN_SEL_MASK                                  0x0001C000
#define EXT_THERM_SEN_SEL_SHIFT                                 14
#define EXT_THERM_SEN_SEL_SMB_P0                                (0x0L << 14)
#define EXT_THERM_SEN_SEL_SMB_P1                                (0x1L << 14)
#define EXT_THERM_SEN_SLAVE_ADDR_MASK                           0x01FE0000
#define EXT_THERM_SEN_SLAVE_ADDR_SHIFT                          17

	} board_io;

#define SHARED_CFG_PCIE_SIZE		(sizeof(struct shared_cfg_pcie)/4)
	struct shared_cfg_pcie {
		u32 pcie_cfg;
#define SHARED_CFG_PCIE_PCI_GEN_MASK                            0x00000003
#define SHARED_CFG_PCIE_PCI_GEN_SHIFT                           0
#define SHARED_CFG_PCIE_PCI_GEN_PCI_GEN1                        0x0
#define SHARED_CFG_PCIE_PCI_GEN_PCI_GEN2                        0x1
#define SHARED_CFG_PCIE_PCI_GEN_PCI_GEN3                        0x2
#define SHARED_CFG_PCIE_ASPM_SUPPORT_MASK                       0x0000000C
#define SHARED_CFG_PCIE_ASPM_SUPPORT_SHIFT                      2
#define SHARED_CFG_PCIE_ASPM_SUPPORT_L0S_L1_ENABLED             (0x0L << 2)
#define SHARED_CFG_PCIE_ASPM_SUPPORT_L0S_DISABLED               (0x1L << 2)
#define SHARED_CFG_PCIE_ASPM_SUPPORT_L1_DISABLED                (0x2L << 2)
#define SHARED_CFG_PCIE_ASPM_SUPPORT_L0S_L1_DISABLED            (0x3L << 2)
#define SHARED_CFG_PCIE_PREVENT_PCIE_L1_MENTRY_MASK             0x00000010
#define SHARED_CFG_PCIE_PREVENT_PCIE_L1_MENTRY_SHIFT            4
#define SHARED_CFG_PCIE_PREVENT_PCIE_L1_MENTRY_DISABLED         (0x0L << 4)
#define SHARED_CFG_PCIE_PREVENT_PCIE_L1_MENTRY_ENABLED          (0x1L << 4)
#define SHARED_CFG_PCIE_PCIE_GEN1_PREEMPHASIS_MASK              0x000000E0
#define SHARED_CFG_PCIE_PCIE_GEN1_PREEMPHASIS_SHIFT             5
#define SHARED_CFG_PCIE_PCIE_GEN1_PREEMPHASIS_HW                (0x0L << 5)
#define SHARED_CFG_PCIE_PCIE_GEN1_PREEMPHASIS_0DB               (0x1L << 5)
#define SHARED_CFG_PCIE_PCIE_GEN1_PREEMPHASIS_3_5DB             (0x2L << 5)
#define SHARED_CFG_PCIE_PCIE_GEN1_PREEMPHASIS_6_0DB             (0x3L << 5)
#define SHARED_CFG_PCIE_PCIE_GEN2_PREEMPHASIS_MASK              0x00000700
#define SHARED_CFG_PCIE_PCIE_GEN2_PREEMPHASIS_SHIFT             8
#define SHARED_CFG_PCIE_PCIE_GEN2_PREEMPHASIS_HW                (0x0L << 8)
#define SHARED_CFG_PCIE_PCIE_GEN2_PREEMPHASIS_0DB               (0x1L << 8)
#define SHARED_CFG_PCIE_PCIE_GEN2_PREEMPHASIS_3_5DB             (0x2L << 8)
#define SHARED_CFG_PCIE_PCIE_GEN2_PREEMPHASIS_6_0DB             (0x3L << 8)
#define PF_MSIX_MAX_NUM_MASK                                    0x001FF800
#define PF_MSIX_MAX_NUM_SHIFT                                   11

	} pcie;

#define SHARED_CFG_FEATURES_SIZE	(sizeof(struct shared_cfg_features)/4)
	struct shared_cfg_features {
		u32 lldp_cfg;
#define SHARED_CFG_FEATURES_LLDP_TRANSMIT_INTERVAL_MASK         0x000000FF
#define SHARED_CFG_FEATURES_LLDP_TRANSMIT_INTERVAL_SHIFT        0
#define SHARED_CFG_FEATURES_LLDP_DEVICE_TYPE_ID_MASK            0x0000FF00
#define SHARED_CFG_FEATURES_LLDP_DEVICE_TYPE_ID_SHIFT           8

		u32 features_cfg;
#define SHARED_CFG_FEATURES_HIDE_DCBX_FEATURE_MASK              0x00000001
#define SHARED_CFG_FEATURES_HIDE_DCBX_FEATURE_SHIFT             0
#define SHARED_CFG_FEATURES_HIDE_DCBX_FEATURE_DISABLED          0x0
#define SHARED_CFG_FEATURES_HIDE_DCBX_FEATURE_ENABLED           0x1
#define SHARED_CFG_FEATURES_FLR_CAPABILITY_MASK                 0x00000002
#define SHARED_CFG_FEATURES_FLR_CAPABILITY_SHIFT                1
#define SHARED_CFG_FEATURES_FLR_CAPABILITY_DISABLED             (0x0L << 1)
#define SHARED_CFG_FEATURES_FLR_CAPABILITY_ENABLED              (0x1L << 1)

	} features;

#define SHARED_CFG_LINK_SETTINGS_SIZE	0

#define SHARED_CFG_MGMT_SIZE		(sizeof(struct shared_cfg_mgmt)/4)
	struct shared_cfg_mgmt {
		u32 smbus_config;
#define SHARED_CFG_MGMT_SMBUS_TIMING_MASK                       0x00000001
#define SHARED_CFG_MGMT_SMBUS_TIMING_SHIFT                      0
#define SHARED_CFG_MGMT_SMBUS_TIMING_100KHZ                     0x0
#define SHARED_CFG_MGMT_SMBUS_TIMING_400KHZ                     0x1
#define SHARED_CFG_MGMT_SMBUS_ADDRESS__MASK                     0x000001FE
#define SHARED_CFG_MGMT_SMBUS_ADDRESS__SHIFT                    1
#define SHARED_CFG_MGMT_SMBUS_ARB_MASK                          0x00000200
#define SHARED_CFG_MGMT_SMBUS_ARB_SHIFT                         9
#define SHARED_CFG_MGMT_SMBUS_ARB_DISABLED                      (0x0L << 9)
#define SHARED_CFG_MGMT_SMBUS_ARB_ENABLED                       (0x1L << 9)
#define SHARED_CFG_MGMT_SMBUS_SELECTION_MASK                    0x0003FC00
#define SHARED_CFG_MGMT_SMBUS_SELECTION_SHIFT                   10
#define SHARED_CFG_MGMT_SMBUS_SELECTION_SMB_P0                  (0x0L << 10)
#define SHARED_CFG_MGMT_SMBUS_SELECTION_SMB_P1                  (0x1L << 10)
#define SHARED_CFG_MGMT_SMBUS_SELECTION_N_A                     (0xFFL << 10)
#define SHARED_CFG_MGMT_NC_MSI_OVER_RMII_MASK                   0x00040000
#define SHARED_CFG_MGMT_NC_MSI_OVER_RMII_SHIFT                  18
#define SHARED_CFG_MGMT_NC_MSI_OVER_RMII_DISABLED               (0x0L << 18)
#define SHARED_CFG_MGMT_NC_MSI_OVER_RMII_ENABLED                (0x1L << 18)
#define SHARED_CFG_MGMT_NC_MSI_OVER_SMBUS_MASK                  0x00080000
#define SHARED_CFG_MGMT_NC_MSI_OVER_SMBUS_SHIFT                 19
#define SHARED_CFG_MGMT_NC_MSI_OVER_SMBUS_DISABLED              (0x0L << 19)
#define SHARED_CFG_MGMT_NC_MSI_OVER_SMBUS_ENABLED               (0x1L << 19)
#define SHARED_CFG_MGMT_NC_MSI_OVER_PCIEVDM_MASK                0x00100000
#define SHARED_CFG_MGMT_NC_MSI_OVER_PCIEVDM_SHIFT               20
#define SHARED_CFG_MGMT_NC_MSI_OVER_PCIEVDM_DISABLED            (0x0L << 20)
#define SHARED_CFG_MGMT_NC_MSI_OVER_PCIEVDM_ENABLED             (0x1L << 20)

		u32 mgmt_cfg;
#define SHARED_CFG_MGMT_MGMT_FIRMWARE_SELECTION_TYPE_MASK       0x00000007
#define SHARED_CFG_MGMT_MGMT_FIRMWARE_SELECTION_TYPE_SHIFT      0
#define SHARED_CFG_MGMT_MGMT_FIRMWARE_SELECTION_TYPE_DISABLED   0x0
#define SHARED_CFG_MGMT_MGMT_FIRMWARE_SELECTION_TYPE_NC_MSI     0x1
#define SHARED_CFG_MGMT_MGMT_FIRMWARE_SELECTION_TYPE_SMASH      0x2

		u32 mfw_cfg;
#define NCSI_ID_METHOD_MASK                                     0x00000001
#define NCSI_ID_METHOD_SHIFT                                    0
#define NCSI_ID_METHOD_GPIO                                     0x0
#define NCSI_ID_METHOD_NVRAM                                    0x1
#define NCSI_ID_MASK                                            0x00000006
#define NCSI_ID_SHIFT                                           1

	} mgmt;

#define SHARED_CFG_PHY_SIZE		(sizeof(struct shared_cfg_phy)/4)
	struct shared_cfg_phy {
		u32 core_cfg;
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_MASK                   0x000000FF
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_SHIFT                  0
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_1X10G                  0x0
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_2X10G                  0x1
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_4X10G                  0x2
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_1X25G                  0x3
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_2X25G                  0x4
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_1X40G                  0x5
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_1X50G                  0x6
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_2X10G__P_2X25G         0x7
#define SHARED_CFG_PHY_NETWORK_PORT_MODE_2X10G__P_1X50G         0x8
#define SHARED_CFG_PHY_ENFORCE_PREEMPHASIS_CFG_MASK             0x00000100
#define SHARED_CFG_PHY_ENFORCE_PREEMPHASIS_CFG_SHIFT            8
#define SHARED_CFG_PHY_ENFORCE_PREEMPHASIS_CFG_DISABLED         (0x0L << 8)
#define SHARED_CFG_PHY_ENFORCE_PREEMPHASIS_CFG_ENABLED          (0x1L << 8)

		u32 lane_swap;
#define SHARED_CFG_PHY_RX_LANE0_SWAP_MASK                       0x0000000F
#define SHARED_CFG_PHY_RX_LANE0_SWAP_SHIFT                      0
#define SHARED_CFG_PHY_RX_LANE1_SWAP_MASK                       0x000000F0
#define SHARED_CFG_PHY_RX_LANE1_SWAP_SHIFT                      4
#define SHARED_CFG_PHY_RX_LANE2_SWAP_MASK                       0x00000F00
#define SHARED_CFG_PHY_RX_LANE2_SWAP_SHIFT                      8
#define SHARED_CFG_PHY_RX_LANE3_SWAP_MASK                       0x0000F000
#define SHARED_CFG_PHY_RX_LANE3_SWAP_SHIFT                      12
#define SHARED_CFG_PHY_TX_LANE0_SWAP_MASK                       0x000F0000
#define SHARED_CFG_PHY_TX_LANE0_SWAP_SHIFT                      16
#define SHARED_CFG_PHY_TX_LANE1_SWAP_MASK                       0x00F00000
#define SHARED_CFG_PHY_TX_LANE1_SWAP_SHIFT                      20
#define SHARED_CFG_PHY_TX_LANE2_SWAP_MASK                       0x0F000000
#define SHARED_CFG_PHY_TX_LANE2_SWAP_SHIFT                      24
#define SHARED_CFG_PHY_TX_LANE3_SWAP_MASK                       0xF0000000
#define SHARED_CFG_PHY_TX_LANE3_SWAP_SHIFT                      28

		u32 lane_polarity;
#define SHARED_CFG_PHY_RX_LANE0_POL_FLIP_MASK                   0x00000001
#define SHARED_CFG_PHY_RX_LANE0_POL_FLIP_SHIFT                  0
#define SHARED_CFG_PHY_RX_LANE1_POL_FLIP_MASK                   0x00000002
#define SHARED_CFG_PHY_RX_LANE1_POL_FLIP_SHIFT                  1
#define SHARED_CFG_PHY_RX_LANE2_POL_FLIP_MASK                   0x00000004
#define SHARED_CFG_PHY_RX_LANE2_POL_FLIP_SHIFT                  2
#define SHARED_CFG_PHY_RX_LANE3_POL_FLIP_MASK                   0x00000008
#define SHARED_CFG_PHY_RX_LANE3_POL_FLIP_SHIFT                  3
#define SHARED_CFG_PHY_TX_LANE0_POL_FLIP_MASK                   0x00000010
#define SHARED_CFG_PHY_TX_LANE0_POL_FLIP_SHIFT                  4
#define SHARED_CFG_PHY_TX_LANE1_POL_FLIP_MASK                   0x00000020
#define SHARED_CFG_PHY_TX_LANE1_POL_FLIP_SHIFT                  5
#define SHARED_CFG_PHY_TX_LANE2_POL_FLIP_MASK                   0x00000040
#define SHARED_CFG_PHY_TX_LANE2_POL_FLIP_SHIFT                  6
#define SHARED_CFG_PHY_TX_LANE3_POL_FLIP_MASK                   0x00000080
#define SHARED_CFG_PHY_TX_LANE3_POL_FLIP_SHIFT                  7

		u32 preemphasis;
#define SHARED_CFG_PHY_LANE0_PREEMP_MASK                        0x000000FF
#define SHARED_CFG_PHY_LANE0_PREEMP_SHIFT                       0
#define SHARED_CFG_PHY_LANE1_PREEMP_MASK                        0x0000FF00
#define SHARED_CFG_PHY_LANE1_PREEMP_SHIFT                       8
#define SHARED_CFG_PHY_LANE2_PREEMP_MASK                        0x00FF0000
#define SHARED_CFG_PHY_LANE2_PREEMP_SHIFT                       16
#define SHARED_CFG_PHY_LANE3_PREEMP_MASK                        0xFF000000
#define SHARED_CFG_PHY_LANE3_PREEMP_SHIFT                       24

		u32 driver_current;
#define SHARED_CFG_PHY_LANE0_AMP_MASK                           0x000000FF
#define SHARED_CFG_PHY_LANE0_AMP_SHIFT                          0
#define SHARED_CFG_PHY_LANE1_AMP_MASK                           0x0000FF00
#define SHARED_CFG_PHY_LANE1_AMP_SHIFT                          8
#define SHARED_CFG_PHY_LANE2_AMP_MASK                           0x00FF0000
#define SHARED_CFG_PHY_LANE2_AMP_SHIFT                          16
#define SHARED_CFG_PHY_LANE3_AMP_MASK                           0xFF000000
#define SHARED_CFG_PHY_LANE3_AMP_SHIFT                          24

	} phy;

#define SHARED_CFG_PRE_BOOT_SIZE	(sizeof(struct shared_cfg_pre_boot)/4)
	struct shared_cfg_pre_boot {
		u32 exp_rom_cfg;
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_MASK             0x000000FF
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_SHIFT            0
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_DISABLED         0x0
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_2K               0x1
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_4K               0x2
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_8K               0x3
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_16K              0x4
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_32K              0x5
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_64K              0x6
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_128K             0x7
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_256K             0x8
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_512K             0x9
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_1M               0xA
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_2M               0xB
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_4M               0xC
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_8M               0xD
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_16M              0xE
#define SHARED_CFG_PRE_BOOT_EXPANSION_ROM_SIZE_32M              0xF

	} pre_boot;

#define SHARED_CFG_VF_SIZE		(sizeof(struct shared_cfg_VF)/4)
	struct shared_cfg_VF {
		u32 mf_cfg;
#define SHARED_CFG_VF_ENABLE_SRIOV_MASK                         0x00000001
#define SHARED_CFG_VF_ENABLE_SRIOV_SHIFT                        0
#define SHARED_CFG_VF_ENABLE_SRIOV_DISABLED                     0x0
#define SHARED_CFG_VF_ENABLE_SRIOV_ENABLED                      0x1
#define SHARED_CFG_VF_MSI_MX_AUTO_MASK_MASK                     0x00000002
#define SHARED_CFG_VF_MSI_MX_AUTO_MASK_SHIFT                    1
#define SHARED_CFG_VF_MSI_MX_AUTO_MASK_DISABLED                 (0x0L << 1)
#define SHARED_CFG_VF_MSI_MX_AUTO_MASK_ENABLED                  (0x1L << 1)

	} VF;

#define SHARED_CFG_DUAL_PHY_SIZE	0

#define SHARED_CFG_NPAR_SIZE	0

#define SHARED_CFG_TEMPERATURE_CONTROL_SIZE		(sizeof(struct shared_cfg_temperature_control)/4)
	struct shared_cfg_temperature_control {
		u32 temperature_monitor1;
#define TEMP_FAN_THRESH_MASK                                    0x0000007F
#define TEMP_FAN_THRESH_SHIFT                                   0
#define TEMP_SHUT_THRESH_MASK                                   0x00003F80
#define TEMP_SHUT_THRESH_SHIFT                                  7
#define TEMP_FAN_GPIO_MASK                                      0x003FC000
#define TEMP_FAN_GPIO_SHIFT                                     14
#define TEMP_SHUT_GPIO_MASK                                     0x3FC00000
#define TEMP_SHUT_GPIO_SHIFT                                    22

		u32 temperature_monitor2;
#define TEMP_PERIOD_MASK                                        0x0000FFFF
#define TEMP_PERIOD_SHIFT                                       0
#define TEMP_SMBUS_ADDR_MASK                                    0x00FF0000
#define TEMP_SMBUS_ADDR_SHIFT                                   16

		u32 reserved1;
#define SHARED_CFG_TEMPERATURE_CONTROL_RESERVED_MASK            0x00000001
#define SHARED_CFG_TEMPERATURE_CONTROL_RESERVED_SHIFT           0

	} temperature_control;

#define SHARED_CFG_EMBEDDED_BMC_SIZE		(sizeof(struct shared_cfg_embedded_bmc)/4)
	struct shared_cfg_embedded_bmc {
		u32 reserved[4];

	} embedded_bmc;

	u32 reserved[139];	/* 0x6C */

	u32 crc;

} shared_cfg_t;

/******************************************
 * port_cfg structs
 ******************************************/

typedef struct port_cfg {
	u32 num_sections;
	u32 sections_offset[NVM_CFG_SECTION_MAX];

#define PORT_CFG_BOARD_SIZE		(sizeof(struct port_cfg_board)/4)
	struct port_cfg_board {
		u32 power_dissipated;
#define PORT_CFG_BOARD_POWER_DIS_D0_MASK                        0x000000FF
#define PORT_CFG_BOARD_POWER_DIS_D0_SHIFT                       0
#define PORT_CFG_BOARD_POWER_DIS_D1_MASK                        0x0000FF00
#define PORT_CFG_BOARD_POWER_DIS_D1_SHIFT                       8
#define PORT_CFG_BOARD_POWER_DIS_D2_MASK                        0x00FF0000
#define PORT_CFG_BOARD_POWER_DIS_D2_SHIFT                       16
#define PORT_CFG_BOARD_POWER_DIS_D3_MASK                        0xFF000000
#define PORT_CFG_BOARD_POWER_DIS_D3_SHIFT                       24

		u32 power_consumed;
#define PORT_CFG_BOARD_POWER_CONS_D0_MASK                       0x000000FF
#define PORT_CFG_BOARD_POWER_CONS_D0_SHIFT                      0
#define PORT_CFG_BOARD_POWER_CONS_D1_MASK                       0x0000FF00
#define PORT_CFG_BOARD_POWER_CONS_D1_SHIFT                      8
#define PORT_CFG_BOARD_POWER_CONS_D2_MASK                       0x00FF0000
#define PORT_CFG_BOARD_POWER_CONS_D2_SHIFT                      16
#define PORT_CFG_BOARD_POWER_CONS_D3_MASK                       0xFF000000
#define PORT_CFG_BOARD_POWER_CONS_D3_SHIFT                      24

		u32 pci_id;
#define PORT_CFG_BOARD_VENDOR_ID_MASK                           0x0000FFFF
#define PORT_CFG_BOARD_VENDOR_ID_SHIFT                          0
#define PORT_CFG_BOARD_VENDOR_DEVICE_ID_MASK                    0xFFFF0000
#define PORT_CFG_BOARD_VENDOR_DEVICE_ID_SHIFT                   16

		u32 pci_subsys_id;
#define PORT_CFG_BOARD_SUBSYSTEM_VENDOR_ID_MASK                 0x0000FFFF
#define PORT_CFG_BOARD_SUBSYSTEM_VENDOR_ID_SHIFT                0
#define PORT_CFG_BOARD_SUBSYSTEM_DEVICE_ID_MASK                 0xFFFF0000
#define PORT_CFG_BOARD_SUBSYSTEM_DEVICE_ID_SHIFT                16

		u32 board_cfg1;
#define PORT_CFG_BOARD_LED_MODE_MASK                            0x000000FF
#define PORT_CFG_BOARD_LED_MODE_SHIFT                           0
#define PORT_CFG_BOARD_LED_MODE_MAC1                            0x0
#define PORT_CFG_BOARD_LED_MODE_PHY1                            0x1
#define PORT_CFG_BOARD_LED_MODE_PHY2                            0x2
#define PORT_CFG_BOARD_LED_MODE_PHY3                            0x3
#define PORT_CFG_BOARD_LED_MODE_MAC2                            0x4
#define PORT_CFG_BOARD_LED_MODE_PHY4                            0x5
#define PORT_CFG_BOARD_LED_MODE_PHY5                            0x6
#define PORT_CFG_BOARD_LED_MODE_PHY6                            0x7
#define PORT_CFG_BOARD_LED_MODE_MAC3                            0x8
#define PORT_CFG_BOARD_LED_MODE_PHY7                            0x9
#define PORT_CFG_BOARD_LED_MODE_PHY8                            0xA
#define PORT_CFG_BOARD_LED_MODE_PHY9                            0xB
#define PORT_CFG_BOARD_LED_MODE_MAC4                            0xC
#define PORT_CFG_BOARD_LED_MODE_PHY10                           0xD
#define PORT_CFG_BOARD_LED_MODE_PHY11                           0xE
#define PORT_CFG_BOARD_LED_MODE_PHY12                           0xF
#define PORT_CFG_BOARD_LED_SPEED_0_MASK                         0x0000FF00
#define PORT_CFG_BOARD_LED_SPEED_0_SHIFT                        8
#define PORT_CFG_BOARD_LED_SPEED_0_1G_OR_LOWER                  (0x1L << 8)
#define PORT_CFG_BOARD_LED_SPEED_0_10G                          (0x2L << 8)
#define PORT_CFG_BOARD_LED_SPEED_0_25G                          (0x8L << 8)
#define PORT_CFG_BOARD_LED_SPEED_0_40G                          (0x10L << 8)
#define PORT_CFG_BOARD_LED_SPEED_0_50G                          (0x20L << 8)
#define PORT_CFG_BOARD_LED_SPEED_0_100G                         (0x40L << 8)
#define PORT_CFG_BOARD_LED_SPEED_1_MASK                         0x00FF0000
#define PORT_CFG_BOARD_LED_SPEED_1_SHIFT                        16
#define PORT_CFG_BOARD_LED_SPEED_1_1G_OR_LOWER                  (0x1L << 16)
#define PORT_CFG_BOARD_LED_SPEED_1_10G                          (0x2L << 16)
#define PORT_CFG_BOARD_LED_SPEED_1_25G                          (0x8L << 16)
#define PORT_CFG_BOARD_LED_SPEED_1_40G                          (0x10L << 16)
#define PORT_CFG_BOARD_LED_SPEED_1_50G                          (0x20L << 16)
#define PORT_CFG_BOARD_LED_SPEED_1_100G                         (0x40L << 16)

	} board;

#define PORT_CFG_BOARD_IO_SIZE	0

#define PORT_CFG_PCIE_SIZE		(sizeof(struct port_cfg_pcie)/4)
	struct port_cfg_pcie {
		u32 pcie_cfg1;
#define PORT_CFG_PCIE_BAR1_SIZE_MASK                            0x0000000F
#define PORT_CFG_PCIE_BAR1_SIZE_SHIFT                           0
#define PORT_CFG_PCIE_BAR2_SIZE_MASK                            0x000000F0
#define PORT_CFG_PCIE_BAR2_SIZE_SHIFT                           4
#define PORT_CFG_PCIE_BAR3_SIZE_MASK                            0x00000F00
#define PORT_CFG_PCIE_BAR3_SIZE_SHIFT                           8
#define PORT_CFG_PCIE_BAR_SIZE_DISABLED                         0x0
#define PORT_CFG_PCIE_BAR_SIZE_64K                              0x1
#define PORT_CFG_PCIE_BAR_SIZE_128K                             0x2
#define PORT_CFG_PCIE_BAR_SIZE_256K                             0x3
#define PORT_CFG_PCIE_BAR_SIZE_512K                             0x4
#define PORT_CFG_PCIE_BAR_SIZE_1M                               0x5
#define PORT_CFG_PCIE_BAR_SIZE_2M                               0x6
#define PORT_CFG_PCIE_BAR_SIZE_4M                               0x7
#define PORT_CFG_PCIE_BAR_SIZE_8M                               0x8
#define PORT_CFG_PCIE_BAR_SIZE_16M                              0x9
#define PORT_CFG_PCIE_BAR_SIZE_32M                              0xA
#define PORT_CFG_PCIE_BAR_SIZE_64M                              0xB
#define PORT_CFG_PCIE_BAR_SIZE_128M                             0xC
#define PORT_CFG_PCIE_BAR_SIZE_256M                             0xD
#define PORT_CFG_PCIE_BAR_SIZE_512M                             0xE
#define PORT_CFG_PCIE_BAR_SIZE_1G                               0xF

	} pcie;

#define PORT_CFG_FEATURES_SIZE		(sizeof(struct port_cfg_features)/4)
	struct port_cfg_features {
		u32 features;
#define PORT_CFG_FEATURES_ENABLE_WOL_ON_ACPI_PATTERN_MASK       0x00000001
#define PORT_CFG_FEATURES_ENABLE_WOL_ON_ACPI_PATTERN_SHIFT      0
#define PORT_CFG_FEATURES_ENABLE_WOL_ON_ACPI_PATTERN_DISABLED   0x0
#define PORT_CFG_FEATURES_ENABLE_WOL_ON_ACPI_PATTERN_ENABLED    0x1
#define PORT_CFG_FEATURES_MAGIC_PACKET_WOL_MASK                 0x00000002
#define PORT_CFG_FEATURES_MAGIC_PACKET_WOL_SHIFT                1
#define PORT_CFG_FEATURES_MAGIC_PACKET_WOL_DISABLED             (0x0L << 1)
#define PORT_CFG_FEATURES_MAGIC_PACKET_WOL_ENABLED              (0x1L << 1)
#define PORT_CFG_FEATURES_DCBX_MODE_MASK                        0x0000003C
#define PORT_CFG_FEATURES_DCBX_MODE_SHIFT                       2
#define PORT_CFG_FEATURES_DCBX_MODE_DISABLED                    (0x0L << 2)
#define PORT_CFG_FEATURES_DCBX_MODE_IEEE                        (0x1L << 2)
#define PORT_CFG_FEATURES_DCBX_MODE_CEE                         (0x2L << 2)
#define PORT_CFG_FEATURES_MF_MODE_MASK                          0x000007C0
#define PORT_CFG_FEATURES_MF_MODE_SHIFT                         6
#define PORT_CFG_FEATURES_MF_MODE_MF_ALLOWED                    (0x0L << 6)
#define PORT_CFG_FEATURES_MF_MODE_FORCED_SF                     (0x1L << 6)
#define PORT_CFG_FEATURES_MF_MODE_NPAR1_0                       (0x2L << 6)
#define PORT_CFG_FEATURES_MF_MODE_NPAR1_5                       (0x3L << 6)
#define PORT_CFG_FEATURES_MF_MODE_NPAR2_0                       (0x4L << 6)
#define PORT_CFG_FEATURES_MF_MODE_BD                            (0x5L << 6)
#define PORT_CFG_FEATURES_MF_MODE_UFP                           (0x6L << 6)
#define PORT_CFG_FEATURES_MF_MODE_AFEX                          (0x7L << 6)

	} features;

#define PORT_CFG_LINK_SETTINGS_SIZE		(sizeof(struct port_cfg_link_settings)/4)
	struct port_cfg_link_settings {
		u32 speed_cap_mask;
#define SPEED_CAPABILITY_DRV_MASK                               0x0000FFFF
#define SPEED_CAPABILITY_DRV_SHIFT                              0
#define SPEED_CAPABILITY_DRV_1G                                 0x1
#define SPEED_CAPABILITY_DRV_10G                                0x2
#define SPEED_CAPABILITY_DRV_25G                                0x4
#define SPEED_CAPABILITY_DRV_40G                                0x8
#define SPEED_CAPABILITY_DRV_50G                                0x10
#define SPEED_CAPABILITY_DRV_100M                               0x8000
#define SPEED_CAPABILITY_FW_MASK                                0xFFFF0000
#define SPEED_CAPABILITY_FW_SHIFT                               16
#define SPEED_CAPABILITY_FW_1G                                  (0x1L << 16)
#define SPEED_CAPABILITY_FW_10G                                 (0x2L << 16)
#define SPEED_CAPABILITY_FW_25G                                 (0x4L << 16)
#define SPEED_CAPABILITY_FW_40G                                 (0x8L << 16)
#define SPEED_CAPABILITY_FW_50G                                 (0x10L << 16)
#define SPEED_CAPABILITY_FW_100M                                (0x8000UL << 16)

		u32 link_settings;
#define LINK_SPEED_DRV_MASK                                     0x0000000F
#define LINK_SPEED_DRV_SHIFT                                    0
#define LINK_SPEED_DRV_AUTONEG                                  0x0
#define LINK_SPEED_DRV_1G                                       0x1
#define LINK_SPEED_DRV_10G                                      0x2
#define LINK_SPEED_DRV_25G                                      0x3
#define LINK_SPEED_DRV_40G                                      0x4
#define LINK_SPEED_DRV_50G                                      0x5
#define LINK_SPEED_DRV_100M                                     0xF
#define FLOW_CONTROL_DRV_MASK                                   0x00000070
#define FLOW_CONTROL_DRV_SHIFT                                  4
#define FLOW_CONTROL_DRV_AUTO                                   (0x0L << 4)
#define FLOW_CONTROL_DRV_TX                                     (0x1L << 4)
#define FLOW_CONTROL_DRV_RX                                     (0x2L << 4)
#define FLOW_CONTROL_DRV_BOTH                                   (0x3L << 4)
#define FLOW_CONTROL_DRV_NONE                                   (0x4L << 4)
#define LINK_SPEED_FW_MASK                                      0x00000780
#define LINK_SPEED_FW_SHIFT                                     7
#define LINK_SPEED_FW_AUTONEG                                   (0x0L << 7)
#define LINK_SPEED_FW_1G                                        (0x1L << 7)
#define LINK_SPEED_FW_10G                                       (0x2L << 7)
#define LINK_SPEED_FW_25G                                       (0x3L << 7)
#define LINK_SPEED_FW_40G                                       (0x4L << 7)
#define LINK_SPEED_FW_50G                                       (0x5L << 7)
#define LINK_SPEED_FW_100M                                      (0xFL << 7)
#define FLOW_CONTROL_FW_MASK                                    0x00003800
#define FLOW_CONTROL_FW_SHIFT                                   11
#define FLOW_CONTROL_FW_AUTO                                    (0x0L << 11)
#define FLOW_CONTROL_FW_TX                                      (0x1L << 11)
#define FLOW_CONTROL_FW_RX                                      (0x2L << 11)
#define FLOW_CONTROL_FW_BOTH                                    (0x3L << 11)
#define FLOW_CONTROL_FW_NONE                                    (0x4L << 11)
#define OPTIC_MDL_VNDR_ENF_MASK                                 0x0000C000
#define OPTIC_MDL_VNDR_ENF_SHIFT                                14
#define OPTIC_MDL_VNDR_ENF_NO_ENFORCEMENT                       (0x0L << 14)
#define OPTIC_MDL_VNDR_ENF_DISABLE_TX_LASER                     (0x1L << 14)
#define OPTIC_MDL_VNDR_ENF_WARNING_MSG                          (0x2L << 14)
#define OPTIC_MDL_VNDR_ENF_POWER_DOWN                           (0x3L << 14)
#define D3_LINK_SPEED_FW_MASK                                   0x000F0000
#define D3_LINK_SPEED_FW_SHIFT                                  16
#define D3_LINK_SPEED_FW_AUTONEG                                (0x0L << 16)
#define D3_LINK_SPEED_FW_1G                                     (0x1L << 16)
#define D3_LINK_SPEED_FW_10G                                    (0x2L << 16)
#define D3_LINK_SPEED_FW_25G                                    (0x3L << 16)
#define D3_LINK_SPEED_FW_40G                                    (0x4L << 16)
#define D3_LINK_SPEED_FW_50G                                    (0x5L << 16)
#define D3_LINK_SPEED_FW_100M                                   (0xFL << 16)
#define D3_FLOW_CONTROL_FW_MASK                                 0x00700000
#define D3_FLOW_CONTROL_FW_SHIFT                                20
#define D3_FLOW_CONTROL_FW_AUTO                                 (0x0L << 20)
#define D3_FLOW_CONTROL_FW_TX                                   (0x1L << 20)
#define D3_FLOW_CONTROL_FW_RX                                   (0x2L << 20)
#define D3_FLOW_CONTROL_FW_BOTH                                 (0x3L << 20)
#define D3_FLOW_CONTROL_FW_NONE                                 (0x4L << 20)

		u32 eee_power_mode;
#define EEE_POWER_MODE_MASK                                     0x00000001
#define EEE_POWER_MODE_SHIFT                                    0
#define EEE_POWER_MODE_DISABLED                                 0x0
#define EEE_POWER_MODE_ENABLED                                  0x1
#define PORT_CFG_LINK_SETTINGS_RESERVE_MASK                     0x00000002
#define PORT_CFG_LINK_SETTINGS_RESERVE_SHIFT                    1
#define D3_SPEED_CAPABILITY_FW_MASK                             0x0003FFFC
#define D3_SPEED_CAPABILITY_FW_SHIFT                            2
#define D3_SPEED_CAPABILITY_FW_1G                               (0x1L << 2)
#define D3_SPEED_CAPABILITY_FW_10G                              (0x2L << 2)
#define D3_SPEED_CAPABILITY_FW_25G                              (0x4L << 2)
#define D3_SPEED_CAPABILITY_FW_40G                              (0x8L << 2)
#define D3_SPEED_CAPABILITY_FW_50G                              (0x10L << 2)
#define D3_SPEED_CAPABILITY_FW_100M                             (0x8000UL << 2)

	} link_settings;

#define PORT_CFG_MGMT_SIZE		(sizeof(struct port_cfg_mgmt)/4)
	struct port_cfg_mgmt {
		u32 mgmt_traffic;
#define PORT_CFG_MGMT_NC_MSI_OVER_RMII__M_DEPRECATED_MASK       0x00000001
#define PORT_CFG_MGMT_NC_MSI_OVER_RMII__M_DEPRECATED_SHIFT      0
#define PORT_CFG_MGMT_NC_MSI_OVER_SMBUS__M_DEPRECATED_MASK      0x00000002
#define PORT_CFG_MGMT_NC_MSI_OVER_SMBUS__M_DEPRECATED_SHIFT     1
#define PORT_CFG_MGMT_NC_MSI_OVER_PCIEVDM__M_DEPRECATED_MASK    0x00000004
#define PORT_CFG_MGMT_NC_MSI_OVER_PCIEVDM__M_DEPRECATED_SHIFT   2

	} mgmt;

#define PORT_CFG_PHY_SIZE		(sizeof(struct port_cfg_phy)/4)
	struct port_cfg_phy {
		u32 ext_phy;
#define PORT_CFG_PHY_EXTERNAL_PHY_TYPE_MASK                     0x000000FF
#define PORT_CFG_PHY_EXTERNAL_PHY_TYPE_SHIFT                    0
#define PORT_CFG_PHY_EXTERNAL_PHY_TYPE_DIRECT                   0x0
#define PORT_CFG_PHY_EXTERNAL_PHY_TYPE_BCM84856                 0x1
#define PORT_CFG_PHY_EXTERNAL_PHY_ADDRESS_MASK                  0x0000FF00
#define PORT_CFG_PHY_EXTERNAL_PHY_ADDRESS_SHIFT                 8

		u32 phy_cfg;
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_MASK                  0x000000FF
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_SHIFT                 0
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_BYPASS                0x0
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_KR                    0x1
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_KR2                   0x2
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_KR4                   0x3
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_XFI                   0x4
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_SFI                   0x5
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_1000X                 0x6
#define PORT_CFG_PHY_SERDES_NET_INTERFACE_SGMII                 0x7
#define PORT_CFG_PHY_AN_MODE_MASK                               0x0000FF00
#define PORT_CFG_PHY_AN_MODE_SHIFT                              8
#define PORT_CFG_PHY_AN_MODE_NONE                               (0x0L << 8)
#define PORT_CFG_PHY_AN_MODE_CL73                               (0x1L << 8)
#define PORT_CFG_PHY_AN_MODE_CL37                               (0x2L << 8)
#define PORT_CFG_PHY_AN_MODE_CL73_BAM                           (0x3L << 8)
#define PORT_CFG_PHY_AN_MODE_CL37_BAM                           (0x4L << 8)
#define PORT_CFG_PHY_AN_MODE_SGMII                              (0x5L << 8)
#define PORT_CFG_PHY_OPTICAL_MODULE_I2C_SELECTION_MASK          0x00FF0000
#define PORT_CFG_PHY_OPTICAL_MODULE_I2C_SELECTION_SHIFT         16
#define PORT_CFG_PHY_OPTICAL_MODULE_I2C_SELECTION_0             (0x0L << 16)
#define PORT_CFG_PHY_OPTICAL_MODULE_I2C_SELECTION_1             (0x1L << 16)
#define PORT_CFG_PHY_OPTICAL_MODULE_I2C_SELECTION_2             (0x2L << 16)
#define PORT_CFG_PHY_OPTICAL_MODULE_I2C_SELECTION_N_A           (0xFFL << 16)

		u32 multi_phy_config;
#define PHY_SELECTION_MASK                                      0x00000007
#define PHY_SELECTION_SHIFT                                     0
#define PHY_SELECTION_HARDWARE_DEFAULT                          0x0
#define PHY_SELECTION_FIRST_PHY                                 0x1
#define PHY_SELECTION_SECOND_PHY                                0x2
#define PHY_SELECTION_FIRST_PHY_PRIORITY                        0x3
#define PHY_SELECTION_SECOND_PHY_PRIORITY                       0x4
#define PHY_SWAPPED_MASK                                        0x00000008
#define PHY_SWAPPED_SHIFT                                       3
#define PHY_SWAPPED_DISABLED                                    (0x0L << 3)
#define PHY_SWAPPED_ENABLED                                     (0x1L << 3)

		u32 lane_config;
#define SWAP_PHY_POLARITY_MASK                                  0x00000001
#define SWAP_PHY_POLARITY_SHIFT                                 0
#define SWAP_PHY_POLARITY_DISABLED                              0x0
#define SWAP_PHY_POLARITY_ENABLED                               0x1

		u32 cmn_pin_cfg;
#define PHY_RESET_MASK                                          0x000000FF
#define PHY_RESET_SHIFT                                         0
#define TX_FAULT_MASK                                           0x0000FF00
#define TX_FAULT_SHIFT                                          8

		u32 xgbt_phy_cfg;
#define COPPER_PAIR_SWAP_MASK                                   0x000000FF
#define COPPER_PAIR_SWAP_SHIFT                                  0
#define ENABLE_CMS_MASK                                         0x00000100
#define ENABLE_CMS_SHIFT                                        8
#define ENABLE_CMS_DISABLED                                     (0x0L << 8)
#define ENABLE_CMS_ENABLED                                      (0x1L << 8)

		u32 sfp_ctrl;
#define TX_LASER_MASK                                           0x000000FF
#define TX_LASER_SHIFT                                          0
#define MOD_ABS_MASK                                            0x0000FF00
#define MOD_ABS_SHIFT                                           8
#define PWR_DIS_MASK                                            0x00FF0000
#define PWR_DIS_SHIFT                                           16

		u32 qsfp_ctrl;
#define WRONG_MOD_TYPE_MASK                                     0x000000FF
#define WRONG_MOD_TYPE_SHIFT                                    0
#define CURRENT_FAULT_MASK                                      0x0000FF00
#define CURRENT_FAULT_SHIFT                                     8
#define QSFP_LP_MODE_MASK                                       0x00FF0000
#define QSFP_LP_MODE_SHIFT                                      16
#define QSFP_RESET_MASK                                         0xFF000000
#define QSFP_RESET_SHIFT                                        24

	} phy;

#define PORT_CFG_PRE_BOOT_SIZE	0

#define PORT_CFG_VF_SIZE	0

#define PORT_CFG_DUAL_PHY_SIZE		(sizeof(struct port_cfg_dual_phy)/4)
	struct port_cfg_dual_phy {
		u32 ext_phy2_cfg;
#define EXT_PHY2_TYPE_MASK                                      0x000000FF
#define EXT_PHY2_TYPE_SHIFT                                     0
#define EXT_PHY2_TYPE_NONE                                      0x0
#define EXT_PHY2_TYPE_DIRECT                                    0x1
#define EXT_PHY2_TYPE_BCM84856                                  0x2
#define EXT_PHY2_ADDR_MASK                                      0x0000FF00
#define EXT_PHY2_ADDR_SHIFT                                     8

		u32 secondary_tx_pre_memphasis_coef_[2];

		u32 secondary_rx_equalizer_coef__[2];

		u32 speed_cap_mask2;
#define SPEED_CAPABILITY2_D0_MASK                               0x0000FFFF
#define SPEED_CAPABILITY2_D0_SHIFT                              0
#define SPEED_CAPABILITY2_D0_1G                                 0x1
#define SPEED_CAPABILITY2_D0_10G                                0x2
#define SPEED_CAPABILITY2_D0_25G                                0x4
#define SPEED_CAPABILITY2_D0_40G                                0x8
#define SPEED_CAPABILITY2_D0_50G                                0x10
#define SPEED_CAPABILITY2_D0_100M                               0x8000
#define SPEED_CAPABILITY2_D3_MASK                               0xFFFF0000
#define SPEED_CAPABILITY2_D3_SHIFT                              16
#define SPEED_CAPABILITY2_D3_1G                                 (0x1L << 16)
#define SPEED_CAPABILITY2_D3_10G                                (0x2L << 16)
#define SPEED_CAPABILITY2_D3_25G                                (0x4L << 16)
#define SPEED_CAPABILITY2_D3_40G                                (0x8L << 16)
#define SPEED_CAPABILITY2_D3_50G                                (0x10L << 16)
#define SPEED_CAPABILITY2_D3_100M                               (0x8000UL << 16)

		u32 link_config;
#define LINK_SPEED2_MASK                                        0x0000000F
#define LINK_SPEED2_SHIFT                                       0
#define LINK_SPEED2_AUTONEG                                     0x0
#define LINK_SPEED2_1G                                          0x1
#define LINK_SPEED2_10G                                         0x2
#define LINK_SPEED2_25G                                         0x3
#define LINK_SPEED2_40G                                         0x4
#define LINK_SPEED2_50G                                         0x5
#define LINK_SPEED2_100M                                        0xF
#define FLOW_CONTROL2_MASK                                      0x00000070
#define FLOW_CONTROL2_SHIFT                                     4
#define FLOW_CONTROL2_AUTO                                      (0x0L << 4)
#define FLOW_CONTROL2_TX                                        (0x1L << 4)
#define FLOW_CONTROL2_RX                                        (0x2L << 4)
#define FLOW_CONTROL2_BOTH                                      (0x3L << 4)
#define FLOW_CONTROL2_NONE                                      (0x4L << 4)

		u32 fw_wol_link_cfg;
#define FW_LINK_SPEED2_MASK                                     0x0000000F
#define FW_LINK_SPEED2_SHIFT                                    0
#define FW_LINK_SPEED2_AUTONEG                                  0x0
#define FW_LINK_SPEED2_1G                                       0x1
#define FW_LINK_SPEED2_10G                                      0x2
#define FW_LINK_SPEED2_25G                                      0x3
#define FW_LINK_SPEED2_40G                                      0x4
#define FW_LINK_SPEED2_50G                                      0x5
#define FW_LINK_SPEED2_100M                                     0xF
#define FW_FLOW_CONTROL2_MASK                                   0x00000070
#define FW_FLOW_CONTROL2_SHIFT                                  4
#define FW_FLOW_CONTROL2_AUTO                                   (0x0L << 4)
#define FW_FLOW_CONTROL2_TX                                     (0x1L << 4)
#define FW_FLOW_CONTROL2_RX                                     (0x2L << 4)
#define FW_FLOW_CONTROL2_BOTH                                   (0x3L << 4)
#define FW_FLOW_CONTROL2_NONE                                   (0x4L << 4)

		u32 default_cfg;
#define NET_SERDES_IF_MASK                                      0x0000000F
#define NET_SERDES_IF_SHIFT                                     0
#define NET_SERDES_IF_BYPASS                                    0x0
#define NET_SERDES_IF_KR                                        0x1
#define NET_SERDES_IF_KR2                                       0x2
#define NET_SERDES_IF_KR4                                       0x3
#define NET_SERDES_IF_XFI                                       0x4
#define NET_SERDES_IF_SFI                                       0x5
#define NET_SERDES_IF_1000X                                     0x6
#define NET_SERDES_IF_SGMII                                     0x7

	} dual_phy;

#define PORT_CFG_NPAR_SIZE		(sizeof(struct port_cfg_npar)/4)
	struct port_cfg_npar {
		u32 port_cfg;
#define PORT_CFG_NPAR_NUMBER_OF_PARTITIONS_PER_PORT_MASK        0x000000FF
#define PORT_CFG_NPAR_NUMBER_OF_PARTITIONS_PER_PORT_SHIFT       0

	} npar;

#define PORT_CFG_TEMPERATURE_CONTROL_SIZE	0

#define PORT_CFG_EMBEDDED_BMC_SIZE	(sizeof(struct port_cfg_embedded_bmc)/4)
	struct port_cfg_embedded_bmc {
		nvm_cfg_mac_address_t bmc_mac_address;

		u32 reserved[6];

	} embedded_bmc;

	u32 reserved[129];	/* 0x94 */

	u32 crc;

} port_cfg_t;

/******************************************
 * func_cfg structs
 ******************************************/

typedef struct func_cfg {
	u32 num_sections;
	u32 sections_offset[NVM_CFG_SECTION_MAX];

#define FUNC_CFG_BOARD_SIZE		(sizeof(struct func_cfg_board)/4)
	struct func_cfg_board {
		nvm_cfg_mac_address_t mac_address;

	} board;

#define FUNC_CFG_BOARD_IO_SIZE	0

#define FUNC_CFG_PCIE_SIZE		(sizeof(struct func_cfg_pcie)/4)
	struct func_cfg_pcie {
		u32 mf_pci_id;
#define FUNC_CFG_PCIE_MF_VENDOR_DEVICE_ID_MASK                  0x0000FFFF
#define FUNC_CFG_PCIE_MF_VENDOR_DEVICE_ID_SHIFT                 0

		u32 kcs_pci_cfg;
#define FUNC_CFG_PCIE_KCS_DEVICE_ID_MASK                        0x0000FFFF
#define FUNC_CFG_PCIE_KCS_DEVICE_ID_SHIFT                       0
#define FUNC_CFG_PCIE_KCS_MODE_MASK                             0x00010000
#define FUNC_CFG_PCIE_KCS_MODE_SHIFT                            16
#define FUNC_CFG_PCIE_KCS_MODE_DISABLED                         (0x0L << 16)
#define FUNC_CFG_PCIE_KCS_MODE_ENABLED                          (0x1L << 16)

		u32 uart_pci_cfg;
#define FUNC_CFG_PCIE_UART_DEVICE_ID_MASK                       0x0000FFFF
#define FUNC_CFG_PCIE_UART_DEVICE_ID_SHIFT                      0
#define FUNC_CFG_PCIE_UART_MODE_MASK                            0x00010000
#define FUNC_CFG_PCIE_UART_MODE_SHIFT                           16
#define FUNC_CFG_PCIE_UART_MODE_DISABLED                        (0x0L << 16)
#define FUNC_CFG_PCIE_UART_MODE_ENABLED                         (0x1L << 16)

	} pcie;

#define FUNC_CFG_FEATURES_SIZE	0

#define FUNC_CFG_LINK_SETTINGS_SIZE	0

#define FUNC_CFG_MGMT_SIZE	0

#define FUNC_CFG_PHY_SIZE	0

#define FUNC_CFG_PRE_BOOT_SIZE		(sizeof(struct func_cfg_pre_boot)/4)
	struct func_cfg_pre_boot {
		u32 mba_cfg1;
#define FUNC_CFG_PRE_BOOT_MBA_MASK                              0x00000001
#define FUNC_CFG_PRE_BOOT_MBA_SHIFT                             0
#define FUNC_CFG_PRE_BOOT_MBA_DISABLED                          0x0
#define FUNC_CFG_PRE_BOOT_MBA_ENABLED                           0x1
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_TYPE_MASK                    0x00000006
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_TYPE_SHIFT                   1
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_TYPE_AUTO                    (0x0L << 1)
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_TYPE_BBS                     (0x1L << 1)
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_TYPE_INT18H                  (0x2L << 1)
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_TYPE_INT19H                  (0x3L << 1)
#define FUNC_CFG_PRE_BOOT_MBA_DELAY_TIME_MASK                   0x00000078
#define FUNC_CFG_PRE_BOOT_MBA_DELAY_TIME_SHIFT                  3
#define FUNC_CFG_PRE_BOOT_MBA_SETUP_HOT_KEY_MASK                0x00000080
#define FUNC_CFG_PRE_BOOT_MBA_SETUP_HOT_KEY_SHIFT               7
#define FUNC_CFG_PRE_BOOT_MBA_SETUP_HOT_KEY_CTRL_S              (0x0L << 7)
#define FUNC_CFG_PRE_BOOT_MBA_SETUP_HOT_KEY_CTRL_B              (0x1L << 7)
#define FUNC_CFG_PRE_BOOT_MBA_HIDE_SETUP_PROMPT_MASK            0x00000100
#define FUNC_CFG_PRE_BOOT_MBA_HIDE_SETUP_PROMPT_SHIFT           8
#define FUNC_CFG_PRE_BOOT_MBA_HIDE_SETUP_PROMPT_DISABLED        (0x0L << 8)
#define FUNC_CFG_PRE_BOOT_MBA_HIDE_SETUP_PROMPT_ENABLED         (0x1L << 8)
#define FUNC_CFG_PRE_BOOT_MBA_LINK_SPEED_MASK                   0x00001E00
#define FUNC_CFG_PRE_BOOT_MBA_LINK_SPEED_SHIFT                  9
#define FUNC_CFG_PRE_BOOT_MBA_LINK_SPEED_AUTONEG                (0x0L << 9)
#define FUNC_CFG_PRE_BOOT_MBA_LINK_SPEED_1G                     (0x1L << 9)
#define FUNC_CFG_PRE_BOOT_MBA_LINK_SPEED_10G                    (0x2L << 9)
#define FUNC_CFG_PRE_BOOT_MBA_LINK_SPEED_25G                    (0x3L << 9)
#define FUNC_CFG_PRE_BOOT_MBA_LINK_SPEED_40G                    (0x4L << 9)
#define FUNC_CFG_PRE_BOOT_MBA_LINK_SPEED_50G                    (0x5L << 9)
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_RETRY_COUNT_MASK             0x0000E000
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_RETRY_COUNT_SHIFT            13
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_PROTOCOL_MASK                0x00070000
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_PROTOCOL_SHIFT               16
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_PROTOCOL_PXE                 (0x0L << 16)
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_PROTOCOL_ISCSI_BOOT          (0x1L << 16)
#define FUNC_CFG_PRE_BOOT_MBA_BOOT_PROTOCOL_NONE                (0x7L << 16)
#define FUNC_CFG_PRE_BOOT_FORCE_EXPANSION_ROM_ADV_MASK          0x00080000
#define FUNC_CFG_PRE_BOOT_FORCE_EXPANSION_ROM_ADV_SHIFT         19
#define FUNC_CFG_PRE_BOOT_FORCE_EXPANSION_ROM_ADV_DISABLED      (0x0L << 19)
#define FUNC_CFG_PRE_BOOT_FORCE_EXPANSION_ROM_ADV_ENABLED       (0x1L << 19)

		u32 mba_cfg2;
#define FUNC_CFG_PRE_BOOT_MBA_VLAN_VALUE_MASK                   0x0000FFFF
#define FUNC_CFG_PRE_BOOT_MBA_VLAN_VALUE_SHIFT                  0
#define FUNC_CFG_PRE_BOOT_MBA_VLAN_MASK                         0x00010000
#define FUNC_CFG_PRE_BOOT_MBA_VLAN_SHIFT                        16
#define FUNC_CFG_PRE_BOOT_MBA_VLAN_DISABLED                     (0x0L << 16)
#define FUNC_CFG_PRE_BOOT_MBA_VLAN_ENABLED                      (0x1L << 16)

	} pre_boot;

#define FUNC_CFG_VF_SIZE		(sizeof(struct func_cfg_VF)/4)
	struct func_cfg_VF {
		u32 vf_cfg;
#define FUNC_CFG_VF_VF_PCI_DEVICE_ID_MASK                       0x0000FFFF
#define FUNC_CFG_VF_VF_PCI_DEVICE_ID_SHIFT                      0
#define FUNC_CFG_VF_VF_BAR1_SIZE_MASK                           0x000F0000
#define FUNC_CFG_VF_VF_BAR1_SIZE_SHIFT                          16
#define FUNC_CFG_VF_VF_BAR2_SIZE_MASK                           0x00F00000
#define FUNC_CFG_VF_VF_BAR2_SIZE_SHIFT                          20
#define FUNC_CFG_VF_VF_BAR3_SIZE_MASK                           0x0F000000
#define FUNC_CFG_VF_VF_BAR3_SIZE_SHIFT                          24
#define FUNC_CFG_VF_VF_BAR_SIZE_DISABLED                        0x0
#define FUNC_CFG_VF_VF_BAR_SIZE_4K                              0x1
#define FUNC_CFG_VF_VF_BAR_SIZE_8K                              0x2
#define FUNC_CFG_VF_VF_BAR_SIZE_16K                             0x3
#define FUNC_CFG_VF_VF_BAR_SIZE_32K                             0x4
#define FUNC_CFG_VF_VF_BAR_SIZE_64K                             0x5
#define FUNC_CFG_VF_VF_BAR_SIZE_128K                            0x6
#define FUNC_CFG_VF_VF_BAR_SIZE_256K                            0x7
#define FUNC_CFG_VF_VF_BAR_SIZE_512K                            0x8
#define FUNC_CFG_VF_VF_BAR_SIZE_1M                              0x9
#define FUNC_CFG_VF_VF_BAR_SIZE_2M                              0xA
#define FUNC_CFG_VF_VF_BAR_SIZE_4M                              0xB
#define FUNC_CFG_VF_VF_BAR_SIZE_8M                              0xC
#define FUNC_CFG_VF_VF_BAR_SIZE_16M                             0xD
#define FUNC_CFG_VF_VF_BAR_SIZE_32M                             0xE
#define FUNC_CFG_VF_VF_BAR_SIZE_64M                             0xF

		u32 vf_cfg2;
#define FUNC_CFG_VF_NUMBER_OF_VFS_PER_PF_MASK                   0x000000FF
#define FUNC_CFG_VF_NUMBER_OF_VFS_PER_PF_SHIFT                  0
#define FUNC_CFG_VF_NUMBER_OF_VNICS_PER_PF_MASK                 0x0001FF00
#define FUNC_CFG_VF_NUMBER_OF_VNICS_PER_PF_SHIFT                8
#define FUNC_CFG_VF_MSI_MX_VECTORS_PER_VF_MASK                  0x07FE0000
#define FUNC_CFG_VF_MSI_MX_VECTORS_PER_VF_SHIFT                 17

	} VF;

#define FUNC_CFG_DUAL_PHY_SIZE	0

#define FUNC_CFG_NPAR_SIZE		(sizeof(struct func_cfg_npar)/4)
	struct func_cfg_npar {
		u32 func_cfg;
#define BW_WEIGHT_MASK                                          0x000000FF
#define BW_WEIGHT_SHIFT                                         0
#define BW_MAX_MASK                                             0x00FFFF00
#define BW_MAX_SHIFT                                            8

	} npar;

#define FUNC_CFG_TEMPERATURE_CONTROL_SIZE	0

#define FUNC_CFG_EMBEDDED_BMC_SIZE		(sizeof(struct func_cfg_embedded_bmc)/4)
	struct func_cfg_embedded_bmc {
		u32 reserved[4];

	} embedded_bmc;

	u32 reserved[152];	/* 0x38 */

	u32 crc;

} func_cfg_t;

/* Hard-coded values for initial NVRAM structure offset/length metadata values */
#define SHARED_CFG_BOARD_OFFSET_V0			0x84
#define SHARED_CFG_BOARD_SIZE_V0			0x6
#define SHARED_CFG_BOARD_IO_OFFSET_V0			0x9c
#define SHARED_CFG_BOARD_IO_SIZE_V0			0x1
#define SHARED_CFG_PCIE_OFFSET_V0			0xa0
#define SHARED_CFG_PCIE_SIZE_V0				0x1
#define SHARED_CFG_FEATURES_OFFSET_V0			0xa4
#define SHARED_CFG_FEATURES_SIZE_V0			0x2
#define SHARED_CFG_LINK_SETTINGS_OFFSET_V0		0xac
#define SHARED_CFG_LINK_SETTINGS_SIZE_V0		0x0
#define SHARED_CFG_MGMT_OFFSET_V0			0xac
#define SHARED_CFG_MGMT_SIZE_V0				0x3
#define SHARED_CFG_PHY_OFFSET_V0			0xb8
#define SHARED_CFG_PHY_SIZE_V0				0x5
#define SHARED_CFG_PRE_BOOT_OFFSET_V0			0xcc
#define SHARED_CFG_PRE_BOOT_SIZE_V0			0x1
#define SHARED_CFG_VF_OFFSET_V0				0xd0
#define SHARED_CFG_VF_SIZE_V0				0x1
#define SHARED_CFG_DUAL_PHY_OFFSET_V0			0xd4
#define SHARED_CFG_DUAL_PHY_SIZE_V0			0x0
#define SHARED_CFG_NPAR_OFFSET_V0			0xd4
#define SHARED_CFG_NPAR_SIZE_V0				0x0
#define SHARED_CFG_TEMPERATURE_CONTROL_OFFSET_V0	0xd4
#define SHARED_CFG_TEMPERATURE_CONTROL_SIZE_V0		0x3
#define SHARED_CFG_EMBEDDED_BMC_OFFSET_V0		0xe0
#define SHARED_CFG_EMBEDDED_BMC_SIZE_V0		0x4

#define PORT_CFG_BOARD_OFFSET_V0			0x84
#define PORT_CFG_BOARD_SIZE_V0				0x5
#define PORT_CFG_BOARD_IO_OFFSET_V0			0x98
#define PORT_CFG_BOARD_IO_SIZE_V0			0x0
#define PORT_CFG_PCIE_OFFSET_V0				0x98
#define PORT_CFG_PCIE_SIZE_V0				0x1
#define PORT_CFG_FEATURES_OFFSET_V0			0x9c
#define PORT_CFG_FEATURES_SIZE_V0			0x1
#define PORT_CFG_LINK_SETTINGS_OFFSET_V0		0xa0
#define PORT_CFG_LINK_SETTINGS_SIZE_V0		0x3
#define PORT_CFG_MGMT_OFFSET_V0				0xac
#define PORT_CFG_MGMT_SIZE_V0				0x1
#define PORT_CFG_PHY_OFFSET_V0				0xb0
#define PORT_CFG_PHY_SIZE_V0				0x8
#define PORT_CFG_PRE_BOOT_OFFSET_V0			0xd0
#define PORT_CFG_PRE_BOOT_SIZE_V0			0x0
#define PORT_CFG_VF_OFFSET_V0				0xd0
#define PORT_CFG_VF_SIZE_V0				0x0
#define PORT_CFG_DUAL_PHY_OFFSET_V0			0xd0
#define PORT_CFG_DUAL_PHY_SIZE_V0			0x9
#define PORT_CFG_NPAR_OFFSET_V0				0xf4
#define PORT_CFG_NPAR_SIZE_V0				0x1
#define PORT_CFG_TEMPERATURE_CONTROL_OFFSET_V0	0xf8
#define PORT_CFG_TEMPERATURE_CONTROL_SIZE_V0	0x0
#define PORT_CFG_EMBEDDED_BMC_OFFSET_V0		0xf8
#define PORT_CFG_EMBEDDED_BMC_SIZE_V0			0x8

#define FUNC_CFG_BOARD_OFFSET_V0			0x84
#define FUNC_CFG_BOARD_SIZE_V0				0x2
#define FUNC_CFG_BOARD_IO_OFFSET_V0			0x8c
#define FUNC_CFG_BOARD_IO_SIZE_V0			0x0
#define FUNC_CFG_PCIE_OFFSET_V0				0x8c
#define FUNC_CFG_PCIE_SIZE_V0				0x3
#define FUNC_CFG_FEATURES_OFFSET_V0			0x98
#define FUNC_CFG_FEATURES_SIZE_V0			0x0
#define FUNC_CFG_LINK_SETTINGS_OFFSET_V0		0x98
#define FUNC_CFG_LINK_SETTINGS_SIZE_V0		0x0
#define FUNC_CFG_MGMT_OFFSET_V0				0x98
#define FUNC_CFG_MGMT_SIZE_V0				0x0
#define FUNC_CFG_PHY_OFFSET_V0				0x98
#define FUNC_CFG_PHY_SIZE_V0				0x0
#define FUNC_CFG_PRE_BOOT_OFFSET_V0			0x98
#define FUNC_CFG_PRE_BOOT_SIZE_V0			0x2
#define FUNC_CFG_VF_OFFSET_V0				0xa0
#define FUNC_CFG_VF_SIZE_V0				0x2
#define FUNC_CFG_DUAL_PHY_OFFSET_V0			0xa8
#define FUNC_CFG_DUAL_PHY_SIZE_V0			0x0
#define FUNC_CFG_NPAR_OFFSET_V0				0xa8
#define FUNC_CFG_NPAR_SIZE_V0				0x1
#define FUNC_CFG_TEMPERATURE_CONTROL_OFFSET_V0	0xac
#define FUNC_CFG_TEMPERATURE_CONTROL_SIZE_V0	0x0
#define FUNC_CFG_EMBEDDED_BMC_OFFSET_V0		0xac
#define FUNC_CFG_EMBEDDED_BMC_SIZE_V0			0x4

#define NVM_CFG_SECTION_LEN_OFF(x, y)		(x << NVM_CFG_SECTION_LEN_SHIFT | y)

#endif /* NVM_CFG_H */
