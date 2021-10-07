/*
 *         
 * $Id: phymod.xml,v 1.1.2.5 2013/09/12 10:43:06 nirf Exp $
 * 
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


#ifndef _PHYMOD_H__H_
#define _PHYMOD_H__H_

#include <phymod/phymod_definitions.h>

/*!
 * @enum phymod_dispatch_type_e
 * @brief Supported Drivers 
 */ 
typedef enum phymod_dispatch_type_e {
#ifdef PHYMOD_EAGLE_SUPPORT
    phymodDispatchTypeEagle,
#endif /*PHYMOD_EAGLE_SUPPORT  */
#ifdef PHYMOD_FALCON_SUPPORT
    phymodDispatchTypeFalcon,
#endif /*PHYMOD_FALCON_SUPPORT  */
#ifdef PHYMOD_QSGMIIE_SUPPORT
    phymodDispatchTypeQsgmiie,
#endif /*PHYMOD_QSGMIIE_SUPPORT  */
#ifdef PHYMOD_TSCE_SUPPORT
    phymodDispatchTypeTsce,
#endif /*PHYMOD_TSCE_SUPPORT  */
#ifdef PHYMOD_TSCF_SUPPORT
    phymodDispatchTypeTscf,
#endif /*PHYMOD_TSCF_SUPPORT  */
#ifdef PHYMOD_PHY8806X_SUPPORT
    phymodDispatchTypePhy8806x,
#endif /*PHYMOD_PHY8806X_SUPPORT  */
#ifdef PHYMOD_FURIA_SUPPORT
    phymodDispatchTypeFuria,
#endif /*PHYMOD_FURIA_SUPPORT  */
#ifdef PHYMOD_VIPER_SUPPORT
    phymodDispatchTypeViper,
#endif /*PHYMOD_VIPER_SUPPORT  */
#ifdef PHYMOD_SESTO_SUPPORT
    phymodDispatchTypeSesto,
#endif /*PHYMOD_SESTO_SUPPORT  */
#ifdef PHYMOD_QUADRA28_SUPPORT
    phymodDispatchTypeQuadra28,
#endif /*PHYMOD_QUADRA28_SUPPORT  */
#ifdef PHYMOD_QTCE_SUPPORT
    phymodDispatchTypeQtce,
#endif /*PHYMOD_QTCE_SUPPORT  */
#ifdef PHYMOD_HURACAN_SUPPORT
    phymodDispatchTypeHuracan,
#endif /*PHYMOD_HURACAN_SUPPORT  */
#ifdef PHYMOD_MADURA_SUPPORT
    phymodDispatchTypeMadura,
#endif /*PHYMOD_MADURA_SUPPORT  */
#ifdef PHYMOD_FURIA_SUPPORT
    phymodDispatchTypeFuria_82212,
#endif /*PHYMOD_FURIA_SUPPORT  */
#ifdef PHYMOD_DINO_SUPPORT
    phymodDispatchTypeDino,
#endif /*PHYMOD_DINO_SUPPORT  */
    phymodDispatchTypeCount
} phymod_dispatch_type_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_dispatch_type_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_dispatch_type_t validation */
int phymod_dispatch_type_t_validate(phymod_dispatch_type_t phymod_dispatch_type);
#define PHYMOD_MAX_LANES_PER_CORE PHYMOD_CONFIG_MAX_LANES_PER_CORE

/*! 
 * phymod_bus_read_f
 *
 * @brief function definition for register read operations 
 *
 * @param [inout] user_acc        - Optional application data
 * @param [in]  core_addr       - Core address
 * @param [in]  reg_addr        - address to read
 * @param [out]  val             - read value
 */
typedef int (*phymod_bus_read_f)(void* user_acc, uint32_t core_addr, uint32_t reg_addr, uint32_t* val);

/*! 
 * phymod_bus_write_f
 *
 * @brief function definition for register write operations 
 *
 * @param [inout] user_acc        - Optional application data
 * @param [in]  core_addr       - Core address
 * @param [in]  reg_addr        - address to write
 * @param [in]  val             - write value
 */
typedef int (*phymod_bus_write_f)(void* user_acc, uint32_t core_addr, uint32_t reg_addr, uint32_t val);

/*! 
 * phymod_bus_write_disabled_f
 *
 * @brief function definition for finding out if wormboot  
 *
 * @param [inout] user_acc        - Optional application data
 * @param [out]  val             - val - 0 if writes are enabled, 1 otherwise
 */
typedef int (*phymod_bus_write_disabled_f)(void* user_acc, uint32_t* val);

/*! 
 * phymod_bus_mutex_take_f
 *
 * @brief function definition for take bus mutex 
 *
 * @param [inout] user_acc        - Optional application data
 */
typedef int (*phymod_bus_mutex_take_f)(void* user_acc);

/*! 
 * phymod_bus_mutex_give_f
 *
 * @brief function definition for give bus mutex 
 *
 * @param [inout] user_acc        - Optional application data
 */
typedef int (*phymod_bus_mutex_give_f)(void* user_acc);

/*! 
 * @brief Bus capabilities 
 */ 
#define PHYMOD_BUS_CAP_WR_MODIFY 0x1 /**< The bus support modify functionality */
#define PHYMOD_BUS_CAP_LANE_CTRL 0x2 /**< The bus support direct lane configuration */

#define PHYMOD_BUS_CAP_WR_MODIFY_SET(bus) ((bus)->bus_capabilities |= PHYMOD_BUS_CAP_WR_MODIFY)
#define PHYMOD_BUS_CAP_LANE_CTRL_SET(bus) ((bus)->bus_capabilities |= PHYMOD_BUS_CAP_LANE_CTRL)

#define PHYMOD_BUS_CAP_WR_MODIFY_CLR(bus) ((bus)->bus_capabilities &= ~PHYMOD_BUS_CAP_WR_MODIFY)
#define PHYMOD_BUS_CAP_LANE_CTRL_CLR(bus) ((bus)->bus_capabilities &= ~PHYMOD_BUS_CAP_LANE_CTRL)

#define PHYMOD_BUS_CAP_WR_MODIFY_GET(bus) ((bus)->bus_capabilities & PHYMOD_BUS_CAP_WR_MODIFY ? 1 : 0)
#define PHYMOD_BUS_CAP_LANE_CTRL_GET(bus) ((bus)->bus_capabilities & PHYMOD_BUS_CAP_LANE_CTRL ? 1 : 0)

typedef struct phymod_bus_s {
    char* bus_name;
    phymod_bus_read_f read;
    phymod_bus_write_f write;
    phymod_bus_write_disabled_f is_write_disabled;
    phymod_bus_mutex_take_f mutex_take;
    phymod_bus_mutex_give_f mutex_give;
    uint32_t bus_capabilities;
} phymod_bus_t;

/* phymod_bus_t initialization and validation */
int phymod_bus_t_validate(const phymod_bus_t* phymod_bus);
int phymod_bus_t_init(phymod_bus_t* phymod_bus);

/*! 
 * @brief Phymod access flags 
 */ 
#define PHYMOD_ACC_F_CLAUSE45 0x1 /**< Use CL45 to access */
#define PHYMOD_ACC_F_QMODE 0x2 /**< Qmode lane mask access */
#define PHYMOD_ACC_F_PRECONDITION 0x4 /**< Qmode lane mask access */

#define PHYMOD_ACC_F_CLAUSE45_SET(access) ((access)->flags |= PHYMOD_ACC_F_CLAUSE45)
#define PHYMOD_ACC_F_QMODE_SET(access) ((access)->flags |= PHYMOD_ACC_F_QMODE)
#define PHYMOD_ACC_F_PRECONDITION_SET(access) ((access)->flags |= PHYMOD_ACC_F_PRECONDITION)

#define PHYMOD_ACC_F_CLAUSE45_CLR(access) ((access)->flags &= ~PHYMOD_ACC_F_CLAUSE45)
#define PHYMOD_ACC_F_QMODE_CLR(access) ((access)->flags &= ~PHYMOD_ACC_F_QMODE)
#define PHYMOD_ACC_F_PRECONDITION_CLR(access) ((access)->flags &= ~PHYMOD_ACC_F_PRECONDITION)

#define PHYMOD_ACC_F_CLAUSE45_GET(access) ((access)->flags & PHYMOD_ACC_F_CLAUSE45 ? 1 : 0)
#define PHYMOD_ACC_F_QMODE_GET(access) ((access)->flags & PHYMOD_ACC_F_QMODE ? 1 : 0)
#define PHYMOD_ACC_F_PRECONDITION_GET(access) ((access)->flags & PHYMOD_ACC_F_PRECONDITION ? 1 : 0)

#define PHYMOD_ACC_DEVAD_0_OVERRIDE_MASK (0x80000000) /**< bit 31 for Override DEVAD=0 */
#define PHYMOD_ACC_DEVAD_FORCE_MASK (0x40000000) /**< bit 30 Force DEVAD */
#define PHYMOD_ACC_DEVAD_MASK (0x1f) /**< bit [4:0] DEVAD override value */

typedef struct phymod_access_s {
    void* user_acc; /**< Optional application data - not used by PHY driver */
    phymod_bus_t* bus; /**< PHY bus driver */
    uint32_t flags; /**< PHYMOD_ACC_F_xxx flags */
    uint32_t lane_mask; /**< specific lanes bitmap */
    uint32_t addr; /**< PHY address (PHYAD) used by this PHY */
    uint32_t devad; /**< Default clause 45 DEVAD if none are specified */
    uint8_t pll_idx; /**< PLL number to be used. Default to 0 if multi-pll is not supported */
} phymod_access_t;

/* phymod_access_t initialization and validation */
int phymod_access_t_validate(const phymod_access_t* phymod_access);
int phymod_access_t_init(phymod_access_t* phymod_access);

#define PHYMOD_ACC_USER_ACC(access_) ((access_)->user_acc)
#define PHYMOD_ACC_BUS(access_) ((access_)->bus)
#define PHYMOD_ACC_FLAGS(access_) ((access_)->flags)
#define PHYMOD_ACC_LANE_MASK(access_) ((access_)->lane_mask)
#define PHYMOD_ACC_ADDR(access_) ((access_)->addr)
#define PHYMOD_ACC_DEVAD(access_) ((access_)->devad)
#define PHYMOD_ACC_PLLIDX(access_) ((access_)->pll_idx)


/*!
 * @enum phymod_port_loc_e
 * @brief phymod line side/system side port location in the board layout 
 */ 
typedef enum phymod_port_loc_e {
    phymodPortLocDC = 0, /**< Loc/side don't care */
    phymodPortLocLine, /**< Loc/line side */
    phymodPortLocSys, /**< Loc/system side */
    phymodPortCount
} phymod_port_loc_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_port_loc_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_port_loc_t validation */
int phymod_port_loc_t_validate(phymod_port_loc_t phymod_port_loc);
typedef struct phymod_core_access_s {
    phymod_port_loc_t port_loc; /**< phymod line side/system side port location in board layout */
    uint32_t device_op_mode; /**< phy device operation mode for chip speicific designs */
    phymod_access_t access;
    phymod_dispatch_type_t type; /**< Driver Type */
} phymod_core_access_t;

/* phymod_core_access_t initialization and validation */
int phymod_core_access_t_validate(const phymod_core_access_t* phymod_core_access);
int phymod_core_access_t_init(phymod_core_access_t* phymod_core_access);

typedef struct phymod_phy_access_s {
    phymod_port_loc_t port_loc; /**< phymod line side/system side port location in board layout */
    uint32_t device_op_mode; /**< phy device operation mode for chip speicific designs */
    phymod_access_t access;
    phymod_dispatch_type_t type; /**< Driver Type */
} phymod_phy_access_t;

/* phymod_phy_access_t initialization and validation */
int phymod_phy_access_t_validate(const phymod_phy_access_t* phymod_phy_access);
int phymod_phy_access_t_init(phymod_phy_access_t* phymod_phy_access);

typedef struct phymod_port_config_s {
    uint32_t speed;
    uint8_t interface;
    uint8_t sys_lane_count;
    uint8_t ln_lane_count;
    uint8_t an_mode;
    uint8_t an_cl72;
    uint8_t fs_cl72;
    uint8_t an_fec;
    uint8_t port_is_higig;
    uint8_t fiber_pref;
    uint8_t port_mode;
    uint8_t is_bootmaster;
    uint8_t fs_fec;
    uint8_t is_flex;
} phymod_port_config_t;

/* phymod_port_config_t initialization and validation */
int phymod_port_config_t_validate(const phymod_port_config_t* phymod_port_config);
int phymod_port_config_t_init(phymod_port_config_t* phymod_port_config);

/*! 
 * phymod_firmware_loader_f
 *
 * @brief function definition for firmware loading 
 *
 * @param [in]  core            - core access information
 * @param [in]  length          - Firmware length
 * @param [in]  data            - Frimware data
 */
typedef int (*phymod_firmware_loader_f)(const phymod_core_access_t* core, uint32_t length, const uint8_t* data);


/*!
 * @struct phymod_value_override_s
 * @brief The value_override structure is used for values which can be auto-set (enable=0) or override (enable=1 + value) 
 */ 
typedef struct phymod_value_override_s {
    uint32_t enable;
    int32_t value;
} phymod_value_override_t;

/* phymod_value_override_t initialization and validation */
int phymod_value_override_t_validate(const phymod_value_override_t* phymod_value_override);
int phymod_value_override_t_init(phymod_value_override_t* phymod_value_override);

/*! 
 * phymod_core_probe
 *
 * @brief Probe Core 
 *
 * @param [in]  access          - Access information
 * @param [out]  type            - Driver type
 * @param [out]  is_probed       - 1 if core probed successfully, 0 otherwise
 */
int phymod_core_probe(const phymod_access_t* access, phymod_dispatch_type_t* type, int* is_probed);

/*! 
 * phymod_core_identify
 *
 * @brief Initialize phymod module 
 *
 * @param [in]  core            - core access information
 * @param [in]  core_id         - If 0 read the id from the hardware and compare to the driver id. Else compare this parameter to the driver id.
 * @param [out]  is_identified   - 1 if indentification IDs match type, 0 otherwise
 */
int phymod_core_identify(const phymod_core_access_t* core, uint32_t core_id, uint32_t* is_identified);


/*!
 * @enum phymod_core_version_e
 * @brief core version 
 */ 
typedef enum phymod_core_version_e {
    phymodCoreVersionFalconA0 = 0,
    phymodCoreVersionEagleA0,
    phymodCoreVersionQsgmiieA0,
    phymodCoreVersionTsce4A0,
    phymodCoreVersionTsce12A0,
    phymodCoreVersionTscfA0,
    phymodCoreVersionTscfB0,
    phymodCoreVersionPhy8806x,
    phymodCoreVersionFuriaA2,
    phymodCoreVersionViperXA0,
    phymodCoreVersionViperGA0,
    phymodCoreVersionSestoA0,
    phymodCoreVersionQuadra28,
    phymodCoreVersionHuracan,
    phymodCoreVersionMadura,
    phymodCoreVersionSestoB0,
    phymodCoreVersionDino,
    phymodCoreVersionCount
} phymod_core_version_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_core_version_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_core_version_t validation */
int phymod_core_version_t_validate(phymod_core_version_t phymod_core_version);

/*!
 * @struct phymod_core_info_s
 * @brief Core information 
 */ 
typedef struct phymod_core_info_s {
    phymod_core_version_t core_version; /**< core version */
    uint32_t serdes_id; /**< serdes_id */
    uint32_t phy_id0; /**< phy_id0 */
    uint32_t phy_id1; /**< phy_id1 */
    char name[30];
} phymod_core_info_t;

/* phymod_core_info_t initialization and validation */
int phymod_core_info_t_validate(const phymod_core_info_t* phymod_core_info);
int phymod_core_info_t_init(phymod_core_info_t* phymod_core_info);

/*! 
 * phymod_core_info_get
 *
 * @brief Retrive core information 
 *
 * @param [in]  core            - core access information
 * @param [out]  info            - core information
 */
int phymod_core_info_get(const phymod_core_access_t* core, phymod_core_info_t* info);


/*!
 * @struct phymod_lane_map_s
 * @brief Core information 
 */ 
typedef struct phymod_lane_map_s {
    uint32_t num_of_lanes; /**< Number of elements in lane_map_rx/tx arrays */
    uint32_t lane_map_rx[PHYMOD_MAX_LANES_PER_CORE]; /**< lane_map_rx[x]=y means that rx lane x is mapped to rx lane y */
    uint32_t lane_map_tx[PHYMOD_MAX_LANES_PER_CORE]; /**< lane_map_tx[x]=y means that tx lane x is mapped to tx lane y */
} phymod_lane_map_t;

/* phymod_lane_map_t initialization and validation */
int phymod_lane_map_t_validate(const phymod_lane_map_t* phymod_lane_map);
int phymod_lane_map_t_init(phymod_lane_map_t* phymod_lane_map);

/*! 
 * phymod_core_lane_map_set
 *
 * @brief Set/get lane mapping 
 *
 * @param [in]  core            - core access information
 * @param [in]  lane_map        - core information
 */
int phymod_core_lane_map_set(const phymod_core_access_t* core, const phymod_lane_map_t* lane_map);
/*! 
 * phymod_core_lane_map_get
 *
 * @brief Set/get lane mapping 
 *
 * @param [in]  core            - core access information
 * @param [out]  lane_map        - core information
 */
int phymod_core_lane_map_get(const phymod_core_access_t* core, phymod_lane_map_t* lane_map);


/*!
 * @enum phymod_reset_mode_e
 * @brief Reset modes 
 */ 
typedef enum phymod_reset_mode_e {
    phymodResetModeHard = 0, /**< Hard Reset */
    phymodResetModeSoft, /**< Soft Reset */
    phymodResetModeCount
} phymod_reset_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_reset_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_reset_mode_t validation */
int phymod_reset_mode_t_validate(phymod_reset_mode_t phymod_reset_mode);

/*!
 * @enum phymod_reset_direction_e
 * @brief Reset modes 
 */ 
typedef enum phymod_reset_direction_e {
    phymodResetDirectionIn = 0, /**< In Reset */
    phymodResetDirectionOut, /**< Out of Reset */
    phymodResetDirectionInOut, /**< Toggle Reset */
    phymodResetDirectionCount
} phymod_reset_direction_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_reset_direction_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_reset_direction_t validation */
int phymod_reset_direction_t_validate(phymod_reset_direction_t phymod_reset_direction);
/*! 
 * phymod_core_reset_set
 *
 * @brief Reset Core 
 *
 * @param [in]  core            - core access information
 * @param [in]  reset_mode      - reset mode
 * @param [in]  direction       - reset direction
 */
int phymod_core_reset_set(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t direction);
/*! 
 * phymod_core_reset_get
 *
 * @brief Reset Core 
 *
 * @param [in]  core            - core access information
 * @param [in]  reset_mode      - reset mode
 * @param [out]  direction       - reset state
 */
int phymod_core_reset_get(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t* direction);


/*!
 * @struct phymod_core_firmware_info_s
 * @brief Firmware information 
 */ 
typedef struct phymod_core_firmware_info_s {
    uint32_t fw_version;
    uint32_t fw_crc;
} phymod_core_firmware_info_t;

/* phymod_core_firmware_info_t initialization and validation */
int phymod_core_firmware_info_t_validate(const phymod_core_firmware_info_t* phymod_core_firmware_info);
int phymod_core_firmware_info_t_init(phymod_core_firmware_info_t* phymod_core_firmware_info);

/*! 
 * phymod_core_firmware_info_get
 *
 * @brief Retrive firmware information 
 *
 * @param [in]  core            - core access information
 * @param [out]  fw_info         - 
 */
int phymod_core_firmware_info_get(const phymod_core_access_t* core, phymod_core_firmware_info_t* fw_info);


/*!
 * @enum phymod_firmware_media_type_e
 * @brief media_type for firmware 
 */ 
typedef enum phymod_firmware_media_type_e {
    phymodFirmwareMediaTypePcbTraceBackPlane = 0, /**< back plane */
    phymodFirmwareMediaTypeCopperCable, /**< copper cable */
    phymodFirmwareMediaTypeOptics, /**< optical */
    phymodFirmwareMediaTypeCount
} phymod_firmware_media_type_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_firmware_media_type_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_firmware_media_type_t validation */
int phymod_firmware_media_type_t_validate(phymod_firmware_media_type_t phymod_firmware_media_type);

/*!
 * @struct phymod_firmware_core_config_s
 * @brief Firmware core config 
 */ 
typedef struct phymod_firmware_core_config_s {
    uint32_t CoreConfigFromPCS;
    uint32_t VcoRate; /**< vco rate */
    uint32_t disable_write_pll_iqp; /**< When 1 ucode will not update pll_iqp */
} phymod_firmware_core_config_t;

/* phymod_firmware_core_config_t initialization and validation */
int phymod_firmware_core_config_t_validate(const phymod_firmware_core_config_t* phymod_firmware_core_config);
int phymod_firmware_core_config_t_init(phymod_firmware_core_config_t* phymod_firmware_core_config);

/*! 
 * phymod_phy_firmware_core_config_set
 *
 * @brief Set/get firmware operation mode 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  fw_core_config   - firmware core config
 */
int phymod_phy_firmware_core_config_set(const phymod_phy_access_t* phy, phymod_firmware_core_config_t fw_core_config);
/*! 
 * phymod_phy_firmware_core_config_get
 *
 * @brief Set/get firmware operation mode 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  fw_core_config   - firmware core config
 */
int phymod_phy_firmware_core_config_get(const phymod_phy_access_t* phy, phymod_firmware_core_config_t* fw_core_config);


/*!
 * @struct phymod_firmware_lane_config_s
 * @brief Firmware lane config 
 */ 
typedef struct phymod_firmware_lane_config_s {
    uint32_t LaneConfigFromPCS;
    uint32_t AnEnabled; /**< Autoneg */
    uint32_t DfeOn; /**< Enable DFE */
    uint32_t ForceBrDfe; /**< Force Baud rate DFE */
    uint32_t LpDfeOn; /**< Enable low power DFE */
    phymod_firmware_media_type_t MediaType; /**< Media Type */
    uint32_t UnreliableLos; /**< For optical use */
    uint32_t ScramblingDisable; /**< disable scrambling */
    uint32_t Cl72AutoPolEn; /**< Forced CL72 */
    uint32_t Cl72RestTO; /**< Forced CL72 */
} phymod_firmware_lane_config_t;

/* phymod_firmware_lane_config_t initialization and validation */
int phymod_firmware_lane_config_t_validate(const phymod_firmware_lane_config_t* phymod_firmware_lane_config);
int phymod_firmware_lane_config_t_init(phymod_firmware_lane_config_t* phymod_firmware_lane_config);

/*! 
 * phymod_phy_firmware_lane_config_set
 *
 * @brief Set/get firmware operation mode 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  fw_lane_config   - firmware lane config
 */
int phymod_phy_firmware_lane_config_set(const phymod_phy_access_t* phy, phymod_firmware_lane_config_t fw_lane_config);
/*! 
 * phymod_phy_firmware_lane_config_get
 *
 * @brief Set/get firmware operation mode 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  fw_lane_config   - firmware lane config
 */
int phymod_phy_firmware_lane_config_get(const phymod_phy_access_t* phy, phymod_firmware_lane_config_t* fw_lane_config);


/*!
 * @enum phymod_sequencer_operation_e
 * @brief Sequencer operations 
 */ 
typedef enum phymod_sequencer_operation_e {
    phymodSeqOpStop = 0, /**< Stop Sequencer */
    phymodSeqOpStart, /**< Start Sequencer */
    phymodSeqOpRestart, /**< Toggle Sequencer */
    phymodSeqOpCount
} phymod_sequencer_operation_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_sequencer_operation_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_sequencer_operation_t validation */
int phymod_sequencer_operation_t_validate(phymod_sequencer_operation_t phymod_sequencer_operation);
/*! 
 * phymod_core_pll_sequencer_restart
 *
 * @brief Start/Stop the sequencer 
 *
 * @param [in]  core            - core access information
 * @param [in]  flags           - 
 * @param [in]  operation       - 
 */
int phymod_core_pll_sequencer_restart(const phymod_core_access_t* core, uint32_t flags, phymod_sequencer_operation_t operation);

#define PHYMOD_SEQ_F_WAIT_UNTIL_DONE 0x1 /**< Use this flag to wait for PLL lock when starting the sequencer */

typedef enum phymod_core_event_e {
    phymodCoreEventPllLock = 0, /**< PLL locked */
    phymodCoreEventCount
} phymod_core_event_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_core_event_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_core_event_t validation */
int phymod_core_event_t_validate(phymod_core_event_t phymod_core_event);
#define PHYMOD_TIMEOUT_DEFAULT 0xFFFFFFFF /**< Use default timeout */

/*! 
 * phymod_core_wait_event
 *
 * @brief Wait for core event 
 *
 * @param [in]  core            - core access information
 * @param [in]  event           - event to wait for
 * @param [in]  timeout         - 
 */
int phymod_core_wait_event(const phymod_core_access_t* core, phymod_core_event_t event, uint32_t timeout);

/*! 
 * phymod_phy_rx_restart
 *
 * @brief  re-tune rx path 
 *
 * @param [in]  phy             - phy access information
 */
int phymod_phy_rx_restart(const phymod_phy_access_t* phy);


/*!
 * @struct phymod_polarity_s
 * @brief Polarity bitmaps. The bitmap refers to number of lanes in access structure. 
            For example if the structure contains 2 lanes (at any position), two first bits of these bitmaps are relevant. 
 */ 
typedef struct phymod_polarity_s {
    uint32_t rx_polarity; /**< TX polarity bitmap */
    uint32_t tx_polarity; /**< RX polarity bitmap */
} phymod_polarity_t;

/* phymod_polarity_t initialization and validation */
int phymod_polarity_t_validate(const phymod_polarity_t* phymod_polarity);
int phymod_polarity_t_init(phymod_polarity_t* phymod_polarity);

/*! 
 * phymod_phy_polarity_set
 *
 * @brief Set phy polarity 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  polarity        - 
 */
int phymod_phy_polarity_set(const phymod_phy_access_t* phy, const phymod_polarity_t* polarity);
/*! 
 * phymod_phy_polarity_get
 *
 * @brief Set phy polarity 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  polarity        - 
 */
int phymod_phy_polarity_get(const phymod_phy_access_t* phy, phymod_polarity_t* polarity);

/*! 
 * @brief when field of phymod_tx_t struct set to this val. the phymod_tx_set API will not change the specified member value in the HW 
 */ 
#define PHYMOD_TX_DO_NOT_CHANGE_VAL 0xFFFFFFFF


/*!
 * @struct phymod_tx_s
 * @brief TX Parameters 
 */ 
typedef struct phymod_tx_s {
    int8_t pre;
    int8_t main;
    int8_t post;
    int8_t post2;
    int8_t post3;
    int8_t amp;
} phymod_tx_t;

/* phymod_tx_t initialization and validation */
int phymod_tx_t_validate(const phymod_tx_t* phymod_tx);
int phymod_tx_t_init(phymod_tx_t* phymod_tx);

/*! 
 * phymod_phy_tx_set
 *
 * @brief Set/Get TX Parameters 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  tx              - 
 */
int phymod_phy_tx_set(const phymod_phy_access_t* phy, const phymod_tx_t* tx);
/*! 
 * phymod_phy_tx_get
 *
 * @brief Set/Get TX Parameters 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  tx              - 
 */
int phymod_phy_tx_get(const phymod_phy_access_t* phy, phymod_tx_t* tx);

typedef enum phymod_media_typed_e {
    phymodMediaTypeChipToChip = 0,
    phymodMediaTypeShort,
    phymodMediaTypeMid,
    phymodMediaTypeLong,
    phymodMediaTypeCount
} phymod_media_typed_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_media_typed_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_media_typed_t validation */
int phymod_media_typed_t_validate(phymod_media_typed_t phymod_media_typed);
/*! 
 * phymod_phy_media_type_tx_get
 *
 * @brief Request for default TX parameters configuration per media type 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  media           - 
 * @param [out]  tx              - 
 */
int phymod_phy_media_type_tx_get(const phymod_phy_access_t* phy, phymod_media_typed_t media, phymod_tx_t* tx);


/*!
 * @struct phymod_tx_override_s
 * @brief TX parameters which can be auto-set or override 
 */ 
typedef struct phymod_tx_override_s {
    phymod_value_override_t phase_interpolator;
} phymod_tx_override_t;

/* phymod_tx_override_t initialization and validation */
int phymod_tx_override_t_validate(const phymod_tx_override_t* phymod_tx_override);
int phymod_tx_override_t_init(phymod_tx_override_t* phymod_tx_override);

/*! 
 * phymod_phy_tx_override_set
 *
 * @brief Set/Get TX override Parameters 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  tx_override     - 
 */
int phymod_phy_tx_override_set(const phymod_phy_access_t* phy, const phymod_tx_override_t* tx_override);
/*! 
 * phymod_phy_tx_override_get
 *
 * @brief Set/Get TX override Parameters 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  tx_override     - 
 */
int phymod_phy_tx_override_get(const phymod_phy_access_t* phy, phymod_tx_override_t* tx_override);

#define PHYMOD_NUM_DFE_TAPS 14 /**< MAX num of DFE TAPS */

/*! 
 * @brief rx_adaptation check 
 */ 
#define PHYMOD_RX_ADAPTATION_ON 0x1 /**< rx_adaptation check */

#define PHYMOD_RX_ADAPTATION_ON_SET(rx) ((rx)->rx_adaptation_on |= PHYMOD_RX_ADAPTATION_ON)

#define PHYMOD_RX_ADAPTATION_ON_CLR(rx) ((rx)->rx_adaptation_on &= ~PHYMOD_RX_ADAPTATION_ON)

#define PHYMOD_RX_ADAPTATION_ON_GET(rx) ((rx)->rx_adaptation_on & PHYMOD_RX_ADAPTATION_ON ? 1 : 0)


/*!
 * @struct phymod_rx_s
 * @brief RX Parameters 
 */ 
typedef struct phymod_rx_s {
    phymod_value_override_t vga;
    uint32_t num_of_dfe_taps; /**< number of elements in DFE array */
    phymod_value_override_t dfe[PHYMOD_NUM_DFE_TAPS];
    phymod_value_override_t peaking_filter;
    phymod_value_override_t low_freq_peaking_filter;
    uint32_t rx_adaptation_on;
} phymod_rx_t;

/* phymod_rx_t initialization and validation */
int phymod_rx_t_validate(const phymod_rx_t* phymod_rx);
int phymod_rx_t_init(phymod_rx_t* phymod_rx);

/*! 
 * phymod_phy_rx_set
 *
 * @brief Set/Get RX Parameters 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  rx              - 
 */
int phymod_phy_rx_set(const phymod_phy_access_t* phy, const phymod_rx_t* rx);
/*! 
 * phymod_phy_rx_get
 *
 * @brief Set/Get RX Parameters 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  rx              - 
 */
int phymod_phy_rx_get(const phymod_phy_access_t* phy, phymod_rx_t* rx);

/*! 
 * phymod_phy_rx_adaptation_resume
 *
 * @brief PHY Rx adaptation resume 
 *
 * @param [in]  phy             - phy access information
 */
int phymod_phy_rx_adaptation_resume(const phymod_phy_access_t* phy);


/*!
 * @struct phymod_phy_reset_s
 * @brief Direction (In,Out,inOut) for RX/TX to reset 
 */ 
typedef struct phymod_phy_reset_s {
    phymod_reset_direction_t rx;
    phymod_reset_direction_t tx;
} phymod_phy_reset_t;

/* phymod_phy_reset_t initialization and validation */
int phymod_phy_reset_t_validate(const phymod_phy_reset_t* phymod_phy_reset);
int phymod_phy_reset_t_init(phymod_phy_reset_t* phymod_phy_reset);

/*! 
 * phymod_phy_reset_set
 *
 * @brief Reset phy 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  reset           - 
 */
int phymod_phy_reset_set(const phymod_phy_access_t* phy, const phymod_phy_reset_t* reset);
/*! 
 * phymod_phy_reset_get
 *
 * @brief Reset phy 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  reset           - 
 */
int phymod_phy_reset_get(const phymod_phy_access_t* phy, phymod_phy_reset_t* reset);

typedef enum phymod_power_e {
    phymodPowerOff = 0, /**< turn off power */
    phymodPowerOn, /**< turn on power */
    phymodPowerOffOn, /**< toggle power */
    phymodPowerNoChange, /**< stay where you are */
    phymodPowerCount
} phymod_power_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_power_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_power_t validation */
int phymod_power_t_validate(phymod_power_t phymod_power);

/*!
 * @struct phymod_phy_power_s
 * @brief Power operation (On,Off,Off and On) for RX/TX 
 */ 
typedef struct phymod_phy_power_s {
    phymod_power_t rx;
    phymod_power_t tx;
} phymod_phy_power_t;

/* phymod_phy_power_t initialization and validation */
int phymod_phy_power_t_validate(const phymod_phy_power_t* phymod_phy_power);
int phymod_phy_power_t_init(phymod_phy_power_t* phymod_phy_power);

/*! 
 * phymod_phy_power_set
 *
 * @brief Control phy power 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  power           - 
 */
int phymod_phy_power_set(const phymod_phy_access_t* phy, const phymod_phy_power_t* power);
/*! 
 * phymod_phy_power_get
 *
 * @brief Control phy power 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  power           - 
 */
int phymod_phy_power_get(const phymod_phy_access_t* phy, phymod_phy_power_t* power);

typedef enum phymod_phy_hg2_codec_e {
    phymodBcmHG2CodecOff = 0, /**< hg2 codec off  */
    phymodBcmHG2CodecOnWith8ByteIPG, /**< 8-byte IPG setting */
    phymodBcmHG2CodecOnWith9ByteIPG, /**< 9-byte IPG setting */
    phymodBcmHG2CodecCount
} phymod_phy_hg2_codec_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_phy_hg2_codec_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_phy_hg2_codec_t validation */
int phymod_phy_hg2_codec_t_validate(phymod_phy_hg2_codec_t phymod_phy_hg2_codec);
/*! 
 * phymod_phy_hg2_codec_control_set
 *
 * @brief Control phy hg2 codec 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  hg2_codec       - 
 */
int phymod_phy_hg2_codec_control_set(const phymod_phy_access_t* phy, phymod_phy_hg2_codec_t hg2_codec);
/*! 
 * phymod_phy_hg2_codec_control_get
 *
 * @brief Control phy hg2 codec 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  hg2_codec       - 
 */
int phymod_phy_hg2_codec_control_get(const phymod_phy_access_t* phy, phymod_phy_hg2_codec_t* hg2_codec);

typedef enum phymod_phy_tx_lane_control_e {
    phymodTxTrafficDisable = 0, /**< disable tx traffic */
    phymodTxTrafficEnable, /**< enable tx traffic */
    phymodTxReset, /**< reset tx data path */
    phymodTxSquelchOn, /**< squelch tx */
    phymodTxSquelchOff, /**< squelch tx off */
    phymodTxElectricalIdleEnable, /**< enable elctrical idle */
    phymodTxElectricalIdleDisable, /**< disable elctrical idle */
    phymodTxCount
} phymod_phy_tx_lane_control_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_phy_tx_lane_control_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_phy_tx_lane_control_t validation */
int phymod_phy_tx_lane_control_t_validate(phymod_phy_tx_lane_control_t phymod_phy_tx_lane_control);

/*!
 * @struct phymod_autoneg_oui_s
 * @brief autoneg_oui 
 */ 
typedef struct phymod_autoneg_oui_s {
    uint32_t oui; /**< New oui */
    uint16_t oui_override_bam73_adv; /**< Adv new OUI for BAM73 */
    uint16_t oui_override_bam73_det; /**< Detect new OUI for BAM73 */
    uint16_t oui_override_hpam_adv; /**< Adv new OUI for HPAM */
    uint16_t oui_override_hpam_det; /**< Detect new OUI for HPAM */
    uint16_t oui_override_bam37_adv; /**< Adv new OUI for BAM37 */
    uint16_t oui_override_bam37_det; /**< Detect new OUI for BAM37 */
} phymod_autoneg_oui_t;

/* phymod_autoneg_oui_t initialization and validation */
int phymod_autoneg_oui_t_validate(const phymod_autoneg_oui_t* phymod_autoneg_oui);
int phymod_autoneg_oui_t_init(phymod_autoneg_oui_t* phymod_autoneg_oui);

typedef enum phymod_phy_rx_lane_control_e {
    phymodRxReset, /**< reset rx data path */
    phymodRxSquelchOn, /**< squelch rx */
    phymodRxSquelchOff, /**< squelch rx off */
    phymodRxCount
} phymod_phy_rx_lane_control_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_phy_rx_lane_control_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_phy_rx_lane_control_t validation */
int phymod_phy_rx_lane_control_t_validate(phymod_phy_rx_lane_control_t phymod_phy_rx_lane_control);
/*! 
 * phymod_phy_tx_lane_control_set
 *
 * @brief TX transmission control 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  tx_control      - 
 */
int phymod_phy_tx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t tx_control);
/*! 
 * phymod_phy_tx_lane_control_get
 *
 * @brief TX transmission control 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  tx_control      - 
 */
int phymod_phy_tx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t* tx_control);

/*! 
 * phymod_phy_rx_lane_control_set
 *
 * @brief Rx control 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  rx_control      - 
 */
int phymod_phy_rx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t rx_control);
/*! 
 * phymod_phy_rx_lane_control_get
 *
 * @brief Rx control 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  rx_control      - 
 */
int phymod_phy_rx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t* rx_control);

/*! 
 * phymod_phy_fec_enable_set
 *
 * @brief forced speed FEC control 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  enable          - 
 */
int phymod_phy_fec_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
/*! 
 * phymod_phy_fec_enable_get
 *
 * @brief forced speed FEC control 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  enable          - 
 */
int phymod_phy_fec_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*! 
 * phymod_phy_autoneg_oui_set
 *
 * @brief Change OUI to consortium OUI 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  an_oui          - 
 */
int phymod_phy_autoneg_oui_set(const phymod_phy_access_t* phy, phymod_autoneg_oui_t an_oui);
/*! 
 * phymod_phy_autoneg_oui_get
 *
 * @brief Change OUI to consortium OUI 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  an_oui          - 
 */
int phymod_phy_autoneg_oui_get(const phymod_phy_access_t* phy, phymod_autoneg_oui_t* an_oui);

/*! 
 * phymod_phy_eee_set
 *
 * @brief energy efficient control 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  enable          - 
 */
int phymod_phy_eee_set(const phymod_phy_access_t* phy, uint32_t enable);
/*! 
 * phymod_phy_eee_get
 *
 * @brief energy efficient control 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  enable          - 
 */
int phymod_phy_eee_get(const phymod_phy_access_t* phy, uint32_t* enable);

typedef enum phymod_interface_e {
    phymodInterfaceBypass = 0,
    phymodInterfaceSR,
    phymodInterfaceSR4,
    phymodInterfaceKX,
    phymodInterfaceKX4,
    phymodInterfaceKR,
    phymodInterfaceKR2,
    phymodInterfaceKR4,
    phymodInterfaceCX,
    phymodInterfaceCX2,
    phymodInterfaceCX4,
    phymodInterfaceCR,
    phymodInterfaceCR2,
    phymodInterfaceCR4,
    phymodInterfaceCR10,
    phymodInterfaceXFI,
    phymodInterfaceSFI,
    phymodInterfaceSFPDAC,
    phymodInterfaceXGMII,
    phymodInterface1000X,
    phymodInterfaceSGMII,
    phymodInterfaceXAUI,
    phymodInterfaceRXAUI,
    phymodInterfaceX2,
    phymodInterfaceXLAUI,
    phymodInterfaceXLAUI2,
    phymodInterfaceCAUI,
    phymodInterfaceQSGMII,
    phymodInterfaceLR4,
    phymodInterfaceLR,
    phymodInterfaceLR2,
    phymodInterfaceER,
    phymodInterfaceER2,
    phymodInterfaceER4,
    phymodInterfaceSR2,
    phymodInterfaceSR10,
    phymodInterfaceCAUI4,
    phymodInterfaceVSR,
    phymodInterfaceLR10,
    phymodInterfaceKR10,
    phymodInterfaceCAUI4_C2C,
    phymodInterfaceCAUI4_C2M,
    phymodInterfaceZR,
    phymodInterfaceLRM,
    phymodInterfaceXLPPI,
    phymodInterfaceOTN,
    phymodInterfaceCount
} phymod_interface_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_interface_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_interface_t validation */
int phymod_interface_t_validate(phymod_interface_t phymod_interface);

/*!
 * @enum phymod_ref_clk_e
 * @brief Reference Clock 
 */ 
typedef enum phymod_ref_clk_e {
    phymodRefClk156Mhz = 0, /**< 156.25MHz */
    phymodRefClk125Mhz, /**< 125MHz */
    phymodRefClk106Mhz, /**< 106Mhz */
    phymodRefClk161Mhz, /**< 161Mhz */
    phymodRefClk174Mhz, /**< 174Mhz */
    phymodRefClk312Mhz, /**< 312Mhz */
    phymodRefClk322Mhz, /**< 322Mhz */
    phymodRefClk349Mhz, /**< 349Mhz */
    phymodRefClk644Mhz, /**< 644Mhz */
    phymodRefClk698Mhz, /**< 698Mhz */
    phymodRefClk155Mhz, /**< 155Mhz */
    phymodRefClk156P6Mhz, /**< 156P6Mhz */
    phymodRefClk157Mhz, /**< 157Mhz */
    phymodRefClk158Mhz, /**< 158Mhz */
    phymodRefClk159Mhz, /**< 159Mhz */
    phymodRefClk168Mhz, /**< 168Mhz */
    phymodRefClk172Mhz, /**< 172Mhz */
    phymodRefClk173Mhz, /**< 173Mhz */
    phymodRefClk169P409Mhz, /**< 169P409Mhz */
    phymodRefClk348P125Mhz, /**< 348P125Mhz */
    phymodRefClk162P948Mhz, /**< 162P948Mhz */
    phymodRefClk336P094Mhz, /**< 336P094Mhz */
    phymodRefClk168P12Mhz, /**< 168P12Mhz */
    phymodRefClk346P74Mhz, /**< 346P74Mhz */
    phymodRefClk167P41Mhz, /**< 167P41Mhz */
    phymodRefClk345P28Mhz, /**< 345P28Mhz */
    phymodRefClk162P26Mhz, /**< 162P26Mhz */
    phymodRefClk334P66Mhz, /**< 334P66Mhz */
    phymodRefClkCount
} phymod_ref_clk_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_ref_clk_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_ref_clk_t validation */
int phymod_ref_clk_t_validate(phymod_ref_clk_t phymod_ref_clk);

/*!
 * @enum phymod_triple_core_e
 * @brief Triple Core modes 
 */ 
typedef enum phymod_triple_core_e {
    phymodTripleCore444 = 0, /**< 120G */
    phymodTripleCore343, /**< 100G 3-4-3 */
    phymodTripleCore442, /**< 100G 4-4-2 */
    phymodTripleCore244, /**< 100G 2-4-4 */
    phymodTripleCoreCount
} phymod_triple_core_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_triple_core_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_triple_core_t validation */
int phymod_triple_core_t_validate(phymod_triple_core_t phymod_triple_core);

/*!
 * @enum phymod_otn_type_e
 * @brief OTN type 
 */ 
typedef enum phymod_otn_type_e {
    phymodOTNOTU1 = 0,
    phymodOTNOTU1e,
    phymodOTNOTU2,
    phymodOTNOTU2e,
    phymodOTNOTU2f,
    phymodOTNOTU3,
    phymodOTNOTU3e2,
    phymodOTNOTU4,
    phymodOTNCount
} phymod_otn_type_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_otn_type_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_otn_type_t validation */
int phymod_otn_type_t_validate(phymod_otn_type_t phymod_otn_type);
typedef struct phymod_phy_inf_config_s {
    phymod_interface_t interface_type;
    uint32_t data_rate;
    uint32_t interface_modes;
    phymod_ref_clk_t ref_clock; /**< Core reference clock. */
    uint16_t pll_divider_req; /**< Core pll divider request. */
    void* device_aux_modes; /**< Device auxiliary modes. */
    phymod_otn_type_t otn_type; /**< OTN type. */
} phymod_phy_inf_config_t;

/* phymod_phy_inf_config_t initialization and validation */
int phymod_phy_inf_config_t_validate(const phymod_phy_inf_config_t* phymod_phy_inf_config);
int phymod_phy_inf_config_t_init(phymod_phy_inf_config_t* phymod_phy_inf_config);

#define PHYMOD_DEFAULT_RATE 0xFFFFFFFF /**< Use data_rate=PHYMOD_DEFAULT_RATE to set interface default */

/*! 
 * phymod_phy_interface_config_set
 *
 * @brief TX transmission disable 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  flags           - See PHYMOD_INTF_F_
 * @param [in]  config          - 
 */
int phymod_phy_interface_config_set(const phymod_phy_access_t* phy, uint32_t flags, const phymod_phy_inf_config_t* config);
/*! 
 * phymod_phy_interface_config_get
 *
 * @brief TX transmission disable 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  flags           - See PHYMOD_INTF_F_
 * @param [in]  ref_clock       - Input core reference clock
 * @param [out]  config          - 
 */
int phymod_phy_interface_config_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_ref_clk_t ref_clock, phymod_phy_inf_config_t* config);

/*! 
 * @brief Interface properties 
 */ 
#define PHYMOD_INTF_MODES_HIGIG 0x1 /**< Interface is Higig */
#define PHYMOD_INTF_MODES_OS2 0x2 /**< Force working in OS 2 */
#define PHYMOD_INTF_MODES_SCR 0x4 /**< enable scrambler */
#define PHYMOD_INTF_MODES_HALF_DUPLEX 0x8 /**< Interface is half-duplex */
#define PHYMOD_INTF_MODES_FIBER 0x10 /**< Interface is connected to fiber */
#define PHYMOD_INTF_MODES_TRIPLE_CORE 0x20 /**< 127G/120G 444(default) or less(100G) interface specified by TC_xxx */
#define PHYMOD_INTF_MODES_TC_343 0x40 /**< triple-core 343 */
#define PHYMOD_INTF_MODES_TC_442 0x80 /**< triple-core 442 */
#define PHYMOD_INTF_MODES_TC_244 0x100 /**< triple-core 244 */
#define PHYMOD_INTF_MODES_BACKPLANE 0x200 /**< Interface is connected to backplane */
#define PHYMOD_INTF_MODES_COPPER 0x400 /**< Interface is connected to copper cable */
#define PHYMOD_INTF_MODES_OTN 0x800 /**< Interface is OTN */
#define PHYMOD_INTF_MODES_SIMPLEX 0x1000 /**< Interface is SIMPLEX */
#define PHYMOD_INTF_MODES_UNRELIABLE_LOS 0x2000 /**< Interface is UNRELIABLE_LOS */

#define PHYMOD_INTF_MODES_HIGIG_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_HIGIG)
#define PHYMOD_INTF_MODES_OS2_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_OS2)
#define PHYMOD_INTF_MODES_SCR_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_SCR)
#define PHYMOD_INTF_MODES_HALF_DUPLEX_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_HALF_DUPLEX)
#define PHYMOD_INTF_MODES_FIBER_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_FIBER)
#define PHYMOD_INTF_MODES_TRIPLE_CORE_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_TRIPLE_CORE)
#define PHYMOD_INTF_MODES_TC_343_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_TC_343)
#define PHYMOD_INTF_MODES_TC_442_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_TC_442)
#define PHYMOD_INTF_MODES_TC_244_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_TC_244)
#define PHYMOD_INTF_MODES_BACKPLANE_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_BACKPLANE)
#define PHYMOD_INTF_MODES_COPPER_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_COPPER)
#define PHYMOD_INTF_MODES_OTN_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_OTN)
#define PHYMOD_INTF_MODES_SIMPLEX_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_SIMPLEX)
#define PHYMOD_INTF_MODES_UNRELIABLE_LOS_SET(config) ((config)->interface_modes |= PHYMOD_INTF_MODES_UNRELIABLE_LOS)

#define PHYMOD_INTF_MODES_HIGIG_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_HIGIG)
#define PHYMOD_INTF_MODES_OS2_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_OS2)
#define PHYMOD_INTF_MODES_SCR_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_SCR)
#define PHYMOD_INTF_MODES_HALF_DUPLEX_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_HALF_DUPLEX)
#define PHYMOD_INTF_MODES_FIBER_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_FIBER)
#define PHYMOD_INTF_MODES_TRIPLE_CORE_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_TRIPLE_CORE)
#define PHYMOD_INTF_MODES_TC_343_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_TC_343)
#define PHYMOD_INTF_MODES_TC_442_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_TC_442)
#define PHYMOD_INTF_MODES_TC_244_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_TC_244)
#define PHYMOD_INTF_MODES_BACKPLANE_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_BACKPLANE)
#define PHYMOD_INTF_MODES_COPPER_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_COPPER)
#define PHYMOD_INTF_MODES_OTN_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_OTN)
#define PHYMOD_INTF_MODES_SIMPLEX_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_SIMPLEX)
#define PHYMOD_INTF_MODES_UNRELIABLE_LOS_CLR(config) ((config)->interface_modes &= ~PHYMOD_INTF_MODES_UNRELIABLE_LOS)

#define PHYMOD_INTF_MODES_HIGIG_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_HIGIG ? 1 : 0)
#define PHYMOD_INTF_MODES_OS2_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_OS2 ? 1 : 0)
#define PHYMOD_INTF_MODES_SCR_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_SCR ? 1 : 0)
#define PHYMOD_INTF_MODES_HALF_DUPLEX_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_HALF_DUPLEX ? 1 : 0)
#define PHYMOD_INTF_MODES_FIBER_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_FIBER ? 1 : 0)
#define PHYMOD_INTF_MODES_TRIPLE_CORE_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_TRIPLE_CORE ? 1 : 0)
#define PHYMOD_INTF_MODES_TC_343_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_TC_343 ? 1 : 0)
#define PHYMOD_INTF_MODES_TC_442_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_TC_442 ? 1 : 0)
#define PHYMOD_INTF_MODES_TC_244_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_TC_244 ? 1 : 0)
#define PHYMOD_INTF_MODES_BACKPLANE_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_BACKPLANE ? 1 : 0)
#define PHYMOD_INTF_MODES_COPPER_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_COPPER ? 1 : 0)
#define PHYMOD_INTF_MODES_OTN_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_OTN ? 1 : 0)
#define PHYMOD_INTF_MODES_SIMPLEX_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_SIMPLEX ? 1 : 0)
#define PHYMOD_INTF_MODES_UNRELIABLE_LOS_GET(config) ((config)->interface_modes & PHYMOD_INTF_MODES_UNRELIABLE_LOS ? 1 : 0)

/*! 
 * @brief Flags for phymod_phy_interface_config_set API 
 */ 
#define PHYMOD_INTF_F_DONT_TURN_OFF_PLL 0x1 /**< Dont turn off PLL when switching the speed */
#define PHYMOD_INTF_F_DONT_OVERIDE_FW_MODE 0x2 /**< Dont change  fw mode settings during interface_config */
#define PHYMOD_INTF_F_DONT_OVERIDE_TX_PARAMS 0x4 /**< Dont change tx parameters during interface_config */
#define PHYMOD_INTF_F_PLL_DIVIDER_OVERRIDE 0x8 /**< Change pll div parameters during interface_config */
#define PHYMOD_INTF_F_PCS_TABLE_OVERRIDE 0x10 /**< Change pcs parameter table during interface_config */
#define PHYMOD_INTF_F_INTF_PARAM_SET_ONLY 0x20 /**< Only change interface parameter variable */
#define PHYMOD_INTF_F_SET_CORE_MAP_MODE 0x40 /**< Only Set Mode information */
#define PHYMOD_INTF_F_SET_SPD_TRIGGER 0x80 /**< Only Trigger Speed  */
#define PHYMOD_INTF_F_SET_SPD_DISABLE 0x100 /**< Only Disable Speed  */
#define PHYMOD_INTF_F_SET_SPD_NO_TRIGGER 0x200 /**< Only Disable Speed  */
#define PHYMOD_INTF_F_CL72_REQUESTED_BY_CNFG 0x400 /**< CL72 is requested by config file */
#define PHYMOD_INTF_F_CL72_REQUESTED_BY_API 0x800 /**< CL72 is requested by API */
#define PHYMOD_INTF_F_UPDATE_SPEED_LINKUP 0x1000 /**< Only update speed */
#define PHYMOD_INTF_F_UPDATE_INTERNAL_SERDES_ONLY 0x2000 /**< Only update mac speed */
#define PHYMOD_INTF_F_CORE_MAP_MODE_FLIP 0x4000 /**< Flag to indicate flip between 244 and 442 */

/*! 
 * @brief soc config properties for device_op_mode to specify phy device operation mode 
 */ 
#define PHYMOD_INTF_CONFIG_PHY_GEARBOX_ENABLE 0x1 /**< enables the gearbox data path mode */
#define PHYMOD_INTF_CONFIG_PHY_PIN_COMPATIBILITY_ENABLE 0x2 /**< enables backward pin-compatibility */
#define PHYMOD_INTF_CONFIG_PORT_PHY_MODE_REVERSE 0x4 /**< enables the port reverse direction */

#define PHYMOD_INTF_CONFIG_PHY_GEARBOX_ENABLE_SET(phy_acc) ((phy_acc)->device_op_mode |= PHYMOD_INTF_CONFIG_PHY_GEARBOX_ENABLE)
#define PHYMOD_INTF_CONFIG_PHY_PIN_COMPATIBILITY_ENABLE_SET(phy_acc) ((phy_acc)->device_op_mode |= PHYMOD_INTF_CONFIG_PHY_PIN_COMPATIBILITY_ENABLE)
#define PHYMOD_INTF_CONFIG_PORT_PHY_MODE_REVERSE_SET(phy_acc) ((phy_acc)->device_op_mode |= PHYMOD_INTF_CONFIG_PORT_PHY_MODE_REVERSE)

#define PHYMOD_INTF_CONFIG_PHY_GEARBOX_ENABLE_CLR(phy_acc) ((phy_acc)->device_op_mode &= ~PHYMOD_INTF_CONFIG_PHY_GEARBOX_ENABLE)
#define PHYMOD_INTF_CONFIG_PHY_PIN_COMPATIBILITY_ENABLE_CLR(phy_acc) ((phy_acc)->device_op_mode &= ~PHYMOD_INTF_CONFIG_PHY_PIN_COMPATIBILITY_ENABLE)
#define PHYMOD_INTF_CONFIG_PORT_PHY_MODE_REVERSE_CLR(phy_acc) ((phy_acc)->device_op_mode &= ~PHYMOD_INTF_CONFIG_PORT_PHY_MODE_REVERSE)

#define PHYMOD_INTF_CONFIG_PHY_GEARBOX_ENABLE_GET(phy_acc) ((phy_acc)->device_op_mode & PHYMOD_INTF_CONFIG_PHY_GEARBOX_ENABLE ? 1 : 0)
#define PHYMOD_INTF_CONFIG_PHY_PIN_COMPATIBILITY_ENABLE_GET(phy_acc) ((phy_acc)->device_op_mode & PHYMOD_INTF_CONFIG_PHY_PIN_COMPATIBILITY_ENABLE ? 1 : 0)
#define PHYMOD_INTF_CONFIG_PORT_PHY_MODE_REVERSE_GET(phy_acc) ((phy_acc)->device_op_mode & PHYMOD_INTF_CONFIG_PORT_PHY_MODE_REVERSE ? 1 : 0)

/*! 
 * phymod_phy_cl72_set
 *
 * @brief Set/Get CL72 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  cl72_en         - 
 */
int phymod_phy_cl72_set(const phymod_phy_access_t* phy, uint32_t cl72_en);
/*! 
 * phymod_phy_cl72_get
 *
 * @brief Set/Get CL72 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  cl72_en         - 
 */
int phymod_phy_cl72_get(const phymod_phy_access_t* phy, uint32_t* cl72_en);

typedef struct phymod_cl72_status_s {
    uint32_t enabled;
    uint32_t locked;
} phymod_cl72_status_t;

/* phymod_cl72_status_t initialization and validation */
int phymod_cl72_status_t_validate(const phymod_cl72_status_t* phymod_cl72_status);
int phymod_cl72_status_t_init(phymod_cl72_status_t* phymod_cl72_status);

/*! 
 * phymod_phy_cl72_status_get
 *
 * @brief Get CL72 status 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  status          - 
 */
int phymod_phy_cl72_status_get(const phymod_phy_access_t* phy, phymod_cl72_status_t* status);

/*! 
 * @brief Capabilities for CL73 
 */ 
#define PHYMOD_AN_CAP_1G_KX 0x1
#define PHYMOD_AN_CAP_10G_KX4 0x2
#define PHYMOD_AN_CAP_10G_KR 0x4
#define PHYMOD_AN_CAP_40G_KR4 0x8
#define PHYMOD_AN_CAP_40G_CR4 0x10
#define PHYMOD_AN_CAP_100G_CR10 0x20
#define PHYMOD_AN_CAP_100G_CR4 0x40
#define PHYMOD_AN_CAP_100G_KR4 0x80
#define PHYMOD_AN_CAP_100G_KR10 0x100
#define PHYMOD_AN_CAP_100M 0x200

#define PHYMOD_AN_CAP_1G_KX_SET(cap) (cap |= PHYMOD_AN_CAP_1G_KX)
#define PHYMOD_AN_CAP_10G_KX4_SET(cap) (cap |= PHYMOD_AN_CAP_10G_KX4)
#define PHYMOD_AN_CAP_10G_KR_SET(cap) (cap |= PHYMOD_AN_CAP_10G_KR)
#define PHYMOD_AN_CAP_40G_KR4_SET(cap) (cap |= PHYMOD_AN_CAP_40G_KR4)
#define PHYMOD_AN_CAP_40G_CR4_SET(cap) (cap |= PHYMOD_AN_CAP_40G_CR4)
#define PHYMOD_AN_CAP_100G_CR10_SET(cap) (cap |= PHYMOD_AN_CAP_100G_CR10)
#define PHYMOD_AN_CAP_100G_CR4_SET(cap) (cap |= PHYMOD_AN_CAP_100G_CR4)
#define PHYMOD_AN_CAP_100G_KR4_SET(cap) (cap |= PHYMOD_AN_CAP_100G_KR4)
#define PHYMOD_AN_CAP_100G_KR10_SET(cap) (cap |= PHYMOD_AN_CAP_100G_KR10)
#define PHYMOD_AN_CAP_100M_SET(cap) (cap |= PHYMOD_AN_CAP_100M)

#define PHYMOD_AN_CAP_1G_KX_CLR(cap) (cap &= ~PHYMOD_AN_CAP_1G_KX)
#define PHYMOD_AN_CAP_10G_KX4_CLR(cap) (cap &= ~PHYMOD_AN_CAP_10G_KX4)
#define PHYMOD_AN_CAP_10G_KR_CLR(cap) (cap &= ~PHYMOD_AN_CAP_10G_KR)
#define PHYMOD_AN_CAP_40G_KR4_CLR(cap) (cap &= ~PHYMOD_AN_CAP_40G_KR4)
#define PHYMOD_AN_CAP_40G_CR4_CLR(cap) (cap &= ~PHYMOD_AN_CAP_40G_CR4)
#define PHYMOD_AN_CAP_100G_CR10_CLR(cap) (cap &= ~PHYMOD_AN_CAP_100G_CR10)
#define PHYMOD_AN_CAP_100G_CR4_CLR(cap) (cap &= ~PHYMOD_AN_CAP_100G_CR4)
#define PHYMOD_AN_CAP_100G_KR4_CLR(cap) (cap &= ~PHYMOD_AN_CAP_100G_KR4)
#define PHYMOD_AN_CAP_100G_KR10_CLR(cap) (cap &= ~PHYMOD_AN_CAP_100G_KR10)
#define PHYMOD_AN_CAP_100M_CLR(cap) (cap &= ~PHYMOD_AN_CAP_100M)

#define PHYMOD_AN_CAP_1G_KX_GET(cap) (cap & PHYMOD_AN_CAP_1G_KX ? 1 : 0)
#define PHYMOD_AN_CAP_10G_KX4_GET(cap) (cap & PHYMOD_AN_CAP_10G_KX4 ? 1 : 0)
#define PHYMOD_AN_CAP_10G_KR_GET(cap) (cap & PHYMOD_AN_CAP_10G_KR ? 1 : 0)
#define PHYMOD_AN_CAP_40G_KR4_GET(cap) (cap & PHYMOD_AN_CAP_40G_KR4 ? 1 : 0)
#define PHYMOD_AN_CAP_40G_CR4_GET(cap) (cap & PHYMOD_AN_CAP_40G_CR4 ? 1 : 0)
#define PHYMOD_AN_CAP_100G_CR10_GET(cap) (cap & PHYMOD_AN_CAP_100G_CR10 ? 1 : 0)
#define PHYMOD_AN_CAP_100G_CR4_GET(cap) (cap & PHYMOD_AN_CAP_100G_CR4 ? 1 : 0)
#define PHYMOD_AN_CAP_100G_KR4_GET(cap) (cap & PHYMOD_AN_CAP_100G_KR4 ? 1 : 0)
#define PHYMOD_AN_CAP_100G_KR10_GET(cap) (cap & PHYMOD_AN_CAP_100G_KR10 ? 1 : 0)
#define PHYMOD_AN_CAP_100M_GET(cap) (cap & PHYMOD_AN_CAP_100M ? 1 : 0)

/*! 
 * @brief Capabilities for CL73_BAM 
 */ 
#define PHYMOD_BAM_CL73_CAP_20G_KR2 0x1
#define PHYMOD_BAM_CL73_CAP_20G_CR2 0x2
#define PHYMOD_BAM_CL73_CAP_40G_KR2 0x4
#define PHYMOD_BAM_CL73_CAP_40G_CR2 0x8
#define PHYMOD_BAM_CL73_CAP_50G_KR2 0x10
#define PHYMOD_BAM_CL73_CAP_50G_CR2 0x20
#define PHYMOD_BAM_CL73_CAP_50G_KR4 0x40
#define PHYMOD_BAM_CL73_CAP_50G_CR4 0x80
#define PHYMOD_BAM_CL73_CAP_20G_KR1 0x100
#define PHYMOD_BAM_CL73_CAP_20G_CR1 0x200
#define PHYMOD_BAM_CL73_CAP_25G_KR1 0x400
#define PHYMOD_BAM_CL73_CAP_25G_CR1 0x800

#define PHYMOD_BAM_CL73_CAP_20G_KR2_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_20G_KR2)
#define PHYMOD_BAM_CL73_CAP_20G_CR2_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_20G_CR2)
#define PHYMOD_BAM_CL73_CAP_40G_KR2_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_40G_KR2)
#define PHYMOD_BAM_CL73_CAP_40G_CR2_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_40G_CR2)
#define PHYMOD_BAM_CL73_CAP_50G_KR2_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_50G_KR2)
#define PHYMOD_BAM_CL73_CAP_50G_CR2_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_50G_CR2)
#define PHYMOD_BAM_CL73_CAP_50G_KR4_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_50G_KR4)
#define PHYMOD_BAM_CL73_CAP_50G_CR4_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_50G_CR4)
#define PHYMOD_BAM_CL73_CAP_20G_KR1_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_20G_KR1)
#define PHYMOD_BAM_CL73_CAP_20G_CR1_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_20G_CR1)
#define PHYMOD_BAM_CL73_CAP_25G_KR1_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_25G_KR1)
#define PHYMOD_BAM_CL73_CAP_25G_CR1_SET(bam73_cap) (bam73_cap |= PHYMOD_BAM_CL73_CAP_25G_CR1)

#define PHYMOD_BAM_CL73_CAP_20G_KR2_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_20G_KR2)
#define PHYMOD_BAM_CL73_CAP_20G_CR2_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_20G_CR2)
#define PHYMOD_BAM_CL73_CAP_40G_KR2_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_40G_KR2)
#define PHYMOD_BAM_CL73_CAP_40G_CR2_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_40G_CR2)
#define PHYMOD_BAM_CL73_CAP_50G_KR2_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_50G_KR2)
#define PHYMOD_BAM_CL73_CAP_50G_CR2_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_50G_CR2)
#define PHYMOD_BAM_CL73_CAP_50G_KR4_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_50G_KR4)
#define PHYMOD_BAM_CL73_CAP_50G_CR4_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_50G_CR4)
#define PHYMOD_BAM_CL73_CAP_20G_KR1_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_20G_KR1)
#define PHYMOD_BAM_CL73_CAP_20G_CR1_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_20G_CR1)
#define PHYMOD_BAM_CL73_CAP_25G_KR1_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_25G_KR1)
#define PHYMOD_BAM_CL73_CAP_25G_CR1_CLR(bam73_cap) (bam73_cap &= ~PHYMOD_BAM_CL73_CAP_25G_CR1)

#define PHYMOD_BAM_CL73_CAP_20G_KR2_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_20G_KR2 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_20G_CR2_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_20G_CR2 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_40G_KR2_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_40G_KR2 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_40G_CR2_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_40G_CR2 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_50G_KR2_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_50G_KR2 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_50G_CR2_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_50G_CR2 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_50G_KR4_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_50G_KR4 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_50G_CR4_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_50G_CR4 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_20G_KR1_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_20G_KR1 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_20G_CR1_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_20G_CR1 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_25G_KR1_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_25G_KR1 ? 1 : 0)
#define PHYMOD_BAM_CL73_CAP_25G_CR1_GET(bam73_cap) (bam73_cap & PHYMOD_BAM_CL73_CAP_25G_CR1 ? 1 : 0)

/*! 
 * @brief Capabilities for CL37BAM 
 */ 
#define PHYMOD_BAM_CL37_CAP_2P5G 0x1
#define PHYMOD_BAM_CL37_CAP_5G_X4 0x2
#define PHYMOD_BAM_CL37_CAP_6G_X4 0x4
#define PHYMOD_BAM_CL37_CAP_10G_HIGIG 0x8
#define PHYMOD_BAM_CL37_CAP_10G_CX4 0x10
#define PHYMOD_BAM_CL37_CAP_12G_X4 0x20
#define PHYMOD_BAM_CL37_CAP_12P5_X4 0x40
#define PHYMOD_BAM_CL37_CAP_13G_X4 0x80
#define PHYMOD_BAM_CL37_CAP_15G_X4 0x100
#define PHYMOD_BAM_CL37_CAP_16G_X4 0x200
#define PHYMOD_BAM_CL37_CAP_20G_X4_CX4 0x400
#define PHYMOD_BAM_CL37_CAP_20G_X4 0x800
#define PHYMOD_BAM_CL37_CAP_21G_X4 0x1000
#define PHYMOD_BAM_CL37_CAP_25P455G 0x2000
#define PHYMOD_BAM_CL37_CAP_31P5G 0x4000
#define PHYMOD_BAM_CL37_CAP_32P7G 0x8000
#define PHYMOD_BAM_CL37_CAP_40G 0x10000
#define PHYMOD_BAM_CL37_CAP_10G_X2_CX4 0x20000
#define PHYMOD_BAM_CL37_CAP_10G_DXGXS 0x40000
#define PHYMOD_BAM_CL37_CAP_10P5G_DXGXS 0x80000
#define PHYMOD_BAM_CL37_CAP_12P7_DXGXS 0x100000
#define PHYMOD_BAM_CL37_CAP_15P75G_R2 0x200000
#define PHYMOD_BAM_CL37_CAP_20G_X2_CX4 0x400000
#define PHYMOD_BAM_CL37_CAP_20G_X2 0x800000

#define PHYMOD_BAM_CL37_CAP_2P5G_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_2P5G)
#define PHYMOD_BAM_CL37_CAP_5G_X4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_5G_X4)
#define PHYMOD_BAM_CL37_CAP_6G_X4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_6G_X4)
#define PHYMOD_BAM_CL37_CAP_10G_HIGIG_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_10G_HIGIG)
#define PHYMOD_BAM_CL37_CAP_10G_CX4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_10G_CX4)
#define PHYMOD_BAM_CL37_CAP_12G_X4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_12G_X4)
#define PHYMOD_BAM_CL37_CAP_12P5_X4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_12P5_X4)
#define PHYMOD_BAM_CL37_CAP_13G_X4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_13G_X4)
#define PHYMOD_BAM_CL37_CAP_15G_X4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_15G_X4)
#define PHYMOD_BAM_CL37_CAP_16G_X4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_16G_X4)
#define PHYMOD_BAM_CL37_CAP_20G_X4_CX4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_20G_X4_CX4)
#define PHYMOD_BAM_CL37_CAP_20G_X4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_20G_X4)
#define PHYMOD_BAM_CL37_CAP_21G_X4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_21G_X4)
#define PHYMOD_BAM_CL37_CAP_25P455G_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_25P455G)
#define PHYMOD_BAM_CL37_CAP_31P5G_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_31P5G)
#define PHYMOD_BAM_CL37_CAP_32P7G_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_32P7G)
#define PHYMOD_BAM_CL37_CAP_40G_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_40G)
#define PHYMOD_BAM_CL37_CAP_10G_X2_CX4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_10G_X2_CX4)
#define PHYMOD_BAM_CL37_CAP_10G_DXGXS_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_10G_DXGXS)
#define PHYMOD_BAM_CL37_CAP_10P5G_DXGXS_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_10P5G_DXGXS)
#define PHYMOD_BAM_CL37_CAP_12P7_DXGXS_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_12P7_DXGXS)
#define PHYMOD_BAM_CL37_CAP_15P75G_R2_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_15P75G_R2)
#define PHYMOD_BAM_CL37_CAP_20G_X2_CX4_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_20G_X2_CX4)
#define PHYMOD_BAM_CL37_CAP_20G_X2_SET(bam37_cap) (bam37_cap |= PHYMOD_BAM_CL37_CAP_20G_X2)

#define PHYMOD_BAM_CL37_CAP_2P5G_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_2P5G)
#define PHYMOD_BAM_CL37_CAP_5G_X4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_5G_X4)
#define PHYMOD_BAM_CL37_CAP_6G_X4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_6G_X4)
#define PHYMOD_BAM_CL37_CAP_10G_HIGIG_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_10G_HIGIG)
#define PHYMOD_BAM_CL37_CAP_10G_CX4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_10G_CX4)
#define PHYMOD_BAM_CL37_CAP_12G_X4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_12G_X4)
#define PHYMOD_BAM_CL37_CAP_12P5_X4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_12P5_X4)
#define PHYMOD_BAM_CL37_CAP_13G_X4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_13G_X4)
#define PHYMOD_BAM_CL37_CAP_15G_X4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_15G_X4)
#define PHYMOD_BAM_CL37_CAP_16G_X4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_16G_X4)
#define PHYMOD_BAM_CL37_CAP_20G_X4_CX4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_20G_X4_CX4)
#define PHYMOD_BAM_CL37_CAP_20G_X4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_20G_X4)
#define PHYMOD_BAM_CL37_CAP_21G_X4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_21G_X4)
#define PHYMOD_BAM_CL37_CAP_25P455G_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_25P455G)
#define PHYMOD_BAM_CL37_CAP_31P5G_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_31P5G)
#define PHYMOD_BAM_CL37_CAP_32P7G_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_32P7G)
#define PHYMOD_BAM_CL37_CAP_40G_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_40G)
#define PHYMOD_BAM_CL37_CAP_10G_X2_CX4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_10G_X2_CX4)
#define PHYMOD_BAM_CL37_CAP_10G_DXGXS_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_10G_DXGXS)
#define PHYMOD_BAM_CL37_CAP_10P5G_DXGXS_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_10P5G_DXGXS)
#define PHYMOD_BAM_CL37_CAP_12P7_DXGXS_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_12P7_DXGXS)
#define PHYMOD_BAM_CL37_CAP_15P75G_R2_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_15P75G_R2)
#define PHYMOD_BAM_CL37_CAP_20G_X2_CX4_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_20G_X2_CX4)
#define PHYMOD_BAM_CL37_CAP_20G_X2_CLR(bam37_cap) (bam37_cap &= ~PHYMOD_BAM_CL37_CAP_20G_X2)

#define PHYMOD_BAM_CL37_CAP_2P5G_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_2P5G ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_5G_X4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_5G_X4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_6G_X4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_6G_X4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_10G_HIGIG_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_10G_HIGIG ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_10G_CX4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_10G_CX4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_12G_X4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_12G_X4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_12P5_X4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_12P5_X4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_13G_X4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_13G_X4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_15G_X4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_15G_X4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_16G_X4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_16G_X4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_20G_X4_CX4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_20G_X4_CX4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_20G_X4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_20G_X4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_21G_X4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_21G_X4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_25P455G_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_25P455G ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_31P5G_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_31P5G ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_32P7G_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_32P7G ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_40G_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_40G ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_10G_X2_CX4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_10G_X2_CX4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_10G_DXGXS_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_10G_DXGXS ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_10P5G_DXGXS_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_10P5G_DXGXS ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_12P7_DXGXS_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_12P7_DXGXS ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_15P75G_R2_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_15P75G_R2 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_20G_X2_CX4_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_20G_X2_CX4 ? 1 : 0)
#define PHYMOD_BAM_CL37_CAP_20G_X2_GET(bam37_cap) (bam37_cap & PHYMOD_BAM_CL37_CAP_20G_X2 ? 1 : 0)


/*!
 * @enum phymod_an_mode_type_e
 * @brief an mode type 
 */ 
typedef enum phymod_an_mode_type_e {
    phymod_AN_MODE_NONE = 0,
    phymod_AN_MODE_CL73,
    phymod_AN_MODE_CL37,
    phymod_AN_MODE_CL73BAM,
    phymod_AN_MODE_CL37BAM,
    phymod_AN_MODE_HPAM,
    phymod_AN_MODE_SGMII,
    phymod_AN_MODE_CL37BAM_10P9375G_VCO,
    phymod_AN_MODE_Count
} phymod_an_mode_type_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_an_mode_type_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_an_mode_type_t validation */
int phymod_an_mode_type_t_validate(phymod_an_mode_type_t phymod_an_mode_type);

/*!
 * @enum phymod_cl37_sgmii_speed_e
 * @brief cl37 sgmii speed type 
 */ 
typedef enum phymod_cl37_sgmii_speed_e {
    phymod_CL37_SGMII_10M = 0,
    phymod_CL37_SGMII_100M,
    phymod_CL37_SGMII_1000M,
    phymod_CL37_SGMII_Count
} phymod_cl37_sgmii_speed_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_cl37_sgmii_speed_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_cl37_sgmii_speed_t validation */
int phymod_cl37_sgmii_speed_t_validate(phymod_cl37_sgmii_speed_t phymod_cl37_sgmii_speed);

/*!
 * @struct phymod_autoneg_ability_s
 * @brief autoneg_ability 
 */ 
typedef struct phymod_autoneg_ability_s {
    uint32_t an_cap;
    uint32_t cl73bam_cap;
    uint32_t cl37bam_cap;
    uint32_t an_fec;
    uint32_t an_cl72;
    uint32_t an_hg2;
    uint32_t capabilities;
    phymod_cl37_sgmii_speed_t sgmii_speed;
    uint32_t an_master_lane; /**< Master lane belongs to port. For 10 lane port[0-9], for 4 lane port [0-3], for 2 lane port [0-1]. This paramter is ignored for single lane port */
} phymod_autoneg_ability_t;

/* phymod_autoneg_ability_t initialization and validation */
int phymod_autoneg_ability_t_validate(const phymod_autoneg_ability_t* phymod_autoneg_ability);
int phymod_autoneg_ability_t_init(phymod_autoneg_ability_t* phymod_autoneg_ability);

/*! 
 * @brief Capabilities for phymod_phy_autoneg_set API 
 */ 
#define PHYMOD_AN_CAP_CL37 0x1
#define PHYMOD_AN_CAP_CL73 0x2
#define PHYMOD_AN_CAP_CL37BAM 0x4
#define PHYMOD_AN_CAP_CL73BAM 0x8
#define PHYMOD_AN_CAP_HPAM 0x10
#define PHYMOD_AN_CAP_SGMII 0x20
#define PHYMOD_AN_CAP_SYMM_PAUSE 0x40
#define PHYMOD_AN_CAP_ASYM_PAUSE 0x80

#define PHYMOD_AN_CAP_CL37_SET(an) ((an)->capabilities |= PHYMOD_AN_CAP_CL37)
#define PHYMOD_AN_CAP_CL73_SET(an) ((an)->capabilities |= PHYMOD_AN_CAP_CL73)
#define PHYMOD_AN_CAP_CL37BAM_SET(an) ((an)->capabilities |= PHYMOD_AN_CAP_CL37BAM)
#define PHYMOD_AN_CAP_CL73BAM_SET(an) ((an)->capabilities |= PHYMOD_AN_CAP_CL73BAM)
#define PHYMOD_AN_CAP_HPAM_SET(an) ((an)->capabilities |= PHYMOD_AN_CAP_HPAM)
#define PHYMOD_AN_CAP_SGMII_SET(an) ((an)->capabilities |= PHYMOD_AN_CAP_SGMII)
#define PHYMOD_AN_CAP_SYMM_PAUSE_SET(an) ((an)->capabilities |= PHYMOD_AN_CAP_SYMM_PAUSE)
#define PHYMOD_AN_CAP_ASYM_PAUSE_SET(an) ((an)->capabilities |= PHYMOD_AN_CAP_ASYM_PAUSE)

#define PHYMOD_AN_CAP_CL37_CLR(an) ((an)->capabilities &= ~PHYMOD_AN_CAP_CL37)
#define PHYMOD_AN_CAP_CL73_CLR(an) ((an)->capabilities &= ~PHYMOD_AN_CAP_CL73)
#define PHYMOD_AN_CAP_CL37BAM_CLR(an) ((an)->capabilities &= ~PHYMOD_AN_CAP_CL37BAM)
#define PHYMOD_AN_CAP_CL73BAM_CLR(an) ((an)->capabilities &= ~PHYMOD_AN_CAP_CL73BAM)
#define PHYMOD_AN_CAP_HPAM_CLR(an) ((an)->capabilities &= ~PHYMOD_AN_CAP_HPAM)
#define PHYMOD_AN_CAP_SGMII_CLR(an) ((an)->capabilities &= ~PHYMOD_AN_CAP_SGMII)
#define PHYMOD_AN_CAP_SYMM_PAUSE_CLR(an) ((an)->capabilities &= ~PHYMOD_AN_CAP_SYMM_PAUSE)
#define PHYMOD_AN_CAP_ASYM_PAUSE_CLR(an) ((an)->capabilities &= ~PHYMOD_AN_CAP_ASYM_PAUSE)

#define PHYMOD_AN_CAP_CL37_GET(an) ((an)->capabilities & PHYMOD_AN_CAP_CL37 ? 1 : 0)
#define PHYMOD_AN_CAP_CL73_GET(an) ((an)->capabilities & PHYMOD_AN_CAP_CL73 ? 1 : 0)
#define PHYMOD_AN_CAP_CL37BAM_GET(an) ((an)->capabilities & PHYMOD_AN_CAP_CL37BAM ? 1 : 0)
#define PHYMOD_AN_CAP_CL73BAM_GET(an) ((an)->capabilities & PHYMOD_AN_CAP_CL73BAM ? 1 : 0)
#define PHYMOD_AN_CAP_HPAM_GET(an) ((an)->capabilities & PHYMOD_AN_CAP_HPAM ? 1 : 0)
#define PHYMOD_AN_CAP_SGMII_GET(an) ((an)->capabilities & PHYMOD_AN_CAP_SGMII ? 1 : 0)
#define PHYMOD_AN_CAP_SYMM_PAUSE_GET(an) ((an)->capabilities & PHYMOD_AN_CAP_SYMM_PAUSE ? 1 : 0)
#define PHYMOD_AN_CAP_ASYM_PAUSE_GET(an) ((an)->capabilities & PHYMOD_AN_CAP_ASYM_PAUSE ? 1 : 0)

/*! 
 * @brief Flags for autoneg_set API 
 */ 
#define PHYMOD_AN_F_ALLOW_PLL_CHANGE 0x1
#define PHYMOD_AN_F_SET_PRIOR_ENABLE 0x2
#define PHYMOD_AN_F_ALLOW_CL72_CONFIG_CHANGE 0x4

#define PHYMOD_AN_F_ALLOW_PLL_CHANGE_SET(an) ((an)->flags |= PHYMOD_AN_F_ALLOW_PLL_CHANGE)
#define PHYMOD_AN_F_SET_PRIOR_ENABLE_SET(an) ((an)->flags |= PHYMOD_AN_F_SET_PRIOR_ENABLE)
#define PHYMOD_AN_F_ALLOW_CL72_CONFIG_CHANGE_SET(an) ((an)->flags |= PHYMOD_AN_F_ALLOW_CL72_CONFIG_CHANGE)

#define PHYMOD_AN_F_ALLOW_PLL_CHANGE_CLR(an) ((an)->flags &= ~PHYMOD_AN_F_ALLOW_PLL_CHANGE)
#define PHYMOD_AN_F_SET_PRIOR_ENABLE_CLR(an) ((an)->flags &= ~PHYMOD_AN_F_SET_PRIOR_ENABLE)
#define PHYMOD_AN_F_ALLOW_CL72_CONFIG_CHANGE_CLR(an) ((an)->flags &= ~PHYMOD_AN_F_ALLOW_CL72_CONFIG_CHANGE)

#define PHYMOD_AN_F_ALLOW_PLL_CHANGE_GET(an) ((an)->flags & PHYMOD_AN_F_ALLOW_PLL_CHANGE ? 1 : 0)
#define PHYMOD_AN_F_SET_PRIOR_ENABLE_GET(an) ((an)->flags & PHYMOD_AN_F_SET_PRIOR_ENABLE ? 1 : 0)
#define PHYMOD_AN_F_ALLOW_CL72_CONFIG_CHANGE_GET(an) ((an)->flags & PHYMOD_AN_F_ALLOW_CL72_CONFIG_CHANGE ? 1 : 0)

/*! 
 * phymod_phy_autoneg_ability_set
 *
 * @brief Set/Get autoneg 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  an_ability_set_type   - 
 */
int phymod_phy_autoneg_ability_set(const phymod_phy_access_t* phy, const phymod_autoneg_ability_t* an_ability_set_type);
/*! 
 * phymod_phy_autoneg_ability_get
 *
 * @brief Set/Get autoneg 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  an_ability_get_type   - 
 */
int phymod_phy_autoneg_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

/*! 
 * phymod_phy_autoneg_remote_ability_get
 *
 * @brief Get  remote link autoneg 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  an_ability_get_type   - 
 */
int phymod_phy_autoneg_remote_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

typedef struct phymod_autoneg_control_s {
    phymod_an_mode_type_t an_mode;
    uint32_t num_lane_adv; /**< The number of lanes the autoneg advert */
    uint32_t flags; /**< see AN_F */
    uint32_t enable;
} phymod_autoneg_control_t;

/* phymod_autoneg_control_t initialization and validation */
int phymod_autoneg_control_t_validate(const phymod_autoneg_control_t* phymod_autoneg_control);
int phymod_autoneg_control_t_init(phymod_autoneg_control_t* phymod_autoneg_control);

/*! 
 * phymod_phy_autoneg_set
 *
 * @brief Set/Get autoneg 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  an              - 
 */
int phymod_phy_autoneg_set(const phymod_phy_access_t* phy, const phymod_autoneg_control_t* an);
/*! 
 * phymod_phy_autoneg_get
 *
 * @brief Set/Get autoneg 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  an              - 
 * @param [out]  an_done         - 
 */
int phymod_phy_autoneg_get(const phymod_phy_access_t* phy, phymod_autoneg_control_t* an, uint32_t* an_done);

typedef struct phymod_autoneg_status_s {
    uint32_t enabled;
    uint32_t locked;
    uint32_t data_rate;
    phymod_interface_t interface;
} phymod_autoneg_status_t;

/* phymod_autoneg_status_t initialization and validation */
int phymod_autoneg_status_t_validate(const phymod_autoneg_status_t* phymod_autoneg_status);
int phymod_autoneg_status_t_init(phymod_autoneg_status_t* phymod_autoneg_status);

/*! 
 * phymod_phy_autoneg_status_get
 *
 * @brief Get Autoneg status 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  status          - 
 */
int phymod_phy_autoneg_status_get(const phymod_phy_access_t* phy, phymod_autoneg_status_t* status);


/*!
 * @enum phymod_firmware_load_method_e
 * @brief Firmware load method 
 */ 
typedef enum phymod_firmware_load_method_e {
    phymodFirmwareLoadMethodNone = 0, /**< Don't load FW */
    phymodFirmwareLoadMethodInternal, /**< Load FW internaly */
    phymodFirmwareLoadMethodExternal, /**< Load FW by a given function */
    phymodFirmwareLoadMethodProgEEPROM, /**< Program EEPROM */
    phymodFirmwareLoadMethodCount
} phymod_firmware_load_method_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_firmware_load_method_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_firmware_load_method_t validation */
int phymod_firmware_load_method_t_validate(phymod_firmware_load_method_t phymod_firmware_load_method);
/*! 
 * @brief Core init flags 
 */ 
#define PHYMOD_CORE_INIT_F_UNTIL_FW_LOAD 0x1 /**< Run init sequence until FW load stage (without loading FW) */
#define PHYMOD_CORE_INIT_F_RESUME_AFTER_FW_LOAD 0x2 /**< Run init sequence from FW loading stage (without loading FW) */
#define PHYMOD_CORE_INIT_F_FIRMWARE_LOAD_VERIFY 0x4 /**< Verify FW loaded correctly */
#define PHYMOD_CORE_INIT_F_EXECUTE_PASS1 0x8 /**< core init state pass1 */
#define PHYMOD_CORE_INIT_F_EXECUTE_PASS2 0x10 /**< core init state pass2 */
#define PHYMOD_CORE_INIT_F_BYPASS_CRC_CHECK 0x20 /**< by pass crc check */
#define PHYMOD_CORE_INIT_F_FW_FORCE_DOWNLOAD 0x40 /**< force the FW download */
#define PHYMOD_CORE_INIT_F_EXECUTE_FW_LOAD 0x80 /**< Perform the firmware load */
#define PHYMOD_CORE_INIT_F_RESET_CORE_FOR_FW_LOAD 0x100 /**< Reset Core for F/W Load */

#define PHYMOD_CORE_INIT_F_UNTIL_FW_LOAD_SET(conf) ((conf)->flags |= PHYMOD_CORE_INIT_F_UNTIL_FW_LOAD)
#define PHYMOD_CORE_INIT_F_RESUME_AFTER_FW_LOAD_SET(conf) ((conf)->flags |= PHYMOD_CORE_INIT_F_RESUME_AFTER_FW_LOAD)
#define PHYMOD_CORE_INIT_F_FIRMWARE_LOAD_VERIFY_SET(conf) ((conf)->flags |= PHYMOD_CORE_INIT_F_FIRMWARE_LOAD_VERIFY)
#define PHYMOD_CORE_INIT_F_EXECUTE_PASS1_SET(conf) ((conf)->flags |= PHYMOD_CORE_INIT_F_EXECUTE_PASS1)
#define PHYMOD_CORE_INIT_F_EXECUTE_PASS2_SET(conf) ((conf)->flags |= PHYMOD_CORE_INIT_F_EXECUTE_PASS2)
#define PHYMOD_CORE_INIT_F_BYPASS_CRC_CHECK_SET(conf) ((conf)->flags |= PHYMOD_CORE_INIT_F_BYPASS_CRC_CHECK)
#define PHYMOD_CORE_INIT_F_FW_FORCE_DOWNLOAD_SET(conf) ((conf)->flags |= PHYMOD_CORE_INIT_F_FW_FORCE_DOWNLOAD)
#define PHYMOD_CORE_INIT_F_EXECUTE_FW_LOAD_SET(conf) ((conf)->flags |= PHYMOD_CORE_INIT_F_EXECUTE_FW_LOAD)
#define PHYMOD_CORE_INIT_F_RESET_CORE_FOR_FW_LOAD_SET(conf) ((conf)->flags |= PHYMOD_CORE_INIT_F_RESET_CORE_FOR_FW_LOAD)

#define PHYMOD_CORE_INIT_F_UNTIL_FW_LOAD_CLR(conf) ((conf)->flags &= ~PHYMOD_CORE_INIT_F_UNTIL_FW_LOAD)
#define PHYMOD_CORE_INIT_F_RESUME_AFTER_FW_LOAD_CLR(conf) ((conf)->flags &= ~PHYMOD_CORE_INIT_F_RESUME_AFTER_FW_LOAD)
#define PHYMOD_CORE_INIT_F_FIRMWARE_LOAD_VERIFY_CLR(conf) ((conf)->flags &= ~PHYMOD_CORE_INIT_F_FIRMWARE_LOAD_VERIFY)
#define PHYMOD_CORE_INIT_F_EXECUTE_PASS1_CLR(conf) ((conf)->flags &= ~PHYMOD_CORE_INIT_F_EXECUTE_PASS1)
#define PHYMOD_CORE_INIT_F_EXECUTE_PASS2_CLR(conf) ((conf)->flags &= ~PHYMOD_CORE_INIT_F_EXECUTE_PASS2)
#define PHYMOD_CORE_INIT_F_BYPASS_CRC_CHECK_CLR(conf) ((conf)->flags &= ~PHYMOD_CORE_INIT_F_BYPASS_CRC_CHECK)
#define PHYMOD_CORE_INIT_F_FW_FORCE_DOWNLOAD_CLR(conf) ((conf)->flags &= ~PHYMOD_CORE_INIT_F_FW_FORCE_DOWNLOAD)
#define PHYMOD_CORE_INIT_F_EXECUTE_FW_LOAD_CLR(conf) ((conf)->flags &= ~PHYMOD_CORE_INIT_F_EXECUTE_FW_LOAD)
#define PHYMOD_CORE_INIT_F_RESET_CORE_FOR_FW_LOAD_CLR(conf) ((conf)->flags &= ~PHYMOD_CORE_INIT_F_RESET_CORE_FOR_FW_LOAD)

#define PHYMOD_CORE_INIT_F_UNTIL_FW_LOAD_GET(conf) ((conf)->flags & PHYMOD_CORE_INIT_F_UNTIL_FW_LOAD ? 1 : 0)
#define PHYMOD_CORE_INIT_F_RESUME_AFTER_FW_LOAD_GET(conf) ((conf)->flags & PHYMOD_CORE_INIT_F_RESUME_AFTER_FW_LOAD ? 1 : 0)
#define PHYMOD_CORE_INIT_F_FIRMWARE_LOAD_VERIFY_GET(conf) ((conf)->flags & PHYMOD_CORE_INIT_F_FIRMWARE_LOAD_VERIFY ? 1 : 0)
#define PHYMOD_CORE_INIT_F_EXECUTE_PASS1_GET(conf) ((conf)->flags & PHYMOD_CORE_INIT_F_EXECUTE_PASS1 ? 1 : 0)
#define PHYMOD_CORE_INIT_F_EXECUTE_PASS2_GET(conf) ((conf)->flags & PHYMOD_CORE_INIT_F_EXECUTE_PASS2 ? 1 : 0)
#define PHYMOD_CORE_INIT_F_BYPASS_CRC_CHECK_GET(conf) ((conf)->flags & PHYMOD_CORE_INIT_F_BYPASS_CRC_CHECK ? 1 : 0)
#define PHYMOD_CORE_INIT_F_FW_FORCE_DOWNLOAD_GET(conf) ((conf)->flags & PHYMOD_CORE_INIT_F_FW_FORCE_DOWNLOAD ? 1 : 0)
#define PHYMOD_CORE_INIT_F_EXECUTE_FW_LOAD_GET(conf) ((conf)->flags & PHYMOD_CORE_INIT_F_EXECUTE_FW_LOAD ? 1 : 0)
#define PHYMOD_CORE_INIT_F_RESET_CORE_FOR_FW_LOAD_GET(conf) ((conf)->flags & PHYMOD_CORE_INIT_F_RESET_CORE_FOR_FW_LOAD ? 1 : 0)


/*!
 * @enum phymod_datapath_e
 * @brief Datapath 
 */ 
typedef enum phymod_datapath_e {
    phymodDatapathNormal = 0, /**< Normal Datapath */
    phymodDatapathUll, /**< Ultra Low Latency Datapath, provide minimum latency */
    phymodDatapathCount
} phymod_datapath_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_datapath_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_datapath_t validation */
int phymod_datapath_t_validate(phymod_datapath_t phymod_datapath);

/*!
 * @enum phymod_tx_input_voltage_e
 * @brief Tx input voltage 
 */ 
typedef enum phymod_tx_input_voltage_e {
    phymodTxInputVoltageDefault = 0, /**< Default */
    phymodTxInputVoltage1p00, /**< Tx input voltage as 1.00v */
    phymodTxInputVoltage1p25, /**< Tx input voltage as 1.25v */
    phymodTxInputVoltageCount
} phymod_tx_input_voltage_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_tx_input_voltage_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_tx_input_voltage_t validation */
int phymod_tx_input_voltage_t_validate(phymod_tx_input_voltage_t phymod_tx_input_voltage);

/*!
 * @struct phymod_afe_pll_s
 * @brief AFE/PLL related parameter 
 */ 
typedef struct phymod_afe_pll_s {
    uint32_t afe_pll_change_default; /**< If 1 change AFE/PLL reg */
    uint32_t ams_pll_iqp;
    uint32_t ams_pll_en_hrz;
} phymod_afe_pll_t;

/* phymod_afe_pll_t initialization and validation */
int phymod_afe_pll_t_validate(const phymod_afe_pll_t* phymod_afe_pll);
int phymod_afe_pll_t_init(phymod_afe_pll_t* phymod_afe_pll);

typedef struct phymod_core_init_config_s {
    phymod_lane_map_t lane_map;
    phymod_polarity_t polarity_map;
    phymod_firmware_load_method_t firmware_load_method;
    phymod_firmware_loader_f firmware_loader;
    phymod_firmware_core_config_t firmware_core_config;
    phymod_phy_inf_config_t interface; /**< init values for all lanes */
    uint32_t flags; /**< init flags */
    phymod_datapath_t op_datapath; /**< Datapath */
    int rx_fifo_sync_offset; /**< Extra cycles between read and write pointer in the RX data path FIFO, it also accept negative values */
    int tx_fifo_sync_offset; /**< Extra cycles between read and write pointer in the TX data path FIFO, it also accepts negative values */
    phymod_tx_input_voltage_t tx_input_voltage; /**< Tx input voltage */
    phymod_afe_pll_t afe_pll; /**< AFE/PLL register value */
    uint8_t core_mode;
} phymod_core_init_config_t;

/* phymod_core_init_config_t initialization and validation */
int phymod_core_init_config_t_validate(const phymod_core_init_config_t* phymod_core_init_config);
int phymod_core_init_config_t_init(phymod_core_init_config_t* phymod_core_init_config);

typedef struct phymod_core_status_s {
    uint32_t pmd_active;
} phymod_core_status_t;

/* phymod_core_status_t initialization and validation */
int phymod_core_status_t_validate(const phymod_core_status_t* phymod_core_status);
int phymod_core_status_t_init(phymod_core_status_t* phymod_core_status);

/*! 
 * phymod_core_init
 *
 * @brief Core Initialization 
 *
 * @param [in]  core            - core access information
 * @param [in]  init_config     - 
 * @param [in]  core_status     - 
 */
int phymod_core_init(const phymod_core_access_t* core, const phymod_core_init_config_t* init_config, const phymod_core_status_t* core_status);

/*! 
 * phymod_phy_pll_multiplier_get
 *
 * @brief Core vco freq get function 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  core_vco_pll_multiplier   - 
 */
int phymod_phy_pll_multiplier_get(const phymod_phy_access_t* phy, uint32_t* core_vco_pll_multiplier);


/*!
 * @enum phymod_operation_mode_e
 * @brief Operation mode 
 */ 
typedef enum phymod_operation_mode_e {
    phymodOperationModeRetimer = 0, /**< Retimer mode */
    phymodOperationModeRepeater, /**< Repeater Mode */
    phymodOperationModeCount
} phymod_operation_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_operation_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_operation_mode_t validation */
int phymod_operation_mode_t_validate(phymod_operation_mode_t phymod_operation_mode);

/*!
 * @enum phymod_autoneg_link_qualifier_e
 * @brief AN link qualifier type 
 */ 
typedef enum phymod_autoneg_link_qualifier_e {
    phymodAutonegLinkQualifierRegisterWrite = 0, /**< Qualify link based on Register Write */
    phymodAutonegLinkQualifierKRBlockLock, /**< Qualify link based on KR Block lock */
    phymodAutonegLinkQualifierKR4BlockLock, /**< Qualify link based on KR4 Block Lock */
    phymodAutonegLinkQualifierKR4PMDLock, /**< Qualify link based on KR4 PMD lock */
    phymodAutonegLinkQualifierExternalPCS, /**< Qualify link based on External PCS */
    phymodAutonegLinkQualifierDefault, /**< HW default */
    phymodAutonegLinkQualifierCount
} phymod_autoneg_link_qualifier_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_autoneg_link_qualifier_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_autoneg_link_qualifier_t validation */
int phymod_autoneg_link_qualifier_t_validate(phymod_autoneg_link_qualifier_t phymod_autoneg_link_qualifier);
typedef struct phymod_rx_los_s {
    uint32_t rx_los_en;
    uint32_t rx_los_invert_en;
} phymod_rx_los_t;

/* phymod_rx_los_t initialization and validation */
int phymod_rx_los_t_validate(const phymod_rx_los_t* phymod_rx_los);
int phymod_rx_los_t_init(phymod_rx_los_t* phymod_rx_los);

typedef struct phymod_phy_init_config_s {
    phymod_polarity_t polarity;
    uint32_t tx_params_user_flag[PHYMOD_MAX_LANES_PER_CORE];
    phymod_tx_t tx[PHYMOD_MAX_LANES_PER_CORE];
    uint32_t cl72_en;
    uint32_t an_en;
    phymod_operation_mode_t op_mode;
    phymod_autoneg_link_qualifier_t an_link_qualifier;
    phymod_phy_inf_config_t interface; /**< init values for all lanes */
    phymod_rx_los_t rx_los; /**< rx los config for all lanes */
    uint32_t ext_phy_tx_params_user_flag[PHYMOD_MAX_LANES_PER_CORE];
    phymod_tx_t ext_phy_tx[PHYMOD_MAX_LANES_PER_CORE];
} phymod_phy_init_config_t;

/* phymod_phy_init_config_t initialization and validation */
int phymod_phy_init_config_t_validate(const phymod_phy_init_config_t* phymod_phy_init_config);
int phymod_phy_init_config_t_init(phymod_phy_init_config_t* phymod_phy_init_config);

/*! 
 * phymod_phy_init
 *
 * @brief Phy Initialization 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  init_config     - 
 */
int phymod_phy_init(const phymod_phy_access_t* phy, const phymod_phy_init_config_t* init_config);


/*!
 * @enum phymod_loopback_mode_e
 * @brief Loopback modes 
 */ 
typedef enum phymod_loopback_mode_e {
    phymodLoopbackGlobal = 0,
    phymodLoopbackGlobalPMD,
    phymodLoopbackRemotePMD,
    phymodLoopbackRemotePCS,
    phymodLoopbackCount
} phymod_loopback_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_loopback_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_loopback_mode_t validation */
int phymod_loopback_mode_t_validate(phymod_loopback_mode_t phymod_loopback_mode);
/*! 
 * phymod_phy_loopback_set
 *
 * @brief Set/get loopback mode 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  loopback        - loopback mode
 * @param [in]  enable          - is loopback set
 */
int phymod_phy_loopback_set(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t enable);
/*! 
 * phymod_phy_loopback_get
 *
 * @brief Set/get loopback mode 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  loopback        - loopback mode
 * @param [out]  enable          - is loopback set
 */
int phymod_phy_loopback_get(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t* enable);

/*! 
 * phymod_phy_rx_pmd_locked_get
 *
 * @brief Get rx pmd locked indication 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  rx_pmd_locked   - bmap of rx pmd locked indications
 */
int phymod_phy_rx_pmd_locked_get(const phymod_phy_access_t* phy, uint32_t* rx_pmd_locked);

/*! 
 * phymod_phy_rx_signal_detect_get
 *
 * @brief Get rx pmd locked indication 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  rx_signal_detect   - signal detect indication
 */
int phymod_phy_rx_signal_detect_get(const phymod_phy_access_t* phy, uint32_t* rx_signal_detect);

/*! 
 * phymod_phy_link_status_get
 *
 * @brief Get link up status indication 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  link_status     - 
 */
int phymod_phy_link_status_get(const phymod_phy_access_t* phy, uint32_t* link_status);

/*! 
 * phymod_phy_status_dump
 *
 * @brief Get the serdes status 
 *
 * @param [in]  phy             - phy access information
 */
int phymod_phy_status_dump(const phymod_phy_access_t* phy);


/*!
 * @enum phymod_pcs_userspeed_mode_e
 * @brief modes for phymod_phy_pcs_userspeed API 
 */ 
typedef enum phymod_pcs_userspeed_mode_e {
    phymodPcsUserSpeedModeST = 0, /**< PCS Sw Table */
    phymodPcsUserSpeedModeHTO, /**< PCS Hw Table Override */
    phymodPcsUserSpeedModeCount
} phymod_pcs_userspeed_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_pcs_userspeed_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_pcs_userspeed_mode_t validation */
int phymod_pcs_userspeed_mode_t_validate(phymod_pcs_userspeed_mode_t phymod_pcs_userspeed_mode);

/*!
 * @enum phymod_pcs_userspeed_param_e
 * @brief parameters for phymod_phy_pcs_userspeed API 
 */ 
typedef enum phymod_pcs_userspeed_param_e {
    phymodPcsUserSpeedParamEntry = 0, /**< ST current entry */
    phymodPcsUserSpeedParamHCD, /**< ST HCD */
    phymodPcsUserSpeedParamClear, /**< HTO Clear */
    phymodPcsUserSpeedParamPllDiv, /**< ST/HTO PLL DIV */
    phymodPcsUserSpeedParamPmaOS, /**< ST/HTO PMA OS */
    phymodPcsUserSpeedParamScramble, /**< Scramble mode */
    phymodPcsUserSpeedParamEncode, /**< Encode mode */
    phymodPcsUserSpeedParamCl48CheckEnd, /**< CL48 Check end */
    phymodPcsUserSpeedParamBlkSync, /**< Block sync mode */
    phymodPcsUserSpeedParamReorder, /**< Reorder mode */
    phymodPcsUserSpeedParamCl36Enable, /**< CL36 enable */
    phymodPcsUserSpeedParamDescr1, /**< Descramble1 mode */
    phymodPcsUserSpeedParamDecode1, /**< Decode1 mode */
    phymodPcsUserSpeedParamDeskew, /**< Deskew mode */
    phymodPcsUserSpeedParamDescr2, /**< Descramble2 mode */
    phymodPcsUserSpeedParamDescr2ByteDel,
    phymodPcsUserSpeedParamBrcm64B66, /**< Drcm64/66 descramble */
    phymodPcsUserSpeedParamSgmii, /**< SGMII mode */
    phymodPcsUserSpeedParamClkcnt0, /**< clock count0 */
    phymodPcsUserSpeedParamClkcnt1, /**< clock count1 */
    phymodPcsUserSpeedParamLpcnt0, /**< Loop count0 */
    phymodPcsUserSpeedParamLpcnt1, /**< Loop count1 */
    phymodPcsUserSpeedParamMacCGC, /**< Mac CGC */
    phymodPcsUserSpeedParamRepcnt, /**< Repeat Count */
    phymodPcsUserSpeedParamCrdtEn, /**< Credit Enable */
    phymodPcsUserSpeedParamPcsClkcnt, /**< PCS clock count */
    phymodPcsUserSpeedParamPcsCGC, /**< PCS CGC */
    phymodPcsUserSpeedParamCl72En, /**< Cl72 enable */
    phymodPcsUserSpeedParamNumOfLanes, /**< Num of lanes */
    phymodPcsUserSpeedParamCount
} phymod_pcs_userspeed_param_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_pcs_userspeed_param_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_pcs_userspeed_param_t validation */
int phymod_pcs_userspeed_param_t_validate(phymod_pcs_userspeed_param_t phymod_pcs_userspeed_param);
typedef struct phymod_pcs_userspeed_config_s {
    phymod_pcs_userspeed_mode_t mode;
    phymod_pcs_userspeed_param_t param;
    uint32_t value;
    uint16_t current_entry;
} phymod_pcs_userspeed_config_t;

/* phymod_pcs_userspeed_config_t initialization and validation */
int phymod_pcs_userspeed_config_t_validate(const phymod_pcs_userspeed_config_t* phymod_pcs_userspeed_config);
int phymod_pcs_userspeed_config_t_init(phymod_pcs_userspeed_config_t* phymod_pcs_userspeed_config);

/*! 
 * phymod_phy_pcs_userspeed_set
 *
 * @brief Set/Get User Speed Paramateres 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  config          - 
 */
int phymod_phy_pcs_userspeed_set(const phymod_phy_access_t* phy, const phymod_pcs_userspeed_config_t* config);
/*! 
 * phymod_phy_pcs_userspeed_get
 *
 * @brief Set/Get User Speed Paramateres 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  config          - 
 */
int phymod_phy_pcs_userspeed_get(const phymod_phy_access_t* phy, phymod_pcs_userspeed_config_t* config);

/*! 
 * phymod_phy_reg_read
 *
 * @brief Read phymod register 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  reg_addr        - Register address
 * @param [out]  val             - read value
 */
int phymod_phy_reg_read(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t* val);

/*! 
 * phymod_phy_reg_write
 *
 * @brief Write phymod register 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  reg_addr        - Register address
 * @param [in]  val             - write value
 */
int phymod_phy_reg_write(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t val);

/*! 
 * phymod_phy_rev_id
 *
 * @brief Read Revision id 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  rev_id          - Revision id
 */
int phymod_phy_rev_id(const phymod_phy_access_t* phy, uint32_t* rev_id);

/*! 
 * phymod_phy_lane_cross_switch_map_set
 *
 * @brief Get/Set cross switch swap 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  tx_array        - lane_map[x]=y means that Output data on Physical TX lane x will come from corresponding physical RX lane y
 */
int phymod_phy_lane_cross_switch_map_set(const phymod_phy_access_t* phy, const uint32_t* tx_array);
/*! 
 * phymod_phy_lane_cross_switch_map_get
 *
 * @brief Get/Set cross switch swap 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  tx_array        - lane_map[x]=y means that Output data on Physical TX lane x will come from corresponding physical RX lane y
 */
int phymod_phy_lane_cross_switch_map_get(const phymod_phy_access_t* phy, uint32_t* tx_array);

/*! 
 * @brief PHY interrupt sources 
 */ 
#define PHYMOD_INTR_TIMESYNC_FRAMESYNC 0x1 /**< Timesync framesync interrupt */
#define PHYMOD_INTR_TIMESYNC_TIMESTAMP 0x2 /**< Timesync timestamp interrupt */
#define PHYMOD_INTR_LINK_EVENT 0x4 /**< Link status event interrupt (link up/down, change in signal detect, etc.) */
#define PHYMOD_INTR_AUTONEG_EVENT 0x8 /**< Auto-negotiation event interrupt (auto-neg start/done) */
#define PHYMOD_INTR_PLL_EVENT 0x10 /**< PLL status event interrupt (lock lost/found) */
#define PHYMOD_INTR_UC_EVENT 0x20 /**< Microcontroller event interrupt (e.g. uC messaging event) */
#define PHYMOD_INTR_EMON_EVENT 0x40 /**< Error monitor event interrupt (e.g. PCS monitor event) */
#define PHYMOD_INTR_AUX_EVENT 0x80 /**< Non-standard PHY event interrupt */

/*! 
 * phymod_phy_intr_enable_set
 *
 * @brief Get/Set PHY interrupt enable mask 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  enable          - Mask of interrupts to enable/disable. Bit set means enable interrupt. See PHYMOD_INTR_xxx for list of supported interrupts.
 */
int phymod_phy_intr_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
/*! 
 * phymod_phy_intr_enable_get
 *
 * @brief Get/Set PHY interrupt enable mask 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  enable          - Mask of enabled interrupts. Bit set means interrupt enabled. See PHYMOD_INTR_xxx for list of supported interrupts.
 */
int phymod_phy_intr_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*! 
 * phymod_phy_intr_status_get
 *
 * @brief PHY interrupt status get 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  intr_status     - Get mask of active interrupts. See PHYMOD_INTR_xxx for list of supported interrupts.
 */
int phymod_phy_intr_status_get(const phymod_phy_access_t* phy, uint32_t* intr_status);

/*! 
 * phymod_phy_intr_status_clear
 *
 * @brief Clear PHY interrupt status  
 *
 * @param [in]  phy             - phy access information
 * @param [in]  intr_clr        - Mask of interrupts to be cleared. See PHYMOD_INTR_xxx for list of supported interrupts.
 */
int phymod_phy_intr_status_clear(const phymod_phy_access_t* phy, uint32_t intr_clr);

/*! 
 * phymod_phy_i2c_read
 *
 * @brief Read data from I2C device attached to PHY 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  flags           - Optional device flags (currently unused)
 * @param [in]  addr            - I2C slave address
 * @param [in]  offset          - Offset in I2C device to read from
 * @param [in]  size            - Number of bytes to read
 * @param [out]  data            - 
 */
int phymod_phy_i2c_read(const phymod_phy_access_t* phy, uint32_t flags, uint32_t addr, uint32_t offset, uint32_t size, uint8_t* data);

/*! 
 * phymod_phy_i2c_write
 *
 * @brief Write data to I2C device attached to PHY 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  flags           - Optional device flags (currently unused)
 * @param [in]  addr            - I2C slave address
 * @param [in]  offset          - Offset in I2C device to write to
 * @param [in]  size            - Number of bytes to write
 * @param [in]  data            - 
 */
int phymod_phy_i2c_write(const phymod_phy_access_t* phy, uint32_t flags, uint32_t addr, uint32_t offset, uint32_t size, const uint8_t* data);


/*!
 * @enum phymod_gpio_mode_e
 * @brief GPIO pin mode 
 */ 
typedef enum phymod_gpio_mode_e {
    phymodGpioModeDisabled = 0,
    phymodGpioModeOutput,
    phymodGpioModeInput,
    phymodGpioModeCount
} phymod_gpio_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_gpio_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_gpio_mode_t validation */
int phymod_gpio_mode_t_validate(phymod_gpio_mode_t phymod_gpio_mode);
/*! 
 * phymod_phy_gpio_config_set
 *
 * @brief Set/Get the configuration of a PHY GPIO pin 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  pin_no          - GPIO pin number [0-7]
 * @param [in]  gpio_mode       - GPIO mode
 */
int phymod_phy_gpio_config_set(const phymod_phy_access_t* phy, int pin_no, phymod_gpio_mode_t gpio_mode);
/*! 
 * phymod_phy_gpio_config_get
 *
 * @brief Set/Get the configuration of a PHY GPIO pin 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  pin_no          - GPIO pin number [0-7]
 * @param [out]  gpio_mode       - GPIO mode
 */
int phymod_phy_gpio_config_get(const phymod_phy_access_t* phy, int pin_no, phymod_gpio_mode_t* gpio_mode);

/*! 
 * phymod_phy_gpio_pin_value_set
 *
 * @brief Set/Get the output/input value of a PHY GPIO pin 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  pin_no          - GPIO Pin number [0-7]
 * @param [in]  value           - GPIO Pin value (0 or 1)
 */
int phymod_phy_gpio_pin_value_set(const phymod_phy_access_t* phy, int pin_no, int value);
/*! 
 * phymod_phy_gpio_pin_value_get
 *
 * @brief Set/Get the output/input value of a PHY GPIO pin 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  pin_no          - GPIO Pin number [0-7]
 * @param [out]  value           - GPIO Pin value (0 or 1)
 */
int phymod_phy_gpio_pin_value_get(const phymod_phy_access_t* phy, int pin_no, int* value);


/*!
 * @enum phymod_osr_mode_e
 * @brief over sample modes 
 */ 
typedef enum phymod_osr_mode_e {
    phymodOversampleMode1 = 0,
    phymodOversampleMode2,
    phymodOversampleMode3,
    phymodOversampleMode3P3,
    phymodOversampleMode4,
    phymodOversampleMode5,
    phymodOversampleMode7P5,
    phymodOversampleMode8,
    phymodOversampleMode8P25,
    phymodOversampleMode10,
    phymodOversampleMode16P5,
    phymodOversampleMode20P625,
    phymodOversampleModeCount
} phymod_osr_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_osr_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_osr_mode_t validation */
int phymod_osr_mode_t_validate(phymod_osr_mode_t phymod_osr_mode);
/*! 
 * phymod_osr_mode_to_actual_os
 *
 * @brief Get oversample actual value 
 *
 * @param [in]  osr_mode        - OS mode
 * @param [out]  os_int          - OS integer value
 * @param [out]  os_remainder    - OS reminder
 */
int phymod_osr_mode_to_actual_os(phymod_osr_mode_t osr_mode, uint32_t* os_int, uint32_t* os_remainder);

/*! 
 * @brief Timesync capability flags 
 */ 
#define PHYMOD_TS_CAP_MPLS 0x1
#define PHYMOD_TS_CAP_ENHANCED_TSFIFO 0x2
#define PHYMOD_TS_CAP_INBAND_TS 0x4
#define PHYMOD_TS_CAP_FOLLOW_UP_ASSIST 0x8
#define PHYMOD_TS_CAP_DELAY_RESP_ASSIST 0x10
#define PHYMOD_TS_CAP_CAPTURE_TIMESTAMP_MSG 0x20

#define PHYMOD_TS_CAP_MPLS_SET(ts_cap) (ts_cap |= PHYMOD_TS_CAP_MPLS)
#define PHYMOD_TS_CAP_ENHANCED_TSFIFO_SET(ts_cap) (ts_cap |= PHYMOD_TS_CAP_ENHANCED_TSFIFO)
#define PHYMOD_TS_CAP_INBAND_TS_SET(ts_cap) (ts_cap |= PHYMOD_TS_CAP_INBAND_TS)
#define PHYMOD_TS_CAP_FOLLOW_UP_ASSIST_SET(ts_cap) (ts_cap |= PHYMOD_TS_CAP_FOLLOW_UP_ASSIST)
#define PHYMOD_TS_CAP_DELAY_RESP_ASSIST_SET(ts_cap) (ts_cap |= PHYMOD_TS_CAP_DELAY_RESP_ASSIST)
#define PHYMOD_TS_CAP_CAPTURE_TIMESTAMP_MSG_SET(ts_cap) (ts_cap |= PHYMOD_TS_CAP_CAPTURE_TIMESTAMP_MSG)

#define PHYMOD_TS_CAP_MPLS_CLR(ts_cap) (ts_cap &= ~PHYMOD_TS_CAP_MPLS)
#define PHYMOD_TS_CAP_ENHANCED_TSFIFO_CLR(ts_cap) (ts_cap &= ~PHYMOD_TS_CAP_ENHANCED_TSFIFO)
#define PHYMOD_TS_CAP_INBAND_TS_CLR(ts_cap) (ts_cap &= ~PHYMOD_TS_CAP_INBAND_TS)
#define PHYMOD_TS_CAP_FOLLOW_UP_ASSIST_CLR(ts_cap) (ts_cap &= ~PHYMOD_TS_CAP_FOLLOW_UP_ASSIST)
#define PHYMOD_TS_CAP_DELAY_RESP_ASSIST_CLR(ts_cap) (ts_cap &= ~PHYMOD_TS_CAP_DELAY_RESP_ASSIST)
#define PHYMOD_TS_CAP_CAPTURE_TIMESTAMP_MSG_CLR(ts_cap) (ts_cap &= ~PHYMOD_TS_CAP_CAPTURE_TIMESTAMP_MSG)

#define PHYMOD_TS_CAP_MPLS_GET(ts_cap) (ts_cap & PHYMOD_TS_CAP_MPLS ? 1 : 0)
#define PHYMOD_TS_CAP_ENHANCED_TSFIFO_GET(ts_cap) (ts_cap & PHYMOD_TS_CAP_ENHANCED_TSFIFO ? 1 : 0)
#define PHYMOD_TS_CAP_INBAND_TS_GET(ts_cap) (ts_cap & PHYMOD_TS_CAP_INBAND_TS ? 1 : 0)
#define PHYMOD_TS_CAP_FOLLOW_UP_ASSIST_GET(ts_cap) (ts_cap & PHYMOD_TS_CAP_FOLLOW_UP_ASSIST ? 1 : 0)
#define PHYMOD_TS_CAP_DELAY_RESP_ASSIST_GET(ts_cap) (ts_cap & PHYMOD_TS_CAP_DELAY_RESP_ASSIST ? 1 : 0)
#define PHYMOD_TS_CAP_CAPTURE_TIMESTAMP_MSG_GET(ts_cap) (ts_cap & PHYMOD_TS_CAP_CAPTURE_TIMESTAMP_MSG ? 1 : 0)

/*! 
 * @brief Timesync flags> 
 */ 
#define PHYMOD_TS_F_CAPTURE_TS_ENABLE 0x1
#define PHYMOD_TS_F_HEARTBEAT_TS_ENABLE 0x2
#define PHYMOD_TS_F_RX_CRC_ENABLE 0x4
#define PHYMOD_TS_F_8021AS_ENABLE 0x8
#define PHYMOD_TS_F_L2_ENABLE 0x10
#define PHYMOD_TS_F_IP4_ENABLE 0x20
#define PHYMOD_TS_F_IP6_ENABLE 0x40
#define PHYMOD_TS_F_CLOCK_SRC_EXT 0x80
#define PHYMOD_TS_F_CLOCK_SRC_EXT_MODE 0x100
#define PHYMOD_TS_F_1588_ENCRYPTED_MODE 0x200
#define PHYMOD_TS_F_FOLLOW_UP_ASSIST_ENABLE 0x400
#define PHYMOD_TS_F_DELAY_RESP_ASSIST_ENABLE 0x800
#define PHYMOD_TS_F_64BIT_TIMESTAMP_ENABLE 0x1000
#define PHYMOD_TS_F_1588_OVER_HSR_ENABLE 0x2000
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_SYNC 0x4000
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_DELAY_REQ 0x8000
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_REQ 0x10000
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_RESP 0x20000
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_SYNC 0x40000
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_DELAY_REQ 0x80000
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_REQ 0x100000
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_RESP 0x200000

#define PHYMOD_TS_F_CAPTURE_TS_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CAPTURE_TS_ENABLE)
#define PHYMOD_TS_F_HEARTBEAT_TS_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_HEARTBEAT_TS_ENABLE)
#define PHYMOD_TS_F_RX_CRC_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_RX_CRC_ENABLE)
#define PHYMOD_TS_F_8021AS_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_8021AS_ENABLE)
#define PHYMOD_TS_F_L2_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_L2_ENABLE)
#define PHYMOD_TS_F_IP4_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_IP4_ENABLE)
#define PHYMOD_TS_F_IP6_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_IP6_ENABLE)
#define PHYMOD_TS_F_CLOCK_SRC_EXT_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CLOCK_SRC_EXT)
#define PHYMOD_TS_F_CLOCK_SRC_EXT_MODE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CLOCK_SRC_EXT_MODE)
#define PHYMOD_TS_F_1588_ENCRYPTED_MODE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_1588_ENCRYPTED_MODE)
#define PHYMOD_TS_F_FOLLOW_UP_ASSIST_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_FOLLOW_UP_ASSIST_ENABLE)
#define PHYMOD_TS_F_DELAY_RESP_ASSIST_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_DELAY_RESP_ASSIST_ENABLE)
#define PHYMOD_TS_F_64BIT_TIMESTAMP_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_64BIT_TIMESTAMP_ENABLE)
#define PHYMOD_TS_F_1588_OVER_HSR_ENABLE_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_1588_OVER_HSR_ENABLE)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_SYNC_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_SYNC)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_DELAY_REQ_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_DELAY_REQ)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_REQ_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_REQ)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_RESP_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_RESP)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_SYNC_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_SYNC)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_DELAY_REQ_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_DELAY_REQ)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_REQ_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_REQ)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_RESP_SET(ts_flags) (ts_flags |= PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_RESP)

#define PHYMOD_TS_F_CAPTURE_TS_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CAPTURE_TS_ENABLE)
#define PHYMOD_TS_F_HEARTBEAT_TS_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_HEARTBEAT_TS_ENABLE)
#define PHYMOD_TS_F_RX_CRC_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_RX_CRC_ENABLE)
#define PHYMOD_TS_F_8021AS_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_8021AS_ENABLE)
#define PHYMOD_TS_F_L2_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_L2_ENABLE)
#define PHYMOD_TS_F_IP4_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_IP4_ENABLE)
#define PHYMOD_TS_F_IP6_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_IP6_ENABLE)
#define PHYMOD_TS_F_CLOCK_SRC_EXT_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CLOCK_SRC_EXT)
#define PHYMOD_TS_F_CLOCK_SRC_EXT_MODE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CLOCK_SRC_EXT_MODE)
#define PHYMOD_TS_F_1588_ENCRYPTED_MODE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_1588_ENCRYPTED_MODE)
#define PHYMOD_TS_F_FOLLOW_UP_ASSIST_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_FOLLOW_UP_ASSIST_ENABLE)
#define PHYMOD_TS_F_DELAY_RESP_ASSIST_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_DELAY_RESP_ASSIST_ENABLE)
#define PHYMOD_TS_F_64BIT_TIMESTAMP_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_64BIT_TIMESTAMP_ENABLE)
#define PHYMOD_TS_F_1588_OVER_HSR_ENABLE_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_1588_OVER_HSR_ENABLE)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_SYNC_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_SYNC)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_DELAY_REQ_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_DELAY_REQ)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_REQ_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_REQ)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_RESP_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_RESP)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_SYNC_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_SYNC)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_DELAY_REQ_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_DELAY_REQ)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_REQ_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_REQ)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_RESP_CLR(ts_flags) (ts_flags &= ~PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_RESP)

#define PHYMOD_TS_F_CAPTURE_TS_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CAPTURE_TS_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_HEARTBEAT_TS_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_HEARTBEAT_TS_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_RX_CRC_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_RX_CRC_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_8021AS_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_8021AS_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_L2_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_L2_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_IP4_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_IP4_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_IP6_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_IP6_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_CLOCK_SRC_EXT_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CLOCK_SRC_EXT ? 1 : 0)
#define PHYMOD_TS_F_CLOCK_SRC_EXT_MODE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CLOCK_SRC_EXT_MODE ? 1 : 0)
#define PHYMOD_TS_F_1588_ENCRYPTED_MODE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_1588_ENCRYPTED_MODE ? 1 : 0)
#define PHYMOD_TS_F_FOLLOW_UP_ASSIST_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_FOLLOW_UP_ASSIST_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_DELAY_RESP_ASSIST_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_DELAY_RESP_ASSIST_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_64BIT_TIMESTAMP_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_64BIT_TIMESTAMP_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_1588_OVER_HSR_ENABLE_GET(ts_flags) (ts_flags & PHYMOD_TS_F_1588_OVER_HSR_ENABLE ? 1 : 0)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_SYNC_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_SYNC ? 1 : 0)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_DELAY_REQ_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_DELAY_REQ ? 1 : 0)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_REQ_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_REQ ? 1 : 0)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_RESP_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CAPTURE_TIMESTAMP_TX_PDELAY_RESP ? 1 : 0)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_SYNC_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_SYNC ? 1 : 0)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_DELAY_REQ_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_DELAY_REQ ? 1 : 0)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_REQ_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_REQ ? 1 : 0)
#define PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_RESP_GET(ts_flags) (ts_flags & PHYMOD_TS_F_CAPTURE_TIMESTAMP_RX_PDELAY_RESP ? 1 : 0)

/*! 
 * @brief Load control flags> 
 */ 
#define PHYMOD_TS_LDCTL_TN_LOAD 0x1
#define PHYMOD_TS_LDCTL_TIMECODE_LOAD 0x2
#define PHYMOD_TS_LDCTL_SYNCOUT_LOAD 0x4
#define PHYMOD_TS_LDCTL_NCO_DIVIDER_LOAD 0x8
#define PHYMOD_TS_LDCTL_LOCAL_TIME_LOAD 0x10
#define PHYMOD_TS_LDCTL_NCO_ADDEND_LOAD 0x20
#define PHYMOD_TS_LDCTL_DPLL_LOOP_FILTER_LOAD 0x40
#define PHYMOD_TS_LDCTL_DPLL_REF_PHASE_LOAD 0x80
#define PHYMOD_TS_LDCTL_DPLL_REF_PHASE_DELTA_LOAD 0x100
#define PHYMOD_TS_LDCTL_DPLL_K3_LOAD 0x200
#define PHYMOD_TS_LDCTL_DPLL_K2_LOAD 0x400
#define PHYMOD_TS_LDCTL_DPLL_K1_LOAD 0x800

#define PHYMOD_TS_LDCTL_TN_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_TN_LOAD)
#define PHYMOD_TS_LDCTL_TIMECODE_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_TIMECODE_LOAD)
#define PHYMOD_TS_LDCTL_SYNCOUT_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_SYNCOUT_LOAD)
#define PHYMOD_TS_LDCTL_NCO_DIVIDER_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_NCO_DIVIDER_LOAD)
#define PHYMOD_TS_LDCTL_LOCAL_TIME_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_LOCAL_TIME_LOAD)
#define PHYMOD_TS_LDCTL_NCO_ADDEND_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_NCO_ADDEND_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_LOOP_FILTER_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_DPLL_LOOP_FILTER_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_REF_PHASE_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_DPLL_REF_PHASE_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_REF_PHASE_DELTA_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_DPLL_REF_PHASE_DELTA_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_K3_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_DPLL_K3_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_K2_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_DPLL_K2_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_K1_LOAD_SET(ts_ldctl) (ts_ldctl |= PHYMOD_TS_LDCTL_DPLL_K1_LOAD)

#define PHYMOD_TS_LDCTL_TN_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_TN_LOAD)
#define PHYMOD_TS_LDCTL_TIMECODE_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_TIMECODE_LOAD)
#define PHYMOD_TS_LDCTL_SYNCOUT_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_SYNCOUT_LOAD)
#define PHYMOD_TS_LDCTL_NCO_DIVIDER_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_NCO_DIVIDER_LOAD)
#define PHYMOD_TS_LDCTL_LOCAL_TIME_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_LOCAL_TIME_LOAD)
#define PHYMOD_TS_LDCTL_NCO_ADDEND_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_NCO_ADDEND_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_LOOP_FILTER_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_DPLL_LOOP_FILTER_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_REF_PHASE_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_DPLL_REF_PHASE_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_REF_PHASE_DELTA_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_DPLL_REF_PHASE_DELTA_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_K3_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_DPLL_K3_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_K2_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_DPLL_K2_LOAD)
#define PHYMOD_TS_LDCTL_DPLL_K1_LOAD_CLR(ts_ldctl) (ts_ldctl &= ~PHYMOD_TS_LDCTL_DPLL_K1_LOAD)

#define PHYMOD_TS_LDCTL_TN_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_TN_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_TIMECODE_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_TIMECODE_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_SYNCOUT_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_SYNCOUT_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_NCO_DIVIDER_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_NCO_DIVIDER_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_LOCAL_TIME_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_LOCAL_TIME_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_NCO_ADDEND_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_NCO_ADDEND_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_DPLL_LOOP_FILTER_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_DPLL_LOOP_FILTER_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_DPLL_REF_PHASE_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_DPLL_REF_PHASE_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_DPLL_REF_PHASE_DELTA_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_DPLL_REF_PHASE_DELTA_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_DPLL_K3_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_DPLL_K3_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_DPLL_K2_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_DPLL_K2_LOAD ? 1 : 0)
#define PHYMOD_TS_LDCTL_DPLL_K1_LOAD_GET(ts_ldctl) (ts_ldctl & PHYMOD_TS_LDCTL_DPLL_K1_LOAD ? 1 : 0)


/*!
 * @enum phymod_timesync_timer_mode_e
 * @brief Timesync timer mode 
 */ 
typedef enum phymod_timesync_timer_mode_e {
    phymodTimesyncTimerModeNone = 0x0,
    phymodTimesyncTimerModeDefault = 0x1,
    phymodTimesyncTimerMode32Bit = 0x2,
    phymodTimesyncTimerMode48Bit = 0x4,
    phymodTimesyncTimerMode64Bit = 0x8,
    phymodTimesyncTimerMode80Bit = 0x10,
    phymodTimesyncTimerModeCount
} phymod_timesync_timer_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_timesync_timer_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_timesync_timer_mode_t validation */
int phymod_timesync_timer_mode_t_validate(phymod_timesync_timer_mode_t phymod_timesync_timer_mode);

/*!
 * @enum phymod_timesync_global_mode_e
 * @brief Timesync timer mode 
 */ 
typedef enum phymod_timesync_global_mode_e {
    phymodTimesyncGLobalModeFree = 0x0,
    phymodTimesyncGLobalModeSyncIn = 0x1,
    phymodTimesyncGLobalModeCpu = 0x2,
    phymodTimesyncGLobalModeCount
} phymod_timesync_global_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_timesync_global_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_timesync_global_mode_t validation */
int phymod_timesync_global_mode_t_validate(phymod_timesync_global_mode_t phymod_timesync_global_mode);

/*!
 * @enum phymod_timesync_framesync_mode_e
 * @brief Timesync framsync mode 
 */ 
typedef enum phymod_timesync_framesync_mode_e {
    phymodTimesyncFramsyncModeNone = 0x0,
    phymodTimesyncFramsyncModeSyncIn0 = 0x1,
    phymodTimesyncFramsyncModeSyncIn1 = 0x2,
    phymodTimesyncFramsyncModeSyncOut = 0x3,
    phymodTimesyncFramsyncModeCpu = 0x4,
    phymodTimesyncFramsyncModeCount
} phymod_timesync_framesync_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_timesync_framesync_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_timesync_framesync_mode_t validation */
int phymod_timesync_framesync_mode_t_validate(phymod_timesync_framesync_mode_t phymod_timesync_framesync_mode);

/*!
 * @enum phymod_timesync_syncout_mode_e
 * @brief Timesync syncout mode 
 */ 
typedef enum phymod_timesync_syncout_mode_e {
    phymodTimesyncSyncoutModeDisable,
    phymodTimesyncSyncoutModeOneTime,
    phymodTimesyncSyncoutModePulseTrain,
    phymodTimesyncSyncoutModePulseTrainWithSync,
    phymodTimesyncSyncoutModeCount
} phymod_timesync_syncout_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_timesync_syncout_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_timesync_syncout_mode_t validation */
int phymod_timesync_syncout_mode_t_validate(phymod_timesync_syncout_mode_t phymod_timesync_syncout_mode);

/*!
 * @enum phymod_timesync_event_msg_action_e
 * @brief Actions on timesync event messages 
 */ 
typedef enum phymod_timesync_event_msg_action_e {
    phymodTimesyncEventMsgActionNone,
    phymodTimesyncEventMsgActionEgrModeUpdateCorrectionField,
    phymodTimesyncEventMsgActionEgrModeReplaceCorrectionFieldOrigin,
    phymodTimesyncEventMsgActionEgrModeCaptureTimestamp,
    phymodTimesyncEventMsgActionIngModeUpdateCorrectionField,
    phymodTimesyncEventMsgActionIngModeInsertTimestamp,
    phymodTimesyncEventMsgActionIngModeInsertDelaytime,
    phymodTimesyncEventMsgActionCount
} phymod_timesync_event_msg_action_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_timesync_event_msg_action_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_timesync_event_msg_action_t validation */
int phymod_timesync_event_msg_action_t_validate(phymod_timesync_event_msg_action_t phymod_timesync_event_msg_action);
typedef struct phymod_timesync_timer_adjust_s {
    phymod_timesync_timer_mode_t mode; /**< Timer mode */
    int delta; /**< Delta */
} phymod_timesync_timer_adjust_t;

/* phymod_timesync_timer_adjust_t initialization and validation */
int phymod_timesync_timer_adjust_t_validate(const phymod_timesync_timer_adjust_t* phymod_timesync_timer_adjust);
int phymod_timesync_timer_adjust_t_init(phymod_timesync_timer_adjust_t* phymod_timesync_timer_adjust);

typedef struct phymod_timesync_inband_ctrl_s {
    uint32_t flags; /**< Flags */
    int resv0_id; /**< Reserved ID */
    phymod_timesync_timer_mode_t timer_mode; /**< Timer mode */
} phymod_timesync_inband_ctrl_t;

/* phymod_timesync_inband_ctrl_t initialization and validation */
int phymod_timesync_inband_ctrl_t_validate(const phymod_timesync_inband_ctrl_t* phymod_timesync_inband_ctrl);
int phymod_timesync_inband_ctrl_t_init(phymod_timesync_inband_ctrl_t* phymod_timesync_inband_ctrl);

typedef struct phymod_timesync_framesync_s {
    phymod_timesync_framesync_mode_t mode; /**< Mode */
    uint32_t length_threshold; /**< Threshold */
    uint32_t event_offset; /**< Even off set */
} phymod_timesync_framesync_t;

/* phymod_timesync_framesync_t initialization and validation */
int phymod_timesync_framesync_t_validate(const phymod_timesync_framesync_t* phymod_timesync_framesync);
int phymod_timesync_framesync_t_init(phymod_timesync_framesync_t* phymod_timesync_framesync);

typedef struct phymod_timesync_syncout_s {
    uint16_t pulse_1_length; /**< Pulse 1 length in nanoseconds */
    uint16_t pulse_2_length; /**< Pulse 2 length in nanoseconds */
    uint32_t interval; /**< Interval in nanoseconds */
    uint64_t timestamp; /**< Syncout timestamp */
} phymod_timesync_syncout_t;

/* phymod_timesync_syncout_t initialization and validation */
int phymod_timesync_syncout_t_validate(const phymod_timesync_syncout_t* phymod_timesync_syncout);
int phymod_timesync_syncout_t_init(phymod_timesync_syncout_t* phymod_timesync_syncout);

typedef struct phymod_timesync_timespec_s {
    uint8_t isnegative; /**< Sign identifier. */
    uint64_t seconds; /**< Seconds absolute value. */
    uint32_t nanoseconds; /**< Nanoseconds absolute value. */
} phymod_timesync_timespec_t;

/* phymod_timesync_timespec_t initialization and validation */
int phymod_timesync_timespec_t_validate(const phymod_timesync_timespec_t* phymod_timesync_timespec);
int phymod_timesync_timespec_t_init(phymod_timesync_timespec_t* phymod_timesync_timespec);

/*! 
 * @brief Timesync MPLS label flags 
 */ 
#define PHYMOD_TS_MPLS_LABEL_F_IN 0x1
#define PHYMOD_TS_MPLS_LABEL_F_OUT 0x2

#define PHYMOD_TS_MPLS_LABEL_F_IN_SET(ts_mpls_label_f) (ts_mpls_label_f |= PHYMOD_TS_MPLS_LABEL_F_IN)
#define PHYMOD_TS_MPLS_LABEL_F_OUT_SET(ts_mpls_label_f) (ts_mpls_label_f |= PHYMOD_TS_MPLS_LABEL_F_OUT)

#define PHYMOD_TS_MPLS_LABEL_F_IN_CLR(ts_mpls_label_f) (ts_mpls_label_f &= ~PHYMOD_TS_MPLS_LABEL_F_IN)
#define PHYMOD_TS_MPLS_LABEL_F_OUT_CLR(ts_mpls_label_f) (ts_mpls_label_f &= ~PHYMOD_TS_MPLS_LABEL_F_OUT)

#define PHYMOD_TS_MPLS_LABEL_F_IN_GET(ts_mpls_label_f) (ts_mpls_label_f & PHYMOD_TS_MPLS_LABEL_F_IN ? 1 : 0)
#define PHYMOD_TS_MPLS_LABEL_F_OUT_GET(ts_mpls_label_f) (ts_mpls_label_f & PHYMOD_TS_MPLS_LABEL_F_OUT ? 1 : 0)

typedef struct phymod_timesync_mpls_label_s {
    uint32_t value; /**< MPLS label bits [19:0] */
    uint32_t mask; /**< MPLS label mask bits [19:0] */
    uint32_t flags; /**< MPLS label flags */
} phymod_timesync_mpls_label_t;

/* phymod_timesync_mpls_label_t initialization and validation */
int phymod_timesync_mpls_label_t_validate(const phymod_timesync_mpls_label_t* phymod_timesync_mpls_label);
int phymod_timesync_mpls_label_t_init(phymod_timesync_mpls_label_t* phymod_timesync_mpls_label);

/*! 
 * @brief Timesync MPLS control flags 
 */ 
#define PHYMOD_TS_MPLS_F_ENABLE 0x1
#define PHYMOD_TS_MPLS_F_ENTROPY_ENABLE 0x2
#define PHYMOD_TS_MPLS_F_SPECIAL_LABEL_ENABLE 0x4
#define PHYMOD_TS_MPLS_F_CONTROL_WORD_ENABLE 0x8

#define PHYMOD_TS_MPLS_F_ENABLE_SET(ts_mpls_f) (ts_mpls_f |= PHYMOD_TS_MPLS_F_ENABLE)
#define PHYMOD_TS_MPLS_F_ENTROPY_ENABLE_SET(ts_mpls_f) (ts_mpls_f |= PHYMOD_TS_MPLS_F_ENTROPY_ENABLE)
#define PHYMOD_TS_MPLS_F_SPECIAL_LABEL_ENABLE_SET(ts_mpls_f) (ts_mpls_f |= PHYMOD_TS_MPLS_F_SPECIAL_LABEL_ENABLE)
#define PHYMOD_TS_MPLS_F_CONTROL_WORD_ENABLE_SET(ts_mpls_f) (ts_mpls_f |= PHYMOD_TS_MPLS_F_CONTROL_WORD_ENABLE)

#define PHYMOD_TS_MPLS_F_ENABLE_CLR(ts_mpls_f) (ts_mpls_f &= ~PHYMOD_TS_MPLS_F_ENABLE)
#define PHYMOD_TS_MPLS_F_ENTROPY_ENABLE_CLR(ts_mpls_f) (ts_mpls_f &= ~PHYMOD_TS_MPLS_F_ENTROPY_ENABLE)
#define PHYMOD_TS_MPLS_F_SPECIAL_LABEL_ENABLE_CLR(ts_mpls_f) (ts_mpls_f &= ~PHYMOD_TS_MPLS_F_SPECIAL_LABEL_ENABLE)
#define PHYMOD_TS_MPLS_F_CONTROL_WORD_ENABLE_CLR(ts_mpls_f) (ts_mpls_f &= ~PHYMOD_TS_MPLS_F_CONTROL_WORD_ENABLE)

#define PHYMOD_TS_MPLS_F_ENABLE_GET(ts_mpls_f) (ts_mpls_f & PHYMOD_TS_MPLS_F_ENABLE ? 1 : 0)
#define PHYMOD_TS_MPLS_F_ENTROPY_ENABLE_GET(ts_mpls_f) (ts_mpls_f & PHYMOD_TS_MPLS_F_ENTROPY_ENABLE ? 1 : 0)
#define PHYMOD_TS_MPLS_F_SPECIAL_LABEL_ENABLE_GET(ts_mpls_f) (ts_mpls_f & PHYMOD_TS_MPLS_F_SPECIAL_LABEL_ENABLE ? 1 : 0)
#define PHYMOD_TS_MPLS_F_CONTROL_WORD_ENABLE_GET(ts_mpls_f) (ts_mpls_f & PHYMOD_TS_MPLS_F_CONTROL_WORD_ENABLE ? 1 : 0)

typedef struct phymod_timesync_mpls_ctrl_s {
    uint32_t flags; /**< Flags */
    uint32_t special_label; /**< bits [19:0] */
    phymod_timesync_mpls_label_t labels[10]; /**< Timesync MPLS labels */
    int size; /**< Number of elements in label array */
} phymod_timesync_mpls_ctrl_t;

/* phymod_timesync_mpls_ctrl_t initialization and validation */
int phymod_timesync_mpls_ctrl_t_validate(const phymod_timesync_mpls_ctrl_t* phymod_timesync_mpls_ctrl);
int phymod_timesync_mpls_ctrl_t_init(phymod_timesync_mpls_ctrl_t* phymod_timesync_mpls_ctrl);

typedef struct phymod_timesync_config_s {
    uint32_t capabilities; /**< Flags PHYMOD_TS_CAP_* */
    uint32_t flags; /**< Flags PHYMOD_TS_F_* */
    uint16_t itpid; /**< 1588 inner tag */
    uint16_t otpid; /**< 1588 outer tag */
    uint16_t otpid2; /**< 1588 outer tag2 */
    phymod_timesync_timer_adjust_t timer_adjust; /**< Inband TS control */
    phymod_timesync_inband_ctrl_t inband_ctrl; /**< Inband TS control */
    phymod_timesync_global_mode_t gmode; /**< Global mode */
    phymod_timesync_syncout_t syncout; /**< Syncout */
    uint16_t ts_divider; /**< TS divider */
    phymod_timesync_timespec_t original_timecode; /**< Original timecode to be inserted */
    uint32_t rx_link_delay; /**< RX link delay */
    phymod_timesync_event_msg_action_t tx_sync_mode; /**< sync */
    phymod_timesync_event_msg_action_t tx_delay_req_mode; /**< delay request */
    phymod_timesync_event_msg_action_t tx_pdelay_req_mode; /**< pdelay request */
    phymod_timesync_event_msg_action_t tx_pdelay_resp_mode; /**< pdelay response */
    phymod_timesync_event_msg_action_t rx_sync_mode; /**< sync */
    phymod_timesync_event_msg_action_t rx_delay_req_mode; /**< delay request */
    phymod_timesync_event_msg_action_t rx_pdelay_req_mode; /**< pdelay request */
    phymod_timesync_event_msg_action_t rx_pdelay_resp_mode; /**< pdelay response */
    phymod_timesync_mpls_ctrl_t mpls_ctrl; /**< MPLS control */
    uint32_t sync_freq; /**< sync frequency */
    uint16_t phy_1588_dpll_k1; /**< DPLL K1 */
    uint16_t phy_1588_dpll_k2; /**< DPLL K2 */
    uint16_t phy_1588_dpll_k3; /**< DPLL K3 */
    uint64_t phy_1588_dpll_loop_filter; /**< DPLL loop filter */
    uint64_t phy_1588_dpll_ref_phase; /**< DPLL ref phase */
    uint32_t phy_1588_dpll_ref_phase_delta; /**< DPLL ref phase delta */
} phymod_timesync_config_t;

/* phymod_timesync_config_t initialization and validation */
int phymod_timesync_config_t_validate(const phymod_timesync_config_t* phymod_timesync_config);
int phymod_timesync_config_t_init(phymod_timesync_config_t* phymod_timesync_config);

/*! 
 * phymod_timesync_config_set
 *
 * @brief Set/Get timesync configuration 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  config          - Timesync configuration
 */
int phymod_timesync_config_set(const phymod_phy_access_t* phy, const phymod_timesync_config_t* config);
/*! 
 * phymod_timesync_config_get
 *
 * @brief Set/Get timesync configuration 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  config          - Timesync configuration
 */
int phymod_timesync_config_get(const phymod_phy_access_t* phy, phymod_timesync_config_t* config);

/*! 
 * phymod_timesync_enable_set
 *
 * @brief Set/Get timesync enable 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  enable          - Timesync enable
 */
int phymod_timesync_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
/*! 
 * phymod_timesync_enable_get
 *
 * @brief Set/Get timesync enable 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  enable          - Timesync enable
 */
int phymod_timesync_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*! 
 * phymod_timesync_nco_addend_set
 *
 * @brief Set/Get timesync enable 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  freq_step       - NCO frequency step (device-specific)
 */
int phymod_timesync_nco_addend_set(const phymod_phy_access_t* phy, uint32_t freq_step);
/*! 
 * phymod_timesync_nco_addend_get
 *
 * @brief Set/Get timesync enable 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  freq_step       - NCO frequency step (device-specific)
 */
int phymod_timesync_nco_addend_get(const phymod_phy_access_t* phy, uint32_t* freq_step);

/*! 
 * phymod_timesync_framesync_mode_set
 *
 * @brief Set/Get framesync mode 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  framesync       - Framesync mode
 */
int phymod_timesync_framesync_mode_set(const phymod_phy_access_t* phy, const phymod_timesync_framesync_t* framesync);
/*! 
 * phymod_timesync_framesync_mode_get
 *
 * @brief Set/Get framesync mode 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  framesync       - Framesync mode
 */
int phymod_timesync_framesync_mode_get(const phymod_phy_access_t* phy, phymod_timesync_framesync_t* framesync);

/*! 
 * phymod_timesync_local_time_set
 *
 * @brief Set/Get local time 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  local_time      - Local time (device-specific)
 */
int phymod_timesync_local_time_set(const phymod_phy_access_t* phy, uint64_t local_time);
/*! 
 * phymod_timesync_local_time_get
 *
 * @brief Set/Get local time 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  local_time      - Local time (device-specific)
 */
int phymod_timesync_local_time_get(const phymod_phy_access_t* phy, uint64_t* local_time);

/*! 
 * phymod_timesync_load_ctrl_set
 *
 * @brief Set/Get load mode mode 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  load_once       - Load control mask PHYMOD_TS_LDCTL_xxx
 * @param [in]  load_always     - Load control mask PHYMOD_TS_LDCTL_xxx (persistent across FrameSync)
 */
int phymod_timesync_load_ctrl_set(const phymod_phy_access_t* phy, uint32_t load_once, uint32_t load_always);
/*! 
 * phymod_timesync_load_ctrl_get
 *
 * @brief Set/Get load mode mode 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  load_once       - Load control mask PHYMOD_TS_LDCTL_xxx
 * @param [out]  load_always     - Load control mask PHYMOD_TS_LDCTL_xxx (persistent across FrameSync)
 */
int phymod_timesync_load_ctrl_get(const phymod_phy_access_t* phy, uint32_t* load_once, uint32_t* load_always);

/*! 
 * phymod_timesync_tx_timestamp_offset_set
 *
 * @brief Set/Get timesync Tx AFE delay 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  ts_offset       - Tx AFE delay in ns
 */
int phymod_timesync_tx_timestamp_offset_set(const phymod_phy_access_t* phy, uint32_t ts_offset);
/*! 
 * phymod_timesync_tx_timestamp_offset_get
 *
 * @brief Set/Get timesync Tx AFE delay 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  ts_offset       - Tx AFE delay in ns
 */
int phymod_timesync_tx_timestamp_offset_get(const phymod_phy_access_t* phy, uint32_t* ts_offset);

/*! 
 * phymod_timesync_rx_timestamp_offset_set
 *
 * @brief Set/Get timesync Rx AFE delay 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  ts_offset       - Rx AFE delay in ns
 */
int phymod_timesync_rx_timestamp_offset_set(const phymod_phy_access_t* phy, uint32_t ts_offset);
/*! 
 * phymod_timesync_rx_timestamp_offset_get
 *
 * @brief Set/Get timesync Rx AFE delay 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  ts_offset       - Rx AFE delay in ns
 */
int phymod_timesync_rx_timestamp_offset_get(const phymod_phy_access_t* phy, uint32_t* ts_offset);

/*! 
 * phymod_timesync_capture_timestamp_get
 *
 * @brief Get timesync capture timestamp 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  cap_ts          - Capture timestamp
 */
int phymod_timesync_capture_timestamp_get(const phymod_phy_access_t* phy, uint64_t* cap_ts);

/*! 
 * phymod_timesync_heartbeat_timestamp_get
 *
 * @brief Get timesync heartbeat timestamp 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  hb_ts           - Heartbeat timestamp
 */
int phymod_timesync_heartbeat_timestamp_get(const phymod_phy_access_t* phy, uint64_t* hb_ts);

/*! 
 * phymod_timesync_do_sync
 *
 * @brief Force a framesync event 
 *
 * @param [in]  phy             - phy access information
 */
int phymod_timesync_do_sync(const phymod_phy_access_t* phy);


/*!
 * @enum phymod_edc_config_method_e
 * @brief Configuration method for Electronic Dispersion Compensation (EDC) 
 */ 
typedef enum phymod_edc_config_method_e {
    phymodEdcConfigMethodNone,
    phymodEdcConfigMethodHardware, /**< EDC mode is set automatically by hardware */
    phymodEdcConfigMethodSoftware, /**< EDC mode is selected by driver software */
    phymodEdcConfigMethodCount
} phymod_edc_config_method_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_edc_config_method_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_edc_config_method_t validation */
int phymod_edc_config_method_t_validate(phymod_edc_config_method_t phymod_edc_config_method);
typedef struct phymod_edc_config_s {
    phymod_edc_config_method_t method; /**< EDC configuration method */
    uint32_t mode_val; /**< Device-specific EDC mode value (valid only when software configuration method is used) */
} phymod_edc_config_t;

/* phymod_edc_config_t initialization and validation */
int phymod_edc_config_t_validate(const phymod_edc_config_t* phymod_edc_config);
int phymod_edc_config_t_init(phymod_edc_config_t* phymod_edc_config);

/*! 
 * phymod_edc_config_set
 *
 * @brief Set/get EDC mode 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  edc_config      - Electronic Dispersion Compensation (EDC) configuration
 */
int phymod_edc_config_set(const phymod_phy_access_t* phy, const phymod_edc_config_t* edc_config);
/*! 
 * phymod_edc_config_get
 *
 * @brief Set/get EDC mode 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  edc_config      - Electronic Dispersion Compensation (EDC) configuration
 */
int phymod_edc_config_get(const phymod_phy_access_t* phy, phymod_edc_config_t* edc_config);

typedef enum phymod_core_mode_e {
    phymodCoreModeDefault = 0, /**< default core mode */
    phymodCoreModeSingle, /**< Core serves a single 4-lane */
    phymodCoreModeDual, /**< Core spits to 2-lane  */
    phymodCoreModeIndepLane, /**< Each lane is independent logic port */
    phymodCoreModeSplit012, /**< Two 1-lane and one 2-lane logic ports */
    phymodCoreModeSplit023, /**< One 2-lane and two 1-lane logic ports */
    phymodCoreModeTriple244, /**< triple-core 244 mode */
    phymodCoreModeTriple343, /**< triple-core 343 mode */
    phymodCoreModeTriple442, /**< triple-core 442 mode */
    phymodCoreModeCount
} phymod_core_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_core_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_core_mode_t validation */
int phymod_core_mode_t_validate(phymod_core_mode_t phymod_core_mode);
/*! 
 * phymod_phy_core_mode_set
 *
 * @brief Set/Get phy core mode 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  core_mode       - 
 */
int phymod_phy_core_mode_set(const phymod_phy_access_t* phy, phymod_core_mode_t core_mode);
/*! 
 * phymod_phy_core_mode_get
 *
 * @brief Set/Get phy core mode 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  core_mode       - 
 */
int phymod_phy_core_mode_get(const phymod_phy_access_t* phy, phymod_core_mode_t* core_mode);


/*!
 * @enum phymod_failover_mode_e
 * @brief Failover configuration 
 */ 
typedef enum phymod_failover_mode_e {
    phymodFailovermodeNone,
    phymodFailovermodeEnable, /**< enable Failover mode */
    phymodFailovermodeCount
} phymod_failover_mode_t;

#ifdef PHYMOD_DIAG
extern enum_mapping_t phymod_failover_mode_t_mapping[];
#endif /*PHYMOD_DIAG*/

/* phymod_failover_mode_t validation */
int phymod_failover_mode_t_validate(phymod_failover_mode_t phymod_failover_mode);
/*! 
 * phymod_failover_mode_set
 *
 * @brief Set/get failover mode  
 *
 * @param [in]  phy             - phy access information
 * @param [in]  failover_mode   - Failover(FOV) configuration
 */
int phymod_failover_mode_set(const phymod_phy_access_t* phy, phymod_failover_mode_t failover_mode);
/*! 
 * phymod_failover_mode_get
 *
 * @brief Set/get failover mode  
 *
 * @param [in]  phy             - phy access information
 * @param [out]  failover_mode   - Failover(FOV) configuration
 */
int phymod_failover_mode_get(const phymod_phy_access_t* phy, phymod_failover_mode_t* failover_mode);

/*! 
 * phymod_port_init
 *
 * @brief Port config init 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  port_config     - port config
 */
int phymod_port_init(const phymod_phy_access_t* phy, const phymod_port_config_t* port_config);

typedef struct phymod_an_try_enable_control_s {
    phymod_an_mode_type_t an_mode;
    uint32_t num_lane_adv; /**< The number of lanes the autoneg advert */
    uint32_t flags; /**< see AN_F */
    uint32_t enable;
    uint32_t speed;
} phymod_an_try_enable_control_t;

/* phymod_an_try_enable_control_t initialization and validation */
int phymod_an_try_enable_control_t_validate(const phymod_an_try_enable_control_t* phymod_an_try_enable_control);
int phymod_an_try_enable_control_t_init(phymod_an_try_enable_control_t* phymod_an_try_enable_control);

typedef struct phymod_phy_an_status_s {
    uint32_t hcd_speed; /**< negotiated speed */
    uint8_t result; /**< Autoneg result. */
} phymod_phy_an_status_t;

/* phymod_phy_an_status_t initialization and validation */
int phymod_phy_an_status_t_validate(const phymod_phy_an_status_t* phymod_phy_an_status);
int phymod_phy_an_status_t_init(phymod_phy_an_status_t* phymod_phy_an_status);

/*! 
 * phymod_phy_autoneg_try_enable
 *
 * @brief Autoneg try enable 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  an              - 
 * @param [out]  an_status       - 
 */
int phymod_phy_autoneg_try_enable(const phymod_phy_access_t* phy, const phymod_an_try_enable_control_t* an, phymod_phy_an_status_t* an_status);

/*! 
 * phymod_phy_short_chn_mode_enable_set
 *
 * @brief Short channel mode 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  enable          - 
 */
int phymod_phy_short_chn_mode_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
/*! 
 * phymod_phy_short_chn_mode_enable_get
 *
 * @brief Short channel mode 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  enable          - 
 */
int phymod_phy_short_chn_mode_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*! 
 * phymod_port_enable_set
 *
 * @brief port enable set 
 *
 * @param [in]  phy             - phy access information
 * @param [in]  enable          - 
 */
int phymod_port_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
/*! 
 * phymod_port_enable_get
 *
 * @brief port enable set 
 *
 * @param [in]  phy             - phy access information
 * @param [out]  enable          - 
 */
int phymod_port_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

#endif /*_PHYMOD_H_*/
