/*
 *         
 * $Id: phymod.xml,v 1.1.2.5 2013/09/12 10:43:06 nirf Exp $
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


#ifndef _QSGMIIE_H__H_
#define _QSGMIIE_H__H_

#include <phymod/phymod_definitions.h>
/*Initialize phymod module*/
int qsgmiie_core_identify(const phymod_core_access_t* core, uint32_t core_id, uint32_t* is_identified);

/*Retrive core information*/
int qsgmiie_core_info_get(const phymod_core_access_t* core, phymod_core_info_t* info);

/*Set\get lane mapping*/
int qsgmiie_core_lane_map_set(const phymod_core_access_t* core, const phymod_lane_map_t* lane_map);
int qsgmiie_core_lane_map_get(const phymod_core_access_t* core, phymod_lane_map_t* lane_map);

/*Reset Core*/
int qsgmiie_core_reset_set(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t direction);
int qsgmiie_core_reset_get(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t* direction);

/*Retrive firmware information*/
int qsgmiie_core_firmware_info_get(const phymod_core_access_t* core, phymod_core_firmware_info_t* fw_info);

/*Set\get firmware operation mode*/
int qsgmiie_phy_firmware_core_config_set(const phymod_phy_access_t* phy, phymod_firmware_core_config_t fw_core_config);
int qsgmiie_phy_firmware_core_config_get(const phymod_phy_access_t* phy, phymod_firmware_core_config_t* fw_core_config);

/*Set\get firmware operation mode*/
int qsgmiie_phy_firmware_lane_config_set(const phymod_phy_access_t* phy, phymod_firmware_lane_config_t fw_lane_config);
int qsgmiie_phy_firmware_lane_config_get(const phymod_phy_access_t* phy, phymod_firmware_lane_config_t* fw_lane_config);

/*Start\Stop the sequencer*/
int qsgmiie_core_pll_sequencer_restart(const phymod_core_access_t* core, uint32_t flags, phymod_sequencer_operation_t operation);

/*Wait for core event*/
int qsgmiie_core_wait_event(const phymod_core_access_t* core, phymod_core_event_t event, uint32_t timeout);

/* re-tune rx path*/
int qsgmiie_phy_rx_restart(const phymod_phy_access_t* phy);

/*Set phy polarity*/
int qsgmiie_phy_polarity_set(const phymod_phy_access_t* phy, const phymod_polarity_t* polarity);
int qsgmiie_phy_polarity_get(const phymod_phy_access_t* phy, phymod_polarity_t* polarity);

/*Set\Get TX Parameters*/
int qsgmiie_phy_tx_set(const phymod_phy_access_t* phy, const phymod_tx_t* tx);
int qsgmiie_phy_tx_get(const phymod_phy_access_t* phy, phymod_tx_t* tx);

/*Set\Get TX override Parameters*/
int qsgmiie_phy_tx_override_set(const phymod_phy_access_t* phy, const phymod_tx_override_t* tx_override);
int qsgmiie_phy_tx_override_get(const phymod_phy_access_t* phy, phymod_tx_override_t* tx_override);

/*Set\Get RX Parameters*/
int qsgmiie_phy_rx_set(const phymod_phy_access_t* phy, const phymod_rx_t* rx);
int qsgmiie_phy_rx_get(const phymod_phy_access_t* phy, phymod_rx_t* rx);

/*Reset phy*/
int qsgmiie_phy_reset_set(const phymod_phy_access_t* phy, const phymod_phy_reset_t* reset);
int qsgmiie_phy_reset_get(const phymod_phy_access_t* phy, phymod_phy_reset_t* reset);

/*Control phy power*/
int qsgmiie_phy_power_set(const phymod_phy_access_t* phy, const phymod_phy_power_t* power);
int qsgmiie_phy_power_get(const phymod_phy_access_t* phy, phymod_phy_power_t* power);

/*TX transmission control*/
int qsgmiie_phy_tx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t tx_control);
int qsgmiie_phy_tx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t* tx_control);

/*Rx control*/
int qsgmiie_phy_rx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t rx_control);
int qsgmiie_phy_rx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t* rx_control);

/*TX transmission disable*/
int qsgmiie_phy_interface_config_set(const phymod_phy_access_t* phy, uint32_t flags, const phymod_phy_inf_config_t* config);
int qsgmiie_phy_interface_config_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_ref_clk_t ref_clock, phymod_phy_inf_config_t* config);

/*Set\Get autoneg*/
int qsgmiie_phy_autoneg_ability_set(const phymod_phy_access_t* phy, const phymod_autoneg_ability_t* an_ability_set_type);
int qsgmiie_phy_autoneg_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

/*Set\Get autoneg*/
int qsgmiie_phy_autoneg_set(const phymod_phy_access_t* phy, const phymod_autoneg_control_t* an);
int qsgmiie_phy_autoneg_get(const phymod_phy_access_t* phy, phymod_autoneg_control_t* an, uint32_t* an_done);

/*Core Initialization*/
int qsgmiie_core_init(const phymod_core_access_t* core, const phymod_core_init_config_t* init_config, const phymod_core_status_t* core_status);

/*Phy Initialization*/
int qsgmiie_phy_init(const phymod_phy_access_t* phy, const phymod_phy_init_config_t* init_config);

/*Set\get loopback mode*/
int qsgmiie_phy_loopback_set(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t enable);
int qsgmiie_phy_loopback_get(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t* enable);

/*Get rx pmd locked indication*/
int qsgmiie_phy_rx_pmd_locked_get(const phymod_phy_access_t* phy, uint32_t* rx_pmd_locked);

/*Get link up status indication*/
int qsgmiie_phy_link_status_get(const phymod_phy_access_t* phy, uint32_t* link_status);

#endif /*_QSGMIIE_H_*/
