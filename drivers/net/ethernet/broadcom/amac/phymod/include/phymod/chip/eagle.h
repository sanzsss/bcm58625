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


#ifndef _EAGLE_H__H_
#define _EAGLE_H__H_

#include <phymod/phymod_definitions.h>
/*Initialize phymod module*/
int eagle_core_identify(const phymod_core_access_t* core, uint32_t core_id, uint32_t* is_identified);

/*Retrive core information*/
int eagle_core_info_get(const phymod_core_access_t* core, phymod_core_info_t* info);

/*Set\get lane mapping*/
int eagle_core_lane_map_set(const phymod_core_access_t* core, const phymod_lane_map_t* lane_map);
int eagle_core_lane_map_get(const phymod_core_access_t* core, phymod_lane_map_t* lane_map);

/*Reset Core*/
int eagle_core_reset_set(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t direction);
int eagle_core_reset_get(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t* direction);

/*Retrive firmware information*/
int eagle_core_firmware_info_get(const phymod_core_access_t* core, phymod_core_firmware_info_t* fw_info);

/*Start\Stop the sequencer*/
int eagle_core_pll_sequencer_restart(const phymod_core_access_t* core, uint32_t flags, phymod_sequencer_operation_t operation);

/*Wait for core event*/
int eagle_core_wait_event(const phymod_core_access_t* core, phymod_core_event_t event, uint32_t timeout);

/* re-tune rx path*/
int eagle_phy_rx_restart(const phymod_phy_access_t* phy);

/*Set phy polarity*/
int eagle_phy_polarity_set(const phymod_phy_access_t* phy, const phymod_polarity_t* polarity);
int eagle_phy_polarity_get(const phymod_phy_access_t* phy, phymod_polarity_t* polarity);

/*Set\Get TX Parameters*/
int eagle_phy_tx_set(const phymod_phy_access_t* phy, const phymod_tx_t* tx);
int eagle_phy_tx_get(const phymod_phy_access_t* phy, phymod_tx_t* tx);

/*Request for default TX parameters configuration per media type*/
int eagle_phy_media_type_tx_get(const phymod_phy_access_t* phy, phymod_media_typed_t media, phymod_tx_t* tx);

/*Set\Get TX override Parameters*/
int eagle_phy_tx_override_set(const phymod_phy_access_t* phy, const phymod_tx_override_t* tx_override);
int eagle_phy_tx_override_get(const phymod_phy_access_t* phy, phymod_tx_override_t* tx_override);

/*Set\Get RX Parameters*/
int eagle_phy_rx_set(const phymod_phy_access_t* phy, const phymod_rx_t* rx);
int eagle_phy_rx_get(const phymod_phy_access_t* phy, phymod_rx_t* rx);

/*Reset phy*/
int eagle_phy_reset_set(const phymod_phy_access_t* phy, const phymod_phy_reset_t* reset);
int eagle_phy_reset_get(const phymod_phy_access_t* phy, phymod_phy_reset_t* reset);

/*Control phy power*/
int eagle_phy_power_set(const phymod_phy_access_t* phy, const phymod_phy_power_t* power);
int eagle_phy_power_get(const phymod_phy_access_t* phy, phymod_phy_power_t* power);

/*TX transmission control*/
int eagle_phy_tx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t tx_control);
int eagle_phy_tx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t* tx_control);

/*Rx control*/
int eagle_phy_rx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t rx_control);
int eagle_phy_rx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t* rx_control);

/*TX transmission disable*/
int eagle_phy_interface_config_set(const phymod_phy_access_t* phy, uint32_t flags, const phymod_phy_inf_config_t* config);
int eagle_phy_interface_config_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_ref_clk_t ref_clock, phymod_phy_inf_config_t* config);

/*Set\Get CL72*/
int eagle_phy_cl72_set(const phymod_phy_access_t* phy, uint32_t cl72_en);
int eagle_phy_cl72_get(const phymod_phy_access_t* phy, uint32_t* cl72_en);

/*Get CL72 status*/
int eagle_phy_cl72_status_get(const phymod_phy_access_t* phy, phymod_cl72_status_t* status);

/*Set\Get autoneg*/
int eagle_phy_autoneg_ability_set(const phymod_phy_access_t* phy, const phymod_autoneg_ability_t* an_ability_set_type);
int eagle_phy_autoneg_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

/*Set\Get autoneg*/
int eagle_phy_autoneg_set(const phymod_phy_access_t* phy, const phymod_autoneg_control_t* an);
int eagle_phy_autoneg_get(const phymod_phy_access_t* phy, phymod_autoneg_control_t* an, uint32_t* an_done);

/*Get Autoneg status*/
int eagle_phy_autoneg_status_get(const phymod_phy_access_t* phy, phymod_autoneg_status_t* status);

/*Core Initialization*/
int eagle_core_init(const phymod_core_access_t* core, const phymod_core_init_config_t* init_config, const phymod_core_status_t* core_status);

/*Phy Initialization*/
int eagle_phy_init(const phymod_phy_access_t* phy, const phymod_phy_init_config_t* init_config);

/*Set\get loopback mode*/
int eagle_phy_loopback_set(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t enable);
int eagle_phy_loopback_get(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t* enable);

/*Get rx pmd locked indication*/
int eagle_phy_rx_pmd_locked_get(const phymod_phy_access_t* phy, uint32_t* rx_pmd_locked);

/*Get link up status indication*/
int eagle_phy_link_status_get(const phymod_phy_access_t* phy, uint32_t* link_status);

/*Read phymod register*/
int eagle_phy_reg_read(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t* val);

/*Write phymod register*/
int eagle_phy_reg_write(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t val);

#endif /*_EAGLE_H_*/
