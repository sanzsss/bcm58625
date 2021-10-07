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


#ifndef _PHY8806X_H__H_
#define _PHY8806X_H__H_

#include <phymod/phymod_definitions.h>
/*Initialize phymod module*/
int phy8806x_core_identify(const phymod_core_access_t* core, uint32_t core_id, uint32_t* is_identified);

/*Retrive core information*/
int phy8806x_core_info_get(const phymod_core_access_t* core, phymod_core_info_t* info);

/*Set\get lane mapping*/
int phy8806x_core_lane_map_set(const phymod_core_access_t* core, const phymod_lane_map_t* lane_map);
int phy8806x_core_lane_map_get(const phymod_core_access_t* core, phymod_lane_map_t* lane_map);

/*Reset Core*/
int phy8806x_core_reset_set(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t direction);
int phy8806x_core_reset_get(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t* direction);

/*Retrive firmware information*/
int phy8806x_core_firmware_info_get(const phymod_core_access_t* core, phymod_core_firmware_info_t* fw_info);

/*Set\get firmware operation mode*/
int phy8806x_phy_firmware_core_config_set(const phymod_phy_access_t* phy, phymod_firmware_core_config_t fw_core_config);
int phy8806x_phy_firmware_core_config_get(const phymod_phy_access_t* phy, phymod_firmware_core_config_t* fw_core_config);

/*Set\get firmware operation mode*/
int phy8806x_phy_firmware_lane_config_set(const phymod_phy_access_t* phy, phymod_firmware_lane_config_t fw_lane_config);
int phy8806x_phy_firmware_lane_config_get(const phymod_phy_access_t* phy, phymod_firmware_lane_config_t* fw_lane_config);

/*Start\Stop the sequencer*/
int phy8806x_core_pll_sequencer_restart(const phymod_core_access_t* core, uint32_t flags, phymod_sequencer_operation_t operation);

/*Wait for core event*/
int phy8806x_core_wait_event(const phymod_core_access_t* core, phymod_core_event_t event, uint32_t timeout);

/* re-tune rx path*/
int phy8806x_phy_rx_restart(const phymod_phy_access_t* phy);

/*Set phy polarity*/
int phy8806x_phy_polarity_set(const phymod_phy_access_t* phy, const phymod_polarity_t* polarity);
int phy8806x_phy_polarity_get(const phymod_phy_access_t* phy, phymod_polarity_t* polarity);

/*Set\Get TX Parameters*/
int phy8806x_phy_tx_set(const phymod_phy_access_t* phy, const phymod_tx_t* tx);
int phy8806x_phy_tx_get(const phymod_phy_access_t* phy, phymod_tx_t* tx);

/*Request for default TX parameters configuration per media type*/
int phy8806x_phy_media_type_tx_get(const phymod_phy_access_t* phy, phymod_media_typed_t media, phymod_tx_t* tx);

/*Set\Get TX override Parameters*/
int phy8806x_phy_tx_override_set(const phymod_phy_access_t* phy, const phymod_tx_override_t* tx_override);
int phy8806x_phy_tx_override_get(const phymod_phy_access_t* phy, phymod_tx_override_t* tx_override);

/*Set\Get RX Parameters*/
int phy8806x_phy_rx_set(const phymod_phy_access_t* phy, const phymod_rx_t* rx);
int phy8806x_phy_rx_get(const phymod_phy_access_t* phy, phymod_rx_t* rx);

/*Reset phy*/
int phy8806x_phy_reset_set(const phymod_phy_access_t* phy, const phymod_phy_reset_t* reset);
int phy8806x_phy_reset_get(const phymod_phy_access_t* phy, phymod_phy_reset_t* reset);

/*Control phy power*/
int phy8806x_phy_power_set(const phymod_phy_access_t* phy, const phymod_phy_power_t* power);
int phy8806x_phy_power_get(const phymod_phy_access_t* phy, phymod_phy_power_t* power);

/*TX transmission control*/
int phy8806x_phy_tx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t tx_control);
int phy8806x_phy_tx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t* tx_control);

/*Rx control*/
int phy8806x_phy_rx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t rx_control);
int phy8806x_phy_rx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t* rx_control);

/*forced speed FEC control*/
int phy8806x_phy_fec_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
int phy8806x_phy_fec_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*energy efficient control*/
int phy8806x_phy_eee_set(const phymod_phy_access_t* phy, uint32_t enable);
int phy8806x_phy_eee_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*TX transmission disable*/
int phy8806x_phy_interface_config_set(const phymod_phy_access_t* phy, uint32_t flags, const phymod_phy_inf_config_t* config);
int phy8806x_phy_interface_config_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_ref_clk_t ref_clock, phymod_phy_inf_config_t* config);

/*Set\Get CL72*/
int phy8806x_phy_cl72_set(const phymod_phy_access_t* phy, uint32_t cl72_en);
int phy8806x_phy_cl72_get(const phymod_phy_access_t* phy, uint32_t* cl72_en);

/*Get CL72 status*/
int phy8806x_phy_cl72_status_get(const phymod_phy_access_t* phy, phymod_cl72_status_t* status);

/*Set\Get autoneg*/
int phy8806x_phy_autoneg_ability_set(const phymod_phy_access_t* phy, const phymod_autoneg_ability_t* an_ability_set_type);
int phy8806x_phy_autoneg_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

/*Get  remote link autoneg*/
int phy8806x_phy_autoneg_remote_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

/*Set\Get autoneg*/
int phy8806x_phy_autoneg_set(const phymod_phy_access_t* phy, const phymod_autoneg_control_t* an);
int phy8806x_phy_autoneg_get(const phymod_phy_access_t* phy, phymod_autoneg_control_t* an, uint32_t* an_done);

/*Get Autoneg status*/
int phy8806x_phy_autoneg_status_get(const phymod_phy_access_t* phy, phymod_autoneg_status_t* status);

/*Core Initialization*/
int phy8806x_core_init(const phymod_core_access_t* core, const phymod_core_init_config_t* init_config, const phymod_core_status_t* core_status);

/*Core vco freq get function*/
int phy8806x_phy_pll_multiplier_get(const phymod_phy_access_t* phy, uint32_t* core_vco_pll_multiplier);

/*Phy Initialization*/
int phy8806x_phy_init(const phymod_phy_access_t* phy, const phymod_phy_init_config_t* init_config);

/*Set\get loopback mode*/
int phy8806x_phy_loopback_set(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t enable);
int phy8806x_phy_loopback_get(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t* enable);

/*Get rx pmd locked indication*/
int phy8806x_phy_rx_pmd_locked_get(const phymod_phy_access_t* phy, uint32_t* rx_pmd_locked);

/*Get link up status indication*/
int phy8806x_phy_link_status_get(const phymod_phy_access_t* phy, uint32_t* link_status);

/*Set/Get User Speed Paramateres*/
int phy8806x_phy_pcs_userspeed_set(const phymod_phy_access_t* phy, const phymod_pcs_userspeed_config_t* config);
int phy8806x_phy_pcs_userspeed_get(const phymod_phy_access_t* phy, phymod_pcs_userspeed_config_t* config);

/*Read phymod register*/
int phy8806x_phy_reg_read(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t* val);

/*Write phymod register*/
int phy8806x_phy_reg_write(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t val);

/*Port config init*/
int phy8806x_port_init(const phymod_phy_access_t* phy, const phymod_port_config_t* port_config);

/*Autoneg try enable*/
int phy8806x_phy_autoneg_try_enable(const phymod_phy_access_t* phy, const phymod_an_try_enable_control_t* an, phymod_phy_an_status_t* an_status);

/*port enable set*/
int phy8806x_port_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
int phy8806x_port_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

#endif /*_PHY8806X_H_*/
