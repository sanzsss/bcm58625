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


#ifndef _VIPER_H__H_
#define _VIPER_H__H_

#include <phymod/phymod_definitions.h>
/*Initialize phymod module*/
int viper_core_identify(const phymod_core_access_t* core, uint32_t core_id, uint32_t* is_identified);

/*Retrive core information*/
int viper_core_info_get(const phymod_core_access_t* core, phymod_core_info_t* info);

/*Set\get lane mapping*/
int viper_core_lane_map_set(const phymod_core_access_t* core, const phymod_lane_map_t* lane_map);
int viper_core_lane_map_get(const phymod_core_access_t* core, phymod_lane_map_t* lane_map);

/*Reset Core*/
int viper_core_reset_set(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t direction);
int viper_core_reset_get(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t* direction);

/*Start\Stop the sequencer*/
int viper_core_pll_sequencer_restart(const phymod_core_access_t* core, uint32_t flags, phymod_sequencer_operation_t operation);

/*Wait for core event*/
int viper_core_wait_event(const phymod_core_access_t* core, phymod_core_event_t event, uint32_t timeout);

/* re-tune rx path*/
int viper_phy_rx_restart(const phymod_phy_access_t* phy);

/*Set phy polarity*/
int viper_phy_polarity_set(const phymod_phy_access_t* phy, const phymod_polarity_t* polarity);
int viper_phy_polarity_get(const phymod_phy_access_t* phy, phymod_polarity_t* polarity);

/*Set\Get TX Parameters*/
int viper_phy_tx_set(const phymod_phy_access_t* phy, const phymod_tx_t* tx);
int viper_phy_tx_get(const phymod_phy_access_t* phy, phymod_tx_t* tx);

/*Set\Get RX Parameters*/
int viper_phy_rx_set(const phymod_phy_access_t* phy, const phymod_rx_t* rx);
int viper_phy_rx_get(const phymod_phy_access_t* phy, phymod_rx_t* rx);

/*Reset phy*/
int viper_phy_reset_set(const phymod_phy_access_t* phy, const phymod_phy_reset_t* reset);
int viper_phy_reset_get(const phymod_phy_access_t* phy, phymod_phy_reset_t* reset);

/*Control phy power*/
int viper_phy_power_set(const phymod_phy_access_t* phy, const phymod_phy_power_t* power);
int viper_phy_power_get(const phymod_phy_access_t* phy, phymod_phy_power_t* power);

/*TX transmission control*/
int viper_phy_tx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t tx_control);
int viper_phy_tx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t* tx_control);

/*Rx control*/
int viper_phy_rx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t rx_control);
int viper_phy_rx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t* rx_control);

/*forced speed FEC control*/
int viper_phy_fec_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
int viper_phy_fec_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*energy efficient control*/
int viper_phy_eee_set(const phymod_phy_access_t* phy, uint32_t enable);
int viper_phy_eee_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*TX transmission disable*/
int viper_phy_interface_config_set(const phymod_phy_access_t* phy, uint32_t flags, const phymod_phy_inf_config_t* config);
int viper_phy_interface_config_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_ref_clk_t ref_clock, phymod_phy_inf_config_t* config);

/*Set\Get autoneg*/
int viper_phy_autoneg_ability_set(const phymod_phy_access_t* phy, const phymod_autoneg_ability_t* an_ability_set_type);
int viper_phy_autoneg_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

/*Get  remote link autoneg*/
int viper_phy_autoneg_remote_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

/*Set\Get autoneg*/
int viper_phy_autoneg_set(const phymod_phy_access_t* phy, const phymod_autoneg_control_t* an);
int viper_phy_autoneg_get(const phymod_phy_access_t* phy, phymod_autoneg_control_t* an, uint32_t* an_done);

/*Get Autoneg status*/
int viper_phy_autoneg_status_get(const phymod_phy_access_t* phy, phymod_autoneg_status_t* status);

/*Core Initialization*/
int viper_core_init(const phymod_core_access_t* core, const phymod_core_init_config_t* init_config, const phymod_core_status_t* core_status);

/*Core vco freq get function*/
int viper_phy_pll_multiplier_get(const phymod_phy_access_t* phy, uint32_t* core_vco_pll_multiplier);

/*Phy Initialization*/
int viper_phy_init(const phymod_phy_access_t* phy, const phymod_phy_init_config_t* init_config);

/*Set\get loopback mode*/
int viper_phy_loopback_set(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t enable);
int viper_phy_loopback_get(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t* enable);

/*Get rx pmd locked indication*/
int viper_phy_rx_pmd_locked_get(const phymod_phy_access_t* phy, uint32_t* rx_pmd_locked);

/*Get link up status indication*/
int viper_phy_link_status_get(const phymod_phy_access_t* phy, uint32_t* link_status);

/*Set/Get User Speed Paramateres*/
int viper_phy_pcs_userspeed_set(const phymod_phy_access_t* phy, const phymod_pcs_userspeed_config_t* config);
int viper_phy_pcs_userspeed_get(const phymod_phy_access_t* phy, phymod_pcs_userspeed_config_t* config);

/*Read phymod register*/
int viper_phy_reg_read(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t* val);

/*Write phymod register*/
int viper_phy_reg_write(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t val);

#endif /*_VIPER_H_*/
