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


#ifndef _SESTO_H__H_
#define _SESTO_H__H_

#include <phymod/phymod_definitions.h>
/*Initialize phymod module*/
int sesto_core_identify(const phymod_core_access_t* core, uint32_t core_id, uint32_t* is_identified);

/*Retrive core information*/
int sesto_core_info_get(const phymod_core_access_t* core, phymod_core_info_t* info);

/*Reset Core*/
int sesto_core_reset_set(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t direction);
int sesto_core_reset_get(const phymod_core_access_t* core, phymod_reset_mode_t reset_mode, phymod_reset_direction_t* direction);

/*Retrive firmware information*/
int sesto_core_firmware_info_get(const phymod_core_access_t* core, phymod_core_firmware_info_t* fw_info);

/*Set\get firmware operation mode*/
int sesto_phy_firmware_core_config_set(const phymod_phy_access_t* phy, phymod_firmware_core_config_t fw_core_config);
int sesto_phy_firmware_core_config_get(const phymod_phy_access_t* phy, phymod_firmware_core_config_t* fw_core_config);

/*Set\get firmware operation mode*/
int sesto_phy_firmware_lane_config_set(const phymod_phy_access_t* phy, phymod_firmware_lane_config_t fw_lane_config);
int sesto_phy_firmware_lane_config_get(const phymod_phy_access_t* phy, phymod_firmware_lane_config_t* fw_lane_config);

/*Start\Stop the sequencer*/
int sesto_core_pll_sequencer_restart(const phymod_core_access_t* core, uint32_t flags, phymod_sequencer_operation_t operation);

/* re-tune rx path*/
int sesto_phy_rx_restart(const phymod_phy_access_t* phy);

/*Set phy polarity*/
int sesto_phy_polarity_set(const phymod_phy_access_t* phy, const phymod_polarity_t* polarity);
int sesto_phy_polarity_get(const phymod_phy_access_t* phy, phymod_polarity_t* polarity);

/*Set\Get TX Parameters*/
int sesto_phy_tx_set(const phymod_phy_access_t* phy, const phymod_tx_t* tx);
int sesto_phy_tx_get(const phymod_phy_access_t* phy, phymod_tx_t* tx);

/*Request for default TX parameters configuration per media type*/
int sesto_phy_media_type_tx_get(const phymod_phy_access_t* phy, phymod_media_typed_t media, phymod_tx_t* tx);

/*Set\Get RX Parameters*/
int sesto_phy_rx_set(const phymod_phy_access_t* phy, const phymod_rx_t* rx);
int sesto_phy_rx_get(const phymod_phy_access_t* phy, phymod_rx_t* rx);

/*PHY Rx adaptation resume*/
int sesto_phy_rx_adaptation_resume(const phymod_phy_access_t* phy);

/*Reset phy*/
int sesto_phy_reset_set(const phymod_phy_access_t* phy, const phymod_phy_reset_t* reset);
int sesto_phy_reset_get(const phymod_phy_access_t* phy, phymod_phy_reset_t* reset);

/*Control phy power*/
int sesto_phy_power_set(const phymod_phy_access_t* phy, const phymod_phy_power_t* power);
int sesto_phy_power_get(const phymod_phy_access_t* phy, phymod_phy_power_t* power);

/*TX transmission control*/
int sesto_phy_tx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t tx_control);
int sesto_phy_tx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_tx_lane_control_t* tx_control);

/*Rx control*/
int sesto_phy_rx_lane_control_set(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t rx_control);
int sesto_phy_rx_lane_control_get(const phymod_phy_access_t* phy, phymod_phy_rx_lane_control_t* rx_control);

/*forced speed FEC control*/
int sesto_phy_fec_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
int sesto_phy_fec_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*TX transmission disable*/
int sesto_phy_interface_config_set(const phymod_phy_access_t* phy, uint32_t flags, const phymod_phy_inf_config_t* config);
int sesto_phy_interface_config_get(const phymod_phy_access_t* phy, uint32_t flags, phymod_ref_clk_t ref_clock, phymod_phy_inf_config_t* config);

/*Set\Get CL72*/
int sesto_phy_cl72_set(const phymod_phy_access_t* phy, uint32_t cl72_en);
int sesto_phy_cl72_get(const phymod_phy_access_t* phy, uint32_t* cl72_en);

/*Get CL72 status*/
int sesto_phy_cl72_status_get(const phymod_phy_access_t* phy, phymod_cl72_status_t* status);

/*Set\Get autoneg*/
int sesto_phy_autoneg_ability_set(const phymod_phy_access_t* phy, const phymod_autoneg_ability_t* an_ability_set_type);
int sesto_phy_autoneg_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

/*Get  remote link autoneg*/
int sesto_phy_autoneg_remote_ability_get(const phymod_phy_access_t* phy, phymod_autoneg_ability_t* an_ability_get_type);

/*Set\Get autoneg*/
int sesto_phy_autoneg_set(const phymod_phy_access_t* phy, const phymod_autoneg_control_t* an);
int sesto_phy_autoneg_get(const phymod_phy_access_t* phy, phymod_autoneg_control_t* an, uint32_t* an_done);

/*Core Initialization*/
int sesto_core_init(const phymod_core_access_t* core, const phymod_core_init_config_t* init_config, const phymod_core_status_t* core_status);

/*Phy Initialization*/
int sesto_phy_init(const phymod_phy_access_t* phy, const phymod_phy_init_config_t* init_config);

/*Set\get loopback mode*/
int sesto_phy_loopback_set(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t enable);
int sesto_phy_loopback_get(const phymod_phy_access_t* phy, phymod_loopback_mode_t loopback, uint32_t* enable);

/*Get rx pmd locked indication*/
int sesto_phy_rx_pmd_locked_get(const phymod_phy_access_t* phy, uint32_t* rx_pmd_locked);

/*Get link up status indication*/
int sesto_phy_link_status_get(const phymod_phy_access_t* phy, uint32_t* link_status);

/*Get the serdes status*/
int sesto_phy_status_dump(const phymod_phy_access_t* phy);

/*Read phymod register*/
int sesto_phy_reg_read(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t* val);

/*Write phymod register*/
int sesto_phy_reg_write(const phymod_phy_access_t* phy, uint32_t reg_addr, uint32_t val);

/*Read Revision id*/
int sesto_phy_rev_id(const phymod_phy_access_t* phy, uint32_t* rev_id);

/*Get/Set PHY interrupt enable mask*/
int sesto_phy_intr_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
int sesto_phy_intr_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

/*PHY interrupt status get*/
int sesto_phy_intr_status_get(const phymod_phy_access_t* phy, uint32_t* intr_status);

/*Clear PHY interrupt status */
int sesto_phy_intr_status_clear(const phymod_phy_access_t* phy, uint32_t intr_clr);

/*Read data from I2C device attached to PHY*/
int sesto_phy_i2c_read(const phymod_phy_access_t* phy, uint32_t flags, uint32_t addr, uint32_t offset, uint32_t size, uint8_t* data);

/*Write data to I2C device attached to PHY*/
int sesto_phy_i2c_write(const phymod_phy_access_t* phy, uint32_t flags, uint32_t addr, uint32_t offset, uint32_t size, const uint8_t* data);

/*Set/Get the configuration of a PHY GPIO pin*/
int sesto_phy_gpio_config_set(const phymod_phy_access_t* phy, int pin_no, phymod_gpio_mode_t gpio_mode);
int sesto_phy_gpio_config_get(const phymod_phy_access_t* phy, int pin_no, phymod_gpio_mode_t* gpio_mode);

/*Set/Get the output/input value of a PHY GPIO pin*/
int sesto_phy_gpio_pin_value_set(const phymod_phy_access_t* phy, int pin_no, int value);
int sesto_phy_gpio_pin_value_get(const phymod_phy_access_t* phy, int pin_no, int* value);

/*Short channel mode*/
int sesto_phy_short_chn_mode_enable_set(const phymod_phy_access_t* phy, uint32_t enable);
int sesto_phy_short_chn_mode_enable_get(const phymod_phy_access_t* phy, uint32_t* enable);

#endif /*_SESTO_H_*/
