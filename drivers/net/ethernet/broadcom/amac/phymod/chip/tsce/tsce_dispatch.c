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


#include <phymod/phymod.h>
#include <phymod/phymod.h>
#include <phymod/phymod_dispatch.h>

#ifdef PHYMOD_TSCE_SUPPORT

#include <phymod/chip/tsce.h>

#include <phymod/chip/eagle.h>

__phymod__dispatch__t__ phymod_tsce_driver = {

    tsce_core_identify,
    tsce_core_info_get,
    tsce_core_lane_map_set,
    tsce_core_lane_map_get,
    tsce_core_reset_set,
    tsce_core_reset_get,
    tsce_core_firmware_info_get,
    tsce_phy_firmware_core_config_set,
    tsce_phy_firmware_core_config_get,
    tsce_phy_firmware_lane_config_set,
    tsce_phy_firmware_lane_config_get,
    tsce_core_pll_sequencer_restart,
    tsce_core_wait_event,
    tsce_phy_rx_restart,
    tsce_phy_polarity_set,
    tsce_phy_polarity_get,
    tsce_phy_tx_set,
    tsce_phy_tx_get,
    eagle_phy_media_type_tx_get,
    tsce_phy_tx_override_set,
    tsce_phy_tx_override_get,
    tsce_phy_rx_set,
    tsce_phy_rx_get,
    tsce_phy_rx_adaptation_resume,
    tsce_phy_reset_set,
    tsce_phy_reset_get,
    tsce_phy_power_set,
    tsce_phy_power_get,
    NULL, /* phymod_phy_hg2_codec_control_set */
    NULL, /* phymod_phy_hg2_codec_control_get */
    tsce_phy_tx_lane_control_set,
    tsce_phy_tx_lane_control_get,
    tsce_phy_rx_lane_control_set,
    tsce_phy_rx_lane_control_get,
    tsce_phy_fec_enable_set,
    tsce_phy_fec_enable_get,
    tsce_phy_autoneg_oui_set,
    tsce_phy_autoneg_oui_get,
    tsce_phy_eee_set,
    tsce_phy_eee_get,
    tsce_phy_interface_config_set,
    tsce_phy_interface_config_get,
    tsce_phy_cl72_set,
    tsce_phy_cl72_get,
    tsce_phy_cl72_status_get,
    tsce_phy_autoneg_ability_set,
    tsce_phy_autoneg_ability_get,
    tsce_phy_autoneg_remote_ability_get,
    tsce_phy_autoneg_set,
    tsce_phy_autoneg_get,
    tsce_phy_autoneg_status_get,
    tsce_core_init,
    tsce_phy_pll_multiplier_get,
    tsce_phy_init,
    tsce_phy_loopback_set,
    tsce_phy_loopback_get,
    tsce_phy_rx_pmd_locked_get,
    tsce_phy_rx_signal_detect_get,
    tsce_phy_link_status_get,
    NULL, /* phymod_phy_status_dump */
    tsce_phy_pcs_userspeed_set,
    tsce_phy_pcs_userspeed_get,
    tsce_phy_reg_read,
    tsce_phy_reg_write,
    NULL, /* phymod_phy_rev_id */
    NULL, /* phymod_phy_lane_cross_switch_map_set */
    NULL, /* phymod_phy_lane_cross_switch_map_get */
    NULL, /* phymod_phy_intr_enable_set */
    NULL, /* phymod_phy_intr_enable_get */
    NULL, /* phymod_phy_intr_status_get */
    NULL, /* phymod_phy_intr_status_clear */
    NULL, /* phymod_phy_i2c_read */
    NULL, /* phymod_phy_i2c_write */
    NULL, /* phymod_phy_gpio_config_set */
    NULL, /* phymod_phy_gpio_config_get */
    NULL, /* phymod_phy_gpio_pin_value_set */
    NULL, /* phymod_phy_gpio_pin_value_get */
    NULL, /* phymod_timesync_config_set */
    NULL, /* phymod_timesync_config_get */
    NULL, /* phymod_timesync_enable_set */
    NULL, /* phymod_timesync_enable_get */
    NULL, /* phymod_timesync_nco_addend_set */
    NULL, /* phymod_timesync_nco_addend_get */
    NULL, /* phymod_timesync_framesync_mode_set */
    NULL, /* phymod_timesync_framesync_mode_get */
    NULL, /* phymod_timesync_local_time_set */
    NULL, /* phymod_timesync_local_time_get */
    NULL, /* phymod_timesync_load_ctrl_set */
    NULL, /* phymod_timesync_load_ctrl_get */
    NULL, /* phymod_timesync_tx_timestamp_offset_set */
    NULL, /* phymod_timesync_tx_timestamp_offset_get */
    NULL, /* phymod_timesync_rx_timestamp_offset_set */
    NULL, /* phymod_timesync_rx_timestamp_offset_get */
    NULL, /* phymod_timesync_capture_timestamp_get */
    NULL, /* phymod_timesync_heartbeat_timestamp_get */
    NULL, /* phymod_timesync_do_sync */
    NULL, /* phymod_edc_config_set */
    NULL, /* phymod_edc_config_get */
    NULL, /* phymod_phy_core_mode_set */
    NULL, /* phymod_phy_core_mode_get */
    NULL, /* phymod_failover_mode_set */
    NULL, /* phymod_failover_mode_get */
    NULL, /* phymod_port_init */
    NULL, /* phymod_phy_autoneg_try_enable */
    NULL, /* phymod_phy_short_chn_mode_enable_set */
    NULL, /* phymod_phy_short_chn_mode_enable_get */
    NULL, /* phymod_port_enable_set */
    NULL, /* phymod_port_enable_get */
};

#endif /* PHYMOD_TSCE_SUPPORT */
