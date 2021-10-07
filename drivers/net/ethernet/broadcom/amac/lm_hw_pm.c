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

#include "lm_hw_pm.h"
#include "pm_intf.h"
#include "lm_defs.h"
#include "linux/printk.h"
#include <linux/netdevice.h>
#include <bcm-amac-core.h>

#undef EMULATION_TEST
#define EMULATION_TEST (0)
#if !EMULATION_TEST
#include <phymod/phymod.h>

#define err_code_t uint16_t

#ifdef UNCOMMENT
static err_code_t falcon_tsc_display_eye_scan(const phymod_access_t *pa);
#endif
static phymod_bus_t device_pmm_bus;
static pmm_config_t device_pmm_cfg;

void phymod_usleep(uint32_t uS);
void phymod_sleep(int secs);
#endif


/* Local function protocol */
#if !EMULATION_TEST
static int lm_hw_pm_bringup_lcpll(pm_device_t pdev);
static int lm_hw_pm_load_port_mode(pm_device_t pdev, u32 core_config);
static void lm_hw_pm_init_local_config(pm_device_t pdev);
static int  lm_hw_pm_probe_core(pm_device_t pdev);
static int lm_hw_pm_init_config(pm_device_t pdev);

static void lm_hw_pm_set_mode(pm_device_t pdev, int port_id, u32 num_lanes,
			      u32 lane_map, u32 max_speed);

static int lm_hw_pm_cfg_phy_core_mode(pm_device_t pdev);
static int  lm_hw_pm_probe_phy(pm_device_t pdev,
			      u8 port_id,
			      pmm_core_t *core, pmm_phy_ctrl_t *pc);

static int lm_hw_pm_cfg_leds(pm_device_t pdev);
static int lm_hw_pm_sfp_laser_ctrl(pm_device_t pdev, u8 on);
static int lm_hw_pm_sfp_power_ctrl(pm_device_t pdev, u8 on);
static int lm_hw_pm_qsfp_reset_ctrl(pm_device_t pdev, u8 on);
static int lm_hw_pm_qsfp_low_power_ctrl(pm_device_t pdev, u8 on);
static int lm_hw_pm_init_bus(pm_device_t pdev);
static int lm_hw_pm_phy_config_init(pm_device_t pdev,pmm_phy_ctrl_t *pc);
static int lm_hw_pm_speed_to_intf_config_get(pm_device_t pdev,
					     u8 port,
					     u32 speed,
					     phymod_phy_inf_config_t *
					     interface_config);
static u32 lm_hw_pm_intf_to_phymod(pm_device_t pdev, u32 pmm_interface);
/*static int lm_hw_pm_link_get(pm_device_t pdev, pmm_phy_ctrl_t *pc);*/
static int lm_hw_pm_core_config_init(pm_device_t pdev,pmm_core_t *core);
static u32 lm_hw_pm_get_lane_start(struct pmm_phymod_phy *phy);
static int lm_hw_pm_an_set(pm_device_t pdev,pmm_phy_ctrl_t *pc);
static u32 lm_hw_pm_an_mode_to_phymod(u32 an_mode);
static int lm_hw_pm_enable_set(pm_device_t pdev, u32 power_en);
static int lm_hw_pm_phy_init(pm_device_t pdev,pmm_phy_ctrl_t *pc);
static int lm_hw_pm_autoneg_ability_set(pm_device_t pdev,
					struct pmm_phy_ctrl *pc);
#ifdef EXTRA_CODE
static int lm_pm_load_bcm848xx_phy_firmware(pm_device_t pdev);
static void lm_pm_write_bcm848xx_internal(pm_device_t pdev, u32 offset,
					  u32 value);
#endif
static int lm_pm_bcm848xx_cfg_link(pm_device_t pdev,pm_phy_cfg_t *phy_cfg);
static void lm_hw_pm_get_lane_and_polarity(pm_phy_cfg_t *phy_cfg,
					   u16 *tx_lane_swap,
					   u16 *rx_lane_swap,
					   u8 *rx_polarity, u8 *tx_polarity);
#endif
static void lm_hw_pm_cfg_mac_port_mode(pm_device_t pdev);

#if !EMULATION_TEST

#ifndef CHIMP_VIEW
lm_status_t lm_hw_pm_get_eye_scan(pm_device_t pdev, u8 *outstr_p);
#endif

static int lm_hw_pm_an_set(pm_device_t pdev,pmm_phy_ctrl_t *pc)
{
   pmm_config_t *pmm_cfg;
   struct pmm_phymod_phy *phy = &pc->int_phy;
   phymod_autoneg_control_t an;
   pm_phy_cfg_t phy_cfg;
	u32 an_mode;

   pmm_cfg = GET_PMM_CFG(dev);

   pm_user_get_config(pdev,&phy_cfg);

	/*an_mode = (phy_cfg.phy_cfg_mode & PORT_CFG_PHY_AN_MODE_MASK)
	 * >> PORT_CFG_PHY_AN_MODE_SHIFT;
	 */

   an_mode = PORT_CFG_PHY_AN_MODE_CL73_BAM;

   if (an_mode != PORT_CFG_PHY_AN_MODE_NONE) {
      phymod_autoneg_control_t_init(&an);

      an.enable = phy_cfg.an_enabled;
      an.num_lane_adv = 1;
      an.an_mode = lm_hw_pm_an_mode_to_phymod(an_mode);

		if (pmm_cfg->num_ports == 1)
        PHYMOD_AN_F_ALLOW_PLL_CHANGE_SET(&an);

      if ((phymod_phy_autoneg_set(&phy->pm_phy, &an)) != PM_OK) {
			pr_err("phymod_phy_autoneg_set() failed\n");
         return PM_ERROR;
      }
   }

   return PM_OK;
}

static int lm_hw_pm_enable_set(pm_device_t pdev, u32 power_en)
{
   phymod_phy_power_t phy_power;
   pmm_config_t *pmm_cfg;
   pmm_phy_ctrl_t *phy;
   pm_phy_cfg_t phy_cfg;

   pmm_cfg = GET_PMM_CFG(pdev);
   pm_user_get_config(pdev,&phy_cfg);

   phy = &pmm_cfg->phy[phy_cfg.port_idx];

   if (power_en) {
      phy_power.tx = phymodPowerOn;
      phy_power.rx = phymodPowerOn;
   } else {
      phy_power.tx = phymodPowerOff;
      phy_power.rx = phymodPowerOff;
   }

   if (phymod_phy_power_set(&phy->int_phy.pm_phy,&phy_power) != PM_OK) {
		pr_err("phymod_phy_power_set() failed\n");
     return PM_ERROR;
   }

   return PM_OK;
}

static u32 lm_hw_pm_an_mode_to_phymod(u32 an_mode)
{
   switch (an_mode) {
      case PORT_CFG_PHY_AN_MODE_NONE:
         return phymod_AN_MODE_NONE;

      case PORT_CFG_PHY_AN_MODE_CL73:
         return phymod_AN_MODE_CL73;

      case PORT_CFG_PHY_AN_MODE_CL37:
         return phymod_AN_MODE_CL37;

      case PORT_CFG_PHY_AN_MODE_CL73_BAM:
         return phymod_AN_MODE_CL73BAM;

      case PORT_CFG_PHY_AN_MODE_CL37_BAM:
         return phymod_AN_MODE_CL37BAM;

      case PORT_CFG_PHY_AN_MODE_SGMII:
         return phymod_AN_MODE_SGMII;

      default:
         return phymod_AN_MODE_NONE;
   }
}

static void lm_hw_pm_set_mode(pm_device_t pdev, int port_id, u32 num_lanes,
			      u32 lane_map, u32 max_speed)
{
   pmm_config_t *pmm_cfg;

   pmm_cfg = GET_PMM_CFG(dev);

   pmm_cfg->phy[port_id].int_phy.lane_map = lane_map;
   pmm_cfg->phy[port_id].int_phy.max_speed = max_speed;
   pmm_cfg->phy[port_id].int_phy.num_lanes = num_lanes;
   pmm_cfg->num_ports++;
}

static void lm_hw_pm_init_local_config(pm_device_t pdev)
{
   pmm_config_t *pmm_cfg;

   pmm_cfg = GET_PMM_CFG(pdev);

   pmm_cfg->core_config = 0xfffffff;
}

static int lm_hw_pm_load_port_mode(pm_device_t pdev, u32 core_config)
{
   pmm_config_t *pmm_cfg;

   pmm_cfg = GET_PMM_CFG(pdev);

   pmm_cfg->num_ports = 0;

   switch (core_config) {
      case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X10G:
         lm_hw_pm_set_mode(pdev,0,1,0x1,10000);
         break;

      case SHARED_CFG_PHY_NETWORK_PORT_MODE_2X10G:
         lm_hw_pm_set_mode(pdev,0,1,0x1,10000);
         lm_hw_pm_set_mode(pdev,1,1,0x2,10000);
         break;

      case SHARED_CFG_PHY_NETWORK_PORT_MODE_4X10G:
#if defined(NS2HACK)
        /* ON SVK, the first 2 ports are 2.5G max */
         lm_hw_pm_set_mode(pdev,0,1,0x1,2500);
         lm_hw_pm_set_mode(pdev,1,1,0x2,2500);
#else
         lm_hw_pm_set_mode(pdev,0,1,0x1,10000);
         lm_hw_pm_set_mode(pdev,1,1,0x2,10000);
#endif
         lm_hw_pm_set_mode(pdev,2,1,0x4,10000);
         lm_hw_pm_set_mode(pdev,3,1,0x8,10000);
         break;

      case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X25G:
         lm_hw_pm_set_mode(pdev,0,1,0x1,25000);
         break;

      case SHARED_CFG_PHY_NETWORK_PORT_MODE_2X25G:
        lm_hw_pm_set_mode(pdev,0,1,0x1,25000);
        lm_hw_pm_set_mode(pdev,1,1,0x2,25000);
        break;

      case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X40G:
         lm_hw_pm_set_mode(pdev,0,4,0xf,40000);
         break;

      case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X50G:
         lm_hw_pm_set_mode(pdev,0,2,0x3,50000);
         break;

      default:
        return PM_ERROR;
   }

   return PM_OK;
}

#define SPDCAP_2_LEDSPD(spdcap) (((spdcap) & (SPEED_CAPABILITY_DRV_1G \
				   | SPEED_CAPABILITY_DRV_10G)) | (((spdcap) & \
				   (SPEED_CAPABILITY_DRV_25G | \
				    SPEED_CAPABILITY_DRV_40G | \
				    SPEED_CAPABILITY_DRV_50G)) << 1))

static int lm_hw_pm_cfg_leds(pm_device_t pdev)
{
#ifdef UNCOMMENT /* Legacy Cumulus code: irrelevant */
	u32 led0 = 0, led1 = 0, led_mode, val;
   pm_phy_cfg_t phy_cfg;

   /* IPC is only available for Cumulus */
   if (!CHIP_IS_CUMULUS(pdev))
       return PM_OK;

   pm_user_get_config(pdev,&phy_cfg);

   DbgMessage2(pdev,INFORM,"core_cfg=0x%x; board_cfg1=0x%x\n",
           phy_cfg.core_config, phy_cfg.board_cfg1);
   /* Configure LED mode */
   switch (phy_cfg.core_config) {
      case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X40G:
         REG_WRITE(pdev,IPC(led_port_mode_a0),0);
         break;
      case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X10G:
      case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X25G:
      case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X50G:
         REG_WRITE(pdev,IPC(led_port_mode_a0),1);
         break;
      case SHARED_CFG_PHY_NETWORK_PORT_MODE_2X10G__P_2X25G:
      case SHARED_CFG_PHY_NETWORK_PORT_MODE_2X25G:
         REG_WRITE(pdev,IPC(led_port_mode_a0),2);
        break;
      case SHARED_CFG_PHY_NETWORK_PORT_MODE_2X10G:
         REG_WRITE(pdev,IPC(led_port_mode_a0),4);
         break;
      default:
         return PM_ERROR;
   }

   /* Todo: lane swapping check */

   REG_WRITE(pdev,IPC(led_ctrl),IPC_CAPTURE_LED_A0);

	DbgMessage4(pdev, INFORM,
		    "port_idx=0x%x; chip_rev=0x%x; Ax=0x%x; chip_id=0x%x\n",
           PORT_IDX(pdev), CHIP_REV(pdev), CHIP_REV_Ax, CHIP_ID(pdev));

   /* Check if LED0 and LED1 are configured */
   if (phy_cfg.board_cfg1 & (PORT_CFG_BOARD_LED_SPEED_0_MASK |
               PORT_CFG_BOARD_LED_SPEED_1_MASK)) {
       led0 = (phy_cfg.board_cfg1 & PORT_CFG_BOARD_LED_SPEED_0_MASK) >>
           PORT_CFG_BOARD_LED_SPEED_0_SHIFT;
       led1 = (phy_cfg.board_cfg1 & PORT_CFG_BOARD_LED_SPEED_1_MASK) >>
           PORT_CFG_BOARD_LED_SPEED_1_SHIFT;
   } else {
       /* Translate to SPDx_EN bit definitions, skipping 20G */
       led0 = SPEED_CAPABILITY_DRV_1G | SPEED_CAPABILITY_DRV_10G;
       led0 = SPDCAP_2_LEDSPD(led0);
       led1 = SPEED_CAPABILITY_DRV_25G |SPEED_CAPABILITY_DRV_40G
           | SPEED_CAPABILITY_DRV_50G;
       led1 = SPDCAP_2_LEDSPD(led1);
   }
   led_mode = (phy_cfg.board_cfg1 & PORT_CFG_BOARD_LED_MODE_MASK) >>
       PORT_CFG_BOARD_LED_MODE_SHIFT;
	pr_info(pdev, INFORM, "%s: led0=0x%x; led1=0x%x\n", __func__,
		led0, led1);

	/* A workaround to keep PM link: restore the LED_SWAP and
	 * LED_ACTIVITY_SWAP
	 */
   if (CHIP_CFG_VALUE(pdev) == CM_A) {
      REG_WRITE(pdev,IPC(led_swap),0x3210);
      REG_WRITE(pdev,IPC(led_activity_swap),0x3210);
   }

   /* Check for Cumulus A0 for a LED bug */
   if (PORT_IDX(pdev) == 0 || CHIP_REV_A0(pdev)) {
      DbgMessage(pdev,INFORM,"P0\n");
      /* Configure LED mode */
      REG_WRITE(pdev,IPC(p0_led_mode_a0),led_mode);

      REG_WRITE(pdev,IPC(p0_led_spd0_en_a0),led0);
      REG_WRITE(pdev,IPC(p0_led_spd1_en_a0),led1);

      val = REG_READ(pdev,IPC(p0_led_control));
		REG_WRITE(pdev, IPC(p0_led_control),
			  val | IPC_P0_LED_CONTROL_BLINK_RATE_ENA_A0);

   } else {
      DbgMessage(pdev,INFORM,"P1\n");
      /* Configure LED mode */
      REG_WRITE(pdev,IPC(p1_led_mode_a0),led_mode);

      REG_WRITE(pdev,IPC(p1_led_spd0_en_a0),led0);
      REG_WRITE(pdev,IPC(p1_led_spd1_en_a0),led1);

      val = REG_READ(pdev,IPC(p1_led_control));
		REG_WRITE(pdev, IPC(p1_led_control),
			  val | IPC_P1_LED_CONTROL_BLINK_RATE_ENA_A0);
   }
#endif
   return PM_OK;
}

static int lm_hw_pm_uncfg_leds(pm_device_t pdev)
{
#ifdef UNCOMMENT /*Legacy cumulus code*/
	u32 val, or_val;
   /* Wipe out LED setting: to keep link alive, the setting wipe-out
	 *needs to be done through lane swapping registers.
	 */
   if (CHIP_CFG_VALUE(pdev) == CM_A) {
      if (PORT_IDX(pdev) == 0) {
          /* Wipe out lane 0 config */
          or_val = 0x1 << 1;
      } else {
          /* Wipe out lane 1 config */
          or_val = 0x1 << 5;
      }
      val = REG_READ(pdev,IPC(led_swap));
      REG_WRITE(pdev, IPC(led_swap), val | or_val);
      val = REG_READ(pdev,IPC(led_activity_swap));
      REG_WRITE(pdev, IPC(led_activity_swap), val | or_val);
   }
#endif
   return PM_OK;
}

static int lm_hw_pm_sfp_laser_ctrl(pm_device_t pdev, u8 on)
{
   pm_phy_cfg_t phy_cfg;
	u8 gpio_pin;

   pm_user_get_config(pdev,&phy_cfg);

   gpio_pin = (phy_cfg.sfp_ctrl & TX_LASER_MASK) >> TX_LASER_SHIFT;

	if (gpio_pin != 0xff)
     GPIO_WRITE(pdev,gpio_pin,!on);

   return PM_OK;
}

static int lm_hw_pm_sfp_power_ctrl(pm_device_t pdev, u8 on)
{
   pm_phy_cfg_t phy_cfg;
	u8 gpio_pin;

   pm_user_get_config(pdev,&phy_cfg);

   gpio_pin = (phy_cfg.sfp_ctrl & PWR_DIS_MASK) >> PWR_DIS_SHIFT;

	if (gpio_pin != 0xff)
     GPIO_WRITE(pdev,gpio_pin,!on);

   return PM_OK;
}

static int lm_hw_pm_qsfp_reset_ctrl(pm_device_t pdev, u8 on)
{
   pm_phy_cfg_t phy_cfg;
	u8 gpio_pin;

   pm_user_get_config(pdev,&phy_cfg);

   gpio_pin = (phy_cfg.qsfp_ctrl & QSFP_RESET_MASK) >> QSFP_RESET_SHIFT;

	if (gpio_pin != 0xff)
     GPIO_WRITE(pdev,gpio_pin,on);

   return PM_OK;
}

static int lm_hw_pm_qsfp_low_power_ctrl(pm_device_t pdev, u8 on)
{
   pm_phy_cfg_t phy_cfg;
	u8 gpio_pin;

   pm_user_get_config(pdev,&phy_cfg);

	gpio_pin =
	    (phy_cfg.qsfp_ctrl & QSFP_LP_MODE_MASK) >> QSFP_LP_MODE_SHIFT;

	if (gpio_pin != 0xff)
     GPIO_WRITE(pdev,gpio_pin,on);

   return PM_OK;
}

static int lm_hw_pm_bringup_lcpll(pm_device_t pdev)
{
  return PM_OK;
#ifdef UNCOMMENT /* Legacy cumulus code */
  int i;

	/*Port Macro is driven by 156.25MHz clock.  This clock is disabled
    * on POR.  We need to enable this PLL if it's in disabled state.
    * This should be enabled by
    */
   if (REG_READ(pdev,IPC(lcpll_e28_pwrdn)) != LCPLL_E28_PWRDN_PWRUP) {
      REG_WRITE(pdev,IPC(lcpll_e28_pwrdn),LCPLL_E28_PWRDN_PWRUP);
      REG_WRITE(pdev,IPC(lcpll_e28_reset_vco),0);
      PM_WAIT(pdev,10000);
      REG_WRITE(pdev,IPC(lcpll_e28_reset_post),0);
   }

   /* Check if LCPLL is locked */
   for (i = 0 ; i < 100; i++) {
		if (REG_READ(pdev, IPC(lcpll_e28_lock)) & LCPLL_E28_LOCK_LOCK)
         return PM_OK;

      PM_WAIT(pdev,1000);
   }

  return PM_ERROR;
#endif
}
#endif

int lm_hw_pm_reset(pm_device_t pdev)
{
   /* Bring Port Macro out of reset */
   /* IPC is only available for Cumulus */

   lm_hw_pm_wr_reg(pdev,PM_XLPORT_XGXS0_CTRL_REG,0,
                   PM_XLPORT_XGXS0_IDDQ | PM_XLPORT_XGXS0_TSC_POWER_DOWN,
                   ACCESS_GENERIC,0);
   PM_WAIT(pdev,200);

   /* Bring TSC out of reset */
   lm_hw_pm_wr_reg(pdev,PM_XLPORT_XGXS0_CTRL_REG,0,
			PM_XLPORT_XGXS0_LCREFINT_ENABLE |
			PM_XLPORT_XGXS0_HARD_RESET, ACCESS_GENERIC, 0);

   /* Reset all MIB counters */
   lm_hw_pm_wr_reg(pdev, PM_XLPORT_MIB_RESET, 0, 0xf, ACCESS_GENERIC, 0);
   PM_WAIT(pdev, 100);
   lm_hw_pm_wr_reg(pdev, PM_XLPORT_MIB_RESET, 0, 0x0, ACCESS_GENERIC, 0);

   return PM_OK;
}

int lm_hw_pm_probe(pm_device_t pdev)
{
   pm_phy_cfg_t phy_cfg;

   pm_user_get_config(pdev,&phy_cfg);

   lm_hw_pm_init_local_config(pdev);

#if !EMULATION_TEST
   /* Bring up LCPLL first. */
   if (lm_hw_pm_bringup_lcpll(pdev) != PM_OK) {
		pr_err("LCPLL is not locked!\n");
      return PM_ERROR;
   }
#endif

   lm_hw_pm_reset(pdev);
   lm_hw_pm_cfg_mac_port_mode(pdev);
	lm_hw_pm_cfg_phy_core_mode(pdev);

#if !EMULATION_TEST
   /* Bring QSFP out of reset */
   lm_hw_pm_qsfp_reset_ctrl(pdev,1);

   /* Remove the QSFP out of Low power mode*/
   lm_hw_pm_qsfp_low_power_ctrl(pdev,0);

	/* Initialize external PHY if it's necessary */
   switch (phy_cfg.ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_TYPE_MASK) {
#ifdef UNCOMMENT /* Legacy cumulus code */
      case PORT_CFG_PHY_EXTERNAL_PHY_TYPE_BCM84856:
        /* Remove External PHY out of reset */
        REG_WRITE(pdev,IPC(ext_phy_reset),0);
        /* Load FW to KOI. */
        if (lm_pm_load_bcm848xx_phy_firmware(pdev) != PM_OK)
          return PM_ERROR;
        break;
#endif

      default:
        break;
     }
#endif

   return PM_OK;
}

#if !EMULATION_TEST
static int lm_hw_pm_cfg_phy_core_mode(pm_device_t pdev)
{
   pm_phy_cfg_t phy_cfg;
   pmm_config_t *pmm_cfg;
   pmm_core_t *core;
	u32 required_core_config;

   pmm_cfg = GET_PMM_CFG(pdev);
   core = &pmm_cfg->pmm_core;

   pm_user_get_config(pdev,&phy_cfg);

   required_core_config = phy_cfg.core_config;

   /* If the core config is different, then we have to reload */
   if (phy_cfg.core_config == SHARED_CFG_PHY_NETWORK_PORT_MODE_1X50G) {
      /* We can support 40G and 50G when we are in 1x50G */
       if (phy_cfg.speed == 40000) {
			required_core_config =
			    SHARED_CFG_PHY_NETWORK_PORT_MODE_1X40G;
       }

      if (phy_cfg.speed == 50000)
			required_core_config =
			    SHARED_CFG_PHY_NETWORK_PORT_MODE_1X50G;
   }

   if (pmm_cfg->core_config != required_core_config) {
      pmm_cfg->core_config = required_core_config;
      core->init = FALSE;

      lm_hw_pm_load_port_mode(pdev,required_core_config);
		/* Initialize PHY bus which initialize bus interface
		 * read/write functions.
		 */
      lm_hw_pm_init_bus(pdev);
      lm_hw_pm_init_config(pdev);
   }

   return PM_OK;
}

int lm_hw_pm_bringup_phy(pm_device_t pdev)
{
   pmm_config_t *pmm_cfg;
   pm_phy_cfg_t phy_cfg;
   phymod_phy_access_t *pm_phy;

   pm_user_get_config(pdev,&phy_cfg);

#ifdef PM_DEBUG
	pr_emerg
	    ("core_type=0x%x;core_config=0x%x;lane_swap=0x%x;lane_pol=0x%x\n",
	     phy_cfg.core_type, phy_cfg.core_config, phy_cfg.lane_swap,
          phy_cfg.lane_polarity);
	pr_emerg("speed_cap_mask=0x%x;phy_cfg_mode=0x%x;core_addr=0x%x",
		 phy_cfg.speed_cap_mask, phy_cfg.phy_cfg_mode,
		 phy_cfg.core_addr);
	pr_emerg("sfp_ctrl=0x%x;qsfp_ctrl=0x%x\n", phy_cfg.sfp_ctrl,
          phy_cfg.qsfp_ctrl);
#endif

	phy_cfg.core_addr = TSC_CORE_ADDR;
   lm_hw_pm_cfg_phy_core_mode(pdev);

   pmm_cfg = GET_PMM_CFG(pdev);
   pm_phy = &pmm_cfg->phy[phy_cfg.port_idx].int_phy.pm_phy;

	phy_cfg.core_addr = TSC_CORE_ADDR;
   lm_hw_pm_phy_init(pdev,&pmm_cfg->phy[phy_cfg.port_idx]);

	/* Initialize external PHY if it's necessary */
   switch (phy_cfg.ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_TYPE_MASK) {
      case PORT_CFG_PHY_EXTERNAL_PHY_TYPE_BCM84856:
        lm_pm_bcm848xx_cfg_link(pdev,&phy_cfg);
        lm_hw_pm_cfg_leds(pdev);
        break;

      default:
        /* This is for internal PHY by default. */
        lm_hw_pm_sfp_power_ctrl(pdev,1);
        lm_hw_pm_sfp_laser_ctrl(pdev,1);
        lm_hw_pm_cfg_leds(pdev);
        break;
     }

	phy_cfg.core_addr = TSC_CORE_ADDR;
   lm_hw_pm_cfg_mac_core(pdev,phy_cfg.speed ? phy_cfg.speed : 10000);

   return PM_OK;
}

int lm_hw_pm_get_link_status(pm_device_t pdev, int *link_status, u32 *speed)
{
   pmm_config_t *pmm_cfg;
   phymod_phy_access_t *pm_phy;
   phymod_phy_inf_config_t interface_config;
	u32 link_up;
   pm_phy_cfg_t phy_cfg;
	u16 val;
	u8 phy_addr;

   pm_user_get_config(pdev,&phy_cfg);
   pmm_cfg = GET_PMM_CFG(pdev);
   pm_phy = &pmm_cfg->phy[phy_cfg.port_idx].int_phy.pm_phy;
   *link_status = 0;

   switch (phy_cfg.ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_TYPE_MASK) {
      case PORT_CFG_PHY_EXTERNAL_PHY_TYPE_BCM84856:
		phy_addr =
		    (phy_cfg.
		     ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_ADDRESS_MASK) >>
          PORT_CFG_PHY_EXTERNAL_PHY_ADDRESS_SHIFT;

		val =
		    lm_hw_pm_mdio_read45(pdev, phy_addr, PHY_DEVAD_USER,
					 BCM8485x_USER_STATUS);

        if (val & BCM8485x_USER_STATUS_COPPER_LINK_UP) {
           *link_status = 1;

           switch (val & BCM8485x_USER_STATUS_SPEED_MASK) {
              case BCM8485x_USER_STATUS_SPEED_100MBPS:
                 *speed = 100;
                 break;

              case BCM8485x_USER_STATUS_SPEED_1GBPS:
                 *speed = 1000;
                 break;

              case BCM8485x_USER_STATUS_SPEED_10GBPS:
                 *speed = 10000;
                 break;

              default:
                *link_status = 0;
                break;
            }
          }
        break;

      default:
        phymod_phy_link_status_get(pm_phy, &link_up);

        if (link_up) {
          *link_status = 1;
			phymod_phy_interface_config_get(pm_phy, 0,
							phymodRefClk156Mhz,
                                          &interface_config);
          *speed = interface_config.data_rate;
        }
        break;
   }

   return PM_OK;
}

int lm_hw_pm_bringdown_phy(pm_device_t pdev)
{
   lm_hw_pm_uncfg_leds(pdev);
   return PM_OK;
}


/*****************************************************************************
 *
 * FUNCTION:      lm_hw_pm_init_config
 *
 * DESCRIPTION:   This function is called from pmm_probe. It's responsible for
 *                initializing the phys and their cores.
 *
 * IN PARAMETERS:  port_cfg - An array of port configuration of up to 4 ports
 *
 * OUT PARAMETERS: none
 *
 * RETURNS:        PM_OK/ERROR
 *
 *****************************************************************************/

static int lm_hw_pm_init_config(pm_device_t pdev)
{
   int port;
   pmm_phymod_phy_t *phy_p;
   pmm_core_t *core;
   pmm_config_t *pmm_cfg;

   DbgMessage(pdev,VERBOSE,"lm_hw_pm_init_config():\n");

   pmm_cfg = GET_PMM_CFG(pdev);

   /* Initialize the PHY according to the port configuration. In case this
    * is the first initialization for the core, initialize it as well.
    */
   for (port = 0; port < pmm_cfg->num_ports; port++) {
       phy_p = &pmm_cfg->phy[port].int_phy;
       core = &pmm_cfg->pmm_core;

		if (!core->init)
         lm_hw_pm_probe_core(pdev);

      lm_hw_pm_probe_phy(pdev,port,core,&pmm_cfg->phy[port]);
   }

   return 0;
}

void *pdev_g;

int firmware_loader_function(const phymod_core_access_t* core, uint32_t length, const uint8_t* data)
{
    /* Add code to write to XLPORT_CTRL to set the bit for mem access */
    u32 i, K;

    pr_emerg("------->Begining of firmware loading function...now in while loop ..<--------------\n");
    K = (length / 16) + (length % 16);

    lm_hw_pm_wr_reg(pdev_g, XLPORT_WC_UCMEM_CTRL, 0, 1, ACCESS_GENERIC, 0);
    for(i = 0 ; i < K  ; i++)
    {
#if 0
        for(j = 0; j < 4; j++)
        {
            wdata[j] = 0;
            for(k = 0; k < 4; k++)
            {
                wdata[j] |= (data[(i*16)+ (j*4) +k] << (k*8));  
            }
        }
#endif
        /*FIXME: Assuming little endian*/
        lm_hw_pm_tsc_wr_mem(pdev_g, XLPORT_WC_UCMEM_DATA + i, 1, (u32 *)(data + i * 16 ));
    }

    lm_hw_pm_wr_reg(pdev_g, XLPORT_WC_UCMEM_CTRL, 0, 0, ACCESS_GENERIC, 0);
    pr_emerg("---->End of firmware loading function...now in while loop ..<--------------\n");
    return 0;
}


/*****************************************************************************
 *
 * FUNCTION:      lm_hw_pm_core_config_init
 *
 * DESCRIPTION:   This function is called from pmm_core_init, and it configures
 *                the required core parameters, such as lane map/swap.
 *
 * IN PARAMETERS:  core - pointer to the pmm core object.
 *
 * OUT PARAMETERS: none
 *
 * RETURNS:        PM_OK/ERROR
 *
 *****************************************************************************/
static int lm_hw_pm_core_config_init(pm_device_t pdev,pmm_core_t *core)
{
   int i;
	u16 lane_map_rx;
	u16 lane_map_tx;
   pmm_config_t *pmm_cfg;
   phymod_core_init_config_t *core_init_config = &core->init_config;
   pm_phy_cfg_t phy_cfg;

   pm_user_get_config(pdev,&phy_cfg);
   pmm_cfg = GET_PMM_CFG(pdev);
   phymod_core_init_config_t_init(core_init_config);

	pdev_g = pdev;
	core_init_config->firmware_loader = firmware_loader_function;
	core_init_config->firmware_load_method =
	    phymodFirmwareLoadMethodInternal;
	core_init_config->firmware_load_method =
	    phymodFirmwareLoadMethodExternal;
   core_init_config->lane_map.num_of_lanes = NUM_LANES;
   PHYMOD_CORE_INIT_F_FIRMWARE_LOAD_VERIFY_SET(core_init_config);

	lm_hw_pm_get_lane_and_polarity(&phy_cfg, &lane_map_tx, &lane_map_rx,
				       NULL, NULL);

   for (i = 0; i < NUM_LANES; i++) {
		core_init_config->lane_map.lane_map_rx[i] =
		    (lane_map_rx >> (i * 4)) & 0xf;
   }

   for (i = 0; i < NUM_LANES; i++) {
		core_init_config->lane_map.lane_map_tx[i] =
		    (lane_map_tx >> (i * 4)) & 0xf;
   }

#ifdef WIN_CDIAG
#ifdef interface
#undef interface
#endif
#endif

   if (lm_hw_pm_speed_to_intf_config_get(pdev,
                                         phy_cfg.port_idx,
					      pmm_cfg->phy[phy_cfg.port_idx].
					      int_phy.max_speed,
					      &core_init_config->interface) !=
	    PM_OK) {
      return PM_ERROR;
   }

   return PM_OK;
}

static int  lm_hw_pm_probe_core(pm_device_t pdev)
{
   pm_phy_cfg_t phy_cfg;
	u8 core_addr;
   phymod_core_access_t *pm_core;
   phymod_access_t *pm_acc;
   phymod_dispatch_type_t probed_type;
   pmm_core_t *core;
   pmm_config_t *pmm_cfg;
   phymod_core_status_t core_status;
   phymod_bus_t *pmm_bus;
   int is_probed;

   pm_user_get_config(pdev,&phy_cfg);
   pmm_cfg = GET_PMM_CFG(pdev);
   pmm_bus = GET_PMM_BUS_BLK(pdev);

   core = &pmm_cfg->pmm_core;

   DbgMessage(pdev,VERBOSE,"lm_hw_pm_probe_core():\n");

   core->init = TRUE;
   core_addr = phy_cfg.core_addr;
   pm_core = &core->pm_core;
   pm_acc = &pm_core->access;


   phymod_core_access_t_init(pm_core);

   phymod_access_t_init(pm_acc);

   /* Initialize phymod_bus interface. */
   PHYMOD_ACC_USER_ACC(pm_acc) = pdev;
   PHYMOD_ACC_BUS(pm_acc) = pmm_bus;
   PHYMOD_ACC_ADDR(pm_acc) = core_addr;

   /* Probe the core with the given core configuration */
   phymod_core_probe(pm_acc, &probed_type, &is_probed);

   /* Initialize core type so that correct driver interface is used. */
   if (phy_cfg.core_type == PM_PHY_CFG_CORE_TYPE_FALCON) {
#ifdef PHYMOD_TSCF_SUPPORT
      pm_core->type = phymodDispatchTypeTscf;
#endif /*PHYMOD_TSCF_SUPPORT  */
   } else {
#ifdef PHYMOD_TSCE_SUPPORT
      pm_core->type = phymodDispatchTypeTsce;
#endif /*PHYMOD_TSCE_SUPPORT  */
   }

   if (lm_hw_pm_core_config_init(pdev,core) != PM_OK) {
		DbgMessage1(pdev, FATAL, "#%s, config_init() failed\n",
			    __func__);
       return PM_ERROR;
   }

	/* Indicates that PMD is not active so that PHYMOD can bring
	 * PMD block out of reset.
    */
   core_status.pmd_active = 0;

	if ((phymod_core_init(pm_core, &core->init_config, &core_status)) !=
	    PM_OK) {
        return PM_ERROR;
   }

   /* Reset Swap counter register. */
	//lm_hw_pm_tsc_wr(pdev, core_addr, 0xa000, 0, 0, 0xfffc);
   lm_hw_pm_tsc_wr(pdev,core_addr,0xa000,0xfffc);

   return PM_OK;
}

int lm_hw_phymod_tsc_rd(void* user_acc, uint32_t core_addr, uint32_t reg_addr, uint32_t* val)
{
	return lm_hw_pm_tsc_rd(user_acc, core_addr, reg_addr, 0, 0, val);
}

#if 0
int lm_hw_phymod_tsc_wr(void* user_acc, uint32_t core_addr, uint32_t reg_addr, uint32_t val)
{
	return lm_hw_pm_tsc_wr(user_acc, core_addr, reg_addr, 0, 0, val);
}
#endif

static int lm_hw_pm_init_bus(pm_device_t pdev)
{
   phymod_bus_t *pmm_bus;

   DbgMessage(pdev,VERBOSE,"lm_hw_pm_init_bus():\n");

   pmm_bus = GET_PMM_BUS_BLK(pdev);

   phymod_bus_t_init(pmm_bus);

   pmm_bus->bus_name = "pm";
	pmm_bus->read = (phymod_bus_read_f)lm_hw_phymod_tsc_rd;
	//pmm_bus->write = (phymod_bus_write_f)lm_hw_phymod_tsc_wr;
   pmm_bus->write = (phymod_bus_write_f)lm_hw_pm_tsc_wr;
   PHYMOD_BUS_CAP_WR_MODIFY_SET(pmm_bus);
   PHYMOD_BUS_CAP_LANE_CTRL_SET(pmm_bus);

   return PM_OK;
}
#endif

int lm_hw_pm_cfg_mtu(pm_device_t pdev)
{
    pm_phy_cfg_t phy_cfg;

    pm_user_get_config(pdev,&phy_cfg);

    lm_hw_pm_wr_reg(pdev,PM_XLMAC_RX_MAX_SIZE,0,phy_cfg.mtu,
                    ACCESS_PER_PORT,phy_cfg.port_idx);

    return PM_OK;
}

int lm_hw_pm_toggle_lagf(pm_device_t pdev)
{
	u32 lw, hw;
    pm_phy_cfg_t phy_cfg;

    pm_user_get_config(pdev,&phy_cfg);
    lm_hw_pm_rd_reg(pdev,PM_XLPORT_LAG_FAILOVER_CONFIG,
                    &hw,&lw,ACCESS_PER_PORT,phy_cfg.port_idx);
    lm_hw_pm_wr_reg(pdev,PM_XLPORT_LAG_FAILOVER_CONFIG,
                    hw,
                    lw & ~PM_XLPORT_LAG_FAILOVER_CONFIG_LINK_UP,
                    ACCESS_PER_PORT,phy_cfg.port_idx);
    lm_hw_pm_wr_reg(pdev, PM_XLPORT_LAG_FAILOVER_CONFIG,
                    hw, lw | PM_XLPORT_LAG_FAILOVER_CONFIG_LINK_UP,
                    ACCESS_PER_PORT,phy_cfg.port_idx);

    return PM_OK;
}

int pm_user_get_config(void *pd, pm_phy_cfg_t *phy_cfg)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)pd;
	u32 port_num;

	port_num = privp->ndev->ifindex;

	if (port_num == 3 || port_num == 2) {
		phy_cfg->speed = 1000; /* 1Gbps */
		phy_cfg->speed = 2500; /* 2.5 Gbps */
		/*phy_cfg->phy_cfg_mode = PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_SFI;*/
		phy_cfg->phy_cfg_mode = PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_SGMII;
	} else if (port_num == 0 || port_num == 1) {
		phy_cfg->speed = 10000; /* 10Gbps */
		phy_cfg->phy_cfg_mode = PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_XFI;
	}

	/*FIXME: Hard code to 10G for now */


	phy_cfg->speed = 2500; /* 2.5Gbps */
	phy_cfg->phy_cfg_mode = PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_SGMII;

	phy_cfg->speed = 10000; /* 10Gbps */
	phy_cfg->phy_cfg_mode = PM_PHY_CFG_PHY_CFG_MODE_SERDES_INTF_XFI;

	
	phy_cfg->mtu = 9022;
	/* port_idx is the function # */
	phy_cfg->port_idx = port_num;

	phy_cfg->preemphasis = 0;
#ifdef MAC_LOOPBACK
	phy_cfg->lpbk |= PM_PHY_CFG_LPBK_MAC;
#else
	phy_cfg->lpbk = 0;
#endif
	phy_cfg->an_enabled = 0;
	phy_cfg->ext_phy= 0;
	phy_cfg->driver_current = 0;
	phy_cfg->sfp_ctrl = 0xffffff;
	phy_cfg->qsfp_ctrl = 0xffffffff;
	phy_cfg->core_type =  PM_PHY_CFG_CORE_TYPE_EAGLE;
	phy_cfg->core_addr = TSC_CORE_ADDR + port_num;
	phy_cfg->core_config = PM_PHY_CFG_CORE_CONFIG_PORT_MODE_4X10G;
	phy_cfg->lane_swap = 0x32103210;
	phy_cfg->lane_polarity = 0;
	phy_cfg->speed_cap_mask =  PM_PHY_CFG_SPEED_CAP_MASK_DRV_1G |
		PM_PHY_CFG_SPEED_CAP_MASK_DRV_10G;
	phy_cfg->pause = PM_PHY_CFG_PAUSE_RX | PM_PHY_CFG_PAUSE_TX;

	phy_cfg->mac_addr[0] = 0x01;
	phy_cfg->mac_addr[1] = 0x23;
	phy_cfg->mac_addr[2] = 0x45;
	phy_cfg->mac_addr[3] = 0xab;
	phy_cfg->mac_addr[4] = 0xcd;
	phy_cfg->mac_addr[5] = 0xef;
	return 0;
}

int lm_hw_pm_cfg_pause(pm_device_t pdev)
{
   pm_phy_cfg_t phy_cfg;
	u32 val1[2];
	u32 val2[2];
	u8 port_idx;

   pm_user_get_config(pdev,&phy_cfg);
   port_idx = phy_cfg.port_idx;

#ifdef UNCOMMENT
   if (CHIP_IS_CUMULUS(pdev)) {
      /* Enable PAUSE frame filter at the PA block */
      val1[0] = REG_READ(pdev,PA(per_port_cntl[port_idx]));
      val1[0] |= PER_PORT_CNTL_RXP_PFC_PKT_FILTER_EN |
        PER_PORT_CNTL_RXP_PAUSE_PKT_FILTER_EN;
      REG_WRITE(pdev,PA(per_port_cntl[port_idx]),val1[0]);
   }
#endif

   /* Program Source MAC address for Pause Frames */
   lm_hw_pm_wr_reg(pdev,PM_XLMAC_TX_MAC_ADDR,
                   (phy_cfg.mac_addr[0] << 8) | phy_cfg.mac_addr[1],
			(phy_cfg.mac_addr[2] << 24) | (phy_cfg.
						       mac_addr[3] << 16) |
                   (phy_cfg.mac_addr[4] << 8) | phy_cfg.mac_addr[5],
                   ACCESS_PER_PORT,port_idx);

   if (phy_cfg.pause & PM_PHY_CFG_PAUSE_PFC_ENABLE) {
      /* Disable LLFC first. */
		lm_hw_pm_rd_reg(pdev, PM_XLMAC_PAUSE_CTRL, &val1[1], &val1[0],
				ACCESS_PER_PORT, port_idx);
		lm_hw_pm_rd_reg(pdev, PM_XLMAC_LLFC_CONTROL, &val2[1], &val2[0],
				ACCESS_PER_PORT, port_idx);
		val1[0] &=
		    ~(XLMAC_PAUSE_CTRL_RX_PAUSE_EN |
		      XLMAC_PAUSE_CTRL_TX_PAUSE_EN);
		val2[0] &=
		    ~(XLMAC_LLFC_CTRL_RX_LLFC_EN | XLMAC_LLFC_CTRL_TX_LLFC_EN);
		lm_hw_pm_wr_reg(pdev, PM_XLMAC_PAUSE_CTRL, val1[1], val1[0],
				ACCESS_PER_PORT, port_idx);
		lm_hw_pm_wr_reg(pdev, PM_XLMAC_LLFC_CONTROL, val2[1], val2[0],
				ACCESS_PER_PORT, port_idx);

      /* Configure PFC */
		lm_hw_pm_rd_reg(pdev, PM_XLMAC_PFC_CTRL, &val1[1], &val1[0],
				ACCESS_PER_PORT, port_idx);
		if (phy_cfg.pause & PM_PHY_CFG_PAUSE_RX)
         val1[1] |= XLMAC_PFC_CTRL_RX_PFC_EN_HI;
		else
         val1[1] &= ~XLMAC_PFC_CTRL_RX_PFC_EN_HI;

      if (phy_cfg.pause & PM_PHY_CFG_PAUSE_TX) {
			val1[1] |=
			    XLMAC_PFC_CTRL_TX_PFC_EN_HI |
			    XLMAC_PFC_CTRL_PFC_REFRESH_EN_HI;
      } else {
			val1[1] &=
			    ~(XLMAC_PFC_CTRL_TX_PFC_EN_HI |
			      XLMAC_PFC_CTRL_PFC_REFRESH_EN_HI);
      }

		lm_hw_pm_wr_reg(pdev, PM_XLMAC_PFC_CTRL, val1[1], val1[0],
				ACCESS_PER_PORT, port_idx);
	} else {
      /* Disable PFC first. */
		lm_hw_pm_rd_reg(pdev, PM_XLMAC_PFC_CTRL, &val1[1], &val1[0],
				ACCESS_PER_PORT, port_idx);
      val1[1] &= ~XLMAC_PFC_CTRL_RX_PFC_EN_HI;
		val1[1] &=
		    ~(XLMAC_PFC_CTRL_TX_PFC_EN_HI |
		      XLMAC_PFC_CTRL_PFC_REFRESH_EN_HI);
		lm_hw_pm_wr_reg(pdev, PM_XLMAC_PFC_CTRL, val1[1], val1[0],
				ACCESS_PER_PORT, port_idx);

      /* Configure LLFC */
		lm_hw_pm_rd_reg(pdev, PM_XLMAC_PAUSE_CTRL, &val1[1], &val1[0],
				ACCESS_PER_PORT, port_idx);
		lm_hw_pm_rd_reg(pdev, PM_XLMAC_LLFC_CONTROL, &val2[1], &val2[0],
				ACCESS_PER_PORT, port_idx);

      if (phy_cfg.pause & PM_PHY_CFG_PAUSE_RX) {
         val1[0] |= XLMAC_PAUSE_CTRL_RX_PAUSE_EN;
         val2[0] |= XLMAC_LLFC_CTRL_RX_LLFC_EN;
      } else {
         val1[0] &= ~XLMAC_PAUSE_CTRL_RX_PAUSE_EN;
         val2[0] &= ~XLMAC_LLFC_CTRL_RX_LLFC_EN;
     }

      if (phy_cfg.pause & PM_PHY_CFG_PAUSE_TX) {
         val1[0] |= XLMAC_PAUSE_CTRL_TX_PAUSE_EN;
         val2[0] |= XLMAC_LLFC_CTRL_TX_LLFC_EN;
      } else {
         val1[0] &= ~XLMAC_PAUSE_CTRL_TX_PAUSE_EN;
         val2[0] &= ~XLMAC_LLFC_CTRL_TX_LLFC_EN;
      }

		lm_hw_pm_wr_reg(pdev, PM_XLMAC_PAUSE_CTRL, val1[1], val1[0],
				ACCESS_PER_PORT, port_idx);
		lm_hw_pm_wr_reg(pdev, PM_XLMAC_LLFC_CONTROL, val2[1], val2[0],
				ACCESS_PER_PORT, port_idx);
   }

   return PM_OK;
}

static void lm_hw_pm_cfg_mac_port_mode(pm_device_t pdev)
{
    pm_phy_cfg_t phy_cfg;
	u32 msb;
	u32 lsb;

	lm_hw_pm_rd_reg(pdev, PM_XLPORT_MODE_REG, &msb, &lsb, ACCESS_GENERIC,
			0);
	lsb &=
	    ~(PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_MASK |
	      PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_MASK);

	pm_user_get_config(pdev, &phy_cfg);

	switch (phy_cfg.core_config & SHARED_CFG_PHY_NETWORK_PORT_MODE_MASK) {
	case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X50G:
	case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X25G:
	case SHARED_CFG_PHY_NETWORK_PORT_MODE_1X40G:
		lsb |= PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SINGLE |
		    PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SINGLE;

		break;

	case SHARED_CFG_PHY_NETWORK_PORT_MODE_4X10G:
	case SHARED_CFG_PHY_NETWORK_PORT_MODE_2X10G:
	case SHARED_CFG_PHY_NETWORK_PORT_MODE_2X25G:
		lsb |= PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_QUAD |
		    PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_QUAD;
		break;

	default:
		/* Single port by default. */
		lsb |= PM_XLPORT_MODE_REG_XPORT0_CORE_PORT_MODE_SINGLE |
		    PM_XLPORT_MODE_REG_XPORT0_PHY_PORT_MODE_SINGLE;
		break;
	}

	lm_hw_pm_wr_reg(pdev, PM_XLPORT_MODE_REG, msb, lsb, ACCESS_GENERIC, 0);
	/* Remove XMAC_MP out of reset */
	lm_hw_pm_wr_reg(pdev, PM_XLPORT_MAC_CONTROL, 0, 0, ACCESS_GENERIC, 0);

	/* Reset all MIB counters */
	lm_hw_pm_wr_reg(pdev, PM_XLPORT_MIB_RESET, 0, 0xf, ACCESS_GENERIC, 0);
	PM_WAIT(pdev, 100);
	lm_hw_pm_wr_reg(pdev, PM_XLPORT_MIB_RESET, 0, 0x0, ACCESS_GENERIC, 0);
}

int lm_hw_pm_cfg_mac_core(pm_device_t pdev, u32 speed)
{
	pm_phy_cfg_t phy_cfg;
	u32 msb;
	u32 lsb;
	u8 port_idx;
	/*u32 mac_reg = XLMAC_CTRL_TX_EN | XLMAC_CTRL_RX_EN;*/
	u32 enable_ports = 0;
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)pdev;
#ifdef EXTRA_CODE
	pmm_config_t *pmm_cfg;
	phymod_phy_inf_config_t interface_config;
#endif

	enable_ports = enable_ports | (0x1 << XLPORT_ENABLE_REG__PORT3) |
	   (0x1 << XLPORT_ENABLE_REG__PORT2) | (0x1 << XLPORT_ENABLE_REG__PORT1)
	    | (0x1 << XLPORT_ENABLE_REG__PORT0);
	/* step 1 enable all XLPORT */
	lm_hw_pm_wr_reg(pdev, XLPORT_ENABLE_REG, 0, enable_ports, 1, 0);

	pm_user_get_config(pdev,&phy_cfg);

	port_idx = privp->amac_id;

#ifdef EXTRA_CODE
#if !EMULATION_TEST
    if ((phy_cfg.ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_TYPE_MASK) ==
        PORT_CFG_PHY_EXTERNAL_PHY_TYPE_BCM84856) {
      pmm_cfg = GET_PMM_CFG(pdev);
      lm_hw_pm_speed_to_intf_config_get(pdev,port_idx,
                                        speed,&interface_config);

		if (pmm_cfg->phy[port_idx].int_phy.interface_type !=
		    interface_config.interface_type) {
			pmm_cfg->phy[port_idx].int_phy.interface_type =
			    interface_config.interface_type;

			phymod_phy_interface_config_set(&pmm_cfg->phy[port_idx].
							int_phy.pm_phy,
                                         0 /* flags */,
                                         &interface_config);
      }
    }
#endif

    if (phy_cfg.lpbk & PM_PHY_CFG_LPBK_MAC)
      mac_reg |= XLMAC_CTRL_LOCAL_LPBK;

    /* Put XLMAC in reset */
    lm_hw_pm_wr_reg(pdev,PM_XLMAC_CTRL,0,mac_reg | XLMAC_CTRL_SOFT_RESET,
			ACCESS_PER_PORT, port_idx);

#ifdef UNCOMMENT	/* There is no PA block in Pegasus PM */
    /* Put PA into reset state. */
    val = REG_READ(pdev,PA(port_config));
    val |= PA_ENABLE_LOOPBACK | PA_ENABLE_LOOPBACK_INT_HDR_SKIP;
    val &= ~(PA_TX_PORT_INIT_TX_PORT0_RST << port_idx);
    REG_WRITE(pdev,PA(port_config),
              val | (PA_TX_PORT_INIT_TX_PORT0_RST << port_idx));
    PM_WAIT(pdev,100);
    /* Release PA out of reset */
    REG_WRITE(pdev,PA(port_config),val);

    /* Configure PA block to frop PAUSE frames */
    REG_WRITE(pdev,PA(per_port_cntl[port_idx]),
              PER_PORT_CNTL_RXP_PFC_PKT_FILTER_EN |
              PER_PORT_CNTL_RXP_PAUSE_PKT_FILTER_EN);
#endif

	/* Config CLPORT so that device can transmit packets
	 * from 36-byte instead of 49bytes
	 */
    if (phy_cfg.core_type == PM_PHY_CFG_CORE_TYPE_FALCON)
      mac_reg |= CLMAC_CTRL_ALLOW_40B_AND_GREATER_PKT;

    /* Take XLMAC out of reset */
	lm_hw_pm_wr_reg(pdev, PM_XLMAC_CTRL, 0, mac_reg, ACCESS_PER_PORT,
			port_idx);

    switch (speed) {
       case 2500:
          val = XLMAC_MODE_SPEED_MODE_LINK_2G5;
          break;

       case 1000:
          val = XLMAC_MODE_SPEED_MODE_LINK_1G;
          break;

       case 100:
          val = XLMAC_MODE_SPEED_MODE_LINK_100M;
          break;

       case 10:
          val = XLMAC_MODE_SPEED_MODE_LINK_10M;
          break;

       default:
          val = XLMAC_MODE_SPEED_MODE_LINK_10G_PLUS;
          break;
    }

    lm_hw_pm_wr_reg(pdev,PM_XLMAC_MODE,0,val,ACCESS_PER_PORT,port_idx);
#endif

    lm_hw_pm_cfg_pause(pdev);

	/* Hardcode MTU to maximum and let RE_VNIC configuration
	 * dictates the MTU size
	 */
    lm_hw_pm_wr_reg(pdev,PM_XLMAC_RX_MAX_SIZE,0,
			phy_cfg.mtu, ACCESS_PER_PORT, port_idx);

	lm_hw_pm_rd_reg(pdev, PM_XLMAC_TX_CTRL, &msb, &lsb, ACCESS_PER_PORT,
			port_idx);
    lsb &= ~XLMAC_TX_CTRL_CRC_MODE_MASK;
    lsb |= XLMAC_TX_CTRL_CRC_MODE_APPEND;

    /* Enable padding. */
    lsb |= XLMAC_TX_CTRL_PAD_EN;
	lm_hw_pm_wr_reg(pdev, PM_XLMAC_TX_CTRL, msb, lsb, ACCESS_PER_PORT,
			port_idx);

    /* Enable parallel PFC interface */
	lm_hw_pm_wr_reg(pdev, PM_XLPORT_FLOW_CONTROL_CONFIG, 0, 1,
			ACCESS_PER_PORT, port_idx);

    /* Enable all lanes on the MAC level. */
    lm_hw_pm_wr_reg(pdev,PM_XLPORT_ENABLE_REG,0,0xf, ACCESS_GENERIC,0);

    return PM_OK;
}

#if !EMULATION_TEST
static u32 lm_hw_pm_intf_to_phymod(pm_device_t pdev, u32 pmm_interface)
{
   phymod_interface_t phymod_interface;

   switch (pmm_interface) {
      case PORT_CFG_PHY_SERDES_NET_INTERFACE_KR:
         phymod_interface = phymodInterfaceKR;
         break;

      case PORT_CFG_PHY_SERDES_NET_INTERFACE_KR2:
         phymod_interface = phymodInterfaceKR2;
         break;

      case PORT_CFG_PHY_SERDES_NET_INTERFACE_KR4:
         phymod_interface = phymodInterfaceKR4;
         break;

      case PORT_CFG_PHY_SERDES_NET_INTERFACE_XFI:
         phymod_interface = phymodInterfaceXFI;
         break;

      case PORT_CFG_PHY_SERDES_NET_INTERFACE_SFI:
         phymod_interface = phymodInterfaceSFI;
         break;

      case PORT_CFG_PHY_SERDES_NET_INTERFACE_1000X:
         phymod_interface = phymodInterface1000X;
         break;

      case PORT_CFG_PHY_SERDES_NET_INTERFACE_SGMII:
         phymod_interface = phymodInterfaceSGMII;
         break;

      default:
         phymod_interface = phymodInterfaceBypass;
         break;
   }

   return phymod_interface;
}

int get_speed(struct bcm_amac_priv *privp)
{
	u32 val_msb, val_lsb;

	lm_hw_pm_rd_reg(privp, XLMAC_MODE, &val_msb, &val_lsb, 0, 0);
	return ((val_lsb >> 4) & 0x7);
}

static int lm_hw_pm_speed_to_intf_config_get(pm_device_t pdev,
					     u8 port,
					     u32 speed,
					     phymod_phy_inf_config_t *
					     interface_config)
{
   pm_phy_cfg_t phy_cfg;
	u32 interface_type;

	DbgMessage2(pdev, VERBOSE, "intf_config_get(): port=%x speed=%d\n",
		    port, speed);

   phymod_phy_inf_config_t_init(interface_config);
   pm_user_get_config(pdev,&phy_cfg);

	interface_type =
	    phy_cfg.phy_cfg_mode & PORT_CFG_PHY_SERDES_NET_INTERFACE_MASK;

   interface_config->data_rate = speed;

   switch (speed) {
      case 100:
      case 1000:
        switch (phy_cfg.ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_TYPE_MASK)  {
          case PORT_CFG_PHY_EXTERNAL_PHY_TYPE_BCM84856:
            /* For 10G-base-T PHY, it has to be SGMII interface for
			 * 100MBPS/1000MBPS.
			 */
            interface_config->interface_type = phymodInterfaceSGMII;
            break;

          default:
            interface_config->interface_type = phymodInterface1000X;
            break;
          }
        break;

      case 10000:
		switch (phy_cfg.ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_TYPE_MASK) {
          case PORT_CFG_PHY_EXTERNAL_PHY_TYPE_BCM84856:
            /* For 10G-base-T, it has to be XFI interface */
            interface_config->interface_type = phymodInterfaceXFI;
            break;

          default:
            interface_config->interface_type = phymodInterfaceSFI;
            break;
          }
        break;

      case 25000:
         interface_config->interface_type = phymodInterfaceKR;
         break;

      case 40000:
      case 100000:
         interface_config->interface_type = phymodInterfaceKR4;
         break;

      case 20000:
      case 50000:
         interface_config->interface_type = phymodInterfaceKR2;
         break;

      default:
        interface_config->data_rate = 0;
        /* Use NVRAM config if the speed is unknown. */
		interface_config->interface_type =
		    lm_hw_pm_intf_to_phymod(pdev, interface_type);
        break;
   }

   interface_config->ref_clock = phymodRefClk156Mhz;

   return PM_OK;
}

static u32 lm_hw_pm_get_lane_start(struct pmm_phymod_phy *phy)
{
   int lane;

	for (lane = 0; lane < NUM_LANES; lane++)
		if (phy->lane_map & (1 << lane))
         break;

   return lane;
}

static int lm_hw_pm_phy_config_init(pm_device_t pdev,pmm_phy_ctrl_t *pc)
{
	u8 lane, lane_start;
	u8 start_lane_idx;
    struct pmm_phymod_phy *phy = &pc->int_phy;
    phymod_phy_init_config_t *init_config = &phy->init_config;
	u8 rx_polarity;
	u8 tx_polarity;
	u32 preemp;
	u32 amp;
    pm_phy_cfg_t phy_cfg;

	DbgMessage1(pdev, VERBOSE, "phy_config_init(): lane_map == 0x%x\n",
		    phy->lane_map);

    pm_user_get_config(pdev,&phy_cfg);

    phymod_phy_init_config_t_init(init_config);

    init_config->cl72_en = 0;
    init_config->polarity.rx_polarity = 0;
    init_config->polarity.tx_polarity = 0;
    lane_start = lm_hw_pm_get_lane_start(phy);

    /* Get TX and RX polarity */
	lm_hw_pm_get_lane_and_polarity(&phy_cfg, NULL, NULL, &rx_polarity,
				       &tx_polarity);

    preemp = phy_cfg.preemphasis;
    amp = phy_cfg.driver_current;

    start_lane_idx = 0;

    for (lane = 0; lane < NUM_LANES; lane++) {
       if (phy->lane_map & (1 << lane)) {
          init_config->tx[start_lane_idx].post2 = 0x0;
          init_config->tx[start_lane_idx].post3 = 0x0;

			if (phy_cfg.
			    core_config &
			    SHARED_CFG_PHY_ENFORCE_PREEMPHASIS_CFG_ENABLED) {
				init_config->tx[start_lane_idx].pre =
				    (preemp >> (lane * 4)) & 0xff;
				init_config->tx[start_lane_idx].amp =
				    (amp >> (lane * 4)) & 0xff;

          } else{
               /* Default: Pre/Main/Post == 20/60/32 */
               init_config->tx[start_lane_idx].pre = 0x14;
               init_config->tx[start_lane_idx].amp = 0xc;
               init_config->tx[start_lane_idx].main = 0x3c;
               init_config->tx[start_lane_idx].post = 0x20;
          }

          if (rx_polarity & (1 << lane))
				init_config->polarity.rx_polarity |=
				    (1 << (lane - lane_start));

          if (tx_polarity & (1 << lane))
				init_config->polarity.tx_polarity |=
				    (1 << (lane - lane_start));

          start_lane_idx++;
       }
    }

    init_config->an_en = 0;

    return PM_OK;
}

static int lm_hw_pm_probe_phy(pm_device_t pdev,
			      u8 port_id,
			      pmm_core_t *core, pmm_phy_ctrl_t *pc)
{
    pm_phy_cfg_t phy_cfg;
    pmm_phymod_phy_t *phy = &pc->int_phy;

    pm_user_get_config(pdev,&phy_cfg);
    phymod_phy_access_t_init(&phy->pm_phy);

    /* Copy core_access struct to the phy_access struct */
    memcpy(&phy->pm_phy,&core->pm_core, sizeof(phymod_core_access_t));

    /* Update the lane map (which lane/s apply for this port) */
    phy->pm_phy.access.lane_mask = phy->lane_map;

    /* Set the core pointer */
    phy->core = core;

    lm_hw_pm_phy_config_init(pdev,pc);

    return PM_OK;
}

void phymod_usleep(uint32_t usecs)
{
  PM_WAIT(NULL,usecs);
}

void phymod_sleep(int secs)
{
  PM_WAIT(NULL,1000000*secs);
}

void *phymod_malloc(size_t size, char *descr)
{
  return NULL;
}

void phymod_free(void *buf)
{
}

void phymod_printf(char *s,...)
{
}

static int lm_hw_pm_phy_init(pm_device_t pdev,pmm_phy_ctrl_t *pc)
{
   struct pmm_phymod_phy *phy;
   struct pmm_core *core;
   phymod_core_info_t core_info;
   phymod_phy_inf_config_t interface_config;
	u32 loopback_reg, core_addr, loopback_val, lane;
   int rc;
   pm_phy_cfg_t phy_cfg;

   pm_user_get_config(pdev,&phy_cfg);
   phy = &pc->int_phy;
   core = phy->core;

   rc = phymod_phy_init(&phy->pm_phy, &phy->init_config);

   if (rc != PM_OK) {
		DbgMessage1(pdev, FATAL,
			    "Failed calling phymod_phy_init 0x%x\n", rc);
      return PM_ERROR;
   }

   /* Set the port to its max or init_speed */
   if ((lm_hw_pm_speed_to_intf_config_get(pdev,phy_cfg.port_idx,
					       phy_cfg.speed,
					       &interface_config)) != PM_OK) {
		pr_err("Failed calling pmm_speed_to_interface_config_get\n");
      return PM_ERROR;
   }

	if (phymod_phy_interface_config_set(&phy->pm_phy, 0 /* flags */,
                                       &interface_config) != PM_OK) {
		pr_err("Failed setting interface config\n");
      return PM_ERROR;
   }

   phy->interface_type = interface_config.interface_type;

   if (phymod_core_info_get(&core->pm_core, &core_info) != PM_OK) {
      DbgMessage(NULL,FATAL,"Failed getting core info\n");
      return PM_ERROR;
   }

	pr_debug("PHY: core_ver=0x%x, serdes_id=0x%x, PHYID=%#x/%#x,%s\n",
	       core_info.core_version, core_info.serdes_id,
	       core_info.phy_id0, core_info.phy_id1, core_info.name);

   memset((void*)&interface_config, 0, sizeof(phymod_phy_inf_config_t));

   /* We don't need to care about AUTONEG in serdes for external PHY. */
   if ((phy_cfg.ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_TYPE_MASK) == 
       PORT_CFG_PHY_EXTERNAL_PHY_TYPE_DIRECT) {
       if (phy_cfg.an_enabled) {
          if (lm_hw_pm_autoneg_ability_set(pdev,pc) != PM_OK) {
				pr_err("Failed advertising AN abilities\n");
            return PM_ERROR;
          }

          if (lm_hw_pm_an_set(pdev,pc) != PM_OK) {
				pr_err("Failed setting AN mode\n");
            return PM_ERROR;
          }
       }
   }

   core_addr = phy_cfg.core_addr;

   if (phy_cfg.core_type == PM_PHY_CFG_CORE_TYPE_FALCON) {
      loopback_reg = 0x800d162;
		//loopback_reg = 0x800d162;

      if (phy_cfg.lpbk == PM_PHY_CFG_LPBK_INT_PHY)
        loopback_val = 0xb;
      else
        loopback_val = 0xa;
   } else {
      loopback_reg = 0x800d0d2;
		//loopback_reg = 0xd0d2;

      if (phy_cfg.lpbk == PM_PHY_CFG_LPBK_INT_PHY)
         loopback_val = 0x7;
      else
         loopback_val = 0x6;

   }

   for (lane = 0; lane < NUM_LANES; lane++) {
     if (phy->lane_map & (1 << lane)) {
#if 0
			lm_hw_pm_tsc_wr(pdev, core_addr,
					loopback_reg , lane,
					0, loopback_val);
#endif
       lm_hw_pm_tsc_wr(pdev, core_addr, loopback_reg | (lane << 16), loopback_val);
     }
   }

   /* Enable the PHY */
   lm_hw_pm_enable_set(pdev,1);

   return PM_OK;
}

static int lm_hw_pm_autoneg_ability_set(pm_device_t pdev,
					struct pmm_phy_ctrl *pc)
{
   phymod_autoneg_ability_t phymod_autoneg_ability;
   pmm_phymod_phy_t *phy;
   pm_phy_cfg_t phy_cfg;
	u32 an_tech_ability = 0;
	u32 an_bam37_ability = 0;
	u32 an_bam73_ability = 0;

   phy = &pc->int_phy;

   pm_user_get_config(pdev,&phy_cfg);

   phymod_autoneg_ability_t_init(&phymod_autoneg_ability);

   if (phy_cfg.autoneg_cap & PM_PHY_CFG_AUTONEG_CAP_50GBPS)
      PHYMOD_BAM_CL73_CAP_50G_KR2_SET(an_bam73_ability);

	if (phy_cfg.autoneg_cap & PM_PHY_CFG_AUTONEG_CAP_25GBPS)
     PHYMOD_BAM_CL73_CAP_25G_KR1_SET(an_bam73_ability);

   if (phy_cfg.autoneg_cap & PM_PHY_CFG_AUTONEG_CAP_10GBPS) {
      PHYMOD_AN_CAP_10G_KR_SET(an_tech_ability);
      PHYMOD_AN_CAP_10G_KR_SET(an_bam37_ability);
   }

   if (phy_cfg.autoneg_cap & PM_PHY_CFG_AUTONEG_CAP_1GBPS) {
     PHYMOD_AN_CAP_1G_KX_SET(an_tech_ability);
     PHYMOD_AN_CAP_1G_KX_SET(an_bam37_ability);
   }

   /* Advertise the flow-control capabilities */
	if (phy_cfg.pause & PM_PHY_CFG_PAUSE_TX)
      PHYMOD_AN_CAP_ASYM_PAUSE_SET(&phymod_autoneg_ability);
	else
      PHYMOD_AN_CAP_ASYM_PAUSE_CLR(&phymod_autoneg_ability);

	if (phy_cfg.pause & PM_PHY_CFG_PAUSE_RX)
      PHYMOD_AN_CAP_SYMM_PAUSE_SET(&phymod_autoneg_ability);
	else
      PHYMOD_AN_CAP_SYMM_PAUSE_CLR(&phymod_autoneg_ability);

   phymod_autoneg_ability.an_cap = an_tech_ability;
   phymod_autoneg_ability.cl37bam_cap = an_bam37_ability;
   phymod_autoneg_ability.cl73bam_cap = an_bam73_ability;

   PHYMOD_AN_CAP_SGMII_SET(&phymod_autoneg_ability);
   phymod_autoneg_ability.sgmii_speed = phymod_CL37_SGMII_1000M;

   /* Set CL72 ability */
   phymod_autoneg_ability.an_cl72 = 1;
   phymod_autoneg_ability.an_fec = 1;

   phymod_phy_autoneg_ability_set(&phy->pm_phy, &phymod_autoneg_ability);

   return PM_OK;
}
#endif

int lm_pm_get_counter(pm_device_t pdev, u32 counter_idx, pm_counter_t *counter)
{
  pm_phy_cfg_t phy_cfg;

  if (counter_idx > PM_XLPORT_XTHOL)
    return PM_ERROR;

  pm_user_get_config(pdev,&phy_cfg);

  return lm_hw_pm_rd_reg(pdev,counter_idx,&counter->high,&counter->low,
                         ACCESS_PER_PORT,phy_cfg.port_idx);
}

int lm_pm_get_rx_counters(pm_device_t pdev,pm_rx_statistics_t *rx_counters)
{
  pm_phy_cfg_t phy_cfg;
  pm_counter_t *counter;
	u32 i;

  pm_user_get_config(pdev,&phy_cfg);
  counter = (pm_counter_t *)rx_counters;

  for (i=PM_XLPORT_R64; i <= PM_XLPORT_RRPKT; i++) {
     if (lm_hw_pm_rd_reg(pdev,i,&counter->high,&counter->low,
				    ACCESS_PER_PORT,
				    phy_cfg.port_idx) != PM_OK) {
        return PM_ERROR;
     }

     counter++;
  }

  return PM_OK;
}

int lm_pm_get_tx_counters(pm_device_t pdev,pm_tx_statistics_t *tx_counters)
{
  pm_phy_cfg_t phy_cfg;
  pm_counter_t *counter;
	u32 i;

  pm_user_get_config(pdev,&phy_cfg);
  counter = (pm_counter_t *)tx_counters;

  for (i=PM_XLPORT_T64; i <= PM_XLPORT_XTHOL; i++) {
     if (lm_hw_pm_rd_reg(pdev,i,&counter->high,&counter->low,
				    ACCESS_PER_PORT,
				    phy_cfg.port_idx) != PM_OK) {
       return PM_ERROR;
     }

     counter++;
  }

  return PM_OK;
}

#ifdef EXTRA_CODE
static void lm_pm_write_bcm848xx_internal(pm_device_t pdev, u32 offset,
					  u32 value)
{
  lm_hw_pm_mdio_write45(pdev,PHY_ADDR_BCAST,PHY_DEVAD_PMD,
			      BCM8485x_PMD_DL_ADDR_LOW,
			      (u16)(offset & 0xffff));
  lm_hw_pm_mdio_write45(pdev,PHY_ADDR_BCAST,PHY_DEVAD_PMD,
			      BCM8485x_PMD_DL_ADDR_HIGH, (u16)(offset >> 16));
  lm_hw_pm_mdio_write45(pdev,PHY_ADDR_BCAST,PHY_DEVAD_PMD,
			      BCM8485x_PMD_DL_DATA_LOW, (u16)(value & 0xffff));
  lm_hw_pm_mdio_write45(pdev,PHY_ADDR_BCAST,PHY_DEVAD_PMD,
			      BCM8485x_PMD_DL_DATA_HIGH, (u16)(value >> 16));
  lm_hw_pm_mdio_write45(pdev,PHY_ADDR_BCAST,PHY_DEVAD_PMD,
                        BCM8485x_PMD_DL_PROC_CTRL,
                        BCM8485x_PMD_DL_PROC_CTRL_WRITE_MODE);
}

static int lm_pm_load_bcm848xx_phy_firmware(pm_device_t pdev)
{
  pm_phy_cfg_t phy_cfg;
	u32 length;
	u32 image_offset;
	u32 i;
	u8 phy_addr;
	u32 val;

  pm_user_get_config(pdev,&phy_cfg);

  /* We only load firmware from port#0. */
  if (phy_cfg.port_idx != 0)
    return PM_ERROR;

  if (lm_pm_get_extphy_fw_info(pdev,&image_offset,&length) != PM_OK)
    return PM_ERROR;

  phy_addr = (phy_cfg.ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_ADDRESS_MASK) >>
    PORT_CFG_PHY_EXTERNAL_PHY_ADDRESS_SHIFT;

  phy_addr &= 0x1fe;
  /* Use based-address to download FW. */
	/* Setup broadcast mode so that we download FW to both ports at the
   * same time.
   */
  lm_hw_pm_mdio_write45(pdev,phy_addr,PHY_DEVAD_USER,0x4117,0xf003);
	lm_hw_pm_mdio_write45(pdev, phy_addr + 1, PHY_DEVAD_USER, 0x4117,
			      0xf003);

  lm_hw_pm_mdio_write45(pdev,phy_addr,PHY_DEVAD_USER,0x4107,0x0401);
	lm_hw_pm_mdio_write45(pdev, phy_addr + 1, PHY_DEVAD_USER, 0x4107,
			      0x0401);

  /* Halt the ARM's. */
  lm_hw_pm_mdio_write45(pdev,phy_addr,PHY_DEVAD_USER,0x4188,0x0040);
	lm_hw_pm_mdio_write45(pdev, phy_addr + 1, PHY_DEVAD_USER, 0x4188,
			      0x0040);
  lm_hw_pm_mdio_write45(pdev,phy_addr,PHY_DEVAD_USER,0x4186,0x8000);
	lm_hw_pm_mdio_write45(pdev, phy_addr + 1, PHY_DEVAD_USER, 0x4186,
			      0x8000);
  lm_hw_pm_mdio_write45(pdev,phy_addr,PHY_DEVAD_USER,0x4181,0x017c);
	lm_hw_pm_mdio_write45(pdev, phy_addr + 1, PHY_DEVAD_USER, 0x4181,
			      0x017c);
  lm_hw_pm_mdio_write45(pdev,phy_addr,PHY_DEVAD_USER,0x4181,0x0040);
	lm_hw_pm_mdio_write45(pdev, phy_addr + 1, PHY_DEVAD_USER, 0x4181,
			      0x0040);

  /* Configure all ARM's in broadcast mode. */
  lm_pm_write_bcm848xx_internal(pdev,0xc3000000,0x00000010);
  lm_pm_write_bcm848xx_internal(pdev,0xffff0000,0xe59f1018);
  lm_pm_write_bcm848xx_internal(pdev,0xffff0004,0xee091f11);
  lm_pm_write_bcm848xx_internal(pdev,0xffff0008,0xe3a00000);
  lm_pm_write_bcm848xx_internal(pdev,0xffff000c,0xe3a01806);
  lm_pm_write_bcm848xx_internal(pdev,0xffff0010,0xe8a00002);
  lm_pm_write_bcm848xx_internal(pdev,0xffff0014,0xe1500001);
  lm_pm_write_bcm848xx_internal(pdev,0xffff0018,0x3afffffc);
  lm_pm_write_bcm848xx_internal(pdev,0xffff001c,0xeafffffe);
  lm_pm_write_bcm848xx_internal(pdev,0xffff0020,0x00040021);

  lm_hw_pm_mdio_write45(pdev,phy_addr,PHY_DEVAD_USER,0x4181,0x0000);
	lm_hw_pm_mdio_write45(pdev, phy_addr + 1, PHY_DEVAD_USER, 0x4181,
			      0x0000);

  /* Write the PHY FW to ARM's internal memory of all cores. */
  lm_hw_pm_mdio_write45(pdev,PHY_ADDR_BCAST,PHY_DEVAD_PMD,
                        BCM8485x_PMD_DL_ADDR_HIGH,0);
  lm_hw_pm_mdio_write45(pdev,PHY_ADDR_BCAST,PHY_DEVAD_PMD,
                        BCM8485x_PMD_DL_ADDR_LOW,0);
	lm_hw_pm_mdio_write45(pdev, PHY_ADDR_BCAST, PHY_DEVAD_PMD,
			      BCM8485x_PMD_DL_PROC_CTRL,
                        BCM8485x_PMD_DL_PROC_CTRL_DL_MODE);

  for (i = 0; i < length; i+=4) {
    val = lm_pm_get_extphy_fw_data(pdev,image_offset+i);
		lm_hw_pm_mdio_write45(pdev, PHY_ADDR_BCAST, PHY_DEVAD_PMD,
				      BCM8485x_PMD_DL_DATA_HIGH,
				      (u16)(val >> 16));
		lm_hw_pm_mdio_write45(pdev, PHY_ADDR_BCAST, PHY_DEVAD_PMD,
				      BCM8485x_PMD_DL_DATA_LOW,
				      (u16)(val & 0xffff));
  }

	lm_hw_pm_mdio_write45(pdev, PHY_ADDR_BCAST, PHY_DEVAD_PMD,
			      BCM8485x_PMD_DL_PROC_CTRL, 0);

  lm_pm_write_bcm848xx_internal(pdev,0xc3000000,0x00000000);

  /* Reset processor for execution */
	lm_hw_pm_mdio_write45(pdev, PHY_ADDR_BCAST, PHY_DEVAD_PMD, 0xa008,
			      0x0000);
  lm_hw_pm_mdio_write45(pdev,phy_addr,PHY_DEVAD_USER,0x8004,0x5555);
	lm_hw_pm_mdio_write45(pdev, phy_addr + 1, PHY_DEVAD_USER, 0x8004,
			      0x5555);

	lm_hw_pm_mdio_write45(pdev, PHY_ADDR_BCAST, PHY_DEVAD_PMD,
			      BCM8485x_PMD_CTRL_1, BCM8485x_PMD_CTRL_1_RESET);

  /* Wait until it's out of reset */
  for (i = 0; i < 3000; i++) {
		if (!
		    (lm_hw_pm_mdio_read45
		     (pdev, phy_addr, PHY_DEVAD_PMD,
		      BCM8485x_PMD_CTRL_1) & BCM8485x_PMD_CTRL_1_RESET))
      break;

    PM_WAIT(pdev,1000);
  }

  return PM_OK;
}
#endif

static int lm_pm_bcm848xx_cfg_link(pm_device_t pdev,pm_phy_cfg_t *phy_cfg)
{
	u16 phy_val[4];
	u16 current_phy_val[4];
	u8 phy_addr;

	phy_addr =
	    (phy_cfg->
	     ext_phy & PORT_CFG_PHY_EXTERNAL_PHY_ADDRESS_MASK) >>
     PORT_CFG_PHY_EXTERNAL_PHY_ADDRESS_SHIFT;

   phy_val[0] = lm_hw_pm_mdio_read45(pdev,phy_addr,PHY_DEVAD_AN,
                                     BCM8485x_AN_10GBASET_AN_CTRL);
   phy_val[1] = lm_hw_pm_mdio_read45(pdev,phy_addr,PHY_DEVAD_AN,
                                     BCM8485x_AN_1000BASET_CTRL);
   phy_val[2] = lm_hw_pm_mdio_read45(pdev,phy_addr,PHY_DEVAD_AN,
                                     BCM8485x_AN_COPPER_AUTONEG_ADV);

   /* Save a copy before making any changes */
   current_phy_val[0] = phy_val[0];
   current_phy_val[1] = phy_val[1];
   current_phy_val[2] = phy_val[2];

   if (phy_cfg->an_enabled) {
      /* Configure 10Gbps advertisement */

     if (phy_cfg->autoneg_cap & PM_PHY_CFG_AUTONEG_CAP_10GBPS)
        phy_val[0] |= BCM8485x_AN_10GBASET_AN_CTRL_10G_ABILITY;
     else
        phy_val[0] &= ~BCM8485x_AN_10GBASET_AN_CTRL_10G_ABILITY;

     /* Configure 1Gbps advertisement */
     if (phy_cfg->autoneg_cap & PM_PHY_CFG_AUTONEG_CAP_1GBPS)
			phy_val[1] |=
			    (BCM8485x_AN_1000BASET_CTRL_1000MBPS_HALF |
                       BCM8485x_AN_1000BASET_CTRL_1000MBPS_FULL);
     else
			phy_val[1] &=
			    ~(BCM8485x_AN_1000BASET_CTRL_1000MBPS_HALF |
                        BCM8485x_AN_1000BASET_CTRL_1000MBPS_FULL);

     if (phy_cfg->autoneg_cap & PM_PHY_CFG_AUTONEG_CAP_100MBPS)
			phy_val[2] |=
			    (BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_FULL |
                       BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_HALF);
     else
			phy_val[2] &=
			    ~(BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_FULL |
                        BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_HALF);

   } else {
       phy_val[0] &= ~BCM8485x_AN_10GBASET_AN_CTRL_10G_ABILITY;
       phy_val[1] &= ~(BCM8485x_AN_1000BASET_CTRL_1000MBPS_HALF |
                       BCM8485x_AN_1000BASET_CTRL_1000MBPS_FULL);
       phy_val[2] &= ~(BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_FULL |
                       BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_HALF);

       /* Use selective autoneg */
       switch (phy_cfg->speed)  {
          case 100:
			phy_val[2] |=
			    (BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_FULL |
                            BCM8485x_AN_COPPER_AUTONEG_ADV_100MBPS_HALF);
             break;

          case 1000:
			phy_val[1] |=
			    (BCM8485x_AN_1000BASET_CTRL_1000MBPS_HALF |
                            BCM8485x_AN_1000BASET_CTRL_1000MBPS_FULL);
             break;

         case 10000:
          phy_val[0] |= BCM8485x_AN_10GBASET_AN_CTRL_10G_ABILITY;
          break;

          default:
             break;
       }
   }

   /* Pause advertisement configuration. */
   switch (phy_cfg->pause & (PM_PHY_CFG_PAUSE_TX | PM_PHY_CFG_PAUSE_RX)) {
      case PM_PHY_CFG_PAUSE_TX:
        /* PAUSE = 0 ; ASYM_DIR = 1 */
        phy_val[2] &= ~BCM8485x_AN_COPPER_AUTONEG_ADV_PAUSE_CAP;
        phy_val[2] |= BCM8485x_AN_COPPER_AUTONEG_ADV_ASYM_PAUSE;
        break;

      case PM_PHY_CFG_PAUSE_RX:
         /* PAUSE = 1 ; ASYM_DIR = 1 */
        phy_val[2] |= BCM8485x_AN_COPPER_AUTONEG_ADV_PAUSE_CAP;
        phy_val[2] |= BCM8485x_AN_COPPER_AUTONEG_ADV_ASYM_PAUSE;
        break;

      case (PM_PHY_CFG_PAUSE_TX|PM_PHY_CFG_PAUSE_RX):
         /* Symmetrical Pause - PAUSE = 1 ; ASYM_DIR = 0 */
        phy_val[2] |= BCM8485x_AN_COPPER_AUTONEG_ADV_PAUSE_CAP;
        phy_val[2] &= ~BCM8485x_AN_COPPER_AUTONEG_ADV_ASYM_PAUSE;
        break;

      default:
         /* PAUSE = 0 ; ASYM_DIR = 0 */
         phy_val[2] &= ~(BCM8485x_AN_COPPER_AUTONEG_ADV_ASYM_PAUSE |
                         BCM8485x_AN_COPPER_AUTONEG_ADV_PAUSE_CAP);
         break;
   }

	lm_hw_pm_mdio_write45(pdev, phy_addr, PHY_DEVAD_AN,
			      BCM8485x_AN_10GBASET_AN_CTRL, phy_val[0]);
	lm_hw_pm_mdio_write45(pdev, phy_addr, PHY_DEVAD_AN,
			      BCM8485x_AN_1000BASET_CTRL, phy_val[1]);
	lm_hw_pm_mdio_write45(pdev, phy_addr, PHY_DEVAD_AN,
			      BCM8485x_AN_COPPER_AUTONEG_ADV, phy_val[2]);

   /* If any changes in configuration, force restart AUTONEG */
   if ((current_phy_val[0] != phy_val[0]) ||
       (current_phy_val[1] != phy_val[1]) ||
       (current_phy_val[2] != phy_val[2])) {
       phy_val[0] = lm_hw_pm_mdio_read45(pdev,phy_addr,PHY_DEVAD_AN,
                                         BCM8485x_AN_1000BASET_MII_CTRL);

       phy_val[0] |= BCM8485x_AN_1000BASET_MII_CTRL_AUTONEG_ENABLE  |
         BCM8485x_AN_1000BASET_MII_CTRL_RESTART_AUTONEG;

       lm_hw_pm_mdio_write45(pdev,phy_addr,PHY_DEVAD_AN,
                             BCM8485x_AN_1000BASET_MII_CTRL,
                             phy_val[0]);
   }

   /* Configure internal loopback if we have to. */
   phy_val[0] = lm_hw_pm_mdio_read45(pdev,phy_addr,PHY_DEVAD_PCS,
                                     BCM8485x_PCS_CTRL_1);

	if (phy_cfg->lpbk == PM_PHY_CFG_LPBK_INT_PHY)
     phy_val[0] |= BCM8485x_PCS_CTRL_1_LOOPBACK;
	else
     phy_val[0] &= ~BCM8485x_PCS_CTRL_1_LOOPBACK;

	lm_hw_pm_mdio_write45(pdev, phy_addr, PHY_DEVAD_PCS,
			      BCM8485x_PCS_CTRL_1, phy_val[0]);

   return PM_OK;
}

static void lm_hw_pm_get_lane_and_polarity(pm_phy_cfg_t *phy_cfg,
					   u16 *tx_lane_swap,
					   u16 *rx_lane_swap,
					   u8 *rx_polarity, u8 *tx_polarity)
{
	u32 lane_map_rx;
	u32 lane_map_tx;
	u32 rx_pol;
	u32 tx_pol;

   lane_map_rx = phy_cfg->lane_swap &
     (SHARED_CFG_PHY_RX_LANE0_SWAP_MASK |
      SHARED_CFG_PHY_RX_LANE1_SWAP_MASK |
      SHARED_CFG_PHY_RX_LANE2_SWAP_MASK |
      SHARED_CFG_PHY_RX_LANE3_SWAP_MASK);

   lane_map_tx = phy_cfg->lane_swap &
     (SHARED_CFG_PHY_TX_LANE0_SWAP_MASK |
      SHARED_CFG_PHY_TX_LANE1_SWAP_MASK |
      SHARED_CFG_PHY_TX_LANE2_SWAP_MASK |
      SHARED_CFG_PHY_TX_LANE3_SWAP_MASK);

	rx_pol =
	    phy_cfg->
	    lane_polarity & (SHARED_CFG_PHY_RX_LANE0_POL_FLIP_MASK |
                                      SHARED_CFG_PHY_RX_LANE1_POL_FLIP_MASK |
                                      SHARED_CFG_PHY_RX_LANE2_POL_FLIP_MASK |
                                      SHARED_CFG_PHY_RX_LANE3_POL_FLIP_MASK);

	tx_pol =
	    phy_cfg->
	    lane_polarity & (SHARED_CFG_PHY_TX_LANE0_POL_FLIP_MASK |
                                      SHARED_CFG_PHY_TX_LANE1_POL_FLIP_MASK |
                                      SHARED_CFG_PHY_TX_LANE2_POL_FLIP_MASK |
                                      SHARED_CFG_PHY_TX_LANE3_POL_FLIP_MASK);

   if (phy_cfg->board_cfg & SHARED_CFG_BOARD_PORT_SWAP_ENABLED) {
		switch (phy_cfg->board_cfg &
			SHARED_CFG_BOARD_PORT_LAYOUT_MASK) {
         case SHARED_CFG_BOARD_PORT_LAYOUT_2P_10:
            lane_map_rx =
              (lane_map_rx & SHARED_CFG_PHY_RX_LANE3_SWAP_MASK) |
              (lane_map_rx & SHARED_CFG_PHY_RX_LANE2_SWAP_MASK) |
			    ((lane_map_rx & SHARED_CFG_PHY_RX_LANE1_SWAP_MASK)
			     >> 4) | ((lane_map_rx &
				       SHARED_CFG_PHY_RX_LANE0_SWAP_MASK) << 4);

            lane_map_tx =
              (lane_map_tx & SHARED_CFG_PHY_TX_LANE3_SWAP_MASK) |
              (lane_map_tx & SHARED_CFG_PHY_TX_LANE2_SWAP_MASK) |
			    ((lane_map_tx & SHARED_CFG_PHY_TX_LANE1_SWAP_MASK)
			     >> 4) | ((lane_map_tx &
				       SHARED_CFG_PHY_TX_LANE0_SWAP_MASK) << 4);

            rx_pol =
              (rx_pol & SHARED_CFG_PHY_RX_LANE3_POL_FLIP_MASK) |
              (rx_pol & SHARED_CFG_PHY_RX_LANE2_POL_FLIP_MASK) |
			    ((rx_pol & SHARED_CFG_PHY_RX_LANE1_POL_FLIP_MASK) >>
			     1) | ((rx_pol &
				    SHARED_CFG_PHY_RX_LANE0_POL_FLIP_MASK) <<
				   1);
            tx_pol =
              (tx_pol & SHARED_CFG_PHY_TX_LANE3_POL_FLIP_MASK) |
              (tx_pol & SHARED_CFG_PHY_TX_LANE2_POL_FLIP_MASK) |
			    ((tx_pol & SHARED_CFG_PHY_TX_LANE1_POL_FLIP_MASK) >>
			     1) | ((tx_pol &
				    SHARED_CFG_PHY_TX_LANE0_POL_FLIP_MASK) <<
				   1);
            break;

         case SHARED_CFG_BOARD_PORT_LAYOUT_4P_1032:
            lane_map_rx =
			    ((lane_map_rx & SHARED_CFG_PHY_RX_LANE3_SWAP_MASK)
			     >> 4) | ((lane_map_rx &
				       SHARED_CFG_PHY_RX_LANE2_SWAP_MASK) << 4)
			    | ((lane_map_rx & SHARED_CFG_PHY_RX_LANE1_SWAP_MASK)
			       >> 4) | ((lane_map_rx &
					 SHARED_CFG_PHY_RX_LANE0_SWAP_MASK) <<
					4);

            lane_map_tx =
			    ((lane_map_tx & SHARED_CFG_PHY_TX_LANE3_SWAP_MASK)
			     >> 4) | ((lane_map_tx &
				       SHARED_CFG_PHY_TX_LANE2_SWAP_MASK) << 4)
			    | ((lane_map_tx & SHARED_CFG_PHY_TX_LANE1_SWAP_MASK)
			       >> 4) | ((lane_map_tx &
					 SHARED_CFG_PHY_TX_LANE0_SWAP_MASK) <<
					4);

            rx_pol =
			    ((rx_pol & SHARED_CFG_PHY_RX_LANE3_POL_FLIP_MASK) >>
			     1) | ((rx_pol &
				    SHARED_CFG_PHY_RX_LANE2_POL_FLIP_MASK) << 1)
			    | ((rx_pol & SHARED_CFG_PHY_RX_LANE1_POL_FLIP_MASK)
			       >> 1) | ((rx_pol &
					 SHARED_CFG_PHY_RX_LANE0_POL_FLIP_MASK)
					<< 1);
            tx_pol =
			    ((tx_pol & SHARED_CFG_PHY_TX_LANE3_POL_FLIP_MASK) >>
			     1) | ((tx_pol &
				    SHARED_CFG_PHY_TX_LANE2_POL_FLIP_MASK) << 1)
			    | ((tx_pol & SHARED_CFG_PHY_TX_LANE1_POL_FLIP_MASK)
			       >> 1) | ((tx_pol &
					 SHARED_CFG_PHY_TX_LANE0_POL_FLIP_MASK)
					<< 1);
            break;

         case SHARED_CFG_BOARD_PORT_LAYOUT_4P_2301:
             /*  0 1 2 3 --> 2 3 0 1 */
            lane_map_rx =
			    ((lane_map_rx & SHARED_CFG_PHY_RX_LANE3_SWAP_MASK)
			     >> 8) | ((lane_map_rx &
				       SHARED_CFG_PHY_RX_LANE2_SWAP_MASK) >> 8)
			    | ((lane_map_rx & SHARED_CFG_PHY_RX_LANE1_SWAP_MASK)
			       << 8) | ((lane_map_rx &
					 SHARED_CFG_PHY_RX_LANE0_SWAP_MASK) <<
					8);

            lane_map_tx =
			    ((lane_map_tx & SHARED_CFG_PHY_TX_LANE3_SWAP_MASK)
			     >> 8) | ((lane_map_tx &
				       SHARED_CFG_PHY_TX_LANE2_SWAP_MASK) >> 8)
			    | ((lane_map_tx & SHARED_CFG_PHY_TX_LANE1_SWAP_MASK)
			       << 8) | ((lane_map_tx &
					 SHARED_CFG_PHY_TX_LANE0_SWAP_MASK) <<
					8);

            rx_pol =
			    ((rx_pol & SHARED_CFG_PHY_RX_LANE3_POL_FLIP_MASK) >>
			     2) | ((rx_pol &
				    SHARED_CFG_PHY_RX_LANE2_POL_FLIP_MASK) >> 2)
			    | ((rx_pol & SHARED_CFG_PHY_RX_LANE1_POL_FLIP_MASK)
			       << 2) | ((rx_pol &
					 SHARED_CFG_PHY_RX_LANE0_POL_FLIP_MASK)
					<< 2);
            tx_pol =
			    ((tx_pol & SHARED_CFG_PHY_TX_LANE3_POL_FLIP_MASK) >>
			     2) | ((tx_pol &
				    SHARED_CFG_PHY_TX_LANE2_POL_FLIP_MASK) >> 2)
			    | ((tx_pol & SHARED_CFG_PHY_TX_LANE1_POL_FLIP_MASK)
			       << 2) | ((tx_pol &
					 SHARED_CFG_PHY_TX_LANE0_POL_FLIP_MASK)
					<< 2);
            break;

         case SHARED_CFG_BOARD_PORT_LAYOUT_4P_3210:
           /*  0 1 2 3 --> 3 2 1 0 */
            lane_map_rx =
			    ((lane_map_rx & SHARED_CFG_PHY_RX_LANE3_SWAP_MASK)
			     >> 12) | ((lane_map_rx &
					SHARED_CFG_PHY_RX_LANE2_SWAP_MASK) >> 4)
			    | ((lane_map_rx & SHARED_CFG_PHY_RX_LANE1_SWAP_MASK)
			       << 4) | ((lane_map_rx &
					 SHARED_CFG_PHY_RX_LANE0_SWAP_MASK) <<
					12);

            lane_map_tx =
			    ((lane_map_tx & SHARED_CFG_PHY_TX_LANE3_SWAP_MASK)
			     >> 12) | ((lane_map_tx &
					SHARED_CFG_PHY_TX_LANE2_SWAP_MASK) >> 4)
			    | ((lane_map_tx & SHARED_CFG_PHY_TX_LANE1_SWAP_MASK)
			       << 4) | ((lane_map_tx &
					 SHARED_CFG_PHY_TX_LANE0_SWAP_MASK) <<
					12);

            rx_pol =
			    ((rx_pol & SHARED_CFG_PHY_RX_LANE3_POL_FLIP_MASK) >>
			     3) | ((rx_pol &
				    SHARED_CFG_PHY_RX_LANE2_POL_FLIP_MASK) >> 1)
			    | ((rx_pol & SHARED_CFG_PHY_RX_LANE1_POL_FLIP_MASK)
			       << 1) | ((rx_pol &
					 SHARED_CFG_PHY_RX_LANE0_POL_FLIP_MASK)
					<< 3);
            tx_pol =
			    ((tx_pol & SHARED_CFG_PHY_TX_LANE3_POL_FLIP_MASK) >>
			     3) | ((tx_pol &
				    SHARED_CFG_PHY_TX_LANE2_POL_FLIP_MASK) >> 1)
			    | ((tx_pol & SHARED_CFG_PHY_TX_LANE1_POL_FLIP_MASK)
			       << 1) | ((tx_pol &
					 SHARED_CFG_PHY_TX_LANE0_POL_FLIP_MASK)
					<< 3);
            break;

         default:
            break;
      }
   }

	if (tx_lane_swap)
		*tx_lane_swap =
		    (u16)(lane_map_tx >> SHARED_CFG_PHY_TX_LANE0_SWAP_SHIFT);

	if (rx_lane_swap)
		*rx_lane_swap = (u16)lane_map_rx;

	if (rx_polarity)
		*rx_polarity = (u8)rx_pol;

	if (tx_polarity)
		*tx_polarity =
		    (u8)(tx_pol >> SHARED_CFG_PHY_TX_LANE0_POL_FLIP_SHIFT);
}

#ifndef CHIMP_VIEW
#if (defined(DOS) && !defined(UEFI64)) || defined(LINUX)
u8 *krnl_logger_buf;
#endif
u8 *usr_logger_buf;
lm_status_t lm_hw_pm_get_eye_scan(pm_device_t pdev, u8 *outstr_p)
{
   phymod_access_t *pm_acc;
   pmm_core_t   *core;
   pmm_config_t *pmm_cfg;       
   phymod_core_access_t *pm_core;
	phymod_bus_t *pmm_bus;	/* do we need this? */
   pm_phy_cfg_t phy_cfg;
	lm_status_t rc = 0;
   
   pm_user_get_config(pdev,&phy_cfg);
   pmm_cfg = GET_PMM_CFG(pdev);   
   pmm_bus = GET_PMM_BUS_BLK(pdev);

   core = &pmm_cfg->pmm_core;
   pm_core = &core->pm_core;
   pm_acc = &pm_core->access;
   
#if (defined(DOS) && !defined(UEFI64))
   usr_logger_buf = outstr_p;
#elif defined(LINUX)
   krnl_logger_buf = outstr_p;
#endif    

#ifdef UNCOMMENT
   rc = falcon_tsc_display_eye_scan (pm_acc);
#endif
   return rc;
}
#endif

void lm_hw_pm_minimal_default_xlmac_cfg(pm_device_t pdev, u8 ten_gig_en)
{
	u32 mac_control;
	u32 mode, mode1;
	u32 enable_ports = 0;
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)pdev;
	u8 port = privp->amac_id;

	enable_ports = enable_ports | (0x1 << XLPORT_ENABLE_REG__PORT3) | (0x1
	      << XLPORT_ENABLE_REG__PORT2) | (0x1 << XLPORT_ENABLE_REG__PORT1)
	      | (0x1 << XLPORT_ENABLE_REG__PORT0);
/* step 1 enable all XLPORT */
	lm_hw_pm_wr_reg(pdev, XLPORT_ENABLE_REG, 0, enable_ports, 1, 0);
/* program port xgxs0 ctrl */
/* program xlport config reg */

	mac_control =
	    0 << XLPORT_MAC_CONTROL__XMAC0_BYPASS_OSTS | 0 <<
	    XLPORT_MAC_CONTROL__XMAC0_RESET;
	lm_hw_pm_wr_reg(pdev, XLPORT_MAC_CONTROL, 0, mac_control, 1, 0);

	lm_hw_pm_wr_reg(pdev, XLPORT_MAC_RSV_MASK, 0, 0x58, 0, port);

/*applying soft reset */
	lm_hw_pm_wr_reg(pdev, XLMAC_CTRL, 0, 1 << XLMAC_CTRL__SOFT_RESET, 0, port);
	lm_hw_pm_wr_reg(pdev, XLMAC_TX_CRC_CORRUPT_CTRL, 0, 0, 0, port);

	if (ten_gig_en == 0) {
/*speed mode-1G */
		mode =
		    (HDR_MODE_IEEE | SPEED_MODE_LINK_1G <<
		     XLMAC_MODE__SPEED_MODE_R);
		lm_hw_pm_wr_reg(pdev, XLMAC_MODE, 0, mode, 0, port);

	} else if (ten_gig_en == 1) {
/*speed_mode 10G */
		mode =
		    (HDR_MODE_IEEE | SPEED_MODE_LINK_10G_PLUS <<
		     XLMAC_MODE__SPEED_MODE_R);
		lm_hw_pm_wr_reg(pdev, XLMAC_MODE, 0, mode, 0, port);
	} else if (ten_gig_en == 2) {
/*speed_mode 10G */
		mode =
		    (HDR_MODE_IEEE | SPEED_MODE_LINK_2G5 <<
		     XLMAC_MODE__SPEED_MODE_R);
		lm_hw_pm_wr_reg(pdev, XLMAC_MODE, 0, mode, 0, port);
	}
	pr_emerg("set speed mode\n");

/* releasing soft reset */
	lm_hw_pm_wr_reg(pdev, XLMAC_CTRL, 0, 0 << XLMAC_CTRL__SOFT_RESET, 0, port);
	pr_emerg("released soft reset\n");

/* rx ctrl reg */
	if (ten_gig_en == 1) {
		lm_hw_pm_wr_reg(pdev, XLMAC_RX_CTRL, 0,
				0 << XLMAC_RX_CTRL__STRICT_PREAMBLE, 0, port);
	}

	mode = (1 << XLMAC_CTRL__TX_EN | 1 << XLMAC_CTRL__RX_EN
		| 0 << XLMAC_CTRL__SOFT_RESET);
#ifdef MAC_LOOPBACK
	mode |= (1 << XLMAC_CTRL__LOCAL_LPBK);
#endif
	lm_hw_pm_wr_reg(pdev, XLMAC_CTRL, 0, mode, 0, port);

	pr_emerg("tx and rx en done\n");

	mode = 0x1; /* parallel fc en */

	lm_hw_pm_wr_reg(pdev, XLPORT_FLOW_CONTROL_CONFIG, 0, mode, 0, port);

	mode = 0xFFE3FFFF;
	mode1 = 0x1F;
	lm_hw_pm_wr_reg(pdev, XLMAC_PAUSE_CTRL, mode1, mode, 0, port);
}

void dump_pm_regs(pm_device_t pdev)
{
   int j;
   u32 val, val1;
   for(j=0x600; j<=0x610; j++)
   {
	lm_hw_pm_rd_reg(pdev, j, &val1, &val, 0, 0);
	pr_emerg("PM Reg. #%X: Val:%#X\n", j, val);
   }
   lm_hw_pm_tsc_rd(pdev, TSC_CORE_ADDR, 0x900e, 1, 0, &val);
   pr_emerg("------->TSC Reg. #%X: Val:%#X<--------\n", 0x900e, val);
   lm_hw_pm_tsc_rd(pdev, TSC_CORE_ADDR, 0x800d0f0, 1, 1, &val);
   pr_emerg("------->PMD/Eagle Reg. #%X: Val:%#X<--------\n", 0xd0f0, val);

}

/*PRBS mode seelction:
# 3'd0 -> PRBS 7| 3'd1 -> PRBS 9|c3'd2 -> PRBS 11| 3'd3 -> PRBS 15| 3'd4 -> PRBS 23| 3'd5 -> PRBS 31| 3'd6 -> PRBS 58
*/
#ifdef EXTRA_CODE
MDIO_MMDSEL_AER_COM_MDIO_AER=0xffde
TLB_TX_PRBS_GEN_CONFIG=0xd0e1
TLB_RX_PRBS_CHK_CONFIG=0xd0d1
TLB_RX_PRBS_CHK_ERR_CNT_MSB_STATUS=0xd0da
TLB_RX_PRBS_CHK_ERR_CNT_LSB_STATUS=0xd0db
TLB_RX_PRBS_CHK_LOCK_STATUS=0xd0d9
#endif

/*prepend MSB with 0x800 to use legacy WR function*/
#define MDIO_MMDSEL_AER_COM_MDIO_AER 0x800ffde
#define TLB_TX_PRBS_GEN_CONFIG 0x800d0e1
#define TLB_RX_PRBS_CHK_CONFIG 0x800d0d1
#define TLB_RX_PRBS_CHK_ERR_CNT_MSB_STATUS 0x800d0da
#define TLB_RX_PRBS_CHK_ERR_CNT_LSB_STATUS 0x800d0db
#define TLB_RX_PRBS_CHK_LOCK_STATUS 0x800d0d9

u32 prbs_mode = 0x5;

bool prbs_test_per_lane(void *pdev, int lane)
{
	u32 m, l, x;
	u8 prbsmode;
	prbsmode = prbs_mode * 2;
	lm_hw_pm_tsc_wr_ln(pdev, TSC_CORE_ADDR, TLB_TX_PRBS_GEN_CONFIG, lane, 1, prbsmode);
	lm_hw_pm_tsc_wr_ln(pdev, TSC_CORE_ADDR, TLB_RX_PRBS_CHK_CONFIG, lane, 1, prbsmode);

	/*Enable generater & checker by setting 0th bit/adding 1*/
	prbsmode |= 0x1;

	lm_hw_pm_tsc_wr_ln(pdev, TSC_CORE_ADDR, TLB_TX_PRBS_GEN_CONFIG, lane, 1, prbsmode);
	lm_hw_pm_tsc_wr_ln(pdev, TSC_CORE_ADDR, TLB_RX_PRBS_CHK_CONFIG, lane, 1, prbsmode);

	/*1st Read to discard/dump error count reg.:*/
	lm_hw_pm_tsc_rd_ln(pdev, TSC_CORE_ADDR, TLB_RX_PRBS_CHK_ERR_CNT_MSB_STATUS, lane, 1, &m);
	lm_hw_pm_tsc_rd_ln(pdev, TSC_CORE_ADDR, TLB_RX_PRBS_CHK_ERR_CNT_LSB_STATUS, lane, 1, &l);
	msleep(1000);
	/*2nd Read for actual values*/
	lm_hw_pm_tsc_rd_ln(pdev, TSC_CORE_ADDR, TLB_RX_PRBS_CHK_ERR_CNT_MSB_STATUS, lane, 1, &m);
	lm_hw_pm_tsc_rd_ln(pdev, TSC_CORE_ADDR, TLB_RX_PRBS_CHK_ERR_CNT_LSB_STATUS, lane, 1, &l);

	m &= 0xFFFF;
	l &= 0xFFFF;

	lm_hw_pm_tsc_rd_ln(pdev, TSC_CORE_ADDR, TLB_RX_PRBS_CHK_LOCK_STATUS, lane, 1, &x);
	x &= 0x1;

#ifdef PM_PHYMOD_DEBUG
	pr_emerg("x=%d, l=%d, m=%d lane:%d\n",  x, l, m, lane);
#endif

	if (x == 1 && m == 0 && l == 0) {
		pr_emerg("PRBS test passed for lane:%d\n", lane);
		return true;
	} else {
		pr_emerg("PRBS test failed for lane:%d\n", lane);
		return false;
	}
}
EXPORT_SYMBOL(prbs_test_per_lane);
EXPORT_SYMBOL(dump_pm_regs);
