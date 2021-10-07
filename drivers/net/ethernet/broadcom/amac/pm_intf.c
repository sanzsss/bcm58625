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


#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/printk.h>
#include <linux/delay.h>
#ifndef CHIMP_VIEW
#include "include/pm_intf.h"
#else
#include "include/lm_hw_pm.h"
#endif

//#include "bcm-amac-dbg.h"
/* Local function protocol */


static u16 lm_hw_pm_wr_mdio_cmd(pm_device_t pdev, u32 cmd, int clause45);


pa_reg_t pa;
void *apb2pbus_base;
void *pm_gphy_cfg_status_base;
static u8 stage_id;

void lm_hw_pm_apb2pbus_brdg_init(pm_device_t pdev)
{
#ifdef PM_PHYMOD_DEBUG
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)pdev;

	pr_emerg("apb2pbus_base:%p, privp->hw.reg.apb2pbus_base=%p,",
		 apb2pbus_base, privp->hw.reg.apb2pbus_base);
	pr_emerg("pm_gphy_cfg_status_base:%p\n", pm_gphy_cfg_status_base);
#endif
	pm_gphy_cfg_status_base =
	    ioremap_nocache(PORTMACRO_GPHY_CONFIG_STATUS_BASE, 0x14);

	writel(0xa23,
	       pm_gphy_cfg_status_base + ETH_SS_0_PORT_MACRO_EEE_CONFIG_OFFSET);

	PBUS_IF_WADDR = apb2pbus_base + APB2PBUS_WADDR_OFFSET;
	PBUS_IF_RADDR = apb2pbus_base + APB2PBUS_RADDR_OFFSET;
	PBUS_IF_WCTRL = apb2pbus_base + APB2PBUS_WCTRL_OFFSETSET;
	PBUS_IF_RCTRL = apb2pbus_base + APB2PBUS_RCTRL_OFFSET;
	PBUS_IF_WR_DATA = apb2pbus_base + APB2PBUS_WDATA0_OFFSET;
	PBUS_IF_RD_DATA = apb2pbus_base + APB2PBUS_RDATA0_OFFSET;
	PBUS_IF_WR_GO = apb2pbus_base + APB2PBUS_WR_GO_OFFSET;
	PBUS_IF_RD_GO = apb2pbus_base + APB2PBUS_RD_GO_OFFSET;
	PBUS_IF_STATUS = apb2pbus_base + APB2PBUS_RD_WR_STATUS_OFFSET;
	PBUS_IF_STATUS_MASK = apb2pbus_base + APB2PBUS_RD_WR_STATUS_MASK_OFFSET;
	PBUS_IF_DEBUG_CNTL = apb2pbus_base + APB2PBUS_CTRL_OFFSET;

	/* Disable APB2PBUS i/f transaction complete/error interrupts */
	/* REG_WRITE(pdev, PBUS_IF_STATUS_MASK, 0); */
}

/* wr=0 for RD  & wr = 1 for WR */
static int lm_hw_pm_start_access(pm_device_t pdev, bool wr)
{
	u32 i;

#ifdef UNCOMMENT
  /* Make sure we get grant first */
  REG_WRITE(pdev,PBUS_IF_ARB,PM_SW_ARB_REQ);

  for (i=0; i < PBUS_TIMEOUT;i++) {
    if ((REG_READ(pdev,PBUS_IF_ARB) & PM_SW_ARB_GRANT))
      return PM_OK;

     /* Wait for 1uS */
     PM_WAIT(pdev,1);
  }

  /* Bail out if we cannot get grant */
  if (i == PBUS_TIMEOUT)
    return PM_ERROR;
#endif

  /* Wait until the transaction is not busy. */
  for (i=0; i < PBUS_TIMEOUT;i++) {
		if (wr) {
			if (!(REG_READ(pdev, PBUS_IF_WR_GO) & PA_PM_IF_GO))
				break;
		} else {
			if (!(REG_READ(pdev, PBUS_IF_RD_GO) & PA_PM_IF_GO))
      break;
		}

     /* Wait for 1uS */
     PM_WAIT(pdev,1);
  }

	if (i == PBUS_TIMEOUT)
		return PM_ERROR;

  return PM_OK;
}

/* wr=0 for RD  & wr = 1 for WR */
static int lm_hw_pm_end_access(pm_device_t pdev, bool wr)
{
	u32 i;
	int status = PM_ERROR;

  /* Wait until the transaction is complete. */
  for (i=0; i < PBUS_TIMEOUT;i++) {
		if (wr) {
			if (REG_READ(pdev, PBUS_IF_STATUS) & PA_PM_IF_WR_DONE) {
				status = PM_OK;
      break;
			}
		} else {
			if (REG_READ(pdev, PBUS_IF_STATUS) & PA_PM_IF_RD_DONE) {
				status = PM_OK;
				break;
			}
		}

     /* Wait for 1uS */
     PM_WAIT(pdev,1);
  }

	if (REG_READ(pdev, PBUS_IF_STATUS) &
	    (PA_PM_IF_RD_ERROR | PA_PM_IF_WR_ERROR))
		status = PM_ERROR;

  /* Finally release arbitration. */
	/* REG_WRITE(pdev, PBUS_IF_ARB,PM_SW_ARB_CLR); */
	return status;
}

int lm_hw_pm_wr_reg(pm_device_t pdev,
		    u32 addr, u32 data_hi, u32 data_low, u8 reg_type, u8 port)
{
	u32 val;
	int status;
#ifdef PM_PHYMOD_DEBUG
	u32 msb, lsb;
#endif

	if (lm_hw_pm_start_access(pdev, 1) != PM_OK) {
		pr_err("PM_wait for WR failed\n");
      return PM_ERROR;
   }
	/*DbgMessage5(pdev,VERBOSE,"lm_hw_pm_wr_reg():addr=0x%x data=0x%x/%x,
	 *reg_type=%x, port=%x\n",
	 *addr,
	 *data_hi,
	 *data_low,
	 *reg_type,
	 *port);
	 */

	REG_WRITE(pdev, PBUS_IF_WCTRL,
		  (8 << PA_PM_IF_BYTE_COUNT_SFT) |
		  (PA_PM_IF_TYPE_REGISTER_ACCESS << PA_PM_IF_TYPE_ACCESS_SFT));

	val = ((u32)port << PM_IF_ADDR_PORT_NUM_SFT) |
     (addr << PM_IF_ADDR_REG_OFFSET_SFT);

  if (reg_type==ACCESS_GENERIC)
    val |= PM_IF_ADDR_REG_TYPE_GEN_REG;

	val |= (stage_id << PM_IF_ADDR_MEM_STAGE_ID_SFT);

	REG_WRITE(pdev, PBUS_IF_WADDR, val);
  REG_WRITE(pdev,PBUS_IF_WR_DATA,data_low);
	REG_WRITE(pdev, PBUS_IF_WR_DATA + 4, data_hi);

	REG_WRITE(pdev, PBUS_IF_WR_GO, PA_PM_IF_GO);

  /* Release arbitration. */
	status = lm_hw_pm_end_access(pdev, 1);

#ifdef PM_PHYMOD_DEBUG
	lm_hw_pm_rd_reg(pdev, addr, &msb, &lsb, reg_type, port);
	/* Rd & Wr values may differ as some fields of register may be
	 * WR only & some may be RD only
	 */
	pr_emerg
	    ("[Verify:]addr:%#X | Wr| MSB:%#X, LSB:%#X, Rd| MSB:%#X, LSB:%#X\n",
	     addr, data_hi, data_low, msb, lsb);
#endif

	return status;
}

int lm_hw_pm_rd_reg(pm_device_t pdev,
		    u32 addr, u32 *data_msb,
		    u32 *data_lsb, u8 reg_type, u8 port)
{
	u32 val;
	int status;

	if (lm_hw_pm_start_access(pdev, 0) != PM_OK) {
		pr_err("PM_wait for WR failed\n");
      return PM_ERROR;
   }

	REG_WRITE(pdev, PBUS_IF_RCTRL,
		  (8 << PA_PM_IF_BYTE_COUNT_SFT) |
		  (PA_PM_IF_TYPE_REGISTER_ACCESS << PA_PM_IF_TYPE_ACCESS_SFT));
	val = ((u32)port << PM_IF_ADDR_PORT_NUM_SFT) | (addr <<
		PM_IF_ADDR_REG_OFFSET_SFT);

  if (reg_type==ACCESS_GENERIC)
      val |= PM_IF_ADDR_REG_TYPE_GEN_REG;

	val |= (stage_id << PM_IF_ADDR_MEM_STAGE_ID_SFT);
	REG_WRITE(pdev, PBUS_IF_RADDR, val);


	/*DbgMessage5(pdev,VERBOSE,"rd():addr=0x%x data=0x%x/%x,reg_type=%x,
	 *port=%x\n",
	 *addr,
	 **data_msb,
	 *          *data_lsb,
	 *reg_type,
	 *port);
	 */

	REG_WRITE(pdev, PBUS_IF_RD_GO, PA_PM_IF_GO);
   
	status = lm_hw_pm_end_access(pdev, 0);

	*data_lsb = REG_READ(pdev, PBUS_IF_RD_DATA);
	*data_msb = REG_READ(pdev, PBUS_IF_RD_DATA + 4);
	return status;
}

int lm_hw_pm_tsc_rd_mem(pm_device_t pdev, u32 addr, bool reg_or_mem, u32 *i_data, u32 *o_data)
{
	int i, status;
   
	if(reg_or_mem) /* read from TSC-E memory is not supported */
		return -1;

	lm_hw_pm_wr_reg(pdev, XLPORT_WC_UCMEM_CTRL, 0, reg_or_mem, ACCESS_GENERIC, 0);


	if (lm_hw_pm_start_access(pdev, 1) != PM_OK) {
		pr_err("PM_wait for WR failed\n");
		return PM_ERROR;
   }
	/* We write 128-bit at a time as memory access. */
	REG_WRITE(pdev, PBUS_IF_WCTRL,
		  (16 << PA_PM_IF_BYTE_COUNT_SFT) | (PA_PM_IF_TYPE_MEMORY_ACCESS
		   << PA_PM_IF_TYPE_ACCESS_SFT));

	REG_WRITE(pdev, PBUS_IF_WADDR, addr);

	for (i = 0; i < 4; i++)
		REG_WRITE(pdev, PBUS_IF_WR_DATA + (i * 4), i_data[i]);
	REG_WRITE(pdev, PBUS_IF_WR_GO, PA_PM_IF_GO);
	lm_hw_pm_end_access(pdev, 1);

	if (lm_hw_pm_start_access(pdev, 0) != PM_OK) {
		pr_err("PM_wait for RD failed\n");
		return PM_ERROR;
   }
	REG_WRITE(pdev, PBUS_IF_RADDR, addr);
	REG_WRITE(pdev, PBUS_IF_RCTRL,
		  (16 << PA_PM_IF_BYTE_COUNT_SFT) | (PA_PM_IF_TYPE_MEMORY_ACCESS
		   << PA_PM_IF_TYPE_ACCESS_SFT));

	REG_WRITE(pdev, PBUS_IF_RD_GO, PA_PM_IF_GO);

	status = lm_hw_pm_end_access(pdev, 0);
	for (i = 0; i < 4; i++)
		o_data[i] = REG_READ(pdev, PBUS_IF_RD_DATA + (i * 4));

	return status;
}

int lm_hw_pm_tsc_wr_mem(pm_device_t pdev, u32 addr, bool reg_or_mem, u32 *data)
{
   int i;

#ifdef UNCOMMENT
/*FIXME: not every time needed for ucode loaing.
 * It's done outside loop which calls this
 */
	lm_hw_pm_wr_reg(pdev, XLPORT_WC_UCMEM_CTRL, 0, reg_or_mem, ACCESS_GENERIC, 0);
#endif

	if (lm_hw_pm_start_access(pdev, 1) != PM_OK) {
		pr_err("PM_wait failed\n");
      return PM_ERROR;
   }

	REG_WRITE(pdev, PBUS_IF_WCTRL,
		  (16 << PA_PM_IF_BYTE_COUNT_SFT) | (PA_PM_IF_TYPE_MEMORY_ACCESS
		   << PA_PM_IF_TYPE_ACCESS_SFT));

	REG_WRITE(pdev, PBUS_IF_WADDR, addr);

	for (i = 0; i < 4; i++)
		REG_WRITE(pdev, PBUS_IF_WR_DATA + (i * 4), data[i]);

	REG_WRITE(pdev, PBUS_IF_WR_GO, PA_PM_IF_GO);

	return lm_hw_pm_end_access(pdev, 1);
}

int lm_hw_pm_tsc_rd_ln(void *pdev, u32 core_addr, u32 reg_addr, u8 lane, bool tsc_or_eagle, u32 *val)
{
	u32 data_in[4];
	u32 data_out[4];
   int rc;

	/* Bit 64 : 1 - memory write ; 0 - memory read
    * Bit 63..48 : 16-bit write data. 
    * Bit 47..32 : 16-bit write data mask.
    * Bit 31..0  : 32-bit TSC register address.
    */

	/* lane = 0x6 for broast cast to all 4-lanes */
	data_in[0] = UCMEM_ADDR_LANE(core_addr, tsc_or_eagle, reg_addr, lane, 0);
   data_in[1] = 0;
   data_in[2] = 0; /* Read operation */
   data_in[3] = 0;

	rc = lm_hw_pm_tsc_rd_mem((void *)pdev,
				 XLPORT_WC_UCMEM_DATA, 0, data_in, data_out);
   *val = (data_out[1] & 0xffff);
#ifdef PM_PHYMOD_DEBUG
	pr_emerg("%s():UCMEM_ADDR:%#X|Val = %#X\n", __func__, data_in[0], *val);
#endif

	/* DbgMessage3(NULL,VERBOSE,"lm_hw_pm_tsc_rd():core_addr=%x,
	 * reg_addr=0x%x,
	 * data=0x%x\n", core_addr,reg_addr,data_out[1]);
	 */
   
   return rc;
}

int lm_hw_pm_tsc_rd(void *pdev, u32 core_addr, u32 reg_addr, u8 lane, bool tsc_or_eagle, u32 *val)
{
	u32 data_in[4];
	u32 data_out[4];
   int rc;

	/* Bit 64 : 1 - memory write ; 0 - memory read
    * Bit 63..48 : 16-bit write data. 
    * Bit 47..32 : 16-bit write data mask.
    * Bit 31..0  : 32-bit TSC register address.
    */

	/* lane = 0x6 for broast cast to all 4-lanes */
   data_in[0] = UCMEM_ADDR(core_addr,0,reg_addr,0);
   data_in[1] = 0;
   data_in[2] = 0; /* Read operation */
   data_in[3] = 0;

	rc = lm_hw_pm_tsc_rd_mem((void *)pdev,
				 XLPORT_WC_UCMEM_DATA, 0, data_in, data_out);
   *val = (data_out[1] & 0xffff);
#ifdef PM_PHYMOD_DEBUG
	pr_emerg("%s():UCMEM_ADDR:%#X|Val = %#X\n", __func__, data_in[0], *val);
#endif
	mdelay(1); /*FIXME: Calibrated value: PRBS fails for less than 1mS */

	/* DbgMessage3(NULL,VERBOSE,"lm_hw_pm_tsc_rd():core_addr=%x,
	 * reg_addr=0x%x,
	 * data=0x%x\n", core_addr,reg_addr,data_out[1]);
	 */
   
   return rc;
}

#ifdef UNCOMMENT
int lm_hw_pm_tsc_wr_old(void *pdev, u32 core_addr, u32 reg_addr, u8 lane, bool tsc_or_eagle, u32 val)
{
   int rc;
	u16 data16 = (u16)val;
	u16 mask = (u16)(val >> 16);
	u32 rd_data;
	u32 data[4];


	/* reg_or_mem = 0 for register access */
	lm_hw_pm_wr_reg(pdev, XLPORT_WC_UCMEM_CTRL, 0, 0, ACCESS_GENERIC, 0);

	/* DbgMessage3(NULL,VERBOSE,"lm_hw_pm_tsc_wr():core_addr=%x,
	 * reg_addr=0x%x, val=0x%x\n", core_addr,reg_addr,val);
	 */

	/* Bit 64 : 1 - memory write ; 0 - memory read
	 * Bit 63..48 : 16-bit write data.
	 * Bit 47..32 : 16-bit write data mask.
	 * Bit 31..0  : 32-bit TSC register address.
	 */
	/* lane = 0x6 for broast cast to all 4-lanes */
	data[0] = UCMEM_ADDR(core_addr, tsc_or_eagle, reg_addr, lane, 0);
	data[1] = 0;
	data[2] = 1;		/* Write operation */
	data[3] = 0;

	if (mask) {
		/* Mask is valid */
		lm_hw_pm_tsc_rd(pdev, core_addr, reg_addr, lane, tsc_or_eagle, &rd_data);
		data16 = (u16)(rd_data & ~mask);
		data16 |= (val & mask);
	}

	/* data[31:16]: data value, data[0:15]: data mask */
	data[1] = UCMEM_DATA(data16);

	rc = lm_hw_pm_tsc_wr_mem((void *)pdev, XLPORT_WC_UCMEM_DATA, 0, data);
#ifdef PM_PHYMOD_DEBUG
	pr_emerg("[Verify WR addr: %#X | Val:%#X]", data[0], val);
	lm_hw_pm_tsc_rd((void *)pdev, core_addr, reg_addr, lane, tsc_or_eagle, &data[4]);
	pr_emerg("-------%s() done----\n", __func__);
#endif

	return rc;
}
#endif

int lm_hw_pm_tsc_wr(void *user_acc,u32 core_addr, u32 reg_addr, u32 val)
{
   int rc;
   //uint32_t val = (uint32_t) val1;
   u16 data16 = (u16)val;
   u32 rd_data;
   u16 mask = (u16)(val >> 16);
   u32 data[4];

   //DbgMessage3(NULL,VERBOSE,"lm_hw_pm_tsc_wr():core_addr=%x, reg_addr=0x%x, val=0x%x\n",
   //core_addr,reg_addr,val);

   /* 
    * Bit 64 : 1 - memory write ; 0 - memory read
    * Bit 63..48 : 16-bit write data. 
    * Bit 47..32 : 16-bit write data mask.
    * Bit 31..0  : 32-bit TSC register address.
    */
   data[0] = (UCMEM_ADDR(core_addr,0,reg_addr,1) );
   data[1] = 0;
   data[2] = 1; /* Write operation */
   data[3] = 0;

   if (mask) {
     /* Mask is valid */
     lm_hw_pm_tsc_rd(user_acc, core_addr, reg_addr, 0, 0, &rd_data);
     data16 = (u16)(rd_data & ~mask);
     data16 |= (val & mask);
   }

   data[1] = UCMEM_DATA(data16);
   rc = lm_hw_pm_tsc_wr_mem((void *)user_acc,
                            XLPORT_WC_UCMEM_DATA,0, data);

   return rc;
}

int lm_hw_pm_tsc_wr_ln(void *user_acc,u32 core_addr, u32 reg_addr, u8 lane, bool tsc_or_eagle, u32 val)
{
   int rc;
   //uint32_t val = (uint32_t) val1;
   u16 data16 = (u16)val;
   u32 rd_data;
   u16 mask = (u16)(val >> 16);
   u32 data[4];

   //DbgMessage3(NULL,VERBOSE,"lm_hw_pm_tsc_wr():core_addr=%x, reg_addr=0x%x, val=0x%x\n",
   //core_addr,reg_addr,val);

   /* 
    * Bit 64 : 1 - memory write ; 0 - memory read
    * Bit 63..48 : 16-bit write data. 
    * Bit 47..32 : 16-bit write data mask.
    * Bit 31..0  : 32-bit TSC register address.
    */
   data[0] = (UCMEM_ADDR_LANE(core_addr, tsc_or_eagle, reg_addr, lane, 0) );
   data[1] = 0;
   data[2] = 1; /* Write operation */
   data[3] = 0;

   if (mask) {
     /* Mask is valid */
     lm_hw_pm_tsc_rd(user_acc, core_addr, reg_addr, 0, 0, &rd_data);
     data16 = (u16)(rd_data & ~mask);
     data16 |= (val & mask);
   }

   data[1] = UCMEM_DATA(data16);
   rc = lm_hw_pm_tsc_wr_mem((void *)user_acc,
                            XLPORT_WC_UCMEM_DATA,0, data);

   return rc;
}


void lm_hw_pm_mdio_write45(pm_device_t pdev, u8 phy_addr, u8 devad, u16 address,
			   u16 val)
{
   /* Address phase */
   lm_hw_pm_wr_mdio_cmd(pdev,
			     ((u32)phy_addr <<
			      NW_SERDES_MDIO_COMM_PHY_ADDR_SFT) | ((u32)devad
			      << NW_SERDES_MDIO_COMM_REG_ADDR_SFT)
			      | address | NW_SERDES_MDIO_COMM_START_BUSY, 1);

   /* Data Phase */
   lm_hw_pm_wr_mdio_cmd(pdev,
			     ((u32)phy_addr <<
			      NW_SERDES_MDIO_COMM_PHY_ADDR_SFT) | ((u32)devad
			      << NW_SERDES_MDIO_COMM_REG_ADDR_SFT)
			      | val | NW_SERDES_MDIO_COMM_START_BUSY |
                        NW_SERDES_MDIO_COMM_COMMAND_CMD_WR,1);
}

u16 lm_hw_pm_mdio_read45(pm_device_t pdev, u8 phy_addr, u8 devad, u16 address)
{
   /* Address phase */
   lm_hw_pm_wr_mdio_cmd(pdev,
			     ((u32)phy_addr <<
			      NW_SERDES_MDIO_COMM_PHY_ADDR_SFT) | ((u32)devad
			      << NW_SERDES_MDIO_COMM_REG_ADDR_SFT)
			      | address | NW_SERDES_MDIO_COMM_START_BUSY, 1);

   /* Data Phase */
   return lm_hw_pm_wr_mdio_cmd(pdev,
				    ((u32)phy_addr <<
				     NW_SERDES_MDIO_COMM_PHY_ADDR_SFT) | ((u32)
				     devad << NW_SERDES_MDIO_COMM_REG_ADDR_SFT)
				     | address | NW_SERDES_MDIO_COMM_START_BUSY
				     | NW_SERDES_MDIO_COMM_COMMAND_CMD_RD, 1);
}

static u16 lm_hw_pm_wr_mdio_cmd(pm_device_t pdev, u32 cmd, int clause45)
{
#ifdef UNCOMMENT /*Legacy Cumulus code : irrelevant here*/
	u32 j;
	u32 mdio_mode;
	u32 val;


   mdio_mode = (MDIO_CLK_DIVISOR << SWREG_MDIO_MODE_CLOCK_CNT_SFT);

   if (clause45)
      mdio_mode |= SWREG_MDIO_MODE_CLAUSE_45;
   else
      mdio_mode &= ~SWREG_MDIO_MODE_CLAUSE_45;

   REG_WRITE(pdev,IPC(nw_serdes_mdio_mode),mdio_mode);
   
   REG_WRITE(pdev,IPC(nw_serdes_mdio_comm),cmd);

   for (j = 0; j < MDIO_TIMEOUT; j++) {
      val = REG_READ(pdev,IPC(nw_serdes_mdio_comm));

      if (!(val & NW_SERDES_MDIO_COMM_START_BUSY)) {
         val = REG_READ(pdev,IPC(nw_serdes_mdio_comm));
         break;
      }

      PM_WAIT(pdev,1);
   }

	return (u16)(val & NW_SERDES_MDIO_COMM_DATA_MASK);
#endif
	return 0;
}

u16 lm_hw_pm_mdio_read22(pm_device_t pdev, u8 phy_addr, u16 address)
{
  return lm_hw_pm_wr_mdio_cmd(pdev,
				    ((u32)phy_addr <<
				     NW_SERDES_MDIO_COMM_PHY_ADDR_SFT) | ((u32)
				     address <<
				     NW_SERDES_MDIO_COMM_REG_ADDR_SFT)
				     | NW_SERDES_MDIO_COMM_COMMAND_CMD_RD |
                              NW_SERDES_MDIO_COMM_START_BUSY,0);
}

void lm_hw_pm_mdio_write22(pm_device_t pdev, u8 phy_addr, u16 address, u16 val)
{
  lm_hw_pm_wr_mdio_cmd(pdev,
			     ((u32)phy_addr <<
			      NW_SERDES_MDIO_COMM_PHY_ADDR_SFT) | ((u32)address
			      << NW_SERDES_MDIO_COMM_REG_ADDR_SFT)
			      | val | NW_SERDES_MDIO_COMM_COMMAND_CMD_WR |
                       NW_SERDES_MDIO_COMM_START_BUSY,0);
}

EXPORT_SYMBOL(apb2pbus_base);
EXPORT_SYMBOL(lm_hw_pm_apb2pbus_brdg_init);
EXPORT_SYMBOL(lm_hw_pm_rd_reg);
EXPORT_SYMBOL(lm_hw_pm_wr_reg);
EXPORT_SYMBOL(lm_hw_pm_minimal_default_xlmac_cfg);
EXPORT_SYMBOL(lm_hw_pm_reset);
EXPORT_SYMBOL(lm_hw_pm_cfg_mac_core);
EXPORT_SYMBOL(lm_hw_pm_bringup_phy);
EXPORT_SYMBOL(lm_hw_pm_get_link_status);
