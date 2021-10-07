/*
 * Copyright (C) 2015 Broadcom Corporation
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

#include <asm/mach/arch.h>

static const char *const bcm_nsp_dt_compat[] __initconst = {
	"brcm,nsp",
	NULL,
};
#include <asm/siginfo.h>
#include <asm/signal.h>
#define FSR_EXTERNAL		(1 << 12)
#define FSR_READ		(0 << 10)
#define FSR_IMPRECISE		0x0406

static int bcm_nsp_abort_handler(unsigned long addr, unsigned int fsr,
				  struct pt_regs *regs)
{
	/*
	 * We want to ignore aborts forwarded from the PCIe bus that are
	 * expected and shouldn't really be passed by the PCIe controller.
	 * The biggest disadvantage is the same FSR code may be reported when
	 * reading non-existing APB register and we shouldn't ignore that.
	 */
	if (fsr == (FSR_EXTERNAL | FSR_READ | FSR_IMPRECISE))
		return 0;

	return 1;
}

static void __init bcm_nsp_init_early(void)
{
	hook_fault_code(16 + 6, bcm_nsp_abort_handler, SIGBUS, BUS_OBJERR,
			"imprecise external abort");
}


DT_MACHINE_START(NSP_DT, "Broadcom Northstar Plus SoC")
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.dt_compat = bcm_nsp_dt_compat,
	.init_early	= bcm_nsp_init_early,
MACHINE_END
