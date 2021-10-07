/* Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This file contains the Pegasus IOMUX driver that supports group
 * based PINMUX configuration. The PWM is functional only when the
 * corresponding mfio pin group is selected as gpio.
 */


#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "../core.h"
#include "../pinctrl-utils.h"

#define PEGASUS_NUM_IOMUX		13
#define PEGASUS_NUM_PWM_MUX		2

#define PEGASUS_PIN_MUX_BASE0		0x00
#define PEGASUS_PIN_MUX_BASE1		0x01
#define PEGASUS_PIN_CONF_BASE		0x02
#define PEGASUS_MUX_PAD_FUNC1_OFFSET	0x04

#define PEGASUS_PIN_SRC_MASK		0x01
#define PEGASUS_PIN_PULL_MASK		0x03
#define PEGASUS_PIN_DRIVE_STRENGTH_MASK	0x07

#define PEGASUS_PIN_PULL_UP		0x01
#define PEGASUS_PIN_PULL_DOWN		0x02

#define PEGASUS_PIN_INPUT_EN_MASK	0x01

/*
 * Pegasus IOMUX register description
 *
 * @base: base address number
 * @offset: register offset for mux configuration of a group
 * @shift: bit shift for mux configuration of a group
 * @mask: mask bits
 * @alt: alternate function to set to
 */
struct pegasus_mux {
	unsigned int base;
	unsigned int offset;
	unsigned int shift;
	unsigned int mask;
	unsigned int alt;
};

/*
 * Keep track of Pegasus IOMUX configuration and prevent double
 * configuration
 *
 * @pegasus_mux: Pegasus IOMUX register description
 * @is_configured: flag to indicate whether a mux setting has already
 * been configured
 */
struct pegasus_mux_log {
	struct pegasus_mux mux;
	bool is_configured;
};

/*
 * Group based IOMUX configuration
 *
 * @name: name of the group
 * @pins: array of pins used by this group
 * @num_pins: total number of pins used by this group
 * @mux: Pegasus group based IOMUX configuration
 */
struct pegasus_pin_group {
	const char *name;
	const unsigned *pins;
	const unsigned num_pins;
	const struct pegasus_mux mux;
};

/*
 * Pegasus mux function and supported pin groups
 *
 * @name: name of the function
 * @groups: array of groups that can be supported by this function
 * @num_groups: total number of groups that can be supported by function
 */
struct pegasus_pin_function {
	const char *name;
	const char * const *groups;
	const unsigned num_groups;
};

/*
 * Pegasus IOMUX pinctrl core
 *
 * @pctl: pointer to pinctrl_dev
 * @dev: pointer to device
 * @base0: first IOMUX register base
 * @base1: second IOMUX register base
 * @pinconf_base: configuration register base
 * @groups: pointer to array of groups
 * @num_groups: total number of groups
 * @functions: pointer to array of functions
 * @num_functions: total number of functions
 * @mux_log: pointer to the array of mux logs
 * @lock: lock to protect register access
 */
struct pegasus_pinctrl {
	struct pinctrl_dev *pctl;
	struct device *dev;
	void __iomem *base0;
	void __iomem *base1;
	void __iomem *pinconf_base;

	const struct pegasus_pin_group *groups;
	unsigned num_groups;

	const struct pegasus_pin_function *functions;
	unsigned num_functions;

	struct pegasus_mux_log *mux_log;

	spinlock_t lock;
};

/*
 * Pin configuration info
 *
 * @base: base address number
 * @offset: register offset from base
 * @src_shift: slew rate control bit shift in the register
 * @input_en: input enable control bit shift
 * @pull_shift: pull-up/pull-down control bit shift in the register
 * @drive_shift: drive strength control bit shift in the register
 */
struct pegasus_pinconf {
	unsigned int base;
	unsigned int offset;
	unsigned int src_shift;
	unsigned int input_en;
	unsigned int pull_shift;
	unsigned int drive_shift;
};

/*
 * Description of a pin in Pegasus
 *
 * @pin: pin number
 * @name: pin name
 * @pin_conf: pin configuration structure
 */
struct pegasus_pin {
	unsigned pin;
	char *name;
	struct pegasus_pinconf pin_conf;
};

#define PEGASUS_PIN_DESC(p, n, b, o, s, i, pu, d)	\
{						\
	.pin = p,				\
	.name = n,				\
	.pin_conf = {				\
		.base = b,			\
		.offset = o,			\
		.src_shift = s,			\
		.input_en = i,			\
		.pull_shift = pu,		\
		.drive_shift = d,		\
	}					\
}

/*
 * List of pins in Pegasus
 */
static struct pegasus_pin pegasus_pins[] = {
	PEGASUS_PIN_DESC(0, "mfio_0", 2, 0, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(1, "mfio_1", 2, 0, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(2, "mfio_2", 2, 0, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(3, "mfio_3", 2, 0, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(4, "mfio_4", 2, 8, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(5, "mfio_5", 2, 0xc, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(6, "mfio_6", 2, 8, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(7, "mfio_7", 2, 0xc, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(8, "mfio_8", 2, 0x20, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(9, "mfio_9", 2, 0x20, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(10, "mfio_10", 2, 0x20, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(11, "mfio_11", 2, 0x24, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(12, "mfio_12", 2, 0x24, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(13, "mfio_13", 2, 0x24, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(14, "mfio_14", 2, 0x10, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(15, "mfio_15", 2, 0x18, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(16, "mfio_16", 2, 0x10, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(17, "mfio_17", 2, 0x10, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(18, "mfio_18", 2, 0x14, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(19, "mfio_19", 2, 0x14, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(20, "mfio_20", 2, 0x14, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(21, "mfio_21", 2, 0x14, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(22, "mfio_22", 2, 0x18, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(23, "mfio_23", 2, 0x18, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(24, "mfio_24", 2, 0x1c, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(25, "mfio_25", 2, 0x18, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(26, "mfio_26", 2, 0x10, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(27, "mfio_27", 2, 0x2c, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(28, "mfio_28", 2, 0x2c, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(29, "mfio_29", 2, 0x2c, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(30, "mfio_30", 2, 0x30, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(31, "mfio_31", 2, 0x30, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(32, "mfio_32", 2, 0x30, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(33, "mfio_33", 2, 0x30, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(34, "mfio_34", 2, 0x34, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(35, "mfio_35", 2, 0x34, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(36, "mfio_36", 2, 0x34, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(37, "mfio_37", 2, 0x34, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(38, "mfio_38", 2, 0x38, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(39, "mfio_39", 2, 0x38, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(40, "mfio_40", 2, 0x38, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(41, "mfio_41", 2, 0x38, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(42, "mfio_42", 2, 0x3c, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(43, "mfio_43", 2, 0xc, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(44, "mfio_44", 2, 0xc, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(45, "mfio_45", 2, 4, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(46, "mfio_46", 2, 4, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(47, "mfio_47", 2, 4, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(48, "mfio_48", 2, 4, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(49, "mfio_49", 2, 0x90, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(50, "mfio_50", 2, 0x90, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(51, "mfio_51", 2, 0x90, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(52, "mfio_52", 2, 0x90, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(53, "mfio_53", 2, 0x94, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(54, "mfio_54", 2, 0x94, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(55, "mfio_55", 2, 0x4c, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(56, "mfio_56", 2, 0x4c, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(57, "mfio_57", 2, 0x4c, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(58, "mfio_58", 2, 0x50, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(59, "mfio_59", 2, 0x50, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(60, "mfio_60", 2, 0x50, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(61, "mfio_61", 2, 0x50, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(62, "mfio_62", 2, 0x50, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(63, "sflash_clk", 2, 0x24, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(64, "sflash_cs_I", 2, 0x28, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(65, "sflash_mosi", 2, 0x28, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(66, "sflash_miso", 2, 0x28, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(67, "sflash_wp_n", 2, 0x28, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(68, "sflash_hold_n", 2, 0x2c, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(69, "dstrap_bit3", 2, 0x20, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(70, "dstrap_bit4", 2, 0x60, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(71, "i2c1_scl", 2, 0xac, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(72, "i2c1_sda", 2, 0xac, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(73, "i2c2_scl", 2, 0xa8, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(74, "i2c2_sda", 2, 0xa8, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(75, "i2c3_scl", 2, 0xa8, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(76, "i2c3_sda", 2, 0xa8, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(77, "i2s_ws_0", 2, 0x80, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(78, "i2s_sdout_0", 2, 0x88, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(79, "i2s_sdin_0", 2, 0x8c, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(80, "i2s_bitclk_0", 2, 0x84, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(81, "i2s_mclk_0", 2, 0x88, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(82, "i2s_ws_1", 2, 0x80, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(83, "i2s_sdout_1", 2, 0x88, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(84, "i2s_sdin_1", 2, 0x8c, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(85, "i2s_bitclk_1", 2, 0x84, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(86, "i2s_mclk_1", 2, 0x84, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(87, "i2s_ws_2", 2, 0x80, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(88, "i2s_sdout_2", 2, 0x88, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(89, "i2s_sdin_2", 2, 0x8c, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(90, "i2s_bitclk_2", 2, 0x80, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(91, "i2s_mclk_2", 2, 0x84, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(92, "pcie0_perstb", 2, 0x74, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(93, "pcie1_perstb", 2, 0x74, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(94, "pcie2_perstb", 2, 0x74, 23, 22, 19, 16),
	PEGASUS_PIN_DESC(95, "usb3_vbus_ppc", 2, 0x7c, 15, 14, 11, 8),
	PEGASUS_PIN_DESC(96, "usb2_vbus_ppc", 2, 0x7c, 7, 6, 3, 0),
	PEGASUS_PIN_DESC(97, "usb3_vbus_ovc", 2, 0x7c, 31, 30, 27, 24),
	PEGASUS_PIN_DESC(98, "usb2_vbus_ovc", 2, 0x7c, 23, 22, 19, 16),
};

/*
 * List of groups of pins
 */

static const unsigned spi0_pins[] = {0, 1, 2, 3};
static const unsigned gpio_0_3_pins[] = {0, 1, 2, 3};

static const unsigned uart1_in_out_pins[] = {4, 5};
static const unsigned uart1_cts_rts_pins[] = {6, 7};
static const unsigned synce_output_mfio_4_5_pins[] = {4, 5};
static const unsigned time_sync_input_mfio_6_7_pins[] = {6, 7};
static const unsigned time_sync_output_mfio_4_5_pins[] = {4, 5};
static const unsigned time_sync_output_mfio_6_7_pins[] = {6, 7};
static const unsigned gpio_7_9_pins[] = {4, 5};
static const unsigned gpio_6_8_pins[] = {6, 7};

static const unsigned mdio_pins[] = {8, 9};
static const unsigned gpio_10_11_pins[] = {8, 9};

static const unsigned pwm_0_1_pins[] = {10, 11};
static const unsigned pwm_2_3_pins[] = {12, 13};
static const unsigned gpio_12_13_pins[] = {10, 11};
static const unsigned gpio_14_15_pins[] = {12, 13};
static const unsigned synce_input_mfio_10_11_pins[] = {10, 11};
static const unsigned synce_input_mfio_12_13_pins[] = {12, 13};
static const unsigned synce_output_mfio_10_11_pins[] = {10, 11};
static const unsigned synce_output_mfio_12_13_pins[] = {12, 13};

static const unsigned emmc_mfio_14_26_pins[] = {14, 15, 16, 17, 18, 19, 20,
						21, 22, 23, 24, 25, 26};
static const unsigned sdio_mfio_14_19_pins[] = {14, 15, 16, 17, 18, 19};
static const unsigned gpio_16_19_pins[] = {20, 21, 22, 23};
static const unsigned sdio_mfio_24_26_pins[] = {24, 25, 26};

static const unsigned nand_pins[] = {27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
					37, 38, 39, 40, 41, 42};
static const unsigned emmc_mfio_27_39_pins[] = {27, 28, 29, 30, 31, 32, 33,
						34, 35, 36, 37, 38, 39};
static const unsigned gpio_4_5_0_pins[] = {40, 41, 42};
static const unsigned sdio_mfio_27_32_pins[] = {27, 28, 29, 30, 31, 32};
static const unsigned gpio_16_19_mfio_33_36_pins[] = {33, 34, 35, 36};
static const unsigned sdio_mfio_37_39_pins[] = {37, 38, 39};

static const unsigned uart2_pins[] = {43, 44};
static const unsigned gpio_20_21_pins[] = {43, 44};
static const unsigned time_sync_input_mfio_43_44_pins[] = {43, 44};
static const unsigned time_sync_output_mfio_43_44_pins[] = {43, 44};

static const unsigned spi1_pins[] = {45, 46, 47, 48};
static const unsigned gpio_22_25_pins[] = {45, 46, 47, 48};
static const unsigned audio_mfio_45_48_pins[] = {45, 46, 47, 48};

static const unsigned uart3_pins[] = {49, 50};
static const unsigned gpio_26_27_pins[] = {49, 50};
static const unsigned time_sync_input_mfio_49_50_pins[] = {49, 50};
static const unsigned time_sync_output_mfio_49_50_pins[] = {49, 50};

static const unsigned gpio_28_31_pins[] = {51, 52, 53, 54};
static const unsigned sgpio_mfio_51_54_pins[] = {51, 52, 53, 54};
static const unsigned audio_mfio_51_54_pins[] = {51, 52, 53, 54};
static const unsigned gpio_28_30_pins[] = {51, 52, 53};
static const unsigned uart1_ext_clk_pins[] = {54};

static const unsigned parallel_led_pins[] = {55, 56, 57, 58, 59, 60, 61, 62};
static const unsigned sgpio_mfio_55_58_pins[] = {55, 56, 57, 58};
static const unsigned serial_led_pins[] = {59, 60};

#define PEGASUS_PIN_GROUP(group_name, ba, off, sh, ma, al)	\
{							\
	.name = __stringify(group_name) "_grp",		\
	.pins = group_name ## _pins,			\
	.num_pins = ARRAY_SIZE(group_name ## _pins),	\
	.mux = {					\
		.base = ba,				\
		.offset = off,				\
		.shift = sh,				\
		.mask = ma,				\
		.alt = al,				\
	}						\
}


/*
 * List of Pegasus pin groups
 */
static const struct pegasus_pin_group pegasus_pin_groups[] = {
	/*
	 * IOMUXREG[1:0]
	 * bits - func[MFIO]
	 * 00 - SPI0[0-3]
	 * 01 - GPIO0-3[0-3]
	 * 10 - GPIO0-3[0-3]
	 * 11 - GPIO0-3[0-3]
	 */
	PEGASUS_PIN_GROUP(spi0, 0, 0, 0, 3, 0),
	PEGASUS_PIN_GROUP(gpio_0_3, 0, 0, 0, 3, 1),
	PEGASUS_PIN_GROUP(gpio_0_3, 0, 0, 0, 3, 2),
	PEGASUS_PIN_GROUP(gpio_0_3, 0, 0, 0, 3, 3),

	/*
	 * IOMUXREG[3:2]
	 * bits - func[MFIO]
	 * 00 - UART1[4-5]
	 * 01 - TSYNC OUTPUT[4-5]
	 * 10 - SYNC OUTPUT[4-5]
	 * 11 - GPIO7,9[4-5]
	 */
	PEGASUS_PIN_GROUP(uart1_in_out, 0, 0, 2, 3, 0),
	PEGASUS_PIN_GROUP(time_sync_output_mfio_4_5, 0, 0, 2, 3, 1),
	PEGASUS_PIN_GROUP(synce_output_mfio_4_5, 0, 0, 2, 3, 2),
	PEGASUS_PIN_GROUP(gpio_7_9, 0, 0, 2, 3, 3),
	/*
	 * IOMUXREG[21:20]
	 * bits - func[MFIO]
	 * 00 - UART1[6-7]
	 * 01 - TSYNC OUTPUT[6-7]
	 * 10 - TSYNC INPUT[6-7]
	 * 11 - GPIO6,8[6-7]
	 */
	PEGASUS_PIN_GROUP(uart1_cts_rts, 0, 0, 0x14, 3, 0),
	PEGASUS_PIN_GROUP(time_sync_output_mfio_6_7, 0, 0, 0x14, 3, 1),
	PEGASUS_PIN_GROUP(time_sync_input_mfio_6_7, 0, 0, 0x14, 3, 2),
	PEGASUS_PIN_GROUP(gpio_6_8, 0, 0, 0x14, 3, 3),

	/*
	 * IOMUXREG[5:4]
	 * bits - func[MFIO]
	 * 00 - MDIO[8-9]
	 * 01 - GPIO10-11[8-9]
	 * 10 - GPIO10-11[8-9]
	 * 11 - GPIO10-11[8-9]
	 */
	PEGASUS_PIN_GROUP(mdio, 0, 0, 4, 3, 0),
	PEGASUS_PIN_GROUP(gpio_10_11, 0, 0, 4, 3, 1),
	PEGASUS_PIN_GROUP(gpio_10_11, 0, 0, 4, 3, 2),
	PEGASUS_PIN_GROUP(gpio_10_11, 0, 0, 4, 3, 3),

	/*
	 * IOMUXREG[7:6]
	 * bits - func[MFIO]
	 * 00 - PWM0-1[10-11]
	 * 01 - SYNC INPUT[10-11]
	 * 10 - GPIO12-13[10-11]
	 * 11 - SYNC OUTPUT[10-11]
	 */
	PEGASUS_PIN_GROUP(pwm_0_1, 0, 0, 6, 3, 0),
	PEGASUS_PIN_GROUP(synce_input_mfio_10_11, 0, 0, 6, 3, 1),
	PEGASUS_PIN_GROUP(gpio_12_13, 0, 0, 6, 3, 2),
	PEGASUS_PIN_GROUP(synce_output_mfio_10_11, 0, 0, 6, 3, 3),
	/*
	 * IOMUXREG[23:22]
	 * bits - func[MFIO]
	 * 00 - PWM2-3[12-13]
	 * 01 - SYNC INPUT[12-13]
	 * 10 - GPIO14-15[12-13]
	 * 11 - SYNC OUTPUT[12-13]
	 */
	PEGASUS_PIN_GROUP(pwm_2_3, 0, 0, 0x16, 3, 0),
	PEGASUS_PIN_GROUP(synce_input_mfio_12_13, 0, 0, 0x16, 3, 1),
	PEGASUS_PIN_GROUP(gpio_14_15, 0, 0, 0x16, 3, 2),
	PEGASUS_PIN_GROUP(synce_output_mfio_12_13, 0, 0, 0x16, 3, 3),

	/*
	 * IOMUXREG[9:8]
	 * bits - func[MFIO]
	 * 00 - EMMC[14-26]
	 * 01 - SDIO[14-19], GPIO16-19[20-23], SDIO[24-26]
	 * 10 - SDIO[14-19], GPIO16-19[20-23], SDIO[24-26]
	 * 11 - SDIO[14-19], GPIO16-19[20-23], SDIO[24-26]
	 */
	PEGASUS_PIN_GROUP(emmc_mfio_14_26, 0, 0, 8, 3, 0),
	PEGASUS_PIN_GROUP(sdio_mfio_14_19, 0, 0, 8, 3, 1),
	PEGASUS_PIN_GROUP(gpio_16_19, 0, 0, 8, 3, 1),
	PEGASUS_PIN_GROUP(sdio_mfio_24_26, 0, 0, 8, 3, 1),
	PEGASUS_PIN_GROUP(sdio_mfio_14_19, 0, 0, 8, 3, 2),
	PEGASUS_PIN_GROUP(gpio_16_19, 0, 0, 8, 3, 2),
	PEGASUS_PIN_GROUP(sdio_mfio_24_26, 0, 0, 8, 3, 2),
	PEGASUS_PIN_GROUP(sdio_mfio_14_19, 0, 0, 8, 3, 3),
	PEGASUS_PIN_GROUP(gpio_16_19, 0, 0, 8, 3, 3),
	PEGASUS_PIN_GROUP(sdio_mfio_24_26, 0, 0, 8, 3, 3),

	/*
	 * IOMUXREG[11:10]
	 * bits - func[MFIO]
	 * 00 - NAND[27-42]
	 * 01 - SDIO[27-32], GPIO16-19[33-36], SDIO[37-39], GPIO4,5,0[40-42]
	 * 10 - EMMC[27-39], GPIO4,5,0[40-42]
	 * 11 - NAND[27-42]
	 */
	PEGASUS_PIN_GROUP(nand, 0, 0, 0xa, 3, 0),
	PEGASUS_PIN_GROUP(emmc_mfio_27_39, 0, 0, 0xa, 3, 2),
	PEGASUS_PIN_GROUP(gpio_4_5_0, 0, 0, 0xa, 3, 2),
	PEGASUS_PIN_GROUP(sdio_mfio_27_32, 0, 0, 0xa, 3, 1),
	PEGASUS_PIN_GROUP(gpio_16_19_mfio_33_36, 0, 0, 0xa, 3, 1),
	PEGASUS_PIN_GROUP(sdio_mfio_37_39, 0, 0, 0xa, 3, 2),
	PEGASUS_PIN_GROUP(gpio_4_5_0, 0, 0, 0xa, 3, 2),
	PEGASUS_PIN_GROUP(nand, 0, 0, 0xa, 3, 3),

	/*
	 * IOMUXREG[13:12]
	 * bits - func[MFIO]
	 * 00 - UART2[43-44]
	 * 01 - TSYNC INPUT[43-44]
	 * 10 - GPIO20-21[43-44]
	 * 11 - TSYNC OUTPUT[43-44]
	 */
	PEGASUS_PIN_GROUP(uart2, 0, 0, 0xc, 3, 0),
	PEGASUS_PIN_GROUP(time_sync_input_mfio_43_44, 0, 0, 0xc, 3, 1),
	PEGASUS_PIN_GROUP(gpio_20_21, 0, 0, 0xc, 3, 2),
	PEGASUS_PIN_GROUP(time_sync_output_mfio_43_44, 0, 0, 0xc, 3, 3),

	/*
	 * IOMUXREG[15:14]
	 * bits - func[MFIO]
	 * 00 - SPI1[45-48]
	 * 01 - AUDIO[45-48]
	 * 10 - GPIO22-25[45-48]
	 * 11 - GPIO22-25[45-48]
	 */
	PEGASUS_PIN_GROUP(spi1, 0, 0, 0xe, 3, 0),
	PEGASUS_PIN_GROUP(audio_mfio_45_48, 0, 0, 0xe, 3, 1),
	PEGASUS_PIN_GROUP(gpio_22_25, 0, 0, 0xe, 3, 2),
	PEGASUS_PIN_GROUP(gpio_22_25, 0, 0, 0xe, 3, 3),

	/*
	 * IOMUXREG[17:16]
	 * bits - func[MFIO]
	 * 00 - UART3[49-50]
	 * 01 - TSYNC INPUT[49-50]
	 * 10 - GPIO26-27[49-50]
	 * 11 - TSYNC OUTPUT[49-50]
	 */
	PEGASUS_PIN_GROUP(uart3, 0, 0, 0x10, 3, 0),
	PEGASUS_PIN_GROUP(time_sync_input_mfio_49_50, 0, 0, 0x10, 3, 1),
	PEGASUS_PIN_GROUP(gpio_26_27, 0, 0, 0x10, 3, 2),
	PEGASUS_PIN_GROUP(time_sync_output_mfio_49_50, 0, 0, 0x10, 3, 3),

	/*
	 * IOMUXREG[19:18]
	 * bits - func[MFIO]
	 * 00 - GPIO28-31[51-54]
	 * 01 - AUDIO[51-54]
	 * 10 - SGPIO[51-54]
	 * 11 - GPIO[28-30], UART1[54]
	 */
	PEGASUS_PIN_GROUP(gpio_28_31, 0, 0, 0x12, 3, 0),
	PEGASUS_PIN_GROUP(audio_mfio_51_54, 0, 0, 0x12, 3, 1),
	PEGASUS_PIN_GROUP(sgpio_mfio_51_54, 0, 0, 0x12, 3, 2),
	PEGASUS_PIN_GROUP(gpio_28_30, 0, 0, 0x12, 3, 3),
	PEGASUS_PIN_GROUP(uart1_ext_clk, 0, 0, 0x12, 3, 3),

	/*
	 * bits - func[MFIO]
	 * 000 - PARALLEL LED[55-62]
	 * 001 - SGPIO[55-58], SERIAL LED[59-60], NONE[61-62]
	 */
	PEGASUS_PIN_GROUP(parallel_led, 1, 0, 0, 7, 0),
	PEGASUS_PIN_GROUP(sgpio_mfio_55_58, 1, 0, 0, 7, 1),
	PEGASUS_PIN_GROUP(serial_led, 1, 0, 0, 7, 1),
};


/*
 * List of groups supported by functions
 */
static const char * const spi_grps[] = {"spi0_grp", "spi1_grp"};

static const char * const uart_grps[] = {"uart1_in_out_grp",
				"uart1_cts_rts_grp", "uart2_grp",
				"uart3_grp", "uart1_ext_clk_grp"};

static const char * const nand_grps[] = {"nand_grp"};

static const char * const emmc_grps[] = {"emmc_mfio_14_26_grp",
					"emmc_mfio_27_39_grp"};

static const char * const sdio_grps[] = {"sdio_mfio_14_19_grp",
					"sdio_mfio_27_32_grp"};

static const char * const mdio_grps[] = {"mdio_grp"};

static const char * const gpio_grps[] = {"gpio_0_3_grp", "gpio_7_9_grp",
	"gpio_6_8_grp",	"gpio_10_11_grp", "gpio_12_13_grp", "gpio_14_15",
	"gpio_16_19_grp", "gpio_4_5_0_grp", "gpio_16_19_mfio_33_36_grp",
	"gpio_20_21_grp", "gpio_22_25_grp", "gpio_26_27_grp",
	"gpio_28_31_grp", "gpio_28_30_grp", "gpio_28_29_grp", "gpio_30_31_grp"};

static const char * const sgpio_grps[] = {"sgpio_mfio_51_54_grp",
					"sgpio_mfio_55_58_grp"};

static const char * const audio_grps[] = {"audio_mfio_45_48_grp",
					"audio_mfio_51_54_grp"};

static const char * const pwm_grps[] = {"pwm_0_1_grp", "pwm_2_3_grp"};

static const char * const parled_grps[] = {"parallel_led_grp"};

static const char * const serled_grps[] = {"serial_led_grp"};

static const char * const synce_grps[] = {"synce_output_mfio_4_5_grp",
	"synce_input_mfio_10_11_grp", "synce_input_mfio_12_13_grp",
	"synce_output_mfio_10_11_grp", "synce_output_mfio_12_13_grp"};

static const char * const timesync_grps[] = {"time_sync_input_mfio_6_7_grp",
	"time_sync_output_mfio_4_5_grp", "time_sync_output_mfio_6_7_grp",
	"time_sync_input_mfio_43_44_grp", "time_sync_output_mfio_43_44_grp",
	"time_sync_input_mfio_49_50_grp", "time_sync_output_mfio_49_50_grp"};


#define PEGASUS_PIN_FUNCTION(func)			\
{							\
	.name = #func,					\
	.groups = func ## _grps,			\
	.num_groups = ARRAY_SIZE(func ## _grps),	\
}

/*
 * List of supported functions
 */
static const struct pegasus_pin_function pegasus_pin_functions[] = {
	PEGASUS_PIN_FUNCTION(spi),
	PEGASUS_PIN_FUNCTION(uart),
	PEGASUS_PIN_FUNCTION(nand),
	PEGASUS_PIN_FUNCTION(emmc),
	PEGASUS_PIN_FUNCTION(sdio),
	PEGASUS_PIN_FUNCTION(mdio),
	PEGASUS_PIN_FUNCTION(gpio),
	PEGASUS_PIN_FUNCTION(sgpio),
	PEGASUS_PIN_FUNCTION(audio),
	PEGASUS_PIN_FUNCTION(pwm),
	PEGASUS_PIN_FUNCTION(parled),
	PEGASUS_PIN_FUNCTION(serled),
	PEGASUS_PIN_FUNCTION(synce),
	PEGASUS_PIN_FUNCTION(timesync),
};

static int pegasus_get_groups_count(struct pinctrl_dev *pctrl_dev)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->num_groups;
}

static const char *pegasus_get_group_name(struct pinctrl_dev *pctrl_dev,
				      unsigned selector)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->groups[selector].name;
}

static int pegasus_get_group_pins(struct pinctrl_dev *pctrl_dev,
			      unsigned selector, const unsigned **pins,
			      unsigned *num_pins)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	*pins = pinctrl->groups[selector].pins;
	*num_pins = pinctrl->groups[selector].num_pins;

	return 0;
}

static void pegasus_pin_dbg_show(struct pinctrl_dev *pctrl_dev,
			     struct seq_file *s, unsigned offset)
{
	seq_printf(s, " %s", dev_name(pctrl_dev->dev));
}

static struct pinctrl_ops pegasus_pinctrl_ops = {
	.get_groups_count = pegasus_get_groups_count,
	.get_group_name = pegasus_get_group_name,
	.get_group_pins = pegasus_get_group_pins,
	.pin_dbg_show = pegasus_pin_dbg_show,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_dt_free_map,
};

static int pegasus_get_functions_count(struct pinctrl_dev *pctrl_dev)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->num_functions;
}

static const char *pegasus_get_function_name(struct pinctrl_dev *pctrl_dev,
					 unsigned selector)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->functions[selector].name;
}

static int pegasus_get_function_groups(struct pinctrl_dev *pctrl_dev,
				   unsigned selector,
				   const char * const **groups,
				   unsigned * const num_groups)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	*groups = pinctrl->functions[selector].groups;
	*num_groups = pinctrl->functions[selector].num_groups;

	return 0;
}

static int pegasus_pinmux_set(struct pegasus_pinctrl *pinctrl,
			  const struct pegasus_pin_function *func,
			  const struct pegasus_pin_group *grp,
			  struct pegasus_mux_log *mux_log)
{
	const struct pegasus_mux *mux = &grp->mux;
	int i;
	u32 val, mask;
	unsigned long flags;
	void __iomem *base_address;

	for (i = 0; i < PEGASUS_NUM_IOMUX; i++) {
		if ((mux->shift != mux_log[i].mux.shift) ||
			(mux->base != mux_log[i].mux.base) ||
			(mux->offset != mux_log[i].mux.offset))
			continue;

		/* if this is a new configuration, just do it! */
		if (!mux_log[i].is_configured)
			break;

		/*
		 * IOMUX has been configured previously and one is trying to
		 * configure it to a different function
		 */
		if (mux_log[i].mux.alt != mux->alt) {
			dev_err(pinctrl->dev,
				"double configuration error detected!\n");
			dev_err(pinctrl->dev, "func:%s grp:%s\n",
				func->name, grp->name);
			return -EINVAL;
		}

		return 0;
	}
	if (i == PEGASUS_NUM_IOMUX)
		return -EINVAL;

	mask = mux->mask;
	mux_log[i].mux.alt = mux->alt;
	mux_log[i].is_configured = true;

	switch (mux->base) {
	case PEGASUS_PIN_MUX_BASE0:
		base_address = pinctrl->base0;
		break;

	case PEGASUS_PIN_MUX_BASE1:
		base_address = pinctrl->base1;
		break;

	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(base_address + grp->mux.offset);
	val &= ~(mask << grp->mux.shift);
	val |= grp->mux.alt << grp->mux.shift;
	writel(val, (base_address + grp->mux.offset));
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	return 0;
}

static int pegasus_pinmux_enable(struct pinctrl_dev *pctrl_dev,
			     unsigned func_select, unsigned grp_select)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	const struct pegasus_pin_function *func;
	const struct pegasus_pin_group *grp;

	if (grp_select > pinctrl->num_groups ||
		func_select > pinctrl->num_functions)
		return -EINVAL;

	func = &pinctrl->functions[func_select];
	grp = &pinctrl->groups[grp_select];

	dev_dbg(pctrl_dev->dev, "func:%u name:%s grp:%u name:%s\n",
		func_select, func->name, grp_select, grp->name);

	dev_dbg(pctrl_dev->dev, "offset:0x%08x shift:%u alt:%u\n",
		grp->mux.offset, grp->mux.shift, grp->mux.alt);

	return pegasus_pinmux_set(pinctrl, func, grp, pinctrl->mux_log);
}

static int pegasus_pin_set_enable(struct pinctrl_dev *pctrldev,
		unsigned int pin, u16 enable)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct pegasus_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	u32 val;
	void __iomem *base_address;

	base_address = pinctrl->pinconf_base;
	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(base_address + pin_data->pin_conf.offset);
	val &= ~(PEGASUS_PIN_SRC_MASK << pin_data->pin_conf.input_en);

	if (!enable)
		val |= PEGASUS_PIN_INPUT_EN_MASK << pin_data->pin_conf.input_en;

	writel(val, (base_address + pin_data->pin_conf.offset));
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	dev_dbg(pctrldev->dev, "pin:%u set enable:%d\n", pin, enable);
	return 0;
}

static int pegasus_pin_get_enable(struct pinctrl_dev *pctrldev,
		unsigned int pin)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct pegasus_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	int enable;

	spin_lock_irqsave(&pinctrl->lock, flags);
	enable = readl(pinctrl->pinconf_base + pin_data->pin_conf.offset);
	enable = (enable >> pin_data->pin_conf.input_en) &
			PEGASUS_PIN_INPUT_EN_MASK;
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	if (!enable)
		enable = PEGASUS_PIN_INPUT_EN_MASK;
	else
		enable = 0;

	dev_dbg(pctrldev->dev, "pin:%u get disable:%d\n", pin, enable);
	return enable;
}

static int pegasus_pin_set_slew(struct pinctrl_dev *pctrldev, unsigned int pin,
			    u16 slew)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct pegasus_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	u32 val;
	void __iomem *base_address;

	base_address = pinctrl->pinconf_base;
	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(base_address + pin_data->pin_conf.offset);
	val &= ~(PEGASUS_PIN_SRC_MASK << pin_data->pin_conf.src_shift);

	if (slew)
		val |= PEGASUS_PIN_SRC_MASK << pin_data->pin_conf.src_shift;

	writel(val, (base_address + pin_data->pin_conf.offset));
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	dev_dbg(pctrldev->dev, "pin:%u set slew:%d\n", pin, slew);
	return 0;
}

static int pegasus_pin_get_slew(struct pinctrl_dev *pctrldev, unsigned int pin,
			    u16 *slew)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct pegasus_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(pinctrl->pinconf_base + pin_data->pin_conf.offset);
	*slew = (val >> pin_data->pin_conf.src_shift) & PEGASUS_PIN_SRC_MASK;
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	dev_dbg(pctrldev->dev, "pin:%u get slew:%d\n", pin, *slew);
	return 0;
}

static int pegasus_pin_set_pull(struct pinctrl_dev *pctrldev, unsigned int pin,
			    bool pull_up, bool pull_down)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct pegasus_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	u32 val;
	void __iomem *base_address;

	base_address = pinctrl->pinconf_base;
	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(base_address + pin_data->pin_conf.offset);
	val &= ~(PEGASUS_PIN_PULL_MASK << pin_data->pin_conf.pull_shift);

	if (pull_up == true)
		val |= PEGASUS_PIN_PULL_UP << pin_data->pin_conf.pull_shift;
	if (pull_down == true)
		val |= PEGASUS_PIN_PULL_DOWN << pin_data->pin_conf.pull_shift;
	writel(val, (base_address + pin_data->pin_conf.offset));
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	dev_dbg(pctrldev->dev, "pin:%u set pullup:%d pulldown: %d\n",
		pin, pull_up, pull_down);
	return 0;
}

static void pegasus_pin_get_pull(struct pinctrl_dev *pctrldev,
			     unsigned int pin, bool *pull_up,
			     bool *pull_down)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct pegasus_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(pinctrl->pinconf_base + pin_data->pin_conf.offset);
	val = (val >> pin_data->pin_conf.pull_shift) & PEGASUS_PIN_PULL_MASK;
	*pull_up = false;
	*pull_down = false;

	if (val == PEGASUS_PIN_PULL_UP)
		*pull_up = true;

	if (val == PEGASUS_PIN_PULL_DOWN)
		*pull_down = true;
	spin_unlock_irqrestore(&pinctrl->lock, flags);
}

static int pegasus_pin_set_strength(struct pinctrl_dev *pctrldev,
		unsigned int pin, u16 strength)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct pegasus_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	u32 val;
	unsigned long flags;
	void __iomem *base_address;

	/* make sure drive strength is supported */
	if (strength < 2 || strength > 16 || (strength % 2))
		return -ENOTSUPP;

	base_address = pinctrl->pinconf_base;
	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(base_address + pin_data->pin_conf.offset);
	val &= ~(PEGASUS_PIN_DRIVE_STRENGTH_MASK <<
		 pin_data->pin_conf.drive_shift);
	val |= ((strength / 2) - 1) << pin_data->pin_conf.drive_shift;
	writel(val, (base_address + pin_data->pin_conf.offset));
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	dev_dbg(pctrldev->dev, "pin:%u set drive strength:%d mA\n",
		pin, strength);
	return 0;
}

static int pegasus_pin_get_strength(struct pinctrl_dev *pctrldev,
		unsigned int pin, u16 *strength)
{
	struct pegasus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrldev);
	struct pegasus_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&pinctrl->lock, flags);
	val = readl(pinctrl->pinconf_base + pin_data->pin_conf.offset);
	*strength = (val >> pin_data->pin_conf.drive_shift) &
					PEGASUS_PIN_DRIVE_STRENGTH_MASK;
	*strength = (*strength + 1) * 2;
	spin_unlock_irqrestore(&pinctrl->lock, flags);

	dev_dbg(pctrldev->dev, "pin:%u get drive strength:%d mA\n",
		pin, *strength);
	return 0;
}

static int pegasus_pin_config_get(struct pinctrl_dev *pctldev, unsigned pin,
			      unsigned long *config)
{
	struct pegasus_pin *pin_data = pctldev->desc->pins[pin].drv_data;
	enum pin_config_param param = pinconf_to_config_param(*config);
	bool pull_up, pull_down;
	u16 arg = 0;
	int ret;

	if (pin_data->pin_conf.base == -1)
		return -ENOTSUPP;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		pegasus_pin_get_pull(pctldev, pin, &pull_up, &pull_down);
		if ((pull_up == false) && (pull_down == false))
			return 0;
		else
			return -EINVAL;

	case PIN_CONFIG_BIAS_PULL_UP:
		pegasus_pin_get_pull(pctldev, pin, &pull_up, &pull_down);
		if (pull_up)
			return 0;
		else
			return -EINVAL;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		pegasus_pin_get_pull(pctldev, pin, &pull_up, &pull_down);
		if (pull_down)
			return 0;
		else
			return -EINVAL;

	case PIN_CONFIG_DRIVE_STRENGTH:
		ret = pegasus_pin_get_strength(pctldev, pin, &arg);
		if (ret)
			return ret;
		*config = pinconf_to_config_packed(param, arg);
		return 0;

	case PIN_CONFIG_SLEW_RATE:
		ret = pegasus_pin_get_slew(pctldev, pin, &arg);
		if (ret)
			return ret;
		*config = pinconf_to_config_packed(param, arg);
		return 0;

	case PIN_CONFIG_INPUT_ENABLE:
		ret = pegasus_pin_get_enable(pctldev, pin);
		if (ret)
			return 0;
		else
			return -EINVAL;

	default:
		return -ENOTSUPP;
	}
}

static int pegasus_pin_config_set(struct pinctrl_dev *pctrldev, unsigned pin,
			      unsigned long *configs, unsigned num_configs)
{
	struct pegasus_pin *pin_data = pctrldev->desc->pins[pin].drv_data;
	enum pin_config_param param;
	unsigned int i;
	u16 arg;
	int ret = -ENOTSUPP;

	if (pin_data->pin_conf.base == -1)
		return -ENOTSUPP;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			ret = pegasus_pin_set_pull(pctrldev, pin, false, false);
			if (ret < 0)
				goto out;
			break;

		case PIN_CONFIG_BIAS_PULL_UP:
			ret = pegasus_pin_set_pull(pctrldev, pin, true, false);
			if (ret < 0)
				goto out;
			break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			ret = pegasus_pin_set_pull(pctrldev, pin, false, true);
			if (ret < 0)
				goto out;
			break;

		case PIN_CONFIG_DRIVE_STRENGTH:
			ret = pegasus_pin_set_strength(pctrldev, pin, arg);
			if (ret < 0)
				goto out;
			break;

		case PIN_CONFIG_SLEW_RATE:
			ret = pegasus_pin_set_slew(pctrldev, pin, arg);
			if (ret < 0)
				goto out;
			break;

		case PIN_CONFIG_INPUT_ENABLE:
			ret = pegasus_pin_set_enable(pctrldev, pin, arg);
			if (ret < 0)
				goto out;
			break;

		default:
			dev_err(pctrldev->dev, "invalid configuration\n");
			return -ENOTSUPP;
		}
	}
out:
	return ret;
}
static struct pinmux_ops pegasus_pinmux_ops = {
	.get_functions_count = pegasus_get_functions_count,
	.get_function_name = pegasus_get_function_name,
	.get_function_groups = pegasus_get_function_groups,
	.set_mux = pegasus_pinmux_enable,
};

static const struct pinconf_ops pegasus_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = pegasus_pin_config_get,
	.pin_config_set = pegasus_pin_config_set,
};

static struct pinctrl_desc pegasus_pinctrl_desc = {
	.name = "pegasus-pinmux",
	.pctlops = &pegasus_pinctrl_ops,
	.pmxops = &pegasus_pinmux_ops,
	.confops = &pegasus_pinconf_ops,
};

static int pegasus_mux_log_init(struct pegasus_pinctrl *pinctrl)
{
	struct pegasus_mux_log *log;
	unsigned int i;

	pinctrl->mux_log = devm_kcalloc(pinctrl->dev, PEGASUS_NUM_IOMUX,
					sizeof(struct pegasus_mux_log),
					GFP_KERNEL);
	if (!pinctrl->mux_log)
		return -ENOMEM;

	for (i = 0; i < (PEGASUS_NUM_IOMUX-1); i++) {
		pinctrl->mux_log[i].is_configured = false;
		log = &pinctrl->mux_log[i];
		log->mux.base = PEGASUS_PIN_MUX_BASE0;
		log->mux.offset = 0;
		log->mux.shift = (i * 2);
		log->mux.alt = 0;
	}

	/* This one for last iomux group(parallel/serial LED) */
	pinctrl->mux_log[PEGASUS_NUM_IOMUX - 1].is_configured = false;
	log = &pinctrl->mux_log[(PEGASUS_NUM_IOMUX - 1)];
	log->mux.base = PEGASUS_PIN_MUX_BASE1;
	log->mux.offset = 0;
	log->mux.shift = 0;
	log->mux.alt =  0;

	return 0;
}

static int pegasus_pinmux_probe(struct platform_device *pdev)
{
	struct pegasus_pinctrl *pinctrl;
	struct resource *res;
	int i, ret;
	struct pinctrl_pin_desc *pins;
	unsigned num_pins = ARRAY_SIZE(pegasus_pins);

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	pinctrl->dev = &pdev->dev;
	platform_set_drvdata(pdev, pinctrl);
	spin_lock_init(&pinctrl->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pinctrl->base0 = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pinctrl->base0)) {
		dev_err(&pdev->dev, "unable to map I/O space\n");
		return PTR_ERR(pinctrl->base0);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	pinctrl->base1 = devm_ioremap_nocache(&pdev->dev, res->start,
					resource_size(res));
	if (!pinctrl->base1) {
		dev_err(&pdev->dev, "unable to map I/O space\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	pinctrl->pinconf_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pinctrl->pinconf_base)) {
		dev_err(&pdev->dev, "unable to map I/O space\n");
		return PTR_ERR(pinctrl->pinconf_base);
	}

	ret = pegasus_mux_log_init(pinctrl);
	if (ret) {
		dev_err(&pdev->dev, "unable to initialize IOMUX log\n");
		return ret;
	}

	pins = devm_kcalloc(&pdev->dev, num_pins, sizeof(*pins), GFP_KERNEL);
	if (!pins)
		return -ENOMEM;

	for (i = 0; i < num_pins; i++) {
		pins[i].number = pegasus_pins[i].pin;
		pins[i].name = pegasus_pins[i].name;
		pins[i].drv_data = &pegasus_pins[i];
	}

	pinctrl->groups = pegasus_pin_groups;
	pinctrl->num_groups = ARRAY_SIZE(pegasus_pin_groups);
	pinctrl->functions = pegasus_pin_functions;
	pinctrl->num_functions = ARRAY_SIZE(pegasus_pin_functions);
	pegasus_pinctrl_desc.pins = pins;
	pegasus_pinctrl_desc.npins = num_pins;

	pinctrl->pctl = pinctrl_register(&pegasus_pinctrl_desc, &pdev->dev,
			pinctrl);
	if (!pinctrl->pctl) {
		dev_err(&pdev->dev, "unable to register IOMUX pinctrl\n");
		return -EINVAL;
	}

	return 0;
}

static const struct of_device_id pegasus_pinmux_of_match[] = {
	{.compatible = "brcm,pegasus-pinmux"},
	{ }
};

static struct platform_driver pegasus_pinmux_driver = {
	.driver = {
		.name = "pegasus-pinmux",
		.of_match_table = pegasus_pinmux_of_match,
	},
	.probe = pegasus_pinmux_probe,
};

static int __init pegasus_pinmux_init(void)
{
	return platform_driver_register(&pegasus_pinmux_driver);
}
arch_initcall(pegasus_pinmux_init);

MODULE_DESCRIPTION("Broadcom Pegasus IOMUX driver");
MODULE_LICENSE("GPL v2");
