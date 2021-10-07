/*
 * Copyright (C) 2016 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _SI32260_SPI_H
#define _SI32260_SPI_H

#define REG_READ		_IOR(0xAA, 0, unsigned long)
#define REG_WRITE		_IOW(0xAA, 1, unsigned long)
#define RAM_READ		_IOR(0xAA, 2, unsigned long)
#define RAM_WRITE		_IOW(0xAA, 3, unsigned long)

struct si32260_ioctl_data {
	unsigned char channel;
	unsigned char reg;
	unsigned int  data;
	unsigned int address;
};

#define SI32260_ID_REG			0
#define SI32260_RESET_REG		1
#define SI32260_MSTREN_REG		2
#define SI32260_MSTRSTAT_REG		3
#define SI32260_RAMSTAT_REG		4
#define SI32260_RAM_ADDR_HI		5
#define SI32260_RAM_DATA_B0		6
#define SI32260_RAM_DATA_B1		7
#define SI32260_RAM_DATA_B2		8
#define SI32260_RAM_DATA_B3		9
#define SI32260_RAM_ADDR_LO		10

#endif
