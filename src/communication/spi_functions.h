
/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 *
 */

#ifndef SPI_FUNCTIONS_H
#define SPI_FUNCTIONS_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* Commands */
#define ENCODER_NO_OP         0x00
#define ENCODER_RD_POS        0x10
#define ENCODER_SET_ZERO_PT   0x70
#define ENCODER_EEPROM_WR     0x80
#define ENCODER_EEPROM_RD     0x90

/**
 * @groupdef spi Functions to deal communication by SPI protocol
 */

/**
 * INITIALIZE SPI DEVICE
 * @ingroup spi
 */
int spi_init(uint8_t mode, uint32_t speed, uint8_t cs);
/* Parameters
- mode:		0 .. 3
- speed (Hz):	48000000, 24000000, 12000000, 6000000, 3000000, 1500000, 750000, 375000, 187000,, 93700, 46800, 23400, 11700, 5800, 2900, 1500000
- cs:		0 or 1	//Chip select
returns spi_dev
*/

/**
 * TERMINATES SPI DEVICE
 * @ingroup spi
 */
void spi_end(int spi_dev);

/**
 * READ DATA FROM A/D CONVERTER MCP3208
 * @ingroup spi
 * @param ch Set Channel (0 .. 7)
 * @param sgl (0: to single or 1: to pseudo-differencial)
 */
int adc_read(int spi_dev, uint8_t ch, uint8_t sgl, short int *data);

/**
 * WRITE DATA TO D/A CONVERTER MCP3922
 * @ingroup spi
 */
int dac_write(int spi_dev, uint8_t ch, uint8_t _shdn, unsigned short int data);
/* Parameters
- ch (channel):	0 or 1
- _shdn (shutdown):
0	Output buffer disabled, Output is high impedance
1	Output Power Down Control bit
- data:		0 .. 4095
*/

/**
 * TRANSFER N BYTES
 * @ingroup spi
 */
int spi_trans_bytes(int spi_dev,uint8_t *send, uint8_t *receive,int n);

#endif
