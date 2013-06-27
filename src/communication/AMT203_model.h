/*
 *  Copyright 2012 Sasanka Nagavalli
 *
 *  This file is part of the FlexMARS Project.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef AMT203_H
#define AMT203_H

#include <stdint.h>

#include "spi.h"

/* Commands */
#define AMT203_CMD_NOP        0x00
#define AMT203_CMD_RD_POS     0x10
#define AMT203_CMD_SET_ZERO   0x70
#define AMT203_CMD_EEPROM_WR  0x80
#define AMT203_CMD_EEPROM_RD  0x90

/* Responses */
#define AMT203_RES_WAIT  0xa5

typedef struct amt203_t {
    spi_t * spi;
    void (*chip_select)(int on);
} amt203_t;

int amt203_init(amt203_t * amt203, spi_t * spi, void (*chip_select)(int on));
int amt203_position_read(amt203_t * amt203, uint16_t * position);
int amt203_zero_set(amt203_t * amt203);
int amt203_eeprom_write(amt203_t * amt203, uint8_t addr, uint8_t data);
int amt203_eeprom_read(amt203_t * amt203, uint8_t addr, uint8_t * data);

#endif /* AMT203_H */
