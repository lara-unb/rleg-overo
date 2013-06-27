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

#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <util/delay.h>

#include "spi.h"

#include "amt203.h"

static void amt203_delay()
{
    /* AMT203 requires minimum 5us delay in between consecutive transfers */
    _delay_us(20);
}

static void amt203_xfer_byte(amt203_t * amt203, uint8_t sbyte, uint8_t * rbyte)
{
    spi_t * spi = amt203->spi;
    
    /* Every transfer must be gated by the chip select signal. */
    amt203->chip_select(1);
    spi_xfer(spi, sbyte, rbyte);
    amt203->chip_select(0);

    /* Be safe */
    amt203_delay();
}

static int amt203_wait_for_ack(amt203_t * amt203, uint8_t cmd, int max_errors)
{
    int errors = 0;
    uint8_t res = 0;
    while (res != cmd) {
        amt203_xfer_byte(amt203, cmd, &res);
        if (res != AMT203_RES_WAIT) {
            errors++;
            if (errors > max_errors) { return -1; }
        }
    }
    return 0;
}

int amt203_init(amt203_t * amt203, spi_t * spi, void (*chip_select)(int on))
{
    uint8_t res = 0;
    int i=0;
    assert(amt203 != NULL);
    assert(spi != NULL);
    assert(chip_select != NULL);

    memset(amt203, 0, sizeof(amt203_t));
    amt203->spi = spi;
    amt203->chip_select = chip_select;
    
    for (i=0; i<5; i++) { amt203_xfer_byte(amt203, AMT203_CMD_NOP, &res); }    
    return 0;
}

int amt203_position_read(amt203_t * amt203, uint16_t * position)
{
    uint8_t cmd = AMT203_CMD_RD_POS;
    uint8_t res = 0;
    assert(amt203 != NULL);

    /* Send command */
    amt203_xfer_byte(amt203, cmd, &res);
    if (amt203_wait_for_ack(amt203, cmd, 5) != 0) { return -1; }
    
    /* Get high byte */
    amt203_xfer_byte(amt203, AMT203_CMD_NOP, &res);
    *position |= ((uint16_t)res << 8);
    
    /* Get low byte */
    amt203_xfer_byte(amt203, AMT203_CMD_NOP, &res);
    *position |= (uint16_t)res;
       
    return (*position > 0x0fff);
}

int amt203_zero_set(amt203_t * amt203)
{
    uint8_t cmd = AMT203_CMD_SET_ZERO;
    uint8_t res = 0;
    assert(amt203 != NULL);
    
    /* Send command */
    amt203_xfer_byte(amt203, cmd, &res);
    if (amt203_wait_for_ack(amt203, AMT203_CMD_EEPROM_WR, INT_MAX) != 0) { 
        return -1; 
    }

    /* Note: 
     *   From datasheet, the encoder must be power-cycled after this command
     *   for the new zero position to be used in the position calculation. */

    return 0;
}

int amt203_eeprom_write(amt203_t * amt203, uint8_t addr, uint8_t data)
{
    uint8_t cmd = AMT203_CMD_EEPROM_WR;
    uint8_t res = 0;
    assert(amt203 != NULL);
    
    /* Send command */
    amt203_xfer_byte(amt203, cmd, &res);
    amt203_xfer_byte(amt203, addr, &res);
    amt203_xfer_byte(amt203, data, &res);
    if (amt203_wait_for_ack(amt203, cmd, 5) != 0) { return -1; }
    
    return 0;
}

int amt203_eeprom_read(amt203_t * amt203, uint8_t addr, uint8_t * data)
{
    uint8_t cmd = AMT203_CMD_EEPROM_RD;
    uint8_t res = 0;
    assert(amt203 != NULL);
    
    /* Send command */
    amt203_xfer_byte(amt203, cmd, &res);
    amt203_xfer_byte(amt203, addr, &res);
    amt203_xfer_byte(amt203, 0x01, &res);
    if (amt203_wait_for_ack(amt203, cmd, 5) != 0) { return -1; }
    
    /* Get data */
    amt203_xfer_byte(amt203, AMT203_CMD_NOP, &res);
    *data = res;
    
    return 0;
}
