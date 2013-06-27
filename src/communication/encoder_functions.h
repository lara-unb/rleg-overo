/**
 * @file encoder_functions.h	
 * Rev 0 - 18/06/2013	
 * RLEG project - 2013
 *
 * @brief main commmands of the AMT 203 encoder
 * @author Claudia Ochoa
 * @date 2012-2013
 */

#ifndef ENCODER_FUNCTIONS_H
#define ENCODER_FUNCTIONS_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
//#include <linux/i2c-dev.h>

#define SUCCESS 1
#define FAILURE -1

/* Commands */
#define ENCODER_NO_OP         0x00
#define ENCODER_RD_POS        0x10
#define ENCODER_SET_ZERO_PT   0x70
#define ENCODER_EEPROM_WR     0x80
#define ENCODER_EEPROM_RD     0x90

/* Responses */
#define ENCODER_WAIT_RESP     0xA5

/**
 * @groupdef enc Function of Encoder AMT203
 */

/**
 * @brief INITIALIZE ENCODER
 * @ingroup enc
 * @param spi Communication Port
 * @param[in] status Register
 * @param[in] data Data to write
 
 * @return flag with SUCCESS or FAILURE
 */

int encoder_init(int spi,);


/**
 * @brief READ POSITION
 * @ingroup enc
 * @param spi Communication Port
 * @param[in] status Register
 * @param[in] data Data to write
 
 * @return flag with SUCCESS or FAILURE
 */

int encoder_read_pos();


/**
 * @brief SET ZERO POINT
 * @ingroup enc
 * @param spi_dev Communication Dev
 * @param[in] data Data to write
 
 * @return flag with SUCCESS or FAILURE
 */

  int enc_set_zero(int spi_dev,unsigned short int *data);


/**
 * @brief SEND BYTE TO THE ENCODER
 * @ingroup enc
 * @param spi_dev Communication Dev
 * @param[in] status Register
 * @param[in] data Data to write
 
 * @return flag with SUCCESS or FAILURE
 */

  int send_cmd(int spi_dev,unsigned short int *data);

  
/**
 * @brief GET DATA FROM THE ENCODER
 * @ingroup enc
 * @param spi_dev Communication Dev
 * @param[in] data Data to write
 * @return flag with SUCCESS or FAILURE
 */
  int enc_read(int spi_dev,unsigned short int *data);
  
 
/**
 * @brief delay between reads
 * @ingroup enc
 * @param spi_dev Communication Dev
 * @param[in] data Data to write
 * @return flag with SUCCESS or FAILURE
 */

  int enc_wait_for_ack(int spi_dev, uint8_t cmd, int max_errors);
  
 


#endif /* AMT203_FUNCTIONS_H */

