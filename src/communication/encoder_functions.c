/**
 * @file encoder_functions.c
 * Rev 0 - 19/06/2013	
 * RLEG project - 2013
 *
 * @brief SPI communication between Gumstix Overo Fire and the AMT 203 encoder
 * @author Claudia Ochoa
 * @date 2012-2013
 */

#include "encoder_functions.h"
#include "spi_functions.h"
//#include <time.h>


/*Here, the all the functions created at enconder_funtions.h are declared*/


int enc_zero_set(int spi_dev){

    uint8_t send = ENCODER_SET_ZERO_PT;
    uint8_t receive = 0;
    
    /* Send command */
    spi_trans_bytes(spi_dev,&send,&receive,1);
    if (enc_wait_for_ack(spi_dev, ENCODER_EEPROM_WR, INT_MAX) != SUCCESS){ 
        return FAILURE;
    }

    /* Note: 
     *   From datasheet, the encoder must be power-cycled after this command
     *   for the new zero position to be used in the position calculation. */

    return SUCCESS;
}

int enc_read_pos(int spi_dev,unsigned short int *data){

    uint8_t send[] = {ENCODER_RD_POS,ENCODER_NO_OP};
    uint8_t receive[2] = {0,};
    
    /* Send command */
    spi_trans_bytes(spi_dev,send,receive,1);
    if (enc_wait_for_ack(spi_dev, ENCODER_RD_POS, INT_MAX) != SUCCESS) { 
        return FAILURE; 
    }

	/**
	* TODO Check this function, in case of error put delay before each byte data
	*/
    if(spi_trans_bytes(spi_dev,send,receive,2) != SUCCESS) { 
        return FAILURE; 
    }

   data[0] = receive[0];
   data[1] = receive[1];

   return SUCCESS;
}

int enc_wait_for_ack(int spi_dev, uint8_t ack, int max_errors)
{
    int errors = 0;
    uint8_t send = ENCODER_NO_OP;
    uint8_t receive = 0;
	
  //nanosleep(10);
	//spi_trans_bytes(spi_dev,send,receive,1);
    while (receive != ack) {
        spi_trans_bytes(spi_dev,&send,&receive,1);
        if (receive != ENCODER_WAIT_RESP) {
            errors++;
            if (errors > max_errors) { return FAILURE; }
        }
    }

    return SUCCESS;
}
