/* @Author     : Om Raheja
 * @Date       : 6th February 2019
 * @Filename   : I2C.h
 * @Course     :IoT Embedded Firmware(Spring 2019)
 * */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

/* Libraries containing default Gecko configuration values */
#include "em_gpio.h"
#include "em_i2c.h"
#include "scheduler.h"
#include "i2cspmhalconfig.h"
#include "i2cspm.h"


/* Define Macros*/
#define INTERRUPT_FLAGS (I2C_IEN_ACK | I2C_IEN_NACK | I2C_IEN_RXDATAV)
#define SLAVE_ADDRESS      (0x40)
//#define SLAVE_CMD		   (uint8_t)(0xE5)

/* Global Variables */
//uint8_t buffer_write;
//uint8_t buffer_read[2] = {0};
uint16_t i2c_data;
uint16_t tempvalue;
//float tempvalue;
I2C_TransferReturn_TypeDef status;
I2C_TransferSeq_TypeDef pseudo_write;
I2C_TransferSeq_TypeDef i2c_read;

/*Function Prototypes*/

/*@Function Name: i2c_configure()
 *@brief        : Configures the structure by populating the
 *				  members of the structures.
 *@param [in]   : void
 *@return type  : void
 */
void i2c_configure();

/*@Function Name: transfer_sequence()
 *@brief        : Configures the structure by populating the
 *				  members of the structures.
 *@param [in]   : void
 *@return type  : void
 */
void transfer_sequence();

/*@Function Name: getTempVal()
 *@brief        : This function calculates the temperature in
 *				  celcius and prints it on a serial terminal.
 *@param [in]   : void
 *@return type  : void
 */
void getTempVal(void);

/*@Function Name: I2C_write()
 *@brief        : Defines attributes related to write sequence.
 *@param [in]   : void
 *@return type  : void
 */
void I2C_write(void);

/*@Function Name: I2C_read()
 *@brief        : Defines attributes related to read sequence.
 *@param [in]   : void
 *@return type  : void
 */
void I2C_read(void);

 /*@Function Name: I2C0_IRQHandler()
  *@brief        : Interrupt Service Routine for I2C.
  *@param [in]   : void
  *@return type  : void
  */
void I2C0_IRQHandler(void);

#endif /* SRC_I2C_H_ */
