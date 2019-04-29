/* @Author     : Om Raheja
 * @Date       : 6th February 2019
 * @Filename   : I2C.c
 * @Course     : IoT Embedded Firmware(Spring 2019)
 * */

/* User defined Headers */
#include <src/I2C.h>
#include "log.h"
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
//#include "app/bluetooth/common/util/infrastructure.h"
#include "em_core.h"

/* flags */
bool read;
bool write;

/* Initialize the I2C structure */
void i2c_configure()
{
	/* I2C driver instance initialization structure.
	This data structure contains a number of I2C configuration options
	required for driver instance initialization.
	This struct is passed to @ref I2C_Init() when initializing a I2C
	instance. */
	I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_DEFAULT;

	i2cInit.sclPin = 10;
	i2cInit.sdaPin = 11;
	i2cInit.portLocationScl = 14;
	i2cInit.portLocationSda = 16;

	/*Initialize I2C peripheral*/
	I2CSPM_Init(&i2cInit);

//	/* Humidity sensor gpio enable */
//	GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 0);

	I2C_IntEnable(I2C0, I2C_IEN_RXDATAV | I2C_IEN_RXFULL | I2C_IEN_TXC);

	/* Enable I2C interrupt */
	NVIC_EnableIRQ(I2C0_IRQn);
}

/* SLAVE_CMD (Slave address) declared in I2C.h */
//Define the slave address
//uint8_t buffer_write = SLAVE_CMD;
uint8_t slaveCommand = 0xE5;

uint8_t buffer_read[2] = {0};

/* Perform the I2C write */
void I2C_write(void)
{
	/* Indicate write sequence: S+ADDR(W)+DATA0+P */
	pseudo_write.flags 			= I2C_FLAG_WRITE;
	pseudo_write.addr  			= (SLAVE_ADDRESS << 1);
	pseudo_write.buf[0].data 	= &slaveCommand;
	pseudo_write.buf[0].len		= 1;

	/* Set write flag */
	write = true;

	/* Start an I2C Transfer */
    I2C_TransferInit(I2C0, &pseudo_write);
}

/* Perform the I2C Read */
void I2C_read(void)
{
	/* Indicate read sequence */

	i2c_read.addr= (SLAVE_ADDRESS << 1);
	i2c_read.flags 	= I2C_FLAG_READ;
	i2c_read.buf[0].data = buffer_read;
	i2c_read.buf[0].len	= 2;

	/* Set read flag */
	read = true;

	/* Start an I2C Transfer */
	I2C_TransferInit(I2C0, &i2c_read);
}

/* Get value of the Temperature */
void getTempVal(void)
{
	/*@Reference: Silicon Labs Sample Code (SoC- Thermometer)
	 * */
	/* Store the temperature sensor value in a variable */
	i2c_data = 0;
	/* Store the temperature sensor value in a variable */
	i2c_data |= ((uint16_t)(buffer_read[0]) << 8);
	i2c_data |= buffer_read[1];

	//Calculate the humidity value
	tempvalue = (125*i2c_data/65536) - 6;

	if(tempvalue > 45)
		gecko_external_signal(HUMIDITY_FLAG);

	/* Print the value on serial terminal */
	LOG_INFO("Humidity -> %d percent", tempvalue);
}

/* I2C0 IRQ Handler */
void I2C0_IRQHandler(void)
{

	I2C_TransferReturn_TypeDef i2c_transfer_status = I2C_Transfer(I2C0);

	/* Check if i2c transfer is done */
	if(i2c_transfer_status == i2cTransferDone)
	{
//		LOG_INFO("I2C TRANSFER DONE");
		if(read == 1)
		{
//			timer_events.complete_i2c_read = true;
//			timer_events.def_event = false;
//			gecko_external_signal(timer_events.complete_i2c_read);
			gecko_external_signal(I2C_READ_DONE);
			read = 0;
		}

		if(write == 1)
		{
//			timer_events.complete_i2c_write = true;
//			timer_events.def_event = false;
//			gecko_external_signal(timer_events.complete_i2c_write);
			gecko_external_signal(I2C_WRITE_DONE);
			write = 0;
		}
	}
	else if(i2c_transfer_status != i2cTransferInProgress)
	{
		LOG_INFO("I2C TRANSFER FAILED!");
	}
}
