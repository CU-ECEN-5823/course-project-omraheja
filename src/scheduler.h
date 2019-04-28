/* @Author     : Om Raheja
 * @Date       : 6th February 2019
 * @Filename   : scheduler.h
 * @Course     : IoT Embedded Firmware(Spring 2019)
 * */
#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

// Standard C Library Headers
#include <stdbool.h>
#include "em_gpio.h"
#include "log.h"
#include "I2C.h"
#include "delay_us.h"
#include "sleep.h"
#include "em_core.h"
#include "display.h"
#include "native_gecko.h"


//Function Prototype
//void scheduler(uint32_t om);
void scheduler(void);

//Global variables
I2C_TransferReturn_TypeDef i2c_transfer_status;

/* Declaring the events */
struct timer_events{
	bool power_on_setup_timer_event;
	bool timer_expired;
	bool complete_i2c_write;
	bool complete_i2c_read;
	bool def_event;
	bool PushButtonFlag;
};

//for LCD
#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1
#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

// Declare instance of the structure
struct timer_events timer_events;

////////////////////////////////
// External signals for scheduler
#define UF_FLAG				0x10
#define COMP1_FLAG			0x20
#define I2C_WRITE_DONE 		0x40
#define I2C_READ_DONE 		0x80

/* Declaring the states */
typedef enum {
	TEMP_SENSOR_POWER_OFF,
//	TEMP_SENSOR_WAIT_FOR_POWER_UP,
	TEMP_SENSOR_WAIT_FOR_I2C_WRITE_COMPLETE,
	TEMP_SENSOR_WAIT_FOR_I2C_READ_COMPLETE
}temp_sensor_state ;

#endif /* SRC_SCHEDULER_H_ */
