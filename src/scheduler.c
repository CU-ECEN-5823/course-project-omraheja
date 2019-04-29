/* @Author     : Om Raheja
 * @Date       : 6th February 2019
 * @Filename   : scheduler.c
 * @Course     : IoT Embedded Firmware(Spring 2019)
 * @Description: Implementation of state machine
 *
 * */

/* User defined headers */
#include "scheduler.h"

CORE_DECLARE_IRQ_STATE;

/* The Enumeration is declared in schduler.h */
temp_sensor_state next_state;								  //to store the next event to be set
temp_sensor_state current_state = TEMP_SENSOR_POWER_OFF;      //set current state as "power off state"

/* State Machine Implementation*/
//void scheduler(uint32_t event_signals)
void scheduler(void)
{
	switch(current_state)
	{
	//STATE : 0
		case (TEMP_SENSOR_POWER_OFF):
		//timer_events.power_on_setup_timer_event is set in LETIMER IRQ
//				if(timer_events.power_on_setup_timer_event && event_signals)         //UnderFlow interrupt
				if(timer_events.power_on_setup_timer_event)         //UnderFlow interrupt
				{
					CORE_ENTER_CRITICAL();
					timer_events.power_on_setup_timer_event = false;        //clear the event
					timer_events.def_event = true;
					CORE_EXIT_CRITICAL();

					GPIO_PinOutSet(gpioPortD, 15);

//					timerWaitUs(80000);
//
//					LOG_INFO("STATE 0");
//
//					next_state = TEMP_SENSOR_WAIT_FOR_POWER_UP;             //set next state
//				}
//		  	  break;
//	//STATE : 1
//		case (TEMP_SENSOR_WAIT_FOR_POWER_UP):
//		//timer_events.timer_expired is set in LETIMER IRQ
////			  if(timer_events.timer_expired && event_signals)           				//COMP1 interrupt
//				if(timer_events.timer_expired)
//				  {
//					  CORE_ENTER_CRITICAL();
//					  timer_events.timer_expired = false;
//					  timer_events.def_event = true;
//					  CORE_EXIT_CRITICAL();//Clear the event

//					  LOG_INFO("STATE 1");

	//				  SLEEP_SleepBlockBegin(sleepEM2);
					  I2C_write();							      				//I2C write to transition to the next state
					  next_state = TEMP_SENSOR_WAIT_FOR_I2C_WRITE_COMPLETE;		//set next state
				  }
		  	  break;
	//STATE : 2
		case (TEMP_SENSOR_WAIT_FOR_I2C_WRITE_COMPLETE):
//			if (timer_events.complete_i2c_write && event_signals) {
			if(timer_events.complete_i2c_write) {
				CORE_ENTER_CRITICAL();
				timer_events.complete_i2c_write = false;					//Clear the event
				timer_events.def_event = true;
				CORE_EXIT_CRITICAL();

//				LOG_INFO("STATE 2");

				I2C_read();													//I2C read to transition to the next state
				next_state = TEMP_SENSOR_WAIT_FOR_I2C_READ_COMPLETE;		//set next state
			}
			break;

    //STATE : 3
		case(TEMP_SENSOR_WAIT_FOR_I2C_READ_COMPLETE):
		//timer_events.complete_i2c_read is set in I2C IRQn
//			if(timer_events.complete_i2c_read && event_signals)
			if(timer_events.complete_i2c_read)
			{
				CORE_ENTER_CRITICAL();
				timer_events.complete_i2c_read = false;			//clear the event
				timer_events.def_event = true;
				CORE_EXIT_CRITICAL();

//				LOG_INFO("STATE 3");

//				SLEEP_SleepBlockEnd(sleepEM2);

				getTempVal();										//Get temperature value
				next_state = TEMP_SENSOR_POWER_OFF;				//set the next state
			}
			break;
	  }

	  //Set current state as the next state if they are not equal
	  if(current_state != next_state)
	  {
//		  LOG_INFO("Scheduler state changed from %d to %d", current_state, next_state);
		  current_state = next_state;
	  }
}
