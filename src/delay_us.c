/* @Author     		: Om Raheja
 * @Date       		: 6th February 2019
 * @Filename   		: delay_us.c
 * @Course     		: IoT Embedded Firmware(Spring 2019)
 * @File Description: timerWaitUs() generates a delay of (us_wait) microseconds.
 * */

/* User defined headers*/
#include "delay_us.h"
#include "letimer.h"



void timerWaitUs(uint32_t us_wait)
{
	uint32_t real_time;      	//variable to store the real time count of LETIMER_CounterGet(LETIMER0).
	uint32_t ticks;          	//Calculate the ticks to achieve the required delay
	uint32_t time_per_tick;
	uint32_t lastTicks;


	if((ENERGY_STATE == 0) || (ENERGY_STATE == 1) || (ENERGY_STATE == 2))
	{
		time_per_tick = 61.0351;
	}

	else if(ENERGY_STATE == 3)
	{
		time_per_tick = 1000;
	}

	//Get real time count
	real_time = LETIMER_CounterGet(LETIMER0);

	//Calculate ticks
	ticks = us_wait / time_per_tick ;

	if(real_time>ticks)
	{
		//calculate offset
		lastTicks = real_time - ticks;
	}
	else
	{
		lastTicks = LETIMER_CompareGet(LETIMER0, 0) - (ticks-real_time);
	}

	while(LETIMER_CounterGet(LETIMER0)!= lastTicks);

}
