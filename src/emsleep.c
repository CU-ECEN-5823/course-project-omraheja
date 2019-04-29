/* @Author     : Om Raheja
 * @Date       : 30th January 2019
 * @Filename   : emsleep.c
 * @Description:This file consists of function required to configure the sleep
 *              functionalities.
 * @IoT Embedded Firmware(Spring 2019)
 * */

/* User defined headers */
#include "em_cmu.h"
#include "em_letimer.h"
#include "gpio.h"
#include "sleep.h"
#include "emsleep.h"

//Function to configure sleep functionalities
void om_config()
{
	//No callbacks needed. Thus using NULL for sleep init.
	const SLEEP_Init_t init = { 0 };
	SLEEP_InitEx(&init);
}
