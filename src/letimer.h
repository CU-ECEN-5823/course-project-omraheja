/* @Author     : Om Raheja
 * @Date       : 30th January 2019
 * @Filename   : letimer.h
 * @Description: This file contains the function prototypes of all the functions used in
 * 				 letimer.c
 * @Course     :IoT Embedded Firmware(Spring 2019)
 * */

#ifndef SRC_LETIMER_H_
#define SRC_LETIMER_H_

#include "em_cmu.h"
#include "em_letimer.h"
#include "gpio.h"
#include "emsleep.h"
#include "scheduler.h"
#include "log.h"

/*Macros*/

/* TICKS count for EM0, EM1 and EM2
 * LETIMER0 = 16bit counter. Therefore, Total ticks = 65536.
 * Frequency of Clock in EM0, EM1 and EM2 using a prescaler
 * of 2 = 16384Hz.
 * Time period per tick = 1/16384 = approx. 61.0351 microsecconds
 * Max time that counter can count = 65536 * 61.0351 usec
 * 								   = 4 seconds
 * Therefore, if we need a time period of 3 seconds,
 * we will use the tick count to be 49152.
 * (using unitary method)
 * */
#define TICKS     (49152)

/* TICKS count for EM3
 * LETIMER0 = 16bit counter. Therefore, Total ticks = 65536.
 * Frequency of Clock in EM3 = 1000 Hz.
 * Time period per tick = 1/1000 = 1 milliecconds
 * Max time that counter can count = 65536 * 1 msec
 * 								   = 65.536 seconds
 * Therefore, if we need a time period of 3 seconds,
 * we will use the tick count to be 3002.
 * (using unitary method)
 * */
#define TICKS_EM3 (3000)


/* Function Prototypes */

/*@Function Name: letimer_init(void)
 *@return       : void
 *@brief        : The function :
 *                1) enables the oscillator, selects the clock
 *                source, pre-scales the clock frequency according to the
 *                Energy mode selected.
 *                2) Configures the LETIMER0.
 * */
void letimer_init(void);

/*@Function Name: LETIMER0_IRQHandler(void)
 *@return       : void
 *@brief        : The function sets the event in the scheduler.
 * */
void LETIMER0_IRQHandler(void);

#endif /* SRC_LETIMER_H_ */
