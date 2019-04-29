/* @Author     		: Om Raheja
 * @Date       		: 6th February 2019
 * @Filename   		: delay_us.h
 * @Course     		: IoT Embedded Firmware(Spring 2019)
 * @File Description: timerWaitUs() generates a delay of 1microsecond.
 * */

#ifndef SRC_DELAY_US_H_
#define SRC_DELAY_US_H_

/* User defined header */
#include "em_letimer.h"

/*Function Prototype*/

/*@Function Name: timerWaitUs()
 *@brief        : This function generates a delay of (us_wait) microseconds
 *@param [in]   : uint32_t us_wait (time in microseconds)
 *@return type  : void
 */
void timerWaitUs(uint32_t us_wait);


#endif /* SRC_DELAY_US_H_ */
