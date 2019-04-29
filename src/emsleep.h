/* @Author     : Om Raheja
 * @Date       : 30th January 2019
 * @Filename   : emsleep.h
 * @Description:This file consists of function prototype present in emsleep.c
 * @IoT Embedded Firmware(Spring 2019)
 * */

#ifndef SRC_EMSLEEP_H_
#define SRC_EMSLEEP_H_

//Define the energy state
#define ENERGY_STATE 3

/*Function prototypes*/

/*@Function Name: om_config()
 *@return       : void
 *@brief        : The function configures sleep functionality.
 *				  As no callbacks are required, the SLEEP_Init_t init is
 *				  initialized to NULL.
 * */
void om_config();

#endif /* SRC_EMSLEEP_H_ */
